#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import math
import serial
import threading
from collections import deque
from functools import reduce
from operator import xor

# 🔥 우리가 처리하는 NMEA tag 집합 — 다른 sentence는 checksum 검사 전에 버림
_WANTED_TAGS = frozenset(('THS', 'GGA', 'VTG', 'GST'))


def euler_to_quaternion(yaw_rad):
    return 0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)


def nmea_checksum_ok(sentence):
    """NMEA checksum 검증. bytes 직접 XOR로 Python ord()-loop보다 빠름."""
    if not sentence.startswith('$') or '*' not in sentence:
        return False
    try:
        body, cksum = sentence[1:].split('*', 1)
        # bytes 객체 직접 iterate → int 변환 없이 XOR
        calc = reduce(xor, body.encode('ascii', errors='ignore'), 0)
        return calc == int(cksum.strip()[:2], 16)
    except Exception:
        return False


def nmea_to_decimal(coord_str, hemi):
    if not coord_str or not hemi:
        return None
    try:
        dot = coord_str.index('.')
        deg_len = dot - 2
        deg = float(coord_str[:deg_len])
        minutes = float(coord_str[deg_len:])
        decimal = deg + minutes / 60.0
        if hemi in ('S', 'W'):
            decimal = -decimal
        return decimal
    except Exception:
        return None


class Nav2HeadingProvider(Node):
    def __init__(self):
        super().__init__('nav2_heading_provider')

        # 파라미터
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baud', 921600)
        self.declare_parameter('heading_frame_id', 'gps')
        self.declare_parameter('gps_frame_id', 'gps')
        self.declare_parameter('gst_timeout_sec', 2.0)
        self.declare_parameter('antenna_yaw_offset_deg', 0.0)
        # 🔥 GPS 공분산 최소값 (m) — EKF가 GPS를 너무 과신하지 않게
        self.declare_parameter('min_sigma_horizontal', 0.10)
        self.declare_parameter('min_sigma_vertical', 0.15)
        # 🔥 fix quality가 이 값 미만이면 발행 안 함 (0=발행, 1=일반GPS 이상)
        self.declare_parameter('min_fix_quality', 1)
        # 🔥 σ가 이 값 초과면 발행 안 함 (m). 초기 boot 시 부정확한 fix로 navsat_transform이 잘못된 datum에 lock되는 것을 방지.
        # GPS 모듈은 q=5 RTK FIXED라고 보고하면서도 첫 fix는 σ=1.6m 같은 부정확 값을 줄 수 있음.
        self.declare_parameter('max_sigma_horizontal', 0.5)
        # 🔥 정지 시 dual-antenna heading 노이즈 회피용 (drop은 안 함 — navsat_transform이 죽음)
        # VTG 속도가 이 값 미만이면 heading 분산을 stationary_yaw_variance로 부풀림
        # 차량 vx_max=0.2라 0.3이면 주행 중에도 항상 stationary로 오판→heading 억제됨.
        # 0.1로 낮춰 주행 시 GPS heading 활성화(휠 vyaw 오류 보정). 정지 시 휠 vx=0 < 0.1.
        self.declare_parameter('stationary_speed_threshold_mps', 0.1)
        # VTG가 이 기간보다 오래되면 stationary 판정 비활성 (안전 fallback)
        self.declare_parameter('vtg_timeout_sec', 2.0)
        # 🔥 주행 중 heading yaw 분산 (rad²). 0.01 ≈ σ 5.7°
        self.declare_parameter('heading_yaw_variance', 0.01)
        # 🔥 정지 중 heading yaw 분산 (rad²). 25.0 ≈ σ 286° (EKF가 완전 무시)
        # 정지 게이팅은 휠 /odom 속도 기준(_odom_cb). IMU yaw-rate는 ekf_odom에서 비활성화됨.
        self.declare_parameter('stationary_yaw_variance', 25.0)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.heading_frame = self.get_parameter('heading_frame_id').value
        self.gps_frame = self.get_parameter('gps_frame_id').value
        self.gst_timeout = self.get_parameter('gst_timeout_sec').value
        self.antenna_offset = self.get_parameter('antenna_yaw_offset_deg').value
        self.min_sigma_h = self.get_parameter('min_sigma_horizontal').value
        self.min_sigma_v = self.get_parameter('min_sigma_vertical').value
        self.min_fix_quality = self.get_parameter('min_fix_quality').value
        self.max_sigma_h = self.get_parameter('max_sigma_horizontal').value
        self.stationary_speed_threshold = self.get_parameter('stationary_speed_threshold_mps').value
        self.vtg_timeout = self.get_parameter('vtg_timeout_sec').value
        self.heading_yaw_var = self.get_parameter('heading_yaw_variance').value
        self.stationary_yaw_var = self.get_parameter('stationary_yaw_variance').value

        # 퍼블리셔
        self.pub_float = self.create_publisher(Float64, '/edge_heading', 10)
        self.pub_imu = self.create_publisher(Imu, '/gps/heading_imu', 10)
        self.pub_fix = self.create_publisher(NavSatFix, '/fix', 10)
        self.pub_vel = self.create_publisher(TwistStamped, '/vel', 10)

        # GST 캐시
        self._gst_sigma_lat = None
        self._gst_sigma_lon = None
        self._gst_sigma_alt = None
        self._gst_last_stamp = None

        # VTG 속도 캐시 (heading 게이팅용)
        self._last_vtg_speed_mps = None
        self._last_vtg_stamp = None
        self._last_vtg_course = None

        # 🔥 휠 오도메트리(/odom) 속도 캐시 — VTG 미수신 환경에서 정지 게이팅의 주 소스.
        # can_reader가 20Hz로 신뢰성 있게 발행. VTG보다 우선 사용.
        self._last_odom_speed_mps = None
        self._last_odom_vyaw = None
        self._last_odom_stamp = None
        self.sub_odom = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        # 제자리 회전 임계 (rad/s). 이 이상이면 회전으로 보고 GPS heading 유지.
        self.declare_parameter('stationary_angular_threshold', 0.1)
        self.stationary_angular_threshold = float(self.get_parameter('stationary_angular_threshold').value)

        # 🔥 heading 평활 필터 (q=5 FLOAT ±30° 노이즈 완화). circular moving average.
        self.declare_parameter('heading_filter_window', 20)  # 20샘플 ≈ 1초@20Hz
        self._hdg_win = max(1, int(self.get_parameter('heading_filter_window').value))
        self._yaw_cos_buf = deque(maxlen=self._hdg_win)
        self._yaw_sin_buf = deque(maxlen=self._hdg_win)

        # 통계
        self._line_count = 0
        self._ths_count = 0
        self._gga_count = 0
        self._vtg_count = 0
        self._gst_count = 0
        self._gga_dropped_nofix = 0      # 🔥 NO_FIX로 drop된 개수
        self._ths_dropped_stationary = 0  # 🔥 정지로 drop된 개수
        self._first_line_seen = False
        self._first_fix_logged = False
        self._first_heading_logged = False
        self._last_fix_quality = None
        # status 로그 비활성화 (터미널 노이즈 줄이기). 디버깅 시 주석 해제.
        self.create_timer(3.0, self._status_timer)

        # 시리얼
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1.0)
            # 🔥 OS 커널 버퍼에 누적된 stale 데이터 비우고 첫 '$' 찾을 때까지 byte sync
            import time as _t
            _t.sleep(0.3)
            self.ser.reset_input_buffer()
            # 첫 라인 시작 위치 sync — '\n'까지 읽어서 버림
            self.ser.readline()
            self.get_logger().info(f'✅ Serial opened: {self.port} @ {self.baud} (buffer reset)')
        except serial.SerialException as e:
            self.get_logger().error(f'❌ Serial open failed: {e}')
            raise

        self._running = True
        self._thread = threading.Thread(target=self._serial_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f'✅ GNSS Provider started '
            f'(heading_frame={self.heading_frame}, '
            f'antenna_yaw_offset={self.antenna_offset:.2f}°, '
            f'min_sigma_h={self.min_sigma_h:.3f}m, '
            f'min_fix_quality={self.min_fix_quality}, '
            f'stationary_speed_threshold={self.stationary_speed_threshold:.2f}m/s, '
            f'heading_yaw_var={self.heading_yaw_var:.4f}, '
            f'stationary_yaw_var={self.stationary_yaw_var:.4f})'
        )

    def _status_timer(self):
        gst_age = '∞'
        if self._gst_last_stamp is not None:
            age = (self.get_clock().now() - self._gst_last_stamp).nanoseconds / 1e9
            gst_age = f'{age:.1f}s'
        q_str = f'q={self._last_fix_quality}' if self._last_fix_quality is not None else 'q=?'
        spd_str = f'vtg_spd={self._last_vtg_speed_mps:.2f}' if self._last_vtg_speed_mps is not None else 'vtg_spd=?'
        odom_str = f'odom_spd={self._last_odom_speed_mps:.3f}' if self._last_odom_speed_mps is not None else 'odom_spd=NONE'
        _crs = getattr(self, "_last_vtg_course", None)
        crs_str = f'vtg_course={_crs:.1f}' if _crs is not None else 'vtg_course=?'
        hdg_str = f'raw={getattr(self,"_last_raw_heading",-1):.1f} robot_hdg={getattr(self,"_last_robot_heading",-1):.1f} yaw_enu={getattr(self,"_last_yaw_enu_deg",-1):.1f} {crs_str}'
        self.get_logger().info(
            f'[status] line={self._line_count} '
            f'THS={self._ths_count} (drop_gga={self._gga_dropped_nofix} drop_stat={self._ths_dropped_stationary}) '
            f'GGA={self._gga_count} VTG={self._vtg_count} GST={self._gst_count} '
            f'GST_age={gst_age} {q_str} {spd_str} {odom_str} | {hdg_str}'
        )
        self._line_count = 0
        self._ths_count = 0
        self._gga_count = 0
        self._vtg_count = 0
        self._gst_count = 0
        self._gga_dropped_nofix = 0
        self._ths_dropped_stationary = 0

    def _serial_loop(self):
        while self._running and rclpy.ok():
            try:
                if not self.ser or not self.ser.is_open:
                    break
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode('ascii', errors='ignore').strip()
                if not line:
                    continue

                self._line_count += 1
                if not self._first_line_seen:
                    self.get_logger().info(f'✅ 첫 시리얼 라인: {line[:60]}')
                    self._first_line_seen = True

                # 🔥 tag 조기 체크 — 우리가 안 쓰는 sentence는 checksum 안 돌리고 버림
                # (GSA, RMC, GBS 등 약 절반의 NMEA 라인이 여기서 떨어짐)
                if len(line) < 6 or line[3:6] not in _WANTED_TAGS:
                    continue

                if not nmea_checksum_ok(line):
                    continue

                self._dispatch(line)

            except (serial.SerialException, OSError) as e:
                if self._running:
                    self.get_logger().error(f'Serial read error: {e}')
                break
            except Exception as e:
                if self._running:
                    self.get_logger().warn(f'Loop error: {e}')

    def _dispatch(self, sentence):
        tag = sentence[3:6]
        if tag == 'THS':
            self._handle_ths(sentence)
        elif tag == 'GGA':
            self._handle_gga(sentence)
        elif tag == 'VTG':
            self._handle_vtg(sentence)
        elif tag == 'GST':
            self._handle_gst(sentence)

    # ---------------- THS ----------------
    def _odom_cb(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self._last_odom_speed_mps = math.hypot(vx, vy)
        self._last_odom_vyaw = abs(msg.twist.twist.angular.z)  # 제자리 회전 감지용
        self._last_odom_stamp = self.get_clock().now()

    def _handle_ths(self, sentence):
        body = sentence.split('*', 1)[0]
        parts = body.split(',')
        if len(parts) < 3 or not parts[1] or 'A' not in parts[2]:
            return
        try:
            raw_heading_deg = float(parts[1])
            heading_deg = (raw_heading_deg - self.antenna_offset) % 360.0

            ros_yaw_rad = math.radians(90.0 - heading_deg)
            ros_yaw_rad = math.atan2(math.sin(ros_yaw_rad), math.cos(ros_yaw_rad))

            # 🔥 circular moving average 평활 (q=5 FLOAT 노이즈 완화)
            self._yaw_cos_buf.append(math.cos(ros_yaw_rad))
            self._yaw_sin_buf.append(math.sin(ros_yaw_rad))
            ros_yaw_filt = math.atan2(
                sum(self._yaw_sin_buf) / len(self._yaw_sin_buf),
                sum(self._yaw_cos_buf) / len(self._yaw_cos_buf),
            )
            ros_yaw_rad = ros_yaw_filt

            # 🔧 보정용 최신값 캐시 (status 로그)
            self._last_raw_heading = raw_heading_deg
            self._last_robot_heading = heading_deg
            self._last_yaw_enu_deg = math.degrees(ros_yaw_rad)

            stamp = self.get_clock().now().to_msg()

            # /edge_heading은 항상 발행 (디버그/모니터링용)
            f_msg = Float64()
            f_msg.data = heading_deg
            self.pub_float.publish(f_msg)

            # 🔥 정지 시 dual-antenna heading 노이즈가 EKF map yaw를 흔드는 걸 막기 위해
            # 분산을 크게 부풀림 (EKF가 거의 무시). navsat_transform은 계속 수신.
            # 🔥 정지 게이팅: 휠 /odom 속도 우선, 없으면 VTG fallback.
            # 속도 정보가 전혀 없으면(부팅 직후 등) heading을 신뢰하지 않고 정지로 간주(안전 기본값).
            now = self.get_clock().now()
            speed = None
            if (self._last_odom_speed_mps is not None
                    and self._last_odom_stamp is not None
                    and (now - self._last_odom_stamp).nanoseconds / 1e9 <= self.vtg_timeout):
                speed = self._last_odom_speed_mps
            elif (self._last_vtg_speed_mps is not None
                    and self._last_vtg_stamp is not None
                    and (now - self._last_vtg_stamp).nanoseconds / 1e9 <= self.vtg_timeout):
                speed = self._last_vtg_speed_mps

            # 정지/회전 시 GPS heading 차단 (회전은 휠 vyaw로 추적 — q5 heading은 회전 못 따라감).
            # 직선 이동 시에만 GPS heading 신뢰 → 절대 yaw 보정. (회전 중 발생한 휠 slip 오차는
            # 이후 직진 주행 때 GPS heading이 다시 잡아줌)
            if speed is None or speed < self.stationary_speed_threshold:
                yaw_var = self.stationary_yaw_var
                is_stationary = True
            else:
                yaw_var = self.heading_yaw_var
                is_stationary = False

            imu_msg = Imu()
            imu_msg.header.stamp = stamp
            imu_msg.header.frame_id = self.heading_frame
            qx, qy, qz, qw = euler_to_quaternion(ros_yaw_rad)
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            imu_msg.orientation.w = qw
            imu_msg.orientation_covariance[0] = 999.0
            imu_msg.orientation_covariance[4] = 999.0
            imu_msg.orientation_covariance[8] = yaw_var
            imu_msg.angular_velocity_covariance[0] = -1.0
            imu_msg.linear_acceleration_covariance[0] = -1.0

            # 🔥 정지 시 heading을 EKF에 아예 안 먹임 (high covariance로도 robot_localization
            # absolute orientation이 새어 들어가 yaw drift 유발). 정지 = 휠 vyaw만 → can_reader 모니터와 일치.
            # 이동 시에만 발행 → EKF가 정확한 GPS heading으로 절대 yaw 확정.
            if not is_stationary:
                self.pub_imu.publish(imu_msg)
            else:
                self._ths_dropped_stationary += 1

            self._ths_count += 1

            if not self._first_heading_logged:
                self.get_logger().info(
                    f'✅ 첫 heading 발행: '
                    f'raw={raw_heading_deg:.2f}° offset={self.antenna_offset:.2f}° '
                    f'→ robot_heading={heading_deg:.2f}° '
                    f'(yaw_enu={math.degrees(ros_yaw_rad):.2f}°)'
                )
                self._first_heading_logged = True

        except Exception as e:
            self.get_logger().warn(f'THS parse error: {e} | {sentence}')

    # ---------------- GST ----------------
    def _handle_gst(self, sentence):
        body = sentence.split('*', 1)[0]
        p = body.split(',')
        if len(p) < 9:
            return
        try:
            sigma_lat = float(p[6]) if p[6] else None
            sigma_lon = float(p[7]) if p[7] else None
            sigma_alt = float(p[8]) if p[8] else None

            if sigma_lat is None or sigma_lon is None or sigma_alt is None:
                return

            self._gst_sigma_lat = sigma_lat
            self._gst_sigma_lon = sigma_lon
            self._gst_sigma_alt = sigma_alt
            self._gst_last_stamp = self.get_clock().now()
            self._gst_count += 1
        except Exception as e:
            self.get_logger().warn(f'GST parse error: {e} | {sentence}')

    # ---------------- GGA ----------------
    def _handle_gga(self, sentence):
        body = sentence.split('*', 1)[0]
        p = body.split(',')
        if len(p) < 15:
            return
        try:
            fix_quality = int(p[6]) if p[6] else 0
            self._last_fix_quality = fix_quality

            # 🔥 NO_FIX 또는 낮은 quality면 발행 안 함 (EKF 보호)
            if fix_quality < self.min_fix_quality:
                self._gga_dropped_nofix += 1
                return

            lat = nmea_to_decimal(p[2], p[3])
            lon = nmea_to_decimal(p[4], p[5])
            alt = float(p[9]) if p[9] else 0.0
            hdop = float(p[8]) if p[8] else 99.0

            if lat is None or lon is None:
                return

            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.gps_frame

            status = NavSatStatus()
            if fix_quality == 0:
                status.status = NavSatStatus.STATUS_NO_FIX
            elif fix_quality in (4, 5):
                status.status = NavSatStatus.STATUS_GBAS_FIX
            elif fix_quality == 2:
                status.status = NavSatStatus.STATUS_SBAS_FIX
            else:
                status.status = NavSatStatus.STATUS_FIX
            status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS
            msg.status = status

            msg.latitude = lat
            msg.longitude = lon
            msg.altitude = alt

            # 1) GST 사용 가능하고 충분히 신선하면 → 그 값 사용
            sigma_e = sigma_n = sigma_u = None
            use_gst = False
            if (self._gst_last_stamp is not None
                    and self._gst_sigma_lat is not None):
                age = (self.get_clock().now() - self._gst_last_stamp).nanoseconds / 1e9
                if age <= self.gst_timeout:
                    sigma_n = self._gst_sigma_lat
                    sigma_e = self._gst_sigma_lon
                    sigma_u = self._gst_sigma_alt
                    use_gst = True

            # 2) Fallback: fix quality 기반 추정
            if not use_gst:
                if fix_quality in (4, 5):
                    sigma_e = sigma_n = 0.05
                    sigma_u = 0.10
                elif fix_quality == 2:
                    sigma_e = sigma_n = max(hdop * 1.0, 0.5)
                    sigma_u = sigma_e * 1.5
                else:
                    sigma_e = sigma_n = max(hdop * 2.5, 1.0)
                    sigma_u = sigma_e * 1.5

            # 🔥 σ가 너무 크면 (부정확한 fix) 발행 안 함 — navsat_transform datum 오염 방지
            if max(sigma_e, sigma_n) > self.max_sigma_h:
                self._gga_dropped_nofix += 1
                return

            # 🔥 공분산 최소값 적용 (EKF가 GPS를 너무 과신하지 않게)
            sigma_e = max(sigma_e, self.min_sigma_h)
            sigma_n = max(sigma_n, self.min_sigma_h)
            sigma_u = max(sigma_u, self.min_sigma_v)

            # NavSatFix.position_covariance 는 ENU 순서
            msg.position_covariance = [
                sigma_e ** 2, 0.0,         0.0,
                0.0,         sigma_n ** 2, 0.0,
                0.0,         0.0,         sigma_u ** 2,
            ]
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

            self.pub_fix.publish(msg)
            self._gga_count += 1

            if not self._first_fix_logged:
                src = 'GST' if use_gst else f'fallback(q={fix_quality})'
                self.get_logger().info(
                    f'✅ 첫 /fix 발행: lat={lat:.7f} lon={lon:.7f} alt={alt:.2f}m '
                    f'q={fix_quality} σ_E={sigma_e:.3f} σ_N={sigma_n:.3f} σ_U={sigma_u:.3f} [{src}]'
                )
                self._first_fix_logged = True

        except Exception as e:
            self.get_logger().warn(f'GGA parse error: {e} | {sentence}')

    # ---------------- VTG ----------------
    def _handle_vtg(self, sentence):
        body = sentence.split('*', 1)[0]
        p = body.split(',')
        if len(p) < 9:
            return
        try:
            course_true_deg = float(p[1]) if p[1] else None
            speed_kmh = float(p[7]) if p[7] else 0.0
            speed_mps = speed_kmh / 3.6

            # 🔥 CPU 부하 줄이려고 /vel TwistStamped 발행 제거 (EKF에서 사용 안 함).
            # 속도 캐시만 업데이트하여 heading 게이팅에 사용.
            self._vtg_count += 1
            self._last_vtg_speed_mps = speed_mps
            self._last_vtg_stamp = self.get_clock().now()
            # 🔧 VTG course(실제 진행방향, 직진 시 차량 heading) 캐시 — antenna offset 보정용
            if course_true_deg is not None and speed_mps > 0.15:
                self._last_vtg_course = course_true_deg
        except Exception as e:
            self.get_logger().warn(f'VTG parse error: {e} | {sentence}')

    def destroy_node(self):
        self._running = False
        try:
            if hasattr(self, '_thread') and self._thread.is_alive():
                self._thread.join(timeout=1.5)
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = Nav2HeadingProvider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()