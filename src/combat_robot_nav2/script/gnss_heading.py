#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistStamped
import math
import serial
import threading


def euler_to_quaternion(yaw_rad):
    return 0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)


def nmea_checksum_ok(sentence):
    if not sentence.startswith('$') or '*' not in sentence:
        return False
    try:
        body, cksum = sentence[1:].split('*', 1)
        cksum = cksum.strip()[:2]
        calc = 0
        for ch in body:
            calc ^= ord(ch)
        return calc == int(cksum, 16)
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

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.heading_frame = self.get_parameter('heading_frame_id').value
        self.gps_frame = self.get_parameter('gps_frame_id').value
        self.gst_timeout = self.get_parameter('gst_timeout_sec').value
        self.antenna_offset = self.get_parameter('antenna_yaw_offset_deg').value
        self.min_sigma_h = self.get_parameter('min_sigma_horizontal').value
        self.min_sigma_v = self.get_parameter('min_sigma_vertical').value
        self.min_fix_quality = self.get_parameter('min_fix_quality').value

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

        # 통계
        self._line_count = 0
        self._ths_count = 0
        self._gga_count = 0
        self._vtg_count = 0
        self._gst_count = 0
        self._gga_dropped_nofix = 0      # 🔥 NO_FIX로 drop된 개수
        self._first_line_seen = False
        self._first_fix_logged = False
        self._first_heading_logged = False
        self._last_fix_quality = None
        self.create_timer(5.0, self._status_timer)

        # 시리얼
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1.0)
            self.get_logger().info(f'✅ Serial opened: {self.port} @ {self.baud}')
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
            f'min_fix_quality={self.min_fix_quality})'
        )

    def _status_timer(self):
        gst_age = '∞'
        if self._gst_last_stamp is not None:
            age = (self.get_clock().now() - self._gst_last_stamp).nanoseconds / 1e9
            gst_age = f'{age:.1f}s'
        q_str = f'q={self._last_fix_quality}' if self._last_fix_quality is not None else 'q=?'
        self.get_logger().info(
            f'[status] line={self._line_count} '
            f'THS={self._ths_count} GGA={self._gga_count} '
            f'(dropped={self._gga_dropped_nofix}) '
            f'VTG={self._vtg_count} GST={self._gst_count} '
            f'GST_age={gst_age} {q_str}'
        )
        self._line_count = 0
        self._ths_count = 0
        self._gga_count = 0
        self._vtg_count = 0
        self._gst_count = 0
        self._gga_dropped_nofix = 0

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

            stamp = self.get_clock().now().to_msg()

            f_msg = Float64()
            f_msg.data = heading_deg
            self.pub_float.publish(f_msg)

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
            imu_msg.orientation_covariance[8] = 0.001
            imu_msg.angular_velocity_covariance[0] = -1.0
            imu_msg.linear_acceleration_covariance[0] = -1.0
            self.pub_imu.publish(imu_msg)

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

            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.gps_frame

            if course_true_deg is not None:
                course_enu_rad = math.radians(90.0 - course_true_deg)
                msg.twist.linear.x = speed_mps * math.cos(course_enu_rad)
                msg.twist.linear.y = speed_mps * math.sin(course_enu_rad)
            else:
                msg.twist.linear.x = speed_mps
                msg.twist.linear.y = 0.0

            self.pub_vel.publish(msg)
            self._vtg_count += 1
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