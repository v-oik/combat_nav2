#!/usr/bin/env python3
# xsens gyro z-bias 온라인 보정. 정지(/odom 기준) 시 bias EMA 추정 → 빼서 재발행.
# 자력계 미사용(차폐) — gyro rate만 보정. 절대 yaw는 GPS heading 담당.
import rclpy, math
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

class GyroBiasComp(Node):
    def __init__(self):
        super().__init__("gyro_bias_comp")
        self.declare_parameter("stationary_v", 0.03)
        self.declare_parameter("stationary_w", 0.02)
        self.declare_parameter("bias_alpha", 0.002)
        self.declare_parameter("init_bias_z", 0.004459)
        self.declare_parameter("zupt", True)   # 정지 시 각속도 0 강제 (yaw drift 제거)
        self.sv = self.get_parameter("stationary_v").value
        self.sw = self.get_parameter("stationary_w").value
        self.alpha = self.get_parameter("bias_alpha").value
        self.bz = self.get_parameter("init_bias_z").value
        self.zupt = self.get_parameter("zupt").value
        self.bx = 0.0; self.by = 0.0
        self.stationary = True
        self._n = 0
        self.pub = self.create_publisher(Imu, "/imu/data_corrected", 10)
        self.create_subscription(Imu, "/imu/data", self.imu_cb, 50)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_timer(5.0, self._log)
        self.get_logger().info(f"gyro_bias_comp started init_bz={self.bz:.6f} zupt={self.zupt} (자력계 미사용, gyro rate만 보정)")
    def odom_cb(self, m):
        v = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)
        w = abs(m.twist.twist.angular.z)
        self.stationary = (v < self.sv and w < self.sw)
    def imu_cb(self, m):
        if self.stationary:
            a = self.alpha
            self.bz = (1-a)*self.bz + a*m.angular_velocity.z
            self.bx = (1-a)*self.bx + a*m.angular_velocity.x
            self.by = (1-a)*self.by + a*m.angular_velocity.y
        o = Imu()
        o.header = m.header
        o.orientation = m.orientation
        o.orientation_covariance = m.orientation_covariance
        if self.stationary and self.zupt:
            # ZUPT: /odom 기준 정지로 판정되면 각속도를 0으로 강제 출력.
            # raw-bias 의 잔여 + 노이즈(±수°/min)가 ekf_odom 에 적분되어 정지 중에도
            # odom->base_link yaw 가 드리프트(=local costmap 회전)하던 것을 제거한다.
            # 제자리 회전 시엔 /odom 의 angular.z 가 살아있어 stationary=False → 정상 보정.
            o.angular_velocity.x = 0.0
            o.angular_velocity.y = 0.0
            o.angular_velocity.z = 0.0
        else:
            o.angular_velocity.x = m.angular_velocity.x - self.bx
            o.angular_velocity.y = m.angular_velocity.y - self.by
            o.angular_velocity.z = m.angular_velocity.z - self.bz
        o.angular_velocity_covariance = m.angular_velocity_covariance
        o.linear_acceleration = m.linear_acceleration
        o.linear_acceleration_covariance = m.linear_acceleration_covariance
        self.pub.publish(o)
        self._n += 1
    def _log(self):
        self.get_logger().info(f"[bias] stationary={self.stationary} bz={math.degrees(self.bz)*60:.2f}deg/min pub={self._n}")
        self._n = 0

def main():
    rclpy.init(); n = GyroBiasComp()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally:
        n.destroy_node()
        try:
            if rclpy.ok(): rclpy.shutdown()
        except Exception: pass

if __name__ == "__main__":
    main()
