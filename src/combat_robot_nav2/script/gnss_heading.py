#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nmea_msgs.msg import Sentence
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
import math

def euler_to_quaternion(yaw_rad):
    # ROS 표준: qz = sin(yaw/2), qw = cos(yaw/2)
    return 0.0, 0.0, math.sin(yaw_rad / 2), math.cos(yaw_rad / 2)

class Nav2HeadingProvider(Node):
    def __init__(self):
        super().__init__('nav2_heading_provider')
        # 표준 GPS 드라이버가 발행하는 NMEA 원본 문장 구독
        self.sub = self.create_subscription(Sentence, '/nmea_sentence', self.cb, 10)
        
        # 디버깅용 1D Float 토픽 및 Nav2/EKF용 3D Imu 토픽 발행
        self.pub_float = self.create_publisher(Float64, '/edge_heading', 10)
        self.pub_imu = self.create_publisher(Imu, '/gps/heading_imu', 10)
        
        self.get_logger().info('✅ Nav2 Heading Provider started cleanly!')

    def cb(self, msg):
        # NMEA 문장 중 'THS' (True Heading and Status)가 포함된 문장만 필터링
        if 'THS' in msg.sentence:
            parts = msg.sentence.split(',')
            
            # parts[1]은 각도, parts[2]는 상태('A'일 때만 정상)
            if len(parts) >= 3 and parts[1] and 'A' in parts[2]:
                try:
                    # 1. 원본 헤딩 파싱 (0도=북쪽, 시계방향)
                    heading_deg = float(parts[1])
                    
                    # 2. ROS 좌표계 변환 (0도=동쪽, 반시계방향)
                    ros_yaw_deg = 90.0 - heading_deg
                    ros_yaw_rad = math.radians(ros_yaw_deg)
                    
                    # 3. 디버깅용 헤딩 각도 발행
                    f_msg = Float64()
                    f_msg.data = heading_deg
                    self.pub_float.publish(f_msg)

                    # 4. Nav2/EKF용 IMU 쿼터니언 메시지 발행
                    imu_msg = Imu()
                    imu_msg.header.stamp = self.get_clock().now().to_msg()
                    
                    # 🔥 수정됨: gps_link -> base_footprint (TF 트리에 맞게 변경하여 데이터 증발 방지)
                    imu_msg.header.frame_id = "base_footprint" 
                    
                    qx, qy, qz, qw = euler_to_quaternion(ros_yaw_rad)
                    imu_msg.orientation.x = qx
                    imu_msg.orientation.y = qy
                    imu_msg.orientation.z = qz
                    imu_msg.orientation.w = qw
                    
                    # 방향(Yaw)에 대한 높은 신뢰도 부여 (공분산)
                    imu_msg.orientation_covariance[8] = 0.001 
                    
                    self.pub_imu.publish(imu_msg)
                    
                except Exception as e:
                    self.get_logger().warn(f'Parsing error: {e}')

def main():
    rclpy.init()
    rclpy.spin(Nav2HeadingProvider())
    rclpy.shutdown()

if __name__ == '__main__':
    main()