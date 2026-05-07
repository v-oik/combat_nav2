import os
import math
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, SetEnvironmentVariable
from launch_ros.actions import Node

# ==========================================================
# 💡 Nav2 연동용 헤딩 제공자 (Quaternion 변환 포함)
# ==========================================================
NAV2_HEADING_CODE = """
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
        self.sub = self.create_subscription(Sentence, '/nmea_sentence', self.cb, 10)
        self.pub_float = self.create_publisher(Float64, '/edge_heading', 10)
        self.pub_imu = self.create_publisher(Imu, '/gps/heading_imu', 10)
        self.get_logger().info('Nav2 Heading Provider with Quaternion started!')

    def cb(self, msg):
        if 'THS' in msg.sentence:
            parts = msg.sentence.split(',')
            if len(parts) >= 3 and parts[1] and 'A' in parts[2]:
                try:
                    # 1. 원본 데이터 파싱 (0도=북쪽, 시계방향)
                    heading_deg = float(parts[1])
                    
                    # 2. ROS 좌표계 변환 (0도=동쪽, 반시계방향)
                    # 공식: ROS_Yaw = 90 - GNSS_Heading
                    ros_yaw_deg = 90.0 - heading_deg
                    ros_yaw_rad = math.radians(ros_yaw_deg)
                    
                    # 3. Float64 발행 (디버깅용)
                    f_msg = Float64()
                    f_msg.data = heading_deg
                    self.pub_float.publish(f_msg)

                    # 4. IMU 메시지 발행 (Nav2/EKF용)
                    imu_msg = Imu()
                    imu_msg.header.stamp = self.get_clock().now().to_msg()
                    imu_msg.header.frame_id = "gps_link"
                    
                    qx, qy, qz, qw = euler_to_quaternion(ros_yaw_rad)
                    imu_msg.orientation.x = qx
                    imu_msg.orientation.y = qy
                    imu_msg.orientation.z = qz
                    imu_msg.orientation.w = qw
                    
                    # 공분산 설정 (헤딩 신뢰도 높임)
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
"""
# ==========================================================

def generate_launch_description():
    # SHM 에러 방지 설정
    disable_shm = SetEnvironmentVariable(name='ROS_DISABLE_LO_ONLY', value='1')

    # 1. 네트워크 설정
    setup_network = ExecuteProcess(cmd=['sudo', 'ip', 'addr', 'add', '192.168.1.102/24', 'dev', 'eth0'], output='screen')
    bring_up_interface = ExecuteProcess(cmd=['sudo', 'ip', 'link', 'set', 'eth0', 'up'], output='screen')

    # 2. RS Lidar 실행
    rs_lidar = ExecuteProcess(cmd=['ros2', 'launch', 'rslidar_sdk', 'start.py'], output='screen')

    # 3. Xsens MTi IMU 실행
    xsens_imu = ExecuteProcess(cmd=['ros2', 'launch', 'bluespace_ai_xsens_mti_driver', 'xsens_mti_node.launch.py'], output='screen')

    # 4. NMEA GPS 드라이버
    nmea_gps = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        output='screen',
        parameters=[{'port': '/dev/ttyUSB0', 'baud': 921600}]
    )

    # 5. Nav2용 헤딩 변환 노드 (인라인 실행)
    nav2_heading = ExecuteProcess(
        cmd=['python3', '-u', '-c', NAV2_HEADING_CODE],
        output='screen'
    )

    return LaunchDescription([
        disable_shm,
        setup_network,
        bring_up_interface,
        rs_lidar,
        xsens_imu,
        nmea_gps,
        nav2_heading
    ])