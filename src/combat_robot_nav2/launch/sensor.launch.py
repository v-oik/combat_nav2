import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    # FastDDS SHM 관련 에러 방지
    disable_shm = SetEnvironmentVariable(name='ROS_DISABLE_LO_ONLY', value='1')

    # 1. 라이다 네트워크용 고정 IP 할당 (실행 시 sudo 패스워드 입력 대기 주의)
    setup_network = ExecuteProcess(cmd=['sudo', 'ip', 'addr', 'add', '192.168.1.102/24', 'dev', 'eth0'], output='screen')
    bring_up_interface = ExecuteProcess(cmd=['sudo', 'ip', 'link', 'set', 'eth0', 'up'], output='screen')

    # 2. RoboSense LiDAR 드라이버 실행
    rs_lidar = ExecuteProcess(cmd=['ros2', 'launch', 'rslidar_sdk', 'start.py'], output='screen')

    # 3. Xsens MTi IMU 드라이버 실행 (기본 포트: /dev/ttyUSB0)
    xsens_imu = ExecuteProcess(cmd=['ros2', 'launch', 'bluespace_ai_xsens_mti_driver', 'xsens_mti_node.launch.py'], output='screen')

    # 4. 표준 NMEA GPS 드라이버 실행 (위치 /fix, 원본 문장 /nmea_sentence 발행)
    nmea_gps = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        output='screen',
        parameters=[{
            'port': '/dev/ttyUSB1',
            'baud': 921600
        }]
    )

    # 5. 커스텀 헤딩 파싱 노드 (/nmea_sentence 구독 -> /gps/heading_imu 발행)
    nav2_heading = Node(
        package='combat_robot_nav2',
        executable='nav2_heading_provider.py',
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