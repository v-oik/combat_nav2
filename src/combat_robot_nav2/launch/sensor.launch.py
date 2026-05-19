import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    disable_shm = SetEnvironmentVariable(name='ROS_DISABLE_LO_ONLY', value='1')

    # 1. 라이다 네트워크
    setup_network = ExecuteProcess(
        cmd=['sudo', 'ip', 'addr', 'replace', '192.168.1.102/24', 'dev', 'eth0'],
        output='screen'
    )
    bring_up_interface = ExecuteProcess(
        cmd=['sudo', 'ip', 'link', 'set', 'eth0', 'up'],
        output='screen'
    )

    # 2. RoboSense LiDAR
    rs_lidar = ExecuteProcess(
        cmd=['ros2', 'launch', 'rslidar_sdk', 'start.py'],
        output='screen'
    )

    # 3. Xsens MTi IMU
    xsens_imu = ExecuteProcess(
        cmd=['ros2', 'launch', 'bluespace_ai_xsens_mti_driver', 'xsens_mti_node.launch.py'],
        output='screen'
    )

    # 4. GNSS: /gps/heading_imu + /fix + /vel
    gnss_node = Node(
        package='combat_robot_nav2',
        executable='gnss_heading.py',
        output='screen',
        parameters=[{
            'port': '/dev/ttyUSB1',
            'baud': 921600,
            'heading_frame_id': 'gps',
            'gps_frame_id': 'gps',
            # 🔥 안테나 baseline 보정 (로봇 정북 향했을 때 NMEA heading 값)
            # 측정값 70.5° — 정북 향한 상태에서 echo로 확인한 값
            'antenna_yaw_offset_deg': 70.5
        }]
    )

    return LaunchDescription([
        disable_shm,
        setup_network,
        bring_up_interface,
        rs_lidar,
        xsens_imu,
        gnss_node
    ])