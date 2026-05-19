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

    # 4. GNSS: 시리얼 직접 열어서 헤딩(/gps/heading_imu) + 위치(/fix) + 속도(/vel) 발행
    gnss_node = Node(
        package='combat_robot_nav2',
        executable='gnss_heading.py',
        output='screen',
        parameters=[{
            'port': '/dev/ttyUSB1',
            'baud': 921600,
            'heading_frame_id': 'base_footprint',
            'gps_frame_id': 'gps'
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