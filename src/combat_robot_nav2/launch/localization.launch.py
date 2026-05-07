import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('combat_robot_nav2')
    ekf_config_file = os.path.join(pkg_dir, 'config', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odometry/local')]
    )

    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odometry/global')]
    )

    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': use_sim_time}],
        remappings=[('imu', '/sensing/imu/imu_data'),
                    ('gps/fix', '/sensing/gnss/nav_sat_fix'),
                    # 🔥 해결: local이 아니라 global 데이터를 먹여서 map 프레임과 일치시킵니다!
                    ('odometry/filtered', 'odometry/global')]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        ekf_local_node,
        ekf_global_node,
        navsat_transform_node
    ])