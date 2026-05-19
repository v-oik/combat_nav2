import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('combat_robot_nav2')
    ekf_config_file = os.path.join(pkg_dir, 'config', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    with_gnss = LaunchConfiguration('with_gnss')

    # GPS 센서 이름표 보정 (GNSS 사용 시에만 필요)
    tf_gnss_fix = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'gnss_base_link', 'gps'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(with_gnss)
    )

    # GNSS 미사용 시 map -> odom 을 0점으로 고정 (TF 트리 유지용)
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(with_gnss)
    )

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
        remappings=[('odometry/filtered', 'odometry/global')],
        condition=IfCondition(with_gnss)
    )

    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': use_sim_time}],
        remappings=[('imu', '/gps/heading_imu'),     # 🔥 수정: 지자기 간섭 방지를 위해 듀얼 GPS 헤딩으로 변경
                    ('gps/fix', '/fix'),
                    ('odometry/filtered', 'odometry/local')],
        condition=IfCondition(with_gnss)
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('with_gnss', default_value='true', description='Enable GNSS-based global localization'),
        tf_gnss_fix,
        static_map_to_odom,
        ekf_local_node,
        ekf_global_node,
        navsat_transform_node
    ])