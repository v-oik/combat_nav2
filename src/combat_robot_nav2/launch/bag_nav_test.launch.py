import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_combat_nav = get_package_share_directory('combat_robot_nav2')
    pkg_vehicle_desc = get_package_share_directory('combat_robot_description')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    rviz_config_dir = os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')
    xacro_file = os.path.join(pkg_vehicle_desc, 'urdf', 'robot.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file])

    # ==========================================
    # [Step 1]即時実行 (0秒): 骨格と座標系(EKF) 먼저 꽉 잡기
    # ==========================================
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time': use_sim_time}]
    )

    # 🔥 tf_gnss_fix와 time_sync_node는 중복 및 불필요하여 삭제됨

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_combat_nav, 'launch', 'localization.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ==========================================
    # [Step 2] 2초 뒤 실행: 안정된 좌표계 위에 맵 띄우기
    # ==========================================
    map_yaml_file = os.path.join(pkg_combat_nav, 'map', 'incheon', 'incheon.yaml')
    
    map_server_node = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen', parameters=[{'yaml_filename': map_yaml_file}, {'use_sim_time': use_sim_time}]
    )
    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
        output='screen', parameters=[{'use_sim_time': use_sim_time}, {'autostart': True}, {'node_names': ['map_server']}]
    )
    map_timer = TimerAction(period=2.0, actions=[map_server_node, lifecycle_manager_map])

    # ==========================================
    # [Step 3] 5초 뒤 실행: Navigation 엔진 가동
    # ==========================================
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_combat_nav, 'launch', 'navigation.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    nav_timer = TimerAction(period=5.0, actions=[navigation_launch])

    # ==========================================
    # [Step 4] 7초 뒤 실행: RViz 켜기
    # ==========================================
    rviz2_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config_dir], parameters=[{'use_sim_time': use_sim_time}], output='screen'
    )
    rviz_timer = TimerAction(period=7.0, actions=[rviz2_node])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        rsp_node,
        localization_launch,
        map_timer,
        nav_timer,
        rviz_timer
    ])
