import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('combat_robot_nav2')
    # map_file = os.path.join(pkg_dir, 'map','korea_university_sejong_cam_v2.1' ,'map.yaml')
    # map_file = os.path.join(pkg_dir, 'map', 'korea_university_sejong_cam_v2.1', 'open_field.yaml')
    map_file = os.path.join(pkg_dir, 'map', 'korea_university_sejong_cam_v2.1', 'sejong_clean_map.yaml')
    # 🚀 수정: Nav2 전용 RViz 설정 파일 로드
    rviz_config_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')
    

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file}, {'use_sim_time': True}] # 🚀 수정
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen',
        parameters=[{'use_sim_time': True}, # 🚀 수정
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir], # 🚀 수정: Nav2 패널 자동 열림
        parameters=[{'use_sim_time': True}], # 🚀 수정
        output='screen'
    )

    return LaunchDescription([
        map_server_node,
        lifecycle_manager_node,
        rviz2_node
    ])