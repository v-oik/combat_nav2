import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_dir = get_package_share_directory('combat_robot_nav2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    param_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': param_file,
            'use_sim_time': use_sim_time,  # 🚀 수정: 시뮬레이션 시간 동기화
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        navigation_launch
    ])