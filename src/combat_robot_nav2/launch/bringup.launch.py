import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 패키지 이름 수정됨
    pkg_dir = get_package_share_directory('combat_robot_nav2')
    launch_dir = os.path.join(pkg_dir, 'launch')

    launch_args = {'use_sim_time': 'true'}.items()

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'gazebo.launch.py')),
        launch_arguments=launch_args
    )

    localization_launch = TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'localization.launch.py')),
            launch_arguments=launch_args)]
    )

    navigation_launch = TimerAction(
        period=10.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation.launch.py')),
            launch_arguments=launch_args)]
    )

    map_rviz_launch = TimerAction(
        period=15.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'map_rviz.launch.py')),
            launch_arguments=launch_args)]
    )

    return LaunchDescription([
        gazebo_launch,
        localization_launch,
        navigation_launch,
        map_rviz_launch
    ])