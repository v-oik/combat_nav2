import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    pkg_dir = get_package_share_directory('combat_robot_nav2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    param_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    # BT XML 경로를 패키지 share 기준으로 동적 해석 (절대경로 하드코딩 제거)
    bt_dir = os.path.join(pkg_dir, 'include')
    param_substitutions = {
        'default_nav_to_pose_bt_xml': os.path.join(bt_dir, 'single_plan_bt.xml'),
        'default_nav_through_poses_bt_xml': os.path.join(bt_dir, 'way_plan_bt.xml'),
    }
    configured_params = RewrittenYaml(
        source_file=param_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': configured_params,
            'use_sim_time': use_sim_time,
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        navigation_launch
    ])
