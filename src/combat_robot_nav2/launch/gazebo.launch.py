import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg_bird_nav = get_package_share_directory('combat_robot_nav2')
    pkg_vehicle_desc = get_package_share_directory('combat_robot_description')
    
    # 🔥 핵심 추가: 가제보(Gazebo)가 3D 모델(Mesh)을 찾을 수 있도록 ROS 2 패키지 경로를 알려줍니다.
    # pkg_vehicle_desc는 '.../share/combat_robot_description' 이므로, 상위 폴더인 '.../share'를 경로로 등록합니다.
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_vehicle_desc, '..')
    )

    world_file = '/home/skyautonet/combat_nav2/src/combat_robot_nav2/world/sejong.world'

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    xacro_file = os.path.join(pkg_vehicle_desc, 'urdf', 'robot.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file])

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time': True}]
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'combat_robot', '-z', '0.3'],
        output='screen'
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/front_lidar/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/sensing/imu/imu_data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/sensing/gnss/nav_sat_fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/front_lidar/scan/points', '/front_lidar/pointcloud'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    tf_gnss_fix = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'gnss_base_link', 'gnss_sensor'],
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path,  # 방금 추가한 환경 변수를 실행 목록에 넣습니다.
        gazebo_launch,
        node_robot_state_publisher,
        spawn_entity,
        bridge_node,
        tf_gnss_fix
    ])