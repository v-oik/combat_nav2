from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    rs_lidar = ExecuteProcess(
        cmd=['ros2', 'launch', 'rslidar_sdk', 'start.py'],
        output='screen'
    )
    xsens_imu = ExecuteProcess(
        cmd=['ros2', 'launch', 'bluespace_ai_xsens_mti_driver', 'xsens_mti_node.launch.py'],
        output='screen'
    )
    gnss_node = Node(
        package='combat_robot_nav2',
        executable='gnss_heading.py',
        output='screen',
        parameters=[{
            'port': '/dev/ttyUSB1',
            'baud': 921600,
            'heading_frame_id': 'gps',
            'gps_frame_id': 'gps',
            'antenna_yaw_offset_deg': 274.0,   # q=4 주행 보정값 (이전 254.1은 정지측정이라 부정확)
            'max_sigma_horizontal': 5.0,       # 임시 완화 (RTK 안될 때 단독 GPS로 /fix 발행)
            'heading_yaw_variance': 0.1,       # 주행 시 GPS heading 신뢰도 (q5 노이즈 대응)
            'stationary_yaw_variance': 25.0,   # 정지 시 heading 억제 (발행 차단)
            'stationary_speed_threshold_mps': 0.1,  # vx_max 0.2보다 낮게 — 주행 중 heading 활성
            'heading_filter_window': 20,       # circular MA 평활(~1s)
        }]
    )

    # xsens gyro z-bias 온라인 보정 → /imu/data_corrected 발행 (자력계 차폐로 gyro rate만 사용).
    # EKF가 이 보정 gyro로 회전 추적. EKF(bringup)보다 먼저 떠야 함.
    gyro_bias_comp = Node(
        package='combat_robot_nav2',
        executable='gyro_bias_comp.py',
        output='screen',
        parameters=[{
            'init_bias_z': 0.004459,   # 2026-06-01 측정 (15.33 deg/min)
            'stationary_v': 0.03,
            'stationary_w': 0.02,
            'bias_alpha': 0.002,
        }]
    )

    return LaunchDescription([
        TimerAction(period=1.0, actions=[rs_lidar]),
        xsens_imu,
        gnss_node,
        gyro_bias_comp,
    ])