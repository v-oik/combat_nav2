# tin3_description

TIN3 robot URDF description and meshes.

---

## Package Contents

```
tin3_description/
├── urdf/                    # Robot description files
│   ├── robot.urdf.xacro     # Main entry point
│   ├── chassis.xacro        # Chassis, tracks, wheels
│   ├── gimbal.xacro         # Pan-tilt gimbal
│   ├── camera.xacro         # RGB + IR cameras
│   ├── rifle.xacro          # Rifle holder
│   ├── sensors.xacro        # LiDAR, IMU, GPS
│   ├── macros.xacro         # Inertia calculation macros
│   ├── gazebo.xacro         # Gazebo plugins
│   ├── gazebo_control.xacro # Gazebo controllers
│   └── materials.xacro      # Material definitions
├── meshes/                  # 3D models (DAE format)
│   ├── chassis.dae
│   ├── left_track.dae
│   ├── right_track.dae
│   ├── gimbal_base.dae
│   ├── gimbal_pan.dae
│   ├── gimbal_tilt.dae
│   ├── camera_holder.dae
│   ├── rifle.dae
│   └── lidar.dae
├── launch/
│   └── view_robot.launch.py # View robot in RViz
├── rviz/                    # RViz configurations
└── docs/                    # Documentation
```

---

## Quick Start

```bash
# Build
cd <your_workspace>
colcon build --packages-select tin3_description
source install/setup.bash

# View robot in RViz
ros2 launch tin3_description view_robot.launch.py
```

---

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_gui` | `true` | Show joint_state_publisher GUI |

---

## URDF Structure

```
base_footprint
└── base_link (chassis)
    ├── left_track_visual
    ├── right_track_visual
    ├── left_front_wheel
    ├── left_mid_wheel
    ├── left_rear_wheel
    ├── right_front_wheel
    ├── right_mid_wheel
    ├── right_rear_wheel
    ├── lidar_link
    ├── imu_link
    ├── gps_link
    └── gimbal_base_link
        └── gimbal_pan_link
            └── gimbal_tilt_link
                ├── camera_holder_link
                │   ├── rgb_camera_link → rgb_camera_optical_frame
                │   └── ir_camera_link → ir_camera_optical_frame
                └── rifle_link
```

**Note:** At runtime, the full TF tree has `odom` as root (published by EKF or sim): `odom → base_footprint → base_link → ...`

---

## Dependencies

- `robot_state_publisher`
- `joint_state_publisher_gui`
- `xacro`
- `rviz2`

---

## Usage with Other Packages

This package provides robot description only. For simulation:

```bash
# Use with tin3_gz_simulation
ros2 launch tin3_gz_simulation sim.launch.py
```