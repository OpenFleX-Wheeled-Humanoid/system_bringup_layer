# swerve_bringup

English | [中文](./README-CN.md)

---

Launch files, configuration, and utility scripts for the 4WS4WD swerve drive chassis.

## Description

This package is the top-level bringup layer that ties together the URDF model, hardware interface, and controller into a ready-to-run system. It provides:

- Main launch file for the swerve drive system
- Controller configuration files for different operating modes
- Keyboard teleop script
- VR teleop and lift control nodes
- D435 camera launch

## Launch Files

### swerve_drive.launch.py

Main launch file that starts the complete swerve drive system:

- `robot_state_publisher` (publishes URDF TF)
- `controller_manager` (ros2_control node)
- `joint_state_broadcaster` (publishes joint states)
- `swerve_drive_controller` (motion control)
- RViz (optional)

```bash
ros2 launch swerve_bringup swerve_drive.launch.py
```

Launch arguments:

| Argument | Default | Description |
|----------|---------|-------------|
| `use_rviz` | `true` | Start RViz visualization |
| `steering_can_interface` | `can5` | RS06 steering CAN bus |
| `driving_can_interface` | `can4` | UM drive CAN bus |
| `max_wheel_speed` | `2.0` | Wheel speed limit (m/s) |
| `wheel_accel_limit` | `1.2` | Wheel acceleration limit (m/s^2) |

### d435.launch.py

Launches the Intel RealSense D435 camera node with predefined frame IDs and profiles.

```bash
ros2 launch swerve_bringup d435.launch.py
```

## Configuration Files

| File | Description |
|------|-------------|
| `controllers.yaml` | Default controller config (teleop/standalone mode) |
| `controllers_mapping.yaml` | Mapping mode config (odom TF enabled, higher speed limit) |
| `controllers_navigation.yaml` | Navigation mode config (odom TF disabled, subscribes to `/cmd_vel_safe`) |

## Scripts

| Script | Description |
|--------|-------------|
| `swerve_teleop.py` | Keyboard teleop for omnidirectional driving |
| `vr_teleop_node.py` | VR controller teleop |
| `vr_lift_control_node.py` | VR-driven lift control |
| `waist_chassis_control_node.py` | Waist-chassis coordination |
| `vr_monitor.py` | VR status monitoring |

### Keyboard Teleop Usage

```bash
ros2 run swerve_bringup swerve_teleop.py
```

Key bindings: `i`=forward, `,`=backward, `j`=turn left, `l`=turn right, `a`=strafe left, `d`=strafe right, `k`=stop. Use `q/z` to adjust speed.

## Build

```bash
cd ~/openflex_all/openflex_ws
colcon build --packages-select swerve_bringup
source install/setup.bash
```

## Prerequisites

1. CAN interfaces must be configured and up before launching:
   ```bash
   sudo ip link set can5 type can bitrate 1000000 && sudo ip link set can5 up
   sudo ip link set can4 type can bitrate 1000000 && sudo ip link set can4 up
   ```

2. Required packages must be built: `swerve_description`, `swerve_hardware`, `swerve_controller`

## Dependencies

- `controller_manager`
- `joint_state_broadcaster`
- `robot_state_publisher`
- `swerve_controller`
- `swerve_description`
- `swerve_hardware`
- `xacro`
- `rviz2`
- `rclpy`
- `geometry_msgs`
- `realsense2_camera`

## License

Apache-2.0
