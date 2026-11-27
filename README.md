# System Bringup Integration Layer

English | [中文](./README.zh-CN.md)

---

![Cover](./image/cover.gif)


This layer is responsible for combining models, hardware, controllers, sensors, and teleoperation/monitoring scripts into a bootable real vehicle system.

## Package List

- `swerve_bringup`: Chassis launch files, controller_manager configuration, joint recording/plotting, teleoperation, and VR/waist chassis linkage scripts.

## Main Entry Points

- `swerve_bringup/launch/swerve_drive.launch.py`: Launches chassis model, ros2_control, joint_state_broadcaster, swerve_drive_controller.
- `swerve_bringup/launch/d435.launch.py`: Launches D435 related drivers/configuration.
- `swerve_bringup/scripts/swerve_teleop.py`: Keyboard teleoperation node that publishes `cmd_vel` to control chassis motion.

## Starting the Motion Control Layer

The motion control layer entry point is:

```bash
swerve_bringup/launch/swerve_drive.launch.py
```

Before starting, enter the workspace and load the environment:

```bash
cd ~/openflex_all/openflex_ws
source install/setup.bash
```

Start chassis motion control:

```bash
ros2 launch swerve_bringup swerve_drive.launch.py
```

This launch file will start:

- `robot_state_publisher`
- `ros2_control_node`
- `joint_state_broadcaster`
- `swerve_drive_controller`
- `rviz2`, enabled by default

Common launch parameters:

```bash
ros2 launch swerve_bringup swerve_drive.launch.py use_rviz:=false
```

```bash
ros2 launch swerve_bringup swerve_drive.launch.py \
  steering_can_interface:=can5 \
  driving_can_interface:=can4 \
  max_wheel_speed:=2.0 \
  wheel_accel_limit:=1.2
```

Parameter descriptions:

- `use_rviz`: Whether to launch RViz, default `true`.
- `steering_can_interface`: Steering motor CAN interface, default `can5`.
- `driving_can_interface`: Drive motor CAN interface, default `can4`.
- `max_wheel_speed`: Swerve controller wheel speed limit, unit `m/s`, default `2.0`.
- `wheel_accel_limit`: Wheel speed change rate limit, unit `m/s^2`, default `1.2`.

After starting, you can check controllers and topics:

```bash
ros2 control list_controllers
ros2 topic list | grep -E "cmd_vel|odom|joint_states|tf"
```

## Starting the Keyboard Control Layer

The keyboard control script is:

```bash
swerve_bringup/scripts/swerve_teleop.py
```

In another terminal, load the same workspace environment:

```bash
cd ~/openflex_all/openflex_ws
source install/setup.bash
```

Start keyboard control:

```bash
ros2 run swerve_bringup swerve_teleop.py
```

This node publishes `geometry_msgs/msg/Twist` to `/cmd_vel`. The default `swerve_drive.launch.py` uses `controllers.yaml`, and the controller also subscribes to `/cmd_vel`, so they work together directly.

Common keys:

- `i`: Forward
- `,`: Backward
- `j`: Turn left
- `l`: Turn right
- `k`: Stop
- `a`: Strafe left
- `d`: Strafe right
- `u/o/m/.`: Diagonal motion or diagonal with rotation
- `q/z`: Simultaneously increase/decrease linear and angular velocity
- `w/x`: Only increase/decrease linear velocity
- `e/c`: Only increase/decrease angular velocity
- `Ctrl-C`: Exit, will send zero velocity once on exit

## Responsibility Boundaries

This layer performs system assembly and does not contain core algorithms. Motion control algorithms are in `motion_control_layer`, hardware protocols are in `hardware_sensor_layer`, and navigation and mapping launches are maintained by corresponding layers.

## License

This package is licensed under Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0).

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd.

For details, please refer to the [LICENSE](LICENSE) file or visit: http://creativecommons.org/licenses/by-nc-sa/4.0/

## Acknowledgments

This package is part of the OpenFlex full-body humanoid robot platform ecosystem, developed specifically for research and industrial applications in the humanoid robotics field.

---

## 📞 Contact Us

### Chengdu Changshu Robot Co., Ltd.
**Chengdu Changshu Robotics Co., Ltd.**

| Contact | Information |
|---------|-------------|
| 📧 Email | openarmrobot@gmail.com |
| 📱 Phone/WeChat | +86-17746530375 |
| 🌐 Website | https://openarmx.com/ |
| 🌐 Docs | http://docs.openarmx.com/ |
| 📍 Address | Tianjin Xiqing District · Daochao Robot Experience Base (City of Tomorrow) · Tianjin Humanoid Robot Center |
| 👤 Contact Person | Mr. Wang |
