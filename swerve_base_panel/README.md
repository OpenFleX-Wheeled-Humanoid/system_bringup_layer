# swerve_base_panel

English | [中文](./README-CN.md)

---

RViz2 hold-to-move manual control panel for the OpenFlex swerve base, with multi-key S-curve keyboard control.

## Description

This package provides a RViz2 panel plugin for manual control of the OpenFlex 4WS4WD swerve drive chassis. It supports both mouse click and keyboard control modes, publishes `geometry_msgs/Twist` messages to `/cmd_vel`, and uses a hold-to-move design for safe operation.

Features:

- Hold-to-move directional buttons (forward / backward / left / right)
- Rotate left / rotate right buttons
- Emergency STOP button
- Keyboard control toggle with multi-key combinations and S-curve ramping
- Adjustable speed with slider and +/- buttons
- Real-time linear and angular speed display
- Status indicator

## Panel Controls

### Motion Buttons

| Button | Key | Action |
|--------|-----|--------|
| ⬆ Forward | `W` or `Up` | Move forward |
| ⬇ Backward | `S` or `Down` | Move backward |
| ⬅ Left | `A` or `Left` | Strafe left |
| ➡ Right | `D` or `Right` | Strafe right |
| ↺ Rotate Left | `Q` | Rotate counter-clockwise |
| ↻ Rotate Right | `E` | Rotate clockwise |
| **STOP** | `Space` | Immediately command zero velocity |

> **Hold-to-move**: The robot moves while the button is held and stops when released.

### Speed Control

| Control | Key | Action |
|---------|-----|--------|
| Speed Up | `Z` | Increase speed by 10% |
| Speed Down | `C` | Decrease speed by 10% |
| Speed Slider | — | Drag to set speed percentage |

Speed range: 25% ~ 500% (default 100%)

Default speeds:
- Linear: 0.20 m/s (range: 0.05 ~ 1.00 m/s)
- Angular: 0.20 rad/s (range: 0.05 ~ 1.00 rad/s)

### Keyboard Control

Click the **键盘控制：关 / 键盘控制：开** button to toggle keyboard control mode. When enabled, translation and rotation keys can be held together, for example `W + Q`; opposite keys on the same axis cancel. Each keyboard axis uses a 20 Hz S-curve (0.5 s acceleration time, smoothness 0.3), and releasing keys ramps down smoothly. `Space` stops immediately and clears input state. A held mouse motion button owns the command until release and does not resume stale keyboard input.

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Chassis velocity command |

The topic name is configurable via the RViz panel config.

## Usage

### 1. Add Panel in RViz2

1. Start RViz2
2. Click **Panels → Add New Panel**
3. Select **swerve_base_panel / SwerveBasePanel**
4. The panel appears in the RViz2 interface

### 2. Use with Integrated Bringup

The panel is pre-configured in the integrated system RViz layout:

```bash
ros2 launch openarmx_integrated_bringup integrated_robot_bringup.launch.py
```

## Build

```bash
cd /home/openflex/openflex_all_new/experimental_version/openflex_ws
colcon build --packages-select swerve_base_panel
source install/setup.bash
```

## Dependencies

- `geometry_msgs`
- `pluginlib`
- `rclcpp`
- `rviz_common`
- `rviz2`
- `Qt5` (Core, Widgets)

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
