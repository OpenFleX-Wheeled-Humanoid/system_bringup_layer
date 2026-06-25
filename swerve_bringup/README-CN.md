# swerve_bringup

[English](./README.md) | 中文

---

四转四驱舵轮底盘的启动文件、配置和工具脚本。

## 简介

本包是顶层启动层，将 URDF 模型、硬件接口和控制器整合为可直接运行的系统。提供：

- 舵轮底盘主启动文件
- 不同工作模式的控制器配置文件
- 键盘遥控脚本
- VR 遥控和升降控制节点
- D435 相机启动

## 启动文件

### swerve_drive.launch.py

主启动文件，启动完整舵轮底盘系统：

- `robot_state_publisher`（发布 URDF TF）
- `controller_manager`（ros2_control 节点）
- `joint_state_broadcaster`（发布关节状态）
- `swerve_drive_controller`（运动控制）
- RViz（可选）

```bash
ros2 launch swerve_bringup swerve_drive.launch.py
```

启动参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_rviz` | `true` | 启动 RViz 可视化 |
| `steering_can_interface` | `can5` | RS06 转向 CAN 总线 |
| `driving_can_interface` | `can4` | UM 驱动 CAN 总线 |
| `max_wheel_speed` | `2.0` | 轮速上限（m/s） |
| `wheel_accel_limit` | `1.2` | 轮速变化率上限（m/s^2） |

### d435.launch.py

启动 Intel RealSense D435 相机节点，使用预定义的坐标系 ID 和分辨率配置。

```bash
ros2 launch swerve_bringup d435.launch.py
```

## 配置文件

| 文件 | 说明 |
|------|------|
| `controllers.yaml` | 默认控制器配置（遥控/独立模式） |
| `controllers_mapping.yaml` | 建图模式配置（odom TF 启用，更高速度上限） |
| `controllers_navigation.yaml` | 导航模式配置（odom TF 禁用，订阅 `/cmd_vel_safe`） |

## 脚本

| 脚本 | 说明 |
|------|------|
| `swerve_teleop.py` | 全向键盘遥控 |
| `vr_teleop_node.py` | VR 手柄遥控 |
| `vr_lift_control_node.py` | VR 驱动升降控制 |
| `waist_chassis_control_node.py` | 腰部-底盘协调控制 |
| `vr_monitor.py` | VR 状态监控 |

### 键盘遥控使用

```bash
ros2 run swerve_bringup swerve_teleop.py
```

按键：`i`=前进、`,`=后退、`j`=左转、`l`=右转、`a`=左移、`d`=右移、`k`=停止。使用 `q/z` 调整速度。

## 编译

```bash
cd ~/openflex_all/openflex_ws
colcon build --packages-select swerve_bringup
source install/setup.bash
```

## 前置条件

1. 启动前需配置并启用 CAN 接口：
   ```bash
   sudo ip link set can5 type can bitrate 1000000 && sudo ip link set can5 up
   sudo ip link set can4 type can bitrate 1000000 && sudo ip link set can4 up
   ```

2. 需要先编译以下包：`swerve_description`、`swerve_hardware`、`swerve_controller`

## 依赖

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

## 许可证

Apache-2.0
