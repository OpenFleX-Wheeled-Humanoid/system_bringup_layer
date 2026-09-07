# swerve_base_panel

[English](./README.md) | 中文

---

OpenFlex 舵轮底盘的 RViz2 按压移动手动控制面板，支持键盘组合键和 S 曲线控制。

## 简介

本包提供一个 RViz2 面板插件，用于手动控制 OpenFlex 四转四驱舵轮底盘。支持鼠标点击和键盘两种控制方式，向 `/cmd_vel` 话题发布 `geometry_msgs/Twist` 消息，采用按压移动设计，松开即停，操作安全。

主要特性：

- 按压移动方向按钮（前进 / 后退 / 左移 / 右移）
- 左转 / 右转旋转按钮
- 急停 STOP 按钮
- 键盘控制开关（支持平移、旋转组合键和 S 曲线加减速）
- 速度滑块与加减速按钮
- 实时显示线速度和角速度
- 状态指示栏

## 面板控制说明

### 运动按钮

| 按钮 | 快捷键 | 动作 |
|------|--------|------|
| ⬆ 前进 | `W` 或 `↑` | 向前移动 |
| ⬇ 后退 | `S` 或 `↓` | 向后移动 |
| ⬅ 左移 | `A` 或 `←` | 向左平移 |
| ➡ 右移 | `D` 或 `→` | 向右平移 |
| ↺ 左转 | `Q` | 逆时针旋转 |
| ↻ 右转 | `E` | 顺时针旋转 |
| **STOP** | `Space` | 立即清零速度 |

> **按压移动**：按住按钮时机器人持续运动，松开立即停止。

### 速度控制

| 控件 | 快捷键 | 说明 |
|------|--------|------|
| 加速 | `Z` | 速度增加 10% |
| 减速 | `C` | 速度减少 10% |
| 速度滑块 | — | 拖动设置速度百分比 |

速度范围：25% ～ 500%（默认 100%）

默认速度：
- 线速度：0.20 m/s（范围：0.05 ～ 1.00 m/s）
- 角速度：0.20 rad/s（范围：0.05 ～ 1.00 rad/s）

### 键盘控制

点击 **键盘控制：关 / 键盘控制：开** 按钮切换键盘控制模式。开启后可直接通过键盘控制底盘运动。平移键和旋转键可以同时按住，例如 `W + Q`；同一轴相反方向同时按下时相互抵消。键盘速度在 20 Hz 下使用 0.5 s 加速时间和 0.3 平滑系数的 S 曲线，松键后平滑减速；`Space` 立即停止并清除输入状态。鼠标按住运动按钮时由鼠标独占控制，松开后不会恢复旧的键盘输入。

## 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 底盘速度指令 |

话题名称可通过 RViz 面板配置保存。

## 使用方法

### 1. 在 RViz2 中添加面板

1. 启动 RViz2
2. 点击菜单 **Panels → Add New Panel**
3. 选择 **swerve_base_panel / SwerveBasePanel**
4. 面板出现在 RViz2 界面中

### 2. 随整机系统启动

本面板已预配置在整机 RViz 布局中：

```bash
ros2 launch openarmx_integrated_bringup integrated_robot_bringup.launch.py
```

启动后在 RViz2 界面中即可看到底盘控制面板。

## 编译

```bash
cd /home/openflex/openflex_all_new/experimental_version/openflex_ws
colcon build --packages-select swerve_base_panel
source install/setup.bash
```

## 依赖

- `geometry_msgs`
- `pluginlib`
- `rclcpp`
- `rviz_common`
- `rviz2`
- `Qt5`（Core、Widgets）

## 许可证

本包通过 知识共享 署名-非商业性使用-相同方式共享 4.0 国际许可协议 (CC BY-NC-SA 4.0) 进行许可。

版权所有 (c) 2026 成都长数机器人有限公司 (Chengdu Changshu Robot Co., Ltd.)

详情请参阅 [LICENSE](LICENSE) 文件或访问：http://creativecommons.org/licenses/by-nc-sa/4.0/

## 致谢

本包是 OpenFlex 全身人形机器人平台生态系统的一部分，专为人形机器人领域的研究和工业应用而开发。

---

## 📞 联系我们

### 成都长数机器人有限公司
**Chengdu Changshu Robotics Co., Ltd.**

| 联系方式 | 信息 |
|---------|------|
| 📧 邮箱 | openarmrobot@gmail.com |
| 📱 电话/微信 | +86-17746530375 |
| 🌐 官网 | https://openarmx.com/ |
| 🌐 文档 | http://docs.openarmx.com/ |
| 📍 地址 | 天津市西青区・稻潮机器人体验基地（明日之城）・天津市人形机器人中心 |
| 👤 联系人 | 王先生 |
