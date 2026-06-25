#!/usr/bin/env python3
# Copyright 2024 Your Name
# Licensed under the Apache License, Version 2.0

"""
VR摇杆遥控节点

功能:
1. 订阅左右手VR摇杆数据
2. 将摇杆输入转换为底盘速度命令 (/cmd_vel)
3. 支持死区处理、速度限制和平滑控制

控制映射 (方案2 - 双手协同):
  左手摇杆:
    - Y轴 (前后推) → linear.x (底盘前进/后退)
    - X轴 (左右推) → linear.y (底盘左右平移)

  右手摇杆:
    - X轴 (左右推) → angular.z (底盘旋转)

    按键功能:
      - X键 (左手): 升降台下降（由ld2_can_lift_node处理）
      - Y键 (左手): 升降台上升（由ld2_can_lift_node处理）

    速度控制:
      - 支持 expo 非线性曲线，小幅推杆更细，大幅推杆仍可输出满速
      - Pico 主界面可分别下发平移/旋转速度上限
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import math


class SCurveController:
    """
    S型加减速控制器（基于时间参数）

    使用Jerk限制实现平滑的S型速度曲线
    用户只需设置加速时间和平滑度，自动计算物理参数
    """

    def __init__(self, max_velocity=1.0, acceleration_time=0.5, smoothness=0.3, dt=0.05):
        """
        初始化S型控制器

        参数:
            max_velocity: 最大速度 (m/s 或 rad/s)
            acceleration_time: 从0加速到最大速度的时间 (秒)
            smoothness: 平滑度 (0-1)，越大越平滑
            dt: 控制周期 (秒)
        """
        self.max_velocity = max_velocity
        self.acceleration_time = max(0.1, acceleration_time)
        self.smoothness = max(0.05, min(0.95, smoothness))
        self.dt = dt

        # 根据时间参数自动计算物理参数
        self._calculate_physical_params()

        # 状态变量
        self.current_velocity = 0.0
        self.current_acceleration = 0.0

    def _calculate_physical_params(self):
        """根据时间参数计算最大加速度和最大加加速度"""
        # 计算最大加速度
        self.max_acceleration = self.max_velocity / self.acceleration_time

        # 计算加加速时间（S曲线的弯曲部分）
        jerk_time = self.acceleration_time * self.smoothness

        # 计算最大加加速度
        self.max_jerk = self.max_acceleration / max(jerk_time, 0.01)

    def update(self, target_velocity):
        """
        更新速度，返回平滑后的速度

        参数:
            target_velocity: 目标速度

        返回:
            平滑后的当前速度
        """
        # 限制目标速度
        target_velocity = max(-self.max_velocity, min(self.max_velocity, target_velocity))

        # 计算速度误差
        velocity_error = target_velocity - self.current_velocity

        # 计算期望的加速度
        desired_acceleration = velocity_error / self.dt

        # 限制加速度
        desired_acceleration = max(-self.max_acceleration,
                                   min(self.max_acceleration, desired_acceleration))

        # 计算加速度变化（jerk）
        acceleration_change = desired_acceleration - self.current_acceleration

        # 限制jerk（加加速度）
        max_jerk_change = self.max_jerk * self.dt
        if abs(acceleration_change) > max_jerk_change:
            acceleration_change = max_jerk_change if acceleration_change > 0 else -max_jerk_change

        # 更新加速度
        self.current_acceleration += acceleration_change
        self.current_acceleration = max(-self.max_acceleration,
                                       min(self.max_acceleration, self.current_acceleration))

        # 更新速度
        self.current_velocity += self.current_acceleration * self.dt
        self.current_velocity = max(-self.max_velocity,
                                    min(self.max_velocity, self.current_velocity))

        return self.current_velocity

    def reset(self):
        """重置状态"""
        self.current_velocity = 0.0
        self.current_acceleration = 0.0

    def get_state(self):
        """返回当前 S 曲线参数和状态，用于启动日志和调试。"""
        return {
            'max_acceleration': self.max_acceleration,
            'max_jerk': self.max_jerk,
            'current_velocity': self.current_velocity,
            'current_acceleration': self.current_acceleration,
        }

    def update_params(self, max_velocity=None, acceleration_time=None, smoothness=None):
        """动态更新参数"""
        if max_velocity is not None:
            self.max_velocity = max_velocity
        if acceleration_time is not None:
            self.acceleration_time = max(0.1, acceleration_time)
        if smoothness is not None:
            self.smoothness = max(0.05, min(0.95, smoothness))
        self._calculate_physical_params()


class VRTeleopNode(Node):
    def __init__(self):
        super().__init__('vr_teleop')

        # 声明参数（运动模式）
        self.declare_parameter('max_linear_speed', 0.8)      # 最大线速度 (m/s)
        self.declare_parameter('boost_linear_speed', 2.0)    # 兼容旧参数，当前不再使用
        self.declare_parameter('max_angular_speed', 1.5)     # 最大角速度 (rad/s)
        self.declare_parameter('joystick_deadzone', 0.15)    # 摇杆死区 (0-1)
        self.declare_parameter('linear_expo', 1.0)           # 线速度摇杆指数曲线，1.0=线性
        self.declare_parameter('angular_expo', 1.0)          # 角速度摇杆指数曲线，1.0=线性
        self.declare_parameter('enable_on_startup', True)    # 启动时是否启用控制
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')  # 发布的话题名
        self.declare_parameter('publish_rate', 20.0)         # 发布频率 (Hz)
        self.declare_parameter('estop_toggle_topic', '')     # 外部紧停切换话题（预留给摇杆按下）
        self.declare_parameter('use_vr_chassis_speed_config', False)
        self.declare_parameter('linear_speed_topic', '/pico_chassis/max_linear_speed')
        self.declare_parameter('angular_speed_topic', '/pico_chassis/max_angular_speed')

        # S型加减速参数
        self.declare_parameter('enable_scurve', True)        # 启用S型加减速
        self.declare_parameter('acceleration_time', 0.5)     # 加速时间 (秒)
        self.declare_parameter('smoothness', 0.3)            # 平滑度 (0-1)

        # 获取参数
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.boost_linear_speed = self.get_parameter('boost_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.joystick_deadzone = self.get_parameter('joystick_deadzone').value
        self.linear_expo = max(1.0, self.get_parameter('linear_expo').value)
        self.angular_expo = max(1.0, self.get_parameter('angular_expo').value)
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.estop_toggle_topic = self.get_parameter('estop_toggle_topic').value
        self.use_vr_chassis_speed_config = self.get_parameter('use_vr_chassis_speed_config').value
        self.linear_speed_topic = self.get_parameter('linear_speed_topic').value
        self.angular_speed_topic = self.get_parameter('angular_speed_topic').value

        # S型加减速参数
        self.enable_scurve = self.get_parameter('enable_scurve').value
        self.acceleration_time = self.get_parameter('acceleration_time').value
        self.smoothness = self.get_parameter('smoothness').value

        # 状态变量
        self.enabled = self.get_parameter('enable_on_startup').value
        self.estop_active = False
        self.current_max_linear_speed = self.max_linear_speed
        self.current_max_angular_speed = self.max_angular_speed

        # 摇杆输入缓存 (初始化为0)
        self.left_joystick_x = 0.0
        self.left_joystick_y = 0.0
        self.right_joystick_x = 0.0
        self.right_joystick_y = 0.0

        # VR数据超时检测
        self.last_vr_data_time = self.get_clock().now()
        self.vr_timeout = 0.5  # 秒
        self.last_timeout_warning = self.get_clock().now()  # 用于节流警告

        # 创建S型加减速控制器（3个独立控制器：vx, vy, wz）
        dt = 1.0 / self.publish_rate  # 控制周期
        if self.enable_scurve:
            self.scurve_vx = SCurveController(
                max_velocity=self.current_max_linear_speed,
                acceleration_time=self.acceleration_time,
                smoothness=self.smoothness,
                dt=dt
            )
            self.scurve_vy = SCurveController(
                max_velocity=self.current_max_linear_speed,
                acceleration_time=self.acceleration_time,
                smoothness=self.smoothness,
                dt=dt
            )
            self.scurve_wz = SCurveController(
                max_velocity=self.current_max_angular_speed,
                acceleration_time=self.acceleration_time,
                smoothness=self.smoothness,
                dt=dt
            )
        else:
            self.scurve_vx = None
            self.scurve_vy = None
            self.scurve_wz = None

        # 创建发布者
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )

        # 全局紧停状态发布 (transient_local 保证后来订阅者也能收到最新状态)
        estop_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.estop_pub = self.create_publisher(Bool, '/vr_estop_active', estop_qos)

        # 创建订阅者 - 左手摇杆
        self.left_joy_x_sub = self.create_subscription(
            Float32,
            '/pico_left_controller/joystick_x',
            self.left_joystick_x_callback,
            10
        )
        self.left_joy_y_sub = self.create_subscription(
            Float32,
            '/pico_left_controller/joystick_y',
            self.left_joystick_y_callback,
            10
        )

        # 创建订阅者 - 右手摇杆
        self.right_joy_x_sub = self.create_subscription(
            Float32,
            '/pico_right_controller/joystick_x',
            self.right_joystick_x_callback,
            10
        )
        self.right_joy_y_sub = self.create_subscription(
            Float32,
            '/pico_right_controller/joystick_y',
            self.right_joystick_y_callback,
            10
        )

        self.linear_speed_sub = None
        self.angular_speed_sub = None
        if self.use_vr_chassis_speed_config:
            self.linear_speed_sub = self.create_subscription(
                Float32,
                self.linear_speed_topic,
                self.linear_speed_callback,
                10
            )
            self.angular_speed_sub = self.create_subscription(
                Float32,
                self.angular_speed_topic,
                self.angular_speed_callback,
                10
            )

        self.estop_toggle_sub = None
        self.last_estop_toggle = False
        if self.estop_toggle_topic:
            self.estop_toggle_sub = self.create_subscription(
                Bool,
                self.estop_toggle_topic,
                self.estop_toggle_callback,
                10
            )

        # 创建定时器 - 定期发布cmd_vel
        timer_period = 1.0 / self.publish_rate
        self.publish_timer = self.create_timer(timer_period, self.publish_cmd_vel)

        # 创建定时器 - 检查VR数据超时
        self.timeout_timer = self.create_timer(0.1, self.check_vr_timeout)

        self.get_logger().info('=' * 60)
        self.get_logger().info('VR Teleop Node Started')
        self.get_logger().info('=' * 60)
        self.get_logger().info(
            f'Max linear speed: {self.current_max_linear_speed} m/s, '
            f'max angular speed: {self.current_max_angular_speed} rad/s')
        self.get_logger().info(f'Joystick deadzone: {self.joystick_deadzone}')
        self.get_logger().info(
            f'Joystick expo: linear={self.linear_expo:.2f}, angular={self.angular_expo:.2f}')
        self.get_logger().info(f'Publishing to: {self.cmd_vel_topic} @ {self.publish_rate} Hz')
        self.get_logger().info(f'Control enabled: {self.enabled}')
        self.get_logger().info(
            f'VR speed config: {"enabled" if self.use_vr_chassis_speed_config else "disabled"}')
        if self.use_vr_chassis_speed_config:
            self.get_logger().info(
                f'Runtime speed topics: linear={self.linear_speed_topic}, angular={self.angular_speed_topic}')
        if self.estop_toggle_topic:
            self.get_logger().info(f'External E-stop toggle topic: {self.estop_toggle_topic}')

        # S型加减速信息
        if self.enable_scurve:
            self.get_logger().info(f'S-Curve smoothing: ENABLED')
            self.get_logger().info(f'  Acceleration time: {self.acceleration_time} s')
            self.get_logger().info(f'  Smoothness: {self.smoothness} (0-1)')
            if self.scurve_vx:
                state = self.scurve_vx.get_state()
                self.get_logger().info(f'  Auto-calculated max acceleration: {state["max_acceleration"]:.2f} m/s²')
                self.get_logger().info(f'  Auto-calculated max jerk: {state["max_jerk"]:.2f} m/s³')
        else:
            self.get_logger().info(f'S-Curve smoothing: DISABLED (direct control)')

        self.get_logger().info('=' * 60)

        # 发布初始紧停状态（false）
        init_estop = Bool()
        init_estop.data = False
        self.estop_pub.publish(init_estop)

        self.get_logger().info('Controls:')
        self.get_logger().info('  Left Joystick Y  → Forward/Backward (linear speed)')
        self.get_logger().info('  Left Joystick X  → Left/Right strafe (linear speed)')
        self.get_logger().info('  Right Joystick X → Rotate (angular speed)')
        self.get_logger().info('  Button X (Left)  → Lift DOWN (handled by ld2_can_lift_node)')
        self.get_logger().info('  Button Y (Left)  → Lift UP (handled by ld2_can_lift_node)')
        self.get_logger().info('=' * 60)

    # ==================== 摇杆回调函数 ====================

    def left_joystick_x_callback(self, msg: Float32):
        """左手摇杆X轴 (左右平移)"""
        self.left_joystick_x = msg.data
        self.last_vr_data_time = self.get_clock().now()

    def left_joystick_y_callback(self, msg: Float32):
        """左手摇杆Y轴 (前后移动)"""
        self.left_joystick_y = msg.data
        self.last_vr_data_time = self.get_clock().now()

    def right_joystick_x_callback(self, msg: Float32):
        """右手摇杆X轴 (旋转)"""
        self.right_joystick_x = msg.data
        self.last_vr_data_time = self.get_clock().now()

    def right_joystick_y_callback(self, msg: Float32):
        """右手摇杆Y轴 (备用,当前未使用)"""
        self.right_joystick_y = msg.data
        self.last_vr_data_time = self.get_clock().now()

    # ==================== 速度配置回调函数 ====================

    def linear_speed_callback(self, msg: Float32):
        speed = self._clamp_speed(msg.data)
        if abs(speed - self.current_max_linear_speed) > 1e-6:
            self.current_max_linear_speed = speed
            self.get_logger().info(f'Updated chassis linear speed limit: {self.current_max_linear_speed:.2f} m/s')

            # 同步更新S型控制器的速度限制
            if self.enable_scurve and self.scurve_vx is not None:
                self.scurve_vx.update_params(max_velocity=speed)
                self.scurve_vy.update_params(max_velocity=speed)

    def angular_speed_callback(self, msg: Float32):
        speed = self._clamp_speed(msg.data)
        if abs(speed - self.current_max_angular_speed) > 1e-6:
            self.current_max_angular_speed = speed
            self.get_logger().info(f'Updated chassis angular speed limit: {self.current_max_angular_speed:.2f} rad/s')

            # 同步更新S型控制器的速度限制
            if self.enable_scurve and self.scurve_wz is not None:
                self.scurve_wz.update_params(max_velocity=speed)

    def estop_toggle_callback(self, msg: Bool):
        """外部紧停输入（右摇杆按下）- 全局紧停"""
        if msg.data and not self.last_estop_toggle:
            self.estop_active = not self.estop_active
            self.enabled = not self.estop_active
            self.stop_chassis()

            # 发布全局紧停状态
            estop_msg = Bool()
            estop_msg.data = self.estop_active
            self.estop_pub.publish(estop_msg)

            if self.estop_active:
                self.get_logger().warn('GLOBAL E-STOP engaged - all subsystems disabled')
            else:
                self.get_logger().info(
                    f'GLOBAL E-STOP released - chassis speed limits '
                    f'({self.current_max_linear_speed:.1f} m/s, {self.current_max_angular_speed:.1f} rad/s)'
                )
        self.last_estop_toggle = msg.data

    # ==================== 数据处理函数 ====================

    def apply_deadzone(self, value: float) -> float:
        """
        应用死区处理

        参数:
            value: 摇杆原始值 (-1 到 1)

        返回:
            处理后的值
        """
        if abs(value) < self.joystick_deadzone:
            return 0.0

        # 重新映射到 0-1 范围 (去除死区后)
        sign = 1.0 if value > 0 else -1.0
        magnitude = abs(value)
        scaled = (magnitude - self.joystick_deadzone) / (1.0 - self.joystick_deadzone)

        return sign * scaled

    @staticmethod
    def _clamp_speed(value: float) -> float:
        return max(0.0, min(2.0, float(value)))

    def apply_expo(self, value: float, expo: float) -> float:
        """应用指数曲线；expo 越大，小幅输入越细。"""
        if value == 0.0 or expo == 1.0:
            return value

        sign = 1.0 if value > 0 else -1.0
        magnitude = abs(value)
        curved = math.pow(magnitude, expo)
        return sign * curved

    def check_vr_timeout(self):
        """检查VR数据超时"""
        elapsed = (self.get_clock().now() - self.last_vr_data_time).nanoseconds / 1e9

        if elapsed > self.vr_timeout and self.enabled:
            # 超时自动停止
            self.stop_chassis()
            # 节流警告: 每5秒最多显示一次
            time_since_last_warning = (self.get_clock().now() - self.last_timeout_warning).nanoseconds / 1e9
            if time_since_last_warning > 5.0:
                self.get_logger().warning('⚠ VR数据超时，已自动停止底盘')
                self.last_timeout_warning = self.get_clock().now()

    def stop_chassis(self):
        """发送停止命令"""
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

    # ==================== 主控制逻辑 ====================

    def publish_cmd_vel(self):
        """定期发布速度命令"""
        cmd_msg = Twist()

        if not self.enabled:
            # 禁用时不发布，让其他节点（如腰部控制）可以控制底盘
            return

        # 应用死区处理
        left_x = self.apply_expo(self.apply_deadzone(self.left_joystick_x), self.linear_expo)
        left_y = self.apply_expo(self.apply_deadzone(self.left_joystick_y), self.linear_expo)
        right_x = self.apply_expo(self.apply_deadzone(self.right_joystick_x), self.angular_expo)

        # 计算目标速度
        target_vx = left_y * self.current_max_linear_speed
        target_vy = -left_x * self.current_max_linear_speed
        target_wz = -right_x * self.current_max_angular_speed

        # 应用S型加减速平滑
        if self.enable_scurve and self.scurve_vx is not None:
            smooth_vx = self.scurve_vx.update(target_vx)
            smooth_vy = self.scurve_vy.update(target_vy)
            smooth_wz = self.scurve_wz.update(target_wz)
        else:
            smooth_vx = target_vx
            smooth_vy = target_vy
            smooth_wz = target_wz

        # 设置速度命令
        cmd_msg.linear.x = smooth_vx
        cmd_msg.linear.y = smooth_vy
        cmd_msg.angular.z = smooth_wz

        # 发布命令
        self.cmd_vel_pub.publish(cmd_msg)

        # 定期打印状态 (仅当有显著移动时)
        if abs(left_x) > 0.05 or abs(left_y) > 0.05 or abs(right_x) > 0.05:
            self.get_logger().info(
                f'VR Control: vx={cmd_msg.linear.x:.2f}, vy={cmd_msg.linear.y:.2f}, '
                f'vz={cmd_msg.angular.z:.2f}',
                throttle_duration_sec=1.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = VRTeleopNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.get_logger().info('Keyboard interrupt - shutting down')
    finally:
        # 停止底盘
        try:
            if rclpy.ok():
                node.stop_chassis()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
