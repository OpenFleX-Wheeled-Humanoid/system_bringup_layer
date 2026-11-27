#!/usr/bin/env python3
"""
腰部增量位置控制底盘+升降台节点 (odom 闭环版)

工作原理：
- 左/右食指扳机任意一个 >= 阈值时激活控制
- 每帧计算腰部位移增量 (当前帧 - 上一帧)，乘以 gain 累加到目标位置
- 订阅 /odom 获取底盘当前位置，用 PID 控制器跟踪目标
- 订阅 /joint_states 获取升降台关节位置（兼容 lift_joint / plate_joint），发布位置目标
- 腰部不动 → 目标不变 → 底盘到位后停止
- 腰部移动 → 目标持续更新 → 底盘跟随
- 左/右食指扳机都松开 → 底盘停止，升降台保持当前位置

坐标映射 (腰部增量先映射到机器人坐标系，再旋转到世界坐标系累加):
  waist +dZ (前) → robot +x → 旋转到世界坐标系
  waist +dX (右) → robot -y → 旋转到世界坐标系
  waist +dY (上) → +target_lift (直接累加)
  waist +dyaw    → +target_theta (直接累加)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64MultiArray
import math
import time


class SimplePID:
    """简单 PID 控制器，带 anti-windup"""

    def __init__(self, kp, ki, kd, max_integral=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_integral = max_integral
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        # anti-windup
        self.integral = max(-self.max_integral, min(self.max_integral, self.integral))
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


class WaistChassisControlNode(Node):
    def __init__(self):
        super().__init__('waist_chassis_control')

        # === 参数声明 ===
        # 腰部→目标 增益
        self.declare_parameter('position_gain', 1.0)
        self.declare_parameter('lift_gain', 1.0)
        self.declare_parameter('angular_gain', 1.0)
        # 底盘 PID
        self.declare_parameter('pid_kp', 1.5)
        self.declare_parameter('pid_ki', 0.0)
        self.declare_parameter('pid_kd', 0.1)
        # 角度 PID
        self.declare_parameter('ang_pid_kp', 2.0)
        self.declare_parameter('ang_pid_ki', 0.0)
        self.declare_parameter('ang_pid_kd', 0.1)
        # 升降台旧速度模式 PID 参数保留，位置模式下不再用于发布升降台速度
        self.declare_parameter('lift_pid_kp', 2.0)
        self.declare_parameter('lift_pid_ki', 0.0)
        self.declare_parameter('lift_pid_kd', 0.05)
        # 速度上限
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 1.5)
        self.declare_parameter('lift_speed', 0.2)
        # 死区与频率
        self.declare_parameter('position_deadzone', 0.01)
        self.declare_parameter('angular_deadzone', 0.02)
        self.declare_parameter('lift_deadzone', 0.01)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('feedback_timeout', 0.5)
        # 话题
        self.declare_parameter('trigger_threshold', 0.5)
        self.declare_parameter('waist_topic', '/pico_tracker/waist/pose')
        self.declare_parameter('left_trigger_topic', '/pico_left_controller/trigger')
        self.declare_parameter('right_trigger_topic', '/pico_right_controller/trigger')
        # 兼容旧参数：可选额外触发源（例如 /pico_right_controller/grip）
        self.declare_parameter('trigger_topic', '')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('position_topic', '/lift_position_controller/commands')
        self.declare_parameter('lift_min_position', -0.424)
        self.declare_parameter('lift_max_position', 0.650)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('lift_joint_candidates', ['lift_joint', 'plate_joint'])

        # === 获取参数 ===
        self.position_gain = self.get_parameter('position_gain').value
        self.lift_gain = self.get_parameter('lift_gain').value
        self.angular_gain = self.get_parameter('angular_gain').value

        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.lift_speed = self.get_parameter('lift_speed').value

        self.position_deadzone = self.get_parameter('position_deadzone').value
        self.angular_deadzone = self.get_parameter('angular_deadzone').value
        self.lift_deadzone = self.get_parameter('lift_deadzone').value
        self.control_rate = self.get_parameter('control_rate').value
        self.feedback_timeout = self.get_parameter('feedback_timeout').value
        self.trigger_threshold = self.get_parameter('trigger_threshold').value
        self.lift_min_position = self.get_parameter('lift_min_position').value
        self.lift_max_position = self.get_parameter('lift_max_position').value
        lift_joint_candidates = self.get_parameter('lift_joint_candidates').get_parameter_value().string_array_value
        self.lift_joint_candidates = list(lift_joint_candidates) if lift_joint_candidates else ['lift_joint', 'plate_joint']

        # === PID 控制器 ===
        pid_kp = self.get_parameter('pid_kp').value
        pid_ki = self.get_parameter('pid_ki').value
        pid_kd = self.get_parameter('pid_kd').value
        self.pid_x = SimplePID(pid_kp, pid_ki, pid_kd)
        self.pid_y = SimplePID(pid_kp, pid_ki, pid_kd)

        ang_kp = self.get_parameter('ang_pid_kp').value
        ang_ki = self.get_parameter('ang_pid_ki').value
        ang_kd = self.get_parameter('ang_pid_kd').value
        self.pid_theta = SimplePID(ang_kp, ang_ki, ang_kd)

        lift_kp = self.get_parameter('lift_pid_kp').value
        lift_ki = self.get_parameter('lift_pid_ki').value
        lift_kd = self.get_parameter('lift_pid_kd').value
        self.pid_lift = SimplePID(lift_kp, lift_ki, lift_kd)

        # === 状态变量 ===
        self.control_active = False
        self.waist_data_received = False
        self.left_trigger_value = 0.0
        self.right_trigger_value = 0.0
        self.legacy_trigger_value = 0.0

        # 腰部上一帧数据 (用于计算增量)
        self.prev_waist_x = 0.0
        self.prev_waist_y = 0.0
        self.prev_waist_z = 0.0
        self.prev_waist_yaw = 0.0

        # 目标位置 (odom 世界坐标系)
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_theta = 0.0
        self.target_lift = 0.0

        # odom 当前位置
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.odom_received = False
        self.odom_last_time = 0.0  # 用于检测 odom 数据过期

        # 升降台当前位置
        self.lift_position = 0.0
        self.lift_received = False
        self.lift_last_time = 0.0

        # grip 按下时的 odom 初始位置 (目标基准)
        self.odom_origin_x = 0.0
        self.odom_origin_y = 0.0
        self.odom_origin_theta = 0.0
        self.lift_origin = 0.0

        # === 发布者 ===
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        position_topic = self.get_parameter('position_topic').value
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.lift_pub = self.create_publisher(Float64MultiArray, position_topic, 10)

        # === 订阅者 ===
        waist_topic = self.get_parameter('waist_topic').value
        left_trigger_topic = self.get_parameter('left_trigger_topic').value
        right_trigger_topic = self.get_parameter('right_trigger_topic').value
        legacy_trigger_topic = self.get_parameter('trigger_topic').value
        odom_topic = self.get_parameter('odom_topic').value

        self.waist_sub = self.create_subscription(
            PoseStamped, waist_topic, self.waist_callback, 10)
        self.left_trigger_sub = self.create_subscription(
            Float32, left_trigger_topic, self.left_trigger_callback, 10)
        self.right_trigger_sub = self.create_subscription(
            Float32, right_trigger_topic, self.right_trigger_callback, 10)
        self.legacy_trigger_sub = None
        if legacy_trigger_topic:
            self.legacy_trigger_sub = self.create_subscription(
                Float32, legacy_trigger_topic, self.trigger_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10)
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        # === 控制定时器 ===
        dt = 1.0 / self.control_rate
        self.control_timer = self.create_timer(dt, self.control_loop)

        self._log_counter = 0

        self.get_logger().info('=' * 60)
        self.get_logger().info('Waist Incremental Position Control (odom closed-loop)')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'position_gain={self.position_gain}, lift_gain={self.lift_gain}, angular_gain={self.angular_gain}')
        self.get_logger().info(f'PID xy: kp={pid_kp} ki={pid_ki} kd={pid_kd}')
        self.get_logger().info(f'PID ang: kp={ang_kp} ki={ang_ki} kd={ang_kd}')
        self.get_logger().info(f'PID lift parameters kept for compatibility: kp={lift_kp} ki={lift_ki} kd={lift_kd}')
        self.get_logger().info(f'max_linear_vel={self.max_linear_vel}, max_angular_vel={self.max_angular_vel}, lift_target_rate_limit={self.lift_speed}')
        self.get_logger().info(f'lift position limits={self.lift_min_position:.3f}~{self.lift_max_position:.3f} m')
        self.get_logger().info(f'deadzone: pos={self.position_deadzone}, ang={self.angular_deadzone}, lift={self.lift_deadzone}')
        self.get_logger().info(f'control_rate={self.control_rate} Hz')
        self.get_logger().info(f'feedback_timeout={self.feedback_timeout:.2f}s')
        self.get_logger().info(
            f'Subscribing: waist={waist_topic}, left_trigger={left_trigger_topic}, '
            f'right_trigger={right_trigger_topic}, legacy_trigger={legacy_trigger_topic or "disabled"}, odom={odom_topic}')
        self.get_logger().info(f'Publishing: cmd_vel={cmd_vel_topic}, lift_position={position_topic}')
        self.get_logger().info(f'Lift joint candidates: {self.lift_joint_candidates}')
        self.get_logger().info('=' * 60)

    # ==================== 回调 ====================

    def waist_callback(self, msg: PoseStamped):
        """处理腰部姿态数据，计算增量并累加到目标"""
        pos = msg.pose.position
        q = msg.pose.orientation

        # 提取绕 Y 轴旋转 (yaw in waist frame)
        siny = 2.0 * (q.w * q.y + q.x * q.z)
        cosy = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        current_yaw = math.atan2(siny, cosy)

        if not self.waist_data_received:
            # 第一帧，只记录，不产生增量
            self.prev_waist_x = pos.x
            self.prev_waist_y = pos.y
            self.prev_waist_z = pos.z
            self.prev_waist_yaw = current_yaw
            self.waist_data_received = True
            return

        if self.control_active:
            # 计算增量
            dx = pos.x - self.prev_waist_x
            dy = pos.y - self.prev_waist_y
            dz = pos.z - self.prev_waist_z
            dyaw = self.normalize_angle(current_yaw - self.prev_waist_yaw)

            # 腰部增量映射到机器人坐标系:
            #   waist -dZ (人前移) → robot +x (前进)
            #   waist +dX (右) → robot -y (右移)
            robot_dx = (-dz) * self.position_gain
            robot_dy = (-dx) * self.position_gain

            # 将机器人坐标系增量旋转到世界坐标系后累加到目标
            # (因为 target_x/y 是世界坐标系中相对于 odom_origin 的偏移)
            cos_th = math.cos(self.odom_theta)
            sin_th = math.sin(self.odom_theta)
            self.target_x += cos_th * robot_dx - sin_th * robot_dy
            self.target_y += sin_th * robot_dx + cos_th * robot_dy

            self.target_theta += dyaw * self.angular_gain
            self.target_theta = self.normalize_angle(self.target_theta)
            self.target_lift += dy * self.lift_gain

        # 更新上一帧
        self.prev_waist_x = pos.x
        self.prev_waist_y = pos.y
        self.prev_waist_z = pos.z
        self.prev_waist_yaw = current_yaw

    def left_trigger_callback(self, msg: Float32):
        self.left_trigger_value = msg.data
        self.update_trigger_activation()

    def right_trigger_callback(self, msg: Float32):
        self.right_trigger_value = msg.data
        self.update_trigger_activation()

    def trigger_callback(self, msg: Float32):
        """兼容旧参数 trigger_topic 的回调"""
        self.legacy_trigger_value = msg.data
        self.update_trigger_activation()

    def update_trigger_activation(self):
        """任意一个触发源超过阈值即激活"""
        new_active = (
            self.left_trigger_value >= self.trigger_threshold
            or self.right_trigger_value >= self.trigger_threshold
            or self.legacy_trigger_value >= self.trigger_threshold
        )

        was_active = self.control_active
        self.control_active = new_active

        # 按下边沿
        if self.control_active and not was_active:
            if not self.waist_data_received:
                self.get_logger().warn('Trigger pressed but no waist data yet.')
                self.control_active = False
                return
            if not self.odom_received:
                self.get_logger().warn('Trigger pressed but no odom data yet.')
                self.control_active = False
                return

            # 记录 odom 原点和升降台原点
            self.odom_origin_x = self.odom_x
            self.odom_origin_y = self.odom_y
            self.odom_origin_theta = self.odom_theta
            self.lift_origin = self.lift_position

            # 目标清零 (相对于原点的偏移)
            self.target_x = 0.0
            self.target_y = 0.0
            self.target_theta = 0.0
            self.target_lift = 0.0

            # 重置 PID
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_theta.reset()
            self.pid_lift.reset()

            self.get_logger().info(
                f'ACTIVATED - odom origin: ({self.odom_origin_x:.3f}, {self.odom_origin_y:.3f}, '
                f'{math.degrees(self.odom_origin_theta):.1f}deg) lift_origin: {self.lift_origin:.3f}')

        # 松开边沿
        if not self.control_active and was_active:
            self.cmd_vel_pub.publish(Twist())
            self.publish_lift_hold()
            self.get_logger().info('DEACTIVATED - chassis stopped, lift holding current position')

    def odom_callback(self, msg: Odometry):
        """获取底盘当前位置"""
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.odom_theta = math.atan2(siny, cosy)
        self.odom_received = True
        self.odom_last_time = time.monotonic()

    def joint_state_callback(self, msg: JointState):
        """获取升降台关节位置，兼容新 lift_joint 和旧 plate_joint"""
        for joint_name in self.lift_joint_candidates:
            if joint_name not in msg.name:
                continue
            idx = msg.name.index(joint_name)
            if idx < len(msg.position):
                self.lift_position = msg.position[idx]
                self.lift_received = True
                self.lift_last_time = time.monotonic()
            return

    def publish_lift_hold(self):
        """有升降台反馈时发布当前位置，避免位置模式下误发 0.0。"""
        if not self.lift_received:
            return
        lift_msg = Float64MultiArray()
        lift_msg.data = [self.lift_position]
        self.lift_pub.publish(lift_msg)

    # ==================== 控制循环 ====================

    def control_loop(self):
        """定时器回调：PID 控制循环"""
        if not self.control_active:
            return

        dt = 1.0 / self.control_rate
        now = time.monotonic()

        # odom 数据过期检测
        if now - self.odom_last_time > self.feedback_timeout:
            self.cmd_vel_pub.publish(Twist())
            self.publish_lift_hold()
            if self._log_counter % 20 == 0:
                self.get_logger().warn(
                    f'Odom data stale (>{self.feedback_timeout:.2f}s) - emergency stop')
            self._log_counter += 1
            return

        # --- 底盘位置误差 (世界坐标系) ---
        # 当前位置相对于 odom_origin 的偏移
        cur_dx = self.odom_x - self.odom_origin_x
        cur_dy = self.odom_y - self.odom_origin_y
        cur_dtheta = self.normalize_angle(self.odom_theta - self.odom_origin_theta)

        # 世界坐标系下的误差
        err_x_world = self.target_x - cur_dx
        err_y_world = self.target_y - cur_dy
        err_theta = self.normalize_angle(self.target_theta - cur_dtheta)

        # 将世界坐标系误差转换到机器人坐标系 (因为 cmd_vel 是机器人坐标系)
        cos_th = math.cos(self.odom_theta)
        sin_th = math.sin(self.odom_theta)
        err_x_robot = cos_th * err_x_world + sin_th * err_y_world
        err_y_robot = -sin_th * err_x_world + cos_th * err_y_world

        # 死区
        if abs(err_x_robot) < self.position_deadzone:
            err_x_robot = 0.0
        if abs(err_y_robot) < self.position_deadzone:
            err_y_robot = 0.0
        if abs(err_theta) < self.angular_deadzone:
            err_theta = 0.0

        # PID 计算
        vx = self.pid_x.compute(err_x_robot, dt)
        vy = self.pid_y.compute(err_y_robot, dt)
        vtheta = self.pid_theta.compute(err_theta, dt)

        # clamp
        vx = max(-self.max_linear_vel, min(self.max_linear_vel, vx))
        vy = max(-self.max_linear_vel, min(self.max_linear_vel, vy))
        vtheta = max(-self.max_angular_vel, min(self.max_angular_vel, vtheta))

        # 发布底盘速度
        cmd_msg = Twist()
        cmd_msg.linear.x = vx
        cmd_msg.linear.y = vy
        cmd_msg.angular.z = vtheta
        self.cmd_vel_pub.publish(cmd_msg)

        # --- 升降台位置目标 ---
        cur_lift = 0.0
        lift_feedback_stale = (
            self.lift_received and now - self.lift_last_time > self.feedback_timeout
        )
        if lift_feedback_stale:
            self.pid_lift.reset()
            lift_target = self.lift_position
            if self._log_counter % 20 == 0:
                self.get_logger().warn(
                    f'Lift feedback stale (>{self.feedback_timeout:.2f}s) - holding lift position')
        elif self.lift_received:
            cur_lift = self.lift_position - self.lift_origin
            err_lift = self.target_lift - cur_lift
            if abs(err_lift) < self.lift_deadzone:
                self.target_lift = cur_lift
                err_lift = 0.0
            max_delta = max(0.0, self.lift_speed * dt)
            if err_lift > max_delta:
                self.target_lift = cur_lift + max_delta
            elif err_lift < -max_delta:
                self.target_lift = cur_lift - max_delta
            lift_target = self.lift_origin + self.target_lift
            lift_target = max(self.lift_min_position, min(self.lift_max_position, lift_target))
        else:
            return

        lift_msg = Float64MultiArray()
        lift_msg.data = [lift_target]
        self.lift_pub.publish(lift_msg)

        # 诊断日志
        self._log_counter += 1
        if self._log_counter % 40 == 0:
            self.get_logger().info(
                f'TGT({self.target_x:.3f},{self.target_y:.3f},{math.degrees(self.target_theta):.1f}deg,lift={self.target_lift:.3f}) '
                f'CUR({cur_dx:.3f},{cur_dy:.3f},{math.degrees(cur_dtheta):.1f}deg,lift={cur_lift:.3f}) '
                f'CMD({vx:.2f},{vy:.2f},{vtheta:.2f},lift_pos={lift_target:.3f})')

    # ==================== 工具函数 ====================

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = WaistChassisControlNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # 安全停止：仅在 context 仍有效时发布零速
        try:
            node.cmd_vel_pub.publish(Twist())
            node.publish_lift_hold()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
