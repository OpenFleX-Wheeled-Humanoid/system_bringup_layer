#!/usr/bin/env python3
"""
VR lift control node.

Maps Pico left-controller buttons directly to the same manual lift controller
path used by the RViz panel:
- Y pressed: jog up
- X pressed: jog down
- release / conflict / timeout / estop: jog stop
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64


class VRLiftControlNode(Node):
    """VR X/Y button to lift_manual_position_controller jog topic."""

    def __init__(self):
        super().__init__("vr_lift_control")

        self.declare_parameter("button_x_topic", "/pico_left_controller/button_x")
        self.declare_parameter("button_y_topic", "/pico_left_controller/button_y")
        self.declare_parameter("jog_command_topic", "/lift_manual_position_controller/jog_command")
        self.declare_parameter("lift_speed", 0.05)
        self.declare_parameter("profile_speed", 0.05)
        self.declare_parameter("vr_timeout", 0.5)
        # Kept for launch-file compatibility with the previous topic-based node.
        self.declare_parameter("position_topic", "/lift_position_controller/commands")
        self.declare_parameter("profile_speed_topic", "/lift_state_controller/profile_speed_cmd")

        self.button_x_topic = self.get_parameter("button_x_topic").value
        self.button_y_topic = self.get_parameter("button_y_topic").value
        self.jog_command_topic = self.get_parameter("jog_command_topic").value
        self.lift_speed = float(self.get_parameter("lift_speed").value)
        self.profile_speed = float(self.get_parameter("profile_speed").value)
        self.vr_timeout = float(self.get_parameter("vr_timeout").value)

        self.button_x_pressed = False
        self.button_y_pressed = False
        self.estop_active = False
        self.active_direction = 0
        self.last_vr_data_time = self.get_clock().now()
        self.last_timeout_warning = self.get_clock().now()

        self.jog_pub = self.create_publisher(Float64, self.jog_command_topic, 10)

        self.create_subscription(Bool, self.button_x_topic, self.button_x_callback, 10)
        self.create_subscription(Bool, self.button_y_topic, self.button_y_callback, 10)

        estop_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(Bool, "/vr_estop_active", self.estop_callback, estop_qos)

        self.create_timer(0.05, self.sync_motion_state)
        self.create_timer(0.1, self.check_vr_timeout)

        self.get_logger().info("VR Lift Control Node initialized")
        self.get_logger().info(f"  X button: DOWN via {self.jog_command_topic}")
        self.get_logger().info(f"  Y button: UP via {self.jog_command_topic}")
        self.get_logger().info(f"  Jog speed: {self.command_speed():.3f} m/s")
        self.get_logger().info(f"  VR timeout: {self.vr_timeout:.2f}s")

    def command_speed(self) -> float:
        # profile_speed is retained for compatibility; lift_speed is the jog speed.
        speed = self.lift_speed if self.lift_speed > 0.0 else self.profile_speed
        return max(0.001, abs(float(speed)))

    def button_x_callback(self, msg: Bool):
        self.last_vr_data_time = self.get_clock().now()
        pressed = bool(msg.data)
        if pressed != self.button_x_pressed:
            self.button_x_pressed = pressed
            self.get_logger().info("X Button PRESSED - lift down" if pressed else "X Button RELEASED")
            self.sync_motion_state()

    def button_y_callback(self, msg: Bool):
        self.last_vr_data_time = self.get_clock().now()
        pressed = bool(msg.data)
        if pressed != self.button_y_pressed:
            self.button_y_pressed = pressed
            self.get_logger().info("Y Button PRESSED - lift up" if pressed else "Y Button RELEASED")
            self.sync_motion_state()

    def estop_callback(self, msg: Bool):
        was_active = self.estop_active
        self.estop_active = bool(msg.data)
        if self.estop_active and not was_active:
            self.get_logger().warn("E-STOP: stopping lift jog")
            self.sync_motion_state()
        elif not self.estop_active and was_active:
            self.get_logger().info("E-STOP released: lift jog input enabled")
            self.sync_motion_state()

    def check_vr_timeout(self):
        elapsed = (self.get_clock().now() - self.last_vr_data_time).nanoseconds / 1e9
        if elapsed <= self.vr_timeout:
            return

        if self.button_x_pressed or self.button_y_pressed:
            self.button_x_pressed = False
            self.button_y_pressed = False
            self.sync_motion_state()

        since_warning = (self.get_clock().now() - self.last_timeout_warning).nanoseconds / 1e9
        if since_warning > 5.0:
            self.get_logger().warning("VR button data timeout; lift jog stopped")
            self.last_timeout_warning = self.get_clock().now()

    def desired_direction(self) -> int:
        if self.estop_active:
            return 0
        if self.button_y_pressed and not self.button_x_pressed:
            return 1
        if self.button_x_pressed and not self.button_y_pressed:
            return -1
        return 0

    def sync_motion_state(self):
        desired = self.desired_direction()
        if desired == self.active_direction:
            return

        if self.active_direction != 0:
            self.publish_jog_command(0.0)
            self.active_direction = 0

        if desired != 0:
            self.publish_jog_command(float(desired) * self.command_speed())
            self.active_direction = desired

    def publish_jog_command(self, speed_mps: float):
        msg = Float64()
        msg.data = float(speed_mps)
        self.jog_pub.publish(msg)
        if abs(speed_mps) < 1e-9:
            self.get_logger().info("Lift jog stop")
        else:
            self.get_logger().info(
                f"Lift jog start: {'up' if speed_mps > 0.0 else 'down'} speed={abs(speed_mps):.3f} m/s"
            )


def main(args=None):
    rclpy.init(args=args)
    node = VRLiftControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt - shutting down")
    finally:
        if node.active_direction != 0:
            node.publish_jog_command(0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
