#!/usr/bin/env python3
"""
VR摇杆数据监控工具

实时显示VR手柄摇杆和按键状态,方便调试
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import sys
import termios
import tty


class VRMonitor(Node):
    def __init__(self):
        super().__init__('vr_monitor')

        # 数据缓存
        self.left_joy_x = 0.0
        self.left_joy_y = 0.0
        self.right_joy_x = 0.0
        self.right_joy_y = 0.0
        self.button_a = False
        self.button_b = False
        self.button_x = False
        self.button_y = False
        self.right_joy_press = False

        # 订阅所有VR话题
        self.create_subscription(Float32, '/pico_left_controller/joystick_x',
                                lambda msg: setattr(self, 'left_joy_x', msg.data), 10)
        self.create_subscription(Float32, '/pico_left_controller/joystick_y',
                                lambda msg: setattr(self, 'left_joy_y', msg.data), 10)
        self.create_subscription(Float32, '/pico_right_controller/joystick_x',
                                lambda msg: setattr(self, 'right_joy_x', msg.data), 10)
        self.create_subscription(Float32, '/pico_right_controller/joystick_y',
                                lambda msg: setattr(self, 'right_joy_y', msg.data), 10)
        self.create_subscription(Bool, '/pico_right_controller/button_a',
                                lambda msg: setattr(self, 'button_a', msg.data), 10)
        self.create_subscription(Bool, '/pico_right_controller/button_b',
                                lambda msg: setattr(self, 'button_b', msg.data), 10)
        self.create_subscription(Bool, '/pico_left_controller/button_x',
                                lambda msg: setattr(self, 'button_x', msg.data), 10)
        self.create_subscription(Bool, '/pico_left_controller/button_y',
                                lambda msg: setattr(self, 'button_y', msg.data), 10)
        self.create_subscription(Bool, '/pico_right_controller/joystick_click',
                                lambda msg: setattr(self, 'right_joy_press', msg.data), 10)

        # 定时打印
        self.create_timer(0.1, self.print_status)

        self.get_logger().info('VR Monitor started - Ctrl+C to exit')
        self.get_logger().info('=' * 70)

    def print_status(self):
        # 清屏并移动光标到顶部
        print('\033[2J\033[H', end='')

        print('=' * 70)
        print('VR Hand Controller Monitor'.center(70))
        print('=' * 70)
        print()

        # 左手摇杆
        print('LEFT HAND JOYSTICK:')
        print(f'  X-axis (strafe): {self.left_joy_x:+.3f}  {"█" * int(abs(self.left_joy_x) * 20)}')
        print(f'  Y-axis (move):   {self.left_joy_y:+.3f}  {"█" * int(abs(self.left_joy_y) * 20)}')
        print()

        # 右手摇杆
        print('RIGHT HAND JOYSTICK:')
        print(f'  X-axis (rotate): {self.right_joy_x:+.3f}  {"█" * int(abs(self.right_joy_x) * 20)}')
        print(f'  Y-axis (unused): {self.right_joy_y:+.3f}  {"█" * int(abs(self.right_joy_y) * 20)}')
        print()

        # 按键状态
        print('BUTTONS:')
        print(f'  A (Right - Enable):  {"[PRESSED]" if self.button_a else "[      ]"}')
        print(f'  B (Right - Head):    {"[PRESSED]" if self.button_b else "[      ]"}')
        print(f'  X (Left  - LiftDn):  {"[PRESSED]" if self.button_x else "[      ]"}')
        print(f'  Y (Left  - LiftUp):  {"[PRESSED]" if self.button_y else "[      ]"}')
        print(f'  R-Stick Press:       {"[PRESSED]" if self.right_joy_press else "[      ]"}')
        print()

        # 映射提示
        print('=' * 70)
        print('CONTROL MAPPING (方案2):')
        print('  Left Joy Y  → Forward/Backward  (前进/后退)')
        print('  Left Joy X  → Left/Right Strafe (左右平移)')
        print('  Right Joy X → Rotate            (旋转)')
        print('=' * 70)
        print('Press Ctrl+C to exit')


def main():
    rclpy.init()
    node = VRMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nExiting...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
