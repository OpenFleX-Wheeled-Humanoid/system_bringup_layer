#!/usr/bin/env python3
"""Swerve drive teleop keyboard node.

Based on teleop_twist_keyboard layout with added single-key strafe (a/d).

Key bindings:
    u  i  o        左前 / 前进 / 右前
    j  k  l        左转 / 停止 / 右转
    m  ,  .        左后 / 后退 / 右后

    a              纯左移 (strafe left)
    d              纯右移 (strafe right)

    U  J  L  O     Shift variants (holonomic diagonals, same as teleop_twist_keyboard)
    M  <  >        Shift variants

    q/z            increase / decrease overall speed
    w/x            increase / decrease linear speed only
    e/c            increase / decrease angular speed only

    Ctrl-C / q     quit (q also increases speed, so use Ctrl-C to exit)
"""

import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

# (linear.x, linear.y, angular.z)
MOVE_BINDINGS = {
    # Original teleop_twist_keyboard layout
    'i': (1, 0, 0),
    'o': (1, 0, -1),
    'j': (0, 0, 1),
    'k': (0, 0, 0),
    'l': (0, 0, -1),
    'u': (1, 0, 1),
    ',': (-1, 0, 0),
    '.': (-1, 0, 1),
    'm': (-1, 0, -1),
    # Shift variants — holonomic (pure lateral + diagonal)
    'O': (1, -1, 0),
    'I': (1, 0, 0),
    'J': (0, 1, 0),
    'L': (0, -1, 0),
    'U': (1, 1, 0),
    '<': (-1, 0, 0),
    '>': (-1, -1, 0),
    'M': (-1, 1, 0),
    # New single-key strafe
    'a': (0, 1, 0),
    'd': (0, -1, 0),
}

# (linear speed multiplier, angular speed multiplier)
SPEED_BINDINGS = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1.0),
    'x': (0.9, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, 0.9),
}

BANNER = """
Swerve Teleop Keyboard
----------------------
Moving:
   u    i    o
   j    k    l
   m    ,    .

Strafe:
   a  左移    d  右移

Shift keys for holonomic diagonals (U/I/O/J/L/M/</>)

q/z : increase/decrease all speeds
w/x : increase/decrease linear speed
e/c : increase/decrease angular speed

CTRL-C to quit
"""

PUBLISH_HZ = 20.0
COMMAND_HOLD_SEC = 2.0


def get_key(settings, timeout):
    tty.setraw(sys.stdin.fileno())
    try:
        readable, _, _ = select.select([sys.stdin], [], [], timeout)
        key = sys.stdin.read(1) if readable else ''
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def save_terminal_settings():
    return termios.tcgetattr(sys.stdin)


def restore_terminal_settings(settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main():
    settings = save_terminal_settings()

    rclpy.init()
    node = Node('swerve_teleop')
    pub = node.create_publisher(Twist, 'cmd_vel', qos_profile_system_default)

    speed = 0.25
    turn = 0.5
    x = 0
    y = 0
    th = 0
    last_motion_input_time = 0.0
    loop_period = 1.0 / PUBLISH_HZ

    print(BANNER)
    print(f'Speed: {speed:.2f}  Turn: {turn:.2f}')

    try:
        while True:
            key = get_key(settings, loop_period)
            now = time.monotonic()

            if key in MOVE_BINDINGS:
                x, y, th = MOVE_BINDINGS[key]
                last_motion_input_time = now
            elif key in SPEED_BINDINGS:
                speed *= SPEED_BINDINGS[key][0]
                turn *= SPEED_BINDINGS[key][1]
                print(f'Speed: {speed:.2f}  Turn: {turn:.2f}')
            elif key == '':  # Ctrl-C
                break
            elif key:
                # Unknown key → stop
                x, y, th = 0, 0, 0
                last_motion_input_time = 0.0

            # Keep republishing the latest motion command briefly so the swerve
            # controller can finish steering alignment before cmd_vel times out.
            if last_motion_input_time > 0.0 and (now - last_motion_input_time) > COMMAND_HOLD_SEC:
                x, y, th = 0, 0, 0
                last_motion_input_time = 0.0

            twist = Twist()
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.angular.z = th * turn
            pub.publish(twist)

    except Exception as e:
        print(e)
    finally:
        # Send stop on exit
        pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
        restore_terminal_settings(settings)


if __name__ == '__main__':
    main()
