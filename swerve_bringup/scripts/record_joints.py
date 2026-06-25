#!/usr/bin/env python3
"""Record /joint_states to CSV for plotting."""

import csv
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Recorder(Node):
    def __init__(self, duration, output):
        super().__init__('joint_recorder')
        self.data = []
        self.names = None
        self.t0 = None
        self.duration = duration
        self.output = output
        self.sub = self.create_subscription(
            JointState, '/joint_states', self.cb, 10)
        self.get_logger().info(
            f'Recording /joint_states for {duration}s → {output}')

    def cb(self, msg):
        now = self.get_clock().now()
        if self.t0 is None:
            self.t0 = now
            self.names = list(msg.name)
        elapsed = (now - self.t0).nanoseconds * 1e-9
        if elapsed > self.duration:
            self.save()
            raise SystemExit
        row = [elapsed]
        row.extend(msg.position)
        row.extend(msg.velocity)
        self.data.append(row)

    def save(self):
        if not self.data:
            self.get_logger().warn('No data recorded')
            return
        header = ['time']
        header += [f'{n}_pos' for n in self.names]
        header += [f'{n}_vel' for n in self.names]
        with open(self.output, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(header)
            w.writerows(self.data)
        self.get_logger().info(
            f'Saved {len(self.data)} samples to {self.output}')


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    output = sys.argv[2] if len(sys.argv) > 2 else '/tmp/joint_states.csv'
    rclpy.init()
    node = Recorder(duration, output)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
