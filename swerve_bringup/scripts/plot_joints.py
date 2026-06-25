#!/usr/bin/env python3
"""Plot joint_states CSV: steering position vs wheel velocity."""

import csv
import sys


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/joint_states.csv'

    import matplotlib.pyplot as plt

    with open(path) as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    if not rows:
        print('No data')
        return

    t = [float(r['time']) for r in rows]

    # Auto-detect steering and wheel joints
    steer_pos_cols = [c for c in rows[0] if 'steering' in c and '_pos' in c]
    wheel_vel_cols = [c for c in rows[0] if 'wheel' in c and '_vel' in c]

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 6))

    for col in steer_pos_cols:
        label = col.replace('_joint_pos', '')
        ax1.plot(t, [float(r[col]) for r in rows], label=label)
    ax1.set_ylabel('Steering Angle (rad)')
    ax1.legend()
    ax1.grid(True)

    for col in wheel_vel_cols:
        label = col.replace('_joint_vel', '')
        ax2.plot(t, [float(r[col]) for r in rows], label=label)
    ax2.set_ylabel('Wheel Velocity (rad/s)')
    ax2.set_xlabel('Time (s)')
    ax2.legend()
    ax2.grid(True)

    fig.suptitle('Steering Angle vs Wheel Velocity')
    plt.tight_layout()
    plt.savefig('/tmp/joint_plot.png', dpi=150)
    print('Saved to /tmp/joint_plot.png')
    plt.show()


if __name__ == '__main__':
    main()
