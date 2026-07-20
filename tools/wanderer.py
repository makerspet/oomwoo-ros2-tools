#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Minimal reactive wanderer for mapping runs: drive forward, turn away when
blocked (tutorial-style), alternating turn direction to cover the room."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy)
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Wanderer(Node):
    def __init__(self):
        super().__init__('wanderer')
        qos = QoSProfile(depth=5, history=QoSHistoryPolicy.KEEP_LAST,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.clear = True
        self.turn_dir = 1.0
        self.turning = 0        # ticks left in a committed turn
        self.create_timer(0.1, self.tick)

    def on_scan(self, msg):
        span = max(1, int(math.radians(30) / msg.angle_increment))
        win = list(msg.ranges[:span]) + list(msg.ranges[-span:])
        ok = [r for r in win if msg.range_min < r < msg.range_max]
        self.clear = (min(ok) > 0.45) if ok else True

    def tick(self):
        t = Twist()
        if self.turning > 0:
            self.turning -= 1
            t.angular.z = 1.0 * self.turn_dir
        elif self.clear:
            t.linear.x = 0.22
        else:
            self.turn_dir = -self.turn_dir      # alternate to spread coverage
            self.turning = 12                    # commit ~1.2 s of turning
            t.angular.z = 1.0 * self.turn_dir
        self.pub.publish(t)


def main():
    rclpy.init()
    n = Wanderer()
    n.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
