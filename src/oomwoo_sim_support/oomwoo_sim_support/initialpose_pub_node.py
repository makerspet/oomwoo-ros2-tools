#!/usr/bin/env python3
# Copyright 2026 Jayadev Rana
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Seed AMCL with a known initial pose, robustly, without a launch timing race.

AMCL in the oomwoo navigation config has ``set_initial_pose: false``, so on a
fresh bringup it holds no pose and Nav2 cannot plan. This node republishes a
known ``/initialpose`` a few times (until AMCL is active and has latched it),
then exits. Used by the coverage bringup where the robot's start pose is known.
"""

import math

from geometry_msgs.msg import PoseWithCovarianceStamped

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class InitialPosePub(Node):
    def __init__(self) -> None:
        super().__init__('initialpose_pub')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('count', 6)
        self.declare_parameter('period', 1.0)
        self.x = self.get_parameter('x').value
        self.y = self.get_parameter('y').value
        self.yaw = self.get_parameter('yaw').value
        self.remaining = int(self.get_parameter('count').value)

        self.pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.timer = self.create_timer(
            float(self.get_parameter('period').value), self._tick)

    def _tick(self) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(self.x)
        msg.pose.pose.position.y = float(self.y)
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        # modest covariance so AMCL trusts it but still refines
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068
        self.pub.publish(msg)
        self.remaining -= 1
        self.get_logger().info(
            f'published /initialpose ({self.x:.2f},{self.y:.2f},{self.yaw:.2f}) '
            f'[{self.remaining} left]')
        if self.remaining <= 0:
            self.timer.cancel()
            raise SystemExit


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InitialPosePub()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
