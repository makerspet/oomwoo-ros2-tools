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
Localization gate: hold navigation until AMCL has actually localized.

AMCL self-seeds (``set_initial_pose: True``), but the Nav2 navigation stack must
not activate until AMCL has published its pose / the ``map->odom`` transform.
Activate the global costmap first and it comes up with no ``map->odom``,
``bt_navigator`` never reaches ACTIVE, and every goal is rejected with
``Action server is inactive. Rejecting the goal`` — the robot never moves. The
old launch bridged that gap with a fixed 8 s wall-clock stagger, which a slow
(GUI / software-rendered) startup could overrun, leaving the stack wedged.

This node gates on the *real* signal instead of a timer: it watches
``/amcl_pose`` (AMCL publishes it on the same update that broadcasts
``map->odom``) and exits(0) the instant it appears — the launch starts
navigation on this node's exit, so nav comes up exactly when localization is
ready, at any startup speed. As a backstop, if AMCL has not localized after a
short grace period it (re)publishes ``/initialpose`` in case the self-seed was
lost. It fails OPEN: after a generous timeout it exits anyway, so a genuinely
stuck AMCL degrades to the old start-nav-anyway behaviour rather than hanging the
launch forever.
"""

import math

from geometry_msgs.msg import PoseWithCovarianceStamped

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class InitialPoseGate(Node):
    def __init__(self) -> None:
        super().__init__('initialpose_pub')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('period', 0.5)            # s between checks
        self.declare_parameter('reseed_after_sec', 4.0)  # backstop-seed if not localized by now
        self.declare_parameter('timeout_sec', 60.0)      # fail-open: release nav anyway after this
        self.x = self.get_parameter('x').value
        self.y = self.get_parameter('y').value
        self.yaw = self.get_parameter('yaw').value
        self.reseed_after = float(self.get_parameter('reseed_after_sec').value)
        self.timeout = float(self.get_parameter('timeout_sec').value)

        self.localized = False
        self.reseeds = 0
        self.start = self.get_clock().now()

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)
        # AMCL publishes /amcl_pose on the update that also broadcasts map->odom,
        # so its arrival is our proof localization is ready for the costmap.
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self._amcl_cb, 10)
        self.timer = self.create_timer(
            float(self.get_parameter('period').value), self._tick)

    def _amcl_cb(self, _msg: PoseWithCovarianceStamped) -> None:
        self.localized = True

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self.start).nanoseconds * 1e-9

    def _seed(self) -> None:
        """Republish the known start pose to /initialpose (self-seed backstop)."""
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

    def _tick(self) -> None:
        el = self._elapsed()
        if self.localized:
            self.get_logger().info(
                f'AMCL localized after {el:.1f}s '
                f'({self.reseeds} backstop seed(s)) — releasing navigation')
            self.timer.cancel()
            raise SystemExit
        if el >= self.timeout:
            self.get_logger().error(
                f'AMCL not localized after {el:.1f}s (no /amcl_pose) — releasing '
                'navigation anyway; check that amcl is up and receiving /scan')
            self.timer.cancel()
            raise SystemExit
        if el >= self.reseed_after:
            # the self-seed apparently didn't take — republish /initialpose
            self._seed()
            self.reseeds += 1
            if self.reseeds == 1 or self.reseeds % 10 == 0:
                self.get_logger().warn(
                    f'AMCL not localized after {el:.1f}s; re-seeding '
                    f'/initialpose ({self.reseeds})')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InitialPoseGate()
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
