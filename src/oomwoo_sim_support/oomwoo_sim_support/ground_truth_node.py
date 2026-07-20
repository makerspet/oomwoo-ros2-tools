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
Publish the robot's ground-truth pose for honest metric evaluation.

The sim's ``gz-sim-odometry-publisher`` produces *noise-free* odometry, so
``/odom`` is effectively ground truth — but odometry does NOT jump when the robot
is teleported ("kidnapped"). To stay correct through kidnaps, this node tracks a
rigid SE(2) offset between the odom frame and the true map frame:

    true(t) = T_offset ∘ odom(t)

At startup ``T_offset`` maps the initial odom to the known spawn pose. Each time
the kidnap injector announces a teleport on ``~/target_pose``, the offset is
recomputed from the odom sampled right after the jump, so the true pose is
correct both at the teleport point and while the robot drives during recovery.

  sub  /odom                         nav_msgs/Odometry
  sub  /kidnap_injector/target_pose  geometry_msgs/PoseStamped   (optional)
  pub  ~/pose                        geometry_msgs/PoseStamped   (map frame)
"""

import math

from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import Odometry

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import Float32


def _yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _compose(a, b):
    """SE(2) compose: apply pose a to pose b. a,b = (x,y,yaw)."""
    ca, sa = math.cos(a[2]), math.sin(a[2])
    return (a[0] + ca * b[0] - sa * b[1],
            a[1] + sa * b[0] + ca * b[1],
            a[2] + b[2])


def _inverse(a):
    ca, sa = math.cos(a[2]), math.sin(a[2])
    return (-(ca * a[0] + sa * a[1]),
            (sa * a[0] - ca * a[1]),
            -a[2])


class GroundTruth(Node):
    def __init__(self) -> None:
        super().__init__('ground_truth')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        self.declare_parameter('spawn_yaw', 0.0)
        self.map_frame = self.get_parameter('map_frame').value
        spawn = (self.get_parameter('spawn_x').value,
                 self.get_parameter('spawn_y').value,
                 self.get_parameter('spawn_yaw').value)

        # T_offset maps odom -> true map. Seeded from spawn on the first odom.
        self.offset = None
        self.spawn = spawn
        self.pending_teleport = None   # (x,y,yaw) to apply on the next odom

        self.pose_pub = self.create_publisher(PoseStamped, '~/pose', 10)
        self.yaw_pub = self.create_publisher(Float32, '~/yaw', 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 20)
        self.create_subscription(
            PoseStamped, '/kidnap_injector/target_pose', self._on_teleport, 10)
        self.get_logger().info('ground_truth up (teleport-aware odom truth)')

    def _on_teleport(self, msg: PoseStamped):
        q = msg.pose.orientation
        self.pending_teleport = (msg.pose.position.x, msg.pose.position.y,
                                 _yaw(q.x, q.y, q.z, q.w))

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        odom = (p.x, p.y, _yaw(q.x, q.y, q.z, q.w))

        if self.offset is None:
            # true(spawn) = spawn = compose(offset, odom0)  ->  offset = spawn ∘ odom0^-1
            self.offset = _compose(self.spawn, _inverse(odom))
        if self.pending_teleport is not None:
            # after the jump, this odom sample corresponds to the teleport pose
            self.offset = _compose(self.pending_teleport, _inverse(odom))
            self.pending_teleport = None

        tx, ty, tyaw = _compose(self.offset, odom)
        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.map_frame
        out.pose.position.x = tx
        out.pose.position.y = ty
        out.pose.orientation.z = math.sin(tyaw / 2.0)
        out.pose.orientation.w = math.cos(tyaw / 2.0)
        self.pose_pub.publish(out)
        self.yaw_pub.publish(Float32(data=float(tyaw)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundTruth()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
