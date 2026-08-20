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
Teleport ("kidnap") the robot in Gazebo for relocalization testing.

Exposes a ``~/kidnap`` (std_srvs/Trigger) service. On each call it picks a random
reachable free pose on the map (at least ``min_jump`` from the current true
pose), teleports the model there via Gazebo's ``set_pose`` service, then
publishes ``/kidnap_trigger`` so the recovery node knows it was moved and
``~/target_pose`` (the new ground-truth pose) so the harness can score
relocalization error.

It also accepts ``~/kidnap_to`` (geometry_msgs/PoseStamped): teleport to a
SPECIFIED pose instead of a random one (same trigger + target_pose behaviour),
e.g. ``ros2 topic pub --once ~/kidnap_to ...``.

Teleport happens in world coordinates; the living_room map frame is aligned with
the world origin, so map xy == world xy (identity offset, configurable).
"""

import math
import random
import subprocess
from typing import Optional

from geometry_msgs.msg import PoseStamped

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from std_msgs.msg import Empty

from std_srvs.srv import Trigger

FREE = 0
OCC_THRESH = 50


def latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1, history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class KidnapInjector(Node):
    def __init__(self) -> None:
        super().__init__('kidnap_injector')
        self.declare_parameter('robot_model_name', 'oomwoo_one')
        self.declare_parameter('world_name', 'default')
        self.declare_parameter('min_jump', 1.5)      # m, min teleport distance
        # m from obstacles (robot center; body radius 0.175 + 3D-mesh overhang margin)
        self.declare_parameter('wall_clearance', 0.50)
        self.declare_parameter('spawn_z', 0.06)
        self.declare_parameter('seed', 42)

        self.model = self.get_parameter('robot_model_name').value
        self.world = self.get_parameter('world_name').value
        self.min_jump = self.get_parameter('min_jump').value
        self.clearance = self.get_parameter('wall_clearance').value
        self.spawn_z = self.get_parameter('spawn_z').value
        self.rng = random.Random(self.get_parameter('seed').value)

        from nav_msgs.msg import OccupancyGrid
        self.info = None
        self.safe_cells = None       # list of (row,col) far enough from walls
        self.true_xy: Optional[tuple] = None

        self.create_subscription(OccupancyGrid, 'map', self._on_map, latched_qos())
        self.create_subscription(
            PoseStamped, 'ground_truth/pose', self._on_truth, 10)
        self.trigger_pub = self.create_publisher(Empty, '/kidnap_trigger', 10)
        self.target_pub = self.create_publisher(PoseStamped, '~/target_pose', 10)
        self.srv = self.create_service(Trigger, '~/kidnap', self._on_kidnap)
        self.create_subscription(
            PoseStamped, '~/kidnap_to', self._on_kidnap_to, 10)
        self.get_logger().info(
            'kidnap_injector up; ~/kidnap (random) + ~/kidnap_to (pose) ready')

    def _on_map(self, msg) -> None:
        if self.info is not None:
            return
        self.info = msg.info
        h, w = msg.info.height, msg.info.width
        grid = np.asarray(msg.data, dtype=np.int16).reshape(h, w)
        free = (grid >= 0) & (grid < OCC_THRESH)
        obstacle = grid >= OCC_THRESH
        infl = max(1, int(round(self.clearance / msg.info.resolution)))
        blocked = _dilate(obstacle, infl)
        safe = free & ~blocked
        ys, xs = np.where(safe)
        self.safe_cells = list(zip(ys.tolist(), xs.tolist()))
        self.get_logger().info(f'{len(self.safe_cells)} safe teleport cells')

    def _on_truth(self, msg: PoseStamped) -> None:
        self.true_xy = (msg.pose.position.x, msg.pose.position.y)

    def _cell_to_world(self, row: int, col: int) -> tuple:
        res = self.info.resolution
        x = self.info.origin.position.x + (col + 0.5) * res
        y = self.info.origin.position.y + (row + 0.5) * res
        return (x, y)

    def _on_kidnap(self, _req, resp: Trigger.Response) -> Trigger.Response:
        if not self.safe_cells:
            resp.success = False
            resp.message = 'no map / no safe cells yet'
            return resp
        # choose a far-enough random safe pose
        target = None
        for _ in range(200):
            row, col = self.rng.choice(self.safe_cells)
            x, y = self._cell_to_world(row, col)
            if self.true_xy is None or math.hypot(
                    x - self.true_xy[0], y - self.true_xy[1]) >= self.min_jump:
                target = (x, y)
                break
        if target is None:
            resp.success = False
            resp.message = 'could not find far pose'
            return resp

        yaw = self.rng.uniform(-math.pi, math.pi)
        if not self._do_kidnap(target[0], target[1], yaw):
            resp.success = False
            resp.message = 'gz set_pose failed'
            return resp
        resp.success = True
        resp.message = f'kidnapped to ({target[0]:.2f},{target[1]:.2f},{yaw:.2f})'
        return resp

    def _on_kidnap_to(self, msg: PoseStamped) -> None:
        # teleport to a SPECIFIED pose (ros2 topic pub ~/kidnap_to ...)
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if not self._do_kidnap(msg.pose.position.x, msg.pose.position.y, yaw):
            self.get_logger().error('kidnap_to: gz set_pose failed')

    def _do_kidnap(self, x: float, y: float, yaw: float) -> bool:
        # teleport, then announce: target_pose for ground_truth, trigger for reco
        if not self._teleport(x, y, yaw):
            return False
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'map'
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        self.target_pub.publish(ps)
        self.trigger_pub.publish(Empty())
        self.get_logger().warn('KIDNAP to (%.2f, %.2f, %.2f)' % (x, y, yaw))
        return True

    def _teleport(self, x: float, y: float, yaw: float) -> bool:
        qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        req = (f'name: "{self.model}" '
               f'position {{ x: {x} y: {y} z: {self.spawn_z} }} '
               f'orientation {{ x: 0 y: 0 z: {qz} w: {qw} }}')
        cmd = ['gz', 'service', '-s', f'/world/{self.world}/set_pose',
               '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
               '--timeout', '3000', '--req', req]
        try:
            out = subprocess.run(cmd, capture_output=True, text=True, timeout=6)
            return 'true' in out.stdout.lower()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'teleport error: {e}')
            return False


def _dilate(mask, radius):
    # Euclidean disk, so clearance is uniform in every direction -- a diamond
    # (4-connected) kernel leaves only ~0.7x the radius diagonally, which can
    # drop a random kidnap onto an obstacle's corner.
    if radius <= 0:
        return mask.copy()
    h, w = mask.shape
    out = np.zeros_like(mask)
    for di in range(-radius, radius + 1):
        for dj in range(-radius, radius + 1):
            if di * di + dj * dj > radius * radius:
                continue
            gi0, gi1 = max(0, -di), min(h, h - di)
            gj0, gj1 = max(0, -dj), min(w, w - dj)
            if gi0 < gi1 and gj0 < gj1:
                out[gi0:gi1, gj0:gj1] |= mask[gi0 + di:gi1 + di,
                                              gj0 + dj:gj1 + dj]
    return out


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KidnapInjector()
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
