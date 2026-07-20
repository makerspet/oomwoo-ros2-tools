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
        self.get_logger().info('kidnap_injector up; ~/kidnap ready')

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
        ok = self._teleport(target[0], target[1], yaw)
        if not ok:
            resp.success = False
            resp.message = 'gz set_pose failed'
            return resp

        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'map'
        ps.pose.position.x = target[0]
        ps.pose.position.y = target[1]
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        self.target_pub.publish(ps)
        self.trigger_pub.publish(Empty())
        resp.success = True
        resp.message = f'kidnapped to ({target[0]:.2f},{target[1]:.2f},{yaw:.2f})'
        self.get_logger().warn('KIDNAP ' + resp.message)
        return resp

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
    if radius <= 0:
        return mask.copy()
    out = mask.copy()
    for _ in range(radius):
        s = out.copy()
        s[1:, :] |= out[:-1, :]
        s[:-1, :] |= out[1:, :]
        s[:, 1:] |= out[:, :-1]
        s[:, :-1] |= out[:, 1:]
        out = s
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
