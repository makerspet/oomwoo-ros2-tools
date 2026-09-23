#!/usr/bin/env python3
# Copyright 2026 OOMWOO
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
Find the docking station in the LiDAR scan and publish its pose.

Fits the dock's known cross-section to the scan, seeded from a prior pose --
the dock's recorded position, the bearing of its beacon, or the node's own last
estimate. Publishes the mouth pose, so a consumer only has to reverse along
+x of that frame.

Runs at a modest rate on purpose: the fit is worth doing well a few times a
second rather than badly every scan, and the robot is nearly stationary while it
lines up.

  subscribes  scan          sensor_msgs/LaserScan   (SensorData QoS)
  subscribes  ~/prior       geometry_msgs/PoseStamped  (optional, in the scan frame)
  publishes   ~/dock_pose   geometry_msgs/PoseStamped  (mouth centre, scan frame)
  publishes   ~/cost        std_msgs/Float32        (fit cost; lower is better)
  publishes   ~/coverage    std_msgs/Float32        (share of the template seen)
  publishes   ~/markers     visualization_msgs/MarkerArray
"""

import math

from geometry_msgs.msg import Point, PoseStamped

import numpy as np

from oomwoo_dock.dock_template import accept, DockFitter, template, wrap

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan

from std_msgs.msg import Float32

from visualization_msgs.msg import Marker, MarkerArray

DEFAULTS = {
    'detect_hz': 2.0,             # fit rate; the scan arrives faster
    'max_range_m': 2.0,           # ignore returns beyond this
    'gate_radius_m': 1.2,         # keep points within this of the prior
    'max_cost': 0.004,            # above this the fit is not believed
    'min_coverage': 0.40,         # template must be SUPPORTED, not just explained
    'min_points': 20,             # fewer than this in the gate: no attempt
    'prior_x': 0.6,               # fallback prior, in the scan frame
    'prior_y': 0.0,
    'prior_yaw_deg': 0.0,
    'hold_prior_s': 3.0,          # reuse the last good pose as prior this long
    'publish_markers': True,
}


class DockDetector(Node):
    """Fit the dock template to each scan and publish the mouth pose."""

    def __init__(self) -> None:
        """Set up parameters, the fitter and the publishers."""
        super().__init__('dock_detector')
        for name, default in DEFAULTS.items():
            self.declare_parameter(name, default)
        self.fit = DockFitter()
        self.tmpl = template()
        self.prior = None
        self.t_prior = None
        self.t_last = None
        self.frame = 'base_scan'
        self.pose_pub = self.create_publisher(PoseStamped, '~/dock_pose', 10)
        self.cost_pub = self.create_publisher(Float32, '~/cost', 10)
        self.cover_pub = self.create_publisher(Float32, '~/coverage', 10)
        self.mark_pub = self.create_publisher(MarkerArray, '~/markers', 5)
        self.create_subscription(
            LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '~/prior', self._on_prior, 10)
        self.get_logger().info('dock_detector: fitting the dock cross-section')

    def _p(self, name):
        return self.get_parameter(name).value

    def _on_prior(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.prior = (msg.pose.position.x, msg.pose.position.y, yaw)
        self.t_prior = self.get_clock().now()

    def _current_prior(self):
        """Last good fit while it is fresh, else the supplied or default prior."""
        now = self.get_clock().now()
        if self.prior is not None and self.t_prior is not None:
            age = (now - self.t_prior).nanoseconds * 1e-9
            if age < self._p('hold_prior_s'):
                return self.prior
        return (self._p('prior_x'), self._p('prior_y'),
                math.radians(self._p('prior_yaw_deg')))

    def _on_scan(self, msg: LaserScan) -> None:
        now = self.get_clock().now()
        period = 1.0 / max(self._p('detect_hz'), 0.1)
        if self.t_last is not None:
            if (now - self.t_last).nanoseconds * 1e-9 < period:
                return
        self.t_last = now
        self.frame = msg.header.frame_id or self.frame

        max_r = self._p('max_range_m')
        ang = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        rng = np.asarray(msg.ranges, dtype=float)
        ok = np.isfinite(rng) & (rng > msg.range_min) & (rng < max_r)
        pts = np.stack([rng[ok] * np.cos(ang[ok]), rng[ok] * np.sin(ang[ok])], axis=1)

        prior = self._current_prior()
        gate = self._p('gate_radius_m')
        if gate > 0.0 and len(pts):
            near = np.linalg.norm(pts - np.array([prior[0], prior[1]]), axis=1) < gate
            pts = pts[near]
        if len(pts) < self._p('min_points'):
            self._publish_cost(None)
            return

        got = self.fit.detect(pts, prior)
        if got is None:
            self._publish_cost(None)
            return
        self._publish_cost(got.cost, got.coverage)
        if not accept(got, self._p('max_cost'), self._p('min_coverage')):
            self.get_logger().info(
                'dock fit rejected: cost %.5f (max %.5f), coverage %.2f (min %.2f),'
                ' %d points' % (got.cost, self._p('max_cost'), got.coverage,
                                self._p('min_coverage'), got.inliers),
                throttle_duration_sec=2.0)
            return
        pose = got.pose
        self.prior = pose
        self.t_prior = now
        self._publish_pose(pose, msg.header.stamp)
        if self._p('publish_markers'):
            self._publish_markers(pose, msg.header.stamp)

    def _publish_cost(self, cost, coverage=0.0) -> None:
        self.cost_pub.publish(Float32(data=float(9.9 if cost is None else cost)))
        self.cover_pub.publish(Float32(data=float(coverage)))

    def _publish_pose(self, pose, stamp) -> None:
        msg = PoseStamped()
        msg.header.frame_id = self.frame
        msg.header.stamp = stamp
        msg.pose.position.x = float(pose[0])
        msg.pose.position.y = float(pose[1])
        msg.pose.orientation.z = math.sin(wrap(pose[2]) / 2.0)
        msg.pose.orientation.w = math.cos(wrap(pose[2]) / 2.0)
        self.pose_pub.publish(msg)

    def _publish_markers(self, pose, stamp) -> None:
        """Draw the fitted template where the estimate puts it."""
        c, s = math.cos(pose[2]), math.sin(pose[2])

        def to_robot(p):
            q = Point()
            q.x = pose[0] + p[0] * c - p[1] * s
            q.y = pose[1] + p[0] * s + p[1] * c
            return q

        arr = MarkerArray()
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = stamp
        m.ns = 'dock'
        m.id = 0
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.008
        m.color.g = 1.0
        m.color.b = 0.4
        m.color.a = 0.9
        m.lifetime.sec = 2
        for a, b in self.tmpl[0]:
            m.points.append(to_robot(a))
            m.points.append(to_robot(b))
        for ctr, r in self.tmpl[1]:
            prev = None
            for k in range(13):
                th = 2.0 * math.pi * k / 12.0
                pt = (ctr[0] + r * math.cos(th), ctr[1] + r * math.sin(th))
                if prev is not None:
                    m.points.append(to_robot(prev))
                    m.points.append(to_robot(pt))
                prev = pt
        arr.markers.append(m)
        axis = Marker()
        axis.header.frame_id = self.frame
        axis.header.stamp = stamp
        axis.ns = 'dock'
        axis.id = 1
        axis.type = Marker.ARROW
        axis.action = Marker.ADD
        axis.pose.orientation.w = 1.0
        axis.scale.x = 0.012
        axis.scale.y = 0.03
        axis.color.g = 1.0
        axis.color.a = 0.9
        axis.lifetime.sec = 2
        axis.points = [to_robot((0.0, 0.0)), to_robot((0.25, 0.0))]
        arr.markers.append(axis)
        self.mark_pub.publish(arr)


def main(args=None) -> None:
    """Spin the detector until shutdown."""
    rclpy.init(args=args)
    node = DockDetector()
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
