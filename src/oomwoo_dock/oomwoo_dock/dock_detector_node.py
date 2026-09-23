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

Fits the dock's known cross-section to the scan and publishes the mouth pose, so
a consumer only has to reverse along +x of that frame.

Where the search starts matters more than the fit itself. Given a bearing from
the dock's IR beacon, a recorded map position, or its own last fix, only a small
offset has to be resolved. Given none of those, the node hunts the whole scan
(`search`): assuming the dock is dead ahead fails the moment somebody parks the
robot facing elsewhere, which on a grid of starting poses was most of them.

Geometry alone cannot always say WHICH dock-shaped thing is the dock, so a fit
must pass the four tests in dock_template.accept and must not jump away from a
fresh estimate. A beacon settles identity outright; these remain the backstop
for when it is out of view.

Runs at a modest rate on purpose: the fit is worth doing well a few times a
second rather than badly every scan, and the robot is nearly stationary while it
lines up.

  subscribes  scan             sensor_msgs/LaserScan  (SensorData QoS)
  subscribes  ~/prior          geometry_msgs/PoseStamped (optional, scan frame)
  subscribes  ~/beacon_bearing std_msgs/Float32       (optional, radians)
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
    'min_coverage': 0.35,         # template must be SUPPORTED, not just explained
    'min_side_coverage': 0.10,    # BOTH sides of the bay must be seen
    'max_intrusions': 2,          # scan points inside the bay: a dock is hollow
    'jump_m': 0.15,               # a fit this far from a fresh estimate...
    'jump_deg': 15.0,             # ...is an impostor, not a correction
    'agree_m': 0.06,              # when lost, two scans must land this close...
    'agree_deg': 8.0,             # ...and this well aligned
    'beacon_range_m': 0.8,        # assumed range when only a bearing is known
    'stale_after_s': 1.5,         # no accepted fix for this long: hunt the scan
    'min_points': 20,             # fewer than this in the gate: no attempt
    'use_fallback_prior': False,  # trust prior_* below when nothing else is known
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
        self.prior = None             # last accepted fit, or a supplied prior
        self.t_prior = None
        self.t_last = None
        self.t_fix = None             # time of the last ACCEPTED fit
        self.pending = None           # a fit waiting for a second opinion
        self.beacon = None            # bearing to the dock's beacon, if any
        self.frame = 'base_scan'
        self.pose_pub = self.create_publisher(PoseStamped, '~/dock_pose', 10)
        self.cost_pub = self.create_publisher(Float32, '~/cost', 10)
        self.cover_pub = self.create_publisher(Float32, '~/coverage', 10)
        self.mark_pub = self.create_publisher(MarkerArray, '~/markers', 5)
        self.create_subscription(
            LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '~/prior', self._on_prior, 10)
        self.create_subscription(
            Float32, '~/beacon_bearing', self._on_beacon, 10)
        self.get_logger().info('dock_detector: fitting the dock cross-section')

    def _p(self, name):
        return self.get_parameter(name).value

    def _on_prior(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.prior = (msg.pose.position.x, msg.pose.position.y, yaw)
        self.t_prior = self.get_clock().now()

    def _on_beacon(self, msg: Float32) -> None:
        """Bearing to the dock's IR beacon, in the scan frame."""
        self.beacon = float(msg.data)

    def _hint(self):
        """
        Where to look for the dock, or None if there is no reason to prefer one.

        Returning None matters: it is what puts the node into a full search of
        the scan instead of a refinement around a guess. A wrong guess is worse
        than no guess, because the scan gets gated around it.
        """
        now = self.get_clock().now()
        if self.prior is not None and self.t_prior is not None:
            age = (now - self.t_prior).nanoseconds * 1e-9
            if age < self._p('hold_prior_s'):
                return self.prior
        if self.beacon is not None:
            # a beacon gives bearing, not range: assume a plausible range and
            # point the dock's mouth back towards the robot
            r = self._p('beacon_range_m')
            return (r * math.cos(self.beacon), r * math.sin(self.beacon),
                    wrap(self.beacon + math.pi))
        if self._p('use_fallback_prior'):
            return (self._p('prior_x'), self._p('prior_y'),
                    math.radians(self._p('prior_yaw_deg')))
        return None

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

        # A hint is a fresh fix, a supplied prior, or a beacon bearing. Without
        # one there is nothing to gate around: gating on the fallback "0.6 m dead
        # ahead" prior threw away the dock's own returns before the search could
        # look at them, leaving 9-23 points of whatever furniture sat ahead.
        now_s = now.nanoseconds * 1e-9
        hint = self._hint()
        if hint is not None:
            gate = self._p('gate_radius_m')
            if gate > 0.0 and len(pts):
                near = np.linalg.norm(
                    pts - np.array([hint[0], hint[1]]), axis=1) < gate
                pts = pts[near]
        if len(pts) < self._p('min_points'):
            self._publish_cost(None)
            return

        got = (self.fit.detect(pts, hint) if hint is not None
               else self.fit.search(pts))
        if got is None:
            self._publish_cost(None)
            return
        self._publish_cost(got.cost, got.coverage)
        if not accept(got, self._p('max_cost'), self._p('min_coverage'),
                      self._p('max_intrusions'), self._p('min_side_coverage')):
            self.get_logger().info(
                'dock fit rejected: cost %.5f (max %.5f), coverage %.2f (min'
                ' %.2f), sides %.2f (min %.2f), %d in the bay (max %d), %d points'
                ' [%s]'
                % (got.cost, self._p('max_cost'), got.coverage,
                   self._p('min_coverage'), got.side_cover,
                   self._p('min_side_coverage'), got.intrusions,
                   self._p('max_intrusions'), got.inliers,
                   'tracking' if hint is not None else 'searching'),
                throttle_duration_sec=2.0)
            self.pending = None
            return
        # Where a dock stands against a wall there is a second, stable fit: the
        # wall as the bay's back and one real side plate as one of its walls. It
        # can score BETTER than the truth, so it is caught by how it arrives --
        # as a jump away from a running estimate. When genuinely lost, two scans
        # must agree instead.
        tracking = (self.t_fix is not None
                    and now_s - self.t_fix < self._p('stale_after_s')
                    and self.prior is not None)
        if tracking:
            good = _near(got.pose, self.prior, self._p('jump_m'),
                         self._p('jump_deg'))
            why = 'jumped %.2f m from the running estimate' % math.hypot(
                got.pose[0] - self.prior[0], got.pose[1] - self.prior[1])
        else:
            good = (self.pending is not None
                    and _near(got.pose, self.pending, self._p('agree_m'),
                              self._p('agree_deg')))
            why = 'waiting for a second scan to agree'
        self.pending = got.pose
        if not good:
            self.get_logger().info('dock fit held: %s' % why,
                                   throttle_duration_sec=2.0)
            return
        self.prior = got.pose
        self.t_prior = now
        self.t_fix = now_s
        self.get_logger().info(
            'dock fix: %+.2f m ahead, %+.2f m across, %+.0f deg; coverage %.2f,'
            ' sides %.2f, %d points'
            % (got.pose[0], got.pose[1], math.degrees(got.pose[2]),
               got.coverage, got.side_cover, got.inliers),
            throttle_duration_sec=5.0)
        self._publish_pose(got.pose, msg.header.stamp)
        if self._p('publish_markers'):
            self._publish_markers(got.pose, msg.header.stamp)

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


def _near(a, b, tol_m, tol_deg):
    """Report whether two estimates describe the same dock."""
    return (math.hypot(a[0] - b[0], a[1] - b[1]) < tol_m
            and abs(wrap(a[2] - b[2])) < math.radians(tol_deg))


def main(args=None) -> None:
    """Spin the detector until shutdown."""
    rclpy.init(args=args)
    node = DockDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        # Ctrl-C can land inside rclpy's message take, which then fails to
        # convert a half-destroyed message. Nothing is actually wrong, but it
        # exits non-zero and buries the run's logs under a traceback.
        if rclpy.ok():
            raise
        node.get_logger().debug('ignoring shutdown race: %s' % exc)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
