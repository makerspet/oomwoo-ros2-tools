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
Global relocalizer service: branch-and-bound scan-to-map matching.

Pure mechanism. On a ~/relocalize (oomwoo_localization_msgs/Relocalize) call it
matches the latest /scan against the loaded /map over the WHOLE map and all
headings (bnb_relocalizer), and returns the best base-frame pose, its score,
and a confidence margin (how much the best beats the next distinct cluster).
It does NOT decide when to relocalize, whether to trust a low-confidence fix,
or how to move -- that policy belongs to an application node. It only reports.

  sub  /map                  nav_msgs/OccupancyGrid  (transient_local)
  sub  /scan                 sensor_msgs/LaserScan   (SensorData QoS)
  tf   base_frame <- scan    (fold the lidar mount into the reported pose)
  srv  ~/relocalize          oomwoo_localization_msgs/Relocalize
  pub  ~/relocalized_pose    geometry_msgs/PoseWithCovarianceStamped (last fix)
  pub  /initialpose          DEBUG ONLY (publish_initialpose), for eyeballing in
                             RViz -- in production localization_manager owns the
                             decision to commit a fix; this node never acts
"""

import math
import time

from geometry_msgs.msg import PoseWithCovarianceStamped

from nav_msgs.msg import OccupancyGrid

import numpy as np

from oomwoo_localization import bnb_relocalizer as bnb

from oomwoo_localization_msgs.srv import Relocalize

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.time import Time

from sensor_msgs.msg import LaserScan

from tf2_ros import Buffer, TransformException, TransformListener


def map_qos() -> QoSProfile:
    return QoSProfile(
        depth=1, history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class GlobalRelocalizer(Node):

    def __init__(self) -> None:
        super().__init__('global_relocalizer')
        self.global_frame = self.declare_parameter('global_frame', 'map').value
        self.base_frame = self.declare_parameter(
            'base_frame', 'base_footprint').value
        self.sigma = self.declare_parameter('sigma_m', 0.10).value
        self.levels = int(self.declare_parameter('levels', 4).value)
        self.max_range = self.declare_parameter('max_range', 8.0).value
        self.occ_thresh = int(self.declare_parameter('occ_thresh', 65).value)
        self.min_score_norm = self.declare_parameter('min_score_norm', 0.5).value
        self.min_conf = self.declare_parameter('min_confidence', 0.15).value
        self.exclude_m = self.declare_parameter('exclude_radius_m', 0.5).value
        self.stride = int(self.declare_parameter('beam_stride', 1).value)
        # DEBUG ONLY: seed /initialpose so the RViz robot jumps to the fix.
        # Production leaves this false -- localization_manager owns the commit.
        self.pub_init = self.declare_parameter(
            'publish_initialpose', False).value
        self.tf_timeout = self.declare_parameter('tf_timeout_s', 0.2).value

        self._prep = None
        self._res = 0.0
        self._origin = (0.0, 0.0)
        self._scan = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos())
        self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '~/relocalized_pose', 10)
        self.init_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.srv = self.create_service(
            Relocalize, '~/relocalize', self._on_relocalize)
        self.get_logger().info(
            'global_relocalizer: call ~/relocalize to match /scan vs /map '
            '(sigma=%.2fm, %d levels)' % (self.sigma, self.levels))

    def _on_map(self, msg: OccupancyGrid) -> None:
        w, h = msg.info.width, msg.info.height
        grid = np.array(msg.data, dtype=np.int16).reshape(h, w)
        occ = grid >= self.occ_thresh
        if not occ.any():
            self._prep = None
            self.get_logger().warn('map has no occupied cells; relocalize off')
            return
        field = bnb.build_likelihood_field(occ, msg.info.resolution, self.sigma)
        self._prep = bnb.prepare(
            field, msg.info.resolution, self.max_range, self.levels)
        self._res = msg.info.resolution
        self._origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.get_logger().info(
            'map ready: %d x %d @ %.3f m/cell' % (w, h, self._res))

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg

    def _scan_xy_base(self):
        msg = self._scan
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        n = ranges.size
        angles = msg.angle_min + np.arange(n) * msg.angle_increment
        valid = (np.isfinite(ranges) & (ranges >= msg.range_min)
                 & (ranges <= msg.range_max))
        if self.stride > 1:
            keep = np.zeros(n, dtype=bool)
            keep[::self.stride] = True
            valid &= keep
        r, a = ranges[valid], angles[valid]
        xs, ys = r * np.cos(a), r * np.sin(a)      # endpoints in the scan frame
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, msg.header.frame_id, Time(),
                timeout=Duration(seconds=self.tf_timeout))
        except TransformException:
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        bx = tf.transform.translation.x + c * xs - s * ys
        by = tf.transform.translation.y + s * xs + c * ys
        return np.stack([bx, by], axis=1)

    def _on_relocalize(self, request, response):
        if self._prep is None or self._scan is None:
            response.success = False
            response.message = 'no map or scan received yet'
            return response
        scan_xy = self._scan_xy_base()
        if scan_xy is None or scan_xy.shape[0] < 10:
            response.success = False
            response.message = 'scan TF unavailable or too few valid beams'
            return response

        t0 = time.monotonic()
        best = bnb.match_bnb(self._prep, self._res, self._origin, scan_xy)
        bx, by, bth = best['pose']
        runner = bnb.match_bnb(
            self._prep, self._res, self._origin, scan_xy,
            exclude=(bx, by, self.exclude_m))
        runtime = time.monotonic() - t0

        s1 = best['score']
        s2 = runner['score'] if runner['pose'] is not None else 0.0
        conf = max(0.0, min(1.0, (s1 - s2) / s1)) if s1 > 0 else 0.0
        norm = s1 / max(best['n_beams'], 1)
        success = norm >= self.min_score_norm and conf >= self.min_conf

        pose = self._pose_msg(bx, by, bth, conf)
        self.pose_pub.publish(pose)
        if self.pub_init and success:
            self.init_pub.publish(pose)

        response.success = success
        response.pose = pose
        response.score = float(s1)
        response.score_normalized = float(norm)
        response.confidence = float(conf)
        response.runtime_s = float(runtime)
        response.message = (
            '%s: x=%.2f y=%.2f yaw=%.1fdeg score=%.2f conf=%.2f (%.0fms)'
            % ('OK' if success else 'LOW', bx, by, math.degrees(bth),
               norm, conf, runtime * 1e3))
        self.get_logger().info(response.message)
        return response

    def _pose_msg(self, x, y, yaw, conf) -> PoseWithCovarianceStamped:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.global_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        pos_var = (0.05 + 0.5 * (1.0 - conf)) ** 2
        yaw_var = math.radians(2.0 + 30.0 * (1.0 - conf)) ** 2
        msg.pose.covariance[0] = pos_var
        msg.pose.covariance[7] = pos_var
        msg.pose.covariance[35] = yaw_var
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GlobalRelocalizer()
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
