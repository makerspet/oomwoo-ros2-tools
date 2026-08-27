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
Score live scans against the static map to flag when the robot is lost.

slam_toolbox keeps its scan-match quality internal, and neither its /pose
covariance nor AMCL's rises on a clean kidnap (both stay confidently wrong). So
we compute the quality ourselves: for each beam, how close does its endpoint
land to a wall in the static map, at the pose the PRIMARY localizer currently
believes (map->scan from TF, continuous, unlike the sparse /pose topic).

  quality = fraction of valid beams whose endpoint is within match_dist_m of a
            mapped wall  (a hard inlier ratio, in [0, 1])

A kidnap makes almost nothing match -> quality collapses -> /localization_lost.
An unmapped shoe/box only blanks a contiguous chunk -> quality stays high -> no
false alarm. The same inlier/outlier split (and a contiguity clustering of the
outliers) is published as an annotated point cloud plus a distance histogram so
the classification can be eyeballed in RViz. The contiguous outlier clusters are
also treated as dynamic obstacles for LOCALIZATION only. Their non-wall beams are stripped out of a
republished /scan_filtered so a stray box or chair no longer drags the running
scan match down. Filtering runs ONLY while the pose is trusted (quality high)
-- when lost, every beam looks like an outlier, so it passes the scan
through untouched -- and it never blanks more than max_filter_frac of the scan,
so it cannot starve the matcher. (That guard is also why the filter helps
day-to-day tracking, not a lost relocalize: rejecting outliers needs a pose you
can still believe.)

For perception experiments, ~/scan_scored republishes the FULL scan (nothing
dropped) with each ray's static-ness -- exp(-d^2 / 2 sigma^2), 1.0 on a mapped
wall and ->0 for a dynamic return -- in its intensity field, leaving clustering
and recognition to the subscriber. DETECTING dynamic obstacles as objects is that
subscriber's job (see oomwoo_perception/dynamic_object_detector), not this
monitor's -- health only measures and filters. Experimental; encoding may change.

  sub  /scan               sensor_msgs/LaserScan   (SensorData QoS)
  sub  /map                nav_msgs/OccupancyGrid  (transient_local)
  tf   map -> scan frame   (looked up at each scan stamp)
  pub  ~/quality           std_msgs/Float32                 (inlier ratio)
  pub  /localization_lost  std_msgs/Empty                   (edge, when lost)
  pub  /scan_filtered      sensor_msgs/LaserScan     (scan minus dynamic obstacles)
  pub  ~/scan_scored       sensor_msgs/LaserScan     (full scan; intensity=static-ness)
  pub  ~/dist_histogram    std_msgs/Float32MultiArray       (per-scan, debug)
  pub  ~/scan_annotated    sensor_msgs/PointCloud2          (debug, map frame)
"""

import math

from nav_msgs.msg import OccupancyGrid

import numpy as np

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

from scipy.ndimage import distance_transform_edt

from sensor_msgs.msg import LaserScan, PointCloud2, PointField

from std_msgs.msg import Empty, Float32, Float32MultiArray

from tf2_ros import Buffer, TransformException, TransformListener

# ~/scan_annotated intensity labels (RViz: colour by intensity)
LBL_INLIER = 1.0        # endpoint matched a mapped wall
LBL_OUTLIER = 2.0       # unmatched, not part of a cluster (noise)
LBL_CLUSTER0 = 10.0     # first dynamic-obstacle cluster; +1 per further cluster


def static_score(d, sigma):
    """Map per-ray wall distance d to static-ness in [0, 1] (1 = on a wall)."""
    big = 10.0 * sigma
    dd = np.minimum(np.where(np.isfinite(d), d, big), big)
    return np.exp(-(dd * dd) / (2.0 * sigma * sigma))


def map_qos() -> QoSProfile:
    return QoSProfile(
        depth=1, history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class LocalizationHealth(Node):

    def __init__(self) -> None:
        super().__init__('localization_health')
        self.global_frame = self.declare_parameter('global_frame', 'map').value
        self.match_dist = self.declare_parameter('match_dist_m', 0.10).value
        self.lost_ratio = self.declare_parameter('lost_ratio', 0.5).value
        self.ok_ratio = self.declare_parameter('ok_ratio', 0.7).value
        self.lost_hold = self.declare_parameter('lost_hold_s', 1.0).value
        self.min_beams = int(self.declare_parameter('min_valid_beams', 30).value)
        self.stride = int(self.declare_parameter('beam_stride', 1).value)
        self.cluster_gap = self.declare_parameter('cluster_gap_m', 0.20).value
        self.min_cluster = int(
            self.declare_parameter('min_cluster_beams', 4).value)
        self.occ_thresh = int(self.declare_parameter('occ_thresh', 65).value)
        self.hist_max = self.declare_parameter('hist_max_m', 1.0).value
        self.hist_bins = int(self.declare_parameter('hist_bins', 20).value)
        self.log_period = self.declare_parameter('log_period_s', 2.0).value
        self.tf_timeout = self.declare_parameter('tf_timeout_s', 0.10).value
        self.pub_annot = self.declare_parameter('publish_annotated', True).value
        self.pub_hist = self.declare_parameter('publish_histogram', True).value
        self.filter_on = self.declare_parameter('filter_enable', True).value
        self.filter_min_q = self.declare_parameter(
            'filter_min_quality', 0.6).value
        self.max_filter_frac = self.declare_parameter(
            'max_filter_frac', 0.4).value
        self.pub_scored = self.declare_parameter(
            'publish_scored_scan', True).value
        self.score_sigma = self.declare_parameter('score_sigma_m', 0.10).value

        self._df = None       # distance-to-nearest-wall field (m), (h, w)
        self._res = 0.0
        self._ox = 0.0
        self._oy = 0.0
        self._w = 0
        self._h = 0
        self._t_low = None    # first time quality dropped below lost_ratio
        self._armed = True    # False after firing, until quality recovers
        self._t_log = None

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos())
        self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.q_pub = self.create_publisher(Float32, '~/quality', 10)
        self.lost_pub = self.create_publisher(Empty, '/localization_lost', 10)
        self.hist_pub = self.create_publisher(
            Float32MultiArray, '~/dist_histogram', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '~/scan_annotated', 5)
        self.filt_pub = self.create_publisher(
            LaserScan, '/scan_filtered', qos_profile_sensor_data)
        self.scored_pub = self.create_publisher(
            LaserScan, '~/scan_scored', qos_profile_sensor_data)
        self.get_logger().info(
            'localization_health: quality = inliers within %.2fm; '
            'lost < %.2f (held %.1fs), recovered > %.2f'
            % (self.match_dist, self.lost_ratio, self.lost_hold, self.ok_ratio))

    def _on_map(self, msg: OccupancyGrid) -> None:
        w, h = msg.info.width, msg.info.height
        grid = np.array(msg.data, dtype=np.int16).reshape(h, w)
        occ = grid >= self.occ_thresh
        if not occ.any():
            self._df = None
            self.get_logger().warn('map has no occupied cells; health disabled')
            return
        # distance_transform_edt gives, for each nonzero cell, the distance to
        # the nearest zero cell -> feed free=1/occupied=0 to get metres-to-wall.
        # Assumes map origin yaw = 0 (true for our saved maps).
        self._df = distance_transform_edt(~occ) * msg.info.resolution
        self._res = msg.info.resolution
        self._ox = msg.info.origin.position.x
        self._oy = msg.info.origin.position.y
        self._w, self._h = w, h
        self.get_logger().info(
            'map ready: %d x %d @ %.3f m/cell' % (w, h, self._res))

    def _on_scan(self, msg: LaserScan) -> None:
        if self._df is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, msg.header.frame_id, msg.header.stamp,
                timeout=Duration(seconds=self.tf_timeout))
        except TransformException:
            return                                  # TF not ready yet; skip scan

        ranges = np.asarray(msg.ranges, dtype=np.float64)
        n = ranges.size
        if n == 0:
            return
        angles = msg.angle_min + np.arange(n) * msg.angle_increment
        valid = (np.isfinite(ranges) & (ranges >= msg.range_min)
                 & (ranges <= msg.range_max))
        if self.stride > 1:
            keep = np.zeros(n, dtype=bool)
            keep[::self.stride] = True
            valid &= keep
        idx = np.nonzero(valid)[0]
        if idx.size < self.min_beams:
            return

        mx, my = self._to_map(tf, ranges[idx], angles[idx])
        gx = np.floor((mx - self._ox) / self._res).astype(int)
        gy = np.floor((my - self._oy) / self._res).astype(int)
        inb = (gx >= 0) & (gx < self._w) & (gy >= 0) & (gy < self._h)
        d = np.full(idx.size, np.inf)
        d[inb] = self._df[gy[inb], gx[inb]]
        inlier = d <= self.match_dist
        quality = float(inlier.mean())

        self.q_pub.publish(Float32(data=quality))
        now = self.get_clock().now()
        self._update_lost(quality, now)
        if self.pub_hist:
            self.hist_pub.publish(Float32MultiArray(
                data=self._histogram(d).astype(np.float32).tolist()))
        labels = None
        if self.pub_annot or self.filter_on:
            labels = self._label(inlier, mx, my)
        if self.pub_annot:
            self.cloud_pub.publish(
                self._cloud(msg.header.stamp, mx, my, labels))
        if self.filter_on:
            self._publish_filtered(msg, idx, labels, quality)
        if self.pub_scored:
            self._publish_scored(msg, idx, d)
        self._maybe_log(quality, inlier, d, now)

    def _to_map(self, tf, r, a):
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        xs, ys = r * np.cos(a), r * np.sin(a)
        mx = tf.transform.translation.x + cos_y * xs - sin_y * ys
        my = tf.transform.translation.y + sin_y * xs + cos_y * ys
        return mx, my

    def _histogram(self, d):
        clipped = np.minimum(np.where(np.isfinite(d), d, self.hist_max),
                             self.hist_max)
        counts, _ = np.histogram(
            clipped, bins=self.hist_bins, range=(0.0, self.hist_max))
        return counts

    def _label(self, inlier, mx, my):
        labels = np.where(inlier, LBL_INLIER, LBL_OUTLIER)
        n = inlier.size
        cid, i = 0, 0
        while i < n:
            if inlier[i]:
                i += 1
                continue
            j = i + 1
            while (j < n and not inlier[j]
                   and math.hypot(mx[j] - mx[j - 1], my[j] - my[j - 1])
                   <= self.cluster_gap):
                j += 1
            if (j - i) >= self.min_cluster:
                labels[i:j] = LBL_CLUSTER0 + cid
                cid += 1
            i = j
        return labels

    def _cloud(self, stamp, mx, my, labels) -> PointCloud2:
        n = mx.size
        buf = np.zeros((n, 4), dtype=np.float32)
        buf[:, 0] = mx
        buf[:, 1] = my
        buf[:, 3] = labels
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = self.global_frame
        msg.height = 1
        msg.width = n
        msg.fields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12,
                       datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * n
        msg.is_dense = True
        msg.data = buf.tobytes()
        return msg

    def _publish_filtered(self, msg, idx, labels, quality) -> None:
        cluster = labels >= LBL_CLUSTER0
        n_cl = int(cluster.sum())
        frac = n_cl / max(len(msg.ranges), 1)
        # only strip obstacles when the pose is trusted (else every beam is an
        # "outlier"), and never blank so much of the scan that matching starves
        if (n_cl == 0 or quality < self.filter_min_q
                or frac > self.max_filter_frac):
            self.filt_pub.publish(msg)              # pass the scan through
            return
        ranges = list(msg.ranges)
        for j in np.nonzero(cluster)[0]:
            ranges[int(idx[j])] = float('inf')      # blank the obstacle beam
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = ranges
        out.intensities = msg.intensities
        self.filt_pub.publish(out)

    def _publish_scored(self, msg, idx, d) -> None:
        # full scan, nothing dropped; intensity = static-ness in [0, 1]
        # (1.0 = endpoint on a mapped wall, -> 0 = a dynamic return).
        inten = np.zeros(len(msg.ranges), dtype=np.float32)
        inten[idx] = static_score(d, self.score_sigma).astype(np.float32)
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = msg.ranges
        out.intensities = inten.tolist()
        self.scored_pub.publish(out)

    def _update_lost(self, quality, now) -> None:
        if quality < self.lost_ratio:
            if self._t_low is None:
                self._t_low = now
            elif (self._armed
                  and (now - self._t_low) > Duration(seconds=self.lost_hold)):
                self.lost_pub.publish(Empty())
                self._armed = False
                self.get_logger().warn(
                    'LOCALIZATION LOST (quality %.2f < %.2f) -- '
                    '/localization_lost' % (quality, self.lost_ratio))
        else:
            self._t_low = None
        if quality >= self.ok_ratio and not self._armed:
            self._armed = True
            self.get_logger().info(
                'localization recovered (quality %.2f) -- re-armed' % quality)

    def _maybe_log(self, quality, inlier, d, now) -> None:
        if (self._t_log is not None
                and (now - self._t_log) < Duration(seconds=self.log_period)):
            return
        self._t_log = now
        counts = self._histogram(d)
        peak = int(counts.max())
        width = self.hist_max / self.hist_bins
        lines = ['quality=%.2f  inliers=%d/%d  (match <= %.2fm)'
                 % (quality, int(inlier.sum()), inlier.size, self.match_dist)]
        for i, c in enumerate(counts):
            bar = '#' * int(round(20 * c / peak)) if peak > 0 else ''
            lines.append('  %.2f-%.2f |%-20s %d'
                         % (i * width, (i + 1) * width, bar, int(c)))
        self.get_logger().info('scan-match histogram:\n' + '\n'.join(lines))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationHealth()
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
