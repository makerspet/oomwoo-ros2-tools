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
Placeholder perception: spot dynamic obstacles in the scored scan.

A STARTING POINT for perception/ML contributions -- not a finished detector. It
subscribes to localization_health's ~/scan_scored (a full LaserScan whose
intensity is each ray's static-ness: 1.0 = on a mapped wall, ->0 = a dynamic
return), thresholds the dynamic rays, groups the contiguous ones into blobs, and
publishes their centroids on ~/objects for RViz.

That is deliberately the whole of it. The interesting work is what you add on
top: classify the blobs (a foot vs a pet vs a ball vs a moved stool), track them
across frames, and turn patterns into intent -- e.g. a foot tapped twice as a
"come clean here" gesture. The scored scan is left un-clustered on purpose so
this node -- not the health monitor -- owns segmentation and recognition, and it
publishes the dynamic points themselves on ~/dynamic_points for RViz.

  sub  <scored_scan>   sensor_msgs/LaserScan   (default /localization_health/scan_scored)
  pub  ~/objects       visualization_msgs/MarkerArray  (a sphere per dynamic blob)
  pub  ~/dynamic_points sensor_msgs/PointCloud2        (the flagged dynamic returns)
"""

import math

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, PointCloud2, PointField

from visualization_msgs.msg import Marker, MarkerArray


def cluster_dynamic(dynamic, xs, ys, gap, min_pts):
    """Group contiguous dynamic rays into (cx, cy, count) blobs."""
    clusters = []
    n = dynamic.size
    i = 0
    while i < n:
        if not dynamic[i]:
            i += 1
            continue
        j = i + 1
        while (j < n and dynamic[j]
               and math.hypot(xs[j] - xs[j - 1], ys[j] - ys[j - 1]) <= gap):
            j += 1
        if (j - i) >= min_pts:
            clusters.append((float(xs[i:j].mean()), float(ys[i:j].mean()),
                             int(j - i)))
        i = j
    return clusters


class DynamicObjectDetector(Node):

    def __init__(self) -> None:
        super().__init__('dynamic_object_detector')
        topic = self.declare_parameter(
            'scored_scan_topic', '/localization_health/scan_scored').value
        self.thresh = self.declare_parameter('dynamic_threshold', 0.5).value
        self.gap = self.declare_parameter('cluster_gap_m', 0.20).value
        self.min_pts = int(self.declare_parameter('min_cluster_beams', 3).value)
        self._last_n = -1
        self.create_subscription(
            LaserScan, topic, self._on_scan, qos_profile_sensor_data)
        self.obj_pub = self.create_publisher(MarkerArray, '~/objects', 5)
        self.pts_pub = self.create_publisher(
            PointCloud2, '~/dynamic_points', 5)
        self.get_logger().info(
            'dynamic_object_detector: %s, intensity < %.2f -> dynamic blobs on '
            '~/objects (a placeholder -- add recognition here)'
            % (topic, self.thresh))

    def _on_scan(self, msg: LaserScan) -> None:
        n = len(msg.ranges)
        if n == 0:
            return
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        if len(msg.intensities) == n:
            inten = np.asarray(msg.intensities, dtype=np.float64)
        else:
            inten = np.ones(n)                      # no scores -> nothing dynamic
        angles = msg.angle_min + np.arange(n) * msg.angle_increment
        valid = (np.isfinite(ranges) & (ranges >= msg.range_min)
                 & (ranges <= msg.range_max))
        dynamic = valid & (inten < self.thresh)     # low static-ness = dynamic
        xs, ys = ranges * np.cos(angles), ranges * np.sin(angles)
        blobs = cluster_dynamic(dynamic, xs, ys, self.gap, self.min_pts)
        self.obj_pub.publish(self._markers(msg.header, blobs))
        # the flagged dynamic returns themselves (empty cloud clears RViz)
        self.pts_pub.publish(self._cloud(msg.header, xs[dynamic], ys[dynamic]))
        if len(blobs) != self._last_n:
            self._last_n = len(blobs)
            self.get_logger().info('tracking %d dynamic object(s)' % len(blobs))

    def _markers(self, header, blobs) -> MarkerArray:
        arr = MarkerArray()
        clear = Marker()
        clear.header = header
        clear.ns = 'dynamic_objects'
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for k, (cx, cy, _cnt) in enumerate(blobs):
            m = Marker()
            m.header = header
            m.ns = 'dynamic_objects'
            m.id = k
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.15
            m.color.r = 1.0
            m.color.g = 0.4
            m.color.a = 0.8
            arr.markers.append(m)
        return arr

    def _cloud(self, header, xs, ys) -> PointCloud2:
        n = xs.size
        buf = np.zeros((n, 3), dtype=np.float32)
        buf[:, 0] = xs
        buf[:, 1] = ys
        msg = PointCloud2()
        msg.header = header                          # scan frame
        msg.height = 1
        msg.width = n
        msg.fields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * n
        msg.is_dense = True
        msg.data = buf.tobytes()
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DynamicObjectDetector()
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
