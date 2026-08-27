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
Inject a large, off-map obstacle into a LaserScan for localization stress tests.

Overwrites a contiguous angular arc of the scan with a near return -- a solid
object the map does not know about -- and republishes it. Because it corrupts a
big, coherent chunk of the scan (not scattered noise), it is exactly the kind of
occlusion that can pull a scan-matcher off. Deterministic and repeatable, so it
makes a clean A/B: point slam_toolbox at this output to measure the damage, then
at localization_health's /scan_filtered (fed from this output) to see whether
stripping the obstacle recovers the pose.

The obstacle spans ``width_deg`` centred at ``center_deg`` in the scan frame, at
``range_m`` (only where that is nearer than the real return -- an obstacle
occludes, it cannot add far returns). With ``sweep`` it pans back and forth
across [sweep_min_deg, sweep_max_deg] at ``sweep_speed_dps`` so it occludes
different walls over time.

  sub  <input_topic>   sensor_msgs/LaserScan   (default /scan)
  pub  <output_topic>  sensor_msgs/LaserScan   (default /scan_stress)
"""

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan


def wrap_pi(a):
    """Wrap angles (rad, array) to (-pi, pi]."""
    return (a + np.pi) % (2.0 * np.pi) - np.pi


class ScanObstacleInjector(Node):
    """Republish a scan with a large fake obstacle painted into an arc."""

    def __init__(self) -> None:
        super().__init__('scan_obstacle_injector')
        in_topic = self.declare_parameter('input_topic', '/scan').value
        out_topic = self.declare_parameter('output_topic', '/scan_stress').value
        self.enable = self.declare_parameter('enable', True).value
        self.width = np.radians(
            self.declare_parameter('width_deg', 60.0).value)
        self.range_m = self.declare_parameter('range_m', 0.6).value
        self.center = np.radians(
            self.declare_parameter('center_deg', 0.0).value)
        self.sweep = self.declare_parameter('sweep', False).value
        self.sweep_speed = np.radians(
            self.declare_parameter('sweep_speed_dps', 20.0).value)
        self.sweep_lo = np.radians(
            self.declare_parameter('sweep_min_deg', -90.0).value)
        self.sweep_hi = np.radians(
            self.declare_parameter('sweep_max_deg', 90.0).value)

        self._dir = 1.0
        self._t_prev = None
        self.create_subscription(
            LaserScan, in_topic, self._on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(
            LaserScan, out_topic, qos_profile_sensor_data)
        self.get_logger().info(
            'scan_obstacle_injector: %s -> %s, %.0fdeg wide at %.2fm%s'
            % (in_topic, out_topic, np.degrees(self.width), self.range_m,
               ' (sweeping)' if self.sweep else ''))

    def _on_scan(self, msg: LaserScan) -> None:
        if not self.enable:
            self.pub.publish(msg)
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.sweep:
            self._advance(t)
        n = len(msg.ranges)
        angles = msg.angle_min + np.arange(n) * msg.angle_increment
        hit = np.abs(wrap_pi(angles - self.center)) <= (self.width / 2.0)
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        # an obstacle only makes a beam nearer; keep NaN/inf beams if it is nearer
        near = ~np.isfinite(ranges) | (self.range_m < ranges)
        ranges[hit & near] = self.range_m
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = ranges.astype(np.float32).tolist()
        out.intensities = msg.intensities
        self.pub.publish(out)

    def _advance(self, t) -> None:
        if self._t_prev is None:
            self._t_prev = t
            return
        dt = t - self._t_prev
        self._t_prev = t
        if dt <= 0.0:
            return
        self.center += self._dir * self.sweep_speed * dt
        if self.center >= self.sweep_hi:
            self.center, self._dir = self.sweep_hi, -1.0
        elif self.center <= self.sweep_lo:
            self.center, self._dir = self.sweep_lo, 1.0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanObstacleInjector()
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
