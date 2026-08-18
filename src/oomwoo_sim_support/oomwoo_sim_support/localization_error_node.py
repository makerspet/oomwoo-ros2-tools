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
Continuous localization-error meter: estimated map->base vs ground truth.

Measures how far the *localizer's* estimate drifts from the true pose while the
robot drives -- the number behind the "LiDAR scan and map walls stay apart"
misregistration. It is localizer-agnostic: it reads the estimate from the TF
map->base_footprint (published by AMCL, or by slam_toolbox in localization
mode), so the same meter scores either one and makes an honest A/B possible.

Set estimate_topic to read the estimate from a PoseWithCovarianceStamped topic
(e.g. /amcl_pose) instead of the TF. That lets a second instance measure a
localizer that is NOT the one owning map->odom -- so AMCL (via /amcl_pose, with
tf_broadcast:false) and slam_toolbox (via TF) can be scored side by side.

Truth comes from oomwoo_sim_support/ground_truth (`/ground_truth/pose`), which
is only real ground truth when `/odom` is the sim's noise-free odometry -- so
run this with odom_source:=ground_truth. With robot_wheels, `/odom` drifts and
comparison conflates odom drift with localizer error.

Each tick it logs a machine-parseable line and publishes the two errors so you
can rqt_plot or echo them:

  LOC_ERR pos=<m> yaw=<deg> | <W>s-rms pos=<m> yaw=<deg> max_pos=<m> (n=<N>)

  sub  /ground_truth/pose   geometry_msgs/PoseStamped   (map frame)
  pub  ~/pos_err_m          std_msgs/Float32
  pub  ~/yaw_err_deg        std_msgs/Float32
"""

from collections import deque
import math

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Float32

from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quat(q) -> float:
    # planar (z) yaw from a quaternion
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a) -> float:
    # fold an angle into [-pi, pi]
    return math.atan2(math.sin(a), math.cos(a))


class LocalizationError(Node):

    def __init__(self) -> None:
        super().__init__('localization_error')
        self.target_frame = self.declare_parameter('target_frame', 'map').value
        self.base_frame = self.declare_parameter(
            'base_frame', 'base_footprint').value
        self.truth_topic = self.declare_parameter(
            'truth_topic', '/ground_truth/pose').value
        rate = self.declare_parameter('rate', 5.0).value
        self.window = self.declare_parameter('window_s', 20.0).value
        self.log_period = self.declare_parameter('log_period_s', 2.0).value
        self.est_topic = self.declare_parameter('estimate_topic', '').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.truth = None            # (x, y, yaw) in map
        self.est = None              # (x, y, yaw) from estimate_topic, if used
        self._samples = deque()      # (stamp, pos_err_m, yaw_err_deg)
        self._last_log = self.get_clock().now()

        self.pos_pub = self.create_publisher(Float32, '~/pos_err_m', 10)
        self.yaw_pub = self.create_publisher(Float32, '~/yaw_err_deg', 10)
        self.create_subscription(
            PoseStamped, self.truth_topic, self._on_truth, 10)
        if self.est_topic:
            self.create_subscription(
                PoseWithCovarianceStamped, self.est_topic, self._on_est, 10)
        self.create_timer(1.0 / max(rate, 1e-3), self._tick)
        source = self.est_topic if self.est_topic else '%s->%s TF' % (
            self.target_frame, self.base_frame)
        self.get_logger().info(
            'localization_error: estimate=[%s] vs %s (needs ground_truth odom)'
            % (source, self.truth_topic))

    def _on_truth(self, msg: PoseStamped) -> None:
        self.truth = (msg.pose.position.x, msg.pose.position.y,
                      yaw_from_quat(msg.pose.orientation))

    def _on_est(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        self.est = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def _estimate(self):
        # localizer estimate: from the pose topic if configured, else the TF
        if self.est_topic:
            return self.est
        try:
            tr = self.tf_buffer.lookup_transform(
                self.target_frame, self.base_frame, Time())
        except TransformException as err:
            self.get_logger().warn(
                'localization_error: TF lookup failed: %s' % err,
                throttle_duration_sec=5.0)
            return None
        return (tr.transform.translation.x, tr.transform.translation.y,
                yaw_from_quat(tr.transform.rotation))

    def _tick(self) -> None:
        if self.truth is None:
            return
        est = self._estimate()
        if est is None:
            return
        ex, ey, eyaw = est
        tx, ty, tyaw = self.truth
        pos_err = math.hypot(ex - tx, ey - ty)
        yaw_err = math.degrees(abs(wrap(eyaw - tyaw)))

        now = self.get_clock().now()
        self._samples.append((now, pos_err, yaw_err))
        # prune by AGE (now - sample >= 0) rather than (now - window), which
        # goes negative -- and rclpy raises -- while sim time < window.
        window = Duration(seconds=self.window)
        while self._samples and (now - self._samples[0][0]) > window:
            self._samples.popleft()
        self.pos_pub.publish(Float32(data=float(pos_err)))
        self.yaw_pub.publish(Float32(data=float(yaw_err)))
        if (now - self._last_log) >= Duration(seconds=self.log_period):
            self._last_log = now
            self._log_stats(pos_err, yaw_err)

    def _log_stats(self, pos_err, yaw_err) -> None:
        ps = [s[1] for s in self._samples]
        ys = [s[2] for s in self._samples]
        rms_p = math.sqrt(sum(p * p for p in ps) / len(ps)) if ps else 0.0
        rms_y = math.sqrt(sum(y * y for y in ys) / len(ys)) if ys else 0.0
        self.get_logger().info(
            'LOC_ERR pos=%.3fm yaw=%.2fdeg | %.0fs-rms pos=%.3fm yaw=%.2fdeg '
            'max_pos=%.3fm (n=%d)'
            % (pos_err, yaw_err, self.window, rms_p, rms_y,
               max(ps) if ps else 0.0, len(ps)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationError()
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
