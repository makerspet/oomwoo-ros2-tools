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
Diff wheel odometry against ground truth to see wheel slip live.

Pairs two ``nav_msgs/Odometry`` streams by header stamp -- in ``robot_wheels``
mode ``/odom`` is the drifting wheel odometry and ``/odom_truth`` is the sim's
noise-free pose -- converts each quaternion to a yaw angle, and reports how the
wheel odometry diverges from the true rotation. Slip appears when the wheels
turn differently than the body: the wheels OVER-read on accelerating wheelspin
and UNDER-read as the body coasts past a stopped wheel on an abrupt stop. That
per-cycle net over-read is what accumulates into raw wheel-odom heading drift,
the drift slam_toolbox quietly corrects against the map every scan.

``/odom`` and ``/odom_truth`` are NOT the same frame, so the raw yaw difference
is dominated by a constant frame offset (plus accumulated drift). The slip is in
the *changes*, so the useful signals are de-trended from per-sample wrapped
deltas, which cancel the constant offset::

    slip_rate = d(wheel_yaw - truth_yaw)/dt  ~= omega_wheel - omega_truth

It reads ~0 while the wheels roll true, spikes + on wheelspin, and spikes - on
the inertial coast at a stop.

Published (``std_msgs/Float32``, full rate -- plot these in Foxglove/rqt):

  pub  ~/slip_rate_dps   the slip detector: deg/s, ~0 when rolling true
  pub  ~/slip_deg        accumulated slip since start (de-trended), 0-based
  pub  ~/yaw_error_deg   raw wheel_yaw - truth_yaw (frame-offset dominated)
  pub  ~/pos_error_mm    ||wheel_xy - truth_xy|| (also frame-offset dominated)
  pub  ~/wheel_yaw_deg   overlay these two to SEE the wheels lag the body
  pub  ~/truth_yaw_deg

Run (with odom_source:=robot_wheels, so /odom is the wheel odometry)::

  ros2 run oomwoo_sim_support odom_slip --ros-args -p use_sim_time:=true

In ground_truth mode the wheel odometry is on /odom_wheel, so pass
``-p wheel_topic:=/odom_wheel``.
"""

import math

from message_filters import ApproximateTimeSynchronizer, Subscriber

from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32


def yaw_from_quat(q):
    """Return the yaw (rad) of a geometry_msgs/Quaternion, planar-safe."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap180(deg):
    """Wrap an angle already in degrees to (-180, 180]."""
    return (deg + 180.0) % 360.0 - 180.0


class OdomSlip(Node):
    """Pair wheel/truth odom by stamp and report the slip between them."""

    def __init__(self):
        super().__init__('odom_slip')
        wheel = self.declare_parameter('wheel_topic', '/odom').value
        truth = self.declare_parameter('truth_topic', '/odom_truth').value
        # flag a console line when the slip RATE exceeds this (deg/s); the raw
        # yaw difference is swamped by the frame offset, so rate is the signal.
        self.flag_dps = self.declare_parameter('slip_rate_flag_dps', 20.0).value
        self.print_period = self.declare_parameter('print_period_s', 0.5).value

        sub_w = Subscriber(self, Odometry, wheel)
        sub_t = Subscriber(self, Odometry, truth)
        # slop 20 ms: tight enough to compare the same instant, loose enough to
        # pair a ~25 Hz truth with a faster wheel stream.
        self.sync = ApproximateTimeSynchronizer([sub_w, sub_t], queue_size=50,
                                                slop=0.02)
        self.sync.registerCallback(self.on_pair)

        self.pub_rate = self.create_publisher(Float32, '~/slip_rate_dps', 10)
        self.pub_slip = self.create_publisher(Float32, '~/slip_deg', 10)
        self.pub_dyaw = self.create_publisher(Float32, '~/yaw_error_deg', 10)
        self.pub_dpos = self.create_publisher(Float32, '~/pos_error_mm', 10)
        self.pub_wyaw = self.create_publisher(Float32, '~/wheel_yaw_deg', 10)
        self.pub_tyaw = self.create_publisher(Float32, '~/truth_yaw_deg', 10)

        self.prev = None      # (t, wheel_yaw_deg, truth_yaw_deg)
        self.accum = 0.0      # de-trended accumulated slip, deg
        self.peak_rate = 0.0
        self.peak_at = 0.0
        self.last_print = None
        self.get_logger().info(
            f'slip: {wheel} (wheel) vs {truth} (truth); '
            f'flagging |slip_rate| >= {self.flag_dps} deg/s')

    def on_pair(self, wheel, truth):
        """Publish and log the slip for one time-matched odom pair."""
        t = wheel.header.stamp.sec + wheel.header.stamp.nanosec * 1e-9
        yw = math.degrees(yaw_from_quat(wheel.pose.pose.orientation))
        yt = math.degrees(yaw_from_quat(truth.pose.pose.orientation))
        dyaw = wrap180(yw - yt)
        dx = wheel.pose.pose.position.x - truth.pose.pose.position.x
        dy = wheel.pose.pose.position.y - truth.pose.pose.position.y
        dpos = math.hypot(dx, dy)

        # Slip rate from per-sample wrapped deltas: (dwheel - dtruth)/dt. The
        # constant frame offset cancels, so this is 0 while rolling true and
        # spikes on wheelspin / inertial coast. Accumulate it (de-trended slip).
        rate = 0.0
        if self.prev is not None:
            pt, pyw, pyt = self.prev
            dt = t - pt
            if dt > 1e-6:
                dslip = wrap180(yw - pyw) - wrap180(yt - pyt)
                self.accum += dslip
                rate = dslip / dt
        self.prev = (t, yw, yt)

        # full rate, so the trace is smooth and catches the 1-frame stop flick
        self.pub_rate.publish(Float32(data=float(rate)))
        self.pub_slip.publish(Float32(data=float(self.accum)))
        self.pub_dyaw.publish(Float32(data=float(dyaw)))
        self.pub_dpos.publish(Float32(data=float(dpos * 1000.0)))
        self.pub_wyaw.publish(Float32(data=float(yw)))
        self.pub_tyaw.publish(Float32(data=float(yt)))

        if abs(rate) > self.peak_rate:
            self.peak_rate = abs(rate)
            self.peak_at = t

        flag = abs(rate) >= self.flag_dps
        due = (self.last_print is None
               or t - self.last_print >= self.print_period)
        if flag or due:
            self.last_print = t
            tag = '  <-- SLIP' if flag else ''
            self.get_logger().info(
                f't={t:9.3f}  wheel={yw:7.2f}  truth={yt:7.2f}  '
                f'slip_rate={rate:+7.1f} deg/s  accum={self.accum:+6.2f} deg  '
                f'dpos={dpos * 1000:5.1f} mm{tag}')


def main():
    """Spin the slip meter until interrupted, then print a summary."""
    rclpy.init()
    node = OdomSlip()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'peak |slip_rate| = {node.peak_rate:.1f} deg/s '
            f'at t={node.peak_at:.3f}; '
            f'net accumulated slip = {node.accum:+.2f} deg')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
