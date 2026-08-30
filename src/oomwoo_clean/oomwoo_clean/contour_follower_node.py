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
Reactive LiDAR contour follower: trace an obstacle's boundary at a fixed standoff.

The proactive, any-shape generalization of the bumper-based wall_clean. Off the
LiDAR it finds the NEAREST boundary point in a forward-biased sector on the follow
side (default right) and servos two errors -- standoff distance and the point's
bearing (want it abeam, -90 deg). That one law handles straight walls and CONCAVE
inside corners. CONVEX outside corners get an explicit recovery: when the near
boundary vanishes (range jumps, or nothing left in the sector) the follower stops
trusting the far reading and ARCS toward the follow side at ~standoff radius until
it re-acquires -- "lose the wall, curve toward it". Left-follow is the mirror
(the scan bearings and the output omega are both negated).

Phase 1: FOLLOW + convex ARC, with a rotate-in-place ALIGN entry. No loop-closure
yet -- it runs until stopped (like wall_clean). See docs/contour_follower_spec.md.

  subscribes  scan             sensor_msgs/LaserScan   (SensorData QoS)
  subscribes  ~/enable         std_msgs/Bool           (runtime stop/go)
  publishes   cmd_vel          geometry_msgs/Twist
  publishes   cleaning_active  std_msgs/Bool           (latched; True while active)
  publishes   ~/state          std_msgs/String         (ALIGN/FOLLOW/ARC/LOST)
"""

import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from sensor_msgs.msg import LaserScan

from std_msgs.msg import Bool, String

TWO_PI = 2.0 * math.pi
ACTIVE_STATES = ('ALIGN', 'FOLLOW', 'ARC')

DEFAULTS = {
    'follow_side': 'right',        # 'right' or 'left'
    'standoff_m': 0.20,            # perpendicular LiDAR-to-boundary target
    'v_nominal': 0.15,             # m/s cruise
    'v_min': 0.05,                 # m/s floor (in corners)
    'sector_min_deg': -170.0,      # follow-side + forward window (right-follow)
    'sector_max_deg': 20.0,
    'max_follow_range_m': 1.0,     # ignore boundaries farther than this
    'bearing_ref_deg': -90.0,      # want the nearest point abeam (right)
    'k_dist': 1.5,                 # rad/s per m of standoff error
    'k_bearing': 1.2,              # rad/s per rad of bearing error
    'omega_max': 1.0,              # rad/s cap
    'slow_angle_deg': 60.0,        # |bearing error| that eases v to the floor
    'convex_jump_m': 0.30,         # d_min jump between frames that triggers ARC
    'convex_arc_radius_m': 0.30,   # ARC radius (~standoff + body offset)
    'convex_arc_max_deg': 200.0,   # ARC sweep with no re-acquire -> LOST
    'reacquire_margin_m': 0.15,    # ARC re-acquires when boundary <= standoff+this
    'align_tol_deg': 10.0,         # ALIGN done when |bearing error| below this
    'k_align': 1.0,                # rad/s per rad, ALIGN rotation
    'align_omega': 0.5,            # rad/s cap for ALIGN rotation
    'pub_hz': 20.0,                # cmd_vel republish rate (control runs on scan)
    'auto_start': True,            # begin ALIGN on launch
}


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


class ContourFollower(Node):
    """Follow a LiDAR-visible obstacle boundary at a fixed standoff."""

    def __init__(self) -> None:
        super().__init__('contour_follower')
        for name, default in DEFAULTS.items():
            self.declare_parameter(name, default)

        self.state = 'IDLE'
        self.enabled = bool(self._p('auto_start'))
        self.cmd = Twist()
        self.prev_t = None            # last scan sim-time, for ARC sweep dt
        self.prev_d = None            # last FOLLOW d_min, for the convex jump
        self.arc_swept = 0.0          # rad swept in the current ARC
        self._active_val = None       # last cleaning_active value published

        latched = QoSProfile(
            depth=1, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.active_pub = self.create_publisher(Bool, 'cleaning_active', latched)
        self.state_pub = self.create_publisher(String, '~/state', 10)
        self.create_subscription(
            LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Bool, '~/enable', self._on_enable, 10)
        self.create_timer(1.0 / max(self._p('pub_hz'), 1.0), self._pub_cmd)

        self.side = -1.0 if self._p('follow_side') == 'left' else 1.0
        self._set_state('ALIGN' if self.enabled else 'IDLE')
        self.get_logger().info(
            'contour_follower: follow %s, standoff %.2fm '
            '(nearest-point + convex arc; no loop-closure yet)'
            % (self._p('follow_side'), self._p('standoff_m')))

    def _p(self, name):
        return self.get_parameter(name).value

    def _pub_cmd(self) -> None:
        self.cmd_pub.publish(self.cmd)

    def _set_cmd(self, v, w) -> None:
        self.cmd.linear.x = float(v)
        self.cmd.angular.z = float(w)

    def _set_state(self, s) -> None:
        if s == self.state:
            return
        self.state = s
        self.state_pub.publish(String(data=s))
        active = s in ACTIVE_STATES
        if active != self._active_val:
            self._active_val = active
            self.active_pub.publish(Bool(data=active))
        self.get_logger().info('state -> %s' % s)

    def _on_enable(self, msg: Bool) -> None:
        self.enabled = bool(msg.data)
        if not self.enabled:
            self._set_cmd(0.0, 0.0)
            self._set_state('IDLE')
        elif self.state in ('IDLE', 'LOST'):
            self.prev_d = None
            self.arc_swept = 0.0
            self._set_state('ALIGN')

    def _nearest(self, msg, smin, smax, max_r):
        """Nearest valid boundary (d, bearing) in [smin, smax], follow-side frame."""
        best_d = None
        best_b = None
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > max_r:
                continue
            b = self.side * math.remainder(
                msg.angle_min + i * msg.angle_increment, TWO_PI)
            if b < smin or b > smax or (best_d is not None and r >= best_d):
                continue
            best_d = r
            best_b = b
        return best_d, best_b

    def _on_scan(self, msg: LaserScan) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.0 if self.prev_t is None else max(0.0, t - self.prev_t)
        self.prev_t = t
        if not self.enabled or self.state in ('IDLE', 'LOST'):
            self._set_cmd(0.0, 0.0)
            return

        b_ref = math.radians(self._p('bearing_ref_deg'))
        smin = math.radians(self._p('sector_min_deg'))
        smax = math.radians(self._p('sector_max_deg'))
        max_r = self._p('max_follow_range_m')

        if self.state == 'ALIGN':
            self._align(msg, b_ref, max_r)
            return

        if self.state == 'FOLLOW':
            d, b = self._nearest(msg, smin, smax, max_r)
            jumped = (self.prev_d is not None and d is not None
                      and (d - self.prev_d) > self._p('convex_jump_m'))
            if d is None or jumped:
                self.arc_swept = 0.0
                self._set_state('ARC')
            else:
                self.prev_d = d
                self._follow(d, b, b_ref)
                return

        if self.state == 'ARC':
            self._arc(msg, smin, smax, max_r, b_ref, dt)

    def _follow(self, d, b, b_ref) -> None:
        e_d = d - self._p('standoff_m')
        e_b = b - b_ref
        # too far -> turn toward the wall (-); nearest point behind abeam -> turn
        # toward it (-); ahead of abeam / closing (concave) -> turn away (+).
        omega = -self._p('k_dist') * e_d + self._p('k_bearing') * e_b
        omega = _clamp(omega, -self._p('omega_max'), self._p('omega_max'))
        slow = math.radians(self._p('slow_angle_deg'))
        v = self._p('v_nominal') * (1.0 - min(1.0, abs(e_b) / max(slow, 1e-3)))
        v = max(self._p('v_min'), v)
        self._set_cmd(v, self.side * omega)

    def _arc(self, msg, smin, smax, max_r, b_ref, dt) -> None:
        v = max(self._p('v_min'), 0.5 * self._p('v_nominal'))
        omega = -v / max(self._p('convex_arc_radius_m'), 1e-3)  # toward follow side
        self.arc_swept += abs(omega) * dt
        d, b = self._nearest(msg, smin, smax, max_r)
        if d is not None and d <= self._p('standoff_m') + self._p('reacquire_margin_m'):
            self.prev_d = d
            self._set_state('FOLLOW')
            self._follow(d, b, b_ref)
            return
        if math.degrees(self.arc_swept) > self._p('convex_arc_max_deg'):
            self.get_logger().warn('convex arc found no boundary -- LOST')
            self._set_cmd(0.0, 0.0)
            self._set_state('LOST')
            return
        self._set_cmd(v, self.side * omega)

    def _align(self, msg, b_ref, max_r) -> None:
        # rotate in place to bring the nearest obstacle abeam on the follow side
        d, b = self._nearest(msg, -math.pi, math.pi, max_r)
        if d is None:
            self._set_cmd(0.0, self.side * self._p('align_omega'))   # search
            return
        e = b - b_ref
        if abs(e) < math.radians(self._p('align_tol_deg')):
            self.prev_d = None
            self._set_state('FOLLOW')
            self._set_cmd(0.0, 0.0)
            return
        omega = _clamp(
            self._p('k_align') * e, -self._p('align_omega'), self._p('align_omega'))
        self._set_cmd(0.0, self.side * omega)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ContourFollower()
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
