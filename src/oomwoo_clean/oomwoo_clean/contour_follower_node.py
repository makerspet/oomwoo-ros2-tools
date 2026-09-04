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
LiDAR it isolates the followed surface in a forward-biased sector on the follow
side (default right), FITS A LINE to it, and servos two errors -- the fitted
perpendicular distance and the bearing to it (want it abeam, -90 deg). That one
law handles straight walls and CONCAVE inside corners. CONVEX outside corners
get an explicit recovery: when the near
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

from geometry_msgs.msg import Point, Twist

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

from sensor_msgs.msg import LaserScan

from std_msgs.msg import Bool, Float32, String

from visualization_msgs.msg import Marker, MarkerArray

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
    'fit_gap_m': 0.10,             # max step between adjacent points on one surface
    'min_fit_points': 6,           # below this, fall back to the nearest beam
    'bearing_ref_deg': -90.0,      # want the nearest point abeam (right)
    'k_approach': 2.0,             # rad of approach angle per m of standoff error
    'alpha_max_deg': 40.0,         # cap on the approach angle (far-wall approach)
    'k_heading': 1.5,              # rad/s per rad of heading error
    'omega_max': 1.0,              # rad/s cap
    'slow_angle_deg': 45.0,        # |heading error| that eases v to the floor
    'publish_markers': True,       # ~/debug_markers for RViz
    'convex_jump_m': 0.30,         # d_min jump between frames that triggers ARC
    'convex_arc_radius_m': 0.30,   # ARC radius (~standoff + body offset)
    'convex_arc_max_deg': 200.0,   # ARC sweep with no re-acquire -> LOST
    'reacquire_margin_m': 0.15,    # ARC re-acquires when boundary <= standoff+this
    'align_tol_deg': 3.0,          # ALIGN done when |bearing error| below this
    'log_period_s': 1.0,           # throttled FOLLOW diagnostic line
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
        self.scan_frame = 'base_scan'
        self._dbg_d = None            # last nearest pick, for debug markers
        self._dbg_b = None
        self._t_log = None            # last diagnostic log time
        self._dbg_fit = None          # fitted segment endpoints, for markers

        latched = QoSProfile(
            depth=1, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.active_pub = self.create_publisher(Bool, 'cleaning_active', latched)
        self.state_pub = self.create_publisher(String, '~/state', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '~/debug_markers', 5)
        self.err_d_pub = self.create_publisher(Float32, '~/standoff_err_m', 10)
        self.err_b_pub = self.create_publisher(Float32, '~/bearing_err_deg', 10)
        self.err_h_pub = self.create_publisher(Float32, '~/heading_err_deg', 10)
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

    def _boundary(self, msg, smin, smax, max_r):
        """
        Fit the followed surface; return (perpendicular distance, bearing, n).

        Seeds on the nearest beam in the sector, grows the contiguous surface
        around it, then total-least-squares fits a line to those points and
        reports the perpendicular distance to that line and the bearing to it.

        Fitting rather than just taking the nearest beam matters. Near the
        perpendicular the range is almost flat -- at 0.2 m, swinging 20 deg
        changes it by 1.3 cm, while the beam-to-beam scatter is around 2 cm -- so
        the ARG-min (which beam is closest) is essentially random over a wide arc,
        and min() over noisy beams is a biased distance. The fit uses every point
        on the surface, so the noise averages down and the wall angle falls out
        directly instead of being inferred from a single beam.
        """
        count = len(msg.ranges)
        pts = [None] * count
        seed = None
        seed_r = None
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > max_r:
                continue
            b = self.side * math.remainder(
                msg.angle_min + i * msg.angle_increment, TWO_PI)
            if b < smin or b > smax:
                continue
            pts[i] = (r * math.cos(b), r * math.sin(b), r, b)
            if seed_r is None or r < seed_r:
                seed, seed_r = i, r
        if seed is None:
            self._dbg_d = self._dbg_b = self._dbg_fit = None
            return None, None, 0

        # grow the contiguous surface either way from the seed
        gap = self._p('fit_gap_m')
        keep = [seed]
        for step in (1, -1):
            j = seed
            while True:
                k = (j + step) % count
                if k == seed or pts[k] is None:
                    break
                if math.hypot(pts[k][0] - pts[j][0],
                              pts[k][1] - pts[j][1]) > gap:
                    break
                keep.append(k)
                j = k
        sel = [pts[k] for k in keep]

        if len(sel) < int(self._p('min_fit_points')):
            self._dbg_d, self._dbg_b = seed_r, pts[seed][3]
            self._dbg_fit = None
            return seed_r, pts[seed][3], len(sel)

        m = float(len(sel))
        cx = sum(p[0] for p in sel) / m
        cy = sum(p[1] for p in sel) / m
        sxx = sum((p[0] - cx) ** 2 for p in sel) / m
        syy = sum((p[1] - cy) ** 2 for p in sel) / m
        sxy = sum((p[0] - cx) * (p[1] - cy) for p in sel) / m
        theta = 0.5 * math.atan2(2.0 * sxy, sxx - syy)   # line direction
        nx, ny = -math.sin(theta), math.cos(theta)       # unit normal
        dist = cx * nx + cy * ny                         # signed, origin to line
        if dist < 0.0:
            nx, ny, dist = -nx, -ny, -dist

        ct, st = math.cos(theta), math.sin(theta)
        ts = [(p[0] - cx) * ct + (p[1] - cy) * st for p in sel]
        self._dbg_fit = ((cx + min(ts) * ct, cy + min(ts) * st),
                         (cx + max(ts) * ct, cy + max(ts) * st))
        self._dbg_d, self._dbg_b = dist, math.atan2(ny, nx)
        return dist, math.atan2(ny, nx), len(sel)

    def _on_scan(self, msg: LaserScan) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.0 if self.prev_t is None else max(0.0, t - self.prev_t)
        self.prev_t = t
        if not self.enabled or self.state in ('IDLE', 'LOST'):
            self._set_cmd(0.0, 0.0)
            return

        self.scan_frame = msg.header.frame_id
        b_ref = math.radians(self._p('bearing_ref_deg'))
        smin = math.radians(self._p('sector_min_deg'))
        smax = math.radians(self._p('sector_max_deg'))
        max_r = self._p('max_follow_range_m')
        self._dbg_d = None
        self._dbg_b = None
        self._step(msg, b_ref, smin, smax, max_r, dt)
        self._pub_markers(b_ref, smin, smax)

    def _step(self, msg, b_ref, smin, smax, max_r, dt) -> None:

        if self.state == 'ALIGN':
            self._align(msg, b_ref, max_r)
            return

        if self.state == 'FOLLOW':
            d, b, _ = self._boundary(msg, smin, smax, max_r)
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
        e_b = b - b_ref                  # + = currently angled toward the wall
        # Outer loop: how far to angle toward/away, CAPPED. Without the cap a far
        # wall demands a saturated turn that the heading term cancels, and the robot
        # crawls in at the speed floor instead of approaching cleanly.
        a_max = math.radians(self._p('alpha_max_deg'))
        alpha = _clamp(self._p('k_approach') * e_d, -a_max, a_max)
        # Inner loop: steer the actual angle onto the desired one.
        e_h = alpha - e_b
        omega = _clamp(-self._p('k_heading') * e_h,
                       -self._p('omega_max'), self._p('omega_max'))
        # Ease off only when the INNER loop is far off (a real corner); a steady
        # approach has e_h ~ 0, so it runs at full speed.
        slow = math.radians(self._p('slow_angle_deg'))
        v = self._p('v_nominal') * (1.0 - min(1.0, abs(e_h) / max(slow, 1e-3)))
        v = max(self._p('v_min'), v)
        self._set_cmd(v, self.side * omega)
        self._pub_errors(e_d, e_b, e_h)
        self._maybe_log(d, e_d, e_b, alpha, e_h, v, omega)

    def _arc(self, msg, smin, smax, max_r, b_ref, dt) -> None:
        v = max(self._p('v_min'), 0.5 * self._p('v_nominal'))
        omega = -v / max(self._p('convex_arc_radius_m'), 1e-3)  # toward follow side
        self.arc_swept += abs(omega) * dt
        d, b, _ = self._boundary(msg, smin, smax, max_r)
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
        d, b, _ = self._boundary(msg, -math.pi, math.pi, max_r)
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

    def _mk(self, mid, mtype, stamp):
        m = Marker()
        m.header.frame_id = self.scan_frame
        m.header.stamp = stamp
        m.ns = 'contour_follower'
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.lifetime.sec = 1
        return m

    def _pt(self, r, b, z=0.0):
        p = Point()
        p.x = r * math.cos(b)
        p.y = r * math.sin(b)
        p.z = z
        return p

    def _pub_markers(self, b_ref, smin, smax) -> None:
        """
        Draw the pick, the standoff target and the sector, for tuning.

        Everything is in the scan frame, so the raw (un-mirrored) bearing is
        side * the follow-side bearing the controller works in.
        """
        if not self._p('publish_markers'):
            return
        stamp = self.get_clock().now().to_msg()
        s = self.side
        arr = MarkerArray()
        if self._dbg_d is not None:
            hit = self._mk(0, Marker.SPHERE, stamp)
            hit.scale.x = hit.scale.y = hit.scale.z = 0.06
            hit.color.g = 1.0
            hit.color.a = 1.0
            hit.pose.position = self._pt(self._dbg_d, s * self._dbg_b)
            arr.markers.append(hit)
            ray = self._mk(1, Marker.LINE_LIST, stamp)
            ray.scale.x = 0.01
            ray.color.g = 1.0
            ray.color.a = 0.8
            ray.points = [self._pt(0.0, 0.0),
                          self._pt(self._dbg_d, s * self._dbg_b)]
            arr.markers.append(ray)
        tgt = self._mk(2, Marker.SPHERE, stamp)
        tgt.scale.x = tgt.scale.y = tgt.scale.z = 0.05
        tgt.color.r = 0.2
        tgt.color.b = 1.0
        tgt.color.a = 0.9
        tgt.pose.position = self._pt(self._p('standoff_m'), s * b_ref)
        arr.markers.append(tgt)
        if self._dbg_fit is not None:
            fit = self._mk(5, Marker.LINE_LIST, stamp)
            fit.scale.x = 0.012
            fit.color.r = 1.0
            fit.color.b = 1.0
            fit.color.a = 0.9
            for px, py in self._dbg_fit:
                p = Point()
                p.x, p.y, p.z = px, s * py, 0.0
                fit.points.append(p)
            arr.markers.append(fit)
        sec = self._mk(3, Marker.LINE_LIST, stamp)
        sec.scale.x = 0.006
        sec.color.r = 1.0
        sec.color.g = 0.6
        sec.color.a = 0.5
        rng = self._p('max_follow_range_m')
        for edge in (smin, smax):
            sec.points.append(self._pt(0.0, 0.0))
            sec.points.append(self._pt(rng, s * edge))
        arr.markers.append(sec)
        txt = self._mk(4, Marker.TEXT_VIEW_FACING, stamp)
        txt.scale.z = 0.07
        txt.color.r = txt.color.g = txt.color.b = 1.0
        txt.color.a = 0.9
        txt.pose.position = self._pt(0.0, 0.0, 0.35)
        shown = '--' if self._dbg_d is None else '%.2f' % self._dbg_d
        txt.text = '%s  d=%s  target=%.2f' % (
            self.state, shown, self._p('standoff_m'))
        arr.markers.append(txt)
        self.marker_pub.publish(arr)

    def _maybe_log(self, d, e_d, e_b, alpha, e_h, v, omega) -> None:
        """
        Throttled one-liner of what the controller is actually steering on.

        Reads: how far the picked boundary is vs the target, how far the robot is
        currently angled toward it, how far we WANT it angled (the capped approach
        angle), and the resulting command. If "toward" tracks "want", the loop is
        doing its job and any residual angle is just the approach in progress.
        """
        now = self.get_clock().now()
        period = Duration(seconds=float(self._p('log_period_s')))
        if self._t_log is not None and (now - self._t_log) < period:
            return
        self._t_log = now
        self.get_logger().info(
            '%-6s d=%.2fm (target %.2f, err %+.2f)  toward=%+5.1f deg  '
            'want=%+5.1f  err=%+5.1f  ->  v=%.2f w=%+.2f'
            % (self.state, d, self._p('standoff_m'), e_d,
               math.degrees(e_b), math.degrees(alpha), math.degrees(e_h),
               v, omega))

    def _pub_errors(self, e_d, e_b, e_h) -> None:
        self.err_d_pub.publish(Float32(data=float(e_d)))
        self.err_b_pub.publish(Float32(data=float(math.degrees(e_b))))
        self.err_h_pub.publish(Float32(data=float(math.degrees(e_h))))


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
