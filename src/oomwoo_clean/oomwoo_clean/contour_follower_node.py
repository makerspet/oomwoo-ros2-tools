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
side (default right), FITS A CIRCLE to a short window of it, and servos two
errors -- the distance to the fitted curve and the bearing of its nearest point
(want it abeam, -90 deg).

Fitting a curve rather than trusting the single nearest beam takes the noise out
of the steering, and keeping the window short keeps the estimate local, so it
follows any shape -- straight wall, round table leg, or CONCAVE inside corner --
without smearing one into the next.

That distance is measured at the BODY CENTRE rather than at the LiDAR, which is
mounted ahead of the wheel axle: in a turn the shell swings wide of where the
LiDAR points, so servoing the raw range grazed tight convex corners.

CONVEX outside corners get an explicit recovery: when the near boundary vanishes
(range jumps, or nothing left in the sector) the follower stops trusting the far
reading and ARCS toward the follow side at ~standoff radius until it re-acquires
-- "lose the wall, curve toward it". Left-follow is the mirror (the scan bearings
and the output omega are both negated).

Phase 1: FOLLOW + convex ARC, with a rotate-in-place ALIGN entry. No loop-closure
yet -- it runs until stopped (like wall_clean). See docs/contour_follower_spec.md.

Every bump is logged: the start of each contact, not every message. With
halt_on_bump (the default) a bump while active also stops the robot dead and
parks it in HALTED, so a collision is left exactly where it happened to be
looked at, instead of being ground into or driven away from. Publish true on
~/enable to resume. A stand-in until a real front guard exists -- the follower steers only
on the nearest surface, so something in its path that is farther than the
followed wall is not avoided.

  subscribes  scan                  sensor_msgs/LaserScan      (SensorData QoS)
  subscribes  bumper_left/contact   ros_gz_interfaces/Contacts (logged; halts if halt_on_bump)
  subscribes  bumper_right/contact  ros_gz_interfaces/Contacts (logged; halts if halt_on_bump)
  subscribes  ~/enable              std_msgs/Bool              (stop/go; resumes HALTED)
  publishes   cmd_vel               geometry_msgs/Twist
  publishes   cleaning_active       std_msgs/Bool              (latched; True while active)
  publishes   ~/state               std_msgs/String            (ALIGN/FOLLOW/ARC/LOST/HALTED)
"""

import math
import time

from geometry_msgs.msg import Point, Twist

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

from ros_gz_interfaces.msg import Contacts

from sensor_msgs.msg import LaserScan

from std_msgs.msg import Bool, Float32, String

from visualization_msgs.msg import Marker, MarkerArray

TWO_PI = 2.0 * math.pi
ACTIVE_STATES = ('ALIGN', 'FOLLOW', 'ARC')
# Display only: a fitted radius above this reads as a flat surface. Noise alone
# bends a wall fit to R ~ 4 m, which is 3 mm of sag across a 0.15 m window.
FLAT_RADIUS_M = 2.0

DEFAULTS = {
    'follow_side': 'right',        # 'right' or 'left'
    'standoff_m': 0.23,            # body centre to surface; flush bumper reaches 0.1745
    'body_offset_m': 0.0745,       # LiDAR ahead of the wheel axle; = URDF lidar_center_offset
    'use_body_clearance': True,    # measure the standoff at the body centre, not the LiDAR
    # Measured and NOT adopted -- see _curvature_ff. Off: today's law holds curves
    # within ~2 cm of the standoff, always outward; these two together land gentle
    # bays on target but put the robot into the wall of a tight one.
    'use_body_bearing': False,     # measure the bearing at the body centre too
    'use_curvature_ff': False,     # turn at the rate the fitted curve needs
    'ff_min_points': 20,           # concave feed-forward only off a fit this well supported
    'ff_max_rms_m': 0.015,         # ...and this close to its points
    'ff_concave_gain': 0.5,        # concave feed-forward scale: its radius is ill-conditioned
    'ff_min_radius_m': 0.05,       # never feed forward a path radius tighter than this
    'ff_max': 0.8,                 # rad/s cap on the feed-forward alone
    'ff_tau_s': 0.3,               # low-pass on the feed-forward, so it ramps in
    'v_nominal': 0.15,             # m/s cruise
    'v_min': 0.05,                 # m/s floor (in corners)
    'sector_min_deg': -170.0,      # follow-side + forward window (right-follow)
    'sector_max_deg': 20.0,
    'max_follow_range_m': 1.0,     # ignore boundaries farther than this
    'fit_gap_m': 0.10,             # max step between adjacent points on one surface
    'fit_window_m': 0.15,          # fit only this far either way from the nearest point
    'min_fit_points': 6,           # below this, fall back to the nearest beam
    'fit_max_dev_m': 0.05,         # fit vs nearest beam: distance disagreement cap
    'fit_max_dev_deg': 35.0,       # fit vs nearest beam: bearing disagreement cap
    'point_guard_rank': 3,         # never report further than the Nth-nearest scan point
    'point_guard_margin_m': 0.010,  # ...plus this: the order statistic's own noise bias
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
    'halt_on_bump': True,          # any bumper contact -> stop dead, state HALTED
    'bump_quiet_s': 0.5,           # a side is 'newly bumped' after this long clear
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
        self._dbg_fit = None          # fitted curve polyline, for markers
        self._dbg_n = 0               # points in the fit window
        self._dbg_body = None         # body-centre distance to the fitted curve
        self._dbg_r = None            # fitted radius, None = straight
        self._bump_last = {}          # side -> time.monotonic() of its last contact
        self._fit_rms = None          # RMS distance of the fit window's points to the fit
        self._ff = 0.0                # filtered curvature feed-forward, rad/s

        latched = QoSProfile(
            depth=1, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.active_pub = self.create_publisher(Bool, 'cleaning_active', latched)
        self.state_pub = self.create_publisher(String, '~/state', 10)
        # Markers go out TRANSIENT_LOCAL: RViz's marker display (and Foxglove)
        # default to asking for it, and a VOLATILE publisher is silently
        # incompatible -- the subscriber connects and simply never draws.
        self.marker_pub = self.create_publisher(
            MarkerArray, '~/debug_markers',
            QoSProfile(depth=5, history=QoSHistoryPolicy.KEEP_LAST,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.err_d_pub = self.create_publisher(Float32, '~/standoff_err_m', 10)
        self.err_b_pub = self.create_publisher(Float32, '~/bearing_err_deg', 10)
        self.err_h_pub = self.create_publisher(Float32, '~/heading_err_deg', 10)
        self.create_subscription(
            LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Bool, '~/enable', self._on_enable, 10)
        self.create_subscription(
            Contacts, 'bumper_left/contact',
            lambda m: self._on_bump(m, 'left'), 10)
        self.create_subscription(
            Contacts, 'bumper_right/contact',
            lambda m: self._on_bump(m, 'right'), 10)
        self.create_timer(1.0 / max(self._p('pub_hz'), 1.0), self._pub_cmd)

        self.side = -1.0 if self._p('follow_side') == 'left' else 1.0
        self._set_state('ALIGN' if self.enabled else 'IDLE')
        self.get_logger().info(
            'contour_follower: follow %s, standoff %.2fm '
            '(local circle fit + convex arc; no loop-closure yet)'
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
        elif self.state in ('IDLE', 'LOST', 'HALTED'):
            self.prev_d = None
            self.arc_swept = 0.0
            self._set_state('ALIGN')

    def _on_bump(self, msg, which) -> None:
        """
        Log every bump; with halt_on_bump, also stop dead and stay stopped.

        Gazebo publishes a contact message on every physics step the bumper is
        pressed -- hundreds a second -- so only the START of a bump is logged: a
        side counts as newly bumped once it has been clear for bump_quiet_s.
        Each line records where the follower thought the surface was at that
        moment, which is the question a collision raises.

        When halting, the command is zeroed and published at once rather than
        on the next timer tick, so the robot does not keep pushing for a cycle.
        """
        if not msg.contacts:
            return
        now = time.monotonic()
        last = self._bump_last.get(which)
        self._bump_last[which] = now
        fresh = last is None or (now - last) > self._p('bump_quiet_s')
        if self.state not in ACTIVE_STATES:
            return
        halt = bool(self._p('halt_on_bump'))
        if not (fresh or halt):
            return
        seen = ('nothing' if self._dbg_d is None else
                'surface at %.2f m, %+.0f deg' % (self._dbg_d, math.degrees(self._dbg_b)))
        if not halt:
            self.get_logger().warn(
                'BUMP (%s bumper) during %s. Follower was tracking: %s [%s]. '
                'Carrying on (halt_on_bump is off).'
                % (which, self.state, seen, self._fit_description()))
            return
        was = self.state
        self._set_cmd(0.0, 0.0)
        self.cmd_pub.publish(self.cmd)
        self._set_state('HALTED')
        self.get_logger().warn(
            'BUMP (%s bumper) during %s -- halted. Follower was tracking: %s [%s]. '
            'Publish true on ~/enable to resume.'
            % (which, was, seen, self._fit_description()))

    @staticmethod
    def _fit_conic(sel):
        """
        Taubin circle fit -> A*(x^2+y^2) + B*x + C*y + D = 0, robot at origin.

        Taubin's algebraic form is used rather than a centre/radius fit because A
        (the curvature term) simply goes to zero on a straight surface instead of
        sending the centre off to infinity, so one estimator covers walls and
        round obstacles alike. Returns None if the window is degenerate.
        """
        x = np.array([p[0] for p in sel])
        y = np.array([p[1] for p in sel])
        xb, yb = x.mean(), y.mean()
        u, v = x - xb, y - yb          # fit centred: keeps the SVD well scaled
        z = u * u + v * v
        zm = z.mean()
        if zm <= 0.0:
            return None
        z0 = (z - zm) / (2.0 * math.sqrt(zm))
        _, _, vt = np.linalg.svd(np.column_stack([z0, u, v]), full_matrices=False)
        a0, a1, a2 = vt[2]
        a0 /= 2.0 * math.sqrt(zm)
        a3 = -zm * a0
        return (a0,                                        # A
                a1 - 2.0 * a0 * xb,                        # B  (un-centred)
                a2 - 2.0 * a0 * yb,                        # C
                a0 * (xb * xb + yb * yb) - a1 * xb - a2 * yb + a3)

    def _boundary(self, msg, smin, smax, max_r):
        """
        Fit the followed surface; return (distance, bearing, n points used).

        Seeds on the nearest beam in the sector, grows the contiguous surface
        around it out to fit_window_m, fits a circle to that window, and reports
        the distance from the robot to the fitted curve and the bearing of the
        nearest point on it.

        Why fit at all: near a wall's perpendicular the range is almost flat --
        at 0.2 m, swinging 20 deg changes it by 1.3 cm against ~2 cm of beam
        scatter -- so the ARG-min (which beam is nearest) is essentially random
        over a wide arc, and min() over noisy beams is a biased distance.

        Why a CIRCLE and a short window rather than a line over the whole
        surface: a line is only right for walls. Measured against synthetic
        scans at the sim LiDAR's specs, a whole-surface line fit smeared across
        inside corners (wall + front wall as one 10 deg-tilted line, which turned
        the robot ~0.8 m early) and gave +-15 deg on a 3 cm stool leg, where the
        curve is nothing like a line. A circle's curvature term goes to zero on a
        flat wall, so it reproduces the line fit there (+-0.8 deg), tracks round
        obstacles (+-0.4 deg), and -- because the window is short -- ignores the
        corner until the robot is at the standoff, turning where the nearest-beam
        method turns but without its noise.
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
            self._dbg_d = self._dbg_b = self._dbg_fit = self._dbg_r = None
            self._dbg_body = None
            self._dbg_n = 0
            return None, None, 0

        # Grow the contiguous surface either way from the seed, breaking at a
        # range discontinuity (fit_gap_m) or at the window edge (fit_window_m).
        # Kept in scan order, so the debug polyline traces the surface.
        gap = self._p('fit_gap_m')
        win = self._p('fit_window_m')
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
                if math.hypot(pts[k][0] - pts[seed][0],
                              pts[k][1] - pts[seed][1]) > win:
                    break
                keep.append(k) if step == 1 else keep.insert(0, k)
                j = k
        sel = [pts[k] for k in keep]

        co = (self._fit_conic(sel)
              if len(sel) >= int(self._p('min_fit_points')) else None)
        if co is not None:
            a, b_, c, d = co
            grad = math.hypot(b_, c)
            if grad > 1e-9:
                # Distance from the robot to the curve, rationalized so the
                # straight case (A -> 0) stays numerically sane; the direction
                # toward the surface is the conic's gradient at the robot.
                disc = max(0.0, b_ * b_ + c * c - 4.0 * a * d)
                dist = 2.0 * abs(d) / (grad + math.sqrt(disc))
                sg = -1.0 if d > 0.0 else 1.0
                bear = math.atan2(sg * c, sg * b_)
                # Sanity-check the fit against the beam that seeded it. On a
                # short arc -- a table leg is ~15 beams -- the algebraic fit can
                # occasionally converge to a tiny circle placed nowhere near the
                # surface: measured at 0.2-1% of frames on a 2 cm leg, once
                # reporting d=0.06 m at a bearing BEHIND the robot. One bad frame
                # commands a full-rate turn, so reject a fit that disagrees with
                # the raw nearest beam and use the beam instead. It never fires
                # on walls, corners or large curves.
                if (abs(dist - seed_r) <= self._p('fit_max_dev_m')
                        and abs(math.remainder(bear - pts[seed][3], TWO_PI))
                        <= math.radians(self._p('fit_max_dev_deg'))):
                    self._dbg_d, self._dbg_b = dist, bear
                    self._dbg_fit = [self._project(p, a, b_, c, d)
                                     for p in sel[::max(1, len(sel) // 20)]]
                    self._dbg_n = len(sel)
                    # Signed radius: + = the surface curves AWAY from the
                    # robot (convex, a table leg), - = it wraps around the robot
                    # (concave, an inside corner). The robot lies outside the
                    # fitted circle exactly when D/A > 0, since
                    # |centre|^2 - R^2 = D/A.
                    self._dbg_r = (
                        math.copysign(math.sqrt(disc) / (2.0 * abs(a)), d * a)
                        if abs(a) > 1e-6 else None)
                    self._fit_rms = self._fit_residual(sel, (a, b_, c, d))
                    self._dbg_body = self._body_distance((a, b_, c, d), dist, bear)
                    reported = (self._dbg_body if self._p('use_body_clearance')
                                else dist)
                    guard = self._point_guard(sel, self._p('point_guard_rank'))
                    self._dbg_body = min(self._dbg_body, guard)
                    if self._p('use_body_bearing'):
                        bear = self._body_bearing((a, b_, c, d), bear)
                    return min(reported, guard), bear, len(sel)

        self._dbg_d, self._dbg_b = seed_r, pts[seed][3]
        self._dbg_fit = self._dbg_r = self._fit_rms = None
        self._dbg_n = len(sel)
        self._dbg_body = self._body_distance(None, seed_r, pts[seed][3])
        reported = self._dbg_body if self._p('use_body_clearance') else seed_r
        if sel:
            guard = self._point_guard(sel, self._p('point_guard_rank'))
            self._dbg_body = min(self._dbg_body, guard)
            reported = min(reported, guard)
        bear = pts[seed][3]
        if self._p('use_body_bearing'):
            # no curve to evaluate: the direction from the body centre to the beam
            bear = math.atan2(pts[seed][1], pts[seed][0] + self._p('body_offset_m'))
        return reported, bear, len(sel)

    def _point_guard(self, sel, rank):
        """
        Distance to the Nth-nearest scan point, measured where the standoff is.

        A fitted circle ROUNDS a sharp corner, so while wrapping one the curve
        passes inside the corner itself and the reported distance is optimistic
        -- measured up to +23 mm around a box corner and +54 mm in a room's
        inside corners, which is most of the clearance the standoff buys. The
        points do not lie: the corner tip is one of them. Reporting the smaller
        of the fitted distance and this one keeps the smooth, low-noise estimate
        everywhere the surface really is smooth, and falls back to raw points
        exactly where the fit is wrong.

        The Nth-nearest rather than the very nearest, because the single nearest
        beam carries the full noise. Even so, the 3rd-nearest of ~60 noisy points
        sits about 1 cm closer than the surface really is, and since on a smooth
        surface that makes the guard win every frame, it held the robot +9.9 mm
        out on EVERY straight wall (and was most of what looked like a curvature
        error). So the guard is lifted by point_guard_margin_m, its own noise
        bias: on a smooth surface it no longer beats the unbiased fit, and at a
        corner, where the fit is 23-54 mm optimistic, it still wins by plenty.
        """
        off = self._p('body_offset_m') if self._p('use_body_clearance') else 0.0
        ds = sorted(math.hypot(p[0] + off, p[1]) for p in sel)
        return ds[min(int(rank) - 1, len(ds) - 1)] + self._p('point_guard_margin_m')

    def _body_bearing(self, co, fallback):
        """
        Bearing of the fitted curve's nearest point, seen from the BODY CENTRE.

        The LiDAR sits body_offset_m ahead of the body centre, and from there
        the nearest point of a CURVED surface is not abeam even when the robot
        runs perfectly tangent to it: it is off by atan(offset / path radius) --
        16.6 deg around a 4 cm leg, 31.8 deg inside a 0.35 m bay, matching the
        logged steady bearing errors. The controller read that geometry as the
        robot being angled, and settled at the wrong distance to cancel it.
        From the body centre, a tangent robot sees the nearest point exactly
        abeam, on any curvature. It is the conic's gradient direction there.
        """
        a, b, c, d = co
        px = -self._p('body_offset_m')
        dp = a * px * px + b * px + d
        bp, cp = 2.0 * a * px + b, c
        if math.hypot(bp, cp) < 1e-9:
            return fallback
        sg = -1.0 if dp > 0.0 else 1.0
        return math.atan2(sg * cp, sg * bp)

    @staticmethod
    def _fit_residual(sel, co):
        """RMS distance of the window's points to the fitted curve (first order)."""
        a, b, c, d = co
        x = np.array([p[0] for p in sel])
        y = np.array([p[1] for p in sel])
        val = a * (x * x + y * y) + b * x + c * y + d
        grad = np.hypot(2.0 * a * x + b, 2.0 * a * y + c)
        return float(np.sqrt(np.mean((val / np.maximum(grad, 1e-9)) ** 2)))

    def _curvature_ff(self, v):
        """
        Turn rate the fitted curve asks for, or 0 when the fit is not trusted.

        A proportional controller can only turn by holding an error, so on a
        curve it settles off the standoff. Feeding forward the turn rate the
        curve itself needs -- v over the path radius -- leaves the feedback
        nothing to hold. Only with the bearing measured at the body centre,
        though: from the LiDAR the geometry already supplied most of this turn,
        and adding it on top spiralled the robot into a table leg.

        Convex and concave are treated differently because their path radii
        are conditioned differently. Convex (a leg, a pillar): the path radius
        is R + standoff, dominated by the standoff, so even a rough R off a
        15-point leg is good enough and the feed-forward always applies. Concave
        (a bay): the path radius is |R| - standoff, a DIFFERENCE of similar
        numbers -- a 0.35 m bay fitted as 0.27 m gives 0.04 m instead of 0.12 m,
        three times the turn. So concave feed-forward needs a well-supported fit,
        is scaled down by ff_concave_gain, and is skipped for a bay the robot
        cannot orbit inside at all.

        OFF BY DEFAULT, on measurement. Today's law turns out to hold curves well
        already: with the bearing taken at the LiDAR, the offset geometry and the
        proportional lag nearly cancel. True clearance on the curve against a
        0.23 m target, harness, today vs body bearing + this feed-forward:

            2 cm leg      0.236 vs 0.248      bay R 1.00   0.233 vs 0.236
            5 cm leg      0.238 vs 0.242      bay R 0.75   0.239 vs 0.234
            15 cm seat    0.247 vs 0.243      bay R 0.50   0.242 vs 0.230
                                              bay R 0.35   0.253 vs 0.211, and
                                              touches (min 0.164)

        Gentle bays land on target, but legs get no better and the tight bay goes
        from 2 cm too far out (safe) to into the wall. (Measured before the point
        guard got its noise margin, which since took ~1 cm off every "today"
        figure -- a straight wall was sitting 9.9 mm out -- so today's law is
        closer still.) The body-centre bearing
        alone is worse still: legs swing ~11 cm out and bays hit.
        """
        r = self._dbg_r
        if not self._p('use_curvature_ff') or r is None or abs(r) > FLAT_RADIUS_M:
            return 0.0
        s = self._p('standoff_m')
        if r > 0.0:                                 # convex: turn toward it
            return _clamp(-v / (r + s), -self._p('ff_max'), self._p('ff_max'))
        if (self._dbg_n < self._p('ff_min_points') or self._fit_rms is None
                or self._fit_rms > self._p('ff_max_rms_m')):
            return 0.0
        path = -r - s                               # concave: |R| - s, turn away
        if path < self._p('ff_min_radius_m'):
            return 0.0
        return _clamp(self._p('ff_concave_gain') * v / path, 0.0, self._p('ff_max'))

    def _command(self, d, b, b_ref, dt):
        """
        Compute the control law: (v, omega, e_d, e_b, alpha, e_h), no side effects.

        Kept free of publishing so the offline harness drives exactly this, not
        a copy of it.
        """
        e_d = d - self._p('standoff_m')
        e_b = b - b_ref                  # + = currently angled toward the wall
        # Outer loop: how far to angle toward/away, CAPPED. Without the cap a far
        # wall demands a saturated turn that the heading term cancels, and the robot
        # crawls in at the speed floor instead of approaching cleanly.
        a_max = math.radians(self._p('alpha_max_deg'))
        alpha = _clamp(self._p('k_approach') * e_d, -a_max, a_max)
        # Inner loop: steer the actual angle onto the desired one.
        e_h = alpha - e_b
        # Ease off only when the INNER loop is far off (a real corner); a steady
        # approach has e_h ~ 0, so it runs at full speed.
        slow = math.radians(self._p('slow_angle_deg'))
        v = self._p('v_nominal') * (1.0 - min(1.0, abs(e_h) / max(slow, 1e-3)))
        v = max(self._p('v_min'), v)
        # Curvature feed-forward, low-passed so a fit appearing or vanishing ramps
        # the turn in and out instead of kicking it.
        k = min(1.0, dt / max(self._p('ff_tau_s'), 1e-3)) if dt > 0.0 else 1.0
        self._ff += (self._curvature_ff(v) - self._ff) * k
        omega = _clamp(-self._p('k_heading') * e_h + self._ff,
                       -self._p('omega_max'), self._p('omega_max'))
        return v, omega, e_d, e_b, alpha, e_h

    def _body_distance(self, co, d_lidar, bear):
        """
        Distance from the BODY CENTRE to the fitted curve.

        The follower's job is to keep the SHELL off the furniture, but the
        LiDAR sits body_offset_m ahead of the wheel axle, so on a tight turn
        the body swings wide of wherever the LiDAR is pointing and grazes what
        the LiDAR clears. Measured on the test harness, servoing the LiDAR's
        range put the body centre 0.169 m from a wall's tip and 0.174 m from a
        box corner against a 0.1745 m body radius -- i.e. contact.

        The fitted conic is a curve in the scan frame, so the body centre is
        just another point to evaluate it at: translate the coefficients to
        p = (-offset, 0) and reuse the same rationalized distance. With no fit
        to evaluate, fall back to treating the seed beam as perpendicular to
        the surface, which gives d + offset*cos(bearing): identical to the
        LiDAR range when running parallel, and correctly SMALLER when angled in.
        """
        off = self._p('body_offset_m')
        if co is None:
            return d_lidar + off * math.cos(bear)
        a, b, c, d = co
        px = -off
        dp = a * px * px + b * px + d
        bp = 2.0 * a * px + b
        cp = c
        grad = math.hypot(bp, cp)
        if grad < 1e-9:
            return d_lidar + off * math.cos(bear)
        disc = max(0.0, bp * bp + cp * cp - 4.0 * a * dp)
        return 2.0 * abs(dp) / (grad + math.sqrt(disc))

    @staticmethod
    def _project(p, a, b, c, d):
        """Pull a scan point onto the fitted curve, for the debug polyline."""
        val = a * (p[0] * p[0] + p[1] * p[1]) + b * p[0] + c * p[1] + d
        gx, gy = 2.0 * a * p[0] + b, 2.0 * a * p[1] + c
        g2 = gx * gx + gy * gy
        if g2 < 1e-18:
            return p[0], p[1]
        return p[0] - val * gx / g2, p[1] - val * gy / g2

    def _on_scan(self, msg: LaserScan) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.0 if self.prev_t is None else max(0.0, t - self.prev_t)
        self.prev_t = t
        if not self.enabled or self.state in ('IDLE', 'LOST', 'HALTED'):
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
        self._pub_markers(b_ref)

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
                self._follow(d, b, b_ref, dt)
                return

        if self.state == 'ARC':
            self._arc(msg, smin, smax, max_r, b_ref, dt)

    def _follow(self, d, b, b_ref, dt=0.0) -> None:
        v, omega, e_d, e_b, alpha, e_h = self._command(d, b, b_ref, dt)
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
            self._follow(d, b, b_ref, dt)
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

    def _pub_markers(self, b_ref) -> None:
        """
        Draw the fitted curve, the picked point and the standoff target.

        Only what the controller is steering on, deliberately: the state and the
        numbers go to the throttled log line, and the search sector's edge lines
        were dropped too -- both were clutter in a view whose job is to show
        where the robot thinks the surface is.

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
            # The fitted curve, lifted clear of the scan so it reads on top.
            fit = self._mk(5, Marker.LINE_STRIP, stamp)
            fit.scale.x = 0.012
            fit.color.r = 1.0
            fit.color.b = 1.0
            fit.color.a = 0.9
            for px, py in self._dbg_fit:
                p = Point()
                p.x, p.y, p.z = px, s * py, 0.03
                fit.points.append(p)
            arr.markers.append(fit)
            # Its endpoints: the extent of the window the fit actually used.
            ends = self._mk(6, Marker.SPHERE_LIST, stamp)
            ends.scale.x = ends.scale.y = ends.scale.z = 0.03
            ends.color.r = 1.0
            ends.color.b = 1.0
            ends.color.a = 0.9
            for px, py in (self._dbg_fit[0], self._dbg_fit[-1]):
                p = Point()
                p.x, p.y, p.z = px, s * py, 0.03
                ends.points.append(p)
            arr.markers.append(ends)
        self.marker_pub.publish(arr)

    def _maybe_log(self, d, e_d, e_b, alpha, e_h, v, omega) -> None:
        """
        Throttled one-liner of what the controller is actually steering on.

        Reads: how far the picked boundary is vs the target, how far the robot is
        currently angled toward it, how far we WANT it angled (the capped approach
        angle), and the resulting command. If "toward" tracks "want", the loop is
        doing its job and any residual angle is just the approach in progress.

        d is what the controller servos -- by default the BODY centre's distance
        to the surface -- with the raw LiDAR range beside it in brackets. The two
        are equal running parallel to a wall and diverge in turns, which is
        exactly where the robot used to graze. The trailing bracket names the
        surface the estimate came off -- how many points, and whether the fit
        came out straight, convex (a leg) or concave (an inside corner).
        """
        now = self.get_clock().now()
        period = Duration(seconds=float(self._p('log_period_s')))
        if self._t_log is not None and (now - self._t_log) < period:
            return
        self._t_log = now
        lidar_d = '' if self._dbg_d is None else ' (lidar %.2f)' % self._dbg_d
        self.get_logger().info(
            '%-6s d=%.2fm%s (target %.2f, err %+.2f)  toward=%+5.1f deg  '
            'want=%+5.1f  err=%+5.1f  ->  v=%.2f w=%+.2f  [%s]'
            % (self.state, d, lidar_d, self._p('standoff_m'), e_d,
               math.degrees(e_b), math.degrees(alpha), math.degrees(e_h),
               v, omega, self._fit_description()))

    def _fit_description(self):
        """One phrase naming the surface the estimate came off, for the log."""
        if self._dbg_fit is None:
            return 'no fit, %d pts' % self._dbg_n
        if self._dbg_r is None or abs(self._dbg_r) > FLAT_RADIUS_M:
            return 'fit %d pts, straight' % self._dbg_n
        return 'fit %d pts, R=%.2f %s' % (
            self._dbg_n, abs(self._dbg_r),
            'convex' if self._dbg_r > 0.0 else 'concave')

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
