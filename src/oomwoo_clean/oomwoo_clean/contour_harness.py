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
Closed-loop 2D test harness for the contour follower -- the torture rig.

Runs the REAL boundary estimator and the REAL control law against a kinematic
robot and a ray-traced LiDAR, with no ROS, no Gazebo and no rendering, so a
40-second scenario finishes in a couple of seconds and can gate CI. Gazebo stays
the authority on everything this deliberately leaves out -- the bookshelf lip
below the scan plane, carpet, wheel slip, real timing -- but every question of
the form "does the control law survive this shape" gets answered here first, in
seconds, repeatably, with a number attached.

Worth having because the alternative is eyeballing RViz: a curvature
feed-forward that looked obviously right was measured here to drive the robot's
clearance around a table leg down from 0.178 m to 0.048 m, in about a minute.

Scenes are lists of segments and circles. The robot is a unicycle of
BODY_RADIUS_M with the LiDAR mounted LIDAR_OFFSET_M ahead of the wheel axle,
matching oomwoo-one, because that offset is exactly what makes a naive
curvature correction misbehave.

Run it for the scenario table:

    python3 -m oomwoo_clean.contour_harness
"""

import math
import random

BODY_RADIUS_M = 0.1745      # oomwoo-one body, base_diameter/2
# What touches first: the bumper, a ring of flat facets. Since the bumper was
# made flush with the body (oomwoo-one), the facets' corners land exactly on
# the body radius, so the robot's outline is a plain circle of this radius.
# It used to stand 5 mm proud and reach 0.1814 m, which is why a Gazebo run
# once halted on a bump with the body centre 0.18 m from the wall.
CONTACT_RADIUS_M = 0.1745
LIDAR_OFFSET_M = 0.0745     # LiDAR ahead of the wheel axle
BEAMS = 360
RANGE_SIGMA_M = 0.01        # matches the sim LiDAR's noise
RANGE_MIN_M = 0.1
SCAN_HZ = 10.0


def segment(p0, p1):
    """Build a wall segment, as ((x0, y0), (x1, y1))."""
    return (p0, p1)


def circle(centre, radius):
    """Build a round obstacle (a table or chair leg)."""
    return (centre, radius)


def arc(cx, cy, r, a0, a1, n=48):
    """Build an arc of wall as n segments, from angle a0 to a1 (radians)."""
    pts = [(cx + r * math.cos(a0 + (a1 - a0) * k / n),
            cy + r * math.sin(a0 + (a1 - a0) * k / n)) for k in range(n + 1)]
    return [segment(pts[k], pts[k + 1]) for k in range(n)]


def box(cx, cy, half):
    """Build a square obstacle's four walls, centred at (cx, cy)."""
    c = [(cx - half, cy - half), (cx + half, cy - half),
         (cx + half, cy + half), (cx - half, cy + half)]
    return [segment(c[i], c[(i + 1) % 4]) for i in range(4)]


def _ray(segs, circs, ang):
    """Range along a ray from the origin at bearing ang; inf if it hits nothing."""
    c, s = math.cos(ang), math.sin(ang)
    best = float('inf')
    for (x0, y0), (x1, y1) in segs:
        dx, dy = x1 - x0, y1 - y0
        den = c * dy - s * dx
        if abs(den) < 1e-12:
            continue
        t = (x0 * dy - y0 * dx) / den          # along the ray
        u = (x0 * s - y0 * c) / den            # along the segment
        if t > 0.0 and 0.0 <= u <= 1.0:
            best = min(best, t)
    for (cx, cy), r in circs:
        b = cx * c + cy * s
        disc = b * b - (cx * cx + cy * cy - r * r)
        if disc < 0.0:
            continue
        t = b - math.sqrt(disc)
        if t > 0.0:
            best = min(best, t)
    return best


def scan(segs, circs, sigma=RANGE_SIGMA_M):
    """Ray-trace one noisy LaserScan's worth of ranges, robot at the origin."""
    inc = 2.0 * math.pi / BEAMS
    out = []
    for i in range(BEAMS):
        r = _ray(segs, circs, -math.pi + i * inc)
        out.append(r + random.gauss(0.0, sigma) if math.isfinite(r) else r)
    return out


def to_robot(world, x, y, th):
    """Express a world (segments, circles) in the robot's frame."""
    c, s = math.cos(-th), math.sin(-th)

    def rot(px, py):
        px, py = px - x, py - y
        return (px * c - py * s, px * s + py * c)

    segs = [(rot(*p0), rot(*p1)) for p0, p1 in world[0]]
    circs = [(rot(*ctr), r) for ctr, r in world[1]]
    return segs, circs


def clearance(world, x, y):
    """Distance from a point to the nearest surface in the world."""
    best = float('inf')
    for (x0, y0), (x1, y1) in world[0]:
        dx, dy = x1 - x0, y1 - y0
        den = dx * dx + dy * dy or 1e-12
        t = max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / den))
        best = min(best, math.hypot(x - (x0 + t * dx), y - (y0 + t * dy)))
    for (cx, cy), r in world[1]:
        best = min(best, abs(math.hypot(x - cx, y - cy) - r))
    return best


def make_follower(**overrides):
    """Build a ContourFollower with no ROS behind it, for offline stepping."""
    from oomwoo_clean.contour_follower_node import ContourFollower, DEFAULTS
    params = dict(DEFAULTS)
    params.update(overrides)
    node = ContourFollower.__new__(ContourFollower)
    node.side = -1.0 if params['follow_side'] == 'left' else 1.0
    node._p = params.get
    node._dbg_d = node._dbg_b = node._dbg_fit = node._dbg_r = None
    node._dbg_n = 0
    node._bump_last = {}
    node._fit_rms = None
    node._ff = 0.0
    return node, params


class Scan:
    """The few LaserScan fields the estimator reads."""

    def __init__(self, ranges):
        """Wrap a list of ranges as the estimator's view of a scan."""
        self.ranges = ranges
        self.angle_min = -math.pi
        self.angle_increment = 2.0 * math.pi / BEAMS
        self.range_min = RANGE_MIN_M


def run(world, start, seconds=40.0, seed=0, settle_s=5.0, **overrides):
    """
    Drive the follower around a world; return a metrics dict.

    start is (x, y, heading). Metrics: min_clearance (body centre to the nearest
    surface -- below CONTACT_RADIUS_M means the bumper touched something), standoff error mean
    and max, bearing_lag (steady-state droop, degrees), laps (net turning / 360)
    and lost_frames.

    Clearance is scored only after settle_s: every scenario starts the robot a
    fixed distance off its wall, and without the settle window that start pose,
    not the follower, sets the minimum for any standoff above it.
    """
    node, params = make_follower(**overrides)
    x, y, th = start
    dt = 1.0 / SCAN_HZ
    random.seed(seed)
    smin = math.radians(params['sector_min_deg'])
    smax = math.radians(params['sector_max_deg'])
    b_ref = math.radians(params['bearing_ref_deg'])
    errs, lags = [], []
    min_clear, turned, lost = float('inf'), 0.0, 0
    for step in range(int(seconds * SCAN_HZ)):
        lx = x + LIDAR_OFFSET_M * math.cos(th)
        ly = y + LIDAR_OFFSET_M * math.sin(th)
        segs, circs = to_robot(world, lx, ly, th)
        d, b, _n = node._boundary(Scan(scan(segs, circs)), smin, smax,
                                  params['max_follow_range_m'])
        if d is None:
            lost += 1
            v = params['v_min']
            w = -params['v_nominal'] / params['convex_arc_radius_m']
        else:
            # the node's own control law, not a copy of it
            v, w, e_d, e_b, _alpha, _e_h = node._command(d, b, b_ref, dt)
            errs.append(e_d)
            lags.append(math.degrees(e_b))
        w_out = node.side * w
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        th += w_out * dt
        turned += w_out * dt
        if step * dt >= settle_s:
            min_clear = min(min_clear, clearance(world, x, y))
    n = max(1, len(errs))
    return {
        'min_clearance': min_clear,
        'hit': min_clear < CONTACT_RADIUS_M,
        'standoff_mean': sum(abs(e) for e in errs) / n,
        'standoff_max': max((abs(e) for e in errs), default=0.0),
        'bearing_lag': sum(lags) / n,
        'laps': abs(math.degrees(turned)) / 360.0,
        'lost_frames': lost,
    }


ROOM = ([segment((-2.0, -2.0), (2.0, -2.0)), segment((2.0, -2.0), (2.0, 2.0)),
         segment((2.0, 2.0), (-2.0, 2.0)), segment((-2.0, 2.0), (-2.0, -2.0))], [])

SCENARIOS = {
    # name: (world, start pose)
    'room': (ROOM, (-1.5, -1.8, 0.0)),
    'box': ((box(0.0, 0.0, 0.3), []), (-0.9, -0.5, 0.0)),
    'table_leg_2cm': (([], [circle((0.0, 0.0), 0.02)]), (-0.22, 0.0, math.pi / 2)),
    'table_leg_5cm': (([], [circle((0.0, 0.0), 0.05)]), (-0.25, 0.0, math.pi / 2)),
    'leg_by_wall': (([segment((-2.0, -0.6), (2.0, -0.6))],
                     [circle((0.4, -0.42), 0.02)]), (-1.2, -0.4, 0.0)),
    # Long enough that a 30 s run never reaches the wall's end -- the tip is a
    # separate scenario, so a corridor failure means a corridor problem.
    'corridor': (([segment((-4.0, -0.6), (6.0, -0.6)),
                   segment((-0.4, 0.0), (1.2, 0.0))], []), (-1.4, -0.4, 0.0)),
    # The tightest convex turn there is: the bare END of a wall, which the
    # follower has to wrap 180 degrees around. Currently grazes -- see
    # test_wrapping_a_wall_end_grazes_it.
    'wall_end': (([segment((-2.0, -0.6), (0.4, -0.6))], []), (-1.2, -0.4, 0.0)),
    # Something standing IN the path while the followed wall stays nearer. The
    # follower only steers on the nearest surface, so the post is ignored until
    # it swings out of the search sector, and then it is never seen at all. The
    # torture-course panel and the second table leg in Gazebo were both this.
    # Placement is what makes it a test: the default standoff puts the path at
    # y = -0.37, and the post sits 0.12 m left of that -- inside the bumper's
    # 0.18 m half-width, but far enough out that by the time it is as near as
    # the wall (0.23 m) it is at +31 deg, past the sector's +20 deg edge.
    # Needs a front guard.
    # A concave bay of R 0.35 cut into a straight wall, opening toward the
    # room: the robot must orbit INSIDE it at only 0.12 m radius. The tight
    # concave case, where the follower settles outward of the standoff.
    'concave_bay': (([segment((-3.0, -0.6), (-0.35, -0.6))]
                     + arc(0.0, -0.6, 0.35, math.pi, 2.0 * math.pi)
                     + [segment((0.35, -0.6), (3.0, -0.6))], []), (-1.4, -0.37, 0.0)),
    'post_in_path': (([segment((-2.0, -0.6), (2.0, -0.6))],
                      [circle((0.3, -0.25), 0.02)]), (-1.2, -0.37, 0.0)),
}


def main():
    """Run every scenario and print the table."""
    print('%-16s %10s %6s %10s %8s %6s %6s'
          % ('scenario', 'min clear', 'hit?', 'stand err', 'lag deg', 'laps', 'lost'))
    for name in sorted(SCENARIOS):
        world, start = SCENARIOS[name]
        m = run(world, start, seconds=40.0, seed=1)
        print('%-16s %9.3fm %6s %9.3fm %8.1f %6.1f %6d'
              % (name, m['min_clearance'], 'HIT' if m['hit'] else 'ok',
                 m['standoff_mean'], m['bearing_lag'], m['laps'],
                 m['lost_frames']))


if __name__ == '__main__':
    main()
