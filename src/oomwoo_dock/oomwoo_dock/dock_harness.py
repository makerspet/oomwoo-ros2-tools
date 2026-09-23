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
Closed-loop 2D rig for docking: ray-traced LiDAR, kinematic robot, no ROS.

The same idea as oomwoo_clean's contour_harness, aimed at the dock: it answers
"does the manoeuvre survive this approach" in seconds, so Gazebo is left to test
what 2D cannot -- ramp tilt, dark plastic, real timing, and the fact that the
dock's visual and collision geometry differ.

The robot reverses in, because the real machine has to back in to wash its mops,
and its LiDAR sits FORWARD of the wheel axle, so the sensor watches the dock over
the robot's own tail the whole way.

Run it for the scenario table:

    python3 -m oomwoo_dock.dock_harness
"""

import math
import random

from oomwoo_dock.dock_template import (
    accept, DockFitter, inv, mul, template, wrap,
)

BEAMS = 360
RANGE_SIGMA_M = 0.010
BODY_RADIUS_M = 0.1745
LIDAR_OFFSET_M = 0.0745
SCAN_HZ = 10.0
DETECT_EVERY = 5                  # detect at 2 Hz; odometry carries the rest
BAY_HALF_M = 0.200                # PHYSICAL half width (collision geometry)
BAY_DEPTH_M = 0.225               # mouth to back face
STAGE_X_M = -0.55                 # staging point outside the mouth, on the axis
T_BL = (LIDAR_OFFSET_M, 0.0, 0.0)


def place(segs, circs, pose):
    """Move dock-frame geometry into another frame."""
    px, py, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)

    def tf(p):
        return (px + p[0] * c - p[1] * s, py + p[0] * s + p[1] * c)

    return [(tf(a), tf(b)) for a, b in segs], [(tf(ctr), r) for ctr, r in circs]


def ray(segs, circs, ang):
    """Range along a ray from the origin; inf if it hits nothing."""
    c, s = math.cos(ang), math.sin(ang)
    best = float('inf')
    for (x0, y0), (x1, y1) in segs:
        dx, dy = x1 - x0, y1 - y0
        den = c * dy - s * dx
        if abs(den) < 1e-12:
            continue
        t = (x0 * dy - y0 * dx) / den
        u = (x0 * s - y0 * c) / den
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
    """One noisy scan's worth of points, sensor at the origin."""
    inc = 2.0 * math.pi / BEAMS
    pts = []
    for i in range(BEAMS):
        a = -math.pi + i * inc
        r = ray(segs, circs, a)
        if math.isfinite(r):
            r += random.gauss(0.0, sigma)
            pts.append((r * math.cos(a), r * math.sin(a)))
    return pts


def scene(back_wall=False, wall_gap=0.0, chair=False):
    """Dock geometry in the DOCK frame, plus optional clutter around it."""
    segs, circs = template()
    if back_wall:
        x = 0.325 + wall_gap
        segs = segs + [((x, -2.0), (x, 2.0))]
    if chair:
        for dx, dy in ((0.15, 0.55), (0.15, 0.95), (0.55, 0.55), (0.55, 0.95)):
            circs = circs + [((dx, dy), 0.02)]
    return segs, circs


def wall_contact(pose_in_dock):
    """Robot shell against a bay wall: a failure."""
    x, y, _th = pose_in_dock
    return x > 0.0 and abs(y) + BODY_RADIUS_M > BAY_HALF_M


def seated(pose_in_dock):
    """Tail at the back face: the intended end of the manoeuvre."""
    return pose_in_dock[0] + BODY_RADIUS_M >= BAY_DEPTH_M - 0.005


def run(dist=0.75, bearing_deg=0.0, yaw_deg=0.0, seed=0, seconds=60.0,
        slip=0.02, back_wall=True, chair=False, detect_every=DETECT_EVERY):
    """
    One docking attempt. Returns a metrics dict.

    The robot starts `dist` from the mouth, `bearing_deg` off the bay axis, with
    `yaw_deg` of heading error, and a prior that is off by up to 0.1 m and 15 deg.
    Metrics: ok, why, lateral and yaw error at the end, depth, detections.
    """
    random.seed(seed)
    fit = DockFitter()
    segs, circs = scene(back_wall=back_wall, chair=chair)
    b = math.radians(bearing_deg)
    t_w_b = (-dist * math.cos(b), -dist * math.sin(b),
             wrap(b + math.radians(yaw_deg)))
    est = mul(inv(t_w_b), (0.0, 0.0, 0.0))
    est = (est[0] + random.uniform(-0.1, 0.1),
           est[1] + random.uniform(-0.1, 0.1),
           wrap(est[2] + math.radians(random.uniform(-15.0, 15.0))))
    dt = 1.0 / SCAN_HZ
    state = 'GOTO'
    detects = 0
    for step in range(int(seconds * SCAN_HZ)):
        if step % detect_every == 0:
            t_w_l = mul(t_w_b, T_BL)
            pts = scan(*place(segs, circs, inv(t_w_l)))
            got = fit.detect(pts, mul(inv(T_BL), est))
            if accept(got):
                est = mul(T_BL, got.pose)
                detects += 1
        x, y, th = inv(est)
        v = w = 0.0
        if state == 'GOTO':
            dx, dy = STAGE_X_M - x, -y
            to_stage = wrap(math.atan2(dy, dx) - th)
            if math.hypot(dx, dy) < 0.04:
                state = 'TURN'
            else:
                w = max(-0.8, min(0.8, 1.8 * to_stage))
                v = 0.14 * max(0.0, math.cos(to_stage))
        elif state == 'TURN':
            err = wrap(th - math.pi)
            w = max(-0.7, min(0.7, -1.6 * err))
            if abs(err) < math.radians(3.0):
                state = 'BACK'
        else:
            v = -0.06
            w = max(-0.5, min(0.5, -(2.5 * y + 1.8 * wrap(th - math.pi))))
            if seated((x, y, th)):
                state = 'DONE'
                break
        v_a = v * (1.0 + random.gauss(0.0, slip))
        w_a = w * (1.0 + random.gauss(0.0, slip)) + random.gauss(0.0, 0.004)
        t_w_b = mul(t_w_b, (v_a * dt, 0.0, w_a * dt))
        est = mul(inv((v * dt, 0.0, w * dt)), est)        # odometry, no slip
        true_pose = inv(mul(inv(t_w_b), (0.0, 0.0, 0.0)))
        if wall_contact(true_pose):
            return {'ok': False, 'why': 'wall hit', 'state': state,
                    'pose': true_pose, 'lateral': true_pose[1],
                    'yaw_err': wrap(true_pose[2] - math.pi), 'detects': detects}
    pose = inv(mul(inv(t_w_b), (0.0, 0.0, 0.0)))
    yaw_err = wrap(pose[2] - math.pi)
    ok = (state == 'DONE'
          and abs(pose[1]) < BAY_HALF_M - BODY_RADIUS_M
          and abs(yaw_err) < math.radians(6.0))
    return {'ok': ok, 'why': '' if ok else 'not seated', 'state': state,
            'pose': pose, 'lateral': pose[1], 'yaw_err': yaw_err,
            'detects': detects}


SCENARIOS = [
    ('0.6 m, head-on', {'dist': 0.6, 'bearing_deg': 0, 'yaw_deg': 0}),
    ('0.6 m, 15 deg off', {'dist': 0.6, 'bearing_deg': 15, 'yaw_deg': 0}),
    ('0.6 m, 25 deg off, yaw 12', {'dist': 0.6, 'bearing_deg': -25, 'yaw_deg': 12}),
    ('0.9 m, head-on', {'dist': 0.9, 'bearing_deg': 0, 'yaw_deg': 0}),
    ('0.9 m, 15 deg off, yaw 12', {'dist': 0.9, 'bearing_deg': 15, 'yaw_deg': 12}),
    ('0.9 m, 25 deg off', {'dist': 0.9, 'bearing_deg': -25, 'yaw_deg': 0}),
    ('0.6 m, head-on, chair beside', {'dist': 0.6, 'bearing_deg': 0, 'chair': True}),
]


def main():
    """Run every scenario and print the table."""
    print('%-30s %9s %11s %9s %7s'
          % ('start', 'result', 'lateral', 'yaw err', 'fixes'))
    good = 0
    for i, (name, kw) in enumerate(SCENARIOS):
        m = run(seed=i, **kw)
        good += m['ok']
        print('%-30s %9s %8.1f mm %7.2f d %7d'
              % (name, 'docked' if m['ok'] else m['why'],
                 m['lateral'] * 1e3, math.degrees(m['yaw_err']), m['detects']))
    print('\n%d/%d docked; the bay allows +-%.1f mm of lateral error'
          % (good, len(SCENARIOS), (BAY_HALF_M - BODY_RADIUS_M) * 1e3))


if __name__ == '__main__':
    main()
