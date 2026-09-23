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
BAY_DEPTH_M = 0.240               # mouth to back face
# Staging point: one robot diameter out from the mouth. Further out is calmer to
# drive but puts the robot into the room, where it met a dining table. Measured
# over twelve parked poses, every distance from 0.30 to 0.55 m docked 12/12; the
# cost of coming in close is final accuracy, 15.8 mm of lateral error against
# 9.2 mm from 0.55 m, both inside the 25.5 mm the bay allows.
STAGE_X_M = -0.35
# Arrive at the staging point tighter than the entry gate (25 mm): turning on
# the spot cannot fix a lateral offset, and from 0.35 m there is only 0.13 m of
# reversing left to correct one.
STAGE_TOL_M = 0.02
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


def scene(back_wall=False, wall_gap=0.0, chair=False, room=False):
    """
    Dock geometry in the DOCK frame, plus optional clutter around it.

    `room` builds a furnished room rather than a dock in a void: walls on three
    sides, a dining table with four legs and two chairs. The bare-dock scene
    flattered the detector -- in a real kitchen the scan is mostly furniture, and
    the search has to pick the dock out of it.
    """
    segs, circs = template()
    if back_wall or room:
        x = 0.325 + wall_gap
        segs = segs + [((x, -2.4), (x, 2.4))]
    if room:
        segs = segs + [((x - 3.4, -2.4), (x, -2.4)),        # side walls
                       ((x - 3.4, 2.4), (x, 2.4)),
                       ((x - 3.4, -2.4), (x - 3.4, 2.4))]   # far wall
        for cx, cy in ((-1.15, 0.55), (-1.15, 1.25), (-1.85, 0.55), (-1.85, 1.25)):
            circs = circs + [((cx, cy), 0.03)]              # dining table legs
        for cx, cy in ((-0.85, -0.9), (-0.85, -1.2), (-1.15, -0.9), (-1.15, -1.2)):
            circs = circs + [((cx, cy), 0.02)]              # a chair
        for cx, cy in ((-2.3, -0.5), (-2.3, -0.8), (-2.6, -0.5), (-2.6, -0.8)):
            circs = circs + [((cx, cy), 0.02)]              # another chair
    if chair:
        for dx, dy in ((0.15, 0.55), (0.15, 0.95), (0.55, 0.55), (0.55, 0.95)):
            circs = circs + [((dx, dy), 0.02)]
    return segs, circs


# Everything the robot can physically hit at scan height, in the dock frame:
# the two side plates and the back spacer. The dock's body sits above the robot.
# The back spacer is deliberately NOT here: the robot's tail is meant to reach
# it, that is how the charging contacts meet. Counting it as an obstacle scored
# eleven perfectly centred dockings as crashes.
OBSTACLES = [(0.0, 0.340, 0.200, 0.230),        # x0, x1, y0, y1 (right plate)
             (0.0, 0.340, -0.230, -0.200)]      # left plate
OVERSHOOT_M = 0.015                             # tail past the back face: shoving


def dock_contact(pose_in_dock):
    """
    Robot shell against any part of the dock, not just the bay walls.

    The first version only checked the bay, so a robot that clipped the OUTSIDE
    of a side plate while manoeuvring was scored as fine. That is exactly what
    was seen in Gazebo when starting close to the dock.
    """
    x, y, _th = pose_in_dock
    for x0, x1, y0, y1 in OBSTACLES:
        near_x = max(x0, min(x, x1))
        near_y = max(y0, min(y, y1))
        if math.hypot(x - near_x, y - near_y) < BODY_RADIUS_M:
            return True
    return False


ENTRY_GUARD_X = -0.22      # past this the bay walls are within reach
ENTRY_LATERAL_M = 0.025    # ...so only enter this well centred
ENTRY_YAW_DEG = 5.0
MAX_ATTEMPTS = 3


AGREE_M = 0.06             # two fits must land this close...
AGREE_DEG = 8.0            # ...and this well aligned, before either is believed
JUMP_M = 0.15              # a fit further than this from a FRESH estimate...
JUMP_DEG = 15.0            # ...is an impostor, not a correction


def agrees(a, b, tol_m=AGREE_M, tol_deg=AGREE_DEG):
    """
    Report whether two consecutive fits describe the same dock.

    A single scan can be fitted confidently in the wrong place -- laid along a
    wall, say -- and the robot then drives to a dock that is not there. Two
    independent scans agreeing is cheap insurance; a phantom rarely repeats.
    """
    return (math.hypot(a[0] - b[0], a[1] - b[1]) < tol_m
            and abs(wrap(a[2] - b[2])) < math.radians(tol_deg))


def goto_point(target, x, y, th, v_max=0.14, w_max=0.8, tol=0.04, prefer=None):
    """
    Drive to a point in the dock frame, forwards or backwards.

    Bidirectional on purpose: the staging point is often BEHIND the robot, and
    the first version could only drive forwards, so it drove a loop around the
    dock -- straight through it, in several starting poses.
    """
    dx, dy = target[0] - x, target[1] - y
    if math.hypot(dx, dy) < tol:
        return 0.0, 0.0, True
    err = wrap(math.atan2(dy, dx) - th)
    # hysteresis: near +-90 deg the cheaper direction flips every step, and the
    # robot dithers instead of driving
    limit = math.pi / 2 + (0.35 if prefer == 'fwd' else -0.35 if prefer == 'back'
                           else 0.0)
    backwards = abs(err) > limit
    if backwards:
        err = wrap(err - math.pi)
    w = max(-w_max, min(w_max, 1.8 * err))
    v = v_max * max(0.0, math.cos(err))
    return (-v if backwards else v), w, False


def control(state, x, y, th, attempts, prefer=None, stage_x=STAGE_X_M,
            stage_tol=STAGE_TOL_M):
    """
    One step of the docking state machine. Returns (v, w, state, attempts).

    x, y, th are the robot's believed pose in the dock frame; the bay mouth is
    at x = 0 and the robot reverses in along +x.
    """
    stage = (stage_x, 0.0)
    if state == 'GOTO':
        # too close to line up safely: pull out along the axis first
        target = stage if x < stage_x + 0.05 else (stage_x - 0.20, y)
        v, w, done = goto_point(target, x, y, th, prefer=prefer, tol=stage_tol)
        if done and target == stage:
            return 0.0, 0.0, 'TURN', attempts
        return v, w, state, attempts
    if state == 'TURN':
        err = wrap(th - math.pi)          # tail must point into the bay
        if abs(err) < math.radians(3.0):
            return 0.0, 0.0, 'BACK', attempts
        return 0.0, max(-0.7, min(0.7, -1.6 * err)), state, attempts
    if state == 'BACK':
        if seated((x, y, th)):
            return 0.0, 0.0, 'DONE', attempts
        # entry gate: do not cross into the bay unless genuinely lined up
        if x > ENTRY_GUARD_X and (abs(y) > ENTRY_LATERAL_M
                                  or abs(wrap(th - math.pi)) > math.radians(ENTRY_YAW_DEG)):
            attempts += 1
            return 0.0, 0.0, ('GIVE_UP' if attempts > MAX_ATTEMPTS else 'REGROUP'), attempts
        w = max(-0.5, min(0.5, -(2.5 * y + 1.8 * wrap(th - math.pi))))
        return -0.06, w, state, attempts
    if state == 'REGROUP':
        v, w, done = goto_point(stage, x, y, th, prefer=prefer, tol=stage_tol)
        return (0.0, 0.0, 'TURN', attempts) if done else (v, w, state, attempts)
    return 0.0, 0.0, state, attempts


SEAT_MARGIN_M = 0.015      # stop this far short and let contact close the gap


def seated(pose_in_dock):
    """
    Tail at the back face: the intended end of the manoeuvre.

    Stopping flush would be right if the estimate were exact. It is not, so the
    stop is 15 mm short: a robot that believes it is 10 mm out when it is flush
    would otherwise keep reversing and shove the dock across the floor.
    """
    return pose_in_dock[0] + BODY_RADIUS_M >= BAY_DEPTH_M - SEAT_MARGIN_M


def run(dist=0.75, bearing_deg=0.0, yaw_deg=0.0, seed=0, seconds=60.0,
        slip=0.02, back_wall=True, chair=False, room=False,
        detect_every=DETECT_EVERY, start=None, prior='fixed', debug=False,
        stage_x=STAGE_X_M, stage_tol=STAGE_TOL_M):
    """
    One docking attempt. Returns a metrics dict.

    `start` is (x, y, yaw) in the DOCK frame, which is how the grid drives this;
    otherwise the robot starts `dist` from the mouth, `bearing_deg` off the bay
    axis, with `yaw_deg` of heading error.

    `prior` picks what the robot starts out knowing:
      'none'  nothing at all: it must find the dock in the scan before it moves
      'near'  a hint within 0.1 m and 15 deg, as a beacon bearing or a recorded
              map position would give

    With no hint the robot HOLDS STILL until a fit is accepted, which is what
    dock_drive does. An earlier version let it drive on an unverified guess, and
    a robot with no fixes at all drove that guess straight into the dock.
    """
    random.seed(seed)
    fit = DockFitter()
    segs, circs = scene(back_wall=back_wall, chair=chair, room=room)
    if start is not None:
        t_w_b = (start[0], start[1], wrap(math.radians(start[2])))
    else:
        b = math.radians(bearing_deg)
        t_w_b = (-dist * math.cos(b), -dist * math.sin(b),
                 wrap(b + math.radians(yaw_deg)))
    if prior == 'near':
        est = mul(inv(t_w_b), (0.0, 0.0, 0.0))
        est = (est[0] + random.uniform(-0.1, 0.1),
               est[1] + random.uniform(-0.1, 0.1),
               wrap(est[2] + math.radians(random.uniform(-15.0, 15.0))))
    else:
        est = None                  # nothing known: hold still and look
    dt = 1.0 / SCAN_HZ
    state = 'GOTO'
    detects = 0
    attempts = 0
    prefer = None
    stale = 99                      # cycles since the last accepted fix
    pending = None                  # a fit waiting for a second opinion
    for step in range(int(seconds * SCAN_HZ)):
        if step % detect_every == 0:
            t_w_l = mul(t_w_b, T_BL)
            pts = scan(*place(segs, circs, inv(t_w_l)))
            # with a fresh fix, refine it; otherwise hunt the whole scan
            got = (fit.detect(pts, mul(inv(T_BL), est))
                   if est is not None and stale < 3 else fit.search(pts))
            if debug and step % (detect_every * 4) == 0:
                true_d = mul(inv(t_w_l), (0.0, 0.0, 0.0))
                print('   t=%4.1f %-7s %s cost=%.5f cover=%.2f side=%.2f intr=%2d err=%.3f'
                      % (step * dt, state,
                         'ACCEPT' if accept(got) else 'reject ',
                         got.cost if got else 9.9,
                         got.coverage if got else 0.0,
                         got.side_cover if got else 0.0,
                         got.intrusions if got else -1,
                         math.hypot(got.pose[0] - true_d[0],
                                    got.pose[1] - true_d[1]) if got else 9.9))
            if accept(got):
                cand = mul(T_BL, got.pose)
                if est is not None and stale < 3:
                    # tracking: a fit that jumps away from the running estimate
                    # is an impostor. The dock against a wall offers a second,
                    # cheaper fit half a metre aside, and it is stable, so two
                    # agreeing scans do not catch it -- but it arrives as a jump.
                    good = agrees(cand, est, JUMP_M, JUMP_DEG)
                else:
                    # lost: believe a fresh fit only when a second scan repeats it
                    good = pending is not None and agrees(cand, pending)
                if good:
                    est = cand
                    detects += 1
                    stale = 0
                else:
                    stale += 1
                pending = cand
            else:
                pending = None
                stale += 1
        if est is None:            # no fix yet: the robot does not move
            continue
        x, y, th = inv(est)
        v, w, state, attempts = control(state, x, y, th, attempts, prefer,
                                        stage_x=stage_x, stage_tol=stage_tol)
        prefer = 'back' if v < -1e-6 else 'fwd' if v > 1e-6 else prefer
        if state == 'DONE':
            break
        if state == 'GIVE_UP':
            break
        v_a = v * (1.0 + random.gauss(0.0, slip))
        w_a = w * (1.0 + random.gauss(0.0, slip)) + random.gauss(0.0, 0.004)
        t_w_b = mul(t_w_b, (v_a * dt, 0.0, w_a * dt))
        est = mul(inv((v * dt, 0.0, w * dt)), est)        # odometry, no slip
        true_pose = inv(mul(inv(t_w_b), (0.0, 0.0, 0.0)))
        if dock_contact(true_pose):
            return {'ok': False, 'why': 'hit the dock', 'state': state,
                    'pose': true_pose, 'lateral': true_pose[1],
                    'yaw_err': wrap(true_pose[2] - math.pi), 'detects': detects}
    pose = inv(mul(inv(t_w_b), (0.0, 0.0, 0.0)))
    yaw_err = wrap(pose[2] - math.pi)
    if detects == 0:
        return {'ok': False, 'why': 'never found the dock', 'state': state,
                'pose': pose, 'lateral': pose[1], 'yaw_err': yaw_err,
                'detects': 0}
    ok = (state == 'DONE'
          and abs(pose[1]) < BAY_HALF_M - BODY_RADIUS_M
          and abs(yaw_err) < math.radians(6.0))
    shoved = pose[0] + BODY_RADIUS_M > BAY_DEPTH_M + OVERSHOOT_M
    if shoved:
        ok = False
    why = ('' if ok else 'shoved the dock' if shoved
           else 'gave up' if state == 'GIVE_UP' else 'not seated')
    return {'ok': ok, 'why': why, 'state': state,
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


# A spread of parked poses for CI: distances, offsets, and orientations that
# include facing away from the dock and standing side-on to it.
CI_POSES = [(-1.10, -0.15, 0), (-1.10, 0.45, 180), (-0.80, -0.45, 90),
            (-0.80, 0.15, 0), (-0.80, -0.15, -90), (-0.55, 0.45, 180),
            (-0.55, -0.15, 0), (-0.55, 0.15, 90), (-0.35, -0.45, -90),
            (-0.35, 0.15, 180), (-0.35, -0.15, 0), (-0.35, 0.45, 90)]


def grid(xs=(-1.1, -0.8, -0.55, -0.35), ys=(-0.45, -0.15, 0.15, 0.45),
         yaws=(0, 90, 180, -90), prior='none', seconds=70.0, room=False):
    """
    Dock from a grid of starting poses, the way a person parks the robot.

    Returns a list of (x, y, yaw, metrics). Poses inside the dock's footprint are
    skipped. Yaw is measured in the dock frame: 180 deg faces the dock.
    """
    out = []
    for i, x in enumerate(xs):
        for j, y in enumerate(ys):
            for k, yaw in enumerate(yaws):
                if dock_contact((x, y, 0.0)):
                    continue
                m = run(seed=100 * i + 10 * j + k, start=(x, y, yaw),
                        prior=prior, seconds=seconds, room=room)
                out.append((x, y, yaw, m))
    return out


def print_grid(results):
    """One line per starting pose, plus a tally of how it went."""
    print('%7s %7s %6s %11s %9s %7s' %
          ('x', 'y', 'yaw', 'result', 'lateral', 'fixes'))
    tally = {}
    for x, y, yaw, m in results:
        why = 'docked' if m['ok'] else (m['why'] or m['state'])
        tally[why] = tally.get(why, 0) + 1
        print('%7.2f %7.2f %6d %11s %8.1f mm %7d'
              % (x, y, yaw, why, m['lateral'] * 1e3, m['detects']))
    print('\n%d runs: %s' % (len(results), ', '.join(
        '%s %d' % (k, v) for k, v in sorted(tally.items(), key=lambda t: -t[1]))))


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
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == 'grid':
        print_grid(grid(prior=sys.argv[2] if len(sys.argv) > 2 else 'none'))
    else:
        main()
