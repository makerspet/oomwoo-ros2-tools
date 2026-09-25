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
Regression gates for the contour follower, open loop and closed loop.

The thresholds are deliberately looser than the measured values -- they exist to
catch a regression, not to pin down noise. Where a number is uncomfortably close
to the bound it says so, because that is a finding rather than a passing test.
"""

import math
import random
from types import SimpleNamespace
from unittest.mock import MagicMock

from oomwoo_clean import contour_harness as harness

import pytest

SMIN = math.radians(-170.0)
SMAX = math.radians(20.0)


def bearing_stats(world, trials=60, seed=0, **overrides):
    """Estimate bearing mean/spread (deg) and mean distance, robot at the origin."""
    node, params = harness.make_follower(**overrides)
    random.seed(seed)
    bs, ds = [], []
    for _ in range(trials):
        segs, circs = world
        d, b, _n = node._boundary(harness.Scan(harness.scan(segs, circs)), SMIN, SMAX,
                                  params['max_follow_range_m'])
        if d is None:
            continue
        bs.append(math.degrees(b))
        ds.append(d)
    n = max(1, len(bs))
    mean = sum(bs) / n
    sd = math.sqrt(sum((v - mean) ** 2 for v in bs) / n)
    return mean, sd, sum(ds) / n, max(abs(v - mean) for v in bs)


def test_wall_is_measured_accurately():
    """A flat wall abeam: the whole point of fitting rather than picking a beam."""
    mean, sd, dist, _worst = bearing_stats(
        ([harness.segment((-3.0, -0.2), (3.0, -0.2))], []))
    assert abs(mean + 90.0) < 2.0        # measured -90.0
    assert sd < 2.0                      # measured 0.7; a raw nearest beam gives 7
    assert abs(dist - 0.20) < 0.01       # measured 0.200


def test_inside_corner_does_not_pull_the_estimate_early():
    """
    An inside corner must not tilt the fit until the robot is at the standoff.

    This is the regression that a whole-surface line fit failed: with no gap in
    the scan it fitted one line across both walls, read ~10 deg tilted, and
    turned the robot away about 0.8 m early.
    """
    for front in (0.8, 0.5, 0.3):
        world = ([harness.segment((-3.0, -0.2), (front, -0.2)),
                  harness.segment((front, -0.2), (front, 3.0))], [])
        mean, _sd, dist, _worst = bearing_stats(world, trials=40)
        assert abs(mean + 90.0) < 4.0, 'corner at %.2f pulled the fit' % front
        assert abs(dist - 0.20) < 0.02


def test_round_obstacles_are_tracked():
    """A table leg is a circle, not a wall; a line fit measured +-15 deg here."""
    for radius in (0.02, 0.03, 0.15):
        world = ([], [harness.circle((0.0, -(0.20 + radius)), radius)])
        mean, sd, dist, _worst = bearing_stats(world, trials=40)
        assert abs(mean + 90.0) < 5.0, 'R=%.2f bearing off' % radius
        assert sd < 6.0, 'R=%.2f bearing noisy' % radius
        assert abs(dist - 0.20) < 0.02, 'R=%.2f distance off' % radius


def test_fit_blowups_are_rejected():
    """
    A short arc can fit a tiny circle nowhere near the surface; reject those.

    Without the seed cross-check this reached 99 deg of bearing error on roughly
    1% of frames, and one such frame commands a full-rate turn.
    """
    random.seed(3)
    node, params = harness.make_follower()
    worst = 0.0
    for _ in range(200):
        ang = random.uniform(-math.pi / 2 - 0.6, -math.pi / 2 + 0.6)
        dist = 0.22
        world = ([], [harness.circle((dist * math.cos(ang), dist * math.sin(ang)), 0.02)])
        d, b, _n = node._boundary(harness.Scan(harness.scan(*world)), SMIN, SMAX,
                                  params['max_follow_range_m'])
        if d is None:
            continue
        worst = max(worst, abs(math.degrees(math.remainder(b - ang, 2.0 * math.pi))))
    # The guard accepts a fit within fit_max_dev_deg of the seed beam, and the
    # seed beam itself carries a few degrees of noise, so that sum is the real
    # bound. What matters is that it is BOUNDED -- unguarded it was 99 deg.
    bound = params['fit_max_dev_deg'] + 10.0
    assert worst < bound, 'worst bearing error %.1f deg (bound %.0f)' % (worst, bound)


def test_degenerate_scans_do_not_crash():
    """Empty and near-empty scans return cleanly rather than throwing."""
    node, _params = harness.make_follower()
    assert node._boundary(harness.Scan([float('inf')] * harness.BEAMS), SMIN, SMAX, 1.0) \
        == (None, None, 0)
    sparse = [float('inf')] * harness.BEAMS
    for i in (100, 101, 102):
        sparse[i] = 0.5
    d, b, n = node._boundary(harness.Scan(sparse), SMIN, SMAX, 1.0)
    assert d is not None and n == 3          # falls back to the seed beam
    assert node._boundary(harness.Scan([]), SMIN, SMAX, 1.0) == (None, None, 0)


@pytest.mark.parametrize('name', ['room', 'corridor', 'leg_by_wall'])
def test_closed_loop_does_not_collide(name):
    """Drive the real control law around each scene and check it never hits."""
    world, start = harness.SCENARIOS[name]
    m = harness.run(world, start, seconds=40.0, seed=1)
    assert not m['hit'], '%s: min clearance %.3f m (body radius %.4f)' % (
        name, m['min_clearance'], harness.BODY_RADIUS_M)
    assert m['standoff_mean'] < 0.06, '%s: standoff error %.3f m' % (
        name, m['standoff_mean'])


@pytest.mark.parametrize('name', ['table_leg_2cm', 'table_leg_5cm'])
def test_orbiting_a_leg_keeps_its_distance(name):
    """
    Circling a table leg: the case with the least margin.

    Proportional control leaves a steady bearing droop on a curve -- the turn
    command has to come from somewhere, and the only source is the bearing error
    itself (omega = k_heading * e_b, so e_b = omega / k_heading ~ 18 deg), so the
    robot orbits nose-out. That used to cost most of the clearance: a 2 cm leg
    measured ~0.178 m against a 0.1745 m body radius, about 3 mm to spare.
    Measuring the standoff at the body centre absorbs the droop geometrically and
    it now measures ~0.196 m. The droop itself is still there and still visible in
    the logs -- two things that did NOT fix it, both measured worse, were a
    curvature feed-forward (0.048 m) and capping speed in turns (0.083 m).
    """
    world, start = harness.SCENARIOS[name]
    m = harness.run(world, start, seconds=40.0, seed=1)
    assert not m['hit'], '%s: min clearance %.3f m' % (name, m['min_clearance'])
    assert m['laps'] > 0.5, 'expected it to circle the leg, got %.1f laps' % m['laps']


@pytest.mark.parametrize('name', ['wall_end', 'box'])
def test_sharp_convex_corners_keep_clearance(name):
    """
    Wrapping a sharp convex corner, the shell must clear what the LiDAR clears.

    Three fixes got this passing, each correcting the last. Servoing the raw
    LiDAR range, a wall's bare end brought the body centre to 0.169 m and a box
    corner to 0.174 m. Measuring at the body centre gave 0.181 and 0.180 m, which
    was scored clear against the 0.1745 m body radius but not against the
    bumper's 0.1814 m reach, and a Gazebo run duly halted. Raising the standoff
    to 0.23 m and refusing to report a surface as further away than the nearest
    scan points now leaves at least 35 mm.
    """
    world, start = harness.SCENARIOS[name]
    m = harness.run(world, start, seconds=40.0, seed=1)
    assert not m['hit'], '%s: min clearance %.3f m (bumper reach %.4f)' % (
        name, m['min_clearance'], harness.CONTACT_RADIUS_M)


def test_body_measure_tracks_the_lidar_measure_when_parallel():
    """
    Parallel to a wall the two measures agree; angled, they differ by the offset.

    This is the invariant that keeps the body-centre measure from altering plain
    wall following: the body centre sits directly behind the LiDAR, so with the
    surface abeam both are the same distance from it. Angled, they separate by
    offset * cos(bearing) -- the body reads CLOSER when the surface has drifted
    behind abeam (what happens on every curve, and the case that was grazing
    corners) and FURTHER when the robot is angled into the wall.

    The point guard only ever subtracts, so each expected value is an upper
    bound, with about 1.5 cm of slack below it.
    """
    node, params = harness.make_follower()
    off = params['body_offset_m']
    for normal_deg in (-90.0, -110.0, -70.0):
        nrm = math.radians(normal_deg)
        nx, ny = math.cos(nrm), math.sin(nrm)
        mid = (0.20 * nx, 0.20 * ny)
        tan = (-ny, nx)
        world = ([harness.segment((mid[0] - 1.5 * tan[0], mid[1] - 1.5 * tan[1]),
                                  (mid[0] + 1.5 * tan[0], mid[1] + 1.5 * tan[1]))], [])
        random.seed(5)
        d_ctrl, _b, _n = node._boundary(harness.Scan(harness.scan(*world)),
                                        SMIN, SMAX, params['max_follow_range_m'])
        delta = d_ctrl - node._dbg_d            # _dbg_d stays the raw LiDAR range
        expected = off * math.cos(nrm)          # the geometry, before the guard
        assert delta <= expected + 0.005, (
            'normal %.0f deg: %+.3f m, further than the geometry allows (%+.3f)'
            % (normal_deg, delta, expected))
        assert delta >= expected - 0.015, (
            'normal %.0f deg: %+.3f m, more than the guard should cost (%+.3f)'
            % (normal_deg, delta, expected))


def test_point_guard_stops_the_fit_cutting_a_sharp_corner():
    """
    A fitted circle rounds a sharp corner; the corner tip must still be reported.

    Wrapping a box corner the fit reports R ~ 0.08 and, being an arc through a
    corner that is not one, sits INSIDE the real corner: measured up to +23 mm of
    optimism around a box and +54 mm in a room's inside corners, which is most of
    the clearance the standoff buys. A Gazebo run halted on exactly this, with
    "R=0.08 convex" in the log line. Reporting no further than the 3rd-nearest
    scan point cures it, because the corner tip is one of the points.
    """
    node, params = harness.make_follower()
    corner = (-0.10, -0.23)                     # ahead-right of the robot
    faces = ([harness.segment(corner, (2.0, -0.23)),
              harness.segment(corner, (-0.10, -2.0))], [])
    bx = -params['body_offset_m']
    true = min(0.23, math.hypot(bx - corner[0], corner[1]))
    random.seed(7)
    worst = max(node._boundary(harness.Scan(harness.scan(*faces)), SMIN, SMAX,
                               params['max_follow_range_m'])[0] - true
                for _ in range(30))
    assert worst < 0.015, 'reports the corner %+.3f m further away than it is' % worst


def _curve_clearance(name, seconds, on_curve, **overrides):
    """Mean true body-centre clearance while on the curve, and the minimum overall."""
    world, start = harness.SCENARIOS[name]
    node, p = harness.make_follower(**overrides)
    x, y, th = start
    dt = 1.0 / harness.SCAN_HZ
    random.seed(1)
    smin, smax = math.radians(p['sector_min_deg']), math.radians(p['sector_max_deg'])
    b_ref = math.radians(p['bearing_ref_deg'])
    on, low = [], float('inf')
    for k in range(int(seconds / dt)):
        lx = x + harness.LIDAR_OFFSET_M * math.cos(th)
        ly = y + harness.LIDAR_OFFSET_M * math.sin(th)
        segs, circs = harness.to_robot(world, lx, ly, th)
        d, b, _n = node._boundary(harness.Scan(harness.scan(segs, circs)), smin, smax,
                                  p['max_follow_range_m'])
        if d is None:
            v, w = p['v_min'], -p['v_nominal'] / p['convex_arc_radius_m']
        else:
            v, w, *_ = node._command(d, b, b_ref, dt)
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        th += w * dt
        c = harness.clearance(world, x, y)
        if k * dt > 5.0:
            low = min(low, c)
            if on_curve(x, y):
                on.append(c)
    return sum(on) / len(on), low


@pytest.mark.parametrize('name, on_curve', [
    ('table_leg_2cm', lambda x, y: True),
    ('table_leg_5cm', lambda x, y: True),
    ('concave_bay', lambda x, y: abs(x) < 0.30 and y < -0.39),
])
def test_curves_are_held_near_the_standoff(name, on_curve):
    """
    On curves the default law holds within ~3 cm of the standoff, never inward.

    With the bearing taken at the LiDAR, the offset geometry (a tangent robot
    sees a curve's nearest point off abeam) and the proportional lag (it must
    hold an error to keep turning) nearly cancel. Measured, with the point
    guard's noise margin: the 2 cm leg +0.6 cm, the R 0.35 bay +1.7 cm, against
    a straight wall's +0.1 cm. This pins that, because the textbook correction --
    bearing at the body centre plus curvature feed-forward -- was measured to
    put the robot INTO the bay's wall.
    """
    mean, low = _curve_clearance(name, 40.0, on_curve)
    standoff = harness.make_follower()[1]['standoff_m']
    assert -0.01 < mean - standoff < 0.03, '%s: %.3f m on the curve' % (name, mean)
    assert low > harness.CONTACT_RADIUS_M, '%s: touched, min %.3f m' % (name, low)


@pytest.mark.xfail(strict=True, reason='known: no front guard -- the follower only '
                                       'steers on the nearest surface')
def test_obstacle_in_the_path_is_avoided():
    """
    A post in the robot's path, nearer the centreline than the wall, gets hit.

    The follower picks the single nearest surface in its search sector. While the
    followed wall sits at the standoff, a post further ahead is never the nearest,
    however squarely it sits in the path; by the time it would be, it has swung
    past the sector's +20 deg edge and drops out of view entirely. Measured on the
    torture course's first version: a panel end in view at +5 deg to +16 deg for
    five seconds, never picked, hit at +43 deg.

    Strict, so it flips loudly the day a front guard lands.
    """
    world, start = harness.SCENARIOS['post_in_path']
    m = harness.run(world, start, seconds=20.0, seed=1)
    assert not m['hit'], 'min clearance %.3f m' % m['min_clearance']


def _bump_ready(state, **overrides):
    """Build a follower with its ROS plumbing stubbed, sitting in the given state."""
    node, _params = harness.make_follower(**overrides)
    node.state = state
    node.enabled = True
    node.cmd = SimpleNamespace(linear=SimpleNamespace(x=0.15),
                               angular=SimpleNamespace(z=-0.4))
    node.cmd_pub = MagicMock()
    node.state_pub = MagicMock()
    node.active_pub = MagicMock()
    node._active_val = True
    node.get_logger = MagicMock()
    return node


def test_bump_halts_an_active_follower():
    """Any contact while following stops the robot at once and parks it."""
    node = _bump_ready('FOLLOW')
    node._on_bump(SimpleNamespace(contacts=[object()]), 'left')
    assert node.state == 'HALTED'
    assert node.cmd.linear.x == 0.0 and node.cmd.angular.z == 0.0
    node.cmd_pub.publish.assert_called_once()        # sent now, not next tick
    assert node._active_val is False                 # no longer counts as cleaning
    node.get_logger().warn.assert_called_once()


def test_empty_contact_messages_and_idle_states_are_ignored():
    """The bumper topics publish empty lists too; and a parked robot stays parked."""
    node = _bump_ready('FOLLOW')
    node._on_bump(SimpleNamespace(contacts=[]), 'right')
    assert node.state == 'FOLLOW' and node.cmd.linear.x == 0.15
    for parked in ('IDLE', 'LOST', 'HALTED'):
        node = _bump_ready(parked)
        node._on_bump(SimpleNamespace(contacts=[object()]), 'right')
        assert node.state == parked
        node.cmd_pub.publish.assert_not_called()


def test_halt_on_bump_can_be_switched_off():
    """halt_on_bump:=false keeps following, for A/B runs -- but still logs the bump."""
    node = _bump_ready('FOLLOW', halt_on_bump=False)
    node._on_bump(SimpleNamespace(contacts=[object()]), 'left')
    assert node.state == 'FOLLOW'
    node.cmd_pub.publish.assert_not_called()
    node.get_logger().warn.assert_called_once()
    assert 'BUMP (left bumper)' in node.get_logger().warn.call_args[0][0]


def test_each_bump_is_logged_once_not_once_per_message(monkeypatch):
    """
    Gazebo sends a contact message every physics step; log the start of a bump.

    A held bumper is one bump. The same side counts as bumped again only after
    it has been clear for bump_quiet_s, and each side is tracked separately.
    """
    from oomwoo_clean import contour_follower_node as cfn
    clock = [100.0]
    monkeypatch.setattr(cfn.time, 'monotonic', lambda: clock[0])
    node = _bump_ready('FOLLOW', halt_on_bump=False)
    hit = SimpleNamespace(contacts=[object()])
    for _ in range(50):                      # held for a second: one bump
        node._on_bump(hit, 'right')
        clock[0] += 0.02
    assert node.get_logger().warn.call_count == 1
    node._on_bump(hit, 'left')               # the other side is its own bump
    assert node.get_logger().warn.call_count == 2
    clock[0] += 1.0                          # clear for longer than bump_quiet_s
    node._on_bump(hit, 'right')
    assert node.get_logger().warn.call_count == 3


def test_enable_resumes_from_halted():
    """Publishing true on ~/enable picks the robot back up with a fresh ALIGN."""
    node = _bump_ready('HALTED')
    node.prev_d = 0.3
    node.arc_swept = 1.0
    node._on_enable(SimpleNamespace(data=True))
    assert node.state == 'ALIGN'
    assert node.prev_d is None and node.arc_swept == 0.0


def test_point_guard_does_not_tax_straight_walls():
    """
    On a smooth wall the guard must not win, or it holds the robot out by its noise.

    The 3rd-nearest of ~60 noisy points sits ~1 cm closer than the wall really
    is. Without a margin the guard beat the unbiased fit on every frame and the
    robot held 9.9 mm out along every straight wall; with it, ~1 mm.
    """
    world = ([harness.segment((-6.0, -0.6), (6.0, -0.6))], [])
    node, p = harness.make_follower()
    x, y, th = -5.0, -0.6 + p['standoff_m'], 0.0
    dt = 1.0 / harness.SCAN_HZ
    random.seed(1)
    b_ref = math.radians(p['bearing_ref_deg'])
    clears = []
    for k in range(int(40.0 / dt)):
        lx = x + harness.LIDAR_OFFSET_M * math.cos(th)
        ly = y + harness.LIDAR_OFFSET_M * math.sin(th)
        segs, circs = harness.to_robot(world, lx, ly, th)
        d, b, _n = node._boundary(harness.Scan(harness.scan(segs, circs)), SMIN, SMAX,
                                  p['max_follow_range_m'])
        v, w, *_ = node._command(d, b, b_ref, dt)
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        th += w * dt
        if k * dt > 5.0:
            clears.append(harness.clearance(world, x, y))
    offset = sum(clears) / len(clears) - p['standoff_m']
    assert abs(offset) < 0.004, 'straight wall held %+.1f mm off the standoff' % (offset * 1000)
