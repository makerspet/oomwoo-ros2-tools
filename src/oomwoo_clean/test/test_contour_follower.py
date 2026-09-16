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
    itself (omega = k_heading * e_b, so e_b = omega / k_heading ~ 18 deg). The
    robot therefore orbits nose-out, which costs clearance: a 2 cm leg measures
    ~0.178 m against a 0.1745 m body radius, i.e. about 3 mm to spare. The gate
    is only that it does not touch; closing that margin needs a control change,
    not a threshold change, and a curvature feed-forward has already been tried
    and measured WORSE (0.048 m).
    """
    world, start = harness.SCENARIOS[name]
    m = harness.run(world, start, seconds=40.0, seed=1)
    assert not m['hit'], '%s: min clearance %.3f m' % (name, m['min_clearance'])
    assert m['laps'] > 0.5, 'expected it to circle the leg, got %.1f laps' % m['laps']


@pytest.mark.xfail(reason='known: no clearance margin at a sharp convex '
                          'corner -- needs a control fix, not a threshold')
@pytest.mark.parametrize('name', ['wall_end', 'box'])
def test_sharp_convex_corners_have_no_clearance_margin(name):
    """
    Wrapping a sharp convex corner, the shell grazes what the LiDAR clears.

    Measured over 40 s: the bare end of a wall brings the body centre to
    0.169 m and a box corner to 0.174 m, against a 0.1745 m body radius. The
    box case is half a millimetre over the line -- which is the finding, not a
    tolerance to widen: there is NO margin at a sharp convex corner, and which
    side of contact a run lands on is luck. It matches the table-leg collision
    seen in Gazebo.

    The cause is that the follower servos the LIDAR's range to the surface, and
    the LiDAR sits 0.0745 m ahead of the wheel axle. Rounding a tight convex
    corner the body swings wide of where the LiDAR points, so a correct 0.20 m
    LiDAR standoff still lets the shell graze. Holding the BODY's clearance
    instead -- the distance from the body centre to the fitted curve, which the
    fitted conic can be evaluated at directly, no extra sensing needed -- is the
    fix this xfail is waiting for. A curvature feed-forward was tried first and
    measured worse (0.048 m), so it is not that.
    """
    world, start = harness.SCENARIOS[name]
    m = harness.run(world, start, seconds=40.0, seed=1)
    assert not m['hit'], '%s: min clearance %.3f m (body radius %.4f)' % (
        name, m['min_clearance'], harness.BODY_RADIUS_M)
