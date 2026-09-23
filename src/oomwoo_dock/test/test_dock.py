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
Regression gates for dock detection and the docking manoeuvre.

Thresholds are looser than the measured values: they exist to catch a
regression, not to pin down noise. The number that matters is the bay's capture
window, 25.5 mm of lateral error, which is what the manoeuvre has to land in.
"""

import math
import random

from oomwoo_dock import dock_harness as harness
from oomwoo_dock.dock_template import accept, DockFitter, inv, mul, wrap

import pytest

CAPTURE_M = harness.BAY_HALF_M - harness.BODY_RADIUS_M


def detect_once(dist, bearing_deg, yaw_deg, seed, back_wall=False, chair=False):
    """Scan a placed dock once and fit it; returns (error_m, yaw_error_deg)."""
    random.seed(seed)
    fit = DockFitter()
    segs, circs = harness.scene(back_wall=back_wall, chair=chair)
    b = math.radians(bearing_deg)
    truth = (dist * math.cos(b), dist * math.sin(b), b + math.radians(yaw_deg))
    pts = harness.scan(*harness.place(segs, circs, truth))
    prior = (truth[0] + random.uniform(-0.1, 0.1),
             truth[1] + random.uniform(-0.1, 0.1),
             wrap(truth[2] + math.radians(random.uniform(-15.0, 15.0))))
    got = fit.detect(pts, prior)
    assert got is not None, 'no fit at %.2f m' % dist
    assert accept(got), 'fit rejected at %.2f m: cost %.5f, coverage %.2f' % (
        dist, got.cost, got.coverage)
    pose = got.pose
    return (math.hypot(pose[0] - truth[0], pose[1] - truth[1]),
            abs(math.degrees(wrap(pose[2] - truth[2]))))


@pytest.mark.parametrize('dist', [0.35, 0.6, 0.9])
def test_detects_the_dock_accurately(dist):
    """A single scan must place the mouth well inside the capture window."""
    for seed in range(4):
        err, yaw = detect_once(dist, 0.0, 0.0, seed)
        assert err < 0.015, '%.2f m: %.1f mm' % (dist, err * 1e3)
        assert yaw < 6.0, '%.2f m: %.1f deg' % (dist, yaw)


def test_survives_the_wall_behind_the_dock():
    """
    The dock stands against a wall, which the template does not model.

    The wall roughly triples the point count. The fit has to ignore what it
    cannot explain rather than being dragged by it.
    """
    for seed in range(4):
        err, yaw = detect_once(0.6, 0.0, 0.0, seed, back_wall=True)
        assert err < 0.02, 'wall behind: %.1f mm' % (err * 1e3)
        assert yaw < 8.0, 'wall behind: %.1f deg' % yaw


def test_survives_furniture_beside_the_dock():
    """Chair legs next to the dock must not capture the fit."""
    for seed in range(4):
        err, yaw = detect_once(0.6, 15.0, 0.0, seed, back_wall=True, chair=True)
        assert err < 0.025, 'with chair: %.1f mm' % (err * 1e3)


def test_inlier_only_scoring_would_be_degenerate():
    """
    Guards the choice of cost function, which is the heart of the fit.

    Averaging distance over inliers alone lets a badly placed template score
    well by dropping the points it cannot explain. The truncated cost used by
    DockFitter must prefer the truth by a wide margin instead.
    """
    random.seed(7)
    fit = DockFitter()
    segs, circs = harness.scene()
    truth = (0.6, 0.0, 0.0)
    pts = harness.scan(*harness.place(segs, circs, truth))
    cost_true, _rms_true, n_true = fit.score(truth, _as_array(pts))
    cost_off, _rms_off, n_off = fit.score((0.4, 0.0, 0.0), _as_array(pts))
    assert cost_off > 5.0 * cost_true, 'cost does not separate a 0.2 m error'
    assert n_true > n_off, 'the true pose should explain more points'


def _as_array(pts):
    import numpy as np
    return np.asarray(pts, dtype=float).reshape(-1, 2)


@pytest.mark.parametrize('name,geom', [
    ('bare wall', ([((0.6, -1.5), (0.6, 1.5))], [])),
    ('wall corner', ([((0.6, -1.5), (0.6, 0.2)), ((0.6, 0.2), (1.6, 0.2))], [])),
])
def test_rejects_scenes_with_no_dock(name, geom):
    """
    Furniture must not be reported as a dock.

    Cost alone does NOT catch this: a bare wall fits the template at 0.00166
    against 0.00170 for a real dock standing against a wall, because a wall
    explains the template's flat faces and merely leaves the bay unaccounted
    for. Coverage is what separates them, measured at 0.28 against 0.69.
    """
    random.seed(11)
    fit = DockFitter()
    pts = harness.scan(*geom)
    got = fit.detect(pts, (0.6, 0.0, 0.0))
    assert not accept(got), \
        '%s accepted as a dock: cost %.5f, coverage %.2f' % (
            name, got.cost, got.coverage)


@pytest.mark.parametrize('index,name,kw',
                         [(i, n, k) for i, (n, k) in enumerate(harness.SCENARIOS)])
def test_docks_from_every_start(index, name, kw):
    """
    Drive the whole manoeuvre; it must seat without touching the dock.

    The prior stands in for the dock's IR beacon or its recorded map position,
    which the robot will have in service. Docking with NO prior at all is a
    harder problem and has its own test below.

    The seed is the scenario's index, not hash(name): Python randomises string
    hashes per process, so that made this test pass or fail depending on the run.
    The time budget allows for a regroup, which the manoeuvre is entitled to do
    when it arrives at the mouth misaligned.
    """
    m = harness.run(seed=index, seconds=90.0, prior='near', **kw)
    assert m['ok'], '%s: %s (lateral %.1f mm, yaw %.2f deg)' % (
        name, m['why'], m['lateral'] * 1e3, math.degrees(m['yaw_err']))
    assert abs(m['lateral']) < CAPTURE_M, '%s: lateral %.1f mm of %.1f mm' % (
        name, m['lateral'] * 1e3, CAPTURE_M * 1e3)


def test_pose_algebra_round_trips():
    """Pose algebra must round-trip; the manoeuvre lives or dies on this."""
    a = (0.4, -0.2, 0.7)
    b = (-0.1, 0.25, -1.2)
    ident = mul(a, inv(a))
    assert max(abs(ident[0]), abs(ident[1]), abs(ident[2])) < 1e-12
    there_and_back = mul(inv(b), mul(b, a))
    for got, want in zip(there_and_back, a):
        assert abs(got - want) < 1e-12


def test_docks_from_parked_poses_without_any_prior():
    """
    Park the robot anywhere near the dock, facing anywhere, with no hint at all.

    With nothing known the robot holds still until it has found the dock, so the
    poses it cannot see from simply do not dock. What must never happen is that
    it moves on a guess: a robot with no fixes once drove the fallback prior
    straight into the dock.

    This is the case a grid of starting poses exposed and a fixed "the dock is
    0.6 m dead ahead" prior could not survive: it docked 3 times in 64. Hunting
    the whole scan for the dock, refusing to enter the mouth unless lined up, and
    driving to the staging point backwards when it lies behind, took the same
    grid to 58. Measured on these twelve poses in a FURNISHED room -- walls, a
    dining table and two chairs, so the scan is mostly furniture: all twelve
    dock, none touch the dock, worst lateral error 12.3 mm of the 25.5 mm the
    bay allows.

    The gate is deliberately below the measured figure. A robot that can do this
    with no beacon at all has margin to spare once the beacon is fitted.
    """
    docked = 0
    for i, pose in enumerate(harness.CI_POSES):
        m = harness.run(seed=i, start=pose, prior='none', seconds=90.0, room=True)
        assert m['why'] != 'hit the dock', 'hit the dock from %s' % (pose,)
        assert m['why'] != 'shoved the dock', 'shoved the dock from %s' % (pose,)
        docked += m['ok']
    assert docked >= 9, 'only %d of %d parked poses docked' % (
        docked, len(harness.CI_POSES))


def test_docks_from_parked_poses_with_a_beacon():
    """
    The same parked poses, with the hint a beacon or map position provides.

    Measured in the furnished room: 12 of 12, none touching the dock, worst
    lateral error 15.2 mm. A hint mainly buys TIME -- without one the robot sits
    and searches until it recognises the dock, which from some parked poses takes
    several seconds of looking.
    """
    docked = 0
    for i, pose in enumerate(harness.CI_POSES):
        m = harness.run(seed=i, start=pose, prior='near', seconds=90.0, room=True)
        assert m['why'] != 'hit the dock', 'hit the dock from %s' % (pose,)
        docked += m['ok']
    assert docked >= 11, 'only %d of %d parked poses docked' % (
        docked, len(harness.CI_POSES))
