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
"""Tests for the cleaning manager's edge bookkeeping, offline and end to end."""

import math

import numpy as np

from oomwoo_clean import contour_harness as harness
from oomwoo_clean.edge_sweep import EdgeSweep

RES = 0.05
HALF = 1.5                                   # a 3 x 3 m room
LEG = ((0.4, 0.3), 0.02)                     # a 4 cm table leg in it
ROOM = [harness.segment((-HALF, -HALF), (HALF, -HALF)),
        harness.segment((HALF, -HALF), (HALF, HALF)),
        harness.segment((HALF, HALF), (-HALF, HALF)),
        harness.segment((-HALF, HALF), (-HALF, -HALF))]


def _sweep(world, **kw):
    x0 = y0 = -HALF - 0.25
    n = int(round(2 * (HALF + 0.25) / RES))
    grid = harness.rasterize(world, RES, x0, y0, n, n,
                             inside=lambda x, y: abs(x) < HALF and abs(y) < HALF)
    return EdgeSweep(np.array(grid), RES, (x0, y0), **kw)


def test_ring_sits_at_the_standoff_and_skips_a_gap_too_narrow():
    """Edge ring cells are a standoff off the surface; a 0.35 m slot has none."""
    slot = [harness.segment((-0.3, -HALF), (-0.3, -0.5)),
            harness.segment((0.05, -HALF), (0.05, -0.5))]
    sw = _sweep((ROOM + slot, []))
    rows, cols = np.nonzero(sw.ring)
    assert rows.size > 0
    assert np.all(np.abs(sw.surface[rows, cols] - 0.23) <= 0.04)
    for r, c in zip(rows, cols):
        x, y = sw.centre(r, c)
        assert not (-0.3 < x < 0.05 and y < -0.7), 'ring inside the slot at (%.2f, %.2f)' % (x, y)


def test_target_puts_the_obstacle_on_the_right():
    """The approach pose faces along the edge with the wall on the follow side."""
    sw = _sweep((ROOM, []))
    x, y, yaw = sw.next_target(0.0, -0.9)          # nearest edge: the south wall
    assert abs(y - (-HALF + 0.23)) < 0.05
    right = (math.cos(yaw - math.pi / 2), math.sin(yaw - math.pi / 2))
    assert right[1] < -0.95, 'right side points %s, not at the south wall' % (right,)


def test_nearest_edge_is_measured_along_the_floor():
    """A wall that is close as the crow flies but behind a partition loses."""
    part = [harness.segment((-HALF, 0.0), (1.0, 0.0))]     # gap only at the east end
    sw = _sweep((ROOM + part, []))
    x, y, _ = sw.next_target(-1.0, -0.3)
    assert y < 0.0, 'picked (%.2f, %.2f), across the partition' % (x, y)


def test_revisit_fires_one_lap_round_a_leg():
    """Circling a leg: no revisit during the first lap, then one shortly after."""
    sw = _sweep((ROOM, [LEG]))
    (cx, cy), r = LEG
    rad = r + 0.23
    lap = 2 * math.pi * rad
    step = 0.01
    fired_at = None
    for k in range(int(3 * lap / step)):
        a = k * step / rad
        if sw.update(cx + rad * math.cos(a), cy + rad * math.sin(a), True):
            fired_at = k * step
            break
    assert fired_at is not None, 'never noticed it was going round again'
    assert lap < fired_at < lap + 0.5, 'fired after %.2f m (one lap is %.2f m)' % (fired_at, lap)


def test_nav2_driving_does_not_count_as_sweeping():
    """Only the follower sweeps edges; being driven past one does not."""
    sw = _sweep((ROOM, []))
    for k in range(200):
        sw.update(-1.2 + k * 0.012, -HALF + 0.23, False)
    assert not sw.done.any()
    assert sw.cleaned.any()


def test_room_with_a_leg_is_swept_once_and_then_it_stops():
    """
    End to end: the loop sweeps the walls and the leg, and finishes by itself.

    Without a manager the follower that reaches the leg circles it forever;
    here the leg costs one lap, and the run ends when no dirty edge is left.
    """
    world = (ROOM, [LEG])
    out = harness.run_edge_clean(world, _sweep(world), (-1.0, -1.0, 0.0))
    assert out['edge_done_pct'] > 95.0, out
    assert out['order'].count('revisit') >= 2, out     # one per closed edge: walls, leg
    assert 'timeout' not in out['order'], out
    assert out['min_clearance'] > harness.CONTACT_RADIUS_M, out
    assert out['seconds'] < 300.0, out
