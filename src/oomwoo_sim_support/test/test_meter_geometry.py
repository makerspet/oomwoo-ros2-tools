# Copyright 2026 Jayadev Rana
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
Unit tests for the coverage meter's grid geometry.

The load-bearing property here is DENOMINATOR MONOTONICITY: the serviceable
denominator built from a smaller robot radius must never be smaller than the
one built from a larger radius. This is the regression test for the
meter/planner radius-coupling review finding — with the meter pinned to the
true body radius, no planner clearance choice can shrink the denominator and
inflate the score.
"""

import numpy as np

from oomwoo_sim_support.coverage_meter_node import (
    _dilate,
    _flood_fill,
    _nearest_free,
    _stamp_disk,
)


def _room_with_alcove():
    """Build a walled room with a 3-cell-wide slot only a slim robot enters."""
    free = np.zeros((30, 30), dtype=bool)
    free[1:29, 1:29] = True                 # room interior, 1-cell wall ring
    free[10:20, 10:20] = False              # solid block in the middle
    free[13:16, 10:20] = True               # 3-cell-wide slot through the block
    return free


def _denominator(free, r_robot, r_clean, edge_margin):
    """Mirror coverage_meter_node._ensure_reachable's serviceable-cell formula."""
    start = _nearest_free(free, 2, 2)
    reach = _flood_fill(free, start)
    drivable = reach & ~_dilate(~free, r_robot)
    cleanable = _dilate(drivable, r_clean) & reach
    if edge_margin > 0:
        cleanable &= ~_dilate(~free, edge_margin)
    return cleanable


def test_denominator_monotone_in_robot_radius():
    """A more timid robot radius must never shrink the denominator."""
    free = _room_with_alcove()
    prev = None
    for r in (6, 5, 4, 3, 2, 1):            # shrinking robot radius
        cur = _denominator(free, r_robot=r, r_clean=4, edge_margin=3)
        if prev is not None:
            # strict superset-or-equal, cell by cell — not just the count
            assert bool(np.all(cur[prev])), \
                'shrinking robot_radius must never remove serviceable cells'
            assert int(cur.sum()) >= int(prev.sum())
        prev = cur


def test_denominator_alcove_counts_only_for_slim_robot():
    """The 3-cell slot is serviceable for a slim robot and unreachable fat."""
    free = _room_with_alcove()
    slim = _denominator(free, r_robot=1, r_clean=2, edge_margin=0)
    fat = _denominator(free, r_robot=4, r_clean=2, edge_margin=0)
    slot = np.zeros_like(free)
    slot[14, 12:18] = True                   # slot centerline
    assert bool((slim & slot).any()), 'slim robot must service the slot'
    assert int(slim.sum()) > int(fat.sum())


def test_dilate_grows_and_preserves_input():
    seed = np.zeros((9, 9), dtype=bool)
    seed[4, 4] = True
    grown = _dilate(seed, 2)
    assert bool(grown[4, 4]) and bool(grown[2, 4]) and bool(grown[4, 6])
    assert not bool(grown[1, 4]), 'radius-2 dilation must not reach distance 3'
    assert int(seed.sum()) == 1, 'input mask must not be mutated'


def test_flood_fill_respects_walls():
    free = np.ones((7, 7), dtype=bool)
    free[:, 3] = False                       # full-height wall
    filled = _flood_fill(free, (0, 0))
    assert bool(filled[6, 2]) and not bool(filled[0, 4]), \
        'flood fill must not cross a solid wall'


def test_nearest_free_finds_closest_cell():
    free = np.zeros((10, 10), dtype=bool)
    free[7, 7] = True
    assert _nearest_free(free, 2, 2, max_r=3) is None
    assert _nearest_free(free, 7, 6) == (7, 7)


def test_stamp_disk_masks_walls():
    covered = np.zeros((11, 11), dtype=bool)
    mask = np.ones((11, 11), dtype=bool)
    mask[:, 6:] = False                      # wall from column 6 on
    _stamp_disk(covered, mask, cx=5, cy=5, rad=3)
    assert bool(covered[5, 5]) and bool(covered[5, 3])
    assert not covered[:, 6:].any(), 'disk must never stamp non-free cells'
