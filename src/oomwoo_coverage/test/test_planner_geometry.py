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
"""Unit tests for the boustrophedon planner's grid helpers and reactive tick."""

from types import SimpleNamespace

import numpy as np

from oomwoo_coverage.coverage_planner_node import (
    _contiguous_runs,
    _decompose_cells,
    _dilate,
    _flood_fill,
    _nearest_true,
    _pt_in_zones,
)


def test_pt_in_zones_inside_and_outside():
    zones = [(1.0, 1.0, 0.35), (-2.0, 0.5, 0.5)]
    assert _pt_in_zones(1.1, 0.9, zones)          # inside first pocket
    assert _pt_in_zones(-2.0, 0.5, zones)         # dead centre of second
    assert not _pt_in_zones(1.5, 1.5, zones)      # between, outside both
    assert not _pt_in_zones(0.0, 0.0, [])         # no zones -> never inside


def test_pt_in_zones_boundary_is_inclusive():
    # a point exactly on the radius counts as inside (<=), so a wedge spot
    # can't slip back in on a floating-point tie
    assert _pt_in_zones(0.35, 0.0, [(0.0, 0.0, 0.35)])


def test_contiguous_runs_splits_on_gaps():
    runs = _contiguous_runs(np.array([1, 2, 3, 7, 8, 12]))
    assert [list(r) for r in runs] == [[1, 2, 3], [7, 8], [12]]


def test_contiguous_runs_empty_and_single():
    assert _contiguous_runs(np.array([], dtype=int)) == []
    assert [list(r) for r in _contiguous_runs(np.array([5]))] == [[5]]


def test_dilate_zero_radius_returns_copy():
    mask = np.zeros((5, 5), dtype=bool)
    mask[2, 2] = True
    out = _dilate(mask, 0)
    assert bool(np.array_equal(out, mask))
    out[0, 0] = True
    assert not bool(mask[0, 0]), 'radius-0 dilation must return a copy'


def test_dilate_radius_reaches_manhattan_ball():
    mask = np.zeros((9, 9), dtype=bool)
    mask[4, 4] = True
    out = _dilate(mask, 2)
    assert bool(out[2, 4]) and bool(out[4, 2]) and bool(out[3, 3])
    assert not bool(out[1, 4])


def test_flood_fill_two_chambers_stay_separate():
    mask = np.ones((6, 8), dtype=bool)
    mask[:, 4] = False                       # wall splits the room in two
    left = _flood_fill(mask, (0, 0))
    assert bool(left[5, 3]) and not left[:, 5:].any()


def test_decompose_open_room_is_one_cell():
    free = np.zeros((20, 20), dtype=bool)
    free[1:19, 1:19] = True
    cells = _decompose_cells(free)
    assert len(cells) == 1
    assert len(cells[0]) == 18                  # every interior row present


def test_decompose_sofa_splits_into_four_cells():
    # a central obstacle (the "sofa") must yield: below, left-of, right-of,
    # above — four cells, each with exactly ONE interval per row, so a
    # serpentine can sweep each without transiting around the obstacle
    free = np.zeros((30, 30), dtype=bool)
    free[1:29, 1:29] = True
    free[10:20, 10:20] = False                  # the sofa
    cells = _decompose_cells(free)
    assert len(cells) == 4
    for cell in cells:
        rows = [r for r, _, _ in cell]
        assert len(rows) == len(set(rows)), 'one interval per row per cell'
    total = sum(b - a + 1 for cell in cells for _, a, b in cell)
    assert total == int(free.sum()), 'decomposition must cover all free cells'


def test_decompose_two_obstacles_seven_cells():
    free = np.zeros((40, 40), dtype=bool)
    free[1:39, 1:39] = True
    free[8:14, 5:15] = False
    free[22:30, 20:35] = False
    cells = _decompose_cells(free)
    assert len(cells) == 7
    total = sum(b - a + 1 for cell in cells for _, a, b in cell)
    assert total == int(free.sum())


def test_decompose_no_cell_straddles_the_obstacle():
    free = np.zeros((30, 30), dtype=bool)
    free[1:29, 1:29] = True
    free[10:20, 10:20] = False
    for cell in _decompose_cells(free):
        for r, a, b in cell:
            if 10 <= r < 20:
                assert b < 10 or a >= 20, \
                    'a cell interval must never span the obstacle'


def test_nearest_true_prefers_close_cells():
    mask = np.zeros((10, 10), dtype=bool)
    mask[3, 3] = True
    mask[9, 9] = True
    assert _nearest_true(mask, cx=4, cy=4) == (3, 3)
    assert _nearest_true(np.zeros((4, 4), dtype=bool), 1, 1, max_r=3) is None


class _FakeNav:
    def __init__(self, ready=True):
        self.ready = ready

    def server_is_ready(self):
        return self.ready


def _reactive_stub(dist, live, ready=True, pose='ok', finished=False, advance=True):
    """Build a CoveragePlanner stand-in with just what _tick_reactive touches."""
    from types import MethodType

    from geometry_msgs.msg import PoseStamped
    from oomwoo_coverage.coverage_planner_node import CoveragePlanner

    sent, published = [], []
    target = PoseStamped()
    target.pose.position.x = dist
    me = SimpleNamespace(
        awaiting=False, goal_deadline=None, finished=finished, wp_index=0,
        cached_poses=[target], min_transit_len=1.0, reach_tol=0.12,
        _pass_cmd_live=live, nav_client=_FakeNav(ready),
        cmd_pub=SimpleNamespace(publish=published.append),
        _maybe_wedge_escape=lambda: False,
        _robot_pose=lambda: (0.0, 0.0, 0.0) if pose == 'ok' else None,
        _advance_to_target=lambda: advance,
        _send_goal_to=sent.append)
    me._stop_pass_cmd = MethodType(CoveragePlanner._stop_pass_cmd, me)
    return me, MethodType(CoveragePlanner._tick_reactive, me), sent, published


def test_transit_dispatch_zeroes_live_pass_command():
    me, tick, sent, published = _reactive_stub(dist=3.0, live=True)
    tick()
    assert len(sent) == 1
    assert len(published) == 1
    assert published[0].linear.x == 0.0 and published[0].angular.z == 0.0
    assert not me._pass_cmd_live


def test_transit_without_live_command_publishes_nothing():
    _, tick, sent, published = _reactive_stub(dist=3.0, live=False)
    tick()
    assert len(sent) == 1 and published == []


def test_transit_with_server_not_ready_still_zeroes_once():
    me, tick, sent, published = _reactive_stub(dist=3.0, live=True, ready=False)
    tick()
    tick()
    assert sent == [] and len(published) == 1


def test_no_pose_zeroes_live_pass_command():
    _, tick, _, published = _reactive_stub(dist=0.5, live=True, pose=None)
    tick()
    assert len(published) == 1


def test_reached_waypoint_tick_is_silent_and_the_transit_tick_zeroes():
    from geometry_msgs.msg import PoseStamped

    me, tick, sent, published = _reactive_stub(dist=0.05, live=True)
    far = PoseStamped()
    far.pose.position.x = 3.0
    me.cached_poses.append(far)
    tick()      # within reach_tol: only the index moves, the command stands
    assert published == [] and me.wp_index == 1 and me._pass_cmd_live
    tick()      # next target is a transit: zero, then the goal
    assert len(sent) == 1 and len(published) == 1 and not me._pass_cmd_live


def test_coverage_complete_zeroes_but_gapfill_restart_does_not():
    _, tick, _, published = _reactive_stub(
        dist=0.5, live=True, advance=False, finished=True)
    tick()
    assert len(published) == 1
    _, tick, _, published = _reactive_stub(
        dist=0.5, live=True, advance=False, finished=False)
    tick()
    assert published == []


def test_normal_pass_tick_keeps_driving_and_marks_command_live():
    me, tick, _, published = _reactive_stub(dist=0.5, live=False)
    me.v_cruise, me.rotate_speed, me.k_heading, me.align_tol = 0.3, 1.0, 1.5, 0.3
    me.no_progress_sec = 10.0
    me._drive_best_dist = None
    me._drive_progress_t = None
    me._elapsed = lambda t: 0.0
    me.get_clock = lambda: SimpleNamespace(now=lambda: 0)
    tick()
    assert len(published) == 1 and published[0].linear.x == 0.3
    assert me._pass_cmd_live
