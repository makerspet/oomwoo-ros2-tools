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
Edge-sweep bookkeeping: which obstacle edges are still dirty, and where next.

The cleaning manager runs the contour follower around one obstacle at a time.
This module is the part of it that needs no ROS, so it can be tested on its own:

  * The EDGE RING is every free map cell whose distance to the nearest obstacle
    is the follower's standoff. It is where the robot's centre runs while it
    cleans an edge: one closed loop around a table leg, one long loop round a
    room. A narrow gap has no ring in it, because no point in it is a standoff
    away from all its walls; neither does a pocket the robot cannot get into.
  * A ring cell is DONE once the robot's centre has passed near it while the
    follower was driving. Driving there under Nav2 does not count: the point is
    to have run along the edge.
  * next_target() finds the nearest dirty ring cell, measured along paths the
    robot fits through rather than as the crow flies, and returns a pose on it
    facing so the obstacle is on the follower's side.
  * update() is called with each new robot pose. While the follower drives, it
    reports a REVISIT once the robot has run over ground the follower already
    swept (earlier than revisit_lookback_m of travel ago) for revisit_abort_m.
    That is what ends a lap of a table leg: after one full loop the robot is
    back on its own track, and the manager moves on instead of circling forever.

Every target is tried once. When a segment ends, whatever its outcome, the
dirty ring cells within attempt_radius_m of its target are written off, so a
spot the follower cannot actually trace, or Nav2 cannot reach, is not retried
forever.
"""

import heapq
import math

import numpy as np

OCC_THRESH = 65          # occupancy >= this is an obstacle (map_server's 0.65)
_NEIGHBOURS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def obstacle_distance(occupied, res, max_m):
    """
    Distance from each cell to the nearest obstacle cell, and the direction.

    Returns (dist, ux, uy): dist in metres centre to centre (inf beyond max_m),
    and (ux, uy) the unit vector pointing from that obstacle toward the cell, the
    way the robot backs off it. Exact Euclidean within max_m, done by testing
    offsets in order of distance, which is fast for the half-metre range needed.
    """
    h, w = occupied.shape
    r = int(math.ceil(max_m / res))
    pad = np.zeros((h + 2 * r, w + 2 * r), bool)
    pad[r:r + h, r:r + w] = occupied
    dist = np.full((h, w), np.inf)
    ux = np.zeros((h, w))
    uy = np.zeros((h, w))
    todo = np.ones((h, w), bool)
    offsets = sorted((dx * dx + dy * dy, dx, dy)
                     for dx in range(-r, r + 1) for dy in range(-r, r + 1)
                     if dx * dx + dy * dy <= r * r)
    for d2, dx, dy in offsets:
        hit = todo & pad[r + dy:r + dy + h, r + dx:r + dx + w]
        if not hit.any():
            continue
        n = math.sqrt(d2)
        dist[hit] = n * res
        if n > 0:
            ux[hit] = -dx / n
            uy[hit] = -dy / n
        todo &= ~hit
    return dist, ux, uy


class EdgeSweep:
    """Track the dirty edge ring on an occupancy grid and pick the next target."""

    def __init__(self, grid, res, origin, standoff=0.23, side='right',
                 robot_radius=0.1745, clearance_margin=0.02, cleaning_radius=0.16,
                 done_radius=0.12, revisit_lookback_m=1.0, revisit_abort_m=0.30,
                 attempt_radius_m=0.30, min_todo_len_m=0.30, ring_tol=None):
        """
        Set up from an occupancy grid (rows = y, values 0..100, -1 unknown).

        origin is the world (x, y) of cell (0, 0)'s corner, as in a ROS map.
        """
        self.res = float(res)
        self.x0, self.y0 = float(origin[0]), float(origin[1])
        self.side = 1.0 if side == 'right' else -1.0
        self.done_radius = done_radius
        self.cleaning_radius = cleaning_radius
        self.revisit_lookback_m = revisit_lookback_m
        self.revisit_abort_m = revisit_abort_m
        self.attempt_radius_m = attempt_radius_m
        grid = np.asarray(grid)
        self.h, self.w = grid.shape
        occupied = grid >= OCC_THRESH
        self.free = (grid >= 0) & ~occupied
        reach = max(standoff, robot_radius) + 0.2
        dist, self.ux, self.uy = obstacle_distance(occupied, self.res, reach)
        # centre-to-centre overstates the distance to the surface by ~half a
        # cell: an obstacle cell's centre sits inside the obstacle
        self.surface = dist - 0.5 * self.res
        unknown_dist, _, _ = obstacle_distance(grid < 0, self.res, reach)
        tol = ring_tol if ring_tol is not None else max(0.03, 0.75 * self.res)
        # Along an axis-aligned wall the ring is one cell wide, on diagonals and
        # round corners wider: a metre of edge is ~1.1-1.3 / res cells (a room:
        # 1.09; the torture course: 1.3). Only used to report lengths, marked
        # approximate, and to size slivers, so a middle value will do.
        self.cells_per_m = 1.2 / self.res
        self.min_todo_cells = max(1, int(min_todo_len_m * self.cells_per_m))
        self.passable = self.free & (self.surface >= robot_radius + clearance_margin)
        self.ring = (self.free & (np.abs(self.surface - standoff) <= tol)
                     & (unknown_dist > standoff))
        self.ring_total = int(self.ring.sum())
        self.done = np.zeros_like(self.ring)            # swept by the follower
        self.attempted = np.zeros_like(self.ring)       # written off, see module doc
        self.first_odo = np.full((self.h, self.w), np.inf)   # follower-swept, odometer
        self.cleaned = np.zeros((self.h, self.w), bool)      # anything under the robot
        self.odo = 0.0
        self._last = None
        self._revisit_run = 0.0
        self.target = None
        self.segments = 0
        self.outcomes = {}
        self._disk_cache = {}

    # ---------------------------------------------------------------- geometry
    def cell(self, x, y):
        """World (x, y) to (row, col), or None off the map."""
        c = int(math.floor((x - self.x0) / self.res))
        r = int(math.floor((y - self.y0) / self.res))
        if 0 <= r < self.h and 0 <= c < self.w:
            return r, c
        return None

    def centre(self, r, c):
        """World (x, y) of a cell's centre."""
        return self.x0 + (c + 0.5) * self.res, self.y0 + (r + 0.5) * self.res

    def _disk(self, radius):
        if radius not in self._disk_cache:
            n = int(math.ceil(radius / self.res))
            self._disk_cache[radius] = [
                (dr, dc) for dr in range(-n, n + 1) for dc in range(-n, n + 1)
                if (dr * dr + dc * dc) * self.res * self.res <= radius * radius]
        return self._disk_cache[radius]

    def _stamp(self, x, y, radius):
        rc = self.cell(x, y)
        if rc is None:
            return []
        out = []
        for dr, dc in self._disk(radius):
            r, c = rc[0] + dr, rc[1] + dc
            if 0 <= r < self.h and 0 <= c < self.w:
                out.append((r, c))
        return out

    # ---------------------------------------------------------------- progress
    def update(self, x, y, following):
        """
        Record a new robot pose; True means the follower is re-sweeping old ground.

        following: the contour follower is driving (not Nav2, not idle).
        """
        if self._last is not None:
            step = math.hypot(x - self._last[0], y - self._last[1])
            if step < 0.5:                   # a bigger jump is a relocalization
                self.odo += step
        else:
            step = 0.0
        self._last = (x, y)
        for r, c in self._stamp(x, y, self.cleaning_radius):
            self.cleaned[r, c] = True
        if not following:
            self._revisit_run = 0.0
            return False
        rc = self.cell(x, y)
        old = (rc is not None
               and self.first_odo[rc] < self.odo - self.revisit_lookback_m)
        for r, c in self._stamp(x, y, self.done_radius):
            if self.first_odo[r, c] == np.inf:
                self.first_odo[r, c] = self.odo
            if self.ring[r, c]:
                self.done[r, c] = True
        self._revisit_run = self._revisit_run + step if old else 0.0
        return self._revisit_run >= self.revisit_abort_m

    def todo(self):
        """Return the ring cells still to sweep: not done, not written off."""
        return self.ring & ~self.done & ~self.attempted

    def _todo_worth_it(self):
        """Return the todo cells, minus slivers too short to be worth a trip."""
        todo = self.todo()
        keep = np.zeros_like(todo)
        seen = np.zeros_like(todo)
        for r0, c0 in zip(*np.nonzero(todo)):
            if seen[r0, c0]:
                continue
            comp, stack = [], [(r0, c0)]
            seen[r0, c0] = True
            while stack:
                r, c = stack.pop()
                comp.append((r, c))
                for dr, dc in _NEIGHBOURS:
                    rr, cc = r + dr, c + dc
                    if (0 <= rr < self.h and 0 <= cc < self.w and todo[rr, cc]
                            and not seen[rr, cc]):
                        seen[rr, cc] = True
                        stack.append((rr, cc))
            if len(comp) >= self.min_todo_cells:
                for r, c in comp:
                    keep[r, c] = True
        return keep

    # ---------------------------------------------------------------- planning
    def next_target(self, x, y):
        """
        Nearest worthwhile dirty ring cell by path length; (x, y, yaw) or None.

        yaw faces along the edge with the obstacle on the follower's side, so the
        follower can start straight away.
        """
        goal = self._todo_worth_it()
        start = self._nearest_passable(x, y)
        if start is None or not goal.any():
            self.target = None
            return None
        best = self._dijkstra_first(start, goal)
        if best is None:
            self.target = None
            return None
        r, c = best
        gx, gy = self.centre(r, c)
        away = math.atan2(self.uy[r, c], self.ux[r, c])   # from obstacle toward cell
        # obstacle on the right: heading = away rotated -90 deg (left: +90)
        yaw = away - self.side * math.pi / 2
        self.target = (gx, gy, math.atan2(math.sin(yaw), math.cos(yaw)))
        return self.target

    def _nearest_passable(self, x, y):
        rc = self.cell(x, y)
        if rc is None:
            return None
        if self.passable[rc]:
            return rc
        rows, cols = np.nonzero(self.passable)
        if rows.size == 0:
            return None
        k = int(np.argmin((rows - rc[0]) ** 2 + (cols - rc[1]) ** 2))
        return int(rows[k]), int(cols[k])

    def _dijkstra_first(self, start, goal):
        dist = {start: 0.0}
        heap = [(0.0, start)]
        while heap:
            d, (r, c) = heapq.heappop(heap)
            if d > dist.get((r, c), math.inf):
                continue
            if goal[r, c]:
                return r, c
            for dr, dc in _NEIGHBOURS:
                rr, cc = r + dr, c + dc
                if not (0 <= rr < self.h and 0 <= cc < self.w and self.passable[rr, cc]):
                    continue
                nd = d + (1.4142 if dr and dc else 1.0)
                if nd < dist.get((rr, cc), math.inf):
                    dist[(rr, cc)] = nd
                    heapq.heappush(heap, (nd, (rr, cc)))
        return None

    # ---------------------------------------------------------------- segments
    def begin_segment(self):
        """Start a segment: a new target is being approached."""
        self.segments += 1
        self._revisit_run = 0.0

    def end_segment(self, outcome):
        """Close the segment; write off the dirty ring around its target."""
        self.outcomes[outcome] = self.outcomes.get(outcome, 0) + 1
        self._revisit_run = 0.0
        if self.target is None:
            return
        for r, c in self._stamp(self.target[0], self.target[1], self.attempt_radius_m):
            if self.ring[r, c] and not self.done[r, c]:
                self.attempted[r, c] = True

    def stats(self):
        """Return the totals for the end-of-run report."""
        cell_area = self.res * self.res
        return {
            'edge_done_pct': 100.0 * self.done.sum() / max(1, self.ring_total),
            'edge_done_m': float(self.done.sum() / self.cells_per_m),
            'edge_skipped_m': float((self.attempted & ~self.done).sum() / self.cells_per_m),
            'edge_total_m': float(self.ring_total / self.cells_per_m),
            'cleaned_m2': float(self.cleaned.sum() * cell_area),
            'free_m2': float(self.free.sum() * cell_area),
            'path_m': self.odo,
            'segments': self.segments,
            'outcomes': dict(self.outcomes),
        }
