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
Global relocalization by correlative scan matching with branch-and-bound.

Pure mechanism: given an occupancy map and one laser scan, return the pose
(x, y, theta) that best aligns the scan to the map, searched EXHAUSTIVELY over
the whole map and all headings -- so the answer is the global optimum of the
discretized search, not a lucky local basin (Olson 2009 correlative matching;
Hess et al. 2016 branch-and-bound, as in Cartographer). No ROS, no scipy, so it
unit-tests offline. Policy (when to relocalize, what to do when ambiguous) lives
elsewhere; this file only finds the best pose and how confident that is.

Pipeline:
  build_likelihood_field  map occupied mask -> per-cell score exp(-d^2/2sigma^2)
  build_pyramid           sliding-window MAX grids (each an upper bound)
  match_exhaustive        brute-force correlation over (x, y, theta) -- oracle
  match_bnb               branch-and-bound; provably same argmax, far less work
"""

import heapq
import math

import numpy as np


def _shift(a, dr, dc):
    """Return a shifted so out[i, j] = a[i + dr, j + dc], zero-filled."""
    h, w = a.shape
    out = np.zeros_like(a)
    gi0, gi1 = max(0, -dr), min(h, h - dr)
    gj0, gj1 = max(0, -dc), min(w, w - dc)
    if gi0 < gi1 and gj0 < gj1:
        out[gi0:gi1, gj0:gj1] = a[gi0 + dr:gi1 + dr, gj0 + dc:gj1 + dc]
    return out


def _sliding_max_2d(a, win):
    """Return the square sliding-window max: out[i, j] = max a[i:i+win, j:j+win]."""
    if win <= 1:
        return a.copy()
    out = a
    for axis in (0, 1):
        acc = out.copy()
        for s in range(1, win):
            acc = np.maximum(acc, _shift(out, s, 0) if axis == 0
                             else _shift(out, 0, s))
        out = acc
    return out


def build_likelihood_field(occ, resolution, sigma_m, truncate=3.0):
    """Return a [0, 1] score grid: exp(-d^2/2sigma^2), d = metres to a wall."""
    h, w = occ.shape
    rad = int(math.ceil(truncate * sigma_m / resolution))
    occf = occ.astype(float)
    field = np.zeros((h, w), dtype=float)
    for di in range(-rad, rad + 1):
        for dj in range(-rad, rad + 1):
            d2 = (di * resolution) ** 2 + (dj * resolution) ** 2
            val = math.exp(-d2 / (2.0 * sigma_m * sigma_m))
            np.maximum(field, val * _shift(occf, di, dj), out=field)
    return field


def build_pyramid(field, levels):
    """Return [field, maxpool(win=2), maxpool(win=4), ...] for BnB bounds."""
    return [field] + [_sliding_max_2d(field, 1 << d)
                      for d in range(1, levels + 1)]


def _offsets(scan_xy, theta, resolution):
    """Rotate the scan by theta and return integer (row, col) cell offsets."""
    c, s = math.cos(theta), math.sin(theta)
    rx = c * scan_xy[:, 0] - s * scan_xy[:, 1]
    ry = s * scan_xy[:, 0] + c * scan_xy[:, 1]
    drow = np.floor(ry / resolution).astype(np.int64)
    dcol = np.floor(rx / resolution).astype(np.int64)
    return drow, dcol


def default_thetas(scan_xy, resolution):
    """Return a full-circle heading grid stepped so the far beam moves <1 cell."""
    reach = float(np.hypot(scan_xy[:, 0], scan_xy[:, 1]).max())
    step = resolution / max(reach, resolution)
    n = int(math.ceil(2.0 * math.pi / step))
    return np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)


def _score_grid(field, drow, dcol):
    """Correlate one rotated scan over every translation: acc[i,j] = sum hits."""
    h, w = field.shape
    acc = np.zeros((h, w), dtype=float)
    for dr, dc in zip(drow.tolist(), dcol.tolist()):
        acc += _shift(field, dr, dc)
    return acc


def _cell_to_world(gi, gj, resolution, origin):
    return origin[0] + gj * resolution, origin[1] + gi * resolution


def match_exhaustive(field, resolution, origin, scan_xy, thetas=None):
    """
    Brute-force correlation over (x, y, theta); the reference oracle.

    Return dict: pose, score, and runner_up (best score/pose in a different
    cluster, for a confidence margin).
    """
    if thetas is None:
        thetas = default_thetas(scan_xy, resolution)
    best = (-1.0, 0, 0, 0.0)                 # score, gi, gj, theta
    peaks = []                               # per-theta top-1 (score, gi, gj, th)
    for th in thetas:
        drow, dcol = _offsets(scan_xy, th, resolution)
        acc = _score_grid(field, drow, dcol)
        gi, gj = np.unravel_index(int(np.argmax(acc)), acc.shape)
        sc = float(acc[gi, gj])
        peaks.append((sc, int(gi), int(gj), float(th)))
        if sc > best[0]:
            best = (sc, int(gi), int(gj), float(th))
    x, y = _cell_to_world(best[1], best[2], resolution, origin)
    runner = _runner_up(peaks, best, resolution, origin)
    return {'pose': (x, y, best[3]), 'score': best[0],
            'n_beams': int(scan_xy.shape[0]), 'runner_up': runner}


def _runner_up(peaks, best, resolution, origin, exclude_m=0.5):
    """Best peak in a spatially distinct cluster from the winner."""
    bx, by = _cell_to_world(best[1], best[2], resolution, origin)
    out = (0.0, None)
    for sc, gi, gj, th in peaks:
        x, y = _cell_to_world(gi, gj, resolution, origin)
        if math.hypot(x - bx, y - by) >= exclude_m and sc > out[0]:
            out = (sc, (x, y, th))
    return {'score': out[0], 'pose': out[1]}


def prepare(field, resolution, max_range, levels=4):
    """
    Precompute the padded max-pool pyramid a map needs for match_bnb.

    Zero-pad so every beam's window is in-array at every base cell -- otherwise
    an off-grid beam gets dropped and the max-pool stops being an upper bound
    (under-counting prunes valid optima). Do this once per map; reuse per scan.
    """
    pad = int(math.ceil(max_range / resolution)) + (1 << levels) + 1
    padded = np.pad(field, pad)
    return {'pyramid': build_pyramid(padded, levels), 'pad': pad,
            'levels': levels, 'shape': field.shape}


def match_bnb(prep, resolution, origin, scan_xy, thetas=None, exclude=None):
    """
    Branch-and-bound correlative match; same argmax as match_exhaustive.

    Return dict: pose, score, n_beams. Depth-first with a best-first frontier;
    prunes any node whose max-pooled upper bound cannot beat the best leaf.
    exclude=(x, y, radius) skips leaves within radius of (x, y) -- run once
    normally, then again excluding the winner to get the runner-up cluster for
    a confidence margin.
    """
    if thetas is None:
        thetas = default_thetas(scan_xy, resolution)
    pyramid, pad, levels = prep['pyramid'], prep['pad'], prep['levels']
    h, w = prep['shape']
    best_score = -1.0
    best = None

    def ub(level, gi, gj, drow, dcol):
        return float(pyramid[level][gi + drow + pad, gj + dcol + pad].sum())

    for th in thetas:
        drow, dcol = _offsets(scan_xy, th, resolution)
        step = 1 << levels
        frontier = []                        # max-heap via negated score
        for gi in range(0, h, step):
            for gj in range(0, w, step):
                b = ub(levels, gi, gj, drow, dcol)
                if b > best_score:
                    heapq.heappush(frontier, (-b, levels, gi, gj))
        while frontier:
            nb, level, gi, gj = heapq.heappop(frontier)
            if -nb <= best_score:
                break                        # frontier is sorted; nothing better
            if level == 0:
                x, y = _cell_to_world(gi, gj, resolution, origin)
                if exclude is not None and math.hypot(
                        x - exclude[0], y - exclude[1]) < exclude[2]:
                    continue                 # skip the winner's cluster
                best_score = -nb
                best = (x, y, th)
                continue
            child = level - 1
            off = 1 << child
            for dgi in (0, off):
                for dgj in (0, off):
                    ci, cj = gi + dgi, gj + dgj
                    if ci >= h or cj >= w:
                        continue
                    b = ub(child, ci, cj, drow, dcol)
                    if b > best_score:
                        heapq.heappush(frontier, (-b, child, ci, cj))
    return {'pose': best, 'score': best_score, 'n_beams': int(scan_xy.shape[0])}
