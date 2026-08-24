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
Stress-test the relocalizer's confidence gate: it must refuse ambiguity.

Puts the real branch-and-bound matcher against ray-cast scans corrupted on
purpose -- a dynamic obstacle, a removed wall, a symmetric room -- and checks
the product-grade invariant: it may ACCEPT a fix only when that fix is actually
correct, and on a genuinely ambiguous map (two equally-good poses) it must
refuse rather than commit to a coin-flip. Pure geometry -- no ROS, no sim.
"""

import math

import numpy as np

from oomwoo_localization import bnb_relocalizer as bnb

RES = 0.05
MAX_RANGE = 3.0
N_BEAMS = 90
SIGMA = 0.10
MIN_SCORE = 0.5      # global_relocalizer's accept thresholds
MIN_CONF = 0.15


def _raycast(occ, x, y, yaw, angles):
    h, w = occ.shape
    out = np.full(angles.size, MAX_RANGE)
    for k, ang in enumerate(angles):
        ca, sa = math.cos(yaw + ang), math.sin(yaw + ang)
        rr = RES * 0.5
        while rr < MAX_RANGE:
            gj, gi = int((x + rr * ca) / RES), int((y + rr * sa) / RES)
            if gi < 0 or gi >= h or gj < 0 or gj >= w or occ[gi, gj]:
                out[k] = rr
                break
            rr += RES * 0.5
    return out


def _scan(occ, pose):
    angles = np.linspace(-math.pi, math.pi, N_BEAMS, endpoint=False)
    return angles, _raycast(occ, pose[0], pose[1], pose[2], angles)


def _evaluate(occ, truth, angles, ranges):
    good = ranges < MAX_RANGE * 0.999
    xy = np.stack([ranges * np.cos(angles), ranges * np.sin(angles)], 1)[good]
    prep = bnb.prepare(
        bnb.build_likelihood_field(occ, RES, SIGMA), RES, MAX_RANGE, 4)
    best = bnb.match_bnb(prep, RES, (0.0, 0.0), xy)
    bx, by, _ = best['pose']
    runner = bnb.match_bnb(prep, RES, (0.0, 0.0), xy, exclude=(bx, by, 0.5))
    s1 = best['score']
    s2 = runner['score'] if runner['pose'] is not None else 0.0
    conf = max(0.0, min(1.0, (s1 - s2) / s1)) if s1 > 0 else 0.0
    norm = s1 / max(best['n_beams'], 1)
    return {'accept': norm >= MIN_SCORE and conf >= MIN_CONF,
            'conf': conf, 'err': math.hypot(bx - truth[0], by - truth[1])}


def _distinct_map():
    occ = np.zeros((60, 60), dtype=bool)
    occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = True
    occ[10:30, 40] = True          # asymmetric interior features
    occ[45, 15:35] = True
    return occ


def _symmetric_map():
    sym = np.zeros((60, 60), dtype=bool)
    sym[0, :] = sym[-1, :] = sym[:, 0] = sym[:, -1] = True   # plain rectangle
    return sym


def test_confidence_gate_never_confidently_wrong():
    occ = _distinct_map()
    truth = (1.4, 1.1, math.radians(37))
    angles, ranges = _scan(occ, truth)

    clean = _evaluate(occ, truth, angles, ranges)
    assert clean['accept'] and clean['err'] < 0.1          # trusted, correct

    clutter = ranges.copy()
    clutter[20:20 + N_BEAMS // 10] = 0.4                    # ~10% dynamic obstacle
    small = _evaluate(occ, truth, angles, clutter)
    assert small['accept'] and small['err'] < 0.1          # tolerates clutter

    gone = ranges.copy()
    wall = np.where(ranges < MAX_RANGE * 0.999)[0]
    gone[wall[:N_BEAMS // 5]] = MAX_RANGE                   # a mapped wall removed
    removed = _evaluate(occ, truth, angles, gone)
    assert removed['accept'] and removed['err'] < 0.1      # robust to a stale map

    heavy = ranges.copy()
    heavy[20:20 + N_BEAMS // 4] = 0.4                       # ~25% obstacle (heavy)
    occluded = _evaluate(occ, truth, angles, heavy)

    sym = _symmetric_map()
    twin = (1.0, 1.3, math.radians(20))
    sa, sr = _scan(sym, twin)
    ambiguous = _evaluate(sym, twin, sa, sr)
    assert not ambiguous['accept']                         # refuses, not a guess
    assert ambiguous['conf'] < 0.1

    # the product-grade invariant: ACCEPT only when the fix is actually right
    for rec in (clean, small, removed, occluded, ambiguous):
        assert (not rec['accept']) or rec['err'] < 0.1
