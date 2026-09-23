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
Dock template and pose estimation from one scan plane.

The dock is found by fitting its KNOWN cross-section to the scan rather than by
hunting for features in it. Two things make that work:

1. The score is a TRUNCATED mean square distance over every point, not a mean
   over inliers. An inlier-only score is degenerate: sliding the template simply
   drops the points that fit worst, so the average improves as the pose gets
   worse. Measured on a head-on scan at 0.6 m, an inlier mean rated a 0.20 m
   error as well as the truth; this score separates them by a factor of 24.

2. Acceptance needs BOTH directions. The cost asks whether the points are
   explained by the template; coverage asks whether the template is supported by
   points. Cost alone cannot tell a dock from a bare wall -- measured 0.00170 for
   a real dock against a wall and 0.00166 for a plain wall with no dock at all --
   because a wall explains the template's long flat faces and simply leaves the
   bay interior unaccounted for. Coverage separates them: 0.69 against 0.28.

3. The search is seeded from a prior -- the dock's recorded map pose, or the
   bearing of its beacon -- so it only has to resolve a modest offset. Seeding
   instead on the dock's two mouth tubes was tried and abandoned: the tubes at
   the BACK of the bay have the same spacing as the ones at the mouth, and at
   0.6 m a 15 mm tube spans about three beams, too few to fit a circle to.

Dock frame: origin at the MOUTH CENTRE, +x INTO the bay (the direction the robot
reverses), +y to the left looking in. Poses are SE(2) tuples (x, y, theta) that
take the child frame into the parent.

The default geometry is the visible cross-section of oomwoo_gazebo's vacuum_dock
at the 0.088 m scan plane, taken from its VISUAL shapes because that is what a
LiDAR sees. Note that model's visual and collision geometry disagree: the bay is
0.43 m wide visually and 0.40 m in collision.
"""

import collections
import math

import numpy as np

TUBE_R = 0.015           # corner tube radius
TUBE_Y = 0.215           # tube centres, lateral
PLATE_Y = 0.2225         # side plate centres, lateral
PLATE_T = 0.015          # side plate thickness
BAY_LEN = 0.310          # mouth tube centre to back tube centre
BACK_X = 0.225           # spacer front face, into the bay
SPACER_HALF = 0.215      # spacer half width
CAP_M = 0.05             # distance beyond which a point stops counting
SAMPLE_STEP_M = 0.02     # template sampling, for the coverage check
COVER_TOL_M = 0.03       # a template sample counts as covered within this

DockFit = collections.namedtuple(
    'DockFit', ['pose', 'cost', 'rms', 'inliers', 'coverage'])


def wrap(a):
    """Wrap an angle to (-pi, pi]."""
    return math.remainder(a, 2.0 * math.pi)


def mul(a, b):
    """Compose poses: a (parent from mid) with b (mid from child)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + b[0] * c - b[1] * s,
            a[1] + b[0] * s + b[1] * c,
            wrap(a[2] + b[2]))


def inv(p):
    """Invert an SE(2) pose."""
    c, s = math.cos(p[2]), math.sin(p[2])
    return (-(p[0] * c + p[1] * s), p[0] * s - p[1] * c, wrap(-p[2]))


def template():
    """Visible dock cross-section in the dock frame: (segments, circles)."""
    segs = []
    for sgn in (-1.0, 1.0):
        y0 = sgn * PLATE_Y - PLATE_T / 2.0
        y1 = sgn * PLATE_Y + PLATE_T / 2.0
        segs += [((0.0, y0), (BAY_LEN, y0)),
                 ((0.0, y1), (BAY_LEN, y1)),
                 ((0.0, y0), (0.0, y1)),
                 ((BAY_LEN, y0), (BAY_LEN, y1))]
    segs.append(((BACK_X, -SPACER_HALF), (BACK_X, SPACER_HALF)))
    circs = [((0.0, sgn * TUBE_Y), TUBE_R) for sgn in (-1.0, 1.0)]
    circs += [((BAY_LEN, sgn * TUBE_Y), TUBE_R) for sgn in (-1.0, 1.0)]
    return segs, circs


class DockFitter:
    """Fits the dock template to scan points; holds the template as arrays."""

    def __init__(self, tmpl=None, cap_m=CAP_M):
        """Pre-arrange the template for vectorised distance evaluation."""
        segs, circs = tmpl if tmpl is not None else template()
        self.a = np.array([s[0] for s in segs], dtype=float)          # (M, 2)
        self.b = np.array([s[1] for s in segs], dtype=float)
        self.ab = self.b - self.a
        self.ab2 = np.maximum((self.ab ** 2).sum(axis=1), 1e-12)
        self.cc = np.array([c[0] for c in circs], dtype=float)        # (K, 2)
        self.cr = np.array([c[1] for c in circs], dtype=float)
        self.cap = float(cap_m)
        self.samples = _sample(segs, circs)                           # (S, 2)

    def distances(self, pts_dock):
        """Distance from each point (dock frame, (N, 2)) to the template."""
        d = pts_dock[:, None, :] - self.a[None, :, :]                 # (N, M, 2)
        t = np.clip((d * self.ab[None, :, :]).sum(axis=2) / self.ab2, 0.0, 1.0)
        foot = self.a[None, :, :] + t[:, :, None] * self.ab[None, :, :]
        seg_d = np.linalg.norm(pts_dock[:, None, :] - foot, axis=2).min(axis=1)
        if len(self.cr):
            rad = np.linalg.norm(pts_dock[:, None, :] - self.cc[None, :, :], axis=2)
            cir_d = np.abs(rad - self.cr[None, :]).min(axis=1)
            return np.minimum(seg_d, cir_d)
        return seg_d

    def coverage(self, pose, pts, tol=COVER_TOL_M):
        """
        Fraction of the template that has a scan point near it.

        The complement of the cost: it catches a template floating over geometry
        that does not contain a dock. Parts of the dock are legitimately hidden
        at an angle, so real docks sit near 0.5-0.75, not 1.0.
        """
        pts = np.asarray(pts, dtype=float).reshape(-1, 2)
        if len(pts) == 0:
            return 0.0
        c, s = math.cos(pose[2]), math.sin(pose[2])
        world = np.stack([pose[0] + self.samples[:, 0] * c - self.samples[:, 1] * s,
                          pose[1] + self.samples[:, 0] * s + self.samples[:, 1] * c],
                         axis=1)
        d = np.linalg.norm(world[:, None, :] - pts[None, :, :], axis=2).min(axis=1)
        return float((d < tol).mean())

    def score(self, pose, pts):
        """
        Score a candidate dock pose against scan points (robot frame, (N, 2)).

        Returns (cost, rms of inliers, inlier count). Lower cost is better.
        """
        if len(pts) == 0:
            return 9.9, 9.9, 0
        c, s = math.cos(-pose[2]), math.sin(-pose[2])
        q = pts - np.array([pose[0], pose[1]])
        local = np.stack([q[:, 0] * c - q[:, 1] * s,
                          q[:, 0] * s + q[:, 1] * c], axis=1)
        d = self.distances(local)
        cost = float((np.minimum(d, self.cap) ** 2).mean())
        inl = d[d < self.cap]
        rms = float(np.sqrt((inl ** 2).mean())) if inl.size else 9.9
        return cost, rms, int(inl.size)

    def refine(self, pose, pts, span=0.08, steps=4):
        """Coordinate descent on (x, y, yaw); local, cheap, good enough."""
        best, best_cost = pose, self.score(pose, pts)[0]
        for _ in range(steps):
            for axis, amp in ((0, span), (1, span), (2, span * 2.0)):
                for delta in (amp, -amp, amp / 3.0, -amp / 3.0):
                    cand = list(best)
                    cand[axis] += delta
                    cost = self.score(tuple(cand), pts)[0]
                    if cost < best_cost:
                        best, best_cost = tuple(cand), cost
            span *= 0.5
        return best, best_cost

    def detect(self, pts, prior, yaw_span_deg=35.0, pos_span_m=0.18, tries=5):
        """
        Locate the dock near `prior`; returns a DockFit, or None.

        `pts` are scan points in the sensor frame and `prior` is the expected
        dock pose in that same frame.
        """
        pts = np.asarray(pts, dtype=float).reshape(-1, 2)
        if len(pts) < 12:
            return None
        starts = []
        for dth in (0.0, yaw_span_deg / 2, -yaw_span_deg / 2,
                    yaw_span_deg, -yaw_span_deg):
            for dx in (0.0, pos_span_m, -pos_span_m):
                for dy in (0.0, pos_span_m, -pos_span_m):
                    pose = (prior[0] + dx, prior[1] + dy,
                            wrap(prior[2] + math.radians(dth)))
                    starts.append((self.score(pose, pts)[0], pose))
        starts.sort(key=lambda t: t[0])
        best = None
        for _c0, pose in starts[:tries]:
            pose, _cost = self.refine(pose, pts)
            cost, rms, n = self.score(pose, pts)
            if best is None or cost < best.cost:
                best = DockFit(pose, cost, rms, n, 0.0)
        if best is None:
            return None
        return best._replace(coverage=self.coverage(best.pose, pts))


def _sample(segs, circs, step=SAMPLE_STEP_M):
    """Points spread over the template, for the coverage check."""
    out = []
    for a, b in segs:
        n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / step))
        for i in range(n + 1):
            t = i / float(n)
            out.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
    for ctr, r in circs:
        for k in range(8):
            th = 2.0 * math.pi * k / 8.0
            out.append((ctr[0] + r * math.cos(th), ctr[1] + r * math.sin(th)))
    return np.array(out, dtype=float)


def accept(fit, max_cost=0.004, min_coverage=0.40):
    """Believe a fit only if the points fit the template AND it is supported."""
    return (fit is not None and fit.cost <= max_cost
            and fit.coverage >= min_coverage)
