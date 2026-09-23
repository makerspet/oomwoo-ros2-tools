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

2. Acceptance takes three tests, because the obvious ones are not enough. Cost
   asks whether the points are explained by the template. Coverage asks the
   reverse, whether the template is supported by points -- cost alone cannot tell
   a dock from a bare wall (0.00166 for a plain wall against 0.00170 for a real
   dock standing against one). But coverage alone is not enough either: laid
   along a long wall, the template's parallel faces all find points and coverage
   reaches 0.46, which sailed past a 0.40 threshold and drove the robot into open
   floor beside the dock, believing it was on the bay axis. The third test is the
   one a wall can never pass: THE BAY MUST BE EMPTY. The dock is defined by the
   hole in it, so scan points inside the volume the robot is about to reverse
   into disqualify the fit.

3. Detection has two modes. With a fresh prior -- the dock's recorded map pose,
   the beacon bearing, or the last accepted fit -- only a modest offset has to be
   resolved. With no prior worth trusting, `search` seeds from every plausible
   cluster in the scan at a spread of angles, so the robot finds the dock without
   having to be pointed at it. A fixed "assume it is dead ahead" prior fails as
   soon as somebody parks the robot facing another way, which is most of the
   time.

4. The prior search is seeded from a prior -- the dock's recorded map pose, or the
   bearing of its beacon -- so it only has to resolve a modest offset. Seeding
   instead on the dock's two mouth tubes was tried and abandoned: at 0.6 m a
   15 mm tube spans about three beams, too few to fit a circle to.

Dock frame: origin at the MOUTH CENTRE, +x INTO the bay (the direction the robot
reverses), +y to the left looking in. Poses are SE(2) tuples (x, y, theta) that
take the child frame into the parent.

The default geometry is the visible cross-section of oomwoo_gazebo's vacuum_dock
at the 0.088 m scan plane, taken from its VISUAL shapes because that is what a
LiDAR sees. Those visuals were made to match the model's collision geometry, so
the bay the robot sees is the 0.40 m bay it has to fit through.
"""

import collections
import math

import numpy as np

TUBE_R = 0.015           # tube radius; the tubes round the mouth edges
TUBE_X = 0.015           # tube centres, into the bay (tangent to the mouth plane)
TUBE_Y = 0.215           # tube centres, lateral
PLATE_IN_Y = 0.200       # side plate INNER faces: the bay is 0.40 m wide
PLATE_OUT_Y = 0.230      # side plate outer faces
PLATE_END_X = 0.340      # plates run from the tubes to the wall behind
BACK_X = 0.240           # spacer front face: the back of the bay
SPACER_HALF = 0.215      # spacer half width
CAP_M = 0.05             # distance beyond which a point stops counting
SAMPLE_STEP_M = 0.02     # template sampling, for the coverage check
COVER_TOL_M = 0.03       # a template sample counts as covered within this
BAY_CLEAR_X = (0.02, 0.22)   # the bay volume the robot will occupy...
BAY_CLEAR_Y = 0.170          # ...which must be EMPTY for this to be a dock.
# 30 mm inside the plates' inner faces, not 10: at 10 mm, range noise alone put a
# correct fit's own plate returns 'inside the bay' and rejected it.
MAX_INTRUSIONS = 2           # allow a couple of stray returns, no more

DockFit = collections.namedtuple(
    'DockFit',
    ['pose', 'cost', 'rms', 'inliers', 'coverage', 'intrusions', 'side_cover'])


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
    """
    Visible dock cross-section in the dock frame: (segments, circles).

    Geometry of oomwoo_gazebo's vacuum_dock at the 0.088 m scan plane, from its
    VISUAL shapes, which is what a LiDAR sees. The bay is 0.40 m wide against a
    0.349 m robot, so the manoeuvre has 25 mm of lateral room on each side.
    """
    segs = []
    for sgn in (-1.0, 1.0):
        for y in (sgn * PLATE_IN_Y, sgn * PLATE_OUT_Y):
            segs.append(((TUBE_X, y), (PLATE_END_X, y)))
        segs.append(((PLATE_END_X, sgn * PLATE_IN_Y),
                     (PLATE_END_X, sgn * PLATE_OUT_Y)))
    segs.append(((BACK_X, -SPACER_HALF), (BACK_X, SPACER_HALF)))
    circs = [((TUBE_X, sgn * TUBE_Y), TUBE_R) for sgn in (-1.0, 1.0)]
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
        # Which side of the BAY each sample belongs to, for the both-sides test.
        # The back face spans both sides, so it must be excluded: leaving it in
        # let a phantom made of wall-plus-one-plate score full marks on both.
        on_back = np.abs(self.samples[:, 0] - BACK_X) < 1e-6
        self.left = (self.samples[:, 1] > 0.1) & ~on_back
        self.right = (self.samples[:, 1] < -0.1) & ~on_back

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

    def side_coverage(self, pose, pts, tol=COVER_TOL_M):
        """
        Coverage of the WEAKER side of the bay.

        A dock standing against a long wall admits a second, self-consistent
        fit: the wall serves as the template's back face and one real side plate
        serves as one wall of the bay, with empty floor for the rest. It scored a
        better cost than the truth (0.0007 against 0.0014) and sailed through the
        bay-clear test, because open floor is empty. What it cannot do is show
        BOTH sides of the bay, so the weaker side is the test that catches it.
        """
        pts = np.asarray(pts, dtype=float).reshape(-1, 2)
        if len(pts) == 0:
            return 0.0
        c, s = math.cos(pose[2]), math.sin(pose[2])
        world = np.stack([pose[0] + self.samples[:, 0] * c - self.samples[:, 1] * s,
                          pose[1] + self.samples[:, 0] * s + self.samples[:, 1] * c],
                         axis=1)
        d = np.linalg.norm(world[:, None, :] - pts[None, :, :], axis=2).min(axis=1)
        seen = d < tol
        return min(float(seen[self.left].mean()), float(seen[self.right].mean()))

    def to_local(self, pose, pts):
        """Express robot-frame points in the dock frame of a candidate pose."""
        c, s = math.cos(-pose[2]), math.sin(-pose[2])
        q = np.asarray(pts, dtype=float).reshape(-1, 2) - np.array(pose[:2])
        return np.stack([q[:, 0] * c - q[:, 1] * s,
                         q[:, 0] * s + q[:, 1] * c], axis=1)

    def intrusions(self, pose, pts):
        """
        Scan points inside the bay: a real bay is empty, a wall is not.

        This is the test that distinguishes a dock from any flat surface the
        template happens to lie along, and it is cheap: the robot is about to
        reverse into that volume, so anything seen in it is disqualifying.
        """
        local = self.to_local(pose, pts)
        inside = ((local[:, 0] > BAY_CLEAR_X[0]) & (local[:, 0] < BAY_CLEAR_X[1])
                  & (np.abs(local[:, 1]) < BAY_CLEAR_Y))
        return int(inside.sum())

    def score(self, pose, pts):
        """
        Score a candidate dock pose against scan points (robot frame, (N, 2)).

        Returns (cost, rms of inliers, inlier count). Lower cost is better.
        """
        if len(pts) == 0:
            return 9.9, 9.9, 0
        d = self.distances(self.to_local(pose, pts))
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
        return self._best_of(starts, pts, tries)

    def search(self, pts, yaws=16, tries=6, min_extent=0.20, max_extent=0.90):
        """
        Find the dock with no usable prior, by seeding from the scan itself.

        Every cluster of about the right size is tried as a candidate, at a
        spread of orientations. Costlier than a seeded fit, so the caller should
        fall back to `detect` once it has a fix worth carrying forward.
        """
        pts = np.asarray(pts, dtype=float).reshape(-1, 2)
        if len(pts) < 12:
            return None
        starts = []
        for cl in _clusters(pts):
            extent = float(np.linalg.norm(cl[-1] - cl[0]))
            if not min_extent <= extent <= max_extent:
                continue
            centre = cl.mean(axis=0)
            near = cl[np.linalg.norm(cl, axis=1).argmin()]
            for seed in (centre, near):
                for k in range(yaws):
                    pose = (float(seed[0]), float(seed[1]),
                            wrap(2.0 * math.pi * k / yaws))
                    starts.append((self.score(pose, pts)[0], pose))
        if not starts:
            return None
        starts.sort(key=lambda t: t[0])
        return self._best_of(starts, pts, tries, span=0.12)

    def _best_of(self, starts, pts, tries, span=0.08):
        """Refine the most promising seeds and keep the best fit."""
        best = None
        for _c0, pose in starts[:tries]:
            pose, _cost = self.refine(pose, pts, span=span)
            cost, rms, n = self.score(pose, pts)
            if best is None or cost < best.cost:
                best = DockFit(pose, cost, rms, n, 0.0, 0, 0.0)
        if best is None:
            return None
        return best._replace(coverage=self.coverage(best.pose, pts),
                             intrusions=self.intrusions(best.pose, pts),
                             side_cover=self.side_coverage(best.pose, pts))


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


def accept(fit, max_cost=0.004, min_coverage=0.35,
           max_intrusions=MAX_INTRUSIONS, min_side_coverage=0.10):
    """
    Believe a fit only if all four tests pass.

    Points explained (cost), template supported (coverage), the bay empty
    (intrusions), and BOTH sides of the bay seen (side_cover). Each one catches a
    different impostor: a wall passes the first, a wall laid along the template
    passes the second, and open floor beside the dock passes the third.
    """
    return (fit is not None and fit.cost <= max_cost
            and fit.coverage >= min_coverage
            and fit.intrusions <= max_intrusions
            and fit.side_cover >= min_side_coverage)


def _clusters(pts, gap=0.08, min_pts=6):
    """Split scan points into contiguous runs, splitting at range jumps."""
    pts = np.asarray(pts, dtype=float).reshape(-1, 2)
    if len(pts) < min_pts:
        return []
    step = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    cut = np.flatnonzero(step > gap) + 1
    return [c for c in np.split(pts, cut) if len(c) >= min_pts]
