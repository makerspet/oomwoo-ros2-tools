#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Offline scan-match bench: replay bench.json through the correlative matcher
and report top candidates vs truth for every entry. Iterate here (seconds per
run) instead of via live 7-minute regressions."""

import json
import math
import sys

import numpy as np

NDIRS = 72
NOHIT = 99.0
RANGE_TOL = 0.15


def dilate(mask, radius):
    out = mask.copy()
    for _ in range(radius):
        s = out.copy()
        s[1:, :] |= out[:-1, :]
        s[:-1, :] |= out[1:, :]
        s[:, 1:] |= out[:, :-1]
        s[:, :-1] |= out[:, 1:]
        out = s
    return out


def build_table(grid, res, ox, oy):
    h, w = grid.shape
    occupied = grid >= 50
    blocked = dilate(occupied | (grid < 0), 3)
    free = (grid == 0) & ~blocked
    cy, cx = np.where(free)
    keep = (cy % 2 == 0) & (cx % 2 == 0)
    cy, cx = cy[keep], cx[keep]
    px = (ox + (cx + 0.5) * res).astype(np.float32)
    py = (oy + (cy + 0.5) * res).astype(np.float32)
    P = px.size
    dirs = np.linspace(-math.pi, math.pi, NDIRS, endpoint=False)
    table = np.full((P, NDIRS), NOHIT, dtype=np.float32)
    max_steps = int(math.hypot(h, w)) + 2
    for j, ang in enumerate(dirs):
        dx, dy = math.cos(ang) * res, math.sin(ang) * res
        undecided = np.ones(P, dtype=bool)
        exs, eys = px.copy(), py.copy()
        for s in range(1, max_steps):
            exs += dx
            eys += dy
            gx = ((exs - ox) / res).astype(np.int32)
            gy = ((eys - oy) / res).astype(np.int32)
            inb = (gx >= 0) & (gx < w) & (gy >= 0) & (gy < h)
            hit = np.zeros(P, dtype=bool)
            ok = inb & undecided
            hit[ok] = occupied[gy[ok], gx[ok]]
            table[hit, j] = s * res
            undecided &= ~hit
            undecided &= inb
            if not undecided.any():
                break
    return px, py, table


def bin_scan(e):
    ranges = np.asarray(e['ranges'], dtype=np.float32)
    bear = e['angle_min'] + np.arange(ranges.size) * e['angle_increment']
    meas = np.full(NDIRS, NOHIT, dtype=np.float32)
    bins = ((bear + math.pi) / (2 * math.pi / NDIRS)).astype(int) % NDIRS
    good = np.isfinite(ranges) & (ranges > e['range_min']) & \
        (ranges < e['range_max'] * 0.99)
    for k in range(NDIRS):
        sel = good & (bins == k)
        if sel.any():
            meas[k] = np.median(ranges[sel])
    return meas


def match(meas, px, py, table):
    results = []
    valid = meas < 90.0            # only informative directions count: many gz
    nvalid = max(1, int(valid.sum()))   # beams return nothing (mesh gaps)
    for k in range(NDIRS):
        yaw = -math.pi + k * (2 * math.pi / NDIRS)
        # absolute dir of meas bin i under yaw hypothesis k: both bin scales
        # start at -pi, so table column j = (i + k + N/2) mod N
        shifted = np.roll(table, -(k + NDIRS // 2), axis=1)
        agree = (np.abs(shifted - meas[None, :]) < RANGE_TOL) & valid[None, :]
        scores = agree.sum(axis=1) / nvalid
        top = np.argsort(scores)[-3:][::-1]
        for i in top:
            results.append((float(scores[i]), float(px[i]), float(py[i]), yaw))
    results.sort(reverse=True)
    return results[:6]


def main(path='/root/bench.json'):
    d = json.load(open(path))
    m = d['map']
    grid = np.asarray(m['data'], dtype=np.int16).reshape(m['h'], m['w'])
    px, py, table = build_table(grid, m['res'], m['ox'], m['oy'])
    print(f'table: {px.size} candidates')

    # ---- synthetic self-test: a scan generated from the table itself must
    # match back to exactly that pose with score 1.0 ----------------------
    for (ti, tk) in [(px.size // 10, 0), (px.size // 2, 18), (px.size - 5, 45)]:
        yaw = -math.pi + tk * (2 * math.pi / NDIRS)
        # beam at relative bearing bin i sees absolute dir j=(i+tk+N/2)%N
        synth = np.array([table[ti, (i + tk + NDIRS // 2) % NDIRS]
                          for i in range(NDIRS)], dtype=np.float32)
        top = match(synth, px, py, table)
        s0, x0, y0, y0w = top[0]
        err = math.hypot(x0 - px[ti], y0 - py[ti])
        dy = abs(math.atan2(math.sin(y0w - yaw), math.cos(y0w - yaw)))
        print(f'SELFTEST cand={ti} yawbin={tk}: s={s0:.2f} err={err:.2f} '
              f'dyaw={dy:.2f} -> {"PASS" if err < 0.06 and dy < 0.05 else "FAIL"}')

    def true_score(meas, tx, ty, tyaw):
        """score of the candidate nearest the true pose at the true yaw bin"""
        i = int(np.argmin((px - tx) ** 2 + (py - ty) ** 2))
        k = int(round((tyaw + math.pi) / (2 * math.pi / NDIRS))) % NDIRS
        shifted = np.roll(table[i:i + 1], -(k + NDIRS // 2), axis=1)
        valid = meas < 90.0
        agree = (np.abs(shifted[0] - meas) < RANGE_TOL) & valid
        return agree.sum() / max(1, int(valid.sum()))

    n_ok = 0
    for i, e in enumerate(d['entries']):
        tx, ty, tyaw = e['true']
        meas = bin_scan(e)
        st = true_score(meas, tx, ty, tyaw)
        st_flip = true_score(meas[::-1].copy(), tx, ty, tyaw)
        print(f'    true-pose score={st:.2f}  (bearing-flipped: {st_flip:.2f})')
        top = match(meas, px, py, table)
        s0, x0, y0, yaw0 = top[0]
        err = math.hypot(x0 - tx, y0 - ty)
        dyaw = abs(math.atan2(math.sin(yaw0 - tyaw), math.cos(yaw0 - tyaw)))
        ok = err <= 0.5
        n_ok += ok
        gap = s0 - next((s for s, x, y, w_ in top[1:]
                         if math.hypot(x - x0, y - y0) > 1.0
                         or abs(math.atan2(math.sin(w_ - yaw0),
                                           math.cos(w_ - yaw0))) > 1.0),
                        0.0)
        print(f'[{i:2d}] true=({tx:5.2f},{ty:5.2f},{tyaw:5.2f}) '
              f'best=({x0:5.2f},{y0:5.2f},{yaw0:5.2f}) s={s0:.2f} '
              f'gap={gap:+.2f} err={err:.2f} dyaw={dyaw:.2f} '
              f'{"OK" if ok else "WRONG"}')
    print(f'RESULT {n_ok}/{len(d["entries"])} correct')


if __name__ == '__main__':
    main(*sys.argv[1:])
