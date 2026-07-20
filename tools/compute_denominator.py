#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Reproduce the coverage meter's denominator for any map + radius, offline.

Same morphology as coverage_meter_node._ensure_reachable (4-connected dilate,
flood fill from the spawn, cleaning-disk dilation, edge-margin cut), so the
"serviceable cells" figure in a run log can be checked without running a sim —
and the effect of any radius choice on the denominator can be quantified.

  python3 tools/compute_denominator.py <map.yaml> <spawn_x> <spawn_y> <robot_radius> [...]

Example — the meter-coupling audit (old planner-coupled radius vs true body):

  python3 tools/compute_denominator.py maps/test_room.yaml   0.0  0.0  0.30 0.1745
  python3 tools/compute_denominator.py maps/living_room.yaml 0.32 1.59 0.24 0.1745
"""
import re
import sys

import numpy as np
import yaml

CLEANING_RADIUS = 0.20
EDGE_MARGIN = 0.15
OCC_THRESH = 50


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    tokens, i = [], 0
    while len(tokens) < 4:
        m = re.match(rb'\s*(#[^\n]*\n|\S+)', data[i:])
        t = m.group(1)
        i += m.end()
        if not t.startswith(b'#'):
            tokens.append(t)
    assert tokens[0] == b'P5', 'only binary PGM (P5) supported'
    w, h = int(tokens[1]), int(tokens[2])
    img = np.frombuffer(data[len(data) - w * h:], dtype=np.uint8).reshape(h, w)
    return img


def to_grid(img, negate, occ_th, free_th):
    p = img.astype(float) / 255.0
    occ = p if negate else 1.0 - p
    grid = np.full(img.shape, -1, dtype=np.int16)
    grid[occ > occ_th] = 100
    grid[occ < free_th] = 0
    return np.flipud(grid)          # image row 0 = top; grid row 0 = lowest y


def _dilate(mask, radius):          # 4-connected, identical to the meter
    out = mask.copy()
    for _ in range(radius):
        n = out.copy()
        n[1:, :] |= out[:-1, :]
        n[:-1, :] |= out[1:, :]
        n[:, 1:] |= out[:, :-1]
        n[:, :-1] |= out[:, 1:]
        out = n
    return out


def _nearest_free(free, cx, cy, max_r=20):
    h, w = free.shape
    if 0 <= cy < h and 0 <= cx < w and free[cy, cx]:
        return (cy, cx)
    for r in range(1, max_r):
        y0, y1 = max(0, cy - r), min(h, cy + r + 1)
        x0, x1 = max(0, cx - r), min(w, cx + r + 1)
        sub = free[y0:y1, x0:x1]
        if sub.any():
            ys, xs = np.where(sub)
            return (y0 + int(ys[0]), x0 + int(xs[0]))
    return None


def _flood_fill(free, start):
    h, w = free.shape
    out = np.zeros_like(free, dtype=bool)
    stack = [start]
    out[start] = True
    while stack:
        y, x = stack.pop()
        for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ny, nx = y + dy, x + dx
            if 0 <= ny < h and 0 <= nx < w and free[ny, nx] and not out[ny, nx]:
                out[ny, nx] = True
                stack.append((ny, nx))
    return out


def main():
    if len(sys.argv) < 5:
        sys.exit(__doc__)
    map_yaml, sx, sy = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
    radii = [float(r) for r in sys.argv[4:]]

    m = yaml.safe_load(open(map_yaml))
    pgm = map_yaml.rsplit('/', 1)[0] + '/' + m['image'] \
        if '/' in map_yaml else m['image']
    img = read_pgm(pgm)
    res = float(m['resolution'])
    ox, oy = float(m['origin'][0]), float(m['origin'][1])
    grid = to_grid(img, int(m.get('negate', 0)),
                   float(m.get('occupied_thresh', 0.65)),
                   float(m.get('free_thresh', 0.25)))
    free = (grid >= 0) & (grid < OCC_THRESH)

    cx, cy = int((sx - ox) / res), int((sy - oy) / res)
    start = _nearest_free(free, cx, cy)
    if start is None:
        sys.exit('spawn not near free space')
    reach = _flood_fill(free, start)
    r_clean = max(1, int(round(CLEANING_RADIUS / res)))
    em = max(0, int(round(EDGE_MARGIN / res)))

    print(f'{map_yaml}: res={res}m  raw_reachable={int(reach.sum())} cells')
    for rr_m in radii:
        rr = max(1, int(round(rr_m / res)))
        drivable = reach & ~_dilate(~free, rr)
        cleanable = _dilate(drivable, r_clean) & reach
        if em:
            cleanable &= ~_dilate(~free, em)
        print(f'  robot_radius={rr_m:<7} ({rr} cells) -> '
              f'serviceable denominator = {int(cleanable.sum())} cells')


if __name__ == '__main__':
    main()
