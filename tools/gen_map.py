#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Generate a pixel-perfect occupancy map for test_room directly from its known
primitive geometry. Deterministic and complete (no SLAM gaps), so the coverage
regression has a correct, stable denominator. Writes test_room.pgm/.yaml."""

import math
import numpy as np
from PIL import Image

RES = 0.05
# map covers a little beyond the room shell
OX, OY = -1.60, -3.05
W, H = 140, 140          # 7.0 x 7.0 m

# occupied rectangles (cx, cy, sx, sy, yaw) — walls + furniture, from the world
BOXES = [
    (1.925, 3.845, 6.9, 0.2, 0.0),      # wall_north
    (1.925, -2.855, 6.9, 0.2, 0.0),     # wall_south
    (5.275, 0.495, 0.2, 6.9, 0.0),      # wall_east
    (-1.425, 0.495, 0.2, 6.9, 0.0),     # wall_west
    (4.55, 3.0, 1.8, 0.8, 0.0),         # sofa (NE)
    (3.4, 0.5, 0.9, 0.5, 0.0),          # coffee_table (E-center)
    (-1.15, 1.4, 0.35, 1.6, 0.0),       # bookshelf (W wall)
    (0.6, -2.5, 1.4, 0.45, 0.0),        # tv_stand (S wall)
]
CYL = []             # plant_pot


def world_to_cell(x, y):
    return int((x - OX) / RES), int((y - OY) / RES)


def main():
    occ = np.zeros((H, W), dtype=bool)
    for (cx, cy, sx, sy, yaw) in BOXES:
        c, s = math.cos(-yaw), math.sin(-yaw)
        for gy in range(H):
            for gx in range(W):
                wx = OX + (gx + 0.5) * RES
                wy = OY + (gy + 0.5) * RES
                lx = c * (wx - cx) - s * (wy - cy)
                ly = s * (wx - cx) + c * (wy - cy)
                if abs(lx) <= sx / 2 and abs(ly) <= sy / 2:
                    occ[gy, gx] = True
    for (cx, cy, r) in CYL:
        for gy in range(H):
            for gx in range(W):
                wx = OX + (gx + 0.5) * RES
                wy = OY + (gy + 0.5) * RES
                if math.hypot(wx - cx, wy - cy) <= r:
                    occ[gy, gx] = True

    # interior (inside the four walls) is free; everything else unknown
    free = np.zeros((H, W), dtype=bool)
    for gy in range(H):
        for gx in range(W):
            wx = OX + (gx + 0.5) * RES
            wy = OY + (gy + 0.5) * RES
            if -1.325 < wx < 5.175 and -2.755 < wy < 3.745:
                free[gy, gx] = True
    free &= ~occ

    # PGM: 0=occupied(black), 254=free(white), 205=unknown(gray). Image row 0 is
    # the TOP, but map y increases upward -> flip vertically on write.
    img = np.full((H, W), 205, dtype=np.uint8)
    img[free] = 254
    img[occ] = 0
    Image.fromarray(np.flipud(img), mode='L').save('/root/newmap/test_room.pgm')

    with open('/root/newmap/test_room.yaml', 'w') as f:
        f.write(f"image: test_room.pgm\nmode: trinary\nresolution: {RES}\n"
                f"origin: [{OX}, {OY}, 0]\nnegate: 0\n"
                f"occupied_thresh: 0.65\nfree_thresh: 0.25\n")
    print(f'generated: free={int(free.sum())} occ={int(occ.sum())} '
          f'unknown={int((~free & ~occ).sum())}')


if __name__ == '__main__':
    main()
