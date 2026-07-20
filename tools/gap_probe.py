#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Where is the uncovered floor? Grab /map + the meter's covered grid, recompute
the cleanable set, and print an ASCII map of covered vs uncovered so the gaps'
location is obvious."""
import time
import numpy as np
import rclpy
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from nav_msgs.msg import OccupancyGrid

CLEAN_R, ROBOT_R = 0.18, 0.20


def latched():
    return QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                      reliability=QoSReliabilityPolicy.RELIABLE,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def dilate(m, r):
    o = m.copy()
    for _ in range(r):
        s = o.copy()
        s[1:, :] |= o[:-1, :]; s[:-1, :] |= o[1:, :]
        s[:, 1:] |= o[:, :-1]; s[:, :-1] |= o[:, 1:]
        o = s
    return o


def flood(free, start):
    h, w = free.shape
    out = np.zeros_like(free); st = [start]; out[start] = True
    while st:
        y, x = st.pop()
        for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ny, nx = y + dy, x + dx
            if 0 <= ny < h and 0 <= nx < w and free[ny, nx] and not out[ny, nx]:
                out[ny, nx] = True; st.append((ny, nx))
    return out


def main():
    rclpy.init(); n = rclpy.create_node('gap')
    got = {}
    n.create_subscription(OccupancyGrid, '/map',
                          lambda m: got.setdefault('map', m), latched())
    n.create_subscription(OccupancyGrid, '/coverage_meter/covered_grid',
                          lambda m: got.__setitem__('cov', m), latched())
    t = time.time()
    while time.time() - t < 20 and ('map' not in got or 'cov' not in got):
        rclpy.spin_once(n, timeout_sec=0.2)
    if 'map' not in got or 'cov' not in got:
        print('missing', list(got)); return
    m = got['map']; res = m.info.resolution
    g = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
    free = (g >= 0) & (g < 50)
    start = tuple(np.argwhere(free)[len(np.argwhere(free)) // 2])
    reach = flood(free, tuple(start))
    rr = max(1, round(ROBOT_R / res)); rc = max(1, round(CLEAN_R / res))
    drivable = reach & ~dilate(~free, rr)
    cleanable = dilate(drivable, rc) & reach
    cov = np.asarray(got['cov'].data, dtype=np.int16).reshape(
        got['cov'].info.height, got['cov'].info.width) >= 100
    uncovered = cleanable & ~cov
    print(f'cleanable={cleanable.sum()} covered={(cov & cleanable).sum()} '
          f'uncovered={uncovered.sum()} '
          f'coverage={(cov & cleanable).sum() / cleanable.sum():.3f}')
    # ASCII (downsample to ~40 cols): '#'=obstacle '.'=covered 'X'=uncovered ' '=outside
    H, W = cleanable.shape
    sy, sx = max(1, H // 40), max(1, W // 40)
    for y in range(H - 1, -1, -sy):
        row = ''
        for x in range(0, W, sx):
            yb, xb = slice(max(0, y - sy), y + 1), slice(x, x + sx)
            if uncovered[yb, xb].any():
                row += 'X'
            elif (g[yb, xb] >= 50).any():
                row += '#'
            elif cleanable[yb, xb].any():
                row += '.'
            else:
                row += ' '
        print(row)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
