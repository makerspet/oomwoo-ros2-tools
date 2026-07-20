#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Render a coverage-run video from a recorded bag (visual proof of the sweep).

Reads /map, /coverage_meter/covered_grid and /ground_truth/pose from a bag
recorded during a coverage regression, and renders an MP4: the room map as
background, cleaned floor filling in green, the robot as a disc with its trail.
A HUD shows sim-relative time and live coverage %. This is the headless-CI
equivalent of watching the run in RViz.

Usage: render_coverage_video.py <bag_dir> <out.mp4> [--fps 12] [--speed 8]
       --speed N renders 1 frame per N seconds of recorded time (default 8x).
"""
import argparse
import os
import subprocess
import sys
import tempfile

import numpy as np
import rclpy.serialization as ser
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from PIL import Image, ImageDraw
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

SCALE = 6                 # px per map cell
ROBOT_R = 0.1745          # m
COL_BG = (30, 30, 34)
COL_FREE = (225, 225, 220)
COL_OCC = (25, 25, 28)
COL_UNK = (120, 120, 118)
COL_COVERED = (110, 200, 110)
COL_TRAIL = (230, 120, 60)
COL_ROBOT = (200, 60, 40)


def grid_to_np(msg):
    return np.array(msg.data, dtype=np.int8).reshape(
        msg.info.height, msg.info.width)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('out')
    ap.add_argument('--fps', type=int, default=12)
    ap.add_argument('--speed', type=float, default=8.0)
    ap.add_argument('--max-sec', type=float, default=0.0,
                    help='cap render to the first N seconds of recorded time '
                         '(0 = whole bag); trims an idle tail')
    a = ap.parse_args()

    reader = SequentialReader()
    reader.open(StorageOptions(uri=a.bag, storage_id='mcap'),
                ConverterOptions('', ''))

    map_msg = None
    covered = None
    poses = []            # (t_sec, x, y)
    grids = []            # (t_sec, np grid) covered_grid snapshots
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        t = t_ns * 1e-9
        if topic == '/map' and map_msg is None:
            map_msg = ser.deserialize_message(data, OccupancyGrid)
        elif topic == '/coverage_meter/covered_grid':
            grids.append((t, ser.deserialize_message(data, OccupancyGrid)))
        elif topic == '/ground_truth/pose':
            m = ser.deserialize_message(data, PoseStamped)
            poses.append((t, m.pose.position.x, m.pose.position.y))
    if map_msg is None or not poses:
        sys.exit('bag lacks /map or poses — nothing to render')
    print(f'map {map_msg.info.width}x{map_msg.info.height}, '
          f'{len(poses)} poses, {len(grids)} coverage grids')

    info = map_msg.info
    W, H = info.width * SCALE, info.height * SCALE
    res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
    g = grid_to_np(map_msg)
    base = np.zeros((info.height, info.width, 3), np.uint8)
    base[:] = COL_UNK
    base[g == 0] = COL_FREE
    base[g > 50] = COL_OCC
    base_img = Image.fromarray(np.flipud(base), 'RGB').resize(
        (W, H), Image.NEAREST)

    def to_px(x, y):
        return ((x - ox) / res * SCALE,
                H - (y - oy) / res * SCALE)

    t0, t1 = poses[0][0], poses[-1][0]
    if a.max_sec > 0:
        t1 = min(t1, t0 + a.max_sec)     # trim the idle tail after the sweep
    frame_dt = a.speed / a.fps
    times = np.arange(t0, t1, frame_dt)
    print(f'{len(times)} frames covering {t1-t0:.0f}s of run '
          f'({a.speed:.0f}x speed)')

    tmp = tempfile.mkdtemp(prefix='covframes_')
    pi = gi = 0
    trail = []
    cov_overlay = None
    cov_pct = 0.0
    total_free = int((g == 0).sum())
    for n, ft in enumerate(times):
        while pi < len(poses) - 1 and poses[pi + 1][0] <= ft:
            pi += 1
            trail.append(to_px(poses[pi][1], poses[pi][2]))
        while gi < len(grids) - 1 and grids[gi + 1][0] <= ft:
            gi += 1
        if grids:
            cg = grid_to_np(grids[gi][1])
            mask = np.flipud(cg > 50)
            cov_overlay = mask
            cov_pct = 100.0 * int((cg > 50).sum()) / max(total_free, 1)

        frame = base_img.copy()
        if cov_overlay is not None:
            ov = np.zeros((info.height, info.width, 4), np.uint8)
            ov[cov_overlay] = (*COL_COVERED, 130)
            frame.paste(Image.fromarray(ov, 'RGBA').resize((W, H), Image.NEAREST),
                        (0, 0), Image.fromarray(ov, 'RGBA').resize((W, H), Image.NEAREST))
        d = ImageDraw.Draw(frame)
        if len(trail) > 1:
            d.line(trail, fill=COL_TRAIL, width=max(2, SCALE // 3))
        x, y = to_px(poses[pi][1], poses[pi][2])
        r = ROBOT_R / res * SCALE
        d.ellipse([x - r, y - r, x + r, y + r], outline=COL_ROBOT,
                  fill=(*COL_ROBOT, 0), width=3)
        d.text((10, 8),
               f't={ft - t0:6.0f}s   coverage(free-cell fill)={cov_pct:5.1f}%',
               fill=(255, 255, 90))
        frame.save(os.path.join(tmp, f'f{n:05d}.png'))
        if n % 50 == 0:
            print(f'  frame {n}/{len(times)}', flush=True)

    subprocess.run(['ffmpeg', '-y', '-framerate', str(a.fps),
                    '-i', os.path.join(tmp, 'f%05d.png'),
                    '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
                    '-vf', 'pad=ceil(iw/2)*2:ceil(ih/2)*2', a.out],
                   check=True, capture_output=True)
    print(f'wrote {a.out}')


if __name__ == '__main__':
    main()
