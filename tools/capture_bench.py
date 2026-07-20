#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Capture a scan-matching benchmark: teleport the robot to N known poses and
record (true pose, laser scan) pairs plus the map, into /root/bench.json.
Run against the live relocalize stack (no recovery needed — only gz + bridge +
map_server must be up; AMCL/recovery may be running but are ignored)."""

import json
import math
import random
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

N_POSES = 15
SEED = 7


def latched():
    return QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                      reliability=QoSReliabilityPolicy.RELIABLE,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def sensor():
    return QoSProfile(depth=5, history=QoSHistoryPolicy.KEEP_LAST,
                      reliability=QoSReliabilityPolicy.BEST_EFFORT,
                      durability=QoSDurabilityPolicy.VOLATILE)


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


def teleport(x, y, yaw):
    qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
    req = (f'name: "oomwoo_one" position {{ x: {x} y: {y} z: 0.06 }} '
           f'orientation {{ x: 0 y: 0 z: {qz} w: {qw} }}')
    out = subprocess.run(
        ['gz', 'service', '-s', '/world/default/set_pose', '--reqtype',
         'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean', '--timeout', '3000',
         '--req', req], capture_output=True, text=True, timeout=8)
    return 'true' in out.stdout.lower()


def main():
    rclpy.init()
    n = rclpy.create_node('bench_capture')
    state = {}

    def on_map(m):
        state['map'] = m

    def on_scan(m):
        state['scan'] = m

    n.create_subscription(OccupancyGrid, '/map', on_map, latched())
    n.create_subscription(LaserScan, '/scan', on_scan, sensor())

    t0 = time.time()
    while time.time() - t0 < 60 and 'map' not in state:
        rclpy.spin_once(n, timeout_sec=0.2)
    m = state['map']
    grid = np.asarray(m.data, dtype=np.int16).reshape(
        m.info.height, m.info.width)
    occupied = grid >= 50
    safe = (grid == 0) & ~dilate(occupied, 10)   # 0.5 m clearance
    ys, xs = np.where(safe)
    rng = random.Random(SEED)
    picks = rng.sample(list(zip(ys.tolist(), xs.tolist())),
                       min(N_POSES, ys.size))

    entries = []
    for (row, col) in picks:
        x = m.info.origin.position.x + (col + 0.5) * m.info.resolution
        y = m.info.origin.position.y + (row + 0.5) * m.info.resolution
        yaw = rng.uniform(-math.pi, math.pi)
        if not teleport(x, y, yaw):
            continue
        # wait for a fresh scan from the new pose
        state.pop('scan', None)
        t0 = time.time()
        while time.time() - t0 < 2.0:
            rclpy.spin_once(n, timeout_sec=0.1)
        t0 = time.time()
        while time.time() - t0 < 3.0 and 'scan' not in state:
            rclpy.spin_once(n, timeout_sec=0.1)
        s = state.get('scan')
        if s is None:
            continue
        entries.append({
            'true': [x, y, yaw],
            'angle_min': s.angle_min, 'angle_increment': s.angle_increment,
            'range_min': s.range_min, 'range_max': s.range_max,
            'ranges': [float(r) for r in s.ranges],
        })
        print(f'captured pose ({x:.2f},{y:.2f},{yaw:.2f}) '
              f'finite={sum(math.isfinite(r) for r in s.ranges)}', flush=True)

    out = {
        'map': {'w': m.info.width, 'h': m.info.height,
                'res': m.info.resolution,
                'ox': m.info.origin.position.x,
                'oy': m.info.origin.position.y,
                'data': [int(v) for v in m.data]},
        'entries': entries,
    }
    with open('/root/bench.json', 'w') as f:
        json.dump(out, f)
    print(f'BENCH_SAVED {len(entries)} entries', flush=True)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
