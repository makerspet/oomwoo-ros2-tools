#!/usr/bin/env python3
# Copyright 2026 Jayadev Rana
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
Measure true coverage % and path efficiency for the coverage behavior.

Coverage and efficiency are computed from the robot's *ground-truth* pose
(see ground_truth_node), never from the planner's own belief:

  coverage   = (reachable free cells swept by the cleaning disk) / (reachable
               free cells).  "Reachable" = free cells flood-filled from the
               robot's start cell, so sealed-off voids never inflate the score.
  efficiency = ideal_path_len / actual_path_len, where
               ideal_path_len = reachable_area / swath_width  (the length of a
               perfect gap-free boustrophedon).  At constant speed this equals
               time efficiency; reported alongside sim time.

  sub  /map            nav_msgs/OccupancyGrid   (transient_local)
  sub  /ground_truth/pose  geometry_msgs/PoseStamped
  pub  ~/ratio         std_msgs/Float32
  pub  ~/efficiency    std_msgs/Float32
Emits a machine-parseable ``COVERAGE_REPORT ...`` log line every second.
"""

import math
from typing import Optional

from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import OccupancyGrid

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from std_msgs.msg import Bool, Float32

FREE = 0
OCC_THRESH = 50


def _chunk_is_reclean(chunk_len: float, chunk_new: int,
                      rad: int, res: float) -> bool:
    """
    Decide whether a driven chunk was re-cleaning already-covered floor.

    A disk of radius ``rad`` cells sweeping VIRGIN floor stamps ~2*rad new
    cells per cell of travel; a chunk that stamped under a quarter of that
    was mostly re-covering cleaned floor — the wasted-transit distance
    ``revisit_ratio`` exists to expose. Chunking (vs per-sample) avoids
    flagging genuine virgin driving, since poses arrive far faster than the
    disk advances one map cell.
    """
    expected = 2.0 * rad * (chunk_len / res)
    return chunk_new < 0.25 * expected


def latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1, history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class CoverageMeter(Node):
    def __init__(self) -> None:
        super().__init__('coverage_meter')
        self.declare_parameter('cleaning_radius', 0.16)
        self.declare_parameter('robot_radius', 0.175)
        self.declare_parameter('edge_margin', 0.15)   # wall/edge band -> floor-care
        self.declare_parameter('coverage_target', 0.90)
        # Sanity gate: the largest plausible ground-truth step between two
        # consecutive pose samples. The robot tops out at 0.2 m/s, so anything
        # near this bound is a teleport — a symptom of an unstable simulation
        # (seen on Docker-under-WSL2 / emulated hosts), not of driving.
        self.declare_parameter('max_pose_step_m', 0.5)
        self.cleaning_radius = self.get_parameter('cleaning_radius').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.edge_margin = self.get_parameter('edge_margin').value
        self.coverage_target = self.get_parameter('coverage_target').value
        self.max_pose_step = self.get_parameter('max_pose_step_m').value
        self.pose_jumps = 0
        self.sim_unstable = False

        self.info = None
        self.free: Optional[np.ndarray] = None       # bool[H,W]
        self.reachable: Optional[np.ndarray] = None   # bool[H,W]
        self.covered: Optional[np.ndarray] = None     # bool[H,W]
        self.total_reachable = 0

        self.last_xy: Optional[tuple] = None
        self.path_len = 0.0
        self.revisit_len = 0.0
        self.t_start: Optional[rclpy.time.Time] = None
        self.t_target: Optional[float] = None         # sim sec to reach target
        self.target_hit = False

        self.create_subscription(OccupancyGrid, 'map', self._on_map, latched_qos())
        self.create_subscription(
            PoseStamped, 'ground_truth/pose', self._on_pose, 20)
        # efficiency measures the CLEANING JOB: path/time start when the
        # planner reports the sweep goal accepted, not during bringup/settling
        self.job_active = False
        self.create_subscription(
            Bool, 'cleaning_active', self._on_active, latched_qos())
        self.ratio_pub = self.create_publisher(Float32, '~/ratio', 10)
        self.eff_pub = self.create_publisher(Float32, '~/efficiency', 10)
        self.revisit_pub = self.create_publisher(
            Float32, '~/revisit_ratio', 10)
        self.unstable_pub = self.create_publisher(
            Bool, '~/sim_unstable', latched_qos())
        self.covered_pub = self.create_publisher(
            OccupancyGrid, '~/covered_grid', latched_qos())
        self.create_timer(1.0, self._report)
        self.get_logger().info('coverage_meter up; waiting for /map + truth')

    # --------------------------------------------------------------- map
    def _on_map(self, msg: OccupancyGrid) -> None:
        if self.info is not None:
            return
        self.info = msg.info
        h, w = msg.info.height, msg.info.width
        grid = np.asarray(msg.data, dtype=np.int16).reshape(h, w)
        self.free = (grid >= 0) & (grid < OCC_THRESH)
        self.covered = np.zeros_like(self.free, dtype=bool)
        self.get_logger().info(
            f'map {w}x{h} @ {msg.info.resolution:.3f}m, {int(self.free.sum())} free cells '
            '(reachable set computed at first truth pose)')

    def _ensure_reachable(self, cx: int, cy: int) -> None:
        if self.reachable is not None:
            return
        start = _nearest_free(self.free, cx, cy)
        if start is None:
            return
        reach = _flood_fill(self.free, start)
        # Honest denominator = the CLEANABLE area: free cells the cleaning disk
        # can physically reach. The robot's center can come no closer than
        # robot_radius to an obstacle, and cleans cleaning_radius around it —
        # so a thin wall ring is uncleanable by ANY robot of this geometry and
        # must not count against coverage.
        res = self.info.resolution
        r_robot = max(1, int(round(self.robot_radius / res)))
        r_clean = max(1, int(round(self.cleaning_radius / res)))
        drivable = reach & ~_dilate(~self.free, r_robot)   # center positions
        cleanable = _dilate(drivable, r_clean) & reach
        # Denominator = the floor a straight-row (boustrophedon) sweep can
        # actually service. The thin wall/furniture edge band can only be
        # reached by wall-following, which the OOMWOO RFC explicitly assigns to
        # the separate FLOOR-CARE module ("defer wall/edge mode to floor-care"),
        # not to coverage. So the edge_margin ring nearest obstacles is excluded
        # here and left to that module.
        em = max(0, int(round(self.edge_margin / res)))
        if em > 0:
            cleanable = cleanable & ~_dilate(~self.free, em)
        self.reachable = cleanable
        self.total_reachable = int(cleanable.sum())
        self.get_logger().info(
            f'serviceable cells from start {start}: {self.total_reachable} '
            f'(raw reachable {int(reach.sum())}, edge_margin {self.edge_margin}m '
            f'deferred to floor-care)')

    # -------------------------------------------------------------- pose
    def _on_active(self, msg: Bool) -> None:
        if msg.data and not self.job_active:
            self.job_active = True
            self.path_len = 0.0
            self.revisit_len = 0.0
            self.last_xy = None
            self.t_start = self.get_clock().now()
            self.get_logger().info('cleaning job active: path/time accounting reset')

    def _on_pose(self, msg: PoseStamped) -> None:
        if self.info is None:
            return
        res = self.info.resolution
        cx = int((msg.pose.position.x - self.info.origin.position.x) / res)
        cy = int((msg.pose.position.y - self.info.origin.position.y) / res)
        self._ensure_reachable(cx, cy)
        if self.reachable is None:
            return
        if self.t_start is None:
            self.t_start = self.get_clock().now()

        rad = max(1, int(round(self.cleaning_radius / res)))

        jumped = False
        if self.job_active:
            xy = (msg.pose.position.x, msg.pose.position.y)
            if self.last_xy is not None:
                step = math.hypot(xy[0] - self.last_xy[0],
                                  xy[1] - self.last_xy[1])
                if step > self.max_pose_step:
                    # A teleport, not driving. Summing it would destroy the
                    # path-length denominator (observed: a glitching pose fed
                    # 1.5e6 m of "path" -> efficiency 0.0001) — and stamping
                    # the landing would mark floor as cleaned that was never
                    # driven. During an ACTIVE job there is no legitimate
                    # teleport, so a single one already invalidates the
                    # measurement: excusing "just one or two" would let a
                    # teleport skip real path and inflate efficiency.
                    jumped = True
                    self.pose_jumps += 1
                    self.get_logger().warning(
                        f'ground-truth pose jumped {step:.2f} m between '
                        f'samples (> {self.max_pose_step} m) — teleport '
                        f'#{self.pose_jumps}; excluded from path length '
                        'and coverage stamping')
                    if not self.sim_unstable:
                        self.sim_unstable = True
                        self.get_logger().error(
                            'SIM UNSTABLE: ground-truth pose teleported '
                            'during an active cleaning job. Coverage and '
                            'efficiency cannot be measured — re-run on a '
                            'native x86-64 Linux host (Docker on '
                            'Windows/WSL2 and emulated hosts destabilize '
                            'Gazebo physics).')
                else:
                    self.path_len += step
                    self._pending_step = step
            self.last_xy = xy

        if not jumped:
            new_cells = _stamp_disk(self.covered, self.reachable, cx, cy, rad)
            # Revisit accounting, CHUNKED: poses arrive far more often than the
            # disk advances one map cell, so judging per-sample would flag
            # virgin-floor driving as revisit. Instead accumulate distance and
            # newly-stamped cells, and every ~0.25 m compare against what a
            # disk sweeping virgin floor would stamp (2*rad cells per cell of
            # travel). A chunk that stamped under a quarter of that is
            # re-cleaning already-covered floor — the around-the-furniture
            # transit distance revisit_ratio exists to expose.
            if self.job_active:
                self._chunk_len = getattr(self, '_chunk_len', 0.0) \
                    + getattr(self, '_pending_step', 0.0)
                self._chunk_new = getattr(self, '_chunk_new', 0) + new_cells
                if self._chunk_len >= 0.25:
                    if _chunk_is_reclean(self._chunk_len, self._chunk_new,
                                         rad, res):
                        self.revisit_len += self._chunk_len
                    self._chunk_len = 0.0
                    self._chunk_new = 0
            self._pending_step = 0.0

    # ------------------------------------------------------------ report
    def _ratio(self) -> float:
        if self.total_reachable == 0 or self.covered is None:
            return 0.0
        return float((self.covered & self.reachable).sum()) / self.total_reachable

    def _ideal_path_len(self) -> float:
        area = self.total_reachable * (self.info.resolution ** 2)
        return area / (2.0 * self.cleaning_radius)

    def _efficiency(self) -> float:
        # Only meaningful once the robot has actually driven a bit; before that
        # a near-zero denominator would explode the ratio.
        if self.path_len < 0.5:
            return 0.0
        return self._ideal_path_len() / self.path_len

    def _report(self) -> None:
        if self.info is None or self.total_reachable == 0:
            return
        ratio = self._ratio()
        eff = 0.0 if self.sim_unstable else self._efficiency()
        revisit = (self.revisit_len / self.path_len
                   if self.path_len > 1e-6 else 0.0)
        self.ratio_pub.publish(Float32(data=float(ratio)))
        self.eff_pub.publish(Float32(data=float(min(eff, 1.0))))
        self.revisit_pub.publish(Float32(data=float(revisit)))
        self.unstable_pub.publish(Bool(data=self.sim_unstable))

        sim_t = 0.0
        if self.t_start is not None:
            sim_t = (self.get_clock().now() - self.t_start).nanoseconds * 1e-9
        if not self.target_hit and ratio >= self.coverage_target:
            self.target_hit = True
            self.t_target = sim_t

        # publish the covered grid so the planner can prune already-cleaned
        # waypoints when it resumes after a Nav2 failure
        g = OccupancyGrid()
        g.header.frame_id = 'map'
        g.header.stamp = self.get_clock().now().to_msg()
        g.info = self.info
        data = np.zeros(self.covered.shape, dtype=np.int8)
        data[self.covered & self.reachable] = 100
        g.data = data.reshape(-1).tolist()
        self.covered_pub.publish(g)

        self.get_logger().info(
            f'COVERAGE_REPORT coverage={ratio:.4f} efficiency={eff:.4f} '
            f'revisit_ratio={revisit:.4f} '
            f'path_m={self.path_len:.2f} ideal_m={self._ideal_path_len():.2f} '
            f'reachable_cells={self.total_reachable} sim_t={sim_t:.1f} '
            f'target_hit={self.target_hit} t_target={self.t_target} '
            f'pose_jumps={self.pose_jumps} sim_unstable={self.sim_unstable}')


# ------------------------------------------------------------ numpy helpers
def _dilate(mask, radius):
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
    """4-connected flood fill of the free region containing `start`."""
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


def _stamp_disk(covered, mask, cx, cy, rad):
    """Mark the cleaning disk; return how many cells were NEWLY covered."""
    h, w = covered.shape
    y0, y1 = max(0, cy - rad), min(h, cy + rad + 1)
    x0, x1 = max(0, cx - rad), min(w, cx + rad + 1)
    if y0 >= y1 or x0 >= x1:
        return 0
    ys = np.arange(y0, y1)[:, None]
    xs = np.arange(x0, x1)[None, :]
    disk = (ys - cy) ** 2 + (xs - cx) ** 2 <= rad * rad
    add = disk & mask[y0:y1, x0:x1] & ~covered[y0:y1, x0:x1]
    covered[y0:y1, x0:x1] |= add
    return int(add.sum())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CoverageMeter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
