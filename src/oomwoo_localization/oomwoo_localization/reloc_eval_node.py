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
Batch-evaluate the global relocalizer, and gate it as a regression test.

Kidnaps the robot to a SYSTEMATIC set of map poses (a free-space grid crossed
with several headings -- so every corner and wall-centre gets sampled), calls
~/relocalize after each teleport, and scores the returned pose against
/ground_truth. Reports success rate, position/heading error, runtime, and --
the product-grade question -- whether the service's own accept flag (score +
confidence) actually predicts correctness. Exits non-zero if the success rate
misses min_success_rate, so the same run is a CI gate.

Run the sim + localization_relocalize (kidnap_injector + ground_truth + map)
first, then this alongside global_relocalizer.
"""

import math
import time

from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import OccupancyGrid

import numpy as np

from oomwoo_localization_msgs.srv import Relocalize

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, \
    QoSReliabilityPolicy


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _ang_diff(a, b):
    return abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)


def _map_qos() -> QoSProfile:
    return QoSProfile(
        depth=1, history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class RelocEval(Node):

    def __init__(self) -> None:
        super().__init__('reloc_eval')
        self.grid_step = self.declare_parameter('grid_step_m', 0.6).value
        self.headings = self.declare_parameter(
            'headings_deg', [0.0, 90.0, 180.0, 270.0]).value
        self.clearance = self.declare_parameter('clearance_m', 0.25).value
        self.free_max = int(self.declare_parameter('free_max', 20).value)
        self.settle = self.declare_parameter('settle_s', 2.5).value
        self.hold = self.declare_parameter('hold_s', 0.0).value
        self.call_timeout = self.declare_parameter('call_timeout_s', 15.0).value
        self.max_pos = self.declare_parameter('max_pos_err_m', 0.20).value
        self.max_yaw = math.radians(
            self.declare_parameter('max_yaw_err_deg', 10.0).value)
        self.min_rate = self.declare_parameter('min_success_rate', 0.9).value
        self.csv_path = self.declare_parameter('csv_path', '').value
        self.srv_name = self.declare_parameter(
            'relocalize_service', '/global_relocalizer/relocalize').value
        self.kidnap_topic = self.declare_parameter(
            'kidnap_topic', '/kidnap_injector/kidnap_to').value
        self.truth_topic = self.declare_parameter(
            'truth_topic', '/ground_truth/pose').value

        self._map = None
        self._truth = None
        self.create_subscription(
            OccupancyGrid, '/map', self._on_map, _map_qos())
        self.create_subscription(
            PoseStamped, self.truth_topic, self._on_truth, 10)
        self.kidnap_pub = self.create_publisher(
            PoseStamped, self.kidnap_topic, 10)
        self.cli = self.create_client(Relocalize, self.srv_name)

    def _on_map(self, msg) -> None:
        self._map = msg

    def _on_truth(self, msg) -> None:
        self._truth = msg

    def _spin(self, seconds) -> None:
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _wait_for(self, pred, timeout) -> bool:
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            if pred():
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return pred()

    def _targets(self):
        info = self._map.info
        res, w, h = info.resolution, info.width, info.height
        ox, oy = info.origin.position.x, info.origin.position.y
        grid = np.array(self._map.data, dtype=np.int16).reshape(h, w)
        free = (grid >= 0) & (grid <= self.free_max)
        cr = int(math.ceil(self.clearance / res))
        ok = free.copy()
        for di in range(-cr, cr + 1):
            for dj in range(-cr, cr + 1):
                if di * di + dj * dj > cr * cr:
                    continue
                sh = np.zeros_like(free)
                gi0, gi1 = max(0, -di), min(h, h - di)
                gj0, gj1 = max(0, -dj), min(w, w - dj)
                if gi0 < gi1 and gj0 < gj1:
                    sh[gi0:gi1, gj0:gj1] = free[gi0 + di:gi1 + di,
                                                gj0 + dj:gj1 + dj]
                ok &= sh
        step = max(1, int(round(self.grid_step / res)))
        out = []
        for gi in range(0, h, step):
            for gj in range(0, w, step):
                if ok[gi, gj]:
                    x = ox + (gj + 0.5) * res
                    y = oy + (gi + 0.5) * res
                    for hd in self.headings:
                        out.append((x, y, math.radians(hd)))
        return out

    def _kidnap(self, x, y, yaw) -> None:
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = float(x), float(y)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.kidnap_pub.publish(msg)

    def _relocalize(self):
        future = self.cli.call_async(Relocalize.Request())
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=self.call_timeout)
        return future.result()

    def run_eval(self) -> int:
        if not self._wait_for(lambda: self._map is not None, 20.0):
            self.get_logger().error('no /map received; aborting')
            return 1
        if not self.cli.wait_for_service(timeout_sec=20.0):
            self.get_logger().error(
                'relocalize service %s not available' % self.srv_name)
            return 1
        targets = self._targets()
        self.get_logger().info(
            'evaluating %d poses (%d cells x %d headings)'
            % (len(targets), len(targets) // max(len(self.headings), 1),
               len(self.headings)))
        rows = []
        for i, (tx, ty, tyaw) in enumerate(targets):
            self._kidnap(tx, ty, tyaw)
            self._spin(self.settle)
            if self._truth is None:
                continue
            truth = (self._truth.pose.position.x, self._truth.pose.position.y,
                     _yaw(self._truth.pose.orientation))
            resp = self._relocalize()
            if resp is None:
                self.get_logger().warn('relocalize call timed out at pose %d' % i)
                continue
            p = resp.pose.pose.pose
            est = (p.position.x, p.position.y, _yaw(p.orientation))
            perr = math.hypot(est[0] - truth[0], est[1] - truth[1])
            yerr = _ang_diff(est[2], truth[2])
            correct = perr <= self.max_pos and yerr <= self.max_yaw
            rows.append((tx, ty, tyaw, truth, est, perr, yerr,
                         resp.score_normalized, resp.confidence,
                         resp.success, correct, resp.runtime_s))
            self.get_logger().info(
                '[%d/%d] perr=%.2fm yerr=%.1fdeg conf=%.2f accept=%s %s'
                % (i + 1, len(targets), perr, math.degrees(yerr),
                   resp.confidence, resp.success,
                   'OK' if correct else 'WRONG'))
            if self.hold > 0.0:
                self._spin(self.hold)      # dwell so the aligned fix is visible
        return self._report(rows)

    def _report(self, rows) -> int:
        if not rows:
            self.get_logger().error('no evaluated poses')
            return 1
        perr = np.array([r[5] for r in rows])
        yerr = np.degrees(np.array([r[6] for r in rows]))
        correct = np.array([r[10] for r in rows])
        accept = np.array([r[9] for r in rows])
        rt = np.array([r[11] for r in rows]) * 1e3
        n = len(rows)
        rate = float(correct.mean())
        # does the service's accept flag predict correctness?
        tp = int((accept & correct).sum())
        prec = tp / max(int(accept.sum()), 1)
        rec = tp / max(int(correct.sum()), 1)
        self.get_logger().info(
            '\n===== RELOCALIZATION EVAL (%d poses) =====\n'
            'success rate      : %.1f%% (%d/%d within %.2fm / %.0fdeg)\n'
            'pos err  mean/95th: %.3f / %.3f m\n'
            'yaw err  mean/95th: %.1f / %.1f deg\n'
            'runtime  mean/95th: %.0f / %.0f ms\n'
            'accept-flag calib : precision %.2f, recall %.2f '
            '(does confidence predict correctness)\n'
            'conf  correct/wrong: %.2f / %.2f'
            % (n, 100 * rate, int(correct.sum()), n, self.max_pos,
               math.degrees(self.max_yaw),
               perr.mean(), np.percentile(perr, 95),
               yerr.mean(), np.percentile(yerr, 95),
               rt.mean(), np.percentile(rt, 95), prec, rec,
               float(np.array([r[8] for r in rows])[correct].mean())
               if correct.any() else 0.0,
               float(np.array([r[8] for r in rows])[~correct].mean())
               if (~correct).any() else 0.0))
        if self.csv_path:
            self._write_csv(rows)
        ok = rate >= self.min_rate
        self.get_logger().info(
            'REGRESSION %s (rate %.1f%% vs min %.1f%%)'
            % ('PASS' if ok else 'FAIL', 100 * rate, 100 * self.min_rate))
        return 0 if ok else 1

    def _write_csv(self, rows) -> None:
        head = ('target_x,target_y,target_yaw,truth_x,truth_y,truth_yaw,'
                'est_x,est_y,est_yaw,pos_err,yaw_err_deg,score_norm,'
                'confidence,accepted,correct,runtime_s')
        lines = [head]
        for r in rows:
            (tx, ty, tyaw, truth, est, perr, yerr,
             sn, conf, acc, cor, rtime) = r
            lines.append(
                '%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,'
                '%.3f,%.3f,%d,%d,%.4f'
                % (tx, ty, tyaw, truth[0], truth[1], truth[2],
                   est[0], est[1], est[2], perr, math.degrees(yerr),
                   sn, conf, int(acc), int(cor), rtime))
        with open(self.csv_path, 'w') as fh:
            fh.write('\n'.join(lines) + '\n')
        self.get_logger().info('wrote %s' % self.csv_path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RelocEval()
    code = 1
    try:
        code = node.run_eval()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
