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
Kidnapped-robot detection and relocalization for OOMWOO.

Behavior: on a saved map with Nav2 + AMCL running, detect when localization
confidence collapses (the robot was picked up and moved, or AMCL diverged),
then actively relocalize: scatter AMCL particles globally and rotate in place to
gather scans until the pose re-converges. Report a clear success/failure signal.

Success (per nav-localize RFC): re-converge within 30 s and ≤ 2 m of truth,
≥ 90 % of the time. This node owns detection + the motion recovery; AMCL owns
the particle filter.

Interfaces:
  sub   /amcl_pose        geometry_msgs/PoseWithCovarianceStamped
  sub   /kidnap_trigger   std_msgs/Empty   (optional external "you were moved")
  srv c /reinitialize_global_localization  std_srvs/Empty  (AMCL)
  pub   /cmd_vel          geometry_msgs/Twist   (exclusive while recovering)
  pub   ~/localization_status  oomwoo status (published as diagnostic string+bool)

/cmd_vel arbitration: while state == RECOVERING this node is the *only* velocity
source. The relocalize launch does not run a Nav2 goal concurrently; if it did,
integrators must gate Nav2 on ~/recovering.
"""

from enum import Enum
import math
from typing import Optional

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist

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

from sensor_msgs.msg import LaserScan

from std_msgs.msg import Bool, Empty, Float32, String

from std_srvs.srv import Empty as EmptySrv


def _pose_differs(a, b, d_m: float = 1.0, d_yaw: float = 1.0) -> bool:
    """Return True when two (x, y, yaw) hypotheses are genuinely distinct."""
    dyaw = abs(math.atan2(math.sin(a[2] - b[2]), math.cos(a[2] - b[2])))
    return math.hypot(a[0] - b[0], a[1] - b[1]) > d_m or dyaw > d_yaw


def _dilate_bool(mask, radius):
    """Binary dilation by a square structuring element."""
    out = mask.copy()
    for _ in range(radius):
        s = out.copy()
        s[1:, :] |= out[:-1, :]
        s[:-1, :] |= out[1:, :]
        s[:, 1:] |= out[:, :-1]
        s[:, :-1] |= out[:, 1:]
        out = s
    return out


class State(Enum):
    TRACKING = 0      # localized, confidence ok
    RECOVERING = 1    # lost -> actively relocalizing
    FAILED = 2        # gave up -> hand off to dock-cycle fallback


def amcl_qos() -> QoSProfile:
    return QoSProfile(
        depth=5,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


def sensor_qos() -> QoSProfile:
    return QoSProfile(
        depth=5,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


def map_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class KidnapRecovery(Node):
    def __init__(self) -> None:
        super().__init__('kidnap_recovery')

        # --- parameters ---------------------------------------------------
        # Confidence proxy: trace of the x,y,yaw covariance from /amcl_pose.
        self.declare_parameter('lost_cov_trace', 0.6)     # enter recovery above
        # AMCL global re-init scatters particles over the whole map; a converged
        # trace in this room settles ~0.1-0.3, so 0.25 is a safe "converged" gate
        # (still << the 2 m accuracy target).
        self.declare_parameter('ok_cov_trace', 0.25)      # converged below
        self.declare_parameter('converge_hold_sec', 1.0)  # stay converged this long
        self.declare_parameter('recovery_timeout_sec', 30.0)
        self.declare_parameter('spin_speed', 0.9)         # rad/s in-place
        self.declare_parameter('drive_speed', 0.16)       # m/s while exploring
        self.declare_parameter('front_clear_m', 0.35)     # obstacle stop distance
        self.declare_parameter('explore_sec', 6.0)        # (legacy, unused)
        self.declare_parameter('settle_sec', 6.0)         # (legacy, unused)
        self.declare_parameter('match_score_ok', 0.75)    # accept scan-match above
        self.declare_parameter('verify_sec', 4.0)         # AMCL confirm window
        self.declare_parameter('reposition_sec', 1.8)     # drive before re-match
        self.declare_parameter('settle_after_trigger_sec', 0.5)

        self.lost_trace = self.get_parameter('lost_cov_trace').value
        self.ok_trace = self.get_parameter('ok_cov_trace').value
        self.hold_sec = self.get_parameter('converge_hold_sec').value
        self.timeout_sec = self.get_parameter('recovery_timeout_sec').value
        self.spin_speed = self.get_parameter('spin_speed').value
        self.drive_speed = self.get_parameter('drive_speed').value
        self.front_clear_m = self.get_parameter('front_clear_m').value
        self.explore_sec = self.get_parameter('explore_sec').value
        self.settle_sec = self.get_parameter('settle_sec').value
        self.match_score_ok = self.get_parameter('match_score_ok').value
        self.verify_sec = self.get_parameter('verify_sec').value
        self.reposition_sec = self.get_parameter('reposition_sec').value

        # --- state --------------------------------------------------------
        self.state = State.TRACKING
        self.last_trace: Optional[float] = None
        self.recover_start: Optional[rclpy.time.Time] = None
        self.converged_since: Optional[rclpy.time.Time] = None
        self.reinit_sent = False
        self.front_clear = True
        self.open_heading = 0.0
        self.last_scan = None
        self.map_grid = None
        self.map_res = 0.05
        self.map_ox = 0.0
        self.map_oy = 0.0
        self.rt_table = None        # expected-range table (built lazily)
        self.rt_px = None
        self.rt_py = None

        # --- ROS plumbing -------------------------------------------------
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl, amcl_qos())
        self.create_subscription(
            Empty, 'kidnap_trigger', self._on_trigger, 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan, sensor_qos())
        self.create_subscription(OccupancyGrid, 'map', self._on_map, map_qos())

        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.recovering_pub = self.create_publisher(Bool, '~/recovering', 10)
        self.conf_pub = self.create_publisher(Float32, '~/confidence', 10)
        self.status_pub = self.create_publisher(String, '~/localization_status', 10)

        self.reinit_cli = self.create_client(
            EmptySrv, 'reinitialize_global_localization')

        self.create_timer(0.1, self._tick)  # 10 Hz control
        self.get_logger().info('kidnap_recovery up; tracking localization')

    # ------------------------------------------------------------- inputs
    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        cov = msg.pose.covariance
        # covariance is row-major 6x6: xx=0, yy=7, yaw yaw=35
        # xy only: yaw stays bimodal in near-symmetric rooms and would pin the full trace high
        self.last_trace = float(cov[0] + cov[7])

    def _on_map(self, msg: OccupancyGrid) -> None:
        if self.map_grid is not None:
            return
        self.map_grid = np.asarray(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width)
        self.map_res = msg.info.resolution
        self.map_ox = msg.info.origin.position.x
        self.map_oy = msg.info.origin.position.y
        self.get_logger().info(
            f'map cached for scan-matching: {msg.info.width}x{msg.info.height}')

    def _on_trigger(self, _msg: Empty) -> None:
        # external "you were picked up / moved" signal (e.g. from sim harness
        # or a future pickup sensor). Force recovery regardless of covariance.
        if self.state != State.RECOVERING:
            self.get_logger().warn('kidnap_trigger received -> entering recovery')
            self._enter_recovery()

    def _on_scan(self, msg: LaserScan) -> None:
        n = len(msg.ranges)
        if n == 0:
            return
        self.last_scan = msg
        # front clearance = min range within +/-25 deg of straight ahead
        span = max(1, int((25.0 * math.pi / 180.0) / msg.angle_increment))
        window = list(msg.ranges[:span]) + list(msg.ranges[-span:])
        valid = [r for r in window if msg.range_min < r < msg.range_max]
        self.front_clear = (min(valid) > self.front_clear_m) if valid else True

        # heading (relative to robot) of the most open direction, smoothed over a
        # window, considering only the forward 180 deg so we drive INTO open space
        # rather than reversing — this traverses the room fast and gives AMCL a
        # diverse, disambiguating scan sequence.
        best_ang, best_avg = 0.0, -1.0
        half = max(1, int((15.0 * math.pi / 180.0) / msg.angle_increment))
        fwd = max(1, int((math.pi / 2.0) / msg.angle_increment))  # +/-90 deg
        for c in range(-fwd, fwd + 1, half):
            idx = [(c + k) % n for k in range(-half, half + 1)]
            rs = [msg.ranges[j] for j in idx
                  if msg.range_min < msg.ranges[j] < msg.range_max]
            if rs:
                avg = sum(rs) / len(rs)
                if avg > best_avg:
                    best_avg, best_ang = avg, c * msg.angle_increment
        self.open_heading = best_ang

    # -------------------------------------------------------------- logic
    def _enter_recovery(self) -> None:
        self.state = State.RECOVERING
        self.recover_start = self.get_clock().now()
        self.phase = 'capture'
        self.phase_start = self.recover_start
        self.converged_since = None
        self.reinit_sent = False

    def _tick(self) -> None:
        now = self.get_clock().now()
        conf = 0.0 if self.last_trace is None else \
            float(max(0.0, 1.0 - self.last_trace / self.lost_trace))
        self.conf_pub.publish(Float32(data=conf))

        if self.state == State.TRACKING:
            self.recovering_pub.publish(Bool(data=False))
            if self.last_trace is not None and self.last_trace >= self.lost_trace:
                self.get_logger().warn(
                    f'localization lost (cov trace {self.last_trace:.2f}) -> recovery')
                self._enter_recovery()
            else:
                self._publish_status('LOCALIZED', True)

        elif self.state == State.RECOVERING:
            self.recovering_pub.publish(Bool(data=True))
            self._do_recovery(now)

        elif self.state == State.FAILED:
            self.recovering_pub.publish(Bool(data=False))
            # Do NOT keep spamming zero cmd_vel: with ~/recovering=False a
            # downstream consumer (dock-cycle) is entitled to drive, and this
            # node must not stomp its commands every tick. The single stop
            # Twist was already sent on entry to FAILED. If AMCL reconverges
            # on its own (covariance back under the OK gate), return to
            # TRACKING instead of staying lost forever.
            if self.last_trace is not None \
                    and self.last_trace < self.ok_trace:
                self.state = State.TRACKING
                self.get_logger().info(
                    'covariance recovered while FAILED -> back to TRACKING')
                self._publish_status('LOCALIZED', True)

    def _do_recovery(self, now) -> None:
        # Recovery = global SCAN-MATCH relocalization:
        #   capture: stop, grab one full 360-deg scan
        #   match:   brute-force score the scan against the map over all free
        #            cells x yaw bins (vectorized; ~1-2 s) -> best pose
        #   seed:    AMCL global re-init (honest covariance spike), then
        #            /initialpose at the matched pose with tight covariance
        #   verify:  slow scan-spin so AMCL updates; xy covariance collapsing
        #            confirms the seed is consistent -> RELOCALIZED
        #   reposition: low-confidence match or failed verify -> drive to a new
        #            vantage point and try again (until timeout)
        # A particle filter alone struggles here: the room is near-symmetric, so
        # a yaw-mirrored hypothesis survives AMCL's 60-beam z_rand-diluted model
        # indefinitely. Directly scoring the full scan against the map is
        # decisive, and AMCL then polishes + tracks from the seed.
        phase_t = (now - self.phase_start).nanoseconds * 1e-9

        if self.phase == 'capture':
            self._stop()
            if phase_t >= 0.6 and self.last_scan is not None \
                    and self.map_grid is not None:
                best, score = self._scan_match(self.last_scan)
                self.get_logger().info(
                    f'scan-match: pose=({best[0]:.2f},{best[1]:.2f},'
                    f'{best[2]:.2f}) score={score:.2f}')
                if score >= self.match_score_ok:
                    if not self.reinit_sent:
                        if self.reinit_cli.service_is_ready():
                            self.reinit_cli.call_async(EmptySrv.Request())
                        self.reinit_sent = True
                    self._publish_initialpose(best)
                    self.phase, self.phase_start = 'verify', now
                else:
                    self.phase, self.phase_start = 'reposition', now

        elif self.phase == 'verify':
            # slow in-place spin: position holds still while rotation keeps
            # AMCL updating so its covariance reflects the seeded estimate
            t = Twist()
            t.angular.z = 0.5 * self.spin_speed
            self.cmd_pub.publish(t)
            trace = self.last_trace if self.last_trace is not None else math.inf
            if phase_t >= 1.0 and trace <= self.ok_trace:
                if self.converged_since is None:
                    self.converged_since = now
                elif (now - self.converged_since).nanoseconds * 1e-9 >= \
                        self.hold_sec:
                    elapsed = (now - self.recover_start).nanoseconds * 1e-9
                    self._stop()
                    self.state = State.TRACKING
                    self.get_logger().info(
                        f'RELOCALIZED in {elapsed:.1f}s (xy cov {trace:.3f})')
                    self._publish_status('RELOCALIZED', True)
                    return
            else:
                self.converged_since = None
            if phase_t >= self.verify_sec:      # verify failed -> new vantage
                self.phase, self.phase_start = 'reposition', now

        else:  # reposition: drive toward open space for a fresh viewpoint
            self._explore()
            if phase_t >= self.reposition_sec:
                self.phase, self.phase_start = 'capture', now

        # timeout -> fail, hand off to dock-cycle find-the-dock fallback
        if (now - self.recover_start).nanoseconds * 1e-9 >= self.timeout_sec:
            self._stop()
            self.state = State.FAILED
            self.get_logger().error(
                'relocalization FAILED (timeout) -> dock-cycle fallback')
            self._publish_status('LOCALIZATION_LOST', False)

    # ------------------------------------------------- global scan matching
    NDIRS = 72          # angular resolution of the range table (5 deg)
    NOHIT = 99.0        # sentinel range for "no obstacle within map"
    RANGE_TOL = 0.15    # m, beam agreement tolerance

    def _build_range_table(self) -> None:
        """
        Raycast the map once into an expected-range table.

        Covers every candidate free cell x NDIRS directions. One-time cost
        (~2-4 s); makes each subsequent global match a pure table correlation.
        """
        grid, res, ox, oy = self.map_grid, self.map_res, self.map_ox, self.map_oy
        h, w = grid.shape
        occupied = grid >= 50
        blocked = _dilate_bool(occupied | (grid < 0), 3)
        free = (grid == 0) & ~blocked
        cy, cx = np.where(free)
        keep = (cy % 2 == 0) & (cx % 2 == 0)     # 10 cm candidate lattice
        cy, cx = cy[keep], cx[keep]
        px = (ox + (cx + 0.5) * res).astype(np.float32)
        py = (oy + (cy + 0.5) * res).astype(np.float32)
        P = px.size
        dirs = np.linspace(-math.pi, math.pi, self.NDIRS,
                           endpoint=False).astype(np.float32)
        table = np.full((P, self.NDIRS), self.NOHIT, dtype=np.float32)
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
                undecided &= inb   # left the map without hitting -> NOHIT
                if not undecided.any():
                    break
        self.rt_px, self.rt_py, self.rt_table = px, py, table
        self.get_logger().info(
            f'range table built: {P} candidates x {self.NDIRS} directions')

    def _scan_match(self, scan) -> tuple:
        """
        Run correlative global localization on the measured scan.

        Compare the measured 360-deg scan against the precomputed
        expected-range table over all candidate poses and yaw bins (yaw =
        circular shift of the table). Score = fraction of directions whose
        measured range agrees within RANGE_TOL.
        Returns ((x, y, yaw), score); score 0.0 when the best match is
        ambiguous (a distinct pose scores nearly as well).
        """
        if self.rt_table is None:
            self._build_range_table()
        # bin the measured scan into the table's NDIRS absolute bearings
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        bear = scan.angle_min + np.arange(ranges.size) * scan.angle_increment
        meas = np.full(self.NDIRS, self.NOHIT, dtype=np.float32)
        bins = ((bear + math.pi) / (2 * math.pi / self.NDIRS)).astype(int) \
            % self.NDIRS
        good = np.isfinite(ranges) & (ranges > scan.range_min) & \
            (ranges < scan.range_max * 0.99)
        for k in range(self.NDIRS):
            sel = good & (bins == k)
            if sel.any():
                meas[k] = np.median(ranges[sel])

        table = self.rt_table
        best_score, best = -1.0, (0.0, 0.0, 0.0)
        second_score, second = -1.0, None
        valid = meas < 90.0             # directions with a measured return
        for k in range(self.NDIRS):                 # yaw hypothesis
            yaw = -math.pi + k * (2 * math.pi / self.NDIRS)
            # beam at relative bearing bin i sees absolute direction bin
            # (i + k + N/2) mod N — both bin scales start at -pi.
            # (validated by the synthetic self-test in tools/offline_match.py)
            shifted = np.roll(table, -(k + self.NDIRS // 2), axis=1)
            # informative = measured a return AND the map has geometry there;
            # on a partial map, directions the map never saw (NOHIT raycast)
            # must not count against the true pose
            informative = valid[None, :] & (shifted < 90.0)
            agree = (np.abs(shifted - meas[None, :]) < self.RANGE_TOL) \
                & informative
            denom = np.maximum(informative.sum(axis=1), 8)
            scores = agree.sum(axis=1) / denom
            i = int(np.argmax(scores))
            s = float(scores[i])
            cand = (float(self.rt_px[i]), float(self.rt_py[i]), yaw)
            if s > best_score:
                if best_score > 0 and _pose_differs(best, cand):
                    second_score, second = best_score, best
                best_score, best = s, cand
            elif s > second_score and _pose_differs(best, cand):
                second_score, second = s, cand

        # ambiguity: with continuous range agreement over 72 directions, a
        # 0.06 absolute score gap between distinct poses is already decisive,
        # and a near-perfect score (>= 0.92) cannot come from a mirrored pose
        # in a furnished room. Only reject genuinely tied hypotheses.
        if (best_score < 0.92 and second is not None
                and second_score > best_score - 0.06):
            self.get_logger().info(
                f'scan-match AMBIGUOUS: best={best} ({best_score:.2f}) vs '
                f'{second} ({second_score:.2f})')
            return best, 0.0
        return best, best_score

    def _publish_initialpose(self, pose) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = pose[0]
        msg.pose.pose.position.y = pose[1]
        msg.pose.pose.orientation.z = math.sin(pose[2] / 2.0)
        msg.pose.pose.orientation.w = math.cos(pose[2] / 2.0)
        msg.pose.covariance[0] = 0.10
        msg.pose.covariance[7] = 0.10
        msg.pose.covariance[35] = 0.10
        self.initialpose_pub.publish(msg)

    # --------------------------------------------------------------- motion
    def _spin(self) -> None:
        t = Twist()
        t.angular.z = self.spin_speed
        self.cmd_pub.publish(t)

    def _explore(self) -> None:
        # Head into open space: steer toward the most open forward heading and
        # drive when the way ahead is clear; if boxed in, rotate to escape. This
        # traverses the room quickly so AMCL gets a diverse, disambiguating scan
        # sequence and re-converges fast.
        t = Twist()
        if self.front_clear:
            t.linear.x = self.drive_speed
            # proportional steer toward the open heading (capped)
            t.angular.z = max(-self.spin_speed,
                              min(self.spin_speed, 1.5 * self.open_heading))
        else:
            t.angular.z = self.spin_speed
        self.cmd_pub.publish(t)

    def _stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def _publish_status(self, reason_code: str, recoverable: bool) -> None:
        # SOFTWARE_INTERFACES.md status shape, serialized until the project
        # picks a status message type.
        self.status_pub.publish(String(
            data=f'state={self.state.name.lower()};reason_code={reason_code};'
                 f'recoverable={recoverable};source=nav-localize'))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KidnapRecovery()
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
