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
Cleaning manager: sweep every obstacle edge on the map once, then stop.

The contour follower on its own cleans one edge and never knows when to stop: it
circles a lone table leg forever, because nothing tells it it has been all the
way round. This node gives it a job list. Repeat until there is nothing left:

  1. Find the nearest obstacle edge not yet swept (edge_sweep.EdgeSweep).
  2. Drive there with Nav2, parking on the edge with the obstacle on the
     follower's side.
  3. Enable the follower and watch where it goes. Stop it the moment it runs
     over edge it has already swept (the end of a lap round a leg, or reaching
     the start of an earlier segment), or if it goes LOST, HALTED or stops
     making progress.

Then print a report (edge swept, floor area passed over, distance, time) and
exit. The robot's pose comes from TF (map -> base_footprint), so the same node
runs on the real robot as in the simulator: nothing here reads ground truth.

It needs a map, localization and Nav2 already running, and the contour
follower started with auto_start:=false. edge_clean.launch.py starts the
follower and this node together; see there for the full simulator recipe.

  subscribes  map                        nav_msgs/OccupancyGrid  (latched)
  subscribes  <follower>/state           std_msgs/String
  publishes   <follower>/enable          std_msgs/Bool
  action      navigate_to_pose           nav2_msgs/NavigateToPose
  publishes   ~/state                    std_msgs/String   (latched)
  publishes   ~/edge_ring                nav_msgs/OccupancyGrid (latched; 100 dirty,
                                         99 written off, 110 swept, 0 elsewhere)
  publishes   ~/cleaned                  nav_msgs/OccupancyGrid (latched; 30 passed over,
                                         0 elsewhere)
  publishes   ~/target                   geometry_msgs/PoseStamped

The two grids are coded for RViz's Map display with Color Scheme "costmap",
where 0 draws transparent: dirty edge magenta, written off cyan, swept green,
floor passed over blue (oomwoo_one's edge_clean.rviz sets this up).
"""

import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import numpy as np
from oomwoo_clean.edge_sweep import EdgeSweep
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy)
from rclpy.time import Time
from std_msgs.msg import Bool, String
import tf2_ros

DEFAULTS = {
    'global_frame': 'map',
    'robot_base_frame': 'base_footprint',
    'follower': 'contour_follower',   # node name: <follower>/enable, <follower>/state
    'standoff_m': 0.23,               # must match the follower's
    'follow_side': 'right',           # must match the follower's
    'robot_radius': 0.1745,           # passable = at least this + margin off obstacles
    'clearance_margin': 0.02,
    'cleaning_radius': 0.16,          # floor counted as cleaned around the centre
    'done_radius': 0.12,              # edge counted as swept this close to the centre
    'revisit_lookback_m': 1.0,        # own track older than this much travel is "old"
    'revisit_abort_m': 0.30,          # stop the follower after this much on old track
    'attempt_radius_m': 0.30,         # dirty edge written off around each target
    'min_todo_len_m': 0.30,           # dirty edge shorter than this is not worth a trip
    'nav_timeout_s': 180.0,
    'max_nav_failures': 3,            # in a row: the robot is stuck, not the targets
    'segment_timeout_s': 900.0,
    'stall_s': 45.0,                  # follower driving but sweeping nothing new
    'rate_hz': 10.0,
    'exit_when_done': True,
}


def _yaw_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class CleanManager(Node):
    """Run the contour follower round every obstacle edge on the map."""

    def __init__(self) -> None:
        """Declare parameters, wire up topics, TF and Nav2; wait for a map."""
        super().__init__('clean_manager')
        for name, default in DEFAULTS.items():
            self.declare_parameter(name, default)
        latched = QoSProfile(
            depth=1, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        fol = '/' + self._p('follower').strip('/')
        self.enable_pub = self.create_publisher(Bool, fol + '/enable', 10)
        self.create_subscription(String, fol + '/state', self._on_follower_state, latched)
        self.create_subscription(OccupancyGrid, 'map', self._on_map, latched)
        self.state_pub = self.create_publisher(String, '~/state', latched)
        self.ring_pub = self.create_publisher(OccupancyGrid, '~/edge_ring', latched)
        self.cleaned_pub = self.create_publisher(OccupancyGrid, '~/cleaned', latched)
        self.target_pub = self.create_publisher(PoseStamped, '~/target', latched)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sweep = None
        self.map_msg = None
        self.follower_state = 'IDLE'
        self.state = None
        self.goal_handle = None
        self.nav_result = None           # None pending, else a GoalStatus code
        self.t_state = None
        self.t_start = None
        self.last_new = None             # (swept-cell count, time) for the stall check
        self.done = False
        self.nav_failures = 0            # consecutive; reset by any successful approach
        self._t_pub = None
        self._set_state('WAIT_MAP')
        self.create_timer(1.0 / max(self._p('rate_hz'), 1.0), self._tick)

    def _p(self, name):
        return self.get_parameter(name).value

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _set_state(self, s, why=''):
        if s == self.state:
            return
        self.state = s
        self.t_state = self._now()
        self.state_pub.publish(String(data=s))
        self.get_logger().info('state -> %s%s' % (s, (' (%s)' % why) if why else ''))

    # ------------------------------------------------------------------ inputs
    def _on_map(self, msg):
        if self.sweep is not None:
            return                       # the first map defines the job
        info = msg.info
        grid = np.array(msg.data, dtype=np.int16).reshape(info.height, info.width)
        p = self._p
        self.sweep = EdgeSweep(
            grid, info.resolution, (info.origin.position.x, info.origin.position.y),
            standoff=p('standoff_m'), side=p('follow_side'), robot_radius=p('robot_radius'),
            clearance_margin=p('clearance_margin'), cleaning_radius=p('cleaning_radius'),
            done_radius=p('done_radius'), revisit_lookback_m=p('revisit_lookback_m'),
            revisit_abort_m=p('revisit_abort_m'), attempt_radius_m=p('attempt_radius_m'),
            min_todo_len_m=p('min_todo_len_m'))
        self.map_msg = msg
        self.get_logger().info(
            'map %dx%d @ %.3f m: ~%.0f m of obstacle edge to sweep'
            % (info.width, info.height, info.resolution,
               self.sweep.stats()['edge_total_m']))

    def _on_follower_state(self, msg):
        self.follower_state = msg.data

    def _pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self._p('global_frame'), self._p('robot_base_frame'), Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        tr = t.transform.translation
        return tr.x, tr.y, _yaw_from_quat(t.transform.rotation)

    # ------------------------------------------------------------------ outputs
    def _enable_follower(self, on):
        self.enable_pub.publish(Bool(data=bool(on)))

    def _grid_msg(self, values):
        m = OccupancyGrid()
        m.header.frame_id = self.map_msg.header.frame_id or self._p('global_frame')
        m.header.stamp = self.get_clock().now().to_msg()
        m.info = self.map_msg.info
        m.data = values.astype(np.int8).ravel().tolist()
        return m

    def _publish_grids(self):
        # values picked for RViz's "costmap" colour scheme, where 0 is transparent
        sw = self.sweep
        ring = np.zeros((sw.h, sw.w), np.int16)
        ring[sw.ring] = 100                              # dirty: magenta
        ring[sw.ring & sw.attempted] = 99                # written off: cyan
        ring[sw.ring & sw.done] = 110                    # swept: green
        self.ring_pub.publish(self._grid_msg(ring))
        cleaned = np.zeros((sw.h, sw.w), np.int16)
        cleaned[sw.cleaned & sw.free] = 30               # passed over: blue
        self.cleaned_pub.publish(self._grid_msg(cleaned))

    # ------------------------------------------------------------------ Nav2
    def _send_nav(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = self._p('global_frame')
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        q = _yaw_quat(yaw)
        ps.pose.orientation.x, ps.pose.orientation.y = q[0], q[1]
        ps.pose.orientation.z, ps.pose.orientation.w = q[2], q[3]
        goal.pose = ps
        self.target_pub.publish(ps)
        self.nav_result = None
        self.goal_handle = None
        self.nav.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.nav_result = GoalStatus.STATUS_ABORTED
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f: setattr(self, 'nav_result', f.result().status))

    def _cancel_nav(self):
        if self.goal_handle is not None and self.nav_result is None:
            self.goal_handle.cancel_goal_async()

    # ------------------------------------------------------------------ the loop
    def _tick(self):
        if self.done:
            return
        now = self._now()
        if self.state == 'WAIT_MAP':
            if self.sweep is not None:
                self._set_state('WAIT_READY')
            return
        pose = self._pose()
        if self.state == 'WAIT_READY':
            if pose is not None and self.nav.server_is_ready():
                self._enable_follower(False)
                self.t_start = now
                self._set_state('PICK')
            return
        if pose is None:
            return
        x, y, _yaw = pose
        following = self.state == 'FOLLOW' and self.follower_state in ('FOLLOW', 'ARC')
        revisit = self.sweep.update(x, y, following)
        if self._t_pub is None or now - self._t_pub > 1.0:
            self._t_pub = now
            self._publish_grids()

        if self.state == 'PICK':
            target = self.sweep.next_target(x, y)
            if target is None:
                self._finish()
                return
            self.sweep.begin_segment()
            self.get_logger().info(
                'segment %d: next dirty edge at (%.2f, %.2f), heading %.0f deg'
                % (self.sweep.segments, target[0], target[1], math.degrees(target[2])))
            self._send_nav(*target)
            self._set_state('NAVIGATE')
        elif self.state == 'NAVIGATE':
            if self.nav_result == GoalStatus.STATUS_SUCCEEDED:
                self.nav_failures = 0
                self._enable_follower(True)
                self.last_new = (int(self.sweep.done.sum()), now)
                self._set_state('FOLLOW')
            elif self.nav_result is not None:
                self._end_segment('nav_failed')
            elif now - self.t_state > self._p('nav_timeout_s'):
                self._cancel_nav()
                self._end_segment('nav_timeout')
        elif self.state == 'FOLLOW':
            n = int(self.sweep.done.sum())
            if n > self.last_new[0]:
                self.last_new = (n, now)
            if revisit:
                self._end_segment('revisit')
            elif self.follower_state in ('LOST', 'HALTED'):
                self._end_segment(self.follower_state.lower())
            elif now - self.last_new[1] > self._p('stall_s'):
                self._end_segment('stalled')
            elif now - self.t_state > self._p('segment_timeout_s'):
                self._end_segment('timeout')

    def _end_segment(self, outcome):
        self._enable_follower(False)
        self.sweep.end_segment(outcome)
        st = self.sweep.stats()
        self.get_logger().info(
            'segment %d ended: %s after %.0f s; edge swept %.1f%%'
            % (self.sweep.segments, outcome, self._now() - self.t_state, st['edge_done_pct']))
        if outcome.startswith('nav_'):
            self.nav_failures += 1
            # Several different targets failing in a row says the robot cannot
            # move, not that every one of them is unreachable: stop and say so,
            # rather than writing off the rest of the map one target at a time.
            if self.nav_failures >= self._p('max_nav_failures'):
                self._finish('Nav2 failed %d targets in a row -- the robot looks stuck'
                             % self.nav_failures)
                return
        self._set_state('PICK')

    def _finish(self, stopped_early=''):
        self._enable_follower(False)
        self._publish_grids()
        st = self.sweep.stats()
        secs = self._now() - self.t_start
        outcomes = ', '.join('%s %d' % kv for kv in sorted(st['outcomes'].items()))
        self.get_logger().info(
            'EDGE CLEAN DONE in %dm %02ds: edges %.1f%% swept (~%.0f of ~%.0f m; ~%.1f m '
            'written off), floor passed over %.1f of %.1f m2, driven %.1f m, %d segments '
            '(%s)%s' % (secs // 60, secs % 60, st['edge_done_pct'], st['edge_done_m'],
                        st['edge_total_m'], st['edge_skipped_m'], st['cleaned_m2'],
                        st['free_m2'], st['path_m'], st['segments'], outcomes or 'none',
                        ('. STOPPED EARLY: ' + stopped_early) if stopped_early else ''))
        self._set_state('DONE', stopped_early)
        self.done = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CleanManager()
    try:
        while rclpy.ok() and not (node.done and node._p('exit_when_done')):
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node._enable_follower(False)      # never leave the follower driving
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
