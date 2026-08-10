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
Tactile "bump" obstacle map from the two bumper switches.

The LiDAR/SLAM map is a LOCALIZATION map: couch skirts, bed valances and curtains
look solid to LiDAR/cameras but a vacuum should push under/through them. Only a
physical contact proves something is truly solid. This node builds that second,
tactile layer -- the real keep-out map -- from the bumpers alone.

The real bumper is TWO switches (left half, right half of the front arc), so a
bump gives left/right/both -- not an exact angle. On each fresh bump this node:
  * looks up the robot pose in `map` (falling back to `odom` if not localized);
  * places the contact along the APPROACH direction (a diff-drive moves along
    its heading, taken from a short pose history) at contact_radius, biased
    toward the side that fired -- more principled than a fixed per-half angle;
  * accumulates it into a sparse occupancy grid -> /bump_map (latched); a cell
    counts hits so repeated contacts reinforce a wall and a one-off stays faint;
  * connects consecutive contacts into wall segments -> /bump_map/walls, but
    starts a NEW segment when the sides alternate left<->right (the wall ended)
    or the gap is too big -- same-side or a `both` continues the run;
  * publishes a /bump_event (oomwoo_msgs/BumpEvent): contact, robot, approach,
    heading, side -- the seam where richer optical-interrupter bumpers (per-side
    depress amount + rate -> exact contact angle) plug in later.

Values are ROS parameters; the launch seeds them from `kaia set bump_map.*` and
they are live.
"""

from collections import deque
import math

from geometry_msgs.msg import Point

from nav_msgs.msg import OccupancyGrid

from oomwoo_msgs.msg import BumpEvent

from rcl_interfaces.msg import SetParametersResult

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.time import Time

from ros_gz_interfaces.msg import Contacts

from std_msgs.msg import ColorRGBA

from tf2_ros import Buffer, TransformException, TransformListener

from visualization_msgs.msg import Marker, MarkerArray

SIDE_CODE = {'both': BumpEvent.SIDE_BOTH,
             'left': BumpEvent.SIDE_LEFT,
             'right': BumpEvent.SIDE_RIGHT}
SIDE_COLOR = {'both': (1.0, 0.9, 0.1), 'left': (0.2, 0.4, 1.0),
              'right': (0.2, 0.9, 0.3)}


def latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1, history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def yaw_from_quat(q) -> float:
    # planar (z) yaw from a quaternion
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class BumpMap(Node):

    def __init__(self) -> None:
        super().__init__('bump_map')
        self.target_frame = self.declare_parameter('target_frame', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter(
            'base_frame', 'base_footprint').value
        self.radius = self.declare_parameter('contact_radius', 0.18).value
        self.side_deg = self.declare_parameter('contact_offset_deg', 45.0).value
        self.res = self.declare_parameter('resolution', 0.05).value
        self.min_hits = self.declare_parameter('occupied_min_hits', 1).value
        self.max_gap = self.declare_parameter('max_segment_gap', 1.0).value
        self.refractory = self.declare_parameter('refractory_sec', 0.8).value
        self.approach_dt = self.declare_parameter('approach_dt', 0.5).value
        self.hist_secs = self.declare_parameter('history_sec', 1.5).value
        fresh = self.declare_parameter('bumper_fresh_sec', 0.3).value
        self.fresh = Duration(seconds=fresh)
        hz = self.declare_parameter('control_hz', 20.0).value
        pub_hz = self.declare_parameter('publish_hz', 2.0).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.grid_pub = self.create_publisher(
            OccupancyGrid, 'bump_map', latched_qos())
        self.marker_pub = self.create_publisher(
            MarkerArray, 'bump_map/walls', latched_qos())
        self.event_pub = self.create_publisher(BumpEvent, 'bump_event', 10)
        self.create_subscription(
            Contacts, 'bumper_left/contact', self._left_cb, 10)
        self.create_subscription(
            Contacts, 'bumper_right/contact', self._right_cb, 10)

        self._left_t = None
        self._right_t = None
        self._last_bump = self.get_clock().now()
        self._last_side = None
        self._hist = deque()         # (stamp, x, y, yaw) in self.frame
        self.frame = None            # locked target frame (map or odom)
        self.cells = {}              # (ix, iy) -> hit count
        self.runs = []               # wall polylines [[(x, y), ...], ...]
        self.pts = []                # contact points [(x, y, side), ...]

        self.create_timer(1.0 / hz, self._tick)
        self.create_timer(1.0 / max(pub_hz, 1e-3), self._publish)
        self.add_on_set_parameters_callback(self._on_params)
        self.get_logger().info(
            'bump_map: building the tactile obstacle map from bumper contacts')

    def _on_params(self, params) -> SetParametersResult:
        for p in params:
            if p.name == 'contact_radius':
                self.radius = p.value
            elif p.name == 'contact_offset_deg':
                self.side_deg = p.value
            elif p.name == 'max_segment_gap':
                self.max_gap = p.value
            elif p.name == 'occupied_min_hits':
                self.min_hits = p.value
            elif p.name == 'refractory_sec':
                self.refractory = p.value
            elif p.name == 'approach_dt':
                self.approach_dt = p.value
            elif p.name == 'bumper_fresh_sec':
                self.fresh = Duration(seconds=p.value)
        return SetParametersResult(successful=True)

    def _left_cb(self, msg: Contacts) -> None:
        if msg.contacts:
            self._left_t = self.get_clock().now()

    def _right_cb(self, msg: Contacts) -> None:
        if msg.contacts:
            self._right_t = self.get_clock().now()

    def _pressed(self, stamp, now) -> bool:
        return stamp is not None and (now - stamp) < self.fresh

    def _resolve_frame(self):
        # Lock to map if its transform is available, else odom.
        if self.frame is not None:
            return self.frame
        for f in (self.target_frame, self.odom_frame):
            if self.tf_buffer.can_transform(f, self.base_frame, Time()):
                self.frame = f
                self.get_logger().info('bump_map: mapping in frame [%s]' % f)
                return f
        return None

    def _lookup(self, frame):
        try:
            tr = self.tf_buffer.lookup_transform(frame, self.base_frame, Time())
        except TransformException as err:
            self.get_logger().warn(
                'bump_map: TF lookup failed: %s' % err,
                throttle_duration_sec=5.0)
            return None
        return (tr.transform.translation.x, tr.transform.translation.y,
                yaw_from_quat(tr.transform.rotation))

    def _tick(self) -> None:
        now = self.get_clock().now()
        self._sample_pose(now)     # keep the approach history, lock frame early
        left = self._pressed(self._left_t, now)
        right = self._pressed(self._right_t, now)
        if not (left or right):
            return
        # one registration per physical bump (the switch fires while held)
        if (now - self._last_bump) < Duration(seconds=self.refractory):
            return
        side = 'both' if left and right else ('left' if left else 'right')
        if self._register(side, now):
            self._last_bump = now
            self._last_side = side

    def _sample_pose(self, now) -> None:
        frame = self._resolve_frame()
        if frame is None:
            return
        pose = self._lookup(frame)
        if pose is None:
            return
        self._hist.append((now, pose[0], pose[1], pose[2]))
        cutoff = now - Duration(seconds=self.hist_secs)
        while self._hist and self._hist[0][0] < cutoff:
            self._hist.popleft()

    def _register(self, side, now) -> bool:
        if not self._hist:
            return False
        _, rx, ry, rth = self._hist[-1]              # robot pose at the bump
        ax, ay = rx, ry                              # approach pose (a bit ago)
        cutoff = now - Duration(seconds=self.approach_dt)
        for (t, hx, hy, _h) in self._hist:
            if t >= cutoff:
                ax, ay = hx, hy
                break
        # approach direction: actual motion if we moved, else the heading
        dx, dy = rx - ax, ry - ay
        adir = math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-3 else rth
        # place the contact along the approach, biased to the side that fired
        phi = math.radians(self.side_deg) * (
            1.0 if side == 'left' else -1.0 if side == 'right' else 0.0)
        cx = rx + self.radius * math.cos(adir + phi)
        cy = ry + self.radius * math.sin(adir + phi)
        self._add(cx, cy, side)
        self._publish_event(now, cx, cy, rx, ry, ax, ay, rth, side)
        return True

    def _add(self, x, y, side) -> None:
        key = (int(math.floor(x / self.res)), int(math.floor(y / self.res)))
        self.cells[key] = self.cells.get(key, 0) + 1
        self.pts.append((x, y, side))
        # continue the current wall run unless the sides alternate left<->right
        # (a two-switch "the wall ended" signal) or the gap is too big.
        cont = bool(self.runs and self.runs[-1])
        if cont:
            px, py = self.runs[-1][-1]
            if math.hypot(x - px, y - py) > self.max_gap:
                cont = False
            elif {self._last_side, side} == {'left', 'right'}:
                cont = False
        if cont:
            self.runs[-1].append((x, y))
        else:
            self.runs.append([(x, y)])
        self._publish()

    def _publish_event(self, now, cx, cy, rx, ry, ax, ay, heading, side) -> None:
        ev = BumpEvent()
        ev.header.frame_id = self.frame
        ev.header.stamp = now.to_msg()
        ev.contact = Point(x=cx, y=cy, z=0.0)
        ev.robot = Point(x=rx, y=ry, z=0.0)
        ev.approach = Point(x=ax, y=ay, z=0.0)
        ev.heading = heading
        ev.side = SIDE_CODE[side]
        self.event_pub.publish(ev)

    def _publish(self) -> None:
        if self.frame is None or not self.cells:
            return
        self._publish_grid()
        self._publish_markers()

    def _publish_grid(self) -> None:
        xs = [ix for ix, _ in self.cells]
        ys = [iy for _, iy in self.cells]
        margin = 2
        minx, maxx = min(xs) - margin, max(xs) + margin
        miny, maxy = min(ys) - margin, max(ys) + margin
        w = maxx - minx + 1
        h = maxy - miny + 1
        data = [-1] * (w * h)                       # -1 = unknown (not bumped)
        for (ix, iy), hits in self.cells.items():
            if hits >= self.min_hits:
                data[(iy - miny) * w + (ix - minx)] = 100
        grid = OccupancyGrid()
        grid.header.frame_id = self.frame
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = self.res
        grid.info.width = w
        grid.info.height = h
        grid.info.origin.position.x = minx * self.res
        grid.info.origin.position.y = miny * self.res
        grid.info.origin.orientation.w = 1.0
        grid.data = data
        self.grid_pub.publish(grid)

    def _marker(self, mid, mtype, scale) -> Marker:
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'bump_map'
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.scale.x = scale
        m.scale.y = scale
        m.pose.orientation.w = 1.0
        return m

    def _publish_markers(self) -> None:
        walls = self._marker(0, Marker.LINE_LIST, 0.02)
        walls.color = ColorRGBA(r=0.9, g=0.2, b=0.2, a=1.0)
        for run in self.runs:
            for i in range(len(run) - 1):
                walls.points.append(Point(x=run[i][0], y=run[i][1], z=0.05))
                walls.points.append(
                    Point(x=run[i + 1][0], y=run[i + 1][1], z=0.05))
        pts = self._marker(1, Marker.POINTS, 0.04)
        for (x, y, side) in self.pts:
            pts.points.append(Point(x=x, y=y, z=0.05))
            r, g, b = SIDE_COLOR[side]
            pts.colors.append(ColorRGBA(r=r, g=g, b=b, a=1.0))
        self.marker_pub.publish(MarkerArray(markers=[walls, pts]))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BumpMap()
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
