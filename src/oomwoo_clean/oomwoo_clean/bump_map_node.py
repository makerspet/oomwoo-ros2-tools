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
Tactile "bump" obstacle map from bumper contacts.

The LiDAR/SLAM map is a LOCALIZATION map: couch skirts, bed valances and curtains
look solid to LiDAR/cameras but a vacuum should push under/through them. Only a
physical contact proves something is truly solid. This node builds that second,
tactile layer -- the real keep-out map -- from the bumpers alone.

On each fresh bump it looks up the robot pose in `map` (falling back to `odom`
if not localized), places the contact point on the bumper arc
(center + contact_radius along the heading, offset toward the bumper that fired),
and:
  * accumulates it into a sparse occupancy grid -> /bump_map (latched), a cell
    counts hits so repeated contacts reinforce a wall and a one-off (a nudged
    chair, a cat) stays faint;
  * connects consecutive nearby contacts into wall segments -> /bump_map/walls
    (LINE_LIST) + the raw contact points, for a clean RViz vector view.

Interfaces:
  subscribes  bumper_left/contact   ros_gz_interfaces/Contacts
  subscribes  bumper_right/contact  ros_gz_interfaces/Contacts
  publishes   bump_map              nav_msgs/OccupancyGrid          (latched)
  publishes   bump_map/walls        visualization_msgs/MarkerArray  (latched)
  uses TF     <target_frame> -> base_footprint  (map, else odom)

Values are ROS parameters; the launch seeds them from `kaia set bump_map.*` and
they are live (ros2 param set / kaia set retunes future contacts).
"""

import math

from geometry_msgs.msg import Point

from nav_msgs.msg import OccupancyGrid

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
        # frames: map is preferred (drift-free, matches SLAM); odom is the
        # fallback when not localized. The frame is locked on the first contact.
        self.target_frame = self.declare_parameter('target_frame', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter(
            'base_frame', 'base_footprint').value
        # contact geometry: the obstacle surface is contact_radius out from the
        # robot center, toward the bumper that fired (both=ahead, left/right off
        # by contact_offset_deg). contact_radius ~ body radius + bumper proud.
        self.radius = self.declare_parameter('contact_radius', 0.18).value
        self.side_deg = self.declare_parameter('contact_offset_deg', 45.0).value
        self.res = self.declare_parameter('resolution', 0.05).value
        self.min_hits = self.declare_parameter('occupied_min_hits', 1).value
        self.max_gap = self.declare_parameter('max_segment_gap', 1.0).value
        self.refractory = self.declare_parameter('refractory_sec', 0.8).value
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
        self.create_subscription(
            Contacts, 'bumper_left/contact', self._left_cb, 10)
        self.create_subscription(
            Contacts, 'bumper_right/contact', self._right_cb, 10)

        self._left_t = None
        self._right_t = None
        self._last_bump = self.get_clock().now()
        self.frame = None            # locked target frame (map or odom)
        self.cells = {}              # (ix, iy) -> hit count
        self.runs = []               # list of polylines [[(x, y), ...], ...]

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
        # Lock to map if its transform is available, else odom. Sticking to one
        # frame keeps the accumulated grid consistent.
        if self.frame is not None:
            return self.frame
        for f in (self.target_frame, self.odom_frame):
            if self.tf_buffer.can_transform(f, self.base_frame, Time()):
                self.frame = f
                self.get_logger().info('bump_map: mapping in frame [%s]' % f)
                return f
        return None

    def _tick(self) -> None:
        now = self.get_clock().now()
        left = self._pressed(self._left_t, now)
        right = self._pressed(self._right_t, now)
        if not (left or right):
            return
        # one registration per physical bump: ignore contacts within the
        # refractory window of the last (the sensor fires while held).
        if (now - self._last_bump) < Duration(seconds=self.refractory):
            return
        side = 'both' if left and right else ('left' if left else 'right')
        if self._register(side):
            self._last_bump = now

    def _register(self, side) -> bool:
        frame = self._resolve_frame()
        if frame is None:
            self.get_logger().warn(
                'bump_map: no TF to map/odom yet; skipping contact',
                throttle_duration_sec=5.0)
            return False
        try:
            tr = self.tf_buffer.lookup_transform(frame, self.base_frame, Time())
        except TransformException as err:
            self.get_logger().warn(
                'bump_map: TF lookup failed: %s' % err,
                throttle_duration_sec=5.0)
            return False
        x = tr.transform.translation.x
        y = tr.transform.translation.y
        th = yaw_from_quat(tr.transform.rotation)
        # place the contact on the bumper arc: ahead for a head-on (both) hit,
        # offset to the side of the bumper that fired.
        phi = math.radians(self.side_deg) * (
            1.0 if side == 'left' else -1.0 if side == 'right' else 0.0)
        cx = x + self.radius * math.cos(th + phi)
        cy = y + self.radius * math.sin(th + phi)
        self._add(cx, cy)
        return True

    def _add(self, x, y) -> None:
        key = (int(math.floor(x / self.res)), int(math.floor(y / self.res)))
        self.cells[key] = self.cells.get(key, 0) + 1
        # extend the current wall run if the last contact is close, else the
        # robot has jumped (a corner / a new surface) -> start a new run.
        if self.runs and self.runs[-1]:
            px, py = self.runs[-1][-1]
            if math.hypot(x - px, y - py) <= self.max_gap:
                self.runs[-1].append((x, y))
            else:
                self.runs.append([(x, y)])
        else:
            self.runs.append([(x, y)])
        self._publish()

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

    def _marker(self, mid, mtype, scale, color) -> Marker:
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'bump_map'
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.scale.x = scale
        m.scale.y = scale
        m.color = color
        m.pose.orientation.w = 1.0
        return m

    def _publish_markers(self) -> None:
        walls = self._marker(
            0, Marker.LINE_LIST, 0.02, ColorRGBA(r=0.9, g=0.2, b=0.2, a=1.0))
        for run in self.runs:
            for i in range(len(run) - 1):
                walls.points.append(Point(x=run[i][0], y=run[i][1], z=0.05))
                walls.points.append(
                    Point(x=run[i + 1][0], y=run[i + 1][1], z=0.05))
        pts = self._marker(
            1, Marker.POINTS, 0.04, ColorRGBA(r=1.0, g=0.9, b=0.1, a=1.0))
        for run in self.runs:
            for (x, y) in run:
                pts.points.append(Point(x=x, y=y, z=0.05))
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
