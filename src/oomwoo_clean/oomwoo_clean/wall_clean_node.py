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
Reactive bump-based wall cleaning (right wall, counterclockwise).

The old, pre-sensor way a vacuum cleans edges: cruise forward on a gentle arc
toward the wall on the RIGHT, and on a bumper contact back OUT along the arc it
drove in on, then turn LEFT -- peeling along the wall and rounding corners,
tracing the room counterclockwise. No LiDAR, no map, no localization -- bumpers
-> /cmd_vel only. Position the robot at a wall with teleop first, then start
this; it publishes cleaning_active so the coverage_meter scores it. Stop with
Ctrl-C (no loop-closure yet).

Back-out retraces the entry arc (reversed) rather than reversing in a straight
line: retracing keeps the robot on ground it just cleared, so it is less likely
to wedge somewhere new while pulling out of a contact.

The turn angle depends on which bumper hit:
  right only -> small  (still hugging this wall, just peel off)
  left  only -> large  (a corner: swing left onto the next wall)
  both       -> medium (head-on)

All motion values are ROS parameters: the launch seeds them from
`kaia set clean.*`, and they are live -- `ros2 param set /wall_clean arc_radius
2.0` retunes the running robot without a relaunch.

Interfaces:
  subscribes  bumper_left/contact   ros_gz_interfaces/Contacts
  subscribes  bumper_right/contact  ros_gz_interfaces/Contacts
  publishes   cmd_vel               geometry_msgs/Twist
  publishes   cleaning_active       std_msgs/Bool   (latched True)
"""

import math

from geometry_msgs.msg import Twist

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

from ros_gz_interfaces.msg import Contacts

from std_msgs.msg import Bool

CRUISE, BACKOFF, TURN = 'cruise', 'backoff', 'turn'


def latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1, history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class WallClean(Node):

    def __init__(self) -> None:
        super().__init__('wall_clean')
        self.v_cruise = self.declare_parameter('v_cruise', 0.15).value
        self.arc_radius = self.declare_parameter('arc_radius', 1.5).value
        self.v_back = self.declare_parameter('v_back', 0.10).value
        self.backoff_s = self.declare_parameter('backoff_s', 0.5).value
        self.turn_speed = self.declare_parameter('turn_speed', 0.7).value
        self.turn_deg = {
            'right': self.declare_parameter('turn_right_deg', 20.0).value,
            'left': self.declare_parameter('turn_left_deg', 90.0).value,
            'both': self.declare_parameter('turn_both_deg', 60.0).value,
        }
        fresh = self.declare_parameter('bumper_fresh_sec', 0.3).value
        self.fresh = Duration(seconds=fresh)
        hz = self.declare_parameter('control_hz', 20.0).value

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        active = self.create_publisher(Bool, 'cleaning_active', latched_qos())
        active.publish(Bool(data=True))
        self.create_subscription(
            Contacts, 'bumper_left/contact', self._left_cb, 10)
        self.create_subscription(
            Contacts, 'bumper_right/contact', self._right_cb, 10)

        self._bump_left_t = None
        self._bump_right_t = None
        self._side = 'both'
        # First leg: I aim the robot at the wall with teleop and launch this, so
        # the approach should be a STRAIGHT line (no arc to compensate for). The
        # arc kicks in only after the first bump, once we're following the wall.
        self._first_leg = True
        # angular rate of the cruise leg we're on, captured at bump time so
        # BACKOFF can retrace that arc in reverse (0.0 = the straight first leg).
        self._entry_arc = 0.0
        self.state = CRUISE
        self.until = self.get_clock().now()
        self.create_timer(1.0 / hz, self._tick)
        # live tuning: `ros2 param set /wall_clean arc_radius 2.0`
        self.add_on_set_parameters_callback(self._on_params)
        self.get_logger().info(
            'wall_clean: reactive right-wall cleaning -- Ctrl-C to stop')

    def _on_params(self, params) -> SetParametersResult:
        # apply live parameter changes so tuning takes effect without a relaunch
        for p in params:
            if p.name == 'v_cruise':
                self.v_cruise = p.value
            elif p.name == 'arc_radius':
                self.arc_radius = p.value
            elif p.name == 'v_back':
                self.v_back = p.value
            elif p.name == 'backoff_s':
                self.backoff_s = p.value
            elif p.name == 'turn_speed':
                self.turn_speed = p.value
            elif p.name == 'turn_right_deg':
                self.turn_deg['right'] = p.value
            elif p.name == 'turn_left_deg':
                self.turn_deg['left'] = p.value
            elif p.name == 'turn_both_deg':
                self.turn_deg['both'] = p.value
            elif p.name == 'bumper_fresh_sec':
                self.fresh = Duration(seconds=p.value)
        return SetParametersResult(successful=True)

    def _left_cb(self, msg: Contacts) -> None:
        if msg.contacts:
            self._bump_left_t = self.get_clock().now()

    def _right_cb(self, msg: Contacts) -> None:
        if msg.contacts:
            self._bump_right_t = self.get_clock().now()

    def _pressed(self, stamp, now) -> bool:
        return stamp is not None and (now - stamp) < self.fresh

    def _drive(self, lin, ang) -> None:
        msg = Twist()
        msg.linear.x = float(lin)
        msg.angular.z = float(ang)
        self.pub.publish(msg)

    def _cruise_omega(self) -> float:
        # Arc rate from the chosen radius: omega = v / r. Driving it from
        # arc_radius (rather than a fixed omega) keeps the arc SHAPE constant
        # at any cruise speed -- same path whether cruising fast or slow.
        return self.v_cruise / max(self.arc_radius, 1e-3)

    def _tick(self) -> None:
        now = self.get_clock().now()
        if self.state == CRUISE:
            # forward + gentle RIGHT arc, drifting toward the wall on the right;
            # the very first leg (teleop-aimed approach) drives dead straight
            arc = 0.0 if self._first_leg else -self._cruise_omega()
            self._drive(self.v_cruise, arc)
            left = self._pressed(self._bump_left_t, now)
            right = self._pressed(self._bump_right_t, now)
            if left or right:
                self._first_leg = False   # from here on, arc toward the wall
                self._entry_arc = arc     # remember this leg's arc to retrace it
                self._side = 'both' if left and right else (
                    'left' if left else 'right')
                self.state = BACKOFF
                self.until = now + Duration(seconds=self.backoff_s)
        elif self.state == BACKOFF:
            # Back OUT along the entry arc, reversed. Reversing a differential-
            # drive path exactly means negating BOTH linear and angular velocity,
            # so at the backoff speed the retrace rate is -entry_arc scaled by
            # v_back/v_cruise (same path curvature, radius arc_radius, opposite
            # travel direction). A straight first leg (entry_arc 0) backs straight.
            retrace = -self._entry_arc * self.v_back / max(self.v_cruise, 1e-3)
            self._drive(-self.v_back, retrace)
            if now >= self.until:
                secs = math.radians(self.turn_deg[self._side]) \
                    / max(self.turn_speed, 1e-3)
                self.state = TURN
                self.until = now + Duration(seconds=secs)
        elif self.state == TURN:
            self._drive(0.0, self.turn_speed)           # rotate LEFT / CCW
            if now >= self.until:
                self.state = CRUISE

    def stop(self) -> None:
        self._drive(0.0, 0.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WallClean()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
