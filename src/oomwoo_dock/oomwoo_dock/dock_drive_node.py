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
Reverse the robot into the dock, using the detector's mouth pose.

Three phases, because a differential-drive robot cannot cancel a lateral offset
by turning on the spot:

  GOTO   drive to a staging point on the bay axis, still facing the dock
  TURN   turn until the TAIL points into the bay
  BACK   reverse along the axis, steering on lateral and heading error

The robot has to back in to wash its mops, and its LiDAR sits ahead of the wheel
axle, so the sensor keeps watching the dock over the robot's own tail. Detections
arrive a couple of times a second; odometry carries the estimate in between,
which is why the estimate is kept in the body frame and moved by each command.

  subscribes  ~/dock_pose      geometry_msgs/PoseStamped  (from dock_detector)
  subscribes  odom             nav_msgs/Odometry          (to carry the estimate)
  subscribes  ~/enable         std_msgs/Bool
  publishes   cmd_vel          geometry_msgs/Twist
  publishes   ~/state          std_msgs/String   GOTO/TURN/BACK/DONE/IDLE
  publishes   ~/docked         std_msgs/Bool     (latched)
"""

import math

from geometry_msgs.msg import PoseStamped, Twist

from nav_msgs.msg import Odometry

from oomwoo_dock.dock_template import inv, mul, wrap

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from std_msgs.msg import Bool, String

DEFAULTS = {
    'lidar_offset_m': 0.0745,     # sensor ahead of the wheel axle
    'body_radius_m': 0.1745,
    'bay_depth_m': 0.225,         # mouth to the back face
    'stage_x_m': -0.55,           # staging point, outside the mouth
    'stage_tol_m': 0.04,
    'turn_tol_deg': 3.0,
    'v_approach': 0.14,
    'v_back': 0.06,
    'k_stage_heading': 1.8,
    'k_turn': 1.6,
    'k_lateral': 2.5,
    'k_heading': 1.8,
    'omega_max': 0.8,
    'pose_timeout_s': 3.0,        # no fix for this long: stop
    'seat_margin_m': 0.005,
    'auto_start': True,
    'pub_hz': 20.0,
}


class DockDrive(Node):
    """Drive the robot into the dock from the detector's pose."""

    def __init__(self) -> None:
        """Set up parameters, publishers and subscriptions."""
        super().__init__('dock_drive')
        for name, default in DEFAULTS.items():
            self.declare_parameter(name, default)
        self.state = 'IDLE'
        self.enabled = bool(self._p('auto_start'))
        self.est = None               # dock pose in the BODY frame
        self.t_fix = None
        self.odom = None
        self.cmd = Twist()
        self._docked_val = None
        latched = QoSProfile(
            depth=1, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '~/state', 10)
        self.docked_pub = self.create_publisher(Bool, '~/docked', latched)
        self.create_subscription(PoseStamped, '~/dock_pose', self._on_pose, 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_subscription(Bool, '~/enable', self._on_enable, 10)
        self.create_timer(1.0 / max(self._p('pub_hz'), 1.0), self._tick)
        self._set_state('GOTO' if self.enabled else 'IDLE')

    def _p(self, name):
        return self.get_parameter(name).value

    def _set_state(self, s) -> None:
        if s == self.state:
            return
        self.state = s
        self.state_pub.publish(String(data=s))
        docked = s == 'DONE'
        if docked != self._docked_val:
            self._docked_val = docked
            self.docked_pub.publish(Bool(data=docked))
        self.get_logger().info('state -> %s' % s)

    def _on_enable(self, msg: Bool) -> None:
        self.enabled = bool(msg.data)
        self._set_state('GOTO' if self.enabled else 'IDLE')

    def _on_pose(self, msg: PoseStamped) -> None:
        """Dock pose in the SCAN frame; shift it to the body frame."""
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        t_l_d = (msg.pose.position.x, msg.pose.position.y, yaw)
        self.est = mul((self._p('lidar_offset_m'), 0.0, 0.0), t_l_d)
        self.t_fix = self.get_clock().now()

    def _on_odom(self, msg: Odometry) -> None:
        """Carry the estimate between fixes using the robot's own motion."""
        now = self.get_clock().now()
        if self.odom is not None and self.est is not None:
            dt = (now - self.odom).nanoseconds * 1e-9
            if 0.0 < dt < 0.5:
                ds = msg.twist.twist.linear.x * dt
                dth = msg.twist.twist.angular.z * dt
                self.est = mul(inv((ds, 0.0, dth)), self.est)
        self.odom = now

    def _tick(self) -> None:
        """Run the state machine and publish a command."""
        if not self.enabled or self.state in ('IDLE', 'DONE'):
            self._send(0.0, 0.0)
            return
        if self.est is None or self.t_fix is None:
            self._send(0.0, 0.0)
            return
        age = (self.get_clock().now() - self.t_fix).nanoseconds * 1e-9
        if age > self._p('pose_timeout_s'):
            self.get_logger().warn('no dock fix for %.1f s; holding' % age,
                                   throttle_duration_sec=2.0)
            self._send(0.0, 0.0)
            return

        x, y, th = inv(self.est)            # robot pose in the DOCK frame
        wmax = self._p('omega_max')
        seat_x = self._p('bay_depth_m') - self._p('body_radius_m')
        if self.state == 'GOTO':
            dx, dy = self._p('stage_x_m') - x, -y
            if math.hypot(dx, dy) < self._p('stage_tol_m'):
                self._set_state('TURN')
                self._send(0.0, 0.0)
                return
            to_stage = wrap(math.atan2(dy, dx) - th)
            w = _clamp(self._p('k_stage_heading') * to_stage, -wmax, wmax)
            self._send(self._p('v_approach') * max(0.0, math.cos(to_stage)), w)
        elif self.state == 'TURN':
            err = wrap(th - math.pi)        # tail must point into the bay
            if abs(err) < math.radians(self._p('turn_tol_deg')):
                self._set_state('BACK')
                self._send(0.0, 0.0)
                return
            self._send(0.0, _clamp(-self._p('k_turn') * err, -wmax, wmax))
        else:
            if x + self._p('body_radius_m') >= (self._p('bay_depth_m')
                                                - self._p('seat_margin_m')) \
                    or x > seat_x:
                self._set_state('DONE')
                self._send(0.0, 0.0)
                self.get_logger().info(
                    'docked: lateral %+.1f mm, yaw %+.2f deg'
                    % (y * 1e3, math.degrees(wrap(th - math.pi))))
                return
            w = -(self._p('k_lateral') * y + self._p('k_heading') * wrap(th - math.pi))
            self._send(-self._p('v_back'), _clamp(w, -wmax, wmax))

    def _send(self, v, w) -> None:
        self.cmd.linear.x = float(v)
        self.cmd.angular.z = float(w)
        self.cmd_pub.publish(self.cmd)


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def main(args=None) -> None:
    """Spin the docking driver until shutdown."""
    rclpy.init(args=args)
    node = DockDrive()
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
