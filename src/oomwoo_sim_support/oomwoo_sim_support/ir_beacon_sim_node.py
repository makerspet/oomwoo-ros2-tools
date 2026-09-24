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
Simulate the dock's IR beacon and the robot's two rear receivers.

Gazebo has no IR receiver sensor, so this computes what the pair WOULD see from
ground truth and publishes it like a driver would. It exists to test the docking
behaviour against a beacon before the hardware is built, and to keep the
behaviour honest about what a beacon can and cannot tell you.

The model, in order of what actually limits it:

1. THE BEACON IS RECESSED IN THE BAY, so its light leaves through the mouth in a
   wedge. With the emitter at the back of a 0.40 m wide bay 0.24 m deep, that is
   about +-40 degrees off the bay axis. A robot parked beside the dock sees
   nothing at all, however good its receivers are. This is the single biggest
   constraint on where docking can start, and it is geometry, not electronics.

2. EACH RECEIVER HAS A LOBE, roughly cosine within its field of view, and the
   baffle between them shadows each from the far side beyond about 70 deg.

3. THE BEARING COMES FROM THE BALANCE of the two, which is what the hardware
   does. For cosine lobes splayed +-s, the ratio is exact:

       (L - R) / (L + R) = tan(theta) * tan(s)

   so theta = atan2(balance, tan(s)). Near the centreline this is sensitive and
   well conditioned; past the splay angle it degrades, which is why the arc where
   BOTH receivers see the beacon is the usable one.

Signal strength falls as 1/r^2 and carries noise, so `~/left` and `~/right` look
like something a real receiver would report.

  subscribes  odom_truth   nav_msgs/Odometry   (ground truth; sim only)
  subscribes  odom         nav_msgs/Odometry   (used while odom_truth is silent)

The world publishes the true pose on /odom_truth only with
odom_source:=robot_wheels. With the default, ground_truth, it is /odom that
carries it and /odom_truth stays silent -- which left the receivers blind, and
the robot docking on the LiDAR alone without anyone noticing.
  publishes   ~/left       std_msgs/Float32    (relative signal, 0..1)
  publishes   ~/right      std_msgs/Float32
  publishes   ~/visible    std_msgs/Bool       (both receivers have signal)
  publishes   ~/bearing    std_msgs/Float32    (radians, base_link axes, seen
                                                FROM the receivers' midpoint;
                                                near +-pi when astern)

The bearing is taken where it is measured, at the receivers 0.164 m behind
base_link. Moving it to base_link needs the range, which a bearing sensor does
not have; done with a guessed range it was 4-5 deg out at the window's edge.
"""

import math
import random

from nav_msgs.msg import Odometry

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import Bool, Float32

DEFAULTS = {
    # The dock's MOUTH in the world frame, with yaw pointing INTO the bay --
    # the frame the detector fits, not the model's own origin. For
    # kitchen_dining's vacuum_dock (model at -2.45, -0.5, yaw 90, bay from
    # model y -0.34 to the back face at -0.10) that is (-2.11, -0.5), 180 deg,
    # and the beacon lands on its visual at (-2.345, -0.5). The model pose was
    # passed here at first, which put the beacon 0.24 m off, shining south.
    'dock_x': -2.11,
    'dock_y': -0.5,
    'dock_yaw_deg': 180.0,
    # Beacon position in the DOCK frame (x into the bay from the mouth centre).
    'beacon_x': 0.235,
    'beacon_y': 0.0,
    'beacon_half_angle_deg': 40.0,   # the wedge the bay mouth leaves it
    'beacon_range_m': 3.0,           # beyond this the receivers see nothing
    # Receiver mounting, matching oomwoo-one's URDF.
    'rx_x': -0.164,
    'rx_y': 0.030,
    'rx_splay_deg': 25.0,
    'rx_fov_deg': 50.0,
    'baffle_len': 0.010,             # fin between them, aft of the receivers
    'noise': 0.02,                   # relative, on each receiver
    'publish_hz': 20.0,
}


def wrap(a):
    """Wrap an angle to (-pi, pi]."""
    return math.remainder(a, 2.0 * math.pi)


class IrBeaconSim(Node):
    """Publish what the robot's rear IR receivers would see from the dock."""

    def __init__(self) -> None:
        """Set up parameters, the subscription and the publishers."""
        super().__init__('ir_beacon_sim')
        for name, default in DEFAULTS.items():
            self.declare_parameter(name, default)
        self.pose = None
        self.t_truth = None           # when odom_truth last delivered
        self.left_pub = self.create_publisher(Float32, '~/left', 10)
        self.right_pub = self.create_publisher(Float32, '~/right', 10)
        self.vis_pub = self.create_publisher(Bool, '~/visible', 10)
        self.bearing_pub = self.create_publisher(Float32, '~/bearing', 10)
        self.create_subscription(Odometry, 'odom_truth', self._on_truth, 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_timer(1.0 / max(self._p('publish_hz'), 1.0), self._tick)
        self.get_logger().info(
            'ir_beacon_sim: dock at (%.2f, %.2f, %.0f deg), beacon visible within'
            ' +-%.0f deg of the bay axis'
            % (self._p('dock_x'), self._p('dock_y'), self._p('dock_yaw_deg'),
               self._p('beacon_half_angle_deg')))

    def _p(self, name):
        return self.get_parameter(name).value

    def _on_truth(self, msg: Odometry) -> None:
        self.t_truth = self.get_clock().now()
        self._set_pose(msg)

    def _on_odom(self, msg: Odometry) -> None:
        # /odom is the truth only when /odom_truth is silent
        if self.t_truth is not None and (
                self.get_clock().now() - self.t_truth).nanoseconds < 1e9:
            return
        self._set_pose(msg)

    def _set_pose(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def _beacon_world(self):
        """Beacon position in the world frame."""
        dx, dy = self._p('dock_x'), self._p('dock_y')
        yaw = math.radians(self._p('dock_yaw_deg'))
        bx, by = self._p('beacon_x'), self._p('beacon_y')
        return (dx + bx * math.cos(yaw) - by * math.sin(yaw),
                dy + bx * math.sin(yaw) + by * math.cos(yaw), yaw)

    def _tick(self) -> None:
        """Publish one sample of both receivers."""
        if self.pose is None:
            return
        rx, ry, rth = self.pose
        bx, by, byaw = self._beacon_world()

        # The bay opens along -x of the dock frame, so that is where light goes.
        to_robot = math.atan2(ry - by, rx - bx)
        off_axis = abs(wrap(to_robot - (byaw + math.pi)))
        rng = math.hypot(rx - bx, ry - by)
        lit = (off_axis < math.radians(self._p('beacon_half_angle_deg'))
               and rng < self._p('beacon_range_m'))

        signals = {}
        for side, sign in (('left', 1.0), ('right', -1.0)):
            # receiver pose in the world frame
            mx = rx + (self._p('rx_x') * math.cos(rth)
                       - sign * self._p('rx_y') * math.sin(rth))
            my = ry + (self._p('rx_x') * math.sin(rth)
                       + sign * self._p('rx_y') * math.cos(rth))
            boresight = wrap(rth + math.pi - sign * math.radians(self._p('rx_splay_deg')))
            ang = wrap(math.atan2(by - my, bx - mx) - boresight)
            gain = math.cos(ang) if abs(ang) < math.radians(self._p('rx_fov_deg')) else 0.0
            # the baffle shadows a ray that crosses the centreline before it
            # clears the baffle's end: past atan(rx_y / baffle_len) off astern,
            # about 72 deg for the URDF's fin. Shadowing the whole far side, as
            # first modelled, left both receivers lit only within about 2 deg
            # of dead astern.
            cross = sign * wrap(math.atan2(by - my, bx - mx) - (rth + math.pi))
            if cross < -math.atan2(self._p('rx_y'), self._p('baffle_len')):
                gain = 0.0
            val = max(0.0, gain) / max(rng * rng, 1e-3) if lit else 0.0
            if val > 0.0:
                val *= 1.0 + random.gauss(0.0, self._p('noise'))
            signals[side] = max(0.0, val)

        left, right = signals['left'], signals['right']
        peak = max(left, right, 1e-9)
        self.left_pub.publish(Float32(data=float(min(1.0, left / peak))))
        self.right_pub.publish(Float32(data=float(min(1.0, right / peak))))
        visible = left > 0.0 and right > 0.0
        self.vis_pub.publish(Bool(data=visible))
        if visible:
            balance = (left - right) / (left + right)
            theta = math.atan2(balance, math.tan(math.radians(self._p('rx_splay_deg'))))
            # theta is measured from dead astern, positive to the LEFT receiver's
            # side, which in base_link is a bearing below +pi
            self.bearing_pub.publish(Float32(data=float(wrap(math.pi - theta))))


def main(args=None) -> None:
    """Spin the beacon simulator until shutdown."""
    rclpy.init(args=args)
    node = IrBeaconSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        # the launch's SIGINT can land inside rclpy's message take, which then
        # fails on a half-destroyed message: a traceback, but nothing wrong
        if rclpy.ok():
            raise
        node.get_logger().debug('ignoring shutdown race: %s' % exc)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
