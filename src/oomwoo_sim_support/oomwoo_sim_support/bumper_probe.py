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
r"""
Drive the robot into walls at three angles and check each bumper half fires.

The sim bumpers can fail SILENTLY. gz-sim's URDF->SDF conversion renames every
collision when it lumps fixed-joint links, so the contact sensors must list the
renamed names, which depend on collision order on base_link. A body collision
that reaches further than the bumper does the same damage from the other side:
the body touches first and the bumper never registers. oomwoo-one's
test_bumper_wiring.py checks the names; only driving into something checks the
geometry. Run this after any change to the robot's collision shapes.

Needs the contour torture course, robot spawned facing +x near its south-east
corner, where there is a wall ahead and one to the right:

    ros2 launch oomwoo_gazebo world.launch.py world:=contour_torture.world \
        x_pose:=2.3 y_pose:=-2.6 headless:=true
    ros2 run oomwoo_sim_support bumper_probe

Hits, with turns measured on /odom so they do not depend on sim speed:

    heading   0 deg  into the east wall    head-on     either half, or both
    heading -60 deg  into the south wall   hit at -30  right half only
    heading -30 deg  into the east wall    hit at +30  left half only

Exits 0 when every hit fires on the expected side, 1 otherwise.
"""

import math
import sys
import time

from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node

from ros_gz_interfaces.msg import Contacts

# (label, heading to turn to in deg, sides allowed to fire, sides required)
TESTS = [
    ('head-on into the east wall (0 deg)', 0.0, {'left', 'right'}, None),
    ('into the south wall, hit at -30 deg', -60.0, {'right'}, {'right'}),
    ('into the east wall, hit at +30 deg', -30.0, {'left'}, {'left'}),
]
SPEED = 0.08          # m/s toward the wall
TURN = 0.5            # rad/s in place
BACK_OFF_M = 0.25
TIMEOUT_S = 60.0      # wall clock, per phase


class BumperProbe(Node):
    """Drive into walls and count contacts per bumper half."""

    def __init__(self):
        """Set up the command publisher and the contact and odometry feeds."""
        super().__init__('bumper_probe')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.hits = {'left': 0, 'right': 0}
        self.yaw = None
        self.xy = None
        self.create_subscription(Contacts, '/bumper_left/contact',
                                 lambda m: self._hit('left', m), 10)
        self.create_subscription(Contacts, '/bumper_right/contact',
                                 lambda m: self._hit('right', m), 10)
        self.create_subscription(Odometry, '/odom', self._odom, 10)

    def _hit(self, side, msg):
        if msg.contacts:
            self.hits[side] += 1

    def _odom(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _send(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.pub.publish(t)
        rclpy.spin_once(self, timeout_sec=0.05)

    def _stop(self):
        for _ in range(5):
            self._send(0.0, 0.0)

    def wait_ready(self):
        """Wait for odometry and both bumper topics; False if they never come."""
        end = time.time() + TIMEOUT_S
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
            if (self.yaw is not None
                    and self.count_publishers('/bumper_left/contact')
                    and self.count_publishers('/bumper_right/contact')):
                return True
        return False

    def turn_to(self, heading):
        """Rotate in place to an absolute heading (rad), closing the loop on /odom."""
        end = time.time() + TIMEOUT_S
        while time.time() < end:
            err = math.remainder(heading - self.yaw, 2.0 * math.pi)
            if abs(err) < math.radians(1.0):
                break
            self._send(0.0, max(-TURN, min(TURN, 2.0 * err)))
        self._stop()

    def drive_until_bump(self):
        """Creep forward until a bumper fires, then press a moment longer."""
        self.hits = {'left': 0, 'right': 0}
        end = time.time() + TIMEOUT_S
        while time.time() < end and not any(self.hits.values()):
            self._send(SPEED, 0.0)
        end = time.time() + 1.0                 # both halves get a chance to report
        while time.time() < end:
            self._send(SPEED, 0.0)
        self._stop()
        return dict(self.hits)

    def back_off(self):
        """Reverse a fixed distance, measured on /odom."""
        x0, y0 = self.xy
        end = time.time() + TIMEOUT_S
        while time.time() < end and math.hypot(self.xy[0] - x0,
                                               self.xy[1] - y0) < BACK_OFF_M:
            self._send(-SPEED, 0.0)
        self._stop()


def main(args=None):
    """Run the three hits and report; exit 0 only if all fire on the right side."""
    rclpy.init(args=args)
    probe = BumperProbe()
    ok = probe.wait_ready()
    if not ok:
        print('FAIL: no /odom or no bumper publishers after %.0f s' % TIMEOUT_S)
    else:
        start = probe.yaw
        for label, heading, allowed, required in TESTS:
            probe.turn_to(start + math.radians(heading))
            hits = probe.drive_until_bump()
            fired = {s for s, n in hits.items() if n}
            passed = bool(fired) and fired <= allowed and (not required
                                                           or required <= fired)
            ok &= passed
            print('%-4s %-38s left %4d  right %4d' % (
                'ok' if passed else 'FAIL', label, hits['left'], hits['right']))
            probe.back_off()
    print('bumper probe: %s' % ('PASS' if ok else 'FAIL'))
    probe.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
