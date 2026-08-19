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
Detect when the robot is lost and relocalize it -- the hybrid recovery.

slam_toolbox scan-matching is accurate but cannot recover from a kidnap (it only
tracks LOCALLY). AMCL can scatter particles across the whole map and re-localize
globally, but is only decimetre-accurate. This node makes them cooperate:

  1. watch AMCL's pose covariance; when its xy-trace stays above diverge_trace
     the robot is LOST (or subscribe to /kidnap_trigger for the sim);
  2. call /reinitialize_global_localization (scatter AMCL across the map) and
     spin in place so AMCL gets the rotational views it needs to converge -- the
     "slow spin to find itself" a lost vacuum does;
  3. once AMCL's trace collapses below converge_trace, stop spinning and publish
     its recovered pose to /initialpose, which RE-SEEDS slam_toolbox (and clears
     its scan buffer) -- so accurate scan-matching localization resumes and
     navigation is back to normal.

For the covariance signal to rise on a kidnap, AMCL needs recovery_alpha_slow/
fast > 0 (localization_relocalize.launch.py sets them).

  sub  /amcl_pose        geometry_msgs/PoseWithCovarianceStamped
  sub  /kidnap_trigger   std_msgs/Empty   (optional, sim convenience)
  pub  /cmd_vel          geometry_msgs/Twist              (spin while recovering)
  pub  /initialpose      geometry_msgs/PoseWithCovarianceStamped  (re-seed)
  call /reinitialize_global_localization              std_srvs/Empty
  call /slam_toolbox/clear_localization_buffer        std_srvs/Empty
"""

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist

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

from std_msgs.msg import Empty as EmptyMsg

from std_srvs.srv import Empty as EmptySrv

OK, RECOVERING = 'ok', 'recovering'


def amcl_qos() -> QoSProfile:
    return QoSProfile(
        depth=5, history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE)


class RelocalizeOnLost(Node):

    def __init__(self) -> None:
        super().__init__('relocalize_on_lost')
        self.diverge = self.declare_parameter('diverge_trace', 0.5).value
        self.converge = self.declare_parameter('converge_trace', 0.25).value
        self.lost_hold = self.declare_parameter('lost_hold_s', 1.0).value
        self.conv_hold = self.declare_parameter('converge_hold_s', 2.0).value
        self.spin_speed = self.declare_parameter('spin_speed', 0.6).value
        self.max_recovery = self.declare_parameter('max_recovery_s', 60.0).value
        use_trigger = self.declare_parameter('use_kidnap_trigger', True).value
        hz = self.declare_parameter('rate', 10.0).value

        self._amcl = None            # (PoseWithCovariance, xy_cov_trace)
        self.state = OK
        self._t_lost = None
        self._t_conv = None
        self._t_start = None

        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, amcl_qos())
        if use_trigger:
            self.create_subscription(
                EmptyMsg, '/kidnap_trigger', self._on_kidnap, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.init_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.reinit = self.create_client(
            EmptySrv, '/reinitialize_global_localization')
        self.clear = self.create_client(
            EmptySrv, '/slam_toolbox/clear_localization_buffer')
        self.create_timer(1.0 / max(hz, 1e-3), self._tick)
        self.get_logger().info(
            'relocalize_on_lost: watching /amcl_pose covariance '
            '(lost > %.2f, recovered < %.2f)' % (self.diverge, self.converge))

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        c = msg.pose.covariance
        self._amcl = (msg.pose, c[0] + c[7])   # xx + yy variance

    def _on_kidnap(self, _msg: EmptyMsg) -> None:
        if self.state == OK:
            self.get_logger().warn('kidnap trigger -- starting recovery')
            self._enter_recovery()

    def _tick(self) -> None:
        if self._amcl is None:
            return
        now = self.get_clock().now()
        trace = self._amcl[1]
        if self.state == OK:
            if trace > self.diverge:
                if self._t_lost is None:
                    self._t_lost = now
                elif (now - self._t_lost) > Duration(seconds=self.lost_hold):
                    self.get_logger().warn(
                        'LOST (amcl xy-cov trace %.2f) -- relocalizing' % trace)
                    self._enter_recovery()
            else:
                self._t_lost = None
        elif self.state == RECOVERING:
            tw = Twist()
            tw.angular.z = self.spin_speed          # spin to help AMCL converge
            self.cmd_pub.publish(tw)
            if trace < self.converge:
                if self._t_conv is None:
                    self._t_conv = now
                elif (now - self._t_conv) > Duration(seconds=self.conv_hold):
                    self._finish_recovery('converged')
                    return
            else:
                self._t_conv = None
            if (now - self._t_start) > Duration(seconds=self.max_recovery):
                self._finish_recovery('timed out')

    def _enter_recovery(self) -> None:
        self.state = RECOVERING
        self._t_lost = None
        self._t_conv = None
        self._t_start = self.get_clock().now()
        if self.reinit.service_is_ready():
            self.reinit.call_async(EmptySrv.Request())
            self.get_logger().info('called /reinitialize_global_localization')

    def _finish_recovery(self, why: str) -> None:
        self.cmd_pub.publish(Twist())               # stop spinning
        if self._amcl is not None:                  # re-seed slam_toolbox + amcl
            ip = PoseWithCovarianceStamped()
            ip.header.frame_id = 'map'
            ip.header.stamp = self.get_clock().now().to_msg()
            ip.pose = self._amcl[0]
            self.init_pub.publish(ip)
        if self.clear.service_is_ready():
            self.clear.call_async(EmptySrv.Request())
        self.get_logger().warn(
            'RELOCALIZED (%s) -- re-seeded slam_toolbox at the AMCL pose' % why)
        self.state = OK
        self._t_conv = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RelocalizeOnLost()
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
