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
Application-logic placeholder: decide what to do when the robot is lost.

localization_health and global_relocalizer are pure mechanism -- health reports
scan-vs-map quality, the relocalizer reports "here is the best pose and how
confident I am". This node owns the POLICY: it watches ~/quality and, when it
stays below lost_ratio past lost_hold_s, DECIDES the robot is lost (publishing
/localization_lost for observers), calls the relocalizer, then on the result
decides whether to trust the fix (commit it to /initialpose) or fall back to a
recovery behavior (drive around, hunt the dock via IR, ...). The fallback is a
STUB here -- it emits the chosen action on ~/recovery_action for a real behavior
node to carry out; actual motion / dock-IR handling comes later. Keeping the
decision here is why global_relocalizer never seeds /initialpose in production
(its publish_initialpose is debug only).

  sub  <quality_topic>      std_msgs/Float32   (default /localization_health/quality)
  cli  <relocalize_service> oomwoo_localization_msgs/Relocalize
  srv  ~/recover            std_srvs/Trigger        (kick recovery by hand)
  pub  /localization_lost   std_msgs/Empty          (edge, when the lost hold trips)
  pub  /initialpose         geometry_msgs/PoseWithCovarianceStamped (on accept)
  pub  ~/recovery_action    std_msgs/String         (commit | <fallback> | ...)
"""

from geometry_msgs.msg import PoseWithCovarianceStamped

from oomwoo_localization_msgs.srv import Relocalize

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import Empty, Float32, String

from std_srvs.srv import Trigger

NOMINAL, RELOCALIZING, FALLBACK = 'nominal', 'relocalizing', 'fallback'


class LocalizationManager(Node):

    def __init__(self) -> None:
        super().__init__('localization_manager')
        self.auto = self.declare_parameter('auto_trigger', True).value
        self.accept_conf = self.declare_parameter(
            'accept_confidence', 0.15).value
        self.seed = self.declare_parameter('seed_on_accept', True).value
        self.fallback = self.declare_parameter(
            'fallback_action', 'dock_search').value
        srv_name = self.declare_parameter(
            'relocalize_service', '/global_relocalizer/relocalize').value
        # the lost DECISION (moved here from localization_health): quality must
        # stay below lost_ratio for lost_hold_s before we call it lost -- that
        # hold is the persistence filter that ignores a one-frame quality dip.
        quality_topic = self.declare_parameter(
            'quality_topic', '/localization_health/quality').value
        self.lost_ratio = self.declare_parameter('lost_ratio', 0.5).value
        self.ok_ratio = self.declare_parameter('ok_ratio', 0.7).value
        self.lost_hold = self.declare_parameter('lost_hold_s', 1.0).value

        self.state = NOMINAL
        self._t_low = None    # first time quality dropped below lost_ratio
        self._armed = True    # False after firing, until quality recovers
        self.cli = self.create_client(Relocalize, srv_name)
        self.create_subscription(Float32, quality_topic, self._on_quality, 10)
        self.lost_pub = self.create_publisher(Empty, '/localization_lost', 10)
        self.init_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.action_pub = self.create_publisher(String, '~/recovery_action', 10)
        self.create_service(Trigger, '~/recover', self._on_recover)
        self.get_logger().info(
            'localization_manager: quality < %.2f for %.1fs -> lost -> '
            'relocalize -> commit if confidence >= %.2f, else %s'
            % (self.lost_ratio, self.lost_hold, self.accept_conf, self.fallback))

    def _on_quality(self, msg: Float32) -> None:
        quality = msg.data
        now = self.get_clock().now()
        if quality < self.lost_ratio:
            if self._t_low is None:
                self._t_low = now
            elif (self._armed
                  and (now - self._t_low) > Duration(seconds=self.lost_hold)):
                self._armed = False
                self.lost_pub.publish(Empty())
                self.get_logger().warn(
                    'LOCALIZATION LOST (quality %.2f < %.2f) -- '
                    '/localization_lost' % (quality, self.lost_ratio))
                if self.auto and self.state == NOMINAL:
                    self._relocalize()
        else:
            self._t_low = None
        if quality >= self.ok_ratio and not self._armed:
            self._armed = True
            self.get_logger().info(
                'localization recovered (quality %.2f) -- re-armed' % quality)

    def _on_recover(self, _req, resp):
        if self.state == RELOCALIZING:
            resp.success = False
            resp.message = 'already relocalizing'
            return resp
        self._relocalize()
        resp.success = True
        resp.message = 'recovery started; watch ~/recovery_action'
        return resp

    def _relocalize(self) -> None:
        if not self.cli.service_is_ready():
            self.get_logger().warn('relocalize service not available')
            self._act('service_unavailable')
            return
        self.state = RELOCALIZING
        self.cli.call_async(
            Relocalize.Request()).add_done_callback(self._result)

    def _result(self, future) -> None:
        resp = future.result()
        if resp is None:
            self.get_logger().warn('relocalize call failed')
            self._fallback()
            return
        p = resp.pose.pose.pose
        self.get_logger().info(
            'relocalizer: conf=%.2f score=%.2f -> %s'
            % (resp.confidence, resp.score_normalized, resp.message))
        if resp.confidence >= self.accept_conf:
            if self.seed:
                self.init_pub.publish(resp.pose)
            self._act('commit')
            self.get_logger().info(
                'committed pose x=%.2f y=%.2f (localization restored)'
                % (p.position.x, p.position.y))
            self.state = NOMINAL
        else:
            self._fallback()

    def _fallback(self) -> None:
        self.state = FALLBACK
        self._act(self.fallback)
        self.get_logger().warn(
            'low-confidence fix -- policy: %s (placeholder). Re-run recovery '
            'via ~/recover once the robot has moved.' % self.fallback)

    def _act(self, action) -> None:
        self.action_pub.publish(String(data=action))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationManager()
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
