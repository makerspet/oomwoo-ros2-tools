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
Drive Nav2 to an RViz 2D Goal Pose, marking the floor clean along the way.

Bridges the RViz "2D Goal Pose" tool (``/goal_pose``) to Nav2's
``NavigateToPose`` action, so dropping a goal in RViz makes the robot drive
there on the loaded map, and latches ``cleaning_active`` true so the
coverage_meter accounts for every cell the robot covers -- not just the goal
leg. The vacuum "cleans wherever it drives".

This is the first, deliberately dumb, cleaning-with-map step: stock Nav2
point-to-point, no coverage planning. It is also the seam where the from-scratch
planner lands later -- it will replace the straight NavigateToPose forward with
a coverage plan for the region around the goal.

Interfaces (as wired):
  subscribes  goal_pose        geometry_msgs/PoseStamped  (RViz 2D Goal Pose)
  action clnt navigate_to_pose nav2_msgs/NavigateToPose
  publishes   cleaning_active  std_msgs/Bool              (latched; always True)
"""

from geometry_msgs.msg import PoseStamped

from nav2_msgs.action import NavigateToPose

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from std_msgs.msg import Bool


def latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1, history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class CleanToGoal(Node):

    def __init__(self) -> None:
        super().__init__('clean_to_goal')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_handle = None
        # Latch cleaning ON: a vacuum cleans wherever it drives, so the
        # coverage_meter should score the whole path, not only the goal leg.
        # Latched (transient_local) so the meter counts it whenever it subscribes.
        self._active_pub = self.create_publisher(
            Bool, 'cleaning_active', latched_qos())
        self._active_pub.publish(Bool(data=True))
        self.create_subscription(PoseStamped, 'goal_pose', self._on_goal, 10)
        self.get_logger().info(
            'clean_to_goal ready -- drop a 2D Goal Pose in RViz')

    def _on_goal(self, msg: PoseStamped) -> None:
        if not self._client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(
                'Nav2 navigate_to_pose action server not up yet; ignoring goal')
            return
        goal = NavigateToPose.Goal()
        goal.pose = msg
        self.get_logger().info(
            'navigating to (%.2f, %.2f)'
            % (msg.pose.position.x, msg.pose.position.y))
        self._client.send_goal_async(goal).add_done_callback(self._on_response)

    def _on_response(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('goal rejected by Nav2')
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        # status 4 == SUCCEEDED (action_msgs/GoalStatus)
        self.get_logger().info('goal finished (status %d)' % future.result().status)
        self._goal_handle = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CleanToGoal()
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
