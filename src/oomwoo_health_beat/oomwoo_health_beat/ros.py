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
ROS wrapper for work-path heartbeats.

Usage in a node, in __init__ and at the point where one unit of work ends:

    self._health = HealthBeat(self)
    ...
    self._health.beat()

The node parameter health_component_id (default '') switches it on. While it is
empty beat() returns at once and no publisher exists, so the node behaves
exactly as without this package. Heartbeats go to /oomwoo/health/component
(std_msgs/String, JSON, reliable, volatile, depth 10) and are stamped from the
node's own clock, the clock the health monitor must share.
"""

from oomwoo_health_beat.core import BeatBuilder, stamp_from_nanoseconds, to_json

from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from std_msgs.msg import String

COMPONENT_TOPIC = '/oomwoo/health/component'
PARAMETER = 'health_component_id'


def heartbeat_qos() -> QoSProfile:
    """Component heartbeats: reliable, volatile, depth 10 (one shared topic)."""
    return QoSProfile(
        depth=10, history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE)


class HealthBeat:
    """Publish a heartbeat per completed unit of work when enabled."""

    def __init__(self, node, topic: str = COMPONENT_TOPIC) -> None:
        """Declare health_component_id on node; create the publisher if set."""
        self._node = node
        self._builder = None
        self._pub = None
        component_id = node.declare_parameter(PARAMETER, '').value
        if component_id:
            self._builder = BeatBuilder(component_id)
            self._pub = node.create_publisher(String, topic, heartbeat_qos())

    @property
    def enabled(self) -> bool:
        """Return whether health_component_id is set."""
        return self._builder is not None

    def beat(self, level: str = 'ok', detail: str = '') -> None:
        """Call where one unit of real work has just completed."""
        if self._builder is None:
            return
        now = stamp_from_nanoseconds(self._node.get_clock().now().nanoseconds)
        self._pub.publish(String(data=to_json(
            self._builder.build(now, level, detail))))
