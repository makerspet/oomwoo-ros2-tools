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
"""Tests for the ROS wrapper: disabled by default, enabled by the parameter."""

import json

from builtin_interfaces.msg import Time

from oomwoo_health_beat.ros import COMPONENT_TOPIC, HealthBeat

import pytest

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from rosgraph_msgs.msg import Clock

from std_msgs.msg import String


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _node(name, component_id=None):
    overrides = []
    if component_id is not None:
        overrides = [Parameter('health_component_id', value=component_id)]
    return Node(name, parameter_overrides=overrides)


def _spin_until(node, box, n, tries=50):
    for _ in range(tries):
        rclpy.spin_once(node, timeout_sec=0.05)
        if len(box) >= n:
            return


def test_disabled_by_default_creates_no_publisher():
    node = _node('beat_off')
    beat = HealthBeat(node)
    assert not beat.enabled
    beat.beat()
    beat.beat('error', 'ignored')
    assert node.count_publishers(COMPONENT_TOPIC) == 0
    node.destroy_node()


def test_empty_id_is_disabled():
    node = _node('beat_empty', '')
    assert not HealthBeat(node).enabled
    node.destroy_node()


def test_enabled_publishes_contract_payload():
    node = _node('beat_on', 'unit_test')
    got = []
    node.create_subscription(
        String, COMPONENT_TOPIC, lambda m: got.append(json.loads(m.data)), 10)
    beat = HealthBeat(node)
    assert beat.enabled
    beat.beat(detail='one')
    beat.beat()
    _spin_until(node, got, 2)
    assert [g['sequence'] for g in got] == [1, 2]
    assert got[0]['component_id'] == 'unit_test'
    assert got[0]['health'] == 'ok' and got[0]['detail'] == 'one'
    now = node.get_clock().now().nanoseconds * 1e-9
    assert 0.0 <= now - got[1]['stamp_sec'] < 5.0
    node.destroy_node()


def test_stamp_is_the_node_sim_clock_as_the_monitor_reads_it():
    node = Node('beat_sim', parameter_overrides=[
        Parameter('health_component_id', value='sim'),
        Parameter('use_sim_time', value=True)])
    got = []
    node.create_subscription(
        String, COMPONENT_TOPIC, lambda m: got.append(json.loads(m.data)), 10)
    clock = node.create_publisher(Clock, '/clock', 10)
    tick = 150_000_000                  # tick * 1e-9 would be 0.15000000000000002
    for _ in range(50):
        clock.publish(Clock(clock=Time(sec=0, nanosec=tick)))
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.get_clock().now().nanoseconds == tick:
            break
    assert node.get_clock().now().nanoseconds == tick
    HealthBeat(node).beat()
    _spin_until(node, got, 1)
    assert got[0]['stamp_sec'] == tick / 1_000_000_000.0 == 0.15
    node.destroy_node()
