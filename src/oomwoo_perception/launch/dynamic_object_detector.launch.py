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
Run the placeholder dynamic-object detector on the scored scan.

Needs localization_health running (it publishes ~/scan_scored), e.g. via the
sim + localization scene. Then:

  ros2 launch oomwoo_perception dynamic_object_detector.launch.py use_sim_time:=true

Add a MarkerArray display on /dynamic_object_detector/objects in RViz to see a
sphere on each dynamic blob (a rolling ball, a foot) as it moves.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='oomwoo_perception',
            executable='dynamic_object_detector',
            name='dynamic_object_detector', output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(
                    use_sim_time, value_type=bool)}]),
    ])
