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
Cleaning behavior: drive Nav2 to RViz 2D Goal Poses, marking the floor clean.

The swappable behavior layer of the cleaning-with-map workflow. Run it alongside
a nav stack (`oomwoo_clean nav.launch.py`) and a robot source (sim or physical);
swap it for another behavior (docking, ...) without touching sim or nav. For now
it is the plain drive-to-goal bridge (no coverage planning yet).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='oomwoo_clean', executable='clean_to_goal',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('goal_pose', '/goal_pose'),
                        ('navigate_to_pose', '/navigate_to_pose'),
                        ('cleaning_active', '/coverage_planner/cleaning_active')]),
    ])
