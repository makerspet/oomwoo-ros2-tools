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
RViz debug view for OOMWOO cleaning.

A ready-made RViz layout — the coverage plan Path, robot model, LiDAR, static
map, and the coverage_meter covered grid are all pre-added, so you don't add
displays by hand. Bring up the sim + cleaning separately (e.g.
``coverage_regression.launch.py ... executor:=reactive``), then run this.
Grows as more cleaning debug tools are added.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    rviz_config = os.path.join(
        get_package_share_directory('oomwoo_clean'),
        'config', 'cleaning_debug.rviz')
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        # sim debug defaults to sim time; set false to watch a real robot
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
