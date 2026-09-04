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
Open the contour-follower debug view in RViz, on its own.

Kept separate from contour_follow.launch.py so the UI can come up first: arrange
the windows and set the camera, then start the follower (and the screen capture)
without RViz restarting underneath you.

Shows the robot, the live scan, and the follower's own debug markers -- the
boundary point it picked, the ray to it, the standoff target it is steering
toward, the search sector, and the current state as text.

  ros2 launch oomwoo_clean contour_follow_rviz.launch.py use_sim_time:=true
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration('use_sim_time')
    default_cfg = os.path.join(
        get_package_share_directory('oomwoo_clean'),
        'rviz', 'contour_follow.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_cfg),
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}]),
    ])
