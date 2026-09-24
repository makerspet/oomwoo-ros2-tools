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
Find the dock and reverse into it.

Park the robot somewhere in front of the dock, roughly facing it, then launch
this. The prior arguments say where the dock is expected in the scan frame; they
only need to be close, and the fit does the rest. RViz is deliberately not
started here, so it can come up first and stay up across restarts.

The launch ends by itself when dock_drive does: once the robot is docked, or
has given up. Pass exit_when_done:=false to keep the nodes running instead.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start the dock detector and the docking driver."""
    use_sim_time = LaunchConfiguration('use_sim_time')
    prior = {
        'prior_x': LaunchConfiguration('prior_x'),
        'prior_y': LaunchConfiguration('prior_y'),
        'prior_yaw_deg': LaunchConfiguration('prior_yaw_deg'),
    }
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('prior_x', default_value='0.6'),
        DeclareLaunchArgument('prior_y', default_value='0.0'),
        DeclareLaunchArgument('prior_yaw_deg', default_value='0.0'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('exit_when_done', default_value='true'),
        Node(
            package='oomwoo_dock',
            executable='dock_detector',
            name='dock_detector',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, prior],
        ),
        Node(
            package='oomwoo_dock',
            executable='dock_drive',
            name='dock_drive',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'auto_start': LaunchConfiguration('auto_start'),
                         'exit_when_done': LaunchConfiguration('exit_when_done')}],
            remappings=[('~/dock_pose', '/dock_detector/dock_pose')],
            # docked or given up: take the detector (and anything that
            # included this launch file) down with it
            on_exit=Shutdown(reason='docking finished'),
        ),
    ])
