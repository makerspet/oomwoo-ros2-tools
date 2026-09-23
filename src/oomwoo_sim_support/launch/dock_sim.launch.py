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
Dock in simulation, with the dock's IR beacon simulated.

Starts ir_beacon_sim and oomwoo_dock's docking nodes, and connects the two: the
beacon bearing seeds the dock detector, and once the beacon is in view the search
spin stops, so the LiDAR fits the dock from where the robot already is. On
the real robot the receiver driver publishes the same two topics instead.

The dock pose defaults to kitchen_dining's vacuum_dock.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start the beacon simulator and the docking nodes, wired together."""
    use_sim_time = LaunchConfiguration('use_sim_time')
    dock_launch = os.path.join(
        get_package_share_directory('oomwoo_dock'), 'launch', 'dock.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('dock_x', default_value='-2.45'),
        DeclareLaunchArgument('dock_y', default_value='-0.5'),
        DeclareLaunchArgument('dock_yaw_deg', default_value='90.0'),
        Node(
            package='oomwoo_sim_support',
            executable='ir_beacon_sim',
            name='ir_beacon_sim',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'dock_x': LaunchConfiguration('dock_x'),
                'dock_y': LaunchConfiguration('dock_y'),
                'dock_yaw_deg': LaunchConfiguration('dock_yaw_deg'),
            }],
            remappings=[
                ('~/bearing', '/dock_detector/beacon_bearing'),
                ('~/visible', '/dock_drive/beacon_visible'),
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dock_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
