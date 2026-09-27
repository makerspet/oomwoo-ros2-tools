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
Sweep every obstacle edge on the map once: cleaning manager + contour follower.

The manager picks the nearest edge not yet swept, drives there with Nav2, runs
the follower along it, and moves on as soon as the follower is back on ground
it already swept -- so a lone table leg gets one lap, not an endless circle.
When no dirty edge is left it prints a report and exits.

Needs a map, localization and Nav2 already up. In the simulator, three
terminals (the map is rasterized from the same layout as the world):

  ros2 launch oomwoo_gazebo world.launch.py world:=contour_torture.world \\
    x_pose:=-2.6 y_pose:=-2.77 odom_source:=robot_wheels
  ros2 launch oomwoo_clean nav.launch.py x_pose:=-2.6 y_pose:=-2.77 \\
    map:=$(ros2 pkg prefix oomwoo_sim_support)/share/oomwoo_sim_support/maps/contour_torture.yaml
  ros2 launch oomwoo_clean edge_clean.launch.py

Watch it in RViz with oomwoo_one's edge_clean.rviz -- wall_follow.rviz plus the
map, the floor already passed over (blue), the obstacle edges (magenta dirty,
green swept, cyan written off) and the next target (arrow):

  ros2 launch oomwoo_bringup monitor_robot.launch.py use_sim_time:=true \\
    rviz_config:=edge_clean.rviz

Only the arguments declared below reach the nodes: `ros2 launch` silently drops
any other name:=value.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    standoff = ParameterValue(LaunchConfiguration('standoff_m'), value_type=float)
    side = LaunchConfiguration('follow_side')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('follow_side', default_value='right'),
        DeclareLaunchArgument('standoff_m', default_value='0.23'),
        DeclareLaunchArgument('halt_on_bump', default_value='false',
                              description='follower stops dead on a bump; the manager '
                                          'then moves on to the next edge'),
        Node(
            package='oomwoo_clean', executable='contour_follower',
            name='contour_follower', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'auto_start': False,          # the manager enables it at each edge
                'follow_side': side,
                'standoff_m': standoff,
                'halt_on_bump': ParameterValue(
                    LaunchConfiguration('halt_on_bump'), value_type=bool),
            }]),
        Node(
            package='oomwoo_clean', executable='clean_manager',
            name='clean_manager', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'follow_side': side,
                'standoff_m': standoff,
            }]),
    ])
