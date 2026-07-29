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
Drive-to-goal cleaning demo on a loaded map (stock Nav2, no coverage planner).

Brings up the living_room sim + map + Nav2 + the coverage_meter + the
clean_to_goal bridge. Drop an RViz 2D Goal Pose and the robot drives there via
Nav2, marking the floor clean wherever it goes. Watch it by running the RViz
debug view separately (kept in its own package so rviz2 stays off the runtime):

  ros2 launch oomwoo_clean_ui cleaning_debug.launch.py

World/map/spawn default to the stock living_room; gui:=true also attaches the
Gazebo GUI.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_sim = get_package_share_directory('oomwoo_sim_support')
    pkg_gazebo = get_package_share_directory('oomwoo_gazebo')

    # The meter always scores against the true robot geometry (0.1745 m
    # inscribed), matching the coverage regressions.
    cleaning_radius = 0.20
    true_robot_radius = 0.1745
    coverage_target = 0.90

    default_world = os.path.join(pkg_gazebo, 'worlds', 'living_room.world')
    default_map = os.path.join(pkg_sim, 'maps', 'living_room.yaml')
    args = [
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('map', default_value=default_map),
        # clearest floor cell of the cluttered living_room (matches the
        # coverage_livingroom regression spawn)
        DeclareLaunchArgument('x_pose', default_value='0.32'),
        DeclareLaunchArgument('y_pose', default_value='1.59'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        # kaiaai convention: 'config' reads robot.model from ~/.kaiaai.yaml
        DeclareLaunchArgument('robot_model', default_value='config'),
        # gui:=true also attaches the Gazebo GUI (needs a display)
        DeclareLaunchArgument('gui', default_value='false'),
    ]

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, 'launch', 'sim_bringup.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'map': LaunchConfiguration('map'),
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
            'yaw': LaunchConfiguration('yaw'),
            'robot_model': LaunchConfiguration('robot_model'),
            'gui': LaunchConfiguration('gui'),
        }.items())

    coverage_meter = Node(
        package='oomwoo_sim_support', executable='coverage_meter',
        output='screen',
        parameters=[{'cleaning_radius': cleaning_radius,
                     'robot_radius': true_robot_radius,
                     'coverage_target': coverage_target, 'use_sim_time': True}],
        remappings=[('map', '/map'),
                    ('ground_truth/pose', '/ground_truth/pose'),
                    ('cleaning_active', '/coverage_planner/cleaning_active')])

    clean_to_goal = Node(
        package='oomwoo_clean', executable='clean_to_goal', output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[('goal_pose', '/goal_pose'),
                    ('navigate_to_pose', '/navigate_to_pose'),
                    ('cleaning_active', '/coverage_planner/cleaning_active')])

    return LaunchDescription(args + [base, coverage_meter, clean_to_goal])
