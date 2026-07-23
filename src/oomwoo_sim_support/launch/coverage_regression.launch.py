# Copyright 2026 Jayadev Rana
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
Coverage-cleaning bringup: base sim + Nav2 + coverage planner + coverage meter.

Headless. The coverage_planner executes a boustrophedon sweep via Nav2; the
coverage_meter scores true coverage % and path efficiency against ground truth
and logs COVERAGE_REPORT lines the regression test asserts on.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg_sim = get_package_share_directory('oomwoo_sim_support')

    cleaning_radius = 0.20
    coverage_target = 0.90

    # world/map/spawn default to the primitives test_room but are overridable so
    # the same regression drives the stock living_room (with collision proxies).
    default_world = os.path.join(pkg_sim, 'worlds', 'test_room.world')
    default_map = os.path.join(pkg_sim, 'maps', 'test_room.yaml')
    reg_args = [
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        # planning clearance ALIGNED WITH THE REAL MACHINE: true inscribed
        # radius 0.1745 rounded up to 0.18 (~5 mm margin). A vacuum is meant
        # to touch things (that's the
        # bumper's job) — inflated clearance (the old 0.30/0.24) sealed off
        # every gap under 2x the value, which is why under-furniture never
        # happened. Contact/wedging risk is carried by the wedge-escape path.
        DeclareLaunchArgument('robot_radius', default_value='0.18'),
        # pinned so the regression gate is reproducible on any machine;
        # override robot_model:=<pkg> to run the suite against another vacuum.
        DeclareLaunchArgument('robot_model', default_value='oomwoo_one'),
        # gui:=true watches this exact regression in the Gazebo GUI
        DeclareLaunchArgument('gui', default_value='false'),
        # contact_aware:=false restores the legacy blind straight reverse escape
        # (for A/B against the bumper-directed peel-off escape)
        DeclareLaunchArgument('contact_aware', default_value='true'),
        # metres between intra-row waypoints; smaller = tighter tracking, fewer
        # cut corners (row_substep:=1.0 restores the old coarse spacing for A/B)
        DeclareLaunchArgument('row_substep', default_value='0.4'),
    ]
    robot_radius = ParameterValue(
        LaunchConfiguration('robot_radius'), value_type=float)
    # The METER always scores against the true robot geometry, never the
    # planner's clearance. robot_radius above is planning conservatism (Nav2
    # margin); if the meter shared it, a more timid planner would shrink the
    # denominator and score HIGHER while cleaning LESS. Floor the planner
    # won't enter must count against the score.
    true_robot_radius = 0.1745

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
        package='oomwoo_sim_support', executable='coverage_meter', output='screen',
        parameters=[{'cleaning_radius': cleaning_radius,
                     'robot_radius': true_robot_radius,
                     'coverage_target': coverage_target, 'use_sim_time': True}],
        remappings=[('map', '/map'),
                    ('ground_truth/pose', '/ground_truth/pose'),
                    ('cleaning_active', '/coverage_planner/cleaning_active')])

    coverage_planner = Node(
        package='oomwoo_coverage', executable='coverage_planner', output='screen',
        parameters=[{'cleaning_radius': cleaning_radius, 'robot_radius': robot_radius,
                     'coverage_target': coverage_target, 'row_overlap': 0.05, 'max_retries': 1,
                     'contact_aware_escape': ParameterValue(
                         LaunchConfiguration('contact_aware'), value_type=bool),
                     'row_substep_m': ParameterValue(
                         LaunchConfiguration('row_substep'), value_type=float),
                     'use_sim_time': True}],
        remappings=[('map', '/map'),
                    ('coverage_ratio', '/coverage_meter/ratio'),
                    ('covered_grid', '/coverage_meter/covered_grid'),
                    ('navigate_to_pose', '/navigate_to_pose')])

    # No fixed timers: the meter self-gates on /map + /ground_truth, and the
    # planner self-gates on /amcl_pose + the Nav2 action server (retrying until
    # ready). cleaning_active is latched, so meter accounting can't miss the
    # planner's start regardless of launch order.
    return LaunchDescription(reg_args + [
        base,
        coverage_meter,
        coverage_planner,
    ])
