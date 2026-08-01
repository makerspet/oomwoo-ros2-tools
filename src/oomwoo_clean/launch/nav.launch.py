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
Nav2 + AMCL on an existing map, for the cleaning-with-map workflow.

Localization and navigation ONLY -- no Gazebo, no RViz -- so it composes with a
separately-launched robot source (`oomwoo_gazebo world.launch.py` in sim, or
`oomwoo_bringup physical.launch.py` on a real robot) and a separate viewer
(`oomwoo_clean_ui cleaning_debug.launch.py`). Uses the selected robot's own
navigation.yaml, so the same command follows `kaia use <robot>`.

auto_localize:=true (default) seeds AMCL at the spawn pose so the sim localizes
itself -- no manual RViz 2D Pose Estimate. Set it false on a real robot (you do
not know the pose there) and seed with the 2D Pose Estimate instead.

coverage:=true adds the ground-truth coverage meter that marks the floor clean
as the robot drives. That meter is ground-truth based, so it is SIM ONLY; on a
real robot leave coverage off (a belief-based estimator does not exist yet). Both
auto_localize and coverage use the spawn pose, which must match the sim's
(world.launch.py defaults), so the robot localizes and the covered cells line up
with the map.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _nav(context, robot_model, map_yaml, use_sim_time):
    model = context.perform_substitution(robot_model)
    if not model:
        try:
            from kaiaai import config  # dep-optional: guarded, falls back
            model = config.get_var('robot.model')
        except Exception:
            model = 'oomwoo_one'
    params = os.path.join(
        get_package_share_directory(model), 'config', 'navigation.yaml')
    bringup = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch', 'bringup_launch.py')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup),
        launch_arguments={
            'map': context.perform_substitution(map_yaml),
            'use_sim_time': context.perform_substitution(use_sim_time),
            'params_file': params,
            'slam': 'False'}.items())]


def generate_launch_description() -> LaunchDescription:
    # The meter scores against the true robot geometry (0.1745 m inscribed),
    # matching the coverage regressions.
    cleaning_radius = 0.20
    true_robot_radius = 0.1745
    coverage_target = 0.90

    default_map = os.path.join(
        get_package_share_directory('oomwoo_gazebo'), 'map', 'living_room.yaml')
    x0 = LaunchConfiguration('x_pose')
    y0 = LaunchConfiguration('y_pose')
    yaw0 = LaunchConfiguration('yaw')

    args = [
        # '' follows `kaia use <robot>`; name a package to override
        DeclareLaunchArgument('robot_model', default_value=''),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # auto-seed AMCL at the spawn pose (sim); false on a real robot
        DeclareLaunchArgument('auto_localize', default_value='true'),
        # ground-truth coverage marking: sim only, off by default
        DeclareLaunchArgument('coverage', default_value='false'),
        # must match the sim spawn (world.launch.py defaults) so the robot
        # localizes and coverage aligns
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='-0.5'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]

    nav = OpaqueFunction(function=_nav, args=[
        LaunchConfiguration('robot_model'),
        LaunchConfiguration('map'),
        LaunchConfiguration('use_sim_time')])

    with_coverage = IfCondition(LaunchConfiguration('coverage'))
    ground_truth = Node(
        package='oomwoo_sim_support', executable='ground_truth',
        output='screen', condition=with_coverage,
        parameters=[{'spawn_x': ParameterValue(x0, value_type=float),
                     'spawn_y': ParameterValue(y0, value_type=float),
                     'spawn_yaw': ParameterValue(yaw0, value_type=float),
                     'use_sim_time': True}],
        remappings=[('odom', '/odom'), ('~/pose', '/ground_truth/pose')])
    coverage_meter = Node(
        package='oomwoo_sim_support', executable='coverage_meter',
        output='screen', condition=with_coverage,
        parameters=[{'cleaning_radius': cleaning_radius,
                     'robot_radius': true_robot_radius,
                     'coverage_target': coverage_target, 'use_sim_time': True}],
        remappings=[('map', '/map'),
                    ('ground_truth/pose', '/ground_truth/pose'),
                    ('cleaning_active', '/coverage_planner/cleaning_active')])

    # Seed AMCL at the spawn pose so the sim localizes without a manual RViz
    # 2D Pose Estimate. initialpose_pub (re)publishes /initialpose until AMCL
    # localizes, then exits; the robot's AMCL does not self-seed, so this is
    # what puts map->odom on the tree.
    seed = Node(
        package='oomwoo_sim_support', executable='initialpose_pub',
        name='initialpose_pub', output='screen',
        condition=IfCondition(LaunchConfiguration('auto_localize')),
        parameters=[{'use_sim_time': True, 'reseed_after_sec': 1.0,
                     'x': ParameterValue(x0, value_type=float),
                     'y': ParameterValue(y0, value_type=float),
                     'yaw': ParameterValue(yaw0, value_type=float)}])

    return LaunchDescription(
        args + [nav, ground_truth, coverage_meter, seed])
