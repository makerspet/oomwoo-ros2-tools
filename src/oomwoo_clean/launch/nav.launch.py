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
Nav2 on an existing map, for the cleaning-with-map workflow.

Localization and navigation ONLY -- no Gazebo, no RViz -- so it composes with a
separately-launched robot source (`oomwoo_gazebo world.launch.py` in sim, or
`oomwoo_bringup physical.launch.py` on a real robot) and a separate viewer
(`oomwoo_clean_ui cleaning_debug.launch.py`). Uses the selected robot's own
navigation.yaml, so the same command follows `kaia use <robot>`.

localization:=
  amcl  (default) -- Nav2 AMCL scan-matches against the map. Real, but only as
        good as the map: if the loaded map does not match the robot's LiDAR,
        AMCL's estimate wanders. auto_localize:=true seeds it at the spawn pose
        (no manual RViz 2D Pose Estimate); set false on a real robot.
  truth -- DEBUG, SIM ONLY. No AMCL: publish a static map->odom at the spawn
        pose. The sim's odometry is noise-free, so map->base then tracks the
        true pose exactly, forever. Use it to take localization error out of the
        picture and debug navigation / the map / tuning on their own.

coverage:=true adds the ground-truth coverage meter (sim only). auto_localize,
coverage and the truth transform all use the spawn pose, which must match the
sim's (world.launch.py defaults) so everything lines up with the map.
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


def _seed_node(xv, yv, yawv):
    # (Re)publish /initialpose until AMCL localizes, then exit. The robot's AMCL
    # does not self-seed, so this is what puts map->odom on the tree in sim.
    return Node(
        package='oomwoo_sim_support', executable='initialpose_pub',
        name='initialpose_pub', output='screen',
        parameters=[{'use_sim_time': True, 'reseed_after_sec': 1.0,
                     'x': xv, 'y': yv, 'yaw': yawv}])


def _truth_localization(mapf, use_sim):
    # Perfect debug localization: a static IDENTITY map->odom. The sim's
    # odometry is noise-free AND world-referenced (odom->base already reports
    # the true world pose), and the map is world-aligned, so map == odom == world
    # -- map->base then equals the true pose for all time, no AMCL, no
    # scan-vs-map fitting.
    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen',
        parameters=[{'yaml_filename': mapf, 'use_sim_time': use_sim,
                     'topic_name': 'map', 'frame_id': 'map'}])
    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        parameters=[{'use_sim_time': use_sim, 'autostart': True,
                     'node_names': ['map_server']}])
    map_odom = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='map_odom_truth', output='screen',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom'])
    return [map_server, lifecycle, map_odom]


def _nav(context, robot_model, map_yaml, use_sim_time, localization,
         auto_localize, x0, y0, yaw0):
    model = context.perform_substitution(robot_model)
    if not model:
        try:
            from kaiaai import config  # dep-optional: guarded, falls back
            model = config.get_var('robot.model')
        except Exception:
            model = 'oomwoo_one'
    params = os.path.join(
        get_package_share_directory(model), 'config', 'navigation.yaml')
    launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')
    mapf = context.perform_substitution(map_yaml)
    sim = context.perform_substitution(use_sim_time)
    xv = float(context.perform_substitution(x0))
    yv = float(context.perform_substitution(y0))
    yawv = float(context.perform_substitution(yaw0))

    if context.perform_substitution(localization) == 'truth':
        nav = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'navigation_launch.py')),
            launch_arguments={'use_sim_time': sim,
                              'params_file': params}.items())
        return _truth_localization(mapf, sim.lower() == 'true') + [nav]

    # amcl (default): the full Nav2 bringup (map_server + AMCL + navigation)
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={'map': mapf, 'use_sim_time': sim,
                          'params_file': params, 'slam': 'False'}.items())
    actions = [bringup]
    if context.perform_substitution(auto_localize).lower() in ('true', '1'):
        actions.append(_seed_node(xv, yv, yawv))
    return actions


def generate_launch_description() -> LaunchDescription:
    # The meter scores against the true robot geometry (0.1745 m inscribed),
    # matching the coverage regressions.
    cleaning_radius = 0.20
    true_robot_radius = 0.1745
    coverage_target = 0.90

    # the WORLD-ALIGNED living_room map (origin -2.75, covers the spawn); the
    # oomwoo_gazebo copy has a different origin that excludes the -2,-0.5 spawn
    default_map = os.path.join(
        get_package_share_directory('oomwoo_sim_support'),
        'maps', 'living_room.yaml')
    x0 = LaunchConfiguration('x_pose')
    y0 = LaunchConfiguration('y_pose')
    yaw0 = LaunchConfiguration('yaw')

    args = [
        # '' follows `kaia use <robot>`; name a package to override
        DeclareLaunchArgument('robot_model', default_value=''),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # 'amcl' (real) or 'truth' (perfect static map->odom, sim debug)
        DeclareLaunchArgument('localization', default_value='amcl',
                              choices=['amcl', 'truth']),
        # auto-seed AMCL at the spawn pose (amcl mode, sim); false on a robot
        DeclareLaunchArgument('auto_localize', default_value='true'),
        # ground-truth coverage marking: sim only, off by default
        DeclareLaunchArgument('coverage', default_value='false'),
        # must match the sim spawn (world.launch.py defaults)
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='-0.5'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]

    nav = OpaqueFunction(function=_nav, args=[
        LaunchConfiguration('robot_model'),
        LaunchConfiguration('map'),
        LaunchConfiguration('use_sim_time'),
        LaunchConfiguration('localization'),
        LaunchConfiguration('auto_localize'),
        x0, y0, yaw0])

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

    return LaunchDescription(args + [nav, ground_truth, coverage_meter])
