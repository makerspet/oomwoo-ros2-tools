#!/usr/bin/env python3
#
# Copyright 2023-2024 KAIA.AI
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

import os

from ament_index_python.packages import get_package_share_path

from kaiaai import config

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction,
    RegisterEventHandler)
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import LifecycleNode, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue

from lifecycle_msgs.msg import Transition


def slam_toolbox_stack(desc_pkg_path, nav_params, map_path, serial_base,
                       sim, x, y, th):
    """
    Nav2 navigation + slam_toolbox localization (no AMCL), composed.

    map_server + its lifecycle_manager and the navigation nodes load into ONE
    component container (so lifecycle calls are intra-process and robust even
    with FastDDS shared memory off), while slam_toolbox owns map->odom from the
    serialized pose-graph. Navigation is started only once slam_toolbox is
    active, so map->odom exists before the costmaps come up.
    """
    slam_params = os.path.join(
        desc_pkg_path, 'config', 'mapper_params_localization.yaml')

    container = Node(
        package='rclcpp_components', executable='component_container_isolated',
        name='nav2_container', output='screen',
        parameters=[{'use_sim_time': sim}])

    # map_server (serves /map) + its lifecycle manager, composed.
    map_and_lm = LoadComposableNodes(
        target_container='nav2_container',
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_map_server', plugin='nav2_map_server::MapServer',
                name='map_server',
                parameters=[{'use_sim_time': sim, 'yaml_filename': map_path}]),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_localization',
                parameters=[{'use_sim_time': sim, 'autostart': True,
                             'node_names': ['map_server']}]),
        ])

    # slam_toolbox owns map->odom; its own /map is remapped so it does not
    # collide with map_server's /map.
    slam = LifecycleNode(
        package='slam_toolbox', executable='localization_slam_toolbox_node',
        name='slam_toolbox', namespace='', output='screen',
        parameters=[slam_params, {
            'use_sim_time': sim,
            'map_file_name': serial_base,
            'map_start_pose': [x, y, th]}],
        remappings=[('/map', '/map_slam'),
                    ('/map_metadata', '/map_slam_metadata')])
    slam_configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam),
        transition_id=Transition.TRANSITION_CONFIGURE))
    slam_activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam, start_state='configuring',
        goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_ACTIVATE))]))

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_path('nav2_bringup'), 'launch'),
            '/navigation_launch.py']),
        launch_arguments={
            'use_sim_time': 'true' if sim else 'false',
            'params_file': nav_params,
            'autostart': 'True',
            'use_composition': 'True',
            'container_name': 'nav2_container'}.items())
    # Bring navigation up only after slam_toolbox is active, so map->odom is
    # published before the costmaps configure.
    start_nav = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam, goal_state='active', entities=[navigation]))

    return [container, map_and_lm, slam, slam_configure, slam_activate,
            start_nav]


def make_nodes(context: LaunchContext, robot_model, map_arg, use_sim_time,
               slam, x_pose, y_pose, yaw, auto_localize, rviz_config,
               localization):
    robot_model_str = context.perform_substitution(robot_model)
    map_path_str = context.perform_substitution(map_arg)
    use_sim_time_str = context.perform_substitution(use_sim_time)
    slam_str = context.perform_substitution(slam)
    auto_str = context.perform_substitution(auto_localize)
    rviz_config_str = context.perform_substitution(rviz_config)
    localization_str = context.perform_substitution(localization).strip().lower()
    sim = use_sim_time_str.lower() == 'true'

    if len(robot_model_str) == 0:
        robot_model_str = config.get_var('robot.model')

    description_package_path = get_package_share_path(robot_model_str)

    # rviz_config is either an absolute path or a file name in the robot package
    # rviz/ folder (e.g. bump_map.rviz to also show the tactile bump map, so one
    # RViz window covers navigation + bump mapping).
    if os.path.isabs(rviz_config_str):
        rviz_config_path = rviz_config_str
    else:
        rviz_config_path = os.path.join(
            description_package_path, 'rviz', rviz_config_str)

    nav_config_path = os.path.join(
        description_package_path, 'config', 'navigation.yaml')

    # slam_toolbox localization needs a serialized pose-graph next to the map
    # (<map>_serial.posegraph). Fall back to AMCL if it is missing, so a plain
    # pgm map still localizes.
    localizing = slam_str.strip().lower() in ('false', '0')
    serial_base = os.path.splitext(map_path_str)[0] + '_serial'
    graph_ok = os.path.exists(serial_base + '.posegraph')
    use_slam = localizing and localization_str == 'slam_toolbox' and graph_ok
    if localizing and localization_str == 'slam_toolbox' and not graph_ok:
        print('navigation: no pose-graph at {}.posegraph -- '
              'falling back to AMCL'.format(serial_base))

    print('Rviz2 config : {}'.format(rviz_config_path))
    print('Nav2  config : {}'.format(nav_config_path))
    print('Map          : {}'.format(map_path_str))
    print('Localizer    : {}'.format('slam_toolbox' if use_slam else 'amcl'))

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': sim}])

    if use_slam:
        x = float(context.perform_substitution(x_pose))
        y = float(context.perform_substitution(y_pose))
        th = float(context.perform_substitution(yaw))
        return slam_toolbox_stack(
            description_package_path, nav_config_path, map_path_str,
            serial_base, sim, x, y, th) + [rviz_node]

    # AMCL, or mapping (slam:=True): nav2's own all-in-one bringup.
    nodes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_path('nav2_bringup'), 'launch'),
                '/bringup_launch.py']),
            launch_arguments={
                'map': map_path_str,
                'use_sim_time': use_sim_time_str,
                'slam': slam_str,
                'params_file': nav_config_path}.items()),
        rviz_node,
    ]

    # Seed AMCL at the known start pose so map->odom appears WITHOUT the manual
    # RViz "2D Pose Estimate". Only in localization mode (slam=False), and by
    # default only in sim (auto_localize=sim); real-robot bringup stays manual.
    auto = auto_str == 'true' or (auto_str == 'sim' and sim)
    if localizing and auto:
        nodes.append(Node(
            package='oomwoo_sim_support', executable='initialpose_pub',
            name='initialpose_pub', output='screen',
            parameters=[{
                'use_sim_time': sim,
                'x': ParameterValue(x_pose, value_type=float),
                'y': ParameterValue(y_pose, value_type=float),
                'yaw': ParameterValue(yaw, value_type=float)}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='robot_model',
            default_value='',
            description='Robot description package name'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                get_package_share_path('oomwoo_gazebo'),
                'map',
                'living_room.yaml'),
            description='Full path to an existing map file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'slam',
            default_value='False',
            choices=['True', 'False'],
            description='Navigate while creating a new map'
        ),
        DeclareLaunchArgument(
            'localization', default_value='slam_toolbox',
            choices=['amcl', 'slam_toolbox'],
            description='Localizer when navigating a saved map (slam:=False): '
                        'slam_toolbox scan-matching (needs a <map>_serial '
                        'pose-graph; falls back to amcl if missing), or amcl.'
        ),
        # Auto-localization: seed AMCL at the known start pose to skip the manual
        # RViz "2D Pose Estimate". Defaults match oomwoo_gazebo world.launch.py.
        DeclareLaunchArgument(
            'auto_localize', default_value='sim',
            choices=['true', 'false', 'sim'],
            description='Seed AMCL at (x_pose,y_pose,yaw) so map->odom appears '
                        'without the manual 2D Pose Estimate. sim = only when '
                        'use_sim_time:=true; real-robot bringup stays manual.'
        ),
        DeclareLaunchArgument(
            'x_pose', default_value='-2.0',
            description='Known start x for auto_localize / slam_toolbox (m)'),
        DeclareLaunchArgument(
            'y_pose', default_value='-0.5',
            description='Known start y for auto_localize / slam_toolbox (m)'),
        DeclareLaunchArgument(
            'yaw', default_value='0.0',
            description='Known start yaw for auto_localize / slam_toolbox (rad)'),
        DeclareLaunchArgument(
            'rviz_config', default_value='navigation.rviz',
            description='RViz config: a file name in the robot package rviz/ '
                        'folder (e.g. bump_map.rviz) or an absolute path'),
        OpaqueFunction(function=make_nodes, args=[
            LaunchConfiguration('robot_model'),
            LaunchConfiguration('map'),
            LaunchConfiguration('use_sim_time'),
            LaunchConfiguration('slam'),
            LaunchConfiguration('x_pose'),
            LaunchConfiguration('y_pose'),
            LaunchConfiguration('yaw'),
            LaunchConfiguration('auto_localize'),
            LaunchConfiguration('rviz_config'),
            LaunchConfiguration('localization'),
        ]),
    ])
