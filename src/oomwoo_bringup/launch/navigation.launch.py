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
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def make_nodes(context: LaunchContext, robot_model, map_arg, use_sim_time,
               slam, x_pose, y_pose, yaw, auto_localize, rviz_config):
    robot_model_str = context.perform_substitution(robot_model)
    map_path_str = context.perform_substitution(map_arg)
    use_sim_time_str = context.perform_substitution(use_sim_time)
    slam_str = context.perform_substitution(slam)
    auto_str = context.perform_substitution(auto_localize)
    rviz_config_str = context.perform_substitution(rviz_config)

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
        description_package_path,
        'config',
        'navigation.yaml')

    print('Rviz2 config : {}'.format(rviz_config_path))
    print('Nav2  config : {}'.format(nav_config_path))
    print('Map          : {}'.format(map_path_str))

    nodes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_path('nav2_bringup'), 'launch'),
                '/bringup_launch.py'
            ]),
            launch_arguments={
                'map': map_path_str,
                'use_sim_time': use_sim_time_str,
                'slam': slam_str,
                'params_file': nav_config_path}.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': use_sim_time_str.lower() == 'true'}],
        )
    ]

    # Seed AMCL at the known start pose so map->odom appears WITHOUT the manual
    # RViz "2D Pose Estimate". Only in localization mode (slam=False), and by
    # default only in sim (auto_localize=sim); real-robot bringup stays manual.
    # AMCL converges from a rough pose, so x_pose/y_pose/yaw need only be close.
    localizing = slam_str.strip().lower() in ('false', '0')
    auto = auto_str == 'true' or (
        auto_str == 'sim' and use_sim_time_str.lower() == 'true')
    if localizing and auto:
        nodes.append(Node(
            package='oomwoo_sim_support', executable='initialpose_pub',
            name='initialpose_pub', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time_str.lower() == 'true',
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
            description='Known start x for auto_localize (m)'),
        DeclareLaunchArgument(
            'y_pose', default_value='-0.5',
            description='Known start y for auto_localize (m)'),
        DeclareLaunchArgument(
            'yaw', default_value='0.0',
            description='Known start yaw for auto_localize (rad)'),
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
        ]),
    ])
