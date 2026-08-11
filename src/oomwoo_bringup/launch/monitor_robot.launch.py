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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def make_rviz2_node(context: LaunchContext, robot_model, use_sim_time,
                    rviz_config):
    robot_model_str = context.perform_substitution(robot_model)
    use_sim_time_str = context.perform_substitution(use_sim_time)
    rviz_config_str = context.perform_substitution(rviz_config)

    if len(robot_model_str) == 0:
        robot_model_str = config.get_var('robot.model')

    # rviz_config is either an absolute path or a file name looked up in the
    # robot description package's rviz/ folder (e.g. sensors.rviz).
    if os.path.isabs(rviz_config_str):
        rviz_config_path = rviz_config_str
    else:
        rviz_config_path = os.path.join(
            get_package_share_path(robot_model_str),
            'rviz',
            rviz_config_str)
    print('Rviz2 config : {}'.format(rviz_config_path))

    return [
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': use_sim_time_str.lower() == 'true'}],
            output='screen'
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='robot_model',
            default_value='',
            description='Robot description package name'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='monitor_robot.rviz',
            description='RViz config: a file name in the robot package rviz/ '
                        'folder (e.g. sensors.rviz) or an absolute path'
        ),
        OpaqueFunction(function=make_rviz2_node, args=[
            LaunchConfiguration('robot_model'),
            LaunchConfiguration('use_sim_time'),
            LaunchConfiguration('rviz_config'),
        ])
    ])
