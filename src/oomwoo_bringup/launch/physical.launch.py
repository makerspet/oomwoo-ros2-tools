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
import re
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import UnlessCondition
from kaiaai import config


def make_nodes(context: LaunchContext, robot_model, lidar_model, use_sim_time):
    robot_model_str = context.perform_substitution(robot_model)
    lidar_model_str = context.perform_substitution(lidar_model)
    use_sim_time_str = context.perform_substitution(use_sim_time)

    if len(robot_model_str) == 0:
      robot_model_str = config.get_var('robot.model')

    description_package_path = get_package_share_path(robot_model_str)
    telem_package_path = get_package_share_path('kaiaai_telemetry')

    urdf_path_name = os.path.join(
      description_package_path,
      'urdf',
      'robot.urdf.xacro')

    config_telem_path_name = os.path.join(
      telem_package_path,
      'config',
      'telem.yaml')

    robot_description = ParameterValue(Command(['xacro ', urdf_path_name]), value_type=str)

    config_override_path_name = os.path.join(
        description_package_path,
        'config',
        'telem.yaml'
    )

    lidar_model = lidar_model_str if len(lidar_model_str) > 0 else \
      robot_model_str + ' default'

    print('URDF file   : {}'.format(urdf_path_name))
    print('Telem params: {}'.format(config_telem_path_name))
    print('Model params: {}'.format(config_override_path_name))
    print('LiDAR model : {}'.format(lidar_model))

    LogInfo(msg='URDF file   : {}'.format(urdf_path_name))
    LogInfo(msg='Telem params: {}'.format(config_telem_path_name))
    LogInfo(msg='Model params: {}'.format(config_override_path_name))
    LogInfo(msg='LiDAR model : {}'.format(lidar_model))

    return [
        Node(
            package="kaiaai_telemetry",
            executable="telem",
            output="screen",
            parameters = [config_telem_path_name, config_override_path_name,
              {'laser_scan.lidar_model': lidar_model_str}]
              if len(lidar_model_str) > 0 else
              [config_telem_path_name, config_override_path_name]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time_str.lower() == 'true',
                'robot_description': robot_description
            }]
        ),
    ]


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            name='robot_model',
            default_value='',
            description='Robot description package name'
        ),
        DeclareLaunchArgument(
            name='lidar_model',
            default_value='',
            choices=['YDLIDAR-X4', 'XIAOMI-LDS02RR', 'YDLIDAR-X2-X2L',
              '3IROBOTIX-DELTA-2G', 'YDLIDAR-X3-PRO', 'YDLIDAR-X3',
              'NEATO-XV11', 'SLAMTEC-RPLIDAR-A1', '3IROBOTIX-DELTA-2A',
              '3IROBOTIX-DELTA-2B', 'LDROBOT-LD14P', 'CAMSENSE-X1',
              'YDLIDAR-SCL', ''],  # 'AUTO'
            description='LiDAR model'
        ),
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use simulation (Gazebo) clock if true'
        ),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output="screen",
            arguments=["udp4", "-p", "8888"]  # , "-v6"
        ),
        OpaqueFunction(function=make_nodes, args=[
            LaunchConfiguration('robot_model'),
            LaunchConfiguration('lidar_model'),
            LaunchConfiguration('use_sim_time'),
        ]),
    ])
