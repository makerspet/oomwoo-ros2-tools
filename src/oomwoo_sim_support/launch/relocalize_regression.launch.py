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
Kidnapped-robot relocalization bringup (headless, no Nav2 nav servers).

Brings up the sim + AMCL + ground truth (via sim_bringup with_nav:=false), plus
the kidnap_recovery node (detect lost -> AMCL global re-init -> spin to recover)
and the kidnap_injector (teleport the robot + signal). Much lighter than the
coverage stack since relocalization only spins in place under AMCL.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_sim = get_package_share_directory('oomwoo_sim_support')

    # pinned by default so the regression gate is reproducible on any machine;
    # override robot_model:=<pkg> to run the suite against another vacuum.
    robot_model = LaunchConfiguration('robot_model')

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, 'launch', 'sim_bringup.launch.py')),
        launch_arguments={'with_nav': 'false',
                          'robot_model': robot_model,
                          # gui:=true watches this exact regression in the GUI
                          'gui': LaunchConfiguration('gui')}.items())

    kidnap_recovery = Node(
        package='oomwoo_nav_localize', executable='kidnap_recovery',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'lost_cov_trace': 0.6, 'ok_cov_trace': 0.25,
                     'recovery_timeout_sec': 30.0, 'spin_speed': 1.2,
                     'drive_speed': 0.20}],
        remappings=[('amcl_pose', '/amcl_pose'),
                    ('kidnap_trigger', '/kidnap_trigger'),
                    ('cmd_vel', '/cmd_vel'),
                    ('reinitialize_global_localization',
                     '/reinitialize_global_localization')])

    kidnap_injector = Node(
        package='oomwoo_sim_support', executable='kidnap_injector',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_model_name': robot_model,
                     'world_name': 'default', 'min_jump': 1.5, 'seed': 42}],
        remappings=[('map', '/map'),
                    ('ground_truth/pose', '/ground_truth/pose')])

    return LaunchDescription([
        DeclareLaunchArgument('robot_model', default_value='oomwoo_one'),
        DeclareLaunchArgument('gui', default_value='false'),
        base,
        # A gentle 20 s stagger to spread startup CPU. It does NOT enforce
        # ordering vs AMCL (AMCL now starts on the sim_bringup spawn event, not
        # a fixed timer): the two nodes self-gate — kidnap_injector waits on a
        # latched /map + a "no safe cells yet" guard, kidnap_recovery starts in
        # TRACKING and only acts on /amcl_pose or a /kidnap_trigger — and the
        # reloc runner separately waits up to 120 s for /amcl_pose + the kidnap
        # service before running any trial.
        TimerAction(period=20.0, actions=[kidnap_recovery, kidnap_injector]),
    ])
