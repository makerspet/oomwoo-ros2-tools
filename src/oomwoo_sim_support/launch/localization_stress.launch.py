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
r"""
Stress localization with a large off-map obstacle, and A/B raw vs filtered scan.

scan_obstacle_injector paints a big obstacle into /scan -> /scan_stress;
slam_toolbox localizes on it and loc_err_slam scores the pose against ground
truth. slam_toolbox also publishes /map here (no nav2/AMCL needed), which is
what localization_health scores against in the filtered arm.

  filter:=false (default)  slam matches /scan_stress          -- does it degrade?
  filter:=true             slam matches /scan_filtered, from   -- does stripping
                           localization_health on /scan_stress    the obstacle help?

Run (robot_wheels so /odom_truth is ground truth):

  ros2 launch oomwoo_gazebo world.launch.py odom_source:=robot_wheels
  ros2 launch oomwoo_sim_support localization_stress.launch.py use_sim_time:=true \\
    map:=/ros_ws/src/oomwoo_gazebo/maps/living_room.yaml
  # then again with filter:=true, and compare loc_err_slam between the two:
  ros2 run foxglove_bridge foxglove_bridge   # plot /loc_err_slam/yaw_err_deg, pos_err_m

Widen/sweep the obstacle to find where it bites:
  obstacle_width_deg:=90 obstacle_sweep:=true
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler)
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

from lifecycle_msgs.msg import Transition


def make_nodes(context: LaunchContext, use_sim_time, map_yaml, serial_map,
               x_pose, y_pose, yaw, do_filter, width, rng, center, sweep,
               rviz, rviz_config):
    sim = context.perform_substitution(use_sim_time).lower() == 'true'
    map_str = context.perform_substitution(map_yaml)
    serial_str = context.perform_substitution(serial_map)
    x = float(context.perform_substitution(x_pose))
    y = float(context.perform_substitution(y_pose))
    th = float(context.perform_substitution(yaw))
    filt = context.perform_substitution(do_filter).lower() == 'true'
    rviz_on = context.perform_substitution(rviz).lower() == 'true'
    rviz_cfg = context.perform_substitution(rviz_config)
    common = {'use_sim_time': sim}

    if not serial_str:
        serial_str = os.path.splitext(map_str)[0] + '_serial'
    slam_params = os.path.join(
        get_package_share_directory('oomwoo_sim_support'),
        'config', 'mapper_params_localization.yaml')
    scan_src = '/scan_filtered' if filt else '/scan_stress'

    injector = Node(
        package='oomwoo_sim_support', executable='scan_obstacle_injector',
        name='scan_obstacle_injector', output='screen',
        parameters=[common, {
            'input_topic': '/scan', 'output_topic': '/scan_stress',
            'width_deg': float(context.perform_substitution(width)),
            'range_m': float(context.perform_substitution(rng)),
            'center_deg': float(context.perform_substitution(center)),
            'sweep': context.perform_substitution(sweep).lower() == 'true'}])

    # slam_toolbox localization: owns map->odom AND publishes /map (which
    # localization_health scores against in the filtered arm).
    slam = LifecycleNode(
        package='slam_toolbox', executable='localization_slam_toolbox_node',
        name='slam_toolbox', namespace='', output='screen',
        parameters=[slam_params, common, {
            'map_file_name': serial_str,
            'map_start_pose': [x, y, th],
            'scan_topic': scan_src}],
        additional_env={'GLOG_minloglevel': '2'})
    slam_configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam),
        transition_id=Transition.TRANSITION_CONFIGURE))
    slam_activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam, start_state='configuring',
        goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_ACTIVATE))]))

    truth = Node(
        package='oomwoo_sim_support', executable='ground_truth',
        name='ground_truth', output='screen',
        parameters=[common, {'spawn_x': x, 'spawn_y': y, 'spawn_yaw': th}])
    meter = Node(
        package='oomwoo_sim_support', executable='localization_error',
        name='loc_err_slam', output='screen',
        parameters=[common, {'target_frame': 'map'}])

    nodes = [injector, slam, slam_configure, slam_activate, truth, meter]
    if filt:
        # score/strip the STRESSED scan: remap /scan -> /scan_stress so
        # /scan_filtered is the obstacle removed (when quality stays trusted).
        nodes.append(Node(
            package='oomwoo_localization', executable='localization_health',
            name='localization_health', output='screen',
            parameters=[common], remappings=[('/scan', '/scan_stress')]))
    if rviz_on:
        if not os.path.isabs(rviz_cfg):
            rviz_cfg = os.path.join(
                get_package_share_directory('oomwoo_one'), 'rviz', rviz_cfg)
        nodes.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', rviz_cfg], parameters=[common]))
    return nodes


def generate_launch_description() -> LaunchDescription:
    default_map = os.path.join(
        get_package_share_directory('oomwoo_gazebo'), 'map', 'living_room.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('serial_map', default_value=''),
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='-0.5'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'filter', default_value='false',
            description='slam matches /scan_filtered (via localization_health) '
                        'instead of the raw /scan_stress'),
        DeclareLaunchArgument('obstacle_width_deg', default_value='60.0'),
        DeclareLaunchArgument('obstacle_range_m', default_value='0.6'),
        DeclareLaunchArgument('obstacle_center_deg', default_value='0.0'),
        DeclareLaunchArgument('obstacle_sweep', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('rviz_config', default_value='bump_map.rviz'),
        OpaqueFunction(function=make_nodes, args=[
            LaunchConfiguration('use_sim_time'),
            LaunchConfiguration('map'),
            LaunchConfiguration('serial_map'),
            LaunchConfiguration('x_pose'),
            LaunchConfiguration('y_pose'),
            LaunchConfiguration('yaw'),
            LaunchConfiguration('filter'),
            LaunchConfiguration('obstacle_width_deg'),
            LaunchConfiguration('obstacle_range_m'),
            LaunchConfiguration('obstacle_center_deg'),
            LaunchConfiguration('obstacle_sweep'),
            LaunchConfiguration('rviz'),
            LaunchConfiguration('rviz_config'),
        ]),
    ])
