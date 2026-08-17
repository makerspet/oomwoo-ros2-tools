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
r"""
Run AMCL and slam_toolbox localization side by side and score both live.

Only one node may own the map->odom TF, so slam_toolbox owns it (canonical) and
AMCL runs with tf_broadcast:false -- it still publishes /amcl_pose, just no TF.
Two localization_error meters then score both against ground truth:

  slam_toolbox : /loc_err_slam/pos_err_m  (read from the map->base TF)
  AMCL         : /loc_err_amcl/pos_err_m  (read from /amcl_pose)

AMCL uses your unmodified navigation.yaml (only tf_broadcast is overridden).
Both localizers share the same map origin only if living_room.yaml and
living_room_serial.* were saved from the SAME mapping session.

Run the sim with odom_source:=truth first (so /odom is real ground truth), then:

  ros2 launch oomwoo_sim_support localization_compare.launch.py \\
    use_sim_time:=true map:=/maps/living_room.yaml
  ros2 launch oomwoo_clean wall_clean_bump_out.launch.py use_sim_time:=true
  rqt_plot /loc_err_amcl/pos_err_m/data /loc_err_slam/pos_err_m/data

serial_map defaults to the map path with .yaml replaced by _serial (the
slam_toolbox pose-graph base, no extension).
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler,
    TimerAction)
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

from lifecycle_msgs.msg import Transition


def make_nodes(context: LaunchContext, use_sim_time, map_yaml, serial_map,
               nav_params, x_pose, y_pose, yaw, autostart, bringup_delay):
    sim = context.perform_substitution(use_sim_time).lower() == 'true'
    map_str = context.perform_substitution(map_yaml)
    serial_str = context.perform_substitution(serial_map)
    nav_str = context.perform_substitution(nav_params)
    x = float(context.perform_substitution(x_pose))
    y = float(context.perform_substitution(y_pose))
    th = float(context.perform_substitution(yaw))
    auto = context.perform_substitution(autostart).lower() == 'true'
    delay = float(context.perform_substitution(bringup_delay))

    if not serial_str:
        serial_str = os.path.splitext(map_str)[0] + '_serial'
    slam_params = os.path.join(
        get_package_share_directory('oomwoo_sim_support'),
        'config', 'mapper_params_localization.yaml')
    common = {'use_sim_time': sim}

    # --- AMCL (no TF; only /amcl_pose) + its map, managed by nav2 lifecycle ---
    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen', parameters=[common, {'yaml_filename': map_str}])
    amcl = Node(
        package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        parameters=[nav_str, common, {'tf_broadcast': False}])
    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        parameters=[common, {'autostart': auto,
                             'node_names': ['map_server', 'amcl']}])
    seed = Node(
        package='oomwoo_sim_support', executable='initialpose_pub',
        name='initialpose_pub', output='screen',
        parameters=[common, {'x': x, 'y': y, 'yaw': th}])

    # --- slam_toolbox localization (canonical map->odom); own /map -> /map_slam
    slam = LifecycleNode(
        package='slam_toolbox', executable='localization_slam_toolbox_node',
        name='slam_toolbox', namespace='', output='screen',
        parameters=[slam_params, common, {
            'map_file_name': serial_str,
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

    # --- ground truth + the two error meters ---
    truth = Node(
        package='oomwoo_sim_support', executable='ground_truth',
        name='ground_truth', output='screen',
        parameters=[common, {'spawn_x': x, 'spawn_y': y, 'spawn_yaw': th}])
    meter_amcl = Node(
        package='oomwoo_sim_support', executable='localization_error',
        name='loc_err_amcl', output='screen',
        parameters=[common, {'estimate_topic': '/amcl_pose',
                             'target_frame': 'map'}])
    meter_slam = Node(
        package='oomwoo_sim_support', executable='localization_error',
        name='loc_err_slam', output='screen',
        parameters=[common, {'target_frame': 'map'}])

    # Bring up nav2 (map_server + amcl) only AFTER slam_toolbox reaches its
    # active state -- i.e. once its one-shot pose-graph deserialize + Ceres load
    # is finished. That load is a ~50-thread burst; if it overlaps map_server's
    # configure it starves the service call and the whole AMCL side aborts.
    # Waiting on the state TRANSITION (not a fixed delay) self-paces to any PC,
    # fast or slow; bringup_delay only adds a small cushion past the first
    # localize burst. map_server/amcl sit unconfigured until then.
    if auto:
        start_nav2 = RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=slam, goal_state='active',
            entities=[TimerAction(period=delay, actions=[lifecycle])]))
        nodes = [map_server, amcl, seed, slam, slam_configure, slam_activate,
                 start_nav2]
    else:
        # manual lifecycle mode: bring nav2 up immediately; user drives slam
        nodes = [map_server, amcl, lifecycle, seed, slam]
    nodes += [truth, meter_amcl, meter_slam]
    return nodes


def generate_launch_description() -> LaunchDescription:
    default_nav = os.path.join(
        get_package_share_directory('oomwoo_one'), 'config', 'navigation.yaml')
    default_map = os.path.join(
        get_package_share_directory('oomwoo_gazebo'), 'map', 'living_room.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument(
            'serial_map', default_value='',
            description='slam_toolbox pose-graph base path (no extension); '
                        'defaults to map with .yaml -> _serial'),
        DeclareLaunchArgument('nav_params', default_value=default_nav),
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='-0.5'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'bringup_delay', default_value='2.0',
            description='Extra seconds after slam_toolbox reaches active '
                        'before starting the nav2 (AMCL) lifecycle bringup'),
        OpaqueFunction(function=make_nodes, args=[
            LaunchConfiguration('use_sim_time'),
            LaunchConfiguration('map'),
            LaunchConfiguration('serial_map'),
            LaunchConfiguration('nav_params'),
            LaunchConfiguration('x_pose'),
            LaunchConfiguration('y_pose'),
            LaunchConfiguration('yaw'),
            LaunchConfiguration('autostart'),
            LaunchConfiguration('bringup_delay'),
        ]),
    ])
