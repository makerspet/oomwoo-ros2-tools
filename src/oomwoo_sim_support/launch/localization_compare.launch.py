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

AMCL + its map_server come up through nav2's own localization_launch (the same,
proven bringup navigation.launch.py uses), fed your unmodified navigation.yaml
with only tf_broadcast rewritten to false. Both localizers share the same map
origin only if living_room.yaml and living_room_serial.* were saved from the
SAME mapping session.

Run the sim with odom_source:=ground_truth first (so /odom is truth), then:

  ros2 launch oomwoo_sim_support localization_compare.launch.py \\
    use_sim_time:=true map:=/maps/living_room.yaml
  ros2 launch oomwoo_clean wall_clean_bump_out.launch.py use_sim_time:=true
  # plot both /loc_err_amcl/pos_err_m and /loc_err_slam/pos_err_m

serial_map defaults to the map path with .yaml replaced by _serial (the
slam_toolbox pose-graph base, no extension).
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction,
    RegisterEventHandler, TimerAction)
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

from lifecycle_msgs.msg import Transition

from nav2_common.launch import RewrittenYaml


def make_nodes(context: LaunchContext, use_sim_time, map_yaml, serial_map,
               nav_params, x_pose, y_pose, yaw, autostart, bringup_delay,
               rviz, rviz_config, recovery):
    sim = context.perform_substitution(use_sim_time)
    sim_bool = sim.lower() == 'true'
    map_str = context.perform_substitution(map_yaml)
    serial_str = context.perform_substitution(serial_map)
    nav_str = context.perform_substitution(nav_params)
    x = float(context.perform_substitution(x_pose))
    y = float(context.perform_substitution(y_pose))
    th = float(context.perform_substitution(yaw))
    auto = context.perform_substitution(autostart)
    delay = float(context.perform_substitution(bringup_delay))
    rviz_on = context.perform_substitution(rviz).lower() == 'true'
    rviz_cfg = context.perform_substitution(rviz_config)
    recovery_on = context.perform_substitution(recovery).lower() == 'true'

    if not serial_str:
        serial_str = os.path.splitext(map_str)[0] + '_serial'
    slam_params = os.path.join(
        get_package_share_directory('oomwoo_sim_support'),
        'config', 'mapper_params_localization.yaml')
    common = {'use_sim_time': sim_bool}

    # AMCL + map_server via nav2's own (proven) localization bringup. Rewrite
    # ONLY tf_broadcast->false in the user's navigation.yaml so slam_toolbox can
    # own map->odom; everything else stays exactly as configured. AMCL still
    # publishes /amcl_pose, which the loc_err_amcl meter reads.
    amcl_rewrites = {'tf_broadcast': 'false'}
    if recovery_on:
        # let AMCL scatter particles when it loses track (kidnap recovery), so
        # its covariance rises for relocalize_on_lost to see.
        amcl_rewrites['recovery_alpha_slow'] = '0.001'
        amcl_rewrites['recovery_alpha_fast'] = '0.1'
    amcl_params = RewrittenYaml(
        source_file=nav_str, param_rewrites=amcl_rewrites, convert_types=True)
    # This image disables FastDDS shared memory, so inter-process lifecycle
    # service calls are slow enough that a separate-process map_server times out
    # the lifecycle_manager (configure fails, whole AMCL side aborts). Compose
    # the nav2 nodes into ONE container -- as navigation.launch.py does via
    # bringup_launch use_composition:=True -- so the lifecycle calls are
    # intra-process and instant.
    container = Node(
        package='rclcpp_components', executable='component_container_isolated',
        name='nav2_container', output='screen', parameters=[common])
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'),
            'launch', 'localization_launch.py')),
        launch_arguments={
            'map': map_str,
            'use_sim_time': sim,
            'autostart': auto,
            'params_file': amcl_params,
            'use_composition': 'True',
            'container_name': 'nav2_container'}.items())

    # AMCL has set_initial_pose:false, so seed it at the known spawn pose.
    seed = Node(
        package='oomwoo_sim_support', executable='initialpose_pub',
        name='initialpose_pub', output='screen',
        parameters=[common, {'x': x, 'y': y, 'yaw': th}])

    # slam_toolbox localization: owns the canonical map->odom TF. Its own /map
    # is remapped to /map_slam so it does not collide with nav2's map_server.
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

    # nav2 comes up first via its reliable bringup; start slam_toolbox a few
    # seconds later so it never competes with that bringup. The delay gates
    # nav2's fast bringup, not slam's variable pose-graph load.
    slam_actions = ([slam, slam_configure, slam_activate]
                    if auto.lower() == 'true' else [slam])
    slam_delayed = TimerAction(period=delay, actions=slam_actions)

    # RViz here, so you DON'T also run navigation.launch.py -- that would start a
    # SECOND map_server/amcl/lifecycle_manager stack in another nav2_container
    # and collide with this one (duplicate node names -> configure fails).
    nodes = [container, localization, seed, slam_delayed,
             truth, meter_amcl, meter_slam]
    if rviz_on:
        if not os.path.isabs(rviz_cfg):
            rviz_cfg = os.path.join(
                get_package_share_directory('oomwoo_one'), 'rviz', rviz_cfg)
        nodes.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', rviz_cfg], parameters=[common]))
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
            'bringup_delay', default_value='6.0',
            description='Seconds after launch to start slam_toolbox; nav2 '
                        '(map_server+amcl) comes up first in this window'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz here (do NOT also run navigation.launch.py '
                        '-- it starts a second, colliding nav2 stack)'),
        DeclareLaunchArgument('rviz_config', default_value='bump_map.rviz'),
        DeclareLaunchArgument(
            'recovery', default_value='false',
            description='Enable AMCL recovery_alpha (kidnap recovery) so its '
                        'covariance rises when lost; relocalize_on_lost needs '
                        'it'),
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
            LaunchConfiguration('rviz'),
            LaunchConfiguration('rviz_config'),
            LaunchConfiguration('recovery'),
        ]),
    ])
