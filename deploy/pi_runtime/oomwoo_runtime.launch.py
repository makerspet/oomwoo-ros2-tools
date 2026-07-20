# Copyright 2026 Jayadev Rana
# SPDX-License-Identifier: Apache-2.0
"""OOMWOO onboard runtime graph for the Pi 4/5 4GB baseline (NO Gazebo).

This is the software that would run on the robot computer: robot_state_publisher,
SLAM or AMCL localization, the trimmed Nav2 stack, and the M1 behaviours
(coverage planner + kidnapped-robot recovery). It's the graph xbattlax's
measurement plan profiles for RSS/PSS/CPU.

  mode:=idle   rsp only (+ the simulated MCU serial link) — the floor baseline
  mode:=slam   + slam_toolbox mapping (drive it with a 5 Hz /scan bag)
  mode:=nav    + map_server + AMCL + Nav2 + M1 behaviours on a known map

No robot/LiDAR is attached during the baseline; replay a recorded bag
(scan+odom+tf, use_sim_time) to exercise SLAM/Nav2 at the real 5 Hz rate.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _setup(context, *_):
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('idle', 'slam', 'nav'):
        # fail LOUDLY: a typo like mode:=navigation would otherwise silently
        # bring up the bare idle graph and the baseline would measure nothing
        raise ValueError(f'mode must be idle|slam|nav, got {mode!r}')
    robot_model = LaunchConfiguration('robot_model').perform(context) or 'oomwoo_one'
    # accept true/True/1 — '== "true"' silently made use_sim_time:=True False
    use_sim_time = LaunchConfiguration(
        'use_sim_time').perform(context).lower() in ('true', '1')
    # config lives beside this launch file so the onboard runtime has NO
    # dependency on oomwoo_sim_support (which pulls Gazebo and isn't installed
    # on the robot).
    here = os.path.dirname(os.path.abspath(__file__))
    pkg_robot = get_package_share_directory(robot_model)
    nav2_params = LaunchConfiguration('nav2_params').perform(context) \
        or os.path.join(here, 'config', 'nav2_params.yaml')
    map_yaml = LaunchConfiguration('map').perform(context) \
        or os.path.join(here, 'config', 'test_room.yaml')
    common = {'use_sim_time': use_sim_time}

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(pkg_robot, 'urdf', 'robot.urdf.xacro')]),
        value_type=str)
    nodes = [Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, **common}])]

    if mode == 'slam':
        nodes.append(Node(
            package='slam_toolbox', executable='async_slam_toolbox_node',
            name='slam_toolbox', output='screen',
            parameters=[{
                **common, 'odom_frame': 'odom', 'map_frame': 'map',
                'base_frame': 'base_footprint', 'scan_topic': '/scan',
                'mode': 'mapping', 'resolution': 0.05,
                'minimum_time_interval': 0.2, 'transform_timeout': 0.5}]))

    if mode == 'nav':
        bt_dir = os.path.join(
            get_package_share_directory('nav2_bt_navigator'), 'behavior_trees')
        lifecycle = ['map_server', 'amcl']
        nodes += [
            Node(package='nav2_map_server', executable='map_server',
                 name='map_server', output='screen',
                 parameters=[{'yaml_filename': map_yaml, 'topic_name': 'map',
                              'frame_id': 'map', **common}]),
            # seed AMCL so it publishes map->odom immediately; without it the
            # global costmap never sees the 'map' frame and planning won't
            # activate. The baseline bag starts the robot near the origin.
            Node(package='nav2_amcl', executable='amcl', name='amcl',
                 output='screen', parameters=[nav2_params, common, {
                     'set_initial_pose': True, 'initial_pose.x': 0.0,
                     'initial_pose.y': 0.0, 'initial_pose.z': 0.0,
                     'initial_pose.yaw': 0.0}]),
            Node(package='nav2_controller', executable='controller_server',
                 name='controller_server', output='screen',
                 parameters=[nav2_params, common]),
            Node(package='nav2_planner', executable='planner_server',
                 name='planner_server', output='screen',
                 parameters=[nav2_params, common]),
            Node(package='nav2_behaviors', executable='behavior_server',
                 name='behavior_server', output='screen',
                 parameters=[nav2_params, common]),
            Node(package='nav2_bt_navigator', executable='bt_navigator',
                 name='bt_navigator', output='screen',
                 parameters=[nav2_params, common, {
                     # nav2_params points the BT XMLs at $(find-pkg-share ...),
                     # which only nav2_bringup's RewrittenYaml expands; pass the
                     # resolved absolute paths or bt_navigator won't activate.
                     'default_nav_to_pose_bt_xml': os.path.join(
                         bt_dir,
                         'navigate_to_pose_w_replanning_and_recovery.xml'),
                     'default_nav_through_poses_bt_xml': os.path.join(
                         bt_dir,
                         'navigate_through_poses_w_replanning_and_recovery.xml'),
                 }]),
            Node(package='nav2_lifecycle_manager',
                 executable='lifecycle_manager', name='lifecycle_manager',
                 output='screen',
                 parameters=[{**common, 'autostart': True, 'bond_timeout': 0.0,
                              'node_names': lifecycle + [
                                  'controller_server', 'planner_server',
                                  'behavior_server', 'bt_navigator']}]),
            # M1 high-level behaviours (no Gazebo deps)
            Node(package='oomwoo_coverage', executable='coverage_planner',
                 output='screen', parameters=[common],
                 remappings=[('navigate_to_pose', '/navigate_to_pose')]),
            Node(package='oomwoo_nav_localize', executable='kidnap_recovery',
                 output='screen', parameters=[common]),
        ]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='idle',
                              description='idle | slam | nav'),
        DeclareLaunchArgument('robot_model', default_value='oomwoo_one'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('nav2_params', default_value=''),
        DeclareLaunchArgument('map', default_value=''),
        OpaqueFunction(function=_setup),
    ])
