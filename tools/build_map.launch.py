# SPDX-License-Identifier: Apache-2.0
"""Map the actual living_room world with slam_toolbox while a simple reactive
wanderer drives the robot around. Used once to (re)generate the saved map that
localization + coverage run against — the map must agree with what the sim
LiDAR actually sees."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg_sim = get_package_share_directory('oomwoo_sim_support')
    pkg_gazebo = get_package_share_directory('oomwoo_gazebo')
    pkg_oomwoo = get_package_share_directory('oomwoo_one')

    world = os.path.join(pkg_sim, 'worlds', 'test_room.world')
    xacro_file = os.path.join(pkg_oomwoo, 'urdf', 'robot.urdf.xacro')
    bridge_sim = os.path.join(pkg_oomwoo, 'config', 'gz_bridge.yaml')

    set_env = [
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.pathsep.join([os.path.join(pkg_gazebo, 'models'), pkg_oomwoo])),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        SetEnvironmentVariable('GALLIUM_DRIVER', 'llvmpipe'),
    ]

    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', '--headless-rendering', '-v', '1', world],
        output='screen')
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]), value_type=str),
            'use_sim_time': True}])
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': bridge_sim, 'use_sim_time': True}])
    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-world', 'default', '-topic', 'robot_description',
                   '-name', 'oomwoo_one', '-x', '0.0', '-y', '0.0',
                   '-z', '0.06'])

    slam = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', output='screen',
        parameters=[{'use_sim_time': True,
                     'odom_frame': 'odom', 'base_frame': 'base_footprint',
                     'map_frame': 'map', 'resolution': 0.05,
                     'max_laser_range': 9.0,
                     'minimum_travel_distance': 0.10,
                     'minimum_travel_heading': 0.15}])

    wanderer = ExecuteProcess(
        cmd=['python3', '/root/oomwoo-dev/tools/wanderer.py'], output='screen')

    # slam_toolbox is a lifecycle node in Jazzy — it must be activated
    slam_activate = ExecuteProcess(
        cmd=['bash', '-c',
             'source /opt/ros/jazzy/setup.bash && sleep 4 && '
             'ros2 lifecycle set /slam_toolbox configure && '
             'ros2 lifecycle set /slam_toolbox activate'],
        output='screen')

    return LaunchDescription(set_env + [
        gz_server, rsp, bridge,
        TimerAction(period=8.0, actions=[spawn]),
        TimerAction(period=12.0, actions=[slam, slam_activate]),
        TimerAction(period=18.0, actions=[wanderer]),
    ])
