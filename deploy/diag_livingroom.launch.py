# SPDX-License-Identifier: Apache-2.0
# Throwaway: headless gz on a chosen world + robot spawn, no nav/amcl.
# Reuses the proven xacro->robot_description wiring from sim_bringup so
# robot_state_publisher actually comes up (CLI -p on multi-line URDF does not).
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_gz = get_package_share_directory('kaiaai_gazebo')
    pkg_ow = get_package_share_directory('oomwoo_one')
    default_world = os.path.join(pkg_gz, 'worlds', 'living_room.world')
    xacro_file = os.path.join(pkg_ow, 'urdf', 'robot.urdf.xacro')
    bridge = os.path.join(pkg_ow, 'config', 'gz_bridge.yaml')

    world = LaunchConfiguration('world')
    x = LaunchConfiguration('x_pose')
    y = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('x_pose', default_value='0.394'),
        DeclareLaunchArgument('y_pose', default_value='-0.3'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.pathsep.join([os.path.join(pkg_gz, 'models'), pkg_ow])),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        SetEnvironmentVariable('GALLIUM_DRIVER', 'llvmpipe'),
        ExecuteProcess(
            cmd=['gz', 'sim', '-s', '-r', '--headless-rendering', '-v', '4', world],
            output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': robot_description, 'use_sim_time': True}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': bridge, 'use_sim_time': True}]),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-world', 'default', '-topic', 'robot_description',
                        '-name', 'oomwoo_one', '-x', x, '-y', y, '-z', '0.06', '-Y', yaw]),
    ])
