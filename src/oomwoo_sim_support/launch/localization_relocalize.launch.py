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
Relocalization scene: kidnap the robot, watch AMCL vs slam_toolbox recover.

Brings up the localization_compare stack (both localizers, ground truth, two
live error meters, RViz) plus kidnap_injector. Run the sim first, then this:

  ros2 launch oomwoo_gazebo world.launch.py odom_source:=robot_wheels
  ros2 launch oomwoo_sim_support localization_relocalize.launch.py \\
    use_sim_time:=true map:=/ros_ws/src/oomwoo_gazebo/maps/living_room.yaml

Use odom_source:=robot_wheels here, NOT ground_truth: a kidnap only fools a
localizer whose ODOMETRY is unaware of the jump. With ground_truth odom the
sim's odometry teleports with the robot, so slam_toolbox rides it and its error
never moves; with robot_wheels the wheels do not turn during a teleport, so odom
stays put and the localizer genuinely gets lost.

Teleport the robot and watch both error plots spike:

  # random reachable pose:
  ros2 service call /kidnap_injector/kidnap std_srvs/srv/Trigger {}
  # or a SPECIFIED pose:
  ros2 topic pub --once /kidnap_injector/kidnap_to geometry_msgs/msg/PoseStamped \\
    "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.5}}}"

relocalize_on_lost then recovers automatically: reinitialize AMCL globally,
spin to converge, and re-seed slam_toolbox at the recovered pose.

recovery:=false (default) leaves AMCL's recovery_alpha at 0, so a global
reinitialize converges cleanly (as in a manual kidnap test). recovery:=true
turns on Augmented-MCL (recovery_alpha_slow/fast), which lets AMCL's covariance
rise on its own so relocalize_on_lost can DETECT lost-ness passively -- but the
continuous random-particle injection also fights a global reinit and can settle
on a wrong cluster, so keep it off unless you are exercising passive detection.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory('oomwoo_sim_support')
    default_map = os.path.join(
        get_package_share_directory('oomwoo_gazebo'), 'map', 'living_room.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument(
            'recovery', default_value='false',
            description='Enable AMCL recovery_alpha (Augmented-MCL) so its '
                        'covariance rises when lost, for passive detection. '
                        'Off by default: the random-particle injection fights a '
                        'global reinit and can converge to a wrong cluster'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'localization_compare.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': LaunchConfiguration('map'),
                'recovery': LaunchConfiguration('recovery')}.items()),
        Node(
            package='oomwoo_sim_support', executable='kidnap_injector',
            name='kidnap_injector', output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}]),
        # Auto recovery: detect lost -> reinitialize AMCL + spin -> re-seed
        # slam_toolbox at the recovered pose (navigation back to normal).
        Node(
            package='oomwoo_sim_support', executable='relocalize_on_lost',
            name='relocalize_on_lost', output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}]),
    ])
