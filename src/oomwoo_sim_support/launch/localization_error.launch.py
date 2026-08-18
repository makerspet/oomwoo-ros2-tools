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
"""
Localization-error measurement: ground_truth + localization_error together.

Scores the running localizer (AMCL, or slam_toolbox localization) against the
sim's ground truth while you drive, for an honest before/after A/B. Run it with
odom_source:=ground_truth so `/odom` is truth; x_pose/y_pose/yaw must match
the robot's spawn pose in world.launch.py (defaults match navigation.launch.py).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration('use_sim_time')
    x = LaunchConfiguration('x_pose')
    y = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')
    sim = {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='-0.5'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        Node(
            package='oomwoo_sim_support', executable='ground_truth',
            name='ground_truth', output='screen',
            parameters=[sim, {
                'spawn_x': ParameterValue(x, value_type=float),
                'spawn_y': ParameterValue(y, value_type=float),
                'spawn_yaw': ParameterValue(yaw, value_type=float)}]),
        Node(
            package='oomwoo_sim_support', executable='localization_error',
            name='localization_error', output='screen',
            parameters=[sim]),
    ])
