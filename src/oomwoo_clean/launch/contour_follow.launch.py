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
Run the reactive LiDAR contour follower (Phase 1: follow + convex arc).

Position the robot near a wall/obstacle with teleop first (roughly the follow
side facing it), then launch this -- it ALIGNs, then traces the boundary. Tune
live, e.g. ros2 param set /contour_follower standoff_m 0.22. Stop with Ctrl-C.

RViz is deliberately NOT started here: bring the view up first with
contour_follow_rviz.launch.py, arrange the windows and set the camera, and only
then start following (and the screen capture).

  ros2 launch oomwoo_gazebo world.launch.py odom_source:=robot_wheels
  ros2 launch oomwoo_clean contour_follow_rviz.launch.py use_sim_time:=true
  ros2 run kaiaai_teleop teleop_keyboard          # park near a wall, then quit
  ros2 launch oomwoo_clean contour_follow.launch.py use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('follow_side', default_value='right'),
        DeclareLaunchArgument('standoff_m', default_value='0.20'),
        Node(
            package='oomwoo_clean', executable='contour_follower',
            name='contour_follower', output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'follow_side': LaunchConfiguration('follow_side'),
                'standoff_m': ParameterValue(
                    LaunchConfiguration('standoff_m'), value_type=float),
            }]),
    ])
