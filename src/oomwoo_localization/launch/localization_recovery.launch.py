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
The full lost-detect -> relocalize -> decide recovery chain.

Brings up the three pieces that cooperate but stay decoupled:
  localization_health   measures scan-vs-map quality -> ~/quality (mechanism)
  global_relocalizer    ~/relocalize: best pose + confidence (pure mechanism)
  localization_manager  the POLICY: watch ~/quality, DECIDE lost
                        (-> /localization_lost), commit a confident fix to
                        /initialpose, or emit a fallback action (dock search)

Run the sim + localization scene WITHOUT the old AMCL spin-recovery first:

  ros2 launch oomwoo_gazebo world.launch.py odom_source:=robot_wheels
  ros2 launch oomwoo_sim_support localization_relocalize.launch.py \\
    use_sim_time:=true auto_recovery:=false \\
    map:=/ros_ws/src/oomwoo_gazebo/maps/living_room.yaml
  ros2 launch oomwoo_localization localization_recovery.launch.py use_sim_time:=true

Then kidnap and watch it run end to end (/localization_lost fires,
~/recovery_action shows commit or the fallback, the RViz robot jumps on commit).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration('use_sim_time')
    sim = ParameterValue(use_sim_time, value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='oomwoo_localization', executable='localization_health',
            name='localization_health', output='screen',
            parameters=[{'use_sim_time': sim}]),
        Node(
            package='oomwoo_localization', executable='global_relocalizer',
            name='global_relocalizer', output='screen',
            parameters=[{'use_sim_time': sim}]),
        Node(
            package='oomwoo_localization', executable='localization_manager',
            name='localization_manager', output='screen',
            parameters=[{'use_sim_time': sim}]),
    ])
