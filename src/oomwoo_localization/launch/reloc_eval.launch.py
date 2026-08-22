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
Batch-evaluate the branch-and-bound global relocalizer against ground truth.

Bring up the sim + the localization scene first (they provide /map,
kidnap_injector, and ground_truth), then run this:

  ros2 launch oomwoo_gazebo world.launch.py odom_source:=robot_wheels
  ros2 launch oomwoo_sim_support localization_relocalize.launch.py \\
    use_sim_time:=true map:=/ros_ws/src/oomwoo_gazebo/maps/living_room.yaml
  ros2 launch oomwoo_localization reloc_eval.launch.py use_sim_time:=true \\
    csv_path:=/root/reloc_eval.csv

reloc_eval kidnaps to a systematic pose grid, calls the relocalizer, scores it
against ground truth, prints a summary, and exits non-zero if the success rate
misses min_success_rate (so it is also a CI regression gate).
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
        DeclareLaunchArgument('csv_path', default_value=''),
        DeclareLaunchArgument(
            'publish_initialpose', default_value='false',
            description='Have global_relocalizer seed /initialpose on each '
                        'accepted fix, so the robot visibly jumps in RViz '
                        'during the batch run (for demos/recording). The eval '
                        'still scores BnB against ground truth independently'),
        Node(
            package='oomwoo_localization', executable='global_relocalizer',
            name='global_relocalizer', output='screen',
            parameters=[{
                'use_sim_time': sim,
                'publish_initialpose': ParameterValue(
                    LaunchConfiguration('publish_initialpose'),
                    value_type=bool)}]),
        Node(
            package='oomwoo_localization', executable='reloc_eval',
            name='reloc_eval', output='screen',
            parameters=[{
                'use_sim_time': sim,
                'csv_path': LaunchConfiguration('csv_path')}]),
    ])
