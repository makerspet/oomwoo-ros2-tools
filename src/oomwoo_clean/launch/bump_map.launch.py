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
Tactile bump-map builder.

Runs the bump_map node alongside any driving (e.g. wall_clean_bump_out, or
coverage cleaning): it turns bumper contacts into /bump_map (OccupancyGrid, the
truly-solid keep-out layer) + /bump_map/walls (RViz wall segments). Values are
read from `kaia set bump_map.<name>` for the active robot (a launch arg still
wins, e.g. contact_radius:=0.19) and are live, so you can tune and relaunch:

  kaia set bump_map.contact_radius 0.19
  kaia set bump_map.max_segment_gap 0.8
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# name -> built-in default; each is overridable by kaia (bump_map.<name>) then :=
BUMP_PARAMS = {
    'contact_radius': 0.18,      # m: obstacle surface out from the robot center
    'contact_offset_deg': 45.0,  # deg: left/right bumper contact angle
    'resolution': 0.05,          # m/cell of the /bump_map grid
    'max_segment_gap': 1.0,      # m: gap above which a new wall run starts
    'refractory_sec': 0.8,       # s: min time between counted bumps
}


def _cfg(name, default):
    # Default from `kaia set bump_map.<name>` for the active robot, else built-in.
    try:
        from kaiaai import config  # dep-optional: guarded, falls back
        value = config.get_var(name)
        return default if value is None else value
    except Exception:
        return default


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('target_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument(
            'occupied_min_hits',
            default_value=str(_cfg('bump_map.occupied_min_hits', 1))),
    ]
    params = {
        'use_sim_time': ParameterValue(
            LaunchConfiguration('use_sim_time'), value_type=bool),
        'target_frame': LaunchConfiguration('target_frame'),
        'odom_frame': LaunchConfiguration('odom_frame'),
        'base_frame': LaunchConfiguration('base_frame'),
        'occupied_min_hits': ParameterValue(
            LaunchConfiguration('occupied_min_hits'), value_type=int),
    }
    for name, default in BUMP_PARAMS.items():
        args.append(DeclareLaunchArgument(
            name, default_value=str(_cfg('bump_map.' + name, default))))
        params[name] = ParameterValue(
            LaunchConfiguration(name), value_type=float)

    node = Node(
        package='oomwoo_clean', executable='bump_map', output='screen',
        parameters=[params],
        remappings=[('bumper_left/contact', '/bumper_left/contact'),
                    ('bumper_right/contact', '/bumper_right/contact')])
    return LaunchDescription(args + [node])
