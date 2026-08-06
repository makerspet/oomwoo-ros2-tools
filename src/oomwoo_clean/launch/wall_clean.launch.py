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
Reactive bump-based wall cleaning behavior (swaps in for the cleaning command).

Runs the wall_clean node -- no Nav2, no map, just bumpers -> /cmd_vel. Position
the robot at a wall with teleop first, then start this. Every motion value is
read from `kaia set clean.<name>` for the active robot (a launch arg still wins,
e.g. v_cruise:=0.2), so you can tune it live and relaunch:

  kaia set clean.v_cruise 0.2
  kaia set clean.turn_left_deg 80
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# name -> built-in default; each is overridable by kaia (clean.<name>) then :=
CLEAN_PARAMS = {
    'v_cruise': 0.15,       # forward cleaning speed (m/s)
    'arc_omega': 0.05,       # gentle right-arc rate while cruising (rad/s)
    'v_back': 0.10,         # backoff reverse speed (m/s)
    'backoff_s': 0.5,       # backoff duration (s)
    'turn_speed': 0.7,      # angular speed while turning left (rad/s)
    'turn_right_deg': 20.0,  # right bumper: small peel-off
    'turn_left_deg': 90.0,   # left bumper: round a corner
    'turn_both_deg': 60.0,   # both bumpers: head-on
}


def _cfg(name, default):
    # Default from `kaia set clean.<name>` for the active robot, else built-in.
    try:
        from kaiaai import config  # dep-optional: guarded, falls back
        value = config.get_var(name)
        return default if value is None else value
    except Exception:
        return default


def generate_launch_description() -> LaunchDescription:
    args = [DeclareLaunchArgument('use_sim_time', default_value='true')]
    params = {'use_sim_time': ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)}
    for name, default in CLEAN_PARAMS.items():
        args.append(DeclareLaunchArgument(
            name, default_value=str(_cfg('clean.' + name, default))))
        params[name] = ParameterValue(
            LaunchConfiguration(name), value_type=float)

    node = Node(
        package='oomwoo_clean', executable='wall_clean', output='screen',
        parameters=[params],
        remappings=[('cmd_vel', '/cmd_vel'),
                    ('bumper_left/contact', '/bumper_left/contact'),
                    ('bumper_right/contact', '/bumper_right/contact'),
                    ('cleaning_active', '/coverage_planner/cleaning_active')])
    return LaunchDescription(args + [node])
