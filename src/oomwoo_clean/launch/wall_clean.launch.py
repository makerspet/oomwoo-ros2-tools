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
Wall cleaning entry point, reserved for fully-featured (sensor-based) cleaning.

For now it forwards to the reactive bump-out clean
(wall_clean_bump_out.launch.py, see docs/wall-follow-bump-out.md), so
`ros2 launch oomwoo_clean wall_clean.launch.py` keeps working unchanged: every
argument (use_sim_time, the clean.* motion params) passes straight through. When
the fully-featured wall cleaning lands, its launch replaces the body below.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    bump_out = os.path.join(
        get_package_share_directory('oomwoo_clean'),
        'launch', 'wall_clean_bump_out.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(bump_out)),
    ])
