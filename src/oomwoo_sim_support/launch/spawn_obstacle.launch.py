# Copyright 2026 Jayadev Rana
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
Spawn a thin wall into the running Gazebo sim -- a real, off-map obstacle.

A 3D box the map does not know about: it occludes the LiDAR physically (so it
just shows up in /scan), is anchored in the world (drive up to it, around it),
and is visible in both Gazebo and RViz. Use it to stress localization -- see
localization_stress.launch.py for the slam + loc_err_slam scoring, with a
filter:=true arm that routes slam through localization_health's /scan_filtered.

Run it AFTER a world is up (needs the /world/<world>/create service). Pick x/y
on open floor you can see; yaw turns the wall's broad face toward the robot:

  ros2 launch oomwoo_gazebo world.launch.py odom_source:=robot_wheels
  ros2 launch oomwoo_sim_support spawn_obstacle.launch.py x:=0.0 y:=-0.5

Remove it (or re-place it) between runs:

  ros2 service call /world/default/remove ros_gz_interfaces/srv/DeleteEntity \
    "{entity: {name: stress_wall, type: 2}}"
"""

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def spawn(context: LaunchContext, name, world, x, y, yaw,
          length, thickness, height):
    nm = context.perform_substitution(name)
    wr = context.perform_substitution(world)
    xx = context.perform_substitution(x)
    yy = context.perform_substitution(y)
    yw = context.perform_substitution(yaw)
    ln = float(context.perform_substitution(length))
    th = float(context.perform_substitution(thickness))
    ht = float(context.perform_substitution(height))
    size = '%g %g %g' % (ln, th, ht)
    # a static box: sensed by the (rendering) gpu_lidar via its visual, and it
    # will not topple or drift. Red so it is obvious in Gazebo and RViz.
    sdf = (
        '<?xml version="1.0"?>'
        '<sdf version="1.8"><model name="%s"><static>true</static>'
        '<link name="link">'
        '<collision name="collision"><geometry><box><size>%s</size></box>'
        '</geometry></collision>'
        '<visual name="visual"><geometry><box><size>%s</size></box></geometry>'
        '<material><ambient>0.85 0.1 0.1 1</ambient>'
        '<diffuse>0.85 0.1 0.1 1</diffuse></material></visual>'
        '</link></model></sdf>' % (nm, size, size))
    return [Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-world', wr, '-name', nm, '-string', sdf,
                   '-x', xx, '-y', yy, '-z', '%g' % (ht / 2.0), '-Y', yw])]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='stress_wall'),
        DeclareLaunchArgument('world', default_value='default'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='-0.5'),
        DeclareLaunchArgument(
            'yaw', default_value='1.5708',
            description='wall heading (rad); 1.5708 faces its broad side '
                        'toward a robot driving along +x'),
        DeclareLaunchArgument('length', default_value='1.5'),
        DeclareLaunchArgument('thickness', default_value='0.05'),
        DeclareLaunchArgument('height', default_value='1.0'),
        OpaqueFunction(function=spawn, args=[
            LaunchConfiguration('name'),
            LaunchConfiguration('world'),
            LaunchConfiguration('x'),
            LaunchConfiguration('y'),
            LaunchConfiguration('yaw'),
            LaunchConfiguration('length'),
            LaunchConfiguration('thickness'),
            LaunchConfiguration('height'),
        ]),
    ])
