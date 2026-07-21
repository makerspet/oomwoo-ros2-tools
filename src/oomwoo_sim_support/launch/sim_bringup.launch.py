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
"""
Headless OOMWOO simulation bringup shared by the regression harnesses.

Brings up, headless by default (offscreen rendering, CI/Docker friendly) or
with the Gazebo GUI on gui:=true; world/map default to test_room:
  * Gazebo via the ros_gz_sim gz_sim.launch.py wrapper (software GL headless)
  * robot_state_publisher + spawn (create -timeout, on Gazebo-ready)
  * ros_gz bridges: sim sensors/actuators + ground-truth model poses
  * Nav2 localization (map_server + AMCL) — gated on the spawn completing
  * Nav2 navigation (planner/controller/bt_navigator/behaviors) — 8 s after
    localization so AMCL's map->odom exists before the costmap activates
  * ground_truth pose publisher

Coverage- and relocalization-specific nodes are added by the including launch.
Startup is event-ordered (spawn-on-ready, Nav2-on-spawn) rather than
clock-ordered; the one deliberate stagger is the 8 s localization->nav gap, and
the application nodes also self-gate on their inputs.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg_sim = get_package_share_directory('oomwoo_sim_support')
    pkg_gazebo = get_package_share_directory('oomwoo_gazebo')

    # coarser 200 Hz physics step (vs the stock 1 kHz) so the bridged /clock
    # is 5x lighter — critical for a stable sim clock under x86 emulation.
    # World + map default to the primitives test_room but are overridable so the
    # same harness can drive the stock living_room (or any other world/map pair).
    default_world = os.path.join(pkg_sim, 'worlds', 'test_room.world')
    default_map = os.path.join(pkg_sim, 'maps', 'test_room.yaml')
    world = LaunchConfiguration('world')
    map_yaml = LaunchConfiguration('map')
    nav2_params = os.path.join(pkg_sim, 'config', 'nav2_params.yaml')

    x0 = LaunchConfiguration('x_pose')
    y0 = LaunchConfiguration('y_pose')
    yaw0 = LaunchConfiguration('yaw')

    args = [
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('map', default_value=default_map),
        # coverage needs the Nav2 nav servers; relocalization does not (it only
        # spins in place under AMCL), so it can bring up a much lighter stack.
        DeclareLaunchArgument('with_nav', default_value='true'),
        # gui:=true runs the IDENTICAL simulation with the Gazebo GUI attached
        # (needs a display); default stays fully headless for CI/regressions.
        DeclareLaunchArgument('gui', default_value='false'),
        # Robot description package, kaiaai-style: 'config' (default) follows
        # `kaia config robot.model <pkg>` (~/.kaiaai.yaml) exactly like the
        # kaiaai_bringup tutorials; any other value names the package directly.
        # The regression launches pin robot_model:=oomwoo_one so CI gates stay
        # reproducible regardless of the machine's kaia config.
        DeclareLaunchArgument('robot_model', default_value='config'),
    ]
    with_nav = IfCondition(LaunchConfiguration('with_nav'))
    gui = IfCondition(LaunchConfiguration('gui'))
    headless = UnlessCondition(LaunchConfiguration('gui'))

    # Software GL is only forced in headless mode; with the GUI we use the
    # host's real GL stack.
    set_env = [
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1', condition=headless),
        SetEnvironmentVariable('GALLIUM_DRIVER', 'llvmpipe', condition=headless),
    ]

    def robot_setup(context):
        """Actions that depend on the selected robot description package."""
        model = context.perform_substitution(
            LaunchConfiguration('robot_model'))
        if model in ('', 'config'):
            # kaiaai convention: read robot.model from ~/.kaiaai.yaml
            try:
                from kaiaai import config  # dep-optional: guarded, falls back
                model = config.get_var('robot.model')
            except Exception:
                model = 'oomwoo_one'
        pkg_robot = get_package_share_directory(model)
        xacro_file = os.path.join(pkg_robot, 'urdf', 'robot.urdf.xacro')
        bridge_sim = os.path.join(pkg_robot, 'config', 'gz_bridge.yaml')

        robot_description = ParameterValue(
            Command(['xacro ', xacro_file]), value_type=str)
        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher', output='screen',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': True}])
        bridge = Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            output='screen',
            parameters=[{'config_file': bridge_sim, 'use_sim_time': True}])
        # EVENT-BASED spawn (mirrors kaiaai's world.launch.py): create runs
        # immediately and WAITS on its own -timeout for Gazebo + the
        # robot_description topic to be ready, instead of firing on a fixed
        # 10 s timer that races a slow/cold GL init. -timeout matches kaiaai.
        spawn = Node(
            package='ros_gz_sim', executable='create', output='screen',
            arguments=['-world', 'default', '-topic', 'robot_description',
                       '-name', model, '-timeout', '180',
                       '-x', x0, '-y', y0, '-z', '0.06', '-Y', yaw0,
                       '-allow_renaming', 'false'])
        # Nav2 comes up only AFTER the robot is actually in the world — gated
        # on the spawn process exiting, not a fixed timer racing gz's cold
        # start. Localization (map_server + AMCL) starts on that event;
        # navigation follows a short, DETERMINISTIC 8 s later so AMCL has
        # published the map->odom transform before the Nav2 global costmap
        # activates (activating it first fails). This stagger is safe — it's
        # measured from the already-spawned robot, not the unpredictable
        # Gazebo startup — so it is not the fragile fixed-timer pattern.
        nav_on_spawn = RegisterEventHandler(
            OnProcessExit(
                target_action=spawn,
                on_exit=localization
                + [TimerAction(period=8.0, actions=navigation)]))
        return [
            # models + meshes resolvable by gz (stock kaiaai models + the robot
            # description). The stock living_room furniture — including the
            # TableMarble .dae — collides correctly as a trimesh headless, so no
            # model overrides are needed.
            SetEnvironmentVariable(
                'GZ_SIM_RESOURCE_PATH',
                os.pathsep.join([os.path.join(pkg_gazebo, 'models'),
                                 pkg_robot])),
            rsp, bridge, spawn, nav_on_spawn,
        ]

    # Gazebo via the official ros_gz_sim wrapper (mirrors kaiaai): it
    # coordinates the server↔GUI startup handshake that the raw `gz sim`
    # command doesn't — the fix for the intermittent black-screen GUI render.
    # Same world/physics either way; only the rendering surface differs, so
    # headless and GUI runs stay comparable. Headless passes -s
    # (server-only) + --headless-rendering so software GL still drives the
    # offscreen LiDAR in CI.
    gz_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={
            'gz_args': ['-s -r --headless-rendering -v 1 ', world],
            'on_exit_shutdown': 'true'}.items(),
        condition=headless)
    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={
            'gz_args': ['-r -v 1 ', world],
            'on_exit_shutdown': 'true'}.items(),
        condition=gui)

    # Explicit localization (map_server + AMCL + lifecycle) with an absolute
    # map path — avoids nav2_bringup's map-arg substitution quirk and keeps the
    # saved map self-contained inside this package.
    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml, 'use_sim_time': True,
                     'topic_name': 'map', 'frame_id': 'map'}])
    # Self-initialize AMCL at the known spawn pose. This uses the configured
    # pose directly (no stamped-message TF lookup), avoiding the "extrapolation
    # into the future" race that blocks localization under a slow sim clock.
    amcl = Node(
        package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        parameters=[nav2_params, {
            'use_sim_time': True, 'set_initial_pose': True,
            # seed AMCL at the spawn pose (float-coerced from the launch args) so
            # a non-origin start (e.g. the clear cell in the cluttered living_room)
            # localizes immediately; test_room keeps its 0,0 default.
            'initial_pose.x': ParameterValue(x0, value_type=float),
            'initial_pose.y': ParameterValue(y0, value_type=float),
            'initial_pose.z': 0.0,
            'initial_pose.yaw': ParameterValue(yaw0, value_type=float),
            # update the filter a bit more often (vs 0.25 m / 0.2 rad) so a
            # kidnapped robot re-converges faster during the recovery drive.
            # recovery_alpha stays disabled (stock): continuous particle
            # injection keeps the published covariance permanently inflated,
            # which destroys the convergence signal both the recovery node and
            # the regression depend on. Wrong-mode escape comes from the
            # explicit global re-init + explore motion instead.
            'update_min_d': 0.15, 'update_min_a': 0.1,
            # SHARP measurement model for the noise-free sim LiDAR. The stock
            # z_hit 0.5 / z_rand 0.5 / 60-beam model is so permissive that a
            # mirrored symmetric hypothesis survives indefinitely after a global
            # re-init (observed: covariance trace pinned at ~6.5 for 45 s).
            # Weighting hits strongly and sampling more beams makes the ghost
            # mode collapse within a few updates.
            'z_hit': 0.95, 'z_rand': 0.05, 'max_beams': 120}])
    lifecycle_loc = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True,
                     'bond_timeout': 0.0,
                     'node_names': ['map_server', 'amcl']}])
    localization = [map_server, amcl, lifecycle_loc]

    # Trimmed Nav2: only the servers coverage needs (planner, controller,
    # behaviors for recovery, bt_navigator). Dropping route/docking/collision/
    # smoother/velocity_smoother/waypoint servers keeps RAM+CPU low enough to
    # run on a 2-core / 3 GB machine. The controller publishes /cmd_vel directly
    # (no smoother chain to remap around).
    nav_common = {'use_sim_time': True}
    controller = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen', condition=with_nav,
        parameters=[nav2_params, nav_common])
    planner = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen', condition=with_nav,
        parameters=[nav2_params, nav_common])
    behavior = Node(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server', output='screen', condition=with_nav,
        parameters=[nav2_params, nav_common])
    # the params yaml uses $(find-pkg-share ...) for the BT XML paths, which
    # only nav2_bringup's RewrittenYaml expands — passing the yaml raw leaves
    # the literal string and bt_navigator fails to activate. Override with
    # resolved absolute paths.
    bt_dir = os.path.join(
        get_package_share_directory('nav2_bt_navigator'), 'behavior_trees')
    bt_nav = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen', condition=with_nav,
        parameters=[nav2_params, nav_common, {
            'default_nav_to_pose_bt_xml': os.path.join(
                bt_dir, 'navigate_to_pose_w_replanning_and_recovery.xml'),
            'default_nav_through_poses_bt_xml': os.path.join(
                bt_dir,
                'navigate_through_poses_w_replanning_and_recovery.xml')}])
    lifecycle_nav = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen', condition=with_nav,
        parameters=[{'use_sim_time': True, 'autostart': True, 'bond_timeout': 0.0,
                     'node_names': ['controller_server', 'planner_server',
                                    'behavior_server', 'bt_navigator']}])
    navigation = [controller, planner, behavior, bt_nav, lifecycle_nav]

    ground_truth = Node(
        package='oomwoo_sim_support', executable='ground_truth', output='screen',
        # float-coerced like the AMCL seed below: a whole-number override
        # (x_pose:=2, YAW=0) would otherwise YAML-parse as int and crash the
        # statically-double-typed node parameters at startup
        parameters=[{'spawn_x': ParameterValue(x0, value_type=float),
                     'spawn_y': ParameterValue(y0, value_type=float),
                     'spawn_yaw': ParameterValue(yaw0, value_type=float),
                     'use_sim_time': True}],
        remappings=[('odom', '/odom'), ('~/pose', '/ground_truth/pose')])

    # AMCL self-initializes at spawn (set_initial_pose); the standalone
    # initialpose_pub node remains available for bringups that need to seed a
    # pose over /initialpose instead.

    # Event-ordered, not clock-ordered: gz starts immediately, robot_setup
    # spawns the robot the moment Gazebo is ready (create -timeout), and
    # localization is gated on that spawn completing. The one remaining timer
    # is the deliberate 8 s localization->nav stagger inside robot_setup (so
    # AMCL publishes map->odom before the Nav2 costmap activates) — measured
    # from the already-spawned robot, not from the unpredictable gz start.
    # ground_truth and the application nodes self-gate on their own inputs.
    return LaunchDescription(args + set_env + [
        gz_server, gz_gui, ground_truth,
        OpaqueFunction(function=robot_setup),
    ])
