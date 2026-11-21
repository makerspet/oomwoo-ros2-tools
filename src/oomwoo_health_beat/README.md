# oomwoo_health_beat

Opt-in work-path heartbeats for the stack health monitor (`docs/SOFTWARE_INTERFACES.md`, "Stack Health Monitor Draft").

A node that owns a `HealthBeat` and sets the parameter `health_component_id` publishes one JSON heartbeat
(`component_id`, `health`, `stamp_sec`, `sequence`, `detail`) on `/oomwoo/health/component`
(`std_msgs/String`, reliable, volatile, depth 10) each time `beat()` is called. `stamp_sec` is the node's own ROS clock,
converted as `nanoseconds / 1e9` like the monitor's own clock, so a beat on the monitor's sim-time tick has age 0, not -1 ulp.

- `health_component_id` defaults to `""`: `beat()` returns at once and no publisher is created, so every node behaves as
  before unless the parameter is set (for example `--ros-args -p health_component_id:=dock_detector`).
- There is no timer in this package. A beat is published only where the node calls `beat()`, after a unit of real work.
- `oomwoo_health_beat.core` has no ROS imports (payload builder, unit-tested with pytest); `oomwoo_health_beat.ros` is the wrapper.
- The health monitor compares `stamp_sec` with its own clock, so both must run on the same time base (all on sim time in
  Gazebo, all on wall time on the robot).
- All beats are level `ok`: none of the three nodes detects its own failure today.

## Where each node beats, and why there

| Node | Beat point | Why this point |
|---|---|---|
| `dock_detector` | `_on_scan`, after `_process_scan` returned for a scan that passed the `detect_hz` rate gate | One finished fit attempt is the node's unit of work, found or not (too few points, no fit, rejected, held, accepted all count). Scans dropped by the rate gate are not work and do not beat; the `/odom`, `~/prior` and `~/beacon_bearing` callbacks never beat. A wedged or dead detector stops processing scans and stops beating. Period about 0.6 s (5 Hz scans, 2 Hz gate). |
| `dock_drive` | `_tick` (20 Hz timer), after `_step` computed the state machine and published the command | The command publish is the node's output. The timer fires in every state, including IDLE and DONE (zero command), so an idle driver beats too; a dead driver stops. The `/dock_pose` and `/odom` callbacks never beat. |
| `coverage_planner` | `_tick` (5 Hz timer), after `_step` returned, for both `executor:=nav2` and `executor:=reactive` | `_step` is the code that advances the executor each cycle: waypoint dispatch, the goal watchdog, the wedge escape and (reactive) the direct drive. Only a planner that is alive and not blocked completes ticks. The map, pose, bumper and coverage callbacks never beat. Ticks before the map arrives, while waiting for a pose, and after "coverage complete" also beat: the planner is alive and has nothing to advance. |

Limits: the beats prove that the loop runs, not that its output is right. A planner whose pose never arrives or whose
waypoints are all skipped still beats; it is the node's own watchdogs (`no_progress_sec`, `goal_timeout_sec`) that handle that.
