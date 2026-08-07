<div align="center">

# OOMWOO Bringup

*Open-source robot vacuum you build yourself.*

ROS 2 Jazzy · Nav2 · SLAM · Launch · Bringup

![License](https://img.shields.io/badge/license-Apache--2.0-blue)
![Status](https://img.shields.io/badge/status-active-brightgreen)
[![Part of OOMWOO](https://img.shields.io/badge/part%20of-OOMWOO-5eead4)](https://github.com/makerspet/oomwoo)

</div>

ROS 2 bring-up and launch package for the [OOMWOO](https://github.com/makerspet/oomwoo)
open-source robot vacuum.

A fork of [kaiaai/kaiaai_bringup](https://github.com/kaiaai/kaiaai_bringup) (jazzy),
renamed to the **`oomwoo_bringup`** package, with `navigation.launch.py` pointing at
[`oomwoo_gazebo`](https://github.com/makerspet/oomwoo_gazebo) for its default map.

## Package contents
- `launch/navigation.launch.py` — Nav2 + localization, or SLAM with `slam:=True`.
- `launch/physical.launch.py` — physical-robot bring-up.
- `launch/cartographer.launch.py`, `explore.launch.py`, `occupancy_grid.launch.py`,
  `monitor_robot.launch.py` — mapping / exploration / monitoring.
- `launch/inspect_urdf.launch.py`, `edit_urdf.launch.py`, `publish_urdf.launch.py` —
  URDF inspection and live-edit helpers.
- `script/` — `watch_urdf.sh`, `upload_robot_description_github.sh`.

The robot model follows `kaia config robot.model` (`oomwoo_one` in the OOMWOO dev
image); override on any launch with `robot_model:=<description_package>`.

## Usage
```
# navigate a known map in simulation
ros2 launch oomwoo_bringup navigation.launch.py use_sim_time:=true

# build a map with SLAM
ros2 launch oomwoo_bringup navigation.launch.py use_sim_time:=true slam:=True
```
For the OOMWOO headless simulation and the coverage / navigation regressions, see the
`oomwoo_sim_support` harness in
[oomwoo-ros2-tools](https://github.com/makerspet/oomwoo-ros2-tools).

## Credits
Forked from [kaiaai/kaiaai_bringup](https://github.com/kaiaai/kaiaai_bringup)
(Apache-2.0). Still depends on the kaiaai ecosystem packages (`kaiaai`,
`kaiaai_teleop`, `kaiaai_telemetry`).

## License
[Apache License 2.0](LICENSE).
