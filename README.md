<div align="center">

# OOMWOO ROS 2 Tools

*Open-source robot vacuum you build yourself.*

Cleaning · Docking · Localization · ROS 2 Jazzy · Nav2 · Gazebo

![License](https://img.shields.io/badge/license-Apache--2.0-blue)
![Status](https://img.shields.io/badge/status-active-brightgreen)
[![Part of OOMWOO](https://img.shields.io/badge/part%20of-OOMWOO-5eead4)](https://github.com/makerspet/oomwoo)

</div>

Reference ROS 2 packages for the [OOMWOO](https://github.com/makerspet/oomwoo)
open-source robot vacuum — **one home for the vacuum's ROS 2 packages** instead of a
repo per package. Behaviours like partition-and-clean, localization, docking and
map-and-clean live here so they build, test and version together.

## Packages

- **`oomwoo_coverage`** — boustrophedon cell-decomposition coverage cleaning: drives a
  full-room sweep through Nav2 (or the reactive executor, `executor:=reactive`), with
  gap-fill and wedge recovery.
- **`oomwoo_nav_localize`** — kidnapped-robot relocalization (global correlative
  scan-match + AMCL) so the robot can recover its pose from any start position on a
  known map.
- **`oomwoo_sim_support`** — everything needed to run *and measure* the above headless
  in Gazebo: sim bring-up, a ground-truth pose publisher, the coverage meter, the
  kidnap injector, and the CLI regression runners.
- **`oomwoo_clean`** — cleaning-specific navigation and RViz debug tooling
  (`cleaning_debug.launch.py`), the home for the from-scratch cleaning rebuild.

## Tutorials

Step-by-step guides on makerspet.com:

- [Simulate OOMWOO-One in Gazebo with ROS 2](https://makerspet.com/blog/simulate-oomwoo-one-robot-vacuum-in-gazebo-with-ros-2/) — bring up the simulation from scratch.
- [Write your first OOMWOO ROS 2 package](https://makerspet.com/blog/write-your-first-oomwoo-ros-2-package/) — a hello-world package in the dev image.
- [Headless sim & coverage cleaning (for LLM agents)](https://makerspet.com/blog/oomwoo-headless-sim-coverage-cleaning-llm-agents/) — run and measure cleaning with no display.
- [The phantom obstacle: teaching OOMWOO to clean near walls](https://makerspet.com/blog/oomwoo-phantom-obstacle-cleaning-fix/) — how the "wedges near furniture" bug was root-caused and fixed.

## Essential commands

Run inside the `makerspet/oomwoo:jazzy-dev` container (each launch in its own
terminal). The default robot model is `oomwoo_one`.

```bash
# select the robot description
kaia config robot.model oomwoo_one

# start the Gazebo simulation
ros2 launch oomwoo_gazebo world.launch.py

# build a map with SLAM while you drive
ros2 launch oomwoo_bringup navigation.launch.py use_sim_time:=true slam:=True

# open RViz to watch the robot and map
ros2 launch oomwoo_bringup monitor_robot.launch.py use_sim_time:=true

# drive with the keyboard
ros2 run kaiaai_teleop teleop_keyboard

# watch the bumpers (contact-tolerant cleaning relies on these)
ros2 topic echo /bumper_left/contact
ros2 topic echo /bumper_right/contact

# save the map you built
ros2 run nav2_map_server map_saver_cli -f ~/maps/map
```

## Quickstart — reproduce the regressions

Copy-paste. Needs Docker on a native **x86-64 Linux** box (not ARM / not an
M-series Mac, and not Docker Desktop on Windows — see the note at the bottom).
Nothing else to install: these packages ship **prebuilt** in the dev image.

### 1. Get the dev image (~10 min, mostly the download)

```bash
# ROS 2 Jazzy + Nav2 + Gazebo + oomwoo_one + these packages, ~9.5 GB
docker pull makerspet/oomwoo:jazzy-dev

# spin it up and get a bash prompt inside (same flow as the makerspet tutorials)
docker run -it --name oom makerspet/oomwoo:jazzy-dev
```

Everything below runs at the **container's bash prompt**. The packages are
already built and sourced — nothing to clone. (Editing them? Rebuild with
`colcon build --symlink-install --packages-select oomwoo_coverage
oomwoo_nav_localize oomwoo_sim_support oomwoo_clean` in `/ros_ws`.)

### 2. Kidnapped-robot test (~4 min)

```bash
bash /ros_ws/src/oomwoo-ros2-tools/deploy/run_reloc_regression.sh
```

Teleports the robot to 10 random spots and recovers each. Prints per-trial lines
and a summary; exits 0 on pass. Expect:

```
RELOC_SUMMARY passed=10/10 success_rate=1.00 target=0.90 ... suite_pass=True
```

### 3. Coverage test (~20 min)

```bash
bash /ros_ws/src/oomwoo-ros2-tools/deploy/run_coverage_regression.sh
```

Sweeps the room, then a gap-fill pass. Prints `COVERAGE_REPORT` lines and a
summary; exits 0 on pass.

### 4. Coverage on the stock living_room (~20 min)

```bash
bash /ros_ws/src/oomwoo-ros2-tools/deploy/run_coverage_livingroom.sh
```

Same harness on the cluttered stock living_room. The room is tight, so efficiency
lands below the open test_room's by design. Note this suite is **variable**
(~50–85 % across runs) and may not meet the 90 % gate the runner enforces — a
known open item on this furniture-dense world (a hard under-furniture pocket
where the robot intermittently wedges), not a regression failure.

### Watching it with the Gazebo GUI

Every simulation runs identically with or without the GUI — one switch:

```bash
ros2 launch oomwoo_sim_support coverage_regression.launch.py gui:=true
```

(Headless is the default; `gui:=true` needs a display, e.g. `docker run` with
X11 forwarding as in the makerspet simulation tutorial.)

### Other vacuum models

The launches follow the kaiaai convention: `kaia config robot.model <package>`
selects the robot description, or pass it explicitly:

```bash
ros2 launch oomwoo_sim_support coverage_regression.launch.py robot_model:=proscenic_m6pro
```

The regression scripts pin `oomwoo_one` by default so results are reproducible.

### Repeat runs / variance

```bash
RUNS=3 bash /ros_ws/src/oomwoo-ros2-tools/deploy/run_coverage_regression.sh
```

Runs the suite 3× and prints min/max/mean/stdev for each metric. Exit codes:
`0` all pass, `1` a run missed its targets, `2` measurement invalid (the meter
detected ground-truth pose teleports — the sim is unstable on that host).

### Notes

- **Host requirement:** the results assume a real x86-64 Linux host (or CI
  runner). Docker Desktop on Windows runs inside a WSL2 VM and ARM Macs emulate
  x86 — both destabilize Gazebo physics (the ground-truth pose can teleport). The
  regression detects that and reports "sim unstable" (exit 2) instead of garbage.
- **Speed:** on 4 vCPU the sim runs ~real-time. On fewer cores it's slower but the
  metrics are unaffected (they're measured in sim time).
- **Reports:** JSON lands at `/root/coverage_report.json` and
  `/root/reloc_report.json`; live logs at `/tmp/coverage_regression.log.1` etc.
- **Clean up:** `exit` the container, then `docker rm -f oom` on the host.

Milestone design notes and the measured M1/M2 baselines are archived under
**[docs/etc/](docs/etc/)**.

## Credits

The initial packages here were authored by **[Jayadev Rana](https://github.com/jayadevrana)**,
commissioned for OOMWOO's M1/M2 milestones — coverage cleaning, kidnapped-robot
localization, a headless Gazebo regression harness, and a Raspberry Pi runtime
baseline. Per-file copyright headers are preserved. Originally developed at
[jayadevrana/oomwoo-m1-ros2](https://github.com/jayadevrana/oomwoo-m1-ros2). Thank you,
Jayadev.

## Contributing

New ROS 2 vacuum packages and improvements are welcome. See the OOMWOO
[requests for contributions](https://github.com/makerspet/oomwoo#-requests-for-contributions)
and [CONTRIBUTING](https://github.com/makerspet/oomwoo/blob/main/docs/CONTRIBUTING.md).
Say hi in the [discussions](https://github.com/makerspet/oomwoo/discussions) or on
[Discord](https://discord.gg/3y2JKz5T25).

## License

[Apache License 2.0](LICENSE).
