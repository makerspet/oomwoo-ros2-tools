<div align="center">

# OOMWOO ROS 2 Tools

*Open-source robot vacuum you build yourself.*

ROS 2 Jazzy · Nav2 · Coverage cleaning · Kidnapped-robot localization · Gazebo · Headless CI

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
  full-room sweep through Nav2, with gap-fill and wedge recovery.
- **`oomwoo_nav_localize`** — kidnapped-robot relocalization (global correlative
  scan-match + AMCL) so the robot can recover its pose from any start position on a
  known map.
- **`oomwoo_sim_support`** — everything needed to run *and measure* the above headless
  in Gazebo: sim bring-up, a ground-truth pose publisher, the coverage meter, the
  kidnap injector, and the CLI regression runners.

See **[QUICKSTART.md](QUICKSTART.md)** to build and run the headless regressions on the
`makerspet/oomwoo:jazzy-dev` image. Milestone design notes and the measured M1/M2
baselines are archived under **[docs/etc/](docs/etc/)**.

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
