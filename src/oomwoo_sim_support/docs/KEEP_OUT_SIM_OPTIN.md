# Simulation-only Nav2 virtual walls (opt-in)

This experimental Nav2 KeepoutFilter integration is OFF by default.
It is a sim-only experiment, not a motor / bumper / MCU safety controller.

The caller must create an absolute-path P5 binary PGM mask, with a
YAML metadata file matching the selected simulation map resolution,
origin, width and height exactly. Black 0 means forbidden; white 255
means free. No ambiguous grey, empty mask, or full-map mask is accepted.

Run ONLY in an isolated offline Gazebo sim:
  ros2 launch oomwoo_sim_support sim_bringup.launch.py \
    map:=/absolute/test_room.yaml keepout_mask:=/absolute/keepout.yaml

The overlay is derived from the chosen Nav2 params file at launch,
using two costmap filter plugins (KeepoutFilter, inflation) on BOTH
global and local costmaps. Stock Nav2 params are unchanged; the overlay
is removed at launch shutdown. Dedicated mask and type-0 info servers
are lifecycle managed. Omit keepout_mask to retain stock behavior.

## Reproducible validation (2026-10-29)

At this contribution's exact commit, from the repository root:

```bash
PYTHONPATH=src/oomwoo_sim_support python3 -m unittest discover \\
  -s src/oomwoo_sim_support/test -p 'test_keepout*.py' -v
PYTHONPATH=src/oomwoo_sim_support python3 -O -m unittest discover \\
  -s src/oomwoo_sim_support/test -p 'test_keepout*.py' -v
```

Host Python (without ROS installed): 10/10 config tests PASS, two launch
checks SKIP (in both ordinary and -O). Inside the existing offline
`makerspet/oomwoo:jazzy-dev` Podman image with ROS2 Jazzy sourced,
`--network none --rm --cpus 2 --memory 1g` and read-only source mount:
**12/12 PASS** (10 config + 2 actual ROS2 launch-context tests),
plus `generate_launch_description()` PASS (16 entities). A separate
real official test_room map (140x140) overlay check PASS, verifying that
ONLY the new filter blocks differ from stock Nav2 configuration.
No ROS2 actions, DDS graph, Gazebo world or physical hardware were started.

Specific follow-up before deployment: launch a fresh isolated Gazebo
instance and prove both local/odom and global/map costmap observations
change to lethal at the forbidden cells with frame-correct TF, lifecycle
ACTIVE and no unexpected motion. Repeat with a forbidden Nav2 destination;
reject a partial successful route that stops short. Then validate that
reactive /cmd_vel coverage respects an independent keepout admission gate.

LIMITS: current validation is static/synthetic until a fresh Gazebo
trial actually verifies both costmaps, TF conversion, lifecycle, and
absence of forbidden entries. A prior Valdi320 private Jazzy/Gazebo
KeepoutFilter proof uses a different integration; it does not establish
this launch working. Nav2 can report a successful partial plan short
of a forbidden destination, so a separate goal-admission check is
mandatory. Reactive room coverage via cmd_vel bypasses Nav2 filters:
it needs its OWN no-go guard and exclusive, watchdog-protected motor
command source. Never use on a physical robot before safety review.
