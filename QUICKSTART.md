# Quickstart — reproduce the results

Copy-paste. Needs Docker on a **native x86-64 Linux** box (not ARM / not an
M-series Mac, and not Docker Desktop on Windows — see the note at the bottom).
Nothing else to install.

## 1. One-time setup (~10 min, mostly the image download)

On the host:

```bash
# the upstream dev image (ROS 2 Jazzy + Nav2 + Gazebo + oomwoo_one), ~9.5 GB
docker pull makerspet/oomwoo:jazzy-dev

# spin it up and get a bash prompt inside (same flow as the makerspet tutorials)
docker run -it --name oom makerspet/oomwoo:jazzy-dev
```

Everything below runs at the **container's bash prompt**. Get the packages into
the stock workspace and build them (stock packages stay as they are):

```bash
git clone https://github.com/jayadevrana/oomwoo-m1-ros2 /ros_ws/src/oomwoo-m1
cd /ros_ws
colcon build --symlink-install \
  --packages-select oomwoo_coverage oomwoo_nav_localize oomwoo_sim_support
source /ros_ws/install/setup.bash
```

## 2. Kidnapped-robot test (~4 min)

```bash
bash /ros_ws/src/oomwoo-m1/deploy/run_reloc_regression.sh
```

Teleports the robot to 10 random spots and recovers each. Prints per-trial
lines and a summary; exits 0 on pass. Expect:

```
RELOC_SUMMARY passed=10/10 success_rate=1.00 target=0.90 ... suite_pass=True
```

## 3. Coverage test (~20 min)

```bash
bash /ros_ws/src/oomwoo-m1/deploy/run_coverage_regression.sh
```

Sweeps the room, then a gap-fill pass. Prints `COVERAGE_REPORT` lines and a
summary; exits 0 on pass.

## 4. Coverage on the stock living_room (~20 min)

```bash
bash /ros_ws/src/oomwoo-m1/deploy/run_coverage_livingroom.sh
```

Same harness on the cluttered stock living_room, used exactly as upstream ships
it — the robot drives under the marble table and cleans between its legs (stock
mesh collisions work headless as-is). The room is tight, so efficiency lands
below the open test_room's by design. Note this suite currently **exits 1**: its
coverage is variable, ~50–85 % across runs, and does not meet the 90 % gate the
runner always enforces — a known open item on this furniture-dense world (a hard
under-furniture pocket where the robot intermittently wedges), not a regression
failure.

## Watching it with the Gazebo GUI

Every simulation runs identically with or without the GUI — one switch:

```bash
ros2 launch oomwoo_sim_support coverage_regression.launch.py gui:=true
```

(Headless is the default; `gui:=true` needs a display, e.g. `docker run` with
X11 forwarding as in the makerspet simulation tutorial.)

## Other vacuum models

The launches follow the kaiaai convention: `kaia config robot.model <package>`
selects the robot description, or pass it explicitly:

```bash
ros2 launch oomwoo_sim_support coverage_regression.launch.py robot_model:=proscenic_m6pro
```

The regression scripts pin `oomwoo_one` by default so results are reproducible.

## Repeat runs / variance

```bash
RUNS=3 bash /ros_ws/src/oomwoo-m1/deploy/run_coverage_regression.sh
```

Runs the suite 3x and prints min/max/mean/stdev for each metric. Exit codes:
`0` all pass, `1` a run missed its targets, `2` measurement invalid (the meter
detected ground-truth pose teleports — the sim is unstable on that host).

## Notes

- **Host requirement:** the results assume a real x86-64 Linux host (or CI
  runner). Docker Desktop on Windows runs inside a WSL2 VM and ARM Macs
  emulate x86 — both destabilize Gazebo physics (the ground-truth pose can
  teleport). The regression now detects that and reports "sim unstable"
  (exit 2) instead of garbage numbers.
- **Speed:** on 4 vCPU the sim runs ~real-time. On fewer cores it's slower but
  the metrics are unaffected (they're measured in sim time).
- **Reports:** JSON lands at `/root/coverage_report.json` and
  `/root/reloc_report.json`; live logs at `/tmp/coverage_regression.log.1` etc.
- **Clean up:** `exit` the container, then `docker rm -f oom` on the host.
