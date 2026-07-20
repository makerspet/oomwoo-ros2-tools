#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Measure the OOMWOO onboard runtime's RSS/PSS/CPU baseline, per xbattlax's
# pi4_4gb_runtime_plan.md. Runs three phases and records each with
# measure_baseline.py (a /proc sampler, no external deps):
#
#   idle : robot_state_publisher only            -> the floor
#   slam : + slam_toolbox, driven by a 5 Hz bag  -> mapping cost
#   nav  : + AMCL + Nav2 + M1 behaviours on a map -> navigation cost
#
# No robot is attached; a recorded scan+odom+tf bag (BAG) replays with --clock
# so SLAM/Nav2 run at the real 5 Hz LiDAR rate. Writes one JSON per phase plus a
# combined baseline_report.json. Run ON the target (Pi 4/5); the numbers are the
# deliverable. Also works on any Linux box for a pipeline dry-run (CPU% differs;
# RSS/PSS is representative).
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
# source whichever overlay workspace holds the robot description + M1 behaviours
[ -f "$HOME/oomwoo_runtime_ws/install/setup.bash" ] && source "$HOME/oomwoo_runtime_ws/install/setup.bash"  # Pi runtime
[ -f /ros_ws/install/setup.bash ] && source /ros_ws/install/setup.bash                                      # makerspet image
[ -f "$HOME/oomwoo-dev/install/setup.bash" ] && source "$HOME/oomwoo-dev/install/setup.bash"                # dev box
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-88} ROS_LOCALHOST_ONLY=1

LAUNCH="$HERE/oomwoo_runtime.launch.py"
MEASURE="$HERE/measure_baseline.py"
SERIAL="$HERE/oomwoo_sim_mcu_serial.py"
BAG=${BAG:-$HERE/scan_bag}   # the clean 5 Hz bag bundled next to this script
OUT=${OUT:-/tmp/pi_baseline}
SETTLE=${SETTLE:-18}
WINDOW=${WINDOW:-25}
mkdir -p "$OUT"

# Fail LOUDLY on missing prerequisites instead of measuring a half-dead graph:
# a silently-absent MCU serial or config yields numbers that look plausible
# but measure the wrong thing.
for f in "$LAUNCH" "$MEASURE" "$SERIAL" "$HERE/config/nav2_params.yaml" \
         "$HERE/config/test_room.yaml"; do
  [ -e "$f" ] || { echo "FATAL: missing prerequisite: $f" >&2; exit 3; }
done
[ -d "$BAG" ] || { echo "FATAL: scan bag not found at $BAG (set BAG=)" >&2; exit 3; }

# the simulated MCU serial link runs across every phase (it's always-on onboard)
python3 "$SERIAL" --link /tmp/oomwoo-mcu-serial >/tmp/mcu.log 2>&1 &
MCU=$!
trap 'kill $MCU 2>/dev/null; pkill -f oomwoo_runtime 2>/dev/null; pkill -f "bag play" 2>/dev/null || true' EXIT

# The always-on MCU serial must survive every phase; NODES is everything the
# per-phase launch spawns, which must be fully reaped before the next phase
# measures (a bare `kill -INT` on the ros2-launch parent leaves children like
# slam_toolbox alive, which then leak into the next phase's totals).
# NB: the first alternative is anchored to the LAUNCH FILE, not the bare
# string 'oomwoo_runtime' — the documented install path is
# ~/oomwoo_runtime_ws/..., so an unanchored match would hit this script's own
# shell (pkill -f matches whole cmdlines incl. paths; on Linux pgrep/pkill
# exclude only themselves, not ancestors) and SIGKILL the harness mid-run.
NODES='oomwoo_runtime\.launch|ros2 launch|slam_toolbox|amcl|controller_server|planner_server|bt_navigator|behavior_server|lifecycle_manager|map_server|robot_state_publisher|coverage_planner|kidnap_recovery|waypoint_follower|velocity_smoother|collision_monitor|component_container|ros2 bag|rosbag2'

reap() {
  # SIGINT the graph + bag, wait for a clean exit, then SIGKILL any straggler.
  pkill -INT -f "$NODES" 2>/dev/null || true
  for _ in $(seq 1 15); do
    pgrep -f "$NODES" >/dev/null 2>&1 || break
    sleep 1
  done
  pkill -KILL -f "$NODES" 2>/dev/null || true
  sleep 2
  # `|| true` is load-bearing: under `set -eo pipefail`, pgrep exiting 1 (the
  # CLEAN case — nothing left to kill) makes the command-substitution fail and
  # would abort the whole run. reap is called bare, so it must return 0.
  local left
  left=$(pgrep -f "$NODES" 2>/dev/null | wc -l | tr -d ' ') || true
  [ "${left:-0}" = 0 ] || echo "  [reap] warning: $left graph procs still alive"
  return 0
}

run_phase() {
  local mode="$1"
  echo "==== phase: $mode ===="
  reap                        # guarantee a clean graph before we start
  local BP=""
  if [ "$mode" != idle ]; then
    # bag LEADS the graph: on a real robot the odom TF + /clock are always
    # flowing, so SLAM/Nav2 must see them before activating or the costmap
    # aborts on a missing base_link->odom transform. Give it a head start.
    # The bag player is EXCLUDED from the measured totals (measure_baseline.py
    # --exclude) — it stands in for the sensor driver, not the robot's budget.
    ros2 bag play "$BAG" --clock 100 > "$OUT/$mode.bag.log" 2>&1 &
    BP=$!
    sleep 8
  fi
  ros2 launch "$LAUNCH" mode:="$mode" use_sim_time:=true > "$OUT/$mode.launch.log" 2>&1 &
  sleep "$SETTLE"
  python3 "$MEASURE" --label "$mode" --duration "$WINDOW" --interval 3 \
    --out "$OUT/$mode.json"
  reap                        # tear the whole graph down before the next phase
}

run_phase idle
run_phase slam
run_phase nav
reap

echo "==== combined baseline ===="
python3 - "$OUT" <<'PY'
import json, glob, os, sys
d = sys.argv[1]
rows = []
for m in ('idle', 'slam', 'nav'):
    p = os.path.join(d, m + '.json')
    if os.path.exists(p):
        r = json.load(open(p))
        rows.append({'phase': m, 'n_proc': r['n_proc'],
                     'rss_mb': r['peak_total_rss_mb'],
                     'pss_mb': r['peak_total_pss_mb'],
                     'mean_cpu_pct': r.get('mean_cpu_pct_over_window',
                                           r.get('total_cpu_pct'))})
print(f"{'phase':6} {'procs':>5} {'RSS_MB':>8} {'PSS_MB':>8} {'meanCPU%':>9}")
for r in rows:
    print(f"{r['phase']:6} {r['n_proc']:5d} {r['rss_mb']:8.1f} "
          f"{r['pss_mb']:8.1f} {r['mean_cpu_pct']:9.1f}")
json.dump({'phases': rows}, open(os.path.join(d, 'baseline_report.json'), 'w'),
          indent=2)
print('\\nwrote', os.path.join(d, 'baseline_report.json'))
PY
