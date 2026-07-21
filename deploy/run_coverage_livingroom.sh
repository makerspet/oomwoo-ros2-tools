#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Headless coverage-cleaning regression on the STOCK living_room world, CLI.
# Same harness as run_coverage_regression.sh, pointed at the cluttered
# living_room + its world-aligned map, spawning at the clearest floor cell.
# Uses the PURE STOCK world and models — no overrides (stock mesh collisions
# work headless). robot_radius 0.18 = true inscribed 0.1745 rounded up (~5 mm), aligned with
# the meter and Nav2 footprint: gaps down to ~0.4 m (under-furniture leg gaps)
# stay sweepable. Contact is a design feature (bumper), not a failure. The
# cell-decomposition sweep reports the honest number for a tight room.
#
#   RUNS=3 ./run_coverage_livingroom.sh   # repeat and report the spread
set -eo pipefail

# resolve the installed share dir for world+map paths
source /opt/ros/jazzy/setup.bash
source /ros_ws/install/setup.bash
[ -f /overlay_ws/install/setup.bash ] && source /overlay_ws/install/setup.bash
[ -f "$HOME/oomwoo-dev/install/setup.bash" ] && source "$HOME/oomwoo-dev/install/setup.bash"
SHARE=$(ros2 pkg prefix oomwoo_sim_support)/share/oomwoo_sim_support

GZSHARE=$(ros2 pkg prefix oomwoo_gazebo)/share/oomwoo_gazebo
WORLD=${WORLD:-$GZSHARE/worlds/living_room.world}
MAP=${MAP:-$SHARE/maps/living_room.yaml}
X=${X:-0.32}; Y=${Y:-1.59}; YAW=${YAW:-0.0}

echo "[run] headless coverage on living_room"
echo "[run] world=$WORLD"
echo "[run] map=$MAP  spawn=($X,$Y)  robot_radius=0.18"
# preserve any incoming LAUNCH_ARGS and forward extra CLI args (e.g. gui:=true)
export LAUNCH_ARGS="world:=$WORLD map:=$MAP x_pose:=$X y_pose:=$Y yaw:=$YAW robot_radius:=0.18 ${LAUNCH_ARGS:-} $*"
export LOG=${LOG:-/tmp/coverage_livingroom.log}
exec bash "$(dirname "${BASH_SOURCE[0]}")/run_coverage_regression.sh"
