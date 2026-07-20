#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Add the M1 behaviours + baseline tooling on top of xbattlax's Pi runtime
# scaffold (ubuntu/install_oomwoo_runtime_jazzy.sh). Run that first, then this.
#
# It clones the M1 packages into the runtime workspace and builds ONLY the two
# behaviour packages (oomwoo_coverage, oomwoo_nav_localize) — oomwoo_sim_support
# is skipped on purpose: it pulls Gazebo/ros_gz, which the onboard runtime
# deliberately omits. Also installs the fixed simulated MCU serial helper and
# the RSS/PSS/CPU baseline tools next to the workspace.
set -eo pipefail   # no -u: ROS setup.bash refs unbound vars

WORKSPACE="${WORKSPACE:-$HOME/oomwoo_runtime_ws}"
PKG_REPO="${PKG_REPO:-https://github.com/jayadevrana/oomwoo-m1-ros2}"
PKG_BRANCH="${PKG_BRANCH:-main}"
SKIP_BUILD=0
[ "${1:-}" = "--skip-build" ] && SKIP_BUILD=1

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "[m1] cloning M1 packages into $WORKSPACE/src/oomwoo-m1"
mkdir -p "$WORKSPACE/src"
if [ -d "$WORKSPACE/src/oomwoo-m1/.git" ]; then
  git -C "$WORKSPACE/src/oomwoo-m1" fetch --depth 1 origin "$PKG_BRANCH"
  git -C "$WORKSPACE/src/oomwoo-m1" reset --hard "origin/$PKG_BRANCH"
else
  git clone --depth 1 -b "$PKG_BRANCH" "$PKG_REPO" "$WORKSPACE/src/oomwoo-m1"
fi

# fixed simulated MCU serial (2 bugs fixed vs the scaffold: startup EIO crash +
# self-echo feedback loop — see the header of oomwoo_sim_mcu_serial.py)
mkdir -p "$HOME/.local/bin"
install -m 0755 "$HERE/oomwoo_sim_mcu_serial.py" "$HOME/.local/bin/oomwoo-sim-mcu-serial"

# baseline tooling lives beside the workspace so `measure_pi_baseline.sh` is
# one command on the robot computer
mkdir -p "$WORKSPACE/pi_runtime"
install -m 0755 "$HERE/oomwoo_runtime.launch.py" "$WORKSPACE/pi_runtime/"
install -m 0755 "$HERE/measure_baseline.py"      "$WORKSPACE/pi_runtime/"
install -m 0755 "$HERE/measure_pi_baseline.sh"   "$WORKSPACE/pi_runtime/"
install -m 0755 "$HERE/filter_bag.py"            "$WORKSPACE/pi_runtime/"
# the baseline is unrunnable without these: the MCU serial helper (always-on
# in every phase), the Nav2/map config the launch consumes, and the scan bag
install -m 0755 "$HERE/oomwoo_sim_mcu_serial.py" "$WORKSPACE/pi_runtime/"
cp -r "$HERE/config"   "$WORKSPACE/pi_runtime/"
[ -d "$HERE/scan_bag" ] && cp -r "$HERE/scan_bag" "$WORKSPACE/pi_runtime/"

if [ "$SKIP_BUILD" -eq 0 ]; then
  echo "[m1] building behaviour packages (sim_support skipped — Gazebo-only)"
  . /opt/ros/jazzy/setup.bash
  cd "$WORKSPACE"
  # rosdep failures are FATAL here — an `|| true` on this line is exactly how
  # a bad dependency key ships silently. --skip-keys covers only the Gazebo
  # packages that intentionally don't exist on the robot.
  rosdep install --from-paths src/oomwoo-m1 --ignore-src -y \
    --skip-keys "ros_gz_sim ros_gz_bridge gz-sim8"
  colcon build --symlink-install \
    --packages-select oomwoo_coverage oomwoo_nav_localize
  rm -rf log/
fi

cat <<EOF

OOMWOO M1 runtime add-on installed.
  Behaviours : oomwoo_coverage, oomwoo_nav_localize (in $WORKSPACE)
  MCU serial : oomwoo-sim-mcu-serial  (~/.local/bin, fixed)
  Baseline   : $WORKSPACE/pi_runtime/measure_pi_baseline.sh

Measure the onboard baseline (needs a scan bag; see pi_runtime/README):
  source $WORKSPACE/install/setup.bash
  BAG=/path/to/scan_bag_clean bash $WORKSPACE/pi_runtime/measure_pi_baseline.sh
EOF
