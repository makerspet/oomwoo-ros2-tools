#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# Regression smoke-test for the OOMWOO jazzy-dev image + sim/cleaning stack.
# Fast, headless (no display), CI-friendly. Exits 0 iff every check passes.
#
# Run inside the image:
#   docker exec <container> bash /ros_ws/src/oomwoo-ros2-tools/deploy/smoke_test.sh
# or from a checkout:
#   bash deploy/smoke_test.sh
#
# What it guards against (regressions we actually hit):
#   * the kaiaai_* -> oomwoo_* package renames silently reverting
#   * the default robot model drifting off oomwoo_one
#   * oomwoo_bringup losing its oomwoo_gazebo map default
#   * headless Gazebo / the bumper contact sensors going silent
#   * the coverage-cleaning stack failing to make progress
# NB: no `set -u` -- ROS 2 setup.bash references unbound vars (AMENT_TRACE_SETUP_FILES).
set -o pipefail

source /opt/ros/jazzy/setup.bash
source /ros_ws/install/setup.bash 2>/dev/null || true
export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-91} ROS_LOCALHOST_ONLY=1

PASS=0; FAIL=0
ok(){ echo "  PASS  $1"; PASS=$((PASS+1)); }
no(){ echo "  FAIL  $1"; FAIL=$((FAIL+1)); }
kill_sim(){ kill -INT "${1:-0}" 2>/dev/null || true; sleep 3;
            pkill -9 -x "gz sim" 2>/dev/null || true; pkill -9 ruby 2>/dev/null || true; sleep 2; }

echo "== 1. package renames (oomwoo_* present, kaiaai_gazebo/kaiaai_bringup gone) =="
PKGS=$(ros2 pkg list 2>/dev/null)
for p in oomwoo_one oomwoo_gazebo oomwoo_bringup oomwoo_coverage oomwoo_nav_localize oomwoo_sim_support; do
  grep -qx "$p" <<<"$PKGS" && ok "$p present" || no "$p MISSING"
done
for p in kaiaai_gazebo kaiaai_bringup; do
  grep -qx "$p" <<<"$PKGS" && no "$p still present (rename reverted?)" || ok "$p absent"
done

echo "== 2. default robot model =="
M=$(python3 -c "from kaiaai import config; print(config.get_var('robot.model'))" 2>/dev/null || true)
[ "$M" = "oomwoo_one" ] && ok "robot.model = oomwoo_one" || no "robot.model = '${M:-?}' (expected oomwoo_one)"

echo "== 3. oomwoo_bringup navigation resolves the oomwoo_gazebo map =="
if timeout 45 ros2 launch oomwoo_bringup navigation.launch.py --show-args 2>&1 \
     | grep -q "oomwoo_gazebo/share/oomwoo_gazebo/map"; then
  ok "navigation.launch.py map default -> oomwoo_gazebo"
else no "navigation.launch.py map default not oomwoo_gazebo"; fi

echo "== 4. world.launch.py exposes the headless switch =="
if timeout 30 ros2 launch oomwoo_gazebo world.launch.py --show-args 2>&1 | grep -q "'headless'"; then
  ok "world.launch.py headless:= arg present"
else no "world.launch.py missing headless arg"; fi

echo "== 5. headless sim + bumper contact =="
ros2 launch oomwoo_sim_support sim_bringup.launch.py with_nav:=false gui:=false \
    robot_model:=oomwoo_one > /tmp/smoke_sim.log 2>&1 &
SIM=$!
UP=0; for i in $(seq 1 70); do
  ros2 topic list 2>/dev/null | grep -q /bumper_left/contact && { UP=1; break; }; sleep 1; done
if [ "$UP" = 1 ]; then
  timeout 6 ros2 topic echo --once /scan sensor_msgs/msg/LaserScan >/dev/null 2>&1 \
    && ok "/scan publishing (sensors up)" || no "/scan silent"
  ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.35}}' >/dev/null 2>&1 &
  DRV=$!; sleep 13
  N=$(timeout 6 ros2 topic echo /bumper_left/contact ros_gz_interfaces/msg/Contacts 2>/dev/null | grep -c collision1)
  kill "$DRV" 2>/dev/null || true
  [ "${N:-0}" -gt 0 ] && ok "bumper fired on wall contact (${N} msgs)" || no "bumper silent"
else no "headless sim did not come up (see /tmp/smoke_sim.log)"; fi
kill_sim "$SIM"

echo "== 6. coverage cleaning makes progress =="
ros2 launch oomwoo_sim_support coverage_regression.launch.py > /tmp/smoke_cov.log 2>&1 &
COV=$!
UP=0; for i in $(seq 1 80); do
  ros2 topic list 2>/dev/null | grep -q /coverage_meter/ratio && { UP=1; break; }; sleep 1; done
if [ "$UP" = 1 ]; then
  R1=$(timeout 5 ros2 topic echo --once /coverage_meter/ratio 2>/dev/null | awk '/data/{print $2; exit}')
  sleep 55
  R2=$(timeout 5 ros2 topic echo --once /coverage_meter/ratio 2>/dev/null | awk '/data/{print $2; exit}')
  if awk "BEGIN{exit !((${R2:-0}) > (${R1:-0}) && (${R2:-0}) > 0)}"; then
    ok "coverage rising (${R1:-0} -> ${R2:-0})"
  else no "coverage not rising (${R1:-0} -> ${R2:-0})"; fi
else no "coverage stack did not come up (see /tmp/smoke_cov.log)"; fi
kill_sim "$COV"

echo
echo "===== SMOKE TEST: ${PASS} passed, ${FAIL} failed ====="
[ "$FAIL" -eq 0 ]
