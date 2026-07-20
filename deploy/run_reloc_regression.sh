#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Headless kidnapped-robot relocalization regression, CLI, CI-friendly.
# Launches the (light, nav-free) sim + AMCL + recovery + injector, runs N kidnap
# trials, and exits 0 iff >= 90% relocalize within 30 s and 2 m. No GUI required.
#
#   RUNS=3 ./run_reloc_regression.sh    # repeat the whole suite 3x, report spread
#
# Exit codes: 0 = all suites pass, 1 = a suite failed.
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /ros_ws/install/setup.bash
[ -f /overlay_ws/install/setup.bash ] && source /overlay_ws/install/setup.bash
[ -f "$HOME/oomwoo-dev/install/setup.bash" ] && source "$HOME/oomwoo-dev/install/setup.bash"
# extra CLI args (e.g. `./run_reloc_regression.sh gui:=true`) forward to launch
LAUNCH_ARGS="${LAUNCH_ARGS:-} $*"
# software GL is only forced headless; a gui:=true run keeps the host's GL
case " $LAUNCH_ARGS " in *" gui:=true"*|*" gui:=True"*) ;;
  *) export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe ;;
esac
# isolate DDS discovery so a co-running ROS graph can't interfere
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77} ROS_LOCALHOST_ONLY=1

RUNS=${RUNS:-1}
TRIALS=${TRIALS:-10}
LOG=${LOG:-/tmp/reloc_regression.log}
REPORT_DIR=$(mktemp -d /tmp/reloc_runs.XXXX)
WORST=0

for i in $(seq 1 "$RUNS"); do
  echo "[run] ($i/$RUNS) launching headless relocalization stack -> $LOG.$i"
  # shellcheck disable=SC2086
  ros2 launch oomwoo_sim_support relocalize_regression.launch.py $LAUNCH_ARGS \
    > "$LOG.$i" 2>&1 &
  LAUNCH_PID=$!
  trap 'kill -INT $LAUNCH_PID 2>/dev/null || true' EXIT

  set +e
  ros2 run oomwoo_sim_support reloc_regression_runner --ros-args \
    -p num_trials:="$TRIALS" -p use_sim_time:=true \
    -p report_path:="$REPORT_DIR/run$i.json"
  CODE=$?
  set -e
  [ "$CODE" -gt "$WORST" ] && WORST=$CODE

  echo "[run] ($i/$RUNS) report:"
  cat "$REPORT_DIR/run$i.json" 2>/dev/null || true
  cp -f "$REPORT_DIR/run$i.json" /root/reloc_report.json 2>/dev/null || true
  echo
  echo "[run] ($i/$RUNS) exit code: $CODE"

  kill -INT $LAUNCH_PID 2>/dev/null || true
  wait $LAUNCH_PID 2>/dev/null || true
  pkill -f "gz sim" 2>/dev/null || true
  sleep 5
done

if [ "$RUNS" -gt 1 ]; then
  echo "[run] ===== VARIANCE over $RUNS suites ====="
  python3 - "$REPORT_DIR" <<'PY'
import json, glob, statistics as st, sys
rs = [json.load(open(p)) for p in sorted(glob.glob(sys.argv[1] + '/run*.json'))]
for k in ('success_rate', 'mean_reloc_time_s'):
    # a 0-success suite reports mean_reloc_time_s as null — drop it, don't crash
    v = [r[k] for r in rs if r.get(k) is not None]
    if v:
        print(f"  {k:18}: min={min(v):.3f} max={max(v):.3f} mean={st.mean(v):.3f}"
              + (f" stdev={st.stdev(v):.3f}" if len(v) > 1 else ""))
print(f"  suite passes      : {sum(bool(r.get('suite_pass')) for r in rs)}/{len(rs)}")
PY
fi
echo "[run] overall exit: $WORST  (0 = PASS)"
exit $WORST
