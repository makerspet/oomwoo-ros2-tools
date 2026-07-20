#!/usr/bin/env bash
# Helper (runs INSIDE the container) — kill any prior stack, then run a
# regression cleanly. Usage: _vm_run.sh {reloc|coverage} [trials]
KIND=${1:-reloc}
TRIALS=${2:-10}

for p in run_reloc run_coverage regression_runner "ros2 launch" "gz sim" \
         parameter_bridge amcl robot_state ground_truth kidnap map_server \
         controller_server planner_server bt_navigator behavior_server \
         lifecycle_manager coverage_planner coverage_meter; do
    pkill -9 -f "$p" 2>/dev/null
done
sleep 4

if [ "$KIND" = "coverage" ]; then
    rm -f /root/coverage_out.log /root/coverage_report.json
    /root/oomwoo-dev/deploy/run_coverage_regression.sh > /root/coverage_out.log 2>&1
    echo "DONE_EXIT=$?" >> /root/coverage_out.log
else
    rm -f /root/reloc_out.log /root/reloc_report.json
    TRIALS="$TRIALS" /root/oomwoo-dev/deploy/run_reloc_regression.sh > /root/reloc_out.log 2>&1
    echo "DONE_EXIT=$?" >> /root/reloc_out.log
fi
