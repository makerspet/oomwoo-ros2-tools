#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Measure the memory + CPU footprint of the running ROS 2 runtime graph.

Linux /proc based, no external deps. Samples every matching process's RSS and
PSS (PSS = proportional set size, the honest number when nodes share libs) plus
per-process CPU% over the window, and system MemAvailable. Run it once per phase
of xbattlax's measurement plan (idle / slam / nav) with a --label.

  python3 measure_baseline.py --label idle --duration 20 --out idle.json

PSS needs read access to /proc/<pid>/smaps_rollup, which you have for your own
processes (the ROS nodes you launched). Run as the same user that runs ROS.
"""
import argparse
import glob
import json
import os
import re
import time

DEFAULT_PATTERN = (
    r"(ros2|/opt/ros|component_container|_ros2_daemon|slam_toolbox|amcl|"
    r"controller_server|planner_server|bt_navigator|behavior_server|"
    r"lifecycle_manager|map_server|robot_state_publisher|coverage_planner|"
    r"kidnap_recovery|ekf_node|robot_localization|parameter_bridge|oomwoo|nav2|"
    r"waypoint_follower|velocity_smoother|collision_monitor)"
)
PAGE_KB = os.sysconf("SC_PAGE_SIZE") // 1024
CLK_TCK = os.sysconf("SC_CLK_TCK")


def cmdline(pid):
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            return f.read().replace(b"\0", b" ").decode("utf-8", "replace").strip()
    except OSError:
        return ""


def short(cl):
    toks = cl.split()
    for t in toks:
        if t.startswith("__node:="):
            return t.split(":=", 1)[1]
    for t in toks:
        if "/lib/" in t or t.endswith(".py"):
            return os.path.basename(t)
    return os.path.basename(toks[0]) if toks else cl[:24]


def match_pids(rx, ex=None):
    out = []
    for d in glob.glob("/proc/[0-9]*"):
        pid = int(os.path.basename(d))
        if pid == os.getpid():
            continue
        cl = cmdline(pid)
        # rx selects the ROS graph; ex drops test scaffolding (the bag player
        # stands in for the LiDAR/MCU driver and must NOT count against the
        # robot's onboard budget) and this sampler itself.
        if cl and rx.search(cl) and not (ex and ex.search(cl)):
            out.append((pid, cl))
    return out


def rss_pss_kb(pid):
    rss = pss = 0
    try:
        with open(f"/proc/{pid}/smaps_rollup") as f:
            for line in f:
                if line.startswith("Rss:"):
                    rss = int(line.split()[1])
                elif line.startswith("Pss:"):
                    pss = int(line.split()[1])
        return rss, pss
    except OSError:
        try:
            with open(f"/proc/{pid}/statm") as f:
                rss = int(f.read().split()[1]) * PAGE_KB
        except OSError:
            pass
        return rss, 0  # PSS unavailable (e.g. not our process)


def cpu_ticks(pid):
    try:
        with open(f"/proc/{pid}/stat") as f:
            p = f.read().rsplit(")", 1)[1].split()
        return int(p[11]) + int(p[12])  # utime + stime (fields 14,15, 0-idx after comm)
    except (OSError, IndexError):
        return 0


def mem_available_kb():
    with open("/proc/meminfo") as f:
        for line in f:
            if line.startswith("MemAvailable:"):
                return int(line.split()[1])
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="idle")
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--interval", type=float, default=2.0)
    ap.add_argument("--pattern", default=DEFAULT_PATTERN)
    ap.add_argument("--exclude",
                    default=r"(ros2 bag|rosbag2| bag play|measure_baseline|"
                            r"measure_pi_baseline|pi_baseline_all|rerun_baseline)",
                    help="drop test scaffolding (bag player, this sampler) from "
                         "the measured graph so totals are the robot's alone")
    ap.add_argument("--out", default="/tmp/baseline.json")
    a = ap.parse_args()
    rx = re.compile(a.pattern)
    ex = re.compile(a.exclude) if a.exclude else None

    base = {pid: cpu_ticks(pid) for pid, _ in match_pids(rx, ex)}
    w0 = time.time()
    samples = []
    end = w0 + a.duration
    while time.time() < end:
        time.sleep(a.interval)
        trss = tpss = 0
        procs = []
        for pid, cl in match_pids(rx, ex):
            rss, pss = rss_pss_kb(pid)
            trss += rss
            tpss += pss
            procs.append({"pid": pid, "name": short(cl),
                          "rss_mb": round(rss / 1024, 1),
                          "pss_mb": round(pss / 1024, 1)})
        samples.append({
            "t": round(time.time() - w0, 1),
            "n_proc": len(procs),
            "total_rss_mb": round(trss / 1024, 1),
            "total_pss_mb": round(tpss / 1024, 1),
            "mem_available_mb": round(mem_available_kb() / 1024, 1),
            "procs": sorted(procs, key=lambda p: -p["rss_mb"]),
        })
    w1 = time.time()
    cpu = {}
    for pid, cl in match_pids(rx, ex):
        dticks = cpu_ticks(pid) - base.get(pid, cpu_ticks(pid))
        cpu[short(cl)] = round(100.0 * dticks / CLK_TCK / (w1 - w0), 1)

    peak_rss = max(s["total_rss_mb"] for s in samples)
    peak_pss = max(s["total_pss_mb"] for s in samples)
    total_cpu = round(sum(cpu.values()), 1)
    report = {
        "label": a.label,
        "n_proc": samples[-1]["n_proc"],
        "peak_total_rss_mb": peak_rss,
        "peak_total_pss_mb": peak_pss,
        # CPU% is a MEAN over the sampling window (Σ utime+stime / wall-secs),
        # taken after the settle delay — i.e. steady-state load, not the
        # launch/costmap-activation transient. Report it as an average.
        "mean_cpu_pct_over_window": total_cpu,
        "cpu_is_window_mean": True,
        "window_s": round(w1 - w0, 1),
        "cpu_by_proc": dict(sorted(cpu.items(), key=lambda kv: -kv[1])),
        "samples": samples,
    }
    with open(a.out, "w") as f:
        json.dump(report, f, indent=2)
    print(f"[{a.label}] procs={report['n_proc']}  peak RSS={peak_rss} MB  "
          f"peak PSS={peak_pss} MB  mean CPU={total_cpu}%  "
          f"({report['window_s']}s) -> {a.out}")


if __name__ == "__main__":
    main()
