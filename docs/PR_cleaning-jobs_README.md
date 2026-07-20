# Coverage Cleaning — jayadevrana

Contribution to [`cleaning-jobs`](../../): **regular / auto whole-map coverage
cleaning** for `oomwoo_one` in Gazebo, driven by Nav2, with a headless CLI
regression that verifies the acceptance metrics.

> Per project convention, the ROS 2 packages are **self-hosted**; this README
> links to them rather than vendoring the code here.

## Self-hosted packages

- **`oomwoo_coverage`** — boustrophedon coverage planner →
  `https://github.com/jayadevrana/oomwoo-m1-ros2` (`/oomwoo_coverage`)
- **`oomwoo_sim_support`** — headless bringup, ground-truth coverage meter,
  regression runner → same repo (`/oomwoo_sim_support`)
- Docker image (fork of `oomwoo-install`): build instructions in the repo's
  `deploy/Dockerfile`.

## What it does

- Loads the saved `test_room` map (the primary regression world), brings up
  Nav2 + AMCL headless. A second script runs the same harness on the stock
  `living_room` (`deploy/run_coverage_livingroom.sh`).
- Plans a **back-and-forth (boustrophedon) sweep** restricted to the robot's
  *reachable* free space (flood-filled from the robot pose, so no waypoint is
  ever stranded behind a wall), spaced by the cleaning swath with configurable
  overlap, and executes it via Nav2 `NavigateThroughPoses`.
- Measures coverage from the robot's **ground-truth** pose (not the planner's
  belief): swept-area / reachable-area, plus a path-efficiency ratio.

## Acceptance metrics

| Metric | Target | How measured |
|---|---|---|
| Coverage | ≥ 90 % | reachable free cells swept by the cleaning disk along the true path |
| Efficiency | ≥ 80 % | ideal gap-free sweep length / actual path length |

## Run (headless, CLI)

```bash
# inside the built image / overlay workspace
./deploy/run_coverage_regression.sh        # exit 0 == PASS; writes coverage_report.json
# or manually:
ros2 launch oomwoo_sim_support coverage_regression.launch.py &
ros2 run  oomwoo_sim_support coverage_regression_runner
```

## Test results

Native x86-64 Linux, headless, straight from `run_coverage_regression.sh`
(uncapped — the sweep runs to completion, `end_reason=sweep_complete`):

```
coverage   = 97.0%  at sweep_complete              (target 90%)
efficiency = 87.8%  at the 90% crossing (806 s)    (target 80%)
             68.5%  over the whole sweep (reported, not gated)
result: PASS
```

The sweep gets most of the way on its own; the gap-fill passes over the leftover
furniture-shadow pockets take it to 97.0%, where it genuinely ends — there is
no stop-at-target cap, and efficiency is judged at the moment coverage first
crosses 90%. On the stock `living_room` (`run_coverage_livingroom.sh`) the same
harness now runs at true-geometry clearance (planner robot_radius 0.18,
inflation 0.10) with cell decomposition + wedge recovery: coverage is variable
~50–85% across runs and does NOT meet the 90% gate — that suite fails by design
when it misses (a known open item, not presented as a pass). The wall/furniture
edge strip is left to the floor-care module per the RFC.

## Notes / scope

- M1 scope: whole-map regular coverage + its headless regression. Spot mode,
  room segmentation, no-go zones, and job pause/resume are follow-on work in the
  `cleaning-jobs` RFC.
- Interfaces follow
  [SOFTWARE_INTERFACES.md](../../../docs/SOFTWARE_INTERFACES.md).
- Requires a **native x86-64** host (ARM emulation has an unstable sim clock).

— Jayadev Rana · Apache-2.0
