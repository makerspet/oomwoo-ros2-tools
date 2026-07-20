# Localization & Kidnapped-Robot Relocalization — jayadevrana

Contribution to [`nav-localize`](../../): **localization on a known map and
kidnapped-robot recovery** for `oomwoo_one` in Gazebo, with a headless CLI
regression that verifies the acceptance metrics.

> The ROS 2 packages are **self-hosted**; this README links to them.

## Self-hosted packages

- **`oomwoo_nav_localize`** — kidnap detection + active relocalization →
  `https://github.com/jayadevrana/oomwoo-m1-ros2` (`/oomwoo_nav_localize`)
- **`oomwoo_sim_support`** — headless bringup, ground-truth, kidnap injector,
  regression runner → same repo.

## What it does

- Brings up the saved `test_room` map (the default regression world) + AMCL
  headless (a light stack — no Nav2 nav servers needed for the spin/explore
  recovery).
- **Global initial localization** at startup (AMCL, no pose given), tracks pose
  during motion, and exposes a confidence signal (covariance trace).
- **Kidnap detection**: enters recovery on a covariance collapse or an external
  pickup/kidnap signal.
- **Active relocalization**: requests AMCL global re-initialization
  (`/reinitialize_global_localization`), then spins to see 360° and **explores**
  (drive + `/scan` obstacle avoidance) so the filter can resolve *position*, not
  just heading, until the pose re-converges.
- **Clear success/failure**: on timeout it declares `LOCALIZATION_LOST` and hands
  off to the future dock-cycle find-the-dock fallback.

## Acceptance metrics

| Metric | Target | How measured |
|---|---|---|
| Relocalization time | ≤ 30 s | from kidnap to re-converged pose |
| Accuracy | ≤ 2 m | converged AMCL pose vs the known teleport target |
| Success rate | ≥ 90 % | over N randomized kidnap trials (default 10) |

Truth is the **known teleport target** the injector commands (sim odometry does
not jump on teleport), so scoring is independent of AMCL.

## Run (headless, CLI)

```bash
./deploy/run_reloc_regression.sh           # exit 0 == PASS; writes reloc_report.json
# or manually:
ros2 launch oomwoo_sim_support relocalize_regression.launch.py &
ros2 run  oomwoo_sim_support reloc_regression_runner --ros-args -p num_trials:=10
```

## Test results

Native x86-64 Linux, headless, straight from `run_reloc_regression.sh` (10 random
kidnaps):

```
success rate = 100% (10/10)   (target 90%)
time         = 6.0 s avg, 9.2 s worst   (target 30 s)
accuracy     = <= 0.12 m every trial    (target 2 m)
result: PASS
```

Recovery uses a one-shot global scan-match to seed AMCL, which is why it lands in
a few seconds and within ~0.1 m instead of drifting near the time limit.

## Notes / scope

- M1 scope: global localization, kidnap detection + recovery, and the headless
  regression. Map-resume (slam_toolbox serialized continuation) is noted as
  follow-on in the RFC.
- Interfaces follow
  [SOFTWARE_INTERFACES.md](../../../docs/SOFTWARE_INTERFACES.md).
- Requires a **native x86-64** host.

— Jayadev Rana · Apache-2.0
