# Provenance of the banked Pi 4 baseline

Which sampler revision produced these files, and the one known impurity.

- **`idle.json`, `slam.json`, `nav.json`, `baseline_report.json`** are the
  clean re-measure on the client's Pi 4 (2 GB) banked at commit `7d709e7`
  ("fix baseline measurement integrity, re-measure clean on Pi 4") — the run
  that excluded the `ros2 bag play` stand-in and reaped nodes between phases.
  One leak remained in that sampler revision: its exclude regex missed the
  `measure_pi_baseline.sh` driver shell, so each phase's `n_proc` and totals
  still include one harness `bash` (~3.2 MB RSS / ~1.1 MB PSS) — the "~1 MB
  measurement-harness shell" footnote in the README. Commit `2503b49` widened
  the sampler's exclude to drop it, but these JSONs were not re-measured with
  the fixed sampler; a re-run with the committed tool will show one fewer
  process per phase and ~1 MB less PSS.
- **`system.txt`** was captured on the same board during the same run
  (revision decode, `free`, OS/ROS versions).

Regenerating is a one-command re-run on the board (writes to
`/tmp/pi_baseline`; copy the JSONs here to bank them):

```bash
BAG=$PWD/scan_bag bash measure_pi_baseline.sh
```
