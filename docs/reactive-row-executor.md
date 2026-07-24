# Reactive row executor — design note

*Status: design / scoping. No code yet — this is the shape to decide on.*

## Why

The coverage **planner** decides *what* to clean and in *what order* (cells →
serpentine passes → gap-fill). Today it also **executes** motion by firing one
Nav2 `NavigateToPose` goal per waypoint. Measured on the living_room sim, that
execution path is the wrong tool for the job:

- **~3 s of overhead per waypoint** — drive ~0.4 m, reach the goal tolerance,
  wait for `bt_navigator` to declare success, send the next goal. At ~190
  waypoints that is **~10 minutes for one clean even when nothing goes wrong**,
  most of it goal-handshake, not cleaning.
- **Costmap phantom** — RPP's forward collision check stopped the robot short of
  obstacles (fixed by disabling it, but the machinery is still fighting us).
- **Corner-cutting** — pure pursuit arcs through row-end U-turns.
- **Collision-averse recoveries** — Nav2 `spin`/`backup` refuse to move in a
  wedge (footprint in lethal cells), so they stall exactly when needed.

None of this is a bug to patch; it is Nav2 being a *point-to-point, costmap-aware*
navigator asked to do *coverage*, whose motion is really **drive-straight →
turn → drive-straight → (bump) edge-follow**. That motion wants a small reactive
controller, not a global planning stack.

## Principle (subsumption / hybrid)

Split *planning* from *executing*, and use each tool where it is strong:

| Concern | Owner |
|---|---|
| What to clean, pass order, gap-fill | **coverage planner** (done) |
| Driving a sweep pass (straight line + row-end turn) | **reactive row executor** (new) |
| Contact reflex (peel off / edge-follow) | **bumper escape** (built) — overrides the row drive while in contact |
| Longer inter-cell transit, routing around furniture, go-to-dock | **Nav2** `NavigateToPose` (kept) |
| Localization | **Nav2 AMCL** (kept) |

Coverage plans the open floor; the executor drives it; the bumper reflex handles
the last few centimetres; Nav2 is demoted to the few jobs it is actually good at.
This is the reactive-control layer the recovery-safety RFC anticipated.

## How it slots in — as a mode of the coverage planner (at first)

The `coverage_planner` node **already has everything the executor needs**: a
`cmd_vel` publisher (used by the escape), `bumper_left|right/contact`
subscriptions, the `amcl_pose` subscription, the 10 Hz `_tick` loop, and the
contact-aware peel-off. So the executor starts life as a **RowDriver mode inside
the existing node**, not a new process — minimal new infrastructure, and it
reuses the escape reflex directly. If it grows, it splits into its own
`reactive-control` node later (same plan the recovery-safety RFC sketches).

The plan the planner already produces is a list of `(x, y)` waypoints. We tag
each segment as one of:

- **`row`** — a straight run of collinear waypoints (a sweep pass, and the short
  intra-cell connector to the next pass). Driven reactively.
- **`transit`** — the hop between two cells (or robot → first cell), which may
  need to route around furniture. Dispatched to Nav2 as one `NavigateToPose`.

`_tick` looks at the current segment's tag and calls the row driver or the Nav2
dispatcher. Rows collapse ~190 discrete goals into ~a few dozen continuous
drives; transits stay ~one Nav2 goal per cell boundary (≈10), where Nav2's
overhead is negligible and its furniture-avoidance is worth having.

## The row-drive control law (the one genuinely new piece)

Given a segment `(p_start → p_end)` and the robot pose from localization:

1. **Follow the line.** `heading_err = wrap(atan2(p_end − pos) − yaw)`;
   `cross_track = signed perpendicular distance from the p_start→p_end line`.
   `cmd.angular.z = −k_h·heading_err − k_ct·cross_track`;
   `cmd.linear.x = v_cruise`, tapered down as `|heading_err|` grows or near p_end.
2. **Turn at the row end.** Within `row_end_tol` of `p_end`, stop linear and
   rotate in place to the next segment's heading (`angular.z = k·heading_err`)
   until aligned, then start the next segment. (Crisp 90° corners — the thing
   pure pursuit couldn't give us — for the price of a short in-place rotate,
   which is cheap because it happens per *pass*, not per waypoint.)
3. **Feedback.** Use `odom` for the high-rate, smooth heading/velocity loop and
   `amcl_pose` to correct odometry drift periodically (a slow vacuum tolerates
   10 Hz control fine). No costmap in the loop — the robot is *meant* to touch
   things; safety is the bumper.

A pass ends on: **reached `p_end`** (→ next segment), **bumper contact** (→
escape reflex, below), **no-progress** (→ skip the pass, same watchdog we just
added), or **off-map / lost localization** (→ stop, hand to safety).

## How it shares the bumper escape

The contact-aware peel-off already built (held-bumper → rotate away from the
pressed side, record a no-go pocket) **becomes the executor's contact reflex**,
unchanged in spirit:

- While driving a row, the executor watches `bumper_*/contact`. Sustained
  contact **overrides** the row drive (subsumption): run the peel-off open-loop
  on `cmd_vel`, then resume the row (or skip it if the pocket is now no-go).
- **Edge-follow (floor-care)** is the natural next behavior on the *same* signal:
  instead of only peeling off, hug the contour (back a hair → turn out → arc in →
  re-contact) to clean right up to the object. Shared contact plumbing means the
  executor, the wedge escape, and edge cleaning are one reflex layer, not three.

## Interfaces

- **Consumes:** the internal plan (already in-node); `amcl_pose` + `odom` + TF
  `map→base_link`; `bumper_left|right/contact`; `coverage_meter/ratio` (skip
  already-clean passes).
- **Produces:** `cmd_vel` (rows + escape); `~/plan` Path (done, for RViz);
  optionally `~/status` (current segment, mode: row/transit/escape) for
  observability. Keep the topic/behaviour contract aligned with
  `SOFTWARE_INTERFACES.md`.
- **New params:** `v_cruise`, `k_heading`, `k_crosstrack`, `row_end_tol`,
  `rotate_speed`, `min_transit_len` (how long a hop must be to go via Nav2 vs.
  drive reactively). All tunable, all with sane defaults.

## Phasing (no big-bang rewrite)

1. **Row driver behind a flag.** Add the RowDriver + segment tagging; `executor:=reactive`
   drives rows reactively and transits via Nav2, `executor:=nav2` keeps today's
   all-Nav2 path. A/B them: **time-to-coverage** should drop sharply (the ~3 s/wp
   overhead disappears), coverage and turns hold or improve.
2. **Fold in the reflex.** Route the bumper escape through the executor; confirm
   peel-off still frees wedges. Add edge-follow as an opt-in.
3. **Retire per-row Nav2.** Once the reactive path wins the A/B, rows never touch
   Nav2; Nav2 is transits + dock + localization only.

The `executor:=nav2` fallback stays as the regression baseline and a safety net.

## Risks / open questions

- **Open-loop straightness.** How well can it hold a line on `odom`+`amcl` alone,
  with no costmap? (Mitigation: the cross-track term + amcl drift correction; a
  bump just triggers the reflex — contact is acceptable.)
- **Dynamic obstacles mid-row** (a foot). No costmap means we rely on the bumper
  and, optionally, **Nav2 Collision Monitor** (`nav2_collision_monitor`) as a
  reactive slow/stop zone from the LiDAR — cheap and composes with this. (Ties
  into Deepak's dynamic-obstacle-yielding work — coordinate.)
- **Transit boundary.** Deciding which connectors are "rows" vs "transits" — a
  length + free-corridor test; err toward Nav2 when a hop crosses unknown space.
- **Localization dropout.** If `map→odom` goes stale mid-row, stop and re-localize
  rather than dead-reckon into a wall.

## Testing / acceptance

Headless regression, `executor:=reactive` vs `executor:=nav2` on living_room
(and a tall room, and a simple rectangle):

- **Time-to-coverage** (the headline win) — expect a large drop.
- **Coverage %** and **turns** — hold or improve.
- **Stuck-freedom** — no wedge lasts more than a couple of seconds (reflex +
  no-progress skip).
- **Regression gate** stays green with the Nav2 fallback so we never lose the
  known-good baseline.

## Related

- `recovery-safety` RFC — this *is* the reactive-control layer it anticipated;
  the wedge escape and edge-follow live here.
- `floor-care` — edge cleaning rides the same bumper reflex.
- `clean-and-map` (Deepak) — coordinate the executor boundary with his SLAM +
  coverage work and the dynamic-obstacle yielding.
- `health-monitor` — stack-liveness / MCU watchdog is orthogonal but shares the
  "reactive, contact-tolerant vacuum" worldview.
