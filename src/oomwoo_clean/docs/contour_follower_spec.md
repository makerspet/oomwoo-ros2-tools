# Contour follower — design spec (draft)

Status: draft for review. Implementation not started.

## 1. Purpose & scope

A reactive behavior that traces the boundary of a **LiDAR-visible** obstacle of
**any continuous shape** — concave *and* convex — at a fixed standoff, so the
vacuum cleans the edge strip all the way around it. It is the LiDAR
generalization of `wall_clean` (which only follows walls/concave corners *by
bumper contact*): proactive, arbitrary-shape, off the LiDAR.

It stays in the **reactive layer**: `/scan` → `/cmd_vel`, no global plan. Sibling
to `wall_clean`; publishes `cleaning_active` so `coverage_meter` scores it.

**In scope:** the *follow primitive* only.
**Out of scope (higher layers, later):** choosing which obstacle to clean next
and driving to it (a sequencer / `clean_to_goal` does that), free-area
boustrophedon fill, and get-unstuck.

## 2. Behavior / state machine

- **IDLE** — waiting for a start trigger.
- **ALIGN** — turn in place until the obstacle is abeam on the follow side (right)
  and the robot is roughly tangent to it. Entered after the higher layer has
  parked the robot near the obstacle.
- **FOLLOW** — the control loop: hold standoff and trace the contour (turn *in* on
  concave, arc *around* on convex).
- **CLOSE** — loop closure detected (returned to the start pose) → stop, report
  `~/done`.
- **LOST** — boundary lost and not re-acquired within the convex-arc recovery →
  stop and report (safety).

## 3. The control law (the crux)

Follow side = **right** (configurable). Convention: obstacle on the right, robot
circles it keeping it on the right. Frame: 0° = forward, −90° = right, CCW +.

### 3.1 Boundary extraction (per scan)
- Look at a **follow-side, forward-biased sector**, e.g. bearings β ∈ [−170°, +20°]
  (right + front, for look-ahead).
- **Nearest boundary point** P = the beam with the minimum range in that sector →
  `(d_min, β_min)`.
- Ignore beams beyond `max_follow_range` so "nearest boundary" is *this* obstacle,
  not a far background wall.

### 3.2 Two error terms
- **Standoff error** `e_d = d_min − standoff` (positive = too far from the obstacle).
- **Bearing error** `e_β = β_min − β_ref`, with `β_ref = −90°` (abeam) for a
  tangent path; a small forward lead may be added.

### 3.3 Command
- Forward speed `v = v_nominal`, eased toward `v_min` when `|e_d|` or `|e_β|` is
  large (a corner) — slow down to arc accurately.
- Angular `ω = −k_dist·e_d − k_bearing·e_β` (signs set so: too far → turn *toward*
  the obstacle; nearest point drifting *behind* abeam → turn toward it to
  re-acquire; nearest point *ahead of* abeam and closing (concave) → turn away).
  Gains and exact signs are tuned in sim.
- This one law covers **straight** (both errors ≈ 0), **concave** (P swings forward
  and closes → turn away, follow the inside corner), and the *onset* of **convex**
  (P swings behind abeam → turn toward, begin to arc).

### 3.4 Convex corner — the make-or-break
- **Failure mode:** at an outside corner the obstacle ends and `d_min` jumps to a
  far/background reading → a naive law reads "obstacle far" → drives straight off
  the corner.
- **Guard + recovery:** if `d_min` jumps by > `convex_jump_m` between frames, *or*
  no boundary within `max_follow_range` remains in the sector → enter a **bounded
  arc**: command a fixed curvature *toward* the follow side (turn right) at radius
  ≈ `convex_arc_radius`, sweeping around where the corner was until a near boundary
  reappears in the sector (then resume FOLLOW). Cap it: if the arc sweeps past
  `convex_arc_max_deg` (~200°) without re-acquiring, the obstacle really ended →
  **LOST**.
- This "lose the wall → curve toward it" is the classic convex behavior; the arc
  radius comes from the standoff so the robot rounds the corner at the right
  offset. **This is the behavior our bumper `wall_clean` cannot do, and the reason
  the follower exists.**

## 4. Standoff geometry

- `standoff` = the perpendicular LiDAR-to-boundary distance we hold (measured
  abeam, −90°). For a wall parallel to heading this equals the body-center
  perpendicular distance (LiDAR fore/aft position doesn't change it).
- Constraints:
  1. `standoff ≥ lidar_min_range` (0.1 m) so the boundary is sensable.
  2. **Body clearance:** the body is a disk of radius `base_diameter/2 = 0.1745 m`;
     its rightmost point is 0.1745 m off center, so `standoff > 0.1745 + gap`.
  3. The **side brush** (extends a few cm past the body) should reach the edge
     strip.
- **Starting value:** `standoff ≈ 0.20–0.22 m` → ~3–5 cm body-to-wall gap with the
  side brush reaching in. Tune in sim so one lap cleans the edge strip without the
  body scraping.
- The **forward LiDAR** (mounted +0.0745 m) doesn't change straight-wall standoff,
  but its abeam/forward reading is taken ahead of center → natural look-ahead on
  the followed side, so a convex corner enters the sector sooner. That head start
  is exactly what the mount move buys (measured in §9 test 5).

## 5. Loop closure / termination

- On entering FOLLOW, record the **start pose** `S = (x, y, θ)` from the
  `map→base_footprint` TF (the existing localization stack).
- Track traveled distance and **net heading change**.
- **CLOSE** when: traveled > `min_loop_dist` AND back within `close_radius` of `S`
  AND `|net heading change| ≈ 360°`. The heading-change gate distinguishes a real
  full loop from merely re-crossing `S`.
- Works for both a **free-standing obstacle** (chair → clockwise loop) and the
  **room boundary** (walls → CCW loop) — both are closed loops.
- On CLOSE, publish `~/done` (obstacle circumnavigated) for the sequencer.

## 6. Interfaces

**Subscribes**
- `/scan` `sensor_msgs/LaserScan` (SensorData QoS) — the boundary.
- TF `map→base_footprint` — pose, for loop closure.
- `bumper_left/contact`, `bumper_right/contact` `ros_gz_interfaces/Contacts` —
  safety: a bump during FOLLOW → back out (reuse `wall_clean`) then resume.

**Publishes**
- `cmd_vel` `geometry_msgs/Twist`.
- `cleaning_active` `std_msgs/Bool` (latched) — for `coverage_meter`.
- `~/state` `std_msgs/String` (IDLE/ALIGN/FOLLOW/CLOSE/LOST) — observability.
- `~/done` `std_msgs/Empty` (edge, on CLOSE) — for the sequencer.

**Control**
- `~/start` service/topic (optional `follow_side`) — begin ALIGN → FOLLOW. Called
  by the higher layer once the robot is parked near the obstacle.
- Live ROS params (retunable while running, like `wall_clean`).

## 7. Parameters (starting values, all live)

| param | start | note |
|---|---|---|
| `follow_side` | right | |
| `standoff_m` | 0.20 | perpendicular LiDAR-to-wall |
| `v_nominal` / `v_min` | 0.15 / 0.05 | m/s |
| `sector_min_deg` / `sector_max_deg` | -170 / 20 | follow-side + forward |
| `max_follow_range_m` | 1.0 | ignore background beyond |
| `bearing_ref_deg` | -90 | abeam (+ optional lead) |
| `k_dist`, `k_bearing` | tune | control gains |
| `convex_jump_m` | 0.30 | `d_min` jump that triggers the arc |
| `convex_arc_radius_m` | 0.30 | ≈ standoff + body offset |
| `convex_arc_max_deg` | 200 | give up → LOST |
| `close_radius_m` | 0.15 | loop-closure return radius |
| `min_loop_dist_m` | 0.5 | before closure can trigger |
| `heading_close_deg` | 300 | net-heading gate for a full loop |

## 8. Integration / handoff

- **Sequencer (later):** navigates to a seed near the next uncleaned obstacle
  (reuse `clean_to_goal` / Nav2), calls `~/start`; on `~/done`, picks the next;
  when all obstacles are circled, hands to the free-area boustrophedon planner.
- **Bump reflex:** a bumper hit on a LiDAR-invisible object during FOLLOW → back
  out (`wall_clean` logic) and resume FOLLOW on the bumped contour (v1); unify the
  two behaviors into one primitive later.
- **Coverage:** `cleaning_active` drives `coverage_meter`, as today.

## 9. Acceptance tests (in sim)

1. **Straight wall** — standoff held within tolerance, no oscillation.
2. **Concave inside corner** — turns in, no collision.
3. **Convex outside corner** (spawn a box via `spawn_obstacle.launch.py`) — arcs
   around, re-acquires, no fly-off. **Primary test.**
4. **Free-standing obstacle** (box / chair model) — full lap, loop closes, `~/done`
   fires once.
5. **LiDAR mount A/B** — run test 3 with `lidar_center_offset` = 0.0 vs 0.0745 and
   measure how much sooner the arc initiates / how cleanly the corner is rounded.
   Quantifies what the forward mount buys.

Metrics: min body-to-obstacle clearance (no scrape), standoff RMS on straights,
corner success rate.

## 10. Build phases

1. **FOLLOW loop** — straight + concave (nearest-point law), standoff holding.
2. **Convex** — arc guard/recovery + LOST.
3. **Loop closure** — start-pose tracking, `~/done`, and the ALIGN entry.
4. **Integration** — sequencer seam (`clean_to_goal`), bump handoff. *(later)*

## Open questions

- **Peninsula / divider obstacles** that merge into the room boundary (the follow
  would transition onto the room wall). Out of scope for v1; the sequencer must
  later recognize it.
- Exact **standoff** for edge-strip coverage with the side brush (tune in sim).
- **Goto-seed**: reuse Nav2 (`clean_to_goal`) or a simpler reactive drive?
- **Side-distance-sensor ablation**: the model already carries L/R single-ray
  sensors — after LiDAR-only works, fuse the side sensor for tighter close-in
  offset and measure the delta (per the earlier plan: baseline, then ablation).
