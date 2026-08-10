# Bump-out wall following — how it works

*Status: implemented.* Node: [`oomwoo_clean/wall_clean_node.py`](../src/oomwoo_clean/oomwoo_clean/wall_clean_node.py),
launch: `ros2 launch oomwoo_clean wall_clean_bump_out.launch.py`
(`wall_clean.launch.py` is reserved for the fully-featured wall cleaning and
currently forwards here).

The old, pre-sensor way a vacuum cleans along an edge — before wall/ToF sensors
or LiDAR were used for this. It keeps the wall on the **right** and traces the
room **counterclockwise** using nothing but the bumpers: `bumper_*/contact →
cmd_vel`. No LiDAR, no map, no localization. It's the reactive baseline that the
sensor-based [`wall_follow`](#see-also) (hold a set distance to the wall) will
supersede.

You **aim the robot at the wall with teleop first**, then launch this. It
publishes a latched `cleaning_active` so the coverage meter scores the run. Stop
with Ctrl-C (there's no loop-closure yet).

## State machine

A 20 Hz loop cycles three states:

| State | Motion | Leaves when |
|---|---|---|
| **CRUISE** | forward `v_cruise` + gentle **right** arc `-arc_omega` (drifting into the wall) | a bumper fires → BACKOFF |
| **BACKOFF** | **backs OUT along the arc it drove in on** (see below), for `backoff_s` | timer elapses → TURN |
| **TURN** | rotate **left** in place at `turn_speed`, by an angle that depends on which bumper hit | angle reached → CRUISE |

The **turn angle** depends on the contact, which is what rounds corners:

- **right only** → small (`turn_right_deg`, 20°) — still hugging this wall, just peel off.
- **left only** → large (`turn_left_deg`, 90°) — a corner; swing left onto the next wall.
- **both** → medium (`turn_both_deg`, 60°) — head-on.

**First leg is straight.** Since you teleop-aim the robot at the wall and launch,
the *first* cruise leg drives dead straight (no arc) so you don't have to
pre-aim off-axis to cancel a curve. The arc engages only after the first bump,
once it's actually following the wall.

## Backing out along the entry arc

CRUISE curves into the wall on an arc (radius `v_cruise/arc_omega`). On contact,
BACKOFF **retraces that arc in reverse** rather than reversing in a straight
line. **Why:** retracing keeps the robot on the strip of floor it just drove
through, so pulling out of a contact is far less likely to wedge it somewhere
new — a straight back-up would swing off the entry path and can drive a corner of
the robot into a nearby obstacle.

The kinematics are simple: reversing a differential-drive path *exactly* means
negating **both** the linear and the angular velocity. So the back-out command is

```
linear  = -v_back
angular = -entry_arc · (v_back / v_cruise)
```

where `entry_arc` is the angular rate of the cruise leg that just bumped
(captured at contact; `0` for the straight first leg → a straight back-up). The
`v_back/v_cruise` factor keeps the **same path curvature** at the (usually
different) backoff speed, so the robot traces the *same arc*, just outward. It
still stops after the same back-out distance as before (`v_back · backoff_s` of
path length) — only the shape changed, from a chord to the original arc.

## Parameters

All motion values are ROS 2 parameters, **kaia-tunable and live**: the launch
seeds each from `kaia set clean.<name>` for the active robot (a `:=` launch arg
still wins), and `kaia set clean.<name> …` (or `ros2 param set /wall_clean …`)
retunes the *running* robot with no relaunch — see the
[kaia CLI live push](https://github.com/kaiaai/kaiaai/blob/jazzy/docs/cli.md#live-push-to-a-running-node).

| `clean.<name>` | Default | Meaning |
|---|---|---|
| `v_cruise` | 0.15 | forward cleaning speed (m/s) |
| `arc_omega` | 0.1 | gentle right-arc rate while cruising (rad/s) |
| `v_back` | 0.10 | back-out reverse speed (m/s) |
| `backoff_s` | 0.5 | back-out duration (s) → distance `v_back·backoff_s` |
| `turn_speed` | 0.7 | angular speed while turning left (rad/s) |
| `turn_right_deg` | 20 | right-bumper turn — small peel-off |
| `turn_left_deg` | 90 | left-bumper turn — round a corner |
| `turn_both_deg` | 60 | both-bumpers turn — head-on |

Two more node params (not in the `clean.*` set): `bumper_fresh_sec` (0.3 s — how
recently a contact counts as "pressed"; live) and `control_hz` (20 — loop rate;
set at launch).

```bash
# 1. teleop-aim the robot at the wall, then:
ros2 launch oomwoo_clean wall_clean_bump_out.launch.py use_sim_time:=true
# 2. tune live while it runs (persists for next launch too):
kaia set clean.arc_omega 0.15
kaia set clean.turn_left_deg 80
```

## Interfaces

- **Subscribes:** `bumper_left/contact`, `bumper_right/contact`
  (`ros_gz_interfaces/Contacts`; non-empty `contacts` = pressed).
- **Publishes:** `cmd_vel` (`geometry_msgs/Twist`); `cleaning_active`
  (`std_msgs/Bool`, latched `True`) for the coverage meter.

## See also

- **`wall_follow`** (planned) — the sensor-based successor: hold `min(range_right)`
  at a setpoint using the side distance sensors, instead of bump-and-peel.
- [`reactive-row-executor.md`](reactive-row-executor.md) — the bumper reflex here
  is the same contact-tolerant idea the coverage executor reuses.
- [floor-care RFC](https://github.com/makerspet/oomwoo/tree/main/contributions/floor-care)
  — edge cleaning builds on this contact behavior.
