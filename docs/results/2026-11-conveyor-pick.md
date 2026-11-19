# Diff-IK on the conveyor pick: results

The `viam-mujoco-sim` example `examples/conveyor_pick.py` (at e87c989) picks parcels off a moving belt with a UR-style
arm and an ePick suction cup. Its `teleop` mode streams a goal over the parcel at 10 Hz through the motion service's
`teleop_*` DoCommands. These runs point that mode at diff-ik's `servo` service instead of `builtin` and compare the two.

## Setup

- **Cell:** `dev/conveyor-mujoco.json`, run locally on `localhost:8085`. It's the example's cell plus diff-ik from
  `bin/diff-ik`. The service's `teleop` block holds the session settings:
  - `solver: qp`, `timeout_ms: 1000`;
  - limits: 180°/s and 360°/s² per joint, 1500 mm/s, 180°/s;
  - the belt rails as `world_state` obstacles (builtin teleop doesn't use them);
  - `gain: 8`, `collision: {d_act_mm: 40}`.
- **Script:** `dev/conveyor_pick.patch`, applied to the example. It adds:
  - `TELEOP_SERVICE` to choose the service, and an insecure local connection when the key is `-`;
  - `--ramp-accel`, `--hover-mm`, `--descend-mm-s`, `--lock-mm` and `--lock-ticks`;
  - diff-ik's `teleop_status` in the per-tick CSV;
  - a fix for a `StopIteration` crash when the parcel leaves the belt mid-run.
- **Same parcels:** the spawner gives each run a different parcel sequence, and a tall parcel can turn a miss into a
  pick. Every set of three below restarted the cell first, so both sides of a comparison got the same parcels. The
  early single runs didn't, and one 0.5 m/s pick from them turned out to be luck.
- **Pass:** the cup grips and lifts the parcel about 150 mm. "Error" is the cup's offset from the parcel's top at the
  grab, in mm (x across the belt, y along it, z height).

## Result

With the final settings, diff-ik picks 3 of 3 at every belt speed from 0.1 to 0.7 m/s. Builtin teleop, as the example
ships, picks at 0.1 and 0.3 m/s and misses from 0.5 m/s.

| Belt | Builtin teleop (0.4 s lookahead, single runs) | diff-ik, final settings |
|---|---|---|
| 0.1 m/s | 1 of 1, gripped at 1.7 s, error (0.9, 2.9, 2.8) | 3 of 3, gripped at 1.0–1.3 s, error ≤ 3 mm |
| 0.3 m/s | 1 of 1, gripped at 2.0 s, error (4.8, 4.9, 2.2) | 3 of 3, gripped at 1.2–1.4 s, error ≤ 5 mm |
| 0.5 m/s | 0 of 1 (locked on, never gripped) | 3 of 3, gripped at 1.3–1.4 s, error ≤ 9 mm |
| 0.7 m/s | 0 of 1 | 3 of 3, gripped at 1.4–1.5 s, x error 14–15 mm |
| 1.0 m/s | 0 of 1 | 2 of 12 across four variants |

**Final settings:** diff-ik as above; script with 0.2 s lookahead, a 2 m/s² ramp-up, 50 mm hover, 400 mm/s descent, and
lock-on within 20 mm along the belt and 10 mm in height for one tick.

With the ramp-up, the clock starts while the parcel is still upstream, so diff-ik's grip times are if anything
conservative next to builtin's.

## What each change did

1. **Gain** (single runs, cell not restarted). At the default 2/s the error closes with a 0.5 s time constant, too slow
   for a parcel at 0.3 m/s: it locked on at 2.0 s and never gripped. The new `servo_start` option `gain` (v0.3.0) fixed
   that. At 0.5 m/s, gain 8 gripped and gain 12 didn't: targets arrive only every 100 ms, so a stiffer loop
   overshoots between them. The default stays at 2/s; the gain is a per-cell override.
2. **Lookahead.** diff-ik moves its target at the parcel's estimated velocity, so it needs 0.2 s of lookahead against
   builtin's 0.4 s. In early single runs at 0.1 m/s, 0 s and 0.4 s both missed and 0.1 s gripped late (3.4 s). The
   remaining 0.2 s is
   delay outside the loop: the parcel position is already stale when it's sent, and the arm trails its commands.
3. **Gripper collision checks** (v0.3.0). The ePick's body, stem and cup are meshes and used to be skipped. They're now
   checked as their bounding boxes. At 0.5 m/s, before the ramp-up, both builds missed all three of the same parcels,
   so the boxes didn't cost a pick.
4. **Ramp-up start** (`--ramp-accel 2`). The goal leaves the park point while the parcel is upstream and meets it at
   belt speed, with no jump in position or speed: for a ramp distance D = v²/(2a), the goal is park + s²/(4D) for
   s < 2D, where s is the parcel's position less (park − D). At 0.5 m/s this went from 0 of 3 to 3 of 3 on the same
   parcels. At 0.7 m/s it still missed 3 of 3.
5. **Faster hover and descent** (50 mm, 400 mm/s). At 0.7 m/s it locked on in 2 of 3 runs instead of 1 but still
   gripped none.
6. **Collision damper start, `d_act_mm` 40.** Once the cup was checked, it came within 100 mm of the belt on every
   descent, inside the default distance where the damper starts slowing the arm. Parcels are at least 80 mm tall, so
   for this cell the damper now starts at 40 mm. The 10 mm minimum to the belt, rails and pedestal is unchanged. On
   its own it didn't turn a 0.7 m/s run into a pick.
7. **Looser, faster lock-on** (20 × 10 mm for one tick, instead of 10 × 5 mm for three). This made 0.7 m/s work, 3 of
   3. Waiting three ticks in the tight window spent about 0.4 s the parcel needed to stay in reach.

## Remaining errors and limits

- **Sideways offset at speed.** The x error grows with belt speed: about 4 mm at 0.3 m/s, 8 mm at 0.5 m/s, 15 mm at
  0.7 m/s. It isn't tilt (orientation error stays under 0.4°). diff-ik measures the sim arm trailing its commands by
  about 200 ms, 260 ms at 1.0 m/s, and the flange drifts off the straight line in proportion to belt speed. That points
  to joints lagging by different amounts. diff-ik plans from the joints it commanded, not the measured ones, so it
  can't correct this today.
- **1.0 m/s.** The parcel crosses the 850 mm reach in 0.85 s. With the final lock-on, 2 of 12 runs picked (ramp
  2 m/s² with 0.2 s lookahead: 1 of 3; ramp 3 m/s²: 0 of 3; lookahead 0.25 s: 1 of 3; 0.3 s: 0 of 3), with errors up
  to 31 mm. Two of the misses were the script's `StopIteration` crash, since fixed. It would need a longer reach or a
  faster arm.
- **The upper arm runs about 100 mm from the pedestal** for the whole pick, at the edge of the default damper distance.
- **Builtin teleop has one run per speed**, from varying parcels, and wasn't retried with the ramp-up or lock changes,
  which are script changes it could also use.

## Compatibility gap found and fixed

The example sends a bare `{"teleop_stop": true}` to clear any teleop pipeline before a run. Builtin accepts it and
stops everything; diff-ik required `component_name` and returned an error, so the unmodified script couldn't run against
it. diff-ik now stops every teleop session when `component_name` is missing (`TestTeleopStopAll`). A 0.3 m/s pick with
the script's bare stops restored picked the parcel with error (4.1, −2.0, 1.2) mm.

## Not done

- A `servo` mode for `conveyor_pick.py` upstream, using `servo_start` directly. That's a change to the sim repo and
  needs its owner's OK.
- Correcting the path from measured joints, which would address the sideways offset.
- The same sweep on the DLS solver, which would show whether QP costs anything in tracking here. DLS can't use the
  rails.
