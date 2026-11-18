# Diff-IK collision avoidance: results

Spec: `docs/superpowers/specs/2026-11-18-diff-ik-collision-design.md`. Plan: `docs/superpowers/plans/2026-11-18-diff-ik-collision.md`.

## Task 0: what the Panda cell provides

I ran `go run ./dev/fsprobe` against `dev/panda-mujoco.json` (port 8083). It builds the frame system from `FrameSystemConfig` and prints `FrameSystemGeometries` at zero inputs, then the live arm's `Kinematics().Geometries(q)`.

- **Table:** one box, on frame `table_origin` (label `table_origin`), 800 × 1200 × 20 mm, centred at z 10. So the top is at z 20.
- **Bin:** in the frame system as **13 mesh pieces** (`bin:piece0` … `bin:piece12`), from the sim's `meshes/bin.stl`. Their points span x 374–526, y −426 to −274 and z −1 to 81 mm. The evidence `world_state` (`dev/panda-collision-ws.json`) stands in a box for it: `bin-box`, 152 × 152 × 82 mm, centred at (450, −350, 40).
- **Gripper:** `pick-gripper` is fixed (0 DoF, parent `pick-arm`), so it counts as a mounted part. Its geometries are mostly **meshes** (base, drivers, couplers, links, followers, in both open and closed variants), plus 8 **box** pads, again both open and closed. The world state adds a box stand-in, `pick-gripper-box`: 90 × 140 × 160 mm, centred 80 mm along the tool axis from the flange.
- **Arm links:** all **boxes**. The model name is `pick-arm`, and the labels are `pick-arm:link1` … `pick-arm:link7` and `pick-arm:attachment_site`. The live model matches `testdata/panda.json`.
- **Start pose:** the flange is at (554.5, 0, 624.5) mm at the start joints [0, 0, 0, −90, 0, 90, −45]°.

### The finding that changes the spec

The spec rejected meshes anywhere at start. In this cell that would make collision unusable, because the bin and most of the gripper are meshes in the frame system. So the spec now:
- **skips** frame-system mesh geometries, listing them in `collision_skipped` with a warning;
- still **rejects** meshes passed explicitly in `world_state`, since the client chose them.

The cost is that a mesh in the frame system is unprotected unless the client supplies a box, capsule or sphere for it in `world_state`, as the evidence runs do for the bin and the gripper.

## Evidence: sim, collision on vs off

All Panda runs used the CLI's in-process loop on `dev/panda-mujoco.json`:
- `--solver qp --max-joint-deg-s 30 --max-joint-deg-s2 60`;
- the world state `dev/panda-collision-ws.json` (a `bin-box` and the `pick-gripper-box` stand-in);
- the defaults `d_min` 10 mm and `d_act` 100 mm.

The minimum clearance is measured at the **commanded** joints (`q_cmd`), not the measured ones. For the collision-off runs, which don't measure distances, it comes from replaying each run's `q_cmd` CSV through the same collision model. Logs and CSVs are in `.runs/coll-*`. The same five runs were repeated as a live demo on `mujoco-sim-mac`, with the same results to within 0.1 mm.

| Run | Collision on | Collision off | Bar | Result |
|---|---|---|---|---|
| Descend 700 mm through the table (default start) | min **9.87 mm** (pick-gripper↔table), 0 brakes, 0 relaxed | min **−46.2 mm** (gripper box 46 mm inside the table); the sim arm jammed (154 resyncs) | on ≥ `d_min − 1`; off goes below `d_min` | pass |
| Orbit −5°/s for 70° through the bin, flange at z 220 | min **10.00 mm** (pick-gripper-box↔bin-box), 0 brakes; the arm stops at the bin side, so tracking error grows (mean 173 mm) | min **−37.3 mm** (gripper 37 mm into the bin), 76 resyncs | on ≥ `d_min − 1` | pass |
| Grasp approach: allow gripper↔table, flange to z 180.8 (tip 5 mm above the table) | flange within **0.34 mm** of target; every non-allowed pair ≥ **128 mm** | — | TCP ≤ 2 mm; others ≥ `d_min − 1` | pass |
| SO-101 tag sweep, wrist camera, collision on (table only) | 1.8 mm mean at 20 mm/s (QP alone: 1.6 mm), no loss, peak acceleration ratio 1.000 | — | within 10% of the QP sweep | **missed (+12%)**; see below |

On every Panda run, the peak acceleration ratio was ≤ 1 and there were 0 QP fallbacks.

### Findings

- **The damper holds `d_min` and stops smoothly.**
  - Both blocking runs stopped at 9.87–10.00 mm, with no brakes and no relaxed ticks: the acceleration-aware `v_max` slowed the approach early enough.
  - With collision off, the same commands put the gripper 37–46 mm inside the obstacle, and the sim arm jammed against it.
- **Allowed contact works with the rest of the arm still protected.** The grasp descent reached 5 mm above the table. The closest protected pair (`link1↔table`) stayed 128 mm away.
- **The approach-speed warning showed a tiny overshoot.**
  - It fired at 0.0008 mm/tick on the table: second-order drift while the gap sits at `d_min`. The warning now has a 0.01 mm/tick tolerance.
  - On the bin orbit it fired at 0.039 mm/tick against a `v_max` of 0.005, as the gripper box's corner reached the bin. That's harmless, since the minimum distance held at 10.00 mm, but it shows the damper can be marginally late at an edge.
- **Frame-system meshes were skipped as designed:** the 13 bin pieces and 18 gripper mesh parts, with one warning each at start. The gripper's 8 box pads stayed in, and they were the closest part in the table descend. The warning list is long; collapsing it to one line per frame would be friendlier.
- **The tag-sweep regression can't be pinned on collision on this Mac.**
  - The loop's tick p99 was *lower* with collision on (median 7.24 ms) than with QP alone (7.65 ms).
  - A QP-only rerun later the same afternoon gave 2.7 mm at 20 mm/s, with 130 ms of lag and 14 overruns, far worse than either earlier run. So between-run variance on this machine is much larger than the 0.2 mm difference.
  - The hover keeps the jaws about 150 mm above the table, outside `d_act`, so most likely no collision rows were active.
  - A repeat of the collision run failed to start with "no tag seen", the pre-positioning issue from Phase 4.
  - The bar is recorded as missed, because a single A/B is what the spec asked for. A fair check needs interleaved repeats on a quieter machine.
- **"No change when off"** is shown by a bit-identical comparison of two runs (a QP run with no model, and one with a zero-pair model) rather than a golden file, as the plan allowed.

### Known limits (also in `devrel_diff-ik_servo.md`)

- Distances are commanded-pose clearances.
- Obstacles are fixed for the session.
- There is no self-collision checking.
- Frame-system meshes are unprotected unless a primitive is supplied in `world_state`.
- Capsule↔box distances can jump near contact (the RDK switches to SAT).
- Sliding along a face can drift about 0.1–0.2 mm below `d_min`.
