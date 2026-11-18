# Diff-IK QP solver and posture task: results

Spec: `docs/superpowers/specs/2026-11-18-diff-ik-qp-solver-design.md`. Plan: `docs/superpowers/plans/2026-11-18-diff-ik-qp-solver.md`.

## Task 0

### The Panda in diff-ik

- **Cell:** `dev/panda-mujoco.json`, port 8083. It copies `franka-mug-sim` and adds the `diff-ik` module and a `servo` service on `pick-arm`. The mujoco-sim module is `~/src/viam-mujoco-sim` on `feat/custom-assets`. The `franka-mug-sim` cloud config is unchanged.
- **Model:** diff-ik loads a 7-DoF model. `dev/exportmodel` exports it to `testdata/panda.json` (as protojson of the model config, which `referenceframe.UnmarshalModelJSON` reads back). Joint ranges, in degrees:

  | Joint index | 0 | 1 | 2 | 3 | 4 | 5 | 6 |
  |---|---|---|---|---|---|---|---|
  | Range | ±166 | ±101 | ±166 | −176 to −4 | ±166 | −1 to 215 | ±166 |

- **Limits: a gap.** The sim arm's `max_joint_deg_s: 30` and `max_joint_deg_s2: 60` don't reach the kinematic model. A live `servo_start` falls back to diff-ik's defaults, 90°/s and 360°/s². So every Panda run passes the arm's limits explicitly (`limits.joint_deg_s` 30, `joint_deg_s2` 60). The CLI needs `--max-joint-deg-s2` for this.
- **DLS hold:** `go run ./cmd/cli --address localhost:8083 --insecure --arm pick-arm --max-joint-deg-s 30 run static` converged on all 10 targets (errors 1.93–2.00 mm against the 2 mm tolerance). The rate was 49.97 Hz, with 2 overruns and 5 resyncs. The resyncs are likely diff-ik's default 360°/s² against the arm's 60°/s².
- **Start joints:** the cell starts at [0, 0, 0, −90, 0, 90, −45]° (in radians: 0,0,0,−1.5708,0,1.5708,−0.785).

### The joint-limit path

- **The straight line into joint 3's limit doesn't work.** The spec's first choice was to drive joint index 3 into its −4° limit by moving outward. I tried pose-task twists from the start joints along +x, +z and +x+z, and also from joint 3 = −10°. In every case diff-ik settled joint 3 at about −26.7°, with the TCP moving only 13–160 mm in 20 s. The sim arm itself reaches −10° on a direct joint move, so the stall comes from the pose task: with the tool's orientation held, the arm runs out of reach before joint 3 straightens. It is not a sim limit.
- **Path used instead: an orbit about base z** that drives joint index 0 into its +166° limit.
  - The target is the start pose rotated about the base z axis at 8°/s; at about 555 mm from the axis, that is about 78 mm/s, under the default 100 mm/s cap.
  - From the start joints, DLS reached joint 0 = 142° after 25 s, and 164° (2° from the limit) about 3 s later. Joint 2 shares the yaw (it was at 78° by then).
  - Past the limit, the remaining yaw has to slide onto the other joints. That is the limit-sliding case the A/B compares.
  - The 40 s / 320° run at 8°/s stopped 0.86–0.90° short of the limit. The evidence run is 5°/s for 420° (84 s), which drives joint 0 (and joint 2) to their limits.

## Sim A/B

Every run used the CLI's in-process loop or the module's `servo_start` with `--solver dls` and `--solver qp`, back to back on the same cell. The Panda runs passed the arm's own limits (`--max-joint-deg-s 30 --max-joint-deg-s2 60`). Logs and CSVs are in `.runs/qp-*`.

| Run | Measure | DLS | QP | Bar for QP | Result |
|---|---|---|---|---|---|
| SO-101 static (fresh cell) | targets converged | 10/10 | 10/10 | no regression | pass |
| | overruns | 2 | 0 | 0 | pass |
| SO-101 circle, 50 mm/s | mean / max error | 22.84 / 24.55 mm | 23.32 / 32.24 mm | mean within 10% of DLS (+2.1%) | pass |
| | overruns | 2 | 1 | 0 | **fail** (see below) |
| Panda orbit, 5°/s for 420°, into joint 0's limit | mean error after joint 0 reached its limit | 373.1 mm | 352.7 mm | lower than DLS | pass |
| | commanded joints past a limit | none | none | none | pass |
| | peak acceleration ratio | **2.04** | **1.000** | ≤ 1 | pass |
| Panda posture, q₀ moved 20° on joint 0 | max joint distance to q₀ after 3 s (measured) | — | 1.32° | ≤ 2° | pass |
| | worst TCP error during the hold | — | 0.04 mm | < 1 mm | pass |
| Panda circle, 50 mm/s | mean / max error | 26.62 / 28.74 mm | 26.96 / 29.41 mm | reported only | — |
| Tag sweep, wrist camera | mean error at 20 mm/s | 1.7 mm | 1.6 mm | within 10% of DLS | pass |
| | tag loss up to 80 mm/s | none | none | — | — |
| Every run | QP fallbacks | — | 0 | — | — |
| Every run other than the orbit | peak acceleration ratio | 1.000 | 1.000 | ≤ 1 | pass |

### Findings

- **The QP keeps the acceleration limit at a joint limit; DLS doesn't.** On the orbit, DLS reached 2.04× its acceleration limit: its position clamp (step 8) runs after `limitAccel`, so the step is cut after the acceleration limit has allowed it. The QP held exactly 1.000, because the discrete braking cap slows the joint before it arrives. This is the main result.
- **Near a limit the QP tracks better.** After joint 0 reached its limit, the QP's mean error was 5% lower. The orbit's 420° asks for more yaw than joints 0 and 2 can give together (about 332°), so by the end the target is out of reach for both solvers and the errors are large. The comparison is like for like. A shorter 320° orbit at 8°/s stopped joint 0 0.9° short of its limit on both solvers and was speed-limited throughout (error about 48 mm on both, QP 3% lower).
- **Posture works on the 7-DoF arm.** With q₀ moved 20° along the self-motion, the measured joints were within 1.32° of q₀ after 3 s, while the end effector moved at most 0.04 mm.
- **Elsewhere the solvers match.** Static, circle and the tag sweep are within 2–6% of each other, sometimes in the QP's favour. Neither solver fell back or faulted.
- **Overruns.** Overruns (DLS / QP) were: SO-101 static from a fresh cell 2 / 0 (4 / 0 in the earlier non-fresh pair), SO-101 circle 2 / 1, Panda circle 0 / 2, Panda orbit5 2 / 5. The QP's p99 solve time is about 15 µs (`TestQPSolveTime`) against a 20 ms tick, and the QP static run had 0. So the overruns are most likely scheduling jitter on this Mac, which runs MuJoCo rendering alongside, not solver cost. The strict "0 overruns" bar is still marked as failed.
- **Run hygiene.** Static targets are picked relative to the start pose. Two static runs that started where a circle run had left the arm failed the same 4 of 10 targets on both solvers (most likely contact with the table), so the table uses runs from a fresh cell. The QP tag sweep also first failed with "no tag seen": the DLS sweep had left the arm where the wrist camera couldn't see the tag. After pre-positioning, as in `dev/so101-mujoco.md`, it ran clean.
- **Panda limits.** The sim arm's 30°/s and 60°/s² don't reach the kinematic model (Task 0), so a session left at the defaults would command 90°/s and 360°/s² that the arm can't follow.

### Recommendation

Keep `dls` as the default for now, and recommend `qp` for:
- sessions that run near joint limits, where it holds the acceleration limit and DLS doesn't;
- 7-DoF arms, which can use the posture task.

Switch the default to `qp` after a hardware A/B on the SO-101 or xArm confirms the sim results. In the sim it costs nothing measurable in tracking.
