# Diff-IK collision avoidance on the QP solver: Design

Date: 2026-11-18
Builds on:
- the QP solver and posture task, `docs/superpowers/specs/2026-11-18-diff-ik-qp-solver-design.md` (merged as `0325d2c`). Its `solveQP` already handles general rows `A·Δq ≥ b`, which this spec fills.
- the IHMC KinematicsToolboxController review in the scope doc ("Cartesian Servoing in the Motion Service"). Collision there is a velocity inequality on the distance between pairs of bodies.

## Goal

Keep the arm and anything rigidly mounted on it a minimum distance from known obstacles while it servos. Do it inside the QP, so the arm slows down and stops at the obstacle, or slides along it where the task allows, instead of the target being clipped after the fact.

The obstacle and allowed-collision inputs take the same shapes the builtin motion service takes with `Move`: a `WorldState`, and a `CollisionSpecification` of allowed frame pairs. A client written for `motion.Move` can pass the same data.

## Decisions

| Topic | Decision |
|---|---|
| Obstacles | Every geometry in the machine's frame system that isn't part of the robot, plus the session's `world_state` obstacles. Both are read once at `servo_start` and fixed for the session. |
| Robot parts | The arm's link geometries, plus the geometries of frames rigidly mounted on the arm (gripper, camera bracket), plus `world_state` transforms whose parent is the arm or a mounted frame (a held object). No self-collision. |
| Allowed contact | `collision_specification: [{allows: [{frame1, frame2}]}]`, in the motion service's JSON shape. Allowed pairs are not checked. |
| Constraint | An acceleration-aware damper per close pair: `J_d·Δq ≥ −v_max(d)`. `v_max` is the fastest approach per tick from which the arm can still stop before `d_min` within its joint acceleration limits, the same discrete braking rule as `brakeCap`. `J_d` is a finite-difference gradient of `DistanceFrom`. |
| Solver | Collision avoidance needs `solver: "qp"`. Asking for it with `dls` is rejected. |
| Fallback | With collision rows active, a failed QP first relaxes each row to what the bounds can reach, then **brakes** instead of falling back to the DLS step, because DLS ignores obstacles. |
| Default | Off unless `collision`, `world_state` or `collision_specification` is given, so existing qp sessions don't change. |
| Geometry | Boxes, capsules, spheres and points, whose `DistanceFrom` is signed in RDK v1.16. Mesh distances are unsigned, and stay 0 inside the mesh, so meshes aren't used. A mesh in the **frame system** (a sim asset such as a bin or gripper body) is skipped, with a warning, and listed in `collision_skipped`. A mesh passed explicitly in `world_state`, or an arm link mesh, is rejected at start. Task 0 found the Panda cell's bin and gripper bodies are frame-system meshes. |
| Starting overlaps | Pairs already closer than `d_min` at `servo_start` are ignored for the session and reported, as the motion service ignores collisions present at the start of a plan. |
| Test arm | The Franka Panda MuJoCo cell (`dev/panda-mujoco.json`), plus the SO-101 tag cell for a no-regression check. |

## Inputs

New `servo_start` fields. They're also accepted in the `teleop` defaults block, which goes through `startSession`.

```json
{
  "solver": "qp",
  "world_state": { "obstacles": [ ... ], "transforms": [ ... ] },
  "collision_specification": [ { "allows": [ { "frame1": "pick-gripper", "frame2": "table" } ] } ],
  "collision": { "d_min_mm": 10, "d_act_mm": 100 }
}
```

| Field | Default | Notes |
|---|---|---|
| `world_state` | none | Decoded as a `common.v1.WorldState` with protojson, the same JSON the motion service API uses, then turned into a `*referenceframe.WorldState` with `WorldStateFromProtobuf`. Obstacle geometries are resolved into the arm's base frame through the frame system at start. |
| `collision_specification` | none | A list of `service.motion.v1.CollisionSpecification`, each with `allows: [{frame1, frame2}]`, decoded with protojson. |
| `collision` | off | `true`, or an object with the fields below, turns avoidance on. Giving `world_state` or `collision_specification` also turns it on. `false` turns it off even when either is given. |
| `collision.d_min_mm` | 10 | The distance to keep, > 0. |
| `collision.d_act_mm` | 100 | Pairs closer than this get a row. Must be > `d_min_mm`. |

Validation, each rejected with a message naming the field:
- `world_state`, `collision_specification` or a `collision` object given with any solver other than `qp`;
- a `world_state` or `collision_specification` that doesn't decode;
- an `allows` name that matches no robot part and no obstacle. A typo would otherwise silently leave a pair checked;
- `d_min_mm` ≤ 0, or `d_act_mm` ≤ `d_min_mm`;
- unknown keys inside `collision`.

## The collision model, built at `servo_start`

1. **Build the frame system.** Build a local frame system from `FrameSystemConfig`, as `frames: true` already does. Add `world_state` transforms to it, as the motion service does.
2. **Robot parts.**
   - **Links:** the arm's link geometries come from the arm's model, `Geometries(q)`, re-posed every tick. The model labels each one `<model name>:<link>`. Because the model name needn't match the arm's resource name, the label is normalized to `<arm name>:<link>`.
   - **Mounted parts:** frames in the arm's subtree with no degrees of freedom of their own, reached from the arm through fixed frames only. Their geometries and fixed transform from the arm's end effector are computed once at start, the way `teleop_start` computes a mounted component's transform.
   - **Movable mounted frames:** a frame in the subtree that is movable itself (for example, a gripper whose kinematics have DoF > 0) is **skipped**, along with everything below it. It's listed in status as `collision_skipped`, and the session doesn't fail. To protect such a part, pass it as a `world_state` transform parented to the arm.
   - **`world_state` transforms:** those parented to the arm or to a mounted frame become mounted parts. Other transforms are frames added to the frame system, as the motion service adds them.
3. **Obstacles.**
   - **From the frame system:** geometries on frames whose chain to `world` is fixed (no DoF anywhere on the chain).
   - **Movable frames outside the arm** (another arm, a gantry) are skipped and listed in `collision_skipped`. The session's frame system has no inputs for them, and this spec doesn't read them.
   - **From `world_state`:** obstacles taken from `ws.Obstacles()`, each transformed into the arm's base frame. **Don't use `WorldState.ObstaclesInWorldFrame`.** It also appends every transform's geometry, unposed, which would turn a held object into a static obstacle.
   - All obstacles are fixed for the session.
4. **Names for `allows`,** following `motionplan`'s collision specifications:
   - **A part** matches:
     - its geometry label (for links, `<arm name>:<link>`, and the raw model label too);
     - its frame's name: the arm's name for every link, or the mounted component's name;
     - `<frame name>_origin`.
   - **An obstacle** matches:
     - its geometry label;
     - its frame name, or for a `world_state` obstacle its reference frame;
     - `<frame name>_origin`.
   - **An entry** matches a pair when one name matches the part and the other matches the obstacle, in either order. So `"pick-arm"` ↔ `"table"` allows every link against the table, and `"pick-arm:link7"` ↔ `"table"` allows only that link.
   - **A name that matches nothing** is an error at start, as in the motion service. The exception is the name of a frame that was skipped as movable: that name is accepted as a no-op, with a warning in the log. So `pick-gripper` ↔ `table` stays valid when the gripper is skipped.
5. **Pairs.** Every (part, obstacle) pair not allowed.
   - Pairs closer than `d_min` at start are suspended and listed in status as `collision_ignored`. This covers, for example, the base link resting on the table.
   - A suspended pair is re-armed once its distance goes above `d_act`, so a part that starts near an obstacle is protected again after it has moved away. It's removed from `collision_ignored` then.
6. **No pairs.** If no pairs are left, collision avoidance is off for the session. Status says so.

## Each tick (QP path, collision on)

1. **Pose the parts.** The arm's link geometries come from `Geometries(qCmd)`. Mounted parts are posed through FK(qCmd) composed with their fixed transform.
2. **Measure every pair** with `DistanceFrom`, in mm and signed: negative means penetration.
3. **Choose the active pairs:** those with d < `d_act`, the closest 16 if there are more.
4. **The gradient,** only if a pair is active. For each joint j, perturb `qCmd[j]` by `fdStep`, re-pose the parts, and re-measure the active pairs. That gives `J_d[j] = (d(q + h·e_j) − d(q − h·e_j)) / (2h)` in mm/rad. This uses central differences, like `Jacobian`.
5. **The rows,** in the solver's step units: Δq in rad per tick, d in mm.
   - With the gap `g = d − d_min`, the row is `J_d·Δq ≥ −v_max`.
   - `A_d = Σ_j |J_d[j]|·a_j·dt²` (mm per tick²) is the most `J_d·Δq` can change in one tick inside the acceleration bounds.
   - `v_max = max(√(2·A_d·g) − A_d/2, min(g, A_d), 0)`. This is `brakeCap`'s discrete rule with c = A_d, so an approach at `v_max` that is slowed by up to A_d per tick stops within g, and the next tick's `v_max` is at least the current approach minus A_d. A_d changes a little with configuration; the relaxation below covers that.
6. **Activation.** `d_act` must be at least the stopping distance at the arm's top approach speed. Status reports `collision_rows`, and the session logs a warning if a pair enters with an approach faster than its `v_max`, meaning `d_act` was too small (it fires above `v_max + A_d + tol`; live pairs are excluded). The warning names the pair and gives its approach speed and `v_max` in mm per tick, so the default can be tuned from the evidence runs.
7. **Solve:** `qpNext` passes these rows as `A`/`b` with the bounds.

The rows only ever forbid motion. At `scale` = 0 (ramp-stop or dead-man) the task asks for no motion, so stopping is unchanged.

## Failures

- **Relaxing first.** If `solveQP` fails with collision rows active, retry once with each row's bound lowered to what the step bounds can reach. Each row then becomes `J_d·Δq ≥ min(−v_max, M) − ε` (ε = 1e-6 mm per tick, because at exactly M the feasible set is a single box corner, which the solver reports infeasible), where `M = Σ_j max(J_d[j]·lb_j, J_d[j]·ub_j)` is the strongest move away the bounds allow. A row the bounds can meet is unchanged; one they can't is relaxed to "move away as hard as the bounds allow". So each row is feasible on its own, but the rows can still conflict with each other. At rest, 0 is in the box, so M ≥ 0 and Δq = 0 meets every relaxed row. Count the retry in `collision_relaxed`.
- **Then braking.** If the relaxed solve also fails, the tick **brakes**: `step[i] = lastStep[i] − clamp(lastStep[i], −a_i·dt², a_i·dt²)`. That is the previous step moved toward zero by at most a·dt², and clipped to the position bounds. It adds no motion the previous tick didn't have, and it respects the acceleration limit. Count it in `collision_brakes` and in `qp_fallbacks`.
- **The fault rule changes for braking.** A brake tick while the arm is still moving (`lastStep` ≠ 0) doesn't count toward the 10-in-a-row fault: braking from full speed can take more than 10 ticks. Failures count once the arm is at rest. At rest, Δq = 0 satisfies every relaxed row and the bounds, so a failure there is a real solver fault. After 10 in a row, the session faults with `qp: <error>`.
- **No active rows.** With no active collision rows, a failed QP falls back to DLS exactly as before.
- **Geometry errors.** A geometry or distance error faults the session with `collision: <error>`. It doesn't fall back.
- **Starting inside `d_min`.** Such pairs are ignored for the session (see "Pairs"), and status lists them.

## Status

On qp sessions, alongside the existing fields:

| Field | Meaning |
|---|---|
| `collision` | `"on"`, `"off"` (turned off, or no pairs) |
| `collision_ignored` | pairs dropped at start because they were already closer than `d_min` |
| `collision_skipped` | movable frames left out (as parts or obstacles) |
| `collision_min_mm` | the smallest distance over all unsuspended pairs, at any distance; absent only when every pair is suspended (or before the first tick) |
| `collision_pair` | that pair, as `"<part>↔<obstacle>"`; empty only when every pair is suspended (or before the first tick) |
| `collision_rows` | active rows this tick |
| `collision_relaxed` | session total of relaxed retries |
| `collision_brakes` | session total of brake ticks |

## Task 0: checks before any code

The results doc records each one. If a check fails, stop and report.

1. **Frame-system obstacles.** In the Panda cell, list the geometries the local frame system yields. Expect a box for the table, from its frame config, and nothing for the bin, whose mesh is a sim attribute. Record what's there. Obstacles the frame system doesn't have come from `world_state` in the evidence runs.
2. **Mounted parts.** Confirm whether `pick-gripper` (Robotiq 2F-85, mounted on `pick-arm`) has a geometry in the frame system, and whether its frame is movable (DoF > 0). If it has no geometry, or it's movable (and so skipped), the evidence runs pass a box for it as a `world_state` transform parented to the arm, and the results say so.
3. **Link geometry.** Confirm what geometry types the **live** arm's `Kinematics()` returns per link, as well as `testdata/panda.json` (all boxes). If the live arm has mesh links, stop and report: meshes are rejected.
4. **Cost.** Benchmark the per-tick collision work on `testdata/panda.json`: about 10 parts and 5 box obstacles, 16 active pairs, and the 7-joint finite difference. The bar is a p99 below 5 ms. If it misses, measure where the time goes and report before going on.

## Tests

**Unit tests:**
- **Gradient:** for box↔box, box↔capsule and box↔sphere pairs, `J_d` matches the change in distance after a small joint step to 1e-3 relative. The Panda fixture's links are boxes, and the obstacles are a box, a capsule and a sphere.
- **Damper:** for a table of approach speeds and gaps, an approach that obeys `v_max` and slows by `A_d` per tick stops within the gap. Include non-integer ratios, as for `brakeCap`.
- **Mesh:** a mesh obstacle, or a mesh link, is rejected at start.
- **Ignored and skipped:** a pair inside `d_min` at start is dropped and listed, and a movable frame is skipped and listed.
- **Allows:** `"pick-arm"`↔`"table"` removes every link↔table pair, and a link-name entry removes only that link's pair. An `allows` name that matches nothing is rejected at start.
- **Activation:** a pair beyond `d_act` gives no row, and a pair inside it gives one. With more than 16 active pairs, only the closest 16 get rows.
- **Approach:** with the Panda fixture on `simIO`, hold a pose, then set the target 50 mm inside a box obstacle below the tool. On every tick:
  - the minimum distance stays ≥ `d_min − 1 mm`;
  - `peak_accel_ratio` ≤ 1.

  The session ends `stalled`, not `faulted`.
- **Sliding:** a target path that grazes the obstacle. The TCP error with collision on is smaller than with the target clamped to the obstacle's surface along its normal. This shows the arm uses its free joints, not just stopping.
- **Fast approach:** an approach at the TCP speed limit (`linear_mm_s` 100, about 2 mm per tick), with joint limits of 90°/s and 360°/s², toward a box obstacle 60 mm away. The rows bind early enough that the minimum distance stays ≥ `d_min − 1 mm`, with no brake ticks.
- **Relaxed rows still bind:** for an approach faster than the bounds can stop for this tick, the relaxed step's `J_d·Δq` equals M (the strongest move away), not the task's choice.
- **Re-arming:** a pair suspended at start is re-armed after it moves beyond `d_act`, and then it constrains again.
- **Brake:** a forced solver failure with rows active, while the arm is moving, gives brake steps (each joint's |step| shrinks by a·dt²), not DLS steps, and doesn't fault until the arm is at rest. After that, 10 more failures fault the session.
- **No change when off:** with no `collision` or `world_state` given (the default), with `"collision": false`, or with no pairs, the sends are bit-identical to the QP path without this feature, and no frame system is read for collision. Shown by a bit-identical comparison of two runs (no model vs a zero-pair model).
- **Config:** every validation case above.

**Module:** parsing, validation, the teleop defaults path, and the status fields.

## Evidence: sim, collision on vs off

All runs use the Panda cell, with `--max-joint-deg-s 30 --max-joint-deg-s2 60`.

| Run | Shows | Bar (collision on) |
|---|---|---|
| Hold the pose, then move the target 100 mm down through the table | stops above the table | minimum part↔table distance ≥ `d_min − 1 mm`. With collision off, the same run goes below `d_min` (the contrast is reported). |
| Orbit past the bin, with the bin as a `world_state` box | avoids or slides along an obstacle the client supplied | minimum part↔bin distance ≥ `d_min − 1 mm`, with the TCP error reported |
| Grasp approach: allow gripper↔table, target 5 mm above the table | allowed contact still works while the other parts stay protected | the TCP reaches within 2 mm of the target, and every other part↔table distance stays ≥ `d_min − 1 mm` |
| SO-101 tag sweep, collision on, with the table as the only obstacle | no regression on an ordinary task | mean error at 20 mm/s within 10% of the QP solver without collision |

On every run, `peak_accel_ratio` ≤ 1 and 0 faults.

The CLI gains `--world-state <file.json>`, `--allow frame1:frame2` (repeatable), `--collision-off`, `--d-min-mm` and `--d-act-mm`, and prints the session's minimum distance and pair, with the brake and relax counts. It also gains a `run descend` scenario, which moves the target down by `--descend-mm` at a set speed.

## Output

- `docs/results/2026-11-qp-collision.md`: the Task 0 findings, the evidence table, and findings.
- `README.md` and `devrel_diff-ik_servo.md`: the new fields, their semantics, and the status fields.
- A watchable demo on `mujoco-sim-mac` (the descend and grasp runs), restored afterwards.

## Process

- Branch `diff-ik-collision`, squash-merged to `main` at the end.
- Subagent-driven: Sonnet implements, Opus reviews.
- Commits use the session's commit trailer. No push or release unless the user asks.

## Out of scope

- Self-collision between links.
- Obstacles that move during a session (`world_state_store` refresh), or changing `world_state` mid-session.
- Collision avoidance on the DLS solver.
- Hardware runs.
