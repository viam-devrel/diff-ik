# Diff-IK QP solver and posture task: Design

Date: 2026-11-18
Builds on:
- the IHMC KinematicsToolboxController review in the scope doc ("Cartesian Servoing in the Motion Service", Prior Art and open question 5);
- the joint acceleration limit (`54416bf`, step 7b `limitAccel` in `loop.go`);
- the Phase 4 moving-tag example (`7da6bbc`).

## Goal

Add an opt-in per-tick quadratic program (QP: a least-squares objective with linear inequality constraints) as a second solver beside the current DLS step. It treats the joint position, velocity and acceleration limits as hard bounds inside the solve, instead of clamping the step afterwards. It also adds a posture task that pulls redundant joints toward a preferred configuration.

This is the first of two specs. The next one adds collision avoidance as velocity inequalities (`J_d·Δq ≥ −(d − d_min)`), using the general constraint rows this spec builds into the solver but leaves empty.

## Decisions

| Topic | Decision |
|---|---|
| Rollout | Opt-in per session: `solver: "qp"`. `"dls"` stays the default, and the DLS path stays byte-identical, so every existing golden is unchanged. |
| Solver | A dense dual active-set QP (Goldfarb–Idnani, the method behind `quadprog`) in pure Go, in a new `qp.go`. No cgo and no new dependency (gonum has no QP solver). It starts from the unconstrained minimum, so it needs no feasible starting point, and it reports infeasibility directly. Bounds are passed to it as rows. No warm start: at n ≤ 7 a cold solve is microseconds. |
| Tasks | Weighted (soft), as in IHMC. Strict priority (hierarchical QP) is out of scope. |
| Posture | A new low-weight task, accepted only with `solver: "qp"`. |
| Test arms | SO-101 (5-DoF) in the Phase 3b and Phase 4 cells. The Franka Panda (7-DoF) from the `franka-mug-sim` machine, run as a local copy of its config. |

## Objective

Each tick, in per-tick step units (radians per tick, as the rest of the loop):

    min over Δq:  ‖W^½ (J·Δq − Δx)‖²  +  (λ² + μ)‖Δq‖²  +  w_p ‖Δq − scale·k·dt·(q₀ − q)‖²
    subject to:   lb ≤ Δq ≤ ub,   A·Δq ≥ b   (no rows in this spec)

- **Task rows, J and Δx:** built exactly as `solve` builds them today, including the TCP speed caps (`capNorm` on the linear and angular parts), the gain, the velocity feed-forward and the ramp `scale`. Only the step from Δx to Δq changes. On the QP path the bounds replace the pointing task's `lockOutward` re-solve loop.
  - **Pose task:** the 6 rows, weighted by `task_weights`. A zero weight drops the row, as today.
  - **Pointing task:** the 3 position rows (weight 1) and the 2 tool-axis rows from `pointingRows`, with the constant weight `pointingAxisWeight = 1e-4`. Its square root, 0.01, makes 1 rad of axis error cost the same as 10 mm of position error, so position wins whenever the two conflict. This stands in for today's strict priority.
- **Damping, λ:**
  - Pose task: the same adaptive λ the DLS step computes today (`dlsStep`, or `dpinv` when there are fewer active rows than joints), from the same weighted rows.
  - Pointing task: taken from the first `prioritySolve` (before the `lockOutward` re-solves), computed per level, λ₁ from the position rows J₁ and λ₂ from the axis rows J₂ projected into J₁'s null space, and applied as `λ = max(λ₁, λ₂)`. It is **not** taken from the axis-weighted stack: there the 0.01 row scale would put σmin below `SingularEps` on every tick and damp the axis away.
  - A tick is singular, for `limited`, exactly when it is today: σmin below `SingularEps` on the path's own rule.
- **μ = 1e-6** keeps the Hessian positive definite when a redundant arm has no damping and no posture, and stops near-null directions from drifting. (The SO-101's wrist roll lies almost in the null space of the 5 pointing rows, because the URDF's rounded constants leave a tiny singular value there. `dpinv` cuts it by rank, and μ = 1e-9 let it drift about 2° in 3 s.)
- **Posture:** `q₀` from `posture.joints_deg`, otherwise the joints at `servo_start`. `w_p` is `posture.weight` and `k` is `posture.gain` (1/s). The pull is multiplied by the ramp `scale`, so at `scale` = 0 (ramp-stop, dead-man) the term becomes `w_p‖Δq‖²` and asks for no motion. It is absent when `posture` is not given.

## Bounds (QP path)

Per joint i, with `dt`, the commanded joints `q = qCmd` and `last = lastStep`:

- position: `lo − q ≤ Δq ≤ hi − q`;
- velocity: `|Δq| ≤ v_max·dt`;
- braking, discrete: toward the limit in the direction of motion, at distance `d ≥ 0`, `|Δq| ≤ s_max = max(√(2·a·d)·dt − c/2, min(d, c), 0)` with c = a·dt². The `min(d, c)` term covers d < c/8, where the first term is negative: a step s ≤ c stops on the next tick. Otherwise, a step s obeying it satisfies (s + c/2)² ≤ 2cd, so slowing by c each tick stops within d for any s (not only whole multiples of c), and the next tick's cap is at least s − c. (The DLS path's continuous `√(2·a·d)·dt` cap can overshoot; the DLS path is left unchanged.)
- acceleration: `last − a·dt² ≤ Δq ≤ last + a·dt²`.

The interval is the intersection of all four. Because the previous tick obeyed the discrete braking cap, the intersection is non-empty in normal running. It can be empty only after a resync, a limit change, or a fallback tick (which uses the DLS path's continuous cap). In that case **position, velocity and braking win**: use the step in the position ∩ velocity ∩ braking interval nearest the acceleration interval. That tick may exceed the acceleration limit, which `peak_accel_ratio` reports.

With `solver: "qp"`, steps 7 (uniform speed scaling), 7b (`limitAccel`) and 8 (the position clamp) are replaced by these bounds. A final clamp to `[lo, hi]` stays as a guard. The QP's solution is sent as is: it is not scaled afterwards, so the direction of motion is the best one inside the bounds rather than the unconstrained direction scaled down.

## Solver: `qp.go`

    type qpProblem struct {
        H      *mat.SymDense // n×n, positive definite
        g      []float64     // n
        lb, ub []float64     // n
        A      *mat.Dense    // m×n, may be nil
        b      []float64     // m
    }
    func solveQP(p qpProblem) (x []float64, active int, iters int, err error)

- Goldfarb–Idnani dual active-set: Cholesky of H, start at `x = −H⁻¹g`, then add the most-violated constraint and drop multipliers as they reach zero. The 2n bounds and the m general rows are one constraint list.
- It returns an error on infeasibility, when the iteration cap (50) is reached, or when the Cholesky fails.
- `active` is the number of active constraints at the solution.
- Constraints count as violated only beyond 1e-12, so a pinned joint (lb = ub, two parallel rows) doesn't make the solver cycle.
- n ≤ 7. m is 0 in this spec. The implementation keeps general rows working and tested, because the collision spec depends on them.

## Fallback and faults

- If `solveQP` returns an error, that tick uses the existing DLS path, unchanged: `dlsStep` or `dpinv` for the pose task, or `prioritySolve` plus `lockOutward` for the pointing task. It is followed by the existing steps 7, 7b and 8, and increments `qp_fallbacks`.
- After 10 consecutive fallbacks, the session ends as `faulted` with the reason `qp: <last error>`.
- A tick is `limited` when any bound is active in the QP solution, or when the tick is singular (as defined under Damping), or when the tick fell back and the old path reported limited. Convergence and stall logic are unchanged.

## Config and status

New `servo_start` fields, also accepted in the `teleop` defaults block:

| Field | Default | Notes |
|---|---|---|
| `solver` | `"dls"` | `"dls"` or `"qp"`. Anything else is rejected. |
| `posture.joints_deg` | joints at session start | One per joint. A wrong length is rejected. |
| `posture.weight` | 1e-3 | Must be in (0, 0.01]: posture motion isn't bound by the TCP speed caps. The Panda posture unit test sets the final default (see Tests). |
| `posture.gain` | 1 | 1/s, must be > 0. |

`posture` with any solver other than `qp` is rejected with a message saying so. The posture can't change during a session: to change it, stop and start a new session.

New status fields, from `servo_status` and each teleop session:
- `solver`, `qp_active` (active constraints this tick), `qp_iters` and `qp_fallbacks` (the session total), on the QP path;
- on both solvers, two fields that cover the Phase 4 follow-up (checking the acceleration limit directly):
  - `q_cmd_deg`: the commanded joints in degrees, from the loop's existing `QCmd`;
  - `peak_accel_ratio`: the session's maximum over ticks and joints of `|step − lastStep| / (a·dt²)`, computed in the loop from the step actually sent. A value ≤ 1 (to 1e-9) means the acceleration limit held on every tick. Client CSVs are written per send, not per tick, so they can't measure this exactly.

## Task 0: checks before any code

The results doc records each one. If a check fails, stop and report.

1. **The Panda in diff-ik.**
   - Copy `franka-mug-sim`'s config into `dev/panda-mujoco.json`, on its own port (8083), adding the `diff-ik` module (`bin/diff-ik`) and a `servo` service on `pick-arm`. The mujoco-sim module is `~/src/viam-mujoco-sim` on branch `feat/custom-assets`, which derives kinematics for MJCF arms.
   - Do not change `franka-mug-sim`'s cloud config or the sim repo.
   - Confirm diff-ik loads a 7-DoF model with the arm's limits (30°/s and 60°/s²), and that a DLS `servo_start` holding the current pose converges.
   - Export the model JSON to `testdata/panda.json` for the unit tests.
2. **A joint-limit path.** diff-ik takes joint ranges from the arm's model and has no override, and the sim arm enforces the same ranges. So find a target path that drives a joint into its real limit. *Finding (Task 0):* a straight line can't drive joint index 3 into its −4° limit, because the pose task runs out of reach at about −26.7°. An orbit of the start pose about base z drives joint index 0 into its +166° limit; the evidence run is 5°/s for 420°.
3. **Solve time.** A Go benchmark of `solveQP` at n = 7 with every bound in the list and 7 active, next to `dlsStep`. The bar is a p99 below 0.5 ms.

## Tests

**`qp_test.go`:**
- unconstrained: matches `−H⁻¹g`;
- one active bound, and several, against hand-solved cases;
- a general row binding, including a row that excludes the origin (`b > 0`), against hand-solved cases;
- an infeasible problem, the iteration cap and a non-positive-definite H return errors;
- a property test on random positive definite problems whose constraints are built around a known feasible point: the solution is feasible and satisfies the KKT conditions to 1e-9.

**The step:**
- Pose task, at a well-conditioned configuration (σmin above `SingularEps`, so λ = 0 on both paths), with no bound active and no posture, and `lastStep` set equal to the DLS step so the acceleration bound is slack: the QP step equals the DLS step (or the `dpinv` step on the 7-DoF Panda) to 1e-6. The tolerance allows for μ (relative error about μ/σ²).
- Pointing task on the SO-101, at a well-conditioned configuration with no bound active and `lastStep` as above: the QP step matches `prioritySolve`'s in the position rows of J·Δq to 1e-7 (μ's bias, about 0.1% of a 4e-5 m step), and on joints 0–3 and the angular rows to 5e-6. (μ scales the 0.01-weighted axis step by about 0.5%, a bias the gain loop absorbs.) The wrist roll is excluded, because it is near-null under the pointing rows. A separate test checks that, without posture, the roll drifts ≤ 0.1° over 150 ticks.
- Fuzzed targets, joints and `lastStep`: the QP step never leaves its bounds.
- Near a joint limit: the QP step reduces the task error more than the DLS step followed by the clamps.
- The discrete braking cap: from any speed that obeys it, decelerating at a·dt² per tick stops at or before the limit, and the next tick's interval is non-empty. This is a table over d and a, plus a random check that includes non-integer s/(a·dt²), the case a cap of `s(s+c)/(2c) ≤ d` gets wrong.

**Posture, with the `testdata/panda.json` model:**
- Build q₀ by moving the hold configuration along the self-motion (integrating the null space of J at the hold pose) until joint index 0 has changed by 20° (at the hold pose, index 3 barely moves along the self-motion), so q₀ holds the same pose.
- Start a session at the hold configuration with that q₀, and hold the pose.
- After 3 s of simulated ticks, every joint is within 2° of q₀, and the TCP error stays below 0.5 mm throughout. (Null-space motion at about k·e rad/s adds second-order TCP drift, about 0.2 mm at the start, whatever the weight.)
- If the defaults miss either bar, the plan's implementer tunes the defaults of `posture.weight` and `posture.gain` until both hold, and records the values. If no setting meets both bars, stop and report.

**The loop:**
- the existing goldens pass on the DLS path, unchanged;
- a new golden for the QP path on the SO-101 static case;
- the acceleration, ramp-stop and dead-man tests become table tests that run on both solvers, plus a QP ramp-stop case with a posture error of 20°, which must stop within N + 1 ticks, where N is the tick count without posture;
- a forced solver failure exercises the fallback and the fault after 10 failures in a row.

**Module:**
- the config cases above;
- `q_cmd_deg` and `peak_accel_ratio` in status;
- a loop test that checks `peak_accel_ratio` ≤ 1 on both solvers in the acceleration test, and a white-box test that overwrites `lastStep` between ticks to show the field going above 1, so the check can fail.

## Evidence: sim A/B

Each run uses both solvers with the existing clients (`servo_bench.py`, the CLI's `run` cases, `tag_sweep.py`).

| Run | Cell | Shows | Bar for `qp` |
|---|---|---|---|
| Static and circle | SO-101, Phase 3b | no regression | circle mean error within 10% of DLS; 0 overruns |
| Orbit about base z, 5°/s for 420°, into joint 0's limit (Task 0 path) | Panda | sliding along a limit | lower mean error than DLS once the limit binds; joint 0's commanded position never passes its limit |
| Posture: one session with the default q₀, then a new session with q₀ moved 20° along the self-motion | Panda | null-space posture | in the second session, every joint within 2° of q₀ in 3 s; TCP error < 1 mm throughout |
| Circle | Panda | the 7-DoF path end to end | reported only, with DLS as the baseline |
| Moving-tag sweep, wrist camera | SO-101 tag cell | the QP under visual servoing | mean error at 20 mm/s within 10% of DLS |

`peak_accel_ratio` at the end of every run:
- **QP:** must be ≤ 1 on every run;
- **DLS:** reported only. It is expected to go above 1 on the joint-limit run, because its position clamp runs after `limitAccel`. That is today's behaviour, and the A/B shows it.

## Output

- `docs/results/2026-11-qp-solver.md`: the Task 0 findings, the A/B table, and a recommendation on whether to make `qp` the default.
- `README.md` and `devrel_diff-ik_servo.md`: `solver`, `posture` and the new status fields.
- `servo_client.py`: append `peak_accel_ratio` to `FOLLOW_COLS` (filled on status rows, like `lag_ms`).
- `dev/panda-mujoco.json` and a runbook, `dev/panda-mujoco.md`.

## Process

- Branch `diff-ik-qp`, squash-merged to `main` at the end.
- Subagent-driven: Sonnet implements, Opus reviews.
- Commits use the session's commit trailer. No push or release unless the user asks.

## Out of scope

- Collision avoidance (the next spec).
- Strict priority through a hierarchical QP.
- Making `qp` the default.
- Changing the DLS path's braking cap or its order of clamps.
- Hardware runs.
- Changes to the sim repo or to `franka-mug-sim`'s cloud config.
