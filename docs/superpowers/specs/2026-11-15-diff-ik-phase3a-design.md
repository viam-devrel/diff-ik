# Diff-IK Phase 3a — 5-DOF Task Formulation on the Simulated SO-101: Design

Date: 2026-11-15
Builds on: `2026-11-15-diff-ik-phase0-1-design.md` and `2026-11-15-diff-ik-phase2-design.md`; results in `docs/results/2026-11-phase0-1.md` and `docs/results/2026-11-phase2.md`.
Brief: `docs/briefs/Diff-IK Motion Service PoC — Project Brief.md`, Phase 3.

## Goal

Make the servo loop behave well on arms that can't reach an arbitrary 6-DOF pose, starting with the 5-DOF SO-101. Validate it against the so-101 module's `devrel:so101:simulated` arm before any hardware run.

## Where this fits

The brief's Phase 3 is split into three sub-projects, each with its own spec, plan and results:

| Sub-project | Scope |
|---|---|
| **3a (this spec)** | The pointing task with position priority, a rank-tolerant solver, and weighted pose tasks, validated on `devrel:so101:simulated`, which is kinematic only |
| 3b | An SO-101 model in `viam-mujoco-sim`, for physics and streaming lag. An optional middle step. |
| 3c | The real SO-101 (`devrel:so101:arm`, Feetech): the Phase 1 metrics, bus rate, `ResyncDeg` on real lag, and the brief's teleop and AprilTag runs |

## SO-101 kinematics: what the task must respect

The Phase 3a spec review checked these numerically, by loading `so101.urdf` and using diffik's `FK` and `Jacobian`:

- **The tool axis is the flange z axis.** That is Viam's orientation-vector direction (`o_x, o_y, o_z`), and the so-101 module's approach-axis cone constrains the same axis (`internal/planning/goal_cloud.go`).
- **Wrist roll (joint 5) spins about the tool axis**, and the TCP sits on that axis. Joint 5's Jacobian column is `[0, 0, 0, −a_c]`: it changes neither the position nor the pointing.
- **Shoulder lift, elbow and wrist flex (joints 2–4) have parallel axes.** The tool axis therefore always lies in the vertical plane through the base, and its azimuth is fixed by the base pan (joint 1).

So **position plus pointing is a rank-4 task on the SO-101**, not rank 5. For a given position, only the tool's pitch within the arm plane is free. Its azimuth isn't free. Arbitrary pointing targets are unreachable, and any design that treats the 5 rows as one square system gets σmin ≈ 0 (measured ≈ 6e-7) everywhere: permanent maximum damping, and a position/pointing compromise set by units.

**Decision: position has priority.** Position is tracked as the primary task. Pointing is pursued only with the freedom left over (null-space projection), and the part it can't reach is reported as a residual. This is arm-agnostic. On a 6-DOF arm (the UR5e), position plus pointing is reachable (rank 5, with roll redundant), so pointing converges fully.

**Singular configurations** come from the elbow, not the wrist. σmin of the SO-101 pose Jacobian depends on joint 3: about 0.011 at −1.5 rad, 0.061 at 0, 0.078 at 0.5 and 0.031 at 1.5. Joint 4 doesn't affect it.

## Decisions

| Topic | Decision |
|---|---|
| Sim target | `devrel:so101:simulated` (and `simulated-gripper`) from the so-101 module, run on a local viam-server. A MuJoCo SO-101 is 3b. |
| Task styles | A **pointing** task with position priority, and a **pose** task with per-row weights (zero drops the row) |
| Default task | `pointing` when the arm has fewer than 6 DOF, `pose` otherwise |
| Solver | Pose with m ≥ n keeps today's n×n weighted DLS **unchanged**. Everything else uses an SVD-based damped pseudo-inverse with a rank tolerance. Pointing uses a two-level priority solve. |
| Compatibility | A UR5e in pose mode with all weights 1 takes the unchanged code path, so its joint commands are bit-identical to `main`, enforced by a golden test |
| Damping knobs | `singular_eps` and `max_damping` become `servo_start` limits and CLI flags. The plan measures and sets SO-101 values, because the UR5e's 0.04 sits inside the SO-101's normal σ range (0.01–0.08). |

## Damped pseudo-inverse with rank tolerance

`dpinv(A, λ)` for any m×n matrix `A`:
- Take the SVD `A = Σ σᵢ uᵢ vᵢᵀ`.
- Keep the `σᵢ > max(1e-4 · σmax, 1e-5)`. The absolute 1e-5 floor stops the structural zero (≈2e-6) from being retained when σmax itself collapses near the elbow singularity, which would flip the reachable/residual split. These form the effective rank r. Dropped directions are true null directions, such as SO-101 roll for position and pointing. The tolerance is 1e-4, not tighter, because both `so101.urdf` and `so101.json` write angles as rounded constants (`3.14159`, `1.5708`). The axes are therefore parallel or perpendicular only to about 3e-6 rad, and the structurally zero σ of `J2·N1` measures about 4e-6·σmax. At 1e-6 that σ is retained: λ2 is pinned at its maximum, the whole pointing error looks reachable, and unreachable targets never converge (reproduced in the spec review).
- `dpinv(A, λ) = Σᵢ≤r σᵢ/(σᵢ² + λ²) · vᵢ uᵢᵀ`.
- **Chiaverini damping** on `σr`, the smallest *retained* singular value: `λ² = λmax²(1 − (σr/ε)²)` when `σr < ε`, else 0.
- An all-zero `A` gives r = 0 and `dpinv = 0`.

A null direction no longer pins λ at its maximum. Damping reacts only to directions that exist and are getting weak.

## Task modes and the per-tick solve

**Pose mode**
- The rows are today's 6: `[Δp; log(R_t·R_cᵀ)]` with `J = [J_lin; J_ang]`. A row with weight 0 is dropped (m ≤ 6).
- **m ≥ n** (the UR5e with all weights, or the SO-101 pose task at 6×5): today's `dlsStep` n×n code, unchanged, with damping on σmin of `W½J` as today. Its σmin is over min(m, n) values, so it is well-defined.
- **m < n** (dropped rows on a 6-DOF arm, or redundant arms): `Δq = dpinv(W½J, λ) · W½Δx`.

**Pointing mode**
- **Primary (position):** `J1 = J_lin` (3×n) and `e1 = Δp`.
- **Secondary (pointing):** `a_c` is the current tool axis and `a_t` the target's (`o_x, o_y, o_z` of the target pose; its `theta` is ignored).
  - `r = angle(a_c, a_t) · unit(a_c × a_t)`, a base-frame rotation vector perpendicular to `a_c`, so `exp(r)·a_c = a_t`. When the axes are antiparallel (`|a_c × a_t| < 1e-9`, `a_c·a_t < 0`), `r = π·u`.
  - `u, v` are an orthonormal pair perpendicular to `a_c`: `u = unit(a_c × ẑ)`, falling back to `unit(a_c × x̂)` when `|a_c × ẑ| < 1e-6`, and `v = a_c × u`.
  - `J2 = [uᵀJ_ang; vᵀJ_ang]` (2×n), and `e2 = [u·r; v·r]`.
  - Both rows are weighted equally, so the solve is invariant to the choice of the `u, v` basis within the plane. The basis fallback when the tool is vertical, which is common for an SO-101 pointing down, therefore can't cause a jump.
- **Priority solve:**
  - `Δx1 = v1·dt` and `Δx2 = v2·dt`, where `v1 = vel_lin + Gain·e1`, capped at `MaxLinearMMPerS`, and `v2 = (u·ω_ff, v·ω_ff) + Gain·e2`, capped at `MaxAngularDegPerS`.
  - `Δq1 = dpinv(J1, λ1)·Δx1`.
  - `N1 = I − pinv(J1)·J1`, the undamped null projector with the same rank tolerance (1e-4).
  - `Δq = Δq1 + dpinv(J2·N1, λ2)·(Δx2 − J2·Δq1)`.
  - Each level gets its own Chiaverini λ from its own matrix.
  - Then joint-velocity scaling, position limits and the send, as today. Scaling is uniform, so the priority structure is kept.
- **Achievable pointing:** `e2_reach = (J2N1)(J2N1)⁺·e2` is the part of the pointing error the null space can still reduce, and `e2_res = e2 − e2_reach` is the part it can't. On the SO-101, the residual is the azimuth mismatch.
- **Roll:** feedforward and twists about the tool axis are discarded in pointing mode.

## Loop changes

- `LoopConfig.Task` is `"pose"` or `"pointing"`. Empty means the DOF-based default.
- **Errors at `q_cmd` and `q_m`** use the same task rows as the solve, at FK(`q_cmd`) and FK(`q_m`).
- **Status fields:**
  - `ErrMM` / `CmdErrMM`: the norm of the active position components (only the non-dropped rows in pose mode).
  - `ErrDeg` / `CmdErrDeg`:
    - pointing mode: the pointing angle, `|e2|` in degrees;
    - pose mode: the norm of the active angular components, which is the full orientation angle when all three are active.
  - New `PointingResidualDeg` (pointing mode): `|e2_res|` at `q_m`, in degrees.
  - New `Task`.
- **Convergence:**
  - Pose mode: active position components ≤ `TolMM` and active angular components ≤ `TolDeg`, plus the existing zero-feedforward condition.
  - Pointing mode: position ≤ `TolMM` and `|e2_reach|` ≤ `TolDeg`. A target whose pointing is unreachable still converges once position is reached and pointing is as good as the arm allows. The residual stays visible in status.
- **Leash:** uses task-space errors. The linear leash uses the position error; the angular leash uses `|e2_reach|` (pointing), so an unreachable residual never blocks advancing along the reachable direction, or the active angular norm (pose). Roll never trips it.
- **Stall:** unchanged in rule and still requires `limited`. Its norm is the weighted active-row norm (pose) or the position norm plus `|e2_reach|` (pointing). A pose target with an unreachable roll is *not* required to stall. It simply doesn't converge, as on any arm whose least-squares equilibrium isn't at a limit.
- **`limited`:** additionally true when either priority level's λ > 0.
- **Validation:**
  - `task_weights` may contain zeros: 6 values ≥ 0, not all zero. `module.go` rejects all-zero *before* `withDefaults`, where "absent" still means all ones.
  - In pointing mode the weights are ignored, because priority replaces weighting; a warning is logged if non-default weights are given.
  - `SingularEps` and `MaxDamping` must be > 0, as today.

## Surface changes

- **`servo_start`:**
  - `task` (`"pose"` or `"pointing"`, default by DOF). An unsupported value returns an error.
  - `task_weights`: 6 values ≥ 0, not all zero.
  - `limits.singular_eps` and `limits.max_damping`: each > 0, with the loop's defaults when absent.
- **`servo_status`:** `task` and `pointing_residual_deg` (pointing mode only), with `err_mm` and `err_deg` as defined above. `limits` also reports `singular_eps` and `max_damping`.
- **`servo_twist` in pointing mode:** reports `"roll_ignored": true` when the angular twist has a non-zero component along the current tool axis. The axis is read from the loop's last FK(`q_cmd`) under the loop mutex; the session doesn't compute FK itself.
- **`cmd/cli`:**
  - `--task`, `--singular-eps` and `--max-damping` for `run`.
  - **`static` in pointing mode** builds *reachable* targets: FK of the start joints perturbed by a seeded random joint offset (up to about 10° per joint), keeping only targets whose position is 20–60 mm away. Pose mode keeps today's generator.
  - **`singular`** gains `--singular-deg`, the joint angle the scenario sweeps through. The UR5e default stays at 0°. For the SO-101, the plan picks the elbow angle with the smallest σmin from the URDF: about −1.29 rad (−74°), measured in plan review. An earlier coarse sample put it at −1.5 rad, but ±0.2 rad around that exceeds the ±1.69 rad limit. Use `--wrist 2`. The flag keeps its name for compatibility; its help text says "singular joint index".
- **Python bench:** `--task` is passed through to `servo_start`. The circle in pointing mode keeps the start orientation, so the reported `pointing_residual_deg` comes from geometry (azimuth changes as the target moves around the base), not from lag. The write-up says so.

## Test setup (no cloud changes)

`dev/so101-sim.json`, checked in, is a local viam-server config with no `cloud` section, so viam-server binds plaintext `localhost:8080` (`config.DefaultBindAddress`). It contains:
- **Modules:**
  - `so-101`, local, `~/src/so-101/bin/arm`, built with `make` in that repo;
  - `diff-ik`, local, `bin/diff-ik`.
- **Components:**
  - `arm`: `devrel:so101:simulated`, `speed_degs_per_sec: 90`. Keep this at or above the loop's joint limits, since the sim moves the slowest joint at this speed.
  - `gripper`: `devrel:so101:simulated-gripper`, framed on `arm`. The servo doesn't drive it.
- **Service:** `servo`: `devrel:diff-ik:servo`, `arms: ["arm"]`.
- **No motion service is needed:** the simulated arm's motion dependency is satisfied by RDK's default motion service.

To run it: `viam-server -config dev/so101-sim.json`. The CLI and the bench then connect with `--address localhost:8080 --insecure`. The paths in the file are absolute for this laptop, and the README says how to change them.

The simulated arm's kinematics come from `so101.json` (`use_urdf` defaults to false). The review confirmed that its geometry, TCP and limits match `so101.urdf`. The simulated arm also lags `q_cmd` by about one tick (it waits for each point's scheduled time), which is well under the default `ResyncDeg`.

## Testing (Go)

- **`testdata/so101.urdf`:** a copy of `so-101/assets/urdf/so101.urdf`, with its license next to it, loaded with `referenceframe.ParseModelXMLFile(path, "arm", nil)` to give the 5-DOF model with no server.
- **`dpinv` tests (`kin_test.go`):**
  - full-rank square input matches the inverse;
  - a tall full-column-rank matrix matches `(AᵀA)⁻¹Aᵀ`;
  - wide input matches `Aᵀ(AAᵀ)⁻¹` at λ = 0;
  - a rank-deficient matrix (the SO-101 5×5 pointing stack) yields the minimum-norm solution with no blow-up, and λ = 0 when the retained σr ≥ ε;
  - at λ > 0, the m < n pose form cross-checks against the n×n form via the push-through identity.
- **`task_test.go`:**
  - pointing rows are zero when the axes match;
  - a known tilt gives the exact `r`;
  - the antiparallel case gives |r| = π;
  - the `u` fallback works when the tool is vertical;
  - the solve is unchanged when `u, v` are rotated within the plane;
  - zero-weight pose rows are dropped (m = 5 and m = 4);
  - roll feedforward is dropped;
  - **finite-difference check:** freeze `u, v` and set `a_t = a_c(q0)` at the base configuration q0, then compare `[u·r; v·r]` at `q0 ± δ` against `−[uᵀ; vᵀ]·J_ang` to within 5%. The error is target − current, hence the sign.
- **`loop_test.go`, on the SO-101 model with `simIO`:**
  - **reachable pointing target** (FK of perturbed joints): converges with position ≤ 2 mm and pointing ≤ 2°. The residual stays ≤ 1°, not ≈ 0: a perturbed-joint target still reaches slightly through the near-null direction (measured 0.3–0.4°).
  - **unreachable pointing target** (the same position, with the axis rotated 20° about base z): reaches final state `converged`, with position ≤ 2 mm, `|e2_reach|` ≤ 2°, and `PointingResidualDeg` equal to the angle between `a_t` and the arm plane, which the test computes (±1°). This is `asin(sin20°·|a_c,xy|)`, not 20° unless the tool is horizontal. Assert the final state, not a tick count; such a target can count as converged on its first tick.
  - **priority holds:** for the unreachable target, the converged position error is no worse than when the same position is targeted alone;
  - **damping:** at mid-workspace configurations, **both λ1 and λ2 are 0** with the SO-101 model. This assertion is what catches a rank tolerance that is too tight. The null roll direction doesn't trigger damping.
  - **pose mode with an unreachable roll:** doesn't converge, and the same target in pointing mode does;
  - **pointing-mode twist** (linear only): tracks with < 1 mm error;
  - the leash ignores roll.
- **UR5e:**
  - pointing mode converges fully, with residual < 0.5°;
  - **compatibility golden test:** before any change, record the UR5e pose-mode joint commands for the Phase 1 converge, singular and twist scenarios at current `main` into `testdata/golden_ur5e.json`. Afterwards, assert bit-identical commands. Record and check on the same architecture. If a gonum assembly or FMA difference appears across machines, fall back to 1e-12 and note it.
- **`module_test.go`:**
  - `task` parsing and the default by DOF (a 5-DOF fake arm gives pointing, the UR5e gives pose);
  - zero weights accepted, all-zero rejected, absent means ones;
  - `singular_eps` and `max_damping` parsing, bounds and status echo;
  - `roll_ignored` reported.

All existing tests must keep passing under `go test -race ./...`.

## Sim validation (results in `docs/results/2026-11-phase3a.md`)

These run against the local sim, starting from a mid-workspace SO-101 pose that the plan chooses from the joint limits, with SO-101 damping values the plan measures.

| Run | Bar |
|---|---|
| `cli validate` | Informational only. The simulated arm's `EndPosition` and its kinematics come from the same model, so both the FK and the Jacobian checks are circular here. They become binding in 3c. |
| `cli run static --task pointing` | 10/10 reachable targets converge with position ≤ 2 mm and pointing ≤ 2°; ≥ 49.5 Hz |
| `cli run singular --task pointing` (elbow) | Reaches the target, joint velocities ≤ 1.1 × limit, no oscillation |
| `cli run circle --task pointing` | Baseline: position error, and `pointing_residual_deg`, which comes from geometry |
| Python bench `--task pointing` (smoke, dead-man, circle) | Smoke passes and the dead-man bar holds, as in Phase 2. The circle is a baseline. |
| Unreachable-pointing demo (bench step or CLI) | Position converges, and the residual is reported |

## Out of scope and open items

- 3b (MuJoCo SO-101) and 3c (hardware).
- A real ramp-down before `arm.Stop` (a Phase 2 open item); hardware makes it more pressing.
- Task frames other than the base. A "look-at" target, pointing the tool at a point, is a likely visual-servoing follow-up. With position priority it fits as a different secondary task.
- More than two priority levels, and joint-limit avoidance as a third level.
