# Diff-IK Servo — Phases 0 & 1 Design

Date: 2026-11-15 · Source brief: `docs/briefs/Diff-IK Motion Service PoC — Project Brief.md`

## Goal

Prove the differential-IK servo loop works before any API surface exists:

- **Phase 0:** FK built from an arm's `Kinematics()` matches RDK, and a finite-difference Jacobian predicts small motions.
- **Phase 1:** a co-located loop holds ≥ 50 Hz against the MuJoCo sim arm, converges on static targets, stays within limits near singularities, and tracks a moving target.

Out of scope here: DoCommand protocol, sessions, dead-man timeout, twist mode, camera-frame targets (Phase 2+); SO-101 hardware (Phase 3); builtin `Move` comparison (Phase 4).

## Decisions

| Question | Decision |
|---|---|
| Target arm | MuJoCo sim arm, which already supports streamed / non-blocking joint moves. |
| Closed vs open loop | **Hybrid.** Integrate from commanded state `q_cmd`; read measured `q_m` every tick; resync `q_cmd = q_m` when they diverge past a threshold. Reported error always uses measured FK. |
| Harness | `cmd/cli` on the laptop running the sim (localhost gRPC ≈ 1 ms), plus pure `go test` on the math against an in-memory arm. |
| Solver | Weighted damped least squares on a finite-difference Jacobian. FK from RDK's `referenceframe.Model`; matrix math from `gonum/mat` (already an RDK dependency). |

Rejected: per-tick QP (no good small dense QP solver in Go; too heavy for a PoC); per-tick RDK IK solver (non-smooth, no velocity semantics, unpredictable tick time — kept as a Phase 4 comparison point only).

## Components

All new code lives in the root `diffik` package.

| File | Role |
|---|---|
| `module.go` (edit) | Fix compile errors: missing `fmt`, unqualified `TempStreamResponse`, and ~9 unused imports. Add `require go.viam.com/rdk` (pinned) to `go.mod`, which currently has none. All `motion.Service` methods stay stubs returning `errUnimplemented`. |
| `kin.go` | Model from arm (`arm.Kinematics`; return type confirmed against the pinned RDK), `fk(q) → (p [3]m, R)`, `jacobian(q) → 6×n`, `poseError(target, current) → [6]float64`. Conventions below. |
| `loop.go` | `Loop`, `LoopConfig`, `Status`, `JointIO`. `SetTarget` takes a pose in mm, base frame. |
| `armio.go` | `armIO`, the `JointIO` adapter over `arm.Arm` (`JointPositions`, `MoveThroughJointPositionsStreamed`, `Stop`). |
| `kin_test.go`, `loop_test.go` | `go test` with no sim; includes `simIO`, a perfect in-memory arm. |
| `cmd/cli/main.go` (edit) | `validate` (Phase 0) and `run <scenario>` (Phase 1). |

### Kinematics conventions

Exported API: `JointIO`, `FK`, `RotVec`, `PoseError`, `Jacobian`, `Oscillations`, `NewArmIO`, `Loop.Config`, `Loop.OnTick`.

Everything is expressed in the **arm's base frame**, in **meters and radians**.

- `fk` converts `Model.Transform` output from mm to m. This is the only place that conversion happens.
- `poseError`: position is `p_t − p_c`; orientation is `log(R_t·R_cᵀ)`, the base-frame rotation vector.
- Jacobian, one column per joint, central differences with `h = 1e-6 rad`:
  - linear rows: `(p(q+hеᵢ) − p(q−hеᵢ)) / 2h`
  - angular rows: `log(R(q+hеᵢ)·R(q−hеᵢ)ᵀ) / 2h`. Never difference orientation-vector or Euler components.
- Jacobian accuracy metric: `‖Δfk − J·Δq‖ / ‖Δfk‖` with `|Δq| = 1°`, perturbing one joint at a time (perturbing all at once lets parallel axes cancel), computed separately for the linear and angular parts.
- With `W = I`, 1 rad of orientation error weighs the same as 1 m of position. At 5 cm / 10° targets the orientation term dominates. That's acceptable for Phase 1; `TaskWeights` is the knob for changing it.

### Loop API

```go
type JointIO interface {
    Read(ctx context.Context) ([]referenceframe.Input, error)
    Send(ctx context.Context, q []referenceframe.Input) error
    Stop(ctx context.Context) error
}

func NewLoop(model referenceframe.Model, io JointIO, cfg LoopConfig) (*Loop, error)
func (l *Loop) SetTarget(p spatialmath.Pose) // latest wins, goroutine-safe
func (l *Loop) Run(ctx context.Context) error // ticks at cfg.RateHz until ctx done or fault
func (l *Loop) Status() Status                // goroutine-safe snapshot
```

`JointIO` has two real implementations: `armIO` (wraps `arm.Arm`) and `simIO` (tests). `armIO.Send` uses `MoveThroughJointPositionsStreamed` with time-stamped `TrajectoryPoint` batches.

### LoopConfig (all calibration knobs, all defaulted)

| Field | Default | Meaning |
|---|---|---|
| `RateHz` | 50 | Tick rate. |
| `TaskWeights` | `[1,1,1,1,1,1]` | Diagonal `W` over (x, y, z, rx, ry, rz) in m / rad. Lower orientation weights for 5-DOF arms later. |
| `Gain` | 2 /s | Task-space proportional gain. |
| `MaxLinearMMPerS` | 100 | Cap on task linear speed. |
| `MaxAngularDegPerS` | 45 | Cap on task angular speed. |
| `MaxJointDegPerS` | `[]float64`, 90 each | Per-joint velocity limit (kinematics carry no velocity limits). Length must equal DOF. |
| `SingularEps` (ε) | 0.04 | `σmin` threshold where damping starts. |
| `MaxDamping` (λmax) | 0.1 | Damping at `σmin = 0`. |
| `ResyncDeg` | 3 × max(`MaxJointDegPerS`)·dt (5.4° at defaults) | Max per-joint `q_cmd`/`q_m` divergence before resync. Sized to allow ~3 ticks of streaming lag so normal fast moves don't resync constantly. Watch the resync count in `run static` and tune. |
| `TolMM`, `TolDeg` | 2, 2 | Convergence tolerance. |

Joint position limits come from `model.DoF()`, inset by the finite-difference step.

Validation: `RateHz ≥ 1`; every scalar knob and every joint limit must be > 0; `TaskWeights` is a `[6]float64` (no length check).

## Per-tick data flow

Internal units are meters and radians; mm/deg only at the edges.

```
 1. q_m = io.Read()
 2. if max|q_cmd − q_m| > ResyncDeg: q_cmd = q_m; resyncs++
 3. e  = poseError(target, fk(q_cmd))
 4. v  = Gain·e; scale v.lin to ≤ MaxLinear, v.ang to ≤ MaxAngular (direction kept)
 5. Δx = v·dt
 6. J  = jacobian(q_cmd)
    σmin = smallest singular value of W^½·J
    λ² = 0 if σmin ≥ ε else λmax²·(1 − (σmin/ε)²)
    Δq = (JᵀWJ + λ²I)⁻¹ JᵀW Δx
 7. s = min(1, minᵢ(MaxJointᵢ·dt / |Δqᵢ|)); Δq *= s
 8. q_next = clamp(q_cmd + Δq, joint limits)
 9. io.Send(q_next); q_cmd = q_next
10. errMeasured = poseError(target, fk(q_m)) → Status in mm / deg
```

On the first tick `q_cmd` is initialized from `q_m`. With no target set, the loop reads but does not send.

### States

| State | Condition |
|---|---|
| `idle` | No target set. |
| `running` | Target set, not converged. |
| `converged` | Measured error ≤ `TolMM` and `TolDeg`. Loop keeps ticking; a moved target returns it to `running`. |
| `stalled` | Step was limited (s < 1 in step 7, a joint clamped in step 8, or σmin < ε) and measured error has not dropped by 1% over the last 1 s. |
| `faulted` | `Read`/`Send` error or non-finite `Δq`. Terminal. |

### Timing and Status

`time.Ticker` at `RateHz`; missed ticks are dropped, not replayed. Each tick records its total time (not a compute/IO split); a tick longer than `dt` is an overrun. `Status` reports: state, `TargetMM`, measured error (mm, deg), commanded error (mm, deg), achieved rate, `LastTick`, p50/p99 tick time, overrun count, resync count, current λ, whether the last step was limited, and `QCmd` / `QMeas`.

## Error handling and stop

| Situation | Behavior |
|---|---|
| `Read` / `Send` error | `faulted` → `io.Stop` → `Run` returns the error after `Stop` returns. |
| Non-finite `Δq` | Same as above. |
| ctx canceled | `io.Stop` on a fresh 2 s-timeout context, then `Run` returns `ctx.Err()`. Arm is stopped before return (same contract as `TempStreamArmJointPositions`). |
| Model DOF ≠ `JointPositions` length | `NewLoop` (or first `Read`) fails before any `Send`. |
| `Kinematics()` returns an error | Construction fails with that error. |
| `armIO.Stop` | Sends `arm.Stop` before waiting for the stream to end. |
| CLI command returns an error | The arm is stopped. |
| Invalid config | `NewLoop` errors: `RateHz < 1`, any scalar knob or joint limit ≤ 0, `len(MaxJointDegPerS)` ≠ DOF. |

Phase 1 stop is `arm.Stop` directly; velocity ramp-down arrives with the Phase 2 dead-man timeout.

## Testing

### `go test` (no sim)

Fixture: a UR5e model from RDK's bundled kinematics files (exact path confirmed during planning).

1. `poseError` is zero for identical poses and has the correct sign for a known offset.
2. Jacobian predicts the FK change of a small random `Δq` within 5% (Phase 0 criterion, in-process).
3. From 5 cm / 10° off, loop with `simIO` converges to ≤ 2 mm / 2° within 3 s of simulated ticks.
4. Through a wrist singularity (q5 = 0): no joint step exceeds `MaxJoint·dt`, and there's no oscillation. An **oscillation** is a sign reversal of a joint's `Δq` where both steps exceed 10% of `MaxJoint·dt` (this ignores jitter near rest). Pass: ≤ 2 oscillations per joint per second.
5. `simIO` with injected lag triggers a resync and the loop still converges.
6. Context cancel calls `Stop` before `Run` returns.

Tests drive ticks directly (a `tick(dt)` method) instead of real time.

### `cmd/cli` against MuJoCo on localhost

The scaffold CLI has no machine connection, so this is new work. Flags: `--address` (default `localhost:8080`), `--insecure` (plaintext dial, for a local server), `--arm` (component name), `--rate`, `--max-joint-deg-s` (0 = loop default), `--resync-deg` (0 = loop default; the sim's ~240 ms streaming lag needs ~15), `--csv`, `--spread` (default 0.1; `validate` samples ± spread × half-range around the start pose), `--wrist` (singular's wrist joint index). Credentials come from `VIAM_API_KEY_ID` / `VIAM_API_KEY` env vars. It connects with `client.New`, gets the arm, builds the model, wraps the arm in `armIO`, and runs the scenario. Prints a summary table; `--csv path` writes a header row and one row per tick (elapsed, state, target, errors, λ, limited, `tick_ms`, q_cmd, q_m) for Phase 4 smoothness/jerk analysis.

| Command | Does | Pass criteria |
|---|---|---|
| `validate` | Drives to 100 random configs within ± `--spread` × half-range of each joint around the start pose. Compares our FK to `arm.EndPosition`; applies a small `Δq` and compares actual pose change to `J·Δq`. | FK ≤ 0.1 mm / 0.1°; Jacobian prediction within 5% (linear check skipped when the 1° nudge moves the flange < 0.1 mm, e.g. flange roll, where model offset dominates). |
| `run static` | 10 targets, 5 cm and ≤ 10° from start. | Converge to ≤ 2 mm / 2°; report time to converge. |
| `run singular` | Targets that force the wrist through singularity. | Must reach the target. Peak measured-joint velocity (over a 10-tick window, since rows are stamped at tick start but read mid-tick; 10% tolerance for PD servo catch-up) ≤ limit × 1.1; ≤ 2 oscillations per joint per second on measured joints, as defined in test 4. Report both. |
| `run circle` | 5 cm-radius circle at 5 cm/s, 2 laps. | Record mean/max tracking error and time lag as the baseline. |

All `run` scenarios report achieved rate (the command fails below 49.5 Hz: 1% tolerance for ticker jitter in the measured average), p50/p99 tick, overruns, and resyncs.

## Open items carried forward

- ~~Streamed-move signature~~ Resolved: `MoveThroughJointPositionsStreamed` with time-stamped `TrajectoryPoint` batches; the sim supports it.
- ~~Is the `validate` FK check circular?~~ Resolved: not circular. The MuJoCo sim's `EndPosition` reads MuJoCo's `flange_site` pose, not the `Kinematics` model. But the sim only requires the MJCF and kinematics to agree within 5 mm / 2° (≈ 1 mm Menagerie vs vendor link-length mismatch on the UR5e), so the 0.1 mm / 0.1° FK bar measures model mismatch and is expected to fail on the sim. Treat it as informational on the sim and binding on the SO-101; the Jacobian (relative) check remains meaningful.
- `dlsStep` with > 6 DOF or a zero task weight makes `JᵀWJ` singular when λ = 0 and pins σmin at 0. Revisit before 5-DOF/SO-101 work (Phase 3).
- How the servo API proposal relates to the existing `TempStreamArmJointPositions` motion method (Phase 4 write-up).
- Camera-frame targets for visual servoing (Phase 2).
