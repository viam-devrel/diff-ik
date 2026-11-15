# Diff-IK Phase 2 — DoCommand Servo Surface: Design

Date: 2026-11-15 · Builds on: `2026-11-15-diff-ik-phase0-1-design.md`, results in `docs/results/2026-11-phase0-1.md`
Brief: `docs/briefs/Diff-IK Motion Service PoC — Project Brief.md`, Phase 2.

## Goal

Expose the Phase 1 servo loop through the module's `motion.Service.DoCommand` as five commands (`servo_start`, `servo_target`, `servo_twist`, `servo_stop`, `servo_status`) with latest-wins targets and a dead-man timeout. Add velocity feedforward so moving targets track without the P-loop's v/K lag that Phase 1 measured (~25 mm at 50 mm/s).

**Brief exit criteria:**
- All five commands work from the Python and TypeScript SDKs.
- Command-to-motion latency and stop latency are measured.
- The dead-man timeout stops the arm within one loop period plus ramp time.

**Added by this design:**
- Feedforward in both pose and twist modes.
- Circle tracking re-measured against the Phase 1 baseline.

## Decisions

| Topic | Decision |
|---|---|
| Scope | The brief's Phase 2, plus loop velocity feedforward. `ResyncDeg` is exposed as a `servo_start` limit. |
| Frames | `world` (the default) or `<arm>_origin`, the arm's base frame in Viam naming. The bare `<arm>` frame is the end effector, so it is rejected. The world→base transform is read once at `servo_start` and treated as static, which assumes a fixed-base arm. Other frames are rejected. Per-tick frames for visual servoing are deferred. |
| Clients | A Python bench script takes every measurement. A TypeScript smoke script runs the same command sequence. |
| Structure | A session layer sits over the existing loop (approach A). The loop gains one motion entry point and a ramp-stop. `module.go` only parses, converts units and dispatches. |
| Units | The module boundary uses mm, degrees and Viam orientation vectors (degrees). Inside, poses are `spatialmath.Pose` in **mm**, as in Viam and the existing `Loop.SetTarget`. Velocities (`vel`, `v̂`, twists) are **SI**: m/s and rad/s, matching the loop's internal error. `module.go` converts degrees→radians and mm/s→m/s for twists; it passes poses through in mm. |
| Clock | `Loop` gets a `Now func() time.Time` field (default `time.Now`) used by the extrapolation hold and the ramp. The session uses the same function. The dead-man check runs in the session's `OnTick` callback, once per loop tick, so there is no separate watchdog goroutine. Unit tests drive the loop package-internally with `l.tick(ctx, dt)` plus `OnTick` under a fake clock, without starting `Run`. One integration test runs the real `Run` with the real clock. |
| Model | The session builds the kinematics model with `arm.Kinematics(ctx)` at `servo_start`, as the CLI does. |

## Architecture

```
client ──DoCommand──▶ module.go   parse/validate, mm/deg ⇄ m/rad, dispatch
                         │
                         ▼
                    session.go    one per arm:
                     • latest-wins target (pose, stamp) / twist slot
                     • world→base transform (read once at start)
                     • target-velocity estimator (pose mode)
                     • dead-man watchdog → loop.RampStop
                         │  SetMotion(pose, vel)
                         ▼
                    loop.go       v = vel + Gain·e → caps → DLS → limits
                         │
                         ▼
                    armio.go ──MoveThroughJointPositionsStreamed──▶ arm
```

### Components

| File | Responsibility |
|---|---|
| `module.go` | `Config{Arms []string}`. `Validate` declares each arm as a dependency and rejects an empty or duplicate `arms`. The frame system needs no declaration: the module manager always injects it, so `framesystem.FromDependencies(deps)` works without it (rdk v1.16 `module/resources.go:291`). `DoCommand` dispatches on `"command"` and converts units. `Move`, `StopPlan` and `Close` interact with sessions as described under Semantics. Other motion methods stay `unimplemented`. Removes the unused scaffold fields `cancelCtx` and the duplicated `resource.Named`. |
| `session.go` | `Session`: owns a `Loop`, its `armIO`, the run goroutine and its cancel, the mode, the world→base transform, the estimator state, the dead-man watchdog, and the final status once ended. Holds the sessions map keyed by arm name. Gets `now func() time.Time`, injectable for tests. |
| `loop.go` | Adds `SetMotion`, `SetVelocity`, `RampStop`, the `Now` clock field, the extrapolation hold, and the `stopped` / `stopped_timeout` states. `ResyncDeg` already exists in `LoopConfig`. |
| `clients/python/servo_bench.py` | Smoke test and measurements; see Measurements. |
| `clients/ts/smoke.ts` | The same five-command sequence over the TypeScript SDK; exits nonzero on any failure. |

## DoCommand protocol

Every request carries `"command"`. Poses are `{x, y, z, o_x, o_y, o_z, theta}`: mm, with the orientation vector's `theta` in degrees.

| Command | Payload (default) | Returns |
|---|---|---|
| `servo_start` | `component_name` (required, must be in `arms`); `frame` (`"world"`); `mode` (`"pose"` or `"twist"`, default `"pose"`); `rate_hz` (50); `limits`: {`joint_deg_s` (90; a number or a per-joint list), `linear_mm_s` (100), `angular_deg_s` (45), `resync_deg` (loop default)}; `task_weights` (6× 1); `timeout_ms` (500); `ramp_ms` (200) | `session_id` (random opaque string), `pose` (current flange pose in `frame`) |
| `servo_target` | `session_id`, `pose` | `state`, `err_mm`, `err_deg` |
| `servo_twist` | `session_id`, `linear` [mm/s ×3], `angular` [deg/s ×3], both expressed in `frame` | `state` |
| `servo_stop` | `session_id` | the same fields as `servo_status`, after the ramp completes |
| `servo_status` | `session_id` | `state`, `err_mm`, `err_deg`, `rate_hz`, `last_command_age_ms`, `limits` (the active values), `tick_p50_ms`, `tick_p99_ms`, `overruns`, `resyncs`, `damping` |

### Semantics

- **One session per arm.** `servo_start` on an arm with a running session returns `arm <name> busy`. `Move` on any configured arm while it has a running session returns the same error. `Move` is otherwise `unimplemented`.
- **Starting doesn't move the arm.** In both modes, `servo_start` reads the measured joints, computes their FK with `model.Transform`, which gives a `spatialmath.Pose` in mm, and calls `SetTarget(fk)`. The package's `FK()` returns metres, so don't use it here; the returned `pose` comes from the same `model.Transform` result. The loop therefore holds the current pose, until the first `servo_target` in pose mode or with zero twist in twist mode.
- **Latest wins.** Each `servo_target` or `servo_twist` overwrites the single slot. Nothing is queued.
- **Dead-man timeout.** If no `servo_target` or `servo_twist` arrives within `timeout_ms` of the last one, or of `servo_start`, the session calls `RampStop(ramp, stopped_timeout)`. This applies in both modes, so holding a pose requires the client to keep sending targets. `timeout_ms` must be in (0, 10000]; it cannot be disabled.
- **Fixed mode.** `servo_twist` on a pose session, or `servo_target` on a twist session, returns `wrong mode: session is <mode>`.
- **Stopping.** `servo_stop` → `RampStop(ramp, stopped)`, then waits for the loop to exit and returns the final status. `StopPlan` with `ComponentName` equal to a session's arm does the same. Module `Close` ramp-stops every session concurrently and waits.
- **Stopping sessions.** Once a stop has begun (dead-man fired, `servo_stop`, `StopPlan` or `Close`), `servo_target` and `servo_twist` return `session stopping: <final state>` instead of a silent success. The dead-man's decision to fire and a command's refresh of the dead-man clock are serialised, so a command either lands before the stop starts or is rejected. `servo_stop` during a dead-man ramp still waits for the ramp and upgrades the final state to `stopped`.
- **Ended sessions.** After a stop, timeout or fault, the session is ended. `servo_status` keeps returning its final status until the next `servo_start` on that arm replaces it. `servo_target`, `servo_twist` and `servo_stop` return `session ended: <state>`, followed by the loop's error when the session ended in a fault (for example `session ended: faulted: sending joints: …`). A session ended by context cancellation (Close's fallback) records `stopped`.
- **Errors** are returned as `DoCommand` errors and never move the arm:
  - unknown `command`;
  - unknown or replaced `session_id`;
  - an arm not in `arms`;
  - unsupported `frame` or `mode`;
  - missing required fields;
  - non-numeric, NaN or Inf values;
  - non-positive limits; `rate_hz` outside [1, 1000]; `ramp_ms` outside (0, 5000] (a stop waits for the ramp);
  - a `timeout_ms` out of range;
  - `task_weights` that aren't 6 values > 0 (zero weights pin σ_min at 0 in `dlsStep`, a known Phase 3 item);
  - a zero-length orientation vector;
  - a twist whose linear or angular magnitude exceeds the session's `linear_mm_s` / `angular_deg_s`. It is rejected, not clipped: the arm can't follow it.
- **States:** `idle`, `running`, `converged`, `stalled`, `stopped`, `stopped_timeout`, `faulted`.

### Frames

At `servo_start`, the session asks the frame-system service for the arm base's pose in `world`: `TransformPose` of the identity pose in frame `<arm>_origin` to `world`. That gives `T_wb`.

- **Inbound:** poses are mapped `T_wb⁻¹ · p`. Twists have their linear and angular vectors rotated by `R_wb⁻¹`, so the twist is about the base frame's axes at the flange point.
- **Outbound:** `servo_start`'s returned `pose` is mapped back with `T_wb`.
- If `frame` is `<arm>_origin`, `T_wb` is the identity.

## Loop changes

### `SetMotion(pose, vel)` and `SetVelocity(vel)`

`SetMotion` replaces direct target setting. `pose` is a base-frame `spatialmath.Pose` in mm; `vel` is a base-frame twist in m/s and rad/s. `SetTarget(p)` remains as `SetMotion(p, 0)`, so the Phase 1 CLI and tests are unchanged.

`SetVelocity(vel)` changes only `vel` and keeps the internally advanced target. `servo_twist` uses it. Twist mode is never subject to the extrapolation hold: a twist persists until the next `servo_twist` or the dead-man.

**Target leash** (both `SetMotion` extrapolation and `SetVelocity`). The leash blocks only an advance that would grow the commanded-side error `e_cmd` (target vs FK(`q_cmd`)), and it gates linear and angular separately:
- Skip the linear advance when `|e_cmd_lin| > 0.1 s × linear limit` (10 mm at defaults) **and** `vel_lin · e_cmd_lin > 0`.
- Skip the angular advance when `|e_cmd_ang| > 0.1 s × angular limit` (4.5°) **and** `vel_ang · e_cmd_ang > 0`.

A target held back by caps, joint limits or singularity damping never runs away. A reversed twist advances the target back toward the arm at once, so the arm can't park at the point where `Gain·e` cancels `vel`. Client-supplied `servo_target` poses are not leashed: the client chose them, and the loop just converges.

The leash and the extrapolation hold are independent; either one stops the advance. For `converged`, "hold active" does not include "leashed".

**Tick order.**
1. Read `q_m` and apply any resync to `q_cmd`. This happens outside `mu`, as today.
2. Compute FK(`q_cmd`).
3. Take `mu`. Apply the hold and leash checks against that FK, advance the stored target, copy it, and release `mu`.
4. Compute the error from the copy.

**Atomic advance.** The target advance is applied to the stored target under `mu`, in the same critical section that reads it at the start of the tick. A `SetMotion` landing mid-tick therefore replaces the target and is never overwritten by an advanced old one.

Each tick, before computing the error, the loop advances its internal target:
- position: `p += v·dt`;
- orientation: `R ← exp([ω]·dt)·R`.

The control law becomes `v = vel + Gain·e`. The linear and angular caps, weighted DLS, uniform joint-velocity scaling and limit clamp apply unchanged to the sum, so feedforward can never exceed the configured limits.

**Extrapolation hold** (`SetMotion` only). `SetMotion` records the interval since the previous call. The loop stops advancing the target, holding it in place, once `max(2 × interval, 100 ms)` has passed since the last `SetMotion`; if there is no previous call, it holds after 100 ms. The 100 ms floor keeps ordinary RPC jitter from freezing a 50 Hz stream. `vel` is still not added after the hold. This bounds how far a client that went quiet can drag the target before the dead-man fires.

**Convergence.** `converged` additionally requires `vel = 0` or (for `SetMotion`) the hold to be active. A moving target or a non-zero twist stays `running`. Stall detection is unchanged.

### `RampStop(ramp, final State)`

- Records the ramp start. On subsequent ticks, `v` is scaled by `s = max(0, 1 − elapsed/ramp)` after the caps.
- Target advancement and `vel` freeze immediately.
- On the first tick where `s = 0`, the loop calls `io.Stop`, sets the state to `final`, and `Run` returns `nil`.
- Calling `RampStop` again during a ramp keeps the earlier deadline but takes the newer final state only if it is `stopped`.
- The ramp-completion check runs before the `target == nil` early return in `tick`. A ramp started while idle therefore still calls `io.Stop`, sets `final`, and ends `Run`.
- During the ramp, the state keeps its normal value, typically `running`. There is no `stopping` state. It becomes `final` only when the ramp completes.
- `Run`'s context cancel keeps its Phase 1 behavior: an immediate `io.Stop`.

### ResyncDeg

`ResyncDeg` comes from `limits.resync_deg` when given. Phase 1 found that the sim's streaming lag is about 240 ms. With feedforward, `q_cmd` leads by roughly latency × joint speed, so the results report resyncs with and without `resync_deg`.

## Target-velocity estimator (pose mode)

On each `servo_target` received at server time `t_k`, with base-frame pose `p_k`:

```
Δt = t_k − t_{k−1}
if no previous target or Δt > 0.5 s:  v̂ = 0, seeded = false        (a jump, not motion)
elif Δt < 5 ms:                        keep v̂                       (too close to differentiate)
else:  raw = [ (p_k − p_{k−1})/Δt  (m/s),  log(R_k·R_{k−1}ᵀ)/Δt  (rad/s) ]
       if |raw_lin| < 1 mm/s and |raw_ang| < 0.5°/s:
           v̂ = 0, seeded = false                                    (a resent pose: stationary)
       elif |raw_lin| > 1.5 × linear limit or |raw_ang| > 1.5 × angular limit:
           v̂ = 0, seeded = false                                    (a step, not motion)
       elif not seeded: v̂ = raw, seeded = true                     (seed: no EMA ramp-up)
       else:            v̂ = 0.5·raw + 0.5·v̂                         (EMA, smooths RPC jitter)
SetMotion(p_k, v̂)
```

- A raw speed above 1.5× the limits is treated as a step, not motion, and is not capped.
  - A 20 mm step resent every 100 ms reads 200 mm/s, or above 150 mm/s with up to 30 ms of jitter, so it never extrapolates.
  - The 1.5× margin keeps targets 10 ms apart in a 50 mm/s stream (about 100 mm/s raw) from reading as a step.
  - The loop's caps still bound `v = v̂ + Gain·e`, so a `v̂` between 1× and 1.5× the limit is safe.
- A resent identical pose snaps `v̂` to exactly 0. A pose held by resending, which the dead-man makes the normal way to hold, therefore reaches `converged`.
- On a constant-velocity stream, `v̂` equals the true velocity from the second sample.
- When the hold starts, `v̂` is not reset; the next target after a gap over 0.5 s resets it.

## Measurements (`clients/python/servo_bench.py`)

The script connects with `RobotClient` using the same address, `--insecure` and API-key environment variables as the Go CLI, and takes `--service` and `--arm`. Each step prints metric, value, bar and PASS/FAIL, and the script exits nonzero on any FAIL.

1. **Smoke.** Runs all five commands in pose mode and then twist mode. Checks these error paths: wrong mode, busy arm, unknown session, unknown command, and a command on an ended session.
2. **Command-to-motion latency.** Ten 20 mm `servo_target` steps, alternating ±X, each from a converged state with targets resent every 100 ms to feed the dead-man.
   - A second client connection polls `JointPositions` as fast as it returns, stamping each sample.
   - Latency is the time from sending `servo_target` to the first sample where any joint differs from the pre-step sample by more than 0.05°.
   - The script reports the median and maximum, plus the `servo_target` round-trip median. There is no bar; this is a baseline.
3. **Stop latency.** A twist of 50 mm/s in +X, re-sent every 100 ms for 1 s, then `servo_stop`.
   - It reports the `servo_stop` round-trip, which includes the ramp, and the time from sending to the arm being stationary. Stationary means three consecutive polls with every joint within 0.05°.
   - It reports the distance travelled after the stop was sent. This is a baseline.
4. **Dead-man.** The same twist, re-sent for 1 s, then nothing.
   - It polls `servo_status` every 20 ms and measures the time from the last command to the state becoming `stopped_timeout`.
   - **Bar:** ≤ `timeout_ms + 2·(1000/rate_hz) + ramp_ms + 50 ms`. One tick is for detection and one for ramp completion; the 50 ms allows for status polling and RPC.
   - It also reports the time until the arm is stationary, which includes the sim's streaming lag. That figure has no bar. The results write-up must state that the bar measures the `stopped_timeout` transition, when `arm.Stop` is issued, rather than the brief's "arm stopped", because of that sim lag.
5. **Circle with feedforward.** In pose mode with `frame: "<arm>_origin"` (the base frame, the same frame `EndPosition` reports), it streams the Phase 1 circle (50 mm radius at 50 mm/s, 2 laps, about the start pose) at 50 Hz from the client.
   - It computes the error the same way as the Phase 1 circle: the commanded target versus the measured flange pose from `EndPosition`, sampled by the second client.
   - It reports the mean and max error, `resyncs` and `overruns` from `servo_status`, and the Phase 1 baseline for comparison. It runs once with the default `resync_deg` and once with `resync_deg: 15`.
   - **Bar** (`resync_deg: 15` run only): mean error ≤ 22.1 mm, which is 30% below Phase 1's 31.6 mm. The default-`resync_deg` run is report-only: feedforward makes `q_cmd` lead by about 240 ms × joint speed, so resyncs may get worse there.

`clients/ts/smoke.ts` repeats step 1 with the TypeScript SDK's `MotionClient.doCommand`.

Results go in `docs/results/2026-11-phase2.md`, in the same format as Phase 1.

## Deployment (sim)

1. `make` builds `bin/diff-ik`.
2. Add the module to the `shape-finder-vision-pick` machine as a local module:
   ```json
   {"type": "local", "name": "diff-ik", "executable_path": "<repo>/bin/diff-ik"}
   ```
3. Add the service:
   ```json
   {"name": "servo", "api": "rdk:service:motion", "model": "devrel:diff-ik:servo", "attributes": {"arms": ["pick-arm"]}}
   ```

This changes the user's cloud machine config, so the exact JSON is shown to the user and confirmed before it is written.

## Testing (Go)

**`session_test.go`** uses a fake clock and `simIO`, driving ticks manually as described under Clock:
- latest-wins: only the last of several queued targets is used;
- a busy arm is rejected;
- wrong-mode commands are rejected;
- an ended session keeps a readable status and rejects commands;
- the dead-man produces `stopped_timeout`, with `Stop` called within `timeout + dt + ramp`;
- the estimator equals a constant velocity to within 1% from the second sample;
- the estimator resets on a 20 mm step resent every 100 ms (raw speed over 1.5× the limit), including with Δt = 130 ms;
- a resent identical pose gives `v̂ = 0` exactly, and the session reaches `converged`;
- the estimator resets after a 0.5 s gap;
- twist mode: `servo_twist` keeps the internally advanced target, with no snap-back on repeated twists;
- an over-limit twist is rejected;
- leash: a twist driving a joint into its limit, then a reversed twist. The arm reverses within 5 ticks and keeps going: it travels more than 30 mm in reverse, with no parking where `Gain·e` cancels `vel`;
- the dead-man fires when started idle (no targets) and still reaches `stopped_timeout` with `Stop` called;
- the estimator ignores samples under 5 ms;
- frame mapping round-trips a pose through a non-trivial `T_wb`.

**`loop_test.go` additions:**
- a constant twist gives a steady-state measured error under 1 mm after 1 s, against simIO without lag;
- extrapolation holds after `max(2 × interval, 100 ms)`;
- `RampStop` scales `v` monotonically to 0 over the ramp, then calls `Stop`, with the final state set;
- `converged` is not reported while `vel ≠ 0`.

**`module_test.go`:**
- `Validate` returns the arm names as dependencies;
- `DoCommand` parses defaults;
- mm/deg ⇄ SI conversion of a pose round-trips;
- every listed error returns the expected message;
- `Move` on a busy arm errors;
- `StopPlan` stops the matching session.

The existing tests must keep passing, and the suite must pass under `go test -race ./...`.

## Out of scope / open items

- Arbitrary or per-tick frames such as camera frames, and mobile-base arms.
- A streaming RPC. The unary `DoCommand` latency measured here is the baseline that argues for one.
- Collision checking.
- Choosing the `ResyncDeg` policy for real hardware. That's Phase 3, informed by these results.
- `dlsStep` for more than 6 DOF or zero task weights, needed before the SO-101 in Phase 3.
