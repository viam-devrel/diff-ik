# Diff-IK Phase 3c, stage 1 — the real SO-101: Design

Date: 2026-11-15
Builds on: Phase 3a/3b specs, and results in `docs/results/2026-11-phase3a.md` and `2026-11-phase3b.md`.
Brief: Phase 3, "Phase 1 metrics met with the reduced 5-DOF task", plus the Feetech bus-rate risk.

## Goal

Run the Phase 1 metrics, plus the Phase 2 DoCommand checks, on the real SO-101 follower with the 5-DOF pointing task, safely. Measure what no sim could:
- bus timing;
- real streaming lag;
- settling and gravity sag;
- the driver's stop behaviour.

The results set the defaults for stage 2 (teleop) and stage 3 (AprilTag).

## Hardware and machine (as found)

- **Machine:** `laptop` (part `laptop-main`, org "HipsterBrown's World of Robots"). Its viam-server runs on this Mac (`viam-server -config ~/Downloads/viam-laptop-main.json`) and binds `:8081`.
- **Arm:** `follower-arm`, model `devrel:so101:arm`, on `/dev/cu.usbmodem58CD1767051`, with `speed_degs_per_sec: 30` and `use_urdf: true`. `follower-gripper` is attached. The leader arm, teleop, cameras and data capture are all disabled.
- **so-101 module:** registry `latest-with-prerelease`, resolving to **v0.14.0**. Time-scheduled `MoveThroughJointPositionsStreamed` has been in since 076751f (2026-09-14), so the arm can stream.
- **Streamed points ignore the driver's speed and acceleration caps** (spec review, so-101 at 0.14.0). `MoveThroughJointPositionsStreamed` writes each point with goal speed 0 and acceleration 0, which on the STS3215 means **maximum speed and unlimited acceleration** (`streamed.go` → `moveJointsUniform(..., streamed=true)`). `speed_degs_per_sec: 30` applies only to `MoveToJointPositions` and to the first-point gate.
  - The speed levels below are therefore **enforced only by the loop's per-tick step clamp.**
  - **A resync jump also runs at full servo speed.** Any `q_cmd − q_meas` gap up to `resync_deg` (for example after an obstruction clears) closes at the servo's top speed. Keep `resync_deg` as small as the droop allows.
  - If the driver falls behind (for example, from bus contention), it writes the late points as soon as it catches up, faster than the level allows. The driver logs `streamed N points … late K (max M)`, **but only when the stream ends by its channel closing**; on context cancel it returns with no log. Today `armIO.Stop` closes the channel and cancels at the same moment, so the line is a race. This stage makes it reliable (see Code changes: graceful drain).
  - **Who reads it:** after every run, Claude reads the line from the machine log (`read_machine_logs` on `laptop-main`) and records it. **Max late < 1 tick is a post-run pass/fail bar**; it can't abort a run mid-way, because the count exists only once the stream ends. A run with no line in the log fails the bar.
- **`Stop` can't be trusted to stop a profiled move, and may speed it up** (spec review; unconfirmed on hardware). `Stop` is `controller.Stop` → `SetVelocity(0)`, which writes the goal-speed register. In position mode **0 means maximum speed**, and the driver docs say `Stop` doesn't interrupt a profiled move.
  - So during any `MoveToJointPositions` (the move to the start pose, `validate`'s moves, the singular pre-move), **only cutting power stops the arm.** Cutting power drops the arm onto the table, because the servos go limp.
  - While streaming, a stop is safe: cancelling the stream leaves the servos holding the last goal, at most one tick ahead.
  - This is reported as a driver finding, not fixed here.
- **On hardware, `validate` is circular.** The driver computes `EndPosition` from the same URDF and the measured joints. FK and Jacobian agreement therefore only confirm the plumbing; physical accuracy needs an external reference, which is stage 3.

## Decisions

| Topic | Decision |
|---|---|
| Approach | **CLI first, then the module.** The diff-ik CLI drives `follower-arm` directly on the same Mac, so it is co-located. Then, with the user's OK at that point, add the diff-ik local module and `servo` service to `laptop-main` for the bench. |
| Safety | **Staged ramp-up.** Three levels (below). Every scenario is re-run at each level, and the next level starts only after a clean pass.
- **The user keeps one hand on the power switch and one near the arm** during every L1 run and every profiled move, because cutting power drops the arm.
- **Ctrl-C while streaming** stops immediately with no ramp (`Loop.Run` → `io.Stop`); the arm holds the last goal, at most about one tick ahead.
- **Ctrl-C during a profiled move** isn't a stop. The CLI must not call `arm.Stop` while a `MoveToJointPositions` is in flight (see Code changes). It prints that the move can only be stopped by cutting power, and waits for the arm to be physically still.
- **Server-side Stop on a lost session.** `MoveToJointPositions` is heartbeat-monitored: if a client with a session dies mid-move (crash, SIGQUIT, laptop sleep), viam-server calls `arm.Stop` once the 2 s heartbeat window lapses. The CLI dials with sessions disabled for stage 1 (see Code changes), which removes this path for the CLI. The bench (Python SDK, sessions on) keeps it for its move to home: don't kill the bench during that move.
- **Between levels,** check servo temperature and load through whatever the driver exposes; if it exposes nothing, record that. An overload latch drops torque. |
| Start pose | **Computed, plus a floor guard.** The start pose comes from the URDF (below), and the user approves it on the real arm before any servo run. A new CLI floor guard rejects any generated target that would bring the end effector or the jaw tip below a floor height. |

### Speed levels (loop limits, CLI flags or `servo_start` limits)

| Level | Joint (`--max-joint-deg-s`) | Linear (mm/s) | Angular (°/s) | Bench dead-man | Bench twist (stop and dead-man) | `resync_deg` |
|---|---|---|---|---|---|---|
| L1 | 30 | 40 | 20 | 300 ms | 30 mm/s | max((measured lag + 3) ticks at the level speed, 2 × measured droop) |
| L2 | 60 | 70 | 30 | 300 ms | 50 mm/s | same rule |
| L3 | 90 | 100 | 45 | 300 ms | 50 mm/s | same rule |

**`resync_deg` is set explicitly** in the CLI (`--resync-deg`) and in the bench (`limits.resync_deg`).
- **Revised on hardware (2026-11-15).** The rule was max(3 ticks at the level speed, 2 × droop), which assumed the sims' 2-tick lag. On the real arm the lag is 4–5 ticks (80–100 ms). At L1 (1.8°) the circle resynced 23 times (mean 21.9 mm, max 46.0 mm). With **(lag + 3) ticks**, 8 ticks × 0.6° = 4.8°, it had 0 resyncs (mean 17.3 mm, max 19.8 mm).
- **Values:** L1 4.8°, L2 9.6°, L3 14.4°.
- **Cost:** after an obstruction clears, a gap of up to `resync_deg` closes at full servo speed.
- **Why:** the default, 3·max/rate, is 1.8° at L1 and 50 Hz. If gravity droop exceeds the threshold, the loop resyncs `q_cmd` to the sagged pose every tick. Raising `resync_deg` above the droop stops the constant resyncing. **It does not remove the sag:** the loop integrates `q_cmd` from FK(`q_cmd`) but judges convergence from FK(measured joints). So `q_cmd` reaches the target, the measured pose stays one droop's worth off, and the commanded velocity is 0.
- **Expected droop:** v0.14.0 writes P48 plus an I term by default (`DefaultArmGains`), which the driver docs put at about 1.3–1.5° before the I term and roughly half that with it. (The docs' 4.8° figure is for p_gain 16.) About 0.7° is about 2 mm at the end effector, right at the convergence tolerance.
- **What happens if the sag is too big:** see pre-servo check 4.
- **Droop** is measured before any servoing (see "Pre-servo checks").

At every level: pointing task, `--singular-eps 0.03 --max-damping 0.05`, and the rate from the probe (50 Hz if the bus allows it).

**The CLI needs linear and angular speed flags.** Today `run` only exposes `--max-joint-deg-s`, so this stage adds `--max-linear-mm-s` and `--max-angular-deg-s`, both defaulting to the loop's values (100 mm/s and 45°/s).

### Start pose

`q_start = (0, −0.61, 0.46, 1.39, 0)` rad, which is (0, −35.0, 26.4, 79.6, 0)°. This was computed from `testdata/so101.urdf`:
- the end effector (Viam's, on the wrist-roll axis) is at (180, 0, 192) mm;
- the tool is tilted 19° from straight down;
- the jaw tip is about 100 mm above the base plane. The real follower's TCP is (6.8, 0, 99.9) mm in the tool frame, and its contact pads reach Z 104.3 (`gripper.go`). The pad end is about 91.6 mm up, and a ±15 mm finger-width box reaches about 88.2 mm;
- every joint is at least 15° inside its limits.

`dev/so101-hw.md` records the pose, how to move there (`MoveToJointPositions`), and the user's sign-off that it is clear on the real setup. Each run starts from `q_start`.

### Pre-servo checks (in `dev/so101-hw.md`, done once by the user with Claude, before L1)

1. **Calibration and direction.** At `q_start`, reached by `MoveToJointPositions` at the driver's 30°/s with the user's hands ready:
   - measure the jaw-pad end with a ruler, and compare against FK. It should be about 92 mm above the base plane (add the base-plane offset to get height above the table), and about 169 mm out horizontally from the shoulder-pan axis (208 mm from the base-frame origin, which sits 38.8 mm behind the pan axis). The 180 mm figure is the end effector on the wrist-roll axis, not the pads;
   - jog each joint +5°, one at a time, and confirm the direction matches the URDF.

   A mismatch over 10 mm or a flipped sign stops stage 1.
2. **Base-plane offset.** Measure how high the URDF base plane (z = 0 of the base frame) sits above the tabletop. The floor guard's `--min-z-mm` is set relative to the **tabletop**, so it is 30 mm minus that offset in base-frame terms.
3. **Limits.** Compare the driver's calibrated joint limits with the URDF's. The driver silently clamps streamed points to its calibrated limits, while the client checks URDF limits. Confirm `q_start` and the singular targets are at least 10° inside both.
4. **Droop.** At `q_start`, after `MoveToJointPositions`, hold 2 s and record commanded minus measured per joint, and **the end-effector error** |FK(commanded) − FK(measured)|.
   - The per-joint maximum sets `resync_deg`.
   - **If the end-effector error is ≤ 1 mm,** the 2 mm convergence bars stand.
   - **If it is over 1 mm,** the convergence tolerance for every run is raised to 2 mm + that error (CLI `--tol-mm`, bench `--tol-mm` → `limits.tol_mm`), and the sag is recorded as a loop finding: the loop has no gravity or integral term, which stage 2 has to address.
   - **If it is over 5 mm,** stop and discuss before servoing.
5. **Sign-off on `qa` and `qb`.** The singular pre-move's `qa` and target `qb` put the arm near-vertical overhead, tool up and back, with the end effector at about (63, 0, 377) and (1, 0, 379) mm. The user approves both, as well as `q_start`.

### Floor guard (`--min-z-mm`, CLI)

- **Default:** off (0 disables it). Stage 1 sets it to **30 mm above the tabletop**, which is `30 − offset` in base-frame z, using the offset from pre-servo check 2. The guard compares base-frame z.
- **What it checks:** for every target, it computes FK of the target pose (or of the target joints) and checks a **jaw box**: the end effector, plus points in the tool frame at Z = `--tip-mm` (default 105) and X/Y = ±15 mm. If any of them is below the floor, the target is rejected.
- **Where:**
  - `static` and `staticReachable` samples, which are skipped and re-drawn;
  - the `circle` start point and a check every 10° around the circle, which rejects the whole circle;
  - the `singular` targets, which reject the scenario;
  - `validate` samples, which are skipped and re-drawn;
  - **every `MoveToJointPositions` the CLI issues:** the move to `--start-joints`, the `validate` samples and nudges, and the singular pre-move to `qa`. The driver's coordinated move is linear in joint space, so the guard checks 20 points along the joint-space line from the measured joints to the goal. A start point that is already low (for example the rest pose) is exempt if z rises steadily from there.
- **On rejection:** it prints which point would go below the floor and by how much.
- **Limit:** for servo targets, the guard checks the targets, not the path the loop takes between them. Those steps are small, and the start pose leaves about 60 mm of margin above a 30 mm floor.
- **Measured margins** (spec review, FK; 30 mm floor):
  - `staticReachable`'s lowest point is 61–69 mm;
  - `validate` at spread 0.1 gets to about 48 mm;
  - random ±10° on every joint gets to 42.7 mm at worst;
  - the circle holds a constant z.
- **`--start-joints` is required whenever `--min-z-mm` is set.** Every scenario starts from the current pose, so without it a `circle` run after `singular` would circle over the base.

## Runs, bars and measurements

**Order:**
1. `probe`;
2. `validate`;
3. at each level L1 → L2 → L3: `static --hold`, `singular`, `circle`, then the bench and the unreachable-pointing check. The bench runs only once the module is added.

| Run | Bar |
|---|---|
| `probe` (new) | Report only.
- **Reads:** 500 `JointPositions` reads, back to back.
- **Read+send:** a ticker-paced loop at the candidate rate (50 Hz for 10 s) through `armIO`, sending a **fixed `q0` read once at the start**. It never re-sends the latest reading: with gravity uncompensated, re-sending the sagged pose would let the arm creep down. Sends are never back to back, because over gRPC that stacks points due milliseconds apart and builds a backlog.
- **Reports:** read p50/p99, tick p50/p99, overruns, and the driver's `late` count from the machine log (`read_machine_logs`).
- **Rate rule:** if tick p99 is over the period or the driver reports late points, retry at 40, 30, … Hz. Use the highest rate that is clean, and say so in the write-up. |
| `validate` | Informational. Uses `--settle-tol 2e-3` (one STS3215 count is 1.534e-3 rad, so this allows a one-count flicker); the new flag defaults to the current 1e-4. |
| `run static --task pointing --hold 1s` | 10/10 converged to ≤ 2 mm / 2°; achieved rate ≥ 0.99 × `--rate` (`minRate` is no longer hard-coded at 49.5); driver max late < 1 tick. `--hold` (new) holds each target 1 s after convergence and reports the drift (max error during the hold) and the final error. **The 2 mm comes from joint readings,** so it excludes backlash and link flex. |
| `run singular --wrist 2 --singular-deg -74` | Reached; joint speed ≤ 1.1 × the level limit; ≤ 2 oscillations/s. Skipped at any level where the floor guard rejects it, with the reason recorded. |
| `run circle --circle-mm-s <level>` | Baseline (no feedforward). |
| Bench `--task pointing` (after the module is added) | Smoke passes; dead-man ≤ timeout + 2 ticks + ramp + 50 ms; feedforward circle mean ≤ 22.1 mm. Every bench session runs with the level's limits (see Code changes).
- **One circle per level,** using the level's `resync_deg` (the hard-coded second circle at `resync_deg` 15 is removed), at the level's circle speed. |
| Unreachable pointing (bench level, after the module is added) | Ad hoc through DoCommand, as in 3a/3b: from `q_start`, `servo_target` at the current position with the tool axis rotated 20° about base z. Bar: `converged`; position ≤ the convergence tolerance; residual within ±1° of the geometry's prediction. |
- **No floor guard is needed on the bench path from `q_start`:** its circle holds z, its twists are x-only at constant z, and the smoke step's twist goes only 5 mm down.
- **It must start from `q_start`:** the bench records home at startup, so move there first. |

**Measurements:**
- **Bus:** probe p50/p99; tick p50/p99, overruns and the driver's late count per run.
- **Streaming lag:** cross-correlation over the static and circle CSVs (ticks and ms). The sims gave 2 ticks (40 ms). The circle speed stays below the linear cap so the cross-correlation measures lag, not the cap.
- **Settling and sag:** the `--hold` drift and final error per target. Gravity isn't compensated on the real arm, so this captures sag.
- **Stop shape:** bench travel after `servo_stop` at the level's twist speed, against the ideal ramp (speed × ramp / 2). This shows whether the driver ramps (as in both SO-101 sims) or cuts a queue (as on the MuJoCo UR5e).
- **Resyncs:** per level.

**Results:** `docs/results/2026-11-phase3c1.md`, containing:
- per-level tables;
- a comparison against the MuJoCo SO-101 (3b);
- recommended rate, `resync_deg`, damping and speed limits for stages 2 and 3.

## Code changes (CLI and bench)

- **`cmd/cli/main.go`:** `--min-z-mm`, `--tip-mm` (default 105), `--settle-tol`, `--hold`, `--max-linear-mm-s`, `--max-angular-deg-s`; the `probe` command; `--start-joints`.
  - `--start-joints` takes comma-separated radians, and is required when `--min-z-mm` is set. When set, `run` and `validate` first make a guarded move to these joints and wait for the arm to settle.
- **No `arm.Stop` during profiled moves.** The CLI wraps every `MoveToJointPositions`:
  - the call runs on a context detached from Ctrl-C (`context.WithoutCancel(ctx)`), so the RPC isn't cancelled while the servos keep driving;
  - "in flight" lasts until the arm is physically still: the RPC has returned and two joint reads 100 ms apart differ by under 2e-3 rad (just over one count);
  - while in flight, neither the `err != nil` defers in `run.go`/`validate.go` nor Ctrl-C call `arm.Stop`; on Ctrl-C it prints "a profiled move can only be stopped by cutting power" and waits;
  - `arm.Stop` stays on the streaming paths.
- **Sessions off for the CLI:** dial with `client.WithDisableSessions()`, so viam-server never calls `arm.Stop` on a lapsed heartbeat during a profiled move. Cost: if the CLI dies mid-stream, there is no heartbeat stop, but the stream's gRPC context ends with the connection and the servos hold the last goal (at most one tick ahead), the same as a stream stop.
- **`armio.go`, graceful drain (loop change):** `armIO.Stop` closes the batch channel, then waits for the stream to return for up to 100 ms, and only then cancels and calls `arm.Stop`. The driver then ends the stream normally and logs its late count. A test with a fake arm checks that a stream that finishes inside the window is not cancelled, and one that hangs is cancelled after 100 ms.
- **Circle speed:** `run circle` and the bench circle take `--circle-mm-s` (default 50, today's value). Stage 1 uses min(50, 0.8 × the level's linear cap): 32 mm/s at L1, 50 mm/s at L2 and L3.
- **Convergence tolerance:** `run` gets `--tol-mm` (default 2, today's value), used only if pre-servo check 4 raises it.
- **`module.go`:** `servo_start` limits accept `tol_mm` and `tol_deg` (the loop already has `TolMM`/`TolDeg` and reports them in status). Validated like the other limits: positive and finite. The bench passes `--tol-mm` through.
- **`run.go`:** `minRate` becomes 0.99 × `--rate`, in place of the hard-coded 49.5.
- **`clients/python/servo_bench.py`:** level-aware arguments `--linear-mm-s`, `--angular-deg-s`, `--rate`, `--timeout-ms`, `--resync-deg`, `--singular-eps`, `--max-damping`, `--twist-mm-s` `--circle-mm-s` and `--tol-mm`, passed into `servo_start`'s limits and to the dead-man, stop and circle steps. Today those steps hard-code `timeout_ms` 500, `rate_hz` 50, a 50 mm/s twist and a 50 mm/s circle. `--resync-deg` applies to every session; the bench runs one circle with it, replacing today's default-resync and resync-15 pair.
- **`cmd/cli/floor.go` (new):** the floor-guard check: a target pose to the end effector and jaw-tip z, compared against the floor. A pure function with unit tests on the SO-101 model.
- **`cmd/cli/probe.go` (new):** the bus probe.
- **`cmd/cli/run.go`, `validate.go`:** wire in the guard, `--hold` and `--settle-tol`.
- **Tests:**
  - floor-guard unit tests: tip below the floor rejected, end effector below the floor rejected, `q_start` accepted, a circle that dips rejected;
  - probe statistics on a fake arm whose streamed method paces like the driver (it waits until each point is due, then writes it);
  - the no-`Stop`-during-profiled-move rule, using a fake arm that records calls.

  The CLI has had no tests so far; these add `cmd/cli` tests with a fake `arm.Arm`.
- **The only planned loop and module changes are the graceful drain in `armio.go` and the `tol_mm`/`tol_deg` limits.** A hardware-exposed loop bug gets a fix and a test, as in earlier phases.

## Config change (needs the user's OK when reached)

Add to `laptop-main`, after the CLI runs pass at L1:
- the local module `diff-ik` → `/Users/nick.hehr/src/diff-ik/bin/diff-ik`;
- the service `servo` (`devrel:diff-ik:servo`, `arms: ["follower-arm"]`).

Nothing else changes. Build `bin/diff-ik` for darwin from the stage-1 branch first, and after any rebuild restart the diff-ik module (Phase 3b lesson: a rebuilt binary isn't reloaded until the module restarts). `follower-arm` is framed on world with no offset, so the bench's `follower-arm_origin` frame is the base. Writes go through `set_machine_config` with a saved backup of the previous config, because the add-item tool wrongly added a registry module on `mujoco-sim-mac`. The so-101 module's `teleop` service stays disabled, because it would fight the servo for the arm.

## Out of scope

- Stage 2 (teleop) and stage 3 (AprilTag, camera-frame targets, physical accuracy).
- Path-level collision checking.
- Changes to the so-101 driver: findings are reported, not fixed.
