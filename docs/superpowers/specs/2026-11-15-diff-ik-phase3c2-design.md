# Diff-IK Phase 3c, stage 2 — leader-arm teleop on the real SO-101: Design

Date: 2026-11-15
Builds on: the stage 1 spec and results (`docs/superpowers/specs/2026-11-15-diff-ik-phase3c1-design.md`, `docs/results/2026-11-phase3c1.md`), the runbook `dev/so101-hw.md`.
Brief: Phase 3 bar, "Teleop and an AprilTag target both run without servo-bus saturation or visible jitter."

## Goal

Teleoperate the real SO-101 follower from the SO-101 leader arm through the diff-ik `servo` service's public DoCommand API, and give the opt-in `smooth` gain profile a teleop verdict against `tuned`. Stage 3 (AprilTag) is out of scope.

## Decisions

| Topic | Decision |
|---|---|
| Input | **Leader arm, Cartesian.** The leader's end-effector pose drives the follower. No joint mirroring. |
| Placement | **A client script** (`clients/python/teleop.py`), using only the public API. No teleop code in the module. |
| Mapping | **Absolute with ramp-in.** The follower target is the leader's FK pose, both in their own base frames (identical arms).
- **Ramp-in is done by the loop's speed caps, not by blending targets:** an engage session at half the L2 limits tracks the raw leader pose until converged, then the client stops it and opens the L2 session.
- **Why not blend:** on 5 DOF, a blended pose (position lerp plus orientation slerp) is generally unreachable. Pan fixes the plane the tool axis can lie in, and the solver would pan the follower sideways, about 90 mm for a 20° pan gap. A raw leader pose is reachable by construction. |
| Task | **The 6-DOF pose task** (`task: "pose"`). The pointing task leaves roll about the tool axis free, so the leader's wrist roll would never reach the follower.
- `servo_start` accepts `pose` on 5 DOF; pointing is only the default when no task is given.
- With 6 rows and 5 joints the loop takes the damped least-squares path, which converges for exact FK targets.
- Every target the client sends is a raw leader FK pose, never a blend, so it is reachable when the two arms' calibrated ranges match.
- Verifying convergence on the 5-DOF SO-101 is part of this stage. |
| Gripper | **Simple mirror,** outside the servo loop, at about 15 Hz. |
| Evaluation | The brief's bars, plus leader → follower lag and tracking error, run under the **tuned** and then the **smooth** gain profile. |

## Hardware and machine (as found after stage 1)

- **Machine:** `laptop-main` runs from `~/Downloads/viam-laptop-main.json`. Its local plaintext listener changes on restart: find it with `lsof` (runbook §1).
- **Follower:** `follower-arm` plus `follower-gripper`.
  - It currently runs the local so-101 build from branch `stream-speed-profile` (`stream_speed_margin` 1.3, `stream_min_speed_deg_s` 10).
  - Deadband 3 is set in EEPROM on servos 1 and 5.
  - Default gains.
- **Leader:** `leader-arm` (`devrel:so101:arm`, `/dev/cu.usbmodem5B415316871`, `use_urdf`, `waypoint_lookahead_deg` 6) and `leader-gripper` (`gripper_type: leader`). **Both are disabled in the config.**
  - Enabling them is a config change that needs the user's OK at the time, with a backup.
  - The leader is not framed in the config; its `EndPosition` is in its own base frame, which is what absolute mapping wants.
- **The so-101 `teleop` service stays disabled.** It would fight the servo for the follower.
- **A `smooth` run needs a driver that knows `servo_gain_profile`.** Build a local, unpushed branch that merges `servo-deadband` into `stream-speed-profile`, then set `servo_gain_profile: "smooth"` on `follower-arm`; set it back to `"tuned"` (or unset it) afterwards. Rebuild and restart the module (runbook lessons).

## The client: `clients/python/teleop.py`

**Connection.** Like `servo_bench.py`: `--address`, `--insecure` or API keys; `--leader leader-arm`, `--follower follower-arm`, `--leader-gripper`, `--follower-gripper`; `--service servo`.
- **Level arguments,** the same as the bench: `--joint-deg-s`, `--linear-mm-s`, `--angular-deg-s`, `--rate`, `--resync-deg`, `--tol-mm`, `--tol-deg`.
- **`--timeout-ms`:** default 300.
- **Other options:** `--min-z-mm` (30 − the base-plane offset from runbook §4.2, i.e. 30 on this table); `--tip-mm` (105); `--duration` (0 = until Ctrl-C); `--csv PREFIX`; `--no-gripper`; `--dry-run`.
- **Two connections:** one for the servo stream, one for the 10 Hz follower pose reads and the status polls.

**Dry run** (`--dry-run`): reads the leader at `--rate` for 10 s with no session, and reports read p50/p99 and whether a read plus about 1 ms of `servo_target` fits the period. Run it once before any session (stage 1: about 7.5 ms per arm read over RPC). If it doesn't fit, use `--rate 40` or 30. The runbook records the rate used.

**Start-up, in order. Any failure goes to the exit path before the follower moves.**
1. Prompt: "Hold the leader arm, then press Enter."
2. Release the leader: `leader-arm` `set_torque` with `enable: false`. This covers the whole bus, including the leader gripper (servo 6).
3. Re-read the leader pose (it may have moved or dropped at release). If the floor check fails, ask the user to lift it and re-check. Never engage below the floor.
4. **Engage session:**
   - `servo_start` with `mode: "pose"`, `task: "pose"` and `frame: "<follower>_origin"`, at **half the L2 limits** (30°/s, 35 mm/s, 15°/s; `resync_deg` 9.6).
   - Stream the raw (deadbanded) leader pose until `servo_target` reports `converged` for 0.5 s, or for at most 15 s. On timeout: stop the session and exit with a message.
   - Then `servo_stop`.
5. **Tracking session:** `servo_start` with the same mode, task and frame, at the L2 limits.

**Tracking.** Each tick, paced to `--rate`:
1. **Read** the leader's `EndPosition`.
2. **Hold with hysteresis:**
   - While the leader is moving, send its raw pose every tick.
   - Once it has stayed within 1 mm and 0.5° of an anchor pose for 0.2 s, **hold:** resend the anchor unchanged each tick.
   - Release the hold as soon as the leader leaves that band.
   - Why: one leader encoder count (0.088°) at 50 Hz reads as about 4°/s of target motion. That seeds the loop's feedforward and keeps it from ever reporting `converged` at rest. A plain deadband (compare against the last *sent* target) would instead turn slow sweeps into steps every few ticks. The servo's velocity estimator would then alternate its feedforward between a burst and zero, which is visible stepping.
3. **Floor clamp:** if the end effector, or the jaw tip (end effector + `tip_mm` × the tool axis), is below `--min-z-mm` + 15 (the margin stands in for the CLI guard's ±15 mm jaw corners), raise the target's z by the deficit and count a clamp.
4. **Send** `servo_target`, and log its reply (`state`, `err_mm`, `err_deg`).

The loop's speed limits still apply: a leader moved faster than the level allows is followed at the limit, not snapped to. Any `servo_target` error goes to the exit path; the client never restarts a session silently.

**Watchdog** (on the second connection):
- **Follower pose:** read `get_end_position` at 10 Hz and log it. If its z, or its jaw tip's z, is below `--min-z-mm`, exit.
- **Runaway error, armed only in the tracking session:** if `err_mm` exceeds 250 mm for 1 s, exit. Lag on a fast move is allowed (at 70 mm/s, a 0.5 s leader move at 300 mm/s opens about 115 mm). This only catches a follower that is far off and staying off. The run protocol keeps the fast move inside that envelope. The floor watchdog is the main guard, and runs in both sessions.
- **Status:** poll `servo_status` at 5 Hz and log `overruns`, `resyncs`, `tick_p99_ms` and `rate_hz`.

**Gripper.** A separate asyncio task at about 15 Hz reads the leader gripper's `get_position` (`position_percentage`) and, when it changes by ≥ 1 %, sends the follower gripper `set_position` with `percentage` and `wait: false`. It's independent of the servo loop, and its errors are logged, not fatal.

**Exit.** On Ctrl-C, `--duration` elapsing, any error, or the watchdog. These are ordered, independent steps, each in its own try/except, so one failing never skips the rest:
1. `servo_stop`. An already-ended session (after a dead-man trip or a fault) returns an error, which is logged and ignored.
2. Cancel the gripper and watchdog tasks.
3. `leader-arm` `set_torque` with `enable: true`. The leader holds where it is, because the driver seeds goals from the present position.
4. `leader-gripper` `set_torque` with `enable: false`. Step 3 re-torques the whole bus, and the trigger has to stay free (as the driver's `teleop` does).
5. Close both connections.

If the client crashes: the follower stops after the dead-man (`timeout_ms`), but **the leader stays limp and falls**. Keep a hand near it (see Safety).

**Logs.**
- `PREFIX-ticks.csv`, one row per tick: monotonic time, the leader pose, the sent target, `resent`, `clamped`, and the `servo_target` reply (`state`, `err_mm`, `err_deg`), plus the client's tick duration.
- `PREFIX-follower.csv`: the 10 Hz follower `get_end_position` rows and the 5 Hz `servo_status` rows (sparse columns), on the same monotonic clock.

## Runs and bars

At L2 limits (60°/s, 70 mm/s, 30°/s; `resync_deg` 9.6), 50 Hz, the default 5 ms lead. For each gain profile (`tuned`, then `smooth`), one session of about 60 s. The user moves the leader through:
1. slow sweeps across the workspace;
2. a fast move (no faster than about 300 mm/s for 0.5 s, inside the runaway watchdog's envelope);
3. a few pick-like motions with the gripper;
4. a rest.

| Measure | Source | Bar |
|---|---|---|
| Bus saturation | driver `streamed … late K` lines; the 5 Hz `servo_status` polls (`overruns`, `tick_p99_ms`; take the maximum over the polls, since p99 covers only the last 500 ticks) | late 0; overruns 0; max tick p99 < the period — **PASS required** |
| Visible jitter | the user's judgement at the arm, with the follower holding still and moving slowly | "no visible jitter" — **PASS required** |
| Pose task on 5-DOF | `servo_target` replies during **rest**: the leader propped on a support or held still by hand, above the floor (no clamps) for ≥ 2 s | `converged` on ≥ 90 % of ticks, with `err_mm` / `err_deg` ≤ tol — PASS required for `tuned` |
| Leader → follower lag | both pose streams interpolated onto a common 50 Hz time base (same monotonic clock), then cross-correlated | report |
| Tracking error | the interpolated follower pose against the leader pose during motion; `err_mm` / `err_deg` at rest | report |
| Client tick timing | logged tick durations | report; the rate must stay ≥ 0.99 × `--rate` |
| Gripper | the user's judgement | works, follows the leader |

`smooth` needs `--tol-mm` / `--tol-deg` widened to its measured droop (stage 1: up to about 11 mm / 4° at reach). It gets the same bars, except that pose-task convergence is reported only.

Results go in `docs/results/2026-11-phase3c2.md`: a per-profile table, the lag and tracking comparison, a recommendation on whether `smooth` suits teleop, and findings.

## Code changes

- **New `clients/python/teleop.py`.** Pure helpers at the top: the hold-with-hysteresis state machine (hold after 0.2 s inside the band, release on leaving it), the floor clamp (the end effector plus the tip along the tool axis, from Viam's orientation vector), and the runaway check. They're covered by `clients/python/test_teleop.py` (plain `assert` checks, run with `uv run`). The client does no orientation interpolation.
- **New `clients/python/teleop_lag.py`:** reads `PREFIX-ticks.csv` and `PREFIX-follower.csv`, interpolates both pose streams onto 50 Hz, and reports lag (cross-correlation), tracking error, the rest-phase converged fraction, the clamp and resend counts, the tick timing, and the status maxima.
- **`dev/so101-hw.md`:** a stage-2 section with the leader setup, the commands and the safety notes.
- **No loop or module changes are expected.** Measured follower joints aren't logged (`servo_status` doesn't expose them); the follower pose comes from `get_end_position`. If the pose task misbehaves on 5 DOF, that's a finding. A fix (for example, weighting the orientation rows down) is a loop change with a test.

**Calibration.** Absolute mapping assumes the two arms' calibrated ranges match. If the leader can reach where the follower can't, the follower driver silently clamps streamed points, and convergence fails. Read both arms' calibrated limits once (`pose` on each), and record them in the runbook.

## Config change (needs the user's OK when reached)

- Enable `leader-arm` and `leader-gripper` on `laptop-main` (remove `disabled`), with a backup.
- For the `smooth` runs, set `servo_gain_profile: "smooth"` on `follower-arm` with the merged local driver build, then restore it.
- Nothing else. The so-101 `teleop` service stays disabled.

## Safety

- **The leader is limp while released.** Keep a hand on it, or rest it on the table, when not moving it. Torque is re-enabled on every exit path.
- **The follower runs at L2 limits; hand on its power switch.** Ctrl-C stops it through `servo_stop`. A dead client stops it within the dead-man timeout.
- **The engage session caps the first move** at half the L2 limits. The start-up floor check, run after release, refuses to engage with the leader below the floor (+15 mm jaw margin).
- **Watchdog:** it exits on a follower floor breach (10 Hz `get_end_position`, both sessions) or a runaway error (`err_mm` > 250 mm for 1 s, tracking session only).
- **Exit steps are independent and ordered** (see Exit). Any `servo_target` error exits; sessions are never restarted silently.
- **Profiled moves:** the client issues none. Only the servo stream moves the follower, so the stage-1 `Stop` hazard doesn't apply.

## Out of scope

- AprilTag and camera targets (stage 3).
- Force feedback to the leader.
- Joint-space mirroring.
- A browser UI.
- Changes to the so-101 driver's own `teleop` service.
