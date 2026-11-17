# Diff-IK Phase 3c, stage 3 — AprilTag visual servoing on the real SO-101: Design

Date: 2026-11-16
Builds on: the stage 1 and 2 specs and results (`docs/results/2026-11-phase3c1.md`, `docs/results/2026-11-phase3c2.md`), the runbook `dev/so101-hw.md`.
Brief:
- the Phase 3 bar: "Teleop and an AprilTag target both run without servo-bus saturation or visible jitter";
- the open question: "should `servo_target` accept poses in a camera frame directly (frame system transform per tick) for visual servoing?";
- stage 1 deferred physical accuracy to here.

## Goal

Servo the real SO-101 follower to an AprilTag seen by a camera, with **two camera setups compared**: a wrist camera (eye-in-hand) and a fixed scene camera (eye-to-hand). Two scenarios:
1. **Tracking:** hover above a hand-held, moving tag.
2. **Static accuracy:** hover 20 mm above a tag at several table positions, and measure the physical error with a ruler.

Answer the brief's camera-frame question with a working implementation.

## Decisions

| Topic | Decision |
|---|---|
| Cameras | **Both, compared.** Two separate config entries: a wrist camera (frame parent `follower-arm`) and a scene camera (frame parent `world`). |
| Tasks | **Tracking + static accuracy**, for each camera. |
| Frames | **In the module, computed locally.** `servo_target` accepts an optional `frame` (any frame-system frame, e.g. a camera's name).
- The session builds the frame system **once at `servo_start`** (one `FrameSystemConfig` RPC).
- Each target is transformed in-process with **the loop's latest measured joints** for the arm and zero inputs for everything else.
- There's no RPC per target and no extra bus reads. A frame-system `TransformPose` would read *every* input-enabled component on each call: the follower, its gripper and the leader arm. |
| Target filter | **In the module,** after the transform: an optional `target_filter` on `servo_start` with an EMA, hold-with-hysteresis (stage 2's logic), a **floor clamp**, and a **feedforward switch**.
- **Why in the module:** with a wrist camera, a still tag's camera-frame pose changes as the arm moves, so only an arm-frame filter is correct.
- **The floor clamp has to be server-side:** the client never sees the arm-frame pose of a camera-frame target before it's applied.
- Off by default. |
| Servo task | **Pointing** (5-DOF: position plus tool axis, roll free). A tilted, hand-held tag gives tool axes that are generally unreachable for the 6-DOF pose task on this arm. Pointing keeps position first and reports the residual angle. |
| Extrinsics | **`viam:camera-calibration`:** `eye_in_hand` for the wrist camera (board on the table), `eye_to_hand` for the scene camera (a small board on the gripper). It applies the frame automatically when its quality gates pass. |
| Intrinsics | **Ours:** `clients/python/intrinsics.py` runs OpenCV's ChArUco calibration on about 20 views captured through the camera API. Neither the AprilTag module nor `camera-calibration` computes intrinsics; both require them (`charuco/charuco.go:241`). |
| Tag detection | **`viam:apriltag:pose_tracker`** (tag36h11, `tag_width_mm` measured), one per camera. |

## Hardware (to be set up by the user)

- **Wrist camera:** rigidly mounted on the follower's gripper.
- **Scene camera:** fixed, about 0.5–1 m from the workspace, oblique.
- **Two ChArUco boards:**
  - a larger one for the table (A4 is fine), for the wrist camera's `eye_in_hand` calibration and for intrinsics;
  - a small, light one (about 100×70 mm, e.g. 5×4 squares of 18 mm) to mount on the gripper for the scene camera's `eye_to_hand`. The module's recommended 300×200 mm aluminium board is too big and heavy for the SO-101.
  - Measure the square and marker sizes, and note the ArUco dictionary.
- **Two tag36h11 AprilTags** of measured size (about 40–60 mm) on rigid cards.
- The webcams are `rdk:builtin:webcam`. Intrinsics go in their `intrinsic_parameters` and `distortion_parameters` attributes.

## Module change: camera-frame targets and a target filter

**Frame system at `servo_start`.** When `servo_start` is given `"frames": true`, the session does the following; building lazily on the first framed target would put the frame-system RPC inside a target call. A `servo_target` with `frame` on a session started without `frames: true` is an error.
- builds a local frame system from one `FrameSystemConfig` RPC and a local `referenceframe.NewFrameSystem` (what `framesystem.NewFromService` does; the implementation calls the two directly through a narrow interface).
  - This needs the full `framesystem.Service`, so the session's `poseTransformer` dependency widens to `framesystem.Service` (or a narrow interface with `FrameSystemConfig` + `TransformPose`, calling `referenceframe.NewFrameSystem` directly). The test fake implements it.
- starts from `referenceframe.NewZeroInputs(fs)`. **It doesn't call `CurrentInputs`:** inside a module, that's `RobotClient.CurrentInputs`, which reads every input-enabled resource (follower, gripper, leader arm) and fails if any one fails, so an unplugged leader would break `servo_start`.

Per target, the arm's entry (keyed by its frame name, e.g. `follower-arm`) is set to the loop's latest measured joints (`QMeas`, radians). Before the first tick `QMeas` is empty, so it falls back to the joints `start()` already reads. The transform is `fs.Transform(inputs.ToLinearInputs(), referenceframe.NewPoseInFrame(frame, pose), sessionFrame)`.
- **Cost:** a local `Transform`, microseconds, with no RPC and no bus reads.
- **Constraint:** every other movable component sits at zero inputs. So a camera must be parented to the session's arm or to a static frame (`world`, etc.), as both cameras here are. A camera parented to another movable frame isn't supported. On the first target for a given `frame`, the session walks that frame's parent chain (`fs.TracebackFrame`) and returns an error if any frame other than the arm's is input-enabled. The result is cached per frame name.
- **Errors:**
  - A missing frame system or service is an error at start if `frames` is requested.
  - An unknown `frame` in a target returns an error and leaves the target unchanged.

**`servo_target` gains an optional `frame` (string).**
- **Absent, or equal to the session frame:** behaviour is unchanged.
- **Otherwise:** the pose is transformed to the session frame with the local frame system and the arm's current measured joints. This runs **before** `s.mu` is taken (no RPC, so it's fast), then the filter is applied, then the velocity estimator, then `SetMotion`.
- **`apply: false`** (default true): transform and return `target_raw` / `target` without filtering or applying. The client uses it for `--axis down` and for the wrist camera's look-then-move, which need the tag's arm-frame pose.
- **Reply additions:**
  - `target`: the filtered target in the session frame;
  - `target_raw`: transformed, before the filter;
  - `reach_deg`: the reachable part of the pointing error (the loop's `convDeg`), which is what convergence uses;
  - `pointing_residual_deg`: the unreachable remainder.
- **Known limitation (measured, not fixed):** the tag pose comes from an image captured earlier than the joints used for the transform. For a wrist camera, arm motion during that gap (τ) becomes target error ≈ v·τ.
  - **Measurement:** a static tag and the wrist camera; step the hover height; regress the arm-frame target's deviation against the follower's velocity (from the 10 Hz `get_end_position`). The slope is τ.

**`servo_start` gains an optional `target_filter`:**
```json
{"ema_alpha": 0.3, "hold_mm": 1.0, "hold_deg": 0.5, "dwell_ms": 200,
 "floor": {"min_z_mm": -10, "tip_mm": 105, "margin_mm": 15}, "feedforward": true}
```
Applied in the **arm base frame** (the floor's `min_z_mm` is a base-frame value; EMA and hold are unchanged by a rigid transform), in this order: EMA → hold → floor → (velocity estimator unless `feedforward: false`) → `SetMotion`.
- **EMA:** `p ← p + α(p_new − p)`, orientation by slerp. **Seeded from the first sample, and reset at session start.** `α = 1` means off.
- **Hold with hysteresis:** pass the target through while it moves. Once it has stayed within `hold_mm` / `hold_deg` of an anchor for `dwell_ms`, hold the latest pose. Release on leaving the band. The anchor resets at session start.
- **Floor:** stage 2's `floor_lift`. Raise z until the flange and the jaw tip (flange + `tip_mm`·tool axis) clear `min_z_mm + margin_mm`. The result reports clamping.
  - At a 20 mm hover over a tag on the table, the tip sits at about +6 mm against the clamp at about +5 mm (`-10 + 15`), so the static-accuracy target isn't clamped.
- **`feedforward: false`:** `SetMotion` gets zero velocity, so the loop chases the latest target without extrapolating.
  - Why: tag noise of about 1 mm at 30 Hz reads as 15–30 mm/s of target velocity, and with a wrist camera the timing skew turns feedforward into positive feedback.
  - It's A/B'd in the tracking runs.
- **Validation:** `0 < ema_alpha ≤ 1`; `hold_*` and `dwell_ms` ≥ 0; `margin_mm` and `tip_mm` ≥ 0.
- Status reports the filter config and a clamp count.
- **Tests:**
  - the filter as a pure type: EMA seeding and convergence, hold semantics including "never steps backwards", floor clamping, feedforward off giving zero velocity;
  - a session test with a fake frame system: `frame` routing uses the loop's measured joints, and the reply fields;
  - a test that the transform doesn't hold `s.mu` (no RPC under the lock).

## Calibration flow (per camera, once)

1. **Intrinsics:**
   - Run `uv run clients/python/intrinsics.py --camera <name> --board …` with the board moved around the view. It captures about 20 accepted views (the ChArUco corners detected), then calibrates.
   - It prints the RMS reprojection error and the `intrinsic_parameters` / `distortion_parameters` JSON.
   - Apply them to the camera's config, as a config change with the user's OK.
   - Bar: RMS < 1 px.
2. **Extrinsics with `viam:camera-calibration`:**
   - Configure a `charuco` pose tracker, plus the `eye_in_hand` or `eye_to_hand` service.
   - **How the module moves the arm:** it plans directly with `armplanning.PlanMotion`, not the motion service, and with no WorldState (`handeye.go:943`). Only obstacles in the frame system count. It executes with `MoveThroughJointPositions(ctx, traj, nil, nil)`, so speed and acceleration come from the follower's config.
   - **SO-101 safeguards:**
     - **Table obstacle:** a frame-system component with geometry (e.g. an `obstacle-pack` box), its top 1–2 mm below the arm's base plane, so the start state isn't in collision.
     - **Slow moves:** temporarily set the follower's `speed_degs_per_sec` (e.g. 20) and `acceleration_degs_per_sec_per_sec`, then restore them.
     - **`workspace_bounds` (absolute):** they bound the *camera* pose (`eye_in_hand`) or the *board* pose (`eye_to_hand`), not the jaw. Set z.min to the table + the camera-or-board-to-lowest-jaw offset + 20 mm, measured at setup and recorded in the runbook.
     - **`input_range_override`:** it can only tighten the limits, and the start pose must lie inside it (`handeye.go:694`). Use the joint ranges used safely in stages 1–2, containing the seed pose.
     - **Seed pose:** start from a **high** pose near the sweep (CLI guarded `pose` move). The final return to it is a single unplanned joint move with no obstacle check (`handeye.go:715`). Auto-apply runs before that return; if the config write reconfigures the module, the return is skipped and the arm stays at its last station.
     - `settle_seconds` 2.
   - Send `{"calibrate": {}}`.
   - **Never send `cancel`, and never use the app's Stop control, during a run.**
     - `cancel` calls `arm.Stop` mid-move (`handeye.go:736-738`).
     - The driver's `Stop` writes goal speed 0, i.e. maximum speed, on every servo.
     - The module's error paths, Close and reconfigure only cancel the context. The driver then finishes the last profiled goal, which is safe.
     - Abort by cutting power.
   - Bars, the module's gates: translation residual ≤ 5 mm, rotation ≤ 2°, mean reprojection ≤ 2 px. Record the result and `auto_applied`.
3. **Check:** with the tag on the table at a ruler-measured position in the arm's base frame, compare the tag pose transformed to `follower-arm_origin` with the measurement. Report the error.

## The client: `clients/python/tag_servo.py`

**Code reuse.** First, refactor `teleop.py`: move the connection, engage, watchdog, ordered exit, signal handling and CSV plumbing into a base class (e.g. `ServoClient` in `clients/python/servo_client.py`) that both clients subclass. That's an explicit plan task, with the teleop tests and the sim rehearsal re-run afterwards.

- **Input:** polls `viam:apriltag:pose_tracker` `GetPoses` on the second connection, as fast as it returns. It takes the configured tag ID's `PoseInFrame`; its reference frame is the camera.
  - **Normal-jump gate:** a detection whose tag normal jumps by more than `--max-normal-jump-deg` (default 20°) from the last accepted one is dropped. That catches the PnP planar-ambiguity flip.
- **Target (camera frame):**
  - **Position:** the flange goes to `p_tag + (hover + tip_mm) · n̂`, where n̂ is the tag's normal flipped to the camera's side (`n̂ · (−p_tag) > 0`). This is robust to the sign of the tag's z.
  - **Orientation:** the tool axis is `−n̂`, or, with `--axis down`, the session frame's −z (for static accuracy with the tag flat on the table).
  - It's sent with `frame: <camera name>`.
- **Session:**
  - `servo_start` with `task: "pointing"`, `frame: "follower-arm_origin"`, `frames: true`, at L2 limits (60°/s, 70 mm/s, 30°/s, `resync_deg` 9.6).
  - `target_filter` gets EMA `--ema-alpha` (default 0.3), hold from `--hold-mm` / `--hold-deg` (default: set from the static-noise run, see Runs), floor `--min-z-mm -10`, and `--feedforward on|off`.
- **Engage:** at half limits, as in stage 2. It ends when **`err_mm` ≤ 10 and `reach_deg` ≤ 5** for 0.2 s. Not `err_deg`: on 5 DOF, a tilted tag's normal can be out of reach, and `err_deg` would then never drop.
- **Sending:**
  - one `servo_target` per accepted detection;
  - **if no detection arrives for 100 ms, re-send the last reply's arm-frame `target` with no `frame`**, so the target stays fixed in the arm frame. Re-sending a stale camera-frame pose with a wrist camera would chase the arm's own motion;
  - after **0.5 s** with no detection, the client **sends `servo_stop` itself** and exits cleanly. If the client is dead, the dead-man stops the follower within `timeout_ms` (300) plus the 200 ms ramp.
- **Static accuracy, wrist camera (look, then move):** a 20 mm hover can't see the tag. So converge at `--hover-mm 150` with detections, freeze the reply's arm-frame target, then descend by sending arm-frame targets (no `frame`, no new detections), lowering the hover to `--final-hover-mm 20` along the tool axis. The scene camera stays closed-loop at 20 mm.
- **Floor:** the module's `target_filter.floor` clamps every target, camera-frame ones included. The watchdog also checks the follower's actual pose at 10 Hz.
- **Logs:** `PREFIX-ticks.csv`, one row per sent target:
  - time and whether it was a detection or a re-send;
  - the tag pose (camera frame);
  - the reply's `target` and `target_raw` (arm frame);
  - state, `err_mm`, `err_deg`, `reach_deg`, `pointing_residual_deg`;
  - the round-trip ms.

  Plus `PREFIX-follower.csv`, as in stage 2.
- **Flags:** `--camera`, `--tracker`, `--tag-id`, `--hover-mm`, `--final-hover-mm`, `--axis normal|down`, `--ema-alpha`, `--hold-mm`, `--hold-deg`, `--feedforward`, `--max-normal-jump-deg`, `--duration`, `--csv`, plus stage 2's level, safety and address flags.
- **Analysis** (`clients/python/tag_lag.py`, reusing `teleop_lag.py`'s helpers):
  - lag (the follower against `target`);
  - **tag noise at rest from `target_raw`** (std while the tag is still);
  - the detection rate, clamps, normal-jump drops, re-sends;
  - the `servo_target` round-trip.

## Runs and bars

Per camera (wrist, then scene), at L2 limits:

| Run | Procedure | Bar |
|---|---|---|
| Intrinsics | about 20 views | RMS < 1 px |
| Extrinsics | `calibrate` | module gates pass (≤ 5 mm, ≤ 2°, ≤ 2 px) |
| Calibration check | tag at a ruler-measured table position | report the error |
| Static noise | tag still in view at the tracking distance, 10 s, filter off (`--ema-alpha 1 --hold-mm 0`) | report the `target_raw` std (mm, °) per axis; set `--hold-mm` / `--hold-deg` to about 3σ for the next runs |
| Tracking | 60 s: hand-held tag moved slowly at 100–200 mm below the hover, then held still for ≥ 3 s | **late 0; overruns 0; max tick p99 < 20 ms; no visible jitter (user)** — PASS required. Report lag, tag noise at rest, detection rate and round-trip. |
| Static accuracy | tag flat on the table at 5 positions; `--axis down`; scene camera closed-loop at 20 mm, wrist camera look-then-move (150 → 20 mm); converge, then the user measures the horizontal offset of the **point on the tool axis between the pads** (the TCP is 6.8 mm off-axis, and roll is free) from the tag centre, and its height | report per position; summary mean and max. **Target: ≤ 10 mm horizontal** (informational; this is the system's physical accuracy: intrinsics, extrinsics, kinematics and backlash) |

Gain profile: `tuned` for static accuracy (precision). For tracking, run `tuned` with `--feedforward on` and `off` (A/B); also run `smooth` if `tuned` shows visible jitter (stage 2 showed `smooth` removes it). Timing skew (wrist camera): the measurement in "Module change", done once.

Results go in `docs/results/2026-11-phase3c3.md`:
- per-camera tables;
- the calibration quality;
- the answer to the brief's question (camera-frame `servo_target`: implemented, with its cost and the timing-skew error);
- a wrist-vs-scene comparison and recommendations.

## Config changes (each needs the user's OK when reached, with a backup)

- Enable both cameras, with intrinsics (after step 1) and frames (after step 2; `camera-calibration` auto-applies).
- Add `viam:apriltag` (pose trackers) and `viam:camera-calibration` (charuco trackers, plus the `eye_in_hand` and `eye_to_hand` services) as registry modules.
- Add a table obstacle as a frame-system component with geometry (top 1–2 mm below the arm's base plane) for the calibration sweep's planning, which ignores the motion service's WorldState.
- Temporarily lower the follower's `speed_degs_per_sec` and acceleration for calibration, then restore them.
- Rebuild and restart the diff-ik module after the module change (runbook).

## Safety

- **Calibration sweeps** are profiled, planned moves run by another module, at the arm's configured (temporarily lowered) speed:
  - keep a hand on the follower's power switch and one near the arm;
  - never send `cancel`, and never press Stop in the app;
  - keep the workspace bounds and input ranges tight;
  - the table obstacle keeps the plans off the table;
  - the final return move is unplanned, so start high.
- **Tracking:** the follower follows a hand-held tag. Keep the tag at least `hover` below anything, and keep hands clear of the follower.
  - **Losing the tag:** for 100 ms, the target is held fixed in the arm frame. After 0.5 s, the client sends `servo_stop` (ramp 200 ms). If the client dies, the dead-man stops the follower within 300 ms + the ramp.
- **Targets are floor-clamped in the module** for camera-frame and arm-frame targets alike, and the watchdog checks the follower's actual pose.
- **Static accuracy at 20 mm hover:** keep the jaw tip clear of the tag card. The watchdog floor is `--min-z-mm -10`.
- **Exit, signals and RPC timeouts:** as stage 2.

## Out of scope

- Image timestamps / latency compensation (measured only).
- Image-based visual servoing (IBVS).
- Multi-tag fusion.
- Depth cameras.
- Changes to `viam:camera-calibration` or `viam:apriltag`.
