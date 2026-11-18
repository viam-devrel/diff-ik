# Diff-IK Phase 4 results — moving-tag visual servoing in MuJoCo, 2026-11-17

Spec: `docs/superpowers/specs/2026-11-17-diff-ik-phase4-tag-sweep-design.md`. Plan: `docs/superpowers/plans/2026-11-17-diff-ik-phase4-tag-sweep.md`.

## Task 0 findings

The cell is `dev/so101-mujoco-tag.json`. It uses viam-mujoco-sim's `so101-asset` branch, a local viam-server on port 8082, and the module built from this branch.

1. **`viam:apriltag` on macOS: runs from source.**
   - A local-only config can't load registry modules: the registry entry failed with "expected ExePath to be absolute path".
   - The module's own repo supports darwin/arm64, so the cell runs `~/src/apriltag` through `dev/apriltag-run.sh`. That wrapper is needed because the repo keeps `run.sh` deliberately non-executable.
   - `tag_width_mm` is the black square's edge: the module passes it to `estimate_tag_pose` as the tag size. The registry text says "corner to corner", which is wrong.
2. **`set_prop_pose` at 50 Hz: passes.**
   - Driving a free prop around the 60 mm circle at 40 mm/s for 10 s gave 50.0 Hz achieved, 0.00 mm xy error, 0.00° tilt, and z 1.0–2.0 mm.
   - Placed 1 mm above its resting height, the prop settles 1 mm between calls, so it is now placed at rest: prop centre at world z = 1 mm, tag face at 2 mm.
3. **Detection rate.**
   - With the arm hovering, `get_poses` plus a `transform_pose` RPC ran at about 85 Hz (wrist) and about 69 Hz (scene).
   - `get_images` runs at about 147 Hz (wrist, 640×480) and about 128 Hz (scene, 1280×720).
   - The sim renders on demand, so detections arrive faster than a real 30 fps webcam would allow. That flatters the sim.
4. **Scene-camera coverage: passes after two fixes.**
   - **Frame orientation.** The camera's `target` attribute aims only the sim's render; the frame system kept the default orientation, so a detection transformed 800 mm off. The camera now has an explicit frame orientation, OV (0, −0.7071, −0.7071) at θ = −90°: +Z toward the circle centre, +Y image-down. `target` is gone.
   - **Lighting.** The default sun is straight down, so the arm's shadow fell across the tag, and specular highlights greyed the white cells. The scene apriltag detector then missed most points. The world now has `lighting: {shadows: false, reflections: false}`. This flatters the sim compared with real lighting.
   - **Result:** 10/10 detections at all 8 points round the circle, with the detected position within 0.1 mm horizontally of the truth.
5. **Reach and tip offset.**
   - `tip_mm` = 98, from the Phase 3b results.
   - `tag_servo.py` with the scene camera, `--axis down --hover-mm 150`, converged at the circle's 0°, 90°, 180° and 270° points around the centre (220, 0) mm, with the error at or below 0.01 mm. The centre stays at (220, 0).
   - Fix found on the way: `tag_servo.py` imported `PoseTracker` after connecting, so the robot client never built a tracker client (`ResourceNotFoundError`). It now imports it before connecting. Hardware stage 3 never ran, which is why this hadn't shown up.
6. **Texture scale: passes.** Both cameras put the tag at rest within 0.06 mm of its true face centre, with normal (0, 0, −1). So the texture renders true to size.

## Setup

- **Path:** the tag slides around a 60 mm circle centred at (220, 0) mm in the arm frame. It runs 3 laps at each of 10, 20, 40 and 80 mm/s, with a 1 s speed ramp between speeds, for 216 s in all.
- **Servo:** the arm hovers 150 mm above the tag with the jaws pointing straight down (`--axis down`, pointing task, `tip_mm` 98).
- **Limits:** joint speed 90°/s, TCP speed 120 mm/s, and the module's default joint acceleration limit of 360°/s².
- **Client filter:** `tag_servo.py`'s defaults: EMA α 0.3, a 1 mm / 0.5° hold with 200 ms dwell, and feed-forward on.
- **Ground truth:** the commanded prop pose, logged on the client's clock.
- **Measured:** the jaw tip, from the end-effector pose read at 50 Hz.
- **Steady laps:** error and lag cover laps 2–3 at each speed.
- **Commands:** in `dev/so101-mujoco.md`.

## Results

| Camera | Speed (mm/s) | Error mean / p95 / max (mm) | Lag (ms) | Error at lag (mm) | Detections (Hz) | Drops | Lost | Resyncs | Overruns | Max lag_ms | Peak joint accel (°/s²) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| wrist-cam | 10 | 0.9 / 1.0 / 1.1 | 85 | 0.1 | 50.0 | 0 | no | 0 | 0 | 0 | 147 |
| wrist-cam | 20 | 1.7 / 1.9 / 2.2 | 85 | 0.2 | 50.0 | 0 | no | 0 | 0 | 0 | 103 |
| wrist-cam | 40 | 3.7 / 4.0 / 4.3 | 90 | 0.7 | 50.0 | 0 | no | 0 | 0 | 40 | 221 |
| wrist-cam | 80 | 8.5 / 9.1 / 9.6 | 100 | 1.9 | 49.9 | 0 | no | 0 | 0 | 40 | 452 |
| scene-cam | 10 | 0.7 / 0.9 / 1.0 | 75 | 0.1 | 50.0 | 0 | no | 0 | 0 | 0 | 132 |
| scene-cam | 20 | 1.6 / 1.8 / 2.0 | 80 | 0.3 | 49.9 | 0 | no | 0 | 0 | 0 | 137 |
| scene-cam | 40 | 3.6 / 3.9 / 4.2 | 85 | 0.8 | 50.0 | 0 | no | 0 | 0 | 40 | 204 |
| scene-cam | 80 | 8.7 / 9.2 / 9.7 | 105 | 2.3 | 49.9 | 0 | no | 0 | 0 | 40 | 447 |

**Summary:** both runs completed with no tag loss, no faults and no drops. The mover ran at 50.0 Hz. Accepted detections came in at 96 Hz (wrist) and 73 Hz (scene); the module sends at most one target per 20 ms tick.

### Bars (per camera)

| Measure | Bar | Wrist | Scene |
|---|---|---|---|
| Mean error at 20 mm/s | ≤ 5 mm | 1.7 mm | 1.6 mm |
| Tag loss | none through 40 mm/s | none, even at 80 | none, even at 80 |
| Overruns | ≤ 1 per 10 s | 0 | 0 |

**All bars pass, for both cameras.**

### Filter check: EMA off

Wrist camera, 40 and 80 mm/s, `--ema-alpha 1`:

| Speed (mm/s) | Error mean / p95 / max (mm) | Lag (ms) | Error at lag (mm) |
|---|---|---|---|
| 40 | 2.4 / 4.0 / 4.7 | 60 | 1.0 |
| 80 | 5.5 / 7.2 / 10.8 | 65 | 2.3 |

## Findings

1. **Moving-target tracking works, and the error is almost entirely lag.**
   - Error grows in proportion to tag speed: roughly 1 mm per 10 mm/s.
   - The best-fit lag is a steady 75–105 ms.
   - Shifting the tag path by that lag leaves 0.1–0.3 mm up to 20 mm/s and at most 2.3 mm at 80 mm/s.
   - So the loop follows the path's shape closely and trails it by about 0.1 s.
2. **Where the lag comes from:**
   - **The client's EMA (α 0.3), about 25 ms.** Turning it off cut the lag from 85–100 ms to 60–65 ms and the mean error by about a third. The cost is noise: the p95 is the same and the max is higher. So the filter trades about 25 ms of lag for smoother targets, and α is a tuning choice for each application.
   - **The arm's own following lag, about 40 ms.** That's the loop's measured `lag_ms`. The sim arm's position servos trail their commands.
   - **Detection, transform and the 50 Hz tick:** the rest.
   - Feed-forward is on. It cancels part of the lag but not all of it, because the module estimates the target's velocity from noisy, filtered targets.
3. **Eye-in-hand and eye-to-hand perform the same here.** Neither camera lost the tag at any speed. The spec's question, whether the wrist camera would lose the tag first, doesn't arise up to 80 mm/s: with about 10 mm of tracking error, a 150 mm hover and a 60° view, the tag stays well inside the frame.
4. **The joint acceleration figures are upper bounds, dominated by noise.**
   - The 80 mm/s rows show 447–452°/s² against the 360°/s² limit. The figure comes from double-differencing measured joints sampled at irregular 20–36 ms intervals.
   - A wider smoothing window shrinks it sharply: 452 with 5 samples, 253 with 9, 177 with 15. That points to noise, not real acceleration.
   - The module limits the *commanded* acceleration, so a direct check needs the commanded joints. They aren't in `servo_status` yet.
5. **The sim flatters these numbers:**
   - shadows and reflections are off;
   - intrinsics and extrinsics are exact;
   - frames render on demand, at roughly 130–150 Hz, against about 30 Hz for a real webcam;
   - the tag never blurs.

   On hardware, expect a lower detection rate and so more lag, plus calibration error added to the at-lag error.

## Follow-ups

- Expose the commanded joints (q_cmd) in `servo_status` (or a debug DoCommand), so the acceleration limit can be checked directly.
- Cap the detection rate in sim (e.g. 30 Hz) to approximate a webcam, if the hardware comparison needs it.
