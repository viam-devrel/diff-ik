# Diff-IK Phase 4 — moving-tag visual servoing in MuJoCo: Design

Date: 2026-11-17
Builds on:
- the stage 3 spec, `docs/superpowers/specs/2026-11-16-diff-ik-phase3c3-design.md`, which covers camera-frame targets, the target filter and `tag_servo.py`;
- the Phase 3b results, `docs/results/2026-11-phase3b.md`, from the MuJoCo SO-101;
- the joint acceleration limit (`54416bf`).

## Goal

Show closed-loop visual servoing of a **moving** target. A single `motion.Move` can't do this, and it is the main evidence for the scope doc "Cartesian Servoing in the Motion Service".

The SO-101 hovers about 150 mm above a flat AprilTag. The sim slides the tag around a circle at increasing speeds. Each speed is measured against the tag's exact, commanded path.

The sweep runs twice, once with each camera:
- **eye-in-hand:** a wrist camera;
- **eye-to-hand:** a fixed scene camera.

In sim the intrinsics and camera mounts are exact, so the measured error is the error of detection and control, with no calibration error mixed in.

## Decisions

| Topic | Decision |
|---|---|
| Arm | The SO-101 in viam-mujoco-sim, branch `so101-asset` (worktree `~/src/viam-mujoco-sim-so101`), using the pointing task. It's the same arm and client as hardware stage 3. |
| Cameras | Two sweeps, one per camera: `wrist-cam` (frame parent: the arm) and `scene-cam` (fixed, about 450 mm from the circle centre at roughly 45°, so the arm hovering over the tag doesn't block it). Only one camera drives each run. |
| Detection | `viam:apriltag:pose_tracker` (tag36h11), one per camera: `wrist-tags` and `scene-tags`. This is the hardware path, so `tag_servo.py` runs unchanged. |
| Tag | A box prop, 75 × 75 × 2 mm, textured with tag36h11 ID 0. The PNG is 10 × 10 cells: the 8-cell black square plus a 1-cell white border. The texture covers each whole face, so the black square is 60 mm and `tag_width_mm` is **60**, the black square's edge as in the runbook (`dev/so101-hw.md`). It lies flat on the table, and the world's `set_prop_pose` moves it. |
| Motion | A circle of 60 mm radius, centred where the arm can hover over all of it. The speeds are 10, 20, 40 and 80 mm/s, each held for 3 laps after a 1 s ramp to that speed. The tag moves continuously through the speed changes; it is never teleported. |
| Ground truth | The commanded tag pose, timestamped on the client's clock, which also timestamps the arm and detection data. |
| Frames and units | Everything the runner logs and the report compares is in **mm, in the arm's base frame** (`arm_origin`), the frame `get_end_position()` reports. `--centre x,y` is given in that frame too. `set_prop_pose` takes **metres in the world frame**: the mover converts using the arm's origin in world, read once at start through the frame system (`transform_pose` of the zero pose in `arm_origin` to `world`; the arm is mounted fixed, at world z = 100 mm in this config). |
| Servo limits | `--linear-mm-s 120`, so the TCP speed cap is above the fastest tag speed and the breaking speed measures tracking rather than the cap. The joint cap is 90°/s, and the acceleration limit is the module default of 360°/s². |
| Look-then-move | Out of scope. It assumes a still tag and belongs with a static grasp run. |

## Task 0: unknowns, checked before any code

Each check's result is written into the results doc. If any check fails, apply the fallback, or stop and report.

1. **Does `viam:apriltag` run on macOS?** Check the registry's platforms and the repo (viam-labs/apriltag). If there is no darwin build, run it from source as a local module.
2. **Can `set_prop_pose` drive a resting prop at 50 Hz?** The tag must follow the path, stay flat and not drift between calls. The README says a `fixed` prop never moves, so the expected path is a free prop placed 1 mm above the table each call; a small settle in z is acceptable. If it slides or tips, stop and ask about a kinematic prop in the sim (a change to the sim repo).
3. **How fast are rendering and detection?** Measure the `wrist-tags` and `scene-tags` `get_poses` rate on this Mac. That rate, not the 50 Hz loop, sets how fast the visual loop updates. Report it.
4. **Does the scene camera detect the tag reliably?** At 450 mm and 1280×720 with a 60° vertical field of view, the camera sees about 1.4 px/mm, so the 60 mm tag spans about 85 px (about 10 px per cell). It should detect the tag across the whole circle. If not, move the camera closer or enlarge the tag.
6. **Does the texture render true to size?** Detect the tag at rest under the wrist camera and compare the detected depth and corner positions with the known geometry. That catches MuJoCo scaling or flipping the box texture, which would make the black square something other than 60 mm and put every detected depth off.
5. **Reach and jaw-tip offset.** Find a circle centre and hover height the pointing task reaches all the way round, with the jaws pointing straight down. Measure the jaw-tip offset from the end effector for `--tip-mm` (Phase 3b found a 98 mm flange offset).

## Sim cell: `dev/so101-mujoco-tag.json`

It copies `dev/so101-mujoco.json`, on port 8082 with the `diff-ik` module from `bin/`, and adds the following.
- **`wrist-cam`** (`viam-labs:mujoco-sim:camera`):
  - frame parent `arm`, offset so the jaws don't block the view;
  - 640×480, 60° field of view, looking along the tool axis.
- **`scene-cam`** (`viam-labs:mujoco-sim:camera`):
  - frame parent `world`;
  - 1280×720, aimed with `target` at the circle centre.
- **The `tag` prop** on `sim-world`, with its texture PNG checked in under `dev/assets/`.
- **The `viam:apriltag` module** and two pose trackers, `wrist-tags` and `scene-tags`, with `tag_width_mm: 60`.

## Runner: `clients/python/tag_sweep.py`

**Small refactor of `tag_servo.py`:** split the engage-and-restart part of `TagServo.run()` into a method `start_tracking()`. It runs the floor check, starts detection and the watchdog, waits for the first tag, runs the engage session at half limits until settled, stops it, and starts the full-limit track session. `TagServo.run()` calls it and then keeps its current tracking and look-then-move code, so its behaviour doesn't change.

**`TagSweep(TagServo)`** overrides `run()` and `go()`:
1. `await self.start_tracking()`. The tag sits at rest at the circle's start point throughout.
2. Stream in phase `track` until settled (2 mm / 2° for 1 s, 30 s timeout), using the existing `settle`.
3. Start the **mover** task. It calls `sim-world`'s `set_prop_pose` at 50 Hz along the speed schedule, and writes each commanded pose as a row of `PREFIX-tag.csv`: `t, x, y, z, speed_mm_s, lap`, in mm in the arm frame.
4. Keep streaming in phase `sweep` until the mover's schedule finishes. That is the end condition. Then return, and `close()` stops the session as usual.

**`go()`** catches `TagLost`, reads the mover's current speed, and writes `breaking_speed_mm_s` to `PREFIX-summary.json`. A run that completes writes `breaking_speed_mm_s: null`. The summary also records the arguments, the arm origin in world, and the Task 0 rates the runner measured: detections per second and the mover's achieved call rate.

**Joint logger:** a third task reads `arm.get_joint_positions()` at 50 Hz into `PREFIX-joints.csv` (`t, q0..q4`, in degrees). The follower CSV's 10 Hz poses are too coarse to check acceleration against the limit.

**`lag_ms` column:** append `lag_ms` to `FOLLOW_COLS` in `servo_client.py`, filled from `servo_status` on status rows. It's appended last, so existing readers, which look columns up by header name, are unaffected.

**Sweep flags:** `--speeds 10,20,40,80`, `--laps 3`, `--radius-mm 60`, `--centre x,y` (mm, arm frame), `--world sim-world` and `--prop tag`.

**Output:** `--csv PREFIX` writes the existing ticks and follower CSVs, plus `PREFIX-tag.csv`, `PREFIX-joints.csv` and `PREFIX-summary.json`.

`circle_point(t, schedule)` is a pure function that maps elapsed time to a position, speed and lap. It integrates arc length so the speed changes stay continuous. It is unit tested.

## Analysis: `clients/python/tag_sweep_report.py`

It reads one or more run prefixes, typically one per camera, and prints a table for each speed. It also writes a markdown table for the results doc. The columns are:
- **Tracking error:** the horizontal distance between the jaw tip (end-effector pose plus `tip_mm` along the tool axis) and the tag centre at the same time. Mean, p95 and max, in the circle's steady laps only (the first lap at each speed is excluded).
- **Lag:** the time shift, from −0.5 to +0.5 s, that best lines up the jaw-tip path with the tag path. Also the error after that shift, which shows how much of the error is lag.
- **Detection:** the accepted detection rate in Hz, plus drops by reason.
- **Loss:** whether the tag was lost at this speed. The first speed with a loss is the breaking speed.
- **Loop health:** resyncs, overruns and the maximum `lag_ms`, from the follower CSV's status rows.
- **Peak joint acceleration:** the second difference of `PREFIX-joints.csv` (50 Hz), after a 5-sample moving average, as the maximum per joint. It is reported next to the 360°/s² limit. It isn't a bar, since sim physics and read timing add to it.

It reuses the existing CSV and interpolation helpers (`_rows`, `_interp` in `teleop_lag.py`) and the lag-search approach in `tag_lag.py`, moving any shared helper into one module rather than copying it. The pure functions (time alignment, error stats, lag search, steady-lap selection, acceleration) are tested on synthetic logs.

## Bars (per camera)

| Measure | Bar |
|---|---|
| Mean tracking error at 20 mm/s | ≤ 5 mm |
| Tag loss | none up to and including 40 mm/s |
| 80 mm/s | reported only |
| Loop health | ≤ 1 overrun per 10 s, and resyncs reported |

## Output

- **Results:** `docs/results/2026-11-phase4-tag-sweep.md`, containing:
  - the setup and the Task 0 findings;
  - both cameras' tables, side by side;
  - findings on whether the wrist camera loses the tag before the scene camera, and where the error comes from (lag versus detection noise).
- **Evidence rows** for the scope doc: an "Evidence" row in both the Google Doc and the Claude Doc, made only after the user approves.
- **Runbook:** a new `dev/so101-mujoco.md` with the commands to reproduce the runs. The sim arm is named `arm`, so the command passes `--follower arm --camera wrist-cam --tracker wrist-tags --joint-deg-s 90 --linear-mm-s 120 --axis down --hover-mm 150 --tip-mm <Task 0 value>` (the client's defaults are `follower-arm` and 60°/s).

## Safety and constraints

- Sim only. Nothing touches the armfarm or hardware.
- Changes to the sim module (`viam-mujoco-sim`) are made only if Task 0 needs them, and only after asking the user, because it's a separate repo.
- Commits use the session's commit trailer. Work happens on branch `diff-ik-phase4`, which is squash-merged to `main` at the end.

## Out of scope

- Look-then-move and grasping.
- Step and random-walk paths.
- The UR5e.
- Running both cameras in one run.
- Changes to the module's control law.
