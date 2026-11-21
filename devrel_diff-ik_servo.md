# Model devrel:diff-ik:servo

A motion service that servos arms in Cartesian space. A client opens a session on an arm and streams pose targets or
twists to it. A 50 Hz differential-IK loop tracks the latest one and streams joint goals to the arm through
`MoveThroughJointPositionsStreamed`.

Every session enforces joint, linear and angular speed limits, joint acceleration limits (which ramp motion at start
and stop), and a mandatory dead-man timeout. Targets can be given in any frame.

The service also answers [`viam:teleop`](https://github.com/viam-modules/teleop)'s `teleop_*` commands, so it can be
that module's `motion_service`.

`Move` is not implemented: it refuses an arm with a live session. `StopPlan` ramp-stops the arm's session.

**Requirements:** each arm needs a kinematic model and must implement `MoveThroughJointPositionsStreamed`. Arms with
fewer than 6 DOF use the pointing task, which tracks position and tool-axis direction and leaves roll free.

## Configuration

```json
{
  "arms": ["arm"],
  "vision_services": ["obstacles-top"],
  "teleop": {
    "timeout_ms": 1000,
    "limits": { "linear_mm_s": 250, "angular_deg_s": 180, "joint_deg_s": 100 }
  }
}
```

| Name | Type | Inclusion | Description |
|---|---|---|---|
| `arms` | []string | Required | Arms this service may servo. They become dependencies. |
| `vision_services` | []string | Optional | Vision services a `live_obstacles` session may poll. They become dependencies. Use the short names; the module declares the full names `rdk:service:vision/<name>` as dependencies. |
| `sensors` | []string | Optional | Force-torque sensors a `guard` or `force` may poll. They become dependencies. |
| `teleop` | object | Optional | `servo_start` defaults for `teleop_*` sessions (`limits`, `rate_hz`, `timeout_ms`, `ramp_ms`, `target_filter`, `task_weights`, ...), plus `teleop_idle_ms` (default 60000). |

Set the joint limit below the arm driver's own speed limit. Drivers such as the xArm reject streamed points above it,
and the session faults.

## DoCommand

Every servo command is keyed by `"command"`. Units are mm and degrees. Poses use Viam's orientation vector in degrees:
`{"x", "y", "z", "o_x", "o_y", "o_z", "theta"}`.

### `servo_start`

Opens a session on one arm and returns `{"session_id", "pose"}`, where `pose` is the arm's current pose in `frame`. A
second session on a busy arm fails.

```json
{
  "command": "servo_start",
  "component_name": "arm",
  "frame": "world",
  "mode": "pose",
  "task": "pose",
  "rate_hz": 50,
  "timeout_ms": 500,
  "ramp_ms": 200,
  "limits": { "joint_deg_s": 90, "joint_deg_s2": 360, "linear_mm_s": 100, "angular_deg_s": 45 },
  "target_filter": { "ema_alpha": 1, "hold_mm": 1, "hold_deg": 0.5, "feedforward": true,
                     "floor": { "min_z_mm": 0, "tip_mm": 100, "margin_mm": 15 } },
  "frames": false
}
```

| Field | Default | Notes |
|---|---|---|
| `component_name` | required | One of `arms`. |
| `frame` | `world` | Session frame: `world` or the arm's name. |
| `mode` | `pose` | `pose` (`servo_target`) or `twist` (`servo_twist`). |
| `task` | `pose`; `pointing` below 6 DOF | `pointing` tracks position and tool-axis direction. |
| `rate_hz` | 50 | 1–1000. |
| `gain` | 2 | Tracking gain in 1/s: the commanded TCP velocity is the target's feed-forward velocity plus `gain` × the error, so an error closes with a time constant of about 1/`gain`. Must be > 0. Reported in the status `limits`. |
| `timeout_ms` | 500 | Dead-man, in (0, 10000]. It can't be disabled. |
| `ramp_ms` | 200 | Stop ramp: the velocity scales to zero over this time. The stop lasts longer if the acceleration limit needs it. |
| `limits.joint_deg_s` | 90 | A number, or one per joint. |
| `limits.joint_deg_s2` | the arm's kinematic model, else 360 | Joint acceleration limit: a number, or one per joint. It can only tighten the model's limit. Each tick, the change in joint velocity is scaled (all joints together) to stay within it, and a joint near a position limit is slowed to √(2·a·d) so it can stop in time. |
| `limits.linear_mm_s` / `angular_deg_s` | 100 / 45 | TCP speed limits. |
| `limits.tol_mm` / `tol_deg` | 2 / 2 | Convergence tolerance. |
| `limits.resync_deg`, `limits.max_lag_ms` | 3 ticks at the joint cap; 300 | See Status fields. |
| `limits.singular_eps`, `limits.max_damping` | 0.04, 0.1 | Damping near singularities. |
| `solver` | `dls` | `dls` or `qp`. `qp` solves a QP each tick. It enforces joint position, velocity and acceleration limits as hard bounds, and adds the posture task. |
| `posture` | see below | `qp` only. `{"joints_deg", "weight", "gain"}`: `joints_deg` defaults to the joints at session start (one value per joint; an empty list or a wrong length is rejected); `weight` defaults to 1e-3 and must be in (0, 0.01]; `gain` defaults to 1/s and must be > 0. Posture motion is not bound by the TCP speed limits, so its weight is capped at 0.01; it moves joints only where the task leaves freedom. Unknown keys are rejected. It is fixed for the session; start a new one to change it. |
| `task_weights` | `[1,1,1,1,1,1]` | x, y, z, rx, ry, rz weights for the pose task. |
| `target_filter` | pass-through, feed-forward on | `ema_alpha` (1 = off), hold deadband `hold_mm` + `hold_deg` with `dwell_ms`, `feedforward`, and `floor` (`min_z_mm` required). The floor raises targets so the flange and a tip `tip_mm` along the tool axis stay `margin_mm` above `min_z_mm`. |
| `frames` | false | When true, `servo_target` accepts a `frame` from the frame system. That frame may hang off static frames or this arm. |
| `world_state` | none | `qp` only. Obstacles, as the motion service's `WorldState` (protojson: a JSON object or a string). Boxes, capsules, spheres and points only; a mesh is rejected. Resolved into the arm's base frame at start and fixed for the session. An obstacle in a movable frame other than this arm is an error. One in a frame that moves with the arm is placed at the start pose, then fixed. Transforms parented to the arm or a mounted frame become robot parts (a held object). Giving it turns collision on. |
| `collision_specification` | none | `qp` only. Allowed contacts, as the motion service's `CollisionSpecification` list (protojson: a JSON list or a string): `[{"allows": [{"frame1", "frame2"}]}]`. A name is a geometry label (links are `<arm>:<link>`), a frame name (the arm's name covers all its links), or `<frame>_origin`. An unknown name is an error. The name of a frame skipped as movable is a no-op, with a warning. Giving it turns collision on. |
| `collision` | off | `qp` only. `true`, or `{"d_min_mm", "d_act_mm"}` (defaults 10 and 100; `d_act_mm` must exceed `d_min_mm`; unknown keys are rejected). Collision is also on when `world_state` or `collision_specification` is given. `"collision": false` turns it off even then. Frame-system meshes on the arm or around it (a gripper model, a fixture) are checked as their bounding box in the mesh's own frame, with a warning; cylinders are still skipped. |
| `live_obstacles` | off | `qp` only. Obstacles from a vision service, polled while the session runs. Turns collision on; `"collision": false` with it is rejected. See below and "Live obstacles". |
| `guard` | off | Stop on contact: `{"sensor", "force_n", "torque_nm", "rate_hz"}`. See "Contact guard". |
| `force` | off | Twist mode only. Regulate the pressing force along the tool axis. See "Force control". |
| `point_velocities` | true | Send per-point target joint velocities to the driver. |

#### `live_obstacles`

```json
"live_obstacles": { "vision": "obstacles-top", "camera": "top-depth", "rate_hz": 8,
                    "workspace": { "min_mm": [200, -600, 20], "max_mm": [1000, 600, 600] },
                    "extrude_to_z_mm": 20 }
```

| Field | Default | Notes |
|---|---|---|
| `vision` | required | One of `vision_services`. |
| `camera` | required | Passed to the vision call. Its frame must be in the frame system, on a fixed frame or the arm. |
| `rate_hz` | 10 | In (0, 50]. Set it below 1/p95 of the service's latency. |
| `stale_ms` | 500 | Must exceed 1000/`rate_hz` + `timeout_ms`, so one slow poll can't stale-stop the session. |
| `timeout_ms` | `stale_ms`/2 | Each vision call's timeout, in (0, `stale_ms`]. |
| `inflate_mm` | 10 | At least 0. Each obstacle grows by this much on every side. An explicit 0 is kept, not replaced by the default. |
| `max` | 32 | Integer, 1-64. The closest `max` to the arm are kept. |
| `workspace` | none | `{"min_mm": [x, y, z], "max_mm": [x, y, z]}` in world. Detections are clipped to it. Needs both keys, min below max on every axis. |
| `extrude_to_z_mm` | none | World z. Each box is extended down to it, so a top-face slab rests on the support. Must lie inside `workspace` when both are given. |
| `min_points` | 10 | Integer, at least 1. Applies only when `workspace` is set: a detection with fewer points left after clipping is dropped. Extrude alone never drops a sparse detection. |

Unknown keys are rejected, and each error names its field. The teleop defaults block takes `live_obstacles` too.

### `servo_target`

Sets the latest pose target; the newest target replaces the previous one. `frame` (optional) is resolved once, using
the arm's current joints. `"apply": false` previews the resolved, filtered and clamped target without moving the arm.
Each target refreshes the dead-man.

```json
{ "command": "servo_target", "session_id": "...", "pose": { "x": 300, "y": 0, "z": 200, "o_x": 0, "o_y": 0, "o_z": -1, "theta": 0 } }
```

Returns `state`, `err_mm`, `err_deg`, `reach_deg`, `pointing_residual_deg`, `target`, `target_raw` and `clamped`.

### `servo_twist`

Sets the latest twist in twist mode: `linear` in mm/s and `angular` in deg/s, each `[x, y, z]`. A zero twist holds
position. Returns `state`, plus `roll_ignored` when the pointing task drops a roll component.

```json
{ "command": "servo_twist", "session_id": "...", "linear": [10, 0, 0], "angular": [0, 0, 0] }
```

### `servo_stop` and `servo_status`

`{"command": "servo_stop", "session_id": "..."}` ramp-stops the session and returns its final status.
`servo_status` returns the status without changing anything.

States: `idle` (no target yet), `running`, `converged`, `stalled` (limit-bound or singular), then one of `stopped`,
`stopped_timeout`, `stopped_stale` (live obstacles went stale), `stopped_contact` (the guard tripped) or `faulted`. Like `stopped_timeout`, `stopped_stale`
isn't overwritten by the dead-man; an explicit `servo_stop` turns it into `stopped`.

### Status fields

`servo_status` and `servo_stop` return:
- `state`, `task`, `err_mm`, `err_deg`;
- `rate_hz`, `tick_p50_ms`, `tick_p99_ms`, `overruns`;
- `resyncs`, `lag_ms`, `damping`, `clamps`, `last_command_age_ms`;
- the active `limits` and `target_filter`;
- `solver`, `q_cmd_deg` (the commanded joints, in degrees) and `peak_accel_ratio`;
- `pointing_residual_deg`, for the pointing task;
- with `qp`: `qp_active`, `qp_iters` and `qp_fallbacks`;
- with `qp` and collision: `collision` (`on` or `off`), `collision_ignored` (pairs suspended at start, listed until re-armed), `collision_skipped` (frames and geometries left out), `collision_pair` (the closest unsuspended pair, `<part>↔<obstacle>`; empty only when every pair is suspended or before the first tick), `collision_min_mm` (its distance, at any distance; absent only in those same cases), `collision_rows` (active rows this tick), `collision_relaxed` and `collision_brakes` (session totals);
- with `live_obstacles`: `live_count` (obstacles in use this tick), `live_age_ms` (age of the newest set), `live_updates`
  and `live_errors`, `live_self_dropped`, `live_skipped`, `live_overflow` and `live_clipped` (session totals),
  `live_backing_away` (true while any live pair is inside `d_min`), `live_pinched` (session total of ticks solved by the second tier),
  `live_waiting` (true until the first set arrives), `live_method` (`detections3d` or `object_point_clouds`) and, after a
  failed poll, `live_last_error`. Live obstacles are named `live:<n>`, or `live:<n>:<label>`, in `collision_pair`.

`peak_accel_ratio` is the session's maximum of |Δstep| / (a·dt²). At or below 1, the acceleration limit held on every
tick.

`lag_ms` is the loop's measured estimate of how far the arm trails its commands, capped at `limits.max_lag_ms`
(default 300, at most 1000). If the arm never moves, the command can run up to that much travel ahead before the first
resync.

`limits.resync_deg` is how far the arm may be from where it should be given that lag before the loop resyncs its
command to the measured joints. The default is 3 ticks of travel at max joint speed. The gap between command and
measured joints, and so the size of a resync snap, can reach lag travel + `resync_deg`.

## Contact guard

With `guard`, the session watches a wrist force-torque sensor (one of `sensors`) and stops itself on contact. Any
solver and mode.

```json
"guard": { "sensor": "wrist-ft", "force_n": 5, "torque_nm": 0.5, "rate_hz": 500 }
```

| Field | Default | Notes |
|---|---|---|
| `sensor` | required | One of `sensors`. Readings are either `force` and `torque` maps of `x`, `y`, `z` (N, N·m; mujoco-sim's `force_torque`) or flat `Fx_N`, `Fy_N`, `Fz_N`, `TRx_Nm`, `TRy_Nm`, `TRz_Nm` (`viam:ufactory:ft_sensor`). |
| `force_n` | required | Trip when \|F − F₀\| reaches this. Must be > 0. |
| `torque_nm` | 0 (off) | Also trip when \|T − T₀\| reaches this. |
| `rate_hz` | 500 | Sensor poll rate, in (0, 2000]. The guard polls in its own goroutine, not once per tick. |

- **Baseline.** `servo_start` averages 5 readings into F₀ and T₀, a software tare, so the tool's weight never counts.
  A sensor that can't be read fails the start. A load picked up mid-session shifts the baseline.
- **Trip.** The loop ticks at once, cancels the joint stream (queued points don't play out), calls the arm's `Stop`,
  then moves it to the *measured* joints (`MoveToJointPositions`), and ends as `stopped_contact`. Commanding the measured joints releases what the arm was pressing with: at contact the
  command is ahead of a blocked arm by its lag, and a plain stop holds that point. The trip ignores the ramp and the
  acceleration limit, so a false trip in free space is a hard stop.
- **Blind sensor.** Three failed reads in a row end the session as `faulted`.
- **Status:** `guard_reads`, `guard_errors`, `guard_force_n` (latest \|F − F₀\|), `guard_peak_n` (until the trip),
  `guard_trip_n` and `guard_reason`.

Results: `docs/results/2026-11-contact-guard.md`.

## Force control

With `force` (twist mode only), the session presses the tool along its z axis at a set force while the client's twist
drives the other directions. It suits press-and-hold, sanding, polishing and wiping.

```json
"force": { "sensor": "wrist-ft", "target_n": 10, "gain": 0.5, "ki": 4, "max_mm_s": 5, "approach_mm_s": 5 }
```

| Field | Default | Notes |
|---|---|---|
| `sensor` | required | One of `sensors`, with readings like the guard's. The sensor frame is taken as the tool frame. |
| `press_axis` | `"-z"` | The sensor z sign that a surface pushing back on the tool reads: `"-z"` for mujoco-sim's `force_torque`, `"+z"` for a sensor with the opposite convention. The xArm sensor's sign is unverified: press on the tool and check `Fz_N`. |
| `target_n` | required | Pressing force to hold, N. Must be > 0. |
| `gain` | 0.5 | Axis speed per N of force error, mm/s per N. Must be > 0. |
| `ki` | 0 | Integral gain, mm/s per N·s. Use it to follow a surface that moves along the axis under a sweep (a tilted face). Reset at contact and clamped to `max_mm_s`. |
| `max_mm_s` | 5 | Cap on the axis speed once in contact. |
| `approach_mm_s` | 5 | Axis speed before contact. The impact at contact grows with it (about 10 N at 5 mm/s on a rigid surface in sim). |
| `contact_n` | min(2, `target_n`/2) | Pressing force that counts as contact. Must be below `target_n`. |
| `rate_hz` | 500 | Sensor poll rate, in (0, 2000]. |
| `comply_xy` | 0 (off) | Sideways compliance, mm/s per N: once in contact, the tool also moves in tool x/y at `comply_xy` × the sideways contact force, clamped to `max_mm_s`. It lets walls centre a peg in a hole. In the peg-in-hole cell, 1.0 worked and 2.0 was worse. |

- **Approach.** Until contact, the tool moves along its axis at `approach_mm_s`. Its axis component of the client's
  twist is replaced. Stream zero twists to hold x/y.
- **Contact.** At first contact the command snaps back to the measured joints, as the guard's trip does, so the
  approach's lag-ahead doesn't become a force spike.
- **Regulation.** Then the axis speed is `gain`·e + `ki`·∫e dt, with e = `target_n` − pressing force, clamped to
  ±`max_mm_s`.
- **Taring.** The tare is taken at start, from the sensor's own poller. Three failed reads in a row end the session as
  `faulted`.
- **Guard.** Add a `guard` (with a `force_n` above the target) as a hard limit.
- **Status:** `force_press_n` (this tick's pressing force), `force_side_n` (the sideways contact force) and
  `force_contact`.
- **Tuning (sim, rigid surface):** `gain` 0.4–1.6 settles in under 0.5 s. It oscillates from about 3. Gains depend on
  how stiff the arm and contact are, so retune on hardware.

Results: `docs/results/2026-11-force-control.md` and `docs/results/2026-11-peg-in-hole.md`.

## QP solver

With `"solver": "qp"`, each tick solves a weighted least-squares QP over the joint step. The task rows are the same
ones the DLS path uses, and the pointing task's axis rows get weight 1e-4, so position wins. Joint position, velocity
and acceleration limits are hard bounds, with a discrete braking cap near a position limit. The posture task pulls the
joints toward `posture.joints_deg` in the null space. Its pull is scaled by the ramp, so stops aren't delayed. If the QP
fails on a tick (and no collision row is active), that tick uses the DLS path (`qp_fallbacks` counts them). After 10 failures in a row the session
faults with `qp: <error>`. `qp` is experimental. See `docs/results/2026-11-qp-solver.md`.

## Collision avoidance

Opt-in, `qp` only (see the `world_state`, `collision_specification` and `collision` fields). It keeps the arm and parts
rigidly mounted on it at least `d_min` from known obstacles, without self-collision. Obstacles are the fixed geometries
in the machine's frame system plus `world_state`, read once at `servo_start`.

- **Shapes.** Boxes, capsules, spheres and points. Frame-system meshes and cylinders are skipped with a warning and
  listed in `collision_skipped`, so they are unprotected unless the client passes a primitive for them in
  `world_state` (`dev/panda-collision-ws.json` boxes in the Panda cell's bin and gripper). Meshes in `world_state` and
  link meshes are rejected. Movable frames (another arm, a gripper with DoF) are skipped too.
- **Damper.** Each pair closer than `d_act` adds a row `J_d·Δq ≥ −v_max`, where `J_d` is the finite-difference gradient
  of the distance. With gap `g = d − d_min` and `A_d = Σ|J_d[j]|·a_j·dt²`,
  `v_max = max(√(2·A_d·g) − A_d/2, min(g, A_d), 0)`. It is the fastest approach from which the joint acceleration
  limits can still stop the arm before `d_min`, so the arm slows into an obstacle, or slides along it where the task
  leaves freedom.
- **Starting close.** A pair closer than `d_min` at start is suspended and listed in `collision_ignored`. It is
  re-armed once it is beyond `d_act`.
- **Fallback.** If the QP fails with rows active, the tick retries with each row relaxed to the strongest move away the
  step bounds allow (`collision_relaxed`), then brakes (`collision_brakes`). It never falls back to DLS, which ignores
  obstacles. A brake tick counts toward the 10-in-a-row fault only once the arm is at rest.
- **Limits.**
  - Capsule↔box distances can jump near contact, where the RDK switches to SAT.
  - Sliding along a face can drift slightly below `d_min` (about 0.2 mm in tests): the damper doesn't push back once
    inside.
  - Distances are measured at the commanded joints, not the measured ones.
  - Static obstacles are fixed for the session (live ones are not, see below), and there is no self-collision.

Design: `docs/superpowers/specs/2026-11-18-diff-ik-collision-design.md`.

## Live obstacles

With `live_obstacles`, the arm also avoids obstacles that a vision service reports while the session runs. Static
obstacles and their suspend-at-start rule still apply. Design: `docs/superpowers/specs/2026-11-18-diff-ik-live-obstacles-design.md`.
Results: `docs/results/2026-11-live-obstacles.md`.

- **Poller.** Each session runs one goroutine that calls the vision service every 1/`rate_hz`, one call at a time. It
  uses its own frame system and parts snapshot. At tick start the loop takes the newest set, which replaces the live
  obstacles wholesale (no tracking of identity between sets). Live pairs are never suspended.
- **Which call.** At start, `GetProperties` picks `GetDetections3D` if the service supports it, else
  `GetObjectPointClouds`; with neither, `servo_start` fails. `GetDetections3D` uses each detection's first transform
  (geometry and pose, in the frame of its parent, which needn't be the camera's name). `GetObjectPointClouds` carries no
  frame, so it uses the camera's name as the frame; a camera whose frame-system name differs from its resource name
  doesn't work on that path. Boxes, capsules and spheres only; other detections are skipped.
- **Clip, refit, extrude.** With `workspace`, each detection's points are clipped to it in world, and the geometry is
  replaced by the world-axis-aligned box around the points left (its orientation is lost). A detection with no points is
  intersected with the workspace instead. `extrude_to_z_mm` then lowers the box's bottom to that z. This removes depth
  edge pixels that join a prop to things above or below it, and turns the top-face slab a downward camera sees into a
  volume.
- **Self test.** A detection is dropped as the arm seeing itself when over half of up to 500 of its points lie within
  20 mm of any robot part. A detection with no points can't be tested and is kept. `viam:obstacles-pointcloud` returns
  points for this; a segmenter that doesn't would have the arm back away from itself.
- **Backing away.** A live obstacle closer than `d_min` adds a row requiring the gap to grow, at up to `linear_mm_s` and
  within the acceleration limit. The task still pulls toward its target, but the row is a hard constraint, so the arm
  holds at `d_min` and resumes when the obstacle leaves.
- **Pinched.** If the back-away rows can't be met together (a part between two obstacles), the second tier keeps "no
  further approach" as a hard limit and adds a soft retreat, so the arm retreats as far as the rows allow together.
  `live_pinched` counts those ticks. Only if that fails does the tick brake.
- **Waiting.** Until the first set arrives the arm holds its position, and `live_waiting` is true. A target set meanwhile
  is held, not walked away from.
- **Staleness.** Failed or timed-out polls are skipped and counted; the last set is kept. If no set arrives for
  `stale_ms` (also counted from `servo_start` before the first), the session ramp-stops with final state
  `stopped_stale`.

**Limits.**
- **The self test can drop a real obstacle in near-contact.** Anything with more than half its visible points within
  20 mm of a robot part (a hand around a link, a small object against the gripper) is treated as the arm itself, so
  protection lapses at very close range.
- No velocity prediction. An obstacle moving toward the arm closes the gap, and the arm then backs away; `rate_hz` and
  `inflate_mm` set how fast an intrusion it can handle.
- The vision API returns no capture time. The joints at request time stand in for the capture pose, which is an error for
  a wrist camera that moves during the call.
- Perception error counts: boxes from a downward camera ran 3-13 mm narrow per side, so keep `inflate_mm` at 10 or more.
- A camera on a movable frame other than the arm is rejected.

**Segmenters.** Both pipelines were tried in the Panda cell (`dev/panda-live.json`, `dev/panda-mujoco.md`):
- `viam:obstacles-pointcloud` (0.2.0 or later). It serves `GetDetections3D`, finds any object, and removes the table
  plane. It also returns the arm, which the self test drops. Tuned for the overhead camera:

```json
{ "name": "obstacles-top", "api": "rdk:service:vision", "model": "viam:obstacles-pointcloud:obstacles-pointcloud",
  "attributes": { "camera_name": "top-depth", "ground_plane_normal_vec": { "x": 0, "y": 0, "z": -1 },
                  "max_dist_from_plane_mm": 25, "min_points_in_plane": 2000, "min_points_in_segment": 50,
                  "ground_angle_tolerance_degs": 10 } }
```

- `viam:detections-to-segments` (1.4.0 or later) over a 2D detector. It serves `GetObjectPointClouds`, only for what the
  detector finds, and never the arm. It takes in every depth point inside the 2D box, so use `workspace`:

```json
{ "name": "red", "api": "rdk:service:vision", "model": "color_detector",
  "attributes": { "detect_color": "#E60D0D", "hue_tolerance_pct": 0.06, "segment_size_px": 200 } }
{ "name": "segments-top", "api": "rdk:service:vision", "model": "viam:vision:detections-to-segments",
  "attributes": { "camera_name": "top-depth", "detector_name": "red", "confidence_threshold_pct": 0.5,
                  "mean_k": 5, "sigma": 1.25, "infer_minimum_depth": true } }
```

Add each service name to `vision_services`.

## viam:teleop compatibility

The service answers `teleop_start`, `teleop_move`, `teleop_stop` and `teleop_status`, each a request with no `command`
key. Point a `viam:teleop` service at it with `"motion_service": "<this service name>"`. See the README for the
semantics.

Note that `teleop_status` polls refresh a teleop session's dead-man for up to `teleop_idle_ms` after the last move,
because `viam:teleop` sends nothing while the leader is still.
