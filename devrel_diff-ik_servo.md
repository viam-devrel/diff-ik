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
  "teleop": {
    "timeout_ms": 1000,
    "limits": { "linear_mm_s": 250, "angular_deg_s": 180, "joint_deg_s": 100 }
  }
}
```

| Name | Type | Inclusion | Description |
|---|---|---|---|
| `arms` | []string | Required | Arms this service may servo. They become dependencies. |
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
| `collision` | off | `qp` only. `true`, or `{"d_min_mm", "d_act_mm"}` (defaults 10 and 100; `d_act_mm` must exceed `d_min_mm`; unknown keys are rejected). Collision is also on when `world_state` or `collision_specification` is given. `"collision": false` turns it off even then. |
| `point_velocities` | true | Send per-point target joint velocities to the driver. |

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
`stopped_timeout` or `faulted`.

### Status fields

`servo_status` and `servo_stop` return:
- `state`, `task`, `err_mm`, `err_deg`;
- `rate_hz`, `tick_p50_ms`, `tick_p99_ms`, `overruns`;
- `resyncs`, `lag_ms`, `damping`, `clamps`, `last_command_age_ms`;
- the active `limits` and `target_filter`;
- `solver`, `q_cmd_deg` (the commanded joints, in degrees) and `peak_accel_ratio`;
- `pointing_residual_deg`, for the pointing task;
- with `qp`: `qp_active`, `qp_iters` and `qp_fallbacks`;
- with `qp` and collision: `collision` (`on` or `off`), `collision_ignored` (pairs suspended at start, listed until re-armed), `collision_skipped` (frames and geometries left out), `collision_pair` (the closest unsuspended pair, `<part>↔<obstacle>`; empty only when every pair is suspended or before the first tick), `collision_min_mm` (its distance, at any distance; absent only in those same cases), `collision_rows` (active rows this tick), `collision_relaxed` and `collision_brakes` (session totals).

`peak_accel_ratio` is the session's maximum of |Δstep| / (a·dt²). At or below 1, the acceleration limit held on every
tick.

`lag_ms` is the loop's measured estimate of how far the arm trails its commands, capped at `limits.max_lag_ms`
(default 300, at most 1000). If the arm never moves, the command can run up to that much travel ahead before the first
resync.

`limits.resync_deg` is how far the arm may be from where it should be given that lag before the loop resyncs its
command to the measured joints. The default is 3 ticks of travel at max joint speed. The gap between command and
measured joints, and so the size of a resync snap, can reach lag travel + `resync_deg`.

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
  - Obstacles are fixed for the session, and there is no self-collision.

Design: `docs/superpowers/specs/2026-11-18-diff-ik-collision-design.md`.

## viam:teleop compatibility

The service answers `teleop_start`, `teleop_move`, `teleop_stop` and `teleop_status`, each a request with no `command`
key. Point a `viam:teleop` service at it with `"motion_service": "<this service name>"`. See the README for the
semantics.

Note that `teleop_status` polls refresh a teleop session's dead-man for up to `teleop_idle_ms` after the last move,
because `viam:teleop` sends nothing while the leader is still.
