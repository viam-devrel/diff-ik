# Model devrel:diff-ik:servo

A motion service that servos arms in Cartesian space. A client opens a session on an arm and streams pose targets or
twists to it. A 50 Hz differential-IK loop tracks the latest one and streams joint goals to the arm through
`MoveThroughJointPositionsStreamed`.

Every session enforces joint, linear and angular speed limits, a velocity ramp at start and stop, and a mandatory
dead-man timeout. Targets can be given in any frame.

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
  "limits": { "joint_deg_s": 90, "linear_mm_s": 100, "angular_deg_s": 45 },
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
| `ramp_ms` | 200 | Velocity ramp at start and stop. |
| `limits.joint_deg_s` | 90 | A number, or one per joint. |
| `limits.linear_mm_s` / `angular_deg_s` | 100 / 45 | TCP speed limits. |
| `limits.tol_mm` / `tol_deg` | 2 / 2 | Convergence tolerance. |
| `limits.resync_deg`, `limits.max_lag_ms` | 3 ticks at the joint cap; 300 | See Status fields. |
| `limits.singular_eps`, `limits.max_damping` | 0.04, 0.1 | Damping near singularities. |
| `task_weights` | `[1,1,1,1,1,1]` | x, y, z, rx, ry, rz weights for the pose task. |
| `target_filter` | pass-through, feed-forward on | `ema_alpha` (1 = off), hold deadband `hold_mm` + `hold_deg` with `dwell_ms`, `feedforward`, and `floor` (`min_z_mm` required). The floor raises targets so the flange and a tip `tip_mm` along the tool axis stay `margin_mm` above `min_z_mm`. |
| `frames` | false | When true, `servo_target` accepts a `frame` from the frame system. That frame may hang off static frames or this arm. |
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
- `pointing_residual_deg`, for the pointing task.

`lag_ms` is the loop's measured estimate of how far the arm trails its commands, capped at `limits.max_lag_ms`
(default 300, at most 1000). If the arm never moves, the command can run up to that much travel ahead before the first
resync.

`limits.resync_deg` is how far the arm may be from where it should be given that lag before the loop resyncs its
command to the measured joints. The default is 3 ticks of travel at max joint speed. The gap between command and
measured joints, and so the size of a resync snap, can reach lag travel + `resync_deg`.

## viam:teleop compatibility

The service answers `teleop_start`, `teleop_move`, `teleop_stop` and `teleop_status`, each a request with no `command`
key. Point a `viam:teleop` service at it with `"motion_service": "<this service name>"`. See the README for the
semantics.

Note that `teleop_status` polls refresh a teleop session's dead-man for up to `teleop_idle_ms` after the last move,
because `viam:teleop` sends nothing while the leader is still.
