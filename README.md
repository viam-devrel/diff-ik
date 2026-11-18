# Module diff-ik

Cartesian servoing for Viam arms: a motion service that tracks a stream of pose targets or twists with a 50 Hz
differential-IK loop, with speed and acceleration limits and a mandatory dead-man on every session. Use it for
teleoperation, visual servoing and learned policies, where the target keeps moving. It also works as the motion
backend of [`viam:teleop`](https://github.com/viam-modules/teleop).

This is a proof of concept. Results are in `docs/results/`.

## Models

This module provides the following model(s):

- [`devrel:diff-ik:servo`](devrel_diff-ik_servo.md): Cartesian servo sessions on arms through `DoCommand`, plus `viam:teleop`'s `teleop_*` commands

## Local SO-101 simulation (Phase 3a)

No cloud machine needed:

    (cd ~/src/so-101 && make)            # builds bin/arm
    go build -o bin/diff-ik ./cmd/module
    viam-server -config dev/so101-sim.json   # serves plaintext localhost:8080

Then, from another shell:

    go run ./cmd/cli --address localhost:8080 --insecure --arm arm --task pointing run static
    uv run clients/python/servo_bench.py --address localhost:8080 --insecure --arm arm --task pointing

The paths in `dev/so101-sim.json` are absolute for one laptop; edit `executable_path` for yours.

For physics (Phase 3b), use the MuJoCo SO-101 from viam-mujoco-sim's `so101-asset` worktree:

    make -C ~/src/viam-mujoco-sim-so101 run.sh   # generated, gitignored
    viam-server -config dev/so101-mujoco.json   # serves plaintext localhost:8082
    go run ./cmd/cli --address localhost:8082 --insecure --arm arm --task pointing run static

## teleop_* compatibility

The service is a drop-in `motion_service` for [`viam:teleop`](https://github.com/viam-modules/teleop). It answers
`teleop_start`, `teleop_move`, `teleop_stop` and `teleop_status` (no `command` key) and `GetPose`.

- `teleop_start` resolves the component to one of `arms` (directly, or by walking its frame-system parents, e.g. a
  gripper on the arm), computes the fixed arm-end-effector-in-component transform once, and opens a `world`/`pose`
  servo session holding the arm's current pose. `Destination` is ignored. A second start for the same component
  replaces its session; an arm with a plain `servo_*` session returns "busy".
- `teleop_move` composes the leader pose with that transform and sets the session target (latest wins, `seq` ignored).
  A missing or ended session returns `component "<name>" not registered in teleop pipeline`.
- `teleop_stop` ramp-stops the session. An unknown component is not an error.
- `teleop_status` returns `{"teleop_status": {running, components, error, last_plan_ms, sessions}}`. **A status call also
  refreshes the dead-man of each live teleop session**, because the module sends no moves while the leader is still but
  polls status every 200 ms. The refresh applies only while the session's last `teleop_start` or `teleop_move` is at most
  `teleop_idle_ms` old (default 60000). If the module dies, or keeps polling with no moves past that cap, `timeout_ms`
  (default 1000 for teleop sessions) ends the session as `stopped_timeout`. So a leader held still for longer than
  the cap ends the session, and the operator starts again.
  A session that ended on its own (`stopped_timeout`, `faulted`, ...) shows up in `error` as
  `"<component>: <state>: <reason>"` (several joined with `; `). The error is report-once for the whole service: the
  first status call consumes it, then it clears so the module can start again.
- `teleop_start` and `teleop_stop` are serialised, so a stop never lands before an in-flight start registers. Only rigid
  mounts are supported: a movable frame between the component and its arm is rejected.

Status (`servo_status` and each teleop session) reports `lag_ms`, the loop's estimate of how far the arm trails its
commands, capped at `limits.max_lag_ms` (default 300, at most 1000). If the arm never moves, the command can run up to that much
travel ahead before the first resync. `limits.resync_deg` is how far the arm may sit from where it
should be, given that lag, before the loop snaps its command back to the measured joints. The gap between command and
measured joints, and so the size of a resync snap, can reach lag travel + `resync_deg`.

The optional `teleop` config block holds `servo_start` defaults for these sessions (`limits`, `rate_hz`, `timeout_ms`,
`ramp_ms`, `target_filter`, `task_weights`, ...) plus `teleop_idle_ms`. `component_name`, `frame` (`world`) and `mode`
(`pose`) are always set.

```json
{ "arms": ["arm"], "teleop": { "timeout_ms": 1000, "limits": { "linear_mm_s": 200 } } }
```

and on the teleop service: `"motion_service": "<this service name>"`.
