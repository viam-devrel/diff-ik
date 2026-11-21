# Module diff-ik

Cartesian servoing for Viam arms: a motion service that tracks a stream of pose targets or twists with a 50 Hz
differential-IK loop, with speed and acceleration limits and a mandatory dead-man on every session. Use it for
teleoperation, visual servoing and learned policies, where the target keeps moving. It also works as the motion
backend of [`viam:teleop`](https://github.com/viam-modules/teleop).

This is a proof of concept. Results are in `docs/results/`.

## Models

This module provides the following model(s):

- [`devrel:diff-ik:servo`](devrel_diff-ik_servo.md): Cartesian servo sessions on arms through `DoCommand`, plus `viam:teleop`'s `teleop_*` commands

## Solvers

`servo_start` takes `"solver": "dls"` (the default) or `"qp"`. `qp` is opt-in and experimental: it enforces joint
position, velocity and acceleration limits as hard bounds and adds a null-space posture task. Design:
[`docs/superpowers/specs/2026-11-18-diff-ik-qp-solver-design.md`](docs/superpowers/specs/2026-11-18-diff-ik-qp-solver-design.md).
Results: [`docs/results/2026-11-qp-solver.md`](docs/results/2026-11-qp-solver.md). Fields are in
[`devrel_diff-ik_servo.md`](devrel_diff-ik_servo.md).

With `qp`, collision avoidance is also opt-in (`collision`, `world_state`, `collision_specification`). It keeps the arm
and its mounted parts at least `d_min` (10 mm) from fixed obstacles with a per-pair distance damper in the QP, and takes
the motion service's `WorldState` and `CollisionSpecification` shapes. Boxes, capsules, spheres and points only; a
frame-system mesh (such as a gripper's model) is checked as its bounding box. Design:
[`docs/superpowers/specs/2026-11-18-diff-ik-collision-design.md`](docs/superpowers/specs/2026-11-18-diff-ik-collision-design.md).
Results: [`docs/results/2026-11-qp-collision.md`](docs/results/2026-11-qp-collision.md).

Collision can also take obstacles from a vision service while a session runs (`live_obstacles`, with the service named
in the module's `vision_services`). The module polls it, backs the arm away from anything inside `d_min`, stops its approach and retreats softly when
pinched, waits for the first detections before moving, and ramp-stops as `stopped_stale` if they stop arriving. It has no
velocity prediction. Design:
[`docs/superpowers/specs/2026-11-18-diff-ik-live-obstacles-design.md`](docs/superpowers/specs/2026-11-18-diff-ik-live-obstacles-design.md).
Results: [`docs/results/2026-11-live-obstacles.md`](docs/results/2026-11-live-obstacles.md).

A session can also stop itself on contact (`guard`), watching a wrist force-torque sensor named in the module's
`sensors`. When it trips, the session commands the measured joints, so the arm stops pressing, and ends as
`stopped_contact`. In a MuJoCo touch-off onto a rigid table at 10–80 mm/s, it cuts the peak force of a client-side
stop by 3.5–4.5× and drops what is left after the stop from 46–359 N to 0.3–1.5 N. A comparison with the builtin motion
service is in [`docs/results/2026-11-builtin-comparison.md`](docs/results/2026-11-builtin-comparison.md). Results:
[`docs/results/2026-11-contact-guard.md`](docs/results/2026-11-contact-guard.md).

A twist session can also hold a pressing force along the tool axis (`force`) while the client's twist sweeps it
sideways, as in sanding or polishing. In MuJoCo it holds 5–20 N on a rigid table to within 0.35 N (0.1 N spread), settling in 0.2–0.5 s.
In a block-sanding cell (`dev/sanding-mujoco.json`) it averages 9.9–10.3 N over raster passes on a tilted block.
Results: [`docs/results/2026-11-force-control.md`](docs/results/2026-11-force-control.md). With sideways compliance (`comply_xy`), a
spiral search plus force-controlled insertion put a 20 mm peg into a hole with 1 mm clearance from up to 5 mm off, in
25 of 25 trials (`dev/peg-mujoco.json`;
[`docs/results/2026-11-peg-in-hole.md`](docs/results/2026-11-peg-in-hole.md)). The force controller reads mujoco-sim's
`force_torque` and the xArm module's `ft_sensor` readings.

A pose session also takes whole action chunks (`servo_trajectory`: timed poses with the observation's age), as a
learned policy emits them. The module aligns each chunk by its age, plays it ahead by the servo's own lag, crossfades
from the previous chunk, and ends the session if no new chunk arrives after the last one runs out. Against an emulated
policy it matched the best client-side playback with one message per chunk instead of 30 a second:
[`docs/results/2026-11-policy-streaming.md`](docs/results/2026-11-policy-streaming.md).

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

## Example: visual servoing on a moving AprilTag (MuJoCo)

The SO-101 hovers 150 mm above an AprilTag. The sim slides the tag around a 60 mm circle at 10, 20, 40 and 80 mm/s,
and the arm follows it using either the wrist camera (eye-in-hand) or a fixed scene camera (eye-to-hand).
[`viam:apriltag`](https://github.com/viam-labs/apriltag) detects the tag, and
`clients/python/tag_servo.py` streams the detections to a servo session as camera-frame targets.

    go build -o bin/diff-ik ./cmd/module
    viam-server -config dev/so101-mujoco-tag.json   # plaintext localhost:8082
    uv run clients/python/tag_sweep.py --insecure --camera wrist-cam --tracker wrist-tags --csv .runs/p4-wrist
    uv run clients/python/tag_sweep_report.py .runs/p4-wrist

In sim, mean tracking error is about 1 mm per 10 mm/s of tag speed (1.7 mm at 20 mm/s). Nearly all of it is
lag of about 85 ms, and neither camera lost the tag up to 80 mm/s. Setup (apriltag from source, pre-positioning the
arm) is in [`dev/so101-mujoco.md`](dev/so101-mujoco.md), and full results are in
[`docs/results/2026-11-phase4-tag-sweep.md`](docs/results/2026-11-phase4-tag-sweep.md).

## teleop_* compatibility

The service is a drop-in `motion_service` for [`viam:teleop`](https://github.com/viam-modules/teleop). It answers
`teleop_start`, `teleop_move`, `teleop_stop` and `teleop_status` (no `command` key) and `GetPose`.

- `teleop_start` resolves the component to one of `arms` (directly, or by walking its frame-system parents, e.g. a
  gripper on the arm), computes the fixed arm-end-effector-in-component transform once, and opens a `world`/`pose`
  servo session holding the arm's current pose. `Destination` is ignored. A second start for the same component
  replaces its session; an arm with a plain `servo_*` session returns "busy".
- `teleop_move` composes the leader pose with that transform and sets the session target (latest wins, `seq` ignored).
  A missing or ended session returns `component "<name>" not registered in teleop pipeline`.
- `teleop_stop` ramp-stops the session; without `component_name` it stops every teleop session, as builtin does. An
  unknown component is not an error.
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
