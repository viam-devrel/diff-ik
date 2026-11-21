# Diff-IK contact guard: results

A guarded touch-off in MuJoCo: the tool twists straight down onto a rigid table and has to stop at 5 N. Step 1 does it
from a client (poll the wrist F/T sensor, call `servo_stop`); step 2 has the servo session stop itself
(`servo_start` `guard`). Same cell and script for both.

## Setup

- **Cell:** `dev/ft-mujoco.json` on `localhost:8086`. It has a UR5e on a 400 mm pedestal, a Robotiq ePick (its tip is
  132 mm from the flange) and a mujoco-sim `force_torque` sensor at the flange (0.1 N noise, 20 ms filter). The table is a
  fixed obstacle with its top at z = 300 mm. The sim arm's joint servos reach their effort limit at about 2° of error,
  which makes the arm about as stiff as a rigid industrial arm: roughly 45 N per mm of commanded travel past the
  surface.
- **Script:** `clients/python/touch_sweep.py` (helpers checked by `test_touch_sweep.py`).
  - Each trial servos to (550, 0, 470) world with the tool down, tares the sensor, and opens a `twist` session at the
    approach speed (`linear_mm_s` = speed, default 360°/s² joint acceleration). It then streams `[0, 0, −v]` at
    50 Hz.
  - It polls the sensor on its own connection at about 690 Hz and logs |F|. It also logs the flange pose on a third
    connection.
  - Client mode stops with `servo_stop` the moment the tared |F| reaches 5 N. `--guard` passes
    `guard: {sensor: wrist-ft, force_n: 5}` and only watches.
  - Three trials at each of 10, 20, 40 and 80 mm/s, then 1 s of logging after the stop.
- **Measured:**
  - contact: |F| first reaches 1 N;
  - peak: max |F|;
  - residual: mean |F| over the 1 s after the stop. The window includes the client's late peak, so a residual close
    to the peak means the force stayed on.

## Result

Means of three trials. The trials at each speed agree within 5%, apart from the guard's small residuals.

| Approach | Client, `ramp_ms` 200: peak / residual | Client, `ramp_ms` 20 | Guard, first version | **Guard, abort trip** |
|---|---|---|---|---|
| 10 mm/s | 46 / 46 N | 35 / 34 N | 23 / 0.9 N | **13 / 0.3 N** |
| 20 mm/s | 93 / 92 N | 69 / 67 N | 36 / 1.5 N | **21 / 0.4 N** |
| 40 mm/s | 185 / 182 N | 135 / 132 N | 97 / 3.7 N | **41 / 1.0 N** |
| 80 mm/s | 365 / 359 N | 267 / 262 N | 147 / 4.5 N | **81 / 1.5 N** |
| 100 mm/s | | | 235 / 7.5 N | **103 / 2.8 N** |

The "abort trip" column is the current guard. The first version appended the measured joints to the stream and then
let `armIO.Stop` drain it (up to 100 ms), so queued points played out first. The abort trip cancels the stream at once,
calls `arm.Stop`, then moves the arm to the measured joints. It was changed after the builtin comparison
(`2026-11-builtin-comparison.md`) showed a client stopping builtin's `Move` this way doing better.

- **Detection is not the problem.** In every trial the client saw 5 N within 3 ms of contact. The force climbs from
  1 N to 5 N within one or two samples.
- **The arm barely moves past contact.** The flange ends 0.0–0.35 mm below where it touched, but with the client stop the
  force keeps rising for 130–330 ms after contact. When contact is this stiff, the force comes from how far the
  *command* got past the surface, not from how far the arm moved.
- **Why the client stop presses on.** At contact the command is already ahead of the arm by the loop's lag. The ramp
  stop then adds its own travel, and `armIO.Stop` lets the stream play out to its last point. The arm then holds that
  point below the surface, so the peak force stays on as the residual. A shorter ramp helps only by the ramp's share of
  that travel, about 25%.
- **The guard** polls the sensor at 500 Hz in its own goroutine. When it trips, the loop ticks at once, cancels the
  stream, stops the arm and moves it to the *measured* joints. The session ends as `stopped_contact`.
  - Commanding the measured joints releases what the arm was pressing with, so the force left after the stop drops to
    0.3–3 N. That's the tool's weight, re-zeroed by the tare, plus gravity sag.
  - The peak comes 5–10 ms after contact, against 130–330 ms for the client stop.
- **What's left of the peak** still grows with speed, at about 1 N per mm/s, against 4.6 for the client with the
  default ramp. It's the arm's momentum and the few milliseconds of command still in flight, going into a rigid
  surface. The guard can't stop that once contact has happened. The levers are to approach slowly near the expected
  contact, or to add compliance (`2026-11-force-control.md`).

## Limits

- The sim's contact softness can't be set: mujoco-sim obstacles don't expose `solref`/`solimp`. Absolute forces
  depend on that and on the servo gains, so compare the guard and the client with each other, not with a real arm.
- The guard reads `force`/`torque` `{x, y, z}` maps, which is mujoco-sim's format. A real F/T driver's readings may
  name them differently.
- The trip jumps the command back to the measured joints, ignoring the acceleration limit. In contact, the arm is
  already stopped. A false trip in free space is a hard stop from the arm's current speed.
- The guard's baseline is taken at `servo_start`. A tool that picks something up mid-session shifts the baseline by
  the load's weight.

## Reproduce

    go build -o bin/diff-ik ./cmd/module
    viam-server -config dev/ft-mujoco.json
    uv run clients/python/touch_sweep.py --insecure --trials 3 --csv .runs/touch-client-r200
    uv run clients/python/touch_sweep.py --insecure --trials 3 --ramp-ms 20 --csv .runs/touch-client-r20
    uv run clients/python/touch_sweep.py --insecure --trials 3 --guard --csv .runs/touch-guard
