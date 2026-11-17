# SO-101 hardware runbook — Phase 3c stages 1 and 2

Spec: `docs/superpowers/specs/2026-11-15-diff-ik-phase3c1-design.md`. Plan: `docs/superpowers/plans/2026-11-15-diff-ik-phase3c1.md`, Part C.

## 1. Setup

- **Machine:** `laptop`, part `laptop-main`. Its viam-server runs on this Mac (`viam-server -config ~/Downloads/viam-laptop-main.json`).
- **Arm:** `follower-arm` (`devrel:so101:arm` v0.14.0).
- **CLI address:** the server's plaintext loopback listener. Find it with
  `lsof -nP -iTCP -sTCP:LISTEN -a -p $(pgrep -f viam-laptop-main)`. Use the `127.0.0.1:<port>` line, with `--insecure`. It was `127.0.0.1:57130` on 2026-11-15, and it changes when the server restarts.
- **Build the CLI once:** `go build -o bin/cli ./cmd/cli`.
- **zsh:** expand a flags variable with `${=FLAGS}`. Plain `$FLAGS` isn't word-split, so the CLI would just print its usage.

## 2. Safety rules

- **During every L1 run and every profiled move,** keep one hand on the power switch and one near the arm. Cutting power drops the arm, because the servos go limp.
- **Ctrl-C while streaming** (the `run` scenarios, `probe`) stops at once. The arm holds its last goal, at most about one tick ahead.
- **Ctrl-C during a profiled move does not stop it.** That covers the start-joints move, `validate`'s moves and the singular pre-move. The CLI prints "a profiled move can only be stopped by cutting power" and waits for the arm. Only power stops it.
- **Never kill the bench during its move to home.** That move is session-based, so a dead client makes the server call `Stop`, and on this driver that means maximum speed.
- **If the CLI hangs in a profiled move,** wait until the arm is still, then `kill -9` it from another terminal. The move's RPC has no deadline, and a second Ctrl-C is absorbed. The CLI runs with sessions off, so killing it doesn't make the server call `Stop`.
- **If the CLI prints "arm.Stop is now disabled for the rest of this process",** the arm didn't settle after a profiled move. Cut power if it needs stopping.

## 3. Start pose `q_start`

- **Joints:** `0,-0.61,0.46,1.39,0` rad, which is (0, −35.0, 26.4, 79.6, 0)°.
- **End effector:** (180, 0, 192) mm, with the tool tilted 19° from straight down.
- **Pad end:** about 92 mm above the base plane, and about 169 mm out from the shoulder-pan axis (208 mm from the base origin, which sits 38.8 mm behind that axis).
- **Sign-off:** the user confirms `q_start` is clear on the real setup. ___

## 4. Pre-servo checks (once, before L1)

1. **Calibration and direction.**
   - Go to `q_start` with `bin/cli … --start-joints 0,-0.61,0.46,1.39,0 probe`. Hands ready: the move uses the driver's 30°/s.
   - Measure the pad end: height above the table ___ mm, and reach from the pan axis ___ mm.
   - Expected: height ≈ 92 mm + the base-plane offset, and reach ≈ 169 mm.
   - Jog each joint +5° from `q_start`, one at a time, with `--start-joints` and the one value changed. Note whether each direction matches the URDF:
     - j0 ___
     - j1 ___
     - j2 ___
     - j3 ___
     - j4 ___
   - **Stop rule:** a mismatch over 10 mm, or a flipped sign, stops stage 1.
2. **Base-plane offset.** The URDF base plane (z = 0) sits ___ mm above the tabletop. So `--min-z-mm` = 30 − offset = ___.
3. **Limits.**
   - Read the driver's calibrated limits and record them: ___.
   - Compare them with the URDF. `q_start`, and the singular `qa`/`qb` below, must be at least 10° inside both.
4. **Droop.**
   - At `q_start`, wait 2 s, then record commanded minus measured for each joint: ___ °.
   - Record the end-effector error |FK(commanded) − FK(measured)|: ___ mm.
   - Set `resync_deg`: `resync_deg` is divergence beyond the measured lag. Keep the default (3 ticks at max joint speed), and raise it only for steady droop (to about 2 × the largest per-joint droop) = ___. Drop the old lag-sized overrides (16.2, 9.6, 4.8).
   - **End-effector error ≤ 1 mm:** the 2 mm bars stand.
   - **> 1 mm:** set `--tol-mm` (and the bench's `--tol-mm`) to 2 + that error = ___, and record the sag as a loop finding.
   - **> 5 mm:** stop and discuss before any servoing.
5. **Sign-off on `qa` and `qb`.**
   - These are for `run singular --wrist 2 --singular-deg -74`. `qa` is `q_start` with joint 2 at −74° + 0.2 rad, and `qb` has joint 2 at −74° − 0.2 rad.
   - Both put the arm near-vertical overhead: end effector at about (63, 0, 377) and (1, 0, 379) mm.
   - User sign-off: ___

## 5. Commands per level

Set the level's variables, then run the commands in this order. **Read the late line after every run** (section 6).

| Level | `J` (°/s) | `LIN` (mm/s) | `ANG` (°/s) | `CIRC` (mm/s) | bench `TWIST` (mm/s) |
|---|---|---|---|---|---|
| L1 | 30 | 40 | 20 | 32 | 30 |
| L2 | 60 | 70 | 30 | 50 | 50 |
| L3 | 90 | 100 | 45 | 50 | 50 |

```zsh
ADDR=127.0.0.1:<port>; MINZ=30; R=<4.8 at L1, 9.6 at L2, 14.4 at L3>; RATE=<probe rate>; TOL=()   # TOL=(--tol-mm <T>) if check 4 raised it
J=30 LIN=40 ANG=20 CIRC=32 TWIST=30 LVL=L1
F=(--address $ADDR --insecure --arm follower-arm --task pointing
   --start-joints 0,-0.61,0.46,1.39,0 --min-z-mm $MINZ
   --max-joint-deg-s $J --max-linear-mm-s $LIN --max-angular-deg-s $ANG --resync-deg $R $TOL
   --singular-eps 0.03 --max-damping 0.05 --rate $RATE)

bin/cli $F probe                                   # once, before L1, at --rate 50, then 40, 30 … per the rate rule
bin/cli $F --settle-tol 2e-3 validate              # once, informational
bin/cli $F --hold 1s --csv .runs/3c1-$LVL-static.csv run static
bin/cli $F --wrist 2 --singular-deg -74 --csv .runs/3c1-$LVL-singular.csv run singular
bin/cli $F --circle-mm-s $CIRC --csv .runs/3c1-$LVL-circle.csv run circle
```

**Bench.** This runs only after the `diff-ik` module and `servo` service have been added to `laptop-main`, with the user's OK. Start it at `q_start`.
```zsh
bin/cli $F probe >/dev/null   # any short CLI call with --start-joints parks the arm at q_start
uv run clients/python/servo_bench.py --address $ADDR --insecure --arm follower-arm --task pointing \
  --joint-deg-s $J --linear-mm-s $LIN --angular-deg-s $ANG --rate $RATE --timeout-ms 300 \
  --twist-mm-s $TWIST --circle-mm-s $CIRC --resync-deg $R --singular-eps 0.03 --max-damping 0.05 [--tol-mm <T>]
```

**Unreachable pointing** (DoCommand, bench level):
- Start from `q_start`.
- `servo_start` with the level's limits, mode `pose`.
- `servo_target` at the current position, with the tool axis rotated 20° about base z.
- Bar: `converged`, position within the tolerance, and the residual within ±1° of the geometry's prediction.

## 6. After every run

- Read `laptop-main`'s log (`read_machine_logs`) for the driver's `streamed N points … late K (max M)` line.
- Record max late: ___.
- **Bar:** max late < 1 tick (20 ms at 50 Hz). The bar fails if the line is missing.

## 7. Between levels

- Servo temperature and load, if the driver exposes them (check its DoCommand); otherwise write "not exposed": ___.
- Go to the next level only after a clean pass and the user's go-ahead.

## Sim rehearsal (2026-11-15, `mujoco-sim-mac`, L1 flags, `--min-z-mm -70`)

| Run | Result |
|---|---|
| probe | reads p50 1.3 ms / p99 2.8 ms; read+send p50 2.3 ms / p99 9.3 ms; 1 overrun |
| validate `--settle-tol 2e-3` | FK 0.033 mm / 0.011°; Jacobian 0.9 % / 0.0 % — PASS |
| static `--hold 1s` | 10/10 converged in 1.2–1.8 s; hold final 0.22–0.27 mm; 49.92 Hz; 0 resyncs |
| singular | reached; peak 29.8°/s (limit 30); ≤ 1 oscillation |
| circle 32 mm/s (no feedforward) | mean 15.8 mm, max 17.4 mm; 49.91 Hz |
| floor guard `--min-z-mm 200` | rejected the start-joints path before moving |
| Ctrl-C during the start-joints move | warning printed, waited for the move, exit 1 |
| bench (L1 flags) | ALL PASS; circle mean 1.1 mm; dead-man 544 ms (≤ 590); stop travel 3.3 mm (ideal 3.0) |

## Hardware log (2026-11-15)

**Pre-servo checks: all pass.**

| Check | Result |
|---|---|
| 1 | Pad end measured 90 mm above the table (FK 92) and 160 mm from the pan axis (FK 169; 9 mm off, just inside the 10 mm stop rule, probably ruler error at the pad tip). All five +5° jogs moved in the URDF's direction. |
| 2 | The base plane sits on the table, offset 0, so `--min-z-mm 30`. |
| 3 | Calibrated limits (deg): j0 −116.0…109.2, j1 −98.5…106.1, j2 −110.2…**84.8**, j3 −108.2…107.4, j4 −172.6…176.5. The CLI loads the arm's model, which carries these limits. Tightest margins: j2 at `qb` 11.3° (to calibrated −110.2 / URDF −96.8), and j3 at `q_start` 15.4° (to URDF 95). |
| 4 | Droop after 2 s at `q_start`: at most 0.35° per joint, end-effector error 0.79 mm. That's under 1 mm, so the 2 mm bars stand and `resync_deg` stays at the default (3 ticks). |

**Bus probe at 50 Hz.** Reads p50 7.5 ms / p99 14.1 ms; read+send p99 12.1 ms with 2 overruns. The driver logged `streamed 499 points … late 0 (max 0s)`. Runs use 50 Hz.

**Finding: pan (servo 1) and wrist roll (servo 5) hunted while holding.**
- j0 swung 10 counts peak to peak (0.88°) and j4 11 counts, at every pose, including with viam-server stopped. These two joints carry no gravity load. The gravity-loaded joints were steady.
- Effect: `validate` stopped at config 32, because `safeArm`'s stillness check never passed.
- The registers matched the driver's tuned table: pan P32/D48/I0, roll P48/D48/I0, min startup force 16, deadband 1.
- **Tuning** (viam-server stopped; `feetech-servo` via a scratch tool; step test = +40 counts and back at 200 counts/s, then a 3 s sample):

  | Pan setting | Hunting after the step (counts peak to peak) |
  |---|---|
  | D 48 → 64 | no change: 10 |
  | min startup force 16 → 8 | no change: 10 |
  | deadband 2 | 7–11 (looked fixed at rest, but still hunted after a move) |
  | **deadband 3, D 48** | **0, 0, 0, 1, 0, 0** |
  | deadband 4 | 0, with a larger settle error |

  Roll at deadband 3: 0, 0, 1, 0.
- **Applied:** servos 1 and 5 at CW/CCW deadband **3**. All gains stay at the driver's table. The driver never writes deadband registers, so no config change is needed.
- **Cost:** a settle offset of up to 3 counts (0.26°, about 0.8 mm at this reach).
- **Verified through the driver:** moves pass the stillness check the first time, and 8 reads in a row are identical.

**L1 CLI runs** (50 Hz; late 0 in every driver line):

| Run | Result |
|---|---|
| `validate` (informational) | 100/100 configs, every move settled. FK 0.236 mm / 0.088°; Jacobian worst case 90.5 % linear / 16.7 % angular. The relative error blows up on near-zero motion: a 1° nudge against a 3-count deadband. |
| `static --hold 1s` (`resync_deg` 1.8) | **PASS.** 10/10 converged in 1.1–2.2 s, at most 2.00 mm / 1.68°. Hold worst 3.15 mm, final 0.69–1.41 mm. 0 overruns, 0 resyncs, tick p99 2.4 ms. |
| `circle` 32 mm/s, `resync_deg` 1.8 (the spec's original rule) | mean 21.9 mm, max 46.0 mm, **23 resyncs**. Lag j0 5 / j1 4 / j2 5 / j3 4 ticks; worst cmd − meas: j0 2.22°, j1 1.91°. |
| `circle` 32 mm/s, `resync_deg` 4.8 (the revised rule) | mean 17.3 mm, max 19.8 mm, **0 resyncs**. |
| `singular` (first run) | reached and within velocity limits (peak j2 31.2°/s, bar ≤ 33), but the oscillation check FAILED: j0 had 7 measured reversals in 1.7 s. The pan's commands held within 0.07°, while its encoder stepped through 6 values (5 counts). That's mechanical wobble inside the deadband, and the metric's minStep (0.06°) is below one encoder count. |
| `singular` (bar on commanded joints) | **PASS.** Reached; peaks j1 24.2, j2 31.1, j3 25.0 °/s; oscillations commanded 0/1/1/1/0, measured 3/1/1/1/0; 50.01 Hz; 0 resyncs. |

Every driver `streamed` line so far reports late 0 (max 0s).

**L1 bench** (after the config change; the backup is `.runs/laptop-main-config-before-servo.json`; `resync_deg` 4.8):

| Step | Result |
|---|---|
| Smoke (9 checks) | **PASS** |
| Dead-man: last cmd → `stopped_timeout` | **528 ms** (bar ≤ 590). Arm stationary at 706 ms with the 0.05° threshold. |
| Feedforward circle, 32 mm/s | **PASS: mean 3.3 mm**, max 5.6 mm, 0 resyncs, residual 0.3° (sim 1.1 mm) |
| Latency and stop at `--motion-eps-deg 0.05` | Invalid: latency median 4 ms, and the arm never read as stationary, because a single-count flicker (0.088°) counts as motion. |
| Latency and stop at `--motion-eps-deg 0.2` (re-run) | command → motion median 132 ms, max 155 ms; `servo_target` round trip 0.8 ms; `servo_stop` → stationary 221 ms; 5.5 mm travel against an ideal ramp of 3.0 mm (the extra ≈ 4–5 ticks of lag × 30 mm/s). |
| Unreachable pointing (axis rotated 20° about base z) | **PASS:** converged, 0.00 mm, residual 6.44° (predicted 6.47°: asin(\|a·n\|) for the pan plane), damping 0. |

**L1 complete: every bar passes.** Use `--motion-eps-deg 0.2` for the bench on hardware.

**L2** (60°/s, 70 mm/s, 30°/s; `resync_deg` 9.6; bench twist and circle 50 mm/s; `--motion-eps-deg 0.2`):

| Run | Result |
|---|---|
| `static --hold 1s` | **PASS.** 10/10 converged in 1.2–2.0 s at ≤ 1.99 mm; hold worst 2.96 mm, final 0.53–1.49 mm; 0 resyncs; tick p99 8.2 ms. |
| `singular` | **PASS.** Peaks j1 37.9, j2 43.2, j3 34.8 °/s (bar ≤ 66). Commanded oscillations 0/0/1/0/0. Measured j2 had 10 reversals: elbow stick-slip while tracking slowly (holds, then jumps 0.1–0.5°); after the catch-up, cmd − meas stays within ±0.5°. Peak start-up lag 5.7°, under the 9.6 resync. |
| `circle` 50 mm/s (no feedforward) | mean 25.6 mm, max 28.7 mm, 0 resyncs (3b sim at 50 mm/s: 22.4). Lag: j0 6, j1 5, j2 6, j3 5 ticks; max cmd − meas 3.5°. |
| Bench | **ALL PASS.** Feedforward circle at 50 mm/s: mean 5.1 mm, max 7.5 mm, 0 resyncs. Dead-man → `stopped_timeout` 525 ms (≤ 590); arm stationary at 1102 ms (stick-slip creep after the ramp). Stop: 287 ms to stationary, 9.0 mm travel against an ideal 5.0 mm. Latency median 133 ms. |
| Unreachable pointing | **PASS.** Residual 6.55° (predicted 6.55°), 0.00 mm. |

Driver `streamed` lines: all 25 streams in L1 and L2 report late 0 (max 0s). Servo temperature and load: not exposed (`controller_status` returns only bus and calibration state).

**L3** (90°/s, 100 mm/s, 45°/s; `resync_deg` 16.2, by the rule with 6-tick lag; bench twist and circle 50 mm/s):

| Run | Result |
|---|---|
| `static --hold 1s` | **PASS.** 10/10 in 1.2–2.1 s; final 0.15–1.38 mm; 0 resyncs; tick p99 3.1 ms. |
| `singular` | **PASS.** Peaks 41.4 / 36.1 / 33.7 °/s; commanded oscillations 0; measured j2 8 (stick-slip). |
| `circle` 50 mm/s | mean 24.8 mm, max 27.7 mm, 0 resyncs. Lag j0 5, j1 4, j2 5, j3 4 ticks; max gap 3.6°. |
| Bench | **ALL PASS.** Feedforward circle 4.0 / 6.2 mm; dead-man 539 ms (stationary at 558 ms); stop 218 ms, 7.7 mm; latency median 81 ms. |
| Unreachable pointing | **PASS.** 6.58° against 6.59° predicted; 0.21 mm. |

**Stage 1 complete.** Results: `docs/results/2026-11-phase3c1.md`.

## Stage 2: leader-arm teleop

Spec: `docs/superpowers/specs/2026-11-15-diff-ik-phase3c2-design.md`. Plan: `docs/superpowers/plans/2026-11-15-diff-ik-phase3c2.md`, Part C. The client is `clients/python/teleop.py`. It reads the leader arm's end pose and streams it to the follower through the `servo` service's pose task.

### Setup

- Enable `leader-arm` and `leader-gripper` on `laptop-main`. This is a config change, so get the user's OK and back up the config first.
- Find the plaintext listener as in stage 1 (section 1). The port changes when the server restarts.
- **Before every run, park the follower at `q_start`:** `bin/cli --address $ADDR --insecure --arm follower-arm --min-z-mm 30 --start-joints 0,-0.61,0.46,1.39,0 pose`. This is a profiled move, so the stage-1 rules apply: hands ready, and Ctrl-C won't stop it. The client refuses to start if the follower is below the floor. Put the leader roughly in the same pose too, so the engage move is short.
- Check that both arms' calibrated limits match. Run `bin/cli … --arm leader-arm pose` and `bin/cli … --arm follower-arm pose` without `--start-joints`, so nothing moves. Record both outputs here.

### Safety

- **The leader goes limp when the client releases it.** Hold it, or rest it on a support, before pressing Enter.
- **Keep a hand near the follower's power switch** for every run.
- **Ctrl-C stops the follower** through `servo_stop`. The client prints "stopping, wait…", re-torques the leader, and ignores a second Ctrl-C until that is done.
- **If the client crashes,** the follower stops within 300 ms (the dead-man timeout), but the leader falls because nothing re-torques it. SIGTERM (a plain `kill`) takes the same clean exit as Ctrl-C. A hung RPC becomes an error after 0.5 s and also exits cleanly.
- **Keep the fast move bounded:** at or under about 300 mm/s, and for no more than about 0.5 s.

### Commands

Set `ADDR=127.0.0.1:<port>`. Check the read timing first. This only reads the leader, and the follower doesn't move.

```
uv run clients/python/teleop.py --address $ADDR --insecure --dry-run
```

If it reports "DOES NOT FIT", add `--rate 40` (or 30) to every session command, record the rate here, and judge "tick p99 < period" against the new period.

Smoke run (10 s) with the leader still torqued, so it never goes limp. It checks engage and rest convergence on the real follower. The gripper mirror is off in this mode.

```
uv run clients/python/teleop.py --address $ADDR --insecure --no-release --duration 10 --csv .runs/3c2-smoke
```

The tuned-profile session (60 s):

```
uv run clients/python/teleop.py --address $ADDR --insecure --csv .runs/3c2-tuned --duration 60
```

The smooth-profile session (60 s), after switching `follower-arm` to `servo_gain_profile: "smooth"`:

```
uv run clients/python/teleop.py --address $ADDR --insecure --csv .runs/3c2-smooth --duration 60 --tol-mm 13 --tol-deg 5
```

Analyse a run:

```
uv run clients/python/teleop_lag.py .runs/3c2-tuned
```

### Protocol

Each run lasts about 60 s: slow sweeps, one bounded fast move, pick-like motions with the gripper, then rest the leader on a support above the floor for at least 2 s.

- **Engage:** rest the leader on a support. Engage ends once the follower is within 10 mm and 5° of the leader for 0.2 s (30 s timeout). Keep the leader roughly still until it has caught up.
- **Smooth profile:** about 11 mm of droop at a target clamped to floor + 15 mm leaves only about 4 mm before the floor watchdog trips. Keep the leader well above the table.
- **Ctrl-C:** the client prints "stopping, wait…" and ignores a second Ctrl-C until the leader is re-torqued.

### After every run

Read the driver's `streamed … late` lines with `read_machine_logs`. The bar is 0 late.

### Results log

(Filled in during Part C.)

**Hardware log (2026-11-15).** Full results are in `docs/results/2026-11-phase3c2.md`.
- **Calibrated limits** (leader / follower, deg): j0 −101.6…134.9 / −116.0…109.2; j1 −105.8…102.9 / −98.5…106.1; j2 −98.3…95.6 / −110.2…84.8; j3 −111.3…93.5 / −108.2…107.4; j4 −167.6…169.6 / −172.6…176.5. The leader's folded rest pose is outside the follower's range.
- **Dry run:** leader read p50 1.8 ms, p99 2.6 ms, so 50 Hz fits.
- **Settings used:** `--linear-mm-s 150 --joint-deg-s 90 --angular-deg-s 45 --resync-deg 16.2 --min-z-mm -10`, plus `--tol-mm 13 --tol-deg 5` for `smooth`.

## Stage 3: AprilTag visual servoing

Spec: `docs/superpowers/specs/2026-11-16-diff-ik-phase3c3-design.md`. Plan: `docs/superpowers/plans/2026-11-16-diff-ik-phase3c3.md`, Part C. The client is `clients/python/tag_servo.py`. It servos the follower to hover over an AprilTag, sending camera-frame targets that the `servo` module transforms into the arm frame (`servo_start` `frames: true`).

### Hardware setup (record the measured values here)

- **Wrist camera** (`wrist-cam`): rigidly mounted on the gripper, frame parent `follower-arm`.
- **Scene camera** (`scene-cam`): fixed, 0.5–1 m from the workspace, oblique, frame parent `world`.
- **ChArUco board for the table** (A4): dictionary ___, squares ___ × ___, square ___ mm, marker ___ mm.
- **Small ChArUco board for the gripper** (about 100 × 70 mm): dictionary ___, squares ___ × ___, square ___ mm, marker ___ mm. Mount it rigidly; it must not flop.
- **Two tag36h11 tags** on rigid cards: IDs ___ and ___, `tag_width_mm` ___ (edge length of the black square: not the diagonal, not including the white border).
- **Offsets for the calibration `workspace_bounds`:**
  - wrist camera to the lowest jaw point: ___ mm;
  - gripper board to the lowest jaw point: ___ mm.

### Config changes

Each one needs the user's OK and a backup of the config first.

1. Enable both cameras (`rdk:builtin:webcam`).
2. Add the `viam:apriltag` module. Add one `viam:apriltag:pose_tracker` per camera (`camera_name`, `tag_family: "tag36h11"`, `tag_width_mm`), named e.g. `wrist-tags` and `scene-tags`.
3. Add the `viam:camera-calibration` module:
   - a `charuco` pose tracker per board/camera pair;
   - an `eye_in_hand` service (wrist camera) and an `eye_to_hand` service (scene camera);
   - `workspace_bounds` with `mode: "absolute"` (bounds on the camera pose, or on the board pose, *not* the jaw: z.min = table + the offset above + 20 mm);
   - `input_range_override` inside the stage 1–2 joint ranges, containing the seed pose;
   - `settle_seconds` 2.
4. Add a **table obstacle**: a frame-system component with box geometry, its top 1–2 mm below the arm's base plane. The calibration module plans with no WorldState, so only frame-system obstacles count.
5. **Before calibrating**, temporarily lower the follower's `speed_degs_per_sec` (20) and `acceleration_degs_per_sec_per_sec`. **Restore them afterwards.**
6. Rebuild `bin/diff-ik` and restart its module after any module change.

### 1. Intrinsics (each camera)

```
uv run clients/python/intrinsics.py --address $ADDR --insecure --camera wrist-cam \
  --dict DICT_5X5_100 --squares 7x5 --square-mm <s> --marker-mm <m> --views 20
```

- Move the board through the whole view: centre and corners, near and far, tilted.
- **Bar:** RMS < 1 px.
- Paste the printed `intrinsic_parameters` and `distortion_parameters` into the camera's config (config change).
- **Note:** `viam:apriltag` uses only fx, fy, ppx and ppy. It never applies distortion, which is an accuracy factor away from the image centre.

### 2. Extrinsics (`viam:camera-calibration`)

- **Wrist camera, `eye_in_hand`:** the A4 board flat on the table.
- **Scene camera, `eye_to_hand`:** the small board on the gripper.

Steps:
1. Move the follower to a **high** seed pose with a clean view of the board (`bin/cli … pose --start-joints …`).
   - The final return to the seed is an unplanned single joint move.
   - Auto-apply runs before that return. If it reconfigures the module, the arm stays at its last station.
2. Send `{"calibrate": {}}` to the service.
   - **Never send `cancel`, and never press Stop in the app, during a sweep.** Both call the driver's `Stop`, which means max speed during a profiled move. Abort by cutting power.
   - Hand on the follower's power switch, the other near the arm.
3. **Bars:** translation residual ≤ 5 mm, rotation ≤ 2°, mean reprojection ≤ 2 px. Record the `quality` block and `auto_applied`.
4. **After `eye_in_hand` auto-applies,** check the wrist camera's frame parent is `follower-arm` (or a 0-DoF frame). Parented to the input-enabled gripper, the module's chain check rejects it.
5. Restore the follower's speed and acceleration.

### 3. Calibration check (each camera)

Put a tag flat at a ruler-measured position in the arm's base frame. Read its pose through the module: `servo_target` with `frame: <camera>` and `apply: false` on a `frames: true` session, or run `tag_servo.py --axis down` and read `tagarm_*` (the tag's arm-frame pose) in the CSV, or `tagarm_mean_mm` from `tag_lag.py`. Compare with the ruler. Record the error: ___ mm.

### 4. Runs (each camera)

Common flags: `--address $ADDR --insecure --camera <cam> --tracker <tracker> --tag-id <id>`. The defaults are L2 limits, `--min-z-mm -10` and `--timeout-ms 300`.

1. **Static noise** (tag still, at the tracking distance, filter off):
   ```
   uv run clients/python/tag_servo.py … --hover-mm 150 --ema-alpha 1 --hold-mm 0 --hold-deg 0 --duration 10 --csv .runs/3c3-<cam>-noise
   uv run clients/python/tag_lag.py .runs/3c3-<cam>-noise --rest all
   ```
   Set `--hold-mm` and `--hold-deg` to about 3× `raw_std_mm_axes` and 3× `raw_normal_std_deg` for the runs below.
2. **Tracking** (60 s; hand-held tag moved slowly, then held still ≥ 3 s), with `--feedforward on`, then `--feedforward off`:
   ```
   uv run clients/python/tag_servo.py … --hover-mm 150 --hold-mm <h> --hold-deg <d> --feedforward on --duration 60 --csv .runs/3c3-<cam>-track-ff
   ```
   - Bars: driver late 0; overruns 0; max tick p99 < 20 ms; no visible jitter (user).
   - Check `det_rate_hz` in `tag_lag.py`; if below ~5 Hz, raise `--gate-stale-s` from 0.2 to ~0.4.
   - If there's jitter with `tuned`, repeat with `servo_gain_profile: "smooth"`. That is a config change on `follower-arm`: it needs the user's OK and a config backup first (see Config changes).
3. **Static accuracy** (tag flat on the table at 5 positions; `--axis down`):
   - scene camera: `--hover-mm 20` (closed loop), measure once converged (CSV state is `converged`);
   - wrist camera: `--hover-mm 150 --final-hover-mm 20` (look, then move), measure at "measure now".
   - Measure the horizontal offset from the tag centre of **the point on the tool axis between the pads** (the TCP is 6.8 mm off-axis and roll is free), and its height.
   - Target ≤ 10 mm (informational).
4. **Timing skew** (wrist camera, once): static tag; alternate separate runs at `--hover-mm 120` and `--hover-mm 180`, each with `--ema-alpha 1 --hold-mm 0 --hold-deg 0 --duration 5`, analysed with `tag_lag.py PREFIX --phase all`. It reports `skew_ms`.

### After every run

- Read the driver's `streamed … late` lines (`read_machine_logs`).
- Run `uv run clients/python/tag_lag.py .runs/<prefix>`.

### Safety

- **Tag lost:** if no detection arrives for 100 ms, the client holds the last applied arm-frame target. After 0.5 s it stops the session. If the client dies, the dead-man stops the follower within 300 ms plus the 200 ms ramp.
- **Targets are floor-clamped in the module** (both camera-frame and arm-frame), and the watchdog checks the follower's real pose.
- **Ctrl-C and SIGTERM** exit cleanly, as in stage 2.
- **Calibration sweeps** are the only planned or profiled motion: never `cancel`, never Stop, start high, hands on power.

### Results log

(Filled in during Part C.)

### Stage 3 sim rehearsal (2026-11-16, `mujoco-sim-mac`, `--min-z-mm -70`)

| Step | Result |
|---|---|
| B0 teleop self-tracking | 49.99 Hz, rest convergence 100 % (0.01 mm), 0 overruns; SIGINT, SIGTERM, `--fault-after` all exit cleanly |
| B1 `frame_check.py` | camera round trip 0.0000 mm; +30 mm via `scene-cam` → dz 29.97 mm; below-floor target clamped, lowest z −55 (floor −70); `apply:false` moved 0.014 mm; zero pose in `arm` previews as the EE pose (0.034 mm) |
| B2 fake tag, hover 150 | **first run failed:** wrist flex parked at its 95° limit, 38° of pointing error reported as reachable, engage timed out. Fixed in a846336 + 722bfa2 (joints at a limit and pushed outward are locked in both the reach split and the solve). After: converged 181/191 track ticks, mean err 0.22 mm, residual 35° reported as unreachable, det 18.6 Hz, 0 overruns |
| B2b look-then-move, `--axis down`, hover 150 → 20 | converged, descended, "measure now"; hold 45 ticks converged, follower EE z 145.6 (jaw tip ≈ z 40 over a tag at z 20); no loss trip |
| B3 `--fake-tag-drop-after 5` | 92 detections, 4 keepalive re-sends, "tag lost: stopped", exit 0 |

**Hardware note:** a tag pose whose normal the 5-DOF arm can't match now converges on position and reports the gap as `pointing_residual_deg`, instead of hanging in engage.

### Point-velocity A/B (2026-11-16, L2 bench, 50 Hz, 5 ms lead, `stream_speed_margin` 1.3, `tuned` gains)

Driver `teleop-smooth` f7af962 (speed-profiled streaming uses `Constraints.Velocities` when present, else Δq/dt); diff-ik 3f10fca (`armIO` sends `(q − prevQ)/period`; `servo_start` `point_velocities`, bench `--no-point-velocities`). Runs alternated on, off, on, off; raw output in `.runs/ab-vel-*.txt`.

| | on 1 | on 2 | off 1 | off 2 |
|---|---|---|---|---|
| command → motion, median / max | 88 / 102 ms | 89 / 99 ms | 89 / 107 ms | 91 / 103 ms |
| stop sent → stationary | 196 ms | 189 ms | 205 ms | 214 ms |
| travel after stop (ideal 5.0) | 6.9 mm | 6.7 mm | 6.6 mm | 6.7 mm |
| dead-man → `stopped_timeout` | 535 ms | — | 535 ms | — |
| last cmd → arm stationary | never (creep) | 514 ms | 548 ms | 1019 ms |
| circle 50 mm/s, mean / max | 3.2 / 5.0 mm | 3.6 / 5.5 mm | 3.4 / 5.4 mm | 3.0 / 4.9 mm |

Driver late 0 on all 28 streams. **No measurable difference** on this servo: the driver's Δq/dt estimate already equals the loop's velocity apart from tick jitter, and everything is inside run-to-run noise. Stop-to-stationary is 10–25 ms shorter with velocities on in both pairs, too small to call with n = 2. The dead-man "stationary" times are stick-slip creep noise, not the change. Keep velocities on (default): they're what a trajex producer (`TempStreamArmJointPositions`) needs on this driver, and they avoid Δq/dt's max-speed request on bunched points.

### Driver streaming defaults fixed (2026-11-16)

so-101 `teleop-smooth` d17f8ba + a7f67f1: speed-profiled streaming is the only streamed behaviour (margin 1.3, floor 10°/s, Acc unlimited, per-point velocities when present); the `stream_speed_margin`, `stream_min_speed_deg_s`, `stream_accel_deg_s2` and `stream_substeps` attributes are gone (old configs still load: unknown keys are ignored). Removed `stream_speed_margin` 1.3 and `stream_min_speed_deg_s` 10 from `follower-arm` on `laptop-main` (backup: `.runs/laptop-main-follower-arm-before-stream-defaults.json`).

Confirmation L2 bench with no stream attributes: latency median 86 / max 99 ms, stop → stationary 184 ms (7.0 mm), dead-man 536 ms, circle 3.6 / 5.4 mm, 0 resyncs, ALL PASS, late 0 on 7 streams. Same as the configured runs above.
