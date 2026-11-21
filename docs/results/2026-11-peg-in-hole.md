# Diff-IK peg-in-hole: results

A spiral search and force-controlled insertion with `servo_start` `force`. A 20 mm peg starts over a 22 mm square hole
(1 mm clearance per side) with up to 5 mm of x/y error. The peg presses onto the plate, spirals outward until it drops
into the hole, then is pushed down until it seats on the bottom. It needed one module addition, lateral compliance
(`force.comply_xy`), and with it the final settings seated 25 of 25 trials.

## Setup

- **Cell:** `dev/peg-mujoco.json` (port 8088), also the DevRelDemos `diff-ik-contact-guard` machine's config. It's the
  sanding cell (UR5e on a pedestal, wrist `force_torque` sensor, the sanding block kept) with two changes:
  - **Tool:** a peg tool, `dev/assets/peg_tool.xml`, loaded as a mujoco-sim gripper `mjcf`. The holder is 50 mm across
    and 40 mm long; the peg is 20 mm across and 60 mm long, with its tip (the TCP) 100 mm from the flange. mujoco-sim
    needs one actuator on a gripper, so it has an unused adhesion actuator.
  - **Fixture:** a 160×160×25 mm plate built from four box obstacles, with a 22 mm square hole centered at
    (550, 180) mm. The plate top is z = 325 mm, the hole bottom is the table at 300 mm, and friction is 0.5. The
    `hole-cam` camera looks at it.
- **Script:** `clients/python/peg.py`, with helpers checked by `test_peg.py`. Each trial:
  1. Moves to the hole plus a random x/y error, uniform over a disc of radius 5 mm (seeded).
  2. Opens a twist session with `force` (target 8 N, `gain` 0.5, `max_mm_s` 8, `comply_xy`) and `guard` at 40 N, and
     waits for contact, then 0.5 s to settle.
  3. **Search:** follows an Archimedean spiral, 1.5 mm pitch, outward from the start. A P controller on the arm's x/y
     streams the twist. The hole counts as found when the tip is 2 mm below the plate surface. If first contact is
     already below the plate (the peg went straight in), the search is skipped.
  4. **Insert:** no x/y twist; the force law drives the peg down. Seated means within 1.5 mm of the 25 mm depth, with
     the press back above 80% of the target.
  5. Lifts straight out.
- **Force readings** come from the session status (`force_press_n`, `force_side_n`), so the client doesn't depend on
  the sensor's reading format.

## Result

| Run (seed 1, same start errors) | Seated | Search | Insert | Peak sideways force |
|---|---|---|---|---|
| `comply_xy` 0 (off), spiral 10 mm/s | 0 / 5* | | stalled at 11 mm (1 trial) | 21 N; 2 guard trips |
| `comply_xy` 0.5, 10 mm/s | 3 / 5* | 2.5–5.4 s | 5.2–6.3 s | 15–36 N; 1 guard trip |
| `comply_xy` 1.0, 10 mm/s | **10 / 10** | 0–8.3 s | 0.1–6.6 s | 1–39 N |
| `comply_xy` 2.0, 10 mm/s | 8 / 10 | 0–8.3 s | 0.2–6.4 s | up to 38 N; 2 guard trips |
| **`comply_xy` 1.0, 6 mm/s (final)** | **10 / 10** | 0–13.3 s | 0.1–6.0 s | **1–11 N** |
| Final, on the demo machine (seed 7) | **5 / 5** | 0–14.2 s | 0.2–6.0 s | 1–12 N |

\* A client bug, since fixed, caused two of the `comply_xy` 0 failures and one of the 0.5 failures: a peg that went
straight in and seated was measured against its own contact height, so it never counted as seated.

- **Without compliance the peg jams.** When the hole is found the peg is off-centre by 1.3 mm, more than the 1 mm
  clearance. Holding x/y presses it into a wall, and friction takes most of the 8 N press: the force read 7 N, and the
  peg crept down at about 0.5 mm/s and stalled 11 mm in. Near the hole the search also catches the edge: the tip dips
  about 1 mm, under the "found" threshold, and the spiral drives it sideways into the wall until the 40 N guard trips.
- **With `comply_xy`, the walls centre the peg.** The tool yields sideways to the contact force. Insertion then takes
  5.6–6.6 s for 25 mm, about 4 mm/s, limited by `max_mm_s` 8 and wall friction.
- **More compliance isn't better.** At 2.0 two edge catches tripped the guard. The catch is an impact, and yielding
  faster doesn't soften it.
- **The search speed sets the edge impact.** Slowing the spiral from 10 to 6 mm/s cut the peak sideways force from
  39 N to 11 N, at the cost of searches of up to about 13 s from 4–5 mm off.
- **Search time grows with the start error**, as an Archimedean spiral does. At 6 mm/s it took 3.8–9.8 s from 2.4–4.7 mm
  off, and 13.3 s twice, probably where the spiral passed the hole on one turn and caught it on the next. Some starts
  within about 2 mm went straight in.

## With the builtin motion service's `Move` only

`clients/python/builtin_peg.py` does the same task with no teleop and no diff-ik session; its helpers are checked by
`test_builtin_peg.py`. Each control cycle:
1. reads the F/T sensor itself (either reading format);
2. advances the z goal by the force law, the spiral and the compliance over a fixed 50 ms;
3. sends one blocking `Move` of the gripper, with a collision specification allowing it to touch the plate pieces and
   the table.

Its guard is client-side: `arm.Stop()` above 40 N.

Same start errors as diff-ik's final run (seed 1), on the demo machine:

| Start error | builtin `Move`: search / insert / peak side | diff-ik: search / insert / peak side |
|---|---|---|
| 1.8 mm | 4.1 s / 7.4 s / 9.6 N | straight in / 0.1 s / 1.2 N |
| 4.4 mm | 18.1 s / 7.9 s / 8.3 N | 8.7 s / 6.0 s / 10.9 N |
| 3.5 mm | 6.0 s / 7.9 s / 9.5 N | 5.8 s / 5.8 s / 8.4 N |
| 4.0 mm | 9.2 s / 8.1 s / 16.1 N | 6.7 s / 5.7 s / 7.9 N |
| 1.5 mm | 9.0 s / 7.8 s / 15.0 N | straight in / 0.2 s / 1.1 N |

- **It works: 5 of 5 seated.** The steps are tiny, so each `Move` plans quickly and the loop runs at 16–20 Hz.
- **It's slower.** Searches take 4–18 s, and insertion 7.4–8.1 s against diff-ik's 5.6–6.0 s. The two pegs diff-ik
  slid straight into, because it yields sideways from first contact, had to search.
- **The goal is open loop.** The client sends goals and doesn't track the arm's actual path. With compliance on during
  the search, the ~3 N of sliding friction drifted the spiral's centre about 6–13 mm away, and one trial never found
  the hole. Compliance had to be limited to insertion. diff-ik's client steers the twist toward the spiral point from
  the *measured* pose, which cancels that drift, so it can stay compliant throughout.
- **Every cycle is a full plan.** Each `Move` solves IK and plans a path. Nothing makes it local: in the sanding
  comparison, one slow `Move` while in contact produced a 290 mm detour. Here the steps stayed small enough that it
  didn't happen, but the API doesn't guarantee it.
- **The client has to know the sensor's frame in the base frame.** That's hard-coded for the commanded tool-down
  orientation. diff-ik rotates by the arm's forward kinematics.

## Sensor readings: mujoco-sim and the xArm

The `viam:ufactory:ft_sensor` model in `viam-ufactory-xarm` returns **flat** readings, while mujoco-sim's
`force_torque` returns nested maps:

| | mujoco-sim `force_torque` | `viam:ufactory:ft_sensor` |
|---|---|---|
| Readings | `force: {x, y, z}` N, `torque: {x, y, z}` N·m | `Fx_N`, `Fy_N`, `Fz_N`, `TRx_Nm`, `TRy_Nm`, `TRz_Nm` |
| Frame | the arm's end-effector frame, +z out of the flange | the sensor's frame |
| Sign | what the tool exerts on the flange: a surface pushing back reads −z | not stated (controller register 0xC8) |
| Tare | `{"command": "tare"}` | `{"tare": true}` (zeroes on the controller) |

diff-ik's poller now reads either format. The guard uses the force's magnitude, so sign doesn't matter there.
Force control needs the sign: `force.press_axis` is `"-z"` (default, mujoco-sim) or `"+z"`, and the sideways
compliance uses the same sign. **The xArm's sign is unverified.** Check it on hardware before using `force` on an
xArm: press on the tool and see which way `Fz_N` moves, then use `"+z"` if pressing reads positive. diff-ik tares in
software at `servo_start`, so the xArm's own tare (and its Studio payload compensation) isn't required, though the
controller's zeroing does no harm. The test clients (`touch_sweep.py`, `press_hold.py`, `sand.py`) still read
mujoco-sim's format directly; `peg.py` reads status and works with either.

## Limits

- **A perfect, square hole with no chamfer, in a rigid world.** It's a 1 mm clearance with no tilt of the peg. Real
  holes are round, chamfered, and slightly misaligned, and real tools and arms give a little.
- **The tool stays vertical.** Insertion has no rotational compliance. A tilted hole would need it.
- **The "found" test is a drop in height**, which needs a flat plate at a known height (`--plate-z`). A force-based
  test (the press falling away) would be more general.
- **`comply_xy` acts in the sensor frame, which is taken to be the tool frame** (true for mujoco-sim, which places the
  sensor at the flange). A sensor mounted rotated about z would need that rotation.

## Reproduce

    go build -o bin/diff-ik ./cmd/module
    viam-server -config dev/peg-mujoco.json   # localhost:8088
    uv run clients/python/peg.py --insecure --trials 10 --csv .runs/peg
    uv run clients/python/peg.py --insecure --trials 5 --comply-xy 0 --speed-mm-s 10   # the jam, for comparison
