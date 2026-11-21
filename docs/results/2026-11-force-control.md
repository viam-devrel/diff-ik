# Diff-IK force control: results

`servo_start` `force` regulates how hard the tool presses along its axis, in a twist session. These runs tune it on a
press-and-hold onto a rigid table, then use it in a block-sanding cell: hold the force while sweeping raster passes
over a tilted block.

## How it works

- Until contact, the tool moves along its axis at `approach_mm_s`.
- At first contact (pressing force ≥ `contact_n`), the command snaps back to the measured joints, as the guard's
  trip does, so the approach's lag-ahead doesn't become a force spike.
- From contact, the speed along the axis is `gain`·e + `ki`·∫e dt, with e = `target_n` − pressing force, clamped to
  ±`max_mm_s`. The integral resets at contact and is clamped to ±`max_mm_s`.
- The client's twist still drives the other directions, which is the hybrid force/position split. The pressing force
  is −(F − F₀)·z in the sensor frame, from the guard's poller (500 Hz, tared at start).

## Press and hold

**Cell:** `dev/ft-mujoco.json`, the cell from `2026-11-contact-guard.md`, with the tool pointing down onto a rigid
table. **Script:** `clients/python/press_hold.py`. Each trial starts 18 mm above contact, logs the pressing force at
about 650 Hz from the client, and holds for 4 s after contact. "Settle" is the time from contact until the force stays
within ±10% of the target. Steady-state numbers are over the last 1.5 s.

**Gain** (10 N target, 10 mm/s approach, one trial each):

| `gain` (mm/s per N) | 0.02 | 0.05 | 0.1 | 0.2 | 0.4 | 0.8 | 1.6 | 3.2 | 6.4 |
|---|---|---|---|---|---|---|---|---|---|
| Settle | never | 4.0 s | 2.3 s | 1.15 s | 485 ms | 283 ms | 393 ms | never | never |
| Steady force ± spread | 5.3 ± 0.5 N | 8.5 ± 0.4 | 9.7 ± 0.1 | 9.8 ± 0.1 | 10.0 ± 0.1 | 10.0 ± 0.1 | 9.9 ± 0.1 | 9.9 ± **2.4** | 9.9 ± **3.3** |

- From 0.4 to 1.6 the hold is steady and settles in under 0.5 s.
- At 3.2 it oscillates, and at 6.4 it never settles.
- The peak was about 20 N at every gain. That's the 10 mm/s impact, the same one the guard couldn't remove.

**Approach speed** sets that peak. With gain 0.8 and a 10 N target, the peak was 20 N at 10 mm/s, and 10.4 N at both
5 and 2 mm/s. At those speeds the impact (about 10 N at 5 mm/s) is below the target.

**Final check** (5 mm/s approach, 3 trials per cell, peak / settle / steady force):

| Target | `gain` 0.5 | `gain` 0.8 |
|---|---|---|
| 5 N | 10.0–11.1 N / 443–478 ms / 4.92–4.98 N | 9.4–11.1 N / 231–353 ms (one trial 3.0 s) / 5.03–5.17 N |
| 10 N | 10.6–12.3 N / 330–364 ms / 9.97–10.09 N | 10.3–10.8 N / 218–235 ms / 9.95–10.12 N |
| 20 N | 20.1–20.5 N / 368–408 ms / 19.67–20.13 N | 20.3–20.6 N / 338–339 ms / 19.89–20.10 N |

- **The spread is 0.10–0.11 N in every case**, which is the sensor's noise.
- **A target below the impact overshoots.** A 5 N target peaks at 10–11 N, the 5 mm/s impact. For light targets,
  approach more slowly.
- **Defaults:** `gain` 0.5, about 6× below where it starts to oscillate, and `approach_mm_s` 5. Gain 0.8 settles
  faster, with a 4× margin.

## Block sanding

**Cell:** `dev/sanding-mujoco.json` (`localhost:8087`), the same cell plus a workpiece. The workpiece is a fixed
240×160×60 mm block on the table, centered at (550, 0, 330) mm and tilted 2° about y, so its top falls toward +x. Its
friction (`static_friction`) is 0.6, in the range of sandpaper on wood. The ePick's 45 mm cup stands in for the sanding
pad.

**Script:** `clients/python/sand.py`.
1. Press at 10 N (a 40 N guard is also on).
2. Settle for 1 s.
3. Sweep five passes along x from 470 to 630 mm, at y = −50 to 50 mm in 25 mm steps, alternating direction.
   - A P controller on the arm's x/y streams the twist, capped at the sweep speed.
   - Even passes run +x, away from the base and downhill. Odd passes run −x.
   - Each pass covers 160 mm, and the surface drops 5.6 mm across it.
4. Log the force per pass. "In band" is the share of samples within ±20% of the target.

**Following the slope needs the integral term.** With only the proportional term, the steady offset is
v·tan 2° / `gain`. That's the force error needed to command the surface's vertical speed:

| `gain` | Sweep | Predicted offset | Measured (downhill / uphill passes) |
|---|---|---|---|
| 0.5 | 10 mm/s | ±0.7 N | −0.7 / +0.7 N |
| 0.5 | 20 mm/s | ±1.4 N | −1.4 / +1.3 N |
| 0.5 | 40 mm/s | ±2.8 N | −3.2 / +2.5 N |
| 1.0 | 40 mm/s | ±1.4 N | −1.5 / +1.4 N |

With `ki` 4 (and `gain` 0.5), every pass averages 9.9–10.3 N at every speed:

| Sweep | Inward passes (−x): in band, spread | Outward passes (+x): in band, spread |
|---|---|---|
| 10 mm/s | 98–100%, ±0.6–0.7 N | 91–94%, ±1.1–1.2 N |
| 20 mm/s | 95%, ±0.9 N | 81%, ±1.5–1.7 N |
| 40 mm/s | 92–95%, ±1.0–1.1 N | 51–56%, ±2.2–2.7 N |

- **The outward passes chatter, and friction causes it.** Binned by x, a 40 mm/s outward pass swings between about 8
  and 11 N every 30 mm (about 1.3 Hz), hardest just after the turn. Without `ki` the outward passes were just as noisy.
- **Lowering the friction to 0.2 removes it.** At 40 mm/s, every pass then reaches 93–99% within ±20% (±0.9–1.2 N).
  The drag from 6 N of friction acts on the pad 132 mm below the flange, and it stick-slips against a stiff arm in a
  way that depends on the sweep direction.
- **Fixes on a real cell:** a compliant pad (foam backing, as real sanders use), a slower sweep, or sweeping toward
  the base. The force law can't remove it: the error isn't along the axis it controls.
- **Turns:** min and max forces of 2–5 N and 15–20 N happen at the turns between passes, where the sweep reverses.

## Limits

- The sim's contact softness can't be set, and the arm is very stiff, about 45 N/mm. The stability limit (gain about
  3) and the friction chatter both depend on that, so gains need retuning on a real arm or a compliant tool.
- The force axis is the tool's z axis, read in the sensor frame, which mujoco-sim places at the flange. A sensor
  mounted rotated about z is fine, but one that isn't coaxial with the tool isn't supported.
- The tool stays vertical. On the 2° block, the round pad rides on its uphill edge. Larger tilts need orientation
  compliance, which isn't in this.
- The tare is at `servo_start`. A tool that picks something up mid-session shifts it.

## Reproduce

    go build -o bin/diff-ik ./cmd/module
    viam-server -config dev/ft-mujoco.json        # press and hold, localhost:8086
    uv run clients/python/press_hold.py --insecure --gains 0.5,0.8 --targets 5,10,20 --trials 3 --guard-n 60
    viam-server -config dev/sanding-mujoco.json   # sanding, localhost:8087
    uv run clients/python/sand.py --insecure --speeds 10,20,40 --csv .runs/sand
