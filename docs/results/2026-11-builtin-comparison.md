# Contact tasks: diff-ik vs the builtin motion service

Can the builtin motion service do diff-ik's guarded touch-off, press-and-hold and sanding? Builtin has no force input,
so the client reads the wrist F/T sensor and closes the loop itself. Same cells, sensor, tool and force law as the
diff-ik runs (`2026-11-contact-guard.md`, `2026-11-force-control.md`).

**Short answer.** Yes for the guarded touch-off, as long as the client stops the arm *and then* commands its measured
joints. Yes for a slow press-and-hold, where it holds as steadily as diff-ik but responds more slowly. Sanding works only
slowly: one `Move` per cycle matches diff-ik at 5 mm/s but can't sweep faster, and `teleop` sweeps faster but holds the
force poorly. The comparison also found a flaw in diff-ik's
guard, which is now fixed.

## Setup

- **Cells:** `dev/ft-mujoco.json` (table) and `dev/sanding-prop-mujoco.json`. The second is the sanding cell with the
  workpiece as a fixed *prop*, so MuJoCo simulates it but motion planning doesn't see it (see "Teleop can't press into
  an obstacle" for why). viam-server v1.17.0; RDK source read at v1.16.0.
- **Builtin client:** `clients/python/builtin_contact.py`, moving the `gripper` component.
  - `guard-move`: one `Move` to 15 mm below the surface while polling F/T at about 650 Hz. At 5 N it calls `arm.Stop()`,
    cancels the `Move`, then sends `MoveToJointPositions` with the measured joints (the "release"; `--no-release` skips
    it).
  - `guard-step`: `Move`s of 0.5 or 0.2 mm, reading the force after each.
  - `hold-teleop` and `hold-move`: diff-ik's force law, z += (`gain`·e + `ki`·∫e)·dt, sent as a pose goal by
    `teleop_move` at 10 Hz, or by one blocking `Move` per cycle.
  - `sand-teleop` and `sand-move`: the same, plus x advancing along two passes over the block.
- **Collision:** `Move` gets a `CollisionSpecification` allowing the gripper to touch the surface.
- **diff-ik:** `touch_sweep.py --guard`, `press_hold.py` and `sand.py --ys 0,0` (the same two passes).
- **Contact time:** for builtin it's when the flange reaches the contact height, because a fast `Move` reads over 1 N
  from acceleration alone. For diff-ik it's when the force reaches 1 N.

## Guarded touch-off (stop at 5 N)

At about 100 mm/s, the only speed builtin's `Move` runs at in this cell (3 trials unless noted):

| | Peak | Time to peak | Force after stop |
|---|---|---|---|
| builtin `Move`, no guard (1 trial) | 148 N | | 88 N for 30 s, then `TimeoutError` |
| builtin `Move` + client `arm.Stop()` (1 trial) | 212 N | 155 ms | 206 N |
| builtin `Move` + client stop + release | **83–92 N** | 18–20 ms | **2.5–3.1 N** |
| diff-ik guard, first version | 229–240 N | 27–30 ms | 6.7–8.2 N |
| diff-ik guard, abort trip (current) | **94–115 N** | 5–8 ms | **2.7–2.9 N** |

- **Builtin can do it, but only with a release.** `arm.Stop()` alone leaves the sim arm holding its *commanded* joints,
  which `Move` had already set at the goal below the surface. The client has to command the measured joints after
  stopping, the same release diff-ik's guard does.
- **The client won at first, and that exposed a diff-ik flaw.** The first guard appended the release point to the
  joint stream and let `armIO.Stop` drain it, so queued points played out first: about 10 ms more pushing. The guard
  now cancels the stream, calls `arm.Stop`, then moves to the measured joints, which is what the builtin client does.
  The two are now level at this speed.
- **The approach speed is where builtin falls short.** `Move` has no per-call speed:
  - The request has no `MoveOptions` field. Only the trajex streaming session (`TempStreamOptions`) takes one.
  - No planner option in `extra` sets a speed. A plan is only a path.
  - Builtin executes the path through the arm's `GoToInputs`, which calls `MoveThroughJointPositions` with no options
    and no `extra`. RDK v1.16 and v1.17 are the same.
  - Speed is therefore an arm-driver setting, which changes the arm for everything.
- **Slowing the arm makes peaks worse here.** The sim arm caps each joint step at `max_joint_deg_s`. A vertical descent
  at 100 mm/s needs only about 10–20°/s per joint, so 36 and 72°/s didn't bind (100–109 N at 100 mm/s, a little higher
  than at 180°/s because the release was slower). At 8°/s the approach is 58 mm/s and the peak 95–98 N; at 3°/s it's
  22 mm/s and 47 N, with 3.8–6.3 N left after the stop. diff-ik's peak falls about 1 N per mm/s: 13 N at 10 mm/s, 21
  at 20, 41 at 40, 81 at 80. So at matched speed builtin peaks about twice as high, because the cap also slows the
  client's release `Move`.
- **The sim arm ignores `MoveOptions`.** It reads only `waitAtEnd` from `extra`, so a client can't even plan a path
  and pace it itself by calling `MoveThroughJointPositions` with a speed. Real drivers vary.
- **The slow builtin option is stepping.** `Move`s of 0.5 mm cost about 145 ms each, so the approach runs at 3.4 mm/s
  with a 9–11 N peak. Steps of 0.2 mm run at 2.6 mm/s with 12–13 N, because each step still runs at full speed.
  diff-ik at 10 mm/s, three times faster, peaks at 11–14 N.

## Press and hold (10 N)

Gain 0.5 mm/s per N, 5 mm/s approach. Builtin is 2 trials, diff-ik 3:

| | Loop | Peak | Settle (±10%) | Steady state |
|---|---|---|---|---|
| builtin `teleop_move` | 10 Hz | 18.2–18.3 N | 1.0 s | 9.96 ± 0.11 N |
| builtin `Move` per cycle | 14.6 Hz | 14.9–15.1 N | 0.19–0.25 s | 9.99 ± 0.26–0.33 N |
| diff-ik `force` | 50 Hz loop, 500 Hz sensor | 10.6–12.3 N | 0.33–0.36 s | 10.0 ± 0.1 N |

At higher gains (one trial each):

| `gain` | builtin `teleop_move` | builtin `Move` per cycle | diff-ik |
|---|---|---|---|
| 1.6 | 0.8 s, ±0.17 N | 0.43 s, ±0.40 N | **0.28 s, ±0.11 N** |
| 3.2 | 0.9 s, ±0.22 N | oscillates, ±1.4 N | oscillates, ±1.2 N |

- **The hold itself can be just as accurate.** Builtin `teleop` holds within 0.1 N, the sensor's noise.
- **Teleop is slow at any gain.** It smooths its goals (an exponential moving average) and runs a planning pipeline,
  which lowers the effective gain. That keeps it stable at gain 3.2, but it never settles faster than about 0.8 s.
- **About these `Move` numbers:** they integrated over the measured time per cycle, which stayed a steady 68 ms with
  no sideways motion. The client now uses a fixed `dt` of 1/`--rate-hz` (see Sanding).
- **`Move` per cycle is fast but rough.** Each cycle is a full-speed burst, so the hold ripples at ±0.3–0.4 N. It
  becomes unstable at the same gain diff-ik does.
- **The approach overshoots in both builtin variants** (15–18 N against diff-ik's 11–12 N). The client's z goal is
  ahead of the arm at contact, and builtin has no equivalent of diff-ik's snap back to the measured joints.

## Sanding (10 N while sweeping)

Two passes at y = 0 over the 2° block: pass 0 runs +x (downhill, away from the base), pass 1 runs −x. `gain` 0.5,
`ki` 4. Spread and share within ±20%; builtin is 2 trials, diff-ik 2.

| Sweep | builtin `teleop_move`, pass 0 / pass 1 | diff-ik, pass 0 / pass 1 |
|---|---|---|
| 20 mm/s | ±3.8–4.4 N, 25–42% / ±1.9–2.2 N, 68–72% | **±1.5 N, 81–82% / ±1.4 N, 86–88%** |
| 40 mm/s | ±4.2–5.2 N, 26–38% / ±2.6–3.1 N, 30–62% | **±2.3–2.4 N, 54–55% / ±1.8–2.1 N, 66–77%** |

- **Teleop nearly loses contact on outward passes.** Its minimum force falls to 0.1–0.8 N, and its maximum reaches
  19–21 N. diff-ik's spread is roughly half, and it never drops below 3 N.
- **The outward chatter is in both.** It's friction against a stiff arm (`2026-11-force-control.md`). Teleop's
  10 Hz, smoothed loop makes it worse.
- **`Move` per cycle sands well, but only slowly.** Each cycle steps x by speed × 50 ms and z by the force law over
  the same fixed 50 ms. Sideways `Move`s take about 180 ms each, against 48 ms when only z changes, so the loop runs at
  5.5 Hz and the sweep reaches only **5 mm/s** whatever speed is asked for (2 trials):

  | 5 mm/s, two passes | Pass 0: spread, within ±20% | Pass 1: spread, within ±20% |
  |---|---|---|
  | builtin `Move` per cycle | ±0.65 N, 96% | ±1.27 N, 89% |
  | diff-ik | ±0.67–0.68 N, 99.9% | ±0.68–0.69 N, 97% |

  An earlier version of this client integrated over the *measured* time per cycle instead. One slow `Move` then made
  the next step huge: x jumped to the end of the pass, z rose by `gain`·(force error)·`dt` (290 mm after a 12 s `Move`),
  the force spiked to 48 N, and contact was lost. That was the test client's fault, not builtin's. `teleop` always
  used a fixed `dt`, so its numbers stand.
- **Means are close for both** (9.5–10.5 N), because both have the integral term.

### Teleop can't press into an obstacle

With the workpiece as an obstacle, which is visible to planning, builtin `teleop` froze mid-pass. Every goal inside
the block was refused (`violation between gripper:body and workpiece_origin`), each refusal cost about 1 s of
planning, and `teleop_move` still returned success. The arm sat still at 4 N while the client kept lowering the goal.

In RDK's `teleop.go`, the planner gets only the goal poses and `extra`. `armplanning.PlanRequest` has `Constraints`
and `ObstaclesInWorldFrame` fields, but `planTeleopMulti` never fills them. From the `MoveRequest`, only
`world_state`'s *transforms* are used (to build the frame system), so its obstacles are ignored, and `WorldState` has
no way to allow a contact anyway. A `CollisionSpecification` in `teleop_start`'s `constraints` is dropped: I tried it
with both frame and geometry names. The one collision option that does reach the planner, `extra`'s
`collision_buffer_mm`, adds clearance (geometries collide when closer than the buffer) and must be positive, so it
can't permit overlap. On the flat table it worked only because a 10 N
press is about 0.2 mm of overlap. The workaround above (the workpiece as a prop) removes the surface from planning
altogether. `Move` does honour the collision specification, but it rejects names the planner doesn't know, so the same
request can't serve both cells.

## Where each falls short

**Builtin:**
- **No force input.** The client has to close every loop over gRPC: sense, decide, then send a goal, a stop or a
  release, each a separate RPC.
- **No per-call speed for `Move`.** The approach speed, which sets the impact, belongs to the arm driver. The safe
  option, stepping, is slow: about 3 mm/s here.
- **Planning sits in the loop.** `Move` per cycle runs at about 15–20 Hz when only z changes and 5.5 Hz with sideways
  motion, which caps a force-held sweep at about 5 mm/s. `teleop` runs at 10 Hz but smooths and lags, so it settles
  slowly and holds the force poorly while sweeping.
- **Contact fights the planner.** Pressing means overlapping a surface the planner considers an obstacle. `Move` can be
  told it's allowed; `teleop` can't, and it fails silently.
- **Builtin's strength:** collision-checked, global paths. diff-ik is local and can't route around anything.

**diff-ik:**
- **The first guard had a release flaw**, now fixed.
- **It still can't avoid the impact.** About 1 N per mm/s remains from the arm's momentum and the few milliseconds of
  command in flight.
- **Gain limits match builtin's.** Force gains go unstable at about 3 on this stiff arm, the same as a `Move` loop.
  50 Hz buys faster settling, not a higher stability limit.
- **The friction chatter remains.** The force law acts only along the tool axis.
- **Force control and collision avoidance don't combine.** In force mode the tool must touch the surface, and diff-ik's
  collision avoidance (QP only, opt-in) would keep it 10 mm away. Use one or the other.
- **No global planning.** Nothing gets the tool around obstacles to the work.

**A practical split:** builtin `Move` to a standoff pose above the work, collision-checked; then a diff-ik session
with `guard` or `force` for the contact phase.

## Limits

- **The sim arm's `Stop` holds the commanded joints when no streamed trajectory is running.** That's why builtin needed
  a client release. Real drivers differ: some decelerate and hold where they stop. The release costs little either
  way.
- **Builtin's approach speed could only be varied arm-wide** (`max_joint_deg_s`), and that also slows the release.
  The guarded comparison covers 22, 58 and 100 mm/s, plus the stepped variant.
- **One stiff, rigid contact.** It has no compliance and contact softness can't be set, so absolute forces won't carry
  over to hardware. The comparisons between the two are what matter.
- **Few trials:** builtin runs are 1–3 trials each.

## Reproduce

    viam-server -config dev/ft-mujoco.json
    uv run clients/python/builtin_contact.py --insecure guard-move --trials 3
    uv run clients/python/builtin_contact.py --insecure guard-step --step-mm 0.5 --trials 2
    uv run clients/python/builtin_contact.py --insecure hold-teleop --trials 2      # and hold-move; --gain 1.6, 3.2
    uv run clients/python/touch_sweep.py --insecure --speeds 10,20,40,80,100 --trials 3 --guard
    viam-server -config dev/sanding-prop-mujoco.json
    uv run clients/python/builtin_contact.py --address localhost:8087 --insecure --surface table --x 470 --x1 630 \
        --start-z 515 --contact-z 364.8 --ki 4 sand-teleop --sweep-mm-s 20 --trials 2   # and 40 mm/s
    uv run clients/python/builtin_contact.py ... sand-move --sweep-mm-s 20 --rate-hz 20 # reaches 5 mm/s
    uv run clients/python/sand.py --insecure --speeds 5,20,40 --ys 0,0
