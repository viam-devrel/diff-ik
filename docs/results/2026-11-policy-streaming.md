# Learned-policy streaming (emulated): results

Can a learned policy's output drive a servo session through `servo_target`? Policies such as ACT, diffusion policies
and VLAs emit **action chunks**: H future actions at a fixed rate, computed from an observation that's already stale
when the chunk arrives. Consecutive chunks disagree, and inference sometimes stalls.

A real model in sim would mostly fail its task off-domain, so these runs emulate the output instead. The policy
follows a known reference path, which gives ground truth to measure against.

## Setup

- **Cell:** `dev/ft-mujoco.json` (UR5e, free space above the table), `servo` with `gain` 8, `linear_mm_s` 400,
  `joint_deg_s` 120 and a `timeout_ms` (dead-man) of 500.
- **Script:** `clients/python/policy_stream.py`; helpers are checked by `test_policy_stream.py`.
  - **Reference:** min-jerk moves through six waypoints, 2.5 s each, 153–320 mm apart. Peak speed is 114–240 mm/s,
    with the tool pointing down.
  - **Policy emulator:** it observes at t_obs, then sleeps a latency of 150 ms median (lognormal, σ 0.4). A 5% chance
    of a stall adds 1.2 s. It then delivers a chunk of 50 actions at 30 Hz (1.67 s): the reference from t_obs, plus a
    per-chunk offset (σ 3 mm, so chunks disagree) and per-action jitter (σ 0.5 mm). Then it repeats.
  - **Executor:** sends one `servo_target` per action at 30 Hz. When the chunk is used up it keeps sending the last
    action.
- **Strategies:**
  - `naive` plays each new chunk from its first action.
  - `aligned` indexes the chunk by `(now − t_obs)·30`, skipping stale actions.
  - `blend` is aligned plus a 200 ms linear crossfade from the previous chunk.
  - `--lead-ms` plays aligned/blend chunks that far ahead of now.
- **Measured:**
  - tracking error: the flange's distance from the reference at the same time;
  - command step: the jump between consecutive targets, about 4–8 mm per tick at these speeds;
  - the session's `peak_accel_ratio`, `resyncs` and `lag_ms`.

## The servo's own lag

With a perfect policy (1 ms latency, no noise), `aligned` tracks with **22 mm RMS** error. The arm's path has the
right shape: shifted 200 ms back, it matches the reference to 1.4 mm RMS. It's a fixed delay through the servo and the
arm, and the session's status reports it (`lag_ms` 240). Raising `gain` from 2 to 8 barely changes it (25 → 22 mm).

Chunks offer a remedy that single targets don't: the client already holds future actions. Playing the chunk 200 ms
ahead (`--lead-ms 200`) brings the perfect-policy error to **1.3 mm RMS, 2.7 mm max**.

## Realistic policy

Three seeds per row; ranges across seeds:

| Strategy | Lead | Tracking RMS | Tracking max | Step p99 | Step max | Held ticks |
|---|---|---|---|---|---|---|
| `naive` | 0 | 43–56 mm | 89–140 mm | 49–146 mm | **175–256 mm** | 0 |
| `aligned` | 0 | 23–27 mm | 52–56 mm | 12–14 mm | 15–18 mm | 0–31 |
| `aligned` | 200 ms | **4.1–10.8 mm** | 7–48 mm | 12–16 mm | 15–48 mm | 7–48 |
| `blend` | 0 | 24–27 mm | 47–57 mm | **9 mm** | 11–18 mm | 0–32 |
| `blend` | 200 ms | **4.3–11.0 mm** | 8–48 mm | 9–14 mm | 12–47 mm | 5–49 |

Each run had 3–5 inferences slower than 3× the median. The acceleration limit was reached in every run
(`peak_accel_ratio` 1.0), with no resyncs.

- **Playing chunks naively is dangerous.** Restarting each chunk from its first action replays stale motion: the
  commanded pose jumps by up to a quarter of a metre between ticks. Only the session's speed and acceleration limits
  keep that from reaching the arm.
- **Aligning by observation time removes the jumps.** What's left is the servo's ~200 ms lag, which shows as a 23–27 mm
  tracking error at these speeds.
- **The lead removes most of the lag.** The remaining 4–11 mm RMS is the per-chunk disagreement (σ 3 mm), plus the
  stalls.
- **Stalls are the worst case:**
  - a 1.2 s stall uses up the 1.67 s chunk, and the lead uses it up 200 ms sooner;
  - the client then holds the last action (5–49 held ticks);
  - when the next chunk lands, the target jumps up to 47 mm and the servo catches up at its limits.

  During the hold the client keeps sending targets, so the dead-man never fires even though the policy has gone quiet.
- **Blending smooths the joins between chunks** (99th-percentile step 9 mm, against 12–15 mm) without changing the
  tracking error.

## What this suggests for the API

All of the above is done by the client with today's `servo_target`. It works, but it puts timing-critical logic in
the client and leaves some gaps:

1. **Lag compensation needs the module's lag.** The client guessed 200 ms; the module measures it every tick
   (`lag_ms`).
2. **A chunk has a horizon, but a target doesn't.** When a chunk runs out, the client has to choose between holding
   (which refreshes the dead-man, so a dead policy goes unnoticed) and stopping (which lets the dead-man fire). A target
   with an expiry time would let the module ramp to a stop at the chunk's end on its own.
3. **The client sends 30 messages a second.** Every one of them crosses the network, so jitter shows up as jitter in
   the arm's motion. One message per chunk (about 5 Hz here) would be much less sensitive. That's the next scenario
   worth testing (network latency).
4. **Alignment needs a clock both sides agree on.** Here the client and module share a laptop. Across machines, a chunk
   would need its age (or the observation's age) rather than an absolute timestamp.

**A candidate command:** `servo_trajectory` (prototyped below). It takes `{"poses": [...], "dt_ms", "age_ms"}`. The module would:
- index the chunk by its own clock;
- play it ahead by its measured lag;
- crossfade from the previous chunk;
- ramp to a stop when the chunk runs out.

That's prototyped below, and compared against this client-side baseline locally and over an emulated network.

## `servo_trajectory` prototype

The module now takes a whole chunk:

```json
{"command": "servo_trajectory", "session_id": "...", "poses": [pose, ...], "dt_ms": 33.3, "age_ms": 152,
 "lead_ms": 200, "blend_ms": 200}
```

- **Timing:** the first pose is for the moment `age_ms` before the command arrives. Each tick samples the chunk at
  now + `lead_ms`, interpolating between poses, and feeds forward the segment's velocity. Without `lead_ms`, the lead
  is the session's measured lag.
- **Blending:** `blend_ms` crossfades from the previous chunk.
- **Expiry:** past the last pose the session holds there. The dead-man counts from the chunk's end, so a policy that
  stops sending gets `timeout_ms` of hold, then the session ends as `stopped_timeout`.
- **Cancelling:** a `servo_target` cancels the trajectory.
- **Status:** `trajectory_lead_ms`, `trajectory_remaining_ms` and `trajectory_ended`.

**Results.** Same emulated policy, three seeds per row. `client` is the best client-side strategy above: blend plus a
200 ms lead, one `servo_target` per action at 30 Hz. `module` is one `servo_trajectory` per chunk, about 6 a second.

| | Tracking RMS | p95 | Worst |
|---|---|---|---|
| Perfect policy: client, 200 ms lead | 1.3 mm | 2.4 mm | 2.7 mm |
| Perfect policy: module, `lead_ms` 200 | 2.8 mm | | 6.4 mm |
| client, blend + 200 ms lead | 4.2–11.3 mm | 7–26 mm | 9–49 mm |
| module, `lead_ms` 200, `blend_ms` 200 | 3.9–12.2 mm | 7–28 mm | 7–54 mm |
| module, `lead_ms` 200, no blend | 4.3–11.3 mm | 8–26 mm | 9–48 mm |
| module, automatic lead (240–260 ms) | 6.5–13.2 mm | 14–30 mm | 23–53 mm |
| client, over a 40 ± 25 ms one-way link | 8.2–13.1 mm | 16–30 mm | 22–52 mm |
| module, over the same link | 7.3–14.9 mm | 12–38 mm | 17–58 mm |

- **The module matches the best client strategy** on a realistic policy, while sending one message per chunk instead
  of 30 a second. The client needs none of the timing logic (aligning, leading, blending, holding).
- **The automatic lead is biased high.** The session's `lag_ms` reads 240–260 ms, but the arm's path is 200 ms behind
  (by cross-correlation), and the module's best lead was 200–220 ms. The lag estimate falls more slowly than it rises,
  so it settles high. Until that's fixed, send `lead_ms`.
- **The perfect-policy floor is higher through the module** (2.8 mm against 1.3 mm). In that run the client sent a
  chunk every few milliseconds (about 400 a second), which a real policy wouldn't. The other difference is
  feedforward: `servo_target` estimates velocity from successive targets, while the trajectory feeds forward each
  segment's exact, piecewise-constant velocity. Not investigated further.
- **Latency costs both strategies the same.** With 40 ± 25 ms of one-way delay, both run about 40 ms later. The module
  can't see transit time, so `age_ms` is short by the one-way latency, and a client's targets arrive that much late
  too. Adding the expected network delay to `lead_ms` would cover it for the module.
- **Jitter costs only the client's smoothness.** On one seed, the link raised the client's arm acceleration from 0.51
  to 0.62 m/s² RMS (99th percentile 1.38 → 1.96 m/s²). The module's stayed at 0.47 → 0.45 RMS (1.60 → 1.47). Its
  targets come from the chunk at the module's own tick rate, not from when messages land.
- **Expiry works, and ends the session.** In some module runs a 1.2 s stall near the episode's end outlasted the
  chunk plus the 500 ms dead-man, and the session ended `stopped_timeout`. That's the intended behaviour (a quiet
  policy stops the arm), but a client that wants to keep going has to start a new session. A softer option, holding
  without ending, could be a flag.

**Still open:**
- **Clocks:** across machines, `age_ms` doesn't include transit time. An RTT-based estimate, or a lead that includes
  the expected network delay, would be needed.
- **No target filter:** chunks bypass the target filter (EMA, deadband and floor clamp). The floor clamp in particular
  should probably apply to chunks too.
- **The lag estimator** needs fixing so "auto" lead is usable.

## Limits

- **The policy is open loop.** Its actions follow the reference regardless of where the arm is. A real policy closes
  the loop through its observations, which can pull its actions toward wherever the arm lags.
- **Positions only.** The tool orientation is fixed, and the noise model (a Gaussian offset per chunk, plus jitter) is
  simple. Real chunks can switch between quite different motions, which would make chunk disagreements much larger.
- **One arm, one speed profile, one latency model.** The network is emulated in the client (per-message delay, in
  order), not a real WebRTC link.

## Reproduce

    go build -o bin/diff-ik ./cmd/module

    viam-server -config dev/ft-mujoco.json
    uv run clients/python/policy_stream.py --insecure --gain 8 --strategy aligned --lead-ms 200 --csv .runs/policy
    # perfect policy (servo floor):
    uv run clients/python/policy_stream.py --insecure --gain 8 --stall-p 0 --latency-ms 1 --latency-sigma 0 \
        --bias-mm 0 --jitter-mm 0 --lead-ms 0
    # servo_trajectory, and over an emulated link:
    uv run clients/python/policy_stream.py --insecure --gain 8 --strategy module --module-lead-ms 200 --blend-ms 200
    uv run clients/python/policy_stream.py --insecure --gain 8 --strategy module --module-lead-ms 200 --blend-ms 200 \
        --net-ms 40 --net-jitter-ms 25
