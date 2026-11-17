# Diff-IK xArm6 teleop A/B results: viam:teleop on builtin motion vs diff-ik, 2026-11-16

- **Question:** can diff-ik's `teleop_*` shim stand in for the builtin motion service behind an unmodified `viam:teleop` service on a production arm, and how does it track?
- **Station:** armfarm15 (hackathons org). An SO-101 leader with torque released (manual mode) drives an xArm6 follower (`viam:ufactory` 1.1.38) at 2.5× scale.
- **Services:** two copies of the station's own `teleop` service without data capture, differing only in `motion_service`:
  - `teleop-builtin` uses the builtin motion service;
  - `teleop-diffik` uses diff-ik's `servo` service, loaded as a local module.

  Both use 60 Hz, a 250 mm/s / 180°/s speed cap, and a 100 mm leader tool offset.
- **Recording:** `clients/python/teleop_ab.py` polls both arms' end positions through the cloud (13.6–35.4 Hz), starts the service, records for 45 s, then stops it. One operator drove each run by hand at the station: slow sweeps, some faster moves, and several 3 s holds.
- **Analysis:**
  - **Lag and RMS error:** `teleop_ab.py --analyze`. Lag is the delay (0–800 ms) that minimises the RMS residual between the follower and the scaled leader, after removing a constant offset, over moving samples only. RMS is that residual.
  - **Jitter:** the follower minus its own 150 ms moving average.
  - **Per-speed bins and holds** were computed from the same CSVs.
- **Raw data:** `.runs/ab*-poses.csv` and `.runs/ab*-status.json`, untracked. Arm-limit backup: `.runs/armfarm15-arm-limits-before.json`.

**Verdict:**
- **The shim works.** Unmodified `viam:teleop` drove the xArm6 through diff-ik for every run with 0 errors once diff-ik's joint cap sat under the driver's limit.
- **At matched arm limits, diff-ik tracks better than builtin.** With the arm at 120°/s, 300°/s² and diff-ik's cap at 100°/s:
  - lag 130–150 ms against 320 ms;
  - RMS error in motion 12.7–14.8 mm against 33.1 mm;
  - roughly half the jitter.
- **Against builtin's best run** (240 ms, 13.2 mm, at the arm's original 60°/s limits), RMS error is level and lag is about 40 % lower.
- **Every condition is a single hand-driven 45 s run.** Leader speed differed between runs (see the speed row below), so the differences between nearby numbers are within run-to-run noise.

## Runs

| Run | Backend | xArm speed / accel | diff-ik joint cap | diff-ik resync |
|---|---|---|---|---|
| ab2-builtin | builtin | 60°/s / 60°/s² | — | — |
| ab2-diffik | diff-ik | 60°/s / 60°/s² | 50°/s | default (3°) |
| ab3-builtin | builtin | 120°/s / 300°/s² | — | — |
| ab3-diffik | diff-ik | 120°/s / 300°/s² | 100°/s | default (6°) |
| ab4-diffik | diff-ik | 120°/s / 300°/s² | 100°/s | `resync_deg` 20 |
| ab5-diffik | diff-ik, build `0783ab9` (lag-compensated resync) | 120°/s / 300°/s² | 100°/s | default |

| | ab2-builtin | ab2-diffik | ab3-builtin | ab3-diffik | ab4-diffik | ab5-diffik |
|---|---|---|---|---|---|---|
| Best-fit lag | 240 ms | 250 ms | 320 ms | 150 ms | **130 ms** | 140 ms |
| RMS error in motion, at best lag | 13.2 mm | 42.8 mm | 33.1 mm | 14.8 mm | **12.7 mm** | 14.3 mm |
| RMS at scaled leader speed <20 / 20–100 / 100–200 / >200 mm/s | 3 / 9 / 13 / 16 | 6 / 25 / 31 / 66 | 6 / 22 / 27 / 44 | 5 / 7 / 13 / 23 | 8 / 8 / 9 / 16 | 4 / 6 / 13 / 16 |
| Jitter RMS | 1.72 mm | 0.54 mm | 1.43 mm | 0.72 mm | 0.73 mm | 0.72 mm |
| Scaled leader speed p50 / p95 | 109 / 259 mm/s | 12 / 317 mm/s | 43 / 334 mm/s | 86 / 283 mm/s | 53 / 208 mm/s | 35 / 312 mm/s |
| Holds: count; offset median / max | 5; 1 / 3 mm | 8; 3 / 5 mm | 5; 1 / 2 mm | 2; 1 / 1 mm | 2; 5 / 5 mm | 6; 2 / 4 mm |
| Last planning step (`pipeline_plan_ms`) | 13.6 ms | 2.2 ms | 100.7 ms | 2.2 ms | 2.1 ms | 2.7 ms |
| Ticks held by teleop's speed cap | 1037 | 2026 | 574 | 306 | 77 | 539 |
| Errors | 0 | 0 | 0 | 0 | 0 | 0 |

- **Last planning step** is the last sample in the final status, not a mean.
- **Earlier pair not included:** ab-builtin and ab-diffik are excluded. The operator moved about three times faster in the diff-ik run than in the builtin one, and both runs sat at teleop's 250 mm/s cap for most of their length (720 / 730 ms lag, 90 / 235 mm RMS), so they don't compare the backends.

## Findings

1. **The xArm driver rejects streamed points above its joint speed limit.** In the first diff-ik attempt, diff-ik's 120°/s cap was above the driver's 60°/s `speed_degs_per_sec`.
   - The driver refused a point (`joint 2 velocity -1.08 rad/s exceeds limit 1.06 rad/s`), the session faulted, and teleop reported `session_lost` after 41 moves. The follower stopped where it was.
   - Capping diff-ik at 50°/s fixed the fault but made tracking about 3× worse than builtin (ab2: 42.8 against 13.2 mm). The gap grew with speed: 66 against 16 mm above 200 mm/s.
   - ab2 is confounded: the diff-ik run's leader moved at a p50 of 12 against 109 mm/s and was sampled at 13.6 against 32.5 Hz, so "3×" is not a clean ratio.
   - Raising the arm to 120°/s and 300°/s² and diff-ik's cap to 100°/s removed the gap (ab3–ab5).
   - **Lesson:** a caller-set joint cap is wrong in both directions on this arm. The servo session should read the arm's own limits.
2. **On the xArm, resync is not the bottleneck.** A MuJoCo replay of the recorded leader paths suggested resync was. The simulated arm trails its commands by about 250 ms, so the default threshold resynced constantly, and `resync_deg` 16 brought its RMS from 115 mm down to 35 mm.
   - On the real xArm:
     - `resync_deg` 20 (ab4) and the lag-compensated default (ab5) were within noise of the old default (ab3);
     - ab5's status reported `lag_ms` 0 and 0 resyncs throughout.
   - `lag_ms` read 0 in every status poll, so the arm adds little lag behind its streamed commands. The 130–150 ms end-to-end lag comes from teleop's sampling and speed limiting (539 held ticks in ab5) and from the loop's own gains and rate limits.
   - `lag_ms` only updates when the commanded joints move more than half the resync threshold within the lag window (`loop.go` `updateLag`), so 0 means "no measurable lag", not an exact zero.
3. **Builtin got worse when the arm got faster.** ab3-builtin (320 ms, 33.1 mm) is worse than ab2-builtin (240 ms, 13.2 mm), even though the arm's limits went up. Its last planning step also rose from 13.6 to 100.7 ms.
   - With one run each, this could be operator speed (ab3's p95 was higher) or the planner's time-parameterisation under the new limits. It wasn't investigated.
4. **Diff-ik's follower is smoother.** Jitter was 0.5–0.7 mm against 1.4–1.7 mm for builtin in every run.
   - Holds landed within 5 mm on both backends.
   - Diff-ik's holds were 1–3 mm further off at the median in some runs. Too few holds per run (2–8) to say more.
5. **The `teleop_*` shim keeps a session alive through status polls.** `viam:teleop` sends no moves while the leader is still and polls status every 200 ms. The shim treats a status poll as a dead-man refresh for up to `teleop_idle_ms` (60 s) after the last move.
   - This is what made the A/B possible without changing `viam:teleop`.
   - It is weaker than the plain servo dead-man.

## Station state

armfarm15 still carries the test setup, kept at the user's request:
- the diff-ik module (`0783ab9`), and the `servo`, `teleop-builtin` and `teleop-diffik` services;
- API key `diff-ik-teleop-ab-15`;
- the xArm's raised limits: 120°/s and 300°/s², up from 60 and 60.

The station's own `teleop` service is untouched. Restore values are in `.runs/armfarm15-arm-limits-before.json` and `.runs/armfarm15-before-diffik.json`.
