# Diff-IK live obstacles: results

Spec: `docs/superpowers/specs/2026-11-18-diff-ik-live-obstacles-design.md`. Plan: `docs/superpowers/plans/2026-11-18-diff-ik-live-obstacles.md`.

## Task 0

### The cell: `dev/panda-live.json` (port 8084)

The cell is the Panda MuJoCo cell plus the following:
- **Cameras:**
  - `top-depth`: fixed overhead at (550, 0, 1300) mm, looking straight down, 70° field of view, 640×480 colour and 320×240 depth;
  - `wrist-depth`: on the arm's flange.
- **Props:** two red boxes, `intruder` and `parked`, each 80 × 80 × 120 mm.
- **Modules:** `viam:obstacles-pointcloud` and `viam:detections-to-segments`, both built from source, because registry modules don't load from a local config. The sources are `~/src/obstacles-pointcloud` (0.2.1) and `~/src/detections-to-segments` (tag 1.4.0).
- **Services:**
  - `obstacles-top` and `obstacles-wrist`;
  - `red` (the RDK's `color_detector`);
  - `segments-top` (detections-to-segments over `top-depth` and `red`, with `infer_minimum_depth`).

The sim camera reports `supports_pcd` with intrinsics. Colour and depth come from the same simulated camera, so detections-to-segments' "already registered" assumption holds.

### Methods

- **Both services** report `Detections3DSupported` and `ObjectPCDsSupported` in `GetProperties`.
- **obstacles-pointcloud** builds its 3D detections with the RDK's `detection3d.FromSegmenter`, which attaches a point-cloud child to each detection. The self test needs that child.

### Accuracy (`go run ./dev/liveprobe`)

The intruder was placed at three known poses on the table, centre z 81, top 140. Boxes are in world, measured against the truth.

**Raw output** (before the clean-up step was added to the spec):
- **obstacles-pointcloud, default plane settings:** it didn't remove the table, so it returned one 794 × 1188 × 766 mm box (the table plus everything touching it, including the arm) and about 15 fragments.
- **Tuned settings:** plane band 25 mm, `min_points_in_plane` 2000, ground tolerance 10°, `min_points_in_segment` 50, ground normal (0, 0, −1) in the camera frame. The table is removed, but:
  - a looking-down camera sees only top faces, so boxes are flat slabs;
  - edge pixels in the depth image sometimes join a prop into a 630 mm column with things above or below it.
- **detections-to-segments:** it found both red props, but took in every depth point inside the 2D box, so boxes reached 785 mm tall when the arm passed over a prop.

**After clip, refit and extrude** (workspace over the table, world z 20–600; extruded to z 20):

| Service | Centre error | Box (truth 80 × 80 × 120, z 20–140) | Detections returned |
|---|---|---|---|
| obstacles-top | 1.9–9.4 mm | 61–74 × 74 × 120 mm, z 20–140 | intruder, the bin's rim (149 × 149 × 61 mm, true top 81) and `parked`. No arm pieces with the arm above 600 mm. |
| segments-top | 5.1–13.5 mm | 54–84 × 68–75 × 120 mm, z 20–140 | the two red props |

- **Footprints run 3–13 mm narrow per side:** the top face seen from above is smaller than the full box at its edges. The 10 mm inflation covers most of that.
- **The bin is detected:** obstacles-pointcloud picks it up even though the static collision model skips it as a mesh.

### Self-detection

With the arm lowered (flange at z 220), obstacles-pointcloud returns several arm and gripper pieces inside the workspace, for example 165 × 164 × 433 mm and 73 × 108 × 520 mm boxes near the arm. These are the arm seeing itself. The point-based self test (spec step 5) has to drop them; Task 4's unit tests check the rule, and the sim evidence confirms it live. segments-top never returns the arm, because the colour detector only finds red.

### Latency (top-depth, 320×240 depth)

| Service | p50 | p95 |
|---|---|---|
| obstacles-top (tuned) | 89–92 ms | 94 ms |
| segments-top | 61 ms | 66 ms |

The evidence `rate_hz` is 8 for obstacles-top and 12 for segments-top, below 1/p95 for each.

## Evidence: sim, Panda cell

**Setup.**
- The cell is `dev/panda-live.json`. The arm is limited to 30°/s and 60°/s² (`--max-joint-deg-s 30 --max-joint-deg-s2 60`), and holds a low pose with the flange at z 220 and the gripper about 60 mm above the table.
- The static world state is `dev/panda-collision-ws.json`.
- Live settings:
  - camera `top-depth`;
  - workspace x 200–1000, y −600 to 600, z 20–600, extruded to z 20;
  - `stale_ms` 500 and inflation 10 mm (the default);
  - rates of 8 Hz for `obstacles-top` and 12 Hz for `segments-top`.
- Props are moved by `clients/python/prop_mover.py` (80 × 80 × 120 mm, `--size-mm 80,80,120`).
- **True clearance** comes from `cli clearance`: each run's `q_cmd` is replayed against the props' commanded, uninflated poses, on the same Unix clock.

The CLI's own `collision: min` line is measured against the *inflated, extruded* detected box, so it runs about 20–45 mm more negative than the true clearance. The bars use the true clearance.

### Intruder into a held pose

The intruder slides from y = 350 to y = 70 (its near face at y 30) toward the gripper at (555, 0), holds 3 s, then backs out.

| Pipeline | Intrusion | Tier 2 | True min clearance | Bar (≥ `d_min` − 5 = 5 mm) | Pinched ticks | Self-drops |
|---|---|---|---|---|---|---|
| obstacles-pointcloud | 50 mm/s | hold still (as first specified) | −0.67 mm | fail | 175 | 837 |
| detections-to-segments | 50 mm/s | hold still | −19.68 mm | fail | 72 | 0 |
| obstacles-pointcloud | 25 mm/s | hold still | −17.04 mm | fail | 238 | 1034 |
| obstacles-pointcloud, arm at 90°/s and 360°/s² | 50 mm/s | hold still | −14.01 mm | fail | 188 | 1046 |
| **obstacles-pointcloud** | **50 mm/s** | **soft retreat** | **+5.05 mm** | **pass** | 32 | 1219 |
| obstacles-pointcloud | 25 mm/s | soft retreat | +3.82 mm | fail (−1.2 mm short) | 31 | 1041 |
| detections-to-segments | 50 mm/s | soft retreat | −4.76 mm | fail | 2 | 0 |
| obstacles-pointcloud, retreat weight 1e-5 (experiment, not kept) | 50 mm/s | soft retreat | −12.13 mm | fail | 43 | 1118 |

Every run had a peak acceleration ratio of 1.000, with no stale stops. The intrusion runs had 1–8 brakes and some late warnings (M7), so they were not fault-free.

#### Repeatability (10 runs)

Ten back-to-back runs of the soft-retreat configuration (obstacles-pointcloud, 50 mm/s, `--solver qp`, 30°/s and 60°/s²) on the local sim cell (`dev/panda-live.json`, localhost), same flags and prop path each time, the prop parked away between runs.

| Run | True min clearance (mm) | Pinched ticks | Live updates |
|---|---|---|---|
| 1 | −19.65 | 39 | 199 |
| 2 | −15.27 | 19 | 199 |
| 3 | +2.67 | 37 | 199 |
| 4 | −2.91 | 20 | 199 |
| 5 | +2.76 | 30 | 199 |
| 6 | −2.73 | 19 | 199 |
| 7 | +0.78 | 30 | 199 |
| 8 | +2.06 | 20 | 199 |
| 9 | −12.11 | 32 | 199 |
| 10 | +2.62 | 19 | 199 |

- Min −19.65 mm, median −0.98 mm, max +2.76 mm, mean −4.18 mm, stdev 8.39 mm (sample).
- 5 of 10 runs ended at clearance ≥ 0. None reached the 5 mm bar or the 10 mm `d_min`.
- Every run completed with 0 live errors, no stale stops, and no late warnings. Brakes were 0–8 per run, and waiting-ticks were 10–12.
- **After the camera-origin fix**, the same procedure gave 9 good runs: +3.42, +3.91, +5.13, +3.16, −19.00, −7.09, −19.45, −7.78 and −2.03 mm (median −2.03, mean −4.41, stdev 9.63, 4 of 9 ≥ 0, pinched 24–53, live updates 199–200). That's the same spread as before, including the cluster around +3 to +5 mm and the one near −19 mm. Four further attempts failed with `context canceled` and `stopped_stale` after another viam-server started on the same machine. They're not counted.
- Two further runs (11, 12), taken only because the script loop ran 12 times, gave −0.50 mm and +0.47 mm. They are not in the statistics above. Including them: median −0.02 mm, mean −3.48 mm, stdev 7.76 mm, 6 of 12 at ≥ 0.
- For comparison, two earlier demo runs with the same flags on the cloud-connected mujoco-sim-mac gave −10.24 mm and +4.99 mm. The +5.05 mm pass in the table above is higher than all ten of these runs, so it was a favourable draw rather than typical behaviour.

### Parked prop on the arm's path

The prop sits on the orbit's arc at (480, −277). The arm orbits from the low pose at −5°/s for 60°.
- **True min clearance: 16.91 mm. Pass.** The arm stopped short of the prop.
- Mean tracking error was 167 mm, because the arm waits at the prop while the target keeps orbiting.

The first attempt used the CLI's `circle`, whose 50 mm radius is smaller than the gripper, so a prop on its path started inside the gripper and the self test dropped it. That run isn't counted.

### Stale stop

The obstacles-pointcloud module was frozen with SIGSTOP for 2 s during a hold.
- **Result:** the session ended `stopped_stale` 0.73 s after the freeze (`stale_ms` 500 plus the 300 ms ramp). **Pass.**
- **A crash doesn't go stale:** a first attempt killed the module process outright. viam-server restarted it within about 250 ms, inside `stale_ms`, so after 2 errors the feed recovered and the session correctly kept running.

### Intrusion speed sweep (reported only)

obstacles-pointcloud, with soft retreat:

| Speed | 50 mm/s | 100 mm/s | 200 mm/s | 400 mm/s |
|---|---|---|---|---|
| True min clearance | +5.05 mm | −19.48 mm | −22.04 mm | −18.09 mm |

### Findings

- **Tier 2 had to change.** As first specified, tier 2 held the arm still when back-away rows conflicted. Several arm parts end up inside one inflated, extruded box, with conflicting retreat directions. Against a box that keeps advancing, holding still meant a guaranteed intrusion (175–238 held ticks, −0.7 to −17 mm). Arm speed wasn't the limit: a slower box was *worse*, and a 3× faster, 6× harder-accelerating arm barely helped. Tier 2 now keeps "no further approach" hard and adds a soft retreat term. This took obstacles-pointcloud at 50 mm/s from −0.67 to +5.05 mm in one run, and the unit test from −9.95 to +7.30 mm. The spec is updated. The repeatability runs show that single run was a favourable draw (see the next bullet).
- **The retreat weight is a trade-off.** 1e-6 is about the same cost per mm as task error. 1e-5 overshot in the sim (−12 mm), and the larger 1e-4 broke the 0.1 mm static-pinch bar. Better weighting, such as scaling with the intrusion depth or a per-part priority, is follow-up work.
- **Reactive avoidance does not reliably keep up with 50 mm/s intrusions at these arm limits** (30°/s, 60°/s², roughly 150–250 ms from capture to an obstacle update). The single +5.05 mm pass didn't repeat: across ten runs the median was −0.98 mm, the worst −19.65 mm, and none reached the 5 mm bar. Faster intrusions end 18–22 mm inside the prop. The retreat is limited by perception latency and the arm's acceleration, and there's no prediction. Predicting obstacle velocity (out of scope), a stronger retreat, or a margin that grows with approach speed would be the next step.
- **detections-to-segments is about 10 mm worse than obstacles-pointcloud** at the same speed, even at 12 Hz. Its boxes come from the 2D detection's extent, which is narrower and noisier than the clustered points.
- **The self test works.** obstacles-pointcloud returned the arm as 721–1219 detections per run; all were dropped, and the arm never backed away from itself. detections-to-segments never sees the arm.
- **The parked-prop and stale-stop bars pass.**
- **Two bars miss:** the slow intrusion (+3.82 mm) and detections-to-segments at 50 mm/s (−4.76 mm), both against the 5 mm bar. They're recorded as failed, not relaxed.

### Not done and deviations

- **Parked prop:** the arm stopped short (16.9 mm), but completing the path after the prop was removed wasn't shown; the prop stayed for the whole run.
- **Pipelines:** the parked, stale and speed-sweep runs used obstacles-pointcloud only; the spec asked for both pipelines.
- **Tick time:** the unit test gives 1.1 ms p99 at 256 pairs; the whole-tick p99 in the sim runs was 8.7–15.0 ms, with 0–4 overruns.
- **Brakes and late warnings:** the intrusion runs had 1–8 brakes, and some late warnings (M7).
- **The `mujoco-sim-mac` demo** ran (two intruder runs: −10.24 and +4.99 mm). The real-robot runs are below.

The logs and CSVs are in `.runs/live-*`.

## Evidence: real arm, armfarm15

**Setup.**
- An xArm with a RealSense depth camera on the wrist (`cam`), looking down at the table.
- Vision: obstacles-pointcloud 0.2.1 as `obstacles-cam` on `cam`, ground normal (0, 0, −1).
- Sessions run in the module (`servo-live`, `servo_start` over DoCommand), because the CLI's local loop is too slow over the cloud (1.26 s for one probe call).
- Live settings: `rate_hz` 4, workspace x 100–800, y −500 to 500, z 15–600, extruded to z 15, inflation 10 mm, `d_min` 10 mm.
- The vision call takes about 600 ms on the machine, so live updates arrive at about 1.3–1.6 Hz.

### Static hold (20 s)

The arm held still and converged, with 33 updates, no errors and no self-detection drops.

### Box slid toward the gripper by hand (40 s, before the fix)

- One update put an obstacle 13.9 mm inside the wrist camera's geometry. The arm backed away about 24 mm in 0.65 s, pulling away before contact. It then held 16–34 mm from the box, outside `d_min`, as designed.
- The box seemed to drop in and out of the detections between updates during the approach. After the fix (below) it never did, so this was most likely the stretched obstacle appearing or not, depending on whether the camera-origin point joined the box's cluster on that update.

### Wrist camera descending onto the box

The arm centred the camera over a box (top at z 190) and stepped straight down 20 mm at a time, reading raw depth and the detections at each step.

- **Minimum range isn't the limit.** The box top read within ±1 mm down to 154 mm camera-to-surface, the closest the gripper allows.
- **The stretched obstacle (bug, fixed).** Pixels with no depth reading all map to one point at the camera's origin, and obstacles-pointcloud attached it to the box's cluster. So every box detection reached from the box top up to the camera. The refit box therefore swallowed the camera and gripper; this explains the 13.9 mm "inside the camera" in the slide test. The whole-detection self test didn't catch it: it was 1 point in about 89,000.
- **Fix.** Detection points within 1 mm of the camera's own origin are dropped when they are mapped to world, before the clip, refit and self test. A first version instead left every point within 20 mm of the arm out of the refit. It was replaced because it also removed a real obstacle's points in the band between `d_min` (10 mm) and 20 mm. The descent and sideways runs below used that first version; they never brought a real obstacle within 20 mm of the modelled arm, so the result would have been the same.
- **After the fix**, with live obstacles only (no static box), `stale_ms` 3000:

| Step | Camera to box top | Live clearance, gripper to box | Updates in 3 s |
|---|---|---|---|
| centred | 294 mm | 134 mm | |
| 1 | 274 mm | 115–117 mm | 4 |
| 2 | 254 mm | 95–96 mm | 4 |
| 3 | 234 mm | 74–76 mm | 4 |
| 4 | 214 mm | 54–56 mm | 4 |
| 5 | 194 mm | 34–36 mm | 4 |
| 6 | 174 mm | 15–16 mm (see the gripper-model note below) | 4 |

The nearest pair was the gripper and the box at every step, with one live obstacle, no back-away, no self drops and no errors. An earlier run with `stale_ms` 1500 ended `stopped_stale` at step 5, after one slow vision call; 3000 suits this camera.

### Sideways passes over the box (after the fix)

Because the box couldn't be moved by hand, the arm made the sideways motion instead, passing over the stationary box at 20 mm/s (flange from (515, −2) to (191, 159), which puts the camera over the box centre, and back). Live obstacles only, `stale_ms` 3000.

| Pass | Flange z | Gripper tip above box top | Min live clearance | Back-away ticks |
|---|---|---|---|---|
| A (test 2's height) | 500 | about 145 mm | 131 mm | 0 |
| B | 420 | about 65 mm by the old gripper model (see below) | 52–60 mm steady; two one-update blips to 21 mm | 0 |

- **No dropouts.** Clearance fell and rose smoothly on every approach and retreat; whenever the box was in view, it was the nearest obstacle on every update. The session converged at every waypoint, with no stale stop, errors or self drops.
- **Test 2's 16–34 mm was the bug.** At test 2's height the gripper never came closer than 131 mm to the box.
- **The 21 mm blips are the gripper's own fingertip.** Both happened about 125–155 mm from the "over" pose, on the way in and on the way out. Parked there, 7 of 20 detection reads include a flat 5 × 5 mm patch of 56–213 points at z 233, directly under the gripper, and the fingertip is visible at the bottom edge of the wrist image. The station's gripper geometry (lower box, z 265–377 at that pose) ends about 32 mm above the real fingertip. So those points are farther than 20 mm from the modelled gripper, and the self test doesn't remove them. Inflated by 10 mm, the patch sits about 22 mm below the modelled gripper, which matches the 21 mm blips. It's intermittent because the patch only sometimes has enough points to make a cluster.
- **Consequence.** The fingertip stays a fixed distance from the gripper model, so with `d_min` 10 mm it never causes a back-away. A larger `d_min`, or a gripper model ending further from the real tip, would.
- **Machine fix (applied).** armfarm15's gripper frame now has one box, 75 × 120 × 200 mm, reaching 195 mm below the flange (set through `fragment_mods`, so the shared fragment is unchanged). It replaces the ufactory module's two boxes, which ended 155 mm below the flange. In a 30 s hold at the blip pose, the fingertip patch appeared once in 57 updates and was dropped as self-detection, with no back-away. The steady clearance there is now 18–19 mm, to the box beside it, measured to the fingertips.
- **The earlier clearances were measured against the short gripper model.** The real fingertips are at least 32 mm lower than the model they used, so the real gaps were about 30 mm smaller than the tables above suggest. At the last descent step the fingertips were within a few millimetres of the box top.

Logs: `.runs/hw-sweep.out`, `.runs/hw-sweep.csv`, `.runs/depthwalk-live3000.out`.
