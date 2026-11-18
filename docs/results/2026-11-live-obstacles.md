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

- **Tier 2 had to change.** As first specified, tier 2 held the arm still when back-away rows conflicted. Several arm parts end up inside one inflated, extruded box, with conflicting retreat directions. Against a box that keeps advancing, holding still meant a guaranteed intrusion (175–238 held ticks, −0.7 to −17 mm). Arm speed wasn't the limit: a slower box was *worse*, and a 3× faster, 6× harder-accelerating arm barely helped. Tier 2 now keeps "no further approach" hard and adds a soft retreat term. This took obstacles-pointcloud at 50 mm/s from −0.67 to +5.05 mm, and the unit test from −9.95 to +7.30 mm. The spec is updated.
- **The retreat weight is a trade-off.** 1e-6 is about the same cost per mm as task error. 1e-5 overshot in the sim (−12 mm), and the larger 1e-4 broke the 0.1 mm static-pinch bar. Better weighting, such as scaling with the intrusion depth or a per-part priority, is follow-up work.
- **Reactive avoidance keeps up with about 50 mm/s intrusions at these arm limits** (30°/s, 60°/s², roughly 150–250 ms from capture to an obstacle update). Faster intrusions end 18–22 mm inside the prop: the retreat is limited by perception latency and the arm's acceleration, and there's no prediction. Predicting obstacle velocity (out of scope) would be the next step.
- **detections-to-segments is about 10 mm worse than obstacles-pointcloud** at the same speed, even at 12 Hz. Its boxes come from the 2D detection's extent, which is narrower and noisier than the clustered points.
- **The self test works.** obstacles-pointcloud returned the arm as 721–1219 detections per run; all were dropped, and the arm never backed away from itself. detections-to-segments never sees the arm.
- **The parked-prop and stale-stop bars pass.**
- **Two bars miss:** the slow intrusion (+3.82 mm) and detections-to-segments at 50 mm/s (−4.76 mm), both against the 5 mm bar. They're recorded as failed, not relaxed.

### Not done and deviations

- **Parked prop:** the arm stopped short (16.9 mm), but completing the path after the prop was removed wasn't shown; the prop stayed for the whole run.
- **Pipelines:** the parked, stale and speed-sweep runs used obstacles-pointcloud only; the spec asked for both pipelines.
- **Tick time:** the unit test gives 1.1 ms p99 at 256 pairs; the whole-tick p99 in the sim runs was 8.7–15.0 ms, with 0–4 overruns.
- **Brakes and late warnings:** the intrusion runs had 1–8 brakes, and some late warnings (M7).
- **Still to come:** the real-robot runs and the `mujoco-sim-mac` demo.

The logs and CSVs are in `.runs/live-*`.
