# Diff-IK live obstacles from a vision service: Design

Date: 2026-11-18
Builds on:
- collision avoidance on the QP solver, `docs/superpowers/specs/2026-11-18-diff-ik-collision-design.md` (merged as `1470f58`);
- its results, `docs/results/2026-11-qp-collision.md`.

## Goal

React to obstacles that appear or move **during** a servo session, detected by a vision service from a depth camera, in the camera's frame. Today's collision avoidance fixes its obstacles at `servo_start`.

A one-shot motion plan can't do this: its plan goes stale the moment something moves. The servo loop re-measures distances every tick, so it only needs the obstacles to stay current.

The work is prototyped in the MuJoCo Panda cell with a depth camera and moving props, then tried on a real robot with the user present.

## Decisions

| Topic | Decision |
|---|---|
| Source | A vision service polled by the module, at a configured rate. It uses `GetDetections3D` when the service's `GetProperties` reports `Detections3DSupported`, otherwise the bounding geometry from `GetObjectPointClouds`. Obstacles are bounding geometries only; observed points are used for one thing, the self-detection test. |
| Segmenters | **`viam:obstacles-pointcloud` (≥ 0.2.0):** class-agnostic. It removes the table plane and serves `GetDetections3D`. **`viam:detections-to-segments` (≥ 1.4.0) over a 2D detector**, such as the RDK's `color_detector`: labelled, and the arm isn't detected. Both are evidence pipelines. |
| Hand-off | A poller goroutine per session publishes a whole obstacle set. The loop goroutine takes it at the start of a tick and stays the collision model's only owner. |
| Intrusion | A live obstacle closer than `d_min` makes the arm **back away**: the row requires the gap to grow, within the step bounds. If the back-away rows can't be met together (the part is pinched between obstacles), the second tier applies: **no further approach**, plus a **soft retreat** (see "The row for a live pair"). |
| Clean-up (added after Task 0) | Each detection's points are **clipped** to a configured workspace box in world, its box is **refit** around the points that remain, and it can be **extruded** down to a support height. Task 0 found that edge pixels in the depth image joined props into 600–800 mm columns with things above or below them, and that a camera looking down sees only top faces. |
| Before the first set | The arm **holds its position** (the task step is scaled to 0) until the first live set arrives, so it never moves blind. Status shows `live_waiting`. |
| Stale data | Keep the last set for `stale_ms`, then ramp-stop the session as `stopped_stale`. |
| Static obstacles | Unchanged. Frame-system and `world_state` obstacles, and their suspend-at-start rule, still apply alongside the live ones. Backing away applies to live obstacles only. |

## Inputs

The module config gains:

| Field | Notes |
|---|---|
| `vision_services` | Vision service names this service may poll. They become dependencies, as `arms` do. |

`servo_start` gains:

```json
"live_obstacles": { "vision": "obstacles", "camera": "depth-cam", "rate_hz": 10,
                    "stale_ms": 500, "inflate_mm": 10, "max": 32 }
```

| Field | Default | Notes |
|---|---|---|
| `vision` | required | One of `vision_services` |
| `camera` | required | Passed to the vision call. Its frame must be in the frame system. |
| `rate_hz` | 10 | In (0, 50] |
| `stale_ms` | 500 | Must be > 1000/`rate_hz` + `timeout_ms`, so one slow poll can't stale-stop the session |
| `timeout_ms` | `stale_ms`/2 | Each vision call's timeout: > 0 and ≤ `stale_ms`. Task 0 sets `rate_hz` below 1/p95 of the segmenter's latency. |
| `inflate_mm` | 10 | ≥ 0. Each detection's geometry grows by this much on every side. |
| `max` | 32 | 1–64. The closest `max` to the arm are kept. |
| `workspace` | none | `{"min_mm": [x, y, z], "max_mm": [x, y, z]}` in **world**. When given, every detection is clipped to it (see Data flow step 4b). min < max on each axis. |
| `extrude_to_z_mm` | none | When given, each detection's box is extended down to this world z, for example the table top, so a top-face slab becomes a volume resting on the support. It must lie inside `workspace` when both are given. |
| `min_points` | 10 | A detection whose points drop below this after clipping is discarded (counted in `live_clipped`). Applies only when `workspace` is set; extrude alone never drops a sparse detection. Detections that carry no points are clipped by their box alone. |

`live_obstacles` needs `"solver": "qp"`, and turns collision on, with the defaults of the collision spec when no `collision` object is given. `"collision": false` together with `live_obstacles` is rejected. Each validation error names its field. The teleop defaults block accepts `live_obstacles` too.

## Data flow

**The poller goroutine** is started at `servo_start` and stopped through the session's context. Polls are **sequential**: one call in flight, and a slow call delays the next poll rather than overlapping it.
**At `servo_start`:**
- Call the vision service's `GetProperties`. Use `GetDetections3D` if `Detections3DSupported` is set, otherwise `GetObjectPointClouds` if `ObjectPCDsSupported` is set. Reject the session if neither is.
- Build the poller's **own** frame system from the same `FrameSystemConfig` parts, and its **own** snapshot of the robot parts: the same links and mounted parts the collision model uses, posed by the poller. The poller never touches the loop's collision model, and never shares a `FrameSystem` with the session or the loop. The RDK's `FrameSystem` fills caches lazily with no lock.

**Each poll (every 1/`rate_hz`):**
1. Read the arm's measured joints, `q_cap`. The vision API returns no capture time, so the joints at request time stand in for the capture pose; for a wrist camera, the arm's motion during the call is an accepted error.
2. Call the chosen method on `camera`, with timeout `timeout_ms`.
3. **Take each detection's geometry and points:**
   - **`GetDetections3D`:** each `detection3d.Detection` carries a transform tree. Use only `Transforms[0]`: the obstacle is its geometry composed with its pose, in the frame `Transforms[0].Parent()` (the camera's reference frame, which needn't equal the camera's name). Its child transforms are ignored, except the point-cloud child (`<root>/points`), which is used only for the self test. The label is the top classification, if any.
   - **`GetObjectPointClouds`:** each `viz.Object` has a bounding `Geometry` and a `PointCloud`, in the camera's frame. The label is `Geometry.Label()`.
   - Skip and count any detection with no geometry, or a shape other than box, capsule or sphere.
4. **Transform** each geometry, and its points, into the arm's base frame. For `GetDetections3D`, the points child is expressed in the root's frame, so compose the points with `Transforms[0]`'s pose first, then apply the parent-to-base transform. Find the points child by its geometry type (a `pointcloud.PointCloud`, such as `*pointcloud.BasicOctree`), not by its `<root>/points` name. For `GetObjectPointClouds`, whose objects carry no frame, the source frame is the **camera name**. Cameras aren't dependencies of this service, so their properties can't be read. A camera whose frame-system name differs from its resource name isn't supported on that path; the docs say so. All transforms go through the poller's frame system, with the arm at `q_cap` and the other frames at zero inputs. A camera on a movable frame other than the arm is rejected at start.
4b. **Clip, refit and extrude** (when `workspace` is set), in **world**. Transform the points and geometry to world first; after this step, transform the result to the arm's base frame.
   - **Clip:** drop the points outside `workspace`. If fewer than `min_points` remain, drop the detection and count it in `live_clipped`.
   - **Refit:** replace the geometry with the **world-axis-aligned box** around the remaining points. The detection's orientation is lost; a box is the conservative choice.
   - **No points:** for a detection with no points, the new box is the intersection of the geometry's world-aligned bounding box with `workspace`. If the intersection is empty, the detection is dropped.
   - **Extrude:** if `extrude_to_z_mm` is set, lower the box's bottom face to that z, keeping its top.
5. **Drop self-detections.** Take up to 500 of the detection's observed points (an even subsample), after clipping. If more than 50% of them are within 20 mm of **any** robot part posed at `q_cap` (`DistanceFrom` a point geometry), it's the arm seeing itself: drop it and count it. A detection with no points can't be tested, and is kept. This works on the points the camera actually saw, not on the box volume: a segmenter that wraps the whole arm in one box still sees arm-surface points. Task 0 checks the rule.
6. **Inflate** each geometry by `inflate_mm`, then keep the closest `max` to the arm (the overflow is counted).
7. **Publish** the set, with the publish time, in a mutex-guarded pending slot; a newer set replaces an older one that hasn't been taken yet.

**The loop, at tick start:**
1. Take the pending set if there is one. It replaces the live obstacles wholesale; there's no identity tracking.
2. Rebuild the live pairs: every robot part against every live obstacle, minus the `collision_specification` allows. Every live obstacle's names include `live`, so `{"frame1": "pick-gripper", "frame2": "live"}` allows all of them. `live` is always a known name for allow validation, so it's accepted at `servo_start` before any live obstacle exists. Static pairs, and their suspension state, are untouched.
   - The collision branch of the tick runs whenever there are **any** pairs, static or live. So with `live_obstacles` set, a session with no static obstacles still gets live rows.
   - The collision status is `"on"` whenever `live_obstacles` is set, and the "collision requested but no pairs" warning doesn't apply.
3. Live pairs are never suspended.
4. **Before the first set arrives:** hold position, and report `live_waiting: true`. The age counts from `servo_start`.
   - The hold is a **separate factor** multiplied into the tick's `scale`, which is 0 while waiting. It must **not** use the ramp state: no `rampStart`, no `RampStop`, and `ramping` stays false. Otherwise the tick's ramp-complete check (ramping, scale 0, `canStop`) would end the session on the first tick.
   - `advance` holds the target while waiting, the same way it does while ramping. That stops `SetMotion`/teleop feed-forward from walking the target away, which would make the arm lunge when the first set arrives.
5. If the newest set (or the wait for the first) is older than `stale_ms`, start a ramp-stop through `session.beginStop`, so `s.stopping` is set, with final state `stopped_stale`. As with `stopped_timeout`, the dead-man can't overwrite it, and an explicit `servo_stop` upgrades it to `stopped`.

## The row for a live pair

- **g = d − d_min ≥ 0:** the existing acceleration-aware damper, `J_d·Δq ≥ −v_max`.
- **g < 0 (inside `d_min`): back away.** The row is `J_d·Δq ≥ r`, with `r = min(−g/2, v_retreat·dt)`.
  - `v_retreat` is the session's `linear_mm_s`, so the arm can't retreat faster than it may move.
  - If the bounds can't reach `r` this tick (for example, the arm is moving the other way and must decelerate first), the existing relax step lowers the row to the strongest move away the bounds allow, `min(r, M) − ε`. So the arm retreats as fast as its acceleration limit permits.
- **Pinched (two-tier relax):** relaxing each row on its own doesn't make the rows feasible together. A part can be pinched between two live obstacles, or between a live obstacle and a static one at `d_min`, and several arm parts inside one obstacle have conflicting gradients. In that case the relax step has a **second tier**: if the first relaxed solve also fails **and** there are back-away rows, retry with every back-away row kept hard at "no further approach", `min(0, M) − ε`, and add a weighted **soft retreat** objective per back-away row, `w_b·(J_d·Δq − r)²` (`H += w_b·J_dᵀJ_d`, `g −= w_b·r·J_d`; `w_b` is `retreatWeight` = 1e-6, about the same cost per mm as task error, in m² per mm² because task rows are in metres and `J_d`, `r` in mm). The arm retreats as far as the conflicting rows allow together, instead of holding still. Static rows keep their tier-1 bounds. At rest, Δq = 0 still satisfies every hard row, which keeps the collision spec's fault rule sound. Why: the sim showed the pure hold against an advancing obstacle: the arm stood still for most ticks while the obstacle kept coming and ended inside it (−0.67 to −19.68 mm true clearance across runs before the change). 1e-5 overshot in the sim (−12 mm), and 1e-4 broke the 0.1 mm static-pinch bar. Count it in `live_pinched`. Only if the second tier fails does the tick brake.
- **The late warning** (`CollisionLate`) excludes all live pairs: a detection can appear inside `d_act` (or inside `d_min`, where `v_max` is 0), so "d_act too small" would be misleading. For static pairs the threshold is `v_max + A_d + tol`.
- **Interaction with the task:** the task still pulls toward the target, but the rows are hard constraints, so the arm holds at `d_min` and resumes tracking when the obstacle leaves.
- **Moving obstacles:** the damper only constrains the arm's own motion. An obstacle moving toward the arm still closes the gap, and the arm then backs away. This version has no velocity estimate or prediction. The poll rate and `inflate_mm` set how fast an intrusion it can handle, and the evidence measures that.

## Failures

| Case | Behaviour |
|---|---|
| Vision RPC error or timeout | That update is skipped and counted. The last set is kept until `stale_ms`. |
| The newest set is older than `stale_ms` | Ramp-stop, final state `stopped_stale`, with the reason in `servo_status` |
| A transform error for one detection | That detection is skipped and counted |
| A detection's frame can't be placed (`GetDetections3D` path) | Safer deviation: the whole poll fails and the session goes stale, rather than skipping that detection |
| The camera name isn't a frame (`GetDetections3D` path) | Safer deviation: accepted, because detections carry their own frames. The check below applies to the `GetObjectPointClouds` path |
| The self test sees a real obstacle in near-contact | **Limit:** the self test can drop a real obstacle in near-contact. Anything with more than half its visible points within 20 mm of a robot part (a hand around a link, a small object against the gripper) is treated as the arm itself, so protection lapses at very close range |
| More than `max` detections | The closest `max` are kept, and the overflow is counted |
| The poller panics | Recovered and counted as an error, so the session goes stale after `stale_ms` |
| The camera frame is missing from the frame system, or sits on a movable frame other than the arm | Rejected at `servo_start` |
| Session end | The poller stops through the session's context, and no RPC is left in flight past its timeout |

The relax and brake fallbacks from the collision spec apply, plus the second tier for back-away rows. The brake path covers live rows too.

## Status

When `live_obstacles` is configured:

| Field | Meaning |
|---|---|
| `live_count` | Live obstacles in use this tick |
| `live_age_ms` | Age of the newest set |
| `live_updates`, `live_errors` | Session totals |
| `live_self_dropped`, `live_skipped`, `live_overflow`, `live_clipped` | Session totals |
| `live_backing_away` | True while any live pair has g < 0 |
| `live_pinched` | Session total of second-tier relaxes |
| `live_waiting` | True until the first live set arrives |
| `live_method` | `"detections3d"` or `"object_point_clouds"` |

`collision_min_mm` and `collision_pair` include live pairs. Live obstacles are named `live:<n>`, or `live:<n>:<label>` when the detection has a label.

## Task 0: checks before any code

These run in the Panda cell. The results doc records each one; if a check fails, stop and report.

1. **The segmenters run here.** Do `viam:obstacles-pointcloud` (≥ 0.2.0) and `viam:detections-to-segments` (≥ 1.4.0) load in a local config on this Mac? If the registry builds don't, build them from source, which is what `viam:apriltag` needed. Confirm that obstacles-pointcloud answers `GetDetections3D`, and that detections-to-segments with `color_detector` answers `GetObjectPointClouds` with bounding geometries.
2. **A depth camera in the sim.** Add `depth: true` cameras to the Panda cell, one fixed overhead and one on the wrist, and confirm their point clouds line up with the frame system. Place a known box prop, and check that both pipelines' detected boxes, **after clip, refit and extrude** (a workspace above the table, extruded to the table top), fit the prop's true box within 10 mm. That means: footprint x/y edges, top z, and bottom at the support.
   - **Recorded before clean-up was added:** raw boxes were accurate in x and y (about 10 mm), but they were top-face slabs. They were sometimes merged into 600–800 mm columns by depth edge pixels, and detections-to-segments took in everything inside the 2D box. The tuned obstacles-pointcloud settings were: plane band 25 mm, `min_points_in_plane` 2000, ground tolerance 10°, `min_points_in_segment` 50, ground normal (0, 0, −1) in the camera frame. Latency p95 was 94 ms (obstacles-pointcloud) and 63 ms (detections-to-segments).
3. **Self-detection.** Does obstacles-pointcloud's `GetDetections3D` include the points child? Without one, the box around the arm would be kept (no points means kept) and the arm would back away from itself; if it's missing, stop and report. With the arm in view, what does it return for the arm? Does the 50% overlap rule drop it without dropping a prop held 30 mm from the gripper?
4. **Rate and cost.** Measure each pipeline's latency (p50 and p95) at the camera's resolution, and set the evidence runs' `rate_hz` below 1/p95. Also measure the tick p99 at the **maximum** pair count (all parts × 32 live boxes, with the 16 closest as rows), against the 5 ms bar.

## Tests

**Unit tests,** with a fake vision service, a fake arm and the Panda fixture:
- **Poller:**
  - transforms detections at `q_cap`, for a fixed camera and for a wrist camera;
  - inflation;
  - the `max` cap, keeping the closest;
  - the self-drop rule;
  - skipping detections with no geometry or an unsupported shape;
  - counting errors;
  - stopping when the session ends, with nothing in flight.
- **Hand-off:**
  - the set is taken only at tick start;
  - live pairs are rebuilt while static pairs keep their suspension state;
  - `live` in `allows` works;
  - no data race under `-race`.
- **Reading detections:** a `detection3d.Detection` with a non-camera-named parent frame, a pose plus a geometry, and a points child gives exactly one obstacle, at the composed pose. The method is chosen from `GetProperties`.
- **Self-drop:** a box around the whole arm, with its observed points on the links, is dropped. A prop 30 mm from the gripper is kept.
- **Clip, refit and extrude:**
  - a column of points from z −200 to 700 above a prop, clipped to z [20, 600], refits to the in-workspace part;
  - fewer than `min_points` remaining drops the detection;
  - a slab at the top face, extruded to z 20, reaches the table;
  - a pointless detection is intersected with the workspace.
- **Backing away:**
  - an obstacle placed 5 mm inside `d_min` with the arm at rest: the arm moves away until d ≥ `d_min − 0.5`, with `peak_accel_ratio` ≤ 1;
  - the same while the arm is moving toward the obstacle: it decelerates, then retreats.
- **Penetration:** an inflated live box that already overlaps a part (d < 0) gives a back-away row with a usable gradient, and the arm separates.
- **Pinched:** a part between two live boxes inside `d_min`, or between a live box and a static one, holds via the second tier and doesn't fault (`live_pinched` > 0).
- **The late warning** doesn't fire for back-away rows.
- **Waiting:** before the first set, the arm doesn't move and `live_waiting` is true.
- **`stopped_stale` is final:** a dead-man timeout during the stale ramp doesn't overwrite it.
- **Staleness:**
  - the fake service stops answering, so the session ramp-stops after `stale_ms` as `stopped_stale`;
  - one dropped update doesn't stop it;
  - a session that never receives a first set goes stale after `stale_ms`.
- **Off:** with no `live_obstacles`, the sends are bit-identical to the collision path without it (a two-run comparison).
- **Module:**
  - parsing and every validation case;
  - `vision_services` as dependencies;
  - the status keys;
  - the teleop defaults path.

## Evidence: sim, Panda cell

The camera feeds the segmenter, which feeds the servo, so this is the real perception path, not injected geometry. Each run uses both pipelines.

| Run | Shows | Bar |
|---|---|---|
| The arm holds a pose while a box prop slides into its path at 50 mm/s | it moves out of the way, then returns | min commanded clearance to the prop ≥ `d_min − 5 mm`; 0 faults; peak acceleration ratio ≤ 1 |
| The arm tracks a circle while a prop parks on the path | it detours or stops, then completes the circle once the prop is removed | no part↔prop distance below `d_min − 5 mm` |
| The vision service is stopped mid-run | a clean stop | `stopped_stale` within `stale_ms` plus one ramp |
| Props swept into the path at 100, 200 and 400 mm/s | where reactive avoidance stops keeping up | reported only: min clearance at each speed |

The clearance is measured against the prop's **true**, uninflated geometry, at the poses the runner commanded with `set_prop_pose`, by replaying `q_cmd` through the collision model. It is not measured against the detected box.

The bar of `d_min − 5 mm` is looser than the static one to allow for perception error; Task 0's box-error measurement checks that 5 mm is right.

## Real robot

After the sim passes, and only with the user present:
- an armfarm arm with a RealSense, or the SO-101 with a depth camera, whichever is available;
- the same two pipelines (an ML or color detector for detections-to-segments);
- a hand or a box moved into the workspace.

Before touching an armfarm machine's config, the exact changes are written up and the user is asked.

## Output

- `docs/results/2026-11-live-obstacles.md`: Task 0, the sim evidence, and the real-robot runs.
- Docs: `live_obstacles`, `vision_services`, the status keys, and the two pipelines.
- A watchable demo on `mujoco-sim-mac`, restored afterwards.

## Process

- Branch `diff-ik-live-obstacles`.
- Spec, then plan, then subagent-driven work (Sonnet implements, Opus reviews).
- Squash-merged to `main`. No push or release unless the user asks.

## Out of scope

- Velocity prediction for moving obstacles.
- Object identity tracking.
- Using point clouds directly (only bounding geometry is used).
- A client push API.
- More than one camera per session.
- Allowing one specific live object (for example a pick target) by label. Only `live` (all live obstacles) is an allow name in this version.
