# Diff-IK Live Obstacles Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** a servo session can take obstacles from a vision service while it runs, and stay clear of them. It backs away from intrusions, holds still when pinched, waits for the first detections, and stops if detections go stale.

**Architecture:** each session runs a poller goroutine. It owns its own frame system and its own snapshot of the robot parts. It calls the vision service (`GetDetections3D` or `GetObjectPointClouds`, chosen from `GetProperties`), turns detections into inflated geometries in the arm's base frame, drops self-detections using the observed points, and publishes a whole set into a mutex-guarded pending slot. At tick start, the loop takes the set and rebuilds the collision model's live pairs. Live pairs inside `d_min` get back-away rows, with a two-tier relax. Staleness raises a status flag that the session turns into a ramp-stop with final state `stopped_stale`.

**Tech stack:** Go and the Viam RDK v1.16 (`services/vision`, `vision/detection3d`, `vision` objects, `pointcloud`, `referenceframe`, `spatialmath`); the MuJoCo sim; the `viam:obstacles-pointcloud` and `viam:detections-to-segments` modules.

**Spec:** `docs/superpowers/specs/2026-11-18-diff-ik-live-obstacles-design.md`. It is the source of truth; read it before any task. The collision spec and code this builds on: `docs/superpowers/specs/2026-11-18-diff-ik-collision-design.md`, and `collision.go`, `collision_tick.go`, `collision_math.go`, `qpbounds.go` and `loop.go`.

**Conventions:**
- Work in `/Users/nick.hehr/src/diff-ik` on branch `diff-ik-live-obstacles`. Never push or switch branches.
- Commit messages end with these two lines:
  ```
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_012FAJr9jcJyz2f1AEiCkX3y
  ```
- After each task, `go test ./...`, `go test -race ./...`, `go vet ./...` and `gofmt -l .` must all be clean.
- **Threading rule (critical):** the loop goroutine is the only owner of `l.coll` and its pairs. The poller owns its own `*referenceframe.FrameSystem` and its own parts snapshot. They share only the pending slot, under a mutex. Never share a `FrameSystem` between goroutines: it fills caches lazily, with no lock.
- Units: geometry is in mm, joints in rad, and steps in rad per tick.

---

## File structure

| File | Responsibility |
|---|---|
| `live.go` (new) | `LiveConfig`, `liveObstacle`, the `liveSource` interface and its `vision.Service` adapter, `poller` (the goroutine and `pollOnce`), detection parsing, the self test, inflation, the closest-`max` cut |
| `live_test.go` (new) | poller unit tests, with a fake `liveSource` |
| `collision.go`, `collision_tick.go` | live obstacles in the model (`setLive`, the `live` name), back-away rows, `collRow.backAway` |
| `qpbounds.go` | `qpNext` relax tiers (0, 1 and 2) |
| `loop.go` | the pending slot, taking a set at tick start, the waiting hold, staleness, the tier-2 retry, `live_*` status, the `StateStoppedStale` constant |
| `session.go` | starts and stops the poller; turns a stale status into a ramp-stop through `beginStop` |
| `module.go` | `Config.VisionServices`, `live_obstacles` parsing, status keys |
| `cmd/cli/*` | `--live-vision`, `--live-camera` and related flags; the `clearance` subcommand |
| `clients/python/prop_mover.py` (new) | moves sim props along a path, logging their true poses |
| `dev/panda-live.json` (new), docs, results | the cell, the runbook, the evidence |

---

### Task 1 (spec Task 0): the segmenters, depth cameras and self test

The controller does this task, because it runs the sim.

- [ ] **Step 1: The cell.** `dev/panda-live.json` and `dev/liveprobe/` already exist from a first pass; build on them, don't overwrite. The tuned obstacles-pointcloud settings are recorded in the spec's Task 0 item 2.
  - Starting point: a copy of `dev/panda-mujoco.json`.
  - Add `"depth": true` cameras: `top-depth`, fixed overhead and looking down at the table, at 640×480; and `wrist-depth`, with the arm as its frame parent, at 640×480.
  - Add two props to `sim-world`: a red box `intruder`, 80 × 80 × 120 mm, starting off the table, and a second box, `parked`.
  - Add the modules:
    - `viam:obstacles-pointcloud` (≥ 0.2.0), with services `obstacles-top` (camera `top-depth`, ground normal set for a camera looking down, `max_dist_from_plane_mm` 10) and `obstacles-wrist`;
    - `viam:detections-to-segments` (≥ 1.4.0), with a `color_detector` named `red` (detecting red) and a service `segments-top` over `top-depth` and `red`, with `infer_minimum_depth: true`.

  The sim camera's own API is in `~/src/viam-mujoco-sim/src/mujoco_module/models/camera.py`; check that its depth and point-cloud attributes, its intrinsics, and the order of its images (color first or depth first) suit both modules.
- [ ] **Step 2: Loading.** Start the cell. If a registry module doesn't load in a local config, clone it and build it from source: both are Go modules. Use a local module entry, as `viam:apriltag` needed. Record what was needed.
- [ ] **Step 3: Accuracy.** Write a scratch probe:
  - place `intruder` at 3 known poses with `set_prop_pose`;
  - call each service (`GetProperties`, then `GetDetections3D` or `GetObjectPointClouds`);
  - transform the results into the arm's base frame, using a scratch copy of the spec's step 3/4 logic;
  - compare the box centres and sizes with the truth.

  Give the probe the spec's clip, refit and extrude step (workspace: world z from 20 to 600 mm, over the table; extrude to z 20), and measure the cleaned boxes. Bar: footprint and top within 10 mm, bottom at the table. Record the method each service supports, and **whether obstacles-pointcloud's `GetDetections3D` includes a points child**. If it doesn't, stop and report (spec Task 0 item 3).
- [ ] **Step 4: Self test.** With the arm in the overhead camera's view, record what each service returns for the arm, and whether the spec's point rule drops it (more than 50% of up to 500 points within 20 mm of any part) without dropping `intruder` placed 30 mm from the gripper.
- [ ] **Step 5: Latency.** Time 50 calls of each service, and record p50 and p95. Choose an evidence `rate_hz` below 1/p95.
- [ ] **Step 6: Record.** Write `docs/results/2026-11-live-obstacles.md` with a Task 0 section, and commit the cell and results doc with "Live obstacles Task 0: segmenters, depth cameras, self test".

---

### Task 2: live obstacles in the collision model, and back-away rows

**Files:** modify `collision.go` and `collision_tick.go`; test in `collision_live_test.go` (new).

- [ ] **Step 1: The types.** In `collision.go`:
  ```go
  // liveObstacle is one live obstacle (spec "Data flow"): a geometry in the arm's base frame, already inflated.
  type liveObstacle struct {
  	geom  spatialmath.Geometry
  	label string // may be empty
  }
  ```
  Give `collObstacle` and `collPair` a `live bool` field. Live pairs are never suspended.
- [ ] **Step 2: `setLive`.**

  `func (c *collisionModel) setLive(obs []liveObstacle)` replaces the live obstacles and pairs. It runs on the loop goroutine only.
  - Remove every obstacle and pair with `live == true`. Static obstacles keep their indices, so append live obstacles at the end, and keep static pairs and their `suspended` flags.
  - For live obstacle k, the names are `["live:<k>" or "live:<k>:<label>", "live"]`.
  - Add a pair (part, live k) for every part not allowed against those names. Allows are stored on the model at build time: keep the parsed allow pairs as a field `allows [][2]string`, and reuse the same matching function.
  - Add `func (c *collisionModel) hasPairs() bool { return len(c.pairs) > 0 }`.

  In `NewCollisionModel`, `live` is always a known name for allow validation: an allow naming `live` is never "matches nothing".
- [ ] **Step 3: Back-away rows,** in `rows()` (collision_tick.go). For an active **live** pair with `g = d − dMin < 0`:
  - `collRow.backAway = true`, and `collRow.r = min(−g/2, vRetreat)`, where `vRetreat` is mm per tick. Give `rows` a `vRetreat float64` parameter, which the loop passes as `MaxLinearMMPerS·dt`.
  - Static pairs, and live pairs with g ≥ 0, keep `vmax = collisionVmax(Ad, g)` and `backAway = false`.

  Add `backAway bool` and `r float64` to `collRow`.
- [ ] **Step 4: Tests:**
  - `setLive` keeps static pairs, including a suspended one, and replaces live pairs;
  - `live` in allows removes all live pairs, and is accepted at build time with no live obstacles;
  - a live obstacle 5 mm inside `d_min` gives a back-away row with `r` = 2.5 (`v_retreat` large), and with `v_retreat` 1 it's 1;
  - a static pair inside `d_min` gives no back-away row;
  - **penetration:** an inflated live box overlapping a link (d < 0) gives a row with a nonzero gradient. Check it against an independent finite difference, as `TestCollisionGradient` does.
- [ ] **Step 5.** Commit with "Collision model: live obstacles and back-away rows".

---

### Task 3: relax tiers and the loop's collision branch

**Files:** modify `qpbounds.go` and `loop.go`; tests in `collision_live_loop_test.go` (new).

- [ ] **Step 1: The tiers in `qpNext`.** Change `relax bool` to `tier int`:
  - **Tier 0:** `b = −vmax`, or `b = r` for back-away rows.
  - **Tier 1:** `b = min(−vmax, M) − ε`, or `min(r, M) − ε` for back-away rows.
  - **Tier 2:** static rows as in tier 1; back-away rows `min(r, M, 0) − ε`.

  `M = relaxBound(jd, lb, ub)` and `ε = relaxEps`. Update the callers: today's `false` is 0, and `true` is 1.
- [ ] **Step 2: The loop.**
  - Run the collision branch when `l.coll != nil && l.coll.hasPairs()`.
  - **Keep collision "on" in live mode.** When live mode is on (a `l.live != nil` set by `SetLive` in Task 5; add the field now as nil), the per-tick status write (loop.go around lines 719–722) must leave `Collision = "on"` even with no pairs, and `SetCollision` must not set "off". Without live mode, behaviour is unchanged.
  - After tier 1 fails, if any row is `backAway`, try tier 2. If it succeeds, count `LivePinched++`, and set `qpFails = 0` as for a successful solve.
  - The late warning skips `backAway` rows.
  - Pass `vRetreat = l.cfg.MaxLinearMMPerS * dt` to `rows`.
- [ ] **Step 3: Tests**, on the Panda fixture with `SetCollision` and white-box `setLive`:
  - **Backing away at rest:** a live box placed 5 mm inside `d_min` of the attachment site, with the arm at rest. Within 100 ticks, the minimum distance is ≥ `d_min − 0.5`, `PeakAccelRatio` ≤ 1, and there's no fault.
  - **Backing away while moving toward it:** set the target so the arm moves toward the box, then `setLive` the box inside `d_min` mid-motion. The arm decelerates, then retreats, with `PeakAccelRatio` ≤ 1.
  - **Pinched:** a part between two live boxes on opposite sides along one axis, both inside `d_min`, or between one live box and a static box at `d_min`. Build it so tier 1 really is infeasible: check that tier 1 fails in a white-box call first, because otherwise a wrist rotation might satisfy both rows. Then: no fault over 200 ticks, and `LivePinched` > 0. The part may not get closer to either box by more than 0.1 mm in total, a tolerance for ε plus linearisation.
  - **The late warning** stays empty while backing away.
  - **No change:** a two-run comparison. A static-collision scenario on the Panda fixture (the approach test's box), run once with plain `SetCollision`, and once with the same model after `setLive(nil)`. The sends must be bit-identical, which shows that live support adds nothing when no live obstacles exist.
  - **Cost** (spec Task 0 item 4): `TestLiveRowsTime` measures `rows()` with the Panda parts and 32 live boxes, all within `d_act` of some part (so the 16-row cap applies), over 1000 calls. The bar is p99 < 5 ms. Log p50 and p99, and skip under `-race` and `-short`.
- [ ] **Step 4.** Commit with "Loop: back-away rows with a second relax tier".

---

### Task 4: the poller (`live.go`)

**Files:** create `live.go` and `live_test.go`.

- [ ] **Step 1: The source interface and its adapter.**
  ```go
  // liveSource is what the poller needs from a vision service (tests fake it).
  type liveSource interface {
  	GetProperties(ctx context.Context, extra map[string]interface{}) (*vision.Properties, error)
  	GetDetections3D(ctx context.Context, cameraName string, extra map[string]interface{}) ([]*detection3d.Detection, error)
  	GetObjectPointClouds(ctx context.Context, cameraName string, extra map[string]interface{}) ([]*viz.Object, error)
  }
  ```
  A `vision.Service` already satisfies this (verified in plan review); `viz` is `go.viam.com/rdk/vision`. For `q_cap`, use a small `jointReader` interface (`func(ctx) ([]float64, error)`). The session passes an adapter over the arm's `JointPositions`; tests pass a fake that returns the sim's joints. For `GetObjectPointClouds`, the source frame is the camera name (spec step 4: cameras aren't dependencies).
- [ ] **Step 2: Config.**
  ```go
  // LiveConfig is a session's live_obstacles block (spec "Inputs").
  type LiveConfig struct {
  	Vision, Camera                  string
  	RateHz, StaleMS, TimeoutMS      float64
  	InflateMM                       float64
  	Max, MinPoints                  int
  	WorkspaceMin, WorkspaceMax      *r3.Vector // world mm; both or neither
  	ExtrudeToZ                      *float64   // world mm
  }
  ```
  - Defaults: `RateHz` 10, `StaleMS` 500, `TimeoutMS` = `StaleMS`/2, `InflateMM` 10, `Max` 32, `MinPoints` 10.
  - `validate()`:
    - `RateHz` in (0, 50];
    - `StaleMS` > 1000/`RateHz`;
    - `TimeoutMS` in (0, `StaleMS`];
    - `InflateMM` ≥ 0;
    - `Max` in [1, 64], and `MinPoints` ≥ 1;
    - the workspace, when given, has min < max on each axis, and `ExtrudeToZ` lies within its z range;
    - every number finite.
- [ ] **Step 3: The poller.**
  ```go
  type poller struct {
  	cfg    LiveConfig
  	src    liveSource
  	joints jointReader            // measured joints, for q_cap
  	fsys   *referenceframe.FrameSystem // the poller's own; never shared
  	parts  *collisionModel       // the poller's own snapshot: NewCollisionModel(parts, model, armName, CollisionRequest{}, q0)
  	armName, srcFrame string     // srcFrame: the frame detections come in (spec step 4)
  	use3D  bool                  // from GetProperties
  	publish func([]liveObstacle) // stamps the time itself with the loop's clock (see Task 5)
  	errs, skipped, selfDropped, overflow, updates atomic.Int64
  }
  ```
  - `newPoller(ctx, ...)`: call `GetProperties` and choose the method (an error if neither is supported); find the camera's frame (`Properties().DefaultReferenceFrame` or the camera name); build the frame system from `parts` with no world-state transforms; reject the camera if it sits on a movable frame other than the arm.
  - `run(ctx)`: a ticker at `RateHz`, calling `pollOnce` **sequentially**. Recover from a panic in `pollOnce` and count it as an error. Return when `ctx` is done.
  - `pollOnce(ctx)` (the spec's "Each poll", steps 1–7):
    1. Read `q_cap`.
    2. Call the chosen method with a `TimeoutMS` timeout.
    3. Parse:
       - **3D detections:** `Transforms[0]`, its `Pose()` and `Geometry()`. The obstacle is `geom.Transform(pose)` in frame `Transforms[0].Parent()`. The points are the first transform with a `pointcloud.PointCloud` geometry, with each point composed through the root's pose **and that child's own pose**: the RDK's segmenter adapter uses zero, but other producers may not. The label is the top classification.
       - **Objects:** `obj.Geometry` in `srcFrame`; the points are `obj.PointCloud`; the label is `Geometry.Label()`.
       - Skip shapes other than box, capsule or sphere (`NewGeometryConfig(g).Type`).
    4. Transform the geometry and points to `armName+"_origin"` through `p.fsys`, with inputs at zero except the arm at `q_cap`. Reuse the world-state obstacle pose logic from `collision.go` (`NewPoseInFrame(parent, g.Pose())`, then re-pose), since `Transform` of a `GeometriesInFrame` drops the source frame's own transform.
    4b. **Clip, refit and extrude** (spec step 4b), when the workspace is set. Do it in world: transform the points and geometry to `World` first, then clip the points, refit a world-axis-aligned box (`spatialmath.NewBox` at the AABB centre, with zero orientation), and extrude. For a detection with no points, intersect its AABB (from `ToPoints` of the geometry, or the box's corners) with the workspace. Then transform the box to `armName+"_origin"`. Count drops in `clipped`.
    5. **Self test:** subsample at most 500 points evenly, and pose `p.parts` at `q_cap` (`poseParts`). If more than 50% of the points have `spatialmath.NewPoint(pt, "").DistanceFrom(part) ≤ 20` for any part, drop the detection and count it. No points means keep.
    6. **Inflate:** `cfg, _ := spatialmath.NewGeometryConfig(g)`; add `2·InflateMM` to box X, Y and Z, or `InflateMM` to the radius of a sphere or capsule, and `InflateMM` to half the capsule's length (capsule length + 2·inflate); rebuild with `cfg.ParseConfig()`, keeping the pose and label. Then sort by distance to the nearest part at `q_cap`, keep `Max`, and count the overflow.
    7. Call `p.publish(set)` and count `updates`. On any call error, count `errs` and publish nothing.
- [ ] **Step 4: Tests,** with a fake `liveSource`, a fake `jointReader`, and the Panda fixture frame system. The helpers are `buildColl`, `armPart` and `staticPart` in `collision_test.go`, plus `panda(t)`:
  - **Method choice:** from `GetProperties`; neither supported is an error.
  - **3D detections:** a detection with parent frame `cam` (not the camera's name), a pose, a box and a points child gives one obstacle at the composed pose in the arm frame, within 1e-6 mm. Check its points too.
  - **Objects:** a `viz.Object` with geometry and points.
  - **Wrist camera:** a camera parented to the arm, at two different `q_cap`, gives obstacles that follow the arm correctly.
  - **Inflation** of a box, a sphere and a capsule.
  - **The `Max` cap** keeps the closest, and counts the overflow.
  - **Self test:** a box around a link, with points sampled on the link surface, is dropped; a box 30 mm from the gripper with its own points is kept; a box with no points is kept.
  - **Clip, refit and extrude:**
    - a column of points from z −200 to 700 mm, clipped to [20, 600], refits to that range;
    - too few points left drops the detection (`clipped`);
    - a slab extruded to z 20 reaches it;
    - a pointless box is intersected with the workspace.
  - **Unsupported shape** is skipped and counted.
  - **Errors:** a call error publishes nothing and counts the error; a panic in the source is recovered.
  - **Sequential polls:** a slow source (sleeping 2× the period) never has two calls in flight.
  - **Stopping:** cancelling the context stops `run`. Check with a done channel; `goleak` is only an indirect dependency.
- [ ] **Step 5.** Commit with "Live obstacles: the vision poller".

---

### Task 5: the loop and the session

**Files:** modify `loop.go`, `session.go` and `session_test.go` (or create `live_loop_test.go`).

- [ ] **Step 1: The loop's pending slot.**
  ```go
  type livePending struct {
  	mu   sync.Mutex
  	set  []liveObstacle
  	at   time.Time // newest publish
  	have bool      // a set is waiting to be taken
  	ever bool      // at least one set arrived
  }
  ```
  - `func (l *Loop) SetLive(staleMS float64) (publish func([]liveObstacle))` enables live mode, before `Run`. It records the start time with `l.now()`, sets `status.Collision = "on"` and `LiveWaiting = true`, and returns the publish function. That function fills the slot under its mutex and **stamps it with `l.now()`**, the loop's clock: tests run on a fake clock, and a wall-clock stamp would never go stale there.
  - **At tick start**, before the collision rows, under the slot's mutex: if `have`, take the set (`l.coll.setLive`) and clear `have`.
  - **Age:** `now − at`, or `now − start` before the first set.
  - **Stale:** if the age is greater than `staleMS`, set `status.LiveStale = true`. The loop doesn't stop itself; the session does (Step 2).
  - **Waiting:** while no set has ever arrived, multiply the tick's `scale` by 0 through a separate `hold` factor (`ramping` stays false), and pass `held` to `advance` so the target doesn't move. Check `advance`'s signature: it takes `ramping`; add a `hold` argument and treat it the same way.
  - **Status:** `LiveCount`, `LiveAgeMS`, `LiveWaiting`, `LiveStale`, `LiveBackingAway` (any back-away row this tick) and `LivePinched`. The poller's counters reach status through the session (Step 2).
  - **A new state:** `StateStoppedStale State = "stopped_stale"`.
- [ ] **Step 2: The session.**
  - `SessionConfig` gains `Live *LiveConfig` and `LiveSource liveSource`. Task 6's `startSession` fills the source from the service's vision map; tests set it directly.
  - Don't log "collision requested but no pairs" (session.go around line 479) when `Live` is set.
  - At start, when `cfg.Live != nil`:
    - build the collision model (live mode needs one even with no static obstacles: `NewCollisionModel` with the request, or the defaults);
    - `l.SetCollision(cm)`, then `publish := l.SetLive(cfg.Live.StaleMS)`;
    - build the poller with its **own** `NewFrameSystem` and **own** parts model from the same `FrameSystemConfig` parts;
    - start `go poller.run(runCtx)` with the session's run context, so it stops with the session.
  - Keep the poller on the session, so its counters reach status (`SessionStatus` gains them).
  - In `onTick`: if `st.LiveStale && s.stopping == ""`, set `s.stopping = StateStoppedStale` and call `s.loop.RampStop(s.ramp, StateStoppedStale)` directly, exactly as the dead-man does. Don't use `beginStop`: `onTick` already holds `s.mu`, and `beginStop` locks it again, which deadlocks.
  - Add `StateStoppedStale` to the terminal states (session.go around line 154).
  - An explicit `servo_stop` still upgrades the final state to `stopped`, as for `stopped_timeout`.
- [ ] **Step 3: Tests:**
  - **Waiting:** set a target 50 mm from the start pose (or a twist in twist mode) **before** any publish. Over 50 ticks, no motion is sent (the sends equal the start), the stored target doesn't advance, and `LiveWaiting` is true. After a publish, the arm moves toward the target. Without the hold, this test fails.
  - **Staleness:**
    - no publish within `stale_ms` gives a ramp-stop ending in `stopped_stale`;
    - publishing every 100 ms with `stale_ms` 500 never goes stale;
    - one missed publish isn't stale.
  - **Final state:**
    - a dead-man timeout during the stale ramp leaves `stopped_stale`;
    - `servo_stop` during it gives `stopped`.
  - **Hand-off under `-race`:** a goroutine publishing at 100 Hz while ticks run.
  - **The session stops the poller** when it ends.
- [ ] **Step 4.** Commit with "Loop and session: live hand-off, waiting hold, stale stop".

---

### Task 6: the module

**Files:** modify `module.go` and `module_test.go`.

- [ ] **Step 1: Dependencies.**
  - `Config.VisionServices []string` (`json:"vision_services,omitempty"`).
  - `Validate` adds them to the dependencies as vision service resource names, the same way arms are added. Check how `Validate` returns arm names (it returns plain names, which the RDK resolves), and follow that.
  - `NewServo` resolves them with `vision.FromProvider(deps, name)` (`FromDependencies` is deprecated) and keeps a map on the manager. `startSession` sets `SessionConfig.LiveSource` from it.
- [ ] **Step 2: Parsing `live_obstacles`** in `startSession`, as an object with the spec's fields:
  - `vision` must be in `vision_services`;
  - `camera` is required;
  - every number goes through `LiveConfig.validate`;
  - unknown keys are rejected;
  - it requires `"solver": "qp"`; `"collision": false` together with it is rejected;
  - it turns collision on.

  Store it in `SessionConfig.Live`, along with the vision service handle, or its name for the manager to resolve.
- [ ] **Step 3: `statusMap`.** When live mode is on, add:
  - `live_count`, `live_age_ms`, `live_updates`, `live_errors`, `live_self_dropped`, `live_skipped`, `live_overflow`;
  - `live_backing_away`, `live_pinched`, `live_waiting`;
  - `live_method` (`"detections3d"` or `"object_point_clouds"`).
- [ ] **Step 4: Tests:**
  - **Validation:** every case above.
  - **Dependencies:** `Validate` returns the vision service dependencies.
  - **Status keys:** a session with a fake vision service that implements `GetProperties` and `GetDetections3D` reports them. Use the RDK's `vision` fake if there is one (`services/vision/fake`?), or a small `inject`-style fake.
  - **Teleop:** `live_obstacles` in the teleop defaults.
- [ ] **Step 5.** Commit with "Module: vision_services and live_obstacles".

---

### Task 7: the CLI and the evidence tools

**Files:** modify `cmd/cli/main.go` and `cmd/cli/run.go`; create `clients/python/prop_mover.py` and its test.

- [ ] **Step 1: CLI flags.**
  - `--live-vision`, `--live-camera`, `--live-rate-hz`, `--live-stale-ms` and `--live-inflate-mm`.
  - `--live-vision` needs `--solver qp`, and turns collision on.
  - The CLI resolves the vision service from the robot client (`vision.FromProvider` or similar), builds the poller exactly as the session does (with its own frame system and parts), and wires `SetLive`. Factor a helper in package `diffik`, `StartLive(ctx, l, src, joints, parts, model, armName, cfg) (*LiveHandle, error)`, so the session and the CLI share it, and refactor Task 5's session code to use it. `LiveHandle` has `Stop()` and `Stats()`, which returns the poller's counters and `Method`; the session's status and the CLI summary both read it.
  - **The CLI stops on stale data itself:** its `OnTick` (or the recorder's) calls `l.RampStop(ramp, StateStoppedStale)` once, when `Status.LiveStale` becomes true. The CLI runs a bare `Loop`, with no session to do it.
  - The summary prints:
    ```
    live: updates %d errors %d self %d skipped %d overflow %d clipped %d pinched %d waiting-ticks %d
    ```
    and, if it happened, `stopped_stale`. `waiting-ticks` counts the recorded rows with `LiveWaiting`.
- [ ] **Step 2: Hold-while-props-move.** A `run hold` scenario already exists; give it a `--hold` duration long enough for a prop sweep. No new scenario is needed. `circle` already exists too.
- [ ] **Step 3: `clearance` subcommand.**
  ```
  cli clearance --csv RUN.csv --props PROPS.csv
  ```
  For each run row's time, it:
  - interpolates each prop's true pose from `PROPS.csv` (columns `t`, `name`, `x`, `y`, `z`, `ow`, `ox`, `oy`, `oz`, sizes, on the same clock);
  - builds the prop's **uninflated** box in world, transformed to the arm frame;
  - poses the parts at `q_cmd`;
  - takes the minimum distance.

  It prints the minimum clearance per prop and overall, with the time and the part. It needs the frame-system parts from the machine (the `--address` flags) and the arm's model.

  Export the minimal helper it needs from `diffik`: `MinClearance(model, parts, armName, q, obstacles []spatialmath.Geometry) (float64, string, error)`, which builds a parts-only model once and reuses it.

  The run CSV's `elapsed_s` and the props CSV share a clock: both record Unix time. Add an `epoch_s` column to the run CSV header (in `run.go`'s recorder), and have `prop_mover.py` write Unix time.
- [ ] **Step 4: `clients/python/prop_mover.py`.**
  - It moves a named prop along a straight line, from point A to point B in world mm, at a set speed. It can hold at B for N seconds, then optionally move back.
  - It calls `sim-world`'s `set_prop_pose` (see `tag_sweep.py` `place()` for the do_command shape) at 50 Hz, and logs the props CSV with Unix time.
  - Flags: `--prop`, `--from x,y,z`, `--to x,y,z`, `--mm-s`, `--hold-s`, `--back`, `--csv`, `--address`, `--insecure`.
  - Add a small test of the path function, in the style of the other client tests.
- [ ] **Step 5.** Commit with "CLI and tools: live obstacle flags, clearance replay, prop mover".

---

### Task 8: docs

- [ ] Document the following in `devrel_diff-ik_servo.md`, the README's Solvers paragraph, and `dev/panda-mujoco.md` (a "Live obstacles" section, using `dev/panda-live.json`):
  - `vision_services`, `live_obstacles` and its fields;
  - backing away, pinch-hold, waiting and `stopped_stale`;
  - the status keys;
  - the two segmenter pipelines, with example service configs from Task 1;
  - the known limits: no velocity prediction, perception error, and capture time approximated by request time.
- [ ] Commit with "Docs: live obstacles".

---

### Task 9: sim evidence

The controller does this task, with `dev/panda-live.json`.

Each run uses both pipelines (`obstacles-top`, and `segments-top`), at the `rate_hz` chosen in Task 1. True clearance comes from `cli clearance` on the run CSV and the props CSV.

- [ ] **Step 1: Intruder into a held pose.** `cli ... --live-vision obstacles-top --live-camera top-depth run hold --hold 20s`, while `prop_mover.py` slides `intruder` into the gripper's region at 50 mm/s, holds 3 s, then backs out.
  - Bars: true clearance ≥ `d_min − 5`, 0 faults, peak acceleration ratio ≤ 1, and the arm returns to its pose.
- [ ] **Step 2: Parked prop on a circle.** Park `parked` on the circle path, run `run circle`, then remove it.
  - Bar: no part↔prop distance below `d_min − 5`. Report the TCP error.
- [ ] **Step 3: Stale stop.** Stop the vision module mid-run, by removing the service from config or killing the module process.
  - Bar: `stopped_stale` within `stale_ms` + one ramp.
- [ ] **Step 4: Intrusion speed sweep.** Props at 100, 200 and 400 mm/s. Report the true clearance at each.
- [ ] **Step 5: Write up** the results doc's evidence section, and commit with "Live obstacles results: sim evidence".

---

### Task 10: finish, and the hardware checkpoint

- [ ] **Step 1.** Run the full checks (Go, race, vet, gofmt, Python), then get a final Opus whole-branch review, and fix what it finds.
- [ ] **Step 2.** Squash-merge to `main` with "Diff-IK live obstacles from a vision service" and the trailer. Don't push.
- [ ] **Step 3: Hardware checkpoint.** **Stop and ask the user** before anything on a real robot:
  - which robot and depth camera;
  - when they'll be present;
  - and the exact armfarm config changes, written out for their approval.
- [ ] **Step 4.** Offer the watchable sim demo on `mujoco-sim-mac` (back up its config, then restore it).
