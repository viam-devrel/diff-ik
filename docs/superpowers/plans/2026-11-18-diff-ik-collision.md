# Diff-IK Collision Avoidance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** keep the arm and its mounted parts at least `d_min` from known obstacles while it servos. The constraint is an acceleration-aware distance damper inside the QP, and the obstacle and allowed-pair inputs take the motion service's `WorldState` and `CollisionSpecification` shapes.

**Architecture:** at `servo_start`, `NewCollisionModel` builds a list of robot parts and obstacles, and the pairs between them, from a local frame system plus the session's `world_state`. Every tick, `(*collisionModel).rows` measures each pair with `DistanceFrom`, takes a finite-difference distance gradient for the close ones, and returns damper rows that `qpNext` passes to `solveQP` as general rows. If the solve fails, the tick retries with relaxed rows, then brakes. Collision is opt-in, and nothing changes when it's off.

**Tech stack:** Go with gonum and the Viam RDK v1.16 (`referenceframe`, `spatialmath`, `go.viam.com/api` motion and common protos), plus the MuJoCo sim for evidence.

**Spec:** `docs/superpowers/specs/2026-11-18-diff-ik-collision-design.md`. Read it before any task. It is the source of truth for behaviour; this plan is how to build it.

**Conventions for every task:**
- Work in `/Users/nick.hehr/src/diff-ik` on branch `diff-ik-collision`. Never switch branches, and never push.
- Commit messages end with these two lines:
  ```
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_012FAJr9jcJyz2f1AEiCkX3y
  ```
- After each task, `go test ./...`, `go vet ./...` and `gofmt -l .` must all be clean.
- Units:
  - joint steps are rad per tick, and joint angles are rad;
  - poses and geometry from `model.Transform`, `Geometries` and `DistanceFrom` are in mm;
  - `solve` works in metres internally, but collision code never touches it.
- Existing helpers: `brakeCap(a, d, dt)` and `qpBounds` in `qpbounds.go`; `fdStep` in `kin.go`; `panda(t)`, `ur5e(t)`, `so101(t)` and `nearVec` in `kin_test.go`; `newTestLoop`, `runTicks`, `offsetTarget`, `simIO`, `newClockLoopModel` and `clockTicks` in `loop_test.go`; `repeat` in `qpbounds_test.go`; `decodeProto` in `teleop.go`; `fakeFS` and `teleopFS` in the session and teleop tests.

---

## File structure

| File | Responsibility |
|---|---|
| `collision_math.go` (new) | `collisionVmax`, `relaxBound`, `brakeStep`: pure maths |
| `collision.go` (new) | `CollisionRequest`, `collisionModel`, `NewCollisionModel`: build parts, obstacles and pairs, match names, reject meshes, skip movable frames, suspend pairs inside `d_min` |
| `collision_tick.go` (new) | `poseParts`, `rows`: per-tick distances, gradient, damper rows, re-arming |
| `collision_parse.go` (new) | `ParseWorldState`, `ParseCollisionSpecs`: protojson decoding shared by the module and the CLI |
| `*_test.go` for each of those | unit tests |
| `qpbounds.go` | `qpNext` takes the collision rows and a `relax` flag |
| `loop.go` | `SetCollision`, the collision branch in `tick` (relax, then brake, then the fault rule), status fields |
| `session.go` | `SessionConfig.Collision`; builds the local frame system and the model at start |
| `module.go` | parses `world_state`, `collision_specification` and `collision`; status keys |
| `cmd/cli/main.go`, `cmd/cli/run.go` | collision flags, `run descend`, summary lines |
| `docs/results/2026-11-qp-collision.md` (new) | Task 0 findings and evidence |
| `README.md`, `devrel_diff-ik_servo.md`, `dev/panda-mujoco.md` | docs |

---

### Task 1 (spec Task 0, items 1–3): what the Panda cell provides

The controller does this task, because it runs the sim.

- [ ] **Step 1.** Start the Panda cell: `go build -o bin/diff-ik ./cmd/module`, then `viam-server -config dev/panda-mujoco.json` in the background. Port 8083; the world takes about 30 s.
- [ ] **Step 2.** Write a scratch Go program in the scratchpad (not the repo). It connects with `client.New(... rpc.WithInsecure() ...)` and does the following:
  - Reads `machine.FrameSystemConfig(ctx)`.
  - Builds a frame system with `referenceframe.NewFrameSystem("t", cfg.Parts, nil)`.
  - Calls `referenceframe.FrameSystemGeometries(fsys, referenceframe.NewZeroInputs(fsys))`, and for each frame prints its name, its DoF count, and each geometry's label and Go type.
  - Prints `pick-arm`'s `Kinematics()` → `Geometries(currentJoints)`, with labels and types.
- [ ] **Step 3.** Record the findings in a new `docs/results/2026-11-qp-collision.md`, under "Task 0":
  - which frames have geometry (expect the table's box), and whether the bin has any;
  - whether `pick-gripper` has geometry, its DoF, and so whether it would be skipped;
  - the live link geometry types.

  If any live link is a mesh, stop and report.
- [ ] **Step 4.** Write `dev/panda-collision-ws.json`, the evidence `world_state`, in motion-API protojson.
  - **Bin:** a box obstacle in `world` covering the bin. The config has the bin at x 450, y −350, with the mesh's footprint; use a 300 × 200 × 150 mm box centred at (450, −350, 75), and adjust if Step 2 shows otherwise. Label it `bin`.
  - **Gripper:** if Step 2 showed the gripper skipped or without geometry, add a box transform named `pick-gripper-box`, parented to `pick-arm`, about 90 × 140 × 160 mm, centred 80 mm along the tool axis.

  Example shape:
  ```json
  {"obstacles": [{"reference_frame": "world", "geometries": [{"center": {"x": 450, "y": -350, "z": 75, "o_z": 1}, "box": {"dims_mm": {"x": 300, "y": 200, "z": 150}}, "label": "bin"}]}],
   "transforms": [{"reference_frame": "pick-gripper-box", "pose_in_observer_frame": {"reference_frame": "pick-arm", "pose": {"z": 80, "o_z": 1}}, "physical_object": {"center": {"o_z": 1}, "box": {"dims_mm": {"x": 90, "y": 140, "z": 160}}, "label": "pick-gripper-box"}}]}
  ```
- [ ] **Step 5.** Stop the cell, then commit the results doc and the JSON with the message "Collision Task 0: what the Panda cell provides".

---

### Task 2: the maths (`collision_math.go`)

**Files:** create `collision_math.go` and `collision_math_test.go`.

- [ ] **Step 1: Write the failing tests.**
  ```go
  package diffik

  import (
  	"math"
  	"math/rand"
  	"testing"
  )

  // spec "Each tick" 5: v_max is brakeCap's rule with c = A_d. An approach obeying it, slowed by A_d per
  // tick, stops within the gap, and the next tick's v_max is at least the approach minus A_d.
  func TestCollisionVmaxStops(t *testing.T) {
  	r := rand.New(rand.NewSource(4))
  	for k := 0; k < 100000; k++ {
  		Ad := []float64{0.01, 0.3, 2, 9}[k%4]
  		g := r.Float64() * 80
  		v := r.Float64() * collisionVmax(Ad, g)
  		if k%3 == 0 {
  			v = collisionVmax(Ad, g)
  		}
  		if next := collisionVmax(Ad, g-v); next < v-Ad-1e-12 {
  			t.Fatalf("Ad %v g %v v %v: next v_max %v below v−Ad", Ad, g, v, next)
  		}
  		travel := 0.0
  		for s := v; s > 0; s -= Ad {
  			travel += s
  		}
  		if travel > g+1e-9 {
  			t.Fatalf("Ad %v g %v v %v: travels %v", Ad, g, v, travel)
  		}
  	}
  	if collisionVmax(1, 0) != 0 || collisionVmax(1, -3) != 0 {
  		t.Fatal("no gap: v_max must be 0")
  	}
  }

  // spec "Failures": the relaxed bound is the strongest move away the box allows.
  func TestRelaxBound(t *testing.T) {
  	jd := []float64{2, -1, 0}
  	lb, ub := []float64{-1, -1, -1}, []float64{0.5, 3, 1}
  	if got := relaxBound(jd, lb, ub); math.Abs(got-(2*0.5+1)) > 1e-15 { // j0 at ub, j1 at lb
  		t.Fatalf("relaxBound = %v, want 2", got)
  	}
  }

  // spec "Failures": the brake moves lastStep toward zero by at most a·dt², clipped to position.
  func TestBrakeStep(t *testing.T) {
  	dt := 0.02
  	a := []float64{deg2rad(60), deg2rad(60)}
  	c := a[0] * dt * dt
  	q, lo, hi := []float64{0, 1}, []float64{-3, -3}, []float64{3, 1.0001}
  	got := brakeStep([]float64{10 * c, 0.5 * c}, a, dt, q, lo, hi)
  	if math.Abs(got[0]-9*c) > 1e-15 {
  		t.Fatalf("joint 0 step %v, want %v", got[0], 9*c)
  	}
  	if got[1] != 0 { // 0.5c brakes to rest in one tick
  		t.Fatalf("joint 1 step %v, want 0", got[1])
  	}
  	got = brakeStep([]float64{0, 10 * c}, a, dt, q, lo, hi)
  	if got[1] > hi[1]-q[1]+1e-15 { // clipped to the 0.0001 rad left
  		t.Fatalf("joint 1 step %v passes hi", got[1])
  	}
  }
  ```
- [ ] **Step 2.** Run `go test -run 'TestCollisionVmax|TestRelaxBound|TestBrakeStep' ./`. Expected: a build failure (undefined).
- [ ] **Step 3: Implement it.**
  ```go
  package diffik

  import "math"

  // collisionVmax is the fastest approach (mm per tick) toward an obstacle gap g mm away from which slowing
  // by Ad mm per tick² each tick stops within the gap: brakeCap's discrete rule with c = Ad (spec "Each tick").
  func collisionVmax(Ad, g float64) float64 { return brakeCap(Ad, math.Max(g, 0), 1) }

  // relaxBound is the largest jd·Δq over lb ≤ Δq ≤ ub: the strongest move away from an obstacle the step
  // bounds allow (spec "Failures").
  func relaxBound(jd, lb, ub []float64) float64 {
  	s := 0.0
  	for j := range jd {
  		s += math.Max(jd[j]*lb[j], jd[j]*ub[j])
  	}
  	return s
  }

  // brakeStep is the step that moves lastStep toward zero by at most a·dt² per joint (a in rad/s²), clipped
  // to the position limits: no new motion, within the acceleration limit (spec "Failures").
  func brakeStep(last, a []float64, dt float64, q, lo, hi []float64) []float64 {
  	out := make([]float64, len(last))
  	for i, s := range last {
  		c := a[i] * dt * dt
  		out[i] = math.Min(math.Max(s-math.Min(math.Max(s, -c), c), lo[i]-q[i]), hi[i]-q[i])
  	}
  	return out
  }
  ```
- [ ] **Step 4.** Run `go test ./...`. Expected: PASS.
- [ ] **Step 5.** Commit with "Collision maths: damper speed, relax bound, brake step".

---

### Task 3: input parsing (`collision_parse.go`)

**Files:** create `collision_parse.go` and `collision_parse_test.go`.

- [ ] **Step 1: Write the failing tests.**
  - `ParseWorldState` of the bin JSON from Task 1 (inline a copy in the test) gives one obstacle with label `bin`.
  - An unknown field (`{"obstacle": []}`) is an error, because decoding is strict.
  - `ParseCollisionSpecs` of `[{"allows":[{"frame1":"a","frame2":"b"}]},{"allows":[{"frame1":"c","frame2":"d"}]}]` gives `[][2]string{{"a","b"},{"c","d"}}`.
  - An entry with an empty `frame1` is an error.
- [ ] **Step 2.** Run the tests and see them fail.
- [ ] **Step 3: Implement it.**
  ```go
  package diffik

  import (
  	"encoding/json"
  	"fmt"

  	commonpb "go.viam.com/api/common/v1"
  	motionpb "go.viam.com/api/service/motion/v1"
  	"go.viam.com/rdk/referenceframe"
  	"google.golang.org/protobuf/encoding/protojson"
  )

  // ParseWorldState decodes a world_state value (a JSON object, or its bytes) in the motion service API's
  // protojson shape (spec "Inputs"). Decoding is strict: unknown fields are errors.
  func ParseWorldState(v interface{}) (*referenceframe.WorldState, error) {
  	b, err := jsonBytes(v)
  	if err != nil {
  		return nil, fmt.Errorf("world_state: %w", err)
  	}
  	var pb commonpb.WorldState
  	if err := protojson.Unmarshal(b, &pb); err != nil {
  		return nil, fmt.Errorf("world_state: %w", err)
  	}
  	ws, err := referenceframe.WorldStateFromProtobuf(&pb)
  	if err != nil {
  		return nil, fmt.Errorf("world_state: %w", err)
  	}
  	return ws, nil
  }

  // ParseCollisionSpecs decodes a collision_specification list (motion.v1.CollisionSpecification each) into
  // allowed (frame1, frame2) pairs.
  func ParseCollisionSpecs(v interface{}) ([][2]string, error) {
  	items, ok := v.([]interface{})
  	if !ok {
  		return nil, fmt.Errorf("collision_specification must be a list, got %T", v)
  	}
  	var out [][2]string
  	for i, it := range items {
  		b, err := jsonBytes(it)
  		if err != nil {
  			return nil, fmt.Errorf("collision_specification[%d]: %w", i, err)
  		}
  		var pb motionpb.CollisionSpecification
  		if err := protojson.Unmarshal(b, &pb); err != nil {
  			return nil, fmt.Errorf("collision_specification[%d]: %w", i, err)
  		}
  		for _, a := range pb.GetAllows() {
  			if a.GetFrame1() == "" || a.GetFrame2() == "" {
  				return nil, fmt.Errorf("collision_specification[%d]: allows needs frame1 and frame2", i)
  			}
  			out = append(out, [2]string{a.GetFrame1(), a.GetFrame2()})
  		}
  	}
  	return out, nil
  }

  func jsonBytes(v interface{}) ([]byte, error) {
  	if b, ok := v.([]byte); ok {
  		return b, nil
  	}
  	return json.Marshal(v)
  }
  ```
  Check the import paths against `go.mod`'s `go.viam.com/api` version (`go doc go.viam.com/api/service/motion/v1 CollisionSpecification`). `teleop.go` already imports `motionpb`.
- [ ] **Step 4.** Run `go test ./...`. Expected: PASS.
- [ ] **Step 5.** Commit with "Collision inputs: world_state and collision_specification parsing".

---

### Task 4: the collision model (`collision.go`) and per-tick rows (`collision_tick.go`)

**Files:** create `collision.go`, `collision_tick.go` and `collision_test.go`.

This is the core. Read the spec's sections "The collision model, built at `servo_start`" and "Each tick" closely.

- [ ] **Step 1: The types and the builder.** Add to `collision.go`:
  ```go
  package diffik

  // CollisionRequest is a session's collision input (spec "Inputs"). Zero DMinMM/DActMM take the defaults.
  type CollisionRequest struct {
  	WorldState *referenceframe.WorldState // may be nil
  	Allows     [][2]string
  	DMinMM     float64 // default 10
  	DActMM     float64 // default 100
  }

  type collPart struct {
  	names []string             // geometry label(s), owner frame name, <frame>_origin
  	link  string               // arm link: the model's raw geometry label, re-posed with Geometries(q)
  	geom  spatialmath.Geometry // mounted part: the geometry in the arm's end-effector frame
  }

  type collObstacle struct {
  	names []string
  	geom  spatialmath.Geometry // in the arm's base frame (<arm>_origin), fixed
  }

  type collPair struct {
  	part, obs int
  	suspended bool // closer than d_min at start; re-armed beyond d_act (spec "Pairs")
  }

  // collisionModel is a session's robot parts, obstacles and checked pairs (spec "The collision model").
  type collisionModel struct {
  	model      referenceframe.Model
  	parts      []collPart
  	obs        []collObstacle
  	pairs      []collPair
  	dMin, dAct float64
  	skipped    []string // movable frames left out
  }
  ```

  `NewCollisionModel(parts []*referenceframe.FrameSystemPart, model referenceframe.Model, armName string, req CollisionRequest, q []float64) (*collisionModel, error)` does the following, in order. It takes the frame-system **parts** (from `FrameSystemConfig().Parts`) and builds its own frame system, so the `world_state` transforms are added the way the motion service adds them. Add a `warnings []string` field and an exported `func (c *collisionModel) Warnings() []string`, which the session and the CLI use. Returning `*collisionModel` from an exported function compiles fine.
  1. **Defaults and checks.** Apply the defaults (10, 100). Reject `DMinMM <= 0` and `DActMM <= DMinMM`.
  2. **Frame system with the `world_state` transforms.** `fsys, err := referenceframe.NewFrameSystem("collision", parts, transforms)`, where `transforms` is `req.WorldState.Transforms()` (nil if there's no world state). This is how the motion service adds them: each transform's geometry goes on a tail-geometry `X_origin` frame after its pose. **Don't use `ToStaticFrame` + `AddFrame`**: a plain static frame puts its geometry at the frame's origin, which loses the transform's pose (review probe: the gripper box ended up at z 0 instead of 80 mm). Check that `fsys.Frame(armName)` exists and has `len(q)` DoF.
  3. **Frame geometries.** `zero := referenceframe.NewZeroInputs(fsys)`, `lin := zero.ToLinearInputs()`, and `geoms, err := referenceframe.FrameSystemGeometries(fsys, zero)`. Go through the frame names in sorted order (so the result is deterministic), skipping `armName` and `armName+"_origin"`, whose geometries come from the model. For each frame `name` with geometries:
     - **Classify it** by walking parents from `fsys.Frame(name)` up to `world`:
       - if any frame on the way, including `name` itself, has `len(DoF()) > 0` before reaching `armName`, it's **movable**: add `name` to `skipped` and continue;
       - reaching `armName` means it's **mounted**; reaching `world` without passing `armName` means it's an **obstacle**.
     - **Reject unsupported shapes.** `cfg, err := spatialmath.NewGeometryConfig(g)`, and accept only `cfg.Type` of `spatialmath.BoxType`, `SphereType`, `CapsuleType` or `PointType`. For **frame-system** geometries, any other type (meshes, cylinders) is **skipped**: add `"<label> (<type>)"` to `skipped` and a warning, then continue (spec "Geometry", changed after Task 0 found the Panda cell's bin and gripper bodies are frame-system meshes). For `world_state` obstacles and arm links, any other type is an error: `fmt.Errorf("collision: %s is a %s; only boxes, capsules, spheres and points are supported", label, cfg.Type)`. Don't use `ToProtobuf`: `(*Cylinder).ToProtobuf` panics in v1.16, and the concrete shape types are unexported, so a type switch can't be written. That covers meshes, and also cylinders, whose `DistanceFrom` goes through `ToMesh()` and is unsigned too. Use the same check for links and `world_state` obstacles.
     - **Names.** Use `names := nonEmpty(g.Label(), name, trimOrigin(name), trimOrigin(name)+"_origin")`, de-duplicated.
     - **`FrameSystemGeometries` returns every geometry already posed in `world`,** with all inputs at zero, including the arm. So wrap it as `World` and transform from there, never from `name`. Wrapping it as `name` applies the frame's pose a second time: in the review's probes a mounted box landed at (88, 0, 876) instead of (0, 0, 80), and an obstacle under a parent at x 500 landed at x 1100 instead of 600.
     - **Mounted:** `tf, err := fsys.Transform(lin, referenceframe.NewGeometriesInFrame(referenceframe.World, []spatialmath.Geometry{g}), armName)` gives the geometry in the end-effector frame. The part is `collPart{names, "", tf.(*referenceframe.GeometriesInFrame).Geometries()[0]}`, and each tick `geom.Transform(FK(q))` places it. `Geometry.Transform` pre-multiplies: `Compose(pose, g.centre)`.
     - **Obstacle:** `tf, err := fsys.Transform(lin, referenceframe.NewGeometriesInFrame(referenceframe.World, []spatialmath.Geometry{g}), armName+"_origin")`, then the obstacle is `collObstacle{names, tf.(*referenceframe.GeometriesInFrame).Geometries()[0]}`.
     - **Naming:** static geometry lives on `X_origin` frames, and its label is replaced by the frame name, so `trimOrigin` gives the component name.
     - **Parent-walk limit:** `fsys.Parent` hides the model's internal link frames, so a config frame parented to `arm:linkN` is treated as mounted at the end effector. Add a `// ponytail:` comment saying so.
     - **Arm's own frame geometry:** geometry configured on the arm's own frame (`armName`/`armName_origin`, for example a pedestal) is skipped here, because links come from the model. Note that in a comment too.
  4. **`world_state` obstacles:** for each `gif` in `req.WorldState.Obstacles()` (not `ObstaclesInWorldFrame`; see the spec), transform it the same way to `armName+"_origin"`. Names are the label, `gif.Parent()` and `gif.Parent()+"_origin"`. Reject meshes.
  5. **Links:** `lg, err := model.Geometries(q)`. For each `g`, the raw label is `g.Label()`, and the normalized label is `armName + ":" + afterColon(raw)`. Names: `[normalized, raw, armName, armName+"_origin"]`. The part is `collPart{names, raw, nil}`. Reject meshes.
  6. **Allows:** for each `[a, b]` pair, every name must match some part's or obstacle's names, or be in `skipped`.
     - A skipped name is a no-op. Record it, and the caller logs a warning (return the list through a `warnings []string` field).
     - A name that matches nothing returns `fmt.Errorf("collision_specification: %q matches no robot part or obstacle", n)`.
     - A (part, obstacle) pair is allowed when (a ∈ part.names and b ∈ obs.names) or (b ∈ part.names and a ∈ obs.names), for any entry.
  7. **Pairs:** every non-allowed (part, obstacle). Measure all of them once at `q` (with `poseParts`, Step 2), and mark `suspended` if d < dMin.
  8. **Return the model.** With no pairs, it is "off" (`len(c.pairs) == 0`).

- [ ] **Step 2: Per tick, in `collision_tick.go`.**
  ```go
  // poseParts returns every part's geometry in the arm's base frame at joints q (rad).
  func (c *collisionModel) poseParts(q []float64) ([]spatialmath.Geometry, error) {
  	lg, err := c.model.Geometries(q)
  	if err != nil {
  		return nil, err
  	}
  	byLabel := map[string]spatialmath.Geometry{}
  	for _, g := range lg.Geometries() {
  		byLabel[g.Label()] = g
  	}
  	ee, err := c.model.Transform(q) // mm
  	if err != nil {
  		return nil, err
  	}
  	out := make([]spatialmath.Geometry, len(c.parts))
  	for i, p := range c.parts {
  		if p.link != "" {
  			g, ok := byLabel[p.link]
  			if !ok {
  				return nil, fmt.Errorf("link geometry %q missing", p.link)
  			}
  			out[i] = g
  			continue
  		}
  		out[i] = p.geom.Transform(ee)
  	}
  	return out, nil
  }

  // collRow is one damper row J_d·Δq ≥ −vmax (mm per tick), for pair index pair at distance d (mm).
  type collRow struct {
  	jd         []float64 // mm/rad
  	vmax, d    float64
  	pair       int
  }

  const maxCollRows = 16

  // rows measures every pair at q, re-arms suspended pairs beyond d_act, and returns a damper row per active
  // pair (closest maxCollRows), plus the closest unsuspended pair (spec "Each tick"). accel is rad/s² per joint.
  func (c *collisionModel) rows(q, accel []float64, dt float64) (rows []collRow, minD float64, minPair int, err error)
  ```
  The body of `rows`:
  1. Pose the parts at `q`. For each pair, `d, err := posed[p.part].DistanceFrom(c.obs[p.obs].geom)`.
     - **Suspended:** if `d > c.dAct`, clear `suspended`; either way, `continue`.
     - **Closest:** track the minimum d and its index (`minD` starts at +Inf, and `minPair` at −1).
     - **Active:** if `d < c.dAct`, collect `(d, idx)`.
  2. Sort the active pairs by d and keep `maxCollRows`. If none are left, return.
  3. **Gradient,** for each joint j: build `qp` and `qm` as copies of q with `±fdStep` on j, pose both, and for each active pair set `jd[j] = (dp − dm) / (2·fdStep)`.
  4. **The row:** `Ad := Σ_j |jd[j]|·accel[j]·dt²` and `vmax := collisionVmax(Ad, d − c.dMin)`.

  Also add, for status, `func (c *collisionModel) pairName(i int) string`, returning `"<part name 0>↔<obstacle name 0>"`, and `func (c *collisionModel) ignored() []string`, listing the suspended pairs' names.
- [ ] **Step 3: Write the tests** in `collision_test.go`. Build frame systems by hand. The fixture pattern:
  ```go
  // collFS is a world with arm (model m) at the origin plus extra static parts.
  func collFS(t *testing.T, m referenceframe.Model, extra ...*referenceframe.FrameSystemPart) *referenceframe.FrameSystem {
  	t.Helper()
  	parts := append([]*referenceframe.FrameSystemPart{
  		{FrameConfig: referenceframe.NewLinkInFrame(referenceframe.World, spatialmath.NewZeroPose(), "arm", nil), ModelFrame: m},
  	}, extra...)
  	fsys, err := referenceframe.NewFrameSystem("t", parts, nil)
  	if err != nil {
  		t.Fatal(err)
  	}
  	return fsys
  }
  ```
  Static parts use `referenceframe.NewLinkInFrame(parent, pose, name, geometry)`, with geometry from `spatialmath.NewBox(pose, r3.Vector{...}, label)`, `NewCapsule` or `NewSphere`. Use `panda(t)`; its model name is `arm`, so the links are labelled `arm:<link>`. Take `pandaHold` from `loop_test.go`.

  The tests:
  - **Gradient:** for each obstacle shape (box, capsule, sphere) placed 40 mm below the flange at `pandaHold`, check `rows()`'s `jd` against a **central** difference at 1e-4, `(d(q + 1e-4·e_j) − d(q − 1e-4·e_j)) / 2e-4`. It must agree to 1e-3 relative on every joint where |jd| > 1.
    - **Tilt the box obstacle** (orientation vector (0.15, 0.1, 1), θ 17°). A flat box is face-parallel to the link boxes, where the distance isn't differentiable: vertex ties gave relative errors up to 8 in the review's probe.
  - **Allows:**
    - `{"arm","table"}` gives no pairs that involve any link with the table.
    - `{"arm:link7","table"}` removes only link 7's pair.
    - `{"nope","table"}` errors.
    - A skipped (movable) frame's name is accepted as a no-op.
  - **Mesh skipped or rejected:** a static frame with a mesh geometry is skipped, listed in `skipped`, and gives a warning. A `world_state` obstacle with a mesh makes the build error. Build the mesh with `spatialmath.NewMesh`, or skip the case with a `t.Skip` reason if the RDK has no easy constructor, and say so in the report.
  - **Movable skipped:** a second model frame with DoF > 0 holding a geometry appears in `skipped` and gives no obstacle.
  - **Mounted part:** a static box frame parented to `arm` becomes a part, not an obstacle. Its posed geometry follows the flange as q changes.
  - **Suspend and re-arm:**
    - an obstacle overlapping link 0 at start leaves its pair suspended and listed in `ignored()`;
    - moving the arm can't re-arm link 0 (it's the base), so use a pair on the flange instead: start the flange 5 mm from a box, move q so d > d_act, call `rows()`, and check the pair is no longer suspended.
  - **Activation and the cap:**
    - a pair at d > d_act gives no row;
    - with more than 16 pairs within d_act (for example 20 small spheres around the flange), `rows()` returns 16, the closest ones.
  - **Cost** (spec Task 0 item 4): `TestCollisionRowsTime` uses the Panda fixture, 5 box obstacles within d_act of several links, and the 7-joint gradient. Over 1000 calls, the p99 must be under 5 ms. Log p50 and p99, and skip under `-short`.
- [ ] **Step 4.** Run `go test ./...`. Expected: PASS. Log the cost numbers in the report.
- [ ] **Step 5.** Commit with "Collision model and per-tick damper rows".

---

### Task 5: into the loop

**Files:** modify `qpbounds.go`, `loop.go` and `loop_test.go` (or create `collision_loop_test.go`).

- [ ] **Step 1: `qpNext` takes collision rows.** Change its signature to `qpNext(rows taskRows, scale, dt float64, coll []collRow, relax bool)`.
  - After `lb, ub := l.qpBounds(dt)`, when `len(coll) > 0`, build `A` (`len(coll)×n`) and `b`. Row k is `A[k] = coll[k].jd`.
  - `b[k] = −coll[k].vmax`. When `relax` is set, it's `math.Min(−coll[k].vmax, relaxBound(coll[k].jd, lb, ub)) − relaxEps`, with `const relaxEps = 1e-6 // mm per tick`. At exactly M the feasible set is a single box corner, and the solver reports it infeasible (review probe), so the slack is needed. Units: `jd` is mm/rad and Δq is rad per tick, so `A·Δq` and `b` are both mm per tick.
  - Pass `A` and `b` in `qpProblem`, and update every caller. The existing callers pass `nil, false`.
- [ ] **Step 2: Hook the model into the loop.**
  - Add `coll *collisionModel` to `Loop`, and:
    ```go
    // SetCollision turns on collision avoidance (solver qp only). Call it before Run.
    func (l *Loop) SetCollision(c *collisionModel) error {
    	if l.cfg.Solver != SolverQP {
    		return errors.New(`collision avoidance needs solver "qp"`)
    	}
    	l.coll = c
    	return nil
    }
    ```
  - Add to `Status`:
    ```go
    	Collision                            string   // "", "on" or "off" (requested, but no pairs)
    	CollisionMinMM                       float64  // closest unsuspended pair this tick; +Inf if none
    	CollisionPair                        string
    	CollisionRows, CollisionRelaxed      int
    	CollisionBrakes                      int
    	CollisionIgnored, CollisionSkipped   []string
    	CollisionLate                        string   // last "approach faster than v_max" warning (spec "Each tick" 6)
    ```
    Clone the slices in `Status()`.
- [ ] **Step 3: The tick.** In `tick`'s QP branch, replace the single `qpNext` call with this:
  ```go
  	var crows []collRow
  	if l.coll != nil && len(l.coll.pairs) > 0 {
  		accel := make([]float64, len(l.qCmd))
  		for i := range accel {
  			accel[i] = deg2rad(l.cfg.MaxJointDegPerS2[i])
  		}
  		var cerr error
  		crows, cmin, cpair, cerr = l.coll.rows(l.qCmd, accel, dt)
  		if cerr != nil {
  			return fmt.Errorf("collision: %w", cerr)
  		}
  		for _, r := range crows { // spec "Each tick" 6: entered faster than it can stop
  			if v := dotf(r.jd, l.lastStep); -v > r.vmax+1e-9 {
  				late = fmt.Sprintf("%s: approach %.3g mm/tick > v_max %.3g", l.coll.pairName(r.pair), -v, r.vmax)
  			}
  		}
  	}
  	next, qpActive, qpIters, qerr = l.qpNext(rows, scale, dt, crows, false)
  	if qerr != nil && len(crows) > 0 {
  		relaxed = true
  		next, qpActive, qpIters, qerr = l.qpNext(rows, scale, dt, crows, true)
  	}
  	if qerr != nil && len(crows) > 0 { // brake: DLS ignores obstacles (spec "Failures")
  		accel := ...same as above...
  		next = make([]float64, len(l.qCmd))
  		bs := brakeStep(l.lastStep, accel, dt, l.qCmd, l.lo, l.hi)
  		for i := range next {
  			next[i] = l.qCmd[i] + bs[i]
  		}
  		braked, fellBack, limStep = true, true, true
  		if maxAbs(l.lastStep) == 0 { // at rest Δq = 0 is feasible: a real fault
  			if l.qpFails++; l.qpFails >= 10 {
  				return fmt.Errorf("qp: %w", qerr)
  			}
  		}
  	} else if qerr != nil {
  		// the existing DLS fallback path, unchanged
  	} else {
  		l.qpFails, limStep = 0, qpActive > 0
  	}
  ```
  - Add `dotf` and `maxAbs` helpers if they don't exist (`maxAbsDiff` does exist).
  - In the status block, set:
    - `Collision`: `"on"` when `l.coll != nil` and it has pairs, `"off"` when it has none, `""` when there's no model;
    - `CollisionMinMM`, `CollisionPair` (via `pairName`) and `CollisionRows`;
    - `CollisionRelaxed++` and `CollisionBrakes++` on those ticks (and `QPFallbacks++` for brakes, which `fellBack` already does);
    - `CollisionIgnored` from `l.coll.ignored()`, and `CollisionSkipped` from `l.coll.skipped`;
    - `CollisionLate` when `late != ""`.
  - Keep the existing order: posture, bounds, the non-finite check, then the guard.
- [ ] **Step 4: The tests.**
  Each test uses the Panda fixture on `simIO`, `Solver: SolverQP`, 30°/s and 60°/s², and a model from `NewCollisionModel` with a `collFS`. Move the shared fixture helper into a non-test-specific test file, or keep it in `collision_test.go`; it's in the same package.
  - **Approach:**
    - Box obstacle: 300 × 300 × 100 mm, its top 60 mm below the flange at `pandaHold`.
    - Hold, then set the target 110 mm straight down, which puts the target 50 mm inside the box.
    - The closest part is the `attachment_site` box, which reaches about 8.7 mm below the flange, not the flange itself.
    - Run 400 ticks. Every tick: `CollisionMinMM ≥ d_min − 1` and `PeakAccelRatio ≤ 1 + 1e-9`.
    - The final state is `StateStalled`, and `CollisionBrakes == 0`.
  - **Fast approach:**
    - Same box; `linear_mm_s` 100, with joints at 90°/s and 360°/s².
    - The target is 200 mm down, reached by `SetVelocity` at 100 mm/s.
    - `CollisionMinMM` stays ≥ `d_min − 1`, with no brakes.
  - **Relaxed rows still bind:**
    - A white-box test with **one** active row. Use a small sphere obstacle (radius 10 mm) under the attachment site, so only that part is within `d_act`; check `len(rows) == 1`. With a flat box, link6 and link7 are also in range, and three relaxed rows conflict and stay infeasible at any slack.
    - Place the sphere so the attachment site's distance is 20 mm.
    - Set `lastStep` to the step that approaches at 3× `v_max`.
    - Call `qpNext` with `relax=true`. With `M := relaxBound(jd, lb, ub)`, check `M < −v_max` (the precondition), `J_d·Δq ≥ M − 2·relaxEps`, and `|J_d·Δq − M| ≤ 1e-5`.
  - **Brake:**
    - Force `qpSolver` to fail. While the arm is moving toward the box (`lastStep` ≠ 0, set by a few normal ticks first), the ticks brake: each joint's |step| shrinks by `min(|s|, a·dt²)`, and there's no fault even after 20 ticks of braking.
    - Once `lastStep` is 0, 10 more failures fault with `qp:`.
  - **Sliding** (spec "Tests"; this is the reviewer's prototype, which passed):
    - **The box:** tilted (as in the gradient test), 400 × 300 × 100 mm, its top 90 mm below the flange at `pandaHold`, centred 100 mm ahead in +x. Check that no pair is suspended at start (`len(ignored()) == 0`); the box must be clear of `d_min` at the start pose.
    - **The path:** move the target +150 mm in x and −120 mm in z over 400 ticks, a diagonal approach onto the box's top face. A horizontal path into the box's side just stalls on that face, because nothing lifts the arm over it.
    - **The "only stopping" reference:** the same run, but the target is frozen as soon as `CollisionMinMM ≤ d_min + 0.5`.
    - **Bar:** the mean TCP error over the 400 ticks with sliding is smaller than with stopping (prototype: 33.8 against 58.0 mm), and the minimum distance is ≥ `d_min − 1` (prototype: 9.94).
  - **No change when off:** (the spec asks for a golden scenario; comparing two runs directly is equivalent. Note it in the results.) a QP run on the SO-101 static scenario, as `qp_so101_static` in `golden_test.go` does, gives bit-identical sends with no model and with a model that has zero pairs (every pair allowed). Compare the two runs directly.
  - **Re-arming in the loop:** start with the flange 5 mm above a box (suspended), drive up 150 mm (beyond `d_act`), then back down. The pair now constrains: the minimum distance on the way down is ≥ `d_min − 1`.
  - **SetCollision with `dls`** errors.
- **Capsule obstacles:** don't use them in near-contact tests. The RDK's capsule↔box distance switches from the mesh path to SAT near contact and can jump (review measured 14.18 mm to −0.05 mm in one 0.5 mm step). The gradient test's capsule, at about 33 mm, is fine. Task 8's docs note this as a known limit.
- [ ] **Step 5.** Run `go test ./...`. Expected: PASS, and `TestGoldenUR5ePoseCommands` unchanged.
- [ ] **Step 6.** Commit with "Loop: collision rows, relax, brake and status".

---

### Task 6: session and module

**Files:** modify `session.go`, `module.go` and `module_test.go` (and `session_test.go` if needed).

- [ ] **Step 1: Session.**
  - Add `Collision *CollisionRequest` to `SessionConfig`.
  - When `cfg.Collision != nil`, read `m.fs.FrameSystemConfig(ctx)` and pass its `Parts` to `NewCollisionModel`, which builds its own frame system. (If `cfg.Frames` is also set, reuse one `FrameSystemConfig` call for both.) Then:
    ```go
    cm, err := NewCollisionModel(cfgFS.Parts, model, armName, *cfg.Collision, q)
    if err != nil {
    	return nil, nil, err
    }
    if err := l.SetCollision(cm); err != nil {
    	return nil, nil, err
    }
    for _, w := range cm.Warnings() {
    	m.logger.Warnw("collision", "warning", w)
    }
    ```
  - Log `Status.CollisionLate` from `session.onTick` when it changes (`logger.Warnw`).
- [ ] **Step 2: Module.** In `startSession`, after the solver and posture parsing:
  - **`collision`:** `true`, `false`, or an object with `d_min_mm` and `d_act_mm` (numbers > 0). Unknown keys are rejected with `unknown collision key %q`.
  - **`world_state`:** `ParseWorldState(v)`.
  - **`collision_specification`:** `ParseCollisionSpecs(v)`.
  - **When it's on:** collision is on when `collision` is `true` or an object, or when `world_state` or `collision_specification` is given and `collision` isn't `false`.
  - **Solver check:** if any of the three keys is given and the solver isn't `qp`, return ``fmt.Errorf(`world_state, collision_specification and collision need "solver": "qp"`)``. The exception is `"collision": false` on its own.
  - **On:** set `SessionConfig.Collision = &CollisionRequest{ws, allows, dMin, dAct}`.

  The teleop defaults block reaches this through `startSession` as it already does.
- [ ] **Step 3: `statusMap`.** When `st.Collision != ""`, add:
  - `collision`;
  - `collision_min_mm` (omit it when +Inf);
  - `collision_pair`, `collision_rows`, `collision_relaxed` and `collision_brakes`;
  - `collision_ignored` and `collision_skipped` as lists, which may be empty.
- [ ] **Step 4: Tests**, following the existing module tests' style. `teleopFS` in `teleop_test.go` shows how to serve a real frame system to the service. Add a static `table` box part to such a fake.
  - **Error cases:**
    - `world_state` with `dls`;
    - a bad `world_state` (an unknown field);
    - a bad `collision_specification`;
    - an `allows` name that matches nothing;
    - `collision: {"d_min_mm": 0}`;
    - `d_act_mm` ≤ `d_min_mm`;
    - an unknown `collision` key.
  - **Status:** a QP session with `"collision": true` against a fake frame system with a table box reports `collision: "on"` and a finite `collision_min_mm`.
  - **Default off:** a QP session without these keys reports no `collision` key, and makes no extra `FrameSystemConfig` call beyond what `frames` makes. Check `fakeFS.cfgCalls`.
- [ ] **Step 5.** Run `go test ./...`. Expected: PASS.
- [ ] **Step 6.** Commit with "Module: world_state, collision_specification and collision on servo_start".

---

### Task 7: the CLI

**Files:** modify `cmd/cli/main.go` and `cmd/cli/run.go`, and add a test in `cmd/cli`.

- [ ] **Step 1: The flags.**
  - `--world-state <file.json>`: read and parse with `diffik.ParseWorldState(bytes)`.
  - `--allow frame1:frame2`: repeatable, using a `flag.Func` that appends.
  - `--collision`: turns it on without a world state.
  - `--collision-off`.
  - `--d-min-mm` and `--d-act-mm`: 0 means the default.
  - `--descend-mm` (default 100) and `--descend-mm-s` (default 20).

  Validate before the arm moves. Collision flags need `--solver qp`, and `--allow` needs collision on. Collision is on with `--collision`, or with `--world-state` unless `--collision-off` is given.
- [ ] **Step 2: Wiring, in `run`.** After `NewLoop`, when collision is on:
  - read `machine.FrameSystemConfig(ctx)` in `main` before the start-joints move, and pass its `Parts` in through `runOpts`, since `run` doesn't get the machine;
  - `cm, err := diffik.NewCollisionModel(parts, model, armName, req, q)` and `l.SetCollision(cm)`, printing `cm.Warnings()`.
- [ ] **Step 3: `run descend`.** From the start pose, move the target down (−z in the base frame) at `--descend-mm-s` until `--descend-mm`, with `SetTarget` each tick as `circle` does, then hold for 2 s. Print:
  - `descend: final err %.2f mm` (the TCP error to the final target);
  - `collision: min %.2f mm (%s)  brakes %d  relaxed %d  late: %s`, from the minimum `CollisionMinMM` over the rows, its pair, and the final counters.
- [ ] **Step 4: The summary** prints the collision line for every scenario when collision is on.
- [ ] **Step 5: Tests.** A unit test for the descend target helper (a pure function, `descendTarget(start, mm)`), and one for the `--allow` parsing.
- [ ] **Step 6.** Run `go build ./... && go test ./... && go vet ./...`, and run `go run ./cmd/cli -h` to see the new flags.
- [ ] **Step 7.** Commit with "CLI: collision flags and run descend".

---

### Task 8: docs

- [ ] **Step 1: `devrel_diff-ik_servo.md`.**
  - Add `world_state`, `collision_specification` and `collision` to the `servo_start` table, using the spec's semantics: opt-in, qp only, meshes rejected, `allows` names, movable frames skipped, pairs suspended at start and re-armed.
  - Add the status fields.
  - Add a short "Collision avoidance" section covering:
    - the damper, in one formula and one sentence;
    - the relax, then brake, fallback;
    - the known limits: capsule↔box distances can jump near contact (RDK SAT), and sliding along a face can drift slightly below `d_min` (second order; the damper doesn't push back once inside);
    - the fault rule.
- [ ] **Step 2: `README.md`.** One paragraph in the Solvers section, linking to the spec and results.
- [ ] **Step 3: `dev/panda-mujoco.md`.** Add the collision evidence commands from Task 9, and `dev/panda-collision-ws.json`.
- [ ] **Step 4.** Commit with "Docs: collision avoidance".

---

### Task 9: sim evidence

The controller does this task. All runs use the Panda cell, `--solver qp --max-joint-deg-s 30 --max-joint-deg-s2 60`, and the start joints `0,0,0,-1.5708,0,1.5708,-0.785`.

- [ ] **Step 1: Descend through the table,** collision on and then off:
  ```
  go run ./cmd/cli --address localhost:8083 --insecure --arm pick-arm --solver qp --max-joint-deg-s 30 --max-joint-deg-s2 60 --start-joints 0,0,0,-1.5708,0,1.5708,-0.785 --world-state dev/panda-collision-ws.json --descend-mm 700 --descend-mm-s 30 run descend
  ```
  The start pose's TCP is about 625 mm up, so 700 mm goes through the table. Then run the same command with `--collision-off`. For the "off" run, measure the minimum part↔table distance from a separate logging pass, or take the sim's contact as evidence; report what you could measure.
  - Bar for on: min ≥ `d_min − 1` mm, 0 faults, and a peak acceleration ratio ≤ 1.
- [ ] **Step 2: Orbit past the bin:** `--orbit-deg-s 5 --orbit-deg 120 run orbit` with the world state. Pick a direction that sweeps the arm over the bin: the bin is at y < 0, so use a negative orbit rate if the CLI allows it; otherwise start the orbit pose rotated to +90° and orbit through −90°. Record what you used.
  - Bar: min part↔bin ≥ `d_min − 1` mm. Report the TCP error.
- [ ] **Step 3: Grasp approach:** `--allow pick-gripper:table` (or `pick-gripper-box:table`, per Task 1). The CLI's TCP is the arm flange, and the gripper extends 155.8 mm beyond it.
  - Descend until the gripper tip is 5 mm above the table top: `--descend-mm` = flange height − (table top + 155.8 + 5). The table top is z 20 in this cell, and the start pose's flange height comes from the CLI's start output. Print the computed value.
  - Bars: final TCP (flange) error to that target ≤ 2 mm, and every other part↔table pair ≥ `d_min − 1` mm. The CLI prints the minimum over non-allowed pairs.
- [ ] **Step 4: SO-101 tag sweep, no regression.**
  - `servo_client.py` needs a way to pass `collision`. Add `--collision` (a bare flag), which sends `"collision": true`. That's a small change; include a test-file update.
  - Run with `--solver qp --collision`, with the table as the frame-system obstacle (if the tag cell's table has frame geometry; Task 1 tells you the Panda cell's, so check the tag cell's the same way).
  - Bar: mean error at 20 mm/s within 10% of the earlier QP sweep (1.6 mm).
- [ ] **Step 5: Write up the results** in `docs/results/2026-11-qp-collision.md`: the evidence table, findings, `CollisionLate` warnings if any, and any `d_act` tuning. Commit with "Collision results: sim evidence".

---

### Task 10: finish

- [ ] **Step 1.** Run `go test -count=1 ./...`, `go test -race ./...`, `go vet ./...`, `gofmt -l .`, and all of `clients/python/test_*.py`.
- [ ] **Step 2.** Get a final Opus review of `git diff main...diff-ik-collision` against the spec, and fix what it finds.
- [ ] **Step 3.** Squash-merge to `main` with "Diff-IK collision avoidance on the QP solver" and the trailer. Keep the branch, and don't push.
- [ ] **Step 4.** Offer the user the watchable demo on mujoco-sim-mac: back up its config, load the Panda cell with the world state, run descend and grasp, then restore it.
