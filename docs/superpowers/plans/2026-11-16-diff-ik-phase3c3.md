# Diff-IK Phase 3c stage 3 (AprilTag visual servoing) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Servo the real SO-101 follower to an AprilTag seen by a wrist camera and by a scene camera, using camera-frame `servo_target`s transformed in the diff-ik module. Measure tracking (bus and jitter bars) and static physical accuracy.

**Architecture:**
- **Go module** (`diffik`):
  - the loop reports `ReachDeg`;
  - a new pure `targetFilter` (EMA, hold, floor clamp, feedforward switch);
  - sessions optionally build a local frame system at `servo_start` (`frames: true`) and transform camera-frame targets in-process with the loop's measured joints;
  - `servo_target` gains `frame` and `apply`, and richer replies.
- **Python:**
  - `teleop.py`'s session plumbing moves into a shared `servo_client.py` base;
  - `tag_servo.py` builds hover targets from `viam:apriltag` poses;
  - `intrinsics.py` calibrates webcams from ChArUco views;
  - `tag_lag.py` analyses runs.
- **Extrinsics:** `viam:camera-calibration`.

**Tech Stack:** Go (rdk v1.16.0: `referenceframe.NewFrameSystem`, `FrameSystem.Transform`, `spatialmath.Interpolate`); Python ≥ 3.11 via `uv run` (`viam-sdk`, `opencv-contrib-python`, `numpy`).

**Spec:** `docs/superpowers/specs/2026-11-16-diff-ik-phase3c3-design.md`. Read it first.

**Branch:** `diff-ik-phase3c3`.

**Commit trailer** (every commit):
```
Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_012FAJrCqoTNZgDVMfHN6sDY
```

**One addition to the spec:** `servo_target` accepts `"apply": false`. The module transforms the pose (with `frame`) and returns `target_raw` / `target` **without** filtering or applying it. The client needs the tag's arm-frame pose for `--axis down` and for the wrist camera's look-then-move, and only the module can compute it.

**Facts the code relies on** (rdk v1.16.0, verified):
- `framesystem.Service.FrameSystemConfig(ctx) (*framesystem.Config, error)` returns `Config{Parts []*referenceframe.FrameSystemPart}`.
- `referenceframe.NewFrameSystem(name, parts, nil) (*FrameSystem, error)`; `fs.Frame(name) Frame`; `fs.TracebackFrame(frame) ([]Frame, error)` (the frame up to world).
- `fs.Transform(inputs *LinearInputs, PoseInFrame, dst string) (Transformable, error)`; `referenceframe.NewZeroInputs(fs) FrameSystemInputs` (`map[string][]Input`); `.ToLinearInputs()`.
- Production frame names: the arm's model frame is the component name (`follower-arm`), and `<arm>_origin` is its static origin.
- `spatialmath.Interpolate(p1, p2, by) Pose`.
- In this repo: `RotVec(a, b quat.Number) r3.Vector` (kin.go); `deg2rad` (loop.go); `mmPerM` (kin.go); `toolAxis(q)` (loop.go); `parsePose` / `poseMap` (module.go).

---

## File structure

| File | Change | Responsibility |
|---|---|---|
| `loop.go` | modify | `Status.ReachDeg` |
| `filter.go`, `filter_test.go` | create | `TargetFilter` config and `targetFilter` state (EMA, hold, floor, feedforward) |
| `session.go` | modify | `SessionConfig.Frames` / `.Filter`; local frame system; `target(p, frame, apply)` |
| `module.go` | modify | parse `frames`, `target_filter`, `frame`, `apply`; reply and status fields |
| `session_test.go`, `module_test.go` | modify | the frame-system fake; frame routing, filter and reply tests |
| `clients/python/servo_client.py` | create | shared base: connections, sessions, watchdog, ordered exit, signals, CSV |
| `clients/python/teleop.py` | modify | subclass of the base (behaviour unchanged) |
| `clients/python/tag_servo.py`, `test_tag_servo.py` | create | AprilTag servo client and helper tests |
| `clients/python/intrinsics.py`, `test_intrinsics.py` | create | ChArUco intrinsic calibration (synthetic test) |
| `clients/python/tag_lag.py`, `test_tag_lag.py` | create | run analysis |
| `clients/python/frame_check.py` | create (Part B) | sim check of camera-frame targets and the floor clamp |
| `dev/so101-hw.md` | modify | stage-3 runbook |

---

## Part A — code (subagents)

### Task 1: `Status.ReachDeg`

**Files:** `loop.go` (the `Status` struct at ~L135; the status update at ~L515), `loop_test.go`.

- [ ] **Step 1: Write the failing test** in `loop_test.go`, alongside the existing pointing-task tests (reuse their SO-101 model and fake-IO helpers).
  - Set a pointing target whose tool axis is rotated about base z, out of reach (the unreachable-pointing setup in 3a/3b).
  - Run ticks until the loop converges.
  - Assert `st.ReachDeg <= st.ErrDeg`, `st.ReachDeg < 1` (the reachable part converged), and `st.PointingResidualDeg > 5`.
  - With the pose task on a reachable target, assert `ReachDeg` > 0 before convergence.
- [ ] **Step 2: Run.** `go test -run ReachDeg ./` → FAIL (`ReachDeg` undefined).
- [ ] **Step 3: Implement.** Add `ReachDeg float64 // the convergence angle: reachable pointing error (pointing) or active orientation error (pose)` to `Status`. Set `st.ReachDeg = ms.convDeg` next to `st.ErrMM, st.ErrDeg = …`.
- [ ] **Step 4: Run.** `go test ./...` → PASS.
- [ ] **Step 5: Commit.** `git commit -am "loop: Status.ReachDeg (the convergence angle)"`.

### Task 2: `targetFilter`

**Files:** create `filter.go` and `filter_test.go`.

- [ ] **Step 1: Write the failing tests** in `filter_test.go`:

```go
package diffik

import (
	"math"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

func down(x, y, z float64) spatialmath.Pose {
	return spatialmath.NewPose(r3.Vector{X: x, Y: y, Z: z}, &spatialmath.OrientationVectorDegrees{OZ: -1})
}

func TestFilterEMASeedsAndConverges(t *testing.T) {
	f := newTargetFilter(TargetFilter{EMAAlpha: 0.5, Feedforward: true})
	t0 := time.Unix(0, 0)
	out, _ := f.apply(t0, down(0, 0, 100))
	if out.Point().Z != 100 {
		t.Fatalf("first sample seeds the EMA: got z %v", out.Point().Z)
	}
	out, _ = f.apply(t0.Add(20*time.Millisecond), down(0, 0, 200))
	if math.Abs(out.Point().Z-150) > 1e-9 {
		t.Fatalf("alpha 0.5 step: got z %v, want 150", out.Point().Z)
	}
}

func TestFilterOffIsIdentity(t *testing.T) {
	f := newTargetFilter(DefaultTargetFilter())
	p := down(10, 20, 30)
	out, clamped := f.apply(time.Unix(0, 0), p)
	if !spatialmath.PoseAlmostEqual(out, p) || clamped {
		t.Fatal("the default filter must pass poses through")
	}
}

func TestFilterHoldsLatestAndNeverStepsBack(t *testing.T) {
	f := newTargetFilter(TargetFilter{EMAAlpha: 1, HoldMM: 1, HoldDeg: 0.5, Dwell: 200 * time.Millisecond, Feedforward: true})
	t0 := time.Unix(0, 0)
	var xs []float64
	for k := 0; k < 100; k++ { // 3 mm/s drift for 2 s
		out, _ := f.apply(t0.Add(time.Duration(k)*20*time.Millisecond), down(200+0.06*float64(k), 0, 150))
		xs = append(xs, out.Point().X)
	}
	for i := 1; i < len(xs); i++ {
		if xs[i] < xs[i-1] {
			t.Fatalf("target stepped backwards at %d: %v -> %v", i, xs[i-1], xs[i])
		}
	}
	// still: after the dwell, the same pose is returned every call
	var last float64
	for k := 0; k < 20; k++ {
		out, _ := f.apply(t0.Add(3*time.Second+time.Duration(k)*20*time.Millisecond), down(210+0.3*float64(k%2), 0, 150))
		if k > 12 && out.Point().X != last {
			t.Fatalf("not holding: %v vs %v", out.Point().X, last)
		}
		last = out.Point().X
	}
}

func TestFilterFloorClamp(t *testing.T) {
	f := newTargetFilter(TargetFilter{EMAAlpha: 1, Floor: &FloorClamp{MinZMM: -10, TipMM: 105, MarginMM: 15}, Feedforward: true})
	// tool straight down, flange at z 100: tip at -5 < +5 -> lift by 10
	out, clamped := f.apply(time.Unix(0, 0), down(200, 0, 100))
	if !clamped || math.Abs(out.Point().Z-110) > 1e-9 {
		t.Fatalf("got z %v clamped %v, want 110 true", out.Point().Z, clamped)
	}
	// 20 mm hover over a tag on the table: flange at 125, tip at 20 -> no clamp
	if _, clamped := f.apply(time.Unix(1, 0), down(200, 0, 125)); clamped {
		t.Fatal("a 20 mm hover must not be clamped")
	}
	if f.clamps != 1 {
		t.Fatalf("clamps = %d, want 1", f.clamps)
	}
}

func TestTargetFilterValidate(t *testing.T) {
	for _, bad := range []TargetFilter{{EMAAlpha: 0}, {EMAAlpha: 1.5}, {EMAAlpha: 1, HoldMM: -1},
		{EMAAlpha: 1, Floor: &FloorClamp{TipMM: -1}}} {
		if bad.validate() == nil {
			t.Errorf("%+v accepted", bad)
		}
	}
	if err := DefaultTargetFilter().validate(); err != nil {
		t.Fatal(err)
	}
}
```

- [ ] **Step 2: Run.** `go test -run 'Filter' ./` → FAIL (undefined).
- [ ] **Step 3: Implement** `filter.go`:

```go
package diffik

import (
	"errors"
	"math"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

// FloorClamp raises a target so its flange and jaw tip (flange + TipMM along the tool axis) clear
// MinZMM + MarginMM in the arm base frame (stage 2's client floor_lift, moved server-side for
// camera-frame targets the client can't see in the arm frame).
type FloorClamp struct{ MinZMM, TipMM, MarginMM float64 }

// TargetFilter is servo_start's optional target_filter (spec 3c3), applied to pose targets in the
// arm base frame: EMA, then hold with hysteresis, then floor clamp.
type TargetFilter struct {
	EMAAlpha    float64       // (0, 1]; 1 = off
	HoldMM      float64       // 0 with HoldDeg 0 = hold off
	HoldDeg     float64
	Dwell       time.Duration // time inside the band before holding
	Floor       *FloorClamp   // nil = off
	Feedforward bool          // false: SetMotion with zero velocity (no extrapolation of noisy targets)
}

// DefaultTargetFilter passes targets through unchanged, with feedforward on (pre-3c3 behaviour).
func DefaultTargetFilter() TargetFilter { return TargetFilter{EMAAlpha: 1, Feedforward: true} }

func (f TargetFilter) validate() error {
	if !(f.EMAAlpha > 0 && f.EMAAlpha <= 1) {
		return errors.New("target_filter.ema_alpha must be in (0, 1]")
	}
	if !(f.HoldMM >= 0) || !(f.HoldDeg >= 0) || f.Dwell < 0 {
		return errors.New("target_filter hold_mm, hold_deg and dwell_ms must be >= 0")
	}
	if c := f.Floor; c != nil && (!(c.TipMM >= 0) || !(c.MarginMM >= 0) || math.IsNaN(c.MinZMM)) {
		return errors.New("target_filter.floor tip_mm and margin_mm must be >= 0")
	}
	return nil
}

// targetFilter is one session's filter state; reset by creating a new one per session.
type targetFilter struct {
	cfg     TargetFilter
	ema     spatialmath.Pose // nil until the first sample seeds it
	anchor  spatialmath.Pose
	since   time.Time
	holding bool
	clamps  int
}

func newTargetFilter(cfg TargetFilter) *targetFilter { return &targetFilter{cfg: cfg} }

// apply filters p (mm, arm base frame) received at t and reports whether the floor clamp moved it.
func (f *targetFilter) apply(t time.Time, p spatialmath.Pose) (spatialmath.Pose, bool) {
	if f.ema == nil || f.cfg.EMAAlpha >= 1 {
		f.ema = p
	} else {
		f.ema = spatialmath.Interpolate(f.ema, p, f.cfg.EMAAlpha)
	}
	out := f.ema
	if f.cfg.HoldMM > 0 || f.cfg.HoldDeg > 0 {
		if f.anchor == nil {
			f.anchor, f.since = out, t
		} else {
			mm := out.Point().Sub(f.anchor.Point()).Norm()
			deg := RotVec(out.Orientation().Quaternion(), f.anchor.Orientation().Quaternion()).Norm() * 180 / math.Pi
			if mm > f.cfg.HoldMM || deg > f.cfg.HoldDeg {
				f.anchor, f.since, f.holding = out, t, false
			} else {
				if !f.holding && t.Sub(f.since) >= f.cfg.Dwell {
					f.anchor, f.holding = out, true // hold the LATEST pose, never an older one
				}
				if f.holding {
					out = f.anchor
				}
			}
		}
	}
	if c := f.cfg.Floor; c != nil {
		ov := out.Orientation().OrientationVectorRadians() // (OX, OY, OZ) is the tool axis, normalised
		z := out.Point().Z
		lowest := math.Min(z, z+c.TipMM*ov.OZ)
		if lift := c.MinZMM + c.MarginMM - lowest; lift > 0 {
			f.clamps++
			return spatialmath.NewPose(out.Point().Add(r3.Vector{Z: lift}), out.Orientation()), true
		}
	}
	return out, false
}
```
  If `Interpolate` with `by` = α doesn't give `p1 + α(p2 − p1)` (check its doc comment), use it with the right argument order so the EMA test passes. Confirm that `OrientationVectorRadians()` is normalised; normalise if not.

- [ ] **Step 4: Run.** `gofmt -w filter.go filter_test.go && go test ./...` → PASS.
- [ ] **Step 5: Commit.** `git add filter.go filter_test.go && git commit -m "targetFilter: EMA, hold with hysteresis, floor clamp, feedforward switch"`.

### Task 3: camera-frame targets in sessions and the DoCommand API

**Review amendments (required):**
- **Default filter:** a zero-value `SessionConfig.Filter` (tests build `SessionConfig` directly via `testSessionCfg`) must mean "off". In `start()`, `if cfg.Filter == (TargetFilter{}) { cfg.Filter = DefaultTargetFilter() }`, then `cfg.Filter.validate()` (error out on failure). Existing session tests must stay green unchanged.
- **Zero inputs, once:** compute them once in `start()` (`s.zeroIn = referenceframe.NewZeroInputs(fsys)`). Per call, use `in := maps.Clone(s.zeroIn); in[s.armName] = q`. `NewZeroInputs` → `fs.FrameNames()` lazily writes a cache without synchronisation, so concurrent `servo_target`s would race.
- **Test 1, telling the joint sources apart:**
  - don't add `_origin` parts by hand; `NewFrameSystem` creates `<name>_origin` per part (adding one fails with FrameAlreadyExists);
  - after `start`, nudge the fake arm's joints a few degrees (below `resync_deg`), tick once, then assert the transformed target equals the one computed from `FK(QMeas)` **and** differs from the one computed from `FK(startQ)`.
- **Test 6 (feedforward):** no existing test observes it. Read `s.loop.vel` under `s.loop.mu` (same package) after each `target()`, advancing the fake clock 20 ms between the two targets.
- **Test 8 (new):** make `fakeFS` a pointer that counts `FrameSystemConfig` and `TransformPose` calls. Assert that `start` with `Frames` calls `FrameSystemConfig` once, and that framed `target()` calls make **zero** frame-system calls (no RPC per target).
- **`SessionStatus` gains `Filter TargetFilter` and `Clamps int`,** filled in `status()` under `s.mu`, so `statusMap` can report them.
- **A shared test helper** (e.g. `newFramedTestManager(t, camPose)`) sets `m.fs = &fakeFS{twb, parts, counters}` with the arm and camera parts, for both the session and module tests.


**Files:** `session.go`, `module.go`, `session_test.go`, `module_test.go`.

- [ ] **Step 1: Interface.**
  - Extend `poseTransformer` (session.go ~L75) with `FrameSystemConfig(ctx context.Context) (*framesystem.Config, error)`. The real `framesystem.Service` already satisfies it.
  - Extend the test `fakeFS` (session_test.go ~L132) with a `parts []*referenceframe.FrameSystemPart` field and that method.

- [ ] **Step 2: Write the failing tests** (session_test.go; reuse its helpers that build a manager with a fake arm in `manual` mode and tick the loop by hand):
  1. **`TestSessionFrameTargetUsesMeasuredJoints`:**
     - `fakeFS.parts` = the arm part (`FrameConfig: referenceframe.NewLinkInFrame(referenceframe.World, spatialmath.NewZeroPose(), "<arm>", nil)`, `ModelFrame:` the fake arm's `Kinematics()` model) plus a camera part (`NewLinkInFrame("<arm>", camPose, "cam", nil)`, no model). Add `"<arm>_origin"` the same way the production frame system does; mirror `NewFrameSystem`'s needs. If origin naming makes it awkward, use session frame `world`.
     - Start with `Frames: true`. Tick once so `QMeas` is set. Send `target(pCam, "cam", true)`.
     - Assert the loop's target pose equals `FK(QMeas) ∘ camPose ∘ pCam`, converted to the base frame, within 1e-6 mm.
  2. **`TestSessionFrameRequiresFrames`:** without `Frames`, `target(p, "cam", true)` errors with `frames`.
  3. **`TestSessionUnknownFrame`:** error mentioning the frame name; the loop target is unchanged.
  4. **`TestSessionFrameChainWithOtherMovableFrameRejected`:** a camera parented to a second model frame (another arm part) → error; a second call returns the cached error.
  5. **`TestSessionTargetApplyFalse`:** returns `Raw` / `Target` and leaves the loop target unchanged, without advancing the filter (the filter's EMA isn't seeded).
  6. **`TestSessionFilterFeedforwardOff`:**
     - with `Filter.Feedforward = false`, two targets 20 ms apart, 2 mm apart: the loop gets zero velocity (expose it through the loop's `SetMotion` argument, or the next tick's commanded twist being only proportional). Assert via whatever the existing velocity-feedforward tests use;
     - with `Feedforward = true`, nonzero velocity.
  7. **module_test:**
     - `servo_start` with `"frames": true` and `"target_filter": {"ema_alpha": 0.5, "hold_mm": 1, "hold_deg": 0.5, "dwell_ms": 200, "floor": {"min_z_mm": -10, "tip_mm": 105, "margin_mm": 15}, "feedforward": false}`;
     - `servo_target` with `"frame"` → the reply has `target`, `target_raw`, `reach_deg`, `pointing_residual_deg` and `clamped`;
     - `servo_status` has `target_filter` and `clamps`;
     - invalid `target_filter` values (`ema_alpha` 0) are rejected.
- [ ] **Step 3: Run.** `go test ./...` → the new tests FAIL.
- [ ] **Step 4: Implement.**
  - `SessionConfig` gains `Frames bool` and `Filter TargetFilter`. The module sets the default filter when it's absent.
  - `session` gains `fs *referenceframe.FrameSystem`, `frame string` (the session frame), `startQ []float64`, `filt *targetFilter`, `ff bool`, and `chains map[string]error` with its own small mutex `cmu`.
  - **In `start()`:**
    - if `cfg.Frames`: call `cfgFS, err := m.fs.FrameSystemConfig(ctx)` and `fsys, err := referenceframe.NewFrameSystem("servo", cfgFS.Parts, nil)`; if `fsys.Frame(armName) == nil`, error `"arm %s is not in the frame system"`;
    - keep `q` (already read) as `startQ`, and set `filt: newTargetFilter(cfg.Filter)`, `ff: cfg.Filter.Feedforward`.
  - **`target(p, frame string, apply bool) (TargetResult, error)`**, where `TargetResult{Status SessionStatus; Target, Raw spatialmath.Pose; Clamped bool}`:

```go
// 1. transform (no lock: local computation only)
if frame != "" && frame != s.frame {
	if s.fs == nil {
		return TargetResult{}, errors.New("servo_target frame requires servo_start frames: true")
	}
	if err := s.checkChain(frame); err != nil {
		return TargetResult{}, err
	}
	q := s.loop.Status().QMeas
	if len(q) == 0 {
		q = s.startQ
	}
	in := referenceframe.NewZeroInputs(s.fs)
	in[s.armName] = q
	tf, err := s.fs.Transform(in.ToLinearInputs(), referenceframe.NewPoseInFrame(frame, p), s.frame)
	if err != nil {
		return TargetResult{}, fmt.Errorf("transforming target from %q: %w", frame, err)
	}
	p = tf.(*referenceframe.PoseInFrame).Pose()
}
raw := p
pb := spatialmath.Compose(spatialmath.PoseInverse(s.twb), p) // base frame: the floor is defined there
if !apply {
	return TargetResult{Status: s.status(), Target: raw, Raw: raw}, nil
}
// 2. usable / stopping checks, then filter + estimator + SetMotion under s.mu (as today)
```
    Then, under the lock and keeping the existing `usable` / `stopping` logic:
    - `fb, clamped := s.filt.apply(now, pb)`;
    - `v := s.est.update(now, fb)`, then `if !s.ff { v = [6]float64{} }`;
    - `s.loop.SetMotion(fb, v)`;
    - return `Target: spatialmath.Compose(s.twb, fb)`, `Raw: raw` and `Clamped`.
  - **`checkChain(frame)`:** cached per frame under `cmu`. `fsys.Frame(frame)` nil → error `unknown frame %q`. `TracebackFrame(f)` → if any frame in the chain, other than the arm's model frame, has `len(Frame.DoF()) > 0`, error `frame %q is attached to movable frame %q (only the session's arm is supported)`.
  - Update every existing caller of `target(p)` (module.go, tests) to `target(p, "", true)`.
  - **module.go:**
    - **`servo_start`:** parse `frames` (bool, default false) and `target_filter` (map: `ema_alpha` (default 1), `hold_mm`, `hold_deg`, `dwell_ms`, `floor` {`min_z_mm`, `tip_mm`, `margin_mm`}, `feedforward` (default true)). Unknown keys in `target_filter` → error. `validate()` it.
    - **`servo_target`:** optional `frame` (string) and `apply` (bool, default true). Reply: `state`, `err_mm`, `err_deg`, plus `reach_deg` (st.ReachDeg), `pointing_residual_deg`, `target: poseMap(res.Target)`, `target_raw: poseMap(res.Raw)` and `clamped`.
    - **`statusMap`:** add `target_filter` (the config) and `clamps`.
- [ ] **Step 5: Run.** `gofmt -l . ; go vet ./... ; go test ./... ; go test -race ./...` → PASS.
- [ ] **Step 6: Commit.** `git commit -am "servo: camera-frame targets via a local frame system (frames: true), target_filter, apply:false preview, reach/residual/target in replies"`.

### Task 4: `servo_client.py` base class (refactor `teleop.py`)

**Review amendments (required):**
- `servo_client.py` keeps every `viam` import **inside functions**, as `teleop.py` does now. `test_teleop.py` and `test_tag_servo.py` have no `viam-sdk` dependency, and import modules that import `servo_client`.
- After the refactor, the controller re-runs the stage-2 sim rehearsal (Part B, B0) before Task 5 builds on it.


**Files:** create `clients/python/servo_client.py`; modify `clients/python/teleop.py`.

- [ ] **Step 1: Move the generic parts out of `Teleop` into `class ServoClient`:**
  - `__init__` (attribute initialisation, `released`-style hooks), `now`, `ask`, `level_limits`, `connect`, opening both connections and the follower handles (`robot`, `robot2`, `svc`, `svc2`, `follower2`), CSV file plumbing (subclasses pass their column lists);
  - `start_session(task, half, **extra)` (extra keys merged into `servo_start`, e.g. `frames`, `target_filter`), `stop_session`, `watchdog`;
  - `close()`, with a hook `async def release_extras(self, step)` that subclasses override (teleop re-torques the leader and frees its gripper);
  - the `main()` scaffolding (signal handler, try / except / finally) as `run_client(cls, argparser)`.

  `Teleop(ServoClient)` keeps the leader-specific parts: the leader reads, `stream()`, the gripper task, `run()`, and `release_extras`. Shared pure helpers (`pose_dict`, `tool_axis`, `pose_delta`, `lowest_z`, `floor_lift`, `Hold`, `Runaway`) stay in `teleop.py` or move to `servo_client.py`; `teleop.py` must still export them, because `test_teleop.py` imports them.
- [ ] **Step 2: Verify that behaviour is unchanged:**
  - `uv run clients/python/test_teleop.py` → 5 ok;
  - `uv run clients/python/test_teleop_lag.py` → ok and ok all-held;
  - `uv run clients/python/teleop.py --help` shows the same flags as before (diff the output against `git stash`'s version).
- [ ] **Step 3: Commit.** `git add clients/python && git commit -m "clients: ServoClient base shared by teleop and tag servo"`.

### Task 5: `tag_servo.py` helpers and client

**Review amendments (required):**
- **Keepalive** re-sends the last **applied** reply's `target`, never an `apply: false` preview's.
- **Look-then-move:** the 0.5 s loss timer is **disabled** during the descent, which deliberately stops using detections.
- **Normal-jump gate:**
  - **re-anchor** after 3 consecutive rejections, or when the last accepted detection is older than 200 ms;
  - **reject grazing views** (`grazing(tag)`), since `normal_jump_deg` is sign-free.
- **Log drops:** write a `drop` row (`kind` = `drop`, with the reason: `jump` or `grazing`) so `tag_lag` can count them.
- **viam imports** stay inside functions.


**Files:** create `clients/python/tag_servo.py` and `clients/python/test_tag_servo.py`.

- [ ] **Step 1: Write the failing tests** (`test_tag_servo.py`, run with `uv run`; plain `assert`):

```python
# /// script
# requires-python = ">=3.11"
# ///
import math, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tag_servo import hover_target, normal_jump_deg, lower_along_axis, down_target, grazing  # noqa: E402


def T(x, y, z, ox, oy, oz, th=0.0):
    return {"x": x, "y": y, "z": z, "o_x": ox, "o_y": oy, "o_z": oz, "theta": th}


def test_hover_flips_normal_toward_camera():
    # tag 500 mm in front of the camera (camera looks along +z); tag normal +z (away from camera) or -z
    for oz in (1.0, -1.0):
        t = hover_target(T(0, 0, 500, 0, 0, oz), hover_mm=150, tip_mm=105)
        assert abs(t["z"] - (500 - 255)) < 1e-9, t       # moved toward the camera
        assert abs(t["o_z"] - 1.0) < 1e-9, t              # tool axis points at the tag (away from camera)


def test_hover_tilted_tag():
    s = math.sin(math.radians(30)); c = math.cos(math.radians(30))
    t = hover_target(T(100, 0, 400, s, 0, -c), hover_mm=20, tip_mm=105)
    d = math.dist((t["x"], t["y"], t["z"]), (100, 0, 400))
    assert abs(d - 125) < 1e-9
    assert t["o_x"] * s + t["o_z"] * -c < 0  # axis opposite the camera-facing normal


def test_normal_jump():
    assert abs(normal_jump_deg(T(0, 0, 1, 0, 0, 1), T(0, 0, 1, math.sin(0.2), 0, math.cos(0.2))) - math.degrees(0.2)) < 1e-6


def test_lower_along_axis():
    p = lower_along_axis(T(200, 0, 255, 0, 0, -1), 130)  # tool down, descend 130 mm
    assert abs(p["z"] - 125) < 1e-9 and p["x"] == 200


def test_down_target_ignores_tag_normal():
    # arm-frame tag position (previewed with apply:false); the hover is vertical, whatever the normal says
    t = down_target({"x": 210, "y": -5, "z": 3, "o_x": 0.05, "o_y": 0, "o_z": 0.99, "theta": 0}, hover_mm=20, tip_mm=105)
    assert (t["x"], t["y"], t["z"]) == (210, -5, 128) and (t["o_x"], t["o_y"], t["o_z"]) == (0, 0, -1), t


def test_grazing():
    assert not grazing(T(0, 0, 500, 0, 0, 1))  # tag faces the camera: not grazing
    assert grazing(T(0, 0, 500, 1, 0, 0.1))    # tag seen nearly edge-on: grazing


if __name__ == "__main__":
    for n, f in list(globals().items()):
        if n.startswith("test_"):
            f(); print("ok", n)
```

- [ ] **Step 2: Run.** `uv run clients/python/test_tag_servo.py` → FAIL (no module).
- [ ] **Step 3: Implement the helpers** at the top of `tag_servo.py` (viam imports inside functions only):

```python
def _unit(v):
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v)


def hover_target(tag: dict, hover_mm: float, tip_mm: float) -> dict:
    """Flange pose (camera frame) hovering hover_mm (jaw tip) above the tag along its normal on the camera's
    side, tool axis pointing at the tag. Robust to the tag's z sign: the normal is flipped toward the camera."""
    p = (tag["x"], tag["y"], tag["z"])
    n = _unit((tag["o_x"], tag["o_y"], tag["o_z"]))
    if sum(a * -b for a, b in zip(n, p)) < 0:  # want n . (camera - tag) > 0, camera at the origin
        n = tuple(-c for c in n)
    d = hover_mm + tip_mm
    return {"x": p[0] + d * n[0], "y": p[1] + d * n[1], "z": p[2] + d * n[2],
            "o_x": -n[0], "o_y": -n[1], "o_z": -n[2], "theta": 0.0}


def normal_jump_deg(a: dict, b: dict) -> float:
    na, nb = _unit((a["o_x"], a["o_y"], a["o_z"])), _unit((b["o_x"], b["o_y"], b["o_z"]))
    dot = abs(sum(x * y for x, y in zip(na, nb)))  # sign-free: the flip handles orientation
    return math.degrees(math.acos(max(-1.0, min(1.0, dot))))


def down_target(tag_arm: dict, hover_mm: float, tip_mm: float) -> dict:
    """--axis down: a vertical hover over the tag's arm-frame position. It ignores the tag's normal, so PnP normal
    noise (about 2 mm per degree at a 125 mm lever) doesn't move the target sideways (static accuracy)."""
    return {"x": tag_arm["x"], "y": tag_arm["y"], "z": tag_arm["z"] + hover_mm + tip_mm,
            "o_x": 0, "o_y": 0, "o_z": -1, "theta": 0}


def grazing(tag: dict, max_incidence_deg: float = 70.0) -> bool:
    """True when the camera sees the tag nearly edge-on (|n . p| < cos 70°): the normal is unreliable there."""
    p, n = _unit((tag["x"], tag["y"], tag["z"])), _unit((tag["o_x"], tag["o_y"], tag["o_z"]))
    return abs(sum(a * b for a, b in zip(n, p))) < math.cos(math.radians(max_incidence_deg))


def lower_along_axis(p: dict, mm: float) -> dict:
    """Move the flange mm along its tool axis (toward what it points at)."""
    a = _unit((p["o_x"], p["o_y"], p["o_z"]))
    return dict(p, x=p["x"] + mm * a[0], y=p["y"] + mm * a[1], z=p["z"] + mm * a[2])
```

- [ ] **Step 4: Run.** The tests pass.
- [ ] **Step 5: Implement the client** `class TagServo(ServoClient)`:
  - **Tracker:**
    - `PoseTracker.from_robot(self.robot2, args.tracker)`; `poses = await tracker.get_poses(body_names=[...])`.
    - Check the `viam:apriltag` source (github viam-labs/apriltag) for the body-name format of a tag ID, and the reference frame of the returned `PoseInFrame` (expected: the camera's name). Use `--tag-id` to pick the tag. Convert the `PoseInFrame.pose` with `pose_dict`. Use `--camera` as the `frame`, unless the reply's frame says otherwise; then assert they match and error on mismatch.
    - Poll it in its own task, as fast as it returns, storing `(t, pose)` of the latest accepted detection.
    - **Normal-jump gate:** reject it if `normal_jump_deg(new, last_accepted) > args.max_normal_jump_deg`, and count the drop.
  - **Session:** `start_session(task="pointing", half=…, frames=True, target_filter={...})`, built from `--ema-alpha`, `--hold-mm`, `--hold-deg`, `--dwell-ms` (200), floor `{"min_z_mm": args.min_z_mm, "tip_mm": args.tip_mm, "margin_mm": 15}`, and `"feedforward": args.feedforward == "on"`.
  - **Building the target per accepted detection:**
    - `--axis normal`: `servo_target(pose=hover_target(tag, hover, tip), frame=camera)`.
    - `--axis down`:
      1. preview **the tag pose itself** (no hover offset): `r = servo_target(pose=tag, frame=camera, apply=False)`;
      2. `p = down_target(r["target_raw"], hover, tip)`;
      3. `servo_target(pose=p)` (no frame).
  - **Keepalive and loss:** loop at 50 Hz. If no new accepted detection arrived in the last 100 ms, re-send the last reply's `target` (arm frame, no `frame`), at most every 100 ms. After 0.5 s without one, `raise RuntimeError("tag lost")` → `close()` stops the session.
  - **Engage:** half limits until `err_mm <= 10 and reach_deg <= 5` for 0.2 s (30 s timeout), using the same structure as teleop. Then `stop_session` and `start_session(half=False)`.
  - **Look-then-move** (`--final-hover-mm` set, used with the wrist camera):
    1. After tracking converges (`err_mm <= 2 and reach_deg <= 2` for 1 s), freeze `frozen = last reply["target"]`.
    2. Stop polling detections, and send arm-frame targets `lower_along_axis(frozen, hover - final_hover)` every 100 ms until converged for 1 s, then for `--duration`.
    3. Print "measure now".
  - **Logs:** `PREFIX-ticks.csv` columns: `t, kind (det|resend|frozen), tag_<keys> (camera frame), target_<keys>, raw_<keys> (arm frame), state, err_mm, err_deg, reach_deg, residual_deg, clamped, rtt_ms`. `PREFIX-follower.csv` comes from the base watchdog.
  - **Flags:** `--camera`, `--tracker`, `--tag-id`, `--hover-mm` (150), `--final-hover-mm` (none), `--axis normal|down`, `--ema-alpha` (0.3), `--hold-mm` (1.0), `--hold-deg` (0.5), `--dwell-ms` (200), `--feedforward on|off` (on), `--max-normal-jump-deg` (20), `--duration`, `--csv`, plus the base's level, safety and address flags (defaults: L2, `--min-z-mm -10`, `--tip-mm 105`, `--timeout-ms 300`).
  - **Hidden rehearsal flag** `--fake-tag-base x,y,z` (`argparse.SUPPRESS`): instead of a tracker, synthesise detections by transforming a fixed base-frame tag pose (normal up, at x,y,z mm in `<follower>_origin`) into the camera frame with `self.robot.transform_pose(PoseInFrame(...), camera)` each poll, at 20 Hz. That's used only in Part B.
- [ ] **Step 6: Verify.** `py_compile`; `--help`; the helper tests.
- [ ] **Step 7: Commit.** `git add clients/python && git commit -m "tag_servo: AprilTag hover/accuracy client (pointing, camera-frame targets, keepalive, look-then-move)"`.

### Task 6: `intrinsics.py`

**Review amendments (required):**
- **Calibration API:** current `opencv-contrib-python` (5.0) has **no** `cv2.aruco.calibrateCameraCharuco`. Use `cv2.aruco.CharucoDetector(board).detectBoard(img)`, then `board.matchImagePoints(corners, ids)`, then `cv2.calibrateCamera` as the primary path. A prototype gave fx within 0.2 % and RMS 0.16 px on 15 synthetic views.
- **Synthetic test:** render the board with a white quiet zone (`generateImage(size, marginSize=…)`), and keep the poses roughly in view.
- **View acceptance:** accept a view when its board hull centre moved > 5 % of the image, or its hull area changed > 15 %. No tilt test, since that would need K.
- **Distortion mapping:** OpenCV `(k1, k2, p1, p2, k3)` → `rk1, rk2, tp1, tp2, rk3`.
- **Runbook/results note:** `viam:apriltag` uses only fx, fy, cx and cy, and never applies distortion. That's an accuracy factor.


**Files:** create `clients/python/intrinsics.py` and `clients/python/test_intrinsics.py` (deps: `opencv-contrib-python`, `numpy`, `viam-sdk`).

- [ ] **Step 1: Write the failing synthetic test.**
  - Render a ChArUco board with `cv2.aruco.CharucoBoard((7, 5), 0.04, 0.03, dict)`, `generateImage`.
  - Project it through a known camera (fx=fy=800, cx=640, cy=360, no distortion) at 15 random board poses, using `cv2.projectPoints` of the board's 3D corners → a homography → `cv2.warpPerspective` onto 1280×720 frames.
  - Call `calibrate(frames, board)` and assert fx and fy within 2 %, cx and cy within 5 px, and RMS < 1 px.
- [ ] **Step 2: Run.** It fails.
- [ ] **Step 3: Implement.**
  - `calibrate(frames, board) -> (K, dist, rms, n_views)`: detect with `cv2.aruco.CharucoDetector`; keep views with ≥ 8 corners; call `cv2.calibrateCameraCharuco` (or `calibrateCamera` with `matchImagePoints`, whichever the installed OpenCV supports).
  - CLI, run as `uv run clients/python/intrinsics.py --address … --camera NAME --dict DICT_5X5_100 --squares 7x5 --square-mm 40 --marker-mm 30 --views 20`:
    - grab frames with `Camera.from_robot(...).get_image()` → numpy;
    - accept a frame when ≥ 8 corners are seen and its board pose differs from the accepted ones (centre moved > 5 % of the image, or tilt changed > 5°);
    - print progress ("move the board");
    - at the end, print the RMS and the JSON to paste into the webcam's config:
      `"intrinsic_parameters": {"width_px", "height_px", "fx", "fy", "ppx", "ppy"}, "distortion_parameters": {"rk1", "rk2", "rk3", "tp1", "tp2"}`. Check the builtin webcam's attribute names in the RDK source (`components/camera/videosource` or the webcam docs), and follow those.
- [ ] **Step 4: Run.** The test passes.
- [ ] **Step 5: Commit.**

### Task 7: `tag_lag.py`

**Review amendments (required):**
- **Rest detection:** use a 0.5 s rolling mean of the `target_*` position moving < 0.5 mm. Per-row change never finds rest in the filter-off static-noise run (about 1 mm noise per row). Add `--rest all` to treat the whole run as rest, for the static-noise run.
- **`skew_ms`:** regress the arm-frame target's deviation from its rest mean against the follower's velocity (from `PREFIX-follower.csv`, differentiated and interpolated). The slope is τ in ms. Add a synthetic test: a target deviation of v·0.08 s → `skew_ms` ≈ 80 ± 10.
- **Drops:** count `drop` rows by reason.


**Files:** create `clients/python/tag_lag.py` and `test_tag_lag.py`. Reuse `teleop_lag.py`'s `_rows` and `_interp`.

- [ ] **Step 1: Write the failing test.** Synthetic ticks and follower CSVs:
  - the `target_*` columns move in a circle, then stay still; `raw_*` = target + Gaussian noise (σ 1 mm, seeded);
  - the follower is the target delayed by 150 ms.

  Assert `lag_ms` ≈ 150 ± 10, the rest `raw_std_mm` ≈ 1 ± 0.3, and the detection rate, resend count and clamp count from the columns.
- [ ] **Step 2: Implement.** `analyze(prefix)`, which returns:
  - `lag_ms` (cross-correlating the follower position against the `target_*` position, during motion);
  - `track_err_mm_at_lag`;
  - the rest window (≥ 2 s where the `target_*` change per row is < 0.5 mm): `raw_std_mm` per axis and in total, plus `raw_normal_std_deg`;
  - `det_rate_hz`, `resends`, `clamped`, `normal_jump_drops` (if logged), the `rtt_ms` p50/p99, and the status maxima (`tick_p99`, `overruns`, `resyncs`) from the follower CSV, as in `teleop_lag`.
- [ ] **Step 3: Run.** The test passes.
- [ ] **Step 4: Commit.**

### Task 8: stage-3 runbook

**Review amendment:** after `eye_in_hand` auto-applies, check that the wrist camera's frame parent is `follower-arm` (or a 0-DoF frame). Parented to the input-enabled gripper, the module's chain check rejects it.


**Files:** modify `dev/so101-hw.md` (append `## Stage 3: AprilTag visual servoing`).

- [ ] **Step 1: Write the section,** following the spec:
  1. **Hardware setup checklist:**
     - cameras, boards and tags, with measured sizes;
     - **record the measured camera-or-board-to-lowest-jaw offsets** for the workspace bounds.
  2. **Config changes** (each with OK and a backup):
     - enable both cameras;
     - add the `viam:apriltag` pose trackers (tag36h11, `tag_width_mm`);
     - add `viam:camera-calibration` (`charuco` trackers + `eye_in_hand` / `eye_to_hand`, with `workspace_bounds` absolute, `input_range_override`, `settle_seconds` 2);
     - add the table obstacle (frame-system component with geometry, top 1–2 mm below the base plane);
     - temporarily lower the follower's `speed_degs_per_sec` (20) and acceleration.
  3. **Intrinsics:** the command, and the RMS bar < 1 px.
  4. **Extrinsics:**
     - start from a high seed pose;
     - `{"calibrate": {}}`;
     - **never `cancel`, never the app's Stop; cut power to abort**;
     - the gates, and checking `auto_applied`.
  5. **Calibration check:** the tag at a ruler-measured position; compare.
  6. **Runs per camera:**
     - static noise (filter off);
     - tracking (`--feedforward on` / `off`; `smooth` if jitter);
     - static accuracy (5 positions; `--axis down`; scene camera at 20 mm closed-loop; wrist camera look-then-move 150 → 20);
     - the measurement point: the tool axis between the pads;
     - timing-skew measurement (wrist camera).
  7. **After every run:** the late lines; `tag_lag.py`.
  8. **Restore** the follower's speed and acceleration after calibration.
- [ ] **Step 2: Commit.**

### Task 9: Part A review (Opus)

Review `git diff d45dcd6..HEAD` against the spec and plan. Focus on:
- the frame transform's correctness and locking;
- the filter order (base frame);
- the `apply: false` path not touching the filter;
- chain-check caching;
- the client's keepalive and loss behaviour (arm-frame re-sends only), and `--axis down`;
- the CSV columns that `tag_lag.py` reads.

Fix the findings.

---

## Part B — sim rehearsal (controller)

`mujoco-sim-mac` (start its viam-server, find the listener) has the SO-101 `arm` and `scene-cam` (frame parent `world`). Rebuild `bin/diff-ik` and restart the module first.

- [ ] **B0.** After Task 4, re-run the stage-2 teleop sim rehearsal (`dev/so101-hw.md`: self-tracking, SIGINT/SIGTERM, prompt, `--fault-after`). Confirm `scene-cam` exists in `mujoco-sim-mac`'s config (`read_machine_config`). If it doesn't, add it with the user's OK, frame parent `world`.
- [ ] **B1. `frame_check.py`** (create; small, not shipped as a tool). It:
  1. starts a session with `frames: true` and pointing, on `arm`;
  2. reads the arm's pose (`servo_start` reply);
  3. expresses it in `scene-cam` with `robot.transform_pose`;
  4. sends that with `frame: "scene-cam"`, and asserts the reply's `target_raw` ≈ the start pose (≤ 0.1 mm);
  5. then expresses a pose 30 mm higher in `scene-cam`, sends it, and checks that the arm moves up 30 mm;
  6. then a pose 200 mm *below* the table with `target_filter.floor` set, and asserts `clamped` is true and the arm stays above the floor;
  7. then `apply: false` → the arm doesn't move;
  8. then `apply: false` with a zero pose in frame `"arm"` (the end-effector frame): `target_raw` ≈ the current end-effector pose. That exercises the `QMeas` / arm-chain path, i.e. the eye-in-hand case, with no config change.
- [ ] **B2. Tag servo pipeline:** `uv run clients/python/tag_servo.py --address … --insecure --follower arm --camera scene-cam --fake-tag-base 200,0,20 --hover-mm 150 --min-z-mm -70 --duration 10 --csv .runs/3c3-sim-fake`. Expect engage, then a hover about 255 mm above the fake tag, and a clean exit. Then `tag_lag.py` runs.
- [ ] **B2b. Look-then-move:** `… --fake-tag-base 200,0,20 --axis down --hover-mm 150 --final-hover-mm 20 --min-z-mm -70`. Expect a converged hover at 150, a descent to 20 (jaw tip at about z 40 over a tag at z 20), "measure now", and no loss trip during the descent.
- [ ] **B3. Loss path:** a hidden `--fake-tag-drop-after 5` (add it to the client with `--fake-tag-base`) stops synthesising detections. Expect keepalive re-sends, then "tag lost" at 0.5 s and a clean `servo_stop`.
- [ ] **B4.** Commit any fixes.

---

## Part C — hardware (controller with the user; never a subagent)

The user sets up the cameras, boards and tags first (spec "Hardware").

- [ ] **C1.** Start `laptop-main` (the user's terminal); find the listener; config changes per the runbook (each with OK and a backup).
- [ ] **C2.** Intrinsics, for each camera.
- [ ] **C3.** Extrinsics with `camera-calibration`: the wrist camera `eye_in_hand` (board on the table), then the scene camera `eye_to_hand` (small board on the gripper). Hands on power; never `cancel`. Then restore the follower's speed.
- [ ] **C4.** Calibration check, for each camera.
- [ ] **C5.** Per camera:
  1. static noise → set the hold;
  2. tracking (feedforward on, then off; `smooth` if jitter);
  3. static accuracy at 5 positions;
  4. the timing-skew measurement (wrist camera).
- [ ] **C6.** Results: `docs/results/2026-11-phase3c3.md`. Commit.
