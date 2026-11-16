# Diff-IK Phase 3c stage 1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the diff-ik CLI, loop and bench safe and level-aware enough to run on the real SO-101, rehearse everything on the MuJoCo SO-101, then run stage 1 on hardware with the user present.

**Architecture:** Part A is code: two small library changes (module `tol_mm`/`tol_deg` limits; `armIO.Stop` graceful drain), three new CLI files (`floor.go` floor guard, `safearm.go` profiled-move wrapper, `probe.go` bus probe), CLI wiring, and bench arguments. Part B rehearses on `mujoco-sim-mac`. Part C is the hardware runbook, run by the controller with the user, never by a subagent.

**Tech Stack:** Go 1.2x, `go.viam.com/rdk v1.16.0`, Python bench (`uv run`, `viam-sdk`).

**Spec:** `docs/superpowers/specs/2026-11-15-diff-ik-phase3c1-design.md`. Read its "Hardware and machine" section before any task: the SO-101 driver writes streamed points at **maximum speed with no acceleration cap**, and its `Stop` may **speed up** a profiled `MoveToJointPositions` instead of stopping it.

**Branch:** `diff-ik-phase3c1` (already checked out in `/Users/nick.hehr/src/diff-ik`). Run all Go commands from the repo root. `go test ./...` must pass at the end of every task.

**Commit trailer** (every commit):
```
Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_012FAJrCqoTNZgDVMfHN6sDY
```

---

## File structure

| File | Change | Responsibility |
|---|---|---|
| `module.go` | modify (~L227-235) | accept `limits.tol_mm` / `limits.tol_deg` |
| `module_test.go` | modify | test the new limits |
| `armio.go` | modify `Stop` | graceful drain: close the channel, wait ≤ 100 ms, then cancel |
| `armio_test.go` | modify | drain tests |
| `cmd/cli/floor.go` | create | `floorGuard`: jaw-box z check for poses, joints and joint-space paths |
| `cmd/cli/floor_test.go` | create | floor-guard tests on `testdata/so101.urdf` |
| `cmd/cli/safearm.go` | create | `safeArm`: profiled moves detached from Ctrl-C, held "in flight" until physically still, no `Stop` meanwhile, floor-checked path |
| `cmd/cli/safearm_test.go` | create | no-Stop rule with a recording fake arm |
| `cmd/cli/probe.go` | create | bus probe: reads, then paced read+send of a fixed `q0` |
| `cmd/cli/probe_test.go` | create | probe against a fake arm that paces like the driver |
| `cmd/cli/main.go` | modify | new flags, sessions off, `safeArm`, `--start-joints`, `probe` command |
| `cmd/cli/run.go` | modify | `runOpts`, guard, `--hold`, circle speed, `minRate`, tolerance and speed limits |
| `cmd/cli/validate.go` | modify | `--settle-tol`, guard re-draw |
| `clients/python/servo_bench.py` | modify | level-aware arguments, one circle |
| `dev/so101-hw.md` | create | hardware runbook and pre-servo check records |
| `docs/results/2026-11-phase3c1.md` | create (Part C) | results |

---

## Part A — code (subagents)

### Task 1: `tol_mm` / `tol_deg` servo limits

**Files:**
- Modify: `module.go` (the `limits` loop around L227-235)
- Test: `module_test.go`

- [ ] **Step 1: Write the failing test.** Add to `module_test.go`, modelled on the existing `servo_start` tests there (read the file first and reuse its helpers for building the service and sending `servo_start`; the test at ~L53 shows how to pass `limits` and L69 how to read `limits` back from `servo_status`). The test:
  - starts a session with `"limits": {"tol_mm": 3.5, "tol_deg": 4.0}`;
  - reads `servo_status` and asserts `limits["tol_mm"] == 3.5` and `limits["tol_deg"] == 4.0`.

  If `servo_status` doesn't report `tol_mm`/`tol_deg` (check `module.go` ~L404), add them to the status `limits` map in Step 3 as well. Also add two rows to the invalid-input table at ~L278: `{[]interface{}{"limits", map[string]interface{}{"tol_mm": 0.0}}, "tol_mm"}` and the same for `tol_deg` with `-1.0`.

- [ ] **Step 2: Run it.** `go test -run 'Servo|Limits|Invalid' ./...` → the new test FAILS (unknown limit is ignored, status shows 2/2).

- [ ] **Step 3: Implement.** In `module.go`, add to the `[]struct{key string; dst *float64}` list:
```go
			{"tol_mm", &lc.TolMM},
			{"tol_deg", &lc.TolDeg},
```
  and, if missing, `"tol_mm": st.Config.TolMM, "tol_deg": st.Config.TolDeg` in the status `limits` map.

- [ ] **Step 4: Run.** `go test ./...` → PASS.

- [ ] **Step 5: Commit.** `git commit -am "servo_start: tol_mm and tol_deg limits"` (+ trailer).

### Task 2: `armIO.Stop` graceful drain

**Why:** the SO-101 driver logs `streamed N points … late K (max M)` only when its batch channel closes normally. Today `Stop` closes the channel and cancels at once, so the line is a race. Spec: "armio.go, graceful drain".

**Files:**
- Modify: `armio.go` (`Stop`, L94-113)
- Test: `armio_test.go`

- [ ] **Step 1: Write the failing tests.** Add to `armio_test.go`:

```go
// drainArm finishes its stream shortly after the batch channel closes, and records whether
// its context was cancelled first.
type drainArm struct {
	arm.Arm
	after     time.Duration // how long after close it returns
	cancelled atomic.Bool
}

func (d *drainArm) MoveThroughJointPositionsStreamed(ctx context.Context, b <-chan []arm.TrajectoryPoint, r chan<- arm.Response, extra map[string]interface{}) error {
	for range b {
	}
	select {
	case <-time.After(d.after):
		return nil
	case <-ctx.Done():
		d.cancelled.Store(true)
		return ctx.Err()
	}
}

func (d *drainArm) Stop(context.Context, map[string]interface{}) error { return nil }

func TestArmIOStopDrainsBeforeCancel(t *testing.T) {
	d := &drainArm{Arm: fakeUR5e(t), after: 20 * time.Millisecond}
	io := NewArmIO(d, 50)
	if err := io.Send(context.Background(), []float64{0, -1.2, 1.5, -1.9, -1.57, 0}); err != nil {
		t.Fatal(err)
	}
	if err := io.Stop(context.Background()); err != nil {
		t.Fatal(err)
	}
	if d.cancelled.Load() {
		t.Fatal("stream that finishes inside the drain window was cancelled")
	}
}

func TestArmIOStopCancelsAfterDrainWindow(t *testing.T) {
	d := &drainArm{Arm: fakeUR5e(t), after: time.Hour}
	io := NewArmIO(d, 50)
	if err := io.Send(context.Background(), []float64{0, -1.2, 1.5, -1.9, -1.57, 0}); err != nil {
		t.Fatal(err)
	}
	start := time.Now()
	if err := io.Stop(context.Background()); err != nil {
		t.Fatal(err)
	}
	if el := time.Since(start); el < 100*time.Millisecond || el > time.Second {
		t.Fatalf("Stop took %v, want ~100 ms", el)
	}
	if !d.cancelled.Load() {
		t.Fatal("hung stream was not cancelled")
	}
}
```
  Add `"sync/atomic"` to the imports. `fakeUR5e(t)` already exists in `session_test.go` (same package); use it, and switch the two existing armio tests to it as well.

- [ ] **Step 2: Run.** `go test -run ArmIO ./` → `TestArmIOStopDrainsBeforeCancel` FAILS ("was cancelled"); the window test FAILS on timing (Stop returns at once). `TestArmIOStopReachesArmWhenStreamHangs` still passes.

- [ ] **Step 3: Implement.** Replace `Stop` in `armio.go`:

```go
// drainWait is how long Stop lets an open stream finish on its own after the batch channel
// closes. A normally ended stream lets the driver log its late-point count (so-101 logs it
// only then); a cancelled one returns without it.
const drainWait = 100 * time.Millisecond

func (io *armIO) Stop(ctx context.Context) error {
	var wait chan struct{}
	if io.batches != nil {
		// Close batches first: drivers may only watch the channel (the fake arm does).
		// Send and Stop both run on the loop goroutine, so nothing can send after this close.
		close(io.batches)
		wait = io.s.done
		d := drainWait
		if dl, ok := ctx.Deadline(); ok {
			d = min(d, time.Until(dl)/2) // leave arm.Stop at least half the caller's budget
		}
		select {
		case <-wait:
		case <-time.After(d):
		case <-ctx.Done():
		}
		io.cancel()
		io.batches = nil
	}
	// Stop the arm first: a hung stream must not eat the caller's deadline.
	err := io.a.Stop(ctx, nil)
	if wait != nil {
		select {
		case <-wait:
		case <-ctx.Done():
		}
	}
	return err
}
```

- [ ] **Step 4: Run.** `go test ./...` → PASS, then `go test -race -count=50 -run ArmIO ./` → PASS. Without the half-budget cap, `TestArmIOStopReachesArmWhenStreamHangs` (100 ms ctx) fails intermittently with "arm.Stop got an already-done ctx": the drain would eat the caller's whole deadline.

- [ ] **Step 5: Commit.** `git commit -am "armIO.Stop: drain the stream up to 100 ms before cancelling"`.

### Task 3: floor guard

**Files:**
- Create: `cmd/cli/floor.go`, `cmd/cli/floor_test.go`

- [ ] **Step 1: Write the failing tests** in `cmd/cli/floor_test.go`:

```go
package main

import (
	"math"
	"strings"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// qStart is the stage-1 start pose (spec "Start pose").
var qStart = []float64{0, -0.61, 0.46, 1.39, 0}

func so101(t *testing.T) referenceframe.Model {
	t.Helper()
	m, err := referenceframe.ParseModelXMLFile("../../testdata/so101.urdf", "arm", nil)
	if err != nil {
		t.Fatal(err)
	}
	return m
}

func TestFloorAcceptsStartPose(t *testing.T) {
	g := &floorGuard{model: so101(t), minZ: 30, tipMM: 105}
	if err := g.checkJoints(qStart); err != nil {
		t.Fatal(err)
	}
	// the jaw box bottoms out at ~88 mm at q_start (spec review FK)
	p, _ := g.model.Transform(qStart)
	if z, _ := g.lowest(p); math.Abs(z-88.2) > 1.5 {
		t.Fatalf("lowest jaw-box z = %.1f, want ~88.2", z)
	}
}

func TestFloorRejectsLowTip(t *testing.T) {
	g := &floorGuard{model: so101(t), minZ: 30, tipMM: 105}
	// tool pointing straight down with the end effector at z 100: tip at z -5
	down := spatialmath.NewPose(r3.Vector{X: 200, Z: 100}, &spatialmath.OrientationVectorDegrees{OZ: -1})
	err := g.checkPose(down)
	if err == nil || !strings.Contains(err.Error(), "jaw") {
		t.Fatalf("err = %v, want a jaw rejection", err)
	}
}

func TestFloorRejectsLowEndEffector(t *testing.T) {
	g := &floorGuard{model: so101(t), minZ: 30, tipMM: 105}
	// tool pointing straight up with the end effector at z 10
	up := spatialmath.NewPose(r3.Vector{X: 200, Z: 10}, &spatialmath.OrientationVectorDegrees{OZ: 1})
	err := g.checkPose(up)
	if err == nil || !strings.Contains(err.Error(), "end effector") {
		t.Fatalf("err = %v, want an end-effector rejection", err)
	}
}

func TestFloorRejectsDippingCircle(t *testing.T) {
	g := &floorGuard{model: so101(t), minZ: 30, tipMM: 105}
	start, _ := g.model.Transform(qStart)
	// a circle 70 mm lower than start puts the jaw box at ~18 mm
	low := spatialmath.NewPose(start.Point().Add(r3.Vector{Z: -70}), start.Orientation())
	if err := g.checkCircle(low, 50); err == nil {
		t.Fatal("circle below the floor accepted")
	}
	if err := g.checkCircle(start, 50); err != nil {
		t.Fatalf("circle at q_start rejected: %v", err)
	}
}

func TestFloorPath(t *testing.T) {
	g := &floorGuard{model: so101(t), minZ: 30, tipMM: 105}
	// q_start → the singular pre-move's qa (joint 2 to −74° + 0.2 rad); spec: never below ~88 mm
	qa := []float64{0, -0.61, -74*math.Pi/180 + 0.2, 1.39, 0}
	if err := g.checkPath(qStart, qa); err != nil {
		t.Fatalf("q_start → qa rejected: %v", err)
	}
	// a path from q_start that pitches the shoulder forward and down must be rejected
	if err := g.checkPath(qStart, qLow); err == nil {
		t.Fatal("path into the floor accepted")
	}
	// starting below the floor is allowed while z only rises
	if err := g.checkPath(qLow, qStart); err != nil {
		t.Fatalf("rising path out of a low start rejected: %v", err)
	}
}

func TestNilFloorGuardChecksNothing(t *testing.T) {
	var g *floorGuard
	if g.checkPose(spatialmath.NewZeroPose()) != nil || g.checkJoints(nil) != nil || g.checkPath(nil, nil) != nil || g.checkCircle(nil, 0) != nil {
		t.Fatal("nil guard should accept everything")
	}
}
```

  Add the package-level test variable (found by an FK sweep in plan review; from shoulder-lift 0.9 rad upward the line to `q_start` is no longer monotonic):
```go
// qLow: q_start with the shoulder pitched forward; lowest jaw-box point z ≈ −20.5 mm, and the
// lowest z rises steadily along the joint-space line to q_start.
var qLow = []float64{0, 0.3, 0.46, 1.39, 0}
```

- [ ] **Step 2: Run.** `go test ./cmd/cli/` → FAIL (undefined `floorGuard`).

- [ ] **Step 3: Implement** `cmd/cli/floor.go`:

```go
package main

import (
	"fmt"
	"math"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// floorGuard rejects targets that would bring the end effector or the jaw below minZ
// (base-frame z, mm). The jaw is a box in the tool frame: Z = tipMM, X/Y = ±15 mm.
// A nil guard accepts everything. Spec 3c1 "Floor guard".
type floorGuard struct {
	model referenceframe.Model
	minZ  float64
	tipMM float64
}

// lowest returns the lowest base-frame z (mm) of the jaw box at pose p, and that point's name.
func (g *floorGuard) lowest(p spatialmath.Pose) (float64, string) {
	pts := []struct {
		name string
		v    r3.Vector
	}{
		{"end effector", r3.Vector{}},
		{"jaw tip", r3.Vector{Z: g.tipMM}},
		{"jaw +x", r3.Vector{X: 15, Z: g.tipMM}},
		{"jaw -x", r3.Vector{X: -15, Z: g.tipMM}},
		{"jaw +y", r3.Vector{Y: 15, Z: g.tipMM}},
		{"jaw -y", r3.Vector{Y: -15, Z: g.tipMM}},
	}
	z, name := math.Inf(1), ""
	for _, pt := range pts {
		if w := spatialmath.Compose(p, spatialmath.NewPoseFromPoint(pt.v)).Point().Z; w < z {
			z, name = w, pt.name
		}
	}
	return z, name
}

func (g *floorGuard) checkPose(p spatialmath.Pose) error {
	if g == nil {
		return nil
	}
	if z, name := g.lowest(p); z < g.minZ {
		return fmt.Errorf("floor guard: %s at z %.1f mm is %.1f mm below the floor (%.1f mm)", name, z, g.minZ-z, g.minZ)
	}
	return nil
}

func (g *floorGuard) checkJoints(q []float64) error {
	if g == nil {
		return nil
	}
	p, err := g.model.Transform(q)
	if err != nil {
		return err
	}
	return g.checkPose(p)
}

// checkCircle checks the circle scenario's path: centre's start point offset by radius in x/y,
// every 10°, at the start orientation (both run.go's and the bench's circles hold z and orientation).
func (g *floorGuard) checkCircle(start spatialmath.Pose, radius float64) error {
	if g == nil {
		return nil
	}
	for deg := 0; deg < 360; deg += 10 {
		th := float64(deg) * math.Pi / 180
		p := spatialmath.NewPose(start.Point().Add(r3.Vector{X: radius * math.Cos(th), Y: radius * math.Sin(th)}), start.Orientation())
		if err := g.checkPose(p); err != nil {
			return fmt.Errorf("circle at %d°: %w", deg, err)
		}
	}
	return nil
}

// checkPath checks 21 points on the joint-space line from → to: the driver's coordinated
// MoveToJointPositions is linear in joint space. A start already below the floor (e.g. the
// rest pose) is allowed while the lowest z keeps rising; once a point is below the floor and
// not higher than the one before, or the goal is below, the move is rejected.
func (g *floorGuard) checkPath(from, to []float64) error {
	if g == nil {
		return nil
	}
	if len(from) != len(to) {
		return fmt.Errorf("floor guard: path from %d joints to %d", len(from), len(to))
	}
	const n = 20
	prev := math.Inf(-1)
	for k := 0; k <= n; k++ {
		q := make([]float64, len(from))
		for i := range q {
			q[i] = from[i] + (to[i]-from[i])*float64(k)/n
		}
		p, err := g.model.Transform(q)
		if err != nil {
			return err
		}
		z, name := g.lowest(p)
		if z < g.minZ && (k == n || z <= prev) {
			return fmt.Errorf("floor guard: move path point %d/%d: %s at z %.1f mm is %.1f mm below the floor (%.1f mm)",
				k, n, name, z, g.minZ-z, g.minZ)
		}
		prev = z
	}
	return nil
}
```
  If `model.Transform` takes `[]referenceframe.Input` and `Input` is not `float64` in rdk v1.16.0, convert with the helper the repo already uses (grep `FloatsToInputs` / `referenceframe.Input` in `kin.go`). In this repo `MoveToJointPositions` takes `[]float64`, so it's expected to be `float64`.

- [ ] **Step 4: Run.** `go test ./cmd/cli/` → PASS.
- [ ] **Step 5: Commit.** `git add cmd/cli/floor*.go && git commit -m "cli: floor guard (jaw box, circle, joint-space path)"`.

### Task 4: `safeArm` — profiled moves without `Stop`

**Why (spec "Hardware", "Safety", "Code changes"):** on the SO-101 driver `Stop` = `SetVelocity(0)` = **maximum** speed, and it doesn't interrupt a profiled move. So while a `MoveToJointPositions` is in flight (until the arm is physically still), nothing may call `arm.Stop`, and Ctrl-C must not cancel the RPC. Wrapping the arm puts this rule in one place for `run`, `validate`, `singular`'s pre-move and the `--start-joints` move, and makes the existing `err != nil` defers that call `a.Stop` safe without touching them.

**Files:**
- Create: `cmd/cli/safearm.go`, `cmd/cli/safearm_test.go`

- [ ] **Step 1: Write the failing tests** in `cmd/cli/safearm_test.go`:

```go
package main

import (
	"context"
	"errors"
	"sync"
	"testing"
	"time"

	"go.viam.com/rdk/components/arm"
)

// recArm records calls. MoveToJointPositions blocks until release is closed and records
// whether its context was cancelled; joints stay where the move left them.
type recArm struct {
	arm.Arm
	mu        sync.Mutex
	q         []float64
	calls     []string
	release   chan struct{}
	moveCtxOK bool
}

func (r *recArm) log(s string) { r.mu.Lock(); r.calls = append(r.calls, s); r.mu.Unlock() }

func (r *recArm) JointPositions(context.Context, map[string]interface{}) ([]float64, error) {
	r.mu.Lock()
	defer r.mu.Unlock()
	return append([]float64(nil), r.q...), nil
}

func (r *recArm) IsMoving(context.Context) (bool, error) { return false, nil }

func (r *recArm) MoveToJointPositions(ctx context.Context, q []float64, _ map[string]interface{}) error {
	r.log("move")
	<-r.release
	r.mu.Lock()
	r.moveCtxOK = ctx.Err() == nil
	r.q = append([]float64(nil), q...)
	r.mu.Unlock()
	return nil
}

func (r *recArm) Stop(context.Context, map[string]interface{}) error { r.log("stop"); return nil }

func TestSafeArmNoStopDuringProfiledMove(t *testing.T) {
	r := &recArm{q: []float64{0, 0, 0, 0, 0}, release: make(chan struct{})}
	s := &safeArm{Arm: r}
	ctx, cancel := context.WithCancel(context.Background())
	done := make(chan error, 1)
	go func() { done <- s.MoveToJointPositions(ctx, []float64{0.1, 0, 0, 0, 0}, nil) }()
	time.Sleep(20 * time.Millisecond)
	cancel() // Ctrl-C
	if err := s.Stop(context.Background(), nil); err != nil {
		t.Fatal(err)
	}
	select {
	case <-done:
		t.Fatal("move returned before the arm finished")
	case <-time.After(50 * time.Millisecond):
	}
	close(r.release)
	if err := <-done; !errors.Is(err, context.Canceled) {
		t.Fatalf("err = %v, want context.Canceled once the move finished", err)
	}
	if !r.moveCtxOK {
		t.Fatal("Ctrl-C cancelled the profiled move's RPC context")
	}
	for _, c := range r.calls {
		if c == "stop" {
			t.Fatalf("arm.Stop reached the arm during a profiled move: %v", r.calls)
		}
	}
	// after the move, Stop goes through again
	if err := s.Stop(context.Background(), nil); err != nil {
		t.Fatal(err)
	}
	if r.calls[len(r.calls)-1] != "stop" {
		t.Fatalf("Stop after the move was swallowed: %v", r.calls)
	}
}

func TestSafeArmFloorGuardBlocksMove(t *testing.T) {
	r := &recArm{q: qStart, release: make(chan struct{})}
	close(r.release)
	s := &safeArm{Arm: r, guard: &floorGuard{model: so101(t), minZ: 30, tipMM: 105}}
	if err := s.MoveToJointPositions(context.Background(), qLow, nil); err == nil {
		t.Fatal("move into the floor was issued")
	}
	if len(r.calls) != 0 {
		t.Fatalf("arm saw %v", r.calls)
	}
}
```
  (`qLow` is from Task 3.)

- [ ] **Step 2: Run.** `go test -run SafeArm ./cmd/cli/` → FAIL (undefined `safeArm`).

- [ ] **Step 3: Implement** `cmd/cli/safearm.go`:

```go
package main

import (
	"context"
	"errors"
	"fmt"
	"math"
	"sync/atomic"
	"time"

	"go.viam.com/rdk/components/arm"
)

// safeArm keeps arm.Stop away from profiled moves. On the SO-101 driver Stop writes goal
// speed 0, which means maximum speed, and doesn't interrupt a MoveToJointPositions
// (spec 3c1 "Hardware and machine"). So a profiled move:
//   - is floor-checked along its joint-space path first;
//   - runs on a context detached from Ctrl-C;
//   - counts as in flight until the arm is physically still, and Stop is a no-op meanwhile.
// Streaming paths (armIO) still reach Stop: they never overlap a profiled move.
type safeArm struct {
	arm.Arm
	guard    *floorGuard
	inFlight atomic.Bool
}

const (
	stillTol     = 2e-3 // rad: one STS3215 count is 360/4095° = 1.534e-3 rad, so allow a one-count flicker
	stillGap     = 100 * time.Millisecond
	stillTimeout = 30 * time.Second
)

func (s *safeArm) MoveToJointPositions(ctx context.Context, q []float64, extra map[string]interface{}) error {
	cur, err := s.Arm.JointPositions(ctx, nil)
	if err != nil {
		return err
	}
	if err := s.guard.checkPath(cur, q); err != nil {
		return err
	}
	s.inFlight.Store(true)
	bg := context.WithoutCancel(ctx)
	done := make(chan error, 1)
	go func() { done <- s.Arm.MoveToJointPositions(bg, q, extra) }()
	var moveErr error
	interrupted := ctx.Done()
wait:
	for {
		select {
		case moveErr = <-done:
			break wait
		case <-interrupted:
			fmt.Println("Ctrl-C: a profiled move can only be stopped by cutting power; waiting for the arm to finish")
			interrupted = nil
		}
	}
	if err := s.waitStill(bg); err != nil {
		// still moving: leave inFlight set so nothing calls Stop on a moving arm
		return errors.Join(moveErr, err)
	}
	s.inFlight.Store(false)
	return errors.Join(moveErr, ctx.Err())
}

// waitStill returns once two joint reads stillGap apart differ by < stillTol. The reads are the
// physical test; IsMoving isn't used, because a servo holding against gravity can flicker it.
func (s *safeArm) waitStill(ctx context.Context) error {
	deadline := time.Now().Add(stillTimeout)
	for time.Now().Before(deadline) {
		q1, err := s.Arm.JointPositions(ctx, nil)
		if err != nil {
			return err
		}
		time.Sleep(stillGap)
		q2, err := s.Arm.JointPositions(ctx, nil)
		if err != nil {
			return err
		}
		d := 0.0
		for i := range q1 {
			d = math.Max(d, math.Abs(q2[i]-q1[i]))
		}
		if d < stillTol {
			return nil
		}
	}
	return fmt.Errorf("arm still moving %v after a profiled move returned", stillTimeout)
}

func (s *safeArm) Stop(ctx context.Context, extra map[string]interface{}) error {
	if s.inFlight.Load() {
		fmt.Println("not calling arm.Stop: a profiled move is in flight (on this driver Stop may speed it up)")
		return nil
	}
	return s.Arm.Stop(ctx, extra)
}
```

- [ ] **Step 4: Run.** `gofmt -w cmd/cli/ && go test ./cmd/cli/` → PASS (gofmt reflows the doc-comment list).
- [ ] **Step 5: Commit.** `git add cmd/cli/safearm*.go && git commit -m "cli: safeArm keeps Stop and Ctrl-C away from profiled moves"`.

### Task 5: bus probe

**Files:**
- Create: `cmd/cli/probe.go`, `cmd/cli/probe_test.go`

**Rules (spec `probe` row):** 500 back-to-back `JointPositions` reads; then a ticker-paced loop at `rate` for `dur` through `diffik.NewArmIO`, each tick doing one `Read` and one `Send` of **a fixed `q0` read once at the start** — never re-send the latest reading (gravity creep), never send back to back. Report read p50/p99, tick p50/p99 (tick = read+send duration), overruns (tick > period), and a reminder to read the driver's late count from the machine log.

- [ ] **Step 1: Write the failing test** in `cmd/cli/probe_test.go`:

```go
package main

import (
	"context"
	"slices"
	"sync"
	"testing"
	"time"

	"go.viam.com/rdk/components/arm"
)

// pacedArm streams like the so-101 driver: it waits until each point is due (stream start +
// point time), then "writes" it. Its joints sag a little on every read, like an
// uncompensated arm, so re-sending readings would creep.
type pacedArm struct {
	arm.Arm
	mu      sync.Mutex
	reads   int
	written [][]float64
	at      []time.Time
}

func (p *pacedArm) JointPositions(context.Context, map[string]interface{}) ([]float64, error) {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.reads++
	return []float64{0, -0.61 + 1e-3*float64(p.reads), 0.46, 1.39, 0}, nil
}

func (p *pacedArm) MoveThroughJointPositionsStreamed(ctx context.Context, b <-chan []arm.TrajectoryPoint, r chan<- arm.Response, _ map[string]interface{}) error {
	start := time.Now()
	for batch := range b {
		for _, pt := range batch {
			time.Sleep(time.Until(start.Add(pt.Time)))
			p.mu.Lock()
			p.written = append(p.written, slices.Clone(pt.Positions))
			p.at = append(p.at, time.Now())
			p.mu.Unlock()
		}
	}
	return nil
}

func (p *pacedArm) Stop(context.Context, map[string]interface{}) error { return nil }

func TestProbeHoldsFixedQ0AndPaces(t *testing.T) {
	p := &pacedArm{}
	res, err := probeRun(context.Background(), p, 50, 500*time.Millisecond, 20)
	if err != nil {
		t.Fatal(err)
	}
	if res.reads.n != 20 || res.ticks.n < 20 {
		t.Fatalf("reads %d ticks %d", res.reads.n, res.ticks.n)
	}
	p.mu.Lock()
	defer p.mu.Unlock()
	q0 := p.written[1] // written[0] is the stream's t=0 point at the current joints
	for i, q := range p.written[1:] {
		if !slices.Equal(q, q0) {
			t.Fatalf("point %d = %v, want the fixed q0 %v", i+1, q, q0)
		}
	}
	for i := 2; i < len(p.at); i++ {
		if gap := p.at[i].Sub(p.at[i-1]); gap < 10*time.Millisecond {
			t.Fatalf("points %d and %d written %v apart: sends are back to back", i-1, i, gap)
		}
	}
}
```

- [ ] **Step 2: Run.** `go test -run Probe ./cmd/cli/` → FAIL (undefined `probeRun`).

- [ ] **Step 3: Implement** `cmd/cli/probe.go`:

```go
package main

import (
	"context"
	"fmt"
	"slices"
	"time"

	"diffik"

	"go.viam.com/rdk/components/arm"
)

type durStats struct {
	n        int
	p50, p99 time.Duration
}

func stats(d []time.Duration) durStats {
	if len(d) == 0 {
		return durStats{}
	}
	s := slices.Clone(d)
	slices.Sort(s)
	return durStats{n: len(s), p50: s[len(s)/2], p99: s[len(s)*99/100]}
}

type probeResult struct {
	reads, ticks durStats
	overruns     int
}

// probeRun times nReads back-to-back joint reads, then a ticker-paced read+send loop at rateHz
// for dur that holds a fixed q0 read once at the start (spec 3c1 "probe").
func probeRun(ctx context.Context, a arm.Arm, rateHz float64, dur time.Duration, nReads int) (probeResult, error) {
	var res probeResult
	var reads []time.Duration
	for i := 0; i < nReads; i++ {
		t0 := time.Now()
		if _, err := a.JointPositions(ctx, nil); err != nil {
			return res, err
		}
		reads = append(reads, time.Since(t0))
	}
	res.reads = stats(reads)

	q0, err := a.JointPositions(ctx, nil)
	if err != nil {
		return res, err
	}
	io := diffik.NewArmIO(a, rateHz)
	period := time.Duration(float64(time.Second) / rateHz)
	tk := time.NewTicker(period)
	defer tk.Stop()
	var ticks []time.Duration
	end := time.Now().Add(dur)
	var loopErr error
	for time.Now().Before(end) && loopErr == nil {
		select {
		case <-ctx.Done():
			loopErr = ctx.Err()
			continue
		case <-tk.C:
		}
		t0 := time.Now()
		if _, err := io.Read(ctx); err != nil {
			loopErr = err
			break
		}
		if err := io.Send(ctx, q0); err != nil {
			loopErr = err
			break
		}
		el := time.Since(t0)
		ticks = append(ticks, el)
		if el > period {
			res.overruns++
		}
	}
	stopCtx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	stopErr := io.Stop(stopCtx)
	res.ticks = stats(ticks)
	if loopErr != nil {
		return res, loopErr
	}
	return res, stopErr
}

func probe(ctx context.Context, a arm.Arm, rateHz float64) error {
	res, err := probeRun(ctx, a, rateHz, 10*time.Second, 500)
	fmt.Printf("reads:    n %d  p50 %v  p99 %v\n", res.reads.n, res.reads.p50, res.reads.p99)
	fmt.Printf("read+send at %.0f Hz: n %d  p50 %v  p99 %v  overruns %d\n", rateHz, res.ticks.n, res.ticks.p50, res.ticks.p99, res.overruns)
	period := time.Duration(float64(time.Second) / rateHz)
	fmt.Printf("tick p99 within the %v period: %s\n", period, pass(res.ticks.p99 <= period))
	fmt.Println("now read the driver's `streamed N points … late K (max M)` line from the machine log")
	return err
}
```

- [ ] **Step 4: Run.** `go test ./cmd/cli/` → PASS.
- [ ] **Step 5: Commit.** `git add cmd/cli/probe*.go && git commit -m "cli: bus probe (paced read+send of a fixed q0)"`.

### Task 6: CLI wiring (`main.go`, `run.go`, `validate.go`)

**Files:**
- Modify: `cmd/cli/main.go`, `cmd/cli/run.go`, `cmd/cli/validate.go`

No new unit tests: the logic under test lives in Tasks 3-5. Verification is `go vet`, `go test ./...`, a `--help` check, and Part B's sim run.

- [ ] **Step 1: `main.go` flags.** Add:
```go
	minZ := flag.Float64("min-z-mm", 0, "floor guard: lowest allowed base-frame z (mm) for the end effector and jaw box; enabled whenever the flag is given (it can be 0 or negative)")
	tipMM := flag.Float64("tip-mm", 105, "floor guard: jaw tip distance along the tool z axis (mm)")
	settleTol := flag.Float64("settle-tol", 1e-4, "validate: max joint change (rad) between bracketing reads to accept a settled sample")
	hold := flag.Duration("hold", 0, "static: hold each target this long after convergence and report the drift")
	maxLin := flag.Float64("max-linear-mm-s", 0, "run: linear speed limit (0 = loop default 100)")
	maxAng := flag.Float64("max-angular-deg-s", 0, "run: angular speed limit (0 = loop default 45)")
	tolMM := flag.Float64("tol-mm", 0, "run: convergence position tolerance (0 = loop default 2)")
	circleMMs := flag.Float64("circle-mm-s", 50, "circle: speed around the circle (mm/s)")
	startJoints := flag.String("start-joints", "", "comma-separated joint angles (rad): run/validate/probe first make a guarded profiled move here")
```
  After `flag.Parse()`:
  - `minZSet := false; flag.Visit(func(f *flag.Flag) { if f.Name == "min-z-mm" { minZSet = true } })`.
  - if `minZSet && *startJoints == ""` → `errors.New("--start-joints is required with --min-z-mm")`.
  - validate `*tipMM > 0`, `*settleTol > 0`, `*circleMMs > 0`, `*hold >= 0`, `*maxLin >= 0`, `*maxAng >= 0`, `*tolMM >= 0`.
  - update `usage` to add `       cli [flags] probe`.

- [ ] **Step 2: `main.go` connection and arm.**
  - `opts := []client.RobotClientOption{client.WithDialOptions(dial...), client.WithDisableSessions()}` with a comment: `// Sessions off: a lapsed session heartbeat makes viam-server call arm.Stop, which on the SO-101 speeds up a profiled move (spec 3c1).`
  - after `arm.FromProvider`: build `var guard *floorGuard; if minZSet { guard = &floorGuard{model: model, minZ: *minZ, tipMM: *tipMM} }` (move the `Kinematics` call above this) and wrap: `sa := &safeArm{Arm: a, guard: guard}`; use `sa` (as `arm.Arm`) everywhere below instead of `a`.
  - if `*startJoints != ""`: parse with `strings.Split` + `strconv.ParseFloat`, require `len == len(model.DoF())`, print `moving to start joints <q> (profiled, 30°/s on the SO-101 driver)` and call `sa.MoveToJointPositions(ctx, q, nil)`; return its error.
  - dispatch: add `case args[0] == "probe": return probe(ctx, sa, *rate)`; pass `*settleTol, guard` to `validate`; pass a `runOpts` to `run` (Step 3).

- [ ] **Step 3: `run.go` → `runOpts`.** Replace `run`'s long parameter list:
```go
type runOpts struct {
	rate                                          float64
	csvPath                                       string
	wrist                                         int
	maxJointDegS, maxLinear, maxAngular, resync   float64
	tolMM                                         float64
	task                                          string
	singEps, maxDamp, singDeg                     float64
	hold                                          time.Duration
	circleMMs                                     float64
	guard                                         *floorGuard
}

func run(ctx context.Context, a arm.Arm, model referenceframe.Model, scenario string, o runOpts) (err error)
```
  and build `cfg := diffik.LoopConfig{RateHz: o.rate, ResyncDeg: o.resync, Task: o.task, SingularEps: o.singEps, MaxDamping: o.maxDamp, MaxLinearMMPerS: o.maxLinear, MaxAngularDegPerS: o.maxAngular, TolMM: o.tolMM}`. **Keep the existing `if maxJointDegS > 0 { ... cfg.MaxJointDegPerS = append(...) }` loop** (now on `o.maxJointDegS`), or the level's joint limit silently falls back to 90°/s.

- [ ] **Step 4: `run.go` rate bar.** Replace the `const minRate = 49.5` block with
```go
	// 1% tolerance: a ticker's measured average straddles the rate from first/last tick jitter.
	minRate := 0.99 * o.rate
```
  keeping the print and the NaN-safe check.

- [ ] **Step 5: `run.go` guard + hold in `static` / `staticReachable`.**
  - Both get `guard *floorGuard, hold time.Duration` (closures or params; `body` stays `func(ctx, l, rec, start) error`).
  - Each candidate target: `if err := guard.checkPose(target); err != nil { fmt.Printf("skipping target: %v\n", err); continue }` before `SetTarget` — re-draw, printing which point and by how much (spec "On rejection"). `staticReachable` already has a `tries` cap; give `static` the same pattern: `for i, tries := 0, 0; i < 10; tries++ { if tries >= 1000 { return fmt.Errorf(...) } ... }`, incrementing `i` only for accepted targets. Note: `staticReachable` targets come from `model.Transform(q)`; check that pose.
  - After a converged target, if `hold > 0` call:
```go
// holdTarget keeps the current target for d and reports the worst and final error: on the
// real arm this is settling and gravity sag (spec 3c1 "Settling and sag").
func holdTarget(ctx context.Context, l *diffik.Loop, d time.Duration) {
	var maxMM, maxDeg float64
	end := time.Now().Add(d)
	for time.Now().Before(end) && ctx.Err() == nil {
		s := l.Status()
		maxMM, maxDeg = math.Max(maxMM, s.ErrMM), math.Max(maxDeg, s.ErrDeg)
		time.Sleep(10 * time.Millisecond)
	}
	s := l.Status()
	fmt.Printf("           hold %v: worst %.2f mm %.2f°  final %.2f mm %.2f°\n", d, maxMM, maxDeg, s.ErrMM, s.ErrDeg)
}
```

- [ ] **Step 6: `run.go` circle.** Turn `circle` into `circleAt(speed float64, guard *floorGuard) func(ctx, l, rec, start) error`; keep `radius = 50`, use `speed` for `omega` and for the lag print. Before `SetTarget(first)`: `if err := guard.checkCircle(start, radius); err != nil { return err }`. In `run`: `case "circle": body = circleAt(o.circleMMs, o.guard)`. Print `circle: %.0f mm/s` at the start.

- [ ] **Step 7: `run.go` singular.** Pass `guard` into `singular`; after computing `qb`: `if err := guard.checkJoints(qb); err != nil { return nil, fmt.Errorf("singular target: %w", err) }` before the pre-move. The pre-move itself goes through `safeArm` (path-checked, no Stop while moving).

- [ ] **Step 8: `validate.go`.**
  - signature `validate(ctx, a, model, spread, settleTol float64, guard *floorGuard)`;
  - draw loop: `for k, tries := 0, 0; k < configs; tries++ { if tries >= 10000 { return fmt.Errorf("floor guard rejected too many validate samples (%d accepted)", k) } ... k++ }` (move `k++` to the end of an accepted config; `j := k % len(q)` unchanged);
  - re-draw on the **path**, because `safeArm` path-checks every move and a rejected move would otherwise fail the whole run: read the current joints `cur` and skip the sample (printing the error) if `guard.checkPath(cur, q)` fails; after the move, compute the nudge `q2` and skip that config's Jacobian part (printing the error) if `guard.checkPath(qm, q2)` fails, still counting the FK sample. The final return to `start` goes through `safeArm`, whose path check covers it;
  - `moveAndMeasure(ctx, a, q, settleTol)` replaces the `const still = 1e-4`;
  - the `defer … a.Stop` stays: `safeArm` makes it a no-op while a profiled move is in flight.

- [ ] **Step 9: Verify.** `gofmt -l cmd/ && go vet ./... && go test ./...` → clean and PASS. `go run ./cmd/cli --help 2>&1 | grep -E 'min-z-mm|start-joints|circle-mm-s|tol-mm|hold|settle-tol|tip-mm|max-linear|max-angular'` → all nine flags listed. `go run ./cmd/cli --arm x --min-z-mm 30 run static` → error `--start-joints is required with --min-z-mm`.

- [ ] **Step 10: Commit.** `git commit -am "cli: stage-1 flags, sessions off, safeArm, guard, hold, level-aware circle and rate bar"`.

### Task 7: level-aware bench

**Files:**
- Modify: `clients/python/servo_bench.py`

- [ ] **Step 1: Arguments.** In `main()` add:
```python
    ap.add_argument("--linear-mm-s", type=float, default=None)
    ap.add_argument("--angular-deg-s", type=float, default=None)
    ap.add_argument("--rate", type=float, default=50)
    ap.add_argument("--timeout-ms", type=float, default=500, help="dead-man step's timeout")
    ap.add_argument("--resync-deg", type=float, default=None)
    ap.add_argument("--singular-eps", type=float, default=None)
    ap.add_argument("--max-damping", type=float, default=None)
    ap.add_argument("--tol-mm", type=float, default=None)
    ap.add_argument("--twist-mm-s", type=float, default=50, help="stop and dead-man twist speed")
    ap.add_argument("--circle-mm-s", type=float, default=50)
```

- [ ] **Step 2: Limits in every session.** Replace `start_kw`:
```python
def level_limits(args) -> dict:
    lim = {"joint_deg_s": args.joint_deg_s}
    for key, val in (("linear_mm_s", args.linear_mm_s), ("angular_deg_s", args.angular_deg_s),
                     ("resync_deg", args.resync_deg), ("singular_eps", args.singular_eps),
                     ("max_damping", args.max_damping), ("tol_mm", args.tol_mm)):
        if val is not None:
            lim[key] = val
    return lim


def start_kw(args, **kw):
    base = {"command": "servo_start", "component_name": args.arm, "frame": f"{args.arm}_origin",
            "rate_hz": args.rate, "limits": level_limits(args)}
    if args.task:
        base["task"] = args.task
    base.update(kw)
    return base
```

- [ ] **Step 3: Steps use the level.**
  - `stop_latency`: twist `[args.twist_mm_s, 0, 0]`; add `report("ideal ramp travel", f"{args.twist_mm_s * 0.2 / 2:.1f} mm")` (ramp 200 ms, the module default; confirm in `session.go`/`module.go` and use that constant).
  - `deadman`: `timeout_ms, ramp_ms, rate = args.timeout_ms, 200, args.rate`; twist `[-args.twist_mm_s, 0, 0]`.
  - `circle(svc, arm2, args, home=None)`: drop the `resync` parameter and the local `limits` (resync now comes from `start_kw`); `speed = args.circle_mm_s`; label `f"circle {speed:.0f} mm/s"`; always `report(f"{label}: mean error", f"{mean:.1f} mm", "≤ 22.1 mm", mean <= 22.1)`.
  - In `circle`'s send loop, pace targets with `args.rate` instead of the literal 50 (`t0 + n / args.rate`).
  - `main`: one call, `await circle(svc, arm2, args, home)`.
  - Update the module docstring's usage line to show the L1 flags:
    `uv run clients/python/servo_bench.py --address … --arm follower-arm --task pointing --joint-deg-s 30 --linear-mm-s 40 --angular-deg-s 20 --timeout-ms 300 --twist-mm-s 30 --circle-mm-s 32 --resync-deg <from check 4> --singular-eps 0.03 --max-damping 0.05`.

- [ ] **Step 4: Verify.** `python3 -m py_compile clients/python/servo_bench.py` → no output. `uv run clients/python/servo_bench.py --help` → lists the new flags. (The live run is Part B.)
- [ ] **Step 5: Commit.** `git commit -am "bench: level-aware limits, twist and circle speeds; one circle per run"`.

### Task 8: hardware runbook `dev/so101-hw.md`

**Files:**
- Create: `dev/so101-hw.md`

- [ ] **Step 1: Write it** with these sections (copy numbers from the spec; leave blanks `___` for measurements):
  1. **Setup:** machine `laptop` / part `laptop-main`, arm `follower-arm`, how to find the CLI address (the viam-server log line for the plaintext listener, as on `mujoco-sim-mac`; else the `.local.viam.cloud:8081` address with `VIAM_API_KEY_ID`/`VIAM_API_KEY`).
  2. **Safety rules:** one hand on the power switch and one near the arm during every L1 run and every profiled move; Ctrl-C while streaming stops at once; Ctrl-C during a profiled move does not stop it (only power does, and that drops the arm); never kill the bench during its move to home. If the CLI hangs in a profiled move (its RPC runs without a deadline, and a second Ctrl-C is absorbed), kill it from another terminal with `kill -9` once the arm is still: with sessions off, that doesn't make the server call `Stop`.
  3. **`q_start`:** `0,-0.61,0.46,1.39,0` rad = (0, −35.0, 26.4, 79.6, 0)°; EE (180, 0, 192) mm; pad end ≈ 92 mm above the base plane, ≈ 169 mm from the shoulder-pan axis (208 mm from the base origin). Sign-off line.
  4. **Pre-servo checks 1-5** exactly as in the spec, each with a result line and pass/stop rule; check 4 records per-joint droop, the EE error, and the resulting `resync_deg` and `--tol-mm`.
  5. **`qa`/`qb`** for `singular --wrist 2 --singular-deg -74`: EE ≈ (63, 0, 377) and (1, 0, 379) mm; sign-off line.
  6. **Commands per level** (L1/L2/L3 table from the spec), each as a copy-paste CLI line, e.g. L1:
```
go run ./cmd/cli --address <addr> [--insecure] --arm follower-arm --task pointing \
  --start-joints 0,-0.61,0.46,1.39,0 --min-z-mm <30 − base offset> \
  --max-joint-deg-s 30 --max-linear-mm-s 40 --max-angular-deg-s 20 --resync-deg <R> [--tol-mm <T>] \
  --singular-eps 0.03 --max-damping 0.05 --rate <probe rate> \
  --hold 1s --csv .runs/3c1-L1-static.csv run static
```
     plus `run singular --wrist 2 --singular-deg -74`, `run circle --circle-mm-s 32` (50 at L2/L3), `probe`, `validate --settle-tol 2e-3`, and the bench line.
  7. **After every run:** read the `streamed … late` line with `read_machine_logs` on `laptop-main`; record max late; fail if ≥ 1 tick or missing.
  8. **Between levels:** servo temperature/load if the driver exposes them (check its DoCommand), else "not exposed".
- [ ] **Step 2: Commit.** `git add dev/so101-hw.md && git commit -m "dev: SO-101 hardware runbook for stage 1"`.

### Task 9: final code review (Opus) of Part A

Dispatch a reviewer over `git diff e7471dc..HEAD`, with the spec. Fix findings before Part B.

---

## Part B — rehearsal on the MuJoCo SO-101 (controller, no user needed)

`mujoco-sim-mac` runs the SO-101 sim (`arm`, frame z 120 on a 100 mm riser over a 20 mm table, so the base plane is 100 mm above the tabletop → `--min-z-mm -70`) and the `servo` service.

- [ ] **B1.** `make bin/diff-ik` (or the repo's build target), then restart the module: `pkill -f bin/diff-ik` (viam-server restarts it). Find the plaintext listener port in `/private/tmp/claude-501/-Users-nick-hehr-src-diff-ik/cc53bb8a-07c6-42b7-93b2-cb3cb4096b47/scratchpad/viam-server.log`.
- [ ] **B2.** CLI at L1 against `--arm arm`, with `--start-joints 0,-0.61,0.46,1.39,0 --min-z-mm -70`: `probe`, `validate --settle-tol 2e-3`, `run static --hold 1s`, `run singular --wrist 2 --singular-deg -74`, `run circle --circle-mm-s 32`. Expect everything to pass (sim has no droop; pass `--resync-deg 1.8`).
- [ ] **B3.** Ctrl-C test during the `--start-joints` move: start from a far pose, press Ctrl-C mid-move → the warning prints, the move completes, no `Stop` reaches the arm (sim log), exit code non-zero.
- [ ] **B4.** Floor-guard negative test: `--min-z-mm 200` → `run static` rejects at the start-joints path or re-draws/fails with a floor-guard message, arm never moves below.
- [ ] **B5.** Bench at L1 flags (arm `arm`, `--resync-deg 1.8`), starting from `q_start`: all PASS, circle ≤ 22.1 mm.
- [ ] **B6.** If the sim's module logs a `streamed … late` line, confirm it appears after each run (the sim arm may not log it; then only confirm no regressions).
- [ ] **B7.** Record any fixes; commit.

---

## Part C — hardware (controller with the user present; never a subagent)

Follow `dev/so101-hw.md`. Each step needs the user at the arm; ask before moving it.

- [ ] **C1.** Pre-servo checks 1-5 and sign-offs; fill in `dev/so101-hw.md`. Any stop rule → stop stage 1 and report.
- [ ] **C2.** `probe` at 50 Hz (then 40, 30 … per the rate rule); read the late line.
- [ ] **C3.** `validate --settle-tol 2e-3` (informational).
- [ ] **C4.** L1 CLI runs: static `--hold 1s`, singular, circle at 32 mm/s; late line after each.
- [ ] **C5.** Ask the user's OK, then back up `laptop-main`'s config and add the local `diff-ik` module and `servo` service via `set_machine_config` (spec "Config change").
- [ ] **C6.** L1 bench and the unreachable-pointing check (DoCommand, axis rotated 20° about base z).
- [ ] **C7.** L2, then L3, each only after a clean pass and the user's go-ahead; temperature/load between levels.
- [ ] **C8.** Write `docs/results/2026-11-phase3c1.md` (per-level tables, 3b comparison, recommended rate / `resync_deg` / damping / speeds for stages 2 and 3, driver findings) and commit.
