# Diff-IK Phase 2: DoCommand Servo Surface — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Expose the Phase 1 servo loop through `motion.Service.DoCommand` as five commands: `servo_start`, `servo_target`, `servo_twist`, `servo_stop` and `servo_status`. Targets are latest-wins, a dead-man timeout stops abandoned sessions, and velocity feedforward follows moving targets. The measurements are taken from Python and TypeScript clients.

**Architecture:**
- The loop (`loop.go`) gains:
  - a feedforward twist that advances its target, with an extrapolation hold and a per-axis leash;
  - a ramp-stop;
  - an injectable clock.
- A session layer (`session.go`) owns one loop per arm. It holds the latest-wins slots, the world→base transform, the target-velocity estimator and the dead-man check.
- `module.go` parses the DoCommand maps, converts units and dispatches.

**Tech Stack:** Go 1.26 with `go.viam.com/rdk v1.16.0` and gonum; Python ≥ 3.11 with `viam-sdk`, run through `uv`; TypeScript with `@viamrobotics/sdk` on Node 24 and `tsx`.

**Spec:** `docs/superpowers/specs/2026-11-15-diff-ik-phase2-design.md`. Read it before starting any task. Where this plan and the spec disagree, the spec wins; raise the disagreement in your report.

---

## Conventions for every task

- Work on branch `diff-ik-phase2` in `/Users/nick.hehr/src/diff-ik`. Don't push.
- **Units:** poses are `spatialmath.Pose` in **mm**, as in Viam and the existing `Loop.SetTarget`. Velocities, twists and errors inside the loop are **SI**: m/s, rad/s, m and rad. `mmPerM` (1000) is defined in `kin.go`.
- **Before every commit, run** `gofmt -w .` first, because pasted blocks shift struct-field alignment. Then:
  ```
  gofmt -l .          # must print nothing
  go vet ./...
  go test -race ./...
  ```
- **Commit messages** end with:
  ```
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_012FAJrCqoTNZgDVMfHN6sDY
  ```
- **Existing helpers you will reuse:**
  - In `kin.go`: `FK` (returns m and a quaternion), `RotVec`, `PoseError` (target − current), `Jacobian`, `dlsStep`, `mmPerM`, `fdStep`.
  - In `loop_test.go`: `simIO`, `home`, `newTestLoop`, `offsetTarget`, `runTicks`.
  - In `kin_test.go`: `ur5e(t)`, `rotZ`, `rotX`.

## File map

| File | Change |
|---|---|
| `kin.go` | Add `ExpRot`, the inverse of `RotVec`. |
| `kin_test.go` | Add an `ExpRot` round-trip test. |
| `loop.go` | Add the `Now` clock, the `stopped` and `stopped_timeout` states, `SetMotion`, `SetVelocity`, `advance` (hold and leash), `RampStop` and `errRampDone`. Reorder `tick`. Make `nextState` velocity-aware. |
| `loop_test.go` | Add `fakeClock`, `clockTicks`, and tests for twist tracking, hold, convergence, leash, reversal and ramp. |
| `session.go` (new) | `velEstimator`, `session`, `servoManager`, `SessionConfig`, `SessionStatus`, `Mode`. |
| `session_test.go` (new) | Estimator tests, and session tests against a fake clock and `simIO`, plus one real-clock integration test. |
| `module.go` | `Config.Arms`, `Validate`, constructors, `DoCommand`, `Move` busy check, `StopPlan`, `Close`, and parse/format helpers. Removes the unused scaffold fields. |
| `module_test.go` (new) | Validate, DoCommand parsing and errors, the pose round-trip, and integration through `servo_stop` and `StopPlan`. |
| `clients/python/servo_bench.py` (new) | Bench: smoke, latency, stop, dead-man and circle. |
| `clients/ts/` (new) | `package.json`, `tsconfig.json`, `smoke.ts`. |
| `docs/results/2026-11-phase2.md` (new) | Results. Task 8 is done by the controller, not a subagent. |

---

### Task 1: Loop feedforward — `ExpRot`, clock, `SetMotion` / `SetVelocity`, hold, leash, convergence

**Files:**
- Modify: `kin.go` (append `ExpRot`)
- Modify: `kin_test.go` (append a test)
- Modify: `loop.go`
- Modify: `loop_test.go` (append helpers and tests)

- [ ] **Step 1: Write the failing tests**

Append to `kin_test.go`:

```go
func TestExpRotInvertsRotVec(t *testing.T) {
	for _, v := range []r3.Vector{{X: 0.3}, {Y: -1.2, Z: 0.4}, {X: 1e-9}, {}} {
		got := RotVec(ExpRot(v), quat.Number{Real: 1})
		if got.Sub(v).Norm() > 1e-12 {
			t.Errorf("RotVec(ExpRot(%v)) = %v", v, got)
		}
	}
}
```

Append to `loop_test.go`. Add `"sync"` to its imports.

```go
// fakeClock is a manually advanced clock for Loop.Now and the session.
type fakeClock struct {
	mu sync.Mutex
	t  time.Time
}

func newFakeClock() *fakeClock { return &fakeClock{t: time.Unix(1000, 0)} }

func (c *fakeClock) Now() time.Time {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.t
}

func (c *fakeClock) Advance(d time.Duration) {
	c.mu.Lock()
	defer c.mu.Unlock()
	c.t = c.t.Add(d)
}

func newClockLoop(t *testing.T, q0 []float64, cfg LoopConfig) (*Loop, *simIO, referenceframe.Model, *fakeClock) {
	t.Helper()
	l, sim, m := newTestLoop(t, q0, cfg)
	clk := newFakeClock()
	l.Now = clk.Now
	return l, sim, m, clk
}

// clockTicks runs up to n ticks, advancing clk one period before each.
// It returns true if a tick reported a completed ramp-stop.
func clockTicks(t *testing.T, l *Loop, clk *fakeClock, n int) bool {
	t.Helper()
	dt := 1 / l.cfg.RateHz
	for i := 0; i < n; i++ {
		clk.Advance(time.Duration(dt * float64(time.Second)))
		err := l.tick(context.Background(), dt)
		if errors.Is(err, errRampDone) {
			return true
		}
		if err != nil {
			t.Fatal(err)
		}
	}
	return false
}

func TestTwistAdvancesTargetAndTracks(t *testing.T) {
	l, _, m, clk := newClockLoop(t, home, LoopConfig{})
	start := offsetTarget(t, m, home, r3.Vector{}, 0)
	l.SetTarget(start)
	l.SetVelocity([6]float64{0.02}) // 20 mm/s along base x
	clockTicks(t, l, clk, 50)        // 1 s
	s := l.Status()
	if d := s.TargetMM.X - start.Point().X; math.Abs(d-20) > 0.05 {
		t.Fatalf("target moved %.3f mm, want 20", d)
	}
	if s.ErrMM > 1 || s.State != StateRunning {
		t.Fatalf("err = %.2f mm state = %s, want < 1 mm and running", s.ErrMM, s.State)
	}
}

func TestExtrapolationHoldsAfterFloor(t *testing.T) {
	l, _, m, clk := newClockLoop(t, home, LoopConfig{})
	p0 := offsetTarget(t, m, home, r3.Vector{}, 0)
	l.SetMotion(p0, [6]float64{0.05}) // first call: hold after the 100 ms floor
	clockTicks(t, l, clk, 50)
	s := l.Status()
	if d := s.TargetMM.X - p0.Point().X; math.Abs(d-5) > 0.05 {
		t.Fatalf("target advanced %.3f mm, want 5 (100 ms at 50 mm/s)", d)
	}
	if s.State != StateConverged {
		t.Fatalf("state = %s, want converged once held", s.State)
	}
}

func TestExtrapolationHoldsAfterTwiceInterval(t *testing.T) {
	l, _, m, clk := newClockLoop(t, home, LoopConfig{})
	p0 := offsetTarget(t, m, home, r3.Vector{}, 0)
	l.SetMotion(p0, [6]float64{0.05})
	clockTicks(t, l, clk, 10) // 200 ms
	p1 := spatialmath.NewPose(p0.Point().Add(r3.Vector{X: 10}), p0.Orientation())
	l.SetMotion(p1, [6]float64{0.05}) // interval 200 ms → hold after 400 ms
	clockTicks(t, l, clk, 50)
	if d := l.Status().TargetMM.X - p1.Point().X; math.Abs(d-20) > 0.05 {
		t.Fatalf("target advanced %.3f mm past p1, want 20 (400 ms at 50 mm/s)", d)
	}
}

func TestConvergedRequiresZeroVelocity(t *testing.T) {
	l, _, m, clk := newClockLoop(t, home, LoopConfig{})
	l.SetTarget(offsetTarget(t, m, home, r3.Vector{}, 0))
	l.SetVelocity([6]float64{0.0005}) // 0.5 mm/s: error stays far under tolerance
	clockTicks(t, l, clk, 50)
	if s := l.Status().State; s != StateRunning {
		t.Fatalf("state = %s while moving, want running", s)
	}
	l.SetVelocity([6]float64{})
	clockTicks(t, l, clk, 5)
	if s := l.Status().State; s != StateConverged {
		t.Fatalf("state = %s after zero twist, want converged", s)
	}
}

// Stands in for the spec's "twist into a joint limit" case: a near-frozen arm blocks q_cmd the same way
// a limit does. TestTwistReversalKeepsMoving covers the arm's reverse travel.
// A near-frozen arm (1e-3 °/s joints) cannot follow the twist, so the target must stop
// 10 mm (0.1 s × 100 mm/s) ahead, and a reversed twist must pull it back at once.
func TestLeashBoundsTargetAndReleasesOnReverse(t *testing.T) {
	slow := []float64{1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3}
	l, _, m, clk := newClockLoop(t, home, LoopConfig{MaxJointDegPerS: slow})
	l.SetTarget(offsetTarget(t, m, home, r3.Vector{}, 0))
	l.SetVelocity([6]float64{0.05})
	clockTicks(t, l, clk, 50)
	s := l.Status()
	if s.CmdErrMM > 11 {
		t.Fatalf("target ran %.2f mm ahead of the arm, leash is 10 mm", s.CmdErrMM)
	}
	fwd := s.TargetMM.X
	l.SetVelocity([6]float64{-0.05})
	clockTicks(t, l, clk, 5)
	if x := l.Status().TargetMM.X; x >= fwd {
		t.Fatalf("reversed twist did not move the target back within 5 ticks (%.2f ≥ %.2f)", x, fwd)
	}
	clockTicks(t, l, clk, 45)
	s = l.Status()
	if back := fwd - s.TargetMM.X; back < 19 || s.CmdErrMM > 11 {
		t.Fatalf("target came back %.2f mm (want ≥ 19: through the arm to the far leash), cmd err %.2f mm", back, s.CmdErrMM)
	}
}

// The deadlock the leash must not have: after a forward twist, a reversed twist keeps the arm moving.
func TestTwistReversalKeepsMoving(t *testing.T) {
	l, sim, m, clk := newClockLoop(t, home, LoopConfig{})
	l.SetTarget(offsetTarget(t, m, home, r3.Vector{}, 0))
	l.SetVelocity([6]float64{0.05})
	clockTicks(t, l, clk, 50)
	p1, _, err := FK(m, sim.q)
	if err != nil {
		t.Fatal(err)
	}
	l.SetVelocity([6]float64{-0.05})
	clockTicks(t, l, clk, 50)
	p2, _, err := FK(m, sim.q)
	if err != nil {
		t.Fatal(err)
	}
	if back := (p1.X - p2.X) * mmPerM; back < 30 {
		t.Fatalf("arm reversed only %.1f mm in 1 s at 50 mm/s, want > 30", back)
	}
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `go test ./... -run 'ExpRot|Twist|Extrapolation|ConvergedRequires|Leash'`
Expected: a build failure, because `ExpRot`, `Loop.Now`, `SetMotion`, `SetVelocity` and `errRampDone` are undefined. `errRampDone` is referenced by `clockTicks` and is added in this task.

- [ ] **Step 3: Implement**

Append to `kin.go`:

```go
// ExpRot returns the unit quaternion for base-frame rotation vector v (rad): the inverse of RotVec.
func ExpRot(v r3.Vector) quat.Number {
	th := v.Norm()
	if th < 1e-12 {
		return quat.Number{Real: 1}
	}
	s := math.Sin(th/2) / th
	return quat.Number{Real: math.Cos(th / 2), Imag: s * v.X, Jmag: s * v.Y, Kmag: s * v.Z}
}
```

In `loop.go`:

1. Add `"gonum.org/v1/gonum/num/quat"` to the imports.

2. Replace the states block with this, and add the constants and the error after it:

```go
// Loop states.
const (
	StateIdle           State = "idle"
	StateRunning        State = "running"
	StateConverged      State = "converged"
	StateStalled        State = "stalled"
	StateFaulted        State = "faulted"
	StateStopped        State = "stopped"
	StateStoppedTimeout State = "stopped_timeout"
)

const (
	minHold = 100 * time.Millisecond // floor on the SetMotion extrapolation hold
	leashS  = 0.1                    // the target may lead FK(q_cmd) by this many seconds at the speed limits
)

// errRampDone is returned by tick once a RampStop has completed and the arm has been stopped.
var errRampDone = errors.New("ramp stop complete")
```

3. In `type Loop struct`, add these after `OnTick`:

```go
	// Now is the loop's clock for the extrapolation hold and the ramp. Nil means time.Now. Set before Run.
	Now func() time.Time

	stopErr error // io.Stop result of a completed ramp; written and read only on the loop goroutine
```

Add these after `target spatialmath.Pose`, guarded by `mu`:

```go
	vel       [6]float64    // feedforward twist, base frame, m/s and rad/s
	extrap    bool          // vel came from SetMotion, so the hold applies
	lastSet   time.Time     // time of the last SetMotion
	holdAfter time.Duration // stop extrapolating this long after lastSet
	rampStart time.Time     // zero: no ramp
	rampDur   time.Duration
	rampFinal State
```

4. Replace `SetTarget` with:

```go
// SetTarget sets a stationary target pose (mm, base frame). Latest wins.
func (l *Loop) SetTarget(p spatialmath.Pose) { l.SetMotion(p, [6]float64{}) }

// SetMotion sets the target pose (mm, base frame) and a feedforward twist vel (m/s, rad/s, base frame).
// The loop advances the target at vel until max(2 × the interval since the previous SetMotion, 100 ms)
// has passed, then holds it. Latest wins. Ignored once a RampStop has begun.
func (l *Loop) SetMotion(p spatialmath.Pose, vel [6]float64) {
	l.mu.Lock()
	defer l.mu.Unlock()
	if !l.rampStart.IsZero() {
		return
	}
	now := l.now()
	l.holdAfter = minHold
	if !l.lastSet.IsZero() {
		l.holdAfter = max(minHold, 2*now.Sub(l.lastSet))
	}
	l.lastSet = now
	l.target, l.vel, l.extrap = p, vel, true
	l.errRing = l.errRing[:0]
}

// SetVelocity sets the feedforward twist (m/s, rad/s, base frame) and keeps advancing the current
// target at it, with no hold. Ignored before a target exists or once a RampStop has begun.
func (l *Loop) SetVelocity(vel [6]float64) {
	l.mu.Lock()
	defer l.mu.Unlock()
	if !l.rampStart.IsZero() || l.target == nil {
		return
	}
	l.vel, l.extrap = vel, false
}

func (l *Loop) now() time.Time {
	if l.Now != nil {
		return l.Now()
	}
	return time.Now()
}

// rampScale reports whether a ramp-stop is active and the velocity scale in [0, 1]. Caller holds l.mu.
// Task 2 adds RampStop, which sets rampStart.
func (l *Loop) rampScale() (bool, float64) {
	if l.rampStart.IsZero() {
		return false, 1
	}
	if l.rampDur <= 0 {
		return true, 0
	}
	return true, math.Max(0, 1-float64(l.now().Sub(l.rampStart))/float64(l.rampDur))
}

// advance moves the stored target one tick along vel and returns the target and the feedforward
// twist to use. cp and cq are FK(q_cmd). Caller holds l.mu.
//   - Hold (SetMotion only): after holdAfter the target stays put and no feedforward is added.
//   - Ramp: the target stays put but the feedforward still applies; tick scales it toward zero.
//   - Leash, per axis: an advance is skipped only when the commanded-side error already exceeds
//     leashS × the speed limit and the advance would grow it (vel · e > 0).
func (l *Loop) advance(dt float64, cp r3.Vector, cq quat.Number, ramping bool) (spatialmath.Pose, [6]float64) {
	var zero [6]float64
	if l.extrap && l.now().Sub(l.lastSet) > l.holdAfter {
		return l.target, zero
	}
	vel := l.vel
	if ramping || vel == zero {
		return l.target, vel
	}
	tp, tq := l.target.Point().Mul(1/mmPerM), l.target.Orientation().Quaternion()
	e := PoseError(tp, tq, cp, cq)
	vl, el := r3.Vector{X: vel[0], Y: vel[1], Z: vel[2]}, r3.Vector{X: e[0], Y: e[1], Z: e[2]}
	if !(el.Norm() > leashS*l.cfg.MaxLinearMMPerS/mmPerM && vl.Dot(el) > 0) {
		tp = tp.Add(vl.Mul(dt))
	}
	va, ea := r3.Vector{X: vel[3], Y: vel[4], Z: vel[5]}, r3.Vector{X: e[3], Y: e[4], Z: e[5]}
	if !(ea.Norm() > leashS*deg2rad(l.cfg.MaxAngularDegPerS) && va.Dot(ea) > 0) {
		tq = quat.Mul(ExpRot(va.Mul(dt)), tq)
	}
	q := spatialmath.Quaternion(tq)
	l.target = spatialmath.NewPose(tp.Mul(mmPerM), &q)
	return l.target, vel
}
```

5. In `Run`, inside `if err := l.tick(ctx, dt); err != nil {`, add this before the existing `if ctx.Err() != nil` check:

```go
			if errors.Is(err, errRampDone) {
				return l.stopErr // tick already stopped the arm
			}
```

6. Replace the beginning of `tick`, from `func (l *Loop) tick(...) error {` down to and including the `e := PoseError(tp, tq, cp, cq)` line, with:

```go
func (l *Loop) tick(ctx context.Context, dt float64) error {
	// 0. ramp-stop complete: stop the arm and end. This runs before the idle return so an idle ramp ends too.
	l.mu.Lock()
	ramping, scale := l.rampScale()
	if ramping && scale == 0 {
		l.status.State = l.rampFinal
		l.mu.Unlock()
		l.stopErr = l.stop(nil)
		return errRampDone
	}
	hasTarget := l.target != nil
	l.mu.Unlock()

	// 1. measured joints
	qm, err := l.io.Read(ctx)
	if err != nil {
		return fmt.Errorf("reading joints: %w", err)
	}
	if len(qm) != len(l.lo) {
		return fmt.Errorf("arm reported %d joints, model has %d", len(qm), len(l.lo))
	}
	if l.qCmd == nil {
		l.qCmd = slices.Clone(qm)
	}
	if !hasTarget {
		l.mu.Lock()
		l.status.State = StateIdle
		l.status.QMeas = slices.Clone(qm)
		l.mu.Unlock()
		return nil
	}

	// 2. hybrid resync (outside mu)
	resynced := false
	if maxAbsDiff(l.qCmd, qm) > deg2rad(l.cfg.ResyncDeg) {
		copy(l.qCmd, qm)
		resynced = true
	}

	// 3. FK(q_cmd), then advance the target under mu, then the error from the copy (spec "Tick order")
	cp, cq, err := FK(l.model, l.qCmd)
	if err != nil {
		return err
	}
	l.mu.Lock()
	target, vel := l.advance(dt, cp, cq, ramping)
	l.mu.Unlock()
	tp, tq := target.Point().Mul(1/mmPerM), target.Orientation().Quaternion()
	e := PoseError(tp, tq, cp, cq)
```

7. Replace steps 4–5 (the `lin :=`, `ang :=` and `dx :=` lines) with:

```go
	// 4–5. v = vel + Gain·e, capped, scaled by any ramp → task step
	lin := capNorm(r3.Vector{X: vel[0] + l.cfg.Gain*e[0], Y: vel[1] + l.cfg.Gain*e[1], Z: vel[2] + l.cfg.Gain*e[2]},
		l.cfg.MaxLinearMMPerS/mmPerM).Mul(scale)
	ang := capNorm(r3.Vector{X: vel[3] + l.cfg.Gain*e[3], Y: vel[4] + l.cfg.Gain*e[4], Z: vel[5] + l.cfg.Gain*e[5]},
		deg2rad(l.cfg.MaxAngularDegPerS)).Mul(scale)
	dx := [6]float64{lin.X * dt, lin.Y * dt, lin.Z * dt, ang.X * dt, ang.Y * dt, ang.Z * dt}
```

8. At the end of `tick`, change `st.State = l.nextState(em, limited)` to `st.State = l.nextState(em, limited, vel != ([6]float64{}))`. Then change `nextState`'s signature and its first test:

```go
// nextState picks converged/stalled/running. moving means a feedforward twist is active. Caller holds l.mu.
func (l *Loop) nextState(em [6]float64, limited, moving bool) State {
	if !moving && posMM(em) <= l.cfg.TolMM && angDeg(em) <= l.cfg.TolDeg {
```

`StateConverged` stays where it is.

- [ ] **Step 4: Run all tests**

Run: `go test -race ./...`
Expected: PASS, including every Phase 1 test.

- [ ] **Step 5: Commit**

```bash
gofmt -l . && go vet ./...
git add kin.go kin_test.go loop.go loop_test.go
git commit -m "Loop feedforward: SetMotion/SetVelocity, extrapolation hold, per-axis leash, clock

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_012FAJrCqoTNZgDVMfHN6sDY"
```

---

### Task 2: Loop `RampStop`

**Files:**
- Modify: `loop.go`
- Modify: `loop_test.go`

- [ ] **Step 1: Write the failing tests**

Append to `loop_test.go`:

```go
func TestRampStopScalesToZeroThenStops(t *testing.T) {
	l, sim, m, clk := newClockLoop(t, home, LoopConfig{})
	l.SetTarget(offsetTarget(t, m, home, r3.Vector{}, 0))
	l.SetVelocity([6]float64{0.05})
	clockTicks(t, l, clk, 25)
	n0 := len(sim.sends)
	l.RampStop(200*time.Millisecond, StateStopped)
	if !clockTicks(t, l, clk, 20) {
		t.Fatal("ramp did not complete within 20 ticks")
	}
	if n := len(sim.sends) - n0; n > 10 {
		t.Fatalf("ramp sent %d ticks, want ≤ 10 (200 ms)", n)
	}
	steps := sim.sends[n0-1:]
	first := maxAbsDiff(steps[1], steps[0])
	prev := first
	for k := 2; k < len(steps); k++ {
		d := maxAbsDiff(steps[k], steps[k-1])
		if d > prev*1.02+1e-12 {
			t.Fatalf("ramp step %d grew: %.3g > %.3g", k, d, prev)
		}
		prev = d
	}
	if prev > 0.15*first {
		t.Fatalf("last ramp step %.3g is not near zero (first %.3g)", prev, first)
	}
	if s := l.Status().State; !sim.stopped || s != StateStopped {
		t.Fatalf("stopped = %v state = %s, want stopped", sim.stopped, s)
	}
}

func TestRampStopWhileIdleAndUpgradeToStopped(t *testing.T) {
	l, sim, _, clk := newClockLoop(t, home, LoopConfig{})
	l.RampStop(50*time.Millisecond, StateStoppedTimeout)
	l.RampStop(time.Hour, StateStopped) // keeps the first deadline, upgrades the final state
	if !clockTicks(t, l, clk, 5) {
		t.Fatal("idle ramp did not complete")
	}
	if s := l.Status().State; !sim.stopped || s != StateStopped || len(sim.sends) != 0 {
		t.Fatalf("stopped = %v state = %s sends = %d, want stopped, stopped, 0", sim.stopped, s, len(sim.sends))
	}
}

func TestRampStopKeepsTimeoutState(t *testing.T) {
	l, sim, _, clk := newClockLoop(t, home, LoopConfig{})
	l.RampStop(50*time.Millisecond, StateStoppedTimeout)
	l.RampStop(time.Hour, StateStoppedTimeout)
	clockTicks(t, l, clk, 5)
	if s := l.Status().State; !sim.stopped || s != StateStoppedTimeout {
		t.Fatalf("state = %s, want stopped_timeout", s)
	}
}

func TestRunReturnsNilAfterRampStop(t *testing.T) {
	l, sim, m := newTestLoop(t, home, LoopConfig{})
	l.SetTarget(offsetTarget(t, m, home, r3.Vector{X: 10}, 0))
	go func() {
		time.Sleep(100 * time.Millisecond)
		l.RampStop(100*time.Millisecond, StateStopped)
	}()
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	if err := l.Run(ctx); err != nil {
		t.Fatalf("Run = %v, want nil after a ramp-stop", err)
	}
	if s := l.Status().State; !sim.stopped || s != StateStopped {
		t.Fatalf("stopped = %v state = %s", sim.stopped, s)
	}
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `go test ./... -run RampStop`
Expected: a build failure, because `l.RampStop` is undefined.

- [ ] **Step 3: Implement**

Add to `loop.go`, after `SetVelocity`:

```go
// RampStop scales the commanded velocity linearly to zero over ramp, then stops the arm and ends
// Run (returning nil) with state final. The target stops advancing and SetMotion/SetVelocity are
// ignored from the call on. A second call keeps the first deadline and only upgrades final to StateStopped.
func (l *Loop) RampStop(ramp time.Duration, final State) {
	l.mu.Lock()
	defer l.mu.Unlock()
	if l.rampStart.IsZero() {
		l.rampStart, l.rampDur, l.rampFinal = l.now(), ramp, final
		return
	}
	if final == StateStopped {
		l.rampFinal = final
	}
}
```

Task 1 already did the rest: `rampScale`, the step 0 completion in `tick`, the `errRampDone` handling in `Run`, and the `.Mul(scale)` on `lin` and `ang`. Delete the line `// Task 2 adds RampStop, which sets rampStart.` from `rampScale`'s comment.

- [ ] **Step 4: Run all tests**

Run: `go test -race ./...`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
gofmt -l . && go vet ./...
git add loop.go loop_test.go
git commit -m "Loop RampStop: linear velocity ramp to zero, then stop and end Run with a final state

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_012FAJrCqoTNZgDVMfHN6sDY"
```

---

### Task 3: Target-velocity estimator

**Files:**
- Create: `session.go` (only the estimator for now)
- Create: `session_test.go`

- [ ] **Step 1: Write the failing tests**

Create `session_test.go`:

```go
package diffik

import (
	"math"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

var t0 = time.Unix(1000, 0)

func at(ms float64) time.Time { return t0.Add(time.Duration(ms * float64(time.Millisecond))) }

// poseX is a pose at x mm along base x with a fixed orientation.
func poseX(x float64) spatialmath.Pose {
	return spatialmath.NewPose(r3.Vector{X: x, Y: 400, Z: 300}, &spatialmath.OrientationVectorDegrees{OZ: 1})
}

func newEst() *velEstimator { return &velEstimator{maxLin: 0.1, maxAng: deg2rad(45)} }

func near(a, b float64) bool { return math.Abs(a-b) <= 0.01*math.Abs(b)+1e-12 }

func TestEstimatorConstantVelocityFromSecondSample(t *testing.T) {
	e := newEst()
	if v := e.update(at(0), poseX(0)); v != ([6]float64{}) {
		t.Fatalf("first sample v = %v, want 0", v)
	}
	if v := e.update(at(20), poseX(1)); !near(v[0], 0.05) { // 1 mm / 20 ms = 50 mm/s
		t.Fatalf("second sample v = %.4f m/s, want 0.05", v[0])
	}
	if v := e.update(at(40), poseX(2)); !near(v[0], 0.05) {
		t.Fatalf("third sample v = %.4f m/s, want 0.05", v[0])
	}
}

func TestEstimatorStepResets(t *testing.T) {
	for _, gap := range []float64{100, 130} {
		e := newEst()
		e.update(at(0), poseX(0))
		e.update(at(gap), poseX(0))
		if v := e.update(at(2*gap), poseX(20)); v != ([6]float64{}) { // ≥ 153 mm/s > 1.5 × 100
			t.Fatalf("gap %v ms: 20 mm step gave v = %v, want 0", gap, v)
		}
	}
}

func TestEstimatorGapResets(t *testing.T) {
	e := newEst()
	e.update(at(0), poseX(0))
	e.update(at(20), poseX(1))
	if v := e.update(at(620), poseX(2)); v != ([6]float64{}) {
		t.Fatalf("after a 600 ms gap v = %v, want 0", v)
	}
}

func TestEstimatorIgnoresCloseSamples(t *testing.T) {
	e := newEst()
	e.update(at(0), poseX(0))
	e.update(at(20), poseX(1))
	if v := e.update(at(23), poseX(5)); !near(v[0], 0.05) { // 3 ms later: ignored, v kept
		t.Fatalf("close sample changed v to %.4f", v[0])
	}
	if v := e.update(at(40), poseX(2)); !near(v[0], 0.05) { // differentiated against the 20 ms sample
		t.Fatalf("after ignored sample v = %.4f, want 0.05", v[0])
	}
}

func TestEstimatorStationarySnapsAndReseeds(t *testing.T) {
	e := newEst()
	e.update(at(0), poseX(0))
	e.update(at(20), poseX(1))
	if v := e.update(at(40), poseX(1)); v != ([6]float64{}) {
		t.Fatalf("resent pose v = %v, want exactly 0", v)
	}
	if v := e.update(at(60), poseX(2)); !near(v[0], 0.05) { // reseeded, not EMA-ramped from 0
		t.Fatalf("after stationary v = %.4f, want 0.05", v[0])
	}
}

func TestEstimatorAngular(t *testing.T) {
	e := newEst()
	q0 := spatialmath.NewPose(r3.Vector{}, &spatialmath.OrientationVectorDegrees{OZ: 1})
	q1 := spatialmath.NewPose(r3.Vector{}, &spatialmath.OrientationVectorDegrees{OZ: 1, Theta: 0.2}) // 0.2° about z
	e.update(at(0), q0)
	if v := e.update(at(20), q1); !near(v[5], deg2rad(10)) { // 0.2° / 20 ms = 10 °/s
		t.Fatalf("angular v = %.5f rad/s, want %.5f", v[5], deg2rad(10))
	}
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `go test ./... -run Estimator`
Expected: a build failure, because `velEstimator` is undefined.

- [ ] **Step 3: Implement**

Create `session.go`:

```go
package diffik

import (
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"gonum.org/v1/gonum/num/quat"
)

// velEstimator estimates a pose-mode target's velocity from successive servo_target poses
// (spec "Target-velocity estimator"). maxLin and maxAng are the session's limits in m/s and rad/s.
type velEstimator struct {
	maxLin, maxAng float64

	have   bool
	lastT  time.Time
	lastP  r3.Vector // m
	lastQ  quat.Number
	v      [6]float64
	seeded bool
}

// update records pose p (mm, base frame) received at t and returns the velocity estimate (m/s, rad/s).
func (e *velEstimator) update(t time.Time, p spatialmath.Pose) [6]float64 {
	pm, q := p.Point().Mul(1/mmPerM), p.Orientation().Quaternion()
	dt := t.Sub(e.lastT).Seconds()
	if e.have && dt < 0.005 {
		return e.v // too close to differentiate; keep the earlier sample as the reference
	}
	if !e.have || dt > 0.5 {
		e.v, e.seeded = [6]float64{}, false // a jump, not motion
	} else {
		lin, ang := pm.Sub(e.lastP).Mul(1/dt), RotVec(q, e.lastQ).Mul(1/dt)
		raw := [6]float64{lin.X, lin.Y, lin.Z, ang.X, ang.Y, ang.Z}
		switch {
		case lin.Norm() < 0.001 && ang.Norm() < deg2rad(0.5):
			e.v, e.seeded = [6]float64{}, false // a resent pose: stationary
		case lin.Norm() > 1.5*e.maxLin || ang.Norm() > 1.5*e.maxAng:
			e.v, e.seeded = [6]float64{}, false // a step, not motion
		case !e.seeded:
			e.v, e.seeded = raw, true
		default:
			for i := range e.v {
				e.v[i] = 0.5*raw[i] + 0.5*e.v[i]
			}
		}
	}
	e.have, e.lastT, e.lastP, e.lastQ = true, t, pm, q
	return e.v
}
```

- [ ] **Step 4: Run all tests**

Run: `go test -race ./...`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
gofmt -l . && go vet ./...
git add session.go session_test.go
git commit -m "Add pose-mode target-velocity estimator

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_012FAJrCqoTNZgDVMfHN6sDY"
```

---

### Task 4: Sessions and the session manager

**Files:**
- Modify: `session.go`
- Modify: `session_test.go`

- [ ] **Step 1: Write the failing tests**

Append to `session_test.go`, merging these into its imports: `"context"`, `"errors"`, `"fmt"`, `"slices"`, `"strings"`, `"go.viam.com/rdk/components/arm"`, `"go.viam.com/rdk/components/arm/fake"`, `"go.viam.com/rdk/components/arm/kinematics"`, `"go.viam.com/rdk/logging"`, `"go.viam.com/rdk/referenceframe"` and `"go.viam.com/rdk/resource"`.

```go
// fakeFS stands in for the frame system: arm_origin sits at twb in world.
type fakeFS struct{ twb spatialmath.Pose }

func (f fakeFS) TransformPose(_ context.Context, p *referenceframe.PoseInFrame, dst string,
	_ []*referenceframe.LinkInFrame,
) (*referenceframe.PoseInFrame, error) {
	if p.Parent() != "arm_origin" || dst != referenceframe.World {
		return nil, fmt.Errorf("unexpected transform %s → %s", p.Parent(), dst)
	}
	return referenceframe.NewPoseInFrame(referenceframe.World, spatialmath.Compose(f.twb, p.Pose())), nil
}

func fakeUR5e(t *testing.T) arm.Arm {
	t.Helper()
	cfg := resource.Config{Name: "arm", ConvertedAttributes: &fake.Config{ArmModel: kinematics.UR5e}}
	a, err := fake.NewArm(context.Background(), nil, cfg, logging.NewTestLogger(t))
	if err != nil {
		t.Fatal(err)
	}
	return a
}

// newTestManager returns a manual-tick manager on a fake clock whose sessions drive sim.
func newTestManager(t *testing.T, twb spatialmath.Pose) (*servoManager, *simIO, *fakeClock) {
	t.Helper()
	sim := &simIO{q: slices.Clone(home)}
	clk := newFakeClock()
	m := newServoManager(map[string]arm.Arm{"arm": fakeUR5e(t)}, fakeFS{twb})
	m.now, m.manual = clk.Now, true
	m.newIO = func(arm.Arm, float64) JointIO { return sim }
	return m, sim, clk
}

func testSessionCfg(mode Mode) SessionConfig {
	return SessionConfig{Frame: "arm_origin", Mode: mode, Loop: LoopConfig{RateHz: 50},
		Timeout: time.Second, Ramp: 200 * time.Millisecond}
}

// step runs one tick the way Run would, including OnTick and the end-of-session hand-off.
func step(t *testing.T, s *session, clk *fakeClock, n int) {
	t.Helper()
	dt := 1 / s.loop.cfg.RateHz
	for i := 0; i < n; i++ {
		clk.Advance(time.Duration(dt * float64(time.Second)))
		err := s.loop.tick(context.Background(), dt)
		switch {
		case errors.Is(err, errRampDone):
			s.finish(s.loop.stopErr)
			return
		case err != nil:
			t.Fatal(err)
		}
		s.loop.OnTick(s.loop.Status())
	}
}

func mustStart(t *testing.T, m *servoManager, cfg SessionConfig) (*session, spatialmath.Pose) {
	t.Helper()
	s, p, err := m.start(context.Background(), "arm", cfg)
	if err != nil {
		t.Fatal(err)
	}
	return s, p
}

func wantErr(t *testing.T, err error, substr string) {
	t.Helper()
	if err == nil || !strings.Contains(err.Error(), substr) {
		t.Fatalf("err = %v, want containing %q", err, substr)
	}
}

func TestSessionStartHoldsCurrentPose(t *testing.T) {
	m, sim, clk := newTestManager(t, spatialmath.NewZeroPose())
	s, p := mustStart(t, m, testSessionCfg(ModePose))
	fk, err := ur5e(t).Transform(home)
	if err != nil {
		t.Fatal(err)
	}
	if p.Point().Sub(fk.Point()).Norm() > 1e-6 {
		t.Fatalf("start pose %v, want FK(home) %v", p.Point(), fk.Point())
	}
	step(t, s, clk, 10)
	if d := maxAbsDiff(sim.q, home); d > 1e-9 || s.status().State != StateConverged {
		t.Fatalf("arm moved %.3g rad / state %s, want still and converged", d, s.status().State)
	}
}

func TestSessionBusyUnknownArmBadFrameAndMode(t *testing.T) {
	m, _, _ := newTestManager(t, spatialmath.NewZeroPose())
	mustStart(t, m, testSessionCfg(ModePose))
	_, _, err := m.start(context.Background(), "arm", testSessionCfg(ModePose))
	wantErr(t, err, "busy")
	if !m.busy("arm") {
		t.Fatal("busy(arm) = false with a running session")
	}
	_, _, err = m.start(context.Background(), "nope", testSessionCfg(ModePose))
	wantErr(t, err, "not in this service's arms")

	m2, _, _ := newTestManager(t, spatialmath.NewZeroPose())
	cfg := testSessionCfg(ModePose)
	cfg.Frame = "arm" // the end-effector frame, not the base
	_, _, err = m2.start(context.Background(), "arm", cfg)
	wantErr(t, err, "unsupported frame")
	cfg = testSessionCfg("fly")
	_, _, err = m2.start(context.Background(), "arm", cfg)
	wantErr(t, err, "unsupported mode")
	_, err = m2.get("missing")
	wantErr(t, err, "unknown session_id")
}

func TestSessionLatestWins(t *testing.T) {
	m, _, clk := newTestManager(t, spatialmath.NewZeroPose())
	s, p := mustStart(t, m, testSessionCfg(ModePose))
	p1 := spatialmath.NewPose(p.Point().Add(r3.Vector{X: 30}), p.Orientation())
	p2 := spatialmath.NewPose(p.Point().Add(r3.Vector{Y: 30}), p.Orientation())
	for _, q := range []spatialmath.Pose{p1, p2} {
		if _, err := s.target(q); err != nil {
			t.Fatal(err)
		}
	}
	for i := 0; i < 30; i++ { // 3 s, resending p2 every 100 ms to feed the dead-man
		step(t, s, clk, 5)
		if _, err := s.target(p2); err != nil {
			t.Fatal(err)
		}
	}
	st := s.status()
	if st.TargetMM.Sub(p2.Point()).Norm() > 1e-6 || st.State != StateConverged {
		t.Fatalf("target %v state %s, want p2 %v and converged", st.TargetMM, st.State, p2.Point())
	}
}

func TestSessionWrongModeAndTwistLimits(t *testing.T) {
	m, _, _ := newTestManager(t, spatialmath.NewZeroPose())
	s, p := mustStart(t, m, testSessionCfg(ModePose))
	_, err := s.twist(r3.Vector{}, r3.Vector{})
	wantErr(t, err, "wrong mode: session is pose")

	m2, _, _ := newTestManager(t, spatialmath.NewZeroPose())
	tw, _ := mustStart(t, m2, testSessionCfg(ModeTwist))
	_, err = tw.target(p)
	wantErr(t, err, "wrong mode: session is twist")
	_, err = tw.twist(r3.Vector{X: 0.2}, r3.Vector{}) // 200 mm/s > 100 mm/s
	wantErr(t, err, "exceeds")
	_, err = tw.twist(r3.Vector{}, r3.Vector{Z: deg2rad(60)}) // 60 °/s > 45 °/s
	wantErr(t, err, "exceeds")
}

func TestSessionTwistKeepsAdvancedTarget(t *testing.T) {
	m, _, clk := newTestManager(t, spatialmath.NewZeroPose())
	s, p := mustStart(t, m, testSessionCfg(ModeTwist))
	for i := 0; i < 2; i++ {
		if _, err := s.twist(r3.Vector{X: 0.02}, r3.Vector{}); err != nil {
			t.Fatal(err)
		}
		step(t, s, clk, 25)
	}
	if d := s.status().TargetMM.X - p.Point().X; math.Abs(d-20) > 0.05 {
		t.Fatalf("target moved %.3f mm over two 0.5 s twists at 20 mm/s, want 20 (no snap-back)", d)
	}
}

func TestSessionHeldPoseConverges(t *testing.T) {
	m, _, clk := newTestManager(t, spatialmath.NewZeroPose())
	s, p := mustStart(t, m, testSessionCfg(ModePose))
	goal := spatialmath.NewPose(p.Point().Add(r3.Vector{Z: 20}), p.Orientation())
	for i := 0; i < 30; i++ {
		if _, err := s.target(goal); err != nil {
			t.Fatal(err)
		}
		step(t, s, clk, 5)
	}
	if st := s.status(); st.State != StateConverged {
		t.Fatalf("state = %s err = %.2f mm, want converged on a resent pose", st.State, st.ErrMM)
	}
}

func TestSessionDeadmanTimeout(t *testing.T) {
	m, sim, clk := newTestManager(t, spatialmath.NewZeroPose())
	cfg := testSessionCfg(ModePose)
	cfg.Timeout, cfg.Ramp = 100*time.Millisecond, 100*time.Millisecond
	s, p := mustStart(t, m, cfg)
	ticks := 0
	for !s.ended() && ticks < 100 {
		step(t, s, clk, 1)
		ticks++
	}
	// timeout + 2 ticks + ramp = 100 + 40 + 100 ms = 12 ticks
	if elapsed := time.Duration(ticks) * 20 * time.Millisecond; elapsed > 240*time.Millisecond {
		t.Fatalf("dead-man stop took %v, want ≤ 240 ms", elapsed)
	}
	st := s.status()
	if st.State != StateStoppedTimeout || !sim.stopped {
		t.Fatalf("state = %s stopped = %v, want stopped_timeout and stopped", st.State, sim.stopped)
	}
	_, err := s.target(p)
	wantErr(t, err, "session ended: stopped_timeout")
	if m.busy("arm") {
		t.Fatal("arm still busy after the session ended")
	}
	if _, _, err := m.start(context.Background(), "arm", cfg); err != nil {
		t.Fatalf("restart after timeout: %v", err)
	}
}

func TestSessionWorldFrameMapping(t *testing.T) {
	rz := spatialmath.Quaternion(rotZ(math.Pi / 2))
	twb := spatialmath.NewPose(r3.Vector{X: 100, Y: 200, Z: 300}, &rz)
	m, _, clk := newTestManager(t, twb)
	cfg := testSessionCfg(ModePose)
	cfg.Frame = "world"
	s, pw := mustStart(t, m, cfg)
	fk, _ := ur5e(t).Transform(home)
	if pw.Point().Sub(spatialmath.Compose(twb, fk).Point()).Norm() > 1e-6 {
		t.Fatalf("start pose in world %v, want %v", pw.Point(), spatialmath.Compose(twb, fk).Point())
	}
	pb := spatialmath.NewPose(fk.Point().Add(r3.Vector{X: 10}), fk.Orientation())
	if _, err := s.target(spatialmath.Compose(twb, pb)); err != nil {
		t.Fatal(err)
	}
	step(t, s, clk, 1) // Status.TargetMM is published by tick; v̂ = 0 on a first sample, so the target doesn't move
	if got := s.status().TargetMM; got.Sub(pb.Point()).Norm() > 1e-6 {
		t.Fatalf("base-frame target %v, want %v", got, pb.Point())
	}

	m2, _, clk2 := newTestManager(t, twb)
	cfg.Mode = ModeTwist
	s2, _ := mustStart(t, m2, cfg)
	step(t, s2, clk2, 1)
	before := s2.status().TargetMM
	if _, err := s2.twist(r3.Vector{X: 0.02}, r3.Vector{}); err != nil { // world +x = base −y under Rz(90°)
		t.Fatal(err)
	}
	step(t, s2, clk2, 50)
	d := s2.status().TargetMM.Sub(before)
	if math.Abs(d.Y+20) > 0.05 || math.Abs(d.X) > 0.05 {
		t.Fatalf("base-frame target moved %v mm, want (0, −20, 0)", d)
	}
}

// Real clock and real Run goroutine: servo_stop ramps, stops the arm and ends the session.
func TestSessionStopRealClock(t *testing.T) {
	sim := &simIO{q: slices.Clone(home)}
	m := newServoManager(map[string]arm.Arm{"arm": fakeUR5e(t)}, fakeFS{spatialmath.NewZeroPose()})
	m.newIO = func(arm.Arm, float64) JointIO { return sim }
	s, _ := mustStart(t, m, testSessionCfg(ModePose))
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	st, err := s.stop(ctx)
	if err != nil {
		t.Fatal(err)
	}
	if st.State != StateStopped || !sim.stopped {
		t.Fatalf("state = %s stopped = %v, want stopped", st.State, sim.stopped)
	}
	_, err = s.stop(ctx)
	wantErr(t, err, "session ended: stopped")
	if err := m.close(ctx); err != nil {
		t.Fatal(err)
	}
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `go test ./... -run Session`
Expected: a build failure, because `servoManager`, `session`, `SessionConfig`, `Mode` and the others are undefined.

- [ ] **Step 3: Implement**

Replace the import block of `session.go` with the one below. Then append everything after it to the file. The estimator from Task 3 stays.

```go
import (
	"context"
	"crypto/rand"
	"errors"
	"fmt"
	"maps"
	"slices"
	"sync"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"gonum.org/v1/gonum/num/quat"
)

// Mode is a servo session's command mode, fixed for the session's life.
type Mode string

// Session modes.
const (
	ModePose  Mode = "pose"
	ModeTwist Mode = "twist"
)

var errUnknownSession = errors.New("unknown session_id")

// poseTransformer is the part of framesystem.Service a session needs.
type poseTransformer interface {
	TransformPose(ctx context.Context, pose *referenceframe.PoseInFrame, dst string,
		supplementalTransforms []*referenceframe.LinkInFrame) (*referenceframe.PoseInFrame, error)
}

// SessionConfig is a parsed servo_start request. Loop.RateHz must be set. A one-element
// Loop.MaxJointDegPerS applies to every joint.
type SessionConfig struct {
	Frame   string // "world" or "<arm>_origin"
	Mode    Mode
	Loop    LoopConfig
	Timeout time.Duration // dead-man
	Ramp    time.Duration
}

// SessionStatus is a snapshot of a session, live or final.
type SessionStatus struct {
	Status
	Config     LoopConfig
	LastCmdAge time.Duration
}

// session is one arm's servo session: a loop plus the protocol state around it.
type session struct {
	id            string
	mode          Mode
	loop          *Loop
	twb           spatialmath.Pose // the arm base's pose in the session frame
	now           func() time.Time
	timeout, ramp time.Duration
	done          chan struct{} // closed once the loop has ended and final is set
	cancel        context.CancelFunc

	mu      sync.Mutex
	est     velEstimator
	lastCmd time.Time
	fired   bool
	final   SessionStatus
	runErr  error
}

func (s *session) run(ctx context.Context) { s.finish(s.loop.Run(ctx)) }

// finish records the final status and marks the session ended.
func (s *session) finish(err error) {
	st := s.status()
	s.mu.Lock()
	s.final, s.runErr = st, err
	s.mu.Unlock()
	s.cancel() // release the run context
	close(s.done)
}

func (s *session) ended() bool {
	select {
	case <-s.done:
		return true
	default:
		return false
	}
}

// onTick is the loop's OnTick: the dead-man check, once per tick.
func (s *session) onTick(Status) {
	s.mu.Lock()
	fire := !s.fired && s.now().Sub(s.lastCmd) > s.timeout
	if fire {
		s.fired = true
	}
	s.mu.Unlock()
	if fire {
		s.loop.RampStop(s.ramp, StateStoppedTimeout)
	}
}

func (s *session) usable(m Mode) error {
	if s.ended() {
		return fmt.Errorf("session ended: %s", s.status().State)
	}
	if s.mode != m {
		return fmt.Errorf("wrong mode: session is %s", s.mode)
	}
	return nil
}

// target handles servo_target: p is in the session frame, in mm.
func (s *session) target(p spatialmath.Pose) (SessionStatus, error) {
	if err := s.usable(ModePose); err != nil {
		return SessionStatus{}, err
	}
	pb := spatialmath.Compose(spatialmath.PoseInverse(s.twb), p)
	s.mu.Lock()
	now := s.now()
	v := s.est.update(now, pb)
	s.lastCmd = now
	s.mu.Unlock()
	s.loop.SetMotion(pb, v)
	return s.status(), nil
}

// twist handles servo_twist: lin (m/s) and ang (rad/s) are in the session frame.
func (s *session) twist(lin, ang r3.Vector) (SessionStatus, error) {
	if err := s.usable(ModeTwist); err != nil {
		return SessionStatus{}, err
	}
	cfg := s.loop.Config()
	if lin.Norm() > cfg.MaxLinearMMPerS/mmPerM || ang.Norm() > deg2rad(cfg.MaxAngularDegPerS) {
		return SessionStatus{}, fmt.Errorf("twist exceeds limits (%.0f mm/s, %.0f deg/s)",
			cfg.MaxLinearMMPerS, cfg.MaxAngularDegPerS)
	}
	rinv := spatialmath.PoseInverse(spatialmath.NewPose(r3.Vector{}, s.twb.Orientation()))
	lb := spatialmath.Compose(rinv, spatialmath.NewPoseFromPoint(lin)).Point()
	ab := spatialmath.Compose(rinv, spatialmath.NewPoseFromPoint(ang)).Point()
	s.mu.Lock()
	s.lastCmd = s.now()
	s.mu.Unlock()
	s.loop.SetVelocity([6]float64{lb.X, lb.Y, lb.Z, ab.X, ab.Y, ab.Z})
	return s.status(), nil
}

// stop handles servo_stop: ramp to zero, stop the arm, wait for the loop to end.
func (s *session) stop(ctx context.Context) (SessionStatus, error) {
	if s.ended() {
		return SessionStatus{}, fmt.Errorf("session ended: %s", s.status().State)
	}
	s.loop.RampStop(s.ramp, StateStopped)
	select {
	case <-s.done:
		return s.status(), nil
	case <-ctx.Done():
		return SessionStatus{}, ctx.Err()
	}
}

func (s *session) status() SessionStatus {
	if s.ended() {
		s.mu.Lock()
		defer s.mu.Unlock()
		return s.final
	}
	s.mu.Lock()
	age := s.now().Sub(s.lastCmd)
	s.mu.Unlock()
	return SessionStatus{Status: s.loop.Status(), Config: s.loop.Config(), LastCmdAge: age}
}

// servoManager owns the sessions, at most one live session per arm.
type servoManager struct {
	arms   map[string]arm.Arm
	fs     poseTransformer
	now    func() time.Time
	newIO  func(arm.Arm, float64) JointIO
	manual bool // tests: don't start Run, drive ticks by hand

	mu    sync.Mutex
	byArm map[string]*session
}

func newServoManager(arms map[string]arm.Arm, fs poseTransformer) *servoManager {
	return &servoManager{arms: arms, fs: fs, now: time.Now, newIO: NewArmIO, byArm: map[string]*session{}}
}

// start opens a session on armName. It returns the session and the arm's current pose in cfg.Frame.
func (m *servoManager) start(ctx context.Context, armName string, cfg SessionConfig) (*session, spatialmath.Pose, error) {
	a, ok := m.arms[armName]
	if !ok {
		return nil, nil, fmt.Errorf("arm %q is not in this service's arms", armName)
	}
	if cfg.Mode != ModePose && cfg.Mode != ModeTwist {
		return nil, nil, fmt.Errorf("unsupported mode %q: use \"pose\" or \"twist\"", cfg.Mode)
	}
	// ponytail: one manager lock held across the start RPCs (frame system, Kinematics, Read), which blocks
	// other arms' commands during a start; use per-arm locks if multi-arm starts ever matter.
	m.mu.Lock()
	defer m.mu.Unlock()
	if s := m.byArm[armName]; s != nil && !s.ended() {
		return nil, nil, fmt.Errorf("arm %s busy", armName)
	}
	var twb spatialmath.Pose
	switch cfg.Frame {
	case referenceframe.World:
		pif, err := m.fs.TransformPose(ctx,
			referenceframe.NewPoseInFrame(armName+"_origin", spatialmath.NewZeroPose()), referenceframe.World, nil)
		if err != nil {
			return nil, nil, fmt.Errorf("locating %s_origin in world: %w", armName, err)
		}
		twb = pif.Pose()
	case armName + "_origin":
		twb = spatialmath.NewZeroPose()
	default:
		return nil, nil, fmt.Errorf("unsupported frame %q: use \"world\" or %q", cfg.Frame, armName+"_origin")
	}
	model, err := a.Kinematics(ctx)
	if err != nil {
		return nil, nil, err
	}
	if js := cfg.Loop.MaxJointDegPerS; len(js) == 1 {
		cfg.Loop.MaxJointDegPerS = slices.Repeat(js, len(model.DoF()))
	}
	io := m.newIO(a, cfg.Loop.RateHz)
	l, err := NewLoop(model, io, cfg.Loop)
	if err != nil {
		return nil, nil, err
	}
	l.Now = m.now
	q, err := io.Read(ctx)
	if err != nil {
		return nil, nil, err
	}
	fk, err := model.Transform(q) // mm; FK() would give metres
	if err != nil {
		return nil, nil, err
	}
	l.SetTarget(fk)
	runCtx, cancel := context.WithCancel(context.Background())
	s := &session{
		id: rand.Text(), mode: cfg.Mode, loop: l, twb: twb, now: m.now,
		timeout: cfg.Timeout, ramp: cfg.Ramp, done: make(chan struct{}), cancel: cancel,
		est:     velEstimator{maxLin: l.cfg.MaxLinearMMPerS / mmPerM, maxAng: deg2rad(l.cfg.MaxAngularDegPerS)},
		lastCmd: m.now(),
	}
	l.OnTick = s.onTick
	m.byArm[armName] = s
	if !m.manual {
		go s.run(runCtx)
	}
	return s, spatialmath.Compose(twb, fk), nil
}

func (m *servoManager) get(id string) (*session, error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	for _, s := range m.byArm {
		if s.id == id {
			return s, nil
		}
	}
	return nil, errUnknownSession
}

func (m *servoManager) busy(armName string) bool {
	m.mu.Lock()
	defer m.mu.Unlock()
	s := m.byArm[armName]
	return s != nil && !s.ended()
}

// stopArm ramp-stops armName's live session, if any (StopPlan).
func (m *servoManager) stopArm(ctx context.Context, armName string) error {
	m.mu.Lock()
	s := m.byArm[armName]
	m.mu.Unlock()
	if s == nil || s.ended() {
		return nil
	}
	_, err := s.stop(ctx)
	return err
}

// close ramp-stops every live session concurrently. A session that doesn't finish within its
// ramp + 2 s is cancelled, which stops the arm immediately.
func (m *servoManager) close(ctx context.Context) error {
	m.mu.Lock()
	ss := slices.Collect(maps.Values(m.byArm))
	m.mu.Unlock()
	var wg sync.WaitGroup
	for _, s := range ss {
		if s.ended() {
			continue
		}
		wg.Add(1)
		go func() {
			defer wg.Done()
			s.loop.RampStop(s.ramp, StateStopped)
			select {
			case <-s.done:
			case <-time.After(s.ramp + 2*time.Second):
				s.cancel()
				<-s.done
			case <-ctx.Done():
				s.cancel()
				<-s.done
			}
		}()
	}
	wg.Wait()
	return nil
}
```

`session.run` takes the ctx from `start`; `cancel` is the fallback that `close` uses.

- [ ] **Step 4: Run all tests**

Run: `go test -race ./...`
Expected: PASS. If the twist-frame test fails on sign, check `twist`: `rinv` must be the inverse of the base orientation in the session frame. The test's expectation comes from the spec: world +x maps to base −y under Rz(90°).

- [ ] **Step 5: Commit**

```bash
gofmt -l . && go vet ./...
git add session.go session_test.go
git commit -m "Add servo sessions: latest-wins targets, twists, dead-man, world frame, ramp-stop

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_012FAJrCqoTNZgDVMfHN6sDY"
```

---

### Task 5: Module surface — config, DoCommand, Move, StopPlan, Close

**Files:**
- Modify: `module.go`
- Create: `module_test.go`

- [ ] **Step 1: Write the failing tests**

Create `module_test.go`:

```go
package diffik

import (
	"context"
	"math"
	"slices"
	"strings"
	"testing"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	motion "go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
)

func testServo(t *testing.T, m *servoManager) *diffIkServo {
	t.Helper()
	return newServo(resource.NewName(motion.API, "servo"), logging.NewTestLogger(t), m)
}

func do(t *testing.T, s *diffIkServo, cmd map[string]interface{}) map[string]interface{} {
	t.Helper()
	r, err := s.DoCommand(context.Background(), cmd)
	if err != nil {
		t.Fatalf("%v: %v", cmd["command"], err)
	}
	return r
}

func TestValidate(t *testing.T) {
	deps, _, err := Config{Arms: []string{"a", "b"}}.Validate("services.0")
	if err != nil || !slices.Equal(deps, []string{"a", "b"}) {
		t.Fatalf("deps = %v err = %v", deps, err)
	}
	for _, arms := range [][]string{nil, {"a", "a"}, {""}} {
		if _, _, err := (Config{Arms: arms}).Validate("services.0"); err == nil {
			t.Errorf("arms %q: want error", arms)
		}
	}
}

func TestServoStartDefaultsAndScalarJointLimit(t *testing.T) {
	m, _, clk := newTestManager(t, spatialmath.NewZeroPose())
	s := testServo(t, m)
	r := do(t, s, map[string]interface{}{"command": "servo_start", "component_name": "arm",
		"frame": "arm_origin", "limits": map[string]interface{}{"joint_deg_s": 60.0}})
	id, ok := r["session_id"].(string)
	if !ok || id == "" {
		t.Fatalf("session_id = %v", r["session_id"])
	}
	fk, _ := ur5e(t).Transform(home)
	pm := r["pose"].(map[string]interface{})
	if math.Abs(pm["x"].(float64)-fk.Point().X) > 1e-6 {
		t.Fatalf("pose x = %v, want %v", pm["x"], fk.Point().X)
	}
	sess, err := m.get(id)
	if err != nil {
		t.Fatal(err)
	}
	step(t, sess, clk, 1) // manual manager: one tick so status has a state
	st := do(t, s, map[string]interface{}{"command": "servo_status", "session_id": id})
	lim := st["limits"].(map[string]interface{})
	js := lim["joint_deg_s"].([]interface{})
	if len(js) != 6 || js[0].(float64) != 60 || lim["linear_mm_s"].(float64) != 100 || lim["angular_deg_s"].(float64) != 45 {
		t.Fatalf("limits = %v, want 6×60, 100, 45", lim)
	}
	if st["state"] != "converged" {
		t.Fatalf("state = %v", st["state"])
	}
}

func TestDoCommandErrors(t *testing.T) {
	m, _, _ := newTestManager(t, spatialmath.NewZeroPose())
	s := testServo(t, m)
	start := func(kv ...interface{}) map[string]interface{} {
		c := map[string]interface{}{"command": "servo_start", "component_name": "arm", "frame": "arm_origin"}
		for i := 0; i < len(kv); i += 2 {
			c[kv[i].(string)] = kv[i+1]
		}
		return c
	}
	for _, tc := range []struct {
		cmd  map[string]interface{}
		want string
	}{
		{map[string]interface{}{}, "unknown command"},
		{map[string]interface{}{"command": "servo_fly"}, "unknown command"},
		{map[string]interface{}{"command": "servo_start"}, "component_name is required"},
		{start("component_name", "nope"), "not in this service's arms"},
		{start("frame", "arm"), "unsupported frame"},
		{start("mode", "fly"), "unsupported mode"},
		{start("timeout_ms", 0.0), "timeout_ms"},
		{start("timeout_ms", 20000.0), "timeout_ms"},
		{start("ramp_ms", -1.0), "ramp_ms"},
		{start("rate_hz", math.NaN()), "rate_hz"},
		{start("rate_hz", "fast"), "rate_hz"},
		{start("limits", map[string]interface{}{"linear_mm_s": -5.0}), "linear_mm_s"},
		{start("limits", "slow"), "limits"},
		{start("task_weights", []interface{}{1.0, 1.0, 1.0, 1.0, 1.0}), "task_weights"},
		{start("task_weights", []interface{}{1.0, 1.0, 1.0, 0.0, 1.0, 1.0}), "task_weights"},
		{map[string]interface{}{"command": "servo_status"}, "session_id is required"},
		{map[string]interface{}{"command": "servo_status", "session_id": "nope"}, "unknown session_id"},
	} {
		if _, err := s.DoCommand(context.Background(), tc.cmd); err == nil || !strings.Contains(err.Error(), tc.want) {
			t.Errorf("%v: err = %v, want containing %q", tc.cmd, err, tc.want)
		}
	}

	id := do(t, s, start())["session_id"].(string)
	pose := func(kv ...interface{}) map[string]interface{} {
		p := map[string]interface{}{"x": 100.0, "y": 400.0, "z": 300.0, "o_x": 0.0, "o_y": 0.0, "o_z": 1.0, "theta": 0.0}
		for i := 0; i < len(kv); i += 2 {
			p[kv[i].(string)] = kv[i+1]
		}
		return p
	}
	for _, tc := range []struct {
		cmd  map[string]interface{}
		want string
	}{
		{start(), "busy"},
		{map[string]interface{}{"command": "servo_target", "session_id": id}, "pose is required"},
		{map[string]interface{}{"command": "servo_target", "session_id": id, "pose": pose("o_z", 0.0)}, "orientation"},
		{map[string]interface{}{"command": "servo_target", "session_id": id, "pose": pose("x", "a")}, "x"},
		{map[string]interface{}{"command": "servo_target", "session_id": id, "pose": pose("y", math.Inf(1))}, "y"},
		{map[string]interface{}{"command": "servo_twist", "session_id": id,
			"linear": []interface{}{0.0, 0.0, 0.0}, "angular": []interface{}{0.0, 0.0, 0.0}}, "wrong mode"},
		{map[string]interface{}{"command": "servo_twist", "session_id": id, "linear": []interface{}{0.0}}, "linear"},
	} {
		if _, err := s.DoCommand(context.Background(), tc.cmd); err == nil || !strings.Contains(err.Error(), tc.want) {
			t.Errorf("%v: err = %v, want containing %q", tc.cmd, err, tc.want)
		}
	}
	if _, err := s.Move(context.Background(), motion.MoveReq{ComponentName: "arm"}); err == nil || !strings.Contains(err.Error(), "busy") {
		t.Fatalf("Move on busy arm: err = %v", err)
	}
}

func TestPoseRoundTrip(t *testing.T) {
	n := 1 / math.Sqrt(3)
	in := map[string]interface{}{"x": 12.5, "y": -40.0, "z": 300.0, "o_x": n, "o_y": n, "o_z": n, "theta": 33.0}
	p, err := parsePose(in)
	if err != nil {
		t.Fatal(err)
	}
	out := poseMap(p)
	for k, v := range in {
		if math.Abs(out[k].(float64)-v.(float64)) > 1e-9 {
			t.Errorf("%s: %v → %v", k, v, out[k])
		}
	}
	// a non-unit orientation vector is normalised, not rejected
	in["o_x"], in["o_y"], in["o_z"] = 0.0, 0.0, 2.0
	if p, err = parsePose(in); err != nil || math.Abs(poseMap(p)["o_z"].(float64)-1) > 1e-9 {
		t.Fatalf("non-unit vector: %v %v", poseMap(p), err)
	}
}

func TestServoTwistUnitsAndStopViaDoCommandAndStopPlan(t *testing.T) {
	sim := &simIO{q: slices.Clone(home)}
	m := newServoManager(map[string]arm.Arm{"arm": fakeUR5e(t)}, fakeFS{spatialmath.NewZeroPose()})
	m.newIO = func(arm.Arm, float64) JointIO { return sim }
	s := testServo(t, m)
	id := do(t, s, map[string]interface{}{"command": "servo_start", "component_name": "arm",
		"frame": "arm_origin", "mode": "twist"})["session_id"].(string)
	// unit conversion at the boundary: limits are 100 mm/s and 45 °/s
	for _, tc := range []struct {
		lin, ang []interface{}
		ok       bool
	}{
		{[]interface{}{99.0, 0.0, 0.0}, []interface{}{0.0, 0.0, 0.0}, true},
		{[]interface{}{101.0, 0.0, 0.0}, []interface{}{0.0, 0.0, 0.0}, false},
		{[]interface{}{0.0, 0.0, 0.0}, []interface{}{0.0, 0.0, 44.0}, true},
		{[]interface{}{0.0, 0.0, 0.0}, []interface{}{0.0, 0.0, 46.0}, false},
	} {
		_, err := s.DoCommand(context.Background(), map[string]interface{}{"command": "servo_twist",
			"session_id": id, "linear": tc.lin, "angular": tc.ang})
		if (err == nil) != tc.ok {
			t.Fatalf("twist %v %v: err = %v, want ok = %v", tc.lin, tc.ang, err, tc.ok)
		}
	}
	do(t, s, map[string]interface{}{"command": "servo_twist", "session_id": id,
		"linear": []interface{}{20.0, 0.0, 0.0}, "angular": []interface{}{0.0, 0.0, 0.0}}) // mm/s
	time.Sleep(300 * time.Millisecond)
	fin := do(t, s, map[string]interface{}{"command": "servo_stop", "session_id": id})
	if fin["state"] != "stopped" || !sim.stopped {
		t.Fatalf("servo_stop state = %v stopped = %v", fin["state"], sim.stopped)
	}
	_, err := s.DoCommand(context.Background(), map[string]interface{}{"command": "servo_stop", "session_id": id})
	if err == nil || !strings.Contains(err.Error(), "session ended: stopped") {
		t.Fatalf("second stop err = %v", err)
	}

	sim.stopped = false
	id = do(t, s, map[string]interface{}{"command": "servo_start", "component_name": "arm",
		"frame": "arm_origin"})["session_id"].(string)
	if err := s.StopPlan(context.Background(), motion.StopPlanReq{ComponentName: "arm"}); err != nil {
		t.Fatal(err)
	}
	if st := do(t, s, map[string]interface{}{"command": "servo_status", "session_id": id}); st["state"] != "stopped" || !sim.stopped {
		t.Fatalf("after StopPlan state = %v stopped = %v", st["state"], sim.stopped)
	}
	if err := s.Close(context.Background()); err != nil {
		t.Fatal(err)
	}
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `go test ./... -run 'Validate|ServoStart|DoCommand|PoseRoundTrip|ServoTwist'`
Expected: a build failure, because `newServo`, `parsePose`, `poseMap` and `Config.Arms` are undefined.

- [ ] **Step 3: Implement**

Rewrite `module.go`. Keep `init()` and every method on `diffIkServo` that isn't listed below unchanged: `MoveOnMap`, `MoveOnGlobe`, `GetPose`, `ListPlanStatuses`, `PlanHistory` and `TempStreamArmJointPositions`. `Status` is redefined in the block below; delete the old one. Replace the rest:

```go
package diffik

import (
	"context"
	"errors"
	"fmt"
	"math"
	"slices"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
	motion "go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
)

var (
	Servo            = resource.NewModel("devrel", "diff-ik", "servo")
	errUnimplemented = errors.New("unimplemented")
)

// init: unchanged.

// Config lists the arms this service may servo.
type Config struct {
	Arms []string `json:"arms"`
}

// Validate requires at least one arm and no duplicates, and returns the arms as dependencies.
// The frame system needs no declaration: the module manager always injects it.
func (cfg Config) Validate(path string) ([]string, []string, error) {
	if len(cfg.Arms) == 0 {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "arms")
	}
	seen := map[string]bool{}
	for _, a := range cfg.Arms {
		if a == "" || seen[a] {
			return nil, nil, fmt.Errorf("%s: arms must be unique, non-empty names", path)
		}
		seen[a] = true
	}
	return slices.Clone(cfg.Arms), nil, nil
}

type diffIkServo struct {
	resource.AlwaysRebuild

	name   resource.Name
	logger logging.Logger
	m      *servoManager
}

func newDiffIkServo(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (motion.Service, error) {
	conf, err := resource.NativeConfig[Config](rawConf)
	if err != nil {
		return nil, err
	}
	return NewServo(ctx, deps, rawConf.ResourceName(), conf, logger)
}

// NewServo builds the service from its dependencies: each configured arm and the injected frame system.
func NewServo(ctx context.Context, deps resource.Dependencies, name resource.Name, conf Config, logger logging.Logger) (motion.Service, error) {
	arms := map[string]arm.Arm{}
	for _, n := range conf.Arms {
		a, err := arm.FromDependencies(deps, n)
		if err != nil {
			return nil, err
		}
		arms[n] = a
	}
	fs, err := framesystem.FromDependencies(deps)
	if err != nil {
		return nil, err
	}
	return newServo(name, logger, newServoManager(arms, fs)), nil
}

func newServo(name resource.Name, logger logging.Logger, m *servoManager) *diffIkServo {
	return &diffIkServo{name: name, logger: logger, m: m}
}

func (s *diffIkServo) Name() resource.Name { return s.name }

// Move is not implemented; it refuses an arm that has a live servo session.
func (s *diffIkServo) Move(ctx context.Context, req motion.MoveReq) (bool, error) {
	if s.m.busy(req.ComponentName) {
		return false, fmt.Errorf("arm %s busy", req.ComponentName)
	}
	return false, errUnimplemented
}

// StopPlan ramp-stops the servo session on req.ComponentName, if any.
func (s *diffIkServo) StopPlan(ctx context.Context, req motion.StopPlanReq) error {
	return s.m.stopArm(ctx, req.ComponentName)
}

func (s *diffIkServo) Close(ctx context.Context) error { return s.m.close(ctx) }

// DoCommand implements the servo protocol. See docs/superpowers/specs/2026-11-15-diff-ik-phase2-design.md.
func (s *diffIkServo) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	name, _ := cmd["command"].(string)
	if name == "servo_start" {
		return s.servoStart(ctx, cmd)
	}
	switch name {
	case "servo_target", "servo_twist", "servo_stop", "servo_status":
	default:
		return nil, fmt.Errorf("unknown command %q", name)
	}
	id, ok := cmd["session_id"].(string)
	if !ok {
		return nil, errors.New("session_id is required")
	}
	sess, err := s.m.get(id)
	if err != nil {
		return nil, err
	}
	switch name {
	case "servo_target":
		pm, ok := cmd["pose"].(map[string]interface{})
		if !ok {
			return nil, errors.New("pose is required")
		}
		p, err := parsePose(pm)
		if err != nil {
			return nil, err
		}
		st, err := sess.target(p)
		if err != nil {
			return nil, err
		}
		return map[string]interface{}{"state": string(st.State), "err_mm": st.ErrMM, "err_deg": st.ErrDeg}, nil
	case "servo_twist":
		lin, err := vec3(cmd, "linear")
		if err != nil {
			return nil, err
		}
		ang, err := vec3(cmd, "angular")
		if err != nil {
			return nil, err
		}
		st, err := sess.twist(lin.Mul(1/mmPerM), ang.Mul(math.Pi/180))
		if err != nil {
			return nil, err
		}
		return map[string]interface{}{"state": string(st.State)}, nil
	case "servo_stop":
		st, err := sess.stop(ctx)
		if err != nil {
			return nil, err
		}
		return statusMap(st), nil
	default: // servo_status
		return statusMap(sess.status()), nil
	}
}

func (s *diffIkServo) servoStart(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	armName, ok := cmd["component_name"].(string)
	if !ok || armName == "" {
		return nil, errors.New("component_name is required")
	}
	frame, err := optString(cmd, "frame", referenceframe.World)
	if err != nil {
		return nil, err
	}
	mode, err := optString(cmd, "mode", string(ModePose))
	if err != nil {
		return nil, err
	}
	rate, err := optNum(cmd, "rate_hz", 50)
	if err != nil {
		return nil, err
	}
	if !(rate > 0) {
		return nil, fmt.Errorf("rate_hz must be > 0, got %v", rate)
	}
	timeout, err := optNum(cmd, "timeout_ms", 500)
	if err != nil {
		return nil, err
	}
	if !(timeout > 0 && timeout <= 10000) {
		return nil, fmt.Errorf("timeout_ms must be in (0, 10000], got %v", timeout)
	}
	ramp, err := optNum(cmd, "ramp_ms", 200)
	if err != nil {
		return nil, err
	}
	if !(ramp > 0) {
		return nil, fmt.Errorf("ramp_ms must be > 0, got %v", ramp)
	}
	lc := LoopConfig{RateHz: rate}
	if raw, ok := cmd["limits"]; ok {
		lm, ok := raw.(map[string]interface{})
		if !ok {
			return nil, errors.New("limits must be an object")
		}
		if v, ok := lm["joint_deg_s"]; ok {
			if lc.MaxJointDegPerS, err = numOrList(v, "limits.joint_deg_s"); err != nil {
				return nil, err
			}
		}
		for _, f := range []struct {
			key string
			dst *float64
		}{
			{"linear_mm_s", &lc.MaxLinearMMPerS},
			{"angular_deg_s", &lc.MaxAngularDegPerS},
			{"resync_deg", &lc.ResyncDeg},
		} {
			if v, ok := lm[f.key]; ok {
				n, err := num(v, "limits."+f.key)
				if err != nil {
					return nil, err
				}
				if !(n > 0) {
					return nil, fmt.Errorf("limits.%s must be > 0, got %v", f.key, n)
				}
				*f.dst = n
			}
		}
	}
	if v, ok := cmd["task_weights"]; ok {
		ws, err := numOrList(v, "task_weights")
		if err != nil || len(ws) != 6 {
			return nil, fmt.Errorf("task_weights must be 6 numbers > 0, got %v", v)
		}
		copy(lc.TaskWeights[:], ws)
	}
	sess, pose, err := s.m.start(ctx, armName, SessionConfig{
		Frame: frame, Mode: Mode(mode), Loop: lc,
		Timeout: time.Duration(timeout * float64(time.Millisecond)),
		Ramp:    time.Duration(ramp * float64(time.Millisecond)),
	})
	if err != nil {
		return nil, err
	}
	return map[string]interface{}{"session_id": sess.id, "pose": poseMap(pose)}, nil
}

func (s *diffIkServo) Status(ctx context.Context) (map[string]interface{}, error) {
	return nil, errUnimplemented
}

// num converts a DoCommand number (float64 from protobuf, int in Go callers) and rejects NaN/Inf.
func num(v interface{}, key string) (float64, error) {
	var f float64
	switch n := v.(type) {
	case float64:
		f = n
	case int:
		f = float64(n)
	case int64:
		f = float64(n)
	default:
		return 0, fmt.Errorf("%s must be a number, got %v", key, v)
	}
	if math.IsNaN(f) || math.IsInf(f, 0) {
		return 0, fmt.Errorf("%s must be finite, got %v", key, f)
	}
	return f, nil
}

func optNum(m map[string]interface{}, key string, def float64) (float64, error) {
	v, ok := m[key]
	if !ok {
		return def, nil
	}
	return num(v, key)
}

func optString(m map[string]interface{}, key, def string) (string, error) {
	v, ok := m[key]
	if !ok {
		return def, nil
	}
	s, ok := v.(string)
	if !ok {
		return "", fmt.Errorf("%s must be a string, got %v", key, v)
	}
	return s, nil
}

// numOrList parses a number or a list of numbers, each > 0.
func numOrList(v interface{}, key string) ([]float64, error) {
	items, ok := v.([]interface{})
	if !ok {
		items = []interface{}{v}
	}
	out := make([]float64, len(items))
	for i, it := range items {
		n, err := num(it, key)
		if err != nil {
			return nil, err
		}
		if !(n > 0) {
			return nil, fmt.Errorf("%s values must be > 0, got %v", key, n)
		}
		out[i] = n
	}
	return out, nil
}

func vec3(m map[string]interface{}, key string) (r3.Vector, error) {
	items, ok := m[key].([]interface{})
	if !ok || len(items) != 3 {
		return r3.Vector{}, fmt.Errorf("%s must be 3 numbers", key)
	}
	var f [3]float64
	for i, it := range items {
		n, err := num(it, key)
		if err != nil {
			return r3.Vector{}, err
		}
		f[i] = n
	}
	return r3.Vector{X: f[0], Y: f[1], Z: f[2]}, nil
}

var poseKeys = []string{"x", "y", "z", "o_x", "o_y", "o_z", "theta"}

// parsePose reads {x, y, z (mm), o_x, o_y, o_z, theta (deg)}. The orientation vector is normalised.
func parsePose(m map[string]interface{}) (spatialmath.Pose, error) {
	var f [7]float64
	for i, k := range poseKeys {
		v, ok := m[k]
		if !ok {
			return nil, fmt.Errorf("pose.%s is required", k)
		}
		n, err := num(v, "pose."+k)
		if err != nil {
			return nil, err
		}
		f[i] = n
	}
	o := r3.Vector{X: f[3], Y: f[4], Z: f[5]}
	if o.Norm() < 1e-9 {
		return nil, errors.New("pose orientation vector (o_x, o_y, o_z) must be non-zero")
	}
	o = o.Normalize()
	return spatialmath.NewPose(r3.Vector{X: f[0], Y: f[1], Z: f[2]},
		&spatialmath.OrientationVectorDegrees{OX: o.X, OY: o.Y, OZ: o.Z, Theta: f[6]}), nil
}

func poseMap(p spatialmath.Pose) map[string]interface{} {
	pt, ov := p.Point(), p.Orientation().OrientationVectorDegrees()
	return map[string]interface{}{"x": pt.X, "y": pt.Y, "z": pt.Z, "o_x": ov.OX, "o_y": ov.OY, "o_z": ov.OZ, "theta": ov.Theta}
}

func statusMap(st SessionStatus) map[string]interface{} {
	js := make([]interface{}, len(st.Config.MaxJointDegPerS))
	for i, v := range st.Config.MaxJointDegPerS {
		js[i] = v
	}
	ms := func(d time.Duration) float64 { return float64(d) / float64(time.Millisecond) }
	return map[string]interface{}{
		"state": string(st.State), "err_mm": st.ErrMM, "err_deg": st.ErrDeg, "rate_hz": st.RateHz,
		"last_command_age_ms": ms(st.LastCmdAge),
		"limits": map[string]interface{}{
			"joint_deg_s": js, "linear_mm_s": st.Config.MaxLinearMMPerS,
			"angular_deg_s": st.Config.MaxAngularDegPerS, "resync_deg": st.Config.ResyncDeg,
		},
		"tick_p50_ms": ms(st.TickP50), "tick_p99_ms": ms(st.TickP99),
		"overruns": float64(st.Overruns), "resyncs": float64(st.Resyncs), "damping": st.Damping,
	}
}
```

Notes:
- `parsePose` normalises the orientation vector itself, so a non-unit input never relies on RDK's behaviour.
- `TestPoseRoundTrip` checks the output against the unit-vector input.
- `poseMap` returns the orientation vector RDK computes from the quaternion. For a unit input this round-trips to within 1e-9. If RDK picks the opposite-sign equivalent (`-o`, `-theta`) for some input, change the test's input so the first round-trip is unique, and note it in your report. Don't weaken the tolerance.

- [ ] **Step 4: Run all tests and build the module**

```
go test -race ./...
go build -o bin/diff-ik ./cmd/module
```
Expected: PASS, and the binary builds.

- [ ] **Step 5: Commit**

```bash
gofmt -l . && go vet ./...
git add module.go module_test.go
git commit -m "Module: servo DoCommand protocol, arms config, Move busy check, StopPlan and Close

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_012FAJrCqoTNZgDVMfHN6sDY"
```

---

### Task 6: Python bench client

**Files:**
- Create: `clients/python/servo_bench.py`

This client only runs against a live machine, so there is no TDD loop. The check is that the script parses and its pure helpers work. Real runs happen in Task 8.

- [ ] **Step 1: Write the script**

Create `clients/python/servo_bench.py`:

```python
# /// script
# requires-python = ">=3.11"
# dependencies = ["viam-sdk"]
# ///
"""Phase 2 bench for the diff-ik servo DoCommand surface.

Usage (sim, local plaintext listener):
  uv run clients/python/servo_bench.py --address 127.0.0.1:<port> --insecure --arm pick-arm --joint-deg-s 60
With API keys instead: export VIAM_API_KEY_ID / VIAM_API_KEY and drop --insecure.
Spec: docs/superpowers/specs/2026-11-15-diff-ik-phase2-design.md, "Measurements".
"""

import argparse
import asyncio
import math
import os
import statistics
import sys
import time

FAILS: list[str] = []


def report(metric: str, value: str, bar: str = "report", ok: bool | None = None) -> None:
    mark = "" if ok is None else ("PASS" if ok else "FAIL")
    if ok is False:
        FAILS.append(metric)
    print(f"{metric:<38} {value:<26} {bar:<20} {mark}", flush=True)


def moved(q0: list[float], q: list[float], eps: float = 0.05) -> bool:
    return any(abs(a - b) > eps for a, b in zip(q0, q))


def first_stationary(samples: list[tuple[float, list[float]]], after: float, eps: float = 0.05) -> float | None:
    """Time of the first of three consecutive samples after `after` with every joint within eps of each other."""
    s = [x for x in samples if x[0] >= after]
    for i in range(len(s) - 2):
        a, b, c = s[i][1], s[i + 1][1], s[i + 2][1]
        if not moved(a, b, eps) and not moved(b, c, eps):
            return s[i][0]
    return None


async def connect(args):
    from viam.robot.client import RobotClient
    from viam.rpc.dial import DialOptions

    if args.insecure:
        dial = DialOptions(insecure=True, disable_webrtc=True)
    else:
        dial = DialOptions.with_api_key(os.environ["VIAM_API_KEY"], os.environ["VIAM_API_KEY_ID"])
    return await RobotClient.at_address(args.address, RobotClient.Options(dial_options=dial))


async def cmd(svc, **kw):
    return await svc.do_command(kw)


async def expect_err(svc, label: str, substr: str, **kw) -> None:
    try:
        await cmd(svc, **kw)
        report(f"smoke: {label}", "no error", f"error ∋ '{substr}'", False)
    except Exception as e:  # SDK wraps server errors in several exception types
        report(f"smoke: {label}", "error", f"error ∋ '{substr}'", substr in str(e))


class Poller:
    """Polls the arm on a second connection: joint positions (deg) or end positions (mm)."""

    def __init__(self, arm, end: bool = False):
        self.arm, self.end = arm, end
        self.samples: list[tuple[float, list[float]]] = []
        self.task: asyncio.Task | None = None

    async def _run(self) -> None:
        while True:
            if self.end:
                p = await self.arm.get_end_position()
                v = [p.x, p.y, p.z]
            else:
                v = list((await self.arm.get_joint_positions()).values)
            self.samples.append((time.monotonic(), v))

    def start(self) -> None:
        self.samples = []
        self.task = asyncio.create_task(self._run())

    async def stop(self) -> None:
        if self.task:
            self.task.cancel()
            try:
                await self.task
            except asyncio.CancelledError:
                pass


def start_kw(args, **kw):
    base = {"command": "servo_start", "component_name": args.arm, "frame": f"{args.arm}_origin",
            "limits": {"joint_deg_s": args.joint_deg_s}}
    base.update(kw)
    return base


async def hold_until_converged(svc, sid: str, pose: dict, timeout: float = 6.0) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        r = await cmd(svc, command="servo_target", session_id=sid, pose=pose)
        if r["state"] == "converged":
            return True
        await asyncio.sleep(0.1)
    return False


async def smoke(svc, args) -> None:
    r = await svc.do_command(start_kw(args, mode="pose"))
    sid, p0 = r["session_id"], r["pose"]
    await expect_err(svc, "busy arm", "busy", **start_kw(args))
    await expect_err(svc, "wrong mode", "wrong mode", command="servo_twist", session_id=sid,
                     linear=[0, 0, 0], angular=[0, 0, 0])
    await expect_err(svc, "unknown session", "unknown session_id", command="servo_status", session_id="nope")
    await expect_err(svc, "unknown command", "unknown command", command="servo_fly")
    goal = dict(p0, z=p0["z"] + 10)
    ok = await hold_until_converged(svc, sid, goal)
    st = await cmd(svc, command="servo_status", session_id=sid)
    report("smoke: pose converges", st["state"], "converged", ok)
    fin = await cmd(svc, command="servo_stop", session_id=sid)
    report("smoke: pose stop", fin["state"], "stopped", fin["state"] == "stopped")
    await expect_err(svc, "ended session", "session ended", command="servo_target", session_id=sid, pose=goal)

    r = await svc.do_command(start_kw(args, mode="twist"))
    sid = r["session_id"]
    for _ in range(5):
        await cmd(svc, command="servo_twist", session_id=sid, linear=[0, 0, -10], angular=[0, 0, 0])
        await asyncio.sleep(0.1)
    st = await cmd(svc, command="servo_status", session_id=sid)
    report("smoke: twist running", st["state"], "running", st["state"] == "running")
    fin = await cmd(svc, command="servo_stop", session_id=sid)
    report("smoke: twist stop", fin["state"], "stopped", fin["state"] == "stopped")


async def latency(svc, arm2, args) -> None:
    r = await svc.do_command(start_kw(args, mode="pose"))
    sid, base = r["session_id"], r["pose"]
    poll = Poller(arm2)
    lats, rtts = [], []
    for i in range(10):
        prev = dict(base, x=base["x"] + (0 if i % 2 == 0 else 20))
        tgt = dict(base, x=base["x"] + (20 if i % 2 == 0 else 0))
        if not await hold_until_converged(svc, sid, prev):
            report(f"latency step {i}", "no convergence", "converged", False)
            break
        poll.start()
        await asyncio.sleep(0.15)
        if not poll.samples:
            await poll.stop()
            report(f"latency step {i}", "no joint samples", "samples", False)
            break
        q0 = poll.samples[-1][1]
        t0 = time.monotonic()
        await cmd(svc, command="servo_target", session_id=sid, pose=tgt)
        rtts.append(time.monotonic() - t0)
        lat, last_send = None, t0
        while lat is None and time.monotonic() - t0 < 2:
            await asyncio.sleep(0.01)
            hit = next((ts for ts, q in poll.samples if ts > t0 and moved(q0, q)), None)
            if hit is not None:
                lat = hit - t0
            elif time.monotonic() - last_send > 0.1:
                await cmd(svc, command="servo_target", session_id=sid, pose=tgt)  # feed the dead-man
                last_send = time.monotonic()
        await poll.stop()
        if lat is not None:
            lats.append(lat)
    await cmd(svc, command="servo_stop", session_id=sid)
    if lats:
        report("command→motion latency (median)", f"{statistics.median(lats) * 1000:.0f} ms")
        report("command→motion latency (max)", f"{max(lats) * 1000:.0f} ms")
    report("servo_target round-trip (median)", f"{statistics.median(rtts) * 1000:.1f} ms")
    report("latency steps measured", f"{len(lats)}/10", "10/10", len(lats) == 10)


async def twist_for(svc, sid: str, linear: list[float], seconds: float) -> float:
    """Re-send a twist every 100 ms for `seconds`; returns the time of the last send."""
    end, last = time.monotonic() + seconds, 0.0
    while time.monotonic() < end:
        last = time.monotonic()
        await cmd(svc, command="servo_twist", session_id=sid, linear=linear, angular=[0, 0, 0])
        await asyncio.sleep(0.1)
    return last


async def stop_latency(svc, arm2, args) -> None:
    sid = (await svc.do_command(start_kw(args, mode="twist")))["session_id"]
    poll = Poller(arm2)
    poll.start()
    await twist_for(svc, sid, [50, 0, 0], 1.0)
    p_before = await arm2.get_end_position()
    t0 = time.monotonic()
    fin = await cmd(svc, command="servo_stop", session_id=sid)
    rtt = time.monotonic() - t0
    await asyncio.sleep(1.5)
    await poll.stop()
    p_after = await arm2.get_end_position()
    still = first_stationary(poll.samples, t0)
    report("servo_stop round-trip (incl. ramp)", f"{rtt * 1000:.0f} ms")
    report("stop sent → arm stationary", "never" if still is None else f"{(still - t0) * 1000:.0f} ms")
    d = math.dist((p_before.x, p_before.y, p_before.z), (p_after.x, p_after.y, p_after.z))
    report("travel after stop sent", f"{d:.1f} mm")
    report("stop final state", fin["state"], "stopped", fin["state"] == "stopped")


async def deadman(svc, arm2, args) -> None:
    timeout_ms, ramp_ms, rate = 500, 200, 50
    sid = (await svc.do_command(start_kw(args, mode="twist", timeout_ms=timeout_ms, ramp_ms=ramp_ms,
                                         rate_hz=rate)))["session_id"]
    poll = Poller(arm2)
    poll.start()
    t_last = await twist_for(svc, sid, [-50, 0, 0], 1.0)
    t_to = None
    while time.monotonic() - t_last < 3:
        st = await cmd(svc, command="servo_status", session_id=sid)
        if st["state"] == "stopped_timeout":
            t_to = time.monotonic() - t_last
            break
        await asyncio.sleep(0.02)
    await asyncio.sleep(1.0)
    await poll.stop()
    bar = timeout_ms + 2 * 1000 / rate + ramp_ms + 50
    report("last cmd → stopped_timeout", "never" if t_to is None else f"{t_to * 1000:.0f} ms",
           f"≤ {bar:.0f} ms", t_to is not None and t_to * 1000 <= bar)
    still = first_stationary(poll.samples, t_last)
    report("last cmd → arm stationary", "never" if still is None else f"{(still - t_last) * 1000:.0f} ms")


async def circle(svc, arm2, args, resync: float | None) -> None:
    limits = {"joint_deg_s": args.joint_deg_s}
    if resync:
        limits["resync_deg"] = resync
    r = await svc.do_command(start_kw(args, mode="pose", limits=limits))
    sid, c = r["session_id"], r["pose"]
    radius, speed = 50.0, 50.0
    omega = speed / radius
    laps_t = 2 * 2 * math.pi / omega
    if not await hold_until_converged(svc, sid, dict(c, x=c["x"] + radius)):
        report(f"circle (resync {resync or 'default'})", "no convergence at start", "converged", False)
        await cmd(svc, command="servo_stop", session_id=sid)
        return
    poll = Poller(arm2, end=True)
    poll.start()
    t0, n = time.monotonic(), 0
    while (t := time.monotonic() - t0) < laps_t:
        th = omega * t
        await cmd(svc, command="servo_target", session_id=sid,
                  pose=dict(c, x=c["x"] + radius * math.cos(th), y=c["y"] + radius * math.sin(th)))
        n += 1
        await asyncio.sleep(max(0.0, t0 + n / 50 - time.monotonic()))
    await poll.stop()
    st = await cmd(svc, command="servo_status", session_id=sid)
    await cmd(svc, command="servo_stop", session_id=sid)
    errs = []
    for ts, (x, y, z) in poll.samples:
        th = omega * (ts - t0)
        if 0 <= ts - t0 <= laps_t:
            errs.append(math.dist((x, y, z), (c["x"] + radius * math.cos(th), c["y"] + radius * math.sin(th), c["z"])))
    label = f"circle resync={resync or 'default'}"
    if not errs:
        report(f"{label}: samples", "none", "> 0", False)
        return
    mean = statistics.mean(errs)
    if resync == 15:
        report(f"{label}: mean error", f"{mean:.1f} mm", "≤ 22.1 mm", mean <= 22.1)
    else:
        report(f"{label}: mean error", f"{mean:.1f} mm", "report (P1: 45.8)")
    report(f"{label}: max error", f"{max(errs):.1f} mm")
    report(f"{label}: resyncs / overruns", f"{st['resyncs']:.0f} / {st['overruns']:.0f}")
    report(f"{label}: target rate", f"{n / laps_t:.1f} Hz")


async def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--address", default="localhost:8080")
    ap.add_argument("--insecure", action="store_true")
    ap.add_argument("--service", default="servo")
    ap.add_argument("--arm", required=True)
    ap.add_argument("--joint-deg-s", type=float, default=90)
    ap.add_argument("--steps", default="smoke,latency,stop,deadman,circle")
    args = ap.parse_args()

    from viam.components.arm import Arm
    from viam.services.motion import MotionClient

    robot, robot2 = await connect(args), await connect(args)
    try:
        svc = MotionClient.from_robot(robot, args.service)
        arm2 = Arm.from_robot(robot2, args.arm)
        steps = args.steps.split(",")
        if "smoke" in steps:
            await smoke(svc, args)
        if "latency" in steps:
            await latency(svc, arm2, args)
        if "stop" in steps:
            await stop_latency(svc, arm2, args)
        if "deadman" in steps:
            await deadman(svc, arm2, args)
        if "circle" in steps:
            await circle(svc, arm2, args, None)
            await circle(svc, arm2, args, 15)
    finally:
        await robot.close()
        await robot2.close()
    print(f"\n{'FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL PASS'}")
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
```

- [ ] **Step 2: Check that it parses and the helpers work**

```bash
uv run clients/python/servo_bench.py --help
uv run --with viam-sdk python -I -c "
import importlib.util, sys
spec = importlib.util.spec_from_file_location('b', 'clients/python/servo_bench.py'); b = importlib.util.module_from_spec(spec); spec.loader.exec_module(b)
assert b.moved([0,0],[0,0.06]) and not b.moved([0,0],[0,0.04])
s = [(0,[0]),(1,[1]),(2,[1.01]),(3,[1.02]),(4,[1.02])]
assert b.first_stationary(s, 0) == 1, b.first_stationary(s, 0)
print('ok')"
```
Expected: the help text prints, then `ok`.

Before relying on `DialOptions(insecure=True, disable_webrtc=True)`, `DialOptions.with_api_key(api_key, api_key_id)`, `MotionClient.do_command(dict)` and `Arm.get_end_position()`, verify each against `~/src/viam-python-sdk/src/viam` and fix any mismatch. A connection can only be tested live, in Task 8.

- [ ] **Step 3: Commit**

```bash
git add clients/python/servo_bench.py
git commit -m "Add Python servo bench: smoke, latency, stop, dead-man, circle

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_012FAJrCqoTNZgDVMfHN6sDY"
```

---

### Task 7: TypeScript smoke client

**Files:**
- Create: `clients/ts/package.json`, `clients/ts/tsconfig.json`, `clients/ts/smoke.ts`
- Modify: `.gitignore` (add `clients/ts/node_modules/`)

- [ ] **Step 1: Write the client**

`clients/ts/package.json`:

```json
{
  "name": "diff-ik-ts-smoke",
  "private": true,
  "type": "module",
  "scripts": { "smoke": "tsx smoke.ts", "typecheck": "tsc --noEmit" },
  "dependencies": {
    "@connectrpc/connect-node": "^1.5.0",
    "@viamrobotics/sdk": "^0.74.0",
    "node-datachannel": "^0.29.0"
  },
  "devDependencies": { "@types/node": "^22.5.5", "tsx": "^4.19.3", "typescript": "^5.2.2" }
}
```

`clients/ts/tsconfig.json`:

```json
{
  "compilerOptions": {
    "target": "ES2022", "module": "ESNext", "moduleResolution": "Bundler",
    "strict": true, "skipLibCheck": true, "noEmit": true
  },
  "files": ["smoke.ts"]
}
```

`clients/ts/smoke.ts`. Environment: `HOST` (for example `http://127.0.0.1:<port>` for the plaintext local listener), `ARM`, `SERVICE` (default `servo`) and `JOINT_DEG_S`. `VIAM_API_KEY_ID` and `VIAM_API_KEY` are optional; when set, the client connects through cloud signaling.

```ts
// Phase 2 TypeScript smoke test: the same command sequence as the Python bench's smoke step.
import * as VIAM from '@viamrobotics/sdk';
import { createGrpcTransport } from '@connectrpc/connect-node';

(globalThis as any).VIAM = {
  GRPC_TRANSPORT_FACTORY: (opts: any) => createGrpcTransport({ httpVersion: '2', ...opts }),
};

const env = (k: string, d?: string): string => {
  const v = process.env[k] ?? d;
  if (v === undefined) throw new Error(`set ${k}`);
  return v;
};
const arm = env('ARM');
const fails: string[] = [];
const check = (name: string, ok: boolean, detail: unknown) => {
  console.log(`${name.padEnd(30)} ${ok ? 'PASS' : 'FAIL'}  ${JSON.stringify(detail)}`);
  if (!ok) fails.push(name);
};
const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));

async function main() {
  const keyId = process.env.VIAM_API_KEY_ID;
  const key = process.env.VIAM_API_KEY;
  if (keyId && key) {
    // Cloud signaling dials WebRTC; Node has no RTCPeerConnection, so install the polyfill
    // (as ~/src/viam-typescript-sdk/examples/node/src/main.ts does).
    const wrtc = await import('node-datachannel/polyfill');
    for (const k in wrtc) (globalThis as any)[k] = (wrtc as any)[k];
  }
  const client = await VIAM.createRobotClient({
    host: env('HOST'),
    noReconnect: true,
    ...(keyId && key
      ? {
          credentials: { type: 'api-key', authEntity: keyId, payload: key },
          signalingAddress: 'https://app.viam.com:443',
          iceServers: [{ urls: 'stun:global.stun.twilio.com:3478' }],
        }
      : {}),
  } as any);
  const svc = new VIAM.MotionClient(client, env('SERVICE', 'servo'));
  const call = (c: Record<string, any>) => svc.doCommand(c) as Promise<any>;
  const expectErr = async (name: string, sub: string, c: Record<string, any>) => {
    try {
      await call(c);
      check(name, false, 'no error');
    } catch (e) {
      check(name, String(e).includes(sub), String(e).slice(0, 80));
    }
  };
  const start = (mode: string) => ({
    command: 'servo_start', component_name: arm, frame: `${arm}_origin`, mode,
    limits: { joint_deg_s: Number(env('JOINT_DEG_S', '90')) },
  });

  try {
    const r = await call(start('pose'));
    const sid = r.session_id as string;
    const goal = { ...r.pose, z: r.pose.z + 10 };
    await expectErr('busy arm', 'busy', start('pose'));
    await expectErr('wrong mode', 'wrong mode', { command: 'servo_twist', session_id: sid, linear: [0, 0, 0], angular: [0, 0, 0] });
    await expectErr('unknown session', 'unknown session_id', { command: 'servo_status', session_id: 'nope' });
    await expectErr('unknown command', 'unknown command', { command: 'servo_fly' });
    let state = '';
    for (let i = 0; i < 60 && state !== 'converged'; i++) {
      state = (await call({ command: 'servo_target', session_id: sid, pose: goal })).state;
      await sleep(100);
    }
    check('pose converges', state === 'converged', state);
    const st = await call({ command: 'servo_status', session_id: sid });
    check('status has fields', typeof st.rate_hz === 'number' && typeof st.limits === 'object', Object.keys(st));
    check('pose stop', (await call({ command: 'servo_stop', session_id: sid })).state === 'stopped', 'stop');
    await expectErr('ended session', 'session ended', { command: 'servo_target', session_id: sid, pose: goal });

    const tw = (await call(start('twist'))).session_id as string;
    for (let i = 0; i < 5; i++) {
      await call({ command: 'servo_twist', session_id: tw, linear: [0, 0, -10], angular: [0, 0, 0] });
      await sleep(100);
    }
    check('twist stop', (await call({ command: 'servo_stop', session_id: tw })).state === 'stopped', 'stop');
  } finally {
    await client.disconnect();
  }
  console.log(fails.length ? `FAILED: ${fails.join(', ')}` : 'ALL PASS');
  process.exit(fails.length ? 1 : 0);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
```

Before relying on these TypeScript SDK APIs, check them against `~/src/viam-typescript-sdk/src`:
- `createRobotClient`'s `DialConf`, especially whether a direct `http://` host dials plaintext without `signalingAddress`;
- the `MotionClient` constructor;
- `doCommand` accepting a `Record`;
- `client.disconnect()`.

Fix any mismatch. Also check that npm's latest `@viamrobotics/sdk` 0.x satisfies `^0.74.0`; bump the range if a newer minor is current.

If the SDK can't dial a plaintext host from Node, keep the API-key and signaling branch, which uses the WebRTC polyfill, as the supported path. Write that limitation in a comment at the top of `smoke.ts` and in your report. Don't silently drop the plaintext attempt.

- [ ] **Step 2: Install and type-check**

```bash
cd clients/ts && npm install && npm run typecheck
```
Expected: no type errors. A live run happens in Task 8.

- [ ] **Step 3: Commit**

Add `clients/ts/node_modules/` to `.gitignore`. Commit `package-lock.json` too.

```bash
git add .gitignore clients/ts/package.json clients/ts/package-lock.json clients/ts/tsconfig.json clients/ts/smoke.ts
git commit -m "Add TypeScript servo smoke client

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_012FAJrCqoTNZgDVMfHN6sDY"
```

---

### Task 8: Deploy to the sim, measure, record results — controller only

The controller does this task itself, not a subagent, because it needs the user's confirmation before changing the machine config, and the running sim.

- [ ] **Step 1:** Run `go build -o bin/diff-ik ./cmd/module`.
- [ ] **Step 2:** Show the user the exact JSON to add to the `shape-finder-vision-pick` machine:
  - a local module, `{"type": "local", "name": "diff-ik", "executable_path": "/Users/nick.hehr/src/diff-ik/bin/diff-ik"}`;
  - the service, `{"name": "servo", "api": "rdk:service:motion", "model": "devrel:diff-ik:servo", "attributes": {"arms": ["pick-arm"]}}`.

  **Wait for the user's OK.** Then apply it through the Viam MCP config tools, or have the user apply it.
- [ ] **Step 3:** Confirm viam-server loaded the module and the `servo` service. Find the loopback listener port again (`lsof -nP -iTCP -sTCP:LISTEN | grep viam`); it changes when viam-server restarts. Move the arm to the raised pose `[-90,-90,90,-90,-90,0]°`, as in Phase 1.
- [ ] **Step 4:** Run `uv run clients/python/servo_bench.py --address 127.0.0.1:<port> --insecure --arm pick-arm --joint-deg-s 60`.
- [ ] **Step 5:** Run `cd clients/ts && HOST=http://127.0.0.1:<port> ARM=pick-arm JOINT_DEG_S=60 npm run smoke`.
- [ ] **Step 6:** Write `docs/results/2026-11-phase2.md` in the Phase 1 results format:
  - per-step metric, value, bar and PASS/FAIL tables;
  - the circle comparison with Phase 1 (45.8 mm at the default resync, 31.6 mm at `resync_deg: 15`), with resyncs;
  - the note that the dead-man bar measures the `stopped_timeout` transition, not the arm stopping, because of the ~240 ms sim lag;
  - tuning findings and open items.

  Commit it with `git add docs/results`.
- [ ] **Step 7:** Final review of the whole branch with an Opus reviewer, then superpowers:finishing-a-development-branch.
