# Diff-IK Servo Phases 0 & 1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the differential-IK servo loop and a CLI harness that validates kinematics (Phase 0) and runs servo scenarios against the MuJoCo sim arm (Phase 1).

**Architecture:** The root package `diffik` holds the kinematics helpers (`kin.go`), the servo loop (`loop.go`) and the streamed-move arm adapter (`armio.go`). The loop talks to arms only through the `JointIO` interface, so `go test` can run it against an in-memory arm. `cmd/cli` connects to a machine and drives the loop through scenarios. The `motion.Service` methods in `module.go` stay unimplemented in these phases.

**Tech Stack:** Go 1.26, `go.viam.com/rdk` v1.16.0 (`referenceframe`, `spatialmath`, `components/arm`, `robot/client`), `gonum.org/v1/gonum` (`mat`, `num/quat`), `github.com/golang/geo/r3`. All of these come in with RDK, so there are no new dependencies.

**Spec:** `docs/superpowers/specs/2026-11-15-diff-ik-phase0-1-design.md`

**Verified:** all code below was compiled and `go test -race` passed against RDK v1.16.0 while writing this plan.

---

## Facts about RDK v1.16.0 the code relies on

- `referenceframe.Input` is `float64`. Revolute joints are in radians.
- `Model.Transform(q)` returns the end-effector pose in the base frame in **mm**. `Model.DoF()` returns `[]Limit{Min, Max, ...}`.
- `arm.Arm.Kinematics(ctx)` returns a `referenceframe.Model` directly. The spec's "unparseable kinematics" case therefore becomes "`Kinematics` returned an error".
- The streamed move is `MoveThroughJointPositionsStreamed(ctx, batches <-chan []arm.TrajectoryPoint, responses chan<- arm.Response, extra)`.
  - `TrajectoryPoint{Time, Positions}`: `Time` is measured from stream start. It must be 0 on the first point and strictly increasing after that.
  - The **caller** owns both channels. It writes and closes `batches`, drains `responses`, and closes `responses` after the call returns.
  - Some drivers (the fake arm, for one) exit only when `batches` closes; they ignore context cancellation.
- `spatialmath.QuatToR3AA` returns zero for angles under 1e-6 rad. That would zero out the Jacobian's angular rows, so `kin.go` has its own `RotVec`.
- `kinematics.ModelFromName(kinematics.UR5e, "arm")` loads the bundled UR5e model, which is the test fixture. `fake.NewArm` with `fake.Config{ArmModel: kinematics.UR5e}` gives a streaming-capable fake arm.

## Deviations from the spec (all small)

- `armIO` lives in its own file, `armio.go`, instead of `loop.go`.
- The Jacobian accuracy check perturbs **one joint at a time** by 1°. Perturbing all joints at once let UR5e's parallel axes cancel, which shrank the true motion until second-order terms dominated the error. Task 7 records this in the spec.
- `TaskWeights` is a `[6]float64` array, so a "wrong length" error is impossible and isn't checked.
- `Status` reports total tick time, not compute and I/O time separately.
- `Loop.OnTick` (a callback) feeds the CLI's per-tick recorder and CSV output.

## File map

| File | Responsibility |
|---|---|
| `module.go` | Scaffold `motion.Service`; compile fixes only. |
| `cmd/module/main.go` | Module entrypoint; vet fix only. |
| `kin.go` / `kin_test.go` | `FK`, `RotVec`, `PoseError`, `Jacobian`, `dlsStep`. |
| `loop.go` / `loop_test.go` | `JointIO`, `LoopConfig`, `Loop`, `Status`, states, `Oscillations`; tests use `simIO`. |
| `armio.go` / `armio_test.go` | `NewArmIO`: `JointIO` over the streamed move; tested against RDK's fake arm. |
| `cmd/cli/main.go` | Flags, connection, dispatch. |
| `cmd/cli/validate.go` | Phase 0 `validate`. |
| `cmd/cli/run.go` | Phase 1 `run static|singular|circle`, recorder and CSV. |

---

### Task 1: Make the scaffold compile and commit a baseline

The working tree is all untracked scaffold. `module.go` doesn't compile: it has about 9 unused imports, no `fmt` import while still calling `fmt.Errorf`, and an unqualified `TempStreamResponse`. `cmd/module/main.go` fails `go vet` because of unkeyed fields.

**Files:**
- Modify: `module.go` (import block, every `fmt.Errorf("not implemented")`, the `TempStreamArmJointPositions` signature)
- Modify: `cmd/module/main.go:12`
- Modify: `go.mod`, `go.sum`

- [ ] **Step 1: Replace the import block in `module.go`** with:

```go
import (
	"context"
	"errors"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	motion "go.viam.com/rdk/services/motion"
)
```

- [ ] **Step 2: Fix the bodies and the signature in `module.go`**

```bash
sed -i '' 's/fmt.Errorf("not implemented")/errUnimplemented/g; s/chan<- TempStreamResponse/chan<- motion.TempStreamResponse/' module.go
```

- [ ] **Step 3: Use keyed fields in `cmd/module/main.go`**

Change `resource.APIModel{ motion.API, diffik.Servo}` to `resource.APIModel{API: motion.API, Model: diffik.Servo}`.

- [ ] **Step 4: Pin RDK and tidy**

```bash
go get go.viam.com/rdk@v1.16.0 && go mod tidy && gofmt -w . && go build ./... && go vet ./...
```
Expected: no output from build or vet. `go.mod` has `require go.viam.com/rdk v1.16.0`.

- [ ] **Step 5: Commit the baseline**

```bash
git add .github .gitignore .viam-gen-info DEVELOPER_GUIDE.md Makefile README.md cmd devrel_diff-ik_servo.md go.mod go.sum meta.json module.go
git commit -m "Add generated module scaffold, fixed to compile against RDK v1.16.0

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_012FAJrCqoTNZgDVMfHN6sDY"
```

---

### Task 2: Kinematics helpers

Units and frames follow the spec's "Kinematics conventions": base frame, meters and radians, with mm converted to m only in `FK`. The orientation error is `log(R_t·R_cᵀ)`. Angular Jacobian rows are `log(R(q+h)·R(q−h)ᵀ)/2h`.

**Files:**
- Create: `kin_test.go`
- Create: `kin.go`

- [ ] **Step 1: Write the failing tests** in `kin_test.go`:

```go
package diffik

import (
	"math"
	"math/rand"
	"slices"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm/kinematics"
	"go.viam.com/rdk/referenceframe"
	"gonum.org/v1/gonum/mat"
	"gonum.org/v1/gonum/num/quat"
)

func ur5e(t *testing.T) referenceframe.Model {
	t.Helper()
	m, err := kinematics.ModelFromName(kinematics.UR5e, "arm")
	if err != nil {
		t.Fatal(err)
	}
	return m
}

// rotZ returns a quaternion for a rotation of a radians about base z.
func rotZ(a float64) quat.Number {
	return quat.Number{Real: math.Cos(a / 2), Kmag: math.Sin(a / 2)}
}

func TestPoseError(t *testing.T) {
	q := rotZ(0.3)
	p := r3.Vector{X: 0.4, Y: 0.1, Z: 0.2}
	for i, v := range PoseError(p, q, p, q) {
		if math.Abs(v) > 1e-12 {
			t.Fatalf("identical poses: e[%d] = %v, want 0", i, v)
		}
	}
	// target is 10 mm further in x and rotated a further 0.1 rad about z
	e := PoseError(p.Add(r3.Vector{X: 0.01}), quat.Mul(rotZ(0.1), q), p, q)
	want := [6]float64{0.01, 0, 0, 0, 0, 0.1}
	for i := range e {
		if math.Abs(e[i]-want[i]) > 1e-9 {
			t.Fatalf("e = %v, want %v", e, want)
		}
	}
}

func TestRotVecSmallAngle(t *testing.T) {
	r := RotVec(rotZ(1e-7), quat.Number{Real: 1})
	if math.Abs(r.Z-1e-7) > 1e-12 {
		t.Fatalf("RotVec of 1e-7 rad = %v, want z = 1e-7", r)
	}
}

// relErr is ‖a − b‖ / ‖a‖.
func relErr(a, b r3.Vector) float64 { return a.Sub(b).Norm() / a.Norm() }

// Perturbs one joint at a time by 1°. Perturbing all joints at once lets parallel axes cancel,
// which shrinks the true motion until second-order terms dominate the relative error.
func TestJacobianPredictsSmallMotion(t *testing.T) {
	m := ur5e(t)
	rng := rand.New(rand.NewSource(1))
	n := len(m.DoF())
	for trial := 0; trial < 20; trial++ {
		q := make([]float64, n)
		for i := range q {
			q[i] = (rng.Float64()*2 - 1) * 1.5
		}
		J, err := Jacobian(m, q)
		if err != nil {
			t.Fatal(err)
		}
		p1, r1, _ := FK(m, q)
		for j := 0; j < n; j++ {
			q2 := slices.Clone(q)
			q2[j] += deg2rad(1)
			p2, r2, _ := FK(m, q2)
			col := mat.Col(nil, j, J)
			predLin := r3.Vector{X: col[0], Y: col[1], Z: col[2]}.Mul(deg2rad(1))
			predAng := r3.Vector{X: col[3], Y: col[4], Z: col[5]}.Mul(deg2rad(1))
			if act := p2.Sub(p1); act.Norm() > 1e-5 { // skip joints whose axis passes through the tool point
				if e := relErr(act, predLin); e > 0.05 {
					t.Errorf("trial %d joint %d: linear prediction off by %.1f%%", trial, j, e*100)
				}
			}
			if e := relErr(RotVec(r2, r1), predAng); e > 0.05 {
				t.Errorf("trial %d joint %d: angular prediction off by %.1f%%", trial, j, e*100)
			}
		}
	}
}

func TestDLSStep(t *testing.T) {
	ones := [6]float64{1, 1, 1, 1, 1, 1}
	dx := [6]float64{0.01, -0.02, 0.03, 0.001, 0, -0.002}

	// Well-conditioned diagonal J: no damping, exact solve dq_i = dx_i / d_i.
	d := []float64{1, 2, 3, 1, 2, 3}
	J := mat.NewDense(6, 6, nil)
	for i, v := range d {
		J.Set(i, i, v)
	}
	dq, lambda, _, err := dlsStep(J, ones, dx, 0.04, 0.1)
	if err != nil || lambda != 0 {
		t.Fatalf("lambda = %v, err = %v; want 0, nil", lambda, err)
	}
	for i := range dq {
		if math.Abs(dq[i]-dx[i]/d[i]) > 1e-12 {
			t.Fatalf("dq = %v, want dx/d", dq)
		}
	}

	// Rank-deficient: column 6 is zero, so damping must kick in and the step stays finite.
	J = mat.NewDense(6, 6, nil)
	for i := 0; i < 5; i++ {
		J.Set(i, i, 1)
	}
	dq, lambda, sigmaMin, err := dlsStep(J, ones, dx, 0.04, 0.1)
	if err != nil {
		t.Fatal(err)
	}
	if sigmaMin != 0 || math.Abs(lambda-0.1) > 1e-12 {
		t.Fatalf("sigmaMin = %v lambda = %v, want 0 and 0.1", sigmaMin, lambda)
	}
	if dq[5] != 0 {
		t.Fatalf("dq[5] = %v, want 0 for the dead joint", dq[5])
	}
}
```

- [ ] **Step 2: Run them and watch them fail**

Run: `go test -run 'PoseError|RotVec|Jacobian|DLS' .`
Expected: build failure, `undefined: PoseError` (along with `RotVec`, `Jacobian`, `FK`, `dlsStep`, `deg2rad`).

`deg2rad` gets defined in `loop.go` in Task 3. For this task, add a temporary line `func deg2rad(d float64) float64 { return d * math.Pi / 180 }` at the bottom of `kin.go`, and remove it in Task 3, Step 3.

- [ ] **Step 3: Write `kin.go`**

```go
package diffik

import (
	"errors"
	"math"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"gonum.org/v1/gonum/mat"
	"gonum.org/v1/gonum/num/quat"
)

var errSVD = errors.New("jacobian SVD failed")

const (
	mmPerM = 1000.0
	fdStep = 1e-6 // rad, central-difference step for the Jacobian
)

// FK returns the end-effector position (m) and orientation in the arm's base frame.
// This is the only place model output is converted from mm to m.
func FK(m referenceframe.Model, q []referenceframe.Input) (r3.Vector, quat.Number, error) {
	p, err := m.Transform(q)
	if err != nil {
		return r3.Vector{}, quat.Number{}, err
	}
	return p.Point().Mul(1 / mmPerM), p.Orientation().Quaternion(), nil
}

// RotVec returns log(a·bᵀ) as a base-frame rotation vector in radians.
// spatialmath.QuatToR3AA is not used because it returns zero below 1e-6 rad,
// which would zero out the Jacobian's angular rows.
func RotVec(a, b quat.Number) r3.Vector {
	q := quat.Mul(a, quat.Conj(b))
	if q.Real < 0 { // shortest rotation
		q = quat.Scale(-1, q)
	}
	s := math.Sqrt(q.Imag*q.Imag + q.Jmag*q.Jmag + q.Kmag*q.Kmag)
	k := 2.0 // small-angle limit of 2·atan2(s, w)/s
	if s > 1e-12 {
		k = 2 * math.Atan2(s, q.Real) / s
	}
	return r3.Vector{X: k * q.Imag, Y: k * q.Jmag, Z: k * q.Kmag}
}

// PoseError returns [dx dy dz rx ry rz] (m, rad) from current to target, in the base frame.
func PoseError(tp r3.Vector, tq quat.Number, cp r3.Vector, cq quat.Number) [6]float64 {
	d := tp.Sub(cp)
	r := RotVec(tq, cq)
	return [6]float64{d.X, d.Y, d.Z, r.X, r.Y, r.Z}
}

// Jacobian returns the 6×n base-frame Jacobian at q by central differences.
// Rows are (vx, vy, vz, wx, wy, wz) in m/rad and rad/rad.
func Jacobian(m referenceframe.Model, q []referenceframe.Input) (*mat.Dense, error) {
	n := len(q)
	J := mat.NewDense(6, n, nil)
	qp := make([]referenceframe.Input, n)
	qm := make([]referenceframe.Input, n)
	for i := 0; i < n; i++ {
		copy(qp, q)
		copy(qm, q)
		qp[i] += fdStep
		qm[i] -= fdStep
		pp, rp, err := FK(m, qp)
		if err != nil {
			return nil, err
		}
		pm, rm, err := FK(m, qm)
		if err != nil {
			return nil, err
		}
		dp := pp.Sub(pm).Mul(1 / (2 * fdStep))
		dr := RotVec(rp, rm).Mul(1 / (2 * fdStep))
		J.SetCol(i, []float64{dp.X, dp.Y, dp.Z, dr.X, dr.Y, dr.Z})
	}
	return J, nil
}

// dlsStep solves Δq = (JᵀWJ + λ²I)⁻¹ JᵀW Δx with Chiaverini damping on σmin(W^½J).
func dlsStep(J *mat.Dense, w [6]float64, dx [6]float64, eps, maxDamping float64) (dq []float64, lambda, sigmaMin float64, err error) {
	_, n := J.Dims()
	var wj mat.Dense
	wj.Apply(func(i, _ int, v float64) float64 { return math.Sqrt(w[i]) * v }, J)

	var svd mat.SVD
	if !svd.Factorize(&wj, mat.SVDNone) {
		return nil, 0, 0, errSVD
	}
	vals := svd.Values(nil)
	sigmaMin = vals[len(vals)-1]

	l2 := 0.0
	if sigmaMin < eps {
		r := sigmaMin / eps
		l2 = maxDamping * maxDamping * (1 - r*r)
	}

	var a mat.Dense
	a.Mul(wj.T(), &wj)
	for i := 0; i < n; i++ {
		a.Set(i, i, a.At(i, i)+l2)
	}
	wdx := mat.NewVecDense(6, nil)
	for i := range dx {
		wdx.SetVec(i, math.Sqrt(w[i])*dx[i])
	}
	var b, x mat.VecDense
	b.MulVec(wj.T(), wdx)
	if err := x.SolveVec(&a, &b); err != nil {
		return nil, 0, 0, err
	}
	return append([]float64(nil), x.RawVector().Data...), math.Sqrt(l2), sigmaMin, nil
}
```

Then append the temporary `deg2rad` from Step 2.

- [ ] **Step 4: Run the tests and watch them pass**

Run: `go mod tidy && go test -run 'PoseError|RotVec|Jacobian|DLS' -v .`
Expected: `TestPoseError`, `TestRotVecSmallAngle`, `TestJacobianPredictsSmallMotion` and `TestDLSStep` all PASS. `go mod tidy` moves `gonum` and `golang/geo` to direct requirements.

- [ ] **Step 5: Commit**

```bash
git add kin.go kin_test.go go.mod go.sum
git commit -m "Add FK, rotation-vector error, finite-difference Jacobian and damped least squares step

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_012FAJrCqoTNZgDVMfHN6sDY"
```

---

### Task 3: The servo loop

This task implements the spec's per-tick data flow; comments in `tick` carry the step numbers. `tick(ctx, dt)` is unexported and called directly by tests, so they run in simulated time. `Run` adds the ticker, timing stats and the "stop before return" behavior.

**Files:**
- Create: `loop_test.go`
- Create: `loop.go`
- Modify: `kin.go` (remove the temporary `deg2rad`)

- [ ] **Step 1: Write the failing tests** in `loop_test.go`:

```go
package diffik

import (
	"context"
	"errors"
	"math"
	"slices"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"gonum.org/v1/gonum/num/quat"
)

// simIO is a perfect in-memory arm: Send moves it instantly unless ignore > 0.
type simIO struct {
	q       []float64
	sends   [][]float64
	ignore  int   // drop this many sends (arm "stuck")
	sendErr error // returned by every Send when set
	stopped bool
}

func (s *simIO) Read(context.Context) ([]referenceframe.Input, error) { return slices.Clone(s.q), nil }

func (s *simIO) Send(_ context.Context, q []referenceframe.Input) error {
	if s.sendErr != nil {
		return s.sendErr
	}
	s.sends = append(s.sends, slices.Clone(q))
	if s.ignore > 0 {
		s.ignore--
		return nil
	}
	s.q = slices.Clone(q)
	return nil
}

func (s *simIO) Stop(context.Context) error { s.stopped = true; return nil }

// home is a typical UR5e working pose, away from singularities.
var home = []float64{0, -1.2, 1.5, -1.9, -1.57, 0}

func newTestLoop(t *testing.T, q0 []float64, cfg LoopConfig) (*Loop, *simIO, referenceframe.Model) {
	t.Helper()
	m := ur5e(t)
	sim := &simIO{q: slices.Clone(q0)}
	l, err := NewLoop(m, sim, cfg)
	if err != nil {
		t.Fatal(err)
	}
	return l, sim, m
}

// offsetTarget returns the pose at q moved by d (mm, base frame) and rotated by rot about base z.
func offsetTarget(t *testing.T, m referenceframe.Model, q []float64, d r3.Vector, rot float64) spatialmath.Pose {
	t.Helper()
	p, o, err := FK(m, q)
	if err != nil {
		t.Fatal(err)
	}
	qt := spatialmath.Quaternion(quat.Mul(rotZ(rot), o))
	return spatialmath.NewPose(p.Mul(mmPerM).Add(d), &qt)
}

func runTicks(t *testing.T, l *Loop, n int) {
	t.Helper()
	for i := 0; i < n; i++ {
		if err := l.tick(context.Background(), 1/l.cfg.RateHz); err != nil {
			t.Fatal(err)
		}
	}
}

func TestNewLoopRejectsBadConfig(t *testing.T) {
	m := ur5e(t)
	for name, cfg := range map[string]LoopConfig{
		"rate":    {RateHz: -1},
		"weights": {TaskWeights: [6]float64{1, 1, 1, -1, 1, 1}},
		"limits":  {MaxJointDegPerS: []float64{90, 90}},
	} {
		if _, err := NewLoop(m, &simIO{}, cfg); err == nil {
			t.Errorf("%s: want error", name)
		}
	}
}

func TestIdleWithoutTarget(t *testing.T) {
	l, sim, _ := newTestLoop(t, home, LoopConfig{})
	runTicks(t, l, 5)
	if len(sim.sends) != 0 || l.Status().State != StateIdle {
		t.Fatalf("sends = %d state = %s, want 0 and idle", len(sim.sends), l.Status().State)
	}
}

func TestConvergesOnStaticTarget(t *testing.T) {
	l, _, m := newTestLoop(t, home, LoopConfig{})
	l.SetTarget(offsetTarget(t, m, home, r3.Vector{X: 30, Y: 30, Z: -25}, deg2rad(10))) // ≈ 50 mm, 10°
	runTicks(t, l, 150)                                                                 // 3 s at 50 Hz
	s := l.Status()
	if s.State != StateConverged || s.ErrMM > 2 || s.ErrDeg > 2 {
		t.Fatalf("state = %s err = %.2f mm %.2f°, want converged within 2 mm / 2°", s.State, s.ErrMM, s.ErrDeg)
	}
}

func TestSingularityRespectsLimitsWithoutOscillation(t *testing.T) {
	q0 := slices.Clone(home)
	q0[4] = 0 // UR5e wrist singularity
	l, sim, m := newTestLoop(t, q0, LoopConfig{})
	l.SetTarget(offsetTarget(t, m, q0, r3.Vector{X: 30, Y: 30, Z: -25}, deg2rad(10)))
	runTicks(t, l, 150)

	dt := 1 / l.cfg.RateHz
	minStep := make([]float64, len(q0))
	for i := range minStep {
		lim := deg2rad(l.cfg.MaxJointDegPerS[i]) * dt
		minStep[i] = 0.1 * lim
		prev := q0
		for k, q := range sim.sends {
			if math.Abs(q[i]-prev[i]) > lim+1e-12 {
				t.Fatalf("send %d joint %d step %.4f rad exceeds limit %.4f", k, i, q[i]-prev[i], lim)
			}
			prev = q
		}
	}
	seconds := float64(len(sim.sends)) * dt
	for j, n := range Oscillations(append([][]float64{q0}, sim.sends...), minStep) {
		if float64(n) > 2*seconds {
			t.Errorf("joint %d oscillated %d times in %.1f s", j, n, seconds)
		}
	}
}

func TestResyncWhenArmLags(t *testing.T) {
	l, sim, m := newTestLoop(t, home, LoopConfig{ResyncDeg: 1})
	sim.ignore = 20 // arm ignores the first 20 commands
	l.SetTarget(offsetTarget(t, m, home, r3.Vector{X: 30, Y: 30, Z: -25}, deg2rad(10)))
	runTicks(t, l, 300)
	s := l.Status()
	if s.Resyncs == 0 {
		t.Fatal("want at least one resync")
	}
	if s.State != StateConverged {
		t.Fatalf("state = %s err = %.2f mm %.2f°, want converged after resync", s.State, s.ErrMM, s.ErrDeg)
	}
}

func TestStallsOnUnreachableTarget(t *testing.T) {
	l, _, m := newTestLoop(t, home, LoopConfig{})
	l.SetTarget(offsetTarget(t, m, home, r3.Vector{X: 3000}, 0)) // 3 m away; UR5e reach is ~0.85 m
	runTicks(t, l, 1000)                                         // ~16 s to swing out to full reach, then stalls
	if s := l.Status().State; s != StateStalled {
		t.Fatalf("state = %s, want stalled", s)
	}
}

func TestRunStopsArmOnCancel(t *testing.T) {
	l, sim, _ := newTestLoop(t, home, LoopConfig{})
	ctx, cancel := context.WithTimeout(context.Background(), 100*time.Millisecond)
	defer cancel()
	if err := l.Run(ctx); !errors.Is(err, context.DeadlineExceeded) {
		t.Fatalf("err = %v, want deadline exceeded", err)
	}
	if !sim.stopped {
		t.Fatal("arm not stopped")
	}
}

func TestRunFaultsAndStopsOnSendError(t *testing.T) {
	l, sim, m := newTestLoop(t, home, LoopConfig{})
	sim.sendErr = errors.New("bus error")
	l.SetTarget(offsetTarget(t, m, home, r3.Vector{X: 10}, 0))
	if err := l.Run(context.Background()); !errors.Is(err, sim.sendErr) {
		t.Fatalf("err = %v, want bus error", err)
	}
	if !sim.stopped || l.Status().State != StateFaulted {
		t.Fatalf("stopped = %v state = %s, want stopped and faulted", sim.stopped, l.Status().State)
	}
}

// The CLI's ≥ 50 Hz pass check depends on this reading nominal, not just under it.
func TestRateReadsNominal(t *testing.T) {
	l, _, _ := newTestLoop(t, home, LoopConfig{})
	ctx, cancel := context.WithTimeout(context.Background(), time.Second)
	defer cancel()
	_ = l.Run(ctx)
	if r := l.Status().RateHz; r < 49 || r > 51 { // wide enough for scheduler jitter under -race
		t.Fatalf("rate = %.2f Hz, want ≈ 50", r)
	}
}
```

What the tests cover:
- **`TestConvergesOnStaticTarget`:** about 50 mm / 10° converges to ≤ 2 mm / 2° within 3 s.
- **`TestSingularityRespectsLimitsWithoutOscillation`:** starts at q5 = 0, checks every step against `MaxJoint·dt`, and allows ≤ 2 oscillations per joint per second.
- **`TestResyncWhenArmLags`:** `simIO` ignores 20 sends, a resync fires, and the loop still converges. `ResyncDeg` is set to 1 here because at the default 5.4° a 20-tick lag doesn't diverge enough to trigger.
- **`TestStallsOnUnreachableTarget`:** a target 3 m away. The arm takes about 16 s of simulated time to swing out to full reach before it stalls, hence 1000 ticks.
- **`Run` tests:** canceling stops the arm; a send error leads to `faulted` and the arm stops; the measured rate reads about 50 Hz. The rate is computed from the first tick to the last tick, because ticks ÷ time since `Run` started always reads slightly low.

- [ ] **Step 2: Run them and watch them fail**

Run: `go test -run 'Loop|Idle|Converges|Singularity|Resync|Stalls|RunStops|RunFaults' .`
Expected: build failure, `undefined: NewLoop` (along with `LoopConfig`, `StateIdle`, and so on).

- [ ] **Step 3: Write `loop.go`** and delete the temporary `deg2rad` from `kin.go`:

```go
package diffik

import (
	"context"
	"errors"
	"fmt"
	"math"
	"slices"
	"sync"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// JointIO is how the loop talks to an arm: read measured joints, send a joint target, stop.
type JointIO interface {
	Read(ctx context.Context) ([]referenceframe.Input, error)
	Send(ctx context.Context, q []referenceframe.Input) error
	Stop(ctx context.Context) error
}

// LoopConfig holds the loop's calibration knobs. Zero values take the defaults in withDefaults.
type LoopConfig struct {
	RateHz            float64
	TaskWeights       [6]float64 // x, y, z, rx, ry, rz over m / rad; all-zero means all 1
	Gain              float64    // 1/s
	MaxLinearMMPerS   float64
	MaxAngularDegPerS float64
	MaxJointDegPerS   []float64 // one per joint
	SingularEps       float64
	MaxDamping        float64
	ResyncDeg         float64
	TolMM             float64
	TolDeg            float64
}

func (c LoopConfig) withDefaults(dof int) LoopConfig {
	def := func(v *float64, d float64) {
		if *v == 0 {
			*v = d
		}
	}
	def(&c.RateHz, 50)
	if c.TaskWeights == ([6]float64{}) {
		c.TaskWeights = [6]float64{1, 1, 1, 1, 1, 1}
	}
	def(&c.Gain, 2)
	def(&c.MaxLinearMMPerS, 100)
	def(&c.MaxAngularDegPerS, 45)
	if c.MaxJointDegPerS == nil {
		c.MaxJointDegPerS = make([]float64, dof)
		for i := range c.MaxJointDegPerS {
			c.MaxJointDegPerS[i] = 90
		}
	}
	def(&c.SingularEps, 0.04)
	def(&c.MaxDamping, 0.1)
	if c.ResyncDeg == 0 && c.RateHz > 0 {
		c.ResyncDeg = 3 * slices.Max(c.MaxJointDegPerS) / c.RateHz
	}
	def(&c.TolMM, 2)
	def(&c.TolDeg, 2)
	return c
}

func (c LoopConfig) validate(dof int) error {
	if c.RateHz <= 0 {
		return fmt.Errorf("rate_hz must be > 0, got %v", c.RateHz)
	}
	for i, w := range c.TaskWeights {
		if w < 0 {
			return fmt.Errorf("task weight %d is negative", i)
		}
	}
	if len(c.MaxJointDegPerS) != dof {
		return fmt.Errorf("need %d joint velocity limits, got %d", dof, len(c.MaxJointDegPerS))
	}
	return nil
}

// State is the loop's externally visible state.
type State string

// Loop states.
const (
	StateIdle      State = "idle"
	StateRunning   State = "running"
	StateConverged State = "converged"
	StateStalled   State = "stalled"
	StateFaulted   State = "faulted"
)

// Status is a snapshot of the loop. Errors are measured (from read-back joints) unless prefixed Cmd.
type Status struct {
	State               State
	TargetMM            r3.Vector
	ErrMM, ErrDeg       float64
	CmdErrMM, CmdErrDeg float64
	RateHz              float64
	TickP50, TickP99    time.Duration
	Overruns, Resyncs   int
	Damping             float64
	Limited             bool
	QCmd, QMeas         []float64
}

const tickWindow = 500 // ticks kept for p50/p99

// Loop is a differential-IK servo loop for one arm.
type Loop struct {
	model  referenceframe.Model
	io     JointIO
	cfg    LoopConfig
	lo, hi []float64

	// OnTick, if set before Run, is called after every tick with a status snapshot.
	OnTick func(Status)

	qCmd    []float64
	errRing []float64 // weighted measured error norm, one per tick, for stall detection

	mu       sync.Mutex
	target   spatialmath.Pose
	status   Status
	tickDurs []time.Duration
	ticks    int
	first    time.Time // start of the first tick
	last     time.Time // start of the latest tick
}

// NewLoop builds a loop for model, driven through io.
func NewLoop(model referenceframe.Model, io JointIO, cfg LoopConfig) (*Loop, error) {
	limits := model.DoF()
	dof := len(limits)
	cfg = cfg.withDefaults(dof)
	if err := cfg.validate(dof); err != nil {
		return nil, err
	}
	l := &Loop{model: model, io: io, cfg: cfg, lo: make([]float64, dof), hi: make([]float64, dof)}
	for i, lim := range limits {
		// stay fdStep inside the limits so Jacobian perturbations remain valid inputs
		l.lo[i], l.hi[i] = lim.Min+fdStep, lim.Max-fdStep
	}
	l.status.State = StateIdle
	return l, nil
}

// SetTarget sets the pose (mm, base frame) the loop servos to. Latest wins.
func (l *Loop) SetTarget(p spatialmath.Pose) {
	l.mu.Lock()
	defer l.mu.Unlock()
	l.target = p
	l.errRing = l.errRing[:0]
}

// Config returns the loop's configuration with defaults filled in.
func (l *Loop) Config() LoopConfig { return l.cfg }

// Status returns a snapshot of the loop.
func (l *Loop) Status() Status {
	l.mu.Lock()
	defer l.mu.Unlock()
	s := l.status
	s.QCmd = slices.Clone(s.QCmd)
	s.QMeas = slices.Clone(s.QMeas)
	if n := len(l.tickDurs); n > 0 {
		d := slices.Clone(l.tickDurs)
		slices.Sort(d)
		s.TickP50, s.TickP99 = d[n/2], d[n*99/100]
	}
	// Rate over first→last tick: ticks/elapsed-since-Run would always read just under nominal.
	if span := l.last.Sub(l.first).Seconds(); l.ticks > 1 && span > 0 {
		s.RateHz = float64(l.ticks-1) / span
	}
	return s
}

// Run ticks at cfg.RateHz until ctx is done or the loop faults. It always stops the arm before returning.
func (l *Loop) Run(ctx context.Context) error {
	dt := 1 / l.cfg.RateHz
	period := time.Duration(dt * float64(time.Second))
	t := time.NewTicker(period) // drops ticks we miss; never bursts to catch up
	defer t.Stop()
	for {
		select {
		case <-ctx.Done():
			return l.stop(ctx.Err())
		case <-t.C:
		}
		start := time.Now()
		if err := l.tick(ctx, dt); err != nil {
			if ctx.Err() != nil {
				return l.stop(ctx.Err())
			}
			l.mu.Lock()
			l.status.State = StateFaulted
			l.mu.Unlock()
			return l.stop(err)
		}
		d := time.Since(start)
		l.mu.Lock()
		if l.ticks == 0 {
			l.first = start
		}
		l.last = start
		l.ticks++
		if len(l.tickDurs) == tickWindow {
			l.tickDurs = l.tickDurs[1:]
		}
		l.tickDurs = append(l.tickDurs, d)
		if d > period {
			l.status.Overruns++
		}
		l.mu.Unlock()
		if l.OnTick != nil {
			l.OnTick(l.Status())
		}
	}
}

func (l *Loop) stop(cause error) error {
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	if err := l.io.Stop(ctx); err != nil {
		return errors.Join(cause, fmt.Errorf("stopping arm: %w", err))
	}
	return cause
}

// tick runs one control step. See the spec's per-tick data flow; step numbers match.
func (l *Loop) tick(ctx context.Context, dt float64) error {
	// 1. measured joints
	qm, err := l.io.Read(ctx)
	if err != nil {
		return fmt.Errorf("reading joints: %w", err)
	}
	if len(qm) != len(l.lo) {
		return fmt.Errorf("arm reported %d joints, model has %d", len(qm), len(l.lo))
	}
	l.mu.Lock()
	target := l.target
	l.mu.Unlock()
	if l.qCmd == nil {
		l.qCmd = slices.Clone(qm)
	}
	if target == nil {
		l.mu.Lock()
		l.status.State = StateIdle
		l.status.QMeas = slices.Clone(qm)
		l.mu.Unlock()
		return nil
	}

	// 2. hybrid resync
	resynced := false
	if maxAbsDiff(l.qCmd, qm) > deg2rad(l.cfg.ResyncDeg) {
		copy(l.qCmd, qm)
		resynced = true
	}

	// 3. error at commanded state
	tp, tq := target.Point().Mul(1/mmPerM), target.Orientation().Quaternion()
	cp, cq, err := FK(l.model, l.qCmd)
	if err != nil {
		return err
	}
	e := PoseError(tp, tq, cp, cq)

	// 4–5. capped task velocity → task step
	lin := capNorm(r3.Vector{X: e[0], Y: e[1], Z: e[2]}.Mul(l.cfg.Gain), l.cfg.MaxLinearMMPerS/mmPerM)
	ang := capNorm(r3.Vector{X: e[3], Y: e[4], Z: e[5]}.Mul(l.cfg.Gain), deg2rad(l.cfg.MaxAngularDegPerS))
	dx := [6]float64{lin.X * dt, lin.Y * dt, lin.Z * dt, ang.X * dt, ang.Y * dt, ang.Z * dt}

	// 6. weighted DLS
	J, err := Jacobian(l.model, l.qCmd)
	if err != nil {
		return err
	}
	dq, lambda, sigmaMin, err := dlsStep(J, l.cfg.TaskWeights, dx, l.cfg.SingularEps, l.cfg.MaxDamping)
	if err != nil {
		return err
	}

	// 7. joint velocity clamp, direction kept
	s := 1.0
	for i, d := range dq {
		if math.IsNaN(d) || math.IsInf(d, 0) {
			return fmt.Errorf("non-finite joint step %v", dq)
		}
		if lim := deg2rad(l.cfg.MaxJointDegPerS[i]) * dt; math.Abs(d)*s > lim {
			s = lim / math.Abs(d)
		}
	}

	// 8. position limits
	next := make([]float64, len(dq))
	clamped := false
	for i, d := range dq {
		v := l.qCmd[i] + d*s
		if v < l.lo[i] || v > l.hi[i] {
			v = math.Min(math.Max(v, l.lo[i]), l.hi[i])
			clamped = true
		}
		next[i] = v
	}

	// 9. send
	if err := l.io.Send(ctx, next); err != nil {
		return fmt.Errorf("sending joints: %w", err)
	}
	copy(l.qCmd, next)

	// 10. measured error
	mp, mq, err := FK(l.model, qm)
	if err != nil {
		return err
	}
	em := PoseError(tp, tq, mp, mq)
	limited := s < 1 || clamped || sigmaMin < l.cfg.SingularEps

	l.mu.Lock()
	defer l.mu.Unlock()
	st := &l.status
	if resynced {
		st.Resyncs++
	}
	st.TargetMM = target.Point()
	st.ErrMM, st.ErrDeg = posMM(em), angDeg(em)
	st.CmdErrMM, st.CmdErrDeg = posMM(e), angDeg(e)
	st.Damping, st.Limited = lambda, limited
	st.QCmd, st.QMeas = slices.Clone(l.qCmd), slices.Clone(qm)
	st.State = l.nextState(em, limited)
	return nil
}

// nextState picks converged/stalled/running. Caller holds l.mu.
func (l *Loop) nextState(em [6]float64, limited bool) State {
	if posMM(em) <= l.cfg.TolMM && angDeg(em) <= l.cfg.TolDeg {
		l.errRing = l.errRing[:0]
		return StateConverged
	}
	norm := 0.0
	for i, v := range em {
		norm += l.cfg.TaskWeights[i] * v * v
	}
	norm = math.Sqrt(norm)
	window := int(l.cfg.RateHz) // 1 s of ticks
	if len(l.errRing) == window {
		l.errRing = l.errRing[1:]
	}
	l.errRing = append(l.errRing, norm)
	if limited && len(l.errRing) == window && norm > 0.99*l.errRing[0] {
		return StateStalled
	}
	return StateRunning
}

// Oscillations counts, per joint, sign reversals between successive steps whose size exceeds minStep[i].
// Smaller steps are skipped so jitter near rest is not counted.
func Oscillations(qs [][]float64, minStep []float64) []int {
	out := make([]int, len(minStep))
	for i := range minStep {
		prev := 0.0
		for k := 1; k < len(qs); k++ {
			d := qs[k][i] - qs[k-1][i]
			if math.Abs(d) <= minStep[i] {
				continue
			}
			if prev != 0 && (d > 0) != (prev > 0) {
				out[i]++
			}
			prev = d
		}
	}
	return out
}

func deg2rad(d float64) float64  { return d * math.Pi / 180 }
func posMM(e [6]float64) float64 { return math.Sqrt(e[0]*e[0]+e[1]*e[1]+e[2]*e[2]) * mmPerM }
func angDeg(e [6]float64) float64 {
	return math.Sqrt(e[3]*e[3]+e[4]*e[4]+e[5]*e[5]) * 180 / math.Pi
}

func capNorm(v r3.Vector, max float64) r3.Vector {
	if n := v.Norm(); n > max {
		return v.Mul(max / n)
	}
	return v
}

func maxAbsDiff(a, b []float64) float64 {
	m := 0.0
	for i := range a {
		m = math.Max(m, math.Abs(a[i]-b[i]))
	}
	return m
}
```

- [ ] **Step 4: Run the whole package with the race detector**

Run: `go test -race -count=1 .`
Expected: `ok  diffik`. The `Run` tests use real time (about 1 s in total) and everything else is simulated.

- [ ] **Step 5: Commit**

```bash
git add loop.go loop_test.go kin.go
git commit -m "Add hybrid differential-IK servo loop with limits, damping, resync and stall detection

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_012FAJrCqoTNZgDVMfHN6sDY"
```

---

### Task 4: Streamed-move arm adapter

`armIO` opens `MoveThroughJointPositionsStreamed` on its first `Send`:
- The first batch is `[current joints at t=0, target at t=dt]`.
- Each later `Send` adds one point due `dt` after now.
- `Stop` **closes `batches`** and also cancels the stream context, because the fake arm only watches the channel. Without the close, `Stop` hangs forever. Then it calls `arm.Stop`.

**Files:**
- Create: `armio_test.go`
- Create: `armio.go`

- [ ] **Step 1: Write the failing test** in `armio_test.go`:

```go
package diffik

import (
	"context"
	"slices"
	"testing"
	"time"

	"go.viam.com/rdk/components/arm/fake"
	"go.viam.com/rdk/components/arm/kinematics"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
)

// The fake arm teleports to each streamed waypoint, so this checks armIO's stream plumbing.
func TestArmIOStreamsToFakeArm(t *testing.T) {
	ctx := context.Background()
	cfg := resource.Config{Name: "arm", ConvertedAttributes: &fake.Config{ArmModel: kinematics.UR5e}}
	a, err := fake.NewArm(ctx, nil, cfg, logging.NewTestLogger(t))
	if err != nil {
		t.Fatal(err)
	}
	io := NewArmIO(a, 50)
	want := []float64{0.1, -1.2, 1.5, -1.9, -1.57, 0}
	for _, q := range [][]float64{{0, -1.2, 1.5, -1.9, -1.57, 0}, want} {
		if err := io.Send(ctx, q); err != nil {
			t.Fatal(err)
		}
	}
	// Send returns once the batch is handed over; give the fake a moment to apply it.
	deadline := time.Now().Add(time.Second)
	for {
		got, err := io.Read(ctx)
		if err != nil {
			t.Fatal(err)
		}
		if slices.Equal(got, want) {
			break
		}
		if time.Now().After(deadline) {
			t.Fatalf("joints = %v, want %v", got, want)
		}
		time.Sleep(5 * time.Millisecond)
	}
	if err := io.Stop(ctx); err != nil {
		t.Fatal(err)
	}
}
```

- [ ] **Step 2: Run it and watch it fail**

Run: `go test -run ArmIO .`
Expected: build failure, `undefined: NewArmIO`.

- [ ] **Step 3: Write `armio.go`**

```go
package diffik

import (
	"context"
	"errors"
	"fmt"
	"slices"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/referenceframe"
)

// armIO drives a real arm through MoveThroughJointPositionsStreamed.
// The stream opens on the first Send; each Send adds one waypoint due one tick later.
type armIO struct {
	a  arm.Arm
	dt time.Duration

	batches chan []arm.TrajectoryPoint
	done    chan struct{}
	err     error // stream's return value; read only after done is closed
	cancel  context.CancelFunc
	start   time.Time
	lastT   time.Duration
}

// NewArmIO returns a JointIO for a, pacing waypoints at rateHz.
func NewArmIO(a arm.Arm, rateHz float64) JointIO {
	return &armIO{a: a, dt: time.Duration(float64(time.Second) / rateHz)}
}

func (io *armIO) Read(ctx context.Context) ([]referenceframe.Input, error) {
	return io.a.JointPositions(ctx, nil)
}

func (io *armIO) Send(ctx context.Context, q []referenceframe.Input) error {
	var batch []arm.TrajectoryPoint
	if io.batches == nil {
		cur, err := io.a.JointPositions(ctx, nil)
		if err != nil {
			return err
		}
		io.open()
		batch = append(batch, arm.TrajectoryPoint{Time: 0, Positions: cur}) // first point must be t=0
	}
	t := time.Since(io.start) + io.dt
	if t <= io.lastT {
		t = io.lastT + time.Microsecond // times must strictly increase
	}
	io.lastT = t
	batch = append(batch, arm.TrajectoryPoint{Time: t, Positions: slices.Clone(q)})

	select {
	case io.batches <- batch:
		return nil
	case <-io.done:
		if io.err == nil {
			return errors.New("arm stream ended unexpectedly")
		}
		return fmt.Errorf("arm stream: %w", io.err)
	case <-ctx.Done():
		return ctx.Err()
	}
}

// open starts the streamed move. It runs on its own context so that a canceled loop
// context doesn't tear the stream down before Stop has run.
func (io *armIO) open() {
	ctx, cancel := context.WithCancel(context.Background())
	io.cancel = cancel
	io.batches = make(chan []arm.TrajectoryPoint)
	io.done = make(chan struct{})
	io.start, io.lastT = time.Now(), 0
	responses := make(chan arm.Response)
	go func() {
		for range responses { // acks carry nothing we need
		}
	}()
	go func() {
		io.err = io.a.MoveThroughJointPositionsStreamed(ctx, io.batches, responses, nil)
		close(responses)
		close(io.done)
	}()
}

func (io *armIO) Stop(ctx context.Context) error {
	if io.batches != nil {
		// Close batches as well as cancel: drivers may only watch the channel (the fake arm does).
		// Send and Stop both run on the loop goroutine, so nothing can send after this close.
		close(io.batches)
		io.cancel()
		select {
		case <-io.done:
		case <-ctx.Done():
		}
		io.batches = nil
	}
	return io.a.Stop(ctx, nil)
}
```

- [ ] **Step 4: Run it and watch it pass**

Run: `go test -race -count=1 -timeout 120s -run ArmIO -v .`
Expected: `TestArmIOStreamsToFakeArm` PASS. The first race build of the fake arm package can take a few minutes. A hang that ends at the timeout means `Stop` isn't closing `batches`.

- [ ] **Step 5: Commit**

```bash
git add armio.go armio_test.go
git commit -m "Add JointIO adapter over MoveThroughJointPositionsStreamed

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_012FAJrCqoTNZgDVMfHN6sDY"
```

---

### Task 5: CLI connection and the `validate` command (Phase 0)

The scaffold CLI never connects to a machine; it only builds empty dependencies. This task replaces it.
- **Connection:** `--address` (default `localhost:8080`) and `--arm`. API-key credentials come from `VIAM_API_KEY_ID` / `VIAM_API_KEY` and are optional for a local server that doesn't require auth.
- **`validate`:** 100 random configurations within `--spread` (default 0.6) of each joint's half-range around the start pose. At each one it compares `FK` with `EndPosition`, then nudges one joint by 1° and compares the measured motion with `J·Δq_measured`.

**Files:**
- Modify (replace): `cmd/cli/main.go`
- Create: `cmd/cli/validate.go`

- [ ] **Step 1: Replace `cmd/cli/main.go`**

```go
// Command cli is the Phase 0/1 harness: it connects to a machine, validates the arm's
// kinematics (validate) and runs servo-loop scenarios against it (run static|singular|circle).
package main

import (
	"context"
	"errors"
	"flag"
	"fmt"
	"os"
	"os/signal"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/utils/rpc"
)

const usage = `usage: cli [flags] validate
       cli [flags] run static|singular|circle`

func main() {
	if err := realMain(); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}

func realMain() error {
	address := flag.String("address", "localhost:8080", "machine address")
	armName := flag.String("arm", "", "arm component name (required)")
	rate := flag.Float64("rate", 50, "loop rate in Hz")
	csvPath := flag.String("csv", "", "write one row per tick to this file (run only)")
	spread := flag.Float64("spread", 0.6, "validate: fraction of each joint's half-range to sample around the start pose")
	wrist := flag.Int("wrist", 4, "singular: index of the wrist joint whose zero is singular")
	flag.Parse()
	args := flag.Args()
	if *armName == "" || len(args) == 0 {
		return errors.New(usage)
	}

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt)
	defer stop()
	logger := logging.NewLogger("cli")

	var opts []client.RobotClientOption
	if id, key := os.Getenv("VIAM_API_KEY_ID"), os.Getenv("VIAM_API_KEY"); id != "" {
		opts = append(opts, client.WithDialOptions(rpc.WithEntityCredentials(id,
			rpc.Credentials{Type: rpc.CredentialsTypeAPIKey, Payload: key})))
	}
	machine, err := client.New(ctx, *address, logger, opts...)
	if err != nil {
		return fmt.Errorf("connecting to %s: %w", *address, err)
	}
	defer machine.Close(context.Background())

	a, err := arm.FromProvider(machine, *armName)
	if err != nil {
		return err
	}
	model, err := a.Kinematics(ctx)
	if err != nil {
		return fmt.Errorf("getting kinematics: %w", err)
	}

	switch {
	case args[0] == "validate":
		return validate(ctx, a, model, *spread)
	case args[0] == "run" && len(args) == 2:
		return run(ctx, a, model, args[1], *rate, *csvPath, *wrist)
	default:
		return errors.New(usage)
	}
}
```

- [ ] **Step 2: Write `cmd/cli/validate.go`**

```go
package main

import (
	"context"
	"fmt"
	"math"
	"math/rand"
	"slices"

	"diffik"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"gonum.org/v1/gonum/mat"
)

// validate is Phase 0: our FK vs the arm's EndPosition, and J·Δq vs the measured motion.
func validate(ctx context.Context, a arm.Arm, model referenceframe.Model, spread float64) error {
	start, err := a.JointPositions(ctx, nil)
	if err != nil {
		return err
	}
	limits := model.DoF()
	rng := rand.New(rand.NewSource(1))
	var worstMM, worstDeg, worstLin, worstAng float64
	const configs = 100
	for k := 0; k < configs; k++ {
		q := make([]float64, len(start))
		for i, lim := range limits {
			half := spread * (lim.Max - lim.Min) / 2
			q[i] = clamp(start[i]+(rng.Float64()*2-1)*half, lim.Min, lim.Max)
		}
		qm, ep, err := moveAndMeasure(ctx, a, q)
		if err != nil {
			return fmt.Errorf("config %d: %w", k, err)
		}
		p, o, err := diffik.FK(model, qm)
		if err != nil {
			return err
		}
		e := diffik.PoseError(p, o, ep.Point().Mul(1e-3), ep.Orientation().Quaternion())
		worstMM = math.Max(worstMM, vecMM(e))
		worstDeg = math.Max(worstDeg, vecDeg(e))

		// Jacobian: nudge one joint by 1° and compare predicted vs measured motion.
		j := k % len(q)
		q2 := slices.Clone(qm)
		q2[j] = clamp(q2[j]+math.Pi/180, limits[j].Min, limits[j].Max)
		qm2, ep2, err := moveAndMeasure(ctx, a, q2)
		if err != nil {
			return fmt.Errorf("config %d nudge: %w", k, err)
		}
		J, err := diffik.Jacobian(model, qm)
		if err != nil {
			return err
		}
		dq := make([]float64, len(qm))
		for i := range dq {
			dq[i] = qm2[i] - qm[i] // measured, not commanded, joint change
		}
		var pred mat.VecDense
		pred.MulVec(J, mat.NewVecDense(len(dq), dq))
		actLin := ep2.Point().Sub(ep.Point()).Mul(1e-3)
		actAng := diffik.RotVec(ep2.Orientation().Quaternion(), ep.Orientation().Quaternion())
		if actLin.Norm() > 1e-5 {
			worstLin = math.Max(worstLin, relErr(actLin, r3.Vector{X: pred.AtVec(0), Y: pred.AtVec(1), Z: pred.AtVec(2)}))
		}
		worstAng = math.Max(worstAng, relErr(actAng, r3.Vector{X: pred.AtVec(3), Y: pred.AtVec(4), Z: pred.AtVec(5)}))
	}
	if _, _, err := moveAndMeasure(ctx, a, start); err != nil {
		return err
	}

	fmt.Printf("configs:            %d\n", configs)
	fmt.Printf("FK position error:  %.3f mm  (pass ≤ 0.1)  %s\n", worstMM, pass(worstMM <= 0.1))
	fmt.Printf("FK angle error:     %.3f °   (pass ≤ 0.1)  %s\n", worstDeg, pass(worstDeg <= 0.1))
	fmt.Printf("Jacobian linear:    %.1f %%   (pass ≤ 5)    %s\n", worstLin*100, pass(worstLin <= 0.05))
	fmt.Printf("Jacobian angular:   %.1f %%   (pass ≤ 5)    %s\n", worstAng*100, pass(worstAng <= 0.05))
	if worstMM > 0.1 || worstDeg > 0.1 || worstLin > 0.05 || worstAng > 0.05 {
		return fmt.Errorf("validate failed")
	}
	return nil
}

func moveAndMeasure(ctx context.Context, a arm.Arm, q []float64) ([]float64, spatialmath.Pose, error) {
	if err := a.MoveToJointPositions(ctx, q, nil); err != nil {
		return nil, nil, err
	}
	qm, err := a.JointPositions(ctx, nil)
	if err != nil {
		return nil, nil, err
	}
	ep, err := a.EndPosition(ctx, nil)
	return qm, ep, err
}

func clamp(v, lo, hi float64) float64    { return math.Min(math.Max(v, lo), hi) }
func relErr(act, pred r3.Vector) float64 { return act.Sub(pred).Norm() / act.Norm() }
func vecMM(e [6]float64) float64         { return r3.Vector{X: e[0], Y: e[1], Z: e[2]}.Norm() * 1000 }
func vecDeg(e [6]float64) float64 {
	return r3.Vector{X: e[3], Y: e[4], Z: e[5]}.Norm() * 180 / math.Pi
}

func pass(ok bool) string {
	if ok {
		return "PASS"
	}
	return "FAIL"
}
```

- [ ] **Step 3: Add a stub `run`** so the package builds before Task 6. Create `cmd/cli/run.go` containing:

```go
package main

import (
	"context"
	"errors"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/referenceframe"
)

func run(context.Context, arm.Arm, referenceframe.Model, string, float64, string, int) error {
	return errors.New("not implemented yet")
}
```

- [ ] **Step 4: Build and vet**

Run: `go build ./... && go vet ./...`
Expected: no output.

Run: `go run ./cmd/cli`
Expected: prints the usage text and exits 1, because `--arm` is missing.

- [ ] **Step 5: Commit**

```bash
git add cmd/cli
git commit -m "Add CLI machine connection and Phase 0 validate command

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_012FAJrCqoTNZgDVMfHN6sDY"
```

---

### Task 6: CLI `run` scenarios (Phase 1)

Each scenario starts the loop with `NewArmIO` in a goroutine, drives targets, then cancels the loop, which stops the arm, and prints a summary: rate (pass ≥ 50 Hz), p50/p99 tick time, overruns and resyncs. `--csv` writes one row per tick with these columns:

`elapsed_s, state, target_x/y/z (mm), err_mm, err_deg, cmd_err_mm, cmd_err_deg, damping, limited, q_cmd…, q_meas…`

| Scenario | What it does |
|---|---|
| `static` | 10 seeded targets, each 50 mm in a random direction and rotated ≤ 10° about a random axis. Returns to the start pose between targets. Each target must converge within 10 s. |
| `singular` | Before the loop starts, moves the arm so joint `--wrist` (default 4) is at +0.2 rad. Then targets the pose with that joint at −0.2 rad. Reports peak joint velocity against the limit, and oscillations as defined in the spec (pass ≤ 2 per joint per second). |
| `circle` | Converges to the circle's start point, then moves the target along a 50 mm-radius circle at 50 mm/s for 2 laps, updating at the loop rate. Reports mean and max tracking error, and lag as mean error ÷ speed. |

**Files:**
- Modify (replace stub): `cmd/cli/run.go`
- Modify: `loop.go` (add `Config()`; it's already in the Task 3 listing, so skip if present)

- [ ] **Step 1: Replace `cmd/cli/run.go`**

```go
package main

import (
	"context"
	"encoding/csv"
	"errors"
	"fmt"
	"math"
	"math/rand"
	"os"
	"slices"
	"strconv"
	"sync"
	"time"

	"diffik"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"gonum.org/v1/gonum/num/quat"
)

// recorder keeps every tick's status (and optionally writes it as CSV). OnTick runs on the
// loop goroutine while scenarios read from another, hence the mutex.
type recorder struct {
	mu    sync.Mutex
	start time.Time
	rows  []diffik.Status
	csv   *csv.Writer
}

func (r *recorder) add(s diffik.Status) {
	r.mu.Lock()
	defer r.mu.Unlock()
	r.rows = append(r.rows, s)
	if r.csv == nil {
		return
	}
	f := func(v float64) string { return strconv.FormatFloat(v, 'f', 6, 64) }
	row := []string{f(time.Since(r.start).Seconds()), string(s.State),
		f(s.TargetMM.X), f(s.TargetMM.Y), f(s.TargetMM.Z),
		f(s.ErrMM), f(s.ErrDeg), f(s.CmdErrMM), f(s.CmdErrDeg), f(s.Damping), strconv.FormatBool(s.Limited)}
	for _, v := range s.QCmd {
		row = append(row, f(v))
	}
	for _, v := range s.QMeas {
		row = append(row, f(v))
	}
	_ = r.csv.Write(row)
}

func (r *recorder) since(i int) []diffik.Status {
	r.mu.Lock()
	defer r.mu.Unlock()
	return slices.Clone(r.rows[i:])
}

func (r *recorder) len() int {
	r.mu.Lock()
	defer r.mu.Unlock()
	return len(r.rows)
}

func run(ctx context.Context, a arm.Arm, model referenceframe.Model, scenario string, rate float64, csvPath string, wrist int) error {
	var body func(context.Context, *diffik.Loop, *recorder, spatialmath.Pose) error
	switch scenario {
	case "static":
		body = static
	case "singular":
		var err error
		if body, err = singular(ctx, a, model, wrist); err != nil {
			return err
		}
	case "circle":
		body = circle
	default:
		return fmt.Errorf("unknown scenario %q", scenario)
	}

	l, err := diffik.NewLoop(model, diffik.NewArmIO(a, rate), diffik.LoopConfig{RateHz: rate})
	if err != nil {
		return err
	}
	rec := &recorder{start: time.Now()}
	if csvPath != "" {
		f, err := os.Create(csvPath)
		if err != nil {
			return err
		}
		defer f.Close()
		rec.csv = csv.NewWriter(f)
		defer rec.csv.Flush()
	}
	l.OnTick = rec.add

	q, err := a.JointPositions(ctx, nil)
	if err != nil {
		return err
	}
	p, o, err := diffik.FK(model, q)
	if err != nil {
		return err
	}
	start := poseMM(p, o)

	runCtx, cancel := context.WithCancel(ctx)
	defer cancel()
	loopErr := make(chan error, 1)
	go func() { loopErr <- l.Run(runCtx) }()

	scenarioErr := body(runCtx, l, rec, start)
	cancel()
	if err := <-loopErr; err != nil && !errors.Is(err, context.Canceled) {
		return errors.Join(scenarioErr, err)
	}

	s := l.Status()
	fmt.Printf("rate:     %.1f Hz (pass ≥ 50) %s\n", s.RateHz, pass(s.RateHz >= 50))
	fmt.Printf("tick:     p50 %v  p99 %v\n", s.TickP50, s.TickP99)
	fmt.Printf("overruns: %d   resyncs: %d\n", s.Overruns, s.Resyncs)
	return scenarioErr
}

// waitConverged blocks until the loop has converged on target, or timeout passes.
func waitConverged(ctx context.Context, l *diffik.Loop, target spatialmath.Pose, timeout time.Duration) (time.Duration, bool) {
	t0 := time.Now()
	for time.Since(t0) < timeout && ctx.Err() == nil {
		s := l.Status()
		if s.State == diffik.StateConverged && s.TargetMM == target.Point() {
			return time.Since(t0), true
		}
		time.Sleep(10 * time.Millisecond)
	}
	return time.Since(t0), false
}

// static: 10 targets 50 mm and ≤ 10° from start, returning to start between them.
func static(ctx context.Context, l *diffik.Loop, _ *recorder, start spatialmath.Pose) error {
	rng := rand.New(rand.NewSource(1))
	failed := 0
	for i := 0; i < 10; i++ {
		dir := randUnit(rng)
		rot := axisAngle(randUnit(rng), rng.Float64()*10*math.Pi/180)
		target := spatialmath.NewPose(start.Point().Add(dir.Mul(50)), quatOrient(quat.Mul(rot, start.Orientation().Quaternion())))
		l.SetTarget(target)
		took, ok := waitConverged(ctx, l, target, 10*time.Second)
		s := l.Status()
		fmt.Printf("target %2d: %-9s in %5.2fs  err %.2f mm %.2f°  %s\n", i, s.State, took.Seconds(), s.ErrMM, s.ErrDeg, pass(ok))
		if !ok {
			failed++
		}
		l.SetTarget(start)
		waitConverged(ctx, l, start, 10*time.Second)
	}
	if failed > 0 {
		return fmt.Errorf("%d of 10 targets did not converge", failed)
	}
	return nil
}

// singular moves the arm so the wrist joint sits at +0.2 rad, then (once the loop runs) targets the
// pose with the wrist at −0.2 rad, so the Cartesian path passes close to the wrist singularity.
func singular(ctx context.Context, a arm.Arm, model referenceframe.Model, wrist int) (func(context.Context, *diffik.Loop, *recorder, spatialmath.Pose) error, error) {
	q, err := a.JointPositions(ctx, nil)
	if err != nil {
		return nil, err
	}
	if wrist < 0 || wrist >= len(q) {
		return nil, fmt.Errorf("wrist index %d out of range for %d joints", wrist, len(q))
	}
	qa, qb := slices.Clone(q), slices.Clone(q)
	qa[wrist], qb[wrist] = 0.2, -0.2
	if err := a.MoveToJointPositions(ctx, qa, nil); err != nil {
		return nil, err
	}
	p, o, err := diffik.FK(model, qb)
	if err != nil {
		return nil, err
	}
	target := poseMM(p, o)

	return func(ctx context.Context, l *diffik.Loop, rec *recorder, _ spatialmath.Pose) error {
		from := rec.len()
		l.SetTarget(target)
		waitConverged(ctx, l, target, 10*time.Second)
		rows := rec.since(from)

		cfg := l.Config()
		dt := 1 / cfg.RateHz
		qs := [][]float64{qa}
		for _, r := range rows {
			if r.QCmd != nil {
				qs = append(qs, r.QCmd)
			}
		}
		minStep := make([]float64, len(qa))
		peakOK := true
		for i := range minStep {
			lim := cfg.MaxJointDegPerS[i]
			minStep[i] = 0.1 * lim * math.Pi / 180 * dt
			peak := 0.0
			for k := 1; k < len(qs); k++ {
				peak = math.Max(peak, math.Abs(qs[k][i]-qs[k-1][i])/dt*180/math.Pi)
			}
			fmt.Printf("joint %d: peak %.1f°/s (limit %.0f)\n", i, peak, lim)
			peakOK = peakOK && peak <= lim+1e-6
		}
		seconds := float64(len(qs)-1) * dt
		oscOK := true
		for i, n := range diffik.Oscillations(qs, minStep) {
			fmt.Printf("joint %d: %d oscillations in %.1fs\n", i, n, seconds)
			oscOK = oscOK && float64(n) <= 2*seconds
		}
		fmt.Printf("velocity limits: %s   oscillation: %s\n", pass(peakOK), pass(oscOK))
		if !peakOK || !oscOK {
			return errors.New("singular scenario failed")
		}
		return nil
	}, nil
}

// circle: converge to a point 50 mm from start, then trace a 50 mm-radius circle at 50 mm/s for 2 laps.
func circle(ctx context.Context, l *diffik.Loop, rec *recorder, start spatialmath.Pose) error {
	const radius, speed = 50.0, 50.0 // mm, mm/s
	omega := speed / radius
	at := func(theta float64) spatialmath.Pose {
		return spatialmath.NewPose(start.Point().Add(r3.Vector{X: radius * math.Cos(theta), Y: radius * math.Sin(theta)}), start.Orientation())
	}
	first := at(0)
	l.SetTarget(first)
	if _, ok := waitConverged(ctx, l, first, 10*time.Second); !ok {
		return errors.New("did not reach the circle's start point")
	}

	from := rec.len()
	period := time.Duration(float64(time.Second) / l.Config().RateHz)
	t := time.NewTicker(period)
	defer t.Stop()
	t0 := time.Now()
	for el := 0.0; el < 4*math.Pi/omega; el = time.Since(t0).Seconds() {
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-t.C:
		}
		l.SetTarget(at(omega * el))
	}

	var sum, worst float64
	rows := rec.since(from)
	for _, r := range rows {
		sum += r.ErrMM
		worst = math.Max(worst, r.ErrMM)
	}
	if len(rows) == 0 {
		return errors.New("no ticks recorded during circle")
	}
	mean := sum / float64(len(rows))
	fmt.Printf("tracking error: mean %.2f mm  max %.2f mm\n", mean, worst)
	fmt.Printf("lag (mean error / speed): %.0f ms\n", mean/speed*1000)
	return nil
}

func poseMM(p r3.Vector, o quat.Number) spatialmath.Pose {
	return spatialmath.NewPose(p.Mul(1000), quatOrient(o))
}

func quatOrient(q quat.Number) spatialmath.Orientation {
	sq := spatialmath.Quaternion(q)
	return &sq
}

func axisAngle(axis r3.Vector, angle float64) quat.Number {
	s := math.Sin(angle / 2)
	return quat.Number{Real: math.Cos(angle / 2), Imag: axis.X * s, Jmag: axis.Y * s, Kmag: axis.Z * s}
}

func randUnit(rng *rand.Rand) r3.Vector {
	return r3.Vector{X: rng.NormFloat64(), Y: rng.NormFloat64(), Z: rng.NormFloat64()}.Normalize()
}
```

- [ ] **Step 2: Build, vet and test**

Run: `gofmt -l . ; go build ./... && go vet ./... && go test -race -count=1 ./...`
Expected: `gofmt -l` prints nothing, and tests report `ok  diffik`.

- [ ] **Step 3: Commit**

```bash
git add cmd/cli/run.go loop.go
git commit -m "Add Phase 1 run scenarios (static, singular, circle) with CSV recording

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_012FAJrCqoTNZgDVMfHN6sDY"
```

---

### Task 7: Run against the MuJoCo sim and record results

This task needs the user's laptop with the MuJoCo machine running. **A subagent cannot do it.** Hand it back to the user with the exact commands, then fill in the results they report.

**Files:**
- Modify: `docs/superpowers/specs/2026-11-15-diff-ik-phase0-1-design.md`
- Create: `docs/results/2026-11-phase0-1.md` (results table filled from the user's runs)

- [ ] **Step 1: Bring the spec in line with what was built**
  - In the "Kinematics conventions" Jacobian metric, change "with `|Δq| = 1°` per joint" to "perturbing one joint at a time by 1° (perturbing all joints at once lets parallel axes cancel)".
  - In the error-handling table, change the "`Kinematics()` unparseable" row to "`Kinematics()` returns an error: construction fails with that error".
  - In the Components table, add `armio.go` (holds `armIO`).

- [ ] **Step 2: The user runs** (with `--arm` replaced by the sim arm's name):

```bash
go run ./cmd/cli --arm <arm> validate
go run ./cmd/cli --arm <arm> --csv static.csv run static
go run ./cmd/cli --arm <arm> --csv singular.csv run singular
go run ./cmd/cli --arm <arm> --csv circle.csv run circle
```

What to look for:
- **`validate`:** if the FK error comes out exactly 0.000, the sim's `EndPosition` probably uses the same model, so the check is circular (an open item in the spec). Record that.
- **`run`, first:** if the first `Send` faults with an unimplemented or stream error, the sim arm doesn't support `MoveThroughJointPositionsStreamed`. Stop and report that, because it blocks Phase 1.
- **`run`:** a rate under 50 Hz, or a high p99, points at the streamed move or the gRPC round trip. A high resync count means `ResyncDeg` needs tuning.
- **Circular `validate`:** if the FK check is circular, the Jacobian check is circular too, because it also uses `EndPosition` deltas. The results file must then mark **both** Phase 0 checks as unproven on the sim.

- [ ] **Step 3: Write `docs/results/2026-11-phase0-1.md`** with one table per command: each metric, its value, its pass mark and PASS/FAIL. Then add a short list of tuning changes made (for example, `ResyncDeg`) and any failures still open.

- [ ] **Step 4: Commit**

```bash
git add docs/superpowers/specs docs/results
git commit -m "Record Phase 0/1 sim results and align spec with implementation

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_012FAJrCqoTNZgDVMfHN6sDY"
```
