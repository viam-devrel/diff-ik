# Diff-IK Phase 3a: 5-DOF Task Formulation on the Simulated SO-101 — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the servo loop behave well on arms that can't reach an arbitrary 6-DOF pose: a position-priority pointing task, a rank-tolerant damped pseudo-inverse, and weighted pose tasks with dropped rows. Validate all of it on the so-101 module's simulated arm.

**Architecture:**
- **Pose mode with m ≥ n** keeps today's `dlsStep` untouched. A golden test guarantees UR5e commands stay bit-identical.
- **New paths:** pose with m < n uses `dpinv`; pointing mode uses a two-level priority solve, with position primary and pointing secondary in position's null space. The achievable part of the pointing error (`e2_reach`) decides convergence, and the unreachable part is reported as a residual.
- **Where the code lives:** new math in `task.go` and `kin.go`; `loop.go` dispatches per task; the session, module and CLI add surface fields.

**Tech Stack:** Go 1.26, `go.viam.com/rdk v1.16.0`, gonum `mat`. Python and TypeScript clients as in Phase 2.

**Spec:** `docs/superpowers/specs/2026-11-15-diff-ik-phase3a-design.md`. Read it first. The spec wins over this plan when they disagree; say so in your report if they do.

---

## Conventions for every task

- Branch `diff-ik-phase3a` in `/Users/nick.hehr/src/diff-ik`. Don't push.
- **Units:** poses are `spatialmath.Pose` in mm. Inside the loop, errors and velocities are SI (m, rad, m/s, rad/s). `mmPerM` = 1000 lives in `kin.go`.
- **Before every commit:** run `gofmt -w .`, then `gofmt -l .` (must print nothing), `go vet ./...` and `go test -race ./...`.
- **Commit trailers:**
  ```
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_012FAJrCqoTNZgDVMfHN6sDY
  ```
- **Existing helpers:**
  - `kin.go`: `FK` (returns m and a quaternion), `RotVec`, `PoseError` (target − current), `Jacobian` (6×n), `dlsStep`, `ExpRot`.
  - `loop_test.go`: `simIO`, `home`, `newTestLoop`, `newClockLoop`, `clockTicks`, `fakeClock`, `offsetTarget`, `runTicks`.
  - `kin_test.go`: `ur5e(t)`, `rotZ`, `rotX`.
  - `session_test.go`: `newTestManager`, `fakeUR5e`, `step`, `mustStart`, `wantErr`, `testSessionCfg`.
- **SO-101 mid-workspace joint configurations,** verified in the spec review: `q0a = {0, 0.3, 0.3, 0.5, 0}`, `q0b = {0.3, 0.5, -0.8, 0.4, 0.7}`, `q0c = {0, 0.6, 0.6, 1.0, 0}`.

## File map

| File | Change |
|---|---|
| `testdata/golden_ur5e.json` | New. UR5e pose-mode joint commands recorded at the start of Task 1, before any change. |
| `golden_test.go` | New. Records (`-update-golden`) and checks bit-identity. |
| `testdata/so101.urdf`, `testdata/SO-ARM100-LICENSE` | New. The SO-101 URDF with its collision and visual mesh elements stripped; kinematics identical. |
| `kin.go`, `kin_test.go` | `dpinv`, plus tests and an `so101(t)` helper. |
| `task.go`, `task_test.go` | New: `toolAxis`, `pointingError`, `nullProjector`, `prioritySolve`, `pointingReach`. |
| `loop.go`, `loop_test.go` | `LoopConfig.Task`; `solve` dispatch; per-task measured errors, convergence, stall and leash; new `Status` fields. |
| `session.go`, `session_test.go` | `SessionStatus.RollIgnored` from `servo_twist` in pointing mode. |
| `module.go`, `module_test.go` | `task`, zero-capable `task_weights`, `limits.singular_eps` and `limits.max_damping`, and the new status fields. |
| `cmd/cli/main.go`, `cmd/cli/run.go` | `--task`, `--singular-eps`, `--max-damping`, `--singular-deg`; a reachable static generator for pointing mode. |
| `dev/so101-sim.json`, `README.md` | Local SO-101 sim config and how to run it. |
| `clients/python/servo_bench.py` | `--task` pass-through, and the pointing residual in the circle report. |
| `docs/results/2026-11-phase3a.md` | Task 8, done by the controller. |

---

### Task 1: Golden UR5e commands (record BEFORE any other change)

**Files:**
- Create: `golden_test.go`
- Create: `testdata/golden_ur5e.json`, generated

- [ ] **Step 1: Write the golden test**

`golden_test.go`:

```go
package diffik

import (
	"encoding/json"
	"flag"
	"math"
	"os"
	"slices"
	"testing"

	"github.com/golang/geo/r3"
)

var updateGolden = flag.Bool("update-golden", false, "rewrite testdata/golden_ur5e.json from the current code")

// goldenScenarios runs the Phase 1/2 UR5e pose-mode scenarios and returns every joint command sent,
// as raw float64 bits so the comparison is exact.
func goldenScenarios(t *testing.T) map[string][][]uint64 {
	t.Helper()
	bits := func(sends [][]float64) [][]uint64 {
		out := make([][]uint64, len(sends))
		for i, q := range sends {
			for _, v := range q {
				out[i] = append(out[i], math.Float64bits(v))
			}
		}
		return out
	}
	res := map[string][][]uint64{}

	l, sim, m := newTestLoop(t, home, LoopConfig{})
	l.SetTarget(offsetTarget(t, m, home, r3.Vector{X: 30, Y: 30, Z: -25}, deg2rad(10)))
	runTicks(t, l, 150)
	res["converge"] = bits(sim.sends)

	q0 := slices.Clone(home)
	q0[4] = 0
	l, sim, m = newTestLoop(t, q0, LoopConfig{})
	l.SetTarget(offsetTarget(t, m, q0, r3.Vector{X: 30, Y: 30, Z: -25}, deg2rad(10)))
	runTicks(t, l, 150)
	res["singular"] = bits(sim.sends)

	l, sim, m, clk := newClockLoop(t, home, LoopConfig{})
	l.SetTarget(offsetTarget(t, m, home, r3.Vector{}, 0))
	l.SetVelocity([6]float64{0.05, 0, 0, 0, 0, deg2rad(10)})
	clockTicks(t, l, clk, 50)
	res["twist"] = bits(sim.sends)
	return res
}

// UR5e pose mode with all weights 1 must keep today's code path exactly (spec "Compatibility").
func TestGoldenUR5ePoseCommands(t *testing.T) {
	got := goldenScenarios(t)
	const path = "testdata/golden_ur5e.json"
	if *updateGolden {
		b, err := json.Marshal(got)
		if err != nil {
			t.Fatal(err)
		}
		if err := os.WriteFile(path, b, 0o644); err != nil {
			t.Fatal(err)
		}
		return
	}
	b, err := os.ReadFile(path)
	if err != nil {
		t.Fatalf("%v (record it first with -update-golden on the pre-Phase-3a code)", err)
	}
	var want map[string][][]uint64
	if err := json.Unmarshal(b, &want); err != nil {
		t.Fatal(err)
	}
	for name, w := range want {
		g := got[name]
		if len(g) != len(w) {
			t.Fatalf("%s: %d sends, golden has %d", name, len(g), len(w))
		}
		for i := range w {
			if !slices.Equal(g[i], w[i]) {
				t.Fatalf("%s: send %d differs from golden (bit-exact)", name, i)
			}
		}
	}
}
```

- [ ] **Step 2: Record the golden file on the unchanged code**

```
mkdir -p testdata
go test -run TestGoldenUR5ePoseCommands -update-golden .
go test -run TestGoldenUR5ePoseCommands -count=3 .
```

Expected: the first command writes `testdata/golden_ur5e.json`, and the second passes three times, so the scenarios are deterministic.

- [ ] **Step 3: Commit**

```bash
git add golden_test.go testdata/golden_ur5e.json
git commit -m "Record golden UR5e pose-mode joint commands before Phase 3a changes"
```

---

### Task 2: `dpinv` and the SO-101 test model

**Files:**
- Create: `testdata/so101.urdf`, `testdata/SO-ARM100-LICENSE`
- Modify: `kin.go` (append), `kin_test.go` (append)

- [ ] **Step 1: Create the stripped SO-101 URDF**

```bash
cp ~/src/so-101/assets/urdf/SO-ARM100-LICENSE testdata/
python3 - <<'EOF'
import re
s = open('/Users/nick.hehr/src/so-101/assets/urdf/so101.urdf').read()
# drop collision/visual mesh elements: kinematics are unchanged and the 4.2 MB meshes aren't needed
s = re.sub(r'\n[ \t]*<(collision|visual)>.*?\n[ \t]*</\1>', '', s, flags=re.S)
assert '<mesh' not in s
open('testdata/so101.urdf', 'w').write(s)
EOF
```

At the top of `testdata/so101.urdf`, inside the existing header comment, add one line: `Phase 3a test copy: <collision>/<visual> elements removed (kinematics only).`

- [ ] **Step 2: Write the failing tests** (append to `kin_test.go`, adding `"gonum.org/v1/gonum/mat"` and `"go.viam.com/rdk/referenceframe"` to its imports if missing)

```go
// so101 loads the 5-DOF SO-101 test model (testdata/so101.urdf, meshes stripped).
func so101(t *testing.T) referenceframe.Model {
	t.Helper()
	m, err := referenceframe.ParseModelXMLFile("testdata/so101.urdf", "arm", nil)
	if err != nil {
		t.Fatal(err)
	}
	if n := len(m.DoF()); n != 5 {
		t.Fatalf("so101 DOF = %d, want 5", n)
	}
	return m
}

var (
	q0a = []float64{0, 0.3, 0.3, 0.5, 0}
	q0b = []float64{0.3, 0.5, -0.8, 0.4, 0.7}
	q0c = []float64{0, 0.6, 0.6, 1.0, 0}
)

func mulVec(A mat.Matrix, x []float64) []float64 {
	var y mat.VecDense
	y.MulVec(A, mat.NewVecDense(len(x), x))
	return y.RawVector().Data
}

func nearVec(t *testing.T, got, want []float64, tol float64) {
	t.Helper()
	for i := range want {
		if math.Abs(got[i]-want[i]) > tol {
			t.Fatalf("got %v, want %v (tol %g)", got, want, tol)
		}
	}
}

func TestDpinvMatchesClosedForms(t *testing.T) {
	sq := mat.NewDense(2, 2, []float64{2, 1, 1, 3})
	P, lam, _, r, err := dpinv(sq, 0.04, 0.1, true)
	if err != nil || lam != 0 || r != 2 {
		t.Fatalf("square: λ=%v r=%d err=%v", lam, r, err)
	}
	nearVec(t, mulVec(P, []float64{1, 2}), []float64{0.2, 0.6}, 1e-12) // inverse · b

	tall := mat.NewDense(3, 2, []float64{1, 0, 0, 1, 1, 1})
	P, _, _, _, _ = dpinv(tall, 0.04, 0.1, false)
	var ata, inv, want mat.Dense
	ata.Mul(tall.T(), tall)
	_ = inv.Inverse(&ata)
	want.Mul(&inv, tall.T())
	if !mat.EqualApprox(P, &want, 1e-12) {
		t.Fatalf("tall: dpinv ≠ (AᵀA)⁻¹Aᵀ")
	}

	wide := mat.NewDense(2, 3, []float64{1, 0, 1, 0, 1, 1})
	P, _, _, _, _ = dpinv(wide, 0.04, 0.1, false)
	var aat, inv2, want2 mat.Dense
	aat.Mul(wide, wide.T())
	_ = inv2.Inverse(&aat)
	want2.Mul(wide.T(), &inv2)
	if !mat.EqualApprox(P, &want2, 1e-12) {
		t.Fatalf("wide: dpinv ≠ Aᵀ(AAᵀ)⁻¹")
	}

	zero := mat.NewDense(2, 3, nil)
	P, lam, _, r, err = dpinv(zero, 0.04, 0.1, true)
	if err != nil || r != 0 || lam != 0 || mat.Norm(P, 2) != 0 {
		t.Fatalf("zero: r=%d λ=%v err=%v", r, lam, err)
	}
}

// A rank-deficient system must not blow up, and must give the minimum-norm solution
// (the SO-101 pointing stack has a structural null direction, spec "SO-101 kinematics").
func TestDpinvRankDeficientIsMinimumNorm(t *testing.T) {
	A := mat.NewDense(2, 3, []float64{1, 1, 0, 2, 2, 0}) // rank 1
	P, lam, _, r, err := dpinv(A, 0.04, 0.1, true)
	if err != nil || r != 1 || lam != 0 {
		t.Fatalf("r=%d λ=%v err=%v", r, lam, err)
	}
	x := mulVec(P, []float64{1, 2})
	nearVec(t, x, []float64{0.5, 0.5, 0}, 1e-12) // minimum-norm solution of x+y=1
}

// Chiaverini damping keys on the smallest *retained* singular value.
func TestDpinvDampsOnlyWeakRetainedDirections(t *testing.T) {
	A := mat.NewDense(2, 2, []float64{1, 0, 0, 0.02}) // σ = 1, 0.02 < ε
	_, lam, sr, r, _ := dpinv(A, 0.04, 0.1, true)
	want := 0.1 * math.Sqrt(1-(0.02/0.04)*(0.02/0.04))
	if r != 2 || math.Abs(sr-0.02) > 1e-15 || math.Abs(lam-want) > 1e-12 {
		t.Fatalf("r=%d σr=%v λ=%v, want λ=%v", r, sr, lam, want)
	}
	B := mat.NewDense(2, 2, []float64{1, 0, 0, 1e-9}) // below rank tolerance: dropped, not damped
	_, lam, _, r, _ = dpinv(B, 0.04, 0.1, true)
	if r != 1 || lam != 0 {
		t.Fatalf("near-null direction: r=%d λ=%v, want r=1 λ=0", r, lam)
	}
}

// m < n pose form vs the n×n form at λ > 0 (push-through identity).
func TestDpinvMatchesNormalEquationsWhenDamped(t *testing.T) {
	A := mat.NewDense(2, 3, []float64{1, 0.2, 0, 0, 0.03, 1})
	P, lam, _, _, _ := dpinv(A, 2, 0.1, true) // ε above σmin = 1.0 so λ > 0
	if lam == 0 {
		t.Fatal("want λ > 0")
	}
	var ata mat.Dense
	ata.Mul(A.T(), A)
	for i := 0; i < 3; i++ {
		ata.Set(i, i, ata.At(i, i)+lam*lam)
	}
	var rhs, x mat.VecDense
	rhs.MulVec(A.T(), mat.NewVecDense(2, []float64{0.3, -0.1}))
	if err := x.SolveVec(&ata, &rhs); err != nil {
		t.Fatal(err)
	}
	nearVec(t, mulVec(P, []float64{0.3, -0.1}), x.RawVector().Data, 1e-12)
}
```

- [ ] **Step 3: Run them to verify they fail**

Run: `go test -run 'Dpinv' .`
Expected: a build failure, because `dpinv` is undefined.

- [ ] **Step 4: Implement** (append to `kin.go`)

```go
// rankTol is the relative singular-value cutoff for dpinv. It is 1e-4, not tighter, because
// so101.urdf/so101.json write angles as rounded constants (3.14159, 1.5708): structurally zero
// singular values come out near 4e-6·σmax (spec "Damped pseudo-inverse with rank tolerance").
const rankTol = 1e-4

// rankAbs is an absolute floor: near the SO-101 elbow singularity σmax itself collapses, and a purely
// relative cutoff would then keep the ≈2e-6 structural zero (flipping the reachable/residual split).
const rankAbs = 1e-5

// dpinv returns the damped pseudo-inverse Σ σᵢ/(σᵢ²+λ²)·vᵢuᵢᵀ of A (m×n → n×m), keeping only
// σᵢ > rankTol·σmax. With damp, λ is Chiaverini damping on σr, the smallest retained singular value.
func dpinv(A mat.Matrix, eps, maxDamping float64, damp bool) (P *mat.Dense, lambda, sigmaR float64, rank int, err error) {
	m, n := A.Dims()
	P = mat.NewDense(n, m, nil)
	var svd mat.SVD
	if !svd.Factorize(A, mat.SVDThin) {
		return nil, 0, 0, 0, errSVD
	}
	vals := svd.Values(nil)
	if len(vals) == 0 || vals[0] <= rankAbs {
		return P, 0, 0, 0, nil
	}
	for _, s := range vals {
		if s > max(rankTol*vals[0], rankAbs) {
			rank++
		}
	}
	sigmaR = vals[rank-1]
	l2 := 0.0
	if damp && sigmaR < eps {
		r := sigmaR / eps
		l2 = maxDamping * maxDamping * (1 - r*r)
	}
	var U, V mat.Dense
	svd.UTo(&U)
	svd.VTo(&V)
	for i := 0; i < rank; i++ {
		var o mat.Dense
		o.Outer(vals[i]/(vals[i]*vals[i]+l2), V.ColView(i), U.ColView(i))
		P.Add(P, &o)
	}
	return P, math.Sqrt(l2), sigmaR, rank, nil
}
```

- [ ] **Step 5: Run all tests**

Run: `go test -race ./...`
Expected: PASS, including `TestGoldenUR5ePoseCommands`.

- [ ] **Step 6: Commit**

```bash
gofmt -w . && gofmt -l . && go vet ./...
git add kin.go kin_test.go testdata/so101.urdf testdata/SO-ARM100-LICENSE
git commit -m "Add rank-tolerant damped pseudo-inverse and SO-101 test model"
```

---

### Task 3: Pointing-task math (`task.go`)

**Files:**
- Create: `task.go`, `task_test.go`

- [ ] **Step 1: Write the failing tests** (`task_test.go`)

```go
package diffik

import (
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"gonum.org/v1/gonum/mat"
)

func vnear(t *testing.T, got, want r3.Vector, tol float64) {
	t.Helper()
	if got.Sub(want).Norm() > tol {
		t.Fatalf("got %v, want %v", got, want)
	}
}

func TestToolAxisIsRotatedZ(t *testing.T) {
	vnear(t, toolAxis(rotX(math.Pi/2)), r3.Vector{Y: -1}, 1e-12) // z rotated +90° about x → −y
	vnear(t, toolAxis(rotZ(1.0)), r3.Vector{Z: 1}, 1e-12)
}

func TestPointingErrorExactAndBasis(t *testing.T) {
	ac := r3.Vector{X: 1}
	r, u, v := pointingError(ac, ac)
	if r.Norm() != 0 {
		t.Fatalf("matching axes: r = %v", r)
	}
	if math.Abs(u.Dot(ac)) > 1e-12 || math.Abs(v.Dot(ac)) > 1e-12 || math.Abs(u.Dot(v)) > 1e-12 {
		t.Fatal("u, v not orthonormal to a_c")
	}
	at := r3.Vector{X: math.Cos(0.2), Y: math.Sin(0.2)} // 0.2 rad about +z from a_c
	r, _, _ = pointingError(ac, at)
	vnear(t, r, r3.Vector{Z: 0.2}, 1e-12)

	r, u, _ = pointingError(ac, ac.Mul(-1)) // antiparallel
	if math.Abs(r.Norm()-math.Pi) > 1e-12 || math.Abs(r.Normalize().Dot(u)) < 1-1e-12 {
		t.Fatalf("antiparallel: r = %v", r)
	}
	_, u, v = pointingError(r3.Vector{Z: 1}, r3.Vector{Z: 1}) // vertical tool: x̂ fallback
	if math.Abs(u.Norm()-1) > 1e-12 || math.Abs(v.Norm()-1) > 1e-12 {
		t.Fatalf("vertical fallback: u=%v v=%v", u, v)
	}
}

// pointingRows builds J2 = [uᵀJ_ang; vᵀJ_ang] — shared by tests and the loop.
func TestPointingJacobianMatchesFiniteDifference(t *testing.T) {
	m := so101(t)
	J, err := Jacobian(m, q0b)
	if err != nil {
		t.Fatal(err)
	}
	_, o0, _ := FK(m, q0b)
	a0 := toolAxis(o0)
	_, u, v := pointingError(a0, a0) // freeze u, v at q0; target axis = a_c(q0)
	J2 := pointingRows(J, u, v)
	const h = math.Pi / 180
	for j := 0; j < 5; j++ {
		qp, qm := append([]float64(nil), q0b...), append([]float64(nil), q0b...)
		qp[j] += h
		qm[j] -= h
		_, op, _ := FK(m, qp)
		_, om, _ := FK(m, qm)
		rp, _, _ := pointingError(toolAxis(op), a0)
		rm, _, _ := pointingError(toolAxis(om), a0)
		// error is target − current, so d(e2)/dq = −J2
		fd := [2]float64{-(u.Dot(rp) - u.Dot(rm)) / (2 * h), -(v.Dot(rp) - v.Dot(rm)) / (2 * h)}
		for i := 0; i < 2; i++ {
			if a, b := J2.At(i, j), fd[i]; math.Abs(a-b) > 0.05*math.Max(math.Abs(b), 0.01) {
				t.Fatalf("joint %d row %d: J2 = %.4f, finite difference = %.4f", j, i, a, b)
			}
		}
	}
}

// Roll (joint 5 on the SO-101) moves neither position nor pointing: its J2·N1 column is ~0.
func TestSO101RollIsNullForPointing(t *testing.T) {
	m := so101(t)
	J, _ := Jacobian(m, q0a)
	_, o, _ := FK(m, q0a)
	_, u, v := pointingError(toolAxis(o), toolAxis(o))
	J2 := pointingRows(J, u, v)
	for i := 0; i < 2; i++ {
		if math.Abs(J2.At(i, 4)) > 1e-5 {
			t.Fatalf("roll column of J2 = %.2e, want ~0", J2.At(i, 4))
		}
	}
}

// Rotating u, v within the plane must not change the priority solve's Δq.
func TestPrioritySolveIsBasisInvariant(t *testing.T) {
	m := so101(t)
	J, _ := Jacobian(m, q0b)
	_, o, _ := FK(m, q0b)
	a := toolAxis(o)
	_, u, v := pointingError(a, a)
	c, s := math.Cos(0.7), math.Sin(0.7)
	u2, v2 := u.Mul(c).Add(v.Mul(s)), v.Mul(c).Sub(u.Mul(s))
	dx1 := []float64{0.001, -0.0005, 0.0008}
	e := r3.Vector{}.Add(u.Mul(0.003)).Add(v.Mul(-0.002)) // a pointing step as a vector ⟂ a
	dqA, _, _, err := prioritySolve(J.Slice(0, 3, 0, 5).(*mat.Dense), pointingRows(J, u, v), dx1,
		[]float64{u.Dot(e), v.Dot(e)}, 0.04, 0.1)
	if err != nil {
		t.Fatal(err)
	}
	dqB, _, _, _ := prioritySolve(J.Slice(0, 3, 0, 5).(*mat.Dense), pointingRows(J, u2, v2), dx1,
		[]float64{u2.Dot(e), v2.Dot(e)}, 0.04, 0.1)
	nearVec(t, dqA, dqB, 1e-10)
}

// Mid-workspace SO-101: neither level damps (this is what catches a too-tight rank tolerance).
func TestSO101MidWorkspaceIsUndamped(t *testing.T) {
	m := so101(t)
	for _, q := range [][]float64{q0a, q0b, q0c} {
		J, _ := Jacobian(m, q)
		_, o, _ := FK(m, q)
		a := toolAxis(o)
		_, u, v := pointingError(a, a)
		_, l1, l2, err := prioritySolve(J.Slice(0, 3, 0, 5).(*mat.Dense), pointingRows(J, u, v),
			[]float64{0.001, 0, 0}, []float64{0.002, 0.001}, 0.04, 0.1)
		if err != nil || l1 != 0 || l2 != 0 {
			t.Fatalf("q=%v: λ1=%v λ2=%v err=%v, want 0, 0", q, l1, l2, err)
		}
	}
}

// pointingReach: an azimuth-only pointing error is unreachable on the SO-101 (residual), an
// in-plane one is reachable.
func TestPointingReachSplitsSO101Error(t *testing.T) {
	m := so101(t)
	J, _ := Jacobian(m, q0a)
	_, o, _ := FK(m, q0a)
	a := toolAxis(o)
	_, u, v := pointingError(a, a)
	J1, J2 := J.Slice(0, 3, 0, 5).(*mat.Dense), pointingRows(J, u, v)
	// Rotating about base z changes the azimuth; its component ⟂ a is the unreachable direction.
	z := r3.Vector{Z: 1}
	zp := z.Sub(a.Mul(a.Dot(z))).Normalize()
	reach, res, err := pointingReach(J1, J2, [2]float64{u.Dot(zp) * 0.1, v.Dot(zp) * 0.1})
	if err != nil || res < 0.09 || reach > 0.01 {
		t.Fatalf("azimuth error: reach=%.4f res=%.4f err=%v, want mostly residual", reach, res, err)
	}
	pitch := a.Cross(z).Normalize() // rotation about the horizontal axis ⟂ arm plane: in-plane pitch
	reach, res, _ = pointingReach(J1, J2, [2]float64{u.Dot(pitch) * 0.1, v.Dot(pitch) * 0.1})
	if reach < 0.09 || res > 0.01 {
		t.Fatalf("pitch error: reach=%.4f res=%.4f, want mostly reachable", reach, res)
	}
}
```

The pitch-versus-azimuth decomposition in the last test comes from the spec's kinematics section. If the `reach` and `res` thresholds fail by a small margin at `q0a`, print the values and loosen them to 0.085 and 0.015, then report it. Don't change the direction of either assertion.

- [ ] **Step 2: Run them to verify they fail**

Run: `go test -run 'ToolAxis|PointingError|PointingJacobian|SO101|PrioritySolve|PointingReach' .`
Expected: a build failure (undefined `toolAxis`, `pointingError`, `pointingRows`, `prioritySolve`, `pointingReach`).

- [ ] **Step 3: Implement** (`task.go`)

```go
package diffik

import (
	"math"

	"github.com/golang/geo/r3"
	"gonum.org/v1/gonum/mat"
	"gonum.org/v1/gonum/num/quat"
)

// Task modes (spec "Task modes and the per-tick solve").
const (
	TaskPose     = "pose"
	TaskPointing = "pointing"
)

// toolAxis returns the tool (flange z) axis of orientation q in the base frame — Viam's
// orientation-vector direction.
func toolAxis(q quat.Number) r3.Vector {
	x, y, z, w := q.Imag, q.Jmag, q.Kmag, q.Real
	return r3.Vector{X: 2 * (x*z + w*y), Y: 2 * (y*z - w*x), Z: 1 - 2*(x*x+y*y)}
}

// pointingError returns the rotation vector r (base frame, ⟂ ac) that turns the current tool axis
// ac onto the target axis at, and an orthonormal pair u, v ⟂ ac.
func pointingError(ac, at r3.Vector) (r, u, v r3.Vector) {
	u = ac.Cross(r3.Vector{Z: 1})
	if u.Norm() < 1e-6 {
		u = ac.Cross(r3.Vector{X: 1})
	}
	u = u.Normalize()
	v = ac.Cross(u)
	c := ac.Cross(at)
	switch {
	case c.Norm() >= 1e-9:
		r = c.Normalize().Mul(math.Atan2(c.Norm(), ac.Dot(at)))
	case ac.Dot(at) < 0:
		r = u.Mul(math.Pi)
	}
	return r, u, v
}

// pointingRows returns J2 = [uᵀJ_ang; vᵀJ_ang] (2×n) from the 6×n Jacobian.
func pointingRows(J *mat.Dense, u, v r3.Vector) *mat.Dense {
	_, n := J.Dims()
	J2 := mat.NewDense(2, n, nil)
	for j := 0; j < n; j++ {
		w := r3.Vector{X: J.At(3, j), Y: J.At(4, j), Z: J.At(5, j)}
		J2.Set(0, j, u.Dot(w))
		J2.Set(1, j, v.Dot(w))
	}
	return J2
}

// nullProjector returns N1 = I − pinv(J1)·J1 with the dpinv rank tolerance.
func nullProjector(J1 mat.Matrix) (*mat.Dense, error) {
	_, n := J1.Dims()
	P, _, _, _, err := dpinv(J1, 1, 0, false)
	if err != nil {
		return nil, err
	}
	N := mat.NewDense(n, n, nil)
	N.Mul(P, J1)
	for i := 0; i < n; i++ {
		for j := 0; j < n; j++ {
			d := -N.At(i, j)
			if i == j {
				d++
			}
			N.Set(i, j, d)
		}
	}
	return N, nil
}

// prioritySolve returns Δq = dpinv(J1)Δx1 + dpinv(J2·N1)(Δx2 − J2·Δq1), with independent
// Chiaverini damping per level (spec "Pointing mode → Priority solve").
func prioritySolve(J1, J2 *mat.Dense, dx1, dx2 []float64, eps, maxDamping float64) (dq []float64, l1, l2 float64, err error) {
	P1, l1, _, _, err := dpinv(J1, eps, maxDamping, true)
	if err != nil {
		return nil, 0, 0, err
	}
	dq1 := mat.NewVecDense(len(dx1), dx1)
	var q1 mat.VecDense
	q1.MulVec(P1, dq1)
	N1, err := nullProjector(J1)
	if err != nil {
		return nil, 0, 0, err
	}
	var J2N mat.Dense
	J2N.Mul(J2, N1)
	P2, l2, _, _, err := dpinv(&J2N, eps, maxDamping, true)
	if err != nil {
		return nil, 0, 0, err
	}
	var j2q1, rhs, q2 mat.VecDense
	j2q1.MulVec(J2, &q1)
	rhs.SubVec(mat.NewVecDense(len(dx2), dx2), &j2q1)
	q2.MulVec(P2, &rhs)
	q1.AddVec(&q1, &q2)
	return append([]float64(nil), q1.RawVector().Data...), l1, l2, nil
}

// pointingReach splits pointing error e2 (rad) into the part J2·N1 can still reduce and the
// residual it can't: |e2_reach| and |e2 − e2_reach| (spec "Achievable pointing").
func pointingReach(J1, J2 *mat.Dense, e2 [2]float64) (reach, residual float64, err error) {
	N1, err := nullProjector(J1)
	if err != nil {
		return 0, 0, err
	}
	var J2N, proj mat.Dense
	J2N.Mul(J2, N1)
	P, _, _, _, err := dpinv(&J2N, 1, 0, false)
	if err != nil {
		return 0, 0, err
	}
	proj.Mul(&J2N, P)
	var er mat.VecDense
	er.MulVec(&proj, mat.NewVecDense(2, e2[:]))
	return er.Norm(2), math.Hypot(e2[0]-er.AtVec(0), e2[1]-er.AtVec(1)), nil
}
```

- [ ] **Step 4: Run all tests**

Run: `go test -race ./...`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
gofmt -w . && gofmt -l . && go vet ./...
git add task.go task_test.go
git commit -m "Add pointing-task math: tool axis, pointing error, priority solve, reachable split"
```

---

### Task 4: Loop — task dispatch, errors, convergence, leash, status

**Files:**
- Modify: `loop.go`, `loop_test.go`

- [ ] **Step 1: Write the failing tests** (append to `loop_test.go`)

```go
func newSO101Loop(t *testing.T, q0 []float64, cfg LoopConfig) (*Loop, *simIO, referenceframe.Model, *fakeClock) {
	t.Helper()
	m := so101(t)
	sim := &simIO{q: slices.Clone(q0)}
	l, err := NewLoop(m, sim, cfg)
	if err != nil {
		t.Fatal(err)
	}
	clk := newFakeClock()
	l.Now = clk.Now
	return l, sim, m, clk
}

func fkPose(t *testing.T, m referenceframe.Model, q []float64) spatialmath.Pose {
	t.Helper()
	p, err := m.Transform(q)
	if err != nil {
		t.Fatal(err)
	}
	return p
}

func TestDefaultTaskByDOF(t *testing.T) {
	l, _, _, _ := newSO101Loop(t, q0a, LoopConfig{})
	if l.Config().Task != TaskPointing {
		t.Fatalf("SO-101 default task = %q, want pointing", l.Config().Task)
	}
	u, _, _ := newTestLoop(t, home, LoopConfig{})
	if u.Config().Task != TaskPose {
		t.Fatalf("UR5e default task = %q, want pose", u.Config().Task)
	}
	if _, err := NewLoop(ur5e(t), &simIO{}, LoopConfig{Task: "fly"}); err == nil {
		t.Fatal("unknown task: want error")
	}
}

func TestSO101ReachablePointingConverges(t *testing.T) {
	for _, q0 := range [][]float64{q0a, q0b, q0c} {
		l, _, m, clk := newSO101Loop(t, q0, LoopConfig{})
		qt := []float64{q0[0] + 0.12, q0[1] - 0.1, q0[2] + 0.15, q0[3] - 0.15, q0[4] + 0.1}
		l.SetTarget(fkPose(t, m, qt))
		clockTicks(t, l, clk, 300)
		s := l.Status()
		if s.State != StateConverged || s.ErrMM > 2 || s.ErrDeg > 2 || s.PointingResidualDeg > 1 {
			t.Fatalf("q0=%v: state=%s pos=%.2fmm pointing=%.2f° residual=%.2f°", q0, s.State, s.ErrMM, s.ErrDeg, s.PointingResidualDeg)
		}
	}
}

func TestSO101UnreachablePointingConvergesWithResidual(t *testing.T) {
	for _, q0 := range [][]float64{q0a, q0b, q0c} {
		l, _, m, clk := newSO101Loop(t, q0, LoopConfig{})
		start := fkPose(t, m, q0)
		a := toolAxis(start.Orientation().Quaternion())
		const az = 20 * math.Pi / 180
		rq := spatialmath.Quaternion(quat.Mul(rotZ(az), start.Orientation().Quaternion()))
		l.SetTarget(spatialmath.NewPose(start.Point(), &rq))
		clockTicks(t, l, clk, 300)
		s := l.Status()
		want := math.Asin(math.Sin(az)*math.Hypot(a.X, a.Y)) * 180 / math.Pi // angle between a_t and the arm plane
		if s.State != StateConverged || s.ErrMM > 2 || math.Abs(s.PointingResidualDeg-want) > 1 {
			t.Fatalf("q0=%v: state=%s pos=%.2fmm residual=%.2f° want %.2f°", q0, s.State, s.ErrMM, s.PointingResidualDeg, want)
		}
	}
}

// Priority: an unreachable pointing request must not cost position accuracy (compare with the
// same goal position targeted alone; q0a keeps the elbow away from its singularity).
func TestSO101PointingNeverSacrificesPosition(t *testing.T) {
	m := so101(t)
	start := fkPose(t, m, q0a)
	goal := start.Point().Add(r3.Vector{X: 20, Z: -15})
	rq := spatialmath.Quaternion(quat.Mul(rotZ(25*math.Pi/180), start.Orientation().Quaternion()))
	l, _, _, clk := newSO101Loop(t, q0a, LoopConfig{})
	l.SetTarget(spatialmath.NewPose(goal, &rq))
	clockTicks(t, l, clk, 300)
	alone, _, _, clk2 := newSO101Loop(t, q0a, LoopConfig{})
	alone.SetTarget(spatialmath.NewPose(goal, start.Orientation()))
	clockTicks(t, alone, clk2, 300)
	s, a := l.Status(), alone.Status()
	if s.State != StateConverged || a.State != StateConverged || s.ErrMM > a.ErrMM+0.1 {
		t.Fatalf("unreachable pointing: %s %.3f mm; position alone: %s %.3f mm", s.State, s.ErrMM, a.State, a.ErrMM)
	}
}

func TestSO101PoseModeUnreachableRollDoesNotConverge(t *testing.T) {
	m := so101(t)
	start := fkPose(t, m, q0a)
	rq := spatialmath.Quaternion(quat.Mul(rotZ(25*math.Pi/180), start.Orientation().Quaternion()))
	target := spatialmath.NewPose(start.Point(), &rq)

	l, _, _, clk := newSO101Loop(t, q0a, LoopConfig{Task: TaskPose})
	l.SetTarget(target)
	clockTicks(t, l, clk, 300)
	if s := l.Status(); s.State == StateConverged {
		t.Fatalf("pose mode converged on an unreachable orientation (err %.2f°)", s.ErrDeg)
	}
	p, _, _, clk2 := newSO101Loop(t, q0a, LoopConfig{Task: TaskPointing})
	p.SetTarget(target)
	clockTicks(t, p, clk2, 300)
	if s := p.Status(); s.State != StateConverged {
		t.Fatalf("pointing mode: state %s, want converged", s.State)
	}
}

func TestSO101PointingTwistTracks(t *testing.T) {
	l, _, m, clk := newSO101Loop(t, q0a, LoopConfig{})
	l.SetTarget(fkPose(t, m, q0a))
	l.SetVelocity([6]float64{0, 0.02, 0}) // 20 mm/s along base y
	clockTicks(t, l, clk, 50)
	if s := l.Status(); s.ErrMM > 1 {
		t.Fatalf("twist tracking error %.2f mm, want < 1", s.ErrMM)
	}
}

// A large unreachable residual must not block a reachable angular twist (leash uses e2_reach).
// The twist is about the in-plane pitch axis, signed so the old full-error leash would block it.
func TestSO101LeashIgnoresUnreachableResidual(t *testing.T) {
	l, _, m, clk := newSO101Loop(t, q0a, LoopConfig{})
	start := fkPose(t, m, q0a)
	rq := spatialmath.Quaternion(quat.Mul(rotZ(30*math.Pi/180), start.Orientation().Quaternion()))
	l.SetTarget(spatialmath.NewPose(start.Point(), &rq))
	clockTicks(t, l, clk, 100) // converge with a large residual
	_, cq, _ := FK(m, l.Status().QCmd)
	pitch := toolAxis(cq).Cross(r3.Vector{Z: 1}).Normalize()
	if pitch.Dot(RotVec(rq.Quaternion(), cq)) < 0 { // old full-error leash would block this
		pitch = pitch.Mul(-1)
	}
	w := pitch.Mul(deg2rad(10))
	l.SetVelocity([6]float64{0, 0, 0, w.X, w.Y, w.Z})
	clockTicks(t, l, clk, 25)
	l.mu.Lock()
	got := RotVec(l.target.Orientation().Quaternion(), rq.Quaternion()).Norm() * 180 / math.Pi
	l.mu.Unlock()
	if math.Abs(got-5) > 0.25 {
		t.Fatalf("target rotated %.2f° in 0.5 s at 10°/s, want 5 (leash must ignore the residual)", got)
	}
}

func TestUR5ePointingConvergesFully(t *testing.T) {
	l, _, m, clk := newClockLoop(t, home, LoopConfig{Task: TaskPointing})
	l.SetTarget(offsetTarget(t, m, home, r3.Vector{X: 30, Y: 30, Z: -25}, deg2rad(10)))
	clockTicks(t, l, clk, 300)
	if s := l.Status(); s.State != StateConverged || s.PointingResidualDeg > 0.5 {
		t.Fatalf("state=%s residual=%.2f°", s.State, s.PointingResidualDeg)
	}
}

// Pose mode with a dropped row on a 6-DOF arm (m=5 < n=6) uses dpinv and still converges on the active rows.
func TestUR5ePoseDroppedRowConverges(t *testing.T) {
	l, _, m, clk := newClockLoop(t, home, LoopConfig{TaskWeights: [6]float64{1, 1, 1, 1, 1, 0}})
	l.SetTarget(offsetTarget(t, m, home, r3.Vector{X: 30, Y: 30, Z: -25}, deg2rad(10)))
	clockTicks(t, l, clk, 300)
	if s := l.Status(); s.State != StateConverged {
		t.Fatalf("state=%s err=%.2fmm %.2f°", s.State, s.ErrMM, s.ErrDeg)
	}
}
```

Add `"gonum.org/v1/gonum/num/quat"` to `loop_test.go`'s imports if it isn't there already.

- [ ] **Step 2: Run them to verify they fail**

Run: `go test -run 'DefaultTask|SO101|UR5ePointing|UR5ePoseDropped' .`
Expected: a build failure (no `Task` field, `PointingResidualDeg` or `TaskPointing` in `LoopConfig`/`Status`).

- [ ] **Step 3: Implement in `loop.go`**

1. In `LoopConfig`, add `Task string // "pose" | "pointing"; empty = pointing if DOF < 6 else pose`.

2. At the end of `withDefaults`, before `return c`:
```go
	if c.Task == "" {
		c.Task = TaskPose
		if dof < 6 {
			c.Task = TaskPointing
		}
	}
```

3. In `validate`, replace the task-weight loop with:
```go
	if c.Task != TaskPose && c.Task != TaskPointing {
		return fmt.Errorf("unsupported task %q: use %q or %q", c.Task, TaskPose, TaskPointing)
	}
	for i, w := range c.TaskWeights {
		if w < 0 {
			return fmt.Errorf("task weight %d is negative", i)
		}
	}
```
`withDefaults` already turns all-zero weights into all ones; the module rejects an explicit all-zero (Task 5).

4. In `Status`, add:
```go
	Task                string    // active task mode
	PointingResidualDeg float64   // pointing mode: the unreachable part of the pointing error at q_m
	ToolAxis            r3.Vector // tool axis of FK(q_cmd), base frame (read by servo_twist)
```

5. In `Loop`, add under `mu`:
```go
	lastReach float64 // pointing mode: |e2_reach| (rad) from the previous tick, for the leash
```

6. In `advance`, replace the angular leash block (the `va, ea := …` lines and their `if`) with:
```go
	va := r3.Vector{X: vel[3], Y: vel[4], Z: vel[5]}
	ea := r3.Vector{X: e[3], Y: e[4], Z: e[5]}
	angErr := ea.Norm()
	if l.cfg.Task == TaskPointing {
		// pointing: leash on the reachable part only, and along the pointing rotation (spec "Leash")
		ea, _, _ = pointingError(toolAxis(cq), toolAxis(tq))
		angErr = l.lastReach
	}
	if !(angErr > leashS*deg2rad(l.cfg.MaxAngularDegPerS) && va.Dot(ea) > 0) {
		tq = quat.Mul(ExpRot(va.Mul(dt)), tq)
	}
```

7. In `tick`, delete the `e := PoseError(tp, tq, cp, cq)` line right after `tp, tq := target.Point()…`; `solve` computes its own error. Then replace the old steps 4–6 (from `// 4–5.` through the `dlsStep` call and its error check) with:
```go
	// 4–6. task-space step: pose → weighted DLS (unchanged when m ≥ n) or dpinv; pointing → priority
	J, err := Jacobian(l.model, l.qCmd)
	if err != nil {
		return err
	}
	dq, lambda, limitedTask, cmdPos, cmdAng, err := l.solve(J, tp, tq, cp, cq, vel, scale, dt)
	if err != nil {
		return err
	}
```
Delete the old standalone `J, err := Jacobian(…)` lines lower down. Then replace step 10 and the status block, from `// 10. measured error` to the end of `tick`, with:
```go
	// 10. measured error, per task
	mp, mq, err := FK(l.model, qm)
	if err != nil {
		return err
	}
	ms, err := l.measure(tp, tq, mp, mq, qm)
	if err != nil {
		return err
	}
	limited := s < 1 || clamped || limitedTask

	l.mu.Lock()
	defer l.mu.Unlock()
	st := &l.status
	if resynced {
		st.Resyncs++
	}
	st.TargetMM = target.Point()
	st.ErrMM, st.ErrDeg = ms.posMM, ms.angDeg
	st.CmdErrMM, st.CmdErrDeg = cmdPos, cmdAng
	st.Damping, st.Limited = lambda, limited
	st.Task, st.PointingResidualDeg, st.ToolAxis = l.cfg.Task, ms.residualDeg, toolAxis(cq)
	st.QCmd, st.QMeas = slices.Clone(l.qCmd), slices.Clone(qm)
	l.lastReach = deg2rad(ms.reachDeg)
	st.State = l.nextState(ms, limited, vel != ([6]float64{}))
	return nil
```

8. Add these functions, and replace `nextState`:

```go
// solve computes this tick's joint step for the configured task. cmdPos/cmdAng are the
// commanded-side errors in mm / deg (spec "Status fields").
func (l *Loop) solve(J *mat.Dense, tp r3.Vector, tq quat.Number, cp r3.Vector, cq quat.Number,
	vel [6]float64, scale, dt float64,
) (dq []float64, lambda float64, limited bool, cmdPos, cmdAng float64, err error) {
	c := l.cfg
	_, n := J.Dims()
	if c.Task == TaskPointing {
		ac, at := toolAxis(cq), toolAxis(tq)
		r, u, v := pointingError(ac, at)
		e1 := tp.Sub(cp)
		v1 := capNorm(r3.Vector{X: vel[0], Y: vel[1], Z: vel[2]}.Add(e1.Mul(c.Gain)), c.MaxLinearMMPerS/mmPerM).Mul(scale)
		w := r3.Vector{X: vel[3], Y: vel[4], Z: vel[5]}
		v2 := capNorm(r3.Vector{X: u.Dot(w) + c.Gain*u.Dot(r), Y: v.Dot(w) + c.Gain*v.Dot(r)}, deg2rad(c.MaxAngularDegPerS)).Mul(scale)
		J1 := mat.DenseCopyOf(J.Slice(0, 3, 0, n))
		dq, l1, l2, err := prioritySolve(J1, pointingRows(J, u, v),
			[]float64{v1.X * dt, v1.Y * dt, v1.Z * dt}, []float64{v2.X * dt, v2.Y * dt}, c.SingularEps, c.MaxDamping)
		if err != nil {
			return nil, 0, false, 0, 0, err
		}
		return dq, math.Max(l1, l2), l1 > 0 || l2 > 0, e1.Norm() * mmPerM, r.Norm() * 180 / math.Pi, nil
	}

	e := PoseError(tp, tq, cp, cq)
	lin := capNorm(r3.Vector{X: vel[0] + c.Gain*e[0], Y: vel[1] + c.Gain*e[1], Z: vel[2] + c.Gain*e[2]},
		c.MaxLinearMMPerS/mmPerM).Mul(scale)
	ang := capNorm(r3.Vector{X: vel[3] + c.Gain*e[3], Y: vel[4] + c.Gain*e[4], Z: vel[5] + c.Gain*e[5]},
		deg2rad(c.MaxAngularDegPerS)).Mul(scale)
	dx := [6]float64{lin.X * dt, lin.Y * dt, lin.Z * dt, ang.X * dt, ang.Y * dt, ang.Z * dt}
	cmdPos, cmdAng = activePosMM(e, c.TaskWeights), activeAngDeg(e, c.TaskWeights)

	var rows []int
	for i, w := range c.TaskWeights {
		if w > 0 {
			rows = append(rows, i)
		}
	}
	if len(rows) >= n { // unchanged Phase 1/2 path: bit-identical for the UR5e with all weights
		dq, lambda, sigmaMin, err := dlsStep(J, c.TaskWeights, dx, c.SingularEps, c.MaxDamping)
		return dq, lambda, sigmaMin < c.SingularEps, cmdPos, cmdAng, err
	}
	A := mat.NewDense(len(rows), n, nil)
	b := make([]float64, len(rows))
	for k, i := range rows {
		sw := math.Sqrt(c.TaskWeights[i])
		for j := 0; j < n; j++ {
			A.Set(k, j, sw*J.At(i, j))
		}
		b[k] = sw * dx[i]
	}
	P, lambda, sigmaR, _, err := dpinv(A, c.SingularEps, c.MaxDamping, true)
	if err != nil {
		return nil, 0, false, 0, 0, err
	}
	var x mat.VecDense
	x.MulVec(P, mat.NewVecDense(len(b), b))
	return append([]float64(nil), x.RawVector().Data...), lambda, sigmaR < c.SingularEps, cmdPos, cmdAng, nil
}

// measured holds the per-task measured errors used for status, convergence and stall.
type measured struct {
	posMM, angDeg          float64 // angDeg: pointing angle (pointing) or active orientation norm (pose)
	reachDeg, residualDeg  float64 // pointing mode only
	stallNorm              float64
}

func (l *Loop) measure(tp r3.Vector, tq quat.Number, mp r3.Vector, mq quat.Number, qm []float64) (measured, error) {
	c := l.cfg
	if c.Task != TaskPointing {
		em := PoseError(tp, tq, mp, mq)
		norm := 0.0
		for i, v := range em {
			norm += c.TaskWeights[i] * v * v
		}
		return measured{posMM: activePosMM(em, c.TaskWeights), angDeg: activeAngDeg(em, c.TaskWeights),
			reachDeg: activeAngDeg(em, c.TaskWeights), stallNorm: math.Sqrt(norm)}, nil
	}
	Jm, err := Jacobian(l.model, qm)
	if err != nil {
		return measured{}, err
	}
	r, u, v := pointingError(toolAxis(mq), toolAxis(tq))
	_, n := Jm.Dims()
	reach, res, err := pointingReach(mat.DenseCopyOf(Jm.Slice(0, 3, 0, n)), pointingRows(Jm, u, v), [2]float64{u.Dot(r), v.Dot(r)})
	if err != nil {
		return measured{}, err
	}
	e1 := tp.Sub(mp).Norm()
	return measured{posMM: e1 * mmPerM, angDeg: r.Norm() * 180 / math.Pi, reachDeg: reach * 180 / math.Pi,
		residualDeg: res * 180 / math.Pi, stallNorm: math.Hypot(e1, reach)}, nil
}

// nextState picks converged/stalled/running. moving means a feedforward twist is active. Caller holds l.mu.
// Converged: position within TolMM and the reachable/active angular error within TolDeg.
func (l *Loop) nextState(ms measured, limited, moving bool) State {
	if !moving && ms.posMM <= l.cfg.TolMM && ms.reachDeg <= l.cfg.TolDeg {
		l.errRing = l.errRing[:0]
		return StateConverged
	}
	window := int(l.cfg.RateHz) // 1 s of ticks
	if len(l.errRing) == window {
		l.errRing = l.errRing[1:]
	}
	l.errRing = append(l.errRing, ms.stallNorm)
	if limited && len(l.errRing) == window && ms.stallNorm > 0.99*l.errRing[0] {
		return StateStalled
	}
	return StateRunning
}

// activePosMM / activeAngDeg are the norms of the components whose weight is > 0.
func activePosMM(e [6]float64, w [6]float64) float64 {
	s := 0.0
	for i := 0; i < 3; i++ {
		if w[i] > 0 {
			s += e[i] * e[i]
		}
	}
	return math.Sqrt(s) * mmPerM
}

func activeAngDeg(e [6]float64, w [6]float64) float64 {
	s := 0.0
	for i := 3; i < 6; i++ {
		if w[i] > 0 {
			s += e[i] * e[i]
		}
	}
	return math.Sqrt(s) * 180 / math.Pi
}
```

Add `"gonum.org/v1/gonum/mat"` to `loop.go`'s imports. Delete `posMM` and `angDeg` if they become unused, and keep them if anything else, such as the CLI or tests, still uses them.

The golden check covers bit-identity: with all weights 1, pose mode computes `e`, `lin`, `ang` and `dx` exactly as before and calls the same `dlsStep`. Changes to errors and states don't affect the joint commands.

- [ ] **Step 4: Run all tests, golden included**

Run: `go test -race ./...` and `go test -race -count=3 -run 'Golden|SO101|UR5e' .`
Expected: PASS. If `TestGoldenUR5ePoseCommands` fails, the pose path changed numerically. Fix the code; never re-record the golden file.

- [ ] **Step 5: Commit**

```bash
gofmt -w . && gofmt -l . && go vet ./...
git add loop.go loop_test.go
git commit -m "Loop: pose/pointing task dispatch, position-priority pointing, per-task convergence and leash"
```

---

### Task 5: Session and module surface

**Files:**
- Modify: `session.go`, `session_test.go`, `module.go`, `module_test.go`

- [ ] **Step 1: Write the failing tests**

Append to `session_test.go`:

```go
func TestSessionTwistReportsIgnoredRoll(t *testing.T) {
	m, _, clk := newTestManager(t, spatialmath.NewZeroPose())
	cfg := testSessionCfg(ModeTwist)
	cfg.Loop.Task = TaskPointing
	s, _ := mustStart(t, m, cfg)
	step(t, s, clk, 1) // populate ToolAxis
	a := s.loop.Status().ToolAxis
	st, err := s.twist(r3.Vector{}, a.Mul(deg2rad(10))) // pure roll about the tool axis
	if err != nil || !st.RollIgnored {
		t.Fatalf("roll twist: ignored=%v err=%v", st.RollIgnored, err)
	}
	st, err = s.twist(r3.Vector{X: 0.01}, r3.Vector{})
	if err != nil || st.RollIgnored {
		t.Fatalf("linear twist: ignored=%v err=%v", st.RollIgnored, err)
	}
}
```

Append to `module_test.go`:

```go
func TestServoStartTaskWeightsAndDamping(t *testing.T) {
	m, _, clk := newTestManager(t, spatialmath.NewZeroPose())
	s := testServo(t, m)
	start := func(kv ...interface{}) (map[string]interface{}, error) {
		c := map[string]interface{}{"command": "servo_start", "component_name": "arm", "frame": "arm_origin"}
		for i := 0; i < len(kv); i += 2 {
			c[kv[i].(string)] = kv[i+1]
		}
		return s.DoCommand(context.Background(), c)
	}
	for _, tc := range []struct {
		kv   []interface{}
		want string
	}{
		{[]interface{}{"task", "fly"}, "unsupported task"},
		{[]interface{}{"task_weights", []interface{}{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}, "task_weights"},
		{[]interface{}{"task_weights", []interface{}{1.0, 1.0, 1.0, -1.0, 1.0, 1.0}}, "task_weights"},
		{[]interface{}{"limits", map[string]interface{}{"singular_eps": 0.0}}, "singular_eps"},
		{[]interface{}{"limits", map[string]interface{}{"max_damping": -1.0}}, "max_damping"},
	} {
		if _, err := start(tc.kv...); err == nil || !strings.Contains(err.Error(), tc.want) {
			t.Errorf("%v: err = %v, want containing %q", tc.kv, err, tc.want)
		}
	}
	r, err := start("task", "pointing", "task_weights", []interface{}{1.0, 1.0, 1.0, 0.0, 0.0, 1.0},
		"limits", map[string]interface{}{"singular_eps": 0.02, "max_damping": 0.05})
	if err != nil {
		t.Fatal(err)
	}
	sess, err := m.get(r["session_id"].(string))
	if err != nil {
		t.Fatal(err)
	}
	step(t, sess, clk, 1)
	st := do(t, s, map[string]interface{}{"command": "servo_status", "session_id": r["session_id"]})
	lim := st["limits"].(map[string]interface{})
	if st["task"] != "pointing" || lim["singular_eps"] != 0.02 || lim["max_damping"] != 0.05 {
		t.Fatalf("status task=%v limits=%v", st["task"], lim)
	}
	if _, ok := st["pointing_residual_deg"].(float64); !ok {
		t.Fatalf("pointing_residual_deg missing: %v", st)
	}
}
```

- [ ] **Step 2: Run them to verify they fail**

Run: `go test -run 'IgnoredRoll|TaskWeightsAndDamping' .`
Expected: a build failure (`SessionStatus.RollIgnored` is undefined), then assertion failures.

- [ ] **Step 3: Implement**

`session.go`:
- Add `RollIgnored bool // servo_twist only: an angular component about the tool axis was dropped (pointing mode)` to `SessionStatus`.
- In `twist`, after computing `ab` and before `s.mu.Lock()`:
  ```go
  	rollIgnored := false
  	if s.loop.Config().Task == TaskPointing {
  		if a := s.loop.Status().ToolAxis; a.Norm() > 0 && math.Abs(ab.Dot(a)) > 1e-9 {
  			rollIgnored = true
  		}
  	}
  ```
- Replace the final `return s.status(), nil` with:
  ```go
  	st := s.status()
  	st.RollIgnored = rollIgnored
  	return st, nil
  ```
- Add `"math"` to the imports if needed.

`module.go`:
- In `servoStart`, after `lc := LoopConfig{RateHz: rate}`:
  ```go
  	if v, ok := cmd["task"]; ok {
  		t, ok := v.(string)
  		if !ok || (t != TaskPose && t != TaskPointing) {
  			return nil, fmt.Errorf("unsupported task %v: use %q or %q", v, TaskPose, TaskPointing)
  		}
  		lc.Task = t
  	}
  ```
- In the limits `for _, f := range` table, add `{"singular_eps", &lc.SingularEps}` and `{"max_damping", &lc.MaxDamping}`.
- Replace the `task_weights` block with:
  ```go
  	if v, ok := cmd["task_weights"]; ok {
  		items, ok := v.([]interface{})
  		if !ok || len(items) != 6 {
  			return nil, fmt.Errorf("task_weights must be 6 numbers >= 0, not all zero, got %v", v)
  		}
  		sum := 0.0
  		for i, it := range items {
  			n, err := num(it, "task_weights")
  			if err != nil {
  				return nil, err
  			}
  			if n < 0 {
  				return nil, fmt.Errorf("task_weights must be >= 0, got %v", n)
  			}
  			lc.TaskWeights[i], sum = n, sum+n
  		}
  		if sum == 0 {
  			return nil, errors.New("task_weights must not all be zero")
  		}
  		if lc.Task == TaskPointing {
  			s.logger.Warnw("task_weights are ignored in pointing mode (position has priority)")
  		}
  	}
  ```
  The all-zero check happens here, before `withDefaults` turns all zeros into ones.
- In `statusMap`: add `"task": st.Task` at the top level, and `"singular_eps": st.Config.SingularEps` and `"max_damping": st.Config.MaxDamping` inside `limits`. When `st.Task == TaskPointing`, also set `"pointing_residual_deg": st.PointingResidualDeg`.
- In the `servo_twist` case, return `"roll_ignored": true` in the map when `st.RollIgnored`.
- Check that the module test for `servo_start` with an unknown `task` gets "unsupported task" from the module, not from the loop. Both say it, so either is fine.

In `TestDoCommandErrors` (`module_test.go`), change the row that expects `{1,1,1,0,1,1}` to be rejected. That vector is valid now, and the session it would start makes every later `start()` fail with "busy". Change it to `{start("task_weights", []interface{}{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}), "task_weights"}`.

- [ ] **Step 4: Run all tests**

Run: `go test -race ./...` and `go build ./cmd/module ./cmd/cli`.
Expected: PASS, and the build succeeds.

- [ ] **Step 5: Commit**

```bash
gofmt -w . && gofmt -l . && go vet ./...
git add session.go session_test.go module.go module_test.go
git commit -m "Module/session: task, zero-capable task_weights, singular_eps/max_damping, pointing status, roll_ignored"
```

---

### Task 6: CLI — task flags, reachable static targets, singular centre

**Files:**
- Modify: `cmd/cli/main.go`, `cmd/cli/run.go`

- [ ] **Step 1: Implement**

`main.go`:
- Add the flags:
  ```go
  	task := flag.String("task", "", "run: task mode, pose|pointing (default: pointing if the arm has < 6 joints, else pose)")
  	singEps := flag.Float64("singular-eps", 0, "run: Chiaverini threshold ε (0 = loop default 0.04)")
  	maxDamp := flag.Float64("max-damping", 0, "run: maximum damping λmax (0 = loop default 0.1)")
  	singDeg := flag.Float64("singular-deg", 0, "singular: joint angle (deg) the scenario sweeps through, ±11.5° around it")
  ```
- Change the `--wrist` help text to `"singular: index of the joint whose --singular-deg angle is singular (UR5e wrist: 4; SO-101 elbow: 2)"`.
- Pass the new flags to `run`. Its signature becomes `run(ctx, a, model, scenario, rate, csvPath, wrist, maxJointDegS, resyncDeg, task, singEps, maxDamp, singDeg)`. Reject a negative `--singular-eps` or `--max-damping` with an error.

`run.go`:
- Set `cfg := diffik.LoopConfig{RateHz: rate, ResyncDeg: resyncDeg, Task: task, SingularEps: singEps, MaxDamping: maxDamp}`.
- In the scenario switch:
  - **`static`:** read the arm's current joints with `a.JointPositions(ctx, nil)`. When the loop's effective task (`l.Config().Task` once built, so it is decided after `NewLoop`) is pointing, use `staticReachable(model, q0)`; otherwise use `static`. Restructure so the body is chosen after `NewLoop`. The body is only called after the loop starts, so pick it with a closure that reads `l.Config().Task`.
  - **`singular`:** pass `singDeg` and set `qa[wrist], qb[wrist] = c+0.2, c-0.2`, where `c := singDeg * math.Pi / 180`. Return an error if `c±0.2` falls outside the joint's `model.DoF()` limits. Update the doc comment.
- Add:

```go
// staticReachable is the pointing-mode static scenario: targets are FK of the start joints perturbed
// by a seeded random offset (≤ 10° per joint), keeping those whose position is 20–60 mm away, so
// every target is reachable by construction on any arm (spec "cmd/cli").
func staticReachable(model referenceframe.Model, q0 []float64) func(context.Context, *diffik.Loop, *recorder, spatialmath.Pose) error {
	return func(ctx context.Context, l *diffik.Loop, _ *recorder, start spatialmath.Pose) error {
		rng := rand.New(rand.NewSource(1))
		lim := model.DoF()
		failed := 0
		for i := 0; i < 10; {
			if ctx.Err() != nil {
				return ctx.Err()
			}
			q := slices.Clone(q0)
			for j := range q {
				q[j] = math.Min(math.Max(q[j]+(rng.Float64()*2-1)*10*math.Pi/180, lim[j].Min), lim[j].Max)
			}
			target, err := model.Transform(q)
			if err != nil {
				return err
			}
			if d := target.Point().Sub(start.Point()).Norm(); d < 20 || d > 60 {
				continue
			}
			l.SetTarget(target)
			took, ok := waitConverged(ctx, l, target, 10*time.Second)
			s := l.Status()
			fmt.Printf("target %2d: %-9s in %5.2fs  err %.2f mm %.2f°  residual %.2f°  %s\n",
				i, s.State, took.Seconds(), s.ErrMM, s.ErrDeg, s.PointingResidualDeg, pass(ok))
			if !ok {
				failed++
			}
			l.SetTarget(start)
			waitConverged(ctx, l, start, 10*time.Second)
			i++
		}
		if failed > 0 {
			return fmt.Errorf("%d of 10 targets did not converge", failed)
		}
		return nil
	}
}
```

Check that `waitConverged` decides "reached" from `Status().State == StateConverged`, which is task-aware now. If it compares poses directly, switch it to the state.

- [ ] **Step 2: Build and smoke-test the flag parsing**

```
go build ./cmd/cli && go run ./cmd/cli --arm x --task fly run static 2>&1 | head -3
```
Expected: it builds. The second command fails to connect before reaching `NewLoop`, which is fine; this only checks the flags parse. Also run `go test -race ./...`, which should pass.

- [ ] **Step 3: Commit**

```bash
gofmt -w . && gofmt -l . && go vet ./...
git add cmd/cli/main.go cmd/cli/run.go
git commit -m "CLI: --task/--singular-eps/--max-damping/--singular-deg, reachable static targets in pointing mode"
```

---

### Task 7: Local SO-101 sim config, README and bench pass-through

**Files:**
- Create: `dev/so101-sim.json`
- Modify: `README.md`, `clients/python/servo_bench.py`

- [ ] **Step 1: Create `dev/so101-sim.json`**

```json
{
  "modules": [
    {"type": "local", "name": "so-101", "executable_path": "/Users/nick.hehr/src/so-101/bin/arm"},
    {"type": "local", "name": "diff-ik", "executable_path": "/Users/nick.hehr/src/diff-ik/bin/diff-ik"}
  ],
  "components": [
    {"name": "arm", "api": "rdk:component:arm", "model": "devrel:so101:simulated",
     "attributes": {"speed_degs_per_sec": 90}, "frame": {"parent": "world"}},
    {"name": "gripper", "api": "rdk:component:gripper", "model": "devrel:so101:simulated-gripper",
     "attributes": {}, "frame": {"parent": "arm"}}
  ],
  "services": [
    {"name": "servo", "api": "rdk:service:motion", "model": "devrel:diff-ik:servo",
     "attributes": {"arms": ["arm"]}}
  ]
}
```

The arm needs `frame` so the frame system includes it; without it, `servo_start` with the default `frame: "world"` fails. Check `~/src/so-101/docs/simulated.md` for any attribute the simulated gripper requires (for example a reference to the arm) and add it. If the gripper needs no `frame`, keep the frame anyway so it matches the hardware setup.

- [ ] **Step 2: README section**

Add a "Local SO-101 simulation (Phase 3a)" section to `README.md`:

```markdown
## Local SO-101 simulation (Phase 3a)

No cloud machine needed:

    (cd ~/src/so-101 && make)            # builds bin/arm
    go build -o bin/diff-ik ./cmd/module
    viam-server -config dev/so101-sim.json   # serves plaintext localhost:8080

Then, from another shell:

    go run ./cmd/cli --address localhost:8080 --insecure --arm arm --task pointing run static
    uv run clients/python/servo_bench.py --address localhost:8080 --insecure --arm arm --task pointing

The paths in `dev/so101-sim.json` are absolute for one laptop; edit `executable_path` for yours.
```

- [ ] **Step 3: Bench pass-through** (`clients/python/servo_bench.py`)

- Add `ap.add_argument("--task", default=None, help="pose|pointing (default: the server's default for the arm)")`.
- In `start_kw`, set `base["task"] = args.task` when `args.task` is set.
- In `circle`, after the existing resyncs/overruns line, add:
  ```python
      if "pointing_residual_deg" in st:
          report(f"{label}: pointing residual (geometry, not lag)", f"{st['pointing_residual_deg']:.1f}°")
  ```

Run `uv run clients/python/servo_bench.py --help` and the helper self-check from the Phase 2 plan. Both must still work. Then delete `clients/python/__pycache__`.

- [ ] **Step 4: Commit**

```bash
git add dev/so101-sim.json README.md clients/python/servo_bench.py
git commit -m "Local SO-101 sim config, README run instructions, bench --task"
```

---

### Task 8: Sim validation and results — controller only

The controller does this task itself, because it runs servers.

- [ ] **Step 1:** Build: `(cd ~/src/so-101 && make)`. This is required: the checked-in `bin/arm` predates `MoveThroughJointPositionsStreamed`. and `go build -o bin/diff-ik ./cmd/module`. Start `viam-server -config dev/so101-sim.json` in the background and confirm the `arm` and `servo` resources come up. If something else already holds port 8080 (the mujoco-sim-mac viam-server binds `*:8080`), stop it first, or add `"network": {"bind_address": "localhost:8081"}` to the dev config and use that port.
- [ ] **Step 2:** Pick the SO-101 damping from `testdata/so101.urdf` across the elbow range. Look at **J1's smallest σ** (the position level) and **σmax(J2·N1)**, not only the 6×5 pose σmin. With ε = 0.04, the position level damps whenever the elbow is below about −0.95 rad. Choose `--singular-eps` so damping engages only near the elbow singularity, and `--max-damping` to match. Use `--singular-deg -74` (the pose σmin minimum, about −1.29 rad); ±0.2 rad around −1.5 would exceed the ±1.69 rad limit. Move the arm to `q0a` with `MoveToJointPositions`.
- [ ] **Step 3:** Run:
  - `go run ./cmd/cli --address localhost:8080 --insecure --arm arm validate` (informational);
  - `run static --task pointing`;
  - `run singular --task pointing --wrist 2 --singular-deg <angle>`;
  - `run circle --task pointing`, each with `--csv`, plus the chosen damping flags;
  - the Python bench with `--task pointing` (smoke, dead-man, circle);
  - one unreachable-pointing demo via the bench or a short script (position converges and the residual is reported).
- [ ] **Step 4:** Write `docs/results/2026-11-phase3a.md` in the format of the earlier results. Include the bars from the spec's "Sim validation" table, the chosen damping values and why, and the circular-validate and geometry-residual caveats. Commit it with `git add docs/results`.
- [ ] **Step 5:** Final Opus review of the whole branch, then superpowers:finishing-a-development-branch.
