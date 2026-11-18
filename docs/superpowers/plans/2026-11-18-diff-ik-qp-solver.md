# Diff-IK QP Solver and Posture Task Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an opt-in per-tick QP solver (`solver: "qp"`) that enforces joint position, velocity and acceleration limits as hard bounds, plus a null-space posture task, beside the unchanged DLS path.

**Architecture:** A pure-Go Goldfarb–Idnani dual active-set solver (`qp.go`) solves `min ½xᵀHx + gᵀx` subject to bounds and general rows. The loop builds H and g from the same weighted task rows `solve` already computes, with per-joint bounds from `qpBounds` (`qpbounds.go`), and falls back to the existing DLS path tick by tick when the solver fails. The DLS path keeps its exact floating-point behaviour, so the goldens don't change.

**Tech Stack:** Go 1.x with gonum (`mat`), the Viam RDK `referenceframe`, Python clients run with `uv`, and the viam-mujoco-sim module for the A/B runs.

**Spec:** `docs/superpowers/specs/2026-11-18-diff-ik-qp-solver-design.md`. Read it before starting any task.

**Conventions for every task:**
- Work on branch `diff-ik-qp` in `/Users/nick.hehr/src/diff-ik`.
- Run Go tests with `go test ./...`. The whole suite currently passes, with 167 tests.
- Commit messages end with these two lines:
  ```
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_012FAJr9jcJyz2f1AEiCkX3y
  ```
- Never push.
- Units: inside the loop, joint steps are radians per tick. Positions in `solve` are metres. Config and status values are in mm and degrees.

---

## File structure

| File | Responsibility |
|---|---|
| `qp.go` (new) | `qpProblem`, `qpCore` (Goldfarb–Idnani), `solveQP`, `qpRows` |
| `qp_test.go` (new) | solver unit, property and timing tests |
| `qpbounds.go` (new) | `brakeCap`, `(*Loop).qpBounds`, `(*Loop).qpNext` (objective plus solve) |
| `qpbounds_test.go` (new) | braking cap, bounds and step-equivalence tests |
| `loop.go` | `LoopConfig.Solver` and `Posture`; `solve` returns `taskRows`; steps 7, 7b and 8 move into `clampStep`; QP branch, fallback and fault; `peak_accel_ratio`; posture `q0` |
| `task.go` | `pointingAxisWeight`, `pointingTaskRows`, `SelfMotion` |
| `loop_test.go`, `golden_test.go` | both-solver table tests, posture tests, the QP golden scenario |
| `kin_test.go` | the `panda(t)` model loader |
| `module.go` | `solver` and `posture` parsing; status fields |
| `module_test.go` | config and status cases |
| `cmd/cli/main.go`, `cmd/cli/run.go` | `-solver` and `-posture-*` flags; `run hold` and `run orbit` scenarios; summary lines |
| `clients/python/servo_client.py`, `servo_bench.py`, `test_servo_client.py` | `--solver`; `peak_accel_ratio` in `FOLLOW_COLS` |
| `dev/exportmodel/main.go` (new) | writes an arm's served kinematic model to JSON |
| `testdata/panda.json` (new) | the 7-DoF Panda model fixture |
| `dev/panda-mujoco.json`, `dev/panda-mujoco.md` (new) | the Panda sim cell and its runbook |
| `README.md`, `devrel_diff-ik_servo.md` | documentation for the new fields |
| `docs/results/2026-11-qp-solver.md` (new) | Task 0 findings, the A/B table, and a recommendation |

---

### Task 1 (spec Task 0, items 1 and 2): Panda cell, model fixture, joint-limit path

The controller does this task, not a subagent, because it runs simulators on this Mac. Don't change `franka-mug-sim`'s cloud config, and don't change either sim repo.

**Files:**
- Create: `dev/panda-mujoco.json`, `dev/exportmodel/main.go`, `testdata/panda.json`
- Modify: `.gitignore` only if `testdata/` is ignored. It isn't today.

- [ ] **Step 1: Write the cell config.** Copy the `franka-mug-sim` part config (read it with the Viam MCP `read_machine_config`, machine id `e458a36c-31d3-43fd-97d4-1b84ea087e99`) into `dev/panda-mujoco.json`. Add:
  - `"network": {"bind_address": "localhost:8083"}`;
  - module `{"type":"local","name":"diff-ik","executable_path":"/Users/nick.hehr/src/diff-ik/bin/diff-ik"}`;
  - service `{"name":"servo","api":"rdk:service:motion","model":"devrel:diff-ik:servo","attributes":{"arms":["pick-arm"]}}`.

  Keep the existing `mujoco-sim` module, which points at `/Users/nick.hehr/src/viam-mujoco-sim/run.sh`. Check that `~/src/viam-mujoco-sim` is on branch `feat/custom-assets` (`git -C ~/src/viam-mujoco-sim branch --show-current`).

- [ ] **Step 2: Start it.**
  ```
  go build -o bin/diff-ik ./cmd/module
  viam-server -config dev/panda-mujoco.json   # run in the background, log to the scratchpad
  ```
  Wait for `Modules successfully added` in the log.

- [ ] **Step 3: Write the exporter, `dev/exportmodel/main.go`.**
  ```go
  // Command exportmodel writes an arm's kinematic model, as the machine serves it, to a JSON file
  // for unit tests (e.g. testdata/panda.json).
  package main

  import (
  	"context"
  	"flag"
  	"log"
  	"os"

  	"go.viam.com/rdk/components/arm"
  	"go.viam.com/rdk/logging"
  	"go.viam.com/rdk/referenceframe"
  	"go.viam.com/rdk/robot/client"
  	"go.viam.com/utils/rpc"
  	"google.golang.org/protobuf/encoding/protojson"
  )

  func main() {
  	addr := flag.String("address", "localhost:8083", "plaintext machine address")
  	name := flag.String("arm", "pick-arm", "arm component name")
  	out := flag.String("out", "testdata/panda.json", "output file")
  	flag.Parse()
  	ctx := context.Background()
  	m, err := client.New(ctx, *addr, logging.NewLogger("exportmodel"),
  		client.WithDialOptions(rpc.WithInsecure()), client.WithDisableSessions())
  	if err != nil {
  		log.Fatal(err)
  	}
  	defer m.Close(ctx)
  	a, err := arm.FromProvider(m, *name)
  	if err != nil {
  		log.Fatal(err)
  	}
  	model, err := a.Kinematics(ctx)
  	if err != nil {
  		log.Fatal(err)
  	}
  	// UnmarshalModelJSON reads the model config, not json.Marshal(model)'s wrapper (review finding)
  	b, err := protojson.Marshal(referenceframe.KinematicModelToProtobuf(model).GetModel())
  	if err != nil {
  		log.Fatal(err)
  	}
  	if err := os.WriteFile(*out, b, 0o644); err != nil {
  		log.Fatal(err)
  	}
  }
  ```
  Run `go run ./dev/exportmodel`. If Step 4's loader can't read the file back, stop and report what `a.Kinematics` returned.

- [ ] **Step 4: Add the loader to `kin_test.go`,** after `so101`:
  ```go
  // panda loads the 7-DOF Franka Panda test model (testdata/panda.json, exported from the MuJoCo sim).
  func panda(t *testing.T) referenceframe.Model {
  	t.Helper()
  	b, err := os.ReadFile("testdata/panda.json")
  	if err != nil {
  		t.Fatal(err)
  	}
  	m, err := referenceframe.UnmarshalModelJSON(b, "arm")
  	if err != nil {
  		t.Fatal(err)
  	}
  	if n := len(m.DoF()); n != 7 {
  		t.Fatalf("panda DOF = %d, want 7", n)
  	}
  	return m
  }

  func TestPandaFixture(t *testing.T) {
  	m := panda(t)
  	l, err := NewLoop(m, &simIO{}, LoopConfig{})
  	if err != nil {
  		t.Fatal(err)
  	}
  	t.Logf("panda limits: joint_deg_s %v joint_deg_s2 %v", l.Config().MaxJointDegPerS, l.Config().MaxJointDegPerS2)
  }
  ```
  Add `"os"` to the imports if it's missing. Run `go test -run TestPandaFixture -v`. Expected: PASS. Record the logged acceleration limits; the sim config declares 60°/s². If the model has no limits, the acceleration limit shows the default of 360.

- [ ] **Step 5: Check a DLS hold on the sim.** Run:
  ```
  go run ./cmd/cli --address localhost:8083 --insecure --arm pick-arm run static
  ```
  Expected: the targets converge, with `rate` PASS.

- [ ] **Step 6: Find the joint-limit path.**
  - Panda joint 4 (index 3) has an upper limit near −4°. From the cell's start joints, try straight lines outward in base-frame +x (away from the base) using servo twists.
  - Write a scratch script in the scratchpad (not the repo) that runs `servo_start` (DLS, `"mode": "twist"`) and streams `servo_twist` `linear [30,0,0]` mm/s every 100 ms. Meanwhile it reads the arm's joints with `get_joint_positions` at 10 Hz, until joint index 3 is within 2° of its upper limit or 10 s pass, then sends `servo_stop`.
  - Record the start joints, the direction, and the length travelled to reach the limit.
  - If +x doesn't reach it within the table and reach limits, try the line from a lower start (`--start-joints`) and record what worked. If no path reaches it, stop and report.

- [ ] **Step 7: Record the findings.** Create `docs/results/2026-11-qp-solver.md` with a "Task 0" section: the model's DoF and limits, whether the DLS hold converged, and the joint-limit path (start joints in radians, direction, length in mm).

- [ ] **Step 8: Commit.**
  ```bash
  git add dev/panda-mujoco.json dev/exportmodel testdata/panda.json kin_test.go docs/results/2026-11-qp-solver.md
  git commit -m "QP Task 0: Panda sim cell, model fixture, joint-limit path"
  ```

---

### Task 2: The QP solver (`qp.go`)

**Files:**
- Create: `qp.go`, `qp_test.go`

- [ ] **Step 1: Write the failing tests, `qp_test.go`.**
  ```go
  package diffik

  import (
  	"math"
  	"math/rand"
  	"slices"
  	"testing"
  	"time"

  	"gonum.org/v1/gonum/mat"
  )

  func sym(n int, v ...float64) *mat.SymDense { return mat.NewSymDense(n, v) }

  func wide(n int) (lb, ub []float64) {
  	lb, ub = make([]float64, n), make([]float64, n)
  	for i := range lb {
  		lb[i], ub[i] = -1e6, 1e6
  	}
  	return lb, ub
  }

  // nearVec is the existing helper in kin_test.go; don't redeclare it.

  func TestQPUnconstrained(t *testing.T) {
  	lb, ub := wide(2)
  	x, act, _, err := solveQP(qpProblem{H: sym(2, 2, 0, 0, 4), g: []float64{-2, -8}, lb: lb, ub: ub})
  	if err != nil {
  		t.Fatal(err)
  	}
  	nearVec(t, x, []float64{1, 2}, 1e-12)
  	if act != 0 {
  		t.Fatalf("active = %d, want 0", act)
  	}
  }

  func TestQPBounds(t *testing.T) {
  	x, act, _, err := solveQP(qpProblem{H: sym(2, 2, 0, 0, 4), g: []float64{-2, -8},
  		lb: []float64{-1, -1}, ub: []float64{0.5, 1}})
  	if err != nil {
  		t.Fatal(err)
  	}
  	nearVec(t, x, []float64{0.5, 1}, 1e-12)
  	if act != 2 {
  		t.Fatalf("active = %d, want 2", act)
  	}
  }

  func TestQPCoupledBound(t *testing.T) {
  	// unconstrained minimum (1, 1); with x0 ≤ 0 the minimum is (0, 1.5)
  	x, _, _, err := solveQP(qpProblem{H: sym(2, 2, 1, 1, 2), g: []float64{-3, -3},
  		lb: []float64{-10, -10}, ub: []float64{0, 10}})
  	if err != nil {
  		t.Fatal(err)
  	}
  	nearVec(t, x, []float64{0, 1.5}, 1e-12)
  }

  func TestQPGeneralRowExcludingOrigin(t *testing.T) {
  	lb, ub := wide(2)
  	x, _, _, err := solveQP(qpProblem{H: sym(2, 1, 0, 0, 1), g: []float64{0, 0}, lb: lb, ub: ub,
  		A: mat.NewDense(1, 2, []float64{1, 1}), b: []float64{2}})
  	if err != nil {
  		t.Fatal(err)
  	}
  	nearVec(t, x, []float64{1, 1}, 1e-12)
  }

  func TestQPPinnedBound(t *testing.T) {
  	x, _, _, err := solveQP(qpProblem{H: sym(1, 1), g: []float64{-5}, lb: []float64{0.3}, ub: []float64{0.3}})
  	if err != nil {
  		t.Fatal(err)
  	}
  	nearVec(t, x, []float64{0.3}, 1e-12)
  }

  func TestQPErrors(t *testing.T) {
  	if _, _, _, err := solveQP(qpProblem{H: sym(1, 1), g: []float64{0}, lb: []float64{1}, ub: []float64{0}}); err == nil {
  		t.Error("infeasible bounds: want error")
  	}
  	lb, ub := wide(1)
  	if _, _, _, err := solveQP(qpProblem{H: sym(1, 1), g: []float64{0}, lb: lb, ub: ub,
  		A: mat.NewDense(2, 1, []float64{1, -1}), b: []float64{1, 0}}); err == nil {
  		t.Error("infeasible rows: want error")
  	}
  	if _, _, _, err := solveQP(qpProblem{H: sym(2, 1, 0, 0, -1), g: []float64{0, 0}, lb: []float64{-1, -1}, ub: []float64{1, 1}}); err == nil {
  		t.Error("indefinite H: want error")
  	}
  	orig := qpMaxIter
  	t.Cleanup(func() { qpMaxIter = orig })
  	qpMaxIter = 1
  	if _, _, _, err := solveQP(qpProblem{H: sym(2, 2, 0, 0, 4), g: []float64{-2, -8},
  		lb: []float64{-1, -1}, ub: []float64{0.5, 1}}); err == nil {
  		t.Error("iteration cap: want error")
  	}
  }

  // randomQP builds a positive definite problem whose bounds and rows all hold at a known point.
  func randomQP(r *rand.Rand) qpProblem {
  	n := 1 + r.Intn(7)
  	M := mat.NewDense(n, n, nil)
  	for i := 0; i < n; i++ {
  		for j := 0; j < n; j++ {
  			M.Set(i, j, r.NormFloat64())
  		}
  	}
  	H := mat.NewSymDense(n, nil)
  	for i := 0; i < n; i++ {
  		for j := i; j < n; j++ {
  			v := mat.Dot(M.ColView(i), M.ColView(j))
  			if i == j {
  				v += 0.1
  			}
  			H.SetSym(i, j, v)
  		}
  	}
  	xs, g, lb, ub := make([]float64, n), make([]float64, n), make([]float64, n), make([]float64, n)
  	for i := range xs {
  		xs[i], g[i] = r.NormFloat64(), 3*r.NormFloat64()
  		lb[i], ub[i] = xs[i]-r.Float64(), xs[i]+r.Float64()
  	}
  	m := r.Intn(6)
  	var A *mat.Dense
  	b := make([]float64, m)
  	if m > 0 {
  		A = mat.NewDense(m, n, nil)
  		for k := 0; k < m; k++ {
  			s := 0.0
  			for j := 0; j < n; j++ {
  				A.Set(k, j, r.NormFloat64())
  				s += A.At(k, j) * xs[j]
  			}
  			b[k] = s - r.Float64()
  		}
  	}
  	return qpProblem{H: H, g: g, lb: lb, ub: ub, A: A, b: b}
  }

  func TestQPPropertyKKT(t *testing.T) {
  	r := rand.New(rand.NewSource(1))
  	for trial := 0; trial < 2000; trial++ {
  		p := randomQP(r)
  		res, err := qpCore(p)
  		if err != nil {
  			t.Fatalf("trial %d: %v", trial, err)
  		}
  		C, d := qpRows(p)
  		n := len(p.g)
  		for i := range C {
  			if s := mat.Dot(mat.NewVecDense(n, C[i]), mat.NewVecDense(n, res.x)) - d[i]; s < -1e-9 {
  				t.Fatalf("trial %d: constraint %d violated by %g", trial, i, -s)
  			}
  		}
  		// stationarity: Hx + g = Σ u_j c_j over the active set, u ≥ 0
  		grad := make([]float64, n)
  		for i := 0; i < n; i++ {
  			grad[i] = p.g[i]
  			for j := 0; j < n; j++ {
  				grad[i] += p.H.At(i, j) * res.x[j]
  			}
  		}
  		for k, c := range res.act {
  			if res.u[k] < -1e-9 {
  				t.Fatalf("trial %d: negative multiplier %g", trial, res.u[k])
  			}
  			for i := 0; i < n; i++ {
  				grad[i] -= res.u[k] * C[c][i]
  			}
  		}
  		for i, v := range grad {
  			if math.Abs(v) > 1e-9 {
  				t.Fatalf("trial %d: stationarity residual %g at %d", trial, v, i)
  			}
  		}
  	}
  }

  // spec Task 0 item 3: p99 solve time under 0.5 ms at n = 7 with 7 bounds active.
  func TestQPSolveTime(t *testing.T) {
  	if testing.Short() {
  		t.Skip("timing")
  	}
  	n := 7
  	H := mat.NewSymDense(n, nil)
  	g, lb, ub := make([]float64, n), make([]float64, n), make([]float64, n)
  	for i := 0; i < n; i++ {
  		for j := i; j < n; j++ {
  			H.SetSym(i, j, 0.1)
  		}
  		H.SetSym(i, i, 1+float64(i))
  		g[i], lb[i], ub[i] = -10, -0.01, 0.01 // the unconstrained minimum is far outside: every upper bound binds
  	}
  	p := qpProblem{H: H, g: g, lb: lb, ub: ub}
  	ds := make([]time.Duration, 10000)
  	for k := range ds {
  		t0 := time.Now()
  		if _, act, _, err := solveQP(p); err != nil || act != n {
  			t.Fatalf("act %d err %v", act, err)
  		}
  		ds[k] = time.Since(t0)
  	}
  	slices.Sort(ds)
  	p99 := ds[len(ds)*99/100]
  	t.Logf("solveQP n=7, 7 active: p50 %v p99 %v", ds[len(ds)/2], p99)
  	J := mat.NewDense(6, 7, nil)
  	for i := 0; i < 6; i++ {
  		for j := 0; j < 7; j++ {
  			J.Set(i, j, math.Sin(float64(i*7+j)))
  		}
  	}
  	t0 := time.Now()
  	for k := 0; k < 1000; k++ {
  		if _, _, _, err := dlsStep(J, [6]float64{1, 1, 1, 1, 1, 1}, [6]float64{1e-3}, 0.04, 0.1); err != nil {
  			t.Fatal(err)
  		}
  	}
  	t.Logf("dlsStep 6×7: mean %v", time.Since(t0)/1000)
  	if p99 > 500*time.Microsecond {
  		t.Fatalf("p99 %v exceeds 0.5 ms", p99)
  	}
  }
  ```

- [ ] **Step 2: Run them and check they fail.** Run `go test -run 'TestQP' ./`. Expected: a build failure, `undefined: solveQP`.

- [ ] **Step 3: Write `qp.go`.**
  ```go
  package diffik

  import (
  	"errors"
  	"fmt"
  	"math"

  	"gonum.org/v1/gonum/mat"
  )

  // qpProblem is min ½xᵀHx + gᵀx subject to lb ≤ x ≤ ub and A·x ≥ b (spec "Solver: qp.go").
  type qpProblem struct {
  	H      *mat.SymDense // n×n, positive definite
  	g      []float64     // n
  	lb, ub []float64     // n
  	A      *mat.Dense    // m×n, nil for none
  	b      []float64     // m
  }

  type qpResult struct {
  	x     []float64
  	act   []int     // active constraint indices into qpRows' list
  	u     []float64 // their multipliers, ≥ 0
  	iters int
  }

  var qpMaxIter = 50 // a var so a test can force the cap

  const qpTol = 1e-12 // a constraint counts as violated only beyond this (spec: pinned joints must not cycle)

  var errQPInfeasible = errors.New("infeasible")

  // qpRows lists the constraints as rows c·x ≥ d: for each joint x_i ≥ lb_i then −x_i ≥ −ub_i, then A's rows.
  func qpRows(p qpProblem) (C [][]float64, d []float64) {
  	n := len(p.g)
  	for i := 0; i < n; i++ {
  		lo, hi := make([]float64, n), make([]float64, n)
  		lo[i], hi[i] = 1, -1
  		C, d = append(C, lo, hi), append(d, p.lb[i], -p.ub[i])
  	}
  	if p.A != nil {
  		m, _ := p.A.Dims()
  		for k := 0; k < m; k++ {
  			C, d = append(C, mat.Row(nil, k, p.A)), append(d, p.b[k])
  		}
  	}
  	return C, d
  }

  // solveQP solves p and returns x, the number of active constraints and the iteration count.
  func solveQP(p qpProblem) (x []float64, active, iters int, err error) {
  	r, err := qpCore(p)
  	return r.x, len(r.act), r.iters, err
  }

  // qpCore is Goldfarb–Idnani's dual active-set method (Math. Programming 27, 1983), the method behind
  // quadprog. It starts at the unconstrained minimum and adds the most-violated constraint, dropping
  // constraints whose multipliers reach zero, so it needs no feasible start. J is an H-orthonormal basis
  // (JᵀHJ = I) whose first q columns span the active normals; R (q×q, upper triangular) is J₁ᵀN.
  func qpCore(p qpProblem) (qpResult, error) {
  	n := len(p.g)
  	C, d := qpRows(p)
  	var ch mat.Cholesky
  	if !ch.Factorize(p.H) {
  		return qpResult{}, errors.New("H is not positive definite")
  	}
  	var L, Linv mat.TriDense
  	ch.LTo(&L)
  	if err := Linv.InverseTri(&L); err != nil {
  		return qpResult{}, err
  	}
  	J := make([][]float64, n) // J = L⁻ᵀ
  	R := make([][]float64, n)
  	for i := range J {
  		J[i], R[i] = make([]float64, n), make([]float64, n)
  		for j := range J[i] {
  			J[i][j] = Linv.At(j, i)
  		}
  	}
  	var xv mat.VecDense
  	if err := ch.SolveVecTo(&xv, mat.NewVecDense(n, p.g)); err != nil {
  		return qpResult{}, err
  	}
  	x := make([]float64, n)
  	for i := range x {
  		x[i] = -xv.AtVec(i)
  	}
  	dot := func(a, b []float64) float64 {
  		s := 0.0
  		for i := range a {
  			s += a[i] * b[i]
  		}
  		return s
  	}
  	rot := func(c, s float64, a, b *float64) { *a, *b = c**a+s**b, -s**a+c**b }
  	var act []int
  	var u []float64
  	inA := make([]bool, len(C))
  	res := qpResult{}

  	add := func(pc int, dv []float64) { // dv = Jᵀc_p; zero dv[q+1:] by rotating J's columns
  		q := len(act)
  		for j := n - 1; j > q; j-- {
  			if dv[j] == 0 {
  				continue
  			}
  			h := math.Hypot(dv[j-1], dv[j])
  			c, s := dv[j-1]/h, dv[j]/h
  			dv[j-1], dv[j] = h, 0
  			for i := 0; i < n; i++ {
  				rot(c, s, &J[i][j-1], &J[i][j])
  			}
  		}
  		for i := 0; i <= q; i++ {
  			R[i][q] = dv[i]
  		}
  		act, inA[pc] = append(act, pc), true
  	}
  	drop := func(k int) { // remove active constraint k; restore R's triangle with rotations mirrored in J
  		q := len(act)
  		for j := k; j < q-1; j++ {
  			for i := 0; i <= j+1; i++ {
  				R[i][j] = R[i][j+1]
  			}
  		}
  		for i := 0; i < n; i++ {
  			R[i][q-1] = 0
  		}
  		for j := k; j < q-1; j++ {
  			if R[j+1][j] == 0 {
  				continue
  			}
  			h := math.Hypot(R[j][j], R[j+1][j])
  			c, s := R[j][j]/h, R[j+1][j]/h
  			for col := j; col < q-1; col++ {
  				rot(c, s, &R[j][col], &R[j+1][col])
  			}
  			R[j+1][j] = 0
  			for i := 0; i < n; i++ {
  				rot(c, s, &J[i][j], &J[i][j+1])
  			}
  		}
  		inA[act[k]] = false
  		act = append(act[:k], act[k+1:]...)
  	}

  	for {
  		pc, worst := -1, -qpTol
  		for i := range C {
  			if !inA[i] {
  				if s := dot(C[i], x) - d[i]; s < worst {
  					pc, worst = i, s
  				}
  			}
  		}
  		if pc < 0 {
  			res.x, res.act, res.u = x, act, u
  			return res, nil
  		}
  		up := append(append([]float64(nil), u...), 0)
  		for {
  			res.iters++
  			if res.iters > qpMaxIter {
  				return res, fmt.Errorf("no convergence in %d iterations", qpMaxIter)
  			}
  			q := len(act)
  			dv := make([]float64, n)
  			for j := 0; j < n; j++ {
  				for i := 0; i < n; i++ {
  					dv[j] += J[i][j] * C[pc][i]
  				}
  			}
  			z := make([]float64, n) // step direction in primal space: J₂d₂
  			for j := q; j < n; j++ {
  				for i := 0; i < n; i++ {
  					z[i] += J[i][j] * dv[j]
  				}
  			}
  			r := make([]float64, q) // R⁻¹d₁: how the active multipliers change
  			for i := q - 1; i >= 0; i-- {
  				s := dv[i]
  				for k := i + 1; k < q; k++ {
  					s -= R[i][k] * r[k]
  				}
  				r[i] = s / R[i][i]
  			}
  			t1, k := math.Inf(1), -1 // partial step: a multiplier reaches zero
  			for j := 0; j < q; j++ {
  				if r[j] > qpTol {
  					if t := up[j] / r[j]; t < t1 {
  						t1, k = t, j
  					}
  				}
  			}
  			t2 := math.Inf(1) // full step: constraint pc becomes active
  			if zc := dot(z, C[pc]); zc > qpTol {
  				t2 = -(dot(C[pc], x) - d[pc]) / zc
  			}
  			t := math.Min(t1, t2)
  			if math.IsInf(t, 1) {
  				return res, errQPInfeasible
  			}
  			for j := 0; j < q; j++ {
  				up[j] -= t * r[j]
  			}
  			up[q] += t
  			if !math.IsInf(t2, 1) {
  				for i := range x {
  					x[i] += t * z[i]
  				}
  			}
  			if t == t2 {
  				add(pc, dv)
  				u = up
  				break
  			}
  			drop(k)
  			up = append(up[:k], up[k+1:]...)
  		}
  	}
  }
  ```

- [ ] **Step 4: Run the tests and check they pass.** Run `go test -run 'TestQP' -v ./`. Expected: all PASS, with the timing log showing p99 well under 0.5 ms.
  - If the KKT property test fails, debug the solver with @superpowers:systematic-debugging; don't loosen the test.
  - The most likely fault is a sign in `rot` or the `drop` re-triangularisation. Check against Goldfarb & Idnani §3 or quadprog's `solve.QP.f`.

- [ ] **Step 5: Commit.**
  ```bash
  git add qp.go qp_test.go
  git commit -m "QP solver: Goldfarb–Idnani dual active-set (bounds and general rows)"
  ```

---

### Task 3: Config, `solver` and `posture`

**Files:**
- Modify: `loop.go` (`LoopConfig`, `withDefaults`, `validate`, new consts and type)
- Test: `loop_test.go` (`TestNewLoopRejectsBadConfig`, plus a new defaults test)

- [ ] **Step 1: Write the failing tests.** Add these cases to the map in `TestNewLoopRejectsBadConfig` (around `loop_test.go:93`):
  ```go
  "bad solver":           {Solver: "osqp"},
  "posture without qp":   {Posture: &Posture{}},
  "posture joint count":  {Solver: SolverQP, Posture: &Posture{JointsDeg: []float64{0, 0}}},
  "posture weight < 0":   {Solver: SolverQP, Posture: &Posture{Weight: -1}},
  "posture gain < 0":     {Solver: SolverQP, Posture: &Posture{Gain: -1}},
  ```
  Then add:
  ```go
  func TestSolverAndPostureDefaults(t *testing.T) {
  	l, _, _ := newTestLoop(t, home, LoopConfig{})
  	if l.Config().Solver != SolverDLS || l.Config().Posture != nil {
  		t.Fatalf("defaults: solver %q posture %v", l.Config().Solver, l.Config().Posture)
  	}
  	l, _, _ = newTestLoop(t, home, LoopConfig{Solver: SolverQP, Posture: &Posture{}})
  	if p := l.Config().Posture; p.Weight != 1e-3 || p.Gain != 1 || p.JointsDeg != nil {
  		t.Fatalf("posture defaults %+v", *p)
  	}
  }
  ```

- [ ] **Step 2: Run them and check they fail.** Run `go test -run 'TestNewLoopRejectsBadConfig|TestSolverAndPostureDefaults' ./`. Expected: a build failure, `unknown field Solver`.

- [ ] **Step 3: Implement it.** In `loop.go`, add these fields to `LoopConfig` after `Task`:
  ```go
  	Solver  string   // "dls" (default) | "qp" (spec "QP solver and posture task")
  	Posture *Posture // QP only: pull toward a preferred posture
  ```
  Add, next to the `LoopConfig` type:
  ```go
  // Solver names.
  const (
  	SolverDLS = "dls"
  	SolverQP  = "qp"
  )

  // Posture is the QP's posture task: w_p‖Δq − scale·k·dt·(q₀ − q)‖² (spec "Objective").
  type Posture struct {
  	JointsDeg []float64 // q₀ in degrees; nil means the joints at the first tick
  	Weight    float64   // w_p, default 1e-3
  	Gain      float64   // k in 1/s, default 1
  }
  ```
  At the end of `withDefaults`, before `return c`:
  ```go
  	if c.Solver == "" {
  		c.Solver = SolverDLS
  	}
  	if c.Posture != nil {
  		p := *c.Posture
  		def(&p.Weight, 1e-3)
  		def(&p.Gain, 1)
  		c.Posture = &p
  	}
  ```
  At the end of `validate`, before `return nil`:
  ```go
  	if c.Solver != SolverDLS && c.Solver != SolverQP {
  		return fmt.Errorf("unsupported solver %q: use %q or %q", c.Solver, SolverDLS, SolverQP)
  	}
  	if p := c.Posture; p != nil {
  		if c.Solver != SolverQP {
  			return fmt.Errorf("posture needs solver %q", SolverQP)
  		}
  		if p.JointsDeg != nil && len(p.JointsDeg) != dof {
  			return fmt.Errorf("posture.joints_deg needs %d values, got %d", dof, len(p.JointsDeg))
  		}
  		if !(p.Weight > 0) || !(p.Gain > 0) {
  			return fmt.Errorf("posture weight and gain must be > 0, got %v and %v", p.Weight, p.Gain)
  		}
  	}
  ```

- [ ] **Step 4: Run the tests.** Run `go test ./...`. Expected: everything passes.

- [ ] **Step 5: Commit** with message `"LoopConfig: solver and posture"`.

---

### Task 4: Braking cap and QP bounds

**Files:**
- Create: `qpbounds.go`, `qpbounds_test.go`

- [ ] **Step 1: Write the failing tests, `qpbounds_test.go`.**
  ```go
  package diffik

  import (
  	"math"
  	"math/rand"
  	"testing"
  )

  // spec "Bounds": a step obeying brakeCap, slowed by a·dt² per tick, stops within d, and the next tick's
  // cap is at least s − a·dt². The random cases include non-integer s/(a·dt²).
  func TestBrakeCapStopsInTime(t *testing.T) {
  	r := rand.New(rand.NewSource(2))
  	dt := 0.02
  	for k := 0; k < 100000; k++ {
  		a := []float64{0.1, 1, 6.28, 20}[k%4]
  		d := r.Float64() * 0.5
  		c := a * dt * dt
  		s := r.Float64() * brakeCap(a, d, dt)
  		if k%3 == 0 {
  			s = brakeCap(a, d, dt) // at the cap exactly
  		}
  		if next := brakeCap(a, d-s, dt); next < s-c-1e-12 {
  			t.Fatalf("a %v d %v s %v: next cap %v below s−c %v", a, d, s, next, s-c)
  		}
  		travel := 0.0
  		for v := s; v > 0; v -= c {
  			travel += v
  		}
  		if travel > d+1e-12 {
  			t.Fatalf("a %v d %v s %v: stops after %v", a, d, s, travel)
  		}
  	}
  	if brakeCap(1, 0, dt) != 0 || brakeCap(1, -1, dt) != 0 {
  		t.Fatal("no distance left: cap must be 0")
  	}
  	// a huge acceleration limit must not freeze the arm: the cap tends to d (review finding)
  	if c := brakeCap(1e9, 0.01, dt); c < 0.01-1e-12 {
  		t.Fatalf("brakeCap(1e9, 0.01) = %v, want 0.01", c)
  	}
  }

  func TestQPBoundsIntersect(t *testing.T) {
  	l, _, _ := newTestLoop(t, home, LoopConfig{Solver: SolverQP})
  	runTicks(t, l, 1) // sets qCmd
  	dt := 1 / l.cfg.RateHz
  	l.lastStep[0] = 0.001
  	lb, ub := l.qpBounds(dt)
  	a := deg2rad(l.cfg.MaxJointDegPerS2[0]) * dt * dt
  	v := deg2rad(l.cfg.MaxJointDegPerS[0]) * dt
  	if want := math.Min(0.001+a, v); math.Abs(ub[0]-want) > 1e-15 {
  		t.Fatalf("ub[0] = %v, want %v", ub[0], want)
  	}
  	if want := 0.001 - a; math.Abs(lb[0]-want) > 1e-15 {
  		t.Fatalf("lb[0] = %v, want %v", lb[0], want)
  	}
  	for i := range lb {
  		if lb[i] > ub[i] {
  			t.Fatalf("joint %d: empty interval", i)
  		}
  	}
  }

  // spec "Bounds": an empty intersection (after a resync) lets position win: the point of the position
  // interval nearest the acceleration interval.
  func TestQPBoundsPositionWins(t *testing.T) {
  	l, _, _ := newTestLoop(t, home, LoopConfig{Solver: SolverQP})
  	runTicks(t, l, 1)
  	dt := 1 / l.cfg.RateHz
  	l.qCmd[0] = l.hi[0] - 0.001
  	l.lastStep[0] = 0.05 // far faster than the remaining 1 mrad allows
  	lb, ub := l.qpBounds(dt)
  	if want := l.hi[0] - l.qCmd[0]; lb[0] != want || ub[0] != want {
  		t.Fatalf("got [%v, %v], want both %v", lb[0], ub[0], want)
  	}
  }
  ```

- [ ] **Step 2: Run them and check they fail.** Run `go test -run 'TestBrakeCap|TestQPBounds' ./`. Expected: `undefined: brakeCap`.

- [ ] **Step 3: Write `qpbounds.go`.**
  ```go
  package diffik

  import "math"

  // brakeCap is the largest step (rad per tick) toward a limit d rad away that can still stop in time when
  // slowed by a·dt² per tick, for any step size (spec "Bounds": the discrete braking cap). The min(d, c)
  // term covers d < c/8, where the first term goes negative: a step s ≤ c stops on the next tick.
  func brakeCap(a, d, dt float64) float64 {
  	c := a * dt * dt
  	return math.Max(math.Max(math.Sqrt(2*a*math.Max(d, 0))*dt-c/2, math.Min(d, c)), 0)
  }

  // qpBounds returns per-joint bounds on this tick's step (spec "Bounds (QP path)"): position, velocity,
  // discrete braking and acceleration, intersected. If the intersection is empty, position wins.
  func (l *Loop) qpBounds(dt float64) (lb, ub []float64) {
  	n := len(l.qCmd)
  	lb, ub = make([]float64, n), make([]float64, n)
  	for i := 0; i < n; i++ {
  		a := deg2rad(l.cfg.MaxJointDegPerS2[i])
  		v := deg2rad(l.cfg.MaxJointDegPerS[i]) * dt
  		plo, phi := l.lo[i]-l.qCmd[i], l.hi[i]-l.qCmd[i]
  		lo := math.Max(math.Max(plo, -v), -brakeCap(a, -plo, dt))
  		hi := math.Min(math.Min(phi, v), brakeCap(a, phi, dt))
  		alo, ahi := l.lastStep[i]-a*dt*dt, l.lastStep[i]+a*dt*dt
  		lb[i], ub[i] = math.Max(lo, alo), math.Min(hi, ahi)
  		if lb[i] > ub[i] { // position (and velocity) win: the feasible step nearest the acceleration interval
  			x := math.Min(math.Max(l.lastStep[i], math.Max(plo, -v)), math.Min(phi, v))
  			lb[i], ub[i] = x, x
  		}
  	}
  	return lb, ub
  }
  ```
  Note that `brakeCap(a, -plo, dt)` uses the distance down to `lo`, since `-plo = qCmd − lo`.

- [ ] **Step 4: Run the tests and check they pass.** Run `go test ./...`. Expected: PASS.
  - Note on `TestQPBoundsIntersect`: at `home`, joint 0 sits far from its limits, so the braking cap doesn't bind. If it does, pick a joint that's far from its limits.

- [ ] **Step 5: Commit** with message `"QP bounds: discrete braking cap, acceleration and position"`.

---

### Task 5: Refactor `solve` and steps 7 to 8 (no behaviour change)

**Files:**
- Modify: `loop.go` (`solve`, `tick` steps 7 to 8, new `clampStep`, `taskRows`)
- Modify: `task.go` (`pointingAxisWeight`, `pointingTaskRows`)

- [ ] **Step 1: Move steps 7, 7b and 8 into `clampStep`.** In `tick`, replace the block from `// 7. joint velocity clamp` through the end of `// 8. position limits` (the `next` loop) with:
  ```go
  	next, limStep, err := l.clampStep(dq, dt)
  	if err != nil {
  		return err
  	}
  ```
  Then change `limited := s < 1 || clamped || limitedTask || accelLimited` to `limited := limStep || limitedTask`. Add the function, moving the original code without changing any arithmetic:
  ```go
  // clampStep applies steps 7, 7b and 8 (the DLS path) to dq: the uniform speed scaling, the acceleration
  // limit and the position clamp. It returns the joints to send and whether any of them bound.
  func (l *Loop) clampStep(dq []float64, dt float64) (next []float64, limited bool, err error) {
  	// 7. joint velocity clamp, direction kept
  	s := 1.0
  	for i, d := range dq {
  		if math.IsNaN(d) || math.IsInf(d, 0) {
  			return nil, false, fmt.Errorf("non-finite joint step %v", dq)
  		}
  		if lim := deg2rad(l.cfg.MaxJointDegPerS[i]) * dt; math.Abs(d)*s > lim {
  			s = lim / math.Abs(d)
  		}
  	}
  	// 7b. acceleration: brake ahead of position limits, then clamp the velocity change, all joints together
  	step := make([]float64, len(dq))
  	for i, d := range dq {
  		step[i] = d * s
  	}
  	accelLimited := l.limitAccel(step, dt)
  	// 8. position limits
  	next = make([]float64, len(dq))
  	clamped := false
  	for i := range dq {
  		v := l.qCmd[i] + step[i]
  		if v < l.lo[i] || v > l.hi[i] {
  			v = math.Min(math.Max(v, l.lo[i]), l.hi[i])
  			clamped = true
  		}
  		next[i] = v
  	}
  	return next, s < 1 || clamped || accelLimited, nil
  }
  ```

- [ ] **Step 2: Make `solve` return the weighted task rows.** Add to `loop.go`:
  ```go
  // taskRows is the weighted task the QP's objective uses: ‖A·Δq − b‖² with A = W^½J and b = W^½Δx.
  // solve fills it only for solver qp.
  type taskRows struct {
  	A      *mat.Dense
  	b      []float64
  	lambda float64 // the QP's damping: for pointing, max(λ₁, λ₂) from the first prioritySolve, before lockOutward
  }
  ```
  Change `solve`'s signature to return `(dq []float64, lambda float64, limited bool, cmdPos, cmdAng float64, rows taskRows, err error)` and update every return. Then:
  - **Pointing branch:** right after `dx1, dx2 := ...`, before `prioritySolve` (`lockOutward` zeroes J1 and J2's columns in place, so copy first):
    ```go
    if c.Solver == SolverQP {
    	rows = pointingTaskRows(J1, J2, dx1, dx2)
    }
    ```
    and right after the first `prioritySolve` call (before the `lockOutward` loop), `rows.lambda = math.Max(l1, l2)` when `c.Solver == SolverQP`, so the QP's damping comes from the unlocked rows (spec "Damping") while the locked re-solve stays the fallback.
  - **Pose branch, `len(rows) >= n` path** (rename the local `rows []int` to `active` to avoid the clash):
    ```go
    if c.Solver == SolverQP {
    	A, b := mat.NewDense(6, n, nil), make([]float64, 6)
    	for i := 0; i < 6; i++ {
    		sw := math.Sqrt(c.TaskWeights[i])
    		for j := 0; j < n; j++ {
    			A.Set(i, j, sw*J.At(i, j))
    		}
    		b[i] = sw * dx[i]
    	}
    	rows = taskRows{A: A, b: b} // lambda set below from dlsStep
    }
    ```
  - **Pose branch, `dpinv` path:** after `A` and `b` are built, set `rows = taskRows{A: A, b: b}` when `c.Solver == SolverQP`. (These are the same weighted rows.)
  - **Both pose paths:** once the DLS or `dpinv` λ is known, set `rows.lambda` to it (`rows.lambda = lambda`).

  In `task.go`, add:
  ```go
  // pointingAxisWeight weighs the QP's two tool-axis rows against the position rows (weight 1): its square
  // root, 0.01, makes 1 rad of axis error cost what 10 mm of position error does, so position wins
  // whenever the two conflict (spec "Objective"; stands in for prioritySolve's strict priority).
  const pointingAxisWeight = 1e-4

  // pointingTaskRows stacks J1 (3×n) over √pointingAxisWeight·J2 (2×n), with the matching Δx.
  func pointingTaskRows(J1, J2 *mat.Dense, dx1, dx2 []float64) taskRows {
  	_, n := J1.Dims()
  	sw := math.Sqrt(pointingAxisWeight)
  	A, b := mat.NewDense(5, n, nil), make([]float64, 5)
  	for j := 0; j < n; j++ {
  		for i := 0; i < 3; i++ {
  			A.Set(i, j, J1.At(i, j))
  		}
  		for i := 0; i < 2; i++ {
  			A.Set(3+i, j, sw*J2.At(i, j))
  		}
  	}
  	copy(b, dx1)
  	b[3], b[4] = sw*dx2[0], sw*dx2[1]
  	return taskRows{A: A, b: b}
  }
  ```
  Update the call in `tick` to `dq, lambda, limitedTask, cmdPos, cmdAng, rows, err := l.solve(...)`. Leave `rows` unused for now (`_ = rows`); Task 6 uses it.

- [ ] **Step 3: Check nothing changed.** Run `go test ./...`. Expected: everything passes, including `TestGoldenUR5ePoseCommands`, which is bit-exact.

- [ ] **Step 4: Commit** with message `"Loop: clampStep and taskRows (refactor, DLS path unchanged)"`.

---

### Task 6: The QP tick: objective, fallback, status fields

**Files:**
- Modify: `qpbounds.go` (`qpNext`), `loop.go` (`Loop` fields, `Status` fields, `tick`)
- Test: `loop_test.go`

- [ ] **Step 1: Write the failing tests** in `loop_test.go`:
  ```go
  // spec "Fallback and faults": a failing solver falls back tick by tick and faults after 10 in a row.
  func TestQPFallbackThenFault(t *testing.T) {
  	orig := qpSolver
  	t.Cleanup(func() { qpSolver = orig })
  	qpSolver = func(qpProblem) ([]float64, int, int, error) { return nil, 0, 1, errQPInfeasible }
  	l, sim, m := newTestLoop(t, home, LoopConfig{Solver: SolverQP})
  	l.SetTarget(offsetTarget(t, m, home, r3.Vector{X: 20}, 0))
  	for i := 0; i < 9; i++ {
  		if err := l.tick(context.Background(), 1/l.cfg.RateHz); err != nil {
  			t.Fatalf("tick %d: %v", i, err)
  		}
  	}
  	if s := l.Status(); s.QPFallbacks != 9 || len(sim.sends) != 9 {
  		t.Fatalf("fallbacks %d sends %d, want 9 and 9", s.QPFallbacks, len(sim.sends))
  	}
  	err := l.tick(context.Background(), 1/l.cfg.RateHz)
  	if err == nil || !strings.Contains(err.Error(), "qp: infeasible") {
  		t.Fatalf("tick 10: err %v, want qp: infeasible", err)
  	}
  }

  func TestQPConverges(t *testing.T) {
  	l, _, m := newTestLoop(t, home, LoopConfig{Solver: SolverQP})
  	l.SetTarget(offsetTarget(t, m, home, r3.Vector{X: 30, Y: 30, Z: -25}, deg2rad(10)))
  	runTicks(t, l, 200)
  	s := l.Status()
  	if s.State != StateConverged || s.QPFallbacks != 0 {
  		t.Fatalf("state %s fallbacks %d, want converged and 0", s.State, s.QPFallbacks)
  	}
  	if s.PeakAccelRatio > 1+1e-9 {
  		t.Fatalf("peak accel ratio %v > 1", s.PeakAccelRatio)
  	}
  }

  // spec "Config and status": peak_accel_ratio must be able to fail.
  // White box: joint 0 is 0.5 rad below its limit and the previous step is said to be 1 rad toward it, so
  // limitAccel keeps the step near 1 rad and step 8's position clamp then cuts it: a real violation.
  func TestPeakAccelRatioCanExceedOne(t *testing.T) {
  	m := ur5e(t)
  	q := slices.Clone(home)
  	q[0] = m.DoF()[0].Max - fdStep - 0.5
  	l, _, _ := newTestLoop(t, q, LoopConfig{})
  	l.SetTarget(offsetTarget(t, m, q, r3.Vector{X: 30}, 0))
  	runTicks(t, l, 5)
  	l.lastStep[0] += 1
  	runTicks(t, l, 1)
  	if r := l.Status().PeakAccelRatio; !(r > 1+1e-9) {
  		t.Fatalf("peak accel ratio %v, want > 1", r)
  	}
  }
  ```
  Add `"strings"` to the test imports if it's missing.

- [ ] **Step 2: Run them and check they fail.** Run `go test -run 'TestQPFallback|TestQPConverges|TestPeakAccel' ./`. Expected: a build failure (`undefined: qpSolver`, `s.QPFallbacks`).

- [ ] **Step 3: Implement it.**
  - In `Status`, add after `ReachDeg`:
    ```go
    	QPActive, QPIters  int     // solver qp: active constraints and iterations this tick
    	QPFallbacks        int     // solver qp: ticks that fell back to the DLS path, session total
    	PeakAccelRatio     float64 // max over ticks and joints of |step − lastStep| / (a·dt²); ≤ 1 means the limit held
    ```
  - In `Loop`, add next to `lastStep`:
    ```go
    	q0        []float64 // posture target, rad; set at NewLoop or the first tick
    	qpFails   int       // consecutive QP fallbacks; loop goroutine only
    	peakAccel float64   // see Status.PeakAccelRatio; loop goroutine only
    ```
  - In `NewLoop`, after the limits loop:
    ```go
    	if p := cfg.Posture; p != nil && p.JointsDeg != nil {
    		l.q0 = make([]float64, dof)
    		for i, d := range p.JointsDeg {
    			l.q0[i] = deg2rad(d)
    		}
    	}
    ```
  - In `tick`, inside `if l.qCmd == nil {`, add:
    ```go
    		if l.cfg.Posture != nil && l.q0 == nil {
    			l.q0 = slices.Clone(qm)
    		}
    ```
  - In `qpbounds.go`, add:
    ```go
    // qpMu keeps H positive definite on a redundant arm with no damping and no posture, and stops near-null
    // directions (the SO-101's wrist roll under the pointing rows) from drifting (spec "Objective").
    const qpMu = 1e-6

    // qpSolver is solveQP; tests replace it to force failures.
    var qpSolver = solveQP

    // qpNext builds this tick's QP from the weighted task rows, the damping λ, the posture pull (scaled by
    // the ramp scale) and qpBounds, solves it, and returns the joints to send (spec "Objective").
    func (l *Loop) qpNext(rows taskRows, scale, dt float64) (next []float64, active, iters int, err error) {
    	n := len(l.qCmd)
    	m, _ := rows.A.Dims()
    	wp, pull := 0.0, make([]float64, n)
    	if p := l.cfg.Posture; p != nil {
    		wp = p.Weight
    		for i := range pull {
    			pull[i] = scale * p.Gain * dt * (l.q0[i] - l.qCmd[i])
    		}
    	}
    	H, g := mat.NewSymDense(n, nil), make([]float64, n)
    	for i := 0; i < n; i++ {
    		for j := i; j < n; j++ {
    			s := 0.0
    			for k := 0; k < m; k++ {
    				s += rows.A.At(k, i) * rows.A.At(k, j)
    			}
    			if i == j {
    				s += rows.lambda*rows.lambda + qpMu + wp
    			}
    			H.SetSym(i, j, s)
    		}
    		s := 0.0
    		for k := 0; k < m; k++ {
    			s += rows.A.At(k, i) * rows.b[k]
    		}
    		g[i] = -(s + wp*pull[i])
    	}
    	lb, ub := l.qpBounds(dt)
    	x, active, iters, err := qpSolver(qpProblem{H: H, g: g, lb: lb, ub: ub})
    	if err != nil {
    		return nil, 0, iters, err
    	}
    	next = make([]float64, n)
    	for i := range next {
    		next[i] = math.Min(math.Max(l.qCmd[i]+x[i], l.lo[i]), l.hi[i]) // guard (spec "Bounds")
    	}
    	return next, active, iters, nil
    }
    ```
    Add the `"gonum.org/v1/gonum/mat"` import.
  - In `tick`, replace the Task 5 `clampStep` call with:
    ```go
    	var next []float64
    	var limStep bool
    	var qpActive, qpIters int
    	fellBack := false
    	if l.cfg.Solver == SolverQP {
    		var qerr error
    		next, qpActive, qpIters, qerr = l.qpNext(rows, scale, dt)
    		if qerr != nil {
    			l.qpFails++
    			fellBack, next = true, nil
    			if l.qpFails >= 10 {
    				return fmt.Errorf("qp: %w", qerr)
    			}
    		} else {
    			l.qpFails, limStep = 0, qpActive > 0
    		}
    	}
    	if next == nil {
    		if next, limStep, err = l.clampStep(dq, dt); err != nil {
    			return err
    		}
    	}
    ```
    Remove `_ = rows`.
  - Replace the loop that sets `l.lastStep` after the send with:
    ```go
    	for i := range next {
    		step := next[i] - l.qCmd[i]
    		if r := math.Abs(step-l.lastStep[i]) / (deg2rad(l.cfg.MaxJointDegPerS2[i]) * dt * dt); r > l.peakAccel {
    			l.peakAccel = r
    		}
    		l.lastStep[i] = step
    	}
    ```
  - In the status block, after `st.QCmd, st.QMeas = ...`:
    ```go
    	st.QPActive, st.QPIters, st.PeakAccelRatio = qpActive, qpIters, l.peakAccel
    	if fellBack {
    		st.QPFallbacks++
    	}
    ```

- [ ] **Step 4: Run the tests.** Run `go test ./...`. Expected: everything passes, the goldens included. The DLS path's arithmetic is unchanged: the `lastStep` assignment computes the same value, and `peakAccel` only reads.

- [ ] **Step 5: Commit** with message `"Loop: solver qp tick with fallback, fault and peak_accel_ratio"`.

---

### Task 7: Step equivalence, posture, both-solver tests, QP golden

**Files:**
- Modify: `task.go` (`SelfMotion`), `qpbounds_test.go`, `loop_test.go`, `golden_test.go`, `testdata/golden_ur5e_arm64.json` (adds one key only)

- [ ] **Step 1: Add `SelfMotion` to `task.go`.** The CLI uses it too.
  ```go
  // SelfMotion moves q along the arm's self-motion (the null space of its 6×n Jacobian, n > 6) until joint
  // j has changed by delta rad, pulling the pose back to FK(q) after each small step. The result holds
  // the same end-effector pose with a different posture.
  func SelfMotion(m referenceframe.Model, q []float64, j int, delta float64) ([]float64, error) {
  	p0, o0, err := FK(m, q)
  	if err != nil {
  		return nil, err
  	}
  	cur := slices.Clone(q)
  	const h = 0.002 // rad of joint j per step
  	for k := 0; k < 10000 && math.Abs(delta-(cur[j]-q[j])) > 1e-6; k++ {
  		J, err := Jacobian(m, cur)
  		if err != nil {
  			return nil, err
  		}
  		_, n := J.Dims()
  		if n <= 6 {
  			return nil, fmt.Errorf("self-motion needs more than 6 joints, got %d", n)
  		}
  		var svd mat.SVD
  		if !svd.Factorize(J, mat.SVDFull) {
  			return nil, errSVD
  		}
  		var V mat.Dense
  		svd.VTo(&V)
  		v := mat.Col(nil, n-1, &V)
  		if math.Abs(v[j]) < 0.05 { // else h/v[j] would swing the other joints wildly (review finding)
  			return nil, fmt.Errorf("joint %d barely moves along the self-motion here (|v| = %.3g)", j, math.Abs(v[j]))
  		}
  		want := delta - (cur[j] - q[j])
  		s := math.Copysign(math.Min(h, math.Abs(want)), want) / v[j]
  		for i := range cur {
  			cur[i] += s * v[i]
  		}
  		for it := 0; it < 3; it++ {
  			p, o, err := FK(m, cur)
  			if err != nil {
  				return nil, err
  			}
  			e := PoseError(p0, o0, p, o)
  			Jc, err := Jacobian(m, cur)
  			if err != nil {
  				return nil, err
  			}
  			P, _, _, _, err := dpinv(Jc, 1, 0, false)
  			if err != nil {
  				return nil, err
  			}
  			var dq mat.VecDense
  			dq.MulVec(P, mat.NewVecDense(6, e[:]))
  			for i := range cur {
  				cur[i] += dq.AtVec(i)
  			}
  		}
  		for i, lim := range m.DoF() {
  			if cur[i] < lim.Min || cur[i] > lim.Max {
  				return nil, fmt.Errorf("self-motion took joint %d out of its limits", i)
  			}
  		}
  	}
  	return cur, nil
  }
  ```
  Add `"fmt"`, `"slices"` and `"go.viam.com/rdk/referenceframe"` to `task.go`'s imports as needed. Check that `Jacobian` and `FK` take `[]float64` (`[]referenceframe.Input`); in this RDK version `Input` is `float64`, as `tick` already relies on.

- [ ] **Step 2: Write the step-equivalence and limit tests** in `qpbounds_test.go`:
  ```go
  // qpVsDLS returns the QP step and the DLS (pre-clamp) step for loop l at its current state, with the
  // acceleration bound made slack by setting lastStep to the DLS step (spec "Tests → The step").
  func qpVsDLS(t *testing.T, l *Loop, target spatialmath.Pose) (qp, dls []float64, lambda float64) {
  	t.Helper()
  	dt := 1 / l.cfg.RateHz
  	runTicks(t, l, 1) // qCmd set, target stored
  	l.SetTarget(target)
  	J, err := Jacobian(l.model, l.qCmd)
  	if err != nil {
  		t.Fatal(err)
  	}
  	cp, cq, err := FK(l.model, l.qCmd)
  	if err != nil {
  		t.Fatal(err)
  	}
  	tp, tq := target.Point().Mul(1/mmPerM), target.Orientation().Quaternion()
  	dq, lambda, _, _, _, rows, err := l.solve(J, tp, tq, cp, cq, [6]float64{}, 1, dt)
  	if err != nil {
  		t.Fatal(err)
  	}
  	copy(l.lastStep, dq)
  	next, _, _, err := l.qpNext(rows, 1, dt)
  	if err != nil {
  		t.Fatal(err)
  	}
  	qp = make([]float64, len(next))
  	for i := range next {
  		qp[i] = next[i] - l.qCmd[i]
  	}
  	return qp, dq, lambda
  }

  func TestQPStepMatchesDLSPose(t *testing.T) {
  	for _, c := range []struct {
  		name  string
  		model func(*testing.T) referenceframe.Model
  		q     []float64
  	}{
  		{"ur5e", ur5e, home},
  		{"panda", panda, []float64{0, -0.3, 0, -2.0, 0, 1.8, 0.785}},
  	} {
  		t.Run(c.name, func(t *testing.T) {
  			m := c.model(t)
  			l, err := NewLoop(m, &simIO{q: slices.Clone(c.q)}, LoopConfig{Solver: SolverQP, Task: TaskPose,
  				MaxJointDegPerS: repeat(1000, len(c.q)), MaxJointDegPerS2: repeat(1e9, len(c.q))})
  			if err != nil {
  				t.Fatal(err)
  			}
  			qp, dls, lambda := qpVsDLS(t, l, offsetTarget(t, m, c.q, r3.Vector{X: 1, Y: -1, Z: 0.5}, deg2rad(0.5)))
  			if lambda != 0 {
  				t.Fatalf("precondition: λ = %v, want a well-conditioned configuration", lambda)
  			}
  			nearVec(t, qp, dls, 1e-6)
  		})
  	}
  }

  func TestQPStepMatchesPrioritySolveSO101(t *testing.T) {
  	m := so101(t)
  	q := []float64{0.2, -0.4, 0.6, 0.9, 0.1}
  	l, err := NewLoop(m, &simIO{q: slices.Clone(q)}, LoopConfig{Solver: SolverQP,
  		MaxJointDegPerS: repeat(1000, 5), MaxJointDegPerS2: repeat(1e9, 5)})
  	if err != nil {
  		t.Fatal(err)
  	}
  	qp, dls, lambda := qpVsDLS(t, l, offsetTarget(t, m, q, r3.Vector{X: 1, Z: -1}, deg2rad(0.5)))
  	if lambda != 0 {
  		t.Fatalf("precondition: λ = %v, want a well-conditioned configuration (pick another q)", lambda)
  	}
  	// The wrist roll (index 4) sits almost in the null space of the 5 pointing rows: dpinv's rank cut drops
  	// it, the QP doesn't. Compare the other joints, and the task-space motion J·Δq (review finding).
  	// μ = 1e-6 shrinks the 0.01-scaled axis rows' step by about μ/σ² ≈ 0.5%: a bias the gain loop absorbs.
  	// So the position rows must match tightly, the joints and angular rows to 5e-6.
  	nearVec(t, qp[:4], dls[:4], 5e-6)
  	J, err := Jacobian(m, l.qCmd)
  	if err != nil {
  		t.Fatal(err)
  	}
  	var a, b mat.VecDense
  	a.MulVec(J, mat.NewVecDense(5, qp))
  	b.MulVec(J, mat.NewVecDense(5, dls))
  	nearVec(t, a.RawVector().Data[:3], b.RawVector().Data[:3], 1e-8)
  	nearVec(t, a.RawVector().Data[3:], b.RawVector().Data[3:], 5e-6)
  }

  // With no posture, the QP must not drift the SO-101's near-null wrist roll (review finding: 2.3° in
  // 150 ticks at μ = 1e-9; qpMu = 1e-6 suppresses it).
  func TestQPPointingRollDoesNotDrift(t *testing.T) {
  	m := so101(t)
  	q := []float64{0.2, -0.4, 0.6, 0.9, 0.1}
  	sim := &simIO{q: slices.Clone(q)}
  	l, err := NewLoop(m, sim, LoopConfig{Solver: SolverQP, MaxJointDegPerS2: repeat(1e9, 5)})
  	if err != nil {
  		t.Fatal(err)
  	}
  	l.SetTarget(offsetTarget(t, m, q, r3.Vector{X: 20, Y: -15, Z: 10}, 0))
  	runTicks(t, l, 150)
  	if d := rad2deg(math.Abs(sim.q[4] - q[4])); d > 0.1 {
  		t.Fatalf("wrist roll drifted %.3f°, want ≤ 0.1", d)
  	}
  }

  func repeat(v float64, n int) []float64 {
  	out := make([]float64, n)
  	for i := range out {
  		out[i] = v
  	}
  	return out
  }

  // spec "Tests → The step": fuzzed state, the QP step never leaves its bounds.
  func TestQPStepStaysInBounds(t *testing.T) {
  	r := rand.New(rand.NewSource(3))
  	m := ur5e(t)
  	for trial := 0; trial < 300; trial++ {
  		q := slices.Clone(home)
  		for i := range q {
  			q[i] += 0.3 * r.NormFloat64()
  		}
  		l, err := NewLoop(m, &simIO{q: q}, LoopConfig{Solver: SolverQP})
  		if err != nil {
  			t.Fatal(err)
  		}
  		dt := 1 / l.cfg.RateHz
  		runTicks(t, l, 1)
  		for i := range l.lastStep {
  			l.lastStep[i] = 0.5 * r.NormFloat64() * deg2rad(l.cfg.MaxJointDegPerS[i]) * dt
  		}
  		target := offsetTarget(t, m, q, r3.Vector{X: 100 * r.NormFloat64(), Y: 100 * r.NormFloat64(), Z: 100 * r.NormFloat64()}, r.NormFloat64())
  		lb, ub := l.qpBounds(dt)
  		qpStep, _, _ := qpVsDLSKeepLast(t, l, target)
  		for i := range qpStep {
  			if qpStep[i] < lb[i]-1e-12 || qpStep[i] > ub[i]+1e-12 {
  				t.Fatalf("trial %d joint %d: step %v outside [%v, %v]", trial, i, qpStep[i], lb[i], ub[i])
  			}
  		}
  	}
  }
  ```
  `qpVsDLSKeepLast` is `qpVsDLS` without the `copy(l.lastStep, dq)` line and without the extra `runTicks`. Factor the shared body into one helper with a `keepLast bool` parameter; don't duplicate it.

  Then add the joint-limit comparison:
  ```go
  // spec "Tests → The step": near a joint limit the QP keeps more task progress than DLS plus clamps.
  func TestQPBeatsClampsNearLimit(t *testing.T) {
  	m := ur5e(t)
  	q := slices.Clone(home)
  	q[0] = m.DoF()[0].Max - fdStep - 0.002 // 2 mrad below joint 0's upper limit as the loop sees it
  	l, err := NewLoop(m, &simIO{q: q}, LoopConfig{Solver: SolverQP, MaxJointDegPerS2: repeat(1e9, 6)})
  	if err != nil {
  		t.Fatal(err)
  	}
  	dt := 1 / l.cfg.RateHz
  	runTicks(t, l, 1)
  	target := offsetTarget(t, m, q, r3.Vector{Y: -40}, 0) // needs joint 0 to rotate past its limit (+dq₀)
  	l.SetTarget(target)
  	J, _ := Jacobian(l.model, l.qCmd)
  	cp, cq, _ := FK(l.model, l.qCmd)
  	tp, tq := target.Point().Mul(1/mmPerM), target.Orientation().Quaternion()
  	dq, _, _, _, _, rows, err := l.solve(J, tp, tq, cp, cq, [6]float64{}, 1, dt)
  	if err != nil {
  		t.Fatal(err)
  	}
  	qpNext, _, _, err := l.qpNext(rows, 1, dt)
  	if err != nil {
  		t.Fatal(err)
  	}
  	clNext, _, err := l.clampStep(dq, dt)
  	if err != nil {
  		t.Fatal(err)
  	}
  	resid := func(next []float64) float64 {
  		r, c := rows.A.Dims()
  		s := 0.0
  		for i := 0; i < r; i++ {
  			v := -rows.b[i]
  			for j := 0; j < c; j++ {
  				v += rows.A.At(i, j) * (next[j] - l.qCmd[j])
  			}
  			s += v * v
  		}
  		return math.Sqrt(s)
  	}
  	if rq, rc := resid(qpNext), resid(clNext); !(rq < rc) {
  		t.Fatalf("task residual: qp %g, dls+clamps %g; want qp smaller", rq, rc)
  	}
  }
  ```
  Imports: `"slices"`, `"gonum.org/v1/gonum/mat"`, `"github.com/golang/geo/r3"`, `"go.viam.com/rdk/referenceframe"`, `"go.viam.com/rdk/spatialmath"`.

- [ ] **Step 3: Write the posture tests** in `loop_test.go`:
  ```go
  var pandaHold = []float64{0, -0.3, 0, -2.0, 0, 1.8, 0.785}

  // spec "Tests → Posture": q₀ on the self-motion manifold is reached while the pose holds.
  func TestPostureReachesQ0(t *testing.T) {
  	m := panda(t)
  	q0, err := SelfMotion(m, pandaHold, 0, deg2rad(20))
  	if err != nil {
  		t.Fatal(err)
  	}
  	q0deg := make([]float64, len(q0))
  	for i, v := range q0 {
  		q0deg[i] = rad2deg(v)
  	}
  	sim := &simIO{q: slices.Clone(pandaHold)}
  	l, err := NewLoop(m, sim, LoopConfig{Solver: SolverQP, Posture: &Posture{JointsDeg: q0deg}})
  	if err != nil {
  		t.Fatal(err)
  	}
  	l.SetTarget(offsetTarget(t, m, pandaHold, r3.Vector{}, 0))
  	worst := 0.0
  	for i := 0; i < 150; i++ { // 3 s at 50 Hz
  		runTicks(t, l, 1)
  		worst = math.Max(worst, l.Status().ErrMM)
  	}
  	if worst >= 0.5 {
  		t.Errorf("TCP error reached %.3f mm, want < 0.5", worst)
  	}
  	if d := rad2deg(maxAbsDiff(sim.q, q0)); d > 2 {
  		t.Errorf("joints %.2f° from q0 after 3 s, want ≤ 2", d)
  	}
  }

  // spec "Tests → The loop": a 20° posture error must not keep a ramp-stop going (≤ N + 1 ticks).
  func TestPostureDoesNotDelayRampStop(t *testing.T) {
  	m := panda(t)
  	q0, err := SelfMotion(m, pandaHold, 0, deg2rad(20))
  	if err != nil {
  		t.Fatal(err)
  	}
  	ticks := func(p *Posture) int {
  		l, _, _, clk := newClockLoopModel(t, m, pandaHold, LoopConfig{Solver: SolverQP, Posture: p})
  		l.SetTarget(offsetTarget(t, m, pandaHold, r3.Vector{}, 0))
  		l.SetVelocity([6]float64{0.03})
  		clockTicks(t, l, clk, 25)
  		l.RampStop(200*time.Millisecond, StateStopped)
  		for n := 1; n <= 100; n++ {
  			if clockTicks(t, l, clk, 1) {
  				return n
  			}
  		}
  		t.Fatal("ramp did not complete in 100 ticks")
  		return 0
  	}
  	deg := make([]float64, len(q0))
  	for i, v := range q0 {
  		deg[i] = rad2deg(v)
  	}
  	if a, b := ticks(nil), ticks(&Posture{JointsDeg: deg}); b > a+1 {
  		t.Fatalf("ramp took %d ticks with posture, %d without", b, a)
  	}
  }
  ```
  Look at `newClockLoop` and `clockTicks` in `loop_test.go` (`grep -n 'func newClockLoop\|func clockTicks' loop_test.go`). If `newClockLoop` hard-codes `ur5e`, add a `newClockLoopModel(t, m, q0, cfg)` variant and make `newClockLoop` call it. Check what `clockTicks` returns (true when the ramp completed) and adapt the loop above to it.
  - If `TestPostureReachesQ0` misses either bar with the default weight 1e-3 and gain 1, tune the two defaults in `withDefaults` and the spec's table (also update `TestSolverAndPostureDefaults`), and record the values in the commit message. If nothing meets both bars, stop and report.

- [ ] **Step 4: Run the acceleration, ramp-stop and dead-man tests on both solvers.**
  - Wrap the bodies of `TestAccelerationLimit`, `TestRampStopWaitsForRest` and `TestRampStopScalesToZeroThenStops` in:
    ```go
    for _, solver := range []string{SolverDLS, SolverQP} {
    	t.Run(solver, func(t *testing.T) { /* the existing body, with Solver: solver added to every LoopConfig */ })
    }
    ```
    `TestAccelerationLimit`'s braking check uses the continuous cap, which the QP's discrete cap is tighter than, so it holds for both.
  - Do the same for the session dead-man test. Find it with `grep -n 'StateStoppedTimeout\|stopped_timeout' session_test.go`, and pass the solver through `SessionConfig.Loop.Solver`.
  - In `TestAccelerationLimit`, add a check for both solvers that `l.Status().PeakAccelRatio <= 1+1e-9` after the first run.

- [ ] **Step 5: Add the QP golden.** In `golden_test.go`'s `goldenScenarios`, before `return res`:
  ```go
  	sm := so101(t)
  	qs := []float64{0.2, -0.4, 0.6, 0.9, 0.1}
  	ssim := &simIO{q: slices.Clone(qs)}
  	sl, err := NewLoop(sm, ssim, LoopConfig{Solver: SolverQP, MaxJointDegPerS2: repeat(1e9, 5)})
  	if err != nil {
  		t.Fatal(err)
  	}
  	sl.SetTarget(offsetTarget(t, sm, qs, r3.Vector{X: 20, Y: -15, Z: 10}, 0))
  	runTicks(t, sl, 150)
  	res["qp_so101_static"] = bits(ssim.sends)
  ```
  Run `go test -run TestGoldenUR5ePoseCommands -update-golden ./`, then `git diff --stat testdata/`. Expected: only `golden_ur5e_arm64.json` changes. Then run `git diff testdata/ | grep '^-' | grep -v '^---'` and check it prints nothing (no existing key changed). If an existing key changed, the DLS path is no longer bit-exact: revert and find the cause.
  - The `amd64` golden file doesn't exist on this machine and isn't generated here.

- [ ] **Step 6: Run everything.** Run `go test ./...`. Expected: PASS.

- [ ] **Step 7: Commit** with message `"QP: step equivalence, posture, both-solver loop tests, QP golden"`.

---

### Task 8: Module: `solver`, `posture`, status fields

**Files:**
- Modify: `module.go` (`startSession`, `statusMap`)
- Test: `module_test.go`

- [ ] **Step 1: Write the failing tests.** Find the bad-config table in `module_test.go` (`grep -n 'joint_deg_s2' module_test.go`) and add these cases in its style (the cmd map, then the expected error substring):
  - `{"solver": "osqp"}` → `unsupported solver`
  - `{"posture": map[string]interface{}{}}` (no solver) → `posture needs "solver": "qp"`
  - `{"solver": "qp", "posture": "x"}` → `posture must be an object`
  - `{"solver": "qp", "posture": map[string]interface{}{"weight": 0.0}}` → `posture.weight must be > 0`
  - `{"solver": "qp", "posture": map[string]interface{}{"gain": -1.0}}` → `posture.gain must be > 0`
  - `{"solver": "qp", "posture": map[string]interface{}{"joints_deg": []interface{}{1.0}}}` → `posture.joints_deg needs`

  Then add a status test that starts a `"solver": "qp"` session the way the existing status tests do (`grep -n 'servo_status' module_test.go`), and asserts:
  - `out["solver"] == "qp"`;
  - `q_cmd_deg` is a `[]interface{}` with one entry per joint;
  - `peak_accel_ratio` is present;
  - `qp_active`, `qp_iters` and `qp_fallbacks` are present.

  For a default session, it asserts `solver == "dls"` and no `qp_*` keys.

- [ ] **Step 2: Run them and check they fail.** Run `go test -run TestModule ./` (or whatever the module test names are). Expected: FAIL.

- [ ] **Step 3: Implement it.** In `startSession`, after the `task_weights` block:
  ```go
  	solver, err := optString(cmd, "solver", SolverDLS)
  	if err != nil {
  		return nil, nil, err
  	}
  	if solver != SolverDLS && solver != SolverQP {
  		return nil, nil, fmt.Errorf("unsupported solver %q: use %q or %q", solver, SolverDLS, SolverQP)
  	}
  	lc.Solver = solver
  	if raw, ok := cmd["posture"]; ok {
  		if solver != SolverQP {
  			return nil, nil, errors.New(`posture needs "solver": "qp"`)
  		}
  		pm, ok := raw.(map[string]interface{})
  		if !ok {
  			return nil, nil, errors.New("posture must be an object")
  		}
  		p := &Posture{Weight: 1e-3, Gain: 1}
  		if v, ok := pm["joints_deg"]; ok {
  			items, ok := v.([]interface{})
  			if !ok {
  				return nil, nil, errors.New("posture.joints_deg must be a list of numbers")
  			}
  			for _, it := range items {
  				n, err := num(it, "posture.joints_deg")
  				if err != nil {
  					return nil, nil, err
  				}
  				p.JointsDeg = append(p.JointsDeg, n)
  			}
  		}
  		for _, f := range []struct {
  			key string
  			dst *float64
  		}{{"weight", &p.Weight}, {"gain", &p.Gain}} {
  			if v, ok := pm[f.key]; ok {
  				n, err := num(v, "posture."+f.key)
  				if err != nil {
  					return nil, nil, err
  				}
  				if !(n > 0) {
  					return nil, nil, fmt.Errorf("posture.%s must be > 0, got %v", f.key, n)
  				}
  				*f.dst = n
  			}
  		}
  		lc.Posture = p
  	}
  ```
  The defaults here must match `withDefaults` (and any values Task 7 tuned). The joint count is checked by `LoopConfig.validate` when the session starts.

  In `statusMap`, before `return out`:
  ```go
  	out["solver"] = st.Config.Solver
  	qc := make([]interface{}, len(st.QCmd))
  	for i, v := range st.QCmd {
  		qc[i] = rad2deg(v)
  	}
  	out["q_cmd_deg"] = qc
  	out["peak_accel_ratio"] = st.PeakAccelRatio
  	if st.Config.Solver == SolverQP {
  		out["qp_active"], out["qp_iters"], out["qp_fallbacks"] = float64(st.QPActive), float64(st.QPIters), float64(st.QPFallbacks)
  	}
  ```
  The teleop defaults block goes through `startSession`, so `solver` and `posture` work there with no further change.

- [ ] **Step 4: Run the tests.** Run `go test ./...`. Expected: PASS.

- [ ] **Step 5: Commit** with message `"Module: solver and posture on servo_start; qp and acceleration status fields"`.

---

### Task 9: Clients and CLI

**Files:**
- Modify: `clients/python/servo_client.py`, `clients/python/test_servo_client.py`, `clients/python/servo_bench.py`, `cmd/cli/main.go`, `cmd/cli/run.go`

- [ ] **Step 1: `servo_client.py`.**
  - Append `"peak_accel_ratio"` to `FOLLOW_COLS`.
  - In `follow_row`, the pose row gets one more `""` (six blanks), and the status row appends `st.get("peak_accel_ratio")`.
  - Add `ap.add_argument("--solver", choices=["dls", "qp"], default=None, help="servo_start solver (default: the server's, dls)")` next to `--joint-deg-s`.
  - In the `servo_start` dict built around line 164, add `"solver": a.solver` only when `a.solver` is set.

  Update `test_servo_client.py`'s `follow_row` expectations to the new column count, then run `uv run clients/python/test_servo_client.py` and `uv run clients/python/test_tag_sweep.py`. Expected: ok.

- [ ] **Step 2: `servo_bench.py`.** Add the same `--solver` argument, and in `start_kw` set `base["solver"] = args.solver` when it's set.

- [ ] **Step 3: CLI flags and options.** In `cmd/cli/main.go`, add:
  ```go
  	solver := flag.String("solver", "", "run: dls|qp (default dls)")
  	postureJoint := flag.Int("posture-joint", 0, "run: joint index the posture offset moves along the self-motion (7-DoF arms)")
  	postureDeg := flag.Float64("posture-deg", 0, "run: with --solver qp, posture q0 = the start joints moved this many degrees on --posture-joint along the self-motion (0 = q0 is the start joints)")
  	postureW := flag.Float64("posture-weight", 0, "run: posture weight (0 = no posture task)")
  	postureK := flag.Float64("posture-gain", 0, "run: posture gain 1/s (0 = default 1)")
  	orbitDegS := flag.Float64("orbit-deg-s", 8, "orbit: rate the target pose rotates about base z, deg/s")
  	orbitDeg := flag.Float64("orbit-deg", 320, "orbit: total rotation, deg")
  	maxDegS2 := flag.Float64("max-joint-deg-s2", 0, "run: per-joint acceleration limit in deg/s² (0 = the model's, else 360)")
  ```
  - Accept `run hold` and `run orbit` in `validCmd`.
  - Pass the new values into `runOpts`: add the fields `solver string`, `postureJoint int`, `postureDeg, postureW, postureK float64`, `orbitDegS, orbitDeg float64`, `maxJointDegS2 float64`. Reject `--orbit-deg-s` ≤ 0 and `--max-joint-deg-s2` < 0.
  - In `run`, fill `cfg.MaxJointDegPerS2` from `o.maxJointDegS2` the same way `MaxJointDegPerS` is filled.
  - Update `usage` to list `hold` and `orbit`.

- [ ] **Step 4: Wire `run.go`.**
  - After `cfg := diffik.LoopConfig{...}`, set `cfg.Solver = o.solver`.
  - If `o.postureW > 0 || o.postureDeg != 0`:
    - read `q, _ := a.JointPositions(ctx, nil)`;
    - if `o.postureDeg != 0`, set `q, err = diffik.SelfMotion(model, q, o.postureJoint, o.postureDeg*math.Pi/180)`;
    - set `cfg.Posture = &diffik.Posture{JointsDeg: degrees(q), Weight: o.postureW, Gain: o.postureK}`, where a zero weight or gain means the default.
  - Add two scenarios:
    - **`hold`:** `l.SetTarget(start)`, wait `o.hold` (default 3 s if 0), then print:
      - `hold: worst err %.2f mm`, over all recorded rows;
      - `posture: max joint distance to q0 %.2f°`, from the final `QCmd`, when `cfg.Posture != nil`.
    - **`orbit`:** rotate the target about the base z axis: at elapsed time τ the target is `start` rotated by θ = `o.orbitDegS`·τ about the base z axis. Both the position (x, y rotated about the origin, z kept) and the orientation (left-multiplied by Rz(θ)) rotate. Call `SetTarget` each tick, as `circle` does, until θ reaches `o.orbitDeg`, then hold 1 s. Don't wait for convergence first: the target starts at the current pose. Print:
      - the mean and max `ErrMM` over the orbit;
      - the mean `ErrMM` over the rows where `Limited` is true;
      - per joint, the minimum margin to its limits over all rows' `QCmd`, in degrees (from `model.DoF()`), as `joint %d: min margin %.2f°`.

    Both reuse `rec.since(from)`, the way `circle` does.
  - In the summary, after `overruns`, print `peak accel ratio: %.3f   qp fallbacks: %d` from `s.PeakAccelRatio` and `s.QPFallbacks`.

- [ ] **Step 5: Build and test.** Run `go build ./... && go test ./...`. Expected: PASS. Run `go run ./cmd/cli -h 2>&1 | grep -E 'solver|posture|line'`. Expected: the new flags are listed.

- [ ] **Step 6: Commit** with message `"Clients and CLI: --solver, posture flags, run hold/orbit, peak accel ratio"`.

---

### Task 10: Docs

**Files:**
- Modify: `README.md`, `devrel_diff-ik_servo.md`
- Create: `dev/panda-mujoco.md`

- [ ] **Step 1: `devrel_diff-ik_servo.md`.**
  - In the `servo_start` field table, add rows for `solver` (`dls`; `qp` adds hard joint position, velocity and acceleration bounds and the posture task) and `posture` (`joints_deg`, `weight`, `gain`, with the defaults from Task 7; QP only; fixed for the session).
  - Under "Status fields", add `q_cmd_deg`, `peak_accel_ratio` and, for `qp`, `qp_active`, `qp_iters` and `qp_fallbacks`.
  - Add one paragraph on the QP: weighted tasks, the bounds, and the per-tick fallback with a fault after 10 in a row.

- [ ] **Step 2: `README.md`.** Add a short "Solvers" section after the model list: `solver: "qp"` is opt-in and experimental, with links to the spec and to the results doc.

- [ ] **Step 3: `dev/panda-mujoco.md`.** Write the runbook covering:
  - prerequisites (`~/src/viam-mujoco-sim` on `feat/custom-assets`);
  - starting the cell (`viam-server -config dev/panda-mujoco.json`, port 8083);
  - `go run ./dev/exportmodel` to refresh `testdata/panda.json`;
  - the CLI commands for the Panda A/B runs (Task 11).

- [ ] **Step 4: Commit** with message `"Docs: solver and posture"`.

---

### Task 11: Sim A/B evidence

The controller does this task with the sims running. Run each row for both `--solver dls` and `--solver qp`, and collect the printed summaries into the results doc. Stop the SO-101 cells before starting the Panda cell, and the reverse, so only one MuJoCo process renders at a time.

- [ ] **Step 1: SO-101 static and circle** (cell `dev/so101-mujoco.json`, port 8082, using the Phase 3b commands from `README.md`). For each solver:
  ```
  go run ./cmd/cli --address localhost:8082 --insecure --arm arm --task pointing --solver <s> run static
  go run ./cmd/cli --address localhost:8082 --insecure --arm arm --task pointing --solver <s> --csv .runs/qp-so101-circle-<s>.csv run circle
  ```
  Bar for `qp`: circle mean error within 10% of DLS, and 0 overruns.

- [ ] **Step 2: Panda joint-limit orbit** (`dev/panda-mujoco.json`, port 8083), the path recorded in Task 1. Every Panda run passes the arm's limits, `--max-joint-deg-s 30 --max-joint-deg-s2 60`:
  ```
  go run ./cmd/cli --address localhost:8083 --insecure --arm pick-arm --solver <s> --max-joint-deg-s 30 --max-joint-deg-s2 60 --start-joints 0,0,0,-1.5708,0,1.5708,-0.785 --orbit-deg-s 8 --orbit-deg 320 --csv .runs/qp-panda-orbit-<s>.csv run orbit
  ```
  Bars for `qp`:
  - lower "mean error while limited" than DLS;
  - joint 0's minimum margin ≥ 0;
  - `peak accel ratio` ≤ 1.

  DLS's ratio is reported only.

- [ ] **Step 3: Panda posture.** Run once with q₀ at the start joints, then once with q₀ moved 20°:
  ```
  go run ./cmd/cli --address localhost:8083 --insecure --arm pick-arm --solver qp --max-joint-deg-s 30 --max-joint-deg-s2 60 --start-joints 0,-0.3,0,-2.0,0,1.8,0.785 --posture-weight 0.001 --hold 3s run hold
  go run ./cmd/cli --address localhost:8083 --insecure --arm pick-arm --solver qp --max-joint-deg-s 30 --max-joint-deg-s2 60 --start-joints 0,-0.3,0,-2.0,0,1.8,0.785 --posture-weight 0.001 --posture-deg 20 --hold 3s run hold
  ```
  Use the tuned weight from Task 7 if it changed. Bar for the second run: max joint distance to q₀ ≤ 2°, and worst error < 1 mm.
  - The arm's own limit of 30°/s caps how fast the posture can move. If the bar fails only because 3 s is too short at that speed, report the time it took. Don't relax the bar silently.

- [ ] **Step 4: Panda circle**, both solvers: `go run ./cmd/cli --address localhost:8083 --insecure --arm pick-arm --solver <s> --max-joint-deg-s 30 --max-joint-deg-s2 60 --start-joints 0,0,0,-1.5708,0,1.5708,-0.785 run circle`. Reported only.

- [ ] **Step 5: Moving-tag sweep, wrist camera** (cell `dev/so101-mujoco-tag.json`, port 8082; pre-position as in `dev/so101-mujoco.md`):
  ```
  uv run clients/python/tag_sweep.py --insecure --camera wrist-cam --tracker wrist-tags --solver <s> --csv .runs/qp-tag-<s>
  uv run clients/python/tag_sweep_report.py .runs/qp-tag-dls .runs/qp-tag-qp
  ```
  Bar for `qp`: mean error at 20 mm/s within 10% of DLS.

- [ ] **Step 6: Write `docs/results/2026-11-qp-solver.md`.** Below the Task 0 section, add:
  - the setup;
  - the A/B table, with one row per run and DLS and QP side by side, including the `peak accel ratio` for each;
  - pass or fail for each bar;
  - findings;
  - a recommendation on whether to make `qp` the default.

  Report failures as failures.

- [ ] **Step 7: Commit** with message `"QP solver results: sim A/B"`.

---

### Task 12: Finish

- [ ] **Step 1:** Run `go test ./...`, every `clients/python/test_*.py` with `uv run`, and `go vet ./...`. Expected: all pass.
- [ ] **Step 2:** Dispatch a final Opus code review of `git diff main...diff-ik-qp` against the spec. Fix what it finds and re-review.
- [ ] **Step 3:** Squash-merge:
  ```
  git switch main && git merge --squash diff-ik-qp && git commit
  ```
  Use the message `"Diff-IK QP solver and posture task (opt-in solver: qp)"` with the trailer. Keep the branch. Don't push.
- [ ] **Step 4:** Report to the user:
  - the A/B table and findings;
  - the recommendation;
  - that `main` is unpushed;
  - the next spec (collision avoidance).
