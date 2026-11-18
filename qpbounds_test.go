package diffik

import (
	"math"
	"math/rand"
	"slices"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"gonum.org/v1/gonum/mat"
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

// The empty-interval fallback keeps the braking cap: a joint moving faster than the cap must not
// drive into the limit at speed.
func TestQPBoundsFallbackKeepsBraking(t *testing.T) {
	l, _, _ := newTestLoop(t, home, LoopConfig{Solver: SolverQP})
	runTicks(t, l, 1)
	dt := 1 / l.cfg.RateHz
	a := deg2rad(l.cfg.MaxJointDegPerS2[0])
	l.qCmd[0] = l.hi[0] - 0.05
	l.lastStep[0] = 1.3 * brakeCap(a, 0.05, dt)
	_, ub := l.qpBounds(dt)
	if cap := brakeCap(a, 0.05, dt); ub[0] > cap+1e-15 {
		t.Fatalf("ub[0] = %v exceeds braking cap %v", ub[0], cap)
	}
}

// qpNext must not hand a NaN solution to the arm; the tick then falls back to DLS.
func TestQPNextRejectsNonFinite(t *testing.T) {
	l, _, _ := newTestLoop(t, home, LoopConfig{Solver: SolverQP})
	runTicks(t, l, 1)
	rows := taskRows{A: mat.NewDense(1, 6, []float64{1, 0, 0, 0, 0, 0}), b: []float64{math.NaN()}}
	next, _, _, err := l.qpNext(rows, 1, 1/l.cfg.RateHz)
	if err == nil || next != nil {
		t.Fatalf("next %v err %v, want nil and an error", next, err)
	}
}

// qpVsDLS returns the QP step and the DLS (pre-clamp) step for loop l at its current state. With
// keepLast false it first runs a tick and sets lastStep to the DLS step, making the acceleration bound
// slack (spec "Tests → The step"); with keepLast true it leaves lastStep as the caller set it.
func qpVsDLS(t *testing.T, l *Loop, target spatialmath.Pose, keepLast bool) (qp, dls []float64, lambda float64) {
	t.Helper()
	dt := 1 / l.cfg.RateHz
	if !keepLast {
		runTicks(t, l, 1) // qCmd set, target stored
	}
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
	if !keepLast {
		copy(l.lastStep, dq)
	}
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
			qp, dls, lambda := qpVsDLS(t, l, offsetTarget(t, m, c.q, r3.Vector{X: 1, Y: -1, Z: 0.5}, deg2rad(0.5)), false)
			if lambda != 0 {
				t.Fatalf("precondition: λ = %v, want a well-conditioned configuration", lambda)
			}
			t.Logf("max |qp-dls| = %.3g", maxAbsDiff(qp, dls))
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
	qp, dls, lambda := qpVsDLS(t, l, offsetTarget(t, m, q, r3.Vector{X: 1, Z: -1}, deg2rad(0.5)), false)
	if lambda != 0 {
		t.Fatalf("precondition: λ = %v, want a well-conditioned configuration (pick another q)", lambda)
	}
	// The wrist roll (index 4) sits almost in the null space of the 5 pointing rows: dpinv's rank cut drops
	// it, the QP doesn't. Compare the other joints, and the task-space motion J·Δq (review finding).
	// μ = 1e-6 shrinks the 0.01-scaled axis rows' step by about μ/σ² ≈ 0.5%: a bias the gain loop absorbs.
	// The position rows carry the same μ bias, about 0.1% of a 4e-5 m step (3.6e-8), so they match to 1e-7;
	// the joints and angular rows to 5e-6.
	nearVec(t, qp[:4], dls[:4], 5e-6)
	J, err := Jacobian(m, l.qCmd)
	if err != nil {
		t.Fatal(err)
	}
	var a, b mat.VecDense
	a.MulVec(J, mat.NewVecDense(5, qp))
	b.MulVec(J, mat.NewVecDense(5, dls))
	t.Logf("joints0-3 max diff %.3g; pos rows %.3g; ang rows %.3g", maxAbsDiff(qp[:4], dls[:4]),
		maxAbsDiff(a.RawVector().Data[:3], b.RawVector().Data[:3]), maxAbsDiff(a.RawVector().Data[3:], b.RawVector().Data[3:]))
	nearVec(t, a.RawVector().Data[:3], b.RawVector().Data[:3], 1e-7)
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
		qpStep, _, _ := qpVsDLS(t, l, target, true)
		for i := range qpStep {
			if qpStep[i] < lb[i]-1e-12 || qpStep[i] > ub[i]+1e-12 {
				t.Fatalf("trial %d joint %d: step %v outside [%v, %v]", trial, i, qpStep[i], lb[i], ub[i])
			}
		}
	}
}

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
	rq, rc := resid(qpNext), resid(clNext)
	t.Logf("task residual: qp %g, dls+clamps %g", rq, rc)
	if !(rq < rc) {
		t.Fatalf("task residual: qp %g, dls+clamps %g; want qp smaller", rq, rc)
	}
}
