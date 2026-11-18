package diffik

import (
	"errors"
	"math"
	"slices"
	"strings"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

type pandaRig struct {
	l   *Loop
	sim *simIO
	m   referenceframe.Model
	c   *collisionModel
	ee  spatialmath.Pose
	fp  r3.Vector
}

// newPandaRig is a QP loop on the Panda at q0 with obstacle g posed at pose (arm base frame).
func newPandaRig(t *testing.T, req CollisionRequest, js, js2, lin float64, pose spatialmath.Pose, g spatialmath.Geometry, q0 []float64) *pandaRig {
	t.Helper()
	m := panda(t)
	cfg := LoopConfig{RateHz: 50, Solver: SolverQP, MaxLinearMMPerS: lin,
		MaxJointDegPerS: repeat(js, 7), MaxJointDegPerS2: repeat(js2, 7)}
	sim := &simIO{q: slices.Clone(q0)}
	l, err := NewLoop(m, sim, cfg)
	if err != nil {
		t.Fatal(err)
	}
	c := buildColl(t, m, req, q0, staticPart(referenceframe.World, "obs", pose, g))
	if err := l.SetCollision(c); err != nil {
		t.Fatal(err)
	}
	ee := mustPose(m.Transform(q0))
	l.SetTarget(ee)
	return &pandaRig{l, sim, m, c, ee, ee.Point()}
}

func (r *pandaRig) at(x, y, z float64) spatialmath.Pose {
	return spatialmath.NewPose(r3.Vector{X: x, Y: y, Z: z}, r.ee.Orientation())
}

func (r *pandaRig) tick(t *testing.T) {
	t.Helper()
	if err := r.l.tick(t.Context(), 0.02); err != nil {
		t.Fatal(err)
	}
}

// boxTop is a w×h×thick box centred under the flange, its top gap mm below it.
func (r *pandaRig) boxTop(t *testing.T, gap, thick float64) (spatialmath.Pose, spatialmath.Geometry) {
	return spatialmath.NewPoseFromPoint(r3.Vector{X: r.fp.X, Y: r.fp.Y, Z: r.fp.Z - gap - thick/2}), mustBox(t, r3.Vector{X: 300, Y: 300, Z: thick})
}

func TestCollisionLoopApproach(t *testing.T) {
	m := panda(t)
	fp := eePoint(t, m, pandaHold)
	pose := spatialmath.NewPoseFromPoint(r3.Vector{X: fp.X, Y: fp.Y, Z: fp.Z - 60 - 50})
	r := newPandaRig(t, CollisionRequest{}, 30, 60, 0, pose, mustBox(t, r3.Vector{X: 300, Y: 300, Z: 100}), pandaHold)
	runTicks(t, r.l, 5)
	r.l.SetTarget(r.at(r.fp.X, r.fp.Y, r.fp.Z-110))
	minD := math.Inf(1)
	for i := 0; i < 400; i++ {
		r.tick(t)
		st := r.l.Status()
		minD = math.Min(minD, st.CollisionMinMM)
		if st.CollisionMinMM < r.c.dMin-1 || st.PeakAccelRatio > 1+1e-9 {
			t.Fatalf("tick %d: min %.2f peak %.4f", i, st.CollisionMinMM, st.PeakAccelRatio)
		}
	}
	st := r.l.Status()
	t.Logf("min %.2f state %s brakes %d relaxed %d err %.1f", minD, st.State, st.CollisionBrakes, st.CollisionRelaxed, st.ErrMM)
	if st.State != StateStalled || st.CollisionBrakes != 0 {
		t.Fatalf("state %s brakes %d", st.State, st.CollisionBrakes)
	}
}

func TestCollisionLoopFastApproach(t *testing.T) {
	m := panda(t)
	fp := eePoint(t, m, pandaHold)
	pose := spatialmath.NewPoseFromPoint(r3.Vector{X: fp.X, Y: fp.Y, Z: fp.Z - 60 - 50})
	r := newPandaRig(t, CollisionRequest{}, 90, 360, 100, pose, mustBox(t, r3.Vector{X: 300, Y: 300, Z: 100}), pandaHold)
	runTicks(t, r.l, 5)
	r.l.SetVelocity([6]float64{0, 0, -0.1})
	minD := math.Inf(1)
	for i := 0; i < 400; i++ {
		if i >= 100 { // 2 mm per tick: 200 mm
			r.l.SetVelocity([6]float64{})
		}
		r.tick(t)
		minD = math.Min(minD, r.l.Status().CollisionMinMM)
	}
	st := r.l.Status()
	t.Logf("min %.2f brakes %d relaxed %d peak %.3f", minD, st.CollisionBrakes, st.CollisionRelaxed, st.PeakAccelRatio)
	if minD < r.c.dMin-1 || st.CollisionBrakes != 0 {
		t.Fatalf("min %.2f brakes %d", minD, st.CollisionBrakes)
	}
}

func TestCollisionLoopRelaxedRowBinds(t *testing.T) {
	m := panda(t)
	fp := eePoint(t, m, pandaHold)
	// The sphere sits 30 mm below the flange point: the attachment site is 20 mm from its surface. link7 and
	// link6 are only 64-67 mm farther, so d_act = 50 mm leaves one row (at the default 100 mm they join in
	// and the relaxed rows conflict).
	pose := spatialmath.NewPoseFromPoint(r3.Vector{X: fp.X, Y: fp.Y, Z: fp.Z - 30})
	r := newPandaRig(t, CollisionRequest{DActMM: 50}, 30, 60, 0, pose, mustSphere(t, 10), pandaHold)
	l, dt := r.l, 0.02
	r.l.SetTarget(r.at(fp.X, fp.Y, fp.Z-100))
	runTicks(t, l, 1)
	copy(l.qCmd, pandaHold)
	acc := repeat(deg2rad(60), 7)
	rows, _, _, err := r.c.rows(l.qCmd, acc, dt)
	if err != nil || len(rows) != 1 {
		t.Fatalf("rows %d err %v", len(rows), err)
	}
	row := rows[0]
	t.Logf("row d %.2f vmax %.3f", row.d, row.vmax)
	n2 := dotf(row.jd, row.jd)
	for i := range l.lastStep {
		l.lastStep[i] = -3 * row.vmax * row.jd[i] / n2 // approaching at 3 v_max
	}
	// the late-approach warning: 3 v_max is late; 0.5 v_max and v_max + 0.005 mm are not
	for _, tc := range []struct {
		k    float64
		late bool
	}{{3, true}, {0.5, false}, {1 + 0.005/row.vmax, false}} {
		for i := range l.lastStep {
			l.lastStep[i] = -tc.k * row.vmax * row.jd[i] / n2
		}
		saved, savedQ := slices.Clone(l.lastStep), slices.Clone(l.qCmd)
		r.tick(t)
		late := l.Status().CollisionLate
		if (late != "") != tc.late || (tc.late && !strings.Contains(late, "obs")) {
			t.Fatalf("%gx v_max: CollisionLate %q", tc.k, late)
		}
		copy(l.lastStep, saved)
		copy(l.qCmd, savedQ)
		copy(r.sim.q, savedQ)
		l.status.CollisionLate = ""
	}
	for i := range l.lastStep {
		l.lastStep[i] = -3 * row.vmax * row.jd[i] / n2
	}
	lb, ub := l.qpBounds(dt)
	M := relaxBound(row.jd, lb, ub)
	if !(M < -row.vmax) {
		t.Fatalf("precondition: M %.4f not below -v_max %.4f", M, -row.vmax)
	}
	cp, cq, _ := FK(r.m, l.qCmd)
	J, _ := Jacobian(r.m, l.qCmd)
	tp, tq := l.target.Point().Mul(1/mmPerM), l.target.Orientation().Quaternion()
	_, _, _, _, _, trows, err := l.solve(J, tp, tq, cp, cq, [6]float64{}, 1, dt)
	if err != nil {
		t.Fatal(err)
	}
	next, _, _, err := l.qpNext(trows, 1, dt, rows, true)
	if err != nil {
		t.Fatalf("relaxed solve: %v", err)
	}
	step := make([]float64, 7)
	for i := range step {
		step[i] = next[i] - l.qCmd[i]
	}
	if s := dotf(row.jd, step); s < M-2*relaxEps || math.Abs(s-M) > 1e-5 {
		t.Fatalf("Jd·dq %.6f, want within 1e-5 of M %.6f", s, M)
	}
}

func TestCollisionLoopBrake(t *testing.T) {
	m := panda(t)
	fp := eePoint(t, m, pandaHold)
	pose := spatialmath.NewPoseFromPoint(r3.Vector{X: fp.X, Y: fp.Y, Z: fp.Z - 60 - 50})
	r := newPandaRig(t, CollisionRequest{}, 30, 60, 0, pose, mustBox(t, r3.Vector{X: 300, Y: 300, Z: 100}), pandaHold)
	l := r.l
	r.l.SetTarget(r.at(fp.X, fp.Y, fp.Z-110))
	runTicks(t, l, 10)
	for i := range l.lastStep { // moving at a fair clip, so braking takes more than 10 ticks
		l.lastStep[i] = 0.01
	}
	old := qpSolver
	defer func() { qpSolver = old }()
	qpSolver = func(qpProblem) ([]float64, int, int, error) { return nil, 0, 0, errors.New("forced") }
	a := deg2rad(60) * 0.02 * 0.02
	n := 0
	for ; maxAbs(l.lastStep) > 0; n++ {
		prev := slices.Clone(l.lastStep)
		r.tick(t) // a fault would fail the test
		for i, s := range prev {
			if want := math.Abs(s) - math.Min(math.Abs(s), a); math.Abs(math.Abs(l.lastStep[i])-want) > 1e-12 {
				t.Fatalf("tick %d joint %d: |step| %v, want %v", n, i, l.lastStep[i], want)
			}
		}
	}
	if n < 20 {
		t.Fatalf("only %d brake ticks", n)
	}
	st := l.Status()
	if st.CollisionBrakes != n || st.QPFallbacks != n {
		t.Fatalf("brakes %d fallbacks %d, want %d", st.CollisionBrakes, st.QPFallbacks, n)
	}
	for i := 0; i < 9; i++ {
		r.tick(t)
	}
	if err := l.tick(t.Context(), 0.02); err == nil || !strings.HasPrefix(err.Error(), "qp:") {
		t.Fatalf("10th failure at rest: %v", err)
	}
	t.Logf("%d brake ticks before rest", n)
}

func slideRun(t *testing.T, stopRef bool) (meanErr, minD float64) {
	m := panda(t)
	fp := eePoint(t, m, pandaHold)
	ori := &spatialmath.OrientationVectorDegrees{OX: 0.15, OY: 0.1, OZ: 1, Theta: 17}
	pose := spatialmath.NewPose(r3.Vector{X: fp.X + 100, Y: fp.Y, Z: fp.Z - 90 - 50}, ori)
	r := newPandaRig(t, CollisionRequest{}, 30, 60, 0, pose, mustBox(t, r3.Vector{X: 400, Y: 300, Z: 100}), pandaHold)
	if len(r.c.ignored()) != 0 {
		t.Fatalf("pairs suspended at start: %v", r.c.ignored())
	}
	runTicks(t, r.l, 2)
	minD = math.Inf(1)
	frozen := false
	var fx, fz float64
	sum := 0.0
	for i := 0; i < 400; i++ {
		s := float64(i+1) / 400
		x, z := fp.X+150*s, fp.Z-120*s
		if stopRef {
			if !frozen && r.l.Status().CollisionMinMM <= r.c.dMin+0.5 {
				frozen, fx, fz = true, x, z
			}
		}
		tx, tz := x, z
		if frozen {
			tx, tz = fx, fz
		}
		r.l.SetTarget(r.at(tx, fp.Y, tz))
		r.tick(t)
		cp, _, _ := FK(r.m, r.l.qCmd)
		sum += cp.Mul(mmPerM).Sub(r3.Vector{X: x, Y: fp.Y, Z: z}).Norm()
		minD = math.Min(minD, r.l.Status().CollisionMinMM)
	}
	return sum / 400, minD
}

func TestCollisionLoopSliding(t *testing.T) {
	slide, md := slideRun(t, false)
	stop, md2 := slideRun(t, true)
	t.Logf("sliding err %.2f mm (min d %.2f) vs stopping %.2f mm (min d %.2f)", slide, md, stop, md2)
	if !(slide < stop) || md < 10-1 {
		t.Fatalf("sliding %.2f stopping %.2f min %.2f", slide, stop, md)
	}
}

func TestCollisionLoopOffIsBitIdentical(t *testing.T) {
	run := func(withModel bool) [][]float64 {
		sm := so101(t)
		qs := []float64{0.2, -0.4, 0.6, 0.9, 0.1}
		sim := &simIO{q: slices.Clone(qs)}
		l, err := NewLoop(sm, sim, LoopConfig{Solver: SolverQP, MaxJointDegPerS2: repeat(1e9, 5)})
		if err != nil {
			t.Fatal(err)
		}
		if withModel {
			c := buildColl(t, sm, CollisionRequest{}, qs)
			if len(c.pairs) != 0 {
				t.Fatalf("%d pairs", len(c.pairs))
			}
			if err := l.SetCollision(c); err != nil {
				t.Fatal(err)
			}
		}
		l.SetTarget(offsetTarget(t, sm, qs, r3.Vector{X: 20, Y: -15, Z: 10}, 0))
		runTicks(t, l, 150)
		if st := l.Status(); withModel && st.Collision != "off" {
			t.Fatalf("collision %q", st.Collision)
		}
		return sim.sends
	}
	a, b := run(false), run(true)
	if len(a) != len(b) {
		t.Fatalf("%d vs %d sends", len(a), len(b))
	}
	for i := range a {
		for j := range a[i] {
			if math.Float64bits(a[i][j]) != math.Float64bits(b[i][j]) {
				t.Fatalf("send %d joint %d differs", i, j)
			}
		}
	}
}

func TestCollisionLoopRearms(t *testing.T) {
	m := panda(t)
	fp := eePoint(t, m, pandaHold)
	// the flange parts start 5 mm above the box: suspended
	pose := spatialmath.NewPoseFromPoint(r3.Vector{X: fp.X, Y: fp.Y, Z: fp.Z - 5 - 50})
	r := newPandaRig(t, CollisionRequest{}, 30, 60, 0, pose, mustBox(t, r3.Vector{X: 300, Y: 300, Z: 100}), pandaHold)
	if len(r.c.ignored()) == 0 {
		t.Fatal("nothing suspended at start")
	}
	r.l.SetTarget(r.at(fp.X, fp.Y, fp.Z+150))
	runTicks(t, r.l, 500)
	if ig := r.l.Status().CollisionIgnored; len(ig) != 0 {
		t.Fatalf("still ignored after 150 mm up: %v", ig)
	}
	r.l.SetTarget(r.at(fp.X, fp.Y, fp.Z))
	minD := math.Inf(1)
	for i := 0; i < 500; i++ {
		r.tick(t)
		minD = math.Min(minD, r.l.Status().CollisionMinMM)
	}
	t.Logf("min on the way down %.2f", minD)
	if minD < r.c.dMin-1 {
		t.Fatalf("min %.2f", minD)
	}
}

func TestSetCollisionNeedsQP(t *testing.T) {
	l, _, _ := newTestLoop(t, home, LoopConfig{Solver: SolverDLS})
	if err := l.SetCollision(&collisionModel{}); err == nil {
		t.Fatal("dls accepted collision")
	}
}

func TestSetCollisionSetsStatus(t *testing.T) {
	m := panda(t)
	// overlaps the base link: suspended at start
	post := staticPart(referenceframe.World, "post", spatialmath.NewPoseFromPoint(r3.Vector{Z: 50}), mustBox(t, r3.Vector{X: 100, Y: 100, Z: 100}))
	l, err := NewLoop(m, &simIO{q: slices.Clone(pandaHold)}, LoopConfig{Solver: SolverQP})
	if err != nil {
		t.Fatal(err)
	}
	c := buildColl(t, m, CollisionRequest{}, pandaHold, post)
	if err := l.SetCollision(c); err != nil {
		t.Fatal(err)
	}
	st := l.Status()
	if st.Collision != "on" || len(st.CollisionIgnored) == 0 || !slices.Equal(st.CollisionIgnored, c.ignored()) {
		t.Fatalf("status %q ignored %v", st.Collision, st.CollisionIgnored)
	}
	l2, _ := NewLoop(m, &simIO{q: slices.Clone(pandaHold)}, LoopConfig{Solver: SolverQP})
	if err := l2.SetCollision(buildColl(t, m, CollisionRequest{}, pandaHold)); err != nil {
		t.Fatal(err)
	}
	if got := l2.Status().Collision; got != "off" {
		t.Fatalf("no pairs: %q", got)
	}
}

func TestCollisionBrakeWhileMovingResetsQPFails(t *testing.T) {
	m := panda(t)
	fp := eePoint(t, m, pandaHold)
	pose := spatialmath.NewPoseFromPoint(r3.Vector{X: fp.X, Y: fp.Y, Z: fp.Z - 60 - 50})
	r := newPandaRig(t, CollisionRequest{}, 30, 60, 0, pose, mustBox(t, r3.Vector{X: 300, Y: 300, Z: 100}), pandaHold)
	old := qpSolver
	defer func() { qpSolver = old }()
	qpSolver = func(qpProblem) ([]float64, int, int, error) { return nil, 0, 0, errors.New("forced") }
	r.l.qpFails = 9
	for i := range r.l.lastStep {
		r.l.lastStep[i] = 0.01
	}
	r.tick(t) // braking while moving: must not trip the 10th failure
	if r.l.qpFails != 0 {
		t.Fatalf("qpFails = %d", r.l.qpFails)
	}
}
