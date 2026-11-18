package diffik

import (
	"math"
	"sort"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

// liveRig is a QP loop on the Panda with one far static sphere (so the model exists) and live obstacles set
// by hand. linMM is the session's linear speed, which sets the back-away speed.
func liveRig(t *testing.T, js, js2, linMM float64) *pandaRig {
	m := panda(t)
	fp := eePoint(t, m, pandaHold)
	far := spatialmath.NewPoseFromPoint(r3.Vector{X: fp.X, Y: fp.Y, Z: fp.Z - 1500})
	return newPandaRig(t, CollisionRequest{}, js, js2, linMM, far, mustSphere(t, 10), pandaHold)
}

// minDist is the closest pair distance at the loop's commanded joints.
func (r *pandaRig) minDist(t *testing.T) float64 {
	t.Helper()
	_, d, _, err := r.c.rows(r.l.qCmd, repeat(1, 7), 0.02, 1e9)
	if err != nil {
		t.Fatal(err)
	}
	return d
}

// slab is a 300 mm square, 100 mm thick live box whose face toward the flange point lies off mm from it
// along axis (0, 1, 2 = x, y, z) on the side sign.
func slab(t *testing.T, fp r3.Vector, axis int, sign, off float64) liveObstacle {
	c := []float64{fp.X, fp.Y, fp.Z}
	dims := []float64{300, 300, 300}
	dims[axis] = 100
	c[axis] += sign * (off + 50)
	return liveBoxAt(t, r3.Vector{X: c[0], Y: c[1], Z: c[2]}, r3.Vector{X: dims[0], Y: dims[1], Z: dims[2]})
}

// setSlabs places slabs so the closest part of the model at q is gap mm from each, one per (axis, sign).
// Each slab is moved until its own distance is gap; the others are present as 1 m away.
func (r *pandaRig) setSlabs(t *testing.T, q []float64, gap float64, axes []int, signs []float64) {
	t.Helper()
	fp := eePoint(t, r.m, q)
	offs := make([]float64, len(axes))
	for i := range offs {
		offs[i] = 40
	}
	build := func(i int, only bool) []liveObstacle {
		var out []liveObstacle
		for k := range axes {
			if only && k != i {
				continue
			}
			out = append(out, slab(t, fp, axes[k], signs[k], offs[k]))
		}
		return out
	}
	for i := range axes {
		for it := 0; it < 4; it++ {
			r.c.setLive(build(i, true))
			_, d, _, err := r.c.rows(q, repeat(1, 7), 0.02, 1e9)
			if err != nil {
				t.Fatal(err)
			}
			offs[i] -= d - gap
		}
	}
	r.c.setLive(build(0, false))
	if d := r.minDist(t); math.Abs(d-gap) > 1e-3 {
		t.Fatalf("closest %.4f, want %v", d, gap)
	}
}

func TestLiveBackAwayAtRest(t *testing.T) {
	r := liveRig(t, 30, 60, 100)
	runTicks(t, r.l, 3)
	r.setSlabs(t, r.l.qCmd, 5, []int{2}, []float64{-1})
	reached := -1
	for i := 0; i < 100; i++ {
		r.tick(t)
		if late := r.l.Status().CollisionLate; late != "" { // sticky: none on the way out or while settling
			t.Fatalf("tick %d: late warning: %q", i, late)
		}
		if reached < 0 && r.minDist(t) >= r.c.dMin-0.5 {
			reached = i + 1
		}
	}
	st := r.l.Status()
	d := r.minDist(t)
	t.Logf("start 5.00 mm, now %.3f mm; reached d_min-0.5 after %d ticks; peak accel %.3f", d, reached, st.PeakAccelRatio)
	if reached < 0 || d < r.c.dMin-0.5 || st.PeakAccelRatio > 1+1e-9 {
		t.Fatalf("reached %d d %.3f peak %.4f", reached, d, st.PeakAccelRatio)
	}
}

func TestLiveBackAwayWhileApproaching(t *testing.T) {
	r := liveRig(t, 30, 60, 100)
	r.l.SetTarget(r.at(r.fp.X, r.fp.Y, r.fp.Z-150))
	runTicks(t, r.l, 40)
	vel := maxAbs(r.l.lastStep)
	if vel == 0 {
		t.Fatal("arm not moving")
	}
	// the slab's top is 5 mm from the closest part, directly in the way
	r.setSlabs(t, r.l.qCmd, 5, []int{2}, []float64{-1})
	minD, reached := math.Inf(1), -1
	for i := 0; i < 200; i++ {
		pre := r.minDist(t)
		r.l.status.CollisionLate = ""
		r.tick(t)
		d := r.minDist(t)
		minD = math.Min(minD, d)
		if late := r.l.Status().CollisionLate; pre < r.c.dMin && late != "" {
			t.Fatalf("tick %d: late warning while backing away: %q", i, late)
		}
		if reached < 0 && d >= r.c.dMin-0.5 {
			reached = i + 1
		}
		if st := r.l.Status(); st.PeakAccelRatio > 1+1e-9 {
			t.Fatalf("tick %d: peak %.4f", i, st.PeakAccelRatio)
		}
	}
	d := r.minDist(t)
	t.Logf("moving at %.4f rad/tick; closest %.3f mm, back to d_min-0.5 after %d ticks, final %.3f", vel, minD, reached, d)
	if reached < 0 || d < r.c.dMin-0.5 {
		t.Fatalf("reached %d final %.3f", reached, d)
	}
}

// tier1Fails builds this tick's rows and task rows and reports whether the first relaxed solve fails.
func tier1Fails(t *testing.T, r *pandaRig, dt float64) (rows []collRow, tier1, tier2 error) {
	t.Helper()
	l := r.l
	rows, _, _, err := r.c.rows(l.qCmd, repeat(deg2rad(60), 7), dt, l.cfg.MaxLinearMMPerS*dt)
	if err != nil {
		t.Fatal(err)
	}
	cp, cq, _ := FK(r.m, l.qCmd)
	J, _ := Jacobian(r.m, l.qCmd)
	tp, tq := l.target.Point().Mul(1/mmPerM), l.target.Orientation().Quaternion()
	_, _, _, _, _, trows, err := l.solve(J, tp, tq, cp, cq, [6]float64{}, 1, dt)
	if err != nil {
		t.Fatal(err)
	}
	_, _, _, tier1 = l.qpNext(trows, 1, dt, rows, 1)
	_, _, _, tier2 = l.qpNext(trows, 1, dt, rows, 2)
	return rows, tier1, tier2
}

func pinchRun(t *testing.T, r *pandaRig, axis int) {
	t.Helper()
	r.setSlabs(t, r.l.qCmd, 5, []int{axis, axis}, []float64{-1, 1})
	rows, e1, e2 := tier1Fails(t, r, 0.02)
	nb := 0
	for _, w := range rows {
		if w.backAway {
			nb++
		}
	}
	t.Logf("pinch axis %d: %d rows, %d back-away; tier 1 err %v; tier 2 err %v", axis, len(rows), nb, e1, e2)
	if e1 == nil || e2 != nil {
		t.Fatalf("precondition: tier 1 err %v, tier 2 err %v", e1, e2)
	}
	// the task pulls into one slab
	tgt := []float64{r.fp.X, r.fp.Y, r.fp.Z}
	tgt[axis] += 30
	r.l.SetTarget(r.at(tgt[0], tgt[1], tgt[2]))
	startMin := r.minDist(t)
	minD := startMin
	for i := 0; i < 200; i++ {
		r.tick(t) // a fault fails the test
		minD = math.Min(minD, r.minDist(t))
	}
	st := r.l.Status()
	drift := startMin - minD
	t.Logf("pinched ticks %d, brakes %d, closest %.4f mm from start %.4f, drift %.5f mm, peak %.3f", st.LivePinched, st.CollisionBrakes, minD, startMin, drift, st.PeakAccelRatio)
	if st.LivePinched == 0 || drift > 0.1 || st.PeakAccelRatio > 1+1e-9 {
		t.Fatalf("pinched %d drift %.4f peak %.4f", st.LivePinched, drift, st.PeakAccelRatio)
	}
}

func TestLivePinchedLiveLive(t *testing.T) {
	r := liveRig(t, 30, 60, 100)
	runTicks(t, r.l, 3)
	pinchRun(t, r, 1)
}

// liveDist is the closest live pair's distance at the loop's commanded joints.
func (r *pandaRig) liveDist(t *testing.T) float64 {
	t.Helper()
	posed, err := r.c.poseParts(r.l.qCmd)
	if err != nil {
		t.Fatal(err)
	}
	best := math.Inf(1)
	for i, p := range r.c.pairs {
		if p.live {
			d, err := r.c.pairDist(posed, i)
			if err != nil {
				t.Fatal(err)
			}
			best = math.Min(best, d)
		}
	}
	return best
}

// a live box and a static box, both at 5 mm: the same pinch
func TestLivePinchedLiveStatic(t *testing.T) {
	m := panda(t)
	fp := eePoint(t, m, pandaHold)
	// the static slab on the +y side, its face 5 mm from the closest part: found by shifting
	var r *pandaRig
	off := 40.0
	for it := 0; it < 5; it++ {
		pose := spatialmath.NewPoseFromPoint(r3.Vector{X: fp.X, Y: fp.Y + off + 50, Z: fp.Z})
		r = newPandaRig(t, CollisionRequest{}, 30, 60, 100, pose, mustBox(t, r3.Vector{X: 300, Y: 100, Z: 300}), pandaHold)
		runTicks(t, r.l, 1)
		for i := range r.c.pairs {
			r.c.pairs[i].suspended = false
		}
		off -= r.minDist(t) - 5
	}
	offL := 40.0
	for it := 0; it < 5; it++ {
		r.c.setLive([]liveObstacle{slab(t, fp, 1, -1, offL)})
		offL -= r.liveDist(t) - 5
	}
	r.c.setLive([]liveObstacle{slab(t, fp, 1, -1, offL)})
	rows, e1, e2 := tier1Fails(t, r, 0.02)
	t.Logf("%d rows; tier 1 err %v; tier 2 err %v; static %.3f live %.3f", len(rows), e1, e2, r.minDist(t), r.liveDist(t))
	if e1 == nil || e2 != nil {
		t.Fatalf("precondition: tier 1 err %v, tier 2 err %v", e1, e2)
	}
	r.l.SetTarget(r.at(fp.X, fp.Y+30, fp.Z))
	start := r.minDist(t)
	minD := start
	for i := 0; i < 200; i++ {
		r.tick(t)
		minD = math.Min(minD, r.minDist(t))
	}
	st := r.l.Status()
	t.Logf("pinched ticks %d, brakes %d, drift %.5f mm, peak %.3f", st.LivePinched, st.CollisionBrakes, start-minD, st.PeakAccelRatio)
	if st.LivePinched == 0 || start-minD > 0.1 || st.PeakAccelRatio > 1+1e-9 {
		t.Fatalf("pinched %d drift %.4f peak %.4f", st.LivePinched, start-minD, st.PeakAccelRatio)
	}
}

// Live support adds nothing when no live obstacles exist: bit-identical sends.
func TestLiveNoChangeWithoutLive(t *testing.T) {
	run := func(clear bool) [][]float64 {
		m := panda(t)
		fp := eePoint(t, m, pandaHold)
		pose := spatialmath.NewPoseFromPoint(r3.Vector{X: fp.X, Y: fp.Y, Z: fp.Z - 60 - 50})
		r := newPandaRig(t, CollisionRequest{}, 30, 60, 0, pose, mustBox(t, r3.Vector{X: 300, Y: 300, Z: 100}), pandaHold)
		if clear {
			r.c.setLive(nil)
		}
		runTicks(t, r.l, 5)
		r.l.SetTarget(r.at(r.fp.X, r.fp.Y, r.fp.Z-110))
		runTicks(t, r.l, 300)
		return r.sim.sends
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
	t.Logf("%d sends identical", len(a))
}

func TestLiveRowsTime(t *testing.T) {
	if testing.Short() || raceOn {
		t.Skip("cost test: skipped under -short and -race")
	}
	m := panda(t)
	fp := eePoint(t, m, pandaHold)
	c := buildColl(t, m, CollisionRequest{}, pandaHold)
	var obs []liveObstacle
	for k := 0; k < 32; k++ {
		a := float64(k) * 2 * math.Pi / 32
		obs = append(obs, liveBoxAt(t, r3.Vector{X: fp.X + 90*math.Cos(a), Y: fp.Y + 90*math.Sin(a), Z: fp.Z - 40 + float64(k%4)*30}, r3.Vector{X: 40, Y: 40, Z: 40}))
	}
	c.setLive(obs)
	acc := repeat(1, 7)
	var ds []time.Duration
	nrows := 0
	for i := 0; i < 1000; i++ {
		s := time.Now()
		r, _, _, err := c.rows(pandaHold, acc, 0.02, 2)
		if err != nil {
			t.Fatal(err)
		}
		ds = append(ds, time.Since(s))
		nrows = len(r)
	}
	sort.Slice(ds, func(a, b int) bool { return ds[a] < ds[b] })
	p50, p99 := ds[500], ds[990]
	t.Logf("pairs %d rows %d p50 %v p99 %v", len(c.pairs), nrows, p50, p99)
	if nrows != maxCollRows {
		t.Fatalf("%d rows, want the %d cap", nrows, maxCollRows)
	}
	if p99 >= 5*time.Millisecond {
		t.Fatalf("p99 %v >= 5ms", p99)
	}
}

// A live box overlapping the closest part by 10 mm: the arm separates.
func TestLivePenetrationSeparates(t *testing.T) {
	r := liveRig(t, 30, 60, 100)
	runTicks(t, r.l, 3)
	r.setSlabs(t, r.l.qCmd, -10, []int{2}, []float64{-1})
	if d := r.minDist(t); d > -9.99 {
		t.Fatalf("start d %.3f", d)
	}
	reached := -1
	for i := 0; i < 200; i++ {
		r.tick(t)
		if reached < 0 && r.minDist(t) >= r.c.dMin-0.5 {
			reached = i + 1
		}
	}
	st := r.l.Status()
	d := r.minDist(t)
	t.Logf("start -10 mm, now %.3f; reached d_min-0.5 after %d ticks; peak %.3f", d, reached, st.PeakAccelRatio)
	if reached < 0 || d < r.c.dMin-0.5 || st.PeakAccelRatio > 1+1e-9 {
		t.Fatalf("reached %d d %.3f peak %.4f", reached, d, st.PeakAccelRatio)
	}
}

// movingRun advances a live slab toward the arm 1 mm per tick (50 mm/s), republished every tick, with the task
// pulling into it. It returns the closest true distance at q_cmd and the ticks where tier 1 was infeasible.
func movingRun(t *testing.T, axis int, sign, start, sz float64, ticks int) (minD float64, infeasible int, r *pandaRig) {
	t.Helper()
	r = liveRig(t, 30, 60, 100)
	runTicks(t, r.l, 3)
	fp := eePoint(t, r.m, r.l.qCmd)
	off := 40.0 // calibrate: the slab's closest part starts start mm from it
	for it := 0; it < 5; it++ {
		r.c.setLive([]liveObstacle{cube(t, fp, axis, sign, off, sz)})
		off -= r.liveDist(t) - start
	}
	minD = math.Inf(1)
	for i := 0; i < ticks; i++ {
		r.c.setLive([]liveObstacle{cube(t, fp, axis, sign, off-float64(i), sz)})
		if _, e1, _ := tier1Fails(t, r, 0.02); e1 != nil {
			infeasible++
		}
		r.tick(t)
		minD = math.Min(minD, r.liveDist(t))
	}
	return minD, infeasible, r
}

func TestLiveMovingIntruderRetreats(t *testing.T) {
	const axis, sign, start, sz, ticks = 0, -1.0, 12.0, 60.0, 60
	old := tier2SoftRetreat
	defer func() { tier2SoftRetreat = old }()
	tier2SoftRetreat = false
	dOld, infOld, ro := movingRun(t, axis, sign, start, sz, ticks)
	tier2SoftRetreat = true
	dNew, infNew, rn := movingRun(t, axis, sign, start, sz, ticks)
	st := rn.l.Status()
	t.Logf("tier 1 infeasible ticks old %d new %d; min distance old %.3f new %.3f mm; pinched %d; peak %.3f/%.3f",
		infOld, infNew, dOld, dNew, st.LivePinched, ro.l.Status().PeakAccelRatio, st.PeakAccelRatio)
	if infNew == 0 || st.LivePinched == 0 {
		t.Fatalf("precondition: tier 1 infeasible on %d ticks, pinched %d", infNew, st.LivePinched)
	}
	if dNew < rn.c.dMin-5 || dNew <= dOld {
		t.Fatalf("new min %.3f (need >= %.3f and > old %.3f)", dNew, rn.c.dMin-5, dOld)
	}
	if st.PeakAccelRatio > 1+1e-9 || ro.l.Status().PeakAccelRatio > 1+1e-9 {
		t.Fatalf("peak accel ratio %.4f", st.PeakAccelRatio)
	}
}

// cube is slab with a sz mm cube.
func cube(t *testing.T, fp r3.Vector, axis int, sign, off, sz float64) liveObstacle {
	c := []float64{fp.X, fp.Y, fp.Z}
	c[axis] += sign * (off + sz/2)
	return liveBoxAt(t, r3.Vector{X: c[0], Y: c[1], Z: c[2]}, r3.Vector{X: sz, Y: sz, Z: sz})
}

// A live obstacle that appears inside d_act while the arm approaches is not a "d_act too small" case.
func TestLiveAppearingInsideDActNotLate(t *testing.T) {
	r := liveRig(t, 30, 60, 100)
	r.l.SetTarget(r.at(r.fp.X, r.fp.Y, r.fp.Z-150))
	runTicks(t, r.l, 40)
	r.setSlabs(t, r.l.qCmd, 12, []int{2}, []float64{-1})
	for i := 0; i < 60; i++ {
		r.tick(t)
		if late := r.l.Status().CollisionLate; late != "" {
			t.Fatalf("tick %d: late warning for a live pair: %q", i, late)
		}
	}
}
