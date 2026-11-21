package diffik

import (
	"math"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

func line(t0 time.Time, from r3.Vector, step r3.Vector, n int, dt time.Duration) *trajectory {
	poses := make([]spatialmath.Pose, n)
	for i := range poses {
		poses[i] = spatialmath.NewPoseFromPoint(from.Add(step.Mul(float64(i))))
	}
	return newTrajectory(t0, dt, poses)
}

// Interpolation, the segment velocity, holding before the start and after the end.
func TestTrajectoryAt(t *testing.T) {
	t0 := time.Unix(100, 0)
	tr := line(t0, r3.Vector{X: 100}, r3.Vector{X: 10}, 3, 100*time.Millisecond) // 100 → 120 mm over 0.2 s
	p, _, v, ended := tr.at(t0.Add(150 * time.Millisecond))
	if math.Abs(p.X-115) > 1e-9 || math.Abs(v[0]-0.1) > 1e-9 || ended {
		t.Fatalf("mid: p %v v %v ended %v, want x 115, vx 0.1 m/s", p, v[:3], ended)
	}
	if p, _, v, _ := tr.at(t0.Add(-time.Second)); p.X != 100 || v != ([6]float64{}) {
		t.Fatalf("before: p %v v %v", p, v)
	}
	if p, _, _, ended := tr.at(t0.Add(time.Second)); p.X != 120 || !ended {
		t.Fatalf("after: p %v ended %v", p, ended)
	}
	if !tr.end().Equal(t0.Add(200 * time.Millisecond)) {
		t.Fatalf("end %v", tr.end())
	}
}

// The lead samples ahead, and a new chunk crossfades from the previous one over blend.
func TestTrajectoryLeadAndBlend(t *testing.T) {
	t0 := time.Unix(100, 0)
	old := line(t0, r3.Vector{}, r3.Vector{}, 2, time.Second)     // holds x 0
	tr := line(t0, r3.Vector{X: 10}, r3.Vector{}, 2, time.Second) // holds x 10
	tr.prev, tr.arrive, tr.blend = old, t0, 100*time.Millisecond
	if p, _, _ := tr.sample(t0.Add(50*time.Millisecond), 0); math.Abs(p.Point().X-5) > 1e-9 {
		t.Fatalf("halfway through the blend: x %v, want 5", p.Point().X)
	}
	if p, _, _ := tr.sample(t0.Add(200*time.Millisecond), 0); p.Point().X != 10 {
		t.Fatalf("after the blend: x %v, want 10", p.Point().X)
	}
	mv := line(t0, r3.Vector{}, r3.Vector{X: 10}, 3, 100*time.Millisecond)
	if p, _, _ := mv.sample(t0, 50*time.Millisecond); math.Abs(p.Point().X-5) > 1e-9 {
		t.Fatalf("lead 50 ms: x %v, want 5", p.Point().X)
	}
}

// The loop follows a trajectory, holds its end, and a servo_target (SetMotion) cancels it.
func TestLoopFollowsTrajectory(t *testing.T) {
	l, sim, m, clk := newClockLoop(t, home, noAccel)
	start := offsetTarget(t, m, home, r3.Vector{}, 0)
	poses := []spatialmath.Pose{start, offsetTarget(t, m, home, r3.Vector{X: 20}, 0)}
	tr := newTrajectory(clk.Now(), 500*time.Millisecond, poses)
	tr.lead = 0
	l.SetTarget(start)
	l.SetTrajectory(tr)
	for i := 0; i < 50; i++ { // 1 s: past the chunk's end
		clk.Advance(20 * time.Millisecond)
		runTicks(t, l, 1)
	}
	p, _, _ := FK(m, sim.q)
	p0, _, _ := FK(m, home)
	if d := p.Sub(p0).Mul(mmPerM); math.Abs(d.X-20) > 1 || math.Abs(d.Y) > 1 || math.Abs(d.Z) > 1 {
		t.Fatalf("moved %v mm, want +20 x", d)
	}
	if st := l.Status(); !st.Traj || !st.TrajEnded || st.TrajRemainingMS != 0 {
		t.Fatalf("status traj %v ended %v remaining %v", st.Traj, st.TrajEnded, st.TrajRemainingMS)
	}
	l.SetTarget(start)
	if l.Status().Traj || l.traj != nil {
		t.Fatal("SetMotion kept the trajectory")
	}
}

// The auto-lead reads the median of recent best-fit lag samples, not the jump-up lagTicks; expected() keeps lagTicks.
func TestAutoLeadUsesMedianLag(t *testing.T) {
	l, _, m := newTestLoop(t, home, LoopConfig{})
	l.SetTarget(offsetTarget(t, m, home, r3.Vector{}, 0))
	runTicks(t, l, 2)
	l.lagTicks = 9                          // a spike raised the jump-up estimate
	l.lagSamples = []float64{3, 3, 4, 3, 9} // but most fits say 3
	tr := newTrajectory(l.now(), 100*time.Millisecond, []spatialmath.Pose{
		offsetTarget(t, m, home, r3.Vector{}, 0), offsetTarget(t, m, home, r3.Vector{X: 10}, 0)})
	tr.autoLead = true
	l.SetTrajectory(tr)
	runTicks(t, l, 1)
	want := 3 / l.cfg.RateHz * 1000
	if st := l.Status(); math.Abs(st.TrajLeadMS-want) > 1e-6 || math.Abs(st.LagMedianMS-want) > 1e-6 {
		t.Fatalf("lead %v ms, lag median %v ms, want %v", st.TrajLeadMS, st.LagMedianMS, want)
	}
	l.lagTicks = 9
	l.sent = [][]float64{{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}}
	if got := l.expected(); got[0] != 1 { // round(lagTicks = 9) ticks before the newest
		t.Fatalf("expected() used %v, want the command 9 ticks back", got)
	}
}

func TestMedian(t *testing.T) {
	for _, c := range []struct {
		in   []float64
		want float64
	}{{[]float64{3}, 3}, {[]float64{5, 1, 3}, 3}, {[]float64{4, 1, 3, 2}, 2.5}} {
		if got := median(c.in); got != c.want {
			t.Fatalf("median(%v) = %v, want %v", c.in, got, c.want)
		}
	}
}

// Lag samples are kept only while the arm moves (a frozen qm under moving commands is a stall), capped at lagSampleN.
func TestLagSamplesMovingOnlyAndCapped(t *testing.T) {
	l, _, _ := newTestLoop(t, home, LoopConfig{})
	l.lagSamples = nil
	for i := range 11 {
		l.sent = append(l.sent, []float64{float64(i) / 10}) // 1 rad span, well over ResyncDeg/2
	}
	l.prevQm, l.moved = []float64{0.5}, true
	l.updateLag([]float64{0.5}) // frozen
	if len(l.lagSamples) != 0 {
		t.Fatalf("frozen qm added samples %v", l.lagSamples)
	}
	for i := range 2 * lagSampleN {
		l.updateLag([]float64{0.5 + 0.1*float64(i%2+1)}) // alternating, so each tick moves
	}
	if len(l.lagSamples) != lagSampleN {
		t.Fatalf("%d samples, want %d", len(l.lagSamples), lagSampleN)
	}
}
