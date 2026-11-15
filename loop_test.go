package diffik

import (
	"context"
	"errors"
	"math"
	"slices"
	"sync"
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
	stopErr error // returned by Stop when set
	block   bool  // Read blocks until ctx is done
}

func (s *simIO) Read(ctx context.Context) ([]referenceframe.Input, error) {
	if s.block {
		<-ctx.Done()
		return nil, ctx.Err()
	}
	return slices.Clone(s.q), nil
}

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

func (s *simIO) Stop(context.Context) error { s.stopped = true; return s.stopErr }

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
		"rate":                 {RateHz: -1},
		"weights":              {TaskWeights: [6]float64{1, 1, 1, -1, 1, 1}},
		"limits":               {MaxJointDegPerS: []float64{90, 90}},
		"slow rate":            {RateHz: 0.5},
		"empty limits":         {MaxJointDegPerS: []float64{}},
		"negative joint limit": {MaxJointDegPerS: []float64{90, 90, 90, 90, 90, -90}},
		"negative gain":        {Gain: -1},
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
	clockTicks(t, l, clk, 50)       // 1 s
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

func TestAngularTwistAdvancesTargetAndTracks(t *testing.T) {
	l, sim, m, clk := newClockLoop(t, home, LoopConfig{})
	start := offsetTarget(t, m, home, r3.Vector{}, 0)
	l.SetTarget(start)
	l.SetVelocity([6]float64{5: deg2rad(10)}) // 10 °/s about base z
	clockTicks(t, l, clk, 50)                 // 1 s
	s := l.Status()
	if s.ErrDeg > 0.5 || s.State != StateRunning {
		t.Fatalf("err = %.3f° state = %s, want < 0.5° and running", s.ErrDeg, s.State)
	}
	_, q0, err := FK(m, home)
	if err != nil {
		t.Fatal(err)
	}
	_, q1, err := FK(m, sim.q)
	if err != nil {
		t.Fatal(err)
	}
	r := RotVec(q1, q0)
	if math.Abs(r.Norm()-deg2rad(10)) > deg2rad(0.5) || math.Abs(r.Z) < 0.9*r.Norm() {
		t.Fatalf("arm rotated %v (%.2f°), want 10° ± 0.5° about z", r, r.Norm()*180/math.Pi)
	}
}

func TestRampStopIgnoresNewMotion(t *testing.T) {
	l, _, m, clk := newClockLoop(t, home, LoopConfig{})
	l.SetTarget(offsetTarget(t, m, home, r3.Vector{}, 0))
	clockTicks(t, l, clk, 5)
	want := l.Status().TargetMM
	l.RampStop(200*time.Millisecond, StateStopped)
	l.SetVelocity([6]float64{0.05})
	l.SetTarget(offsetTarget(t, m, home, r3.Vector{X: 50}, 0))
	if !clockTicks(t, l, clk, 20) {
		t.Fatal("ramp did not complete")
	}
	s := l.Status()
	if s.State != StateStopped || s.TargetMM.Sub(want).Norm() > 1e-9 {
		t.Fatalf("state = %s target = %v, want stopped at %v", s.State, s.TargetMM, want)
	}
}

func TestRampStopKeepsStoppedAgainstLaterTimeout(t *testing.T) {
	l, sim, _, clk := newClockLoop(t, home, LoopConfig{})
	l.RampStop(50*time.Millisecond, StateStopped)
	l.RampStop(time.Hour, StateStoppedTimeout)
	clockTicks(t, l, clk, 5)
	if s := l.Status().State; !sim.stopped || s != StateStopped {
		t.Fatalf("state = %s, want stopped", s)
	}
}

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

// A Read that hangs must hit the per-tick deadline and fault, not block the loop forever.
func TestHungReadFaults(t *testing.T) {
	l, sim, _ := newTestLoop(t, home, LoopConfig{})
	sim.block = true
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	start := time.Now()
	err := l.Run(ctx)
	if !errors.Is(err, context.DeadlineExceeded) || ctx.Err() != nil {
		t.Fatalf("err = %v, outer ctx err = %v, want the tick's deadline", err, ctx.Err())
	}
	if d := time.Since(start); d > time.Second {
		t.Fatalf("Run took %v, want < 1 s", d)
	}
	if l.Status().State != StateFaulted || !sim.stopped {
		t.Fatalf("state = %s stopped = %v, want faulted and stopped", l.Status().State, sim.stopped)
	}
}
