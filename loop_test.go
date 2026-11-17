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
	"gonum.org/v1/gonum/mat"
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
	delay   int   // the arm reaches each command this many sends late
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
	if i := len(s.sends) - 1 - s.delay; i >= 0 {
		s.q = slices.Clone(s.sends[i])
	}
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
		"huge max lag":         {MaxLagMS: 1001},
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

// lagRun drives the target along y at a steady speed so the arm moves the whole time.
func lagRun(t *testing.T, delay, ticks int) (*Loop, *simIO) {
	t.Helper()
	l, sim, m := newTestLoop(t, home, LoopConfig{MaxJointDegPerS: []float64{20, 20, 20, 20, 20, 20}, MaxLinearMMPerS: 200})
	sim.delay = delay
	l.SetTarget(offsetTarget(t, m, home, r3.Vector{}, 0))
	l.SetVelocity([6]float64{0, 0.1, 0, 0, 0, 0})
	runTicks(t, l, ticks)
	return l, sim
}

func TestLagCompensatedResync(t *testing.T) {
	const delay = 10 // 200 ms at 50 Hz
	l, _ := lagRun(t, delay, 150)
	s := l.Status()
	if s.Resyncs != 0 {
		t.Fatalf("resyncs = %d, want 0 for a steady lag", s.Resyncs)
	}
	if want := float64(delay) / l.cfg.RateHz * 1000; math.Abs(s.LagMS-want) > 0.15*want {
		t.Fatalf("LagMS = %.1f, want about %.1f", s.LagMS, want)
	}
}

func TestResyncWhenArmStallsMidMotion(t *testing.T) {
	const lag = 10
	l, sim := lagRun(t, lag, 100)
	step := 0.0 // largest per-tick command step just before the stall
	for i := len(sim.sends) - 10; i < len(sim.sends); i++ {
		step = math.Max(step, maxAbsDiff(sim.sends[i], sim.sends[i-1]))
	}
	sim.ignore = 1 << 30 // arm stops following
	before := l.Status().Resyncs
	gap := 0.0
	for i := 1; ; i++ {
		if i > lag+5 {
			t.Fatalf("no resync within %d ticks of the stall", lag+5)
		}
		runTicks(t, l, 1)
		s := l.Status()
		if s.Resyncs > before {
			break
		}
		gap = math.Max(gap, maxAbsDiff(l.qCmd, sim.q))
	}
	if bound := (lag+1)*step + deg2rad(l.cfg.ResyncDeg); gap > bound {
		t.Fatalf("command ran %.2f° ahead of the stalled arm, want <= %.2f°", gap*180/math.Pi, bound*180/math.Pi)
	}
}

func TestLagStaysZeroAtRest(t *testing.T) {
	l, _, m := newTestLoop(t, home, LoopConfig{})
	l.SetTarget(offsetTarget(t, m, home, r3.Vector{}, 0))
	runTicks(t, l, 100)
	if s := l.Status(); s.LagMS != 0 || s.Resyncs != 0 {
		t.Fatalf("LagMS = %.1f resyncs = %d at rest, want 0 and 0", s.LagMS, s.Resyncs)
	}
}

func TestLagHoldsThroughStopThenHold(t *testing.T) {
	l, _ := lagRun(t, 10, 100)
	before := l.Status().LagMS
	l.SetVelocity([6]float64{}) // commands stop; the arm drains its pipeline, then sits still
	runTicks(t, l, 1500)
	if s := l.Status(); math.Abs(s.LagMS-before) > 0.15*before || s.Resyncs != 0 {
		t.Fatalf("LagMS %.1f -> %.1f resyncs = %d, want unchanged and 0", before, s.LagMS, s.Resyncs)
	}
}

// settles runs n more ticks and fails if a resync lands in the last window of them or LagMS is off.
func settles(t *testing.T, l *Loop, sim *simIO, n, window, lag int) {
	t.Helper()
	runTicks(t, l, n-window)
	mid := l.Status().Resyncs
	runTicks(t, l, window)
	s := l.Status()
	if s.Resyncs != mid {
		t.Fatalf("%d resyncs in the last %d ticks, want none (total %d)", s.Resyncs-mid, window, s.Resyncs)
	}
	if want := float64(lag) / l.cfg.RateHz * 1000; math.Abs(s.LagMS-want) > 0.2*want {
		t.Fatalf("LagMS = %.1f, want about %.1f", s.LagMS, want)
	}
}

func TestRecoversFromDroppedSends(t *testing.T) {
	const lag = 10
	l, sim := lagRun(t, lag, 100)
	sim.ignore = 8 // the arm misses 8 updates, then catches up with its pipeline
	settles(t, l, sim, 6*lag+8, 2*lag, lag)
}

func TestRecoversFromStallAndRelease(t *testing.T) {
	const lag = 10
	l, sim, m := newTestLoop(t, home, LoopConfig{MaxLinearMMPerS: 400, MaxJointDegPerS: []float64{45, 45, 45, 45, 45, 45}})
	sim.delay = lag
	l.SetTarget(offsetTarget(t, m, home, r3.Vector{}, 0))
	l.SetVelocity([6]float64{0, 0.25, 0, 0, 0, 0}) // fast: 250 mm/s TCP, default resync_deg (2.7 deg)
	runTicks(t, l, 60)
	sim.ignore = 20 // the arm freezes for 20 ticks, then follows its pipeline again
	settles(t, l, sim, 20+160, 2*lag, lag)
}

func TestArmThatNeverFollowsResyncsRepeatedly(t *testing.T) {
	l, sim := lagRun(t, 0, 0)
	sim.ignore = 1 << 30
	runTicks(t, l, 1)
	step := maxAbsDiff(sim.sends[0], sim.q)
	first := -1
	gap := 0.0
	for i := 0; i < 200; i++ {
		runTicks(t, l, 1)
		if first < 0 && l.Status().Resyncs > 0 {
			first = i
		}
		if first >= 0 {
			gap = math.Max(gap, maxAbsDiff(l.qCmd, sim.q))
		}
	}
	if first < 0 || l.Status().Resyncs < 40 {
		t.Fatalf("resyncs = %d (first at %d), want a steady stream after the first", l.Status().Resyncs, first)
	}
	if bound := deg2rad(l.cfg.ResyncDeg) + 2*step; gap > bound {
		t.Fatalf("gap after the first resync %.2f°, want <= %.2f°", gap*180/math.Pi, bound*180/math.Pi)
	}
}

func TestMaxLagMSSetsRingWindow(t *testing.T) {
	// 10 ticks of lag (200 ms) fits a 300 ms cap but not a 100 ms one (5 ticks), which resyncs instead.
	run := func(maxLagMS float64) int {
		l, sim, m := newTestLoop(t, home, LoopConfig{MaxJointDegPerS: []float64{20, 20, 20, 20, 20, 20}, MaxLinearMMPerS: 200, MaxLagMS: maxLagMS, ResyncDeg: 0.6})
		sim.delay = 10
		l.SetTarget(offsetTarget(t, m, home, r3.Vector{}, 0))
		l.SetVelocity([6]float64{0, 0.1, 0, 0, 0, 0})
		runTicks(t, l, 150)
		return l.Status().Resyncs
	}
	if n := run(300); n != 0 {
		t.Fatalf("300 ms cap: resyncs = %d, want 0", n)
	}
	if n := run(100); n == 0 {
		t.Fatal("100 ms cap: want resyncs for a 200 ms lag")
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
// The twist mixes the in-plane pitch axis with the residual direction r, so a leash on the full
// pointing error would block it (w·r > 0); the reachable part is not near the leash threshold.
func TestSO101LeashIgnoresUnreachableResidual(t *testing.T) {
	l, _, m, clk := newSO101Loop(t, q0a, LoopConfig{})
	start := fkPose(t, m, q0a)
	rq := spatialmath.Quaternion(quat.Mul(rotZ(30*math.Pi/180), start.Orientation().Quaternion()))
	l.SetTarget(spatialmath.NewPose(start.Point(), &rq))
	clockTicks(t, l, clk, 100) // converge with a large residual
	_, cq, _ := FK(m, l.Status().QCmd)
	r, _, _ := pointingError(toolAxis(cq), toolAxis(rq.Quaternion()))
	if r.Norm() < deg2rad(3) {
		t.Fatalf("test needs a large residual, got %.2f°", r.Norm()*180/math.Pi)
	}
	pitch := toolAxis(cq).Cross(r3.Vector{Z: 1}).Normalize()
	if pitch.Dot(r) < 0 {
		pitch = pitch.Mul(-1)
	}
	w := pitch.Add(r.Normalize().Mul(0.3)).Normalize().Mul(deg2rad(10))
	l.SetVelocity([6]float64{0, 0, 0, w.X, w.Y, w.Z})
	clockTicks(t, l, clk, 25)
	l.mu.Lock()
	got := RotVec(l.target.Orientation().Quaternion(), rq.Quaternion()).Norm() * 180 / math.Pi
	l.mu.Unlock()
	if math.Abs(got-5) > 0.25 {
		t.Fatalf("target rotated %.2f° in 0.5 s at 10°/s, want 5 (leash must ignore the residual)", got)
	}
}

// A pure roll twist must not move the target's tool axis or cost position (spec: roll discarded).
func TestSO101PointingLeashIgnoresRoll(t *testing.T) {
	l, _, m, clk := newSO101Loop(t, q0a, LoopConfig{})
	l.SetTarget(fkPose(t, m, q0a))
	clockTicks(t, l, clk, 5)
	_, cq, _ := FK(m, l.Status().QCmd)
	a0 := toolAxis(cq)
	l.mu.Lock()
	tq0 := l.target.Orientation().Quaternion()
	before := toolAxis(tq0)
	l.mu.Unlock()
	w := a0.Mul(deg2rad(10))
	l.SetVelocity([6]float64{0, 0, 0, w.X, w.Y, w.Z})
	clockTicks(t, l, clk, 25)
	l.mu.Lock()
	tq1 := l.target.Orientation().Quaternion()
	l.mu.Unlock()
	after := toolAxis(tq1)
	if ang := before.Angle(after).Radians() * 180 / math.Pi; ang > 0.01 {
		t.Fatalf("roll twist moved the target axis by %.4f°, want < 0.01", ang)
	}
	// the axis alone can't tell a rolled target from an untouched one: the whole orientation must not move either
	if rot := RotVec(tq1, tq0).Norm() * 180 / math.Pi; rot > 0.01 {
		t.Fatalf("roll twist rotated the target by %.3f°, want it discarded", rot)
	}
	if s := l.Status(); s.ErrMM > 0.1 {
		t.Fatalf("position error %.3f mm under a roll twist, want < 0.1", s.ErrMM)
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

// Weight 0 on z drops that row: Status.ErrMM covers x, y only, and the arm does not chase z.
func TestUR5ePoseDroppedPositionRow(t *testing.T) {
	l, _, m, clk := newClockLoop(t, home, LoopConfig{TaskWeights: [6]float64{1, 1, 0, 1, 1, 1}})
	tgt := offsetTarget(t, m, home, r3.Vector{X: 30, Y: 30, Z: -200}, 0)
	l.SetTarget(tgt)
	// slow: the linear cap is taken over the full 3-vector, dropped z included, so x,y get ~¼ of the cap
	clockTicks(t, l, clk, 300)
	s := l.Status()
	if s.State != StateConverged || s.ErrMM > 2 || s.Damping != 0 {
		t.Fatalf("state=%s err=%.2f mm damping=%v, want converged on x,y, undamped", s.State, s.ErrMM, s.Damping)
	}
	p, _, _ := FK(m, s.QMeas)
	if zerr := math.Abs(tgt.Point().Z - p.Z*mmPerM); zerr < 50 {
		t.Fatalf("true z error %.1f mm, want large (z row must be dropped)", zerr)
	}
}

// With m=5 < n=6 active rows the step is dpinv(W½J_active)·W½dx, undamped away from singularity.
func TestUR5ePoseMLessThanNUsesDpinv(t *testing.T) {
	w := [6]float64{1, 1, 1, 1, 1, 0}
	l, sim, m, _ := newClockLoop(t, home, LoopConfig{TaskWeights: w})
	tgt := offsetTarget(t, m, home, r3.Vector{X: 30, Y: 30, Z: -25}, deg2rad(10))
	l.SetTarget(tgt)
	runTicks(t, l, 1)
	got := make([]float64, len(home))
	for i := range got {
		got[i] = sim.sends[0][i] - home[i]
	}
	cp, cq, _ := FK(m, home)
	e := PoseError(tgt.Point().Mul(1/mmPerM), tgt.Orientation().Quaternion(), cp, cq)
	c, dt := l.cfg, 1/l.cfg.RateHz
	lin := capNorm(r3.Vector{X: c.Gain * e[0], Y: c.Gain * e[1], Z: c.Gain * e[2]}, c.MaxLinearMMPerS/mmPerM)
	ang := capNorm(r3.Vector{X: c.Gain * e[3], Y: c.Gain * e[4], Z: c.Gain * e[5]}, deg2rad(c.MaxAngularDegPerS))
	dx := [6]float64{lin.X * dt, lin.Y * dt, lin.Z * dt, ang.X * dt, ang.Y * dt, ang.Z * dt}
	J, err := Jacobian(m, home)
	if err != nil {
		t.Fatal(err)
	}
	A := mat.NewDense(5, 6, nil)
	b := make([]float64, 5)
	for i := 0; i < 5; i++ {
		A.SetRow(i, J.RawRowView(i))
		b[i] = dx[i]
	}
	P, _, _, _, err := dpinv(A, c.SingularEps, c.MaxDamping, true)
	if err != nil {
		t.Fatal(err)
	}
	nearVec(t, got, mulVec(P, b), 1e-12)
	if d := l.Status().Damping; d != 0 {
		t.Fatalf("damping %v away from singularity, want 0", d)
	}
	dls, _, _, err := dlsStep(J, w, dx, c.SingularEps, c.MaxDamping)
	if err != nil {
		t.Fatal(err)
	}
	if maxAbsDiff(dls, got) < 1e-9 {
		t.Fatal("step equals the (singular-damped) dlsStep; the m<n path is not used")
	}
}

// SingularEps above every singular value forces λ > 0; the step is then reported as limited.
func TestPointingLimitedWhenDamped(t *testing.T) {
	l, _, m, clk := newSO101Loop(t, q0a, LoopConfig{SingularEps: 1})
	qt := []float64{q0a[0] + 0.12, q0a[1] - 0.1, q0a[2] + 0.15, q0a[3] - 0.15, q0a[4] + 0.1}
	l.SetTarget(fkPose(t, m, qt))
	clockTicks(t, l, clk, 5)
	if s := l.Status(); s.Damping <= 0 || !s.Limited {
		t.Fatalf("damping=%v limited=%v, want damped and limited", s.Damping, s.Limited)
	}
}

// Pointing mode scales both priority levels by the ramp, and caps the position velocity.
func TestPointingRampScalesAndCaps(t *testing.T) {
	m := so101(t)
	// (a) far reachable target: each tick's position step stays within the linear cap
	l, sim, _, clk := newSO101Loop(t, q0a, LoopConfig{})
	qt := []float64{q0a[0] + 0.6, q0a[1] - 0.5, q0a[2] + 0.5, q0a[3] - 0.3, q0a[4]}
	far := fkPose(t, m, qt)
	if d := far.Point().Sub(fkPose(t, m, q0a).Point()).Norm(); d < 100 {
		t.Fatalf("test target only %.0f mm away", d)
	}
	l.SetTarget(far)
	clockTicks(t, l, clk, 30)
	prev := fkPose(t, m, q0a).Point()
	maxStep := 0.0
	for _, q := range sim.sends {
		p := fkPose(t, m, q).Point()
		maxStep = math.Max(maxStep, p.Sub(prev).Norm())
		prev = p
	}
	// 0.2 mm of slack: the Jacobian is linear, the arm is not
	if lim := l.cfg.MaxLinearMMPerS / l.cfg.RateHz; maxStep > lim+0.2 {
		t.Fatalf("position step %.3f mm per tick, want ≤ %.3f", maxStep, lim)
	} else if maxStep < 0.5*lim {
		t.Fatalf("position step %.3f mm per tick never approached the cap %.3f", maxStep, lim)
	}

	// (b) pointing-dominant target (pitch only, same position): the ramp must scale the pointing level
	l, sim, _, clk = newSO101Loop(t, q0a, LoopConfig{})
	start := fkPose(t, m, q0a)
	_, cq, _ := FK(m, q0a)
	pitch := toolAxis(cq).Cross(r3.Vector{Z: 1}).Normalize()
	tq := spatialmath.Quaternion(quat.Mul(ExpRot(pitch.Mul(deg2rad(40))), start.Orientation().Quaternion()))
	l.SetTarget(spatialmath.NewPose(start.Point(), &tq))
	clockTicks(t, l, clk, 5)
	n0 := len(sim.sends)
	l.RampStop(100*time.Millisecond, StateStopped)
	if !clockTicks(t, l, clk, 10) {
		t.Fatal("ramp did not complete")
	}
	steps := sim.sends[n0-1:]
	first := maxAbsDiff(steps[1], steps[0])
	if first < 1e-6 {
		t.Fatal("no motion before the ramp; target is not pointing-dominant")
	}
	last := first
	for k := 2; k < len(steps); k++ {
		d := maxAbsDiff(steps[k], steps[k-1])
		if d > last*1.02+1e-12 {
			t.Fatalf("ramp step %d grew: %.3g > %.3g", k, d, last)
		}
		last = d
	}
	if last > 0.3*first {
		t.Fatalf("last ramp step %.3g is not near zero (first %.3g)", last, first)
	}
}

// Pose mode: an unreachable error on a dropped row (rz weight 0) must not leash the target (spec "Leash").
func TestPoseLeashIgnoresDroppedRow(t *testing.T) {
	l, _, m, clk := newClockLoop(t, home, LoopConfig{TaskWeights: [6]float64{1, 1, 1, 1, 1, 0}})
	tgt := offsetTarget(t, m, home, r3.Vector{}, deg2rad(20)) // 20° yaw the dropped row can't fix
	l.SetTarget(tgt)
	clockTicks(t, l, clk, 100)
	l.SetVelocity([6]float64{5: deg2rad(10)}) // yaw twist toward the dropped error
	clockTicks(t, l, clk, 25)
	l.mu.Lock()
	got := RotVec(l.target.Orientation().Quaternion(), tgt.Orientation().Quaternion()).Norm() * 180 / math.Pi
	l.mu.Unlock()
	if math.Abs(got-5) > 0.25 {
		t.Fatalf("target yawed %.2f° in 0.5 s at 10°/s, want 5 (dropped row must not leash)", got)
	}
}

func TestStatusReachDeg(t *testing.T) {
	// pointing, unreachable: ReachDeg is the reachable part, below ErrDeg, and converges
	l, _, m, clk := newSO101Loop(t, q0a, LoopConfig{})
	start := fkPose(t, m, q0a)
	rq := spatialmath.Quaternion(quat.Mul(rotZ(deg2rad(20)), start.Orientation().Quaternion()))
	l.SetTarget(spatialmath.NewPose(start.Point(), &rq))
	clockTicks(t, l, clk, 300)
	if s := l.Status(); !(s.ReachDeg <= s.ErrDeg) || s.ReachDeg >= 1 || s.PointingResidualDeg <= 5 {
		t.Fatalf("reach=%.2f err=%.2f residual=%.2f", s.ReachDeg, s.ErrDeg, s.PointingResidualDeg)
	}
	// pose task, reachable: ReachDeg > 0 before convergence
	u, _, um := newTestLoop(t, home, LoopConfig{})
	u.SetTarget(offsetTarget(t, um, home, r3.Vector{X: 30}, deg2rad(10)))
	runTicks(t, u, 1)
	if s := u.Status(); s.ReachDeg <= 0 {
		t.Fatalf("pose ReachDeg = %v, want > 0", s.ReachDeg)
	}
}

// wristTarget returns the pose at q0 rotated by s rad about the pitch axis (tool tilts toward/away from the limit).
func wristTarget(t *testing.T, m referenceframe.Model, q0 []float64, s float64) spatialmath.Pose {
	t.Helper()
	start := fkPose(t, m, q0)
	pitch := toolAxis(start.Orientation().Quaternion()).Cross(r3.Vector{Z: 1}).Normalize()
	h := math.Sin(s / 2)
	rq := spatialmath.Quaternion(quat.Mul(quat.Number{Real: math.Cos(s / 2), Imag: pitch.X * h, Jmag: pitch.Y * h, Kmag: pitch.Z * h},
		start.Orientation().Quaternion()))
	return spatialmath.NewPose(start.Point(), &rq)
}

// Wrist flex pinned at its upper limit: an outward pointing step must lock it in the solve (not just in
// the clamp), so position still converges and the unreachable pointing is reported as residual.
func TestPointingWristAtLimit(t *testing.T) {
	m := so101(t)
	hi := m.DoF()[3].Max
	q0 := []float64{0, 0.3, 0.3, hi - 2e-6, 0}
	for _, tc := range []struct {
		s       float64
		outward bool
	}{{0.35, true}, {-0.35, false}} {
		l, _, _, clk := newSO101Loop(t, q0, LoopConfig{})
		l.SetTarget(wristTarget(t, m, q0, tc.s))
		clockTicks(t, l, clk, 500)
		st := l.Status()
		t.Logf("s=%+.2f state=%s err=%.2fmm residual=%.2f° q3=%.4f/%.4f", tc.s, st.State, st.ErrMM, st.PointingResidualDeg, st.QCmd[3], hi)
		if st.State != StateConverged || st.ErrMM >= 2 {
			t.Fatalf("s=%v: state=%s err=%.2fmm, want converged <2mm", tc.s, st.State, st.ErrMM)
		}
		if tc.outward && st.PointingResidualDeg <= 15 {
			t.Fatalf("outward residual=%.2f°, want >15°", st.PointingResidualDeg)
		}
		if !tc.outward && st.QCmd[3] > hi-0.05 {
			t.Fatalf("inward: joint 3 stayed at limit (q3=%.4f, hi=%.4f)", st.QCmd[3], hi)
		}
	}
}

// qm sagging 1° inside the limit while qCmd sits at it must still lock the joint in measure.
func TestMeasureLocksOnCmdAtLimit(t *testing.T) {
	m := so101(t)
	hi := m.DoF()[3].Max
	qc := []float64{0, 0.3, 0.3, hi, 0}
	qm := slices.Clone(qc)
	qm[3] = hi - deg2rad(1)
	tgt := wristTarget(t, m, qc, 0.35)
	mp, mq := fkPose(t, m, qm), fkPose(t, m, qm).Orientation().Quaternion()
	l, _, _, _ := newSO101Loop(t, qc, LoopConfig{})
	l.qCmd = slices.Clone(qc)
	ms, err := l.measure(tgt.Point().Mul(1/mmPerM), tgt.Orientation().Quaternion(), mp.Point().Mul(1/mmPerM), mq, qm)
	if err != nil {
		t.Fatal(err)
	}
	if ms.residualDeg <= 15 {
		t.Fatalf("residual=%.2f°, want >15° (joint 3 locked via qCmd)", ms.residualDeg)
	}
}
