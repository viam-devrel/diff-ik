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
