package diffik

import (
	"context"
	"errors"
	"fmt"
	"math"
	"sync"
	"sync/atomic"
	"time"

	"github.com/golang/geo/r3"
)

// ForceSource is a wrist force-torque sensor. Readings are either "force" and "torque" maps of x, y, z (N, N·m;
// mujoco-sim's force_torque) or flat Fx_N, Fy_N, Fz_N, TRx_Nm, TRy_Nm, TRz_Nm (viam:ufactory:ft_sensor).
type ForceSource interface {
	Readings(ctx context.Context, extra map[string]interface{}) (map[string]interface{}, error)
}

// GuardConfig is servo_start's guard: stop on contact.
type GuardConfig struct {
	Sensor   string
	ForceN   float64 // trip when |F − F₀| reaches this; required
	TorqueNm float64 // and/or when |T − T₀| reaches this; 0 = off
	RateHz   float64 // sensor poll rate; default 500
}

const (
	defaultGuardHz  = 500
	guardBaseline   = 5 // readings averaged into the baseline at start
	guardMaxErrRuns = 3 // consecutive failed reads that fault the session: a blind guard must not keep moving
)

func (c GuardConfig) withDefaults() GuardConfig {
	if c.RateHz == 0 {
		c.RateHz = defaultGuardHz
	}
	return c
}

func (c GuardConfig) validate() error {
	switch {
	case c.Sensor == "":
		return errors.New("guard.sensor is required")
	case !(c.ForceN >= 0): // 0: monitor only (the force controller's poller); servo_start's guard requires > 0
		return fmt.Errorf("guard.force_n must be > 0, got %v", c.ForceN)
	case !(c.TorqueNm >= 0):
		return fmt.Errorf("guard.torque_nm must be >= 0, got %v", c.TorqueNm)
	case !(c.RateHz > 0 && c.RateHz <= 2000):
		return fmt.Errorf("guard.rate_hz must be in (0, 2000], got %v", c.RateHz)
	}
	return nil
}

func ftVec(m map[string]interface{}, key string) (r3.Vector, error) {
	v, ok := m[key].(map[string]interface{})
	if !ok {
		return r3.Vector{}, fmt.Errorf("reading has no %q map", key)
	}
	var out [3]float64
	for i, k := range []string{"x", "y", "z"} {
		n, ok := v[k].(float64)
		if !ok || math.IsNaN(n) || math.IsInf(n, 0) {
			return r3.Vector{}, fmt.Errorf("reading %s.%s is not a finite number", key, k)
		}
		out[i] = n
	}
	return r3.Vector{X: out[0], Y: out[1], Z: out[2]}, nil
}

// flatVec reads three flat keys (the xArm sensor's Fx_N, ...).
func flatVec(m map[string]interface{}, keys [3]string) (r3.Vector, error) {
	var out [3]float64
	for i, k := range keys {
		n, ok := m[k].(float64)
		if !ok || math.IsNaN(n) || math.IsInf(n, 0) {
			return r3.Vector{}, fmt.Errorf("reading %s is not a finite number", k)
		}
		out[i] = n
	}
	return r3.Vector{X: out[0], Y: out[1], Z: out[2]}, nil
}

func readFT(ctx context.Context, src ForceSource) (f, t r3.Vector, err error) {
	m, err := src.Readings(ctx, nil)
	if err != nil {
		return f, t, err
	}
	if _, flat := m["Fx_N"]; flat {
		if f, err = flatVec(m, [3]string{"Fx_N", "Fy_N", "Fz_N"}); err != nil {
			return f, t, err
		}
		t, err = flatVec(m, [3]string{"TRx_Nm", "TRy_Nm", "TRz_Nm"})
		return f, t, err
	}
	if f, err = ftVec(m, "force"); err != nil {
		return f, t, err
	}
	t, err = ftVec(m, "torque")
	return f, t, err
}

// guard polls a force source and trips the loop on contact. Its baseline is taken at start (a software tare),
// so the tool's own weight never counts.
type guard struct {
	cfg    GuardConfig
	src    ForceSource
	f0, t0 r3.Vector
	trip   func(final State, reason string)

	reads, errs         atomic.Int64
	mu                  sync.Mutex
	lastN, peakN, tripN float64
	lastF               r3.Vector // latest F − F₀, sensor frame, N
	reason              string
}

// force is the latest tared force, F − F₀, in the sensor frame.
func (g *guard) force() r3.Vector {
	g.mu.Lock()
	defer g.mu.Unlock()
	return g.lastF
}

// newGuard reads the baseline; a sensor that can't be read fails servo_start.
func newGuard(ctx context.Context, cfg GuardConfig, src ForceSource, trip func(State, string)) (*guard, error) {
	cfg = cfg.withDefaults()
	if err := cfg.validate(); err != nil {
		return nil, err
	}
	g := &guard{cfg: cfg, src: src, trip: trip}
	for range guardBaseline {
		f, t, err := readFT(ctx, src)
		if err != nil {
			return nil, fmt.Errorf("guard: sensor %q: %w", cfg.Sensor, err)
		}
		g.f0, g.t0 = g.f0.Add(f.Mul(1.0/guardBaseline)), g.t0.Add(t.Mul(1.0/guardBaseline))
	}
	return g, nil
}

func (g *guard) run(ctx context.Context) {
	t := time.NewTicker(time.Duration(float64(time.Second) / g.cfg.RateHz))
	defer t.Stop()
	fails := 0
	for {
		select {
		case <-ctx.Done():
			return
		case <-t.C:
		}
		f, tq, err := readFT(ctx, g.src)
		if err != nil {
			if ctx.Err() != nil {
				return
			}
			g.errs.Add(1)
			if fails++; fails >= guardMaxErrRuns {
				g.fire(StateFaulted, fmt.Sprintf("guard: sensor %q: %v", g.cfg.Sensor, err), 0)
				return
			}
			continue
		}
		fails = 0
		g.reads.Add(1)
		fn, tn := f.Sub(g.f0).Norm(), tq.Sub(g.t0).Norm()
		g.mu.Lock()
		g.lastN, g.peakN, g.lastF = fn, max(g.peakN, fn), f.Sub(g.f0)
		g.mu.Unlock()
		switch {
		case g.cfg.ForceN > 0 && fn >= g.cfg.ForceN:
			g.fire(StateStoppedContact, fmt.Sprintf("force %.1f N >= %.1f N", fn, g.cfg.ForceN), fn)
			return
		case g.cfg.TorqueNm > 0 && tn >= g.cfg.TorqueNm:
			g.fire(StateStoppedContact, fmt.Sprintf("torque %.2f N·m >= %.2f N·m", tn, g.cfg.TorqueNm), fn)
			return
		}
	}
}

func (g *guard) fire(final State, reason string, n float64) {
	g.mu.Lock()
	g.tripN, g.reason = n, reason
	g.mu.Unlock()
	g.trip(final, reason)
}

// GuardStats is the guard's view for status.
type GuardStats struct {
	Reads, Errors        int64
	ForceN, PeakN, TripN float64
	Reason               string
}

func (g *guard) stats() GuardStats {
	g.mu.Lock()
	defer g.mu.Unlock()
	return GuardStats{Reads: g.reads.Load(), Errors: g.errs.Load(), ForceN: g.lastN, PeakN: g.peakN, TripN: g.tripN, Reason: g.reason}
}
