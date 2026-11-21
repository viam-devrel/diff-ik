package diffik

import (
	"context"
	"errors"
	"slices"
	"sync"
	"testing"
	"time"

	"github.com/golang/geo/r3"
)

// A trip snaps the command back to the measured joints (an arm blocked behind its command) and ends the loop.
func TestTripHoldsMeasuredJoints(t *testing.T) {
	l, sim, m := newTestLoop(t, home, LoopConfig{})
	sim.delay = 3 // the arm trails the command, as when it is pressed against a surface
	l.SetVelocity([6]float64{0, 0, -0.05})
	l.SetTarget(offsetTarget(t, m, home, r3.Vector{Z: -50}, 0))
	runTicks(t, l, 10)
	measured := slices.Clone(sim.q)
	l.Trip(StateStoppedContact)
	if err := l.Run(context.Background()); err != nil {
		t.Fatal(err)
	}
	last := sim.sends[len(sim.sends)-1]
	if !slices.Equal(last, measured) || !sim.stopped || l.Status().State != StateStoppedContact {
		t.Fatalf("last send %v, measured %v, stopped %v, state %s", last, measured, sim.stopped, l.Status().State)
	}
}

type fakeFT struct {
	mu   sync.Mutex
	fz   float64
	err  error
	read int
}

func (f *fakeFT) Readings(context.Context, map[string]interface{}) (map[string]interface{}, error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.read++
	if f.err != nil {
		return nil, f.err
	}
	return map[string]interface{}{
		"force":  map[string]interface{}{"x": 0.0, "y": 0.0, "z": f.fz},
		"torque": map[string]interface{}{"x": 0.0, "y": 0.0, "z": 0.0},
	}, nil
}

func (f *fakeFT) set(fz float64, err error) {
	f.mu.Lock()
	f.fz, f.err = fz, err
	f.mu.Unlock()
}

// The baseline (tool weight) never trips; a rise past force_n does, and a blind sensor faults.
func TestGuardTripsOnContactAndFaultsBlind(t *testing.T) {
	src := &fakeFT{fz: 7.4}
	trips := make(chan State, 1)
	g, err := newGuard(context.Background(), GuardConfig{Sensor: "ft", ForceN: 5, RateHz: 1000}, src,
		func(s State, _ string) { trips <- s })
	if err != nil {
		t.Fatal(err)
	}
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	go g.run(ctx)
	time.Sleep(20 * time.Millisecond)
	select {
	case s := <-trips:
		t.Fatalf("tripped on the baseline: %s", s)
	default:
	}
	src.set(7.4+6, nil)
	if s := <-trips; s != StateStoppedContact || g.stats().TripN < 5 {
		t.Fatalf("state %s trip %.1f N", s, g.stats().TripN)
	}

	g2, _ := newGuard(context.Background(), GuardConfig{Sensor: "ft", ForceN: 5, RateHz: 1000}, src,
		func(s State, _ string) { trips <- s })
	src.set(0, errors.New("gone"))
	go g2.run(ctx)
	if s := <-trips; s != StateFaulted {
		t.Fatalf("blind sensor: state %s", s)
	}
}

// Force control: the tool approaches along its axis, then at first contact the command snaps back to the measured
// joints and the axis speed follows the force law.

type flatFT struct{ fz float64 }

func (f flatFT) Readings(context.Context, map[string]interface{}) (map[string]interface{}, error) {
	return map[string]interface{}{"Fx_N": 0.0, "Fy_N": 0.0, "Fz_N": f.fz, "TRx_Nm": 0.0, "TRy_Nm": 0.0, "TRz_Nm": 0.0}, nil
}

// The xArm sensor's flat keys read like mujoco-sim's maps.
func TestFlatReadings(t *testing.T) {
	f, tq, err := readFT(context.Background(), flatFT{fz: -18.4})
	if err != nil || f.Z != -18.4 || tq.Norm() != 0 {
		t.Fatalf("flat reading: f %v t %v err %v", f, tq, err)
	}
	if _, _, err := readFT(context.Background(), &fakeFT{}); err != nil {
		t.Fatalf("map reading: %v", err)
	}
}
