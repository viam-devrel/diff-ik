package diffik

import (
	"context"
	"errors"
	"slices"
	"sync"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"gonum.org/v1/gonum/num/quat"
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
func TestForceApproachThenSnap(t *testing.T) {
	l, sim, m := newTestLoop(t, home, LoopConfig{})
	press := 0.0
	// mujoco-sim's convention: a surface pushing back reads −z
	if err := l.SetForce(ForceConfig{Sensor: "ft", TargetN: 10}, func() r3.Vector { return r3.Vector{Z: -press} }); err != nil {
		t.Fatal(err)
	}
	l.SetTarget(offsetTarget(t, m, home, r3.Vector{}, 0))
	l.SetVelocity([6]float64{})
	p0, o, _ := FK(m, home)
	runTicks(t, l, 25)
	p1, _, _ := FK(m, sim.q)
	if d := p1.Sub(p0).Dot(toolAxis(o)) * mmPerM; d < 1 {
		t.Fatalf("moved %.2f mm along the tool axis before contact, want ≈ 2.5", d)
	}
	sim.delay = 3 // a blocked arm trails its command
	runTicks(t, l, 5)
	measured := slices.Clone(sim.q)
	press = 12 // contact, over the target: the law backs off
	runTicks(t, l, 1)
	if st := l.Status(); !st.ForceContact || st.ForcePressN != 12 {
		t.Fatalf("status %+v", st)
	}
	if d := maxAbsDiff(sim.sends[len(sim.sends)-1], measured); d > deg2rad(0.5) {
		t.Fatalf("first contact tick sent %.3f° from the measured joints", rad2deg(d))
	}
	if v := l.force.axisMMPerS(); v != -1 { // 0.5 mm/s/N × (10 − 12)
		t.Fatalf("axis speed %v, want -1", v)
	}
}

type flatFT struct{ fz float64 }

func (f flatFT) Readings(context.Context, map[string]interface{}) (map[string]interface{}, error) {
	return map[string]interface{}{"Fx_N": 0.0, "Fy_N": 0.0, "Fz_N": f.fz, "TRx_Nm": 0.0, "TRy_Nm": 0.0, "TRz_Nm": 0.0}, nil
}

// The xArm sensor's flat keys read like mujoco-sim's maps, and press_axis "+z" flips the pressing sign.
func TestFlatReadingsAndPressSign(t *testing.T) {
	f, tq, err := readFT(context.Background(), flatFT{fz: -18.4})
	if err != nil || f.Z != -18.4 || tq.Norm() != 0 {
		t.Fatalf("flat reading: f %v t %v err %v", f, tq, err)
	}
	if _, _, err := readFT(context.Background(), &fakeFT{}); err != nil {
		t.Fatalf("map reading: %v", err)
	}
	// press_axis: "-z" reads a push back as −z (mujoco-sim), "+z" as +z
	for _, c := range []struct {
		axis string
		fz   float64
	}{{"-z", -5}, {"+z", 5}} {
		fc := &forceCtl{cfg: ForceConfig{TargetN: 10, PressAxis: c.axis}.withDefaults()}
		fc.update(r3.Vector{Z: c.fz}, 0.02)
		if fc.press != 5 {
			t.Fatalf("press_axis %s: press %v from Fz %v, want 5", c.axis, fc.press, c.fz)
		}
	}
	// comply_xy: a wall pushing the tool toward +x (sensor reads +x with "-z") yields toward +x; the tool points down
	// (180° about x), so tool +x is base +x
	fc := &forceCtl{cfg: ForceConfig{TargetN: 10, ComplyXY: 0.5}.withDefaults(), contact: true}
	fc.update(r3.Vector{X: 4, Z: -10}, 0.02)
	down := quat.Number{Imag: 1} // 180° about x
	if v := fc.overlay([6]float64{}, down); !(v[0] > 0.0019 && v[0] < 0.0021) || v[1] != 0 {
		t.Fatalf("comply velocity %v m/s, want +x 0.002 (0.5 mm/s/N × 4 N)", v[:3])
	}
	if err := (ForceConfig{Sensor: "ft", TargetN: 10, PressAxis: "z"}).withDefaults().validate(); err == nil {
		t.Fatal(`press_axis "z" accepted`)
	}
}
