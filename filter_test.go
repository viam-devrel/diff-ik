package diffik

import (
	"math"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

func down(x, y, z float64) spatialmath.Pose {
	return spatialmath.NewPose(r3.Vector{X: x, Y: y, Z: z}, &spatialmath.OrientationVectorDegrees{OZ: -1})
}

func TestFilterEMASeedsAndConverges(t *testing.T) {
	f := newTargetFilter(TargetFilter{EMAAlpha: 0.5, Feedforward: true})
	t0 := time.Unix(0, 0)
	out, _ := f.apply(t0, down(0, 0, 100))
	if out.Point().Z != 100 {
		t.Fatalf("first sample seeds the EMA: got z %v", out.Point().Z)
	}
	out, _ = f.apply(t0.Add(20*time.Millisecond), down(0, 0, 200))
	if math.Abs(out.Point().Z-150) > 1e-9 {
		t.Fatalf("alpha 0.5 step: got z %v, want 150", out.Point().Z)
	}
}

func TestFilterOffIsIdentity(t *testing.T) {
	f := newTargetFilter(DefaultTargetFilter())
	p := down(10, 20, 30)
	out, clamped := f.apply(time.Unix(0, 0), p)
	if !spatialmath.PoseAlmostEqual(out, p) || clamped {
		t.Fatal("the default filter must pass poses through")
	}
}

func TestFilterHoldsLatestAndNeverStepsBack(t *testing.T) {
	f := newTargetFilter(TargetFilter{EMAAlpha: 1, HoldMM: 1, HoldDeg: 0.5, Dwell: 200 * time.Millisecond, Feedforward: true})
	t0 := time.Unix(0, 0)
	var xs []float64
	for k := 0; k < 100; k++ { // 3 mm/s drift for 2 s
		out, _ := f.apply(t0.Add(time.Duration(k)*20*time.Millisecond), down(200+0.06*float64(k), 0, 150))
		xs = append(xs, out.Point().X)
	}
	for i := 1; i < len(xs); i++ {
		if xs[i] < xs[i-1] {
			t.Fatalf("target stepped backwards at %d: %v -> %v", i, xs[i-1], xs[i])
		}
	}
	// still: after the dwell, the same pose is returned every call
	var last float64
	for k := 0; k < 20; k++ {
		out, _ := f.apply(t0.Add(3*time.Second+time.Duration(k)*20*time.Millisecond), down(210+0.3*float64(k%2), 0, 150))
		if k > 12 && out.Point().X != last {
			t.Fatalf("not holding: %v vs %v", out.Point().X, last)
		}
		last = out.Point().X
	}
}

func TestFilterFloorClamp(t *testing.T) {
	f := newTargetFilter(TargetFilter{EMAAlpha: 1, Floor: &FloorClamp{MinZMM: -10, TipMM: 105, MarginMM: 15}, Feedforward: true})
	// tool straight down, flange at z 100: tip at -5 < +5 -> lift by 10
	out, clamped := f.apply(time.Unix(0, 0), down(200, 0, 100))
	if !clamped || math.Abs(out.Point().Z-110) > 1e-9 {
		t.Fatalf("got z %v clamped %v, want 110 true", out.Point().Z, clamped)
	}
	// 20 mm hover over a tag on the table: flange at 125, tip at 20 -> no clamp
	if _, clamped := f.apply(time.Unix(1, 0), down(200, 0, 125)); clamped {
		t.Fatal("a 20 mm hover must not be clamped")
	}
	if f.clamps != 1 {
		t.Fatalf("clamps = %d, want 1", f.clamps)
	}
}

func TestTargetFilterValidate(t *testing.T) {
	for _, bad := range []TargetFilter{{EMAAlpha: 0}, {EMAAlpha: 1.5}, {EMAAlpha: 1, HoldMM: -1},
		{EMAAlpha: 1, Floor: &FloorClamp{TipMM: -1}}} {
		if bad.validate() == nil {
			t.Errorf("%+v accepted", bad)
		}
	}
	if err := DefaultTargetFilter().validate(); err != nil {
		t.Fatal(err)
	}
}
