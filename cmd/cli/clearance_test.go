package main

import (
	"math"
	"os"
	"path/filepath"
	"testing"

	"diffik"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

func TestInterp(t *testing.T) {
	ss := []propSample{
		{t: 10, pos: r3.Vector{X: 0}, q: [4]float64{1, 0, 0, 0}, size: r3.Vector{X: 1, Y: 1, Z: 1}},
		{t: 12, pos: r3.Vector{X: 100}, q: [4]float64{math.Cos(math.Pi / 4), 0, 0, math.Sin(math.Pi / 4)}},
	}
	s, ok := interp(ss, 11)
	if !ok || math.Abs(s.pos.X-50) > 1e-9 || math.Abs(s.q[0]-math.Cos(math.Pi/8)) > 1e-9 || math.Abs(s.q[3]-math.Sin(math.Pi/8)) > 1e-9 {
		t.Fatalf("%+v %v", s, ok)
	}
	if s, ok := interp(ss, 10); !ok || s.pos.X != 0 {
		t.Fatalf("at start: %+v %v", s, ok)
	}
	for _, x := range []float64{9.9, 12.1} {
		if _, ok := interp(ss, x); ok {
			t.Fatalf("t=%v is outside the span", x)
		}
	}
}

func TestClearanceSynthetic(t *testing.T) {
	m, err := referenceframe.UnmarshalModelJSON(mustRead(t, "../../testdata/panda.json"), "arm")
	if err != nil {
		t.Fatal(err)
	}
	armPose := spatialmath.NewPoseFromPoint(r3.Vector{X: 100, Y: 200})
	parts := []*referenceframe.FrameSystemPart{{FrameConfig: referenceframe.NewLinkInFrame(referenceframe.World, armPose, "arm", nil), ModelFrame: m}}
	q := []float64{0, -0.3, 0, -2.0, 0, 1.8, 0.785}
	pm, err := diffik.NewPartsModel(parts, m, "arm", q, nil)
	if err != nil {
		t.Fatal(err)
	}
	ee, err := m.Transform(q)
	if err != nil {
		t.Fatal(err)
	}
	eeW := spatialmath.Compose(armPose, ee).Point()
	// the prop slides from 800 mm above the end effector down onto it between t=0 and t=10
	size := r3.Vector{X: 50, Y: 50, Z: 50}
	props := map[string][]propSample{"p": {
		{t: 0, pos: eeW.Add(r3.Vector{Z: 800}), q: [4]float64{1, 0, 0, 0}, size: size},
		{t: 10, pos: eeW, q: [4]float64{1, 0, 0, 0}, size: size},
	}}
	rows := []runRow{{t: -1, q: q}, {t: 0, q: q}, {t: 5, q: q}, {t: 10, q: q}, {t: 11, q: q}}
	res, err := clearance(rows, props, pm)
	if err != nil {
		t.Fatal(err)
	}
	p := res["p"]
	if p.n != 3 || p.t != 10 || p.d > 0 || p.part == "" || res[""] != p {
		t.Fatalf("%+v", p)
	}
}

func mustRead(t *testing.T, p string) []byte {
	t.Helper()
	b, err := os.ReadFile(p)
	if err != nil {
		t.Fatal(err)
	}
	return b
}

func TestLoadProps(t *testing.T) {
	p := filepath.Join(t.TempDir(), "props.csv")
	os.WriteFile(p, []byte("t,name,x,y,z,qw,qx,qy,qz,sx,sy,sz\n2,a,1,2,3,1,0,0,0,4,5,6\n1,a,0,0,0,1,0,0,0,4,5,6\n"), 0o644)
	got, err := loadProps(p)
	if err != nil || len(got["a"]) != 2 || got["a"][0].t != 1 || got["a"][1].size.Z != 6 {
		t.Fatalf("%+v %v", got, err)
	}
}

// Hand-computed: an 80 x 80 x 120 box (full dims) against a 1 mm sphere. Fails for half-dims or a wrong quaternion order.
func TestPropBoxHandComputed(t *testing.T) {
	size := r3.Vector{X: 80, Y: 80, Z: 120}
	rz45 := [4]float64{math.Cos(math.Pi / 8), 0, 0, math.Sin(math.Pi / 8)} // w, x, y, z
	dist := func(q [4]float64, at r3.Vector) float64 {
		g, err := propSample{pos: r3.Vector{}, q: q, size: size}.box("p")
		if err != nil {
			t.Fatal(err)
		}
		s, err := spatialmath.NewSphere(spatialmath.NewPoseFromPoint(at), 1, "s")
		if err != nil {
			t.Fatal(err)
		}
		d, err := g.DistanceFrom(s)
		if err != nil {
			t.Fatal(err)
		}
		return d
	}
	for _, c := range []struct {
		name string
		q    [4]float64
		at   r3.Vector
		want float64
	}{
		{"rotated 45 about z", rz45, r3.Vector{X: 60}, 60 - 40*math.Sqrt2 - 1},
		{"unrotated", [4]float64{1, 0, 0, 0}, r3.Vector{X: 60}, 19},
		{"above", [4]float64{1, 0, 0, 0}, r3.Vector{Z: 100}, 39},
	} {
		if d := dist(c.q, c.at); math.Abs(d-c.want) > 1e-6 {
			t.Errorf("%s: %v, want %v", c.name, d, c.want)
		}
	}
}

// A rotated, offset arm base: the replay must match the frame system's own geometries, in world.
func TestClearanceRotatedBaseMatchesFrameSystem(t *testing.T) {
	m, err := referenceframe.UnmarshalModelJSON(mustRead(t, "../../testdata/panda.json"), "arm")
	if err != nil {
		t.Fatal(err)
	}
	armPose := spatialmath.NewPose(r3.Vector{X: 150, Y: -80, Z: 20}, &spatialmath.OrientationVectorDegrees{OZ: 1, Theta: 70})
	parts := []*referenceframe.FrameSystemPart{{FrameConfig: referenceframe.NewLinkInFrame(referenceframe.World, armPose, "arm", nil), ModelFrame: m}}
	q := []float64{0.3, -0.3, 0.1, -2.0, 0.2, 1.8, 0.785}
	pm, err := diffik.NewPartsModel(parts, m, "arm", q, nil)
	if err != nil {
		t.Fatal(err)
	}
	fsys, err := referenceframe.NewFrameSystem("t", parts, nil)
	if err != nil {
		t.Fatal(err)
	}
	in := referenceframe.NewZeroInputs(fsys)
	in["arm"] = q
	gs, err := referenceframe.FrameSystemGeometries(fsys, in)
	if err != nil {
		t.Fatal(err)
	}
	sample := propSample{pos: r3.Vector{X: 300, Y: 100, Z: 250}, q: [4]float64{math.Cos(0.4), 0.3 * math.Sin(0.4), 0, math.Sin(0.4)}, size: r3.Vector{X: 80, Y: 80, Z: 120}}
	n := math.Sqrt(sample.q[0]*sample.q[0] + sample.q[1]*sample.q[1] + sample.q[3]*sample.q[3])
	for i := range sample.q {
		sample.q[i] /= n
	}
	prop, err := sample.box("p")
	if err != nil {
		t.Fatal(err)
	}
	want, count := math.Inf(1), 0
	for _, gif := range gs {
		for _, g := range gif.Geometries() {
			d, err := g.DistanceFrom(prop)
			if err != nil {
				t.Fatal(err)
			}
			want, count = math.Min(want, d), count+1
		}
	}
	if count == 0 {
		t.Fatal("no reference geometries")
	}
	sample.t = 1
	res, err := clearance([]runRow{{t: 1, q: q}}, map[string][]propSample{"p": {sample}}, pm)
	if err != nil {
		t.Fatal(err)
	}
	if got := res[""].d; math.Abs(got-want) > 1e-6 {
		t.Fatalf("replay %v, frame system %v", got, want)
	}
}
