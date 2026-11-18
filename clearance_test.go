package diffik

import (
	"context"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/vision/detection3d"
)

func pandaParts(t *testing.T) (referenceframe.Model, []*referenceframe.FrameSystemPart) {
	m := panda(t)
	arm := &referenceframe.FrameSystemPart{FrameConfig: referenceframe.NewLinkInFrame(referenceframe.World, lvArmPose, "arm", nil), ModelFrame: m}
	return m, []*referenceframe.FrameSystemPart{arm, staticPart(referenceframe.World, "cam", lvCamPose, nil)}
}

func TestStartLiveStats(t *testing.T) {
	m, parts := pandaParts(t)
	l, err := NewLoop(m, nil, LoopConfig{Solver: SolverQP})
	if err != nil {
		t.Fatal(err)
	}
	cm, err := NewCollisionModel(parts[:1], m, "arm", CollisionRequest{}, pandaHold)
	if err != nil {
		t.Fatal(err)
	}
	if err := l.SetCollision(cm); err != nil {
		t.Fatal(err)
	}
	src := &fakeSrc{props: props3D}
	src.dets = []*detection3d.Detection{det3("cam", spatialmath.NewPoseFromPoint(r3.Vector{Z: 400}), box(t, r3.Vector{X: 40, Y: 40, Z: 40}), nil, nil, ""),
		{}} // a malformed one: skipped
	h, err := StartLive(context.Background(), l, src, &fakeJoints{q: pandaHold}, parts, m, "arm", pandaHold, LiveConfig{Vision: "v", Camera: "cam", RateHz: 50})
	if err != nil {
		t.Fatal(err)
	}
	defer h.Stop()
	var st LiveStats
	for i := 0; i < 200 && st.Updates < 2; i++ {
		time.Sleep(10 * time.Millisecond)
		st = h.Stats()
	}
	if st.Updates < 2 || st.Skipped < 2 || st.Method != "detections3d" || st.LastError != "" {
		t.Fatalf("stats %+v", st)
	}
	h.Stop()
	h.Stop()
}

func TestPartsModelMinDistance(t *testing.T) {
	m, parts := pandaParts(t)
	pm, err := NewPartsModel(parts, m, "arm", pandaHold, nil)
	if err != nil {
		t.Fatal(err)
	}
	ee := spatialmath.Compose(lvArmPose, spatialmath.NewPoseFromPoint(eePoint(t, m, pandaHold))).Point()
	atEE := box(t, r3.Vector{X: 100, Y: 100, Z: 100}).Transform(spatialmath.NewPoseFromPoint(ee))
	d, part, err := pm.MinDistance(pandaHold, []spatialmath.Geometry{atEE.Transform(pm.WorldToBase())})
	if err != nil {
		t.Fatal(err)
	}
	if d > 0 || part == "" {
		t.Fatalf("box on the end effector: d %v part %q", d, part)
	}
	// a box moved 1 m away in world z gets a distance of about 1 m less its half height and the part's reach
	far := box(t, r3.Vector{X: 100, Y: 100, Z: 100}).Transform(spatialmath.NewPoseFromPoint(r3.Vector{X: ee.X, Y: ee.Y, Z: ee.Z + 1000}))
	d2, _, err := pm.MinDistance(pandaHold, []spatialmath.Geometry{far.Transform(pm.WorldToBase())})
	if err != nil {
		t.Fatal(err)
	}
	if d2 < 700 || d2 > 1000 {
		t.Fatalf("far box distance %v", d2)
	}
}

func TestStartLiveNeedsQPAndCollision(t *testing.T) {
	m, parts := pandaParts(t)
	for name, cfg := range map[string]LoopConfig{"no collision": {Solver: SolverQP}, "dls": {}} {
		l, err := NewLoop(m, nil, cfg)
		if err != nil {
			t.Fatal(err)
		}
		if h, err := StartLive(context.Background(), l, &fakeSrc{props: props3D}, &fakeJoints{q: pandaHold}, parts, m, "arm", pandaHold, LiveConfig{Vision: "v", Camera: "cam"}); err == nil {
			h.Stop()
			t.Errorf("%s: StartLive succeeded", name)
		}
	}
}
