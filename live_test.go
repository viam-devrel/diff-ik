package diffik

import (
	"context"
	"fmt"
	"math"
	"sort"
	"sync"
	"sync/atomic"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/spatialmath"
	viz "go.viam.com/rdk/vision"
	"go.viam.com/rdk/vision/classification"
	"go.viam.com/rdk/vision/detection3d"
)

type fakeSrc struct {
	props    vision.Properties
	dets     []*detection3d.Detection
	objs     []*viz.Object
	err      error
	doPanic  bool
	block    bool
	delay    time.Duration
	calls    atomic.Int64
	inflight atomic.Int64
	maxIn    atomic.Int64
	lastCam  atomic.Value
}

func (f *fakeSrc) GetProperties(context.Context, map[string]interface{}) (*vision.Properties, error) {
	p := f.props
	return &p, nil
}

func (f *fakeSrc) enter(ctx context.Context, cam string) error {
	f.calls.Add(1)
	f.lastCam.Store(cam)
	n := f.inflight.Add(1)
	for m := f.maxIn.Load(); n > m && !f.maxIn.CompareAndSwap(m, n); m = f.maxIn.Load() {
	}
	defer f.inflight.Add(-1)
	if f.block {
		<-ctx.Done()
		return ctx.Err()
	}
	if f.doPanic {
		panic("boom")
	}
	time.Sleep(f.delay)
	return f.err
}

func (f *fakeSrc) GetDetections3D(ctx context.Context, cam string, _ map[string]interface{}) ([]*detection3d.Detection, error) {
	if err := f.enter(ctx, cam); err != nil {
		return nil, err
	}
	return f.dets, nil
}

func (f *fakeSrc) GetObjectPointClouds(ctx context.Context, cam string, _ map[string]interface{}) ([]*viz.Object, error) {
	if err := f.enter(ctx, cam); err != nil {
		return nil, err
	}
	return f.objs, nil
}

type fakeJoints struct {
	mu sync.Mutex
	q  []float64
}

func (j *fakeJoints) Joints(context.Context) ([]float64, error) {
	j.mu.Lock()
	defer j.mu.Unlock()
	return append([]float64(nil), j.q...), nil
}

func (j *fakeJoints) set(q []float64) { j.mu.Lock(); j.q = q; j.mu.Unlock() }

type sink struct {
	mu   sync.Mutex
	sets [][]liveObstacle
}

func (s *sink) publish(o []liveObstacle) { s.mu.Lock(); s.sets = append(s.sets, o); s.mu.Unlock() }
func (s *sink) n() int                   { s.mu.Lock(); defer s.mu.Unlock(); return len(s.sets) }
func (s *sink) last(t *testing.T) []liveObstacle {
	t.Helper()
	s.mu.Lock()
	defer s.mu.Unlock()
	if len(s.sets) == 0 {
		t.Fatal("nothing published")
	}
	return s.sets[len(s.sets)-1]
}

var (
	lvArmPose = spatialmath.NewPose(r3.Vector{X: 100, Y: 200, Z: 0}, &spatialmath.OrientationVectorDegrees{OZ: 1, Theta: 90})
	lvCamPose = spatialmath.NewPose(r3.Vector{X: 500, Y: -50, Z: 700}, &spatialmath.OrientationVectorDegrees{OX: 0.3, OY: 0.2, OZ: -1, Theta: 25})
)

type lvEnv struct {
	p   *poller
	src *fakeSrc
	out *sink
	j   *fakeJoints
	m   referenceframe.Model
}

// newLV builds a poller over the Panda at armPose, with a fixed camera frame "cam" and a wrist camera "wcam".
// The camera resource name is "camera", which isn't a frame.
func newLV(t *testing.T, armPose spatialmath.Pose, cfg LiveConfig, props vision.Properties) *lvEnv {
	t.Helper()
	m := panda(t)
	arm := &referenceframe.FrameSystemPart{
		FrameConfig: referenceframe.NewLinkInFrame(referenceframe.World, armPose, "arm", nil), ModelFrame: m}
	wcam := spatialmath.NewPose(r3.Vector{X: 10, Y: 20, Z: 30}, &spatialmath.OrientationVectorDegrees{OX: 1, OZ: 1, Theta: 40})
	parts := []*referenceframe.FrameSystemPart{arm,
		staticPart(referenceframe.World, "cam", lvCamPose, nil),
		staticPart("arm", "wcam", wcam, nil)}
	e := &lvEnv{src: &fakeSrc{props: props}, out: &sink{}, j: &fakeJoints{q: pandaHold}, m: m}
	if cfg.Camera == "" {
		cfg.Camera = "camera"
	}
	var err error
	e.p, err = newPoller(context.Background(), cfg, e.src, e.j, parts, m, "arm", pandaHold, e.out.publish)
	if err != nil {
		t.Fatal(err)
	}
	return e
}

var props3D = vision.Properties{Detections3DSupported: true}

func fp(x float64) *float64 { return &x }

func idPose() spatialmath.Pose { return spatialmath.NewZeroPose() }

func box(t *testing.T, dims r3.Vector) spatialmath.Geometry { return mustBox(t, dims) }

func cloud(pts ...r3.Vector) pointcloud.PointCloud {
	pc := pointcloud.NewBasicEmpty()
	for _, p := range pts {
		_ = pc.Set(p, pointcloud.NewBasicData())
	}
	return pc
}

// det3 is a detection in frame parent: a root at pose carrying g, and optionally a points child at childPose.
func det3(parent string, pose spatialmath.Pose, g spatialmath.Geometry, pts pointcloud.PointCloud, childPose spatialmath.Pose, label string) *detection3d.Detection {
	d := &detection3d.Detection{Transforms: []*referenceframe.LinkInFrame{referenceframe.NewLinkInFrame(parent, pose, "d", g)}}
	if pts != nil {
		oct, err := pointcloud.ToBasicOctree(pts, 0)
		if err != nil {
			panic(err)
		}
		d.Transforms = append(d.Transforms, referenceframe.NewLinkInFrame("d", childPose, "d/points", oct))
	}
	if label != "" {
		d.Classifications = classification.Classifications{classification.NewClassification(0.9, label)}
	}
	return d
}

func lnear(a, b r3.Vector, eps float64) bool { return a.Sub(b).Norm() < eps }

func cfgOf(t *testing.T, g spatialmath.Geometry) *spatialmath.GeometryConfig {
	t.Helper()
	c, err := spatialmath.NewGeometryConfig(g)
	if err != nil {
		t.Fatal(err)
	}
	return c
}

func sortedDims(c *spatialmath.GeometryConfig) []float64 {
	d := []float64{c.X, c.Y, c.Z}
	sort.Float64s(d)
	return d
}

func TestLiveConfigValidate(t *testing.T) {
	ok := LiveConfig{}.withDefaults()
	if err := ok.validate(); err != nil {
		t.Fatal(err)
	}
	if ok.RateHz != 10 || ok.StaleMS != 500 || ok.TimeoutMS != 250 || *ok.InflateMM != 10 || ok.Max != 32 || ok.MinPoints != 10 {
		t.Fatalf("defaults %+v", ok)
	}
	lo, hi := r3.Vector{Z: 20}, r3.Vector{X: 1, Y: 1, Z: 600}
	z := 700.0
	nan := math.NaN()
	for name, mut := range map[string]func(c *LiveConfig){
		"rate 0":             func(c *LiveConfig) { c.RateHz = -1 },
		"rate 51":            func(c *LiveConfig) { c.RateHz = 51 },
		"stale":              func(c *LiveConfig) { c.StaleMS = 100 },
		"stale<poll+timeout": func(c *LiveConfig) { c.RateHz, c.StaleMS, c.TimeoutMS = 10, 300, 250 },
		"timeout":            func(c *LiveConfig) { c.TimeoutMS = 600 },
		"inflate":            func(c *LiveConfig) { c.InflateMM = fp(-1) },
		"max":                func(c *LiveConfig) { c.Max = 65 },
		"min points":         func(c *LiveConfig) { c.MinPoints = -1 },
		"one corner":         func(c *LiveConfig) { c.WorkspaceMin = &lo },
		"inverted":           func(c *LiveConfig) { c.WorkspaceMin, c.WorkspaceMax = &hi, &lo },
		"extrude out":        func(c *LiveConfig) { c.WorkspaceMin, c.WorkspaceMax, c.ExtrudeToZ = &lo, &hi, &z },
		"nan":                func(c *LiveConfig) { c.InflateMM = fp(nan) },
	} {
		c := ok
		mut(&c)
		if c.validate() == nil {
			t.Errorf("%s accepted", name)
		}
	}
}

func TestLiveMethodChoice(t *testing.T) {
	for _, tc := range []struct {
		props vision.Properties
		want3 bool
		bad   bool
	}{
		{vision.Properties{Detections3DSupported: true}, true, false},
		{vision.Properties{ObjectPCDsSupported: true}, false, false},
		{vision.Properties{Detections3DSupported: true, ObjectPCDsSupported: true}, true, false},
		{vision.Properties{}, false, true},
	} {
		m := panda(t)
		_, err := newPoller(context.Background(), LiveConfig{Camera: "cam"}, &fakeSrc{props: tc.props}, &fakeJoints{q: pandaHold},
			[]*referenceframe.FrameSystemPart{armPart(m, "arm"), staticPart(referenceframe.World, "cam", lvCamPose, nil)}, m, "arm", pandaHold, func([]liveObstacle) {})
		if (err != nil) != tc.bad {
			t.Fatalf("%+v: err %v", tc.props, err)
		}
	}
	// the camera must not sit on a movable frame other than the arm
	m := panda(t)
	m2 := panda(t)
	parts := []*referenceframe.FrameSystemPart{armPart(m, "arm"),
		{FrameConfig: referenceframe.NewLinkInFrame(referenceframe.World, idPose(), "arm2", nil), ModelFrame: m2},
		staticPart("arm2", "cam", idPose(), nil)}
	if _, err := newPoller(context.Background(), LiveConfig{Camera: "cam"}, &fakeSrc{props: props3D}, &fakeJoints{q: pandaHold}, parts, m, "arm", pandaHold, func([]liveObstacle) {}); err == nil {
		t.Fatal("camera on another arm accepted")
	}
}

func TestLive3DDetections(t *testing.T) {
	e := newLV(t, lvArmPose, LiveConfig{}, props3D)
	rootPose := spatialmath.NewPose(r3.Vector{X: 30, Y: -20, Z: 400}, &spatialmath.OrientationVectorDegrees{OX: 0.2, OZ: 1, Theta: 33})
	childPose := spatialmath.NewPose(r3.Vector{X: 5, Y: 6, Z: 7}, &spatialmath.OrientationVectorDegrees{OY: 0.4, OZ: 1, Theta: 12})
	dims := r3.Vector{X: 40, Y: 50, Z: 60}
	local := []r3.Vector{{X: 1, Y: 2, Z: 3}, {X: -20, Y: 10, Z: 5}, {X: 15, Y: -9, Z: -22}, {X: 3, Y: 30, Z: 8}}
	e.src.dets = []*detection3d.Detection{det3("cam", rootPose, box(t, dims), cloud(local...), childPose, "cup")}
	e.p.pollOnce(context.Background())
	set := e.out.last(t)
	if len(set) != 1 || set[0].label != "cup" {
		t.Fatalf("set %+v", set)
	}
	if cam, _ := e.src.lastCam.Load().(string); cam != "camera" {
		t.Fatalf("camera name passed: %q", cam)
	}
	// expected: arm_origin <- world <- cam <- root; the inverse of the arm's world pose
	toBase := func(inCam spatialmath.Pose) spatialmath.Pose {
		return spatialmath.Compose(spatialmath.PoseInverse(lvArmPose), spatialmath.Compose(lvCamPose, inCam))
	}
	want := toBase(rootPose)
	if !spatialmath.PoseAlmostEqualEps(set[0].geom.Pose(), want, 1e-6) {
		t.Fatalf("pose %v, want %v", spatialmath.PoseToProtobuf(set[0].geom.Pose()), spatialmath.PoseToProtobuf(want))
	}
	c := cfgOf(t, set[0].geom)
	if c.Type != spatialmath.BoxType || math.Abs(c.X-60) > 1e-9 || math.Abs(c.Y-70) > 1e-9 || math.Abs(c.Z-80) > 1e-9 {
		t.Fatalf("box %+v", c)
	}
	// its points: with a workspace, the refit box is the world AABB of the points through cam, root and the child
	lo, hi := r3.Vector{X: -5000, Y: -5000, Z: -5000}, r3.Vector{X: 5000, Y: 5000, Z: 5000}
	e2 := newLV(t, lvArmPose, LiveConfig{WorkspaceMin: &lo, WorkspaceMax: &hi, MinPoints: 4}, props3D)
	e2.src.dets = e.src.dets
	e2.p.pollOnce(context.Background())
	var wp []r3.Vector
	for _, v := range local {
		wp = append(wp, spatialmath.Compose(lvCamPose, spatialmath.Compose(rootPose, spatialmath.Compose(childPose, spatialmath.NewPoseFromPoint(v)))).Point())
	}
	wl, wh := bounds(wp)
	wantC := spatialmath.Compose(spatialmath.PoseInverse(lvArmPose), spatialmath.NewPoseFromPoint(wl.Add(wh).Mul(0.5))).Point()
	g := e2.out.last(t)[0].geom
	if !lnear(g.Pose().Point(), wantC, 1e-6) {
		t.Fatalf("refit centre %v, want %v", g.Pose().Point(), wantC)
	}
	d := wh.Sub(wl)
	got := sortedDims(cfgOf(t, g))
	exp := []float64{d.X + 20, d.Y + 20, d.Z + 20}
	sort.Float64s(exp)
	for i := range got {
		if math.Abs(got[i]-exp[i]) > 1e-6 {
			t.Fatalf("refit dims %v, want %v", got, exp)
		}
	}
}

func TestLiveObjects(t *testing.T) {
	e := newLV(t, lvArmPose, LiveConfig{Camera: "cam"}, vision.Properties{ObjectPCDsSupported: true})
	gp := spatialmath.NewPose(r3.Vector{X: 10, Y: 20, Z: 500}, &spatialmath.OrientationVectorDegrees{OZ: 1, Theta: 20})
	g, err := spatialmath.NewBox(gp, r3.Vector{X: 30, Y: 30, Z: 30}, "thing")
	if err != nil {
		t.Fatal(err)
	}
	e.src.objs = []*viz.Object{{PointCloud: cloud(r3.Vector{X: 10, Y: 20, Z: 500}), Geometry: g}}
	e.p.pollOnce(context.Background())
	set := e.out.last(t)
	want := spatialmath.Compose(spatialmath.PoseInverse(lvArmPose), spatialmath.Compose(lvCamPose, gp))
	if len(set) != 1 || set[0].label != "thing" || !spatialmath.PoseAlmostEqualEps(set[0].geom.Pose(), want, 1e-6) {
		t.Fatalf("set %+v", set)
	}
	if cam, _ := e.src.lastCam.Load().(string); cam != "cam" {
		t.Fatalf("camera name passed: %q", cam)
	}
}

func TestLiveWristCamera(t *testing.T) {
	e := newLV(t, idPose(), LiveConfig{}, props3D)
	wcam := spatialmath.NewPose(r3.Vector{X: 10, Y: 20, Z: 30}, &spatialmath.OrientationVectorDegrees{OX: 1, OZ: 1, Theta: 40})
	inCam := spatialmath.NewPose(r3.Vector{X: 0, Y: 0, Z: 300}, &spatialmath.OrientationVectorDegrees{OZ: 1, Theta: 10})
	e.src.dets = []*detection3d.Detection{det3("wcam", inCam, box(t, r3.Vector{X: 20, Y: 20, Z: 20}), nil, nil, "")}
	var prev r3.Vector
	for i, q := range [][]float64{pandaHold, {0.5, -0.1, 0.2, -1.8, 0.3, 1.5, 0.4}} {
		e.j.set(q)
		e.p.pollOnce(context.Background())
		ee, err := e.m.Transform(q)
		if err != nil {
			t.Fatal(err)
		}
		want := spatialmath.Compose(ee, spatialmath.Compose(wcam, inCam))
		got := e.out.last(t)[0].geom.Pose()
		if !spatialmath.PoseAlmostEqualEps(got, want, 1e-6) {
			t.Fatalf("q%d: pose %v, want %v", i, got.Point(), want.Point())
		}
		if i == 1 && lnear(got.Point(), prev, 1) {
			t.Fatal("obstacle did not follow the arm")
		}
		prev = got.Point()
	}
}

func TestLiveInflation(t *testing.T) {
	e := newLV(t, idPose(), LiveConfig{InflateMM: fp(7)}, props3D)
	pose := spatialmath.NewPose(r3.Vector{X: 400, Y: 100, Z: 300}, &spatialmath.OrientationVectorDegrees{OX: 0.3, OZ: 1, Theta: 20})
	sph, _ := spatialmath.NewSphere(idPose(), 30, "")
	cap, _ := spatialmath.NewCapsule(idPose(), 20, 100, "")
	e.src.dets = []*detection3d.Detection{
		det3("world", pose, box(t, r3.Vector{X: 40, Y: 50, Z: 60}), nil, nil, "b"),
		det3("world", pose, sph, nil, nil, "s"),
		det3("world", pose, cap, nil, nil, "c"),
	}
	e.p.pollOnce(context.Background())
	byLabel := map[string]liveObstacle{}
	for _, o := range e.out.last(t) {
		byLabel[o.label] = o
		if !spatialmath.PoseAlmostEqualEps(o.geom.Pose(), pose, 1e-6) || o.geom.Label() != o.label {
			t.Fatalf("%s: pose or label lost", o.label)
		}
	}
	if len(byLabel) != 3 {
		t.Fatalf("got %d", len(byLabel))
	}
	if c := cfgOf(t, byLabel["b"].geom); c.X != 54 || c.Y != 64 || c.Z != 74 {
		t.Fatalf("box %+v", c)
	}
	if c := cfgOf(t, byLabel["s"].geom); c.R != 37 {
		t.Fatalf("sphere %+v", c)
	}
	if c := cfgOf(t, byLabel["c"].geom); c.R != 27 || c.L != 114 {
		t.Fatalf("capsule %+v", c)
	}
}

func TestLiveMaxCap(t *testing.T) {
	e := newLV(t, idPose(), LiveConfig{Max: 3}, props3D)
	for _, x := range []float64{1400, 1000, 1600, 1100, 1200} { // shuffled; farther means larger x
		e.src.dets = append(e.src.dets, det3("world", spatialmath.NewPoseFromPoint(r3.Vector{X: x, Z: 300}), box(t, r3.Vector{X: 20, Y: 20, Z: 20}), nil, nil, fmt.Sprint(x)))
	}
	e.p.pollOnce(context.Background())
	set := e.out.last(t)
	if len(set) != 3 || set[0].label != "1000" || set[1].label != "1100" || set[2].label != "1200" {
		t.Fatalf("kept %+v", set)
	}
	if e.p.overflow.Load() != 2 {
		t.Fatalf("overflow %d", e.p.overflow.Load())
	}
}

func TestLiveSelfTest(t *testing.T) {
	e := newLV(t, idPose(), LiveConfig{}, props3D)
	posed, err := e.p.parts.poseParts(pandaHold)
	if err != nil {
		t.Fatal(err)
	}
	// a link's own surface points (the base frame is the world here), wrapped in a big box
	link := posed[3].ToPoints(5)
	if len(link) < 20 {
		t.Fatalf("link points %d", len(link))
	}
	selfDet := det3("world", idPose(), box(t, r3.Vector{X: 600, Y: 600, Z: 600}), cloud(link...), idPose(), "arm")
	// 30 mm below the gripper: a cube of points whose nearest part is well beyond 20 mm
	ee := eePoint(t, e.m, pandaHold)
	var c r3.Vector
	for off := 30.0; ; off += 5 {
		c = ee.Sub(r3.Vector{Z: off})
		d := math.Inf(1)
		for _, p := range posed {
			if x, _ := spatialmath.NewPoint(c, "").DistanceFrom(p); x < d {
				d = x
			}
		}
		if d >= 50 {
			break
		}
	}
	var cube []r3.Vector
	for _, a := range []float64{-5, 0, 5} {
		for _, b := range []float64{-5, 0, 5} {
			cube = append(cube, c.Add(r3.Vector{X: a, Y: b, Z: 3}))
		}
	}
	keep := det3("world", idPose(), box(t, r3.Vector{X: 20, Y: 20, Z: 20}), cloud(cube...), idPose(), "keep")
	noPts := det3("world", spatialmath.NewPoseFromPoint(posed[3].Pose().Point()), box(t, r3.Vector{X: 100, Y: 100, Z: 100}), nil, nil, "nopts")
	e.src.dets = []*detection3d.Detection{selfDet, keep, noPts}
	e.p.pollOnce(context.Background())
	set := e.out.last(t)
	if len(set) != 2 || e.p.selfDropped.Load() != 1 {
		t.Fatalf("set %d, selfDropped %d", len(set), e.p.selfDropped.Load())
	}
	labels := []string{set[0].label, set[1].label}
	sort.Strings(labels)
	if labels[0] != "keep" || labels[1] != "nopts" {
		t.Fatalf("labels %v", labels)
	}
}

func clipEnv(t *testing.T, mut func(*LiveConfig)) *lvEnv {
	lo, hi := r3.Vector{X: 200, Y: -600, Z: 20}, r3.Vector{X: 1000, Y: 600, Z: 600}
	cfg := LiveConfig{WorkspaceMin: &lo, WorkspaceMax: &hi}
	if mut != nil {
		mut(&cfg)
	}
	return newLV(t, idPose(), cfg, props3D)
}

func TestLiveClipRefitExtrude(t *testing.T) {
	infl := 20.0 // 2 x the default inflation
	tall := box(t, r3.Vector{X: 50, Y: 50, Z: 1000})

	// a column from z -200 to 700 refits to [20, 600]
	e := clipEnv(t, nil)
	var col []r3.Vector
	for z := -200.0; z <= 700; z += 5 {
		col = append(col, r3.Vector{X: 500 + math.Mod(z, 3), Y: math.Mod(z, 2), Z: z})
	}
	e.src.dets = []*detection3d.Detection{det3("world", idPose(), tall, cloud(col...), idPose(), "")}
	e.p.pollOnce(context.Background())
	g := e.out.last(t)[0].geom
	c := cfgOf(t, g)
	if math.Abs(g.Pose().Point().Z-310) > 1e-9 || math.Abs(c.Z-(580+infl)) > 1e-9 {
		t.Fatalf("column: centre z %v, height %v", g.Pose().Point().Z, c.Z)
	}
	if q := g.Pose().Orientation().Quaternion(); math.Abs(q.Real-1) > 1e-12 {
		t.Fatalf("not world-aligned: %v", q)
	}

	// too few points left
	e = clipEnv(t, nil)
	few := []r3.Vector{{X: 500, Z: 100}, {X: 500, Z: 110}}
	for z := 700.0; z < 760; z += 5 {
		few = append(few, r3.Vector{X: 500, Z: z})
	}
	e.src.dets = []*detection3d.Detection{det3("world", idPose(), tall, cloud(few...), idPose(), "")}
	e.p.pollOnce(context.Background())
	if len(e.out.last(t)) != 0 || e.p.clipped.Load() != 1 {
		t.Fatalf("few: set %d clipped %d", len(e.out.last(t)), e.p.clipped.Load())
	}

	// a slab at z 100..110 extruded to 20
	z20 := 20.0
	e = clipEnv(t, func(c *LiveConfig) { c.ExtrudeToZ = &z20 })
	var slab []r3.Vector
	for x := 400.0; x <= 500; x += 10 {
		for y := -50.0; y <= 50; y += 10 {
			slab = append(slab, r3.Vector{X: x, Y: y, Z: 100 + math.Mod(x+y, 11)})
		}
	}
	e.src.dets = []*detection3d.Detection{det3("world", idPose(), box(t, r3.Vector{X: 100, Y: 100, Z: 10}), cloud(slab...), idPose(), "")}
	e.p.pollOnce(context.Background())
	g = e.out.last(t)[0].geom
	c = cfgOf(t, g)
	lo, hi := g.Pose().Point().Z-c.Z/2+10, g.Pose().Point().Z+c.Z/2-10 // undo the inflation
	if math.Abs(lo-20) > 1e-9 || math.Abs(hi-110) > 1e-9 {
		t.Fatalf("slab z %v..%v, want 20..110", lo, hi)
	}

	// a pointless box is intersected with the workspace: x 700..1100 -> 700..1000, z 300..700 -> 300..600
	e = clipEnv(t, nil)
	e.src.dets = []*detection3d.Detection{det3("world", spatialmath.NewPoseFromPoint(r3.Vector{X: 900, Z: 500}), box(t, r3.Vector{X: 400, Y: 200, Z: 400}), nil, nil, "")}
	e.p.pollOnce(context.Background())
	g = e.out.last(t)[0].geom
	c = cfgOf(t, g)
	if !lnear(g.Pose().Point(), r3.Vector{X: 850, Y: 0, Z: 450}, 1e-9) || math.Abs(c.X-(300+infl)) > 1e-9 || math.Abs(c.Y-(200+infl)) > 1e-9 || math.Abs(c.Z-(300+infl)) > 1e-9 {
		t.Fatalf("pointless: %v %+v", g.Pose().Point(), c)
	}
	// ...and one wholly outside is dropped
	e.src.dets = []*detection3d.Detection{det3("world", spatialmath.NewPoseFromPoint(r3.Vector{X: 1500, Z: 500}), box(t, r3.Vector{X: 100, Y: 100, Z: 100}), nil, nil, "")}
	e.p.pollOnce(context.Background())
	if len(e.out.last(t)) != 0 || e.p.clipped.Load() != 1 {
		t.Fatalf("outside: clipped %d", e.p.clipped.Load())
	}
}

func TestLiveSkipsUnsupportedAndNonFinite(t *testing.T) {
	e := newLV(t, idPose(), LiveConfig{}, props3D)
	cyl, err := spatialmath.NewCylinder(idPose(), 20, 50, "")
	if err != nil {
		t.Fatal(err)
	}
	good := det3("world", spatialmath.NewPoseFromPoint(r3.Vector{X: 600, Z: 300}), box(t, r3.Vector{X: 20, Y: 20, Z: 20}), nil, nil, "good")
	nanPose := det3("world", spatialmath.NewPoseFromPoint(r3.Vector{X: math.NaN(), Z: 300}), box(t, r3.Vector{X: 20, Y: 20, Z: 20}), nil, nil, "nan")
	infDims := det3("world", spatialmath.NewPoseFromPoint(r3.Vector{X: 600}), box(t, r3.Vector{X: math.Inf(1), Y: 20, Z: 20}), nil, nil, "inf")
	noGeom := det3("world", idPose(), nil, nil, nil, "none")
	e.src.dets = []*detection3d.Detection{
		det3("world", idPose(), cyl, nil, nil, "cyl"), nanPose, infDims, noGeom, {}, good}
	e.p.pollOnce(context.Background())
	set := e.out.last(t)
	if len(set) != 1 || set[0].label != "good" || e.p.skipped.Load() != 5 {
		t.Fatalf("set %d skipped %d", len(set), e.p.skipped.Load())
	}
	for _, o := range set {
		if !finiteGeom(o.geom) {
			t.Fatal("non-finite geometry published")
		}
	}
}

func TestLiveErrorsAndPanic(t *testing.T) {
	e := newLV(t, idPose(), LiveConfig{}, props3D)
	e.src.err = fmt.Errorf("rpc down")
	e.p.pollOnce(context.Background())
	if e.out.n() != 0 || e.p.errs.Load() != 1 || e.p.updates.Load() != 0 {
		t.Fatalf("published %d, errs %d", e.out.n(), e.p.errs.Load())
	}
	e.src.err, e.src.doPanic = nil, true
	e.p.pollOnce(context.Background())
	if e.out.n() != 0 || e.p.errs.Load() != 2 {
		t.Fatalf("after panic: published %d, errs %d", e.out.n(), e.p.errs.Load())
	}
	e.src.doPanic = false
	e.p.pollOnce(context.Background())
	if e.out.n() != 1 || e.p.updates.Load() != 1 {
		t.Fatalf("recovery: published %d", e.out.n())
	}
}

func TestLiveSequentialAndStop(t *testing.T) {
	e := newLV(t, idPose(), LiveConfig{RateHz: 50, StaleMS: 200, TimeoutMS: 100}, props3D)
	e.src.delay = 40 * time.Millisecond // twice the period
	ctx, cancel := context.WithCancel(context.Background())
	done := make(chan struct{})
	go func() { e.p.run(ctx); close(done) }()
	time.Sleep(400 * time.Millisecond)
	cancel()
	select {
	case <-done:
	case <-time.After(2 * time.Second):
		t.Fatal("run did not stop")
	}
	if n := e.src.calls.Load(); n < 3 || e.src.maxIn.Load() != 1 {
		t.Fatalf("calls %d, max in flight %d", n, e.src.maxIn.Load())
	}
}

// farCube is a cube of 9 points whose nearest part is at least 50 mm away, and its centre.
func farCube(e *lvEnv, posed []spatialmath.Geometry) (r3.Vector, []r3.Vector) {
	ee, _ := e.m.Transform(pandaHold)
	var c r3.Vector
	for off := 30.0; ; off += 5 {
		c = ee.Point().Sub(r3.Vector{Z: off})
		d := math.Inf(1)
		for _, p := range posed {
			if x, _ := spatialmath.NewPoint(c, "").DistanceFrom(p); x < d {
				d = x
			}
		}
		if d >= 50 {
			break
		}
	}
	var cube []r3.Vector
	for _, a := range []float64{-5, 0, 5} {
		for _, b := range []float64{-5, 0, 5} {
			cube = append(cube, c.Add(r3.Vector{X: a, Y: b, Z: 3}))
		}
	}
	return c, cube
}

func TestLiveSelfTestUsesClippedPoints(t *testing.T) {
	posed0 := func(e *lvEnv) []spatialmath.Geometry {
		p, err := e.p.parts.poseParts(pandaHold)
		if err != nil {
			t.Fatal(err)
		}
		return p
	}
	probe := newLV(t, idPose(), LiveConfig{}, props3D)
	posed := posed0(probe)
	c, cube := farCube(probe, posed)
	lo, hi := c.Sub(r3.Vector{X: 25, Y: 25, Z: 25}), c.Add(r3.Vector{X: 25, Y: 25, Z: 25})
	e := newLV(t, idPose(), LiveConfig{WorkspaceMin: &lo, WorkspaceMax: &hi, MinPoints: 9}, props3D)
	pts := append(posed[3].ToPoints(5), cube...)
	if len(pts) < 30 {
		t.Fatalf("points %d", len(pts))
	}
	e.src.dets = []*detection3d.Detection{det3("world", idPose(), box(t, r3.Vector{X: 600, Y: 600, Z: 600}), cloud(pts...), idPose(), "")}
	e.p.pollOnce(context.Background())
	if len(e.out.last(t)) != 1 || e.p.selfDropped.Load() != 0 {
		t.Fatalf("set %d selfDropped %d: clipped-away arm points were counted", len(e.out.last(t)), e.p.selfDropped.Load())
	}
}

func TestLiveSelfTestThreshold(t *testing.T) {
	for _, tc := range []struct {
		near int
		drop bool
	}{{4, false}, {6, true}} {
		e := newLV(t, idPose(), LiveConfig{}, props3D)
		posed, _ := e.p.parts.poseParts(pandaHold)
		c, _ := farCube(e, posed)
		pts := posed[3].ToPoints(5)[:tc.near]
		for i := 0; i < 10-tc.near; i++ {
			pts = append(pts, c.Add(r3.Vector{X: float64(i)}))
		}
		e.src.dets = []*detection3d.Detection{det3("world", idPose(), box(t, r3.Vector{X: 600, Y: 600, Z: 600}), cloud(pts...), idPose(), "")}
		e.p.pollOnce(context.Background())
		if dropped := e.p.selfDropped.Load() == 1; dropped != tc.drop || (len(e.out.last(t)) == 0) != tc.drop {
			t.Fatalf("%d0%% near: dropped %v, want %v", tc.near, dropped, tc.drop)
		}
	}
}

func TestLiveFlatPointsGetMinimumThickness(t *testing.T) {
	e := clipEnv(t, func(c *LiveConfig) { c.InflateMM = fp(0) })
	var flat []r3.Vector
	for x := 400.0; x < 460; x += 5 {
		flat = append(flat, r3.Vector{X: x, Y: x / 10, Z: 100})
	}
	e.src.dets = []*detection3d.Detection{det3("world", idPose(), box(t, r3.Vector{X: 10, Y: 10, Z: 10}), cloud(flat...), idPose(), "")}
	e.p.pollOnce(context.Background())
	if c := cfgOf(t, e.out.last(t)[0].geom); c.Z < 1 || c.Z > 1+1e-9 {
		t.Fatalf("flat z %v, want the 1 mm floor (and zero inflation honoured)", c.Z)
	}
}

func TestLiveUnplaceableFrameIsAFault(t *testing.T) {
	e := newLV(t, idPose(), LiveConfig{}, props3D)
	e.src.dets = []*detection3d.Detection{det3("nosuchframe", idPose(), box(t, r3.Vector{X: 10, Y: 10, Z: 10}), nil, nil, "")}
	e.p.pollOnce(context.Background())
	if e.out.n() != 0 || e.p.errs.Load() != 1 || e.p.LastError() == "" {
		t.Fatalf("published %d, errs %d, last %q", e.out.n(), e.p.errs.Load(), e.p.LastError())
	}
}

func TestLiveVisionTimeout(t *testing.T) {
	e := newLV(t, idPose(), LiveConfig{StaleMS: 500, TimeoutMS: 50}, props3D)
	e.src.block = true
	t0 := time.Now()
	e.p.pollOnce(context.Background())
	if el := time.Since(t0); el > 250*time.Millisecond || e.p.errs.Load() != 1 || e.out.n() != 0 {
		t.Fatalf("took %v, errs %d", el, e.p.errs.Load())
	}
	if e.p.LastError() == "" {
		t.Fatal("no last error")
	}
}

func TestLiveTopClassificationByScore(t *testing.T) {
	e := newLV(t, idPose(), LiveConfig{}, props3D)
	d := det3("world", spatialmath.NewPoseFromPoint(r3.Vector{X: 900, Z: 300}), box(t, r3.Vector{X: 20, Y: 20, Z: 20}), nil, nil, "")
	d.Classifications = classification.Classifications{classification.NewClassification(0.2, "low"), classification.NewClassification(0.8, "high")}
	e.src.dets = []*detection3d.Detection{d}
	e.p.pollOnce(context.Background())
	if l := e.out.last(t)[0].label; l != "high" {
		t.Fatalf("label %q", l)
	}
}

// A depth camera's no-reading pixels land at its own origin; one such point joined to a real cluster must not
// stretch the refit box up to the camera. The detection frame here is the camera frame.
func TestLiveDropsCameraOriginPoint(t *testing.T) {
	lo, hi := r3.Vector{X: -2000, Y: -2000, Z: -2000}, r3.Vector{X: 2000, Y: 2000, Z: 2000}
	e := newLV(t, idPose(), LiveConfig{WorkspaceMin: &lo, WorkspaceMax: &hi, InflateMM: fp(0)}, props3D)
	var cube []r3.Vector
	for _, a := range []float64{-20, 0, 20} {
		for _, b := range []float64{-20, 0, 20} {
			for _, c := range []float64{0, 20} {
				cube = append(cube, r3.Vector{X: 600 + a, Y: 300 + b, Z: 100 + c})
			}
		}
	}
	e.src.dets = []*detection3d.Detection{det3("world", idPose(), box(t, r3.Vector{X: 40, Y: 40, Z: 20}), cloud(append(cube, r3.Vector{})...), idPose(), "")}
	e.p.pollOnce(context.Background())
	set := e.out.last(t)
	if len(set) != 1 {
		t.Fatalf("set %d", len(set))
	}
	g, c := set[0].geom, cfgOf(t, set[0].geom)
	if !lnear(g.Pose().Point(), r3.Vector{X: 600, Y: 300, Z: 110}, 1e-9) || math.Abs(c.X-40) > 1e-9 || math.Abs(c.Z-20) > 1e-9 {
		t.Fatalf("refit %v %+v, want the cube", g.Pose().Point(), c)
	}
}
