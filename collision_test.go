package diffik

import (
	"math"
	"slices"
	"sort"
	"strings"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

func armPart(m referenceframe.Model, name string) *referenceframe.FrameSystemPart {
	return &referenceframe.FrameSystemPart{
		FrameConfig: referenceframe.NewLinkInFrame(referenceframe.World, spatialmath.NewZeroPose(), name, nil),
		ModelFrame:  m,
	}
}

// staticPart is a static frame at pose under parent, carrying g (posed at the frame's origin).
func staticPart(parent, name string, pose spatialmath.Pose, g spatialmath.Geometry) *referenceframe.FrameSystemPart {
	return &referenceframe.FrameSystemPart{FrameConfig: referenceframe.NewLinkInFrame(parent, pose, name, g)}
}

func mustBox(t *testing.T, dims r3.Vector) spatialmath.Geometry {
	t.Helper()
	g, err := spatialmath.NewBox(spatialmath.NewZeroPose(), dims, "")
	if err != nil {
		t.Fatal(err)
	}
	return g
}

func mustSphere(t *testing.T, r float64) spatialmath.Geometry {
	t.Helper()
	g, err := spatialmath.NewSphere(spatialmath.NewZeroPose(), r, "")
	if err != nil {
		t.Fatal(err)
	}
	return g
}

func tilt(p r3.Vector) spatialmath.Pose {
	return spatialmath.NewPose(p, &spatialmath.OrientationVectorDegrees{OX: 0.15, OY: 0.1, OZ: 1, Theta: 17})
}

func eePoint(t *testing.T, m referenceframe.Model, q []float64) r3.Vector {
	t.Helper()
	p, err := m.Transform(q)
	if err != nil {
		t.Fatal(err)
	}
	return p.Point()
}

func buildColl(t *testing.T, m referenceframe.Model, req CollisionRequest, q []float64, extra ...*referenceframe.FrameSystemPart) *collisionModel {
	t.Helper()
	c, err := NewCollisionModel(append([]*referenceframe.FrameSystemPart{armPart(m, "arm")}, extra...), m, "arm", req, q)
	if err != nil {
		t.Fatal(err)
	}
	return c
}

func pairsOfPart(c *collisionModel, name string) int {
	n := 0
	for _, p := range c.pairs {
		if hasName(c.parts[p.part].names, name) {
			n++
		}
	}
	return n
}

func TestCollisionGradient(t *testing.T) {
	m := panda(t)
	fp := eePoint(t, m, pandaHold)
	capG, err := spatialmath.NewCapsule(spatialmath.NewZeroPose(), 20, 300, "")
	if err != nil {
		t.Fatal(err)
	}
	mountOff := spatialmath.NewPoseFromPoint(r3.Vector{Z: 80})
	gripBox := mustBox(t, r3.Vector{X: 90, Y: 140, Z: 40})
	gp := spatialmath.Compose(mustPose(m.Transform(pandaHold)), mountOff).Point()
	cases := []struct {
		name  string
		pose  spatialmath.Pose
		g     spatialmath.Geometry
		mount bool // also a static box under arm at mountOff
	}{
		{"box", tilt(r3.Vector{X: fp.X, Y: fp.Y, Z: fp.Z - 50}), mustBox(t, r3.Vector{X: 300, Y: 300, Z: 20}), false},
		{"capsule", spatialmath.NewPose(r3.Vector{X: fp.X, Y: fp.Y, Z: fp.Z - 60}, &spatialmath.OrientationVectorDegrees{OX: 1}), capG, false},
		{"sphere", spatialmath.NewPoseFromPoint(r3.Vector{X: fp.X, Y: fp.Y, Z: fp.Z - 70}), mustSphere(t, 30), false},
		{"mounted box", tilt(r3.Vector{X: gp.X, Y: gp.Y, Z: gp.Z - 70}), mustBox(t, r3.Vector{X: 300, Y: 300, Z: 20}), true},
	}
	for _, tc := range cases {
		extra := []*referenceframe.FrameSystemPart{staticPart(referenceframe.World, tc.name, tc.pose, tc.g)}
		if tc.mount {
			extra = append(extra, staticPart("arm", "grip", mountOff, gripBox))
		}
		c := buildColl(t, m, CollisionRequest{}, pandaHold, extra...)
		rows, _, _, err := c.rows(pandaHold, make([]float64, 7), 0.02, 1e9)
		if err != nil {
			t.Fatal(err)
		}
		if len(rows) == 0 {
			t.Fatalf("%s: no rows", tc.name)
		}
		// Independent reference: link geometries straight from the model, the mounted box composed by hand,
		// and the obstacle built here.
		obs := tc.g.Transform(tc.pose)
		ref := func(q []float64) []float64 {
			lg, err := m.Geometries(q)
			if err != nil {
				t.Fatal(err)
			}
			gs := lg.Geometries()
			if tc.mount {
				gs = append(gs, gripBox.Transform(spatialmath.Compose(mustPose(m.Transform(q)), mountOff)))
			}
			out := make([]float64, len(gs))
			for i, g := range gs {
				if out[i], err = g.DistanceFrom(obs); err != nil {
					t.Fatal(err)
				}
			}
			return out
		}
		d0 := ref(pandaHold)
		worst, checked, sawMount := 0.0, 0, false
		for _, r := range rows {
			k, best := -1, math.Inf(1)
			for i, d := range d0 {
				if e := math.Abs(d - r.d); e < best {
					k, best = i, e
				}
			}
			if best > 1e-6 {
				t.Fatalf("%s: row d %v matches no reference geometry (%v)", tc.name, r.d, d0)
			}
			sawMount = sawMount || (tc.mount && k == len(d0)-1)
			for j := range pandaHold {
				qp, qm := slices.Clone(pandaHold), slices.Clone(pandaHold)
				qp[j] += 1e-4
				qm[j] -= 1e-4
				fd := (ref(qp)[k] - ref(qm)[k]) / 2e-4
				if math.Abs(r.jd[j]) > 1 {
					checked++
					worst = math.Max(worst, math.Abs(fd-r.jd[j])/math.Abs(r.jd[j]))
				}
			}
		}
		t.Logf("%s: %d rows, %d joints checked, worst relative error %.2e", tc.name, len(rows), checked, worst)
		if checked == 0 || worst > 1e-3 {
			t.Fatalf("%s: worst relative error %v over %d checks", tc.name, worst, checked)
		}
		if tc.mount && !sawMount {
			t.Fatalf("%s: no row for the mounted box", tc.name)
		}
	}
}

func mustPose(p spatialmath.Pose, err error) spatialmath.Pose {
	if err != nil {
		panic(err)
	}
	return p
}

// world_state obstacles in an arm-mounted frame resolve at the start pose, not at zero joints.
func TestCollisionWorldStateMovesWithArm(t *testing.T) {
	m := panda(t)
	off := spatialmath.NewPoseFromPoint(r3.Vector{Z: 200})
	g := mustBox(t, r3.Vector{X: 50, Y: 50, Z: 50})
	ws, err := referenceframe.NewWorldState([]*referenceframe.GeometriesInFrame{
		referenceframe.NewGeometriesInFrame("arm", []spatialmath.Geometry{g.Transform(off)}),
	}, nil)
	if err != nil {
		t.Fatal(err)
	}
	c := buildColl(t, m, CollisionRequest{WorldState: ws}, pandaHold)
	if len(c.obs) != 1 {
		t.Fatalf("%d obstacles", len(c.obs))
	}
	want := spatialmath.Compose(mustPose(m.Transform(pandaHold)), off).Point()
	if got := c.obs[0].geom.Pose().Point(); got.Sub(want).Norm() > 1e-6 {
		t.Fatalf("obstacle at %v, want %v", got, want)
	}

	// Parent in another movable frame: error.
	other := armPart(m, "arm2")
	ws2, _ := referenceframe.NewWorldState([]*referenceframe.GeometriesInFrame{
		referenceframe.NewGeometriesInFrame("arm2", []spatialmath.Geometry{g}),
	}, nil)
	_, err = NewCollisionModel([]*referenceframe.FrameSystemPart{armPart(m, "arm"), other}, m, "arm", CollisionRequest{WorldState: ws2}, pandaHold)
	if err == nil {
		t.Fatal("obstacle in a movable frame did not error")
	}
	t.Log(err)
}

func TestCollisionAllows(t *testing.T) {
	m := panda(t)
	fp := eePoint(t, m, pandaHold)
	table := staticPart(referenceframe.World, "table", spatialmath.NewPoseFromPoint(r3.Vector{X: fp.X, Y: fp.Y, Z: fp.Z - 300}), mustBox(t, r3.Vector{X: 500, Y: 500, Z: 20}))
	req := func(a ...[2]string) CollisionRequest { return CollisionRequest{Allows: a} }

	base := buildColl(t, m, req(), pandaHold, table)
	if pairsOfPart(base, "arm:link7") == 0 || len(base.pairs) == 0 {
		t.Fatalf("no pairs without allows: %d", len(base.pairs))
	}
	all := buildColl(t, m, req([2]string{"arm", "table"}), pandaHold, table)
	if len(all.pairs) != 0 {
		t.Fatalf("allow arm/table left %d pairs", len(all.pairs))
	}
	one := buildColl(t, m, req([2]string{"arm:link7", "table"}), pandaHold, table)
	if pairsOfPart(one, "arm:link7") != 0 || len(one.pairs) != len(base.pairs)-1 {
		t.Fatalf("allow link7/table: link7 pairs %d, %d of %d left", pairsOfPart(one, "arm:link7"), len(one.pairs), len(base.pairs))
	}
	if _, err := NewCollisionModel([]*referenceframe.FrameSystemPart{armPart(m, "arm"), table}, m, "arm", req([2]string{"nope", "table"}), pandaHold); err == nil {
		t.Fatal("unknown allow name did not error")
	}
}

func TestCollisionMeshHandling(t *testing.T) {
	m := panda(t)
	mesh := spatialmath.NewMesh(spatialmath.NewZeroPose(), []*spatialmath.Triangle{
		spatialmath.NewTriangle(r3.Vector{}, r3.Vector{X: 100}, r3.Vector{Y: 100}),
	}, "bin")
	bin := staticPart(referenceframe.World, "bin", spatialmath.NewPoseFromPoint(r3.Vector{X: 800}), mesh)
	c := buildColl(t, m, CollisionRequest{Allows: [][2]string{{"arm", "bin"}}}, pandaHold, bin)
	if len(c.skipped) != 1 || len(c.Warnings()) == 0 || len(c.obs) != 0 {
		t.Fatalf("skipped %v warnings %v obs %d", c.skipped, c.Warnings(), len(c.obs))
	}

	ws, err := referenceframe.NewWorldState([]*referenceframe.GeometriesInFrame{
		referenceframe.NewGeometriesInFrame(referenceframe.World, []spatialmath.Geometry{mesh}),
	}, nil)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := NewCollisionModel([]*referenceframe.FrameSystemPart{armPart(m, "arm")}, m, "arm", CollisionRequest{WorldState: ws}, pandaHold); err == nil {
		t.Fatal("world_state mesh did not error")
	}
}

func TestCollisionMovableSkipped(t *testing.T) {
	m := panda(t)
	other := armPart(m, "arm2")
	other.FrameConfig = referenceframe.NewLinkInFrame(referenceframe.World, spatialmath.NewPoseFromPoint(r3.Vector{X: 3000}), "arm2", nil)
	c := buildColl(t, m, CollisionRequest{Allows: [][2]string{{"arm2", "arm"}}}, pandaHold, other)
	if !slices.Contains(c.skipped, "arm2") || len(c.obs) != 0 || len(c.pairs) != 0 {
		t.Fatalf("skipped %v obs %d pairs %d", c.skipped, len(c.obs), len(c.pairs))
	}
}

func TestCollisionMountedPart(t *testing.T) {
	m := panda(t)
	grip := staticPart("arm", "gripper", spatialmath.NewPoseFromPoint(r3.Vector{Z: 80}), mustBox(t, r3.Vector{X: 90, Y: 140, Z: 40}))
	c := buildColl(t, m, CollisionRequest{}, pandaHold, grip)
	if len(c.obs) != 0 {
		t.Fatalf("mounted box became %d obstacles", len(c.obs))
	}
	idx := -1
	for i, p := range c.parts {
		if hasName(p.names, "gripper") {
			idx = i
		}
	}
	if idx < 0 {
		t.Fatal("no gripper part")
	}
	q2 := slices.Clone(pandaHold)
	q2[0] += 0.5
	for _, q := range [][]float64{pandaHold, q2} {
		posed, err := c.poseParts(q)
		if err != nil {
			t.Fatal(err)
		}
		ee, _ := m.Transform(q)
		want := spatialmath.Compose(ee, spatialmath.NewPoseFromPoint(r3.Vector{Z: 80})).Point()
		if got := posed[idx].Pose().Point(); got.Sub(want).Norm() > 1e-6 {
			t.Fatalf("gripper at %v, want %v", got, want)
		}
	}
}

func TestCollisionSuspendRearm(t *testing.T) {
	m := panda(t)
	fp := eePoint(t, m, pandaHold)
	// Overlapping the base link: suspended from the start.
	base := staticPart(referenceframe.World, "post", spatialmath.NewPoseFromPoint(r3.Vector{Z: 50}), mustBox(t, r3.Vector{X: 100, Y: 100, Z: 100}))
	c := buildColl(t, m, CollisionRequest{}, pandaHold, base)
	if len(c.ignored()) == 0 {
		t.Fatal("overlapping pair not suspended")
	}
	t.Logf("ignored: %v", c.ignored())

	// A flange pair 5 mm from a box: suspended, then re-armed once the arm moves beyond d_act.
	box := staticPart(referenceframe.World, "blk", spatialmath.NewPoseFromPoint(r3.Vector{X: fp.X, Y: fp.Y, Z: fp.Z - 5 - 10}), mustBox(t, r3.Vector{X: 100, Y: 100, Z: 20}))
	c = buildColl(t, m, CollisionRequest{}, pandaHold, box)
	closest, best := -1, math.Inf(1)
	posed0, _ := c.poseParts(pandaHold)
	for i := range c.pairs {
		if d, _ := c.pairDist(posed0, i); d < best {
			closest, best = i, d
		}
	}
	var allow [][2]string
	for i, p := range c.parts {
		if i != c.pairs[closest].part {
			allow = append(allow, [2]string{p.names[0], "blk"})
		}
	}
	c = buildColl(t, m, CollisionRequest{Allows: allow}, pandaHold, box)
	if len(c.pairs) == 0 || !c.pairs[0].suspended {
		t.Fatalf("flange pair not suspended: %+v", c.pairs)
	}
	q := slices.Clone(pandaHold)
	q[1] -= 0.8 // lift away
	q[3] += 0.8
	rows, _, _, err := c.rows(q, make([]float64, 7), 0.02, 1e9)
	if err != nil {
		t.Fatal(err)
	}
	posed, _ := c.poseParts(q)
	d, _ := c.pairDist(posed, 0)
	if d <= c.dAct {
		t.Skipf("test motion only reached %.1f mm", d)
	}
	if c.pairs[0].suspended || len(rows) != 0 {
		t.Fatalf("not re-armed at d=%.1f: %+v", d, c.pairs)
	}
}

func TestCollisionActivationAndCap(t *testing.T) {
	m := panda(t)
	fp := eePoint(t, m, pandaHold)
	far := staticPart(referenceframe.World, "far", spatialmath.NewPoseFromPoint(r3.Vector{X: fp.X, Y: fp.Y, Z: fp.Z - 1000}), mustSphere(t, 10))
	c := buildColl(t, m, CollisionRequest{}, pandaHold, far)
	rows, minD, _, err := c.rows(pandaHold, make([]float64, 7), 0.02, 1e9)
	if err != nil || len(rows) != 0 || minD < c.dAct {
		t.Fatalf("far pair: rows %d minD %v err %v", len(rows), minD, err)
	}

	var spheres []*referenceframe.FrameSystemPart
	for i := 0; i < 20; i++ {
		a := float64(i) * 2 * math.Pi / 20
		spheres = append(spheres, staticPart(referenceframe.World, "s"+string(rune('a'+i)),
			spatialmath.NewPoseFromPoint(r3.Vector{X: fp.X + 60*math.Cos(a), Y: fp.Y + 60*math.Sin(a), Z: fp.Z - 40}), mustSphere(t, 5)))
	}
	c = buildColl(t, m, CollisionRequest{}, pandaHold, spheres...)
	rows, _, _, err = c.rows(pandaHold, make([]float64, 7), 0.02, 1e9)
	if err != nil {
		t.Fatal(err)
	}
	if len(rows) != maxCollRows {
		t.Fatalf("got %d rows, want %d", len(rows), maxCollRows)
	}
	for i := 1; i < len(rows); i++ {
		if rows[i].d < rows[i-1].d {
			t.Fatalf("rows not closest first: %v then %v", rows[i-1].d, rows[i].d)
		}
	}
}

func TestCollisionRowsTime(t *testing.T) {
	if testing.Short() || raceOn {
		t.Skip("cost test: skipped under -short and -race (the race detector slows it ~8x)")
	}
	m := panda(t)
	fp := eePoint(t, m, pandaHold)
	var obs []*referenceframe.FrameSystemPart
	for k := 0; k < 5; k++ {
		obs = append(obs, staticPart(referenceframe.World, "b"+string(rune('a'+k)),
			tilt(r3.Vector{X: fp.X + float64(k*40-80), Y: fp.Y + 120, Z: fp.Z - 70 + float64(k*60)}), mustBox(t, r3.Vector{X: 100, Y: 60, Z: 60})))
	}
	c := buildColl(t, m, CollisionRequest{}, pandaHold, obs...)
	acc := []float64{1, 1, 1, 1, 1, 1, 1}
	var ds []time.Duration
	nrows := 0
	for i := 0; i < 1000; i++ {
		s := time.Now()
		r, _, _, err := c.rows(pandaHold, acc, 0.02, 1e9)
		if err != nil {
			t.Fatal(err)
		}
		ds = append(ds, time.Since(s))
		nrows = len(r)
	}
	sort.Slice(ds, func(a, b int) bool { return ds[a] < ds[b] })
	p50, p99 := ds[500], ds[990]
	t.Logf("pairs %d rows %d p50 %v p99 %v", len(c.pairs), nrows, p50, p99)
	if nrows == 0 {
		t.Fatal("no active rows: the test measures nothing")
	}
	if p99 >= 5*time.Millisecond {
		t.Fatalf("p99 %v >= 5ms", p99)
	}
}

// world_state obstacles are named by their label, not the reference frame.
func TestCollisionWorldStatePairName(t *testing.T) {
	m := panda(t)
	g := mustBox(t, r3.Vector{X: 50, Y: 50, Z: 50})
	g.SetLabel("bin-box")
	ws, err := referenceframe.NewWorldState([]*referenceframe.GeometriesInFrame{
		referenceframe.NewGeometriesInFrame("world", []spatialmath.Geometry{g.Transform(spatialmath.NewPoseFromPoint(r3.Vector{X: 500}))}),
	}, nil)
	if err != nil {
		t.Fatal(err)
	}
	c := buildColl(t, m, CollisionRequest{WorldState: ws}, pandaHold)
	if len(c.pairs) == 0 {
		t.Fatal("no pairs")
	}
	if got, want := c.pairName(0), c.parts[c.pairs[0].part].names[0]+"↔bin-box"; got != want {
		t.Fatalf("pairName %q, want %q", got, want)
	}
	if !hasName(c.obs[0].names, "world") {
		t.Fatalf("frame name dropped: %v", c.obs[0].names)
	}
}

func TestCollisionRejectsNaNConfig(t *testing.T) {
	m := panda(t)
	for _, req := range []CollisionRequest{{DMinMM: math.NaN()}, {DActMM: math.NaN()}, {DMinMM: 10, DActMM: math.NaN()}} {
		if _, err := NewCollisionModel([]*referenceframe.FrameSystemPart{armPart(m, "arm")}, m, "arm", req, pandaHold); err == nil {
			t.Errorf("%+v: want error", req)
		}
	}
}

func TestCollisionRowsNonFiniteDistance(t *testing.T) {
	m := panda(t)
	c := buildColl(t, m, CollisionRequest{}, pandaHold, staticPart(referenceframe.World, "t", spatialmath.NewPoseFromPoint(r3.Vector{X: 800}), mustBox(t, r3.Vector{X: 100, Y: 100, Z: 100})))
	if len(c.pairs) == 0 {
		t.Fatal("no pairs")
	}
	nan := spatialmath.NewPoseFromPoint(r3.Vector{X: math.NaN()})
	c.obs[0].geom = c.obs[0].geom.Transform(nan)
	if _, _, _, err := c.rows(pandaHold, make([]float64, 7), 0.02, 1e9); err == nil {
		t.Fatal("NaN distance did not error")
	}
}

func TestCollisionArmOnMovableFrame(t *testing.T) {
	m := panda(t)
	rail := armPart(m, "rail")
	arm := armPart(m, "arm")
	arm.FrameConfig = referenceframe.NewLinkInFrame("rail", spatialmath.NewZeroPose(), "arm", nil)
	_, err := NewCollisionModel([]*referenceframe.FrameSystemPart{rail, arm}, m, "arm", CollisionRequest{}, pandaHold)
	if err == nil || !strings.Contains(err.Error(), `mounted on movable frame "rail"`) {
		t.Fatalf("err = %v", err)
	}
}

func TestCollisionHeldMeshRejected(t *testing.T) {
	m := panda(t)
	mesh := spatialmath.NewMesh(spatialmath.NewZeroPose(), []*spatialmath.Triangle{
		spatialmath.NewTriangle(r3.Vector{}, r3.Vector{X: 100}, r3.Vector{Y: 100}),
	}, "held")
	held := referenceframe.NewLinkInFrame("arm", spatialmath.NewPoseFromPoint(r3.Vector{Z: 100}), "held", mesh)
	ws, err := referenceframe.NewWorldState(nil, []*referenceframe.LinkInFrame{held})
	if err != nil {
		t.Fatal(err)
	}
	if _, err := NewCollisionModel([]*referenceframe.FrameSystemPart{armPart(m, "arm")}, m, "arm", CollisionRequest{WorldState: ws}, pandaHold); err == nil {
		t.Fatal("held mesh was skipped, not rejected")
	}
}

// ponytail: no panda fixture with a mesh link, so the classification the link loop relies on is tested directly.
func TestShapeTypeMeshUnsupported(t *testing.T) {
	mesh := spatialmath.NewMesh(spatialmath.NewZeroPose(), []*spatialmath.Triangle{
		spatialmath.NewTriangle(r3.Vector{}, r3.Vector{X: 100}, r3.Vector{Y: 100}),
	}, "link3")
	if _, ok, err := shapeType(mesh); err != nil || ok {
		t.Fatalf("mesh: ok %v err %v", ok, err)
	}
	if err := unsupported("link3", spatialmath.MeshType); err == nil {
		t.Fatal("no error")
	}
}
