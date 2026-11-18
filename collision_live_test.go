package diffik

import (
	"math"
	"slices"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// liveBoxAt is a live obstacle: a w×h×thick box centred at p, in the arm's base frame (the world here).
func liveBoxAt(t *testing.T, p r3.Vector, dims r3.Vector) liveObstacle {
	t.Helper()
	g, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(p), dims, "")
	if err != nil {
		t.Fatal(err)
	}
	return liveObstacle{geom: g}
}

// liveBelow is a large live slab whose top is gap mm below the flange point fp.
func liveBelow(t *testing.T, fp r3.Vector, gap float64) liveObstacle {
	return liveBoxAt(t, r3.Vector{X: fp.X, Y: fp.Y, Z: fp.Z - gap - 50}, r3.Vector{X: 300, Y: 300, Z: 100})
}

func countLive(c *collisionModel) (n int) {
	for _, p := range c.pairs {
		if p.live {
			n++
		}
	}
	return n
}

func TestSetLiveKeepsStatic(t *testing.T) {
	m := panda(t)
	fp := eePoint(t, m, pandaHold)
	// overlaps the flange: static pairs suspended at start
	blk := staticPart(referenceframe.World, "blk", spatialmath.NewPoseFromPoint(r3.Vector{X: fp.X, Y: fp.Y, Z: fp.Z - 5 - 10}), mustBox(t, r3.Vector{X: 100, Y: 100, Z: 20}))
	c := buildColl(t, m, CollisionRequest{}, pandaHold, blk)
	nStatic, ign := len(c.pairs), slices.Clone(c.ignored())
	if len(ign) == 0 {
		t.Fatal("nothing suspended")
	}
	c.setLive([]liveObstacle{liveBelow(t, fp, 50)})
	if countLive(c) != len(c.parts) || len(c.pairs) != nStatic+len(c.parts) {
		t.Fatalf("live pairs %d, total %d, parts %d", countLive(c), len(c.pairs), len(c.parts))
	}
	c.setLive([]liveObstacle{liveBelow(t, fp, 50), liveBelow(t, fp, 80)})
	if countLive(c) != 2*len(c.parts) {
		t.Fatalf("live pairs %d", countLive(c))
	}
	for _, p := range c.pairs {
		if p.live && p.suspended {
			t.Fatal("live pair suspended")
		}
	}
	c.setLive(nil)
	if len(c.pairs) != nStatic || len(c.obs) != 1 || !slices.Equal(c.ignored(), ign) {
		t.Fatalf("after setLive(nil): %d pairs, %d obs, ignored %v", len(c.pairs), len(c.obs), c.ignored())
	}
}

func TestSetLiveNames(t *testing.T) {
	m := panda(t)
	fp := eePoint(t, m, pandaHold)
	o := liveBelow(t, fp, 50)
	o.label = "cup"
	c := buildColl(t, m, CollisionRequest{}, pandaHold)
	if c.hasPairs() {
		t.Fatal("pairs without obstacles")
	}
	c.setLive([]liveObstacle{liveBelow(t, fp, 50), o})
	if !c.hasPairs() || c.obs[0].names[0] != "live:0" || c.obs[1].names[0] != "live:1:cup" || !hasName(c.obs[1].names, "live") {
		t.Fatalf("names %v %v", c.obs[0].names, c.obs[1].names)
	}
}

func TestLiveAllow(t *testing.T) {
	m := panda(t)
	fp := eePoint(t, m, pandaHold)
	// accepted at build time with no live obstacles
	c := buildColl(t, m, CollisionRequest{Allows: [][2]string{{"arm", "live"}}}, pandaHold)
	c.setLive([]liveObstacle{liveBelow(t, fp, 50), liveBelow(t, fp, 80)})
	if c.hasPairs() {
		t.Fatalf("%d pairs after allowing arm vs live", len(c.pairs))
	}
	// one part only
	name := c.parts[0].names[0]
	c = buildColl(t, m, CollisionRequest{Allows: [][2]string{{name, "live"}}}, pandaHold)
	c.setLive([]liveObstacle{liveBelow(t, fp, 50)})
	if countLive(c) != len(c.parts)-1 {
		t.Fatalf("%d live pairs, want %d", countLive(c), len(c.parts)-1)
	}
}

func TestLiveBackAwayRow(t *testing.T) {
	m := panda(t)
	fp := eePoint(t, m, pandaHold)
	c := buildColl(t, m, CollisionRequest{}, pandaHold)
	acc := repeat(1, 7)
	c.setLive([]liveObstacle{liveBelow(t, fp, 50)})
	_, d0, _, err := c.rows(pandaHold, acc, 0.02, 1e9)
	if err != nil {
		t.Fatal(err)
	}
	// slide the slab up so the closest pair is 5 mm away
	c.setLive([]liveObstacle{liveBelow(t, fp, 50-(d0-5))})
	for _, tc := range []struct{ vr, want float64 }{{1e9, 2.5}, {1, 1}} {
		rows, minD, _, err := c.rows(pandaHold, acc, 0.02, tc.vr)
		if err != nil {
			t.Fatal(err)
		}
		if math.Abs(minD-5) > 1e-6 {
			t.Fatalf("closest %v, want 5", minD)
		}
		r := rows[0]
		t.Logf("vRetreat %g: d %.3f r %.3f", tc.vr, r.d, r.r)
		if !r.backAway || math.Abs(r.r-tc.want) > 1e-6 {
			t.Fatalf("row %+v, want back-away r %v", r, tc.want)
		}
	}
	// outside d_min: the damper
	c.setLive([]liveObstacle{liveBelow(t, fp, 50)})
	rows, _, _, _ := c.rows(pandaHold, acc, 0.02, 1e9)
	for _, r := range rows {
		if r.backAway {
			t.Fatalf("back-away row at d %v", r.d)
		}
	}
}

func TestStaticInsideDMinNoBackAway(t *testing.T) {
	m := panda(t)
	fp := eePoint(t, m, pandaHold)
	pose := spatialmath.NewPoseFromPoint(r3.Vector{X: fp.X, Y: fp.Y, Z: fp.Z - 5 - 50})
	c := buildColl(t, m, CollisionRequest{}, pandaHold, staticPart(referenceframe.World, "blk", pose, mustBox(t, r3.Vector{X: 300, Y: 300, Z: 100})))
	for i := range c.pairs {
		c.pairs[i].suspended = false
	}
	rows, minD, _, err := c.rows(pandaHold, repeat(1, 7), 0.02, 1e9)
	if err != nil || len(rows) == 0 || minD >= c.dMin {
		t.Fatalf("rows %d minD %v err %v", len(rows), minD, err)
	}
	for _, r := range rows {
		if r.backAway {
			t.Fatalf("static row is back-away: %+v", r)
		}
	}
}

// An inflated live box overlapping a link (d < 0): the row's gradient matches a finite difference.
func TestLivePenetrationGradient(t *testing.T) {
	m := panda(t)
	fp := eePoint(t, m, pandaHold)
	c := buildColl(t, m, CollisionRequest{}, pandaHold)
	o := liveBoxAt(t, r3.Vector{X: fp.X, Y: fp.Y, Z: fp.Z - 10}, r3.Vector{X: 150, Y: 150, Z: 60})
	c.setLive([]liveObstacle{o})
	rows, _, _, err := c.rows(pandaHold, make([]float64, 7), 0.02, 1e9)
	if err != nil {
		t.Fatal(err)
	}
	ref := func(q []float64) []float64 {
		lg, err := m.Geometries(q)
		if err != nil {
			t.Fatal(err)
		}
		var out []float64
		for _, g := range lg.Geometries() {
			d, err := g.DistanceFrom(o.geom)
			if err != nil {
				t.Fatal(err)
			}
			out = append(out, d)
		}
		return out
	}
	d0 := ref(pandaHold)
	worst, checked, neg := 0.0, 0, 0
	for _, r := range rows {
		if r.d >= 0 {
			continue
		}
		neg++
		if !r.backAway {
			t.Fatalf("penetrating row is not back-away: %+v", r)
		}
		k, best := -1, math.Inf(1)
		for i, d := range d0 {
			if e := math.Abs(d - r.d); e < best {
				k, best = i, e
			}
		}
		if best > 1e-6 {
			t.Fatalf("row d %v matches no reference (%v)", r.d, d0)
		}
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
	t.Logf("%d penetrating rows, %d joints checked, worst relative error %.2e", neg, checked, worst)
	if neg == 0 || checked == 0 || worst > 1e-3 {
		t.Fatalf("neg %d checked %d worst %v", neg, checked, worst)
	}
}
