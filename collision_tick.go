package diffik

import (
	"fmt"
	"math"
	"sort"

	"go.viam.com/rdk/spatialmath"
)

// poseParts returns every part's geometry in the arm's base frame at joints q (rad).
func (c *collisionModel) poseParts(q []float64) ([]spatialmath.Geometry, error) {
	lg, err := c.model.Geometries(q)
	if err != nil {
		return nil, err
	}
	byLabel := map[string]spatialmath.Geometry{}
	for _, g := range lg.Geometries() {
		byLabel[g.Label()] = g
	}
	ee, err := c.model.Transform(q) // mm
	if err != nil {
		return nil, err
	}
	out := make([]spatialmath.Geometry, len(c.parts))
	for i, p := range c.parts {
		if p.link != "" {
			g, ok := byLabel[p.link]
			if !ok {
				return nil, fmt.Errorf("link geometry %q missing", p.link)
			}
			out[i] = g
			continue
		}
		out[i] = p.geom.Transform(ee)
	}
	return out, nil
}

// collRow is one damper row J_d·Δq ≥ −vmax (mm per tick), for pair index pair at distance d (mm).
type collRow struct {
	jd      []float64 // mm/rad
	vmax, d float64
	pair    int
}

const maxCollRows = 16

func (c *collisionModel) pairDist(posed []spatialmath.Geometry, i int) (float64, error) {
	p := c.pairs[i]
	return posed[p.part].DistanceFrom(c.obs[p.obs].geom)
}

// rows measures every pair at q, re-arms suspended pairs beyond d_act, and returns a damper row per active
// pair (closest maxCollRows), plus the closest unsuspended pair (spec "Each tick"). accel is rad/s² per joint.
func (c *collisionModel) rows(q, accel []float64, dt float64) (rows []collRow, minD float64, minPair int, err error) {
	minD, minPair = math.Inf(1), -1
	posed, err := c.poseParts(q)
	if err != nil {
		return nil, 0, -1, err
	}
	ds := make([]float64, len(c.pairs))
	var act []int
	for i := range c.pairs {
		d, err := c.pairDist(posed, i)
		if err != nil {
			return nil, 0, -1, err
		}
		if math.IsNaN(d) || math.IsInf(d, 0) {
			return nil, 0, -1, fmt.Errorf("pair %s: non-finite distance %v", c.pairName(i), d)
		}
		ds[i] = d
		if c.pairs[i].suspended {
			if d > c.dAct {
				c.pairs[i].suspended = false
			}
			continue
		}
		if d < minD {
			minD, minPair = d, i
		}
		if d < c.dAct {
			act = append(act, i)
		}
	}
	sort.Slice(act, func(a, b int) bool { return ds[act[a]] < ds[act[b]] })
	if len(act) > maxCollRows {
		act = act[:maxCollRows]
	}
	if len(act) == 0 {
		return nil, minD, minPair, nil
	}
	jd := make([][]float64, len(act))
	for k := range jd {
		jd[k] = make([]float64, len(q))
	}
	for j := range q {
		qp, qm := append([]float64(nil), q...), append([]float64(nil), q...)
		qp[j] += fdStep
		qm[j] -= fdStep
		pp, err := c.poseParts(qp)
		if err != nil {
			return nil, 0, -1, err
		}
		pm, err := c.poseParts(qm)
		if err != nil {
			return nil, 0, -1, err
		}
		for k, i := range act {
			dp, err := c.pairDist(pp, i)
			if err != nil {
				return nil, 0, -1, err
			}
			dm, err := c.pairDist(pm, i)
			if err != nil {
				return nil, 0, -1, err
			}
			jd[k][j] = (dp - dm) / (2 * fdStep)
		}
	}
	rows = make([]collRow, len(act))
	for k, i := range act {
		Ad := 0.0
		for j := range q {
			Ad += math.Abs(jd[k][j]) * accel[j] * dt * dt
		}
		rows[k] = collRow{jd: jd[k], vmax: collisionVmax(Ad, ds[i]-c.dMin), d: ds[i], pair: i}
	}
	return rows, minD, minPair, nil
}

func (c *collisionModel) pairName(i int) string {
	p := c.pairs[i]
	return c.parts[p.part].names[0] + "↔" + c.obs[p.obs].names[0]
}

// ignored lists the suspended pairs' names.
func (c *collisionModel) ignored() []string {
	var out []string
	for i, p := range c.pairs {
		if p.suspended {
			out = append(out, c.pairName(i))
		}
	}
	return out
}
