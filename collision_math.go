package diffik

import "math"

// collisionVmax is the fastest approach (mm per tick) toward an obstacle gap g mm away from which slowing
// by Ad mm per tick² each tick stops within the gap: brakeCap's discrete rule with c = Ad (spec "Each tick").
func collisionVmax(Ad, g float64) float64 { return brakeCap(Ad, math.Max(g, 0), 1) }

// relaxBound is the largest jd·Δq over lb ≤ Δq ≤ ub: the strongest move away from an obstacle the step
// bounds allow (spec "Failures").
func relaxBound(jd, lb, ub []float64) float64 {
	s := 0.0
	for j := range jd {
		s += math.Max(jd[j]*lb[j], jd[j]*ub[j])
	}
	return s
}

// brakeStep is the step that moves lastStep toward zero by at most a·dt² per joint (a in rad/s²), clipped
// to the position limits: no new motion, within the acceleration limit (spec "Failures").
func brakeStep(last, a []float64, dt float64, q, lo, hi []float64) []float64 {
	out := make([]float64, len(last))
	for i, s := range last {
		c := a[i] * dt * dt
		out[i] = math.Min(math.Max(s-math.Min(math.Max(s, -c), c), lo[i]-q[i]), hi[i]-q[i])
	}
	return out
}
