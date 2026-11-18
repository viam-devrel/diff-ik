package diffik

import (
	"fmt"
	"math"

	"gonum.org/v1/gonum/mat"
)

// brakeCap is the largest step (rad per tick) toward a limit d rad away that can still stop in time when
// slowed by a·dt² per tick, for any step size (spec "Bounds": the discrete braking cap). The min(d, c)
// term covers d < c/8, where the first term goes negative: a step s ≤ c stops on the next tick.
func brakeCap(a, d, dt float64) float64 {
	c := a * dt * dt
	return math.Max(math.Max(math.Sqrt(2*a*math.Max(d, 0))*dt-c/2, math.Min(d, c)), 0)
}

// qpBounds returns per-joint bounds on this tick's step (spec "Bounds (QP path)"): position, velocity,
// discrete braking and acceleration, intersected. If the intersection is empty, position wins.
func (l *Loop) qpBounds(dt float64) (lb, ub []float64) {
	n := len(l.qCmd)
	lb, ub = make([]float64, n), make([]float64, n)
	for i := 0; i < n; i++ {
		a := deg2rad(l.cfg.MaxJointDegPerS2[i])
		v := deg2rad(l.cfg.MaxJointDegPerS[i]) * dt
		plo, phi := l.lo[i]-l.qCmd[i], l.hi[i]-l.qCmd[i]
		lo := math.Max(math.Max(plo, -v), -brakeCap(a, -plo, dt))
		hi := math.Min(math.Min(phi, v), brakeCap(a, phi, dt))
		alo, ahi := l.lastStep[i]-a*dt*dt, l.lastStep[i]+a*dt*dt
		lb[i], ub[i] = math.Max(lo, alo), math.Min(hi, ahi)
		if lb[i] > ub[i] { // position, velocity and braking win: the step in [lo, hi] nearest the acceleration interval
			x := math.Min(math.Max(l.lastStep[i], lo), hi)
			lb[i], ub[i] = x, x
		}
	}
	return lb, ub
}

// qpMu keeps H positive definite on a redundant arm with no damping and no posture, and stops near-null
// directions (the SO-101's wrist roll under the pointing rows) from drifting (spec "Objective").
const qpMu = 1e-6

// qpSolver is solveQP; tests replace it to force failures.
var qpSolver = solveQP

// relaxEps is the slack under a relaxed row's bound: at exactly the reachable maximum the feasible set is
// one box corner, which the solver reports infeasible.
const relaxEps = 1e-6 // mm per tick

// retreatWeight is w_b for tier 2's soft retreat term w_b·(J_d·Δq − r)² (spec "The row for a live pair").
// Task rows are in metres (A·Δq − b, so 1 mm of task error costs 1e-6) while J_d·Δq and r are in mm, so w_b is
// m² per mm²: 1e-6 makes 1 mm of retreat shortfall cost about the same as 1 mm of task error. 1e-5 overshot in the
// sim (-12 mm true clearance); 1e-4 broke the 0.1 mm static-pinch bar (0.2 mm dips in the static pinch tests).
const retreatWeight = 1e-6

// tier2SoftRetreat gates the soft retreat; tests clear it to get the old hold-still tier 2.
var tier2SoftRetreat = true

// qpNext builds this tick's QP from the weighted task rows, the damping λ, the posture pull (scaled by
// the ramp scale) and qpBounds, solves it, and returns the joints to send (spec "Objective").
func (l *Loop) qpNext(rows taskRows, scale, dt float64, coll []collRow, tier int) (next []float64, active, iters int, err error) {
	n := len(l.qCmd)
	m, _ := rows.A.Dims()
	wp, pull := 0.0, make([]float64, n)
	if p := l.cfg.Posture; p != nil {
		wp = p.Weight
		for i := range pull {
			pull[i] = scale * p.Gain * dt * (l.q0[i] - l.qCmd[i])
		}
	}
	H, g := mat.NewSymDense(n, nil), make([]float64, n)
	for i := 0; i < n; i++ {
		for j := i; j < n; j++ {
			s := 0.0
			for k := 0; k < m; k++ {
				s += rows.A.At(k, i) * rows.A.At(k, j)
			}
			if i == j {
				s += rows.lambda*rows.lambda + qpMu + wp
			}
			H.SetSym(i, j, s)
		}
		s := 0.0
		for k := 0; k < m; k++ {
			s += rows.A.At(k, i) * rows.b[k]
		}
		g[i] = -(s + wp*pull[i])
	}
	lb, ub := l.qpBounds(dt)
	prob := qpProblem{H: H, g: g, lb: lb, ub: ub}
	if len(coll) > 0 { // collision damper rows: J_d·Δq ≥ −v_max (back-away rows: ≥ r), relaxed on a retry
		// tier 1: min(bound, M) − ε. tier 2: back-away rows are also capped at 0 (no further approach) and pulled back by a soft retreat term.
		A, b := mat.NewDense(len(coll), n, nil), make([]float64, len(coll))
		for k, r := range coll {
			A.SetRow(k, r.jd)
			b[k] = -r.vmax
			if r.backAway {
				b[k] = r.r
			}
			if tier > 0 {
				M := relaxBound(r.jd, lb, ub)
				if tier == 2 && r.backAway {
					M = math.Min(M, 0)
				}
				b[k] = math.Min(b[k], M) - relaxEps
				if tier == 2 && r.backAway && tier2SoftRetreat { // retreat as far as the capped rows allow together
					for i := 0; i < n; i++ {
						for j := i; j < n; j++ {
							H.SetSym(i, j, H.At(i, j)+retreatWeight*r.jd[i]*r.jd[j])
						}
						g[i] -= retreatWeight * r.r * r.jd[i]
					}
				}
			}
		}
		prob.A, prob.b = A, b
	}
	x, active, iters, err := qpSolver(prob)
	if err != nil {
		return nil, 0, iters, err
	}
	for _, v := range x {
		if math.IsNaN(v) || math.IsInf(v, 0) {
			return nil, 0, iters, fmt.Errorf("non-finite step %v", x)
		}
	}
	next = make([]float64, n)
	for i := range next {
		next[i] = math.Min(math.Max(l.qCmd[i]+x[i], l.lo[i]), l.hi[i]) // guard (spec "Bounds")
	}
	return next, active, iters, nil
}
