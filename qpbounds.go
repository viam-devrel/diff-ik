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

// qpNext builds this tick's QP from the weighted task rows, the damping λ, the posture pull (scaled by
// the ramp scale) and qpBounds, solves it, and returns the joints to send (spec "Objective").
func (l *Loop) qpNext(rows taskRows, scale, dt float64) (next []float64, active, iters int, err error) {
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
	x, active, iters, err := qpSolver(qpProblem{H: H, g: g, lb: lb, ub: ub})
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
