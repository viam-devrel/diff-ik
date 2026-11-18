package diffik

import (
	"fmt"
	"math"
	"slices"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"gonum.org/v1/gonum/mat"
	"gonum.org/v1/gonum/num/quat"
)

// Task modes (spec "Task modes and the per-tick solve").
const (
	TaskPose     = "pose"
	TaskPointing = "pointing"
)

// toolAxis returns the tool (flange z) axis of orientation q in the base frame — Viam's
// orientation-vector direction.
func toolAxis(q quat.Number) r3.Vector {
	x, y, z, w := q.Imag, q.Jmag, q.Kmag, q.Real
	return r3.Vector{X: 2 * (x*z + w*y), Y: 2 * (y*z - w*x), Z: 1 - 2*(x*x+y*y)}
}

// pointingError returns the rotation vector r (base frame, ⟂ ac) that turns the current tool axis
// ac onto the target axis at, and an orthonormal pair u, v ⟂ ac.
func pointingError(ac, at r3.Vector) (r, u, v r3.Vector) {
	u = ac.Cross(r3.Vector{Z: 1})
	if u.Norm() < 1e-6 {
		u = ac.Cross(r3.Vector{X: 1})
	}
	u = u.Normalize()
	v = ac.Cross(u)
	c := ac.Cross(at)
	switch {
	case c.Norm() >= 1e-9:
		r = c.Normalize().Mul(math.Atan2(c.Norm(), ac.Dot(at)))
	case ac.Dot(at) < 0:
		r = u.Mul(math.Pi)
	}
	return r, u, v
}

// pointingRows returns J2 = [uᵀJ_ang; vᵀJ_ang] (2×n) from the 6×n Jacobian.
func pointingRows(J *mat.Dense, u, v r3.Vector) *mat.Dense {
	_, n := J.Dims()
	J2 := mat.NewDense(2, n, nil)
	for j := 0; j < n; j++ {
		w := r3.Vector{X: J.At(3, j), Y: J.At(4, j), Z: J.At(5, j)}
		J2.Set(0, j, u.Dot(w))
		J2.Set(1, j, v.Dot(w))
	}
	return J2
}

// nullProjector returns N1 = I − pinv(J1)·J1 with the dpinv rank tolerance.
func nullProjector(J1 mat.Matrix) (*mat.Dense, error) {
	_, n := J1.Dims()
	P, _, _, _, err := dpinv(J1, 1, 0, false)
	if err != nil {
		return nil, err
	}
	N := mat.NewDense(n, n, nil)
	N.Mul(P, J1)
	for i := 0; i < n; i++ {
		for j := 0; j < n; j++ {
			d := -N.At(i, j)
			if i == j {
				d++
			}
			N.Set(i, j, d)
		}
	}
	return N, nil
}

// prioritySolve returns Δq = dpinv(J1)Δx1 + dpinv(J2·N1)(Δx2 − J2·Δq1), with independent
// Chiaverini damping per level (spec "Pointing mode → Priority solve").
func prioritySolve(J1, J2 *mat.Dense, dx1, dx2 []float64, eps, maxDamping float64) (dq []float64, l1, l2 float64, err error) {
	P1, l1, _, _, err := dpinv(J1, eps, maxDamping, true)
	if err != nil {
		return nil, 0, 0, err
	}
	dq1 := mat.NewVecDense(len(dx1), dx1)
	var q1 mat.VecDense
	q1.MulVec(P1, dq1)
	N1, err := nullProjector(J1)
	if err != nil {
		return nil, 0, 0, err
	}
	var J2N mat.Dense
	J2N.Mul(J2, N1)
	P2, l2, _, _, err := dpinv(&J2N, eps, maxDamping, true)
	if err != nil {
		return nil, 0, 0, err
	}
	var j2q1, rhs, q2 mat.VecDense
	j2q1.MulVec(J2, &q1)
	rhs.SubVec(mat.NewVecDense(len(dx2), dx2), &j2q1)
	q2.MulVec(P2, &rhs)
	q1.AddVec(&q1, &q2)
	return append([]float64(nil), q1.RawVector().Data...), l1, l2, nil
}

// pointingReach splits pointing error e2 (rad) into the part J2·N1 can still reduce and the
// residual it can't: |e2_reach| and |e2 − e2_reach| (spec "Achievable pointing"). A joint within
// limitEps of a limit in [lo, hi] whose reducing step would push it further out is locked (its
// column zeroed in J1 and J2) and the split is recomputed. lo/hi nil: no limits.
func pointingReach(J1, J2 *mat.Dense, e2 [2]float64, q, lo, hi []float64) (reach, residual float64, err error) {
	reach, residual, dq, err := pointingSplit(J1, J2, e2)
	if err != nil || lo == nil {
		return reach, residual, err
	}
	J1, J2 = mat.DenseCopyOf(J1), mat.DenseCopyOf(J2)
	if !lockOutward(J1, J2, dq, q, lo, hi) {
		return reach, residual, nil
	}
	reach, residual, _, err = pointingSplit(J1, J2, e2)
	return reach, residual, err
}

// limitEps: a joint within this of its limit with an outward step is locked (lockOutward), so it
// won't cover that last <0.5 deg toward the limit.
const limitEps = 0.5 * math.Pi / 180 // rad

// lockOutward zeroes (in place) the J1/J2 columns of joints within limitEps of a limit whose step dq
// pushes further out (|dq| > 1e-12). Reports whether any column was zeroed.
func lockOutward(J1, J2 *mat.Dense, dq, q, lo, hi []float64) bool {
	any := false
	for i, d := range dq {
		if (d > 1e-12 && q[i] >= hi[i]-limitEps) || (d < -1e-12 && q[i] <= lo[i]+limitEps) {
			for r := 0; r < 3; r++ {
				J1.Set(r, i, 0)
			}
			for r := 0; r < 2; r++ {
				J2.Set(r, i, 0)
			}
			any = true
		}
	}
	return any
}

// pointingSplit is the limit-blind split; dq is the null-space joint step that would reduce e2.
func pointingSplit(J1, J2 *mat.Dense, e2 [2]float64) (reach, residual float64, dq []float64, err error) {
	N1, err := nullProjector(J1)
	if err != nil {
		return 0, 0, nil, err
	}
	var J2N, proj mat.Dense
	J2N.Mul(J2, N1)
	P, _, _, _, err := dpinv(&J2N, 1, 0, false)
	if err != nil {
		return 0, 0, nil, err
	}
	proj.Mul(&J2N, P)
	e := mat.NewVecDense(2, e2[:])
	var er, q0, q mat.VecDense
	er.MulVec(&proj, e)
	q0.MulVec(P, e)
	q.MulVec(N1, &q0)
	return er.Norm(2), math.Hypot(e2[0]-er.AtVec(0), e2[1]-er.AtVec(1)), q.RawVector().Data, nil
}

// pointingAxisWeight weighs the QP's two tool-axis rows against the position rows (weight 1): its square
// root, 0.01, makes 1 rad of axis error cost what 10 mm of position error does, so position wins
// whenever the two conflict (spec "Objective"; stands in for prioritySolve's strict priority).
const pointingAxisWeight = 1e-4

// pointingTaskRows stacks J1 (3×n) over √pointingAxisWeight·J2 (2×n), with the matching Δx.
func pointingTaskRows(J1, J2 *mat.Dense, dx1, dx2 []float64) taskRows {
	_, n := J1.Dims()
	sw := math.Sqrt(pointingAxisWeight)
	A, b := mat.NewDense(5, n, nil), make([]float64, 5)
	for j := 0; j < n; j++ {
		for i := 0; i < 3; i++ {
			A.Set(i, j, J1.At(i, j))
		}
		for i := 0; i < 2; i++ {
			A.Set(3+i, j, sw*J2.At(i, j))
		}
	}
	copy(b, dx1)
	b[3], b[4] = sw*dx2[0], sw*dx2[1]
	return taskRows{A: A, b: b}
}

// SelfMotion moves q along the arm's self-motion (the null space of its 6×n Jacobian, n > 6) until joint
// j has changed by delta rad, pulling the pose back to FK(q) after each small step. The result holds
// the same end-effector pose with a different posture.
func SelfMotion(m referenceframe.Model, q []float64, j int, delta float64) ([]float64, error) {
	p0, o0, err := FK(m, q)
	if err != nil {
		return nil, err
	}
	cur := slices.Clone(q)
	const h = 0.002 // rad of joint j per step
	for k := 0; k < 10000 && math.Abs(delta-(cur[j]-q[j])) > 1e-6; k++ {
		J, err := Jacobian(m, cur)
		if err != nil {
			return nil, err
		}
		_, n := J.Dims()
		if n <= 6 {
			return nil, fmt.Errorf("self-motion needs more than 6 joints, got %d", n)
		}
		var svd mat.SVD
		if !svd.Factorize(J, mat.SVDFull) {
			return nil, errSVD
		}
		var V mat.Dense
		svd.VTo(&V)
		v := mat.Col(nil, n-1, &V)
		if math.Abs(v[j]) < 0.05 { // else h/v[j] would swing the other joints wildly (review finding)
			return nil, fmt.Errorf("joint %d barely moves along the self-motion here (|v| = %.3g)", j, math.Abs(v[j]))
		}
		want := delta - (cur[j] - q[j])
		s := math.Copysign(math.Min(h, math.Abs(want)), want) / v[j]
		for i := range cur {
			cur[i] += s * v[i]
		}
		for it := 0; it < 3; it++ {
			p, o, err := FK(m, cur)
			if err != nil {
				return nil, err
			}
			e := PoseError(p0, o0, p, o)
			Jc, err := Jacobian(m, cur)
			if err != nil {
				return nil, err
			}
			P, _, _, _, err := dpinv(Jc, 1, 0, false)
			if err != nil {
				return nil, err
			}
			var dq mat.VecDense
			dq.MulVec(P, mat.NewVecDense(6, e[:]))
			for i := range cur {
				cur[i] += dq.AtVec(i)
			}
		}
		for i, lim := range m.DoF() {
			if cur[i] < lim.Min || cur[i] > lim.Max {
				return nil, fmt.Errorf("self-motion took joint %d out of its limits", i)
			}
		}
	}
	if math.Abs(delta-(cur[j]-q[j])) > 1e-6 {
		return nil, fmt.Errorf("self-motion stalled at joint %d: moved %.4g of %.4g rad", j, cur[j]-q[j], delta)
	}
	return cur, nil
}
