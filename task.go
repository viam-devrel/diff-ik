package diffik

import (
	"math"

	"github.com/golang/geo/r3"
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
// residual it can't: |e2_reach| and |e2 − e2_reach| (spec "Achievable pointing").
func pointingReach(J1, J2 *mat.Dense, e2 [2]float64) (reach, residual float64, err error) {
	N1, err := nullProjector(J1)
	if err != nil {
		return 0, 0, err
	}
	var J2N, proj mat.Dense
	J2N.Mul(J2, N1)
	P, _, _, _, err := dpinv(&J2N, 1, 0, false)
	if err != nil {
		return 0, 0, err
	}
	proj.Mul(&J2N, P)
	var er mat.VecDense
	er.MulVec(&proj, mat.NewVecDense(2, e2[:]))
	return er.Norm(2), math.Hypot(e2[0]-er.AtVec(0), e2[1]-er.AtVec(1)), nil
}
