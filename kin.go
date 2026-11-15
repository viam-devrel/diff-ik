package diffik

import (
	"errors"
	"math"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"gonum.org/v1/gonum/mat"
	"gonum.org/v1/gonum/num/quat"
)

var errSVD = errors.New("jacobian SVD failed")

const (
	mmPerM = 1000.0
	fdStep = 1e-6 // rad, central-difference step for the Jacobian
)

// FK returns the end-effector position (m) and orientation in the arm's base frame.
// This is the only place model output is converted from mm to m.
func FK(m referenceframe.Model, q []referenceframe.Input) (r3.Vector, quat.Number, error) {
	p, err := m.Transform(q)
	if err != nil {
		return r3.Vector{}, quat.Number{}, err
	}
	return p.Point().Mul(1 / mmPerM), p.Orientation().Quaternion(), nil
}

// RotVec returns log(a·bᵀ) as a base-frame rotation vector in radians.
// spatialmath.QuatToR3AA is not used because it returns zero below 1e-6 rad,
// which would zero out the Jacobian's angular rows.
func RotVec(a, b quat.Number) r3.Vector {
	q := quat.Mul(a, quat.Conj(b))
	if q.Real < 0 { // shortest rotation
		q = quat.Scale(-1, q)
	}
	s := math.Sqrt(q.Imag*q.Imag + q.Jmag*q.Jmag + q.Kmag*q.Kmag)
	k := 2.0 // small-angle limit of 2·atan2(s, w)/s
	if s > 1e-12 {
		k = 2 * math.Atan2(s, q.Real) / s
	}
	return r3.Vector{X: k * q.Imag, Y: k * q.Jmag, Z: k * q.Kmag}
}

// PoseError returns [dx dy dz rx ry rz] (m, rad) from current to target, in the base frame.
func PoseError(tp r3.Vector, tq quat.Number, cp r3.Vector, cq quat.Number) [6]float64 {
	d := tp.Sub(cp)
	r := RotVec(tq, cq)
	return [6]float64{d.X, d.Y, d.Z, r.X, r.Y, r.Z}
}

// Jacobian returns the 6×n base-frame Jacobian at q by central differences.
// Rows are (vx, vy, vz, wx, wy, wz) in m/rad and rad/rad.
func Jacobian(m referenceframe.Model, q []referenceframe.Input) (*mat.Dense, error) {
	n := len(q)
	J := mat.NewDense(6, n, nil)
	qp := make([]referenceframe.Input, n)
	qm := make([]referenceframe.Input, n)
	for i := 0; i < n; i++ {
		copy(qp, q)
		copy(qm, q)
		qp[i] += fdStep
		qm[i] -= fdStep
		pp, rp, err := FK(m, qp)
		if err != nil {
			return nil, err
		}
		pm, rm, err := FK(m, qm)
		if err != nil {
			return nil, err
		}
		dp := pp.Sub(pm).Mul(1 / (2 * fdStep))
		dr := RotVec(rp, rm).Mul(1 / (2 * fdStep))
		J.SetCol(i, []float64{dp.X, dp.Y, dp.Z, dr.X, dr.Y, dr.Z})
	}
	return J, nil
}

// dlsStep solves Δq = (JᵀWJ + λ²I)⁻¹ JᵀW Δx with Chiaverini damping on σmin(W^½J).
func dlsStep(J *mat.Dense, w [6]float64, dx [6]float64, eps, maxDamping float64) (dq []float64, lambda, sigmaMin float64, err error) {
	_, n := J.Dims()
	var wj mat.Dense
	wj.Apply(func(i, _ int, v float64) float64 { return math.Sqrt(w[i]) * v }, J)

	var svd mat.SVD
	if !svd.Factorize(&wj, mat.SVDNone) {
		return nil, 0, 0, errSVD
	}
	vals := svd.Values(nil)
	sigmaMin = vals[len(vals)-1]

	l2 := 0.0
	if sigmaMin < eps {
		r := sigmaMin / eps
		l2 = maxDamping * maxDamping * (1 - r*r)
	}

	var a mat.Dense
	a.Mul(wj.T(), &wj)
	for i := 0; i < n; i++ {
		a.Set(i, i, a.At(i, i)+l2)
	}
	wdx := mat.NewVecDense(6, nil)
	for i := range dx {
		wdx.SetVec(i, math.Sqrt(w[i])*dx[i])
	}
	var b, x mat.VecDense
	b.MulVec(wj.T(), wdx)
	if err := x.SolveVec(&a, &b); err != nil {
		return nil, 0, 0, err
	}
	return append([]float64(nil), x.RawVector().Data...), math.Sqrt(l2), sigmaMin, nil
}

// ExpRot returns the unit quaternion for base-frame rotation vector v (rad): the inverse of RotVec.
func ExpRot(v r3.Vector) quat.Number {
	th := v.Norm()
	if th < 1e-12 {
		return quat.Number{Real: 1}
	}
	s := math.Sin(th/2) / th
	return quat.Number{Real: math.Cos(th / 2), Imag: s * v.X, Jmag: s * v.Y, Kmag: s * v.Z}
}

// rankTol is the relative singular-value cutoff for dpinv. It is 1e-4, not tighter, because
// so101.urdf/so101.json write angles as rounded constants (3.14159, 1.5708): structurally zero
// singular values come out near 4e-6·σmax (spec "Damped pseudo-inverse with rank tolerance").
const rankTol = 1e-4

// rankAbs is an absolute floor: near the SO-101 elbow singularity σmax itself collapses, and a purely
// relative cutoff would then keep the ≈2e-6 structural zero (flipping the reachable/residual split).
const rankAbs = 1e-5

// dpinv returns the damped pseudo-inverse Σ σᵢ/(σᵢ²+λ²)·vᵢuᵢᵀ of A (m×n → n×m), keeping only
// σᵢ > rankTol·σmax. With damp, λ is Chiaverini damping on σr, the smallest retained singular value.
func dpinv(A mat.Matrix, eps, maxDamping float64, damp bool) (P *mat.Dense, lambda, sigmaR float64, rank int, err error) {
	m, n := A.Dims()
	P = mat.NewDense(n, m, nil)
	var svd mat.SVD
	if !svd.Factorize(A, mat.SVDThin) {
		return nil, 0, 0, 0, errSVD
	}
	vals := svd.Values(nil)
	if len(vals) == 0 || vals[0] <= rankAbs {
		return P, 0, 0, 0, nil
	}
	for _, s := range vals {
		if s > max(rankTol*vals[0], rankAbs) {
			rank++
		}
	}
	sigmaR = vals[rank-1]
	l2 := 0.0
	if damp && sigmaR < eps {
		r := sigmaR / eps
		l2 = maxDamping * maxDamping * (1 - r*r)
	}
	var U, V mat.Dense
	svd.UTo(&U)
	svd.VTo(&V)
	for i := 0; i < rank; i++ {
		var o mat.Dense
		o.Outer(vals[i]/(vals[i]*vals[i]+l2), V.ColView(i), U.ColView(i))
		P.Add(P, &o)
	}
	return P, math.Sqrt(l2), sigmaR, rank, nil
}
