package diffik

import (
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"gonum.org/v1/gonum/mat"
)

func vnear(t *testing.T, got, want r3.Vector, tol float64) {
	t.Helper()
	if got.Sub(want).Norm() > tol {
		t.Fatalf("got %v, want %v", got, want)
	}
}

func TestToolAxisIsRotatedZ(t *testing.T) {
	vnear(t, toolAxis(rotX(math.Pi/2)), r3.Vector{Y: -1}, 1e-12) // z rotated +90° about x → −y
	vnear(t, toolAxis(rotZ(1.0)), r3.Vector{Z: 1}, 1e-12)
}

func TestPointingErrorExactAndBasis(t *testing.T) {
	ac := r3.Vector{X: 1}
	r, u, v := pointingError(ac, ac)
	if r.Norm() != 0 {
		t.Fatalf("matching axes: r = %v", r)
	}
	if math.Abs(u.Dot(ac)) > 1e-12 || math.Abs(v.Dot(ac)) > 1e-12 || math.Abs(u.Dot(v)) > 1e-12 {
		t.Fatal("u, v not orthonormal to a_c")
	}
	at := r3.Vector{X: math.Cos(0.2), Y: math.Sin(0.2)} // 0.2 rad about +z from a_c
	r, _, _ = pointingError(ac, at)
	vnear(t, r, r3.Vector{Z: 0.2}, 1e-12)

	r, u, _ = pointingError(ac, ac.Mul(-1)) // antiparallel
	if math.Abs(r.Norm()-math.Pi) > 1e-12 || math.Abs(r.Normalize().Dot(u)) < 1-1e-12 {
		t.Fatalf("antiparallel: r = %v", r)
	}
	_, u, v = pointingError(r3.Vector{Z: 1}, r3.Vector{Z: 1}) // vertical tool: x̂ fallback
	if math.Abs(u.Norm()-1) > 1e-12 || math.Abs(v.Norm()-1) > 1e-12 {
		t.Fatalf("vertical fallback: u=%v v=%v", u, v)
	}
}

// pointingRows builds J2 = [uᵀJ_ang; vᵀJ_ang] — shared by tests and the loop.
func TestPointingJacobianMatchesFiniteDifference(t *testing.T) {
	m := so101(t)
	J, err := Jacobian(m, q0b)
	if err != nil {
		t.Fatal(err)
	}
	_, o0, _ := FK(m, q0b)
	a0 := toolAxis(o0)
	_, u, v := pointingError(a0, a0) // freeze u, v at q0; target axis = a_c(q0)
	J2 := pointingRows(J, u, v)
	const h = math.Pi / 180
	for j := 0; j < 5; j++ {
		qp, qm := append([]float64(nil), q0b...), append([]float64(nil), q0b...)
		qp[j] += h
		qm[j] -= h
		_, op, _ := FK(m, qp)
		_, om, _ := FK(m, qm)
		rp, _, _ := pointingError(toolAxis(op), a0)
		rm, _, _ := pointingError(toolAxis(om), a0)
		// error is target − current, so d(e2)/dq = −J2
		fd := [2]float64{-(u.Dot(rp) - u.Dot(rm)) / (2 * h), -(v.Dot(rp) - v.Dot(rm)) / (2 * h)}
		for i := 0; i < 2; i++ {
			if a, b := J2.At(i, j), fd[i]; math.Abs(a-b) > 0.05*math.Max(math.Abs(b), 0.01) {
				t.Fatalf("joint %d row %d: J2 = %.4f, finite difference = %.4f", j, i, a, b)
			}
		}
	}
}

// Roll (joint 5 on the SO-101) moves neither position nor pointing: its J2·N1 column is ~0.
func TestSO101RollIsNullForPointing(t *testing.T) {
	m := so101(t)
	J, _ := Jacobian(m, q0a)
	_, o, _ := FK(m, q0a)
	_, u, v := pointingError(toolAxis(o), toolAxis(o))
	J2 := pointingRows(J, u, v)
	for i := 0; i < 2; i++ {
		if math.Abs(J2.At(i, 4)) > 1e-5 {
			t.Fatalf("roll column of J2 = %.2e, want ~0", J2.At(i, 4))
		}
	}
}

// Rotating u, v within the plane must not change the priority solve's Δq.
func TestPrioritySolveIsBasisInvariant(t *testing.T) {
	m := so101(t)
	J, _ := Jacobian(m, q0b)
	_, o, _ := FK(m, q0b)
	a := toolAxis(o)
	_, u, v := pointingError(a, a)
	c, s := math.Cos(0.7), math.Sin(0.7)
	u2, v2 := u.Mul(c).Add(v.Mul(s)), v.Mul(c).Sub(u.Mul(s))
	dx1 := []float64{0.001, -0.0005, 0.0008}
	e := r3.Vector{}.Add(u.Mul(0.003)).Add(v.Mul(-0.002)) // a pointing step as a vector ⟂ a
	dqA, _, _, err := prioritySolve(J.Slice(0, 3, 0, 5).(*mat.Dense), pointingRows(J, u, v), dx1,
		[]float64{u.Dot(e), v.Dot(e)}, 0.04, 0.1)
	if err != nil {
		t.Fatal(err)
	}
	dqB, _, _, _ := prioritySolve(J.Slice(0, 3, 0, 5).(*mat.Dense), pointingRows(J, u2, v2), dx1,
		[]float64{u2.Dot(e), v2.Dot(e)}, 0.04, 0.1)
	nearVec(t, dqA, dqB, 1e-10)
}

// Mid-workspace SO-101: neither level damps. This does not by itself catch a too-tight relative
// tolerance (the absolute floor masks it); see TestDpinvRankToleranceTerms.
func TestSO101MidWorkspaceIsUndamped(t *testing.T) {
	m := so101(t)
	for _, q := range [][]float64{q0a, q0b, q0c} {
		J, _ := Jacobian(m, q)
		_, o, _ := FK(m, q)
		a := toolAxis(o)
		_, u, v := pointingError(a, a)
		_, l1, l2, err := prioritySolve(J.Slice(0, 3, 0, 5).(*mat.Dense), pointingRows(J, u, v),
			[]float64{0.001, 0, 0}, []float64{0.002, 0.001}, 0.04, 0.1)
		if err != nil || l1 != 0 || l2 != 0 {
			t.Fatalf("q=%v: λ1=%v λ2=%v err=%v, want 0, 0", q, l1, l2, err)
		}
	}
}

// pointingReach: an azimuth-only pointing error is unreachable on the SO-101 (residual), an
// in-plane one is reachable.
func TestPointingReachSplitsSO101Error(t *testing.T) {
	m := so101(t)
	J, _ := Jacobian(m, q0a)
	_, o, _ := FK(m, q0a)
	a := toolAxis(o)
	_, u, v := pointingError(a, a)
	J1, J2 := J.Slice(0, 3, 0, 5).(*mat.Dense), pointingRows(J, u, v)
	// Rotating about base z changes the azimuth; its component ⟂ a is the unreachable direction.
	z := r3.Vector{Z: 1}
	zp := z.Sub(a.Mul(a.Dot(z))).Normalize()
	reach, res, err := pointingReach(J1, J2, [2]float64{u.Dot(zp) * 0.1, v.Dot(zp) * 0.1})
	if err != nil || res < 0.09 || reach > 0.01 {
		t.Fatalf("azimuth error: reach=%.4f res=%.4f err=%v, want mostly residual", reach, res, err)
	}
	pitch := a.Cross(z).Normalize() // rotation about the horizontal axis ⟂ arm plane: in-plane pitch
	reach, res, _ = pointingReach(J1, J2, [2]float64{u.Dot(pitch) * 0.1, v.Dot(pitch) * 0.1})
	if reach < 0.09 || res > 0.01 {
		t.Fatalf("pitch error: reach=%.4f res=%.4f, want mostly reachable", reach, res)
	}
}

// With λ1 = 0 the secondary task must not move the tool position: J1·dq = dx1 exactly, and the
// secondary contribution lies in the null space of J1.
func TestPrioritySecondaryLeavesPositionAlone(t *testing.T) {
	m := so101(t)
	J, _ := Jacobian(m, q0b)
	_, o, _ := FK(m, q0b)
	a := toolAxis(o)
	_, u, v := pointingError(a, a)
	J1, J2 := J.Slice(0, 3, 0, 5).(*mat.Dense), pointingRows(J, u, v)
	dx1 := []float64{0.001, -0.0005, 0.0008}
	dq, l1, _, err := prioritySolve(J1, J2, dx1, []float64{0.003, -0.002}, 1e-3, 0.1)
	if err != nil || l1 != 0 {
		t.Fatalf("λ1=%v err=%v, want 0", l1, err)
	}
	dq1, _, _, err := prioritySolve(J1, J2, dx1, []float64{0, 0}, 1e-3, 0.1)
	if err != nil {
		t.Fatal(err)
	}
	var pos, sec mat.VecDense
	pos.MulVec(J1, mat.NewVecDense(5, dq))
	nearVec(t, pos.RawVector().Data, dx1, 1e-10)
	diff := make([]float64, 5)
	for i := range diff {
		diff[i] = dq[i] - dq1[i]
	}
	sec.MulVec(J1, mat.NewVecDense(5, diff))
	if mat.Norm(&sec, 2) > 1e-10 {
		t.Fatalf("|J1·(dq-dq1)| = %g, secondary moved position", mat.Norm(&sec, 2))
	}
}
