package diffik

import (
	"math"
	"math/rand"
	"slices"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm/kinematics"
	"go.viam.com/rdk/referenceframe"
	"gonum.org/v1/gonum/mat"
	"gonum.org/v1/gonum/num/quat"
)

func ur5e(t *testing.T) referenceframe.Model {
	t.Helper()
	m, err := kinematics.ModelFromName(kinematics.UR5e, "arm")
	if err != nil {
		t.Fatal(err)
	}
	return m
}

// rotZ returns a quaternion for a rotation of a radians about base z.
func rotZ(a float64) quat.Number {
	return quat.Number{Real: math.Cos(a / 2), Kmag: math.Sin(a / 2)}
}

// rotX returns a quaternion for a rotation of a radians about base x.
func rotX(a float64) quat.Number {
	return quat.Number{Real: math.Cos(a / 2), Imag: math.Sin(a / 2)}
}

func TestPoseError(t *testing.T) {
	q := rotX(0.3) // different axis from the error rotation, so a body-frame error would differ
	p := r3.Vector{X: 0.4, Y: 0.1, Z: 0.2}
	for i, v := range PoseError(p, q, p, q) {
		if math.Abs(v) > 1e-12 {
			t.Fatalf("identical poses: e[%d] = %v, want 0", i, v)
		}
	}
	// target is 10 mm further in x and rotated a further 0.1 rad about z
	e := PoseError(p.Add(r3.Vector{X: 0.01}), quat.Mul(rotZ(0.1), q), p, q)
	want := [6]float64{0.01, 0, 0, 0, 0, 0.1}
	for i := range e {
		if math.Abs(e[i]-want[i]) > 1e-9 {
			t.Fatalf("e = %v, want %v", e, want)
		}
	}
}

func TestRotVecSmallAngle(t *testing.T) {
	r := RotVec(rotZ(1e-7), quat.Number{Real: 1})
	if math.Abs(r.Z-1e-7) > 1e-12 {
		t.Fatalf("RotVec of 1e-7 rad = %v, want z = 1e-7", r)
	}
}

// relErr is ‖a − b‖ / ‖a‖.
func relErr(a, b r3.Vector) float64 { return a.Sub(b).Norm() / a.Norm() }

// Perturbs one joint at a time by 1°. Perturbing all joints at once lets parallel axes cancel,
// which shrinks the true motion until second-order terms dominate the relative error.
func TestJacobianPredictsSmallMotion(t *testing.T) {
	m := ur5e(t)
	rng := rand.New(rand.NewSource(1))
	n := len(m.DoF())
	for trial := 0; trial < 20; trial++ {
		q := make([]float64, n)
		for i := range q {
			q[i] = (rng.Float64()*2 - 1) * 1.5
		}
		J, err := Jacobian(m, q)
		if err != nil {
			t.Fatal(err)
		}
		p1, r1, _ := FK(m, q)
		for j := 0; j < n; j++ {
			q2 := slices.Clone(q)
			q2[j] += deg2rad(1)
			p2, r2, _ := FK(m, q2)
			col := mat.Col(nil, j, J)
			predLin := r3.Vector{X: col[0], Y: col[1], Z: col[2]}.Mul(deg2rad(1))
			predAng := r3.Vector{X: col[3], Y: col[4], Z: col[5]}.Mul(deg2rad(1))
			if act := p2.Sub(p1); act.Norm() > 1e-5 { // skip joints whose axis passes through the tool point
				if e := relErr(act, predLin); e > 0.05 {
					t.Errorf("trial %d joint %d: linear prediction off by %.1f%%", trial, j, e*100)
				}
			}
			if e := relErr(RotVec(r2, r1), predAng); e > 0.05 {
				t.Errorf("trial %d joint %d: angular prediction off by %.1f%%", trial, j, e*100)
			}
		}
	}
}

func TestDLSStep(t *testing.T) {
	ones := [6]float64{1, 1, 1, 1, 1, 1}
	dx := [6]float64{0.01, -0.02, 0.03, 0.001, 0, -0.002}

	// Well-conditioned diagonal J: no damping, exact solve dq_i = dx_i / d_i.
	d := []float64{1, 2, 3, 1, 2, 3}
	J := mat.NewDense(6, 6, nil)
	for i, v := range d {
		J.Set(i, i, v)
	}
	dq, lambda, _, err := dlsStep(J, ones, dx, 0.04, 0.1)
	if err != nil || lambda != 0 {
		t.Fatalf("lambda = %v, err = %v; want 0, nil", lambda, err)
	}
	for i := range dq {
		if math.Abs(dq[i]-dx[i]/d[i]) > 1e-12 {
			t.Fatalf("dq = %v, want dx/d", dq)
		}
	}

	// Rank-deficient: column 6 is zero, so damping must kick in and the step stays finite.
	J = mat.NewDense(6, 6, nil)
	for i := 0; i < 5; i++ {
		J.Set(i, i, 1)
	}
	dq, lambda, sigmaMin, err := dlsStep(J, ones, dx, 0.04, 0.1)
	if err != nil {
		t.Fatal(err)
	}
	if sigmaMin != 0 || math.Abs(lambda-0.1) > 1e-12 {
		t.Fatalf("sigmaMin = %v lambda = %v, want 0 and 0.1", sigmaMin, lambda)
	}
	if dq[5] != 0 {
		t.Fatalf("dq[5] = %v, want 0 for the dead joint", dq[5])
	}
	for i := 0; i < 5; i++ {
		if want := dx[i] / 1.01; math.Abs(dq[i]-want) > 1e-12 {
			t.Fatalf("dq[%d] = %v, want %v", i, dq[i], want)
		}
	}

	// Weighted, overdetermined 6x5 J, lambda = 0: compare with a direct (JᵀWJ)⁻¹JᵀWdx using plain W.
	J = mat.NewDense(6, 5, nil)
	for i := 0; i < 5; i++ {
		J.Set(i, i, 1)
	}
	J.Set(5, 0, 1)
	J.Set(5, 1, 1)
	w := [6]float64{4, 1, 2, 1, 3, 0.5}
	W := mat.NewDiagDense(6, w[:])
	var JtW, A mat.Dense
	JtW.Mul(J.T(), W)
	A.Mul(&JtW, J)
	bv := mat.NewVecDense(6, dx[:])
	var rhs, ref mat.VecDense
	rhs.MulVec(&JtW, bv)
	if err := ref.SolveVec(&A, &rhs); err != nil {
		t.Fatal(err)
	}
	dq, lambda, _, err = dlsStep(J, w, dx, 0.04, 0.1)
	if err != nil || lambda != 0 {
		t.Fatalf("weighted: lambda = %v, err = %v; want 0, nil", lambda, err)
	}
	for i := range dq {
		if math.Abs(dq[i]-ref.AtVec(i)) > 1e-12 {
			t.Fatalf("weighted dq = %v, want %v", dq, mat.Formatted(&ref))
		}
	}
}
