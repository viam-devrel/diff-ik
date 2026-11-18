package diffik

import (
	"math"
	"math/rand"
	"os"
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

func TestExpRotInvertsRotVec(t *testing.T) {
	for _, v := range []r3.Vector{{X: 0.3}, {Y: -1.2, Z: 0.4}, {X: 1e-9}, {}} {
		got := RotVec(ExpRot(v), quat.Number{Real: 1})
		if got.Sub(v).Norm() > 1e-12 {
			t.Errorf("RotVec(ExpRot(%v)) = %v", v, got)
		}
	}
}

// so101 loads the 5-DOF SO-101 test model (testdata/so101.urdf, meshes stripped).
func so101(t *testing.T) referenceframe.Model {
	t.Helper()
	m, err := referenceframe.ParseModelXMLFile("testdata/so101.urdf", "arm", nil)
	if err != nil {
		t.Fatal(err)
	}
	if n := len(m.DoF()); n != 5 {
		t.Fatalf("so101 DOF = %d, want 5", n)
	}
	return m
}

// panda loads the 7-DOF Franka Panda test model (testdata/panda.json, exported from the MuJoCo sim with
// dev/exportmodel).
func panda(t *testing.T) referenceframe.Model {
	t.Helper()
	b, err := os.ReadFile("testdata/panda.json")
	if err != nil {
		t.Fatal(err)
	}
	m, err := referenceframe.UnmarshalModelJSON(b, "arm")
	if err != nil {
		t.Fatal(err)
	}
	if n := len(m.DoF()); n != 7 {
		t.Fatalf("panda DOF = %d, want 7", n)
	}
	return m
}

func TestPandaFixture(t *testing.T) {
	m := panda(t)
	l, err := NewLoop(m, &simIO{}, LoopConfig{})
	if err != nil {
		t.Fatal(err)
	}
	t.Logf("panda limits: joint_deg_s %v joint_deg_s2 %v", l.Config().MaxJointDegPerS, l.Config().MaxJointDegPerS2)
	for i, lim := range m.DoF() {
		t.Logf("joint %d: [%.1f°, %.1f°]", i, rad2deg(lim.Min), rad2deg(lim.Max))
	}
}

var (
	q0a = []float64{0, 0.3, 0.3, 0.5, 0}
	q0b = []float64{0.3, 0.5, -0.8, 0.4, 0.7}
	q0c = []float64{0, 0.6, 0.6, 1.0, 0}
)

func mulVec(A mat.Matrix, x []float64) []float64 {
	var y mat.VecDense
	y.MulVec(A, mat.NewVecDense(len(x), x))
	return y.RawVector().Data
}

func nearVec(t *testing.T, got, want []float64, tol float64) {
	t.Helper()
	for i := range want {
		if math.Abs(got[i]-want[i]) > tol {
			t.Fatalf("got %v, want %v (tol %g)", got, want, tol)
		}
	}
}

func TestDpinvMatchesClosedForms(t *testing.T) {
	sq := mat.NewDense(2, 2, []float64{2, 1, 1, 3})
	P, lam, _, r, err := dpinv(sq, 0.04, 0.1, true)
	if err != nil || lam != 0 || r != 2 {
		t.Fatalf("square: λ=%v r=%d err=%v", lam, r, err)
	}
	nearVec(t, mulVec(P, []float64{1, 2}), []float64{0.2, 0.6}, 1e-12) // inverse · b

	tall := mat.NewDense(3, 2, []float64{1, 0, 0, 1, 1, 1})
	P, _, _, _, _ = dpinv(tall, 0.04, 0.1, false)
	var ata, inv, want mat.Dense
	ata.Mul(tall.T(), tall)
	_ = inv.Inverse(&ata)
	want.Mul(&inv, tall.T())
	if !mat.EqualApprox(P, &want, 1e-12) {
		t.Fatalf("tall: dpinv ≠ (AᵀA)⁻¹Aᵀ")
	}

	wide := mat.NewDense(2, 3, []float64{1, 0, 1, 0, 1, 1})
	P, _, _, _, _ = dpinv(wide, 0.04, 0.1, false)
	var aat, inv2, want2 mat.Dense
	aat.Mul(wide, wide.T())
	_ = inv2.Inverse(&aat)
	want2.Mul(wide.T(), &inv2)
	if !mat.EqualApprox(P, &want2, 1e-12) {
		t.Fatalf("wide: dpinv ≠ Aᵀ(AAᵀ)⁻¹")
	}

	zero := mat.NewDense(2, 3, nil)
	P, lam, _, r, err = dpinv(zero, 0.04, 0.1, true)
	if err != nil || r != 0 || lam != 0 || mat.Norm(P, 2) != 0 {
		t.Fatalf("zero: r=%d λ=%v err=%v", r, lam, err)
	}
}

// A rank-deficient system must not blow up, and must give the minimum-norm solution
// (the SO-101 pointing stack has a structural null direction, spec "SO-101 kinematics").
func TestDpinvRankDeficientIsMinimumNorm(t *testing.T) {
	A := mat.NewDense(2, 3, []float64{1, 1, 0, 2, 2, 0}) // rank 1
	P, lam, _, r, err := dpinv(A, 0.04, 0.1, true)
	if err != nil || r != 1 || lam != 0 {
		t.Fatalf("r=%d λ=%v err=%v", r, lam, err)
	}
	x := mulVec(P, []float64{1, 2})
	nearVec(t, x, []float64{0.5, 0.5, 0}, 1e-12) // minimum-norm solution of x+y=1
}

// Chiaverini damping keys on the smallest *retained* singular value.
func TestDpinvDampsOnlyWeakRetainedDirections(t *testing.T) {
	A := mat.NewDense(2, 2, []float64{1, 0, 0, 0.02}) // σ = 1, 0.02 < ε
	_, lam, sr, r, _ := dpinv(A, 0.04, 0.1, true)
	want := 0.1 * math.Sqrt(1-(0.02/0.04)*(0.02/0.04))
	if r != 2 || math.Abs(sr-0.02) > 1e-15 || math.Abs(lam-want) > 1e-12 {
		t.Fatalf("r=%d σr=%v λ=%v, want λ=%v", r, sr, lam, want)
	}
	B := mat.NewDense(2, 2, []float64{1, 0, 0, 1e-9}) // below rank tolerance: dropped, not damped
	_, lam, _, r, _ = dpinv(B, 0.04, 0.1, true)
	if r != 1 || lam != 0 {
		t.Fatalf("near-null direction: r=%d λ=%v, want r=1 λ=0", r, lam)
	}
}

// m < n pose form vs the n×n form at λ > 0 (push-through identity).
func TestDpinvMatchesNormalEquationsWhenDamped(t *testing.T) {
	A := mat.NewDense(2, 3, []float64{1, 0.2, 0, 0, 0.03, 1})
	P, lam, _, _, _ := dpinv(A, 2, 0.1, true) // ε above σmin = 1.0 so λ > 0
	if lam == 0 {
		t.Fatal("want λ > 0")
	}
	var ata mat.Dense
	ata.Mul(A.T(), A)
	for i := 0; i < 3; i++ {
		ata.Set(i, i, ata.At(i, i)+lam*lam)
	}
	var rhs, x mat.VecDense
	rhs.MulVec(A.T(), mat.NewVecDense(2, []float64{0.3, -0.1}))
	if err := x.SolveVec(&ata, &rhs); err != nil {
		t.Fatal(err)
	}
	nearVec(t, mulVec(P, []float64{0.3, -0.1}), x.RawVector().Data, 1e-12)
}

// Each rank-tolerance term is pinned separately: the relative one (rankTol·σmax) and the absolute
// floor (rankAbs), plus the all-below-floor guard.
func TestDpinvRankToleranceTerms(t *testing.T) {
	for _, c := range []struct {
		name string
		a, b float64
		rank int
	}{
		{"relative term", 10, 5e-4, 1},    // 5e-4 > rankAbs but < 1e-4·10
		{"absolute floor", 5e-5, 2e-6, 1}, // 2e-6 > 1e-4·5e-5 but < rankAbs
		{"all below floor", 1e-6, 0, 0},
	} {
		P, _, _, rank, err := dpinv(mat.NewDiagDense(2, []float64{c.a, c.b}), 0.04, 0.1, false)
		if err != nil || rank != c.rank {
			t.Fatalf("%s: rank %d err %v, want %d", c.name, rank, err, c.rank)
		}
		if c.rank == 0 && mat.Norm(P, 1) != 0 {
			t.Fatalf("%s: P not all zeros: %v", c.name, mat.Formatted(P))
		}
	}
}
