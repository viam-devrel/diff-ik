package diffik

import (
	"math"
	"math/rand"
	"slices"
	"testing"
	"time"

	"gonum.org/v1/gonum/mat"
)

func sym(n int, v ...float64) *mat.SymDense { return mat.NewSymDense(n, v) }

func wide(n int) (lb, ub []float64) {
	lb, ub = make([]float64, n), make([]float64, n)
	for i := range lb {
		lb[i], ub[i] = -1e6, 1e6
	}
	return lb, ub
}

// nearVec is the existing helper in kin_test.go; don't redeclare it.

func TestQPUnconstrained(t *testing.T) {
	lb, ub := wide(2)
	x, act, _, err := solveQP(qpProblem{H: sym(2, 2, 0, 0, 4), g: []float64{-2, -8}, lb: lb, ub: ub})
	if err != nil {
		t.Fatal(err)
	}
	nearVec(t, x, []float64{1, 2}, 1e-12)
	if act != 0 {
		t.Fatalf("active = %d, want 0", act)
	}
}

func TestQPBounds(t *testing.T) {
	x, act, _, err := solveQP(qpProblem{H: sym(2, 2, 0, 0, 4), g: []float64{-2, -8},
		lb: []float64{-1, -1}, ub: []float64{0.5, 1}})
	if err != nil {
		t.Fatal(err)
	}
	nearVec(t, x, []float64{0.5, 1}, 1e-12)
	if act != 2 {
		t.Fatalf("active = %d, want 2", act)
	}
}

func TestQPCoupledBound(t *testing.T) {
	// unconstrained minimum (1, 1); with x0 ≤ 0 the minimum is (0, 1.5)
	x, _, _, err := solveQP(qpProblem{H: sym(2, 2, 1, 1, 2), g: []float64{-3, -3},
		lb: []float64{-10, -10}, ub: []float64{0, 10}})
	if err != nil {
		t.Fatal(err)
	}
	nearVec(t, x, []float64{0, 1.5}, 1e-12)
}

func TestQPGeneralRowExcludingOrigin(t *testing.T) {
	lb, ub := wide(2)
	x, _, _, err := solveQP(qpProblem{H: sym(2, 1, 0, 0, 1), g: []float64{0, 0}, lb: lb, ub: ub,
		A: mat.NewDense(1, 2, []float64{1, 1}), b: []float64{2}})
	if err != nil {
		t.Fatal(err)
	}
	nearVec(t, x, []float64{1, 1}, 1e-12)
}

func TestQPPinnedBound(t *testing.T) {
	x, _, _, err := solveQP(qpProblem{H: sym(1, 1), g: []float64{-5}, lb: []float64{0.3}, ub: []float64{0.3}})
	if err != nil {
		t.Fatal(err)
	}
	nearVec(t, x, []float64{0.3}, 1e-12)
}

func TestQPErrors(t *testing.T) {
	if _, _, _, err := solveQP(qpProblem{H: sym(1, 1), g: []float64{0}, lb: []float64{1}, ub: []float64{0}}); err == nil {
		t.Error("infeasible bounds: want error")
	}
	lb, ub := wide(1)
	if _, _, _, err := solveQP(qpProblem{H: sym(1, 1), g: []float64{0}, lb: lb, ub: ub,
		A: mat.NewDense(2, 1, []float64{1, -1}), b: []float64{1, 0}}); err == nil {
		t.Error("infeasible rows: want error")
	}
	if _, _, _, err := solveQP(qpProblem{H: sym(2, 1, 0, 0, -1), g: []float64{0, 0}, lb: []float64{-1, -1}, ub: []float64{1, 1}}); err == nil {
		t.Error("indefinite H: want error")
	}
	orig := qpMaxIter
	t.Cleanup(func() { qpMaxIter = orig })
	qpMaxIter = 1
	if _, _, _, err := solveQP(qpProblem{H: sym(2, 2, 0, 0, 4), g: []float64{-2, -8},
		lb: []float64{-1, -1}, ub: []float64{0.5, 1}}); err == nil {
		t.Error("iteration cap: want error")
	}
}

// randomQP builds a positive definite problem whose bounds and rows all hold at a known point.
func randomQP(r *rand.Rand) qpProblem {
	n := 1 + r.Intn(7)
	M := mat.NewDense(n, n, nil)
	for i := 0; i < n; i++ {
		for j := 0; j < n; j++ {
			M.Set(i, j, r.NormFloat64())
		}
	}
	H := mat.NewSymDense(n, nil)
	for i := 0; i < n; i++ {
		for j := i; j < n; j++ {
			v := mat.Dot(M.ColView(i), M.ColView(j))
			if i == j {
				v += 0.1
			}
			H.SetSym(i, j, v)
		}
	}
	xs, g, lb, ub := make([]float64, n), make([]float64, n), make([]float64, n), make([]float64, n)
	for i := range xs {
		xs[i], g[i] = r.NormFloat64(), 3*r.NormFloat64()
		lb[i], ub[i] = xs[i]-r.Float64(), xs[i]+r.Float64()
	}
	m := r.Intn(6)
	var A *mat.Dense
	b := make([]float64, m)
	if m > 0 {
		A = mat.NewDense(m, n, nil)
		for k := 0; k < m; k++ {
			s := 0.0
			for j := 0; j < n; j++ {
				A.Set(k, j, r.NormFloat64())
				s += A.At(k, j) * xs[j]
			}
			b[k] = s - r.Float64()
		}
	}
	return qpProblem{H: H, g: g, lb: lb, ub: ub, A: A, b: b}
}

func TestQPPropertyKKT(t *testing.T) {
	r := rand.New(rand.NewSource(1))
	for trial := 0; trial < 2000; trial++ {
		p := randomQP(r)
		res, err := qpCore(p)
		if err != nil {
			t.Fatalf("trial %d: %v", trial, err)
		}
		C, d := qpRows(p)
		n := len(p.g)
		for i := range C {
			if s := mat.Dot(mat.NewVecDense(n, C[i]), mat.NewVecDense(n, res.x)) - d[i]; s < -1e-9 {
				t.Fatalf("trial %d: constraint %d violated by %g", trial, i, -s)
			}
		}
		// stationarity: Hx + g = Σ u_j c_j over the active set, u ≥ 0
		grad := make([]float64, n)
		for i := 0; i < n; i++ {
			grad[i] = p.g[i]
			for j := 0; j < n; j++ {
				grad[i] += p.H.At(i, j) * res.x[j]
			}
		}
		if len(res.u) != len(res.act) {
			t.Fatalf("trial %d: %d multipliers for %d active constraints", trial, len(res.u), len(res.act))
		}
		for k, c := range res.act {
			if s := mat.Dot(mat.NewVecDense(n, C[c]), mat.NewVecDense(n, res.x)) - d[c]; math.Abs(s) > 1e-9 {
				t.Fatalf("trial %d: active constraint %d not tight (slack %g)", trial, c, s)
			}
			if res.u[k] < -1e-9 {
				t.Fatalf("trial %d: negative multiplier %g", trial, res.u[k])
			}
			for i := 0; i < n; i++ {
				grad[i] -= res.u[k] * C[c][i]
			}
		}
		for i, v := range grad {
			if math.Abs(v) > 1e-9 {
				t.Fatalf("trial %d: stationarity residual %g at %d", trial, v, i)
			}
		}
	}
}

// spec Task 0 item 3: p99 solve time under 0.5 ms at n = 7 with 7 bounds active.
func TestQPSolveTime(t *testing.T) {
	if testing.Short() {
		t.Skip("timing")
	}
	n := 7
	H := mat.NewSymDense(n, nil)
	g, lb, ub := make([]float64, n), make([]float64, n), make([]float64, n)
	for i := 0; i < n; i++ {
		for j := i; j < n; j++ {
			H.SetSym(i, j, 0.1)
		}
		H.SetSym(i, i, 1+float64(i))
		g[i], lb[i], ub[i] = -10, -0.01, 0.01 // the unconstrained minimum is far outside: every upper bound binds
	}
	p := qpProblem{H: H, g: g, lb: lb, ub: ub}
	ds := make([]time.Duration, 10000)
	for k := range ds {
		t0 := time.Now()
		if _, act, _, err := solveQP(p); err != nil || act != n {
			t.Fatalf("act %d err %v", act, err)
		}
		ds[k] = time.Since(t0)
	}
	slices.Sort(ds)
	p99 := ds[len(ds)*99/100]
	t.Logf("solveQP n=7, 7 active: p50 %v p99 %v", ds[len(ds)/2], p99)
	J := mat.NewDense(6, 7, nil)
	for i := 0; i < 6; i++ {
		for j := 0; j < 7; j++ {
			J.Set(i, j, math.Sin(float64(i*7+j)))
		}
	}
	t0 := time.Now()
	for k := 0; k < 1000; k++ {
		if _, _, _, err := dlsStep(J, [6]float64{1, 1, 1, 1, 1, 1}, [6]float64{1e-3}, 0.04, 0.1); err != nil {
			t.Fatal(err)
		}
	}
	t.Logf("dlsStep 6×7: mean %v", time.Since(t0)/1000)
	if p99 > 500*time.Microsecond {
		t.Fatalf("p99 %v exceeds 0.5 ms", p99)
	}
}
