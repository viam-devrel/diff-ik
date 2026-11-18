package diffik

import (
	"errors"
	"fmt"
	"math"

	"gonum.org/v1/gonum/mat"
)

// qpProblem is min ½xᵀHx + gᵀx subject to lb ≤ x ≤ ub and A·x ≥ b (spec "Solver: qp.go").
type qpProblem struct {
	H      *mat.SymDense // n×n, positive definite
	g      []float64     // n
	lb, ub []float64     // n
	A      *mat.Dense    // m×n, nil for none
	b      []float64     // m
}

type qpResult struct {
	x     []float64
	act   []int     // active constraint indices into qpRows' list
	u     []float64 // their multipliers, ≥ 0
	iters int
}

var qpMaxIter = 50 // a var so a test can force the cap

const qpTol = 1e-12 // a constraint counts as violated only beyond this (spec: pinned joints must not cycle)

var errQPInfeasible = errors.New("infeasible")

// qpRows lists the constraints as rows c·x ≥ d: for each joint x_i ≥ lb_i then −x_i ≥ −ub_i, then A's rows.
func qpRows(p qpProblem) (C [][]float64, d []float64) {
	n := len(p.g)
	for i := 0; i < n; i++ {
		lo, hi := make([]float64, n), make([]float64, n)
		lo[i], hi[i] = 1, -1
		C, d = append(C, lo, hi), append(d, p.lb[i], -p.ub[i])
	}
	if p.A != nil {
		m, _ := p.A.Dims()
		for k := 0; k < m; k++ {
			C, d = append(C, mat.Row(nil, k, p.A)), append(d, p.b[k])
		}
	}
	return C, d
}

// solveQP solves p and returns x, the number of active constraints and the iteration count.
func solveQP(p qpProblem) (x []float64, active, iters int, err error) {
	r, err := qpCore(p)
	return r.x, len(r.act), r.iters, err
}

// qpCore is Goldfarb–Idnani's dual active-set method (Math. Programming 27, 1983), the method behind
// quadprog. It starts at the unconstrained minimum and adds the most-violated constraint, dropping
// constraints whose multipliers reach zero, so it needs no feasible start. J is an H-orthonormal basis
// (JᵀHJ = I) whose first q columns span the active normals; R (q×q, upper triangular) is J₁ᵀN.
func qpCore(p qpProblem) (qpResult, error) {
	n := len(p.g)
	C, d := qpRows(p)
	var ch mat.Cholesky
	if !ch.Factorize(p.H) {
		return qpResult{}, errors.New("H is not positive definite")
	}
	var L, Linv mat.TriDense
	ch.LTo(&L)
	if err := Linv.InverseTri(&L); err != nil {
		return qpResult{}, err
	}
	J := make([][]float64, n) // J = L⁻ᵀ
	R := make([][]float64, n)
	for i := range J {
		J[i], R[i] = make([]float64, n), make([]float64, n)
		for j := range J[i] {
			J[i][j] = Linv.At(j, i)
		}
	}
	var xv mat.VecDense
	if err := ch.SolveVecTo(&xv, mat.NewVecDense(n, p.g)); err != nil {
		return qpResult{}, err
	}
	x := make([]float64, n)
	for i := range x {
		x[i] = -xv.AtVec(i)
	}
	dot := func(a, b []float64) float64 {
		s := 0.0
		for i := range a {
			s += a[i] * b[i]
		}
		return s
	}
	rot := func(c, s float64, a, b *float64) { *a, *b = c**a+s**b, -s**a+c**b }
	var act []int
	var u []float64
	inA := make([]bool, len(C))
	res := qpResult{}

	add := func(pc int, dv []float64) { // dv = Jᵀc_p; zero dv[q+1:] by rotating J's columns
		q := len(act)
		for j := n - 1; j > q; j-- {
			if dv[j] == 0 {
				continue
			}
			h := math.Hypot(dv[j-1], dv[j])
			c, s := dv[j-1]/h, dv[j]/h
			dv[j-1], dv[j] = h, 0
			for i := 0; i < n; i++ {
				rot(c, s, &J[i][j-1], &J[i][j])
			}
		}
		for i := 0; i <= q; i++ {
			R[i][q] = dv[i]
		}
		act, inA[pc] = append(act, pc), true
	}
	drop := func(k int) { // remove active constraint k; restore R's triangle with rotations mirrored in J
		q := len(act)
		for j := k; j < q-1; j++ {
			for i := 0; i <= j+1; i++ {
				R[i][j] = R[i][j+1]
			}
		}
		for i := 0; i < n; i++ {
			R[i][q-1] = 0
		}
		for j := k; j < q-1; j++ {
			if R[j+1][j] == 0 {
				continue
			}
			h := math.Hypot(R[j][j], R[j+1][j])
			c, s := R[j][j]/h, R[j+1][j]/h
			for col := j; col < q-1; col++ {
				rot(c, s, &R[j][col], &R[j+1][col])
			}
			R[j+1][j] = 0
			for i := 0; i < n; i++ {
				rot(c, s, &J[i][j], &J[i][j+1])
			}
		}
		inA[act[k]] = false
		act = append(act[:k], act[k+1:]...)
	}

	for {
		pc, worst := -1, -qpTol
		for i := range C {
			if !inA[i] {
				if s := dot(C[i], x) - d[i]; s < worst {
					pc, worst = i, s
				}
			}
		}
		if pc < 0 {
			res.x, res.act, res.u = x, act, u
			return res, nil
		}
		up := append(append([]float64(nil), u...), 0)
		for {
			res.iters++
			if res.iters > qpMaxIter {
				return res, fmt.Errorf("no convergence in %d iterations", qpMaxIter)
			}
			q := len(act)
			dv := make([]float64, n)
			for j := 0; j < n; j++ {
				for i := 0; i < n; i++ {
					dv[j] += J[i][j] * C[pc][i]
				}
			}
			z := make([]float64, n) // step direction in primal space: J₂d₂
			for j := q; j < n; j++ {
				for i := 0; i < n; i++ {
					z[i] += J[i][j] * dv[j]
				}
			}
			r := make([]float64, q) // R⁻¹d₁: how the active multipliers change
			for i := q - 1; i >= 0; i-- {
				s := dv[i]
				for k := i + 1; k < q; k++ {
					s -= R[i][k] * r[k]
				}
				r[i] = s / R[i][i]
			}
			t1, k := math.Inf(1), -1 // partial step: a multiplier reaches zero
			for j := 0; j < q; j++ {
				if r[j] > qpTol {
					if t := up[j] / r[j]; t < t1 {
						t1, k = t, j
					}
				}
			}
			t2 := math.Inf(1) // full step: constraint pc becomes active
			if zc := dot(z, C[pc]); zc > qpTol {
				t2 = -(dot(C[pc], x) - d[pc]) / zc
			}
			t := math.Min(t1, t2)
			if math.IsInf(t, 1) {
				return res, errQPInfeasible
			}
			for j := 0; j < q; j++ {
				up[j] -= t * r[j]
			}
			up[q] += t
			if !math.IsInf(t2, 1) {
				for i := range x {
					x[i] += t * z[i]
				}
			}
			if t == t2 {
				add(pc, dv)
				u = up
				break
			}
			drop(k)
			up = append(up[:k], up[k+1:]...)
		}
	}
}
