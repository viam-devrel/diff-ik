package main

import (
	"context"
	"fmt"
	"math"
	"math/rand"
	"slices"
	"time"

	"diffik"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"gonum.org/v1/gonum/mat"
)

// validate is Phase 0: our FK vs the arm's EndPosition, and J·Δq vs the measured motion.
func validate(ctx context.Context, a arm.Arm, model referenceframe.Model, spread float64) (err error) {
	// Any early exit (error or Ctrl-C) must not leave the arm moving.
	defer func() {
		if err != nil {
			stopCtx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
			defer cancel()
			_ = a.Stop(stopCtx, nil)
		}
	}()
	start, err := a.JointPositions(ctx, nil)
	if err != nil {
		return err
	}
	limits := model.DoF()
	if len(start) != len(limits) {
		return fmt.Errorf("arm reports %d joints but kinematics model has %d", len(start), len(limits))
	}
	rng := rand.New(rand.NewSource(1))
	var worstMM, worstDeg, worstLin, worstAng float64
	const configs = 100
	for k := 0; k < configs; k++ {
		q := make([]float64, len(start))
		for i, lim := range limits {
			half := spread * (lim.Max - lim.Min) / 2
			q[i] = clamp(start[i]+(rng.Float64()*2-1)*half, lim.Min, lim.Max)
		}
		qm, ep, err := moveAndMeasure(ctx, a, q)
		if err != nil {
			return fmt.Errorf("config %d: %w", k, err)
		}
		p, o, err := diffik.FK(model, qm)
		if err != nil {
			return err
		}
		e := diffik.PoseError(p, o, ep.Point().Mul(1e-3), ep.Orientation().Quaternion())
		worstMM = math.Max(worstMM, vecMM(e))
		worstDeg = math.Max(worstDeg, vecDeg(e))

		// Jacobian: nudge one joint by 1° and compare predicted vs measured motion.
		j := k % len(q)
		q2 := slices.Clone(qm)
		step := math.Pi / 180
		if q2[j]+step > limits[j].Max {
			step = -step
		}
		q2[j] = clamp(q2[j]+step, limits[j].Min, limits[j].Max)
		qm2, ep2, err := moveAndMeasure(ctx, a, q2)
		if err != nil {
			return fmt.Errorf("config %d nudge: %w", k, err)
		}
		J, err := diffik.Jacobian(model, qm)
		if err != nil {
			return err
		}
		dq := make([]float64, len(qm))
		for i := range dq {
			dq[i] = qm2[i] - qm[i] // measured, not commanded, joint change
		}
		var pred mat.VecDense
		pred.MulVec(J, mat.NewVecDense(len(dq), dq))
		actLin := ep2.Point().Sub(ep.Point()).Mul(1e-3)
		actAng := diffik.RotVec(ep2.Orientation().Quaternion(), ep.Orientation().Quaternion())
		// Skip near-zero linear motion (e.g. the flange-roll joint): below 0.1 mm the
		// ~1 mm sim/kinematics offset × 1° dominates and relative error is meaningless.
		if actLin.Norm() > 1e-4 {
			worstLin = math.Max(worstLin, relErr(actLin, r3.Vector{X: pred.AtVec(0), Y: pred.AtVec(1), Z: pred.AtVec(2)}))
		}
		if actAng.Norm() > 1e-6 {
			worstAng = math.Max(worstAng, relErr(actAng, r3.Vector{X: pred.AtVec(3), Y: pred.AtVec(4), Z: pred.AtVec(5)}))
		}
	}
	if _, _, err := moveAndMeasure(ctx, a, start); err != nil {
		return err
	}

	fmt.Printf("configs:            %d\n", configs)
	fmt.Printf("FK position error:  %.3f mm  (pass ≤ 0.1)  %s\n", worstMM, pass(worstMM <= 0.1))
	fmt.Printf("FK angle error:     %.3f °   (pass ≤ 0.1)  %s\n", worstDeg, pass(worstDeg <= 0.1))
	fmt.Printf("Jacobian linear:    %.1f %%   (pass ≤ 5)    %s\n", worstLin*100, pass(worstLin <= 0.05))
	fmt.Printf("Jacobian angular:   %.1f %%   (pass ≤ 5)    %s\n", worstAng*100, pass(worstAng <= 0.05))
	if !(worstMM <= 0.1 && worstDeg <= 0.1 && worstLin <= 0.05 && worstAng <= 0.05) { // NaN-safe
		return fmt.Errorf("validate failed")
	}
	return nil
}

func moveAndMeasure(ctx context.Context, a arm.Arm, q []float64) ([]float64, spatialmath.Pose, error) {
	if err := a.MoveToJointPositions(ctx, q, nil); err != nil {
		return nil, nil, err
	}
	// A physics arm is still settling when the move returns (within its path tolerance), so
	// joints and end pose read in two calls can disagree. Bracket the pose read with two joint
	// reads and accept only when the joints didn't move in between.
	const still = 1e-4 // rad (0.006°)
	deadline := time.Now().Add(3 * time.Second)
	// ponytail: on hardware a holding servo can flicker ±1 encoder count (STS3215 ≈ 1.5e-3 rad)
	// forever, so a timeout keeps the best-settled sample with a warning instead of failing.
	var bestQ []float64
	var bestEP spatialmath.Pose
	bestMove := math.Inf(1)
	for {
		q1, err := a.JointPositions(ctx, nil)
		if err != nil {
			return nil, nil, err
		}
		ep, err := a.EndPosition(ctx, nil)
		if err != nil {
			return nil, nil, err
		}
		q2, err := a.JointPositions(ctx, nil)
		if err != nil {
			return nil, nil, err
		}
		moved := 0.0
		for i := range q1 {
			moved = math.Max(moved, math.Abs(q2[i]-q1[i]))
		}
		if moved <= still {
			return q2, ep, nil
		}
		if moved < bestMove {
			bestQ, bestEP, bestMove = q2, ep, moved
		}
		if time.Now().After(deadline) {
			fmt.Printf("warning: joints still moving %.4f° between reads 3 s after reaching %v; using the stillest sample\n",
				bestMove*180/math.Pi, q)
			return bestQ, bestEP, nil
		}
		time.Sleep(20 * time.Millisecond)
	}
}

func clamp(v, lo, hi float64) float64    { return math.Min(math.Max(v, lo), hi) }
func relErr(act, pred r3.Vector) float64 { return act.Sub(pred).Norm() / act.Norm() }
func vecMM(e [6]float64) float64         { return r3.Vector{X: e[0], Y: e[1], Z: e[2]}.Norm() * 1000 }
func vecDeg(e [6]float64) float64 {
	return r3.Vector{X: e[3], Y: e[4], Z: e[5]}.Norm() * 180 / math.Pi
}

func pass(ok bool) string {
	if ok {
		return "PASS"
	}
	return "FAIL"
}
