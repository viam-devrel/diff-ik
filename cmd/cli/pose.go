package main

import (
	"context"
	"fmt"
	"math"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/referenceframe"
)

// showPose waits hold, then prints the measured joints, the FK end effector and the lowest
// jaw-box point. After a --start-joints move it also prints the droop: commanded minus measured
// per joint, and the end-effector error between the two (pre-servo checks 1 and 4, spec 3c1).
// cmd is the --start-joints goal, or nil.
func showPose(ctx context.Context, a arm.Arm, model referenceframe.Model, guard *floorGuard, cmd []float64, hold time.Duration) error {
	select {
	case <-ctx.Done():
		return ctx.Err()
	case <-time.After(hold):
	}
	q, err := a.JointPositions(ctx, nil)
	if err != nil {
		return err
	}
	deg := func(v []float64) []string {
		out := make([]string, len(v))
		for i, x := range v {
			out[i] = fmt.Sprintf("%.2f", x*180/math.Pi)
		}
		return out
	}
	fmt.Printf("joints (rad): %.4f\njoints (deg): %v\n", q, deg(q))
	for i, l := range model.DoF() {
		// the arm's model: on the so-101 driver these are its calibrated limits (spec check 3)
		fmt.Printf("joint %d limits (deg): [%.1f, %.1f]  margin %.1f / %.1f\n", i,
			l.Min*180/math.Pi, l.Max*180/math.Pi, (q[i]-l.Min)*180/math.Pi, (l.Max-q[i])*180/math.Pi)
	}
	p, err := model.Transform(q)
	if err != nil {
		return err
	}
	fmt.Printf("end effector (mm, base frame): %.1f\n", p.Point())
	g := guard
	if g == nil {
		g = &floorGuard{model: model, tipMM: 105}
	}
	z, name := g.lowest(p)
	fmt.Printf("lowest jaw-box point: %s at z %.1f mm\n", name, z)
	if cmd != nil {
		d := make([]float64, len(q))
		for i := range q {
			d[i] = cmd[i] - q[i]
		}
		pc, err := model.Transform(cmd)
		if err != nil {
			return err
		}
		fmt.Printf("droop after %v, commanded − measured (deg): %v\n", hold, deg(d))
		fmt.Printf("end-effector error |FK(cmd) − FK(meas)|: %.2f mm\n", pc.Point().Sub(p.Point()).Norm())
	}
	return nil
}
