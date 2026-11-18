package main

import (
	"diffik"
	"go.viam.com/rdk/referenceframe"
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

func TestOrbitTarget(t *testing.T) {
	start := spatialmath.NewPose(r3.Vector{X: 500, Z: 300}, &spatialmath.OrientationVectorDegrees{OZ: 1})
	got := orbitTarget(start, 90)
	if p := got.Point(); p.Sub(r3.Vector{Y: 500, Z: 300}).Norm() > 1e-6 {
		t.Errorf("point %v, want (0, 500, 300)", p)
	}
	yaw0 := spatialmath.QuatToEulerAngles(start.Orientation().Quaternion()).Yaw
	yaw1 := spatialmath.QuatToEulerAngles(got.Orientation().Quaternion()).Yaw
	if d := yaw1 - yaw0; math.Abs(d-math.Pi/2) > 1e-6 {
		t.Errorf("yaw changed %v rad, want pi/2", d)
	}
}

func TestLimitReached(t *testing.T) {
	lims := []referenceframe.Limit{{Min: -1, Max: 1}, {Min: -1, Max: 1}}
	row := func(a, b float64) diffik.Status { return diffik.Status{QCmd: []float64{a, b}} }
	rows := []diffik.Status{row(0, 0), row(0.5, 0.9), row(0, 0.999)}
	if k, j := limitReached(rows, lims, 0.5); k != 2 || j != 1 {
		t.Errorf("got row %d joint %d, want 2, 1", k, j)
	}
	if k, _ := limitReached(rows[:2], lims, 0.5); k != -1 {
		t.Errorf("got row %d, want none", k)
	}
}
