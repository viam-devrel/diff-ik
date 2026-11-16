package main

import (
	"math"
	"strings"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// qStart is the stage-1 start pose (spec "Start pose").
var qStart = []float64{0, -0.61, 0.46, 1.39, 0}

func so101(t *testing.T) referenceframe.Model {
	t.Helper()
	m, err := referenceframe.ParseModelXMLFile("../../testdata/so101.urdf", "arm", nil)
	if err != nil {
		t.Fatal(err)
	}
	return m
}

func TestFloorAcceptsStartPose(t *testing.T) {
	g := &floorGuard{model: so101(t), minZ: 30, tipMM: 105}
	if err := g.checkJoints(qStart); err != nil {
		t.Fatal(err)
	}
	// the jaw box bottoms out at ~88 mm at q_start (spec review FK)
	p, _ := g.model.Transform(qStart)
	if z, _ := g.lowest(p); math.Abs(z-88.2) > 1.5 {
		t.Fatalf("lowest jaw-box z = %.1f, want ~88.2", z)
	}
}

func TestFloorRejectsLowTip(t *testing.T) {
	g := &floorGuard{model: so101(t), minZ: 30, tipMM: 105}
	// tool pointing straight down with the end effector at z 100: tip at z -5
	down := spatialmath.NewPose(r3.Vector{X: 200, Z: 100}, &spatialmath.OrientationVectorDegrees{OZ: -1})
	err := g.checkPose(down)
	if err == nil || !strings.Contains(err.Error(), "jaw corner") {
		t.Fatalf("err = %v, want a jaw rejection", err)
	}
}

func TestFloorRejectsLowEndEffector(t *testing.T) {
	g := &floorGuard{model: so101(t), minZ: 30, tipMM: 105}
	// tool pointing straight up with the end effector at z 10
	up := spatialmath.NewPose(r3.Vector{X: 200, Z: 10}, &spatialmath.OrientationVectorDegrees{OZ: 1})
	err := g.checkPose(up)
	if err == nil || !strings.Contains(err.Error(), "end effector") {
		t.Fatalf("err = %v, want an end-effector rejection", err)
	}
}

func TestFloorRejectsDippingCircle(t *testing.T) {
	g := &floorGuard{model: so101(t), minZ: 30, tipMM: 105}
	start, _ := g.model.Transform(qStart)
	// a circle 70 mm lower than start puts the jaw box at ~18 mm
	low := spatialmath.NewPose(start.Point().Add(r3.Vector{Z: -70}), start.Orientation())
	if err := g.checkCircle(low, 50); err == nil {
		t.Fatal("circle below the floor accepted")
	}
	if err := g.checkCircle(start, 50); err != nil {
		t.Fatalf("circle at q_start rejected: %v", err)
	}
}

func TestFloorPath(t *testing.T) {
	g := &floorGuard{model: so101(t), minZ: 30, tipMM: 105}
	// q_start → the singular pre-move's qa (joint 2 to −74° + 0.2 rad); spec: never below ~88 mm
	qa := []float64{0, -0.61, -74*math.Pi/180 + 0.2, 1.39, 0}
	if err := g.checkPath(qStart, qa); err != nil {
		t.Fatalf("q_start → qa rejected: %v", err)
	}
	// a path from q_start that pitches the shoulder forward and down must be rejected
	if err := g.checkPath(qStart, qLow); err == nil {
		t.Fatal("path into the floor accepted")
	}
	// starting below the floor is allowed while z only rises
	if err := g.checkPath(qLow, qStart); err != nil {
		t.Fatalf("rising path out of a low start rejected: %v", err)
	}
}

func TestFloorPathGoalBelowFloor(t *testing.T) {
	g := &floorGuard{model: so101(t), minZ: 30, tipMM: 105}
	qMid := []float64{0, 0.1, 0.46, 1.39, 0}
	p, _ := g.model.Transform(qMid)
	if z, _ := g.lowest(p); z >= 30 {
		t.Fatalf("qMid lowest z = %.1f, want < 30", z)
	}
	if err := g.checkPath(qLow, qMid); err == nil {
		t.Fatal("path ending below the floor accepted")
	}
}

func TestFloorRolledPose(t *testing.T) {
	g := &floorGuard{model: so101(t), minZ: 30, tipMM: 105}
	p, _ := g.model.Transform([]float64{0, -0.61, 0.46, 1.39, 0.785})
	want := math.Inf(1)
	for _, sx := range []float64{-15, 15} {
		for _, sy := range []float64{-15, 15} {
			z := spatialmath.Compose(p, spatialmath.NewPoseFromPoint(r3.Vector{X: sx, Y: sy, Z: 105})).Point().Z
			want = math.Min(want, z)
		}
	}
	z, _ := g.lowest(p)
	if z > want+1e-9 || z >= 87 {
		t.Fatalf("lowest = %.2f, corners min = %.2f, want <= both and < 87", z, want)
	}
}

func TestNilFloorGuardChecksNothing(t *testing.T) {
	var g *floorGuard
	if g.checkPose(spatialmath.NewZeroPose()) != nil || g.checkJoints(nil) != nil || g.checkPath(nil, nil) != nil || g.checkCircle(nil, 0) != nil {
		t.Fatal("nil guard should accept everything")
	}
}

// qLow: q_start with the shoulder pitched forward; lowest jaw-box point z ≈ −20.5 mm, and the
// lowest z rises steadily along the joint-space line to q_start.
var qLow = []float64{0, 0.3, 0.46, 1.39, 0}
