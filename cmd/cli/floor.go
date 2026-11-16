package main

import (
	"fmt"
	"math"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// floorGuard rejects targets that would bring the end effector or the jaw below minZ
// (base-frame z, mm). The jaw is a box in the tool frame: Z = tipMM, X/Y = ±15 mm.
// A nil guard accepts everything. Spec 3c1 "Floor guard".
type floorGuard struct {
	model referenceframe.Model
	minZ  float64
	tipMM float64
}

// lowest returns the lowest base-frame z (mm) of the jaw box at pose p, and that point's name.
func (g *floorGuard) lowest(p spatialmath.Pose) (float64, string) {
	pts := []struct {
		name string
		v    r3.Vector
	}{
		{"end effector", r3.Vector{}},
		{"jaw corner +x+y", r3.Vector{X: 15, Y: 15, Z: g.tipMM}},
		{"jaw corner +x-y", r3.Vector{X: 15, Y: -15, Z: g.tipMM}},
		{"jaw corner -x+y", r3.Vector{X: -15, Y: 15, Z: g.tipMM}},
		{"jaw corner -x-y", r3.Vector{X: -15, Y: -15, Z: g.tipMM}},
	}
	z, name := math.Inf(1), ""
	for _, pt := range pts {
		if w := spatialmath.Compose(p, spatialmath.NewPoseFromPoint(pt.v)).Point().Z; w < z {
			z, name = w, pt.name
		}
	}
	return z, name
}

func (g *floorGuard) checkPose(p spatialmath.Pose) error {
	if g == nil {
		return nil
	}
	if z, name := g.lowest(p); math.IsNaN(z) || z < g.minZ {
		return fmt.Errorf("floor guard: %s at z %.1f mm is %.1f mm below the floor (%.1f mm)", name, z, g.minZ-z, g.minZ)
	}
	return nil
}

func (g *floorGuard) checkJoints(q []float64) error {
	if g == nil {
		return nil
	}
	p, err := g.model.Transform(q)
	if err != nil {
		return err
	}
	return g.checkPose(p)
}

// checkCircle checks the circle scenario's path: points offset by radius from start in x/y, every 10°,
// at the start orientation (both run.go's and the bench's circles hold z and orientation).
func (g *floorGuard) checkCircle(start spatialmath.Pose, radius float64) error {
	if g == nil {
		return nil
	}
	for deg := 0; deg < 360; deg += 10 {
		th := float64(deg) * math.Pi / 180
		p := spatialmath.NewPose(start.Point().Add(r3.Vector{X: radius * math.Cos(th), Y: radius * math.Sin(th)}), start.Orientation())
		if err := g.checkPose(p); err != nil {
			return fmt.Errorf("circle at %d°: %w", deg, err)
		}
	}
	return nil
}

// checkPath checks n+1 points (n = max(20, ceil(largest joint change / 2°))) on the joint-space line from → to: the driver's coordinated
// MoveToJointPositions is linear in joint space. A start already below the floor (e.g. the
// rest pose) is allowed while the lowest z keeps rising; once a point is below the floor and
// not higher than the one before, or the goal is below, the move is rejected.
func (g *floorGuard) checkPath(from, to []float64) error {
	if g == nil {
		return nil
	}
	if len(from) != len(to) {
		return fmt.Errorf("floor guard: path from %d joints to %d", len(from), len(to))
	}
	maxD := 0.0
	for i := range from {
		maxD = math.Max(maxD, math.Abs(to[i]-from[i]))
	}
	n := max(20, int(math.Ceil(maxD*180/math.Pi/2)))
	prev := math.Inf(-1)
	for k := 0; k <= n; k++ {
		q := make([]float64, len(from))
		for i := range q {
			q[i] = from[i] + (to[i]-from[i])*float64(k)/float64(n)
		}
		p, err := g.model.Transform(q)
		if err != nil {
			return err
		}
		z, name := g.lowest(p)
		if math.IsNaN(z) || z < g.minZ && (k == n || z <= prev) {
			return fmt.Errorf("floor guard: move path point %d/%d: %s at z %.1f mm is %.1f mm below the floor (%.1f mm)",
				k, n, name, z, g.minZ-z, g.minZ)
		}
		prev = z
	}
	return nil
}
