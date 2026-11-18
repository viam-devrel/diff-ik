package diffik

import (
	"fmt"
	"math"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// PartsModel is the arm's collision parts alone (no obstacles), for replaying commanded joints against
// obstacles measured elsewhere (the CLI's clearance subcommand).
type PartsModel struct {
	c           *collisionModel
	worldToBase spatialmath.Pose
}

// NewPartsModel builds the parts model for armName; q seeds the build. ws (may be nil) supplies the same
// transforms the run's collision model had, so held-object and stand-in parts are included.
func NewPartsModel(parts []*referenceframe.FrameSystemPart, model referenceframe.Model, armName string, q []float64, ws *referenceframe.WorldState) (*PartsModel, error) {
	c, err := NewCollisionModel(parts, model, armName, CollisionRequest{WorldState: ws}, q)
	if err != nil {
		return nil, err
	}
	var transforms []*referenceframe.LinkInFrame
	if ws != nil {
		transforms = ws.Transforms()
	}
	fsys, err := referenceframe.NewFrameSystem("clearance", parts, transforms)
	if err != nil {
		return nil, err
	}
	in := referenceframe.NewZeroInputs(fsys)
	in[armName] = append([]float64(nil), q...)
	tb, err := fsys.Transform(in.ToLinearInputs(), referenceframe.NewPoseInFrame(referenceframe.World, spatialmath.NewZeroPose()), armName+"_origin")
	if err != nil {
		return nil, err
	}
	return &PartsModel{c: c, worldToBase: tb.(*referenceframe.PoseInFrame).Pose()}, nil
}

// Warnings lists what the build left out or ignored (movable frames, unsupported shapes such as meshes).
func (m *PartsModel) Warnings() []string { return m.c.Warnings() }

// WorldToBase is the transform from the world frame to the arm's base frame (mm).
func (m *PartsModel) WorldToBase() spatialmath.Pose { return m.worldToBase }

// MinDistance poses the parts at q and returns the least distance (mm) to any of geoms (in the arm's base
// frame) and the name of the closest part. Penetration is negative or zero per the geometry's distance.
func (m *PartsModel) MinDistance(q []float64, geoms []spatialmath.Geometry) (float64, string, error) {
	posed, err := m.c.poseParts(q)
	if err != nil {
		return 0, "", err
	}
	best, name := math.Inf(1), ""
	for i, p := range posed {
		for _, g := range geoms {
			d, err := p.DistanceFrom(g)
			if err != nil {
				return 0, "", fmt.Errorf("distance: %w", err)
			}
			if d < best {
				best, name = d, m.c.parts[i].names[0]
			}
		}
	}
	return best, name, nil
}
