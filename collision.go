package diffik

import (
	"fmt"
	"sort"
	"strings"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// CollisionRequest is a session's collision input (spec "Inputs"). Zero DMinMM/DActMM take the defaults.
type CollisionRequest struct {
	WorldState *referenceframe.WorldState // may be nil
	Allows     [][2]string
	DMinMM     float64 // default 10
	DActMM     float64 // default 100
}

type collPart struct {
	names []string             // geometry label(s), owner frame name, <frame>_origin
	link  string               // arm link: the model's raw geometry label, re-posed with Geometries(q)
	geom  spatialmath.Geometry // mounted part: the geometry in the arm's end-effector frame
}

type collObstacle struct {
	names []string
	geom  spatialmath.Geometry // in the arm's base frame (<arm>_origin), fixed
}

type collPair struct {
	part, obs int
	suspended bool // closer than d_min at start; re-armed beyond d_act (spec "Pairs")
}

// collisionModel is a session's robot parts, obstacles and checked pairs (spec "The collision model").
type collisionModel struct {
	model      referenceframe.Model
	parts      []collPart
	obs        []collObstacle
	pairs      []collPair
	dMin, dAct float64
	skipped    []string // movable frames and unsupported frame-system shapes left out
	warnings   []string
}

// Warnings lists what the build left out or ignored, for the caller to log.
func (c *collisionModel) Warnings() []string { return c.warnings }

func trimOrigin(s string) string { return strings.TrimSuffix(s, "_origin") }

func afterColon(s string) string {
	if i := strings.LastIndex(s, ":"); i >= 0 {
		return s[i+1:]
	}
	return s
}

// nameSet returns the non-empty, de-duplicated names in order.
func nameSet(ns ...string) []string {
	var out []string
	seen := map[string]bool{}
	for _, n := range ns {
		if n != "" && !seen[n] {
			seen[n] = true
			out = append(out, n)
		}
	}
	return out
}

func hasName(ns []string, n string) bool {
	for _, x := range ns {
		if x == n {
			return true
		}
	}
	return false
}

// shapeType returns g's type, and whether collision supports it (box, sphere, capsule, point).
func shapeType(g spatialmath.Geometry) (spatialmath.GeometryType, bool, error) {
	cfg, err := spatialmath.NewGeometryConfig(g)
	if err != nil {
		return "", false, err
	}
	switch cfg.Type {
	case spatialmath.BoxType, spatialmath.SphereType, spatialmath.CapsuleType, spatialmath.PointType:
		return cfg.Type, true, nil
	}
	return cfg.Type, false, nil
}

func unsupported(label string, t spatialmath.GeometryType) error {
	return fmt.Errorf("collision: %s is a %s; only boxes, capsules, spheres and points are supported", label, t)
}

// onlyGeom returns the single geometry of a transformed GeometriesInFrame.
func onlyGeom(tf referenceframe.Transformable, err error) (spatialmath.Geometry, error) {
	if err != nil {
		return nil, err
	}
	gs := tf.(*referenceframe.GeometriesInFrame).Geometries()
	if len(gs) != 1 {
		return nil, fmt.Errorf("collision: transform returned %d geometries", len(gs))
	}
	return gs[0], nil
}

// NewCollisionModel builds the model from the frame-system parts (FrameSystemConfig().Parts), the arm's model,
// its frame name and the request, measuring every pair once at q (spec "The collision model").
func NewCollisionModel(parts []*referenceframe.FrameSystemPart, model referenceframe.Model, armName string, req CollisionRequest, q []float64) (*collisionModel, error) {
	if req.DMinMM == 0 {
		req.DMinMM = 10
	}
	if req.DActMM == 0 {
		req.DActMM = 100
	}
	if !(req.DMinMM > 0) {
		return nil, fmt.Errorf("collision: d_min_mm must be positive, got %v", req.DMinMM)
	}
	if !(req.DActMM > req.DMinMM) {
		return nil, fmt.Errorf("collision: d_act_mm (%v) must exceed d_min_mm (%v)", req.DActMM, req.DMinMM)
	}
	c := &collisionModel{model: model, dMin: req.DMinMM, dAct: req.DActMM}

	var transforms []*referenceframe.LinkInFrame
	heldNames := map[string]bool{}
	if req.WorldState != nil {
		transforms = req.WorldState.Transforms()
		for _, t := range transforms {
			heldNames[t.Name()] = true
		}
	}
	fsys, err := referenceframe.NewFrameSystem("collision", parts, transforms)
	if err != nil {
		return nil, fmt.Errorf("collision: frame system: %w", err)
	}
	armFrame := fsys.Frame(armName)
	if armFrame == nil {
		return nil, fmt.Errorf("collision: frame %q not in the frame system", armName)
	}
	if len(armFrame.DoF()) != len(q) {
		return nil, fmt.Errorf("collision: frame %q has %d DoF, want %d", armName, len(armFrame.DoF()), len(q))
	}
	for cur := armFrame; ; {
		p, err := fsys.Parent(cur)
		if err != nil {
			return nil, fmt.Errorf("collision: parent of %q: %w", cur.Name(), err)
		}
		if p.Name() == referenceframe.World {
			break
		}
		if len(p.DoF()) > 0 {
			return nil, fmt.Errorf("collision: arm %q is mounted on movable frame %q", armName, p.Name())
		}
		cur = p
	}
	zero := referenceframe.NewZeroInputs(fsys)
	lin := zero.ToLinearInputs()
	start := referenceframe.FrameSystemInputs{}
	for k, v := range zero {
		start[k] = v
	}
	start[armName] = append([]float64(nil), q...)
	startLin := start.ToLinearInputs()
	geoms, err := referenceframe.FrameSystemGeometries(fsys, zero)
	if err != nil {
		return nil, fmt.Errorf("collision: frame geometries: %w", err)
	}
	skipNames := map[string]bool{}
	skip := func(names ...string) {
		for _, n := range names {
			skipNames[n] = true
		}
	}

	frames := make([]string, 0, len(geoms))
	for name := range geoms {
		frames = append(frames, name)
	}
	sort.Strings(frames)
	for _, name := range frames {
		// ponytail: the arm's own frame geometry (e.g. a pedestal on armName/armName_origin) is skipped; links
		// come from the model.
		if name == armName || name == armName+"_origin" {
			continue
		}
		f := fsys.Frame(name)
		if f == nil {
			continue
		}
		// Classify by walking parents. ponytail: fsys.Parent hides the model's internal link frames, so a
		// config frame parented to arm:linkN is treated as mounted at the end effector.
		mounted, movableFrame, err := classifyFrame(fsys, f, armName)
		if err != nil {
			return nil, err
		}
		movable := movableFrame != ""
		if movable {
			c.skipped = append(c.skipped, name)
			skip(name, trimOrigin(name))
			c.warnings = append(c.warnings, fmt.Sprintf("collision: %s is on a movable frame; its geometry is not checked", name))
			continue
		}
		for _, g := range geoms[name].Geometries() {
			typ, ok, err := shapeType(g)
			if err != nil {
				return nil, fmt.Errorf("collision: %s: %w", name, err)
			}
			label := nonEmptyLabel(g.Label(), name)
			names := nameSet(trimOrigin(name), g.Label(), name, trimOrigin(name)+"_origin")
			if !ok {
				if heldNames[trimOrigin(name)] { // a held object must not silently drop out
					return nil, unsupported(label, typ)
				}
				c.skipped = append(c.skipped, fmt.Sprintf("%s (%s)", label, typ))
				skip(names...)
				c.warnings = append(c.warnings, fmt.Sprintf("collision: %s is a %s; frame-system meshes and cylinders are not checked", label, typ))
				continue
			}
			// FrameSystemGeometries returns geometries already posed in world.
			in := referenceframe.NewGeometriesInFrame(referenceframe.World, []spatialmath.Geometry{g})
			if mounted {
				tg, err := onlyGeom(fsys.Transform(lin, in, armName))
				if err != nil {
					return nil, err
				}
				c.parts = append(c.parts, collPart{names: names, geom: tg})
			} else {
				tg, err := onlyGeom(fsys.Transform(lin, in, armName+"_origin"))
				if err != nil {
					return nil, err
				}
				c.obs = append(c.obs, collObstacle{names: names, geom: tg})
			}
		}
	}

	if req.WorldState != nil {
		for _, gif := range req.WorldState.Obstacles() {
			for _, g := range gif.Geometries() {
				if typ, ok, err := shapeType(g); err != nil {
					return nil, err
				} else if !ok {
					return nil, unsupported(nonEmptyLabel(g.Label(), gif.Parent()), typ)
				}
				if pf := fsys.Frame(gif.Parent()); pf != nil {
					if _, mv, err := classifyFrame(fsys, pf, armName); err != nil {
						return nil, err
					} else if mv != "" {
						return nil, fmt.Errorf("world_state obstacle %q is in movable frame %q", nonEmptyLabel(g.Label(), gif.Parent()), mv)
					}
				}
				// Resolved at the start pose, then fixed. Transform the geometry's pose, not the geometry:
				// fsys.Transform leaves a frame's own final transform off geometries, so an obstacle in
				// arm would not follow the flange.
				pp, err := fsys.Transform(startLin, referenceframe.NewPoseInFrame(gif.Parent(), g.Pose()), armName+"_origin")
				var tg spatialmath.Geometry
				if err == nil {
					np := pp.(*referenceframe.PoseInFrame).Pose()
					tg = g.Transform(spatialmath.Compose(np, spatialmath.PoseInverse(g.Pose())))
				}
				if err != nil {
					return nil, fmt.Errorf("collision: world_state obstacle %q: %w", g.Label(), err)
				}
				c.obs = append(c.obs, collObstacle{names: nameSet(g.Label(), trimOrigin(gif.Parent()), gif.Parent(), gif.Parent()+"_origin"), geom: tg})
			}
		}
	}

	lg, err := model.Geometries(q)
	if err != nil {
		return nil, err
	}
	for _, g := range lg.Geometries() {
		raw := g.Label()
		if typ, ok, err := shapeType(g); err != nil {
			return nil, err
		} else if !ok {
			return nil, unsupported(raw, typ)
		}
		c.parts = append(c.parts, collPart{
			names: nameSet(armName+":"+afterColon(raw), raw, armName, armName+"_origin"),
			link:  raw,
		})
	}

	known := func(n string) bool {
		if skipNames[n] {
			return true
		}
		for _, p := range c.parts {
			if hasName(p.names, n) {
				return true
			}
		}
		for _, o := range c.obs {
			if hasName(o.names, n) {
				return true
			}
		}
		return false
	}
	for _, a := range req.Allows {
		for _, n := range a {
			switch {
			case skipNames[n]:
				c.warnings = append(c.warnings, fmt.Sprintf("collision_specification: %q is not checked, so allowing it does nothing", n))
			case !known(n):
				return nil, fmt.Errorf("collision_specification: %q matches no robot part or obstacle", n)
			}
		}
	}
	allowed := func(p collPart, o collObstacle) bool {
		for _, a := range req.Allows {
			if (hasName(p.names, a[0]) && hasName(o.names, a[1])) || (hasName(p.names, a[1]) && hasName(o.names, a[0])) {
				return true
			}
		}
		return false
	}
	for pi, p := range c.parts {
		for oi, o := range c.obs {
			if !allowed(p, o) {
				c.pairs = append(c.pairs, collPair{part: pi, obs: oi})
			}
		}
	}
	if len(c.pairs) == 0 {
		return c, nil
	}
	posed, err := c.poseParts(q)
	if err != nil {
		return nil, err
	}
	for i, p := range c.pairs {
		d, err := posed[p.part].DistanceFrom(c.obs[p.obs].geom)
		if err != nil {
			return nil, err
		}
		if d < c.dMin {
			c.pairs[i].suspended = true
		}
	}
	return c, nil
}

func nonEmptyLabel(a, b string) string {
	if a != "" {
		return a
	}
	return b
}

// classifyFrame walks f's parents to world. mounted: the chain reaches armName. movable: the name of the first
// frame with DoF before that (other than the arm), or "". ponytail: fsys.Parent hides the model's internal
// link frames, so a config frame parented to arm:linkN is treated as mounted at the end effector.
func classifyFrame(fsys *referenceframe.FrameSystem, f referenceframe.Frame, armName string) (mounted bool, movable string, err error) {
	for cur := f; cur.Name() != referenceframe.World; {
		if cur.Name() == armName {
			return true, "", nil
		}
		if len(cur.DoF()) > 0 {
			return false, cur.Name(), nil
		}
		p, err := fsys.Parent(cur)
		if err != nil {
			return false, "", fmt.Errorf("collision: parent of %q: %w", cur.Name(), err)
		}
		cur = p
	}
	return false, "", nil
}
