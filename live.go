package diffik

import (
	"context"
	"errors"
	"fmt"
	"math"
	"sort"
	"sync/atomic"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/spatialmath"
	viz "go.viam.com/rdk/vision"
	"go.viam.com/rdk/vision/detection3d"
)

// LiveSource is what the poller needs from a vision service (tests fake it).
type LiveSource interface {
	GetProperties(ctx context.Context, extra map[string]interface{}) (*vision.Properties, error)
	GetDetections3D(ctx context.Context, cameraName string, extra map[string]interface{}) ([]*detection3d.Detection, error)
	GetObjectPointClouds(ctx context.Context, cameraName string, extra map[string]interface{}) ([]*viz.Object, error)
}

// JointReader reads the arm's measured joints (rad).
type JointReader interface {
	Joints(ctx context.Context) ([]float64, error)
}

// LiveConfig is a session's live_obstacles block (spec "Inputs").
type LiveConfig struct {
	Vision, Camera             string
	RateHz, StaleMS, TimeoutMS float64
	InflateMM                  *float64 // nil: default 10
	Max, MinPoints             int
	WorkspaceMin, WorkspaceMax *r3.Vector // world mm; both or neither
	ExtrudeToZ                 *float64   // world mm
}

func finite(xs ...float64) bool {
	for _, x := range xs {
		if math.IsNaN(x) || math.IsInf(x, 0) {
			return false
		}
	}
	return true
}

func finiteVec(v r3.Vector) bool { return finite(v.X, v.Y, v.Z) }

// withDefaults fills the zero fields (spec "Inputs").
func (c LiveConfig) withDefaults() LiveConfig {
	if c.RateHz == 0 {
		c.RateHz = 10
	}
	if c.StaleMS == 0 {
		c.StaleMS = 500
	}
	if c.TimeoutMS == 0 {
		c.TimeoutMS = c.StaleMS / 2
	}
	if c.InflateMM == nil {
		c.InflateMM = new(float64)
		*c.InflateMM = 10
	}
	if c.Max == 0 {
		c.Max = 32
	}
	if c.MinPoints == 0 {
		c.MinPoints = 10
	}
	return c
}

// validate checks a config that has had its defaults applied.
func (c LiveConfig) validate() error {
	if c.InflateMM == nil || !finite(c.RateHz, c.StaleMS, c.TimeoutMS, *c.InflateMM) {
		return fmt.Errorf("live_obstacles: every number must be finite")
	}
	switch {
	case !(c.RateHz > 0 && c.RateHz <= 50):
		return fmt.Errorf("live_obstacles.rate_hz must be in (0, 50], got %v", c.RateHz)
	case !(c.TimeoutMS > 0 && c.TimeoutMS <= c.StaleMS):
		return fmt.Errorf("live_obstacles.timeout_ms must be in (0, stale_ms], got %v", c.TimeoutMS)
	case !(c.StaleMS > 1000/c.RateHz+c.TimeoutMS):
		return fmt.Errorf("live_obstacles.stale_ms (%v) must exceed 1000/rate_hz + timeout_ms (%v)", c.StaleMS, 1000/c.RateHz+c.TimeoutMS)
	case *c.InflateMM < 0:
		return fmt.Errorf("live_obstacles.inflate_mm must be >= 0, got %v", *c.InflateMM)
	case c.Max < 1 || c.Max > 64:
		return fmt.Errorf("live_obstacles.max must be in [1, 64], got %d", c.Max)
	case c.MinPoints < 1:
		return fmt.Errorf("live_obstacles.min_points must be >= 1, got %d", c.MinPoints)
	case (c.WorkspaceMin == nil) != (c.WorkspaceMax == nil):
		return fmt.Errorf("live_obstacles.workspace needs both min_mm and max_mm")
	}
	if c.WorkspaceMin != nil {
		lo, hi := *c.WorkspaceMin, *c.WorkspaceMax
		if !finiteVec(lo) || !finiteVec(hi) {
			return fmt.Errorf("live_obstacles.workspace: every number must be finite")
		}
		if !(lo.X < hi.X && lo.Y < hi.Y && lo.Z < hi.Z) {
			return fmt.Errorf("live_obstacles.workspace: min_mm must be below max_mm on each axis")
		}
		if c.ExtrudeToZ != nil && !(*c.ExtrudeToZ >= lo.Z && *c.ExtrudeToZ <= hi.Z) {
			return fmt.Errorf("live_obstacles.extrude_to_z_mm (%v) must lie within the workspace z range", *c.ExtrudeToZ)
		}
	}
	if c.ExtrudeToZ != nil && !finite(*c.ExtrudeToZ) {
		return fmt.Errorf("live_obstacles.extrude_to_z_mm must be finite")
	}
	return nil
}

// poller turns a vision service's detections into live obstacles (spec "Data flow"). It owns its frame system
// and parts snapshot; it shares nothing with the loop but publish.
type poller struct {
	cfg               LiveConfig
	src               LiveSource
	joints            JointReader
	fsys              *referenceframe.FrameSystem
	parts             *collisionModel
	armName, srcFrame string
	use3D             bool
	publish           func([]liveObstacle)

	okFrames map[string]error // poller goroutine only

	errs, skipped, selfDropped, overflow, updates, clipped atomic.Int64
	lastErr                                                atomic.Value // string
}

// newPoller reads the source's properties, builds the poller's own frame system and parts snapshot (at q0), and
// checks the camera's frame.
func newPoller(ctx context.Context, cfg LiveConfig, src LiveSource, joints JointReader, parts []*referenceframe.FrameSystemPart,
	model referenceframe.Model, armName string, q0 []float64, publish func([]liveObstacle)) (*poller, error) {
	cfg = cfg.withDefaults()
	if err := cfg.validate(); err != nil {
		return nil, err
	}
	pctx, pcancel := context.WithTimeout(ctx, time.Duration(cfg.TimeoutMS*float64(time.Millisecond)))
	defer pcancel() // start holds the manager lock: a hung service must not stall it
	props, err := src.GetProperties(pctx, nil)
	if err != nil {
		return nil, fmt.Errorf("live_obstacles: vision %q properties: %w", cfg.Vision, err)
	}
	if !props.Detections3DSupported && !props.ObjectPCDsSupported {
		return nil, fmt.Errorf("live_obstacles: vision %q supports neither 3D detections nor object point clouds", cfg.Vision)
	}
	fsys, err := referenceframe.NewFrameSystem("live", parts, nil)
	if err != nil {
		return nil, fmt.Errorf("live_obstacles: frame system: %w", err)
	}
	snap, err := NewCollisionModel(parts, model, armName, CollisionRequest{}, q0)
	if err != nil {
		return nil, fmt.Errorf("live_obstacles: parts snapshot: %w", err)
	}
	p := &poller{cfg: cfg, src: src, joints: joints, fsys: fsys, parts: snap, armName: armName, srcFrame: cfg.Camera,
		use3D: props.Detections3DSupported, publish: publish, okFrames: map[string]error{}}
	if !p.use3D { // detections name their own frames; objects use the camera's
		if err := p.checkFrame(p.srcFrame); err != nil {
			return nil, fmt.Errorf("live_obstacles: camera: %w", err)
		}
	} else if fsys.Frame(p.srcFrame) != nil {
		if err := p.checkFrame(p.srcFrame); err != nil {
			return nil, fmt.Errorf("live_obstacles: camera: %w", err)
		}
	}
	return p, nil
}

// checkFrame reports whether detections in frame can be placed: the frame exists and is fixed or on the arm.
func (p *poller) checkFrame(frame string) error {
	if err, ok := p.okFrames[frame]; ok {
		return err
	}
	var err error
	if f := p.fsys.Frame(frame); f == nil {
		err = fmt.Errorf("frame %q is not in the frame system", frame)
	} else if _, mv, e := classifyFrame(p.fsys, f, p.armName); e != nil {
		err = e
	} else if mv != "" {
		err = fmt.Errorf("frame %q is on movable frame %q", frame, mv)
	}
	p.okFrames[frame] = err
	return err
}

func (p *poller) run(ctx context.Context) {
	t := time.NewTicker(time.Duration(float64(time.Second) / p.cfg.RateHz))
	defer t.Stop()
	for {
		select {
		case <-ctx.Done():
			return
		case <-t.C:
			p.pollOnce(ctx)
		}
	}
}

// cand is a detection on its way through the pipeline, in world mm.
type cand struct {
	geom  spatialmath.Geometry
	pts   []r3.Vector // nil: no points
	label string
}

func (p *poller) pollOnce(ctx context.Context) {
	defer func() {
		if r := recover(); r != nil {
			p.fail(fmt.Errorf("panic: %v", r))
		}
	}()
	cctx, cancel := context.WithTimeout(ctx, time.Duration(p.cfg.TimeoutMS*float64(time.Millisecond)))
	defer cancel()
	q, err := p.joints.Joints(cctx)
	if err != nil {
		p.fail(err)
		return
	}

	// Transforms for this poll: the arm at q, everything else at zero.
	in := referenceframe.NewZeroInputs(p.fsys)
	in[p.armName] = append([]float64(nil), q...)
	lin := in.ToLinearInputs()
	cache := map[string]spatialmath.Pose{}
	toWorld := func(frame string) (spatialmath.Pose, error) {
		if w, ok := cache[frame]; ok {
			return w, nil
		}
		if err := p.checkFrame(frame); err != nil {
			return nil, err
		}
		tf, err := p.fsys.Transform(lin, referenceframe.NewPoseInFrame(frame, spatialmath.NewZeroPose()), referenceframe.World)
		if err != nil {
			return nil, err
		}
		w := tf.(*referenceframe.PoseInFrame).Pose()
		cache[frame] = w
		return w, nil
	}
	tb, err := p.fsys.Transform(lin, referenceframe.NewPoseInFrame(referenceframe.World, spatialmath.NewZeroPose()), p.armName+"_origin")
	if err != nil {
		p.fail(err)
		return
	}
	worldToBase := tb.(*referenceframe.PoseInFrame).Pose()

	var cands []cand
	if p.use3D {
		ds, err := p.src.GetDetections3D(cctx, p.cfg.Camera, nil)
		if err != nil {
			p.fail(err)
			return
		}
		for _, d := range ds {
			if d == nil || len(d.Transforms) == 0 || d.Transforms[0] == nil || d.Transforms[0].Geometry() == nil {
				p.skipped.Add(1)
				continue
			}
			root := d.Transforms[0]
			w, err := toWorld(root.Parent())
			if err != nil {
				p.fail(fmt.Errorf("detection frame %q: %w", root.Parent(), err)) // a config fault: publish nothing, so the session goes stale
				return
			}
			c := cand{geom: root.Geometry().Transform(spatialmath.Compose(w, root.Pose()))}
			for _, t := range d.Transforms[1:] {
				if pc, ok := t.Geometry().(pointcloud.PointCloud); ok {
					c.pts = cloudPoints(pc, spatialmath.Compose(w, spatialmath.Compose(root.Pose(), t.Pose())))
					break
				}
			}
			best := -1
			for i, cl := range d.Classifications {
				if best < 0 || cl.Score() > d.Classifications[best].Score() {
					best = i
				}
			}
			if best >= 0 {
				c.label = d.Classifications[best].Label()
			}
			cands = append(cands, c)
		}
	} else {
		objs, err := p.src.GetObjectPointClouds(cctx, p.cfg.Camera, nil)
		if err != nil {
			p.fail(err)
			return
		}
		w, err := toWorld(p.srcFrame)
		if err != nil {
			p.fail(err)
			return
		}
		for _, o := range objs {
			if o == nil || o.Geometry == nil {
				p.skipped.Add(1)
				continue
			}
			c := cand{geom: o.Geometry.Transform(w), label: o.Geometry.Label()}
			if o.PointCloud != nil {
				c.pts = cloudPoints(o.PointCloud, w)
			}
			cands = append(cands, c)
		}
	}

	posed, err := p.parts.poseParts(q)
	if err != nil {
		p.fail(err)
		return
	}
	type scored struct {
		o liveObstacle
		d float64
	}
	var out []scored
	for _, c := range cands {
		if _, ok, err := shapeType(c.geom); err != nil || !ok || isPoint(c.geom) || !finiteGeom(c.geom) {
			p.skipped.Add(1)
			continue
		}
		if p.cfg.WorkspaceMin != nil || p.cfg.ExtrudeToZ != nil {
			g, pts, ok := p.clipRefit(c)
			if !ok {
				p.clipped.Add(1)
				continue
			}
			c.geom, c.pts = g, pts
		}
		base := c.geom.Transform(worldToBase)
		if p.selfDetection(c.pts, worldToBase, posed) {
			p.selfDropped.Add(1)
			continue
		}
		g, err := inflate(base, *p.cfg.InflateMM, c.label)
		if err != nil || !finiteGeom(g) {
			p.skipped.Add(1)
			continue
		}
		d := math.Inf(1)
		for _, part := range posed {
			if x, err := part.DistanceFrom(g); err == nil && x < d {
				d = x
			}
		}
		out = append(out, scored{liveObstacle{geom: g, label: c.label}, d})
	}
	sort.SliceStable(out, func(i, j int) bool { return out[i].d < out[j].d })
	if len(out) > p.cfg.Max {
		p.overflow.Add(int64(len(out) - p.cfg.Max))
		out = out[:p.cfg.Max]
	}
	set := make([]liveObstacle, len(out))
	for i, s := range out {
		set[i] = s.o
	}
	p.publish(set)
	p.updates.Add(1)
}

func isPoint(g spatialmath.Geometry) bool {
	cfg, err := spatialmath.NewGeometryConfig(g)
	return err != nil || cfg.Type == spatialmath.PointType
}

// cloudPoints returns pc's finite points, moved through pose.
func cloudPoints(pc pointcloud.PointCloud, through spatialmath.Pose) []r3.Vector {
	pts := make([]r3.Vector, 0, pc.Size())
	pc.Iterate(0, 0, func(v r3.Vector, _ pointcloud.Data) bool {
		if finiteVec(v) {
			pts = append(pts, spatialmath.TransformPointByPose(through, v))
		}
		return true
	})
	return pts
}

// clipRefit is spec step 4b, in world: clip the points to the workspace, refit a world-axis-aligned box around
// what is left, and extrude it down. ok is false when the detection is dropped.
func (p *poller) clipRefit(c cand) (spatialmath.Geometry, []r3.Vector, bool) {
	var lo, hi r3.Vector
	var pts []r3.Vector
	if c.pts != nil {
		pts = c.pts
		if p.cfg.WorkspaceMin != nil {
			wl, wh := *p.cfg.WorkspaceMin, *p.cfg.WorkspaceMax
			pts = make([]r3.Vector, 0, len(c.pts))
			for _, v := range c.pts {
				if v.X >= wl.X && v.X <= wh.X && v.Y >= wl.Y && v.Y <= wh.Y && v.Z >= wl.Z && v.Z <= wh.Z {
					pts = append(pts, v)
				}
			}
		}
		if p.cfg.WorkspaceMin != nil && len(pts) < p.cfg.MinPoints { // min_points applies only with a workspace
			return nil, nil, false
		}
		if len(pts) == 0 {
			return nil, nil, false
		}
		lo, hi = bounds(pts)
	} else {
		var ok bool
		if lo, hi, ok = geomAABB(c.geom); !ok {
			return nil, nil, false
		}
		if p.cfg.WorkspaceMin != nil {
			wl, wh := *p.cfg.WorkspaceMin, *p.cfg.WorkspaceMax
			lo = r3.Vector{X: math.Max(lo.X, wl.X), Y: math.Max(lo.Y, wl.Y), Z: math.Max(lo.Z, wl.Z)}
			hi = r3.Vector{X: math.Min(hi.X, wh.X), Y: math.Min(hi.Y, wh.Y), Z: math.Min(hi.Z, wh.Z)}
			if lo.X > hi.X || lo.Y > hi.Y || lo.Z > hi.Z {
				return nil, nil, false
			}
		}
	}
	if p.cfg.ExtrudeToZ != nil && *p.cfg.ExtrudeToZ < lo.Z {
		lo.Z = *p.cfg.ExtrudeToZ
	}
	d := hi.Sub(lo)
	// ponytail: a flat set of points still needs a volume; 1 mm.
	d = r3.Vector{X: math.Max(d.X, 1), Y: math.Max(d.Y, 1), Z: math.Max(d.Z, 1)}
	g, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(lo.Add(hi).Mul(0.5)), d, c.geom.Label())
	if err != nil {
		return nil, nil, false
	}
	return g, pts, true
}

func bounds(pts []r3.Vector) (lo, hi r3.Vector) {
	lo = pts[0]
	hi = pts[0]
	for _, v := range pts[1:] {
		lo = r3.Vector{X: math.Min(lo.X, v.X), Y: math.Min(lo.Y, v.Y), Z: math.Min(lo.Z, v.Z)}
		hi = r3.Vector{X: math.Max(hi.X, v.X), Y: math.Max(hi.Y, v.Y), Z: math.Max(hi.Z, v.Z)}
	}
	return lo, hi
}

// geomAABB is a conservative world-axis-aligned box around a box, sphere or capsule.
func geomAABB(g spatialmath.Geometry) (lo, hi r3.Vector, ok bool) {
	cfg, err := spatialmath.NewGeometryConfig(g)
	if err != nil {
		return lo, hi, false
	}
	var pts []r3.Vector
	var pad float64
	switch cfg.Type {
	case spatialmath.BoxType:
		for _, sx := range []float64{-1, 1} {
			for _, sy := range []float64{-1, 1} {
				for _, sz := range []float64{-1, 1} {
					pts = append(pts, spatialmath.TransformPointByPose(g.Pose(), r3.Vector{X: sx * cfg.X / 2, Y: sy * cfg.Y / 2, Z: sz * cfg.Z / 2}))
				}
			}
		}
	case spatialmath.SphereType:
		pts, pad = []r3.Vector{g.Pose().Point()}, cfg.R
	case spatialmath.CapsuleType:
		h := cfg.L/2 - cfg.R
		pts = []r3.Vector{spatialmath.TransformPointByPose(g.Pose(), r3.Vector{Z: -h}), spatialmath.TransformPointByPose(g.Pose(), r3.Vector{Z: h})}
		pad = cfg.R
	default:
		return lo, hi, false
	}
	lo, hi = bounds(pts)
	e := r3.Vector{X: pad, Y: pad, Z: pad}
	return lo.Sub(e), hi.Add(e), true
}

// selfDetection is spec step 5: more than half of up to 500 observed points (given in world mm) within 20 mm of
// any posed part (base frame) means the arm saw itself.
func (p *poller) selfDetection(pts []r3.Vector, worldToBase spatialmath.Pose, posed []spatialmath.Geometry) bool {
	if len(pts) == 0 {
		return false
	}
	step := 1.0
	if len(pts) > 500 {
		step = float64(len(pts)) / 500
	}
	n, near := 0, 0
	for f := 0.0; int(f) < len(pts) && n < 500; f += step {
		pt := spatialmath.NewPoint(spatialmath.TransformPointByPose(worldToBase, pts[int(f)]), "")
		n++
		for _, part := range posed {
			if d, err := pt.DistanceFrom(part); err == nil && d <= 20 {
				near++
				break
			}
		}
	}
	return float64(near) > 0.5*float64(n)
}

// inflate grows g by m on every side, keeping its pose, and labels it.
func inflate(g spatialmath.Geometry, m float64, label string) (spatialmath.Geometry, error) {
	cfg, err := spatialmath.NewGeometryConfig(g)
	if err != nil {
		return nil, err
	}
	switch cfg.Type {
	case spatialmath.BoxType:
		cfg.X, cfg.Y, cfg.Z = cfg.X+2*m, cfg.Y+2*m, cfg.Z+2*m
	case spatialmath.SphereType:
		cfg.R += m
	case spatialmath.CapsuleType:
		cfg.R, cfg.L = cfg.R+m, cfg.L+2*m
	default:
		return nil, fmt.Errorf("cannot inflate a %s", cfg.Type)
	}
	if label != "" {
		cfg.Label = label
	}
	return cfg.ParseConfig()
}

// finiteGeom reports whether g's pose and dimensions are all finite. A non-finite live geometry would make
// rows() error and fault the session.
func finiteGeom(g spatialmath.Geometry) bool {
	cfg, err := spatialmath.NewGeometryConfig(g)
	if err != nil || !finite(cfg.X, cfg.Y, cfg.Z, cfg.R, cfg.L) || !finiteVec(g.Pose().Point()) {
		return false
	}
	q := g.Pose().Orientation().Quaternion()
	return finite(q.Real, q.Imag, q.Jmag, q.Kmag)
}

// fail counts an error and keeps its message for LastError.
func (p *poller) fail(err error) {
	p.errs.Add(1)
	p.lastErr.Store(err.Error())
}

// LastError is the last poll error or panic message, or "".
func (p *poller) LastError() string {
	s, _ := p.lastErr.Load().(string)
	return s
}

// LiveStats is a poller's session totals.
type LiveStats struct {
	Updates, Errors, Self, Skipped, Overflow, Clipped int64
	LastError, Method                                 string
}

func (p *poller) stats() LiveStats {
	st := LiveStats{Updates: p.updates.Load(), Errors: p.errs.Load(), Self: p.selfDropped.Load(), Skipped: p.skipped.Load(),
		Overflow: p.overflow.Load(), Clipped: p.clipped.Load(), LastError: p.LastError(), Method: "object_point_clouds"}
	if p.use3D {
		st.Method = "detections3d"
	}
	return st
}

// LiveHandle is a running poller.
type LiveHandle struct {
	p      *poller
	cancel context.CancelFunc
}

// StartLive wires live obstacles into l (SetLive) and starts the poller, which owns its frame system and
// parts snapshot. ctx bounds only the start-up calls; Stop ends the poller.
func StartLive(ctx context.Context, l *Loop, src LiveSource, joints JointReader, parts []*referenceframe.FrameSystemPart,
	model referenceframe.Model, armName string, q0 []float64, cfg LiveConfig) (*LiveHandle, error) {
	if l.coll == nil || l.cfg.Solver != SolverQP {
		return nil, errors.New(`live obstacles need solver "qp" and a collision model`)
	}
	publish := l.SetLive(cfg.withDefaults().StaleMS)
	p, err := newPoller(ctx, cfg, src, joints, parts, model, armName, q0, publish)
	if err != nil {
		return nil, err
	}
	rctx, cancel := context.WithCancel(context.Background())
	go p.run(rctx)
	return &LiveHandle{p: p, cancel: cancel}, nil
}

// Stop ends the poller (idempotent, does not wait).
func (h *LiveHandle) Stop() { h.cancel() }

// Stats returns the poller's counters.
func (h *LiveHandle) Stats() LiveStats { return h.p.stats() }

// Validate applies the defaults and checks the config, so a caller can fail before anything moves.
func (c LiveConfig) Validate() error { return c.withDefaults().validate() }

// CheckLive is a dry run of StartLive's start-up checks (the properties call and the camera frame), so a
// caller can fail before anything moves. q0 only seeds the parts snapshot.
func CheckLive(ctx context.Context, src LiveSource, parts []*referenceframe.FrameSystemPart, model referenceframe.Model,
	armName string, q0 []float64, cfg LiveConfig) error {
	_, err := newPoller(ctx, cfg, src, nil, parts, model, armName, q0, nil)
	return err
}
