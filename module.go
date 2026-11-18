package diffik

import (
	"context"
	"errors"
	"fmt"
	"math"
	"slices"
	"sync"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
	motion "go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/spatialmath"
)

var (
	Servo            = resource.NewModel("devrel", "diff-ik", "servo")
	errUnimplemented = errors.New("unimplemented")
)

func init() {
	resource.RegisterService(motion.API, Servo,
		resource.Registration[motion.Service, Config]{
			Constructor: newDiffIkServo,
		},
	)
}

// Config lists the arms this service may servo.
type Config struct {
	Arms []string `json:"arms"`
	// VisionServices names the vision services a live_obstacles session may poll.
	VisionServices []string `json:"vision_services,omitempty"`
	// Teleop holds servo_start defaults (limits, rate_hz, timeout_ms, ...) for viam:teleop sessions.
	Teleop map[string]interface{} `json:"teleop,omitempty"`
}

// Validate requires at least one arm and no duplicates, and returns the arms as dependencies.
// The frame system needs no declaration: the module manager always injects it.
func (cfg Config) Validate(path string) ([]string, []string, error) {
	if len(cfg.Arms) == 0 {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "arms")
	}
	seen := map[string]bool{}
	for _, a := range cfg.Arms {
		if a == "" || seen[a] {
			return nil, nil, fmt.Errorf("%s: arms must be unique, non-empty names", path)
		}
		seen[a] = true
	}
	for _, v := range cfg.VisionServices {
		if v == "" || seen[v] {
			return nil, nil, fmt.Errorf("%s: vision_services must be unique, non-empty names that differ from the arms", path)
		}
		seen[v] = true
	}
	deps := slices.Clone(cfg.Arms)
	for _, v := range cfg.VisionServices {
		deps = append(deps, vision.Named(v).String()) // full names: a plain name clashes with a camera of the same name
	}
	return deps, nil, nil
}

type diffIkServo struct {
	resource.AlwaysRebuild

	name   resource.Name
	logger logging.Logger
	m      *servoManager

	teleopConf map[string]interface{}
	startMu    sync.Mutex // serialises teleop_start
	tmu        sync.Mutex // guards teleop and teleopErr
	teleop     map[string]*teleopEntry
	teleopErr  string // an ended session's reason, reported once by teleop_status
}

func newDiffIkServo(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (motion.Service, error) {
	conf, err := resource.NativeConfig[Config](rawConf)
	if err != nil {
		return nil, err
	}
	return NewServo(ctx, deps, rawConf.ResourceName(), conf, logger)
}

// NewServo builds the service from its dependencies: each configured arm and the injected frame system.
func NewServo(ctx context.Context, deps resource.Dependencies, name resource.Name, conf Config, logger logging.Logger) (motion.Service, error) {
	arms := map[string]arm.Arm{}
	for _, n := range conf.Arms {
		a, err := arm.FromDependencies(deps, n)
		if err != nil {
			return nil, err
		}
		arms[n] = a
	}
	fs, err := framesystem.FromDependencies(deps)
	if err != nil {
		return nil, err
	}
	m := newServoManager(arms, fs, logger)
	if len(conf.VisionServices) > 0 {
		m.visions = map[string]LiveSource{}
	}
	for _, n := range conf.VisionServices {
		v, err := vision.FromProvider(deps, n)
		if err != nil {
			return nil, err
		}
		m.visions[n] = v
	}
	srv := newServo(name, logger, m)
	srv.teleopConf = conf.Teleop
	return srv, nil
}

func newServo(name resource.Name, logger logging.Logger, m *servoManager) *diffIkServo {
	return &diffIkServo{name: name, logger: logger, m: m, teleop: map[string]*teleopEntry{}}
}

func (s *diffIkServo) Name() resource.Name { return s.name }

// Move is not implemented; it refuses an arm that has a live servo session.
func (s *diffIkServo) Move(ctx context.Context, req motion.MoveReq) (bool, error) {
	if s.m.busy(req.ComponentName) {
		return false, fmt.Errorf("arm %s busy", req.ComponentName)
	}
	return false, errUnimplemented
}

// StopPlan ramp-stops the servo session on req.ComponentName, if any.
func (s *diffIkServo) StopPlan(ctx context.Context, req motion.StopPlanReq) error {
	return s.m.stopArm(ctx, req.ComponentName)
}

func (s *diffIkServo) Close(ctx context.Context) error { return s.m.close(ctx) }

// DoCommand implements the servo protocol. See docs/superpowers/specs/2026-11-15-diff-ik-phase2-design.md.
func (s *diffIkServo) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if isTeleop(cmd) {
		return s.teleopCommand(ctx, cmd)
	}
	name, ok := cmd["command"].(string)
	if !ok {
		return nil, errors.New("command is required")
	}
	if name == "servo_start" {
		return s.servoStart(ctx, cmd)
	}
	switch name {
	case "servo_target", "servo_twist", "servo_stop", "servo_status":
	default:
		return nil, fmt.Errorf("unknown command %q", name)
	}
	id, ok := cmd["session_id"].(string)
	if !ok {
		return nil, errors.New("session_id is required")
	}
	sess, err := s.m.get(id)
	if err != nil {
		return nil, err
	}
	switch name {
	case "servo_target":
		pm, ok := cmd["pose"].(map[string]interface{})
		if !ok {
			return nil, errors.New("pose is required")
		}
		p, err := parsePose(pm)
		if err != nil {
			return nil, err
		}
		frame, err := optString(cmd, "frame", "")
		if err != nil {
			return nil, err
		}
		apply, err := optBool(cmd, "apply", true)
		if err != nil {
			return nil, err
		}
		res, err := sess.target(p, frame, apply)
		if err != nil {
			return nil, err
		}
		st := res.Status
		return map[string]interface{}{
			"state": string(st.State), "err_mm": st.ErrMM, "err_deg": st.ErrDeg,
			"reach_deg": st.ReachDeg, "pointing_residual_deg": st.PointingResidualDeg,
			"target": poseMap(res.Target), "target_raw": poseMap(res.Raw), "clamped": res.Clamped,
		}, nil
	case "servo_twist":
		lin, err := vec3(cmd, "linear")
		if err != nil {
			return nil, err
		}
		ang, err := vec3(cmd, "angular")
		if err != nil {
			return nil, err
		}
		st, err := sess.twist(lin.Mul(1/mmPerM), ang.Mul(math.Pi/180))
		if err != nil {
			return nil, err
		}
		out := map[string]interface{}{"state": string(st.State)}
		if st.RollIgnored {
			out["roll_ignored"] = true
		}
		return out, nil
	case "servo_stop":
		st, err := sess.stop(ctx)
		if err != nil {
			return nil, err
		}
		return statusMap(st), nil
	default: // servo_status
		return statusMap(sess.status()), nil
	}
}

func (s *diffIkServo) servoStart(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	sess, pose, err := s.startSession(ctx, cmd)
	if err != nil {
		return nil, err
	}
	return map[string]interface{}{"session_id": sess.id, "pose": poseMap(pose)}, nil
}

func (s *diffIkServo) startSession(ctx context.Context, cmd map[string]interface{}) (*session, spatialmath.Pose, error) {
	armName, ok := cmd["component_name"].(string)
	if !ok || armName == "" {
		return nil, nil, errors.New("component_name is required")
	}
	frame, err := optString(cmd, "frame", referenceframe.World)
	if err != nil {
		return nil, nil, err
	}
	mode, err := optString(cmd, "mode", string(ModePose))
	if err != nil {
		return nil, nil, err
	}
	rate, err := optNum(cmd, "rate_hz", 50)
	if err != nil {
		return nil, nil, err
	}
	if !(rate >= 1 && rate <= 1000) {
		return nil, nil, fmt.Errorf("rate_hz must be in [1, 1000], got %v", rate)
	}
	timeout, err := optNum(cmd, "timeout_ms", 500)
	if err != nil {
		return nil, nil, err
	}
	if !(timeout > 0 && timeout <= 10000) {
		return nil, nil, fmt.Errorf("timeout_ms must be in (0, 10000], got %v", timeout)
	}
	ramp, err := optNum(cmd, "ramp_ms", 200)
	if err != nil {
		return nil, nil, err
	}
	if !(ramp > 0 && ramp <= 5000) {
		return nil, nil, fmt.Errorf("ramp_ms must be in (0, 5000], got %v", ramp)
	}
	lc := LoopConfig{RateHz: rate}
	if v, ok := cmd["task"]; ok {
		t, ok := v.(string)
		if !ok || (t != TaskPose && t != TaskPointing) {
			return nil, nil, fmt.Errorf("unsupported task %v: use %q or %q", v, TaskPose, TaskPointing)
		}
		lc.Task = t
	}
	if raw, ok := cmd["limits"]; ok {
		lm, ok := raw.(map[string]interface{})
		if !ok {
			return nil, nil, errors.New("limits must be an object")
		}
		if v, ok := lm["joint_deg_s"]; ok {
			if lc.MaxJointDegPerS, err = numOrList(v, "limits.joint_deg_s"); err != nil {
				return nil, nil, err
			}
		}
		if v, ok := lm["joint_deg_s2"]; ok {
			if lc.MaxJointDegPerS2, err = numOrList(v, "limits.joint_deg_s2"); err != nil {
				return nil, nil, err
			}
		}
		for _, f := range []struct {
			key string
			dst *float64
		}{
			{"linear_mm_s", &lc.MaxLinearMMPerS},
			{"angular_deg_s", &lc.MaxAngularDegPerS},
			{"resync_deg", &lc.ResyncDeg},
			{"max_lag_ms", &lc.MaxLagMS},
			{"singular_eps", &lc.SingularEps},
			{"max_damping", &lc.MaxDamping},
			{"tol_mm", &lc.TolMM},
			{"tol_deg", &lc.TolDeg},
		} {
			if v, ok := lm[f.key]; ok {
				n, err := num(v, "limits."+f.key)
				if err != nil {
					return nil, nil, err
				}
				if !(n > 0) {
					return nil, nil, fmt.Errorf("limits.%s must be > 0, got %v", f.key, n)
				}
				*f.dst = n
			}
		}
	}
	if v, ok := cmd["task_weights"]; ok {
		items, ok := v.([]interface{})
		if !ok || len(items) != 6 {
			return nil, nil, fmt.Errorf("task_weights must be 6 numbers >= 0, not all zero, got %v", v)
		}
		sum := 0.0
		for i, it := range items {
			n, err := num(it, "task_weights")
			if err != nil {
				return nil, nil, err
			}
			if n < 0 {
				return nil, nil, fmt.Errorf("task_weights must be >= 0, got %v", n)
			}
			lc.TaskWeights[i], sum = n, sum+n
		}
		if sum == 0 {
			return nil, nil, errors.New("task_weights must not all be zero")
		}
		if lc.Task == TaskPointing {
			s.logger.Warnw("task_weights are ignored in pointing mode (position has priority)")
		}
	}
	solver, err := optString(cmd, "solver", SolverDLS)
	if err != nil {
		return nil, nil, err
	}
	if solver != SolverDLS && solver != SolverQP {
		return nil, nil, fmt.Errorf("unsupported solver %q: use %q or %q", solver, SolverDLS, SolverQP)
	}
	lc.Solver = solver
	if raw, ok := cmd["posture"]; ok {
		if solver != SolverQP {
			return nil, nil, errors.New(`posture needs "solver": "qp"`)
		}
		pm, ok := raw.(map[string]interface{})
		if !ok {
			return nil, nil, errors.New("posture must be an object")
		}
		p := &Posture{Weight: 1e-3, Gain: 1}
		for k := range pm {
			if k != "joints_deg" && k != "weight" && k != "gain" {
				return nil, nil, fmt.Errorf("unknown posture key %q", k)
			}
		}
		if v, ok := pm["joints_deg"]; ok {
			items, ok := v.([]interface{})
			if !ok {
				return nil, nil, errors.New("posture.joints_deg must be a list of numbers")
			}
			p.JointsDeg = make([]float64, 0, len(items)) // non-nil, so an empty list fails validate
			for _, it := range items {
				n, err := num(it, "posture.joints_deg")
				if err != nil {
					return nil, nil, err
				}
				p.JointsDeg = append(p.JointsDeg, n)
			}
		}
		for _, f := range []struct {
			key string
			dst *float64
		}{{"weight", &p.Weight}, {"gain", &p.Gain}} {
			if v, ok := pm[f.key]; ok {
				n, err := num(v, "posture."+f.key)
				if err != nil {
					return nil, nil, err
				}
				if f.key == "weight" && !(n > 0 && n <= 1e-2) {
					return nil, nil, fmt.Errorf("posture.weight must be in (0, 0.01], got %v", n)
				}
				if !(n > 0) {
					return nil, nil, fmt.Errorf("posture.%s must be > 0, got %v", f.key, n)
				}
				*f.dst = n
			}
		}
		lc.Posture = p
	}
	coll, err := parseCollision(cmd, solver)
	if err != nil {
		return nil, nil, err
	}
	live, src, err := s.parseLive(cmd, solver)
	if err != nil {
		return nil, nil, err
	}
	if live != nil && coll == nil {
		coll = &CollisionRequest{DMinMM: 10, DActMM: 100}
	}
	frames, err := optBool(cmd, "frames", false)
	if err != nil {
		return nil, nil, err
	}
	pointVel, err := optBool(cmd, "point_velocities", true)
	if err != nil {
		return nil, nil, err
	}
	filter, err := parseTargetFilter(cmd["target_filter"])
	if err != nil {
		return nil, nil, err
	}
	return s.m.start(ctx, armName, SessionConfig{
		Frame: frame, Mode: Mode(mode), Loop: lc, Collision: coll, Live: live, LiveSource: src, Frames: frames, Filter: filter, NoPointVelocities: !pointVel,
		Timeout: time.Duration(timeout * float64(time.Millisecond)),
		Ramp:    time.Duration(ramp * float64(time.Millisecond)),
	})
}

// parseLive reads live_obstacles; nil means off. It returns the vision service named by its "vision".
func (s *diffIkServo) parseLive(cmd map[string]interface{}, solver string) (*LiveConfig, LiveSource, error) {
	raw, ok := cmd["live_obstacles"]
	if !ok {
		return nil, nil, nil
	}
	lm, ok := raw.(map[string]interface{})
	if !ok {
		return nil, nil, errors.New("live_obstacles must be an object")
	}
	if solver != SolverQP {
		return nil, nil, errors.New(`live_obstacles needs "solver": "qp"`)
	}
	if b, ok := cmd["collision"].(bool); ok && !b {
		return nil, nil, errors.New(`live_obstacles needs collision, not "collision": false`)
	}
	c := &LiveConfig{}
	var err error
	for k, v := range lm {
		key := "live_obstacles." + k
		pos := func(f float64) error {
			if !(f > 0) {
				return fmt.Errorf("%s must be > 0, got %v", key, f)
			}
			return nil
		}
		switch k {
		case "vision", "camera":
			str, ok := v.(string)
			if !ok {
				return nil, nil, fmt.Errorf("%s must be a string, got %v", key, v)
			}
			if k == "vision" {
				c.Vision = str
			} else {
				c.Camera = str
			}
		case "rate_hz":
			if c.RateHz, err = num(v, key); err == nil {
				err = pos(c.RateHz)
			}
		case "stale_ms":
			if c.StaleMS, err = num(v, key); err == nil {
				err = pos(c.StaleMS)
			}
		case "timeout_ms":
			if c.TimeoutMS, err = num(v, key); err == nil {
				err = pos(c.TimeoutMS)
			}
		case "inflate_mm":
			var f float64
			if f, err = num(v, key); err == nil {
				c.InflateMM = &f
			}
		case "max", "min_points":
			var f float64
			if f, err = num(v, key); err == nil {
				if err = pos(f); err == nil && f != math.Trunc(f) {
					err = fmt.Errorf("%s must be an integer, got %v", key, f)
				}
			}
			if k == "max" {
				c.Max = int(f)
			} else {
				c.MinPoints = int(f)
			}
		case "extrude_to_z_mm":
			var f float64
			if f, err = num(v, key); err == nil {
				c.ExtrudeToZ = &f
			}
		case "workspace":
			wm, ok := v.(map[string]interface{})
			if !ok {
				return nil, nil, errors.New("live_obstacles.workspace must be an object")
			}
			for wk := range wm {
				if wk != "min_mm" && wk != "max_mm" {
					return nil, nil, fmt.Errorf("unknown live_obstacles.workspace key %q", wk)
				}
			}
			for _, w := range []struct {
				k   string
				dst **r3.Vector
			}{{"min_mm", &c.WorkspaceMin}, {"max_mm", &c.WorkspaceMax}} {
				if _, ok := wm[w.k]; !ok {
					continue
				}
				var vec r3.Vector
				if vec, err = vec3(wm, w.k); err != nil {
					return nil, nil, fmt.Errorf("live_obstacles.workspace.%w", err)
				}
				*w.dst = &vec
			}
		default:
			return nil, nil, fmt.Errorf("unknown live_obstacles key %q", k)
		}
		if err != nil {
			return nil, nil, err
		}
	}
	if c.Vision == "" {
		return nil, nil, errors.New("live_obstacles.vision is required")
	}
	if c.Camera == "" {
		return nil, nil, errors.New("live_obstacles.camera is required")
	}
	src, ok := s.m.visions[c.Vision]
	if !ok {
		return nil, nil, fmt.Errorf("live_obstacles.vision %q is not in this service's vision_services", c.Vision)
	}
	if err := c.withDefaults().validate(); err != nil {
		return nil, nil, err
	}
	return c, src, nil
}

// parseCollision reads collision, world_state and collision_specification; nil means off.
func parseCollision(cmd map[string]interface{}, solver string) (*CollisionRequest, error) {
	rawC, hasC := cmd["collision"]
	wsRaw, hasWS := cmd["world_state"]
	specRaw, hasSpec := cmd["collision_specification"]
	if !hasC && !hasWS && !hasSpec {
		return nil, nil
	}
	if b, isBool := rawC.(bool); hasC && isBool && !b && !hasWS && !hasSpec {
		return nil, nil
	}
	if solver != SolverQP {
		return nil, errors.New(`world_state, collision_specification and collision need "solver": "qp"`)
	}
	req := &CollisionRequest{DMinMM: 10, DActMM: 100}
	on := hasWS || hasSpec
	if hasC {
		switch c := rawC.(type) {
		case bool:
			on = c
		case map[string]interface{}:
			on = true
			for k := range c {
				if k != "d_min_mm" && k != "d_act_mm" {
					return nil, fmt.Errorf("unknown collision key %q", k)
				}
			}
			var err error
			if req.DMinMM, err = optNum(c, "d_min_mm", 10); err != nil {
				return nil, err
			}
			if req.DActMM, err = optNum(c, "d_act_mm", 100); err != nil {
				return nil, err
			}
			if !(req.DMinMM > 0) {
				return nil, fmt.Errorf("collision.d_min_mm must be > 0, got %v", req.DMinMM)
			}
			if !(req.DActMM > req.DMinMM) {
				return nil, fmt.Errorf("collision.d_act_mm must be > d_min_mm, got %v", req.DActMM)
			}
		default:
			return nil, errors.New("collision must be a bool or an object")
		}
	}
	var err error
	if hasWS {
		if req.WorldState, err = ParseWorldState(wsRaw); err != nil {
			return nil, err
		}
	}
	if hasSpec {
		if req.Allows, err = ParseCollisionSpecs(specRaw); err != nil {
			return nil, err
		}
	}
	if !on {
		return nil, nil
	}
	return req, nil
}

func (s *diffIkServo) Status(ctx context.Context) (map[string]interface{}, error) {
	return nil, errUnimplemented
}

// num converts a DoCommand number (float64 from protobuf, int in Go callers) and rejects NaN/Inf.
func num(v interface{}, key string) (float64, error) {
	var f float64
	switch n := v.(type) {
	case float64:
		f = n
	case int:
		f = float64(n)
	case int64:
		f = float64(n)
	default:
		return 0, fmt.Errorf("%s must be a number, got %v", key, v)
	}
	if math.IsNaN(f) || math.IsInf(f, 0) {
		return 0, fmt.Errorf("%s must be finite, got %v", key, f)
	}
	return f, nil
}

func optNum(m map[string]interface{}, key string, def float64) (float64, error) {
	v, ok := m[key]
	if !ok {
		return def, nil
	}
	return num(v, key)
}

func optBool(m map[string]interface{}, key string, def bool) (bool, error) {
	v, ok := m[key]
	if !ok {
		return def, nil
	}
	b, ok := v.(bool)
	if !ok {
		return false, fmt.Errorf("%s must be a bool, got %v", key, v)
	}
	return b, nil
}

// parseTargetFilter reads servo_start's target_filter; absent (nil) gives the pass-through default.
func parseTargetFilter(raw interface{}) (TargetFilter, error) {
	f := DefaultTargetFilter()
	if raw == nil {
		return f, nil
	}
	m, ok := raw.(map[string]interface{})
	if !ok {
		return f, errors.New("target_filter must be an object")
	}
	for k, v := range m {
		var err error
		switch k {
		case "ema_alpha":
			f.EMAAlpha, err = num(v, "target_filter.ema_alpha")
		case "hold_mm":
			f.HoldMM, err = num(v, "target_filter.hold_mm")
		case "hold_deg":
			f.HoldDeg, err = num(v, "target_filter.hold_deg")
		case "dwell_ms":
			var ms float64
			ms, err = num(v, "target_filter.dwell_ms")
			f.Dwell = time.Duration(ms * float64(time.Millisecond))
		case "feedforward":
			f.Feedforward, err = optBool(m, k, true)
		case "floor":
			fm, ok := v.(map[string]interface{})
			if !ok {
				return f, errors.New("target_filter.floor must be an object")
			}
			c := &FloorClamp{}
			for fk, fv := range fm {
				switch fk {
				case "min_z_mm":
					c.MinZMM, err = num(fv, "target_filter.floor.min_z_mm")
				case "tip_mm":
					c.TipMM, err = num(fv, "target_filter.floor.tip_mm")
				case "margin_mm":
					c.MarginMM, err = num(fv, "target_filter.floor.margin_mm")
				default:
					return f, fmt.Errorf("unknown target_filter.floor key %q", fk)
				}
				if err != nil {
					return f, err
				}
			}
			if _, ok := fm["min_z_mm"]; !ok {
				return f, errors.New("target_filter.floor.min_z_mm is required")
			}
			f.Floor = c
		default:
			return f, fmt.Errorf("unknown target_filter key %q", k)
		}
		if err != nil {
			return f, err
		}
	}
	return f, f.validate()
}

func optString(m map[string]interface{}, key, def string) (string, error) {
	v, ok := m[key]
	if !ok {
		return def, nil
	}
	s, ok := v.(string)
	if !ok {
		return "", fmt.Errorf("%s must be a string, got %v", key, v)
	}
	return s, nil
}

// numOrList parses a number or a list of numbers, each > 0.
func numOrList(v interface{}, key string) ([]float64, error) {
	items, ok := v.([]interface{})
	if !ok {
		items = []interface{}{v}
	}
	out := make([]float64, len(items))
	for i, it := range items {
		n, err := num(it, key)
		if err != nil {
			return nil, err
		}
		if !(n > 0) {
			return nil, fmt.Errorf("%s values must be > 0, got %v", key, n)
		}
		out[i] = n
	}
	return out, nil
}

func vec3(m map[string]interface{}, key string) (r3.Vector, error) {
	items, ok := m[key].([]interface{})
	if !ok || len(items) != 3 {
		return r3.Vector{}, fmt.Errorf("%s must be 3 numbers", key)
	}
	var f [3]float64
	for i, it := range items {
		n, err := num(it, key)
		if err != nil {
			return r3.Vector{}, err
		}
		f[i] = n
	}
	return r3.Vector{X: f[0], Y: f[1], Z: f[2]}, nil
}

var poseKeys = []string{"x", "y", "z", "o_x", "o_y", "o_z", "theta"}

// parsePose reads {x, y, z (mm), o_x, o_y, o_z, theta (deg)}. The orientation vector is normalised.
func parsePose(m map[string]interface{}) (spatialmath.Pose, error) {
	var f [7]float64
	for i, k := range poseKeys {
		v, ok := m[k]
		if !ok {
			return nil, fmt.Errorf("pose.%s is required", k)
		}
		n, err := num(v, "pose."+k)
		if err != nil {
			return nil, err
		}
		f[i] = n
	}
	o := r3.Vector{X: f[3], Y: f[4], Z: f[5]}
	if n := o.Norm(); !(n > 1e-9) || math.IsInf(n, 0) {
		return nil, errors.New("pose orientation vector (o_x, o_y, o_z) must be non-zero and finite")
	}
	o = o.Normalize()
	return spatialmath.NewPose(r3.Vector{X: f[0], Y: f[1], Z: f[2]},
		&spatialmath.OrientationVectorDegrees{OX: o.X, OY: o.Y, OZ: o.Z, Theta: f[6]}), nil
}

func poseMap(p spatialmath.Pose) map[string]interface{} {
	pt, ov := p.Point(), p.Orientation().OrientationVectorDegrees()
	return map[string]interface{}{"x": pt.X, "y": pt.Y, "z": pt.Z, "o_x": ov.OX, "o_y": ov.OY, "o_z": ov.OZ, "theta": ov.Theta}
}

func filterMap(f TargetFilter) map[string]interface{} {
	out := map[string]interface{}{
		"ema_alpha": f.EMAAlpha, "hold_mm": f.HoldMM, "hold_deg": f.HoldDeg,
		"dwell_ms": float64(f.Dwell) / float64(time.Millisecond), "feedforward": f.Feedforward,
	}
	if c := f.Floor; c != nil {
		out["floor"] = map[string]interface{}{"min_z_mm": c.MinZMM, "tip_mm": c.TipMM, "margin_mm": c.MarginMM}
	}
	return out
}

func statusMap(st SessionStatus) map[string]interface{} {
	js := make([]interface{}, len(st.Config.MaxJointDegPerS))
	for i, v := range st.Config.MaxJointDegPerS {
		js[i] = v
	}
	as := make([]interface{}, len(st.Config.MaxJointDegPerS2))
	for i, v := range st.Config.MaxJointDegPerS2 {
		as[i] = v
	}
	ms := func(d time.Duration) float64 { return float64(d) / float64(time.Millisecond) }
	out := map[string]interface{}{
		"task":  st.Task,
		"state": string(st.State), "err_mm": st.ErrMM, "err_deg": st.ErrDeg, "rate_hz": st.RateHz,
		"last_command_age_ms": ms(st.LastCmdAge),
		"limits": map[string]interface{}{
			"joint_deg_s": js, "joint_deg_s2": as, "linear_mm_s": st.Config.MaxLinearMMPerS,
			"angular_deg_s": st.Config.MaxAngularDegPerS, "resync_deg": st.Config.ResyncDeg, "max_lag_ms": st.Config.MaxLagMS,
			"singular_eps": st.Config.SingularEps, "max_damping": st.Config.MaxDamping,
			"tol_mm": st.Config.TolMM, "tol_deg": st.Config.TolDeg,
		},
		"tick_p50_ms": ms(st.TickP50), "tick_p99_ms": ms(st.TickP99),
		"overruns": float64(st.Overruns), "resyncs": float64(st.Resyncs), "lag_ms": st.LagMS, "damping": st.Damping,
	}
	out["clamps"] = float64(st.Clamps)
	out["target_filter"] = filterMap(st.Filter)
	if st.Task == TaskPointing {
		out["pointing_residual_deg"] = st.PointingResidualDeg
	}
	out["solver"] = st.Config.Solver
	if st.LiveMethod != "" {
		out["live_count"], out["live_age_ms"] = float64(st.LiveCount), st.LiveAgeMS
		out["live_updates"], out["live_errors"] = float64(st.LiveUpdates), float64(st.LiveErrors)
		out["live_self_dropped"], out["live_skipped"] = float64(st.LiveSelfDropped), float64(st.LiveSkipped)
		out["live_overflow"], out["live_clipped"] = float64(st.LiveOverflow), float64(st.LiveClipped)
		out["live_backing_away"], out["live_pinched"], out["live_waiting"] = st.LiveBackingAway, float64(st.LivePinched), st.LiveWaiting
		out["live_method"] = st.LiveMethod
		if st.LiveLastError != "" {
			out["live_last_error"] = st.LiveLastError
		}
	}
	qc := make([]interface{}, len(st.QCmd))
	for i, v := range st.QCmd {
		qc[i] = rad2deg(v)
	}
	out["q_cmd_deg"] = qc
	out["peak_accel_ratio"] = st.PeakAccelRatio
	if st.Config.Solver == SolverQP {
		out["qp_active"], out["qp_iters"], out["qp_fallbacks"] = float64(st.QPActive), float64(st.QPIters), float64(st.QPFallbacks)
	}
	if st.Collision != "" {
		out["collision"] = st.Collision
		if st.Collision == "on" {
			if !math.IsInf(st.CollisionMinMM, 0) && !math.IsNaN(st.CollisionMinMM) {
				out["collision_min_mm"] = st.CollisionMinMM
			}
			out["collision_pair"] = st.CollisionPair
			out["collision_rows"], out["collision_relaxed"] = float64(st.CollisionRows), float64(st.CollisionRelaxed)
			out["collision_brakes"] = float64(st.CollisionBrakes)
		}
		strs := func(v []string) []interface{} {
			o := make([]interface{}, len(v))
			for i, x := range v {
				o[i] = x
			}
			return o
		}
		out["collision_ignored"], out["collision_skipped"] = strs(st.CollisionIgnored), strs(st.CollisionSkipped)
	}
	return out
}

// MoveOnMap moves a base component to a destination Pose on a SLAM map and returns a unique ExecutionID.
// If the machine is already within PlanDeviationM of the goal, an error is returned.
// Monitor progress with `GetPlan()` and `ListPlanStatuses()`, and check the machine's position via the SLAM service.
// Designed for autonomous indoor navigation of rover bases.
func (s *diffIkServo) MoveOnMap(ctx context.Context, req motion.MoveOnMapReq) (motion.ExecutionID, error) {
	var executionIDRetVal motion.ExecutionID

	return executionIDRetVal, errUnimplemented
}

// MoveOnGlobe moves a base component to a destination GPS point(latitude, longitude and returns a unique ExecutionID.
// If the machine is already within PlanDeviationM of the goal, an error is returned.
// This non-blocking method uses a movement sensor to verify the location of the base.
// You can monitor progress with `GetPlan()` and `ListPlanStatuses()`. Designed for autonomous GPS navigation of rover bases.
func (s *diffIkServo) MoveOnGlobe(ctx context.Context, req motion.MoveOnGlobeReq) (motion.ExecutionID, error) {
	var executionIDRetVal motion.ExecutionID

	return executionIDRetVal, errUnimplemented
}

// GetPose returns the location and orientation of a component within a frame system.
// It returns a `PoseInFrame` describing the pose of the specified component relative to the specified destination frame.
// The `supplemental_transforms` argument can be used to augment the machine's existing frame system with additional frames.
// deprecated, use framesystem.Servce.GetPose
func (s *diffIkServo) GetPose(ctx context.Context, componentName string, destinationFrame string, supplementalTransforms []*referenceframe.LinkInFrame, extra map[string]interface{}) (*referenceframe.PoseInFrame, error) {
	if destinationFrame == "" {
		destinationFrame = referenceframe.World
	}
	return s.m.fs.TransformPose(ctx, referenceframe.NewPoseInFrame(componentName, spatialmath.NewZeroPose()), destinationFrame, supplementalTransforms)
}

// ListPlanStatuses returns the statuses of plans created by `MoveOnGlobe()` or `MoveOnMap()` since the motion service initialized.
// It includes plans that are in progress or have changed state in the last 24 hours.
// All repeated fields are in chronological order.
func (s *diffIkServo) ListPlanStatuses(ctx context.Context, req motion.ListPlanStatusesReq) ([]motion.PlanStatusWithID, error) {
	return nil, errUnimplemented
}

// PlanHistory returns the plan history of the most recent `MoveOnGlobe()` or `MoveOnMap()` call by default.
// The history for earlier executions can be requested by providing an ExecutionID.
// It returns a result if the execution is active or has changed state in the last 24 hours and the machine has not reinitialized.
// Plans never change; replans always create new plans and replans share the ExecutionID of the previously executing plan.
func (s *diffIkServo) PlanHistory(ctx context.Context, req motion.PlanHistoryReq) ([]motion.PlanWithStatus, error) {
	return nil, errUnimplemented
}

// TempStreamArmJointPositions derives and paces a trajectory to armName from the targets it
// receives on targets, according to opts. It blocks until targets is closed and the derived
// trajectory has finished executing on the arm, or until ctx is canceled, whichever comes
// first. Only one call may be in flight for a given armName at a time; a concurrent call for
// the same arm returns an error immediately.
//
// If ctx is canceled, or the implementation fails on its own, it must stop the arm and return
// only once the arm has stopped; until it returns, a concurrent call for the same arm keeps
// returning an error.
//
// The implementation must not return nil before targets is closed: a nil return means every
// target was consumed and the trajectory derived from them has finished executing, leaving the arm at
// rest.
//
// The implementation writes a TempStreamResponse to responses for each acknowledgment it wants to send
// back, and the framework forwards each onto the wire. The framework, not the implementation,
// owns both channels: it writes and closes targets, and it closes responses after the
// implementation returns. So an implementation only reads targets and only writes responses,
// and must not close either.
func (s *diffIkServo) TempStreamArmJointPositions(ctx context.Context, armName string, opts motion.TempStreamOptions, targets <-chan []referenceframe.Input, responses chan<- motion.TempStreamResponse, extra map[string]interface{}) error {
	return errUnimplemented
}
