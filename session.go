package diffik

import (
	"context"
	"crypto/rand"
	"errors"
	"fmt"
	"maps"
	"math"
	"slices"
	"sync"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/spatialmath"
	"gonum.org/v1/gonum/num/quat"
)

// velEstimator estimates a pose-mode target's velocity from successive servo_target poses
// (spec "Target-velocity estimator"). maxLin and maxAng are the session's limits in m/s and rad/s.
type velEstimator struct {
	maxLin, maxAng float64

	have   bool
	lastT  time.Time
	lastP  r3.Vector // m
	lastQ  quat.Number
	v      [6]float64
	seeded bool
}

// update records pose p (mm, base frame) received at t and returns the velocity estimate (m/s, rad/s).
func (e *velEstimator) update(t time.Time, p spatialmath.Pose) [6]float64 {
	pm, q := p.Point().Mul(1/mmPerM), p.Orientation().Quaternion()
	dt := t.Sub(e.lastT).Seconds()
	if e.have && dt < 0.005 {
		return e.v // too close to differentiate; keep the earlier sample as the reference
	}
	if !e.have || dt > 0.5 {
		e.v, e.seeded = [6]float64{}, false // a jump, not motion
	} else {
		lin, ang := pm.Sub(e.lastP).Mul(1/dt), RotVec(q, e.lastQ).Mul(1/dt)
		raw := [6]float64{lin.X, lin.Y, lin.Z, ang.X, ang.Y, ang.Z}
		switch {
		case lin.Norm() < 0.001 && ang.Norm() < deg2rad(0.5):
			e.v, e.seeded = [6]float64{}, false // a resent pose: stationary
		case lin.Norm() > 1.5*e.maxLin || ang.Norm() > 1.5*e.maxAng:
			e.v, e.seeded = [6]float64{}, false // a step, not motion
		case !e.seeded:
			e.v, e.seeded = raw, true
		default:
			for i := range e.v {
				e.v[i] = 0.5*raw[i] + 0.5*e.v[i]
			}
		}
	}
	e.have, e.lastT, e.lastP, e.lastQ = true, t, pm, q
	return e.v
}

// Mode is a servo session's command mode, fixed for the session's life.
type Mode string

// Session modes.
const (
	ModePose  Mode = "pose"
	ModeTwist Mode = "twist"
)

var errUnknownSession = errors.New("unknown session_id")

// poseTransformer is the part of framesystem.Service a session needs.
type poseTransformer interface {
	FrameSystemConfig(ctx context.Context) (*framesystem.Config, error)
	TransformPose(ctx context.Context, pose *referenceframe.PoseInFrame, dst string,
		supplementalTransforms []*referenceframe.LinkInFrame) (*referenceframe.PoseInFrame, error)
}

// SessionConfig is a parsed servo_start request. Loop.RateHz must be set. A one-element
// Loop.MaxJointDegPerS or Loop.MaxJointDegPerS2 applies to every joint.
type SessionConfig struct {
	Frame     string // "world" or "<arm>_origin"
	Mode      Mode
	Loop      LoopConfig
	Timeout   time.Duration // dead-man
	Ramp      time.Duration
	Collision *CollisionRequest // nil: no collision avoidance (solver qp only)
	Frames    bool              // build a local frame system at start so servo_target can take a camera frame
	Filter    TargetFilter      // the zero value means DefaultTargetFilter

	NoPointVelocities bool // armIO only: stream positions without per-point target velocities

	Live       *LiveConfig // nil: no live obstacles (solver qp only)
	LiveSource LiveSource  // the vision service; Task 6's startSession fills it, tests set it directly
	// LiveJoints reads the joints for the poller; nil means the arm's JointPositions. Tests inject it.
	LiveJoints JointReader

	Guard       *GuardConfig // nil: no force guard
	GuardSource ForceSource

	Force       *ForceConfig // nil: no force control (twist mode only)
	ForceSource ForceSource
}

// armJoints adapts an arm's JointPositions to a JointReader.
type armJoints struct{ a arm.Arm }

func (j armJoints) Joints(ctx context.Context) ([]float64, error) {
	in, err := j.a.JointPositions(ctx, nil)
	if err != nil {
		return nil, err
	}
	return in, nil
}

// SessionStatus is a snapshot of a session, live or final.
type SessionStatus struct {
	Status
	Config      LoopConfig
	LastCmdAge  time.Duration
	RollIgnored bool // servo_twist only: an angular component about the tool axis was dropped (pointing mode)
	Filter      TargetFilter
	Clamps      int // targets the floor clamp has raised

	// live obstacles: the poller's session totals (zero when live_obstacles is off)
	LiveUpdates, LiveErrors, LiveSelfDropped, LiveSkipped, LiveOverflow, LiveClipped int64
	LiveLastError, LiveMethod                                                        string

	Guard *GuardStats // nil unless the session has a guard
}

// TargetResult is servo_target's outcome. Raw is the target in the session frame before the filter,
// Target after it (equal to Raw for apply:false).
type TargetResult struct {
	Status      SessionStatus
	Target, Raw spatialmath.Pose
	Clamped     bool
}

// session is one arm's servo session: a loop plus the protocol state around it.
type session struct {
	id            string
	mode          Mode
	loop          *Loop
	twb           spatialmath.Pose // the arm base's pose in the session frame
	now           func() time.Time
	timeout, ramp time.Duration
	done          chan struct{} // closed once the loop has ended and final is set
	cancel        context.CancelFunc
	logger        logging.Logger
	armName       string
	poller        *poller // nil unless live obstacles
	guard         *guard  // nil unless a force guard

	// Set once in start() and read-only after, so the frame transform needs no s.mu.
	fs     *referenceframe.FrameSystem // nil unless Frames
	frame  string                      // the session frame
	startQ []float64
	zeroIn referenceframe.FrameSystemInputs // computed once: NewZeroInputs fills a cache lazily, unsynchronised
	ff     bool
	cmu    sync.Mutex
	chains map[string]error // checkChain results per frame, under cmu

	mu       sync.Mutex
	lastLate string // last logged Status.CollisionLate
	est      velEstimator
	filt     *targetFilter
	lastCmd  time.Time
	stopping State // non-empty once a stop has begun: the final state it will end in
	final    SessionStatus
	runErr   error
}

func (s *session) run(ctx context.Context) { s.finish(s.loop.Run(ctx)) }

// finish records the final status and marks the session ended.
func (s *session) finish(err error) {
	st := s.status()
	switch st.State {
	case StateStopped, StateStoppedTimeout, StateStoppedStale, StateStoppedContact, StateFaulted:
	default:
		st.State = StateStopped // Run ended by context cancel
	}
	s.mu.Lock()
	s.final, s.runErr = st, err
	s.mu.Unlock()
	if stopFailed(err) {
		s.logger.Errorw("servo session ended with error", "arm", s.armName, "error", err)
	}
	s.cancel() // release the run context
	close(s.done)
}

func (s *session) ended() bool {
	select {
	case <-s.done:
		return true
	default:
		return false
	}
}

// endedErr is the error for a command on an ended session.
func (s *session) endedErr() error {
	st := s.status().State
	s.mu.Lock()
	defer s.mu.Unlock()
	if stopFailed(s.runErr) {
		return fmt.Errorf("session ended: %s: %w", st, s.runErr)
	}
	return fmt.Errorf("session ended: %s", st)
}

// beginStop starts a ramp-stop ending in final, unless a stop is already under way (then a servo_stop
// still upgrades the final state: the loop handles that). RampStop runs under s.mu so a command either
// lands before the stop starts or is rejected. Lock order is s.mu then l.mu, never the reverse.
func (s *session) beginStop(final State) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.stopping == "" || final == StateStopped {
		s.stopping = final
	}
	s.loop.RampStop(s.ramp, final)
}

// onTick is the loop's OnTick: the dead-man check, once per tick.
func (s *session) onTick(st Status) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if st.CollisionLate != s.lastLate {
		s.lastLate = st.CollisionLate
		s.logger.Warnw("collision", "late", st.CollisionLate)
	}
	if s.stopping == "" && s.now().Sub(s.lastCmd) > s.timeout {
		s.stopping = StateStoppedTimeout
		s.loop.RampStop(s.ramp, StateStoppedTimeout)
	}
	if st.LiveStale && s.stopping == "" { // s.mu is held: not beginStop
		s.stopping = StateStoppedStale
		s.loop.RampStop(s.ramp, StateStoppedStale)
	}
}

// touch refreshes the dead-man without a command (teleop_status polling).
func (s *session) touch() {
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.stopping == "" {
		s.lastCmd = s.now()
	}
}

func (s *session) usable(m Mode) error {
	if s.ended() {
		return s.endedErr()
	}
	if s.mode != m {
		return fmt.Errorf("wrong mode: session is %s", s.mode)
	}
	return nil
}

// checkChain reports (cached per frame) whether frame can be transformed with every component but the
// session's arm at zero inputs: it must exist and hang off the arm or static frames only.
func (s *session) checkChain(frame string) error {
	s.cmu.Lock()
	defer s.cmu.Unlock()
	if err, ok := s.chains[frame]; ok {
		return err
	}
	err := func() error {
		f := s.fs.Frame(frame)
		if f == nil {
			return fmt.Errorf("unknown frame %q", frame)
		}
		chain, err := s.fs.TracebackFrame(f)
		if err != nil {
			return fmt.Errorf("frame %q: %w", frame, err)
		}
		for _, c := range chain {
			if c.Name() != s.armName && len(c.DoF()) > 0 {
				return fmt.Errorf("frame %q is attached to movable frame %q (only the session's arm is supported)", frame, c.Name())
			}
		}
		return nil
	}()
	s.chains[frame] = err
	return err
}

// target handles servo_target: p is in frame (the session frame when empty), in mm. With apply false it
// only transforms p and reports it.
func (s *session) target(p spatialmath.Pose, frame string, apply bool) (TargetResult, error) {
	if err := s.usable(ModePose); err != nil {
		return TargetResult{}, err
	}
	// 1. transform: local computation, no RPC, no s.mu
	if frame != "" && frame != s.frame {
		if s.fs == nil {
			return TargetResult{}, errors.New("servo_target frame requires servo_start frames: true")
		}
		if err := s.checkChain(frame); err != nil {
			return TargetResult{}, err
		}
		q := s.loop.Status().QMeas
		if len(q) == 0 {
			q = s.startQ
		}
		in := maps.Clone(s.zeroIn)
		in[s.armName] = q
		tf, err := s.fs.Transform(in.ToLinearInputs(), referenceframe.NewPoseInFrame(frame, p), s.frame)
		if err != nil {
			return TargetResult{}, fmt.Errorf("transforming target from %q: %w", frame, err)
		}
		p = tf.(*referenceframe.PoseInFrame).Pose()
	}
	raw := p
	if !apply {
		// Intentionally allowed while stopping: a preview moves nothing.
		return TargetResult{Status: SessionStatus{Status: s.loop.Status(), Config: s.loop.Config()}, Target: raw, Raw: raw}, nil
	}
	pb := spatialmath.Compose(spatialmath.PoseInverse(s.twb), p) // base frame: the floor is defined there
	s.mu.Lock()
	if s.stopping != "" {
		defer s.mu.Unlock()
		return TargetResult{}, fmt.Errorf("session stopping: %s", s.stopping)
	}
	now := s.now()
	fb, clamped := s.filt.apply(now, pb)
	v := s.est.update(now, fb)
	if !s.ff {
		v = [6]float64{}
	}
	s.lastCmd = now
	s.loop.SetMotion(fb, v) // under s.mu so concurrent commands apply in lastCmd order
	s.mu.Unlock()
	return TargetResult{Status: s.status(), Target: spatialmath.Compose(s.twb, fb), Raw: raw, Clamped: clamped}, nil
}

// trajectory handles servo_trajectory: poses (session frame, mm) dt apart, the first for the moment age before now.
// lead nil means the loop's measured lag. The dead-man counts from the chunk's last pose, so a client that stops
// sending gets the rest of the chunk plus timeout before the session stops.
func (s *session) trajectory(poses []spatialmath.Pose, dt, age time.Duration, lead *time.Duration, blend time.Duration) (SessionStatus, error) {
	if err := s.usable(ModePose); err != nil {
		return SessionStatus{}, err
	}
	inv := spatialmath.PoseInverse(s.twb)
	pb := make([]spatialmath.Pose, len(poses))
	for i, p := range poses {
		pb[i] = spatialmath.Compose(inv, p) // base frame
	}
	s.mu.Lock()
	if s.stopping != "" {
		defer s.mu.Unlock()
		return SessionStatus{}, fmt.Errorf("session stopping: %s", s.stopping)
	}
	now := s.now()
	tr := newTrajectory(now.Add(-age), dt, pb)
	tr.arrive, tr.blend = now, blend
	if lead == nil {
		tr.autoLead = true
	} else {
		tr.lead = *lead
	}
	s.lastCmd = now
	if e := tr.end(); e.After(now) {
		s.lastCmd = e
	}
	s.loop.SetTrajectory(tr)
	s.mu.Unlock()
	return s.status(), nil
}

// twist handles servo_twist: lin (m/s) and ang (rad/s) are in the session frame.
func (s *session) twist(lin, ang r3.Vector) (SessionStatus, error) {
	if err := s.usable(ModeTwist); err != nil {
		return SessionStatus{}, err
	}
	cfg := s.loop.Config()
	if lin.Norm() > cfg.MaxLinearMMPerS/mmPerM || ang.Norm() > deg2rad(cfg.MaxAngularDegPerS) {
		return SessionStatus{}, fmt.Errorf("twist exceeds limits (%.0f mm/s, %.0f deg/s)",
			cfg.MaxLinearMMPerS, cfg.MaxAngularDegPerS)
	}
	rinv := spatialmath.PoseInverse(spatialmath.NewPose(r3.Vector{}, s.twb.Orientation()))
	lb := spatialmath.Compose(rinv, spatialmath.NewPoseFromPoint(lin)).Point()
	ab := spatialmath.Compose(rinv, spatialmath.NewPoseFromPoint(ang)).Point()
	rollIgnored := false
	if cfg.Task == TaskPointing {
		if a := s.loop.Status().ToolAxis; a.Norm() > 0 && math.Abs(ab.Dot(a)) > 1e-6*ab.Norm() {
			rollIgnored = true
		}
	}
	s.mu.Lock()
	if s.stopping != "" {
		defer s.mu.Unlock()
		return SessionStatus{}, fmt.Errorf("session stopping: %s", s.stopping)
	}
	s.lastCmd = s.now()
	s.loop.SetVelocity([6]float64{lb.X, lb.Y, lb.Z, ab.X, ab.Y, ab.Z})
	s.mu.Unlock()
	st := s.status()
	st.RollIgnored = rollIgnored
	return st, nil
}

// stop handles servo_stop: ramp to zero, stop the arm, wait for the loop to end.
func (s *session) stop(ctx context.Context) (SessionStatus, error) {
	if s.ended() {
		return SessionStatus{}, s.endedErr()
	}
	s.beginStop(StateStopped)
	select {
	case <-s.done:
		s.mu.Lock()
		runErr := s.runErr
		s.mu.Unlock()
		if stopFailed(runErr) {
			return s.status(), s.endedErr() // "session ended: <state>: <cause>"
		}
		return s.status(), nil
	case <-ctx.Done():
		return SessionStatus{}, ctx.Err()
	}
}

func (s *session) status() SessionStatus {
	if s.ended() {
		s.mu.Lock()
		defer s.mu.Unlock()
		return s.final
	}
	s.mu.Lock()
	age, filt, clamps := max(0, s.now().Sub(s.lastCmd)), s.filt.cfg, s.filt.clamps // a trajectory's lastCmd can be ahead
	s.mu.Unlock()
	ss := SessionStatus{Status: s.loop.Status(), Config: s.loop.Config(), LastCmdAge: age, Filter: filt, Clamps: clamps}
	s.addLive(&ss)
	if s.guard != nil {
		gs := s.guard.stats()
		ss.Guard = &gs
	}
	return ss
}

// trip is the guard's callback: no command lands after it, and the loop ends at once in final.
func (s *session) trip(final State, reason string) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.stopping = final
	s.loop.Trip(final)
	s.logger.Infow("servo guard tripped", "arm", s.armName, "state", final, "reason", reason)
}

// addLive copies the poller's counters into ss.
func (s *session) addLive(ss *SessionStatus) {
	if p := s.poller; p != nil {
		st := p.stats()
		ss.LiveUpdates, ss.LiveErrors, ss.LiveSelfDropped = st.Updates, st.Errors, st.Self
		ss.LiveSkipped, ss.LiveOverflow, ss.LiveClipped = st.Skipped, st.Overflow, st.Clipped
		ss.LiveLastError, ss.LiveMethod = st.LastError, st.Method
	}
}

// servoManager owns the sessions, at most one live session per arm.
type servoManager struct {
	arms   map[string]arm.Arm
	fs     poseTransformer
	logger logging.Logger
	now    func() time.Time
	newIO  func(arm.Arm, float64) JointIO
	manual bool // tests: don't start Run, drive ticks by hand
	// visions are the vision services a live_obstacles session may poll, by name.
	visions map[string]LiveSource
	// sensors are the force-torque sensors a guard may poll, by name.
	sensors map[string]ForceSource

	mu     sync.Mutex
	byArm  map[string]*session
	closed bool
}

func newServoManager(arms map[string]arm.Arm, fs poseTransformer, logger logging.Logger) *servoManager {
	return &servoManager{arms: arms, fs: fs, logger: logger, now: time.Now, newIO: NewArmIO, byArm: map[string]*session{}}
}

// start opens a session on armName. It returns the session and the arm's current pose in cfg.Frame.
func (m *servoManager) start(ctx context.Context, armName string, cfg SessionConfig) (*session, spatialmath.Pose, error) {
	a, ok := m.arms[armName]
	if !ok {
		return nil, nil, fmt.Errorf("arm %q is not in this service's arms", armName)
	}
	if cfg.Mode != ModePose && cfg.Mode != ModeTwist {
		return nil, nil, fmt.Errorf("unsupported mode %q: use \"pose\" or \"twist\"", cfg.Mode)
	}
	// ponytail: one manager lock held across the start RPCs (frame system, Kinematics, Read), which blocks
	// other arms' commands during a start (a slow start can trip their dead-man, which fails safe); use
	// per-arm locks if multi-arm starts ever matter.
	m.mu.Lock()
	defer m.mu.Unlock()
	if m.closed {
		return nil, nil, errors.New("servo service is closing")
	}
	if s := m.byArm[armName]; s != nil && !s.ended() {
		return nil, nil, fmt.Errorf("arm %s busy", armName)
	}
	if cfg.Filter == (TargetFilter{}) {
		cfg.Filter = DefaultTargetFilter()
	}
	if err := cfg.Filter.validate(); err != nil {
		return nil, nil, err
	}
	var twb spatialmath.Pose
	switch cfg.Frame {
	case referenceframe.World:
		pif, err := m.fs.TransformPose(ctx,
			referenceframe.NewPoseInFrame(armName+"_origin", spatialmath.NewZeroPose()), referenceframe.World, nil)
		if err != nil {
			return nil, nil, fmt.Errorf("locating %s_origin in world: %w", armName, err)
		}
		twb = pif.Pose()
	case armName + "_origin":
		twb = spatialmath.NewZeroPose()
	default:
		return nil, nil, fmt.Errorf("unsupported frame %q: use \"world\" or %q", cfg.Frame, armName+"_origin")
	}
	model, err := a.Kinematics(ctx)
	if err != nil {
		return nil, nil, err
	}
	if js := cfg.Loop.MaxJointDegPerS; len(js) == 1 {
		cfg.Loop.MaxJointDegPerS = slices.Repeat(js, len(model.DoF()))
	}
	if as := cfg.Loop.MaxJointDegPerS2; len(as) == 1 {
		cfg.Loop.MaxJointDegPerS2 = slices.Repeat(as, len(model.DoF()))
	}
	io := m.newIO(a, cfg.Loop.RateHz)
	if aio, ok := io.(*armIO); ok { // newIO is per session; test fakes aren't armIOs
		aio.pointVel = !cfg.NoPointVelocities
	}
	l, err := NewLoop(model, io, cfg.Loop)
	if err != nil {
		return nil, nil, err
	}
	l.Now = m.now
	q, err := io.Read(ctx)
	if err != nil {
		return nil, nil, err
	}
	fk, err := model.Transform(q) // mm; FK() would give metres
	if err != nil {
		return nil, nil, err
	}
	l.SetTarget(fk)
	var fsys *referenceframe.FrameSystem
	var zeroIn referenceframe.FrameSystemInputs
	var cfgFS *framesystem.Config
	if cfg.Frames || cfg.Collision != nil || cfg.Live != nil {
		if cfgFS, err = m.fs.FrameSystemConfig(ctx); err != nil {
			return nil, nil, fmt.Errorf("reading the frame system: %w", err)
		}
	}
	creq := cfg.Collision
	if creq == nil && cfg.Live != nil {
		creq = &CollisionRequest{} // live mode needs a model even with no static obstacles
	}
	if creq != nil {
		cm, err := NewCollisionModel(cfgFS.Parts, model, armName, *creq, q)
		if err != nil {
			return nil, nil, err
		}
		if err := l.SetCollision(cm); err != nil {
			return nil, nil, err
		}
		for _, w := range cm.Warnings() {
			m.logger.Warnw("collision", "warning", w)
		}
		if len(cm.pairs) == 0 && cfg.Live == nil {
			m.logger.Warnw("collision requested but no robot-part/obstacle pairs; collision is off", "arm", armName)
		}
	}
	if cfg.Frames {
		if fsys, err = referenceframe.NewFrameSystem("servo", cfgFS.Parts, nil); err != nil {
			return nil, nil, fmt.Errorf("building the frame system: %w", err)
		}
		if fsys.Frame(armName) == nil {
			return nil, nil, fmt.Errorf("arm %s is not in the frame system", armName)
		}
		if n := len(fsys.Frame(armName).DoF()); n != len(q) {
			return nil, nil, fmt.Errorf("arm %s frame has %d DoF, arm reports %d joints", armName, n, len(q))
		}
		zeroIn = referenceframe.NewZeroInputs(fsys)
	}
	runCtx, cancel := context.WithCancel(context.Background())
	s := &session{
		id: rand.Text(), mode: cfg.Mode, loop: l, twb: twb, now: m.now,
		timeout: cfg.Timeout, ramp: cfg.Ramp, done: make(chan struct{}), cancel: cancel, logger: m.logger, armName: armName,
		est:     velEstimator{maxLin: l.cfg.MaxLinearMMPerS / mmPerM, maxAng: deg2rad(l.cfg.MaxAngularDegPerS)},
		lastCmd: m.now(),
		fs:      fsys, frame: cfg.Frame, startQ: q, zeroIn: zeroIn, chains: map[string]error{},
		filt: newTargetFilter(cfg.Filter), ff: cfg.Filter.Feedforward,
	}
	if cfg.Live != nil {
		jr := cfg.LiveJoints
		if jr == nil {
			jr = armJoints{a}
		}
		h, err := StartLive(ctx, l, cfg.LiveSource, jr, cfgFS.Parts, model, armName, q, *cfg.Live)
		if err != nil {
			cancel()
			return nil, nil, err
		}
		s.poller = h.p
		s.cancel = func() { cancel(); h.Stop() }
	}
	if cfg.Guard != nil {
		g, err := newGuard(ctx, *cfg.Guard, cfg.GuardSource, s.trip)
		if err != nil {
			s.cancel()
			return nil, nil, err
		}
		s.guard = g
		go g.run(runCtx) // finish cancels runCtx, so the guard ends with the loop
	}
	if cfg.Force != nil {
		if cfg.Mode != ModeTwist {
			s.cancel()
			return nil, nil, errors.New(`force needs "mode": "twist"`)
		}
		fc := cfg.Force.withDefaults()
		// a monitor-only guard (force_n 0): the force controller's poller and software tare
		mon, err := newGuard(ctx, GuardConfig{Sensor: fc.Sensor, RateHz: fc.RateHz}, cfg.ForceSource, s.trip)
		if err == nil {
			err = l.SetForce(fc, mon.force)
		}
		if err != nil {
			s.cancel()
			return nil, nil, err
		}
		go mon.run(runCtx)
	}
	l.OnTick = s.onTick
	m.byArm[armName] = s
	if !m.manual {
		go s.run(runCtx)
	}
	return s, spatialmath.Compose(twb, fk), nil
}

func (m *servoManager) get(id string) (*session, error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	for _, s := range m.byArm {
		if s.id == id {
			return s, nil
		}
	}
	return nil, errUnknownSession
}

// current is armName's session, live or ended, or nil.
func (m *servoManager) current(armName string) *session {
	m.mu.Lock()
	defer m.mu.Unlock()
	return m.byArm[armName]
}

func (m *servoManager) busy(armName string) bool {
	m.mu.Lock()
	defer m.mu.Unlock()
	s := m.byArm[armName]
	return s != nil && !s.ended()
}

// stopArm ramp-stops armName's live session, if any (StopPlan).
func (m *servoManager) stopArm(ctx context.Context, armName string) error {
	m.mu.Lock()
	s := m.byArm[armName]
	m.mu.Unlock()
	if s == nil || s.ended() {
		return nil
	}
	if _, err := s.stop(ctx); err != nil {
		// A session that ended cleanly just before stop ran is not an error; a failed stop is.
		s.mu.Lock()
		failed := stopFailed(s.runErr)
		s.mu.Unlock()
		if !s.ended() || failed {
			return err
		}
	}
	return nil
}

// close ramp-stops every live session concurrently. A session that doesn't finish within its
// ramp + 2 s is cancelled, which stops the arm immediately.
func (m *servoManager) close(ctx context.Context) error {
	m.mu.Lock()
	m.closed = true
	ss := slices.Collect(maps.Values(m.byArm))
	m.mu.Unlock()
	var wg sync.WaitGroup
	for _, s := range ss {
		if s.ended() {
			continue
		}
		wg.Add(1)
		go func() {
			defer wg.Done()
			s.beginStop(StateStopped)
			select {
			case <-s.done:
			case <-time.After(s.ramp + 2*time.Second):
				s.cancel()
				<-s.done
			case <-ctx.Done():
				s.cancel()
				<-s.done
			}
		}()
	}
	wg.Wait()
	var errs []error
	for _, s := range ss {
		s.mu.Lock()
		if stopFailed(s.runErr) {
			errs = append(errs, fmt.Errorf("arm %s: %w", s.armName, s.runErr))
		}
		s.mu.Unlock()
	}
	return errors.Join(errs...)
}

// stopFailed reports whether a loop's end error should reach the caller. A bare context.Canceled is
// a normal cancel (Close's fallback); anything else, including a cancel joined with an arm.Stop
// failure, is reported.
func stopFailed(err error) bool { return err != nil && err != context.Canceled } //nolint:errorlint // exact match intended
