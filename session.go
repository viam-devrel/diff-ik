package diffik

import (
	"context"
	"crypto/rand"
	"errors"
	"fmt"
	"maps"
	"slices"
	"sync"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
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
	TransformPose(ctx context.Context, pose *referenceframe.PoseInFrame, dst string,
		supplementalTransforms []*referenceframe.LinkInFrame) (*referenceframe.PoseInFrame, error)
}

// SessionConfig is a parsed servo_start request. Loop.RateHz must be set. A one-element
// Loop.MaxJointDegPerS applies to every joint.
type SessionConfig struct {
	Frame   string // "world" or "<arm>_origin"
	Mode    Mode
	Loop    LoopConfig
	Timeout time.Duration // dead-man
	Ramp    time.Duration
}

// SessionStatus is a snapshot of a session, live or final.
type SessionStatus struct {
	Status
	Config     LoopConfig
	LastCmdAge time.Duration
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

	mu       sync.Mutex
	est      velEstimator
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
	case StateStopped, StateStoppedTimeout, StateFaulted:
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
func (s *session) onTick(Status) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.stopping == "" && s.now().Sub(s.lastCmd) > s.timeout {
		s.stopping = StateStoppedTimeout
		s.loop.RampStop(s.ramp, StateStoppedTimeout)
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

// target handles servo_target: p is in the session frame, in mm.
func (s *session) target(p spatialmath.Pose) (SessionStatus, error) {
	if err := s.usable(ModePose); err != nil {
		return SessionStatus{}, err
	}
	pb := spatialmath.Compose(spatialmath.PoseInverse(s.twb), p)
	s.mu.Lock()
	if s.stopping != "" {
		defer s.mu.Unlock()
		return SessionStatus{}, fmt.Errorf("session stopping: %s", s.stopping)
	}
	now := s.now()
	v := s.est.update(now, pb)
	s.lastCmd = now
	s.loop.SetMotion(pb, v) // under s.mu so concurrent commands apply in lastCmd order
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
	s.mu.Lock()
	if s.stopping != "" {
		defer s.mu.Unlock()
		return SessionStatus{}, fmt.Errorf("session stopping: %s", s.stopping)
	}
	s.lastCmd = s.now()
	s.loop.SetVelocity([6]float64{lb.X, lb.Y, lb.Z, ab.X, ab.Y, ab.Z})
	s.mu.Unlock()
	return s.status(), nil
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
	age := s.now().Sub(s.lastCmd)
	s.mu.Unlock()
	return SessionStatus{Status: s.loop.Status(), Config: s.loop.Config(), LastCmdAge: age}
}

// servoManager owns the sessions, at most one live session per arm.
type servoManager struct {
	arms   map[string]arm.Arm
	fs     poseTransformer
	logger logging.Logger
	now    func() time.Time
	newIO  func(arm.Arm, float64) JointIO
	manual bool // tests: don't start Run, drive ticks by hand

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
	io := m.newIO(a, cfg.Loop.RateHz)
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
	runCtx, cancel := context.WithCancel(context.Background())
	s := &session{
		id: rand.Text(), mode: cfg.Mode, loop: l, twb: twb, now: m.now,
		timeout: cfg.Timeout, ramp: cfg.Ramp, done: make(chan struct{}), cancel: cancel, logger: m.logger, armName: armName,
		est:     velEstimator{maxLin: l.cfg.MaxLinearMMPerS / mmPerM, maxAng: deg2rad(l.cfg.MaxAngularDegPerS)},
		lastCmd: m.now(),
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
