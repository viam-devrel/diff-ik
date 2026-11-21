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
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"gonum.org/v1/gonum/mat"
	"gonum.org/v1/gonum/num/quat"
)

// JointIO is how the loop talks to an arm: read measured joints, send a joint target, stop.
type JointIO interface {
	Read(ctx context.Context) ([]referenceframe.Input, error)
	Send(ctx context.Context, q []referenceframe.Input) error
	Stop(ctx context.Context) error
}

// LoopConfig holds the loop's calibration knobs. Zero values take the defaults in withDefaults.
type LoopConfig struct {
	RateHz            float64
	TaskWeights       [6]float64 // x, y, z, rx, ry, rz over m / rad; all-zero means all 1
	Gain              float64    // 1/s
	MaxLinearMMPerS   float64
	MaxAngularDegPerS float64
	MaxJointDegPerS   []float64 // one per joint
	MaxJointDegPerS2  []float64 // acceleration, one per joint; nil: the model's, else defaultJointDegPerS2
	SingularEps       float64
	MaxDamping        float64
	ResyncDeg         float64
	MaxLagMS          float64 // longest arm lag treated as lag; beyond it a gap is divergence and resyncs. Default 300
	TolMM             float64
	TolDeg            float64
	Task              string   // "pose" | "pointing"; empty = pointing if DOF < 6 else pose
	Solver            string   // "dls" (default) | "qp" (spec "QP solver and posture task")
	Posture           *Posture // QP only: pull toward a preferred posture
}

// Solver names.
const (
	SolverDLS = "dls"
	SolverQP  = "qp"
)

// Posture is the QP's posture task: w_p‖Δq − scale·k·dt·(q₀ − q)‖² (spec "Objective").
type Posture struct {
	JointsDeg []float64 // q₀ in degrees; nil means the joints at the first tick
	Weight    float64   // w_p, default 1e-3
	Gain      float64   // k in 1/s, default 1
}

func (c LoopConfig) withDefaults(dof int) LoopConfig {
	def := func(v *float64, d float64) {
		if *v == 0 {
			*v = d
		}
	}
	def(&c.RateHz, 50)
	if c.TaskWeights == ([6]float64{}) {
		c.TaskWeights = [6]float64{1, 1, 1, 1, 1, 1}
	}
	def(&c.Gain, 2)
	def(&c.MaxLinearMMPerS, 100)
	def(&c.MaxAngularDegPerS, 45)
	if c.MaxJointDegPerS == nil {
		c.MaxJointDegPerS = make([]float64, dof)
		for i := range c.MaxJointDegPerS {
			c.MaxJointDegPerS[i] = 90
		}
	}
	def(&c.SingularEps, 0.04)
	def(&c.MaxDamping, 0.1)
	if c.ResyncDeg == 0 && c.RateHz > 0 && len(c.MaxJointDegPerS) > 0 {
		c.ResyncDeg = 3 * slices.Max(c.MaxJointDegPerS) / c.RateHz
	}
	def(&c.MaxLagMS, 300)
	def(&c.TolMM, 2)
	def(&c.TolDeg, 2)
	if c.Task == "" {
		c.Task = TaskPose
		if dof < 6 {
			c.Task = TaskPointing
		}
	}
	if c.Solver == "" {
		c.Solver = SolverDLS
	}
	if c.Posture != nil {
		p := *c.Posture
		def(&p.Weight, 1e-3)
		def(&p.Gain, 1)
		c.Posture = &p
	}
	return c
}

func (c LoopConfig) validate(dof int) error {
	if c.RateHz < 1 {
		return fmt.Errorf("rate_hz must be >= 1, got %v", c.RateHz)
	}
	if c.Task != TaskPose && c.Task != TaskPointing {
		return fmt.Errorf("unsupported task %q: use %q or %q", c.Task, TaskPose, TaskPointing)
	}
	for i, w := range c.TaskWeights {
		if w < 0 {
			return fmt.Errorf("task weight %d is negative", i)
		}
	}
	if len(c.MaxJointDegPerS) != dof {
		return fmt.Errorf("need %d joint velocity limits, got %d", dof, len(c.MaxJointDegPerS))
	}
	for i, v := range c.MaxJointDegPerS {
		if !(v > 0) {
			return fmt.Errorf("joint %d velocity limit must be > 0, got %v", i, v)
		}
	}
	if len(c.MaxJointDegPerS2) != dof {
		return fmt.Errorf("need %d joint acceleration limits, got %d", dof, len(c.MaxJointDegPerS2))
	}
	for i, v := range c.MaxJointDegPerS2 {
		if !(v > 0) {
			return fmt.Errorf("joint %d acceleration limit must be > 0, got %v", i, v)
		}
	}
	for _, p := range []struct {
		name string
		v    float64
	}{
		{"gain", c.Gain}, {"max_linear_mm_per_s", c.MaxLinearMMPerS}, {"max_angular_deg_per_s", c.MaxAngularDegPerS},
		{"singular_eps", c.SingularEps}, {"max_damping", c.MaxDamping}, {"resync_deg", c.ResyncDeg}, {"max_lag_ms", c.MaxLagMS},
		{"tol_mm", c.TolMM}, {"tol_deg", c.TolDeg},
	} {
		if !(p.v > 0) {
			return fmt.Errorf("%s must be > 0, got %v", p.name, p.v)
		}
	}
	if c.MaxLagMS > 1000 {
		return fmt.Errorf("max_lag_ms must be <= 1000, got %v", c.MaxLagMS)
	}
	if c.Solver != SolverDLS && c.Solver != SolverQP {
		return fmt.Errorf("unsupported solver %q: use %q or %q", c.Solver, SolverDLS, SolverQP)
	}
	if p := c.Posture; p != nil {
		if c.Solver != SolverQP {
			return fmt.Errorf("posture needs solver %q", SolverQP)
		}
		if p.JointsDeg != nil && len(p.JointsDeg) != dof {
			return fmt.Errorf("posture.joints_deg needs %d values, got %d", dof, len(p.JointsDeg))
		}
		if !(p.Weight > 0 && p.Weight <= 1e-2) {
			return fmt.Errorf("posture weight must be in (0, 0.01], got %v", p.Weight)
		}
		if !(p.Gain > 0) {
			return fmt.Errorf("posture gain must be > 0, got %v", p.Gain)
		}
	}
	return nil
}

// State is the loop's externally visible state.
type State string

// Loop states.
const (
	StateIdle           State = "idle"
	StateRunning        State = "running"
	StateConverged      State = "converged"
	StateStalled        State = "stalled"
	StateFaulted        State = "faulted"
	StateStopped        State = "stopped"
	StateStoppedTimeout State = "stopped_timeout"
	StateStoppedStale   State = "stopped_stale"   // live obstacles went stale
	StateStoppedContact State = "stopped_contact" // the force guard tripped
)

const (
	defaultJointDegPerS2 = 360 // reaches the default 90°/s joint cap in 0.25 s

	minHold = 100 * time.Millisecond // floor on the SetMotion extrapolation hold
	leashS  = 0.1                    // the target may lead FK(q_cmd) by this many seconds at the speed limits
)

// errRampDone is returned by tick once a RampStop has completed and the arm has been stopped.
var errRampDone = errors.New("ramp stop complete")

// Status is a snapshot of the loop. Errors are measured (from read-back joints) unless prefixed Cmd.
type Status struct {
	State               State
	TargetMM            r3.Vector
	ErrMM, ErrDeg       float64
	CmdErrMM, CmdErrDeg float64
	RateHz              float64
	TickP50, TickP99    time.Duration
	LastTick            time.Duration
	Overruns, Resyncs   int
	LagMS               float64 // estimated arm lag behind the commands, ms
	Damping             float64
	Limited             bool
	QCmd, QMeas         []float64
	Task                string    // active task mode
	PointingResidualDeg float64   // pointing mode: the unreachable part of the pointing error at q_m
	ToolAxis            r3.Vector // tool axis of FK(q_cmd), base frame (read by servo_twist)
	ReachDeg            float64   // the convergence angle: reachable pointing error (pointing) or active orientation error (pose)
	QPActive, QPIters   int       // solver qp: active constraints and iterations this tick
	QPFallbacks         int       // solver qp: ticks that fell back to the DLS path, session total
	PeakAccelRatio      float64   // max over ticks and joints of |step − lastStep| / (a·dt²); ≤ 1 means the limit held

	Collision                          string  // "", "on" or "off" (requested, but no pairs)
	CollisionMinMM                     float64 // closest unsuspended pair this tick; +Inf if none
	CollisionPair                      string
	CollisionRows, CollisionRelaxed    int
	CollisionBrakes                    int
	LivePinched                        int     // live obstacles: ticks solved by the second relax tier (no further approach plus a soft retreat), session total
	LiveCount                          int     // live obstacles in use this tick
	LiveAgeMS                          float64 // age of the newest set (or of the wait for the first)
	LiveWaiting, LiveStale             bool    // no set yet / the newest set is older than stale_ms
	LiveBackingAway                    bool    // any live pair has g < 0 this tick
	CollisionIgnored, CollisionSkipped []string
	CollisionLate                      string // last "approach faster than v_max" warning

}

const tickWindow = 500 // ticks kept for p50/p99

const (
	lagAlpha = 0.1 // EMA weight of a new lag sample when the lag estimate falls
	// moveFrac: qm counts as moving when a joint changed by more than this fraction of one tick of
	// travel at the top joint speed. Below it, a still arm's sensor noise cannot raise the lag estimate.
	moveFrac = 0.1
)

const lateTolMM = 0.01 // mm per tick; smaller overshoots are second-order sliding drift, not a d_act too small

// Loop is a differential-IK servo loop for one arm.
type Loop struct {
	model  referenceframe.Model
	io     JointIO
	cfg    LoopConfig
	lo, hi []float64

	// OnTick, if set before Run, is called after every tick with a status snapshot.
	// The tick that completes a ramp-stop is not reported; read Status after Run returns.
	OnTick func(Status)

	// Now is the loop's clock for the extrapolation hold and the ramp. Nil means time.Now. Set before Run.
	Now func() time.Time

	stopErr error // io.Stop result of a completed ramp; written and read only on the loop goroutine

	qCmd     []float64
	lastStep []float64 // q_cmd change on the last tick, rad; loop goroutine only

	q0        []float64       // posture target, rad; set at NewLoop or the first tick
	qpFails   int             // consecutive QP fallbacks; loop goroutine only
	live      *liveState      // live-obstacle mode; nil: off
	coll      *collisionModel // nil: collision avoidance off; rows() is called from the loop goroutine only
	peakAccel float64         // see Status.PeakAccelRatio; loop goroutine only

	sent     [][]float64 // last sent commands, oldest first, at most MaxLagMS of ticks; loop goroutine only
	lagTicks float64     // estimated arm lag in ticks; loop goroutine only
	prevQm   []float64   // measured joints on the previous tick; loop goroutine only
	moved    bool        // the arm has been seen moving at least once; loop goroutine only

	mu        sync.Mutex
	errRing   []float64 // weighted measured error norm, one per tick, for stall detection
	target    spatialmath.Pose
	vel       [6]float64    // feedforward twist, base frame, m/s and rad/s
	extrap    bool          // vel came from SetMotion, so the hold applies
	lastSet   time.Time     // time of the last SetMotion
	holdAfter time.Duration // stop extrapolating this long after lastSet
	rampStart time.Time     // zero: no ramp
	rampDur   time.Duration
	rampFinal State
	tripFinal State         // non-empty: a guard tripped; the next tick holds the measured joints and ends
	wake      chan struct{} // Trip wakes Run for an immediate tick
	status    Status
	lastReach float64 // pointing mode: |e2_reach| (rad) from the previous tick, for the leash. Not reset on
	// SetMotion: a streamed servo_target calls SetMotion every tick, and resetting would disable the leash.
	tickDurs []time.Duration
	ticks    int
	first    time.Time // start of the first tick
	last     time.Time // start of the latest tick
}

// liveState is the live-obstacle mode's state. Only pend is shared with the poller, under its mutex.
type liveState struct {
	staleMS float64
	start   time.Time
	pend    struct {
		mu   sync.Mutex
		set  []liveObstacle
		at   time.Time // newest publish, on the loop's clock
		have bool      // a set is waiting to be taken
		ever bool      // at least one set arrived
	}
}

// SetLive enables live-obstacle mode (call after SetCollision, before Run) and returns the function the
// poller publishes whole sets with. The slot is stamped with the loop's clock, so fake-clock tests go stale.
func (l *Loop) SetLive(staleMS float64) (publish func([]liveObstacle)) {
	ls := &liveState{staleMS: staleMS, start: l.now()}
	l.live = ls
	l.mu.Lock()
	l.status.Collision, l.status.LiveWaiting = "on", true
	l.mu.Unlock()
	return func(set []liveObstacle) {
		p := &ls.pend
		p.mu.Lock()
		p.set, p.at, p.have, p.ever = set, l.now(), true, true
		p.mu.Unlock()
	}
}

// liveTick runs at tick start on the loop goroutine: it takes a pending set into the model and reports
// the age of the newest set and whether none has ever arrived.
func (l *Loop) liveTick() (age time.Duration, waiting bool) {
	p := &l.live.pend
	p.mu.Lock()
	set, have, ever, at := p.set, p.have, p.ever, p.at
	p.set, p.have = nil, false
	p.mu.Unlock()
	if have { // the pair rebuild runs outside the slot lock
		l.coll.setLive(set)
	}
	if !ever {
		return l.now().Sub(l.live.start), true
	}
	return l.now().Sub(at), false
}

// SetCollision turns on collision avoidance (solver qp only). Call it before Run.
func (l *Loop) SetCollision(c *collisionModel) error {
	if l.cfg.Solver != SolverQP {
		return errors.New(`collision avoidance needs solver "qp"`)
	}
	l.coll = c
	l.mu.Lock()
	defer l.mu.Unlock()
	l.status.Collision = "on"
	l.status.CollisionMinMM = math.Inf(1)
	if !c.hasPairs() && l.live == nil {
		l.status.Collision = "off"
	}
	l.status.CollisionIgnored, l.status.CollisionSkipped = c.ignored(), slices.Clone(c.skipped)
	return nil
}

// NewLoop builds a loop for model, driven through io.
func NewLoop(model referenceframe.Model, io JointIO, cfg LoopConfig) (*Loop, error) {
	limits := model.DoF()
	dof := len(limits)
	cfg = cfg.withDefaults(dof)
	if cfg.MaxJointDegPerS2 == nil {
		cfg.MaxJointDegPerS2 = make([]float64, dof)
		for i := range cfg.MaxJointDegPerS2 {
			cfg.MaxJointDegPerS2[i] = defaultJointDegPerS2
		}
	}
	if err := cfg.validate(dof); err != nil {
		return nil, err
	}
	// the model's acceleration limits (rad/s²) win where they are tighter: the session may only tighten
	cfg.MaxJointDegPerS2 = slices.Clone(cfg.MaxJointDegPerS2)
	for i, lim := range limits {
		if a := lim.MaxAcceleration; a != nil && *a > 0 {
			cfg.MaxJointDegPerS2[i] = math.Min(cfg.MaxJointDegPerS2[i], rad2deg(*a))
		}
	}
	l := &Loop{model: model, io: io, cfg: cfg, lo: make([]float64, dof), hi: make([]float64, dof), lastStep: make([]float64, dof),
		wake: make(chan struct{}, 1)}
	for i, lim := range limits {
		// stay fdStep inside the limits so Jacobian perturbations remain valid inputs
		l.lo[i], l.hi[i] = lim.Min+fdStep, lim.Max-fdStep
	}
	if p := cfg.Posture; p != nil && p.JointsDeg != nil {
		l.q0 = make([]float64, dof)
		for i, d := range p.JointsDeg {
			l.q0[i] = deg2rad(d)
		}
	}
	l.status.State = StateIdle
	l.status.Task = cfg.Task
	return l, nil
}

// SetTarget sets a stationary target pose (mm, base frame). Latest wins.
func (l *Loop) SetTarget(p spatialmath.Pose) { l.SetMotion(p, [6]float64{}) }

// SetMotion sets the target pose (mm, base frame) and a feedforward twist vel (m/s, rad/s, base frame).
// The loop advances the target at vel until max(2 × the interval since the previous SetMotion, 100 ms)
// has passed, then holds it. Latest wins. Ignored once a RampStop has begun.
func (l *Loop) SetMotion(p spatialmath.Pose, vel [6]float64) {
	l.mu.Lock()
	defer l.mu.Unlock()
	if !l.rampStart.IsZero() {
		return
	}
	now := l.now()
	l.holdAfter = minHold
	if !l.lastSet.IsZero() {
		l.holdAfter = max(minHold, 2*now.Sub(l.lastSet))
	}
	l.lastSet = now
	l.target, l.vel, l.extrap = p, vel, true
	l.errRing = l.errRing[:0]
}

// SetVelocity sets the feedforward twist (m/s, rad/s, base frame) and keeps advancing the current
// target at it, with no hold. Ignored before a target exists or once a RampStop has begun.
func (l *Loop) SetVelocity(vel [6]float64) {
	l.mu.Lock()
	defer l.mu.Unlock()
	if !l.rampStart.IsZero() || l.target == nil {
		return
	}
	l.vel, l.extrap = vel, false
}

// RampStop scales the commanded velocity linearly to zero over ramp, then stops the arm and ends
// Run (returning nil) with state final. The target stops advancing and SetMotion/SetVelocity are
// ignored from the call on. A second call keeps the first deadline and only upgrades final to StateStopped.
func (l *Loop) RampStop(ramp time.Duration, final State) {
	l.mu.Lock()
	defer l.mu.Unlock()
	if l.rampStart.IsZero() {
		l.rampStart, l.rampDur, l.rampFinal = l.now(), ramp, final
		return
	}
	if final == StateStopped {
		l.rampFinal = final
	}
}

// Trip ends the session at once, in final: the next tick (run immediately) commands the measured joints,
// which releases what the arm was pressing with, then stops the arm. It skips the ramp and the acceleration
// limit on purpose: in contact the arm is already stopped, and the command is only running ahead of it.
// The first call wins.
func (l *Loop) Trip(final State) {
	l.mu.Lock()
	if l.tripFinal == "" {
		l.tripFinal = final
	}
	l.mu.Unlock()
	select {
	case l.wake <- struct{}{}:
	default:
	}
}

// holder is a JointIO that can send a point the arm should come to rest at (zero point velocity).
type holder interface {
	Hold(ctx context.Context, q []referenceframe.Input) error
}

// aborter is a JointIO that can stop at once and hold given joints (armIO's Abort).
type aborter interface {
	Abort(ctx context.Context, q []referenceframe.Input) error
}

// tripStop is tick's trip path: command the measured joints, then stop the arm.
func (l *Loop) tripStop(ctx context.Context, final State) error {
	qm, err := l.io.Read(ctx)
	if ab, ok := l.io.(aborter); ok && err == nil {
		sctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
		defer cancel()
		err = ab.Abort(sctx, qm)
		l.mu.Lock()
		l.status.State = final
		l.qCmd = slices.Clone(qm)
		l.status.QCmd, l.status.QMeas = slices.Clone(qm), slices.Clone(qm)
		l.mu.Unlock()
		if err != nil {
			err = fmt.Errorf("stopping arm: %w", err)
		}
		l.stopErr = err
		return errRampDone
	}
	if err == nil {
		if h, ok := l.io.(holder); ok {
			err = h.Hold(ctx, qm)
		} else {
			err = l.io.Send(ctx, qm)
		}
	}
	l.mu.Lock()
	l.status.State = final
	if err == nil {
		l.qCmd = slices.Clone(qm)
		l.status.QCmd, l.status.QMeas = slices.Clone(qm), slices.Clone(qm)
	}
	l.mu.Unlock()
	l.stopErr = l.stop(err)
	return errRampDone
}

func (l *Loop) now() time.Time {
	if l.Now != nil {
		return l.Now()
	}
	return time.Now()
}

// rampScale reports whether a ramp-stop is active and the velocity scale in [0, 1]. Caller holds l.mu.
func (l *Loop) rampScale() (bool, float64) {
	if l.rampStart.IsZero() {
		return false, 1
	}
	if l.rampDur <= 0 {
		return true, 0
	}
	return true, math.Max(0, 1-float64(l.now().Sub(l.rampStart))/float64(l.rampDur))
}

// advance moves the stored target one tick along vel and returns the target and the feedforward
// twist to use. cp and cq are FK(q_cmd). Caller holds l.mu.
//   - Hold (SetMotion only): after holdAfter the target stays put and no feedforward is added.
//   - Ramp: the target stays put but the feedforward still applies; tick scales it toward zero.
//   - hold (live obstacles still waiting for a first set): the same, with scale 0 from tick.
//   - Leash, per axis: an advance is skipped only when the commanded-side error already exceeds
//     leashS × the speed limit and the advance would grow it (vel · e > 0).
func (l *Loop) advance(dt float64, cp r3.Vector, cq quat.Number, ramping, hold bool) (spatialmath.Pose, [6]float64) {
	var zero [6]float64
	if l.extrap && l.now().Sub(l.lastSet) > l.holdAfter {
		return l.target, zero
	}
	vel := l.vel
	if ramping || hold || vel == zero {
		return l.target, vel
	}
	tp, tq := l.target.Point().Mul(1/mmPerM), l.target.Orientation().Quaternion()
	e := PoseError(tp, tq, cp, cq)
	if l.cfg.Task != TaskPointing { // pose: a dropped row's error never leashes (spec "Leash": active norm)
		for i, w := range l.cfg.TaskWeights {
			if w == 0 {
				e[i] = 0
			}
		}
	}
	vl, el := r3.Vector{X: vel[0], Y: vel[1], Z: vel[2]}, r3.Vector{X: e[0], Y: e[1], Z: e[2]}
	if !(el.Norm() > leashS*l.cfg.MaxLinearMMPerS/mmPerM && vl.Dot(el) > 0) {
		tp = tp.Add(vl.Mul(dt))
	}
	va := r3.Vector{X: vel[3], Y: vel[4], Z: vel[5]}
	ea := r3.Vector{X: e[3], Y: e[4], Z: e[5]}
	angErr := ea.Norm()
	if l.cfg.Task == TaskPointing {
		// pointing: leash on the reachable part only, and along the pointing rotation (spec "Leash")
		ea, _, _ = pointingError(toolAxis(cq), toolAxis(tq))
		angErr = l.lastReach
		at := toolAxis(tq)
		va = va.Sub(at.Mul(va.Dot(at))) // roll is discarded: never move the target's axis about itself
	}
	if !(angErr > leashS*deg2rad(l.cfg.MaxAngularDegPerS) && va.Dot(ea) > 0) {
		tq = quat.Mul(ExpRot(va.Mul(dt)), tq)
	}
	q := spatialmath.Quaternion(tq)
	l.target = spatialmath.NewPose(tp.Mul(mmPerM), &q)
	return l.target, vel
}

// Config returns the loop's configuration with defaults filled in.
func (l *Loop) Config() LoopConfig { return l.cfg }

// Status returns a snapshot of the loop.
func (l *Loop) Status() Status {
	l.mu.Lock()
	defer l.mu.Unlock()
	s := l.status
	s.QCmd = slices.Clone(s.QCmd)
	s.QMeas = slices.Clone(s.QMeas)
	s.CollisionIgnored, s.CollisionSkipped = slices.Clone(s.CollisionIgnored), slices.Clone(s.CollisionSkipped)
	if n := len(l.tickDurs); n > 0 {
		d := slices.Clone(l.tickDurs)
		slices.Sort(d)
		s.TickP50, s.TickP99 = d[n/2], d[n*99/100]
	}
	// Rate over first→last tick: ticks/elapsed-since-Run would always read just under nominal.
	if span := l.last.Sub(l.first).Seconds(); l.ticks > 1 && span > 0 {
		s.RateHz = float64(l.ticks-1) / span
	}
	return s
}

// Run ticks at cfg.RateHz until ctx is done or the loop faults. It always stops the arm before returning.
func (l *Loop) Run(ctx context.Context) error {
	dt := 1 / l.cfg.RateHz
	period := time.Duration(dt * float64(time.Second))
	t := time.NewTicker(period) // drops ticks we miss; never bursts to catch up
	defer t.Stop()
	for {
		select {
		case <-ctx.Done():
			return l.stop(ctx.Err())
		case <-t.C:
		case <-l.wake:
		}
		start := time.Now()
		// ponytail: one fixed budget; the first tick also opens the stream, so the floor is generous.
		// A hung Read/Send faults (and stops the arm) after this instead of blocking forever.
		tctx, cancel := context.WithTimeout(ctx, max(5*period, 500*time.Millisecond))
		err := l.tick(tctx, dt)
		cancel()
		if err != nil {
			if errors.Is(err, errRampDone) {
				return l.stopErr // tick already stopped the arm
			}
			if ctx.Err() != nil {
				return l.stop(ctx.Err())
			}
			l.mu.Lock()
			l.status.State = StateFaulted
			l.mu.Unlock()
			return l.stop(err)
		}
		d := time.Since(start)
		l.mu.Lock()
		if l.ticks == 0 {
			l.first = start
		}
		l.last = start
		l.ticks++
		if len(l.tickDurs) == tickWindow {
			l.tickDurs = l.tickDurs[1:]
		}
		l.tickDurs = append(l.tickDurs, d)
		l.status.LastTick = d
		if d > period {
			l.status.Overruns++
		}
		l.mu.Unlock()
		if l.OnTick != nil {
			l.OnTick(l.Status())
		}
	}
}

func (l *Loop) stop(cause error) error {
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	if err := l.io.Stop(ctx); err != nil {
		return errors.Join(cause, fmt.Errorf("stopping arm: %w", err))
	}
	return cause
}

// tick runs one control step. See the spec's per-tick data flow; step numbers match.
func (l *Loop) tick(ctx context.Context, dt float64) error {
	// 0. ramp-stop complete: stop the arm and end. This runs before the idle return so an idle ramp ends too.
	// The ramp lasts until the acceleration limit allows stopping in one tick, which may be after ramp.
	l.mu.Lock()
	if final := l.tripFinal; final != "" {
		l.mu.Unlock()
		return l.tripStop(ctx, final)
	}
	ramping, scale := l.rampScale()
	if ramping && scale == 0 && l.canStop(dt) {
		l.status.State = l.rampFinal
		l.mu.Unlock()
		// final state is set even if io.Stop fails; Run returns that error (spec: 'or the stop error')
		l.stopErr = l.stop(nil)
		return errRampDone
	}
	hasTarget := l.target != nil
	l.mu.Unlock()

	// 1. measured joints
	qm, err := l.io.Read(ctx)
	if err != nil {
		return fmt.Errorf("reading joints: %w", err)
	}
	if len(qm) != len(l.lo) {
		return fmt.Errorf("arm reported %d joints, model has %d", len(qm), len(l.lo))
	}
	if l.qCmd == nil {
		l.qCmd = slices.Clone(qm)
		if l.cfg.Posture != nil && l.q0 == nil {
			l.q0 = slices.Clone(qm)
		}
	}
	if !hasTarget {
		l.mu.Lock()
		l.status.State = StateIdle
		l.status.QMeas = slices.Clone(qm)
		l.mu.Unlock()
		return nil
	}

	var liveAge time.Duration
	var liveWaiting bool
	if l.live != nil { // the pending set is taken only here, at tick start
		liveAge, liveWaiting = l.liveTick()
	}

	// 2. hybrid resync (outside mu)
	resynced := false
	l.updateLag(qm)
	if maxAbsDiff(l.expected(), qm) > deg2rad(l.cfg.ResyncDeg) {
		copy(l.qCmd, qm)
		// The ring is kept: a real arm still executes its in-flight commands, so they remain the best model
		// of where it will be. Clearing it makes the loop chase its own pipeline (endless resyncs).
		if !l.moved { // an arm that never follows gets one run-up, not one per resync
			l.lagTicks, l.moved = 0, true
		}
		resynced = true
	}

	// 3. FK(q_cmd), then advance the target under mu, then the error from the copy (spec "Tick order")
	cp, cq, err := FK(l.model, l.qCmd)
	if err != nil {
		return err
	}
	l.mu.Lock()
	ramping, scale = l.rampScale() // a RampStop may have landed since step 0
	// not covered by a test: a missed mid-tick ramp start only costs one full-scale tick
	if liveWaiting { // a separate factor: ramping stays false, so the ramp-complete check can't end the session
		scale = 0
	}
	target, vel := l.advance(dt, cp, cq, ramping, liveWaiting)
	l.mu.Unlock()
	tp, tq := target.Point().Mul(1/mmPerM), target.Orientation().Quaternion()

	// 4–6. task-space step: pose → weighted DLS (unchanged when m ≥ n) or dpinv; pointing → priority
	J, err := Jacobian(l.model, l.qCmd)
	if err != nil {
		return err
	}
	dq, lambda, limitedTask, cmdPos, cmdAng, rows, err := l.solve(J, tp, tq, cp, cq, vel, scale, dt)
	if err != nil {
		return err
	}

	var next []float64
	var limStep bool
	var qpActive, qpIters int
	fellBack, relaxed, braked, pinched := false, false, false, false
	var crows []collRow
	var cmin float64
	var cpair int
	var late string
	if l.cfg.Solver == SolverQP {
		var qerr error
		var accel []float64
		if l.coll != nil && l.coll.hasPairs() {
			accel = make([]float64, len(l.qCmd))
			for i := range accel {
				accel[i] = deg2rad(l.cfg.MaxJointDegPerS2[i])
			}
			var cerr error
			crows, cmin, cpair, cerr = l.coll.rows(l.qCmd, accel, dt, l.cfg.MaxLinearMMPerS*dt)
			if cerr != nil {
				return fmt.Errorf("collision: %w", cerr)
			}
			for _, r := range crows { // entered faster than it can stop: d_act was too small (live pairs can appear inside d_act, so skip them)
				if v := dotf(r.jd, l.lastStep); !r.live && -v > r.vmax+r.ad+lateTolMM {
					late = fmt.Sprintf("%s: approach %.3g mm/tick > v_max %.3g", l.coll.pairName(r.pair), -v, r.vmax)
				}
			}
		}
		next, qpActive, qpIters, qerr = l.qpNext(rows, scale, dt, crows, 0)
		if qerr != nil && len(crows) > 0 {
			relaxed = true
			next, qpActive, qpIters, qerr = l.qpNext(rows, scale, dt, crows, 1)
		}
		if qerr != nil && slices.ContainsFunc(crows, func(r collRow) bool { return r.backAway }) {
			// pinched: back-away rows can't all be met, so no further approach plus a soft retreat (spec "The row for a live pair")
			next, qpActive, qpIters, qerr = l.qpNext(rows, scale, dt, crows, 2)
			pinched = qerr == nil
		}
		switch {
		case qerr != nil && len(crows) > 0: // brake: DLS ignores obstacles
			bs := brakeStep(l.lastStep, accel, dt, l.qCmd, l.lo, l.hi)
			next = make([]float64, len(l.qCmd))
			for i := range next {
				next[i] = l.qCmd[i] + bs[i]
			}
			braked, fellBack, limStep = true, true, true
			if maxAbs(l.lastStep) != 0 {
				l.qpFails = 0 // earlier DLS-fallback failures don't count toward the at-rest fault
			} else { // at rest Δq = 0 is feasible: a real fault
				if l.qpFails++; l.qpFails >= 10 {
					return fmt.Errorf("qp: %w", qerr)
				}
			}
		case qerr != nil:
			l.qpFails++
			fellBack, next = true, nil
			if l.qpFails >= 10 {
				return fmt.Errorf("qp: %w", qerr)
			}
		default:
			l.qpFails, limStep = 0, qpActive > 0
		}
	}
	if next == nil {
		if next, limStep, err = l.clampStep(dq, dt); err != nil {
			return err
		}
	}

	// 9. send
	if err := l.io.Send(ctx, next); err != nil {
		return fmt.Errorf("sending joints: %w", err)
	}
	for i := range next {
		step := next[i] - l.qCmd[i]
		if r := math.Abs(step-l.lastStep[i]) / (deg2rad(l.cfg.MaxJointDegPerS2[i]) * dt * dt); r > l.peakAccel {
			l.peakAccel = r
		}
		l.lastStep[i] = step
	}
	copy(l.qCmd, next)
	// ponytail: the ring holds MaxLagMS of commands; a lag longer than that is treated as divergence (it
	// resyncs), not as lag. Raise max_lag_ms for slower arms, at the cost of a bigger snap on a real stall.
	if w := max(2, int(math.Ceil(l.cfg.MaxLagMS/1000*l.cfg.RateHz))); len(l.sent) >= w {
		old := l.sent[0]
		l.sent = append(l.sent[:0], l.sent[1:]...)
		l.sent = append(l.sent, append(old[:0], next...))
	} else {
		l.sent = append(l.sent, slices.Clone(next))
	}

	// 10. measured error, per task
	mp, mq, err := FK(l.model, qm)
	if err != nil {
		return err
	}
	ms, err := l.measure(tp, tq, mp, mq, qm)
	if err != nil {
		return err
	}
	limited := limStep || limitedTask

	l.mu.Lock()
	defer l.mu.Unlock()
	st := &l.status
	if resynced {
		st.Resyncs++
	}
	st.TargetMM = target.Point()
	st.ErrMM, st.ErrDeg = ms.posMM, ms.angDeg
	st.ReachDeg = ms.convDeg
	st.CmdErrMM, st.CmdErrDeg = cmdPos, cmdAng
	st.Damping, st.Limited = lambda, limited
	st.LagMS = l.lagTicks / l.cfg.RateHz * 1000
	st.Task, st.PointingResidualDeg, st.ToolAxis = l.cfg.Task, ms.residualDeg, toolAxis(cq)
	st.QCmd, st.QMeas = slices.Clone(l.qCmd), slices.Clone(qm)
	st.QPActive, st.QPIters, st.PeakAccelRatio = qpActive, qpIters, l.peakAccel
	if fellBack {
		st.QPFallbacks++
	}
	if l.coll != nil {
		st.Collision = "on"
		if l.coll.hasPairs() {
			st.CollisionMinMM, st.CollisionPair, st.CollisionRows = cmin, "", len(crows)
			if cpair >= 0 {
				st.CollisionPair = l.coll.pairName(cpair)
			}
		} else if l.live == nil {
			st.Collision = "off"
		}
		if relaxed {
			st.CollisionRelaxed++
		}
		if pinched {
			st.LivePinched++
		}
		if l.live != nil {
			st.LiveCount, st.LiveAgeMS, st.LiveWaiting = l.coll.liveCount(), float64(liveAge)/float64(time.Millisecond), liveWaiting
			st.LiveStale = liveAge > time.Duration(l.live.staleMS*float64(time.Millisecond))
			st.LiveBackingAway = slices.ContainsFunc(crows, func(r collRow) bool { return r.backAway })
		}
		if braked {
			st.CollisionBrakes++
		}
		st.CollisionIgnored, st.CollisionSkipped = l.coll.ignored(), slices.Clone(l.coll.skipped)
		if late != "" {
			st.CollisionLate = late
		}
	}
	l.lastReach = deg2rad(ms.convDeg)
	st.State = l.nextState(ms, limited, vel != ([6]float64{}))
	return nil
}

// canStop reports whether the last tick's step is small enough to stop from within the acceleration limit.
func (l *Loop) canStop(dt float64) bool {
	for i, d := range l.lastStep {
		if math.Abs(d) > deg2rad(l.cfg.MaxJointDegPerS2[i])*dt*dt {
			return false
		}
	}
	return true
}

// limitAccel limits this tick's joint step (rad) in place and reports whether it changed it. A joint
// moving toward a position limit is capped at √(2·a·d)·dt, d the distance left, so it can always stop in
// time. Then the change from the last tick's step is scaled, all joints together, to at most a·dt² per
// joint. Steps rather than velocities keep an unlimited tick bit-exact.
func (l *Loop) limitAccel(step []float64, dt float64) bool {
	changed := false
	k := 1.0
	for i := range step {
		a := deg2rad(l.cfg.MaxJointDegPerS2[i])
		d := l.hi[i] - l.qCmd[i]
		if step[i] < 0 {
			d = l.qCmd[i] - l.lo[i]
		}
		if c := math.Sqrt(2*a*math.Max(d, 0)) * dt; math.Abs(step[i]) > c {
			step[i] = math.Copysign(c, step[i])
			changed = true
		}
		if dv := math.Abs(step[i] - l.lastStep[i]); dv*k > a*dt*dt {
			k = a * dt * dt / dv
		}
	}
	if k < 1 {
		for i := range step {
			step[i] = l.lastStep[i] + k*(step[i]-l.lastStep[i])
		}
		changed = true
	}
	return changed
}

// sentAgo returns the command sent d ticks before the most recent one, or nil if the ring is shorter.
func (l *Loop) sentAgo(d int) []float64 {
	if d >= len(l.sent) {
		return nil
	}
	return l.sent[len(l.sent)-1-d]
}

// expected is where the arm should be now given its measured lag: the command sent round(lagTicks) ticks
// ago, or the oldest one still in the ring. With an empty ring it is qCmd.
func (l *Loop) expected() []float64 {
	if len(l.sent) == 0 {
		return l.qCmd
	}
	return l.sentAgo(min(int(math.Round(l.lagTicks)), len(l.sent)-1))
}

// updateLag fits qm to the ring of sent commands and moves lagTicks toward the best-fit age. It skips
// ticks where the commands barely moved over the window (under half of ResyncDeg: it must open before
// the resync test could trip), since the best fit is ambiguous at rest.
// A longer lag is taken at once (the lag grows from zero as motion starts, and an EMA would trail
// it into a resync), but only while the arm is moving, or before it has ever moved (dead time at start):
// a frozen qm under moving commands is a stall, not lag. A shorter lag is smoothed.
func (l *Loop) updateLag(qm []float64) {
	moving := l.prevQm != nil && maxAbsDiff(qm, l.prevQm) > moveFrac*deg2rad(slices.Max(l.cfg.MaxJointDegPerS))/l.cfg.RateHz
	l.prevQm = append(l.prevQm[:0], qm...)
	canRaise := moving || !l.moved
	l.moved = l.moved || moving
	n := len(l.sent)
	if n < 2 || maxAbsDiff(l.sent[0], l.sent[n-1]) <= deg2rad(l.cfg.ResyncDeg)/2 {
		return
	}
	best, bestD := math.Inf(1), 0
	for d := 0; d < n; d++ {
		if e := maxAbsDiff(qm, l.sentAgo(d)); e < best {
			best, bestD = e, d
		}
	}
	if f := float64(bestD); f > l.lagTicks {
		if canRaise {
			l.lagTicks = f
		}
	} else {
		l.lagTicks += lagAlpha * (f - l.lagTicks)
	}
}

// taskRows is the weighted task the QP's objective uses: ‖A·Δq − b‖² with A = W^½J and b = W^½Δx.
// solve fills it only for solver qp.
type taskRows struct {
	A      *mat.Dense
	b      []float64
	lambda float64 // the QP's damping: for pointing, max(λ₁, λ₂) from the first prioritySolve, before lockOutward
}

// clampStep applies steps 7, 7b and 8 (the DLS path) to dq: the uniform speed scaling, the acceleration
// limit and the position clamp. It returns the joints to send and whether any of them bound.
func (l *Loop) clampStep(dq []float64, dt float64) (next []float64, limited bool, err error) {
	// 7. joint velocity clamp, direction kept
	s := 1.0
	for i, d := range dq {
		if math.IsNaN(d) || math.IsInf(d, 0) {
			return nil, false, fmt.Errorf("non-finite joint step %v", dq)
		}
		if lim := deg2rad(l.cfg.MaxJointDegPerS[i]) * dt; math.Abs(d)*s > lim {
			s = lim / math.Abs(d)
		}
	}
	// 7b. acceleration: brake ahead of position limits, then clamp the velocity change, all joints together
	step := make([]float64, len(dq))
	for i, d := range dq {
		step[i] = d * s
	}
	accelLimited := l.limitAccel(step, dt)
	// 8. position limits
	next = make([]float64, len(dq))
	clamped := false
	for i := range dq {
		v := l.qCmd[i] + step[i]
		if v < l.lo[i] || v > l.hi[i] {
			v = math.Min(math.Max(v, l.lo[i]), l.hi[i])
			clamped = true
		}
		next[i] = v
	}
	return next, s < 1 || clamped || accelLimited, nil
}

// solve computes this tick's joint step for the configured task. cmdPos/cmdAng are the
// commanded-side errors in mm / deg (spec "Status fields").
func (l *Loop) solve(J *mat.Dense, tp r3.Vector, tq quat.Number, cp r3.Vector, cq quat.Number,
	vel [6]float64, scale, dt float64,
) (dq []float64, lambda float64, limited bool, cmdPos, cmdAng float64, rows taskRows, err error) {
	c := l.cfg
	_, n := J.Dims()
	if c.Task == TaskPointing {
		ac, at := toolAxis(cq), toolAxis(tq)
		r, u, v := pointingError(ac, at)
		e1 := tp.Sub(cp)
		v1 := capNorm(r3.Vector{X: vel[0], Y: vel[1], Z: vel[2]}.Add(e1.Mul(c.Gain)), c.MaxLinearMMPerS/mmPerM).Mul(scale)
		w := r3.Vector{X: vel[3], Y: vel[4], Z: vel[5]}
		v2 := capNorm(r3.Vector{X: u.Dot(w) + c.Gain*u.Dot(r), Y: v.Dot(w) + c.Gain*v.Dot(r)}, deg2rad(c.MaxAngularDegPerS)).Mul(scale)
		J1 := mat.DenseCopyOf(J.Slice(0, 3, 0, n))
		J2 := pointingRows(J, u, v)
		dx1, dx2 := []float64{v1.X * dt, v1.Y * dt, v1.Z * dt}, []float64{v2.X * dt, v2.Y * dt}
		if c.Solver == SolverQP {
			rows = pointingTaskRows(J1, J2, dx1, dx2)
		}
		dq, l1, l2, err := prioritySolve(J1, J2, dx1, dx2, c.SingularEps, c.MaxDamping)
		if c.Solver == SolverQP {
			rows.lambda = math.Max(l1, l2)
		}
		// Lock outward-at-limit joints and re-solve so the step is consistent with the post-hoc clamp
		// (else clamping one joint corrupts the position task). Each pass locks only new joints.
		for pass := 0; err == nil && pass < n && lockOutward(J1, J2, dq, l.qCmd, l.lo, l.hi); pass++ {
			dq, l1, l2, err = prioritySolve(J1, J2, dx1, dx2, c.SingularEps, c.MaxDamping)
		}
		if err != nil {
			return nil, 0, false, 0, 0, taskRows{}, err
		}
		return dq, math.Max(l1, l2), l1 > 0 || l2 > 0, e1.Norm() * mmPerM, r.Norm() * 180 / math.Pi, rows, nil
	}

	e := PoseError(tp, tq, cp, cq)
	for i, w := range c.TaskWeights { // a dropped row must not use up the active rows' speed cap
		if w == 0 {
			e[i], vel[i] = 0, 0
		}
	}
	lin := capNorm(r3.Vector{X: vel[0] + c.Gain*e[0], Y: vel[1] + c.Gain*e[1], Z: vel[2] + c.Gain*e[2]},
		c.MaxLinearMMPerS/mmPerM).Mul(scale)
	ang := capNorm(r3.Vector{X: vel[3] + c.Gain*e[3], Y: vel[4] + c.Gain*e[4], Z: vel[5] + c.Gain*e[5]},
		deg2rad(c.MaxAngularDegPerS)).Mul(scale)
	dx := [6]float64{lin.X * dt, lin.Y * dt, lin.Z * dt, ang.X * dt, ang.Y * dt, ang.Z * dt}
	cmdPos, cmdAng = activePosMM(e, c.TaskWeights), activeAngDeg(e, c.TaskWeights)

	var active []int
	for i, w := range c.TaskWeights {
		if w > 0 {
			active = append(active, i)
		}
	}
	if len(active) >= n { // unchanged Phase 1/2 path: bit-identical for the UR5e with all weights
		if c.Solver == SolverQP {
			A, b := mat.NewDense(6, n, nil), make([]float64, 6)
			for i := 0; i < 6; i++ {
				sw := math.Sqrt(c.TaskWeights[i])
				for j := 0; j < n; j++ {
					A.Set(i, j, sw*J.At(i, j))
				}
				b[i] = sw * dx[i]
			}
			rows = taskRows{A: A, b: b} // lambda set below from dlsStep
		}
		dq, lambda, sigmaMin, err := dlsStep(J, c.TaskWeights, dx, c.SingularEps, c.MaxDamping)
		rows.lambda = lambda
		return dq, lambda, sigmaMin < c.SingularEps, cmdPos, cmdAng, rows, err
	}
	A := mat.NewDense(len(active), n, nil)
	b := make([]float64, len(active))
	for k, i := range active {
		sw := math.Sqrt(c.TaskWeights[i])
		for j := 0; j < n; j++ {
			A.Set(k, j, sw*J.At(i, j))
		}
		b[k] = sw * dx[i]
	}
	if c.Solver == SolverQP {
		rows = taskRows{A: A, b: b}
	}
	P, lambda, sigmaR, _, err := dpinv(A, c.SingularEps, c.MaxDamping, true)
	if err != nil {
		return nil, 0, false, 0, 0, taskRows{}, err
	}
	rows.lambda = lambda
	var x mat.VecDense
	x.MulVec(P, mat.NewVecDense(len(b), b))
	return append([]float64(nil), x.RawVector().Data...), lambda, sigmaR < c.SingularEps, cmdPos, cmdAng, rows, nil
}

// measured holds the per-task measured errors used for status, convergence and stall.
type measured struct {
	posMM, angDeg        float64 // angDeg: pointing angle (pointing) or active orientation norm (pose)
	convDeg, residualDeg float64 // convDeg: angular error for convergence (reach in pointing, active orientation in pose); residualDeg: pointing only
	stallNorm            float64
}

func (l *Loop) measure(tp r3.Vector, tq quat.Number, mp r3.Vector, mq quat.Number, qm []float64) (measured, error) {
	c := l.cfg
	if c.Task != TaskPointing {
		em := PoseError(tp, tq, mp, mq)
		norm := 0.0
		for i, v := range em {
			norm += c.TaskWeights[i] * v * v
		}
		return measured{posMM: activePosMM(em, c.TaskWeights), angDeg: activeAngDeg(em, c.TaskWeights),
			convDeg: activeAngDeg(em, c.TaskWeights), stallNorm: math.Sqrt(norm)}, nil
	}
	Jm, err := Jacobian(l.model, qm)
	if err != nil {
		return measured{}, err
	}
	r, u, v := pointingError(toolAxis(mq), toolAxis(tq))
	_, n := Jm.Dims()
	// Servos sag inside a limit the command sits at: lock on whichever of qm/qCmd is at the limit.
	qLock := slices.Clone(qm)
	for i, c := range l.qCmd {
		if c >= l.hi[i]-limitEps || c <= l.lo[i]+limitEps {
			qLock[i] = c
		}
	}
	reach, res, err := pointingReach(mat.DenseCopyOf(Jm.Slice(0, 3, 0, n)), pointingRows(Jm, u, v), [2]float64{u.Dot(r), v.Dot(r)}, qLock, l.lo, l.hi)
	if err != nil {
		return measured{}, err
	}
	e1 := tp.Sub(mp).Norm()
	return measured{posMM: e1 * mmPerM, angDeg: r.Norm() * 180 / math.Pi, convDeg: reach * 180 / math.Pi,
		residualDeg: res * 180 / math.Pi, stallNorm: math.Hypot(e1, reach)}, nil
}

// nextState picks converged/stalled/running. moving means a feedforward twist is active. Caller holds l.mu.
// Converged: position within TolMM and the reachable/active angular error within TolDeg.
func (l *Loop) nextState(ms measured, limited, moving bool) State {
	if !moving && ms.posMM <= l.cfg.TolMM && ms.convDeg <= l.cfg.TolDeg {
		l.errRing = l.errRing[:0]
		return StateConverged
	}
	window := int(l.cfg.RateHz) // 1 s of ticks
	if len(l.errRing) == window {
		l.errRing = l.errRing[1:]
	}
	l.errRing = append(l.errRing, ms.stallNorm)
	if limited && len(l.errRing) == window && ms.stallNorm > 0.99*l.errRing[0] {
		return StateStalled
	}
	return StateRunning
}

// activePosMM / activeAngDeg are the norms of the components whose weight is > 0.
func activePosMM(e [6]float64, w [6]float64) float64 {
	s := 0.0
	for i := 0; i < 3; i++ {
		if w[i] > 0 {
			s += e[i] * e[i]
		}
	}
	return math.Sqrt(s) * mmPerM
}

func activeAngDeg(e [6]float64, w [6]float64) float64 {
	s := 0.0
	for i := 3; i < 6; i++ {
		if w[i] > 0 {
			s += e[i] * e[i]
		}
	}
	return math.Sqrt(s) * 180 / math.Pi
}

// Oscillations counts, per joint, sign reversals between successive steps whose size exceeds minStep[i].
// Smaller steps are skipped so jitter near rest is not counted.
func Oscillations(qs [][]float64, minStep []float64) []int {
	out := make([]int, len(minStep))
	for i := range minStep {
		prev := 0.0
		for k := 1; k < len(qs); k++ {
			d := qs[k][i] - qs[k-1][i]
			if math.Abs(d) <= minStep[i] {
				continue
			}
			if prev != 0 && (d > 0) != (prev > 0) {
				out[i]++
			}
			prev = d
		}
	}
	return out
}

func deg2rad(d float64) float64 { return d * math.Pi / 180 }
func rad2deg(r float64) float64 { return r * 180 / math.Pi }

func capNorm(v r3.Vector, max float64) r3.Vector {
	if n := v.Norm(); n > max {
		return v.Mul(max / n)
	}
	return v
}

func maxAbsDiff(a, b []float64) float64 {
	m := 0.0
	for i := range a {
		m = math.Max(m, math.Abs(a[i]-b[i]))
	}
	return m
}

func dotf(a, b []float64) float64 {
	s := 0.0
	for i := range a {
		s += a[i] * b[i]
	}
	return s
}

func maxAbs(a []float64) float64 {
	m := 0.0
	for _, v := range a {
		m = math.Max(m, math.Abs(v))
	}
	return m
}
