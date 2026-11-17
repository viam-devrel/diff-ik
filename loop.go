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
	SingularEps       float64
	MaxDamping        float64
	ResyncDeg         float64
	MaxLagMS          float64 // longest arm lag treated as lag; beyond it a gap is divergence and resyncs. Default 300
	TolMM             float64
	TolDeg            float64
	Task              string // "pose" | "pointing"; empty = pointing if DOF < 6 else pose
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
)

const (
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
}

const tickWindow = 500 // ticks kept for p50/p99

const (
	lagAlpha = 0.1 // EMA weight of a new lag sample when the lag estimate falls
	// moveFrac: qm counts as moving when a joint changed by more than this fraction of one tick of
	// travel at the top joint speed. Below it, a still arm's sensor noise cannot raise the lag estimate.
	moveFrac = 0.1
)

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

	qCmd []float64

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
	status    Status
	lastReach float64 // pointing mode: |e2_reach| (rad) from the previous tick, for the leash. Not reset on
	// SetMotion: a streamed servo_target calls SetMotion every tick, and resetting would disable the leash.
	tickDurs []time.Duration
	ticks    int
	first    time.Time // start of the first tick
	last     time.Time // start of the latest tick
}

// NewLoop builds a loop for model, driven through io.
func NewLoop(model referenceframe.Model, io JointIO, cfg LoopConfig) (*Loop, error) {
	limits := model.DoF()
	dof := len(limits)
	cfg = cfg.withDefaults(dof)
	if err := cfg.validate(dof); err != nil {
		return nil, err
	}
	l := &Loop{model: model, io: io, cfg: cfg, lo: make([]float64, dof), hi: make([]float64, dof)}
	for i, lim := range limits {
		// stay fdStep inside the limits so Jacobian perturbations remain valid inputs
		l.lo[i], l.hi[i] = lim.Min+fdStep, lim.Max-fdStep
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
//   - Leash, per axis: an advance is skipped only when the commanded-side error already exceeds
//     leashS × the speed limit and the advance would grow it (vel · e > 0).
func (l *Loop) advance(dt float64, cp r3.Vector, cq quat.Number, ramping bool) (spatialmath.Pose, [6]float64) {
	var zero [6]float64
	if l.extrap && l.now().Sub(l.lastSet) > l.holdAfter {
		return l.target, zero
	}
	vel := l.vel
	if ramping || vel == zero {
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
	l.mu.Lock()
	ramping, scale := l.rampScale()
	if ramping && scale == 0 {
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
	}
	if !hasTarget {
		l.mu.Lock()
		l.status.State = StateIdle
		l.status.QMeas = slices.Clone(qm)
		l.mu.Unlock()
		return nil
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
	target, vel := l.advance(dt, cp, cq, ramping)
	l.mu.Unlock()
	tp, tq := target.Point().Mul(1/mmPerM), target.Orientation().Quaternion()

	// 4–6. task-space step: pose → weighted DLS (unchanged when m ≥ n) or dpinv; pointing → priority
	J, err := Jacobian(l.model, l.qCmd)
	if err != nil {
		return err
	}
	dq, lambda, limitedTask, cmdPos, cmdAng, err := l.solve(J, tp, tq, cp, cq, vel, scale, dt)
	if err != nil {
		return err
	}

	// 7. joint velocity clamp, direction kept
	s := 1.0
	for i, d := range dq {
		if math.IsNaN(d) || math.IsInf(d, 0) {
			return fmt.Errorf("non-finite joint step %v", dq)
		}
		if lim := deg2rad(l.cfg.MaxJointDegPerS[i]) * dt; math.Abs(d)*s > lim {
			s = lim / math.Abs(d)
		}
	}

	// 8. position limits
	next := make([]float64, len(dq))
	clamped := false
	for i, d := range dq {
		v := l.qCmd[i] + d*s
		if v < l.lo[i] || v > l.hi[i] {
			v = math.Min(math.Max(v, l.lo[i]), l.hi[i])
			clamped = true
		}
		next[i] = v
	}

	// 9. send
	if err := l.io.Send(ctx, next); err != nil {
		return fmt.Errorf("sending joints: %w", err)
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
	limited := s < 1 || clamped || limitedTask

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
	l.lastReach = deg2rad(ms.convDeg)
	st.State = l.nextState(ms, limited, vel != ([6]float64{}))
	return nil
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

// solve computes this tick's joint step for the configured task. cmdPos/cmdAng are the
// commanded-side errors in mm / deg (spec "Status fields").
func (l *Loop) solve(J *mat.Dense, tp r3.Vector, tq quat.Number, cp r3.Vector, cq quat.Number,
	vel [6]float64, scale, dt float64,
) (dq []float64, lambda float64, limited bool, cmdPos, cmdAng float64, err error) {
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
		dq, l1, l2, err := prioritySolve(J1, J2, dx1, dx2, c.SingularEps, c.MaxDamping)
		// Lock outward-at-limit joints and re-solve so the step is consistent with the post-hoc clamp
		// (else clamping one joint corrupts the position task). Each pass locks only new joints.
		for pass := 0; err == nil && pass < n && lockOutward(J1, J2, dq, l.qCmd, l.lo, l.hi); pass++ {
			dq, l1, l2, err = prioritySolve(J1, J2, dx1, dx2, c.SingularEps, c.MaxDamping)
		}
		if err != nil {
			return nil, 0, false, 0, 0, err
		}
		return dq, math.Max(l1, l2), l1 > 0 || l2 > 0, e1.Norm() * mmPerM, r.Norm() * 180 / math.Pi, nil
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

	var rows []int
	for i, w := range c.TaskWeights {
		if w > 0 {
			rows = append(rows, i)
		}
	}
	if len(rows) >= n { // unchanged Phase 1/2 path: bit-identical for the UR5e with all weights
		dq, lambda, sigmaMin, err := dlsStep(J, c.TaskWeights, dx, c.SingularEps, c.MaxDamping)
		return dq, lambda, sigmaMin < c.SingularEps, cmdPos, cmdAng, err
	}
	A := mat.NewDense(len(rows), n, nil)
	b := make([]float64, len(rows))
	for k, i := range rows {
		sw := math.Sqrt(c.TaskWeights[i])
		for j := 0; j < n; j++ {
			A.Set(k, j, sw*J.At(i, j))
		}
		b[k] = sw * dx[i]
	}
	P, lambda, sigmaR, _, err := dpinv(A, c.SingularEps, c.MaxDamping, true)
	if err != nil {
		return nil, 0, false, 0, 0, err
	}
	var x mat.VecDense
	x.MulVec(P, mat.NewVecDense(len(b), b))
	return append([]float64(nil), x.RawVector().Data...), lambda, sigmaR < c.SingularEps, cmdPos, cmdAng, nil
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
