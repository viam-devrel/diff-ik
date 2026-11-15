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
	TolMM             float64
	TolDeg            float64
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
	def(&c.TolMM, 2)
	def(&c.TolDeg, 2)
	return c
}

func (c LoopConfig) validate(dof int) error {
	if c.RateHz < 1 {
		return fmt.Errorf("rate_hz must be >= 1, got %v", c.RateHz)
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
		{"singular_eps", c.SingularEps}, {"max_damping", c.MaxDamping}, {"resync_deg", c.ResyncDeg},
		{"tol_mm", c.TolMM}, {"tol_deg", c.TolDeg},
	} {
		if !(p.v > 0) {
			return fmt.Errorf("%s must be > 0, got %v", p.name, p.v)
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
	Damping             float64
	Limited             bool
	QCmd, QMeas         []float64
}

const tickWindow = 500 // ticks kept for p50/p99

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
	tickDurs  []time.Duration
	ticks     int
	first     time.Time // start of the first tick
	last      time.Time // start of the latest tick
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
	vl, el := r3.Vector{X: vel[0], Y: vel[1], Z: vel[2]}, r3.Vector{X: e[0], Y: e[1], Z: e[2]}
	if !(el.Norm() > leashS*l.cfg.MaxLinearMMPerS/mmPerM && vl.Dot(el) > 0) {
		tp = tp.Add(vl.Mul(dt))
	}
	va, ea := r3.Vector{X: vel[3], Y: vel[4], Z: vel[5]}, r3.Vector{X: e[3], Y: e[4], Z: e[5]}
	if !(ea.Norm() > leashS*deg2rad(l.cfg.MaxAngularDegPerS) && va.Dot(ea) > 0) {
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
	if maxAbsDiff(l.qCmd, qm) > deg2rad(l.cfg.ResyncDeg) {
		copy(l.qCmd, qm)
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
	e := PoseError(tp, tq, cp, cq)

	// 4–5. v = vel + Gain·e, capped, scaled by any ramp → task step
	lin := capNorm(r3.Vector{X: vel[0] + l.cfg.Gain*e[0], Y: vel[1] + l.cfg.Gain*e[1], Z: vel[2] + l.cfg.Gain*e[2]},
		l.cfg.MaxLinearMMPerS/mmPerM).Mul(scale)
	ang := capNorm(r3.Vector{X: vel[3] + l.cfg.Gain*e[3], Y: vel[4] + l.cfg.Gain*e[4], Z: vel[5] + l.cfg.Gain*e[5]},
		deg2rad(l.cfg.MaxAngularDegPerS)).Mul(scale)
	dx := [6]float64{lin.X * dt, lin.Y * dt, lin.Z * dt, ang.X * dt, ang.Y * dt, ang.Z * dt}

	// 6. weighted DLS
	J, err := Jacobian(l.model, l.qCmd)
	if err != nil {
		return err
	}
	dq, lambda, sigmaMin, err := dlsStep(J, l.cfg.TaskWeights, dx, l.cfg.SingularEps, l.cfg.MaxDamping)
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

	// 10. measured error
	mp, mq, err := FK(l.model, qm)
	if err != nil {
		return err
	}
	em := PoseError(tp, tq, mp, mq)
	limited := s < 1 || clamped || sigmaMin < l.cfg.SingularEps

	l.mu.Lock()
	defer l.mu.Unlock()
	st := &l.status
	if resynced {
		st.Resyncs++
	}
	st.TargetMM = target.Point()
	st.ErrMM, st.ErrDeg = posMM(em), angDeg(em)
	st.CmdErrMM, st.CmdErrDeg = posMM(e), angDeg(e)
	st.Damping, st.Limited = lambda, limited
	st.QCmd, st.QMeas = slices.Clone(l.qCmd), slices.Clone(qm)
	st.State = l.nextState(em, limited, vel != ([6]float64{}))
	return nil
}

// nextState picks converged/stalled/running. moving means a feedforward twist is active. Caller holds l.mu.
func (l *Loop) nextState(em [6]float64, limited, moving bool) State {
	if !moving && posMM(em) <= l.cfg.TolMM && angDeg(em) <= l.cfg.TolDeg {
		l.errRing = l.errRing[:0]
		return StateConverged
	}
	norm := 0.0
	for i, v := range em {
		norm += l.cfg.TaskWeights[i] * v * v
	}
	norm = math.Sqrt(norm)
	window := int(l.cfg.RateHz) // 1 s of ticks
	if len(l.errRing) == window {
		l.errRing = l.errRing[1:]
	}
	l.errRing = append(l.errRing, norm)
	if limited && len(l.errRing) == window && norm > 0.99*l.errRing[0] {
		return StateStalled
	}
	return StateRunning
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

func deg2rad(d float64) float64  { return d * math.Pi / 180 }
func posMM(e [6]float64) float64 { return math.Sqrt(e[0]*e[0]+e[1]*e[1]+e[2]*e[2]) * mmPerM }
func angDeg(e [6]float64) float64 {
	return math.Sqrt(e[3]*e[3]+e[4]*e[4]+e[5]*e[5]) * 180 / math.Pi
}

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
