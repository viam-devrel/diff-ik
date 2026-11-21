package diffik

import (
	"context"
	"errors"
	"fmt"
	"slices"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/referenceframe"
)

// stream is one streamed move's completion state; err is read only after done is closed.
type stream struct {
	done chan struct{}
	err  error
}

// armIO drives a real arm through MoveThroughJointPositionsStreamed.
// The stream opens on the first Send; each Send adds one waypoint due one tick later.
type armIO struct {
	a    arm.Arm
	lead time.Duration // how far ahead of now each point is stamped

	pointVel bool                   // attach per-point target velocities
	period   float64                // nominal tick, seconds
	prevQ    []referenceframe.Input // last sent positions (the seed at first)

	batches chan []arm.TrajectoryPoint
	s       *stream
	cancel  context.CancelFunc
	start   time.Time
	lastT   time.Duration
}

// DefaultLead is how far ahead of now each streamed waypoint is stamped. On the real SO-101
// (laptop-local client, 50 Hz) 5 ms took ~15 ms off the lag of a one-tick lead with 0 late
// points (docs/results/2026-11-phase3c1.md). ponytail: a global default from one setup; if a
// slower host or remote client logs late points, make it a servo_start option.
const DefaultLead = 5 * time.Millisecond

// NewArmIO returns a JointIO for a, stamping each waypoint DefaultLead ahead (at most one tick).
func NewArmIO(a arm.Arm, rateHz float64) JointIO {
	return NewArmIOLead(a, rateHz, 0)
}

// NewArmIOLead is NewArmIO with each waypoint stamped lead ahead of now; lead <= 0 means
// DefaultLead. The lead is capped at one tick (1/rateHz). A shorter lead cuts streaming lag,
// at the risk of the driver receiving points late (the so-101 driver logs them).
func NewArmIOLead(a arm.Arm, rateHz float64, lead time.Duration) JointIO {
	if lead <= 0 {
		lead = DefaultLead
	}
	return &armIO{a: a, lead: min(lead, time.Duration(float64(time.Second)/rateHz)), pointVel: true, period: 1 / rateHz}
}

func (io *armIO) Read(ctx context.Context) ([]referenceframe.Input, error) {
	return io.a.JointPositions(ctx, nil)
}

func (io *armIO) Send(ctx context.Context, q []referenceframe.Input) error {
	var batch []arm.TrajectoryPoint
	if io.batches == nil {
		cur, err := io.a.JointPositions(ctx, nil)
		if err != nil {
			return err
		}
		io.open()
		io.prevQ = slices.Clone(cur)
		batch = append(batch, arm.TrajectoryPoint{Time: 0, Positions: cur}) // first point must be t=0
	}
	t := time.Since(io.start) + io.lead
	if t <= io.lastT {
		t = io.lastT + time.Microsecond // times must strictly increase
	}
	io.lastT = t
	pt := arm.TrajectoryPoint{Time: t, Positions: slices.Clone(q)}
	if io.pointVel && len(io.prevQ) == len(q) {
		v := make([]float64, len(q))
		for i := range q {
			v[i] = (q[i] - io.prevQ[i]) / io.period // nominal tick, not the measured interval
		}
		pt.Constraints = &arm.KinematicConstraints{Velocities: v}
	}
	io.prevQ = slices.Clone(pt.Positions)
	batch = append(batch, pt)

	select {
	case io.batches <- batch:
		return nil
	case <-io.s.done:
		if io.s.err == nil {
			return errors.New("arm stream ended unexpectedly")
		}
		return fmt.Errorf("arm stream: %w", io.s.err)
	case <-ctx.Done():
		return ctx.Err()
	}
}

// Hold sends q as a point the arm should come to rest at: its point velocity is zero, not the jump from the
// last point over one tick (the guard's trip snaps the command back to the measured joints).
func (io *armIO) Hold(ctx context.Context, q []referenceframe.Input) error {
	if io.batches != nil {
		io.prevQ = slices.Clone(q)
	}
	return io.Send(ctx, q)
}

// Abort is the guard's trip: cancel the stream at once (no drain, so queued points don't play out), stop the
// arm, then move it to q, the measured joints, for drivers whose Stop holds the last command.
func (io *armIO) Abort(ctx context.Context, q []referenceframe.Input) error {
	var wait chan struct{}
	if io.batches != nil {
		io.cancel()
		close(io.batches)
		wait, io.batches = io.s.done, nil
	}
	err := io.a.Stop(ctx, nil)
	if err == nil {
		err = io.a.MoveToJointPositions(ctx, q, nil)
	}
	if wait != nil {
		select {
		case <-wait:
		case <-ctx.Done():
		}
	}
	return err
}

// open starts the streamed move. It runs on its own context so that a canceled loop
// context doesn't tear the stream down before Stop has run.
func (io *armIO) open() {
	ctx, cancel := context.WithCancel(context.Background())
	io.cancel = cancel
	io.batches = make(chan []arm.TrajectoryPoint)
	st := &stream{done: make(chan struct{})}
	io.s = st
	batches := io.batches
	io.start, io.lastT = time.Now(), 0
	responses := make(chan arm.Response)
	go func() {
		for range responses { // acks carry nothing we need
		}
	}()
	go func() {
		st.err = io.a.MoveThroughJointPositionsStreamed(ctx, batches, responses, nil)
		close(responses)
		close(st.done)
	}()
}

// drainWait is how long Stop lets an open stream finish on its own after the batch channel
// closes. A normally ended stream lets the driver log its late-point count (so-101 logs it
// only then); a cancelled one returns without it. Every Stop pays this, fault and dead-man stops
// included: that's safe because once the channel closes no new points go out and the servos hold
// the last goal, at most one tick ahead.
const drainWait = 100 * time.Millisecond

func (io *armIO) Stop(ctx context.Context) error {
	var wait chan struct{}
	if io.batches != nil {
		// Close batches first: drivers may only watch the channel (the fake arm does).
		// Send and Stop both run on the loop goroutine, so nothing can send after this close.
		close(io.batches)
		wait = io.s.done
		d := drainWait
		if dl, ok := ctx.Deadline(); ok {
			d = min(d, time.Until(dl)/2) // leave arm.Stop at least half the caller's budget
		}
		select {
		case <-wait:
		case <-time.After(d):
		case <-ctx.Done():
		}
		io.cancel()
		io.batches = nil
	}
	// Stop the arm before waiting on the stream; the drain above is capped at half the budget so a
	// hung stream can't eat the caller's deadline.
	err := io.a.Stop(ctx, nil)
	if wait != nil {
		select {
		case <-wait:
		case <-ctx.Done():
		}
	}
	return err
}
