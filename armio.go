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
	a  arm.Arm
	dt time.Duration

	batches chan []arm.TrajectoryPoint
	s       *stream
	cancel  context.CancelFunc
	start   time.Time
	lastT   time.Duration
}

// NewArmIO returns a JointIO for a, pacing waypoints at rateHz.
func NewArmIO(a arm.Arm, rateHz float64) JointIO {
	return &armIO{a: a, dt: time.Duration(float64(time.Second) / rateHz)}
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
		batch = append(batch, arm.TrajectoryPoint{Time: 0, Positions: cur}) // first point must be t=0
	}
	t := time.Since(io.start) + io.dt
	if t <= io.lastT {
		t = io.lastT + time.Microsecond // times must strictly increase
	}
	io.lastT = t
	batch = append(batch, arm.TrajectoryPoint{Time: t, Positions: slices.Clone(q)})

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

func (io *armIO) Stop(ctx context.Context) error {
	var wait chan struct{}
	if io.batches != nil {
		// Close batches as well as cancel: drivers may only watch the channel (the fake arm does).
		// Send and Stop both run on the loop goroutine, so nothing can send after this close.
		close(io.batches)
		io.cancel()
		wait = io.s.done
		io.batches = nil
	}
	// Stop the arm first: a hung stream must not eat the caller's deadline.
	err := io.a.Stop(ctx, nil)
	if wait != nil {
		select {
		case <-wait:
		case <-ctx.Done():
		}
	}
	return err
}
