package diffik

import (
	"context"
	"math"
	"slices"
	"sync/atomic"
	"testing"
	"time"

	"go.viam.com/rdk/components/arm"
)

// The fake arm teleports to each streamed waypoint, so this checks armIO's stream plumbing.
func TestArmIOStreamsToFakeArm(t *testing.T) {
	ctx := context.Background()
	a := fakeUR5e(t)
	io := NewArmIO(a, 50)
	want := []float64{0.1, -1.2, 1.5, -1.9, -1.57, 0}
	for _, q := range [][]float64{{0, -1.2, 1.5, -1.9, -1.57, 0}, want} {
		if err := io.Send(ctx, q); err != nil {
			t.Fatal(err)
		}
	}
	// Send returns once the batch is handed over; give the fake a moment to apply it.
	deadline := time.Now().Add(time.Second)
	for {
		got, err := io.Read(ctx)
		if err != nil {
			t.Fatal(err)
		}
		if slices.Equal(got, want) {
			break
		}
		if time.Now().After(deadline) {
			t.Fatalf("joints = %v, want %v", got, want)
		}
		time.Sleep(5 * time.Millisecond)
	}
	if err := io.Stop(ctx); err != nil {
		t.Fatal(err)
	}
}

// hungArm reads one batch, then ignores both the channel and ctx, like a wedged driver.
type hungArm struct {
	arm.Arm
	release chan struct{}
	stopped chan struct{}
	stopErr error
}

func (h *hungArm) MoveThroughJointPositionsStreamed(ctx context.Context, b <-chan []arm.TrajectoryPoint, r chan<- arm.Response, extra map[string]interface{}) error {
	<-b
	<-h.release
	return nil
}

func (h *hungArm) Stop(ctx context.Context, extra map[string]interface{}) error {
	h.stopErr = ctx.Err()
	close(h.stopped)
	return nil
}

func TestArmIOStopReachesArmWhenStreamHangs(t *testing.T) {
	h := &hungArm{Arm: fakeUR5e(t), release: make(chan struct{}), stopped: make(chan struct{})}
	t.Cleanup(func() { close(h.release) })
	io := NewArmIO(h, 50)
	if err := io.Send(context.Background(), []float64{0, -1.2, 1.5, -1.9, -1.57, 0}); err != nil {
		t.Fatal(err)
	}
	ctx, cancel := context.WithTimeout(context.Background(), 100*time.Millisecond)
	defer cancel()
	start := time.Now()
	if err := io.Stop(ctx); err != nil {
		t.Fatal(err)
	}
	if time.Since(start) > time.Second {
		t.Fatalf("Stop took %v", time.Since(start))
	}
	select {
	case <-h.stopped:
		if h.stopErr != nil {
			t.Fatalf("arm.Stop got an already-done ctx: %v", h.stopErr)
		}
	default:
		t.Fatal("arm.Stop was not called")
	}
}

// drainArm finishes its stream shortly after the batch channel closes, and records whether
// its context was cancelled first.
type drainArm struct {
	arm.Arm
	after     time.Duration // how long after close it returns
	cancelled atomic.Bool
}

func (d *drainArm) MoveThroughJointPositionsStreamed(ctx context.Context, b <-chan []arm.TrajectoryPoint, r chan<- arm.Response, extra map[string]interface{}) error {
	for range b {
	}
	select {
	case <-time.After(d.after):
		return nil
	case <-ctx.Done():
		d.cancelled.Store(true)
		return ctx.Err()
	}
}

func (d *drainArm) Stop(context.Context, map[string]interface{}) error { return nil }

func TestArmIOStopDrainsBeforeCancel(t *testing.T) {
	d := &drainArm{Arm: fakeUR5e(t), after: 20 * time.Millisecond}
	io := NewArmIO(d, 50)
	if err := io.Send(context.Background(), []float64{0, -1.2, 1.5, -1.9, -1.57, 0}); err != nil {
		t.Fatal(err)
	}
	if err := io.Stop(context.Background()); err != nil {
		t.Fatal(err)
	}
	if d.cancelled.Load() {
		t.Fatal("stream that finishes inside the drain window was cancelled")
	}
}

func TestArmIOStopCancelsAfterDrainWindow(t *testing.T) {
	d := &drainArm{Arm: fakeUR5e(t), after: time.Hour}
	io := NewArmIO(d, 50)
	if err := io.Send(context.Background(), []float64{0, -1.2, 1.5, -1.9, -1.57, 0}); err != nil {
		t.Fatal(err)
	}
	start := time.Now()
	if err := io.Stop(context.Background()); err != nil {
		t.Fatal(err)
	}
	if el := time.Since(start); el < 100*time.Millisecond || el > time.Second {
		t.Fatalf("Stop took %v, want ~100 ms", el)
	}
	if !d.cancelled.Load() {
		t.Fatal("hung stream was not cancelled")
	}
}

// leadArm records the first batch's point times.
type leadArm struct {
	arm.Arm
	times chan []time.Duration
}

func (l *leadArm) MoveThroughJointPositionsStreamed(ctx context.Context, b <-chan []arm.TrajectoryPoint, r chan<- arm.Response, extra map[string]interface{}) error {
	batch := <-b
	ts := make([]time.Duration, len(batch))
	for i, p := range batch {
		ts[i] = p.Time
	}
	l.times <- ts
	for range b {
	}
	return nil
}

func (l *leadArm) Stop(context.Context, map[string]interface{}) error { return nil }

func TestArmIOLeadSetsPointTime(t *testing.T) {
	l := &leadArm{Arm: fakeUR5e(t), times: make(chan []time.Duration, 1)}
	io := NewArmIOLead(l, 50, 5*time.Millisecond)
	if err := io.Send(context.Background(), []float64{0, -1.2, 1.5, -1.9, -1.57, 0}); err != nil {
		t.Fatal(err)
	}
	ts := <-l.times
	if len(ts) != 2 || ts[0] != 0 || ts[1] < 5*time.Millisecond || ts[1] > 15*time.Millisecond {
		t.Fatalf("point times %v, want [0, ~5ms] (lead 5 ms, not the 20 ms tick)", ts)
	}
	if err := io.Stop(context.Background()); err != nil {
		t.Fatal(err)
	}
}

func TestArmIODefaultLeadIsCappedAtOneTick(t *testing.T) {
	if got := NewArmIO(nil, 50).(*armIO).lead; got != DefaultLead {
		t.Fatalf("50 Hz lead %v, want DefaultLead %v", got, DefaultLead)
	}
	if got := NewArmIO(nil, 1000).(*armIO).lead; got != time.Millisecond {
		t.Fatalf("1 kHz lead %v, want one tick (1 ms)", got)
	}
}

// capArm records every streamed point.
type capArm struct {
	arm.Arm
	pts chan arm.TrajectoryPoint
}

func (c *capArm) MoveThroughJointPositionsStreamed(ctx context.Context, b <-chan []arm.TrajectoryPoint, r chan<- arm.Response, extra map[string]interface{}) error {
	for batch := range b {
		for _, p := range batch {
			c.pts <- p
		}
	}
	return nil
}

func (c *capArm) Stop(context.Context, map[string]interface{}) error { return nil }

func streamPoints(t *testing.T, pointVel bool, qs ...[]float64) (cur []float64, pts []arm.TrajectoryPoint) {
	t.Helper()
	c := &capArm{Arm: fakeUR5e(t), pts: make(chan arm.TrajectoryPoint, 16)}
	io := NewArmIO(c, 50)
	io.(*armIO).pointVel = pointVel
	cur, err := io.Read(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	for _, q := range qs {
		if err := io.Send(context.Background(), q); err != nil {
			t.Fatal(err)
		}
	}
	if err := io.Stop(context.Background()); err != nil {
		t.Fatal(err)
	}
	for range len(qs) + 1 {
		pts = append(pts, <-c.pts)
	}
	return cur, pts
}

func TestArmIOPointVelocities(t *testing.T) {
	q1 := []float64{0.1, -1.2, 1.5, -1.9, -1.57, 0}
	q2 := []float64{0.12, -1.2, 1.49, -1.9, -1.57, 0.01}
	cur, pts := streamPoints(t, true, q1, q2)
	if pts[0].Constraints != nil {
		t.Fatalf("seed point has Constraints %v", pts[0].Constraints)
	}
	for k, q := range [][]float64{q1, q2} {
		prev := cur
		if k == 1 {
			prev = q1
		}
		c := pts[k+1].Constraints
		if c == nil || c.Accelerations != nil || len(c.Velocities) != len(q) {
			t.Fatalf("point %d constraints = %+v", k+1, c)
		}
		for i := range q {
			if want := (q[i] - prev[i]) * 50; math.Abs(c.Velocities[i]-want) > 1e-9 {
				t.Errorf("point %d v[%d] = %v, want %v", k+1, i, c.Velocities[i], want)
			}
		}
	}
}

func TestArmIOPointVelocitiesOff(t *testing.T) {
	_, pts := streamPoints(t, false, []float64{0.1, -1.2, 1.5, -1.9, -1.57, 0})
	for i, p := range pts {
		if p.Constraints != nil {
			t.Errorf("point %d has Constraints %v with point velocities off", i, p.Constraints)
		}
	}
}
