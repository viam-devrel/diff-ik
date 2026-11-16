package diffik

import (
	"context"
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
