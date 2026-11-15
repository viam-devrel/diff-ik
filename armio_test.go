package diffik

import (
	"context"
	"slices"
	"testing"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/arm/fake"
	"go.viam.com/rdk/components/arm/kinematics"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
)

// The fake arm teleports to each streamed waypoint, so this checks armIO's stream plumbing.
func TestArmIOStreamsToFakeArm(t *testing.T) {
	ctx := context.Background()
	cfg := resource.Config{Name: "arm", ConvertedAttributes: &fake.Config{ArmModel: kinematics.UR5e}}
	a, err := fake.NewArm(ctx, nil, cfg, logging.NewTestLogger(t))
	if err != nil {
		t.Fatal(err)
	}
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
	cfg := resource.Config{Name: "arm", ConvertedAttributes: &fake.Config{ArmModel: kinematics.UR5e}}
	a, err := fake.NewArm(context.Background(), nil, cfg, logging.NewTestLogger(t))
	if err != nil {
		t.Fatal(err)
	}
	h := &hungArm{Arm: a, release: make(chan struct{}), stopped: make(chan struct{})}
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
