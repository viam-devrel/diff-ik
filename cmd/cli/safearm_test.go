package main

import (
	"context"
	"errors"
	"sync"
	"testing"
	"time"

	"go.viam.com/rdk/components/arm"
)

// recArm records calls. MoveToJointPositions blocks until release is closed and records
// whether its context was cancelled; joints stay where the move left them.
type recArm struct {
	arm.Arm
	mu        sync.Mutex
	q         []float64
	calls     []string
	release   chan struct{}
	moveCtxOK bool
	drift     int // after a move, this many JointPositions reads still change q; <0 means forever
}

func (r *recArm) log(s string) { r.mu.Lock(); r.calls = append(r.calls, s); r.mu.Unlock() }

func (r *recArm) JointPositions(context.Context, map[string]interface{}) ([]float64, error) {
	r.mu.Lock()
	defer r.mu.Unlock()
	if r.drift != 0 {
		if r.drift > 0 {
			r.drift--
		}
		r.q[0] += 0.01
	}
	return append([]float64(nil), r.q...), nil
}

func (r *recArm) IsMoving(context.Context) (bool, error) { return false, nil }

func (r *recArm) MoveToJointPositions(ctx context.Context, q []float64, _ map[string]interface{}) error {
	r.log("move")
	<-r.release
	r.mu.Lock()
	r.moveCtxOK = ctx.Err() == nil
	r.q = append([]float64(nil), q...)
	r.mu.Unlock()
	return nil
}

func (r *recArm) Stop(context.Context, map[string]interface{}) error { r.log("stop"); return nil }

func (r *recArm) has(c string) bool {
	r.mu.Lock()
	defer r.mu.Unlock()
	for _, x := range r.calls {
		if x == c {
			return true
		}
	}
	return false
}

func waitFor(t *testing.T, cond func() bool) {
	t.Helper()
	for i := 0; i < 500 && !cond(); i++ {
		time.Sleep(2 * time.Millisecond)
	}
	if !cond() {
		t.Fatal("condition not reached")
	}
}

func TestSafeArmNoStopDuringProfiledMove(t *testing.T) {
	r := &recArm{q: []float64{0, 0, 0, 0, 0}, release: make(chan struct{})}
	s := &safeArm{Arm: r}
	ctx, cancel := context.WithCancel(context.Background())
	done := make(chan error, 1)
	go func() { done <- s.MoveToJointPositions(ctx, []float64{0.1, 0, 0, 0, 0}, nil) }()
	waitFor(t, func() bool { return r.has("move") })
	cancel() // Ctrl-C
	if err := s.Stop(context.Background(), nil); err != nil {
		t.Fatal(err)
	}
	select {
	case <-done:
		t.Fatal("move returned before the arm finished")
	case <-time.After(50 * time.Millisecond):
	}
	close(r.release)
	if err := <-done; !errors.Is(err, context.Canceled) {
		t.Fatalf("err = %v, want context.Canceled once the move finished", err)
	}
	if !r.moveCtxOK {
		t.Fatal("Ctrl-C cancelled the profiled move's RPC context")
	}
	if r.has("stop") {
		t.Fatalf("arm.Stop reached the arm during a profiled move: %v", r.calls)
	}
	// after the move, Stop goes through again
	if err := s.Stop(context.Background(), nil); err != nil {
		t.Fatal(err)
	}
	if !r.has("stop") {
		t.Fatalf("Stop after the move was swallowed: %v", r.calls)
	}
}

func TestSafeArmNoMoveAfterCtrlC(t *testing.T) {
	r := &recArm{q: []float64{0, 0, 0, 0, 0}, release: make(chan struct{})}
	close(r.release)
	s := &safeArm{Arm: r}
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if err := s.MoveToJointPositions(ctx, []float64{0.1, 0, 0, 0, 0}, nil); err == nil {
		t.Fatal("expected an error")
	}
	if r.has("move") {
		t.Fatal("a move was launched after Ctrl-C")
	}
}

func TestSafeArmNoStopWhileSettling(t *testing.T) {
	r := &recArm{q: []float64{0, 0, 0, 0, 0}, release: make(chan struct{}), drift: 5}
	s := &safeArm{Arm: r}
	done := make(chan error, 1)
	go func() { done <- s.MoveToJointPositions(context.Background(), []float64{0.1, 0, 0, 0, 0}, nil) }()
	waitFor(t, func() bool { return r.has("move") })
	close(r.release)
	s.Stop(context.Background(), nil) // in the RPC or the settling window: must be swallowed
	time.Sleep(150 * time.Millisecond)
	s.Stop(context.Background(), nil)
	if err := <-done; err != nil {
		t.Fatal(err)
	}
	r.mu.Lock()
	left := r.drift
	r.mu.Unlock()
	if left != 0 {
		t.Fatalf("move returned before the joints settled (drift left %d)", left)
	}
	if r.has("stop") {
		t.Fatalf("Stop reached the arm while settling: %v", r.calls)
	}
}

func TestSafeArmTimeoutKeepsStopDisabled(t *testing.T) {
	old := stillTimeout
	stillTimeout = 300 * time.Millisecond
	defer func() { stillTimeout = old }()
	r := &recArm{q: []float64{0, 0, 0, 0, 0}, release: make(chan struct{}), drift: -1}
	close(r.release)
	s := &safeArm{Arm: r}
	if err := s.MoveToJointPositions(context.Background(), []float64{0.1, 0, 0, 0, 0}, nil); err == nil {
		t.Fatal("expected a timeout error")
	}
	if err := s.Stop(context.Background(), nil); err != nil {
		t.Fatal(err)
	}
	if r.has("stop") {
		t.Fatal("Stop reached a moving arm after a settle timeout")
	}
}

func TestSafeArmFloorGuardBlocksMove(t *testing.T) {
	r := &recArm{q: qStart, release: make(chan struct{})}
	close(r.release)
	s := &safeArm{Arm: r, guard: &floorGuard{model: so101(t), minZ: 30, tipMM: 105}}
	if err := s.MoveToJointPositions(context.Background(), qLow, nil); err == nil {
		t.Fatal("move into the floor was issued")
	}
	if len(r.calls) != 0 {
		t.Fatalf("arm saw %v", r.calls)
	}
}
