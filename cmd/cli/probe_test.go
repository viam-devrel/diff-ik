package main

import (
	"context"
	"errors"
	"slices"
	"sync"
	"testing"
	"time"

	"go.viam.com/rdk/components/arm"
)

// pacedArm streams like the so-101 driver: it waits until each point is due (stream start +
// point time), then "writes" it. Its joints sag a little on every read, like an
// uncompensated arm, so re-sending readings would creep.
type pacedArm struct {
	arm.Arm
	mu      sync.Mutex
	reads   int
	written [][]float64
	at      []time.Time
	stops   int
	ended   bool
}

func (p *pacedArm) JointPositions(context.Context, map[string]interface{}) ([]float64, error) {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.reads++
	return []float64{0, -0.61 + 1e-3*float64(p.reads), 0.46, 1.39, 0}, nil
}

func (p *pacedArm) MoveThroughJointPositionsStreamed(ctx context.Context, b <-chan []arm.TrajectoryPoint, r chan<- arm.Response, _ map[string]interface{}) error {
	defer func() { p.mu.Lock(); p.ended = true; p.mu.Unlock() }()
	start := time.Now()
	for batch := range b {
		for _, pt := range batch {
			time.Sleep(time.Until(start.Add(pt.Time)))
			p.mu.Lock()
			p.written = append(p.written, slices.Clone(pt.Positions))
			p.at = append(p.at, time.Now())
			p.mu.Unlock()
		}
	}
	return nil
}

func (p *pacedArm) Stop(context.Context, map[string]interface{}) error {
	p.mu.Lock()
	p.stops++
	p.mu.Unlock()
	return nil
}

func TestProbeHoldsFixedQ0AndPaces(t *testing.T) {
	p := &pacedArm{}
	res, err := probeRun(context.Background(), p, 50, 500*time.Millisecond, 20)
	if err != nil {
		t.Fatal(err)
	}
	if res.reads.n != 20 || res.ticks.n < 20 {
		t.Fatalf("reads %d ticks %d", res.reads.n, res.ticks.n)
	}
	p.mu.Lock()
	defer p.mu.Unlock()
	q0 := p.written[1] // written[0] is the stream's t=0 point at the current joints
	for i, q := range p.written[1:] {
		if !slices.Equal(q, q0) {
			t.Fatalf("point %d = %v, want the fixed q0 %v", i+1, q, q0)
		}
	}
	for i := 2; i < len(p.at); i++ {
		if gap := p.at[i].Sub(p.at[i-1]); gap < 5*time.Millisecond {
			t.Fatalf("points %d and %d written %v apart: sends are back to back", i-1, i, gap)
		}
	}
}

func TestProbeRejectsBadRate(t *testing.T) {
	if _, err := probeRun(context.Background(), &pacedArm{}, 0, time.Second, 1); err == nil {
		t.Fatal("rate 0 accepted")
	}
}

func TestProbeCancelStopsOnce(t *testing.T) {
	p := &pacedArm{}
	ctx, cancel := context.WithCancel(context.Background())
	time.AfterFunc(100*time.Millisecond, cancel)
	t0 := time.Now()
	_, err := probeRun(ctx, p, 50, 2*time.Second, 5)
	if !errors.Is(err, context.Canceled) {
		t.Fatalf("err = %v, want context.Canceled", err)
	}
	if d := time.Since(t0); d > time.Second {
		t.Fatalf("took %v after cancel", d)
	}
	p.mu.Lock()
	defer p.mu.Unlock()
	if p.stops != 1 || !p.ended {
		t.Fatalf("stops %d ended %v, want 1 and true", p.stops, p.ended)
	}
}
