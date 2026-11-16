package main

import (
	"context"
	"errors"
	"fmt"
	"math"
	"sync"
	"time"

	"go.viam.com/rdk/components/arm"
)

// safeArm keeps arm.Stop away from profiled moves. On the SO-101 driver Stop writes goal
// speed 0, which means maximum speed, and doesn't interrupt a MoveToJointPositions
// (spec 3c1 "Hardware and machine"). So a profiled move:
//   - is floor-checked along its joint-space path first;
//   - runs on a context detached from Ctrl-C;
//   - counts as in flight until the arm is physically still, and Stop is a no-op meanwhile.
//
// Streaming paths (armIO) still reach Stop: they never overlap a profiled move.
//
// Only MoveToJointPositions is guarded. MoveThroughJointPositions and MoveToPosition pass
// straight through to the embedded arm; the CLI doesn't use them.
type safeArm struct {
	arm.Arm
	guard *floorGuard
	mu    sync.Mutex // guards moves; held by Stop across its check and the arm.Stop call
	moves int        // profiled moves not yet confirmed still; never decremented on a failed settle
}

const (
	stillTol  = 2e-3 // rad: one STS3215 count is 360/4095° = 1.534e-3 rad, so allow a one-count flicker
	stillGap  = 100 * time.Millisecond
	readLimit = 2 * time.Second
)

var stillTimeout = 30 * time.Second // var so tests can shorten it

func (s *safeArm) MoveToJointPositions(ctx context.Context, q []float64, extra map[string]interface{}) error {
	cur, err := s.Arm.JointPositions(ctx, nil)
	if err != nil {
		return err
	}
	if err := s.guard.checkPath(cur, q); err != nil {
		return err
	}
	s.mu.Lock()
	if err := ctx.Err(); err != nil { // Ctrl-C before the move starts: don't start it
		s.mu.Unlock()
		return err
	}
	s.moves++
	s.mu.Unlock()
	bg := context.WithoutCancel(ctx)
	done := make(chan error, 1)
	go func() { done <- s.Arm.MoveToJointPositions(bg, q, extra) }()
	var moveErr error
	interrupted := ctx.Done()
wait:
	for {
		select {
		case moveErr = <-done:
			break wait
		case <-interrupted:
			fmt.Println("Ctrl-C: a profiled move can only be stopped by cutting power; waiting for the arm to finish")
			interrupted = nil
		}
	}
	if err := s.waitStill(bg); err != nil {
		// still moving or unreadable: leave moves incremented so nothing calls Stop on a moving arm
		fmt.Println("arm.Stop is now disabled for the rest of this process; cut power if the arm needs stopping")
		return errors.Join(moveErr, err)
	}
	s.mu.Lock()
	s.moves--
	s.mu.Unlock()
	return errors.Join(moveErr, ctx.Err())
}

// waitStill returns once two joint reads stillGap apart differ by < stillTol. The reads are the
// physical test; IsMoving isn't used, because a servo holding against gravity can flicker it.
func (s *safeArm) waitStill(ctx context.Context) error {
	deadline := time.Now().Add(stillTimeout)
	for time.Now().Before(deadline) {
		q1, err := s.read(ctx)
		if err != nil {
			return err
		}
		time.Sleep(stillGap)
		q2, err := s.read(ctx)
		if err != nil {
			return err
		}
		d := 0.0
		for i := range q1 {
			d = math.Max(d, math.Abs(q2[i]-q1[i]))
		}
		if d < stillTol {
			return nil
		}
	}
	return fmt.Errorf("arm still moving %v after a profiled move returned", stillTimeout)
}

func (s *safeArm) read(ctx context.Context) ([]float64, error) {
	ctx, cancel := context.WithTimeout(ctx, readLimit)
	defer cancel()
	return s.Arm.JointPositions(ctx, nil)
}

func (s *safeArm) Stop(ctx context.Context, extra map[string]interface{}) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.moves > 0 {
		fmt.Println("not calling arm.Stop: a profiled move is in flight (on this driver Stop may speed it up)")
		return nil
	}
	return s.Arm.Stop(ctx, extra)
}
