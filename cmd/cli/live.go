package main

import (
	"context"
	"fmt"
	"strconv"
	"strings"
	"time"

	"diffik"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
)

// liveStaleRamp is how long the CLI ramps to a stop when live detections go stale.
const liveStaleRamp = 300 * time.Millisecond

// armJoints reads the arm's measured joints for the poller.
type armJoints struct{ a arm.Arm }

func (j armJoints) Joints(ctx context.Context) ([]float64, error) {
	return j.a.JointPositions(ctx, nil)
}

// parseWorkspace parses "x0,y0,z0,x1,y1,z1" (world mm).
func parseWorkspace(s string) (lo, hi *r3.Vector, err error) {
	f := strings.Split(s, ",")
	if len(f) != 6 {
		return nil, nil, fmt.Errorf("--live-workspace wants x0,y0,z0,x1,y1,z1, got %d values", len(f))
	}
	v := make([]float64, 6)
	for i, x := range f {
		if v[i], err = strconv.ParseFloat(strings.TrimSpace(x), 64); err != nil {
			return nil, nil, fmt.Errorf("--live-workspace value %d: %w", i, err)
		}
	}
	return &r3.Vector{X: v[0], Y: v[1], Z: v[2]}, &r3.Vector{X: v[3], Y: v[4], Z: v[5]}, nil // LiveConfig.Validate checks order and finiteness
}

type rampStopper interface {
	RampStop(time.Duration, diffik.State)
}

// staleStopper is the CLI's OnTick hook: a bare Loop has no session to stop it, so the first tick with
// Status.LiveStale ramps the loop to stopped_stale. next, if set, sees every status first.
type staleStopper struct {
	l     rampStopper
	next  func(diffik.Status)
	ramp  time.Duration
	fired bool
}

func (s *staleStopper) onTick(st diffik.Status) {
	if s.next != nil {
		s.next(st)
	}
	if st.LiveStale && !s.fired {
		s.fired = true
		s.l.RampStop(s.ramp, diffik.StateStoppedStale)
	}
}

// liveSummary is the run's live line(s), from the recorded rows and the poller's stats.
func liveSummary(st diffik.LiveStats, rows []diffik.Status, stale bool) string {
	waiting, pinched := 0, 0
	for _, r := range rows {
		if r.LiveWaiting {
			waiting++
		}
		pinched = max(pinched, r.LivePinched)
	}
	out := fmt.Sprintf("live: updates %d errors %d self %d skipped %d overflow %d clipped %d pinched %d waiting-ticks %d",
		st.Updates, st.Errors, st.Self, st.Skipped, st.Overflow, st.Clipped, pinched, waiting)
	if st.LastError != "" {
		out += "\nlast error: " + st.LastError
	}
	if stale {
		out += "\nstopped_stale"
	}
	return out
}
