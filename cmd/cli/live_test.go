package main

import (
	"strings"
	"testing"
	"time"

	"diffik"
)

type fakeStop struct {
	n     int
	final diffik.State
}

func (f *fakeStop) RampStop(_ time.Duration, s diffik.State) { f.n++; f.final = s }

func TestStaleStopperFiresOnce(t *testing.T) {
	f := &fakeStop{}
	seen := 0
	s := &staleStopper{l: f, next: func(diffik.Status) { seen++ }, ramp: time.Second}
	s.onTick(diffik.Status{})
	if f.n != 0 {
		t.Fatal("stopped before stale")
	}
	s.onTick(diffik.Status{LiveStale: true})
	s.onTick(diffik.Status{LiveStale: true})
	if f.n != 1 || f.final != diffik.StateStoppedStale || seen != 3 || !s.fired {
		t.Fatalf("stops %d final %q seen %d", f.n, f.final, seen)
	}
}

func TestParseWorkspace(t *testing.T) {
	lo, hi, err := parseWorkspace("-1, 2,3.5,4,5,6")
	if err != nil || lo.X != -1 || lo.Y != 2 || lo.Z != 3.5 || hi.X != 4 || hi.Z != 6 {
		t.Fatalf("%v %v %v", lo, hi, err)
	}
	for _, bad := range []string{"1,2,3", "1,2,3,4,5,x", ""} {
		if _, _, err := parseWorkspace(bad); err == nil {
			t.Errorf("%q accepted", bad)
		}
	}
}

func TestLiveSummary(t *testing.T) {
	rows := []diffik.Status{{LiveWaiting: true}, {LiveWaiting: true, LivePinched: 1}, {LivePinched: 3}}
	got := liveSummary(diffik.LiveStats{Updates: 5, LastError: "boom"}, rows, true)
	want := "live: updates 5 errors 0 self 0 skipped 0 overflow 0 clipped 0 pinched 3 waiting-ticks 2\nlast error: boom\nstopped_stale"
	if got != want || strings.Count(got, "\n") != 2 {
		t.Fatalf("%q", got)
	}
}
