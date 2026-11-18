package diffik

import (
	"context"
	"slices"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/services/vision"
)

// liveSession starts a QP session with live obstacles on the framed UR5e. The poller polls at rate Hz (1: it
// never fires within a test), so tests publish by hand through s.poller.publish. The dead-man is an hour.
func liveSession(t *testing.T, staleMS float64, ramp time.Duration) (*session, *simIO, *fakeClock, *fakeSrc) {
	t.Helper()
	return liveSessionRate(t, 1, staleMS, ramp)
}

func liveSessionRate(t *testing.T, rate, staleMS float64, ramp time.Duration) (*session, *simIO, *fakeClock, *fakeSrc) {
	t.Helper()
	m, sim, clk := newFramedTestManager(t, camPose)
	src := &fakeSrc{props: vision.Properties{Detections3DSupported: true}}
	cfg := testSessionCfg(ModePose)
	cfg.Loop.Solver = SolverQP
	cfg.Timeout, cfg.Ramp = time.Hour, ramp
	cfg.Live = &LiveConfig{Vision: "v", Camera: "cam", RateHz: rate, StaleMS: staleMS, TimeoutMS: 100}
	cfg.LiveSource = src
	cfg.LiveJoints = &fakeJoints{q: slices.Clone(home)}
	s, _ := mustStart(t, m, cfg)
	t.Cleanup(s.cancel)
	return s, sim, clk, src
}

func awayTarget(t *testing.T, s *session) {
	t.Helper()
	s.loop.SetTarget(offsetTarget(t, ur5e(t), home, r3.Vector{X: 50}, 0))
}

func farBox(t *testing.T) []liveObstacle {
	return []liveObstacle{liveBoxAt(t, r3.Vector{X: 5000, Y: 5000, Z: 5000}, r3.Vector{X: 100, Y: 100, Z: 100})}
}

// Without the hold this fails: the arm would head for the target at once.
func TestLiveWaitingHoldsThenMoves(t *testing.T) {
	s, sim, clk, _ := liveSession(t, 1e6, 200*time.Millisecond)
	awayTarget(t, s)
	before := loopTarget(s)
	step(t, s, clk, 50)
	for i, q := range sim.sends {
		if maxAbsDiff(q, home) > 1e-12 {
			t.Fatalf("send %d moved while waiting: %v", i, q)
		}
	}
	if !loopTarget(s).Point().ApproxEqual(before.Point()) {
		t.Fatal("target advanced while waiting")
	}
	if st := s.status(); !st.LiveWaiting || st.LiveStale || s.ended() {
		t.Fatalf("waiting = %v stale = %v ended = %v", st.LiveWaiting, st.LiveStale, s.ended())
	}
	s.poller.publish(farBox(t))
	step(t, s, clk, 25)
	if d := maxAbsDiff(sim.sends[len(sim.sends)-1], home); d < 1e-3 {
		t.Fatalf("arm did not move after the first set (%.2g rad)", d)
	}
	if st := s.status(); st.LiveWaiting || st.LiveCount != 1 {
		t.Fatalf("waiting = %v count = %d, want false, 1", st.LiveWaiting, st.LiveCount)
	}
}

func TestLiveStaleStops(t *testing.T) {
	s, sim, clk, _ := liveSession(t, 1500, 200*time.Millisecond)
	step(t, s, clk, 100) // 2 s, no publish
	if !s.ended() || s.status().State != StateStoppedStale || !sim.stopped {
		t.Fatalf("ended = %v state = %s stopped = %v, want stopped_stale", s.ended(), s.status().State, sim.stopped)
	}
}

func TestLiveFreshNeverStale(t *testing.T) {
	s, _, clk, _ := liveSession(t, 1500, 200*time.Millisecond)
	for i := 0; i < 200; i++ { // 4 s, a set every 100 ms
		if i%5 == 0 {
			s.poller.publish(farBox(t))
		}
		step(t, s, clk, 1)
	}
	if st := s.status(); s.ended() || st.LiveStale || st.LiveAgeMS > 120 {
		t.Fatalf("ended = %v stale = %v age = %.0f ms", s.ended(), st.LiveStale, st.LiveAgeMS)
	}
}

func TestLiveOneMissedPublishNotStale(t *testing.T) {
	s, _, clk, _ := liveSession(t, 1500, 200*time.Millisecond)
	s.poller.publish(farBox(t))
	step(t, s, clk, 50) // 1 s: one missed 500 ms slot, still inside 1.5 s
	s.poller.publish(farBox(t))
	step(t, s, clk, 50)
	if s.ended() || s.status().LiveStale {
		t.Fatal("went stale after one missed publish")
	}
}

func TestLiveStaleFinalStates(t *testing.T) {
	t.Run("dead-man during the stale ramp leaves stopped_stale", func(t *testing.T) {
		s, _, clk, _ := liveSession(t, 1500, 3*time.Second)
		s.timeout = 2 * time.Second // fires mid-ramp: stale at 1.5 s, dead-man at 2 s, ramp ends at 4.5 s
		step(t, s, clk, 300)
		if !s.ended() || s.status().State != StateStoppedStale {
			t.Fatalf("ended = %v state = %s, want stopped_stale", s.ended(), s.status().State)
		}
	})
	t.Run("servo_stop during it gives stopped", func(t *testing.T) {
		s, _, clk, _ := liveSession(t, 1500, 3*time.Second)
		step(t, s, clk, 90) // past stale, mid-ramp
		if s.ended() {
			t.Fatal("ended too early")
		}
		s.beginStop(StateStopped)
		step(t, s, clk, 300)
		if !s.ended() || s.status().State != StateStopped {
			t.Fatalf("ended = %v state = %s, want stopped", s.ended(), s.status().State)
		}
	})
}

// A goroutine publishing at 100 Hz while ticks run: meant for -race.
func TestLiveHandOffRace(t *testing.T) {
	s, _, clk, _ := liveSession(t, 1e6, 200*time.Millisecond)
	stop, done := make(chan struct{}), make(chan struct{})
	box := farBox(t)
	go func() {
		defer close(done)
		for {
			select {
			case <-stop:
				return
			case <-time.After(10 * time.Millisecond):
				s.poller.publish(slices.Clone(box))
			}
		}
	}()
	for i := 0; i < 20; i++ {
		step(t, s, clk, 10)
		time.Sleep(10 * time.Millisecond)
	}
	close(stop)
	<-done
	if st := s.status(); st.LiveWaiting || st.LiveCount != 1 {
		t.Fatalf("waiting = %v count = %d, want false, 1", st.LiveWaiting, st.LiveCount)
	}
}

func TestLiveSessionStopsPoller(t *testing.T) {
	s, _, clk, src := liveSessionRate(t, 50, 1500, 200*time.Millisecond)
	for deadline := time.Now().Add(5 * time.Second); src.calls.Load() < 3; time.Sleep(5 * time.Millisecond) {
		if time.Now().After(deadline) {
			t.Fatal("poller never polled")
		}
	}
	if st := s.status(); st.LiveUpdates == 0 || st.LiveMethod != "detections3d" {
		t.Fatalf("updates = %d method = %q", st.LiveUpdates, st.LiveMethod)
	}
	s.beginStop(StateStopped)
	step(t, s, clk, 50)
	if !s.ended() {
		t.Fatal("session did not end")
	}
	time.Sleep(50 * time.Millisecond) // a poll already in flight finishes
	n := src.calls.Load()
	time.Sleep(150 * time.Millisecond)
	if src.calls.Load() != n || src.inflight.Load() != 0 {
		t.Fatalf("poller still running: calls %d -> %d, inflight %d", n, src.calls.Load(), src.inflight.Load())
	}
}

type hangSrc struct{ fakeSrc }

func (h *hangSrc) GetProperties(ctx context.Context, _ map[string]interface{}) (*vision.Properties, error) {
	<-ctx.Done()
	return nil, ctx.Err()
}

func TestLiveStartTimesOutOnHungProperties(t *testing.T) {
	m, _, _ := newFramedTestManager(t, camPose)
	cfg := testSessionCfg(ModePose)
	cfg.Loop.Solver = SolverQP
	cfg.Live = &LiveConfig{Vision: "v", Camera: "cam", RateHz: 1, StaleMS: 1500, TimeoutMS: 100}
	cfg.LiveSource = &hangSrc{}
	cfg.LiveJoints = &fakeJoints{q: slices.Clone(home)}
	t0 := time.Now()
	_, _, err := m.start(context.Background(), "arm", cfg)
	if err == nil || time.Since(t0) > time.Second {
		t.Fatalf("err = %v after %v, want an error in under 1 s", err, time.Since(t0))
	}
}

func TestLiveBackingAwayReachesStatus(t *testing.T) {
	s, _, clk, _ := liveSession(t, 1e6, 200*time.Millisecond)
	p, _, err := FK(ur5e(t), home)
	if err != nil {
		t.Fatal(err)
	}
	s.poller.publish([]liveObstacle{liveBoxAt(t, p.Mul(mmPerM), r3.Vector{X: 60, Y: 60, Z: 60})})
	step(t, s, clk, 2)
	if st := s.status(); !st.LiveBackingAway || st.LiveCount != 1 {
		t.Fatalf("backing away = %v count = %d, want true, 1", st.LiveBackingAway, st.LiveCount)
	}
}
