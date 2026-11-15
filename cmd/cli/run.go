package main

import (
	"context"
	"encoding/csv"
	"errors"
	"fmt"
	"math"
	"math/rand"
	"os"
	"slices"
	"strconv"
	"sync"
	"time"

	"diffik"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"gonum.org/v1/gonum/num/quat"
)

// recorder keeps every tick's status (and optionally writes it as CSV). OnTick runs on the
// loop goroutine while scenarios read from another, hence the mutex.
type recorder struct {
	mu    sync.Mutex
	start time.Time
	rows  []diffik.Status
	times []time.Duration // elapsed since start, parallel to rows
	csv   *csv.Writer
}

func (r *recorder) add(s diffik.Status) {
	r.mu.Lock()
	defer r.mu.Unlock()
	el := time.Since(r.start)
	r.rows = append(r.rows, s)
	r.times = append(r.times, el)
	if r.csv == nil {
		return
	}
	f := func(v float64) string { return strconv.FormatFloat(v, 'f', 6, 64) }
	row := []string{f(el.Seconds()), string(s.State),
		f(s.TargetMM.X), f(s.TargetMM.Y), f(s.TargetMM.Z),
		f(s.ErrMM), f(s.ErrDeg), f(s.CmdErrMM), f(s.CmdErrDeg), f(s.Damping), strconv.FormatBool(s.Limited), f(s.LastTick.Seconds() * 1000)}
	for _, v := range s.QCmd {
		row = append(row, f(v))
	}
	for _, v := range s.QMeas {
		row = append(row, f(v))
	}
	_ = r.csv.Write(row)
}

func (r *recorder) since(i int) ([]diffik.Status, []time.Duration) {
	r.mu.Lock()
	defer r.mu.Unlock()
	return slices.Clone(r.rows[i:]), slices.Clone(r.times[i:])
}

func (r *recorder) len() int {
	r.mu.Lock()
	defer r.mu.Unlock()
	return len(r.rows)
}

func run(ctx context.Context, a arm.Arm, model referenceframe.Model, scenario string, rate float64, csvPath string, wrist int, maxJointDegS, resyncDeg float64) (err error) {
	// Any error return (including the singular pre-move) must not leave the arm moving.
	defer func() {
		if err != nil {
			stopCtx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
			defer cancel()
			_ = a.Stop(stopCtx, nil)
		}
	}()
	var body func(context.Context, *diffik.Loop, *recorder, spatialmath.Pose) error
	switch scenario {
	case "static":
		body = static
	case "singular":
		if body, err = singular(ctx, a, model, wrist); err != nil {
			return err
		}
	case "circle":
		body = circle
	default:
		return fmt.Errorf("unknown scenario %q", scenario)
	}

	cfg := diffik.LoopConfig{RateHz: rate, ResyncDeg: resyncDeg}
	if maxJointDegS > 0 {
		for range model.DoF() {
			cfg.MaxJointDegPerS = append(cfg.MaxJointDegPerS, maxJointDegS)
		}
	}
	l, err := diffik.NewLoop(model, diffik.NewArmIO(a, rate), cfg)
	if err != nil {
		return err
	}
	rec := &recorder{start: time.Now()}
	if csvPath != "" {
		f, ferr := os.Create(csvPath) // not :=err, the deferred flush below must set the named result
		if ferr != nil {
			return ferr
		}
		defer f.Close()
		rec.csv = csv.NewWriter(f)
		hdr := []string{"elapsed_s", "state", "target_x_mm", "target_y_mm", "target_z_mm", "err_mm", "err_deg",
			"cmd_err_mm", "cmd_err_deg", "damping", "limited", "tick_ms"}
		for _, pre := range []string{"q_cmd_", "q_meas_"} {
			for i := range model.DoF() {
				hdr = append(hdr, pre+strconv.Itoa(i))
			}
		}
		_ = rec.csv.Write(hdr)
		defer func() {
			rec.csv.Flush()
			if cerr := rec.csv.Error(); cerr != nil {
				err = errors.Join(err, cerr)
			}
		}()
	}
	l.OnTick = rec.add

	q, err := a.JointPositions(ctx, nil)
	if err != nil {
		return err
	}
	p, o, err := diffik.FK(model, q)
	if err != nil {
		return err
	}
	start := poseMM(p, o)

	runCtx, cancel := context.WithCancel(ctx)
	defer cancel()
	loopErr := make(chan error, 1)
	go func() { loopErr <- l.Run(runCtx); cancel() }() // a loop fault ends the scenario too

	scenarioErr := body(runCtx, l, rec, start)
	cancel()
	if err := <-loopErr; err != nil && err != context.Canceled {
		// direct comparison: Loop.stop joins ctx.Err() with a failed arm stop, which errors.Is would swallow
		return errors.Join(scenarioErr, err)
	}

	s := l.Status()
	// 1% tolerance: a 50 Hz ticker's measured average straddles 50 from first/last tick jitter.
	const minRate = 49.5
	fmt.Printf("rate:     %.2f Hz (pass ≥ %.1f) %s\n", s.RateHz, minRate, pass(s.RateHz >= minRate))
	fmt.Printf("tick:     p50 %v  p99 %v\n", s.TickP50, s.TickP99)
	fmt.Printf("overruns: %d   resyncs: %d\n", s.Overruns, s.Resyncs)
	if !(s.RateHz >= minRate) { // NaN-safe
		scenarioErr = errors.Join(scenarioErr, fmt.Errorf("loop rate %.2f Hz below %.1f", s.RateHz, minRate))
	}
	return scenarioErr
}

// waitConverged blocks until the loop has converged on target, or timeout passes.
func waitConverged(ctx context.Context, l *diffik.Loop, target spatialmath.Pose, timeout time.Duration) (time.Duration, bool) {
	t0 := time.Now()
	for time.Since(t0) < timeout && ctx.Err() == nil {
		s := l.Status()
		if s.State == diffik.StateConverged && s.TargetMM == target.Point() {
			return time.Since(t0), true
		}
		time.Sleep(10 * time.Millisecond)
	}
	return time.Since(t0), false
}

// static: 10 targets 50 mm and ≤ 10° from start, returning to start between them.
func static(ctx context.Context, l *diffik.Loop, _ *recorder, start spatialmath.Pose) error {
	rng := rand.New(rand.NewSource(1))
	failed := 0
	for i := 0; i < 10; i++ {
		if ctx.Err() != nil {
			return ctx.Err()
		}
		dir := randUnit(rng)
		rot := axisAngle(randUnit(rng), rng.Float64()*10*math.Pi/180)
		target := spatialmath.NewPose(start.Point().Add(dir.Mul(50)), quatOrient(quat.Mul(rot, start.Orientation().Quaternion())))
		l.SetTarget(target)
		took, ok := waitConverged(ctx, l, target, 10*time.Second)
		s := l.Status()
		fmt.Printf("target %2d: %-9s in %5.2fs  err %.2f mm %.2f°  %s\n", i, s.State, took.Seconds(), s.ErrMM, s.ErrDeg, pass(ok))
		if !ok {
			failed++
		}
		l.SetTarget(start)
		waitConverged(ctx, l, start, 10*time.Second)
	}
	if failed > 0 {
		return fmt.Errorf("%d of 10 targets did not converge", failed)
	}
	return nil
}

// singular moves the arm so the wrist joint sits at +0.2 rad, then (once the loop runs) targets the
// pose with the wrist at −0.2 rad, so the Cartesian path passes close to the wrist singularity.
func singular(ctx context.Context, a arm.Arm, model referenceframe.Model, wrist int) (func(context.Context, *diffik.Loop, *recorder, spatialmath.Pose) error, error) {
	q, err := a.JointPositions(ctx, nil)
	if err != nil {
		return nil, err
	}
	if wrist < 0 || wrist >= len(q) {
		return nil, fmt.Errorf("wrist index %d out of range for %d joints", wrist, len(q))
	}
	qa, qb := slices.Clone(q), slices.Clone(q)
	qa[wrist], qb[wrist] = 0.2, -0.2
	if err := a.MoveToJointPositions(ctx, qa, nil); err != nil {
		return nil, err
	}
	p, o, err := diffik.FK(model, qb)
	if err != nil {
		return nil, err
	}
	target := poseMM(p, o)

	return func(ctx context.Context, l *diffik.Loop, rec *recorder, _ spatialmath.Pose) error {
		from := rec.len()
		l.SetTarget(target)
		_, reached := waitConverged(ctx, l, target, 10*time.Second)
		rows, times := rec.since(from)

		cfg := l.Config()
		dt := 1 / cfg.RateHz
		qs := [][]float64{qa}
		ts := []time.Duration{0}
		for k, r := range rows {
			if r.QMeas != nil { // measured, not commanded: the loop already clamps QCmd steps
				qs = append(qs, r.QMeas)
				ts = append(ts, times[k])
			}
		}
		minStep := make([]float64, len(qa))
		peakOK := true
		for i := range minStep {
			lim := cfg.MaxJointDegPerS[i]
			minStep[i] = 0.1 * lim * math.Pi / 180 * dt
			peak := 0.0
			// Velocity over a 10-tick window: rows are stamped at tick start but q is read
			// mid-tick, so after an overrun two adjacent stamps can be ~2 ms apart while the
			// reads are ~20 ms apart. qs[0] is the pre-move pose, not timed on the same clock.
			const win = 10
			for k := 1 + win; k < len(qs); k++ {
				if d := (ts[k] - ts[k-win]).Seconds(); d > 0 {
					peak = math.Max(peak, math.Abs(qs[k][i]-qs[k-win][i])/d*180/math.Pi)
				}
			}
			// 10% tolerance for PD servo catch-up
			fmt.Printf("joint %d: peak %.1f°/s (limit %.0f, pass ≤ %.0f)\n", i, peak, lim, lim*1.1)
			peakOK = peakOK && peak <= lim*1.1
		}
		seconds := float64(len(qs)-1) * dt
		oscOK := true
		for i, n := range diffik.Oscillations(qs, minStep) {
			fmt.Printf("joint %d: %d oscillations in %.1fs\n", i, n, seconds)
			oscOK = oscOK && float64(n) <= 2*seconds
		}
		fmt.Printf("reached target: %s   velocity limits: %s   oscillation: %s\n", pass(reached), pass(peakOK), pass(oscOK))
		if !reached || !peakOK || !oscOK {
			return errors.New("singular scenario failed")
		}
		return nil
	}, nil
}

// circle: converge to a point 50 mm from start, then trace a 50 mm-radius circle at 50 mm/s for 2 laps.
func circle(ctx context.Context, l *diffik.Loop, rec *recorder, start spatialmath.Pose) error {
	const radius, speed = 50.0, 50.0 // mm, mm/s
	omega := speed / radius
	at := func(theta float64) spatialmath.Pose {
		return spatialmath.NewPose(start.Point().Add(r3.Vector{X: radius * math.Cos(theta), Y: radius * math.Sin(theta)}), start.Orientation())
	}
	first := at(0)
	l.SetTarget(first)
	if _, ok := waitConverged(ctx, l, first, 10*time.Second); !ok {
		return errors.New("did not reach the circle's start point")
	}

	from := rec.len()
	period := time.Duration(float64(time.Second) / l.Config().RateHz)
	t := time.NewTicker(period)
	defer t.Stop()
	t0 := time.Now()
	for el := 0.0; el < 4*math.Pi/omega; el = time.Since(t0).Seconds() {
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-t.C:
		}
		l.SetTarget(at(omega * el))
	}

	var sum, worst float64
	rows, _ := rec.since(from)
	for _, r := range rows {
		sum += r.ErrMM
		worst = math.Max(worst, r.ErrMM)
	}
	if len(rows) == 0 {
		return errors.New("no ticks recorded during circle")
	}
	mean := sum / float64(len(rows))
	fmt.Printf("tracking error: mean %.2f mm  max %.2f mm\n", mean, worst)
	fmt.Printf("lag (mean error / speed): %.0f ms\n", mean/speed*1000)
	return nil
}

func poseMM(p r3.Vector, o quat.Number) spatialmath.Pose {
	return spatialmath.NewPose(p.Mul(1000), quatOrient(o))
}

func quatOrient(q quat.Number) spatialmath.Orientation {
	sq := spatialmath.Quaternion(q)
	return &sq
}

func axisAngle(axis r3.Vector, angle float64) quat.Number {
	s := math.Sin(angle / 2)
	return quat.Number{Real: math.Cos(angle / 2), Imag: axis.X * s, Jmag: axis.Y * s, Kmag: axis.Z * s}
}

func randUnit(rng *rand.Rand) r3.Vector {
	return r3.Vector{X: rng.NormFloat64(), Y: rng.NormFloat64(), Z: rng.NormFloat64()}.Normalize()
}
