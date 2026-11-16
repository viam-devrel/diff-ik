package main

import (
	"context"
	"fmt"
	"slices"
	"time"

	"diffik"

	"go.viam.com/rdk/components/arm"
)

type durStats struct {
	n        int
	p50, p99 time.Duration
}

func stats(d []time.Duration) durStats {
	if len(d) == 0 {
		return durStats{}
	}
	s := slices.Clone(d)
	slices.Sort(s)
	return durStats{n: len(s), p50: s[len(s)/2], p99: s[len(s)*99/100]}
}

type probeResult struct {
	reads, ticks durStats
	overruns     int
}

// probeRun times nReads back-to-back joint reads, then a ticker-paced read+send loop at rateHz
// for dur that holds a fixed q0 read once at the start (spec 3c1 "probe").
func probeRun(ctx context.Context, a arm.Arm, rateHz float64, lead, dur time.Duration, nReads int) (probeResult, error) {
	var res probeResult
	if !(rateHz > 0) {
		return res, fmt.Errorf("rate must be > 0, got %v", rateHz)
	}
	var reads []time.Duration
	for i := 0; i < nReads; i++ {
		t0 := time.Now()
		if _, err := a.JointPositions(ctx, nil); err != nil {
			return res, err
		}
		reads = append(reads, time.Since(t0))
	}
	res.reads = stats(reads)

	q0, err := a.JointPositions(ctx, nil)
	if err != nil {
		return res, err
	}
	io := diffik.NewArmIOLead(a, rateHz, lead)
	period := time.Duration(float64(time.Second) / rateHz)
	tk := time.NewTicker(period)
	defer tk.Stop()
	var ticks []time.Duration
	end := time.Now().Add(dur)
	var loopErr error
	for time.Now().Before(end) && loopErr == nil {
		select {
		case <-ctx.Done():
			loopErr = ctx.Err()
			continue
		case <-tk.C:
		}
		t0 := time.Now()
		if _, err := io.Read(ctx); err != nil {
			loopErr = err
			break
		}
		if err := io.Send(ctx, q0); err != nil {
			loopErr = err
			break
		}
		el := time.Since(t0)
		ticks = append(ticks, el)
		if el > period {
			res.overruns++
		}
		select { // drop a pending tick (late wake-up or overrun) so sends stay about one period apart
		case <-tk.C:
		default:
		}
	}
	stopCtx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	stopErr := io.Stop(stopCtx)
	res.ticks = stats(ticks)
	if loopErr != nil {
		return res, loopErr
	}
	return res, stopErr
}

func probe(ctx context.Context, a arm.Arm, rateHz float64, lead time.Duration) error {
	res, err := probeRun(ctx, a, rateHz, lead, 10*time.Second, 500)
	fmt.Printf("reads:    n %d  p50 %v  p99 %v\n", res.reads.n, res.reads.p50, res.reads.p99)
	fmt.Printf("read+send at %.0f Hz: n %d  p50 %v  p99 %v  overruns %d\n", rateHz, res.ticks.n, res.ticks.p50, res.ticks.p99, res.overruns)
	period := time.Duration(float64(time.Second) / rateHz)
	fmt.Printf("tick p99 within the %v period: %s\n", period, pass(err == nil && res.ticks.n > 0 && res.ticks.p99 <= period))
	fmt.Println("now read the driver's `streamed N points … late K (max M)` line from the machine log")
	return err
}
