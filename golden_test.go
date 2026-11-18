package diffik

import (
	"encoding/json"
	"errors"
	"flag"
	"math"
	"os"
	"runtime"
	"slices"
	"testing"

	"github.com/golang/geo/r3"
)

var updateGolden = flag.Bool("update-golden", false, "rewrite testdata/golden_ur5e_<GOARCH>.json from the current code")

// goldenScenarios runs the Phase 1/2 UR5e pose-mode scenarios and returns every joint command sent,
// as raw float64 bits so the comparison is exact.
func goldenScenarios(t *testing.T) map[string][][]uint64 {
	t.Helper()
	bits := func(sends [][]float64) [][]uint64 {
		out := make([][]uint64, len(sends))
		for i, q := range sends {
			for _, v := range q {
				out[i] = append(out[i], math.Float64bits(v))
			}
		}
		return out
	}
	res := map[string][][]uint64{}

	l, sim, m := newTestLoop(t, home, noAccel)
	l.SetTarget(offsetTarget(t, m, home, r3.Vector{X: 30, Y: 30, Z: -25}, deg2rad(10)))
	runTicks(t, l, 150)
	res["converge"] = bits(sim.sends)

	q0 := slices.Clone(home)
	q0[4] = 0
	l, sim, m = newTestLoop(t, q0, noAccel)
	l.SetTarget(offsetTarget(t, m, q0, r3.Vector{X: 30, Y: 30, Z: -25}, deg2rad(10)))
	runTicks(t, l, 150)
	res["singular"] = bits(sim.sends)

	l, sim, m, clk := newClockLoop(t, home, noAccel)
	l.SetTarget(offsetTarget(t, m, home, r3.Vector{}, 0))
	l.SetVelocity([6]float64{0.05, 0, 0, 0, 0, deg2rad(10)})
	clockTicks(t, l, clk, 50)
	res["twist"] = bits(sim.sends)

	sm := so101(t)
	qs := []float64{0.2, -0.4, 0.6, 0.9, 0.1}
	ssim := &simIO{q: slices.Clone(qs)}
	sl, err := NewLoop(sm, ssim, LoopConfig{Solver: SolverQP, MaxJointDegPerS2: repeat(1e9, 5)})
	if err != nil {
		t.Fatal(err)
	}
	sl.SetTarget(offsetTarget(t, sm, qs, r3.Vector{X: 20, Y: -15, Z: 10}, 0))
	runTicks(t, sl, 150)
	res["qp_so101_static"] = bits(ssim.sends)
	return res
}

// UR5e pose mode with all weights 1 must keep today's code path exactly (spec "Compatibility").
func TestGoldenUR5ePoseCommands(t *testing.T) {
	got := goldenScenarios(t)
	// Bit-exact goldens are per-architecture: arm64 fuses x*y+z into FMA instructions, amd64 does not,
	// so the same code yields different last-bit results.
	path := "testdata/golden_ur5e_" + runtime.GOARCH + ".json"
	if *updateGolden {
		b, err := json.Marshal(got)
		if err != nil {
			t.Fatal(err)
		}
		if err := os.WriteFile(path, b, 0o644); err != nil {
			t.Fatal(err)
		}
		return
	}
	b, err := os.ReadFile(path)
	if errors.Is(err, os.ErrNotExist) {
		t.Skipf("no golden for %s; record with -update-golden on the pre-Phase-3a code (FMA fusion differs across GOARCH)", runtime.GOARCH)
	}
	if err != nil {
		t.Fatal(err)
	}
	var want map[string][][]uint64
	if err := json.Unmarshal(b, &want); err != nil {
		t.Fatal(err)
	}
	if len(got) != len(want) {
		t.Fatalf("%d scenarios, golden has %d", len(got), len(want))
	}
	for name := range got {
		if _, ok := want[name]; !ok {
			t.Fatalf("scenario %q missing from golden", name)
		}
	}
	for name, w := range want {
		g := got[name]
		if len(g) != len(w) {
			t.Fatalf("%s: %d sends, golden has %d", name, len(g), len(w))
		}
		for i := range w {
			if !slices.Equal(g[i], w[i]) {
				t.Fatalf("%s: send %d differs from golden (bit-exact)", name, i)
			}
		}
	}
}
