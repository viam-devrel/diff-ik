package diffik

import (
	"math"
	"math/rand"
	"testing"
)

// spec "Each tick" 5: v_max is brakeCap's rule with c = A_d. An approach obeying it, slowed by A_d per
// tick, stops within the gap, and the next tick's v_max is at least the approach minus A_d.
func TestCollisionVmaxStops(t *testing.T) {
	r := rand.New(rand.NewSource(4))
	for k := 0; k < 100000; k++ {
		Ad := []float64{0.01, 0.3, 2, 9}[k%4]
		g := r.Float64() * 80
		v := r.Float64() * collisionVmax(Ad, g)
		if k%3 == 0 {
			v = collisionVmax(Ad, g)
		}
		if next := collisionVmax(Ad, g-v); next < v-Ad-1e-12 {
			t.Fatalf("Ad %v g %v v %v: next v_max %v below v−Ad", Ad, g, v, next)
		}
		travel := 0.0
		for s := v; s > 0; s -= Ad {
			travel += s
		}
		if travel > g+1e-9 {
			t.Fatalf("Ad %v g %v v %v: travels %v", Ad, g, v, travel)
		}
	}
	if collisionVmax(1, 0) != 0 || collisionVmax(1, -3) != 0 {
		t.Fatal("no gap: v_max must be 0")
	}
}

// spec "Failures": the relaxed bound is the strongest move away the box allows.
func TestRelaxBound(t *testing.T) {
	jd := []float64{2, -1, 0}
	lb, ub := []float64{-1, -1, -1}, []float64{0.5, 3, 1}
	if got := relaxBound(jd, lb, ub); math.Abs(got-(2*0.5+1)) > 1e-15 { // j0 at ub, j1 at lb
		t.Fatalf("relaxBound = %v, want 2", got)
	}
}

// spec "Failures": the brake moves lastStep toward zero by at most a·dt², clipped to position.
func TestBrakeStep(t *testing.T) {
	dt := 0.02
	a := []float64{deg2rad(60), deg2rad(60)}
	c := a[0] * dt * dt
	q, lo, hi := []float64{0, 1}, []float64{-3, -3}, []float64{3, 1.0001}
	got := brakeStep([]float64{10 * c, 0.5 * c}, a, dt, q, lo, hi)
	if math.Abs(got[0]-9*c) > 1e-15 {
		t.Fatalf("joint 0 step %v, want %v", got[0], 9*c)
	}
	if got[1] != 0 { // 0.5c brakes to rest in one tick
		t.Fatalf("joint 1 step %v, want 0", got[1])
	}
	got = brakeStep([]float64{0, 10 * c}, a, dt, q, lo, hi)
	if got[1] > hi[1]-q[1]+1e-15 { // clipped to the 0.0001 rad left
		t.Fatalf("joint 1 step %v passes hi", got[1])
	}
}
