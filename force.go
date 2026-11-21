package diffik

import (
	"errors"
	"fmt"
	"math"

	"github.com/golang/geo/r3"
	"gonum.org/v1/gonum/num/quat"
)

// ForceConfig is servo_start's force block: regulate the pressing force along the tool axis (twist mode).
// Until contact the tool moves along its axis at ApproachMMPerS; from contact the axis velocity is
// Gain·e + Ki·∫e dt with e = TargetN − press, clamped to ±MaxMMPerS. The integral removes the steady error
// of following a surface that moves along the axis (a tilted face under a sideways sweep). The client's twist still drives the other directions.
type ForceConfig struct {
	Sensor         string
	TargetN        float64 // required, > 0
	Gain           float64 // mm/s per N; default 0.5
	Ki             float64 // mm/s per N·s; 0 = off (the default)
	MaxMMPerS      float64 // default 5
	ApproachMMPerS float64 // default 5
	ContactN       float64 // pressing force that counts as contact; default min(2, TargetN/2)
	RateHz         float64 // sensor poll rate; default 500
	PressAxis      string  // "-z" (default; mujoco-sim) or "+z": the sensor z sign a surface pushing back reads
	ComplyXY       float64 // mm/s per N: yield sideways (tool x/y) to contact forces; 0 = off (the default)
}

func (c ForceConfig) withDefaults() ForceConfig {
	def := func(v *float64, d float64) {
		if *v == 0 {
			*v = d
		}
	}
	def(&c.Gain, 0.5)
	def(&c.MaxMMPerS, 5)
	def(&c.ApproachMMPerS, 5)
	def(&c.ContactN, math.Min(2, c.TargetN/2))
	def(&c.RateHz, defaultGuardHz)
	if c.PressAxis == "" {
		c.PressAxis = "-z"
	}
	return c
}

func (c ForceConfig) validate() error {
	switch {
	case c.Sensor == "":
		return errors.New("force.sensor is required")
	case !(c.TargetN > 0):
		return fmt.Errorf("force.target_n must be > 0, got %v", c.TargetN)
	case !(c.Gain > 0):
		return fmt.Errorf("force.gain must be > 0, got %v", c.Gain)
	case !(c.Ki >= 0):
		return fmt.Errorf("force.ki must be >= 0, got %v", c.Ki)
	case !(c.MaxMMPerS > 0), !(c.ApproachMMPerS > 0):
		return errors.New("force.max_mm_s and force.approach_mm_s must be > 0")
	case !(c.ContactN > 0 && c.ContactN < c.TargetN):
		return fmt.Errorf("force.contact_n must be in (0, target_n), got %v", c.ContactN)
	case !(c.RateHz > 0 && c.RateHz <= 2000):
		return fmt.Errorf("force.rate_hz must be in (0, 2000], got %v", c.RateHz)
	case c.PressAxis != "-z" && c.PressAxis != "+z":
		return fmt.Errorf(`force.press_axis must be "-z" or "+z", got %q`, c.PressAxis)
	case !(c.ComplyXY >= 0):
		return fmt.Errorf("force.comply_xy must be >= 0, got %v", c.ComplyXY)
	}
	return nil
}

// forceCtl is the loop's force controller. All fields but read are loop-goroutine only.
type forceCtl struct {
	cfg     ForceConfig
	read    func() r3.Vector // latest tared force F − F₀, sensor frame, N
	press   float64          // this tick's pressing force along the tool axis
	push    r3.Vector        // this tick's contact force on the tool, sensor frame: −sign·(F − F₀)
	contact bool
	integ   float64 // Ki·∫e dt, mm/s; clamped to ±MaxMMPerS (anti-windup), zero until contact
}

// sign is the sensor z sign a surface pushing back reads: −1 for "-z" (mujoco-sim), +1 for "+z".
func (f *forceCtl) sign() float64 {
	if f.cfg.PressAxis == "+z" {
		return 1
	}
	return -1
}

// update takes this tick's reading and advances the integral. The contact force on the tool is −sign·F: with "-z", a
// surface pushing the tool back toward the flange reads −z, so the push is F itself, and the press is −push.z.
func (f *forceCtl) update(force r3.Vector, dt float64) {
	f.push = force.Mul(-f.sign())
	f.press = -f.push.Z
	if f.contact {
		f.integ = math.Max(-f.cfg.MaxMMPerS, math.Min(f.cfg.MaxMMPerS, f.integ+f.cfg.Ki*(f.cfg.TargetN-f.press)*dt))
	}
}

// axisMMPerS is the commanded speed along the tool axis (positive = into the surface).
func (f *forceCtl) axisMMPerS() float64 {
	if !f.contact {
		return f.cfg.ApproachMMPerS
	}
	return math.Max(-f.cfg.MaxMMPerS, math.Min(f.cfg.MaxMMPerS, f.cfg.Gain*(f.cfg.TargetN-f.press)+f.integ))
}

// overlay replaces vel's linear component along the tool z axis with the force law's and, with ComplyXY, adds a
// sideways yield to the contact force: ComplyXY·push in tool x/y, clamped to MaxMMPerS. tool is the tool
// orientation in the base frame (FK of the command); the sensor frame is taken as the tool frame.
func (f *forceCtl) overlay(vel [6]float64, tool quat.Number) [6]float64 {
	axis := toolAxis(tool)
	vl := r3.Vector{X: vel[0], Y: vel[1], Z: vel[2]}
	vl = vl.Sub(axis.Mul(vl.Dot(axis))).Add(axis.Mul(f.axisMMPerS() / mmPerM))
	if f.cfg.ComplyXY > 0 && f.contact {
		side := r3.Vector{X: f.push.X, Y: f.push.Y}.Mul(f.cfg.ComplyXY)
		if n := side.Norm(); n > f.cfg.MaxMMPerS {
			side = side.Mul(f.cfg.MaxMMPerS / n)
		}
		vl = vl.Add(rotate(tool, side).Mul(1 / mmPerM))
	}
	vel[0], vel[1], vel[2] = vl.X, vl.Y, vl.Z
	return vel
}
