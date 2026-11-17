package diffik

import (
	"errors"
	"math"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

// FloorClamp raises a target so its flange and jaw tip (flange + TipMM along the tool axis) clear
// MinZMM + MarginMM in the arm base frame (stage 2's client floor_lift, moved server-side for
// camera-frame targets the client can't see in the arm frame).
type FloorClamp struct{ MinZMM, TipMM, MarginMM float64 }

// TargetFilter is servo_start's optional target_filter (spec 3c3), applied to pose targets in the
// arm base frame: EMA, then hold with hysteresis, then floor clamp.
type TargetFilter struct {
	EMAAlpha    float64 // (0, 1]; 1 = off
	HoldMM      float64 // 0 with HoldDeg 0 = hold off
	HoldDeg     float64
	Dwell       time.Duration // time inside the band before holding
	Floor       *FloorClamp   // nil = off
	Feedforward bool          // false: SetMotion with zero velocity (no extrapolation of noisy targets)
}

// DefaultTargetFilter passes targets through unchanged, with feedforward on (pre-3c3 behaviour).
func DefaultTargetFilter() TargetFilter { return TargetFilter{EMAAlpha: 1, Feedforward: true} }

func (f TargetFilter) validate() error {
	if !(f.EMAAlpha > 0 && f.EMAAlpha <= 1) {
		return errors.New("target_filter.ema_alpha must be in (0, 1]")
	}
	if !(f.HoldMM >= 0) || !(f.HoldDeg >= 0) || f.Dwell < 0 {
		return errors.New("target_filter hold_mm, hold_deg and dwell_ms must be >= 0")
	}
	if c := f.Floor; c != nil && (!(c.TipMM >= 0) || !(c.MarginMM >= 0) || math.IsNaN(c.MinZMM)) {
		return errors.New("target_filter.floor tip_mm and margin_mm must be >= 0")
	}
	return nil
}

// targetFilter is one session's filter state; reset by creating a new one per session.
type targetFilter struct {
	cfg     TargetFilter
	ema     spatialmath.Pose // nil until the first sample seeds it
	anchor  spatialmath.Pose
	since   time.Time
	holding bool
	clamps  int
}

func newTargetFilter(cfg TargetFilter) *targetFilter { return &targetFilter{cfg: cfg} }

// apply filters p (mm, arm base frame) received at t and reports whether the floor clamp moved it.
func (f *targetFilter) apply(t time.Time, p spatialmath.Pose) (spatialmath.Pose, bool) {
	if f.ema == nil || f.cfg.EMAAlpha >= 1 {
		f.ema = p
	} else {
		f.ema = spatialmath.Interpolate(f.ema, p, f.cfg.EMAAlpha)
	}
	out := f.ema
	if f.cfg.HoldMM > 0 || f.cfg.HoldDeg > 0 {
		if f.anchor == nil {
			f.anchor, f.since = out, t
		} else {
			mm := out.Point().Sub(f.anchor.Point()).Norm()
			deg := RotVec(out.Orientation().Quaternion(), f.anchor.Orientation().Quaternion()).Norm() * 180 / math.Pi
			if mm > f.cfg.HoldMM || deg > f.cfg.HoldDeg {
				f.anchor, f.since, f.holding = out, t, false
			} else {
				if !f.holding && t.Sub(f.since) >= f.cfg.Dwell {
					f.anchor, f.holding = out, true // hold the LATEST pose, never an older one
				}
				if f.holding {
					out = f.anchor
				}
			}
		}
	}
	if c := f.cfg.Floor; c != nil {
		ov := out.Orientation().OrientationVectorRadians() // (OX, OY, OZ) is the tool axis, normalised
		z := out.Point().Z
		lowest := math.Min(z, z+c.TipMM*ov.OZ)
		if lift := c.MinZMM + c.MarginMM - lowest; lift > 0 {
			f.clamps++
			return spatialmath.NewPose(out.Point().Add(r3.Vector{Z: lift}), out.Orientation()), true
		}
	}
	return out, false
}
