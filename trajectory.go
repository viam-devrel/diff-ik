package diffik

import (
	"math"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"gonum.org/v1/gonum/num/quat"
)

// trajectory is a servo_trajectory chunk: poses (base frame, mm) evenly dt apart, the first one for time t0 on the
// loop's clock. The loop samples it each tick at now + lead. Fields are fixed after SetTrajectory except prev, which
// the loop clears under its mutex.
type trajectory struct {
	t0       time.Time
	dt       time.Duration
	pos      []r3.Vector // mm
	rot      []quat.Number
	lead     time.Duration // used when !autoLead
	autoLead bool          // lead = the loop's measured lag
	blend    time.Duration // crossfade from prev over this long after arrive
	arrive   time.Time
	prev     *trajectory
}

func newTrajectory(t0 time.Time, dt time.Duration, poses []spatialmath.Pose) *trajectory {
	tr := &trajectory{t0: t0, dt: dt}
	for _, p := range poses {
		tr.pos = append(tr.pos, p.Point())
		tr.rot = append(tr.rot, p.Orientation().Quaternion())
	}
	return tr
}

// end is the time of the last pose.
func (tr *trajectory) end() time.Time { return tr.t0.Add(time.Duration(len(tr.pos)-1) * tr.dt) }

// at interpolates the chunk at t: position (mm), orientation, and the segment's velocity (m/s, rad/s, base frame).
// Before the first pose it holds the first; after the last it holds the last with zero velocity and ended true.
func (tr *trajectory) at(t time.Time) (r3.Vector, quat.Number, [6]float64, bool) {
	n := len(tr.pos) - 1
	f := float64(t.Sub(tr.t0)) / float64(tr.dt)
	switch {
	case f <= 0:
		return tr.pos[0], tr.rot[0], [6]float64{}, false
	case f >= float64(n):
		return tr.pos[n], tr.rot[n], [6]float64{}, true
	}
	i := int(f)
	u := f - float64(i)
	dp := tr.pos[i+1].Sub(tr.pos[i])
	dr := RotVec(tr.rot[i+1], tr.rot[i])
	p := tr.pos[i].Add(dp.Mul(u))
	q := quat.Mul(ExpRot(dr.Mul(u)), tr.rot[i])
	s := tr.dt.Seconds()
	return p, q, [6]float64{dp.X / mmPerM / s, dp.Y / mmPerM / s, dp.Z / mmPerM / s, dr.X / s, dr.Y / s, dr.Z / s}, false
}

// sample is the target for tick time now: the chunk at now + lead, crossfaded from prev during the blend window.
func (tr *trajectory) sample(now time.Time, lead time.Duration) (spatialmath.Pose, [6]float64, bool) {
	p, q, v, ended := tr.at(now.Add(lead))
	if pr := tr.prev; pr != nil && tr.blend > 0 && now.Before(tr.arrive.Add(tr.blend)) {
		w := math.Max(0, float64(now.Sub(tr.arrive))/float64(tr.blend))
		pp, pq, pv, _ := pr.at(now.Add(lead))
		p = pp.Add(p.Sub(pp).Mul(w))
		q = quat.Mul(ExpRot(RotVec(q, pq).Mul(w)), pq)
		for k := range v {
			v[k] = pv[k] + (v[k]-pv[k])*w
		}
		ended = false
	}
	qq := spatialmath.Quaternion(q)
	return spatialmath.NewPose(p, &qq), v, ended
}
