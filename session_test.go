package diffik

import (
	"context"
	"errors"
	"fmt"
	"math"
	"slices"
	"strings"
	"sync"
	"sync/atomic"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/arm/fake"
	"go.viam.com/rdk/components/arm/kinematics"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/spatialmath"
)

var t0 = time.Unix(1000, 0)

func at(ms float64) time.Time { return t0.Add(time.Duration(ms * float64(time.Millisecond))) }

// poseX is a pose at x mm along base x with a fixed orientation.
func poseX(x float64) spatialmath.Pose {
	return spatialmath.NewPose(r3.Vector{X: x, Y: 400, Z: 300}, &spatialmath.OrientationVectorDegrees{OZ: 1})
}

func newEst() *velEstimator { return &velEstimator{maxLin: 0.1, maxAng: deg2rad(45)} }

func near(a, b float64) bool { return math.Abs(a-b) <= 0.01*math.Abs(b)+1e-12 }

func TestEstimatorConstantVelocityFromSecondSample(t *testing.T) {
	e := newEst()
	if v := e.update(at(0), poseX(0)); v != ([6]float64{}) {
		t.Fatalf("first sample v = %v, want 0", v)
	}
	if v := e.update(at(20), poseX(1)); !near(v[0], 0.05) { // 1 mm / 20 ms = 50 mm/s
		t.Fatalf("second sample v = %.4f m/s, want 0.05", v[0])
	}
	if v := e.update(at(40), poseX(2)); !near(v[0], 0.05) {
		t.Fatalf("third sample v = %.4f m/s, want 0.05", v[0])
	}
}

func TestEstimatorStepResets(t *testing.T) {
	for _, gap := range []float64{100, 130} {
		e := newEst()
		e.update(at(0), poseX(0))
		e.update(at(gap), poseX(0))
		if v := e.update(at(2*gap), poseX(20)); v != ([6]float64{}) { // ≥ 153 mm/s > 1.5 × 100
			t.Fatalf("gap %v ms: 20 mm step gave v = %v, want 0", gap, v)
		}
	}
}

func TestEstimatorGapResets(t *testing.T) {
	e := newEst()
	e.update(at(0), poseX(0))
	e.update(at(20), poseX(1))
	if v := e.update(at(620), poseX(2)); v != ([6]float64{}) {
		t.Fatalf("after a 600 ms gap v = %v, want 0", v)
	}
}

func TestEstimatorIgnoresCloseSamples(t *testing.T) {
	e := newEst()
	e.update(at(0), poseX(0))
	e.update(at(20), poseX(1))
	if v := e.update(at(23), poseX(5)); !near(v[0], 0.05) { // 3 ms later: ignored, v kept
		t.Fatalf("close sample changed v to %.4f", v[0])
	}
	if v := e.update(at(40), poseX(2)); !near(v[0], 0.05) { // differentiated against the 20 ms sample
		t.Fatalf("after ignored sample v = %.4f, want 0.05", v[0])
	}
}

func TestEstimatorStationarySnapsAndReseeds(t *testing.T) {
	e := newEst()
	e.update(at(0), poseX(0))
	e.update(at(20), poseX(1))
	if v := e.update(at(40), poseX(1)); v != ([6]float64{}) {
		t.Fatalf("resent pose v = %v, want exactly 0", v)
	}
	if v := e.update(at(60), poseX(2)); !near(v[0], 0.05) { // reseeded, not EMA-ramped from 0
		t.Fatalf("after stationary v = %.4f, want 0.05", v[0])
	}
}

func TestEstimatorAngular(t *testing.T) {
	e := newEst()
	q0 := spatialmath.NewPose(r3.Vector{}, &spatialmath.OrientationVectorDegrees{OZ: 1})
	q1 := spatialmath.NewPose(r3.Vector{}, &spatialmath.OrientationVectorDegrees{OZ: 1, Theta: 0.2}) // 0.2° about z
	e.update(at(0), q0)
	if v := e.update(at(20), q1); !near(v[5], deg2rad(10)) { // 0.2° / 20 ms = 10 °/s
		t.Fatalf("angular v = %.5f rad/s, want %.5f", v[5], deg2rad(10))
	}
}

func TestEstimatorAngularBaseFrameTiltedTool(t *testing.T) {
	e := newEst()
	q0 := spatialmath.NewPose(r3.Vector{}, &spatialmath.OrientationVectorDegrees{OX: 1, Theta: 0})
	q1 := spatialmath.NewPose(r3.Vector{}, &spatialmath.OrientationVectorDegrees{OX: 1, Theta: 0.2})
	e.update(at(0), q0)
	v := e.update(at(20), q1)
	if !near(v[3], deg2rad(10)) || math.Abs(v[4]) > 1e-6 || math.Abs(v[5]) > 1e-6 {
		t.Fatalf("v = %v, want angular (%.5f, 0, 0) rad/s about base x", v, deg2rad(10))
	}
}

func TestEstimatorSlowAngularAndAngularJump(t *testing.T) {
	e := newEst()
	rz := func(deg float64) spatialmath.Pose {
		q := spatialmath.Quaternion(rotZ(deg2rad(deg)))
		return spatialmath.NewPose(r3.Vector{}, &q)
	}
	e.update(at(0), rz(0))
	if v := e.update(at(20), rz(0.04)); !near(v[5], deg2rad(2)) { // 2 °/s must not snap to zero
		t.Fatalf("slow rotation v = %v, want v[5] = %.5f", v, deg2rad(2))
	}
	e = newEst()
	e.update(at(0), rz(0))
	if v := e.update(at(20), rz(5)); v != ([6]float64{}) { // 250 °/s > 1.5 × 45
		t.Fatalf("5° jump v = %v, want 0", v)
	}
}

// fakeFS stands in for the frame system: arm_origin sits at twb in world.
type fakeFS struct {
	twb               spatialmath.Pose
	parts             []*referenceframe.FrameSystemPart
	cfgCalls, tfCalls atomic.Int32
}

func (f *fakeFS) FrameSystemConfig(context.Context) (*framesystem.Config, error) {
	f.cfgCalls.Add(1)
	return &framesystem.Config{Parts: f.parts}, nil
}

func (f *fakeFS) TransformPose(_ context.Context, p *referenceframe.PoseInFrame, dst string,
	_ []*referenceframe.LinkInFrame,
) (*referenceframe.PoseInFrame, error) {
	f.tfCalls.Add(1)
	if p.Parent() != "arm_origin" || dst != referenceframe.World {
		return nil, fmt.Errorf("unexpected transform %s → %s", p.Parent(), dst)
	}
	return referenceframe.NewPoseInFrame(referenceframe.World, spatialmath.Compose(f.twb, p.Pose())), nil
}

func fakeUR5e(t *testing.T) arm.Arm {
	t.Helper()
	cfg := resource.Config{Name: "arm", ConvertedAttributes: &fake.Config{ArmModel: kinematics.UR5e}}
	a, err := fake.NewArm(context.Background(), nil, cfg, logging.NewTestLogger(t))
	if err != nil {
		t.Fatal(err)
	}
	return a
}

// newTestManager returns a manual-tick manager on a fake clock whose sessions drive sim.
func newTestManager(t *testing.T, twb spatialmath.Pose) (*servoManager, *simIO, *fakeClock) {
	t.Helper()
	sim := &simIO{q: slices.Clone(home)}
	clk := newFakeClock()
	m := newServoManager(map[string]arm.Arm{"arm": fakeUR5e(t)}, &fakeFS{twb: twb}, logging.NewTestLogger(t))
	m.now, m.manual = clk.Now, true
	m.newIO = func(arm.Arm, float64) JointIO { return sim }
	return m, sim, clk
}

func testSessionCfg(mode Mode) SessionConfig {
	return SessionConfig{Frame: "arm_origin", Mode: mode, Loop: LoopConfig{RateHz: 50},
		Timeout: time.Second, Ramp: 200 * time.Millisecond}
}

// step runs one tick the way Run would, including OnTick and the end-of-session hand-off.
func step(t *testing.T, s *session, clk *fakeClock, n int) {
	t.Helper()
	dt := 1 / s.loop.cfg.RateHz
	for i := 0; i < n; i++ {
		clk.Advance(time.Duration(dt * float64(time.Second)))
		err := s.loop.tick(context.Background(), dt)
		switch {
		case errors.Is(err, errRampDone):
			s.finish(s.loop.stopErr)
			return
		case err != nil:
			t.Fatal(err)
		}
		s.loop.OnTick(s.loop.Status())
	}
}

func mustStart(t *testing.T, m *servoManager, cfg SessionConfig) (*session, spatialmath.Pose) {
	t.Helper()
	s, p, err := m.start(context.Background(), "arm", cfg)
	if err != nil {
		t.Fatal(err)
	}
	return s, p
}

func wantErr(t *testing.T, err error, substr string) {
	t.Helper()
	if err == nil || !strings.Contains(err.Error(), substr) {
		t.Fatalf("err = %v, want containing %q", err, substr)
	}
}

func TestSessionStartHoldsCurrentPose(t *testing.T) {
	m, sim, clk := newTestManager(t, spatialmath.NewZeroPose())
	s, p := mustStart(t, m, testSessionCfg(ModePose))
	fk, err := ur5e(t).Transform(home)
	if err != nil {
		t.Fatal(err)
	}
	if p.Point().Sub(fk.Point()).Norm() > 1e-6 {
		t.Fatalf("start pose %v, want FK(home) %v", p.Point(), fk.Point())
	}
	step(t, s, clk, 10)
	if d := maxAbsDiff(sim.q, home); d > 1e-9 || s.status().State != StateConverged {
		t.Fatalf("arm moved %.3g rad / state %s, want still and converged", d, s.status().State)
	}
}

func TestSessionBusyUnknownArmBadFrameAndMode(t *testing.T) {
	m, _, _ := newTestManager(t, spatialmath.NewZeroPose())
	mustStart(t, m, testSessionCfg(ModePose))
	_, _, err := m.start(context.Background(), "arm", testSessionCfg(ModePose))
	wantErr(t, err, "busy")
	if !m.busy("arm") {
		t.Fatal("busy(arm) = false with a running session")
	}
	_, _, err = m.start(context.Background(), "nope", testSessionCfg(ModePose))
	wantErr(t, err, "not in this service's arms")

	m2, _, _ := newTestManager(t, spatialmath.NewZeroPose())
	cfg := testSessionCfg(ModePose)
	cfg.Frame = "arm" // the end-effector frame, not the base
	_, _, err = m2.start(context.Background(), "arm", cfg)
	wantErr(t, err, "unsupported frame")
	cfg = testSessionCfg("fly")
	_, _, err = m2.start(context.Background(), "arm", cfg)
	wantErr(t, err, "unsupported mode")
	_, err = m2.get("missing")
	wantErr(t, err, "unknown session_id")
}

func TestSessionLatestWins(t *testing.T) {
	m, _, clk := newTestManager(t, spatialmath.NewZeroPose())
	s, p := mustStart(t, m, testSessionCfg(ModePose))
	p1 := spatialmath.NewPose(p.Point().Add(r3.Vector{X: 30}), p.Orientation())
	p2 := spatialmath.NewPose(p.Point().Add(r3.Vector{Y: 30}), p.Orientation())
	for _, q := range []spatialmath.Pose{p1, p2} {
		if _, err := s.target(q, "", true); err != nil {
			t.Fatal(err)
		}
	}
	for i := 0; i < 30; i++ { // 3 s, resending p2 every 100 ms to feed the dead-man
		step(t, s, clk, 5)
		if _, err := s.target(p2, "", true); err != nil {
			t.Fatal(err)
		}
	}
	st := s.status()
	if st.TargetMM.Sub(p2.Point()).Norm() > 1e-6 || st.State != StateConverged {
		t.Fatalf("target %v state %s, want p2 %v and converged", st.TargetMM, st.State, p2.Point())
	}
}

func TestSessionWrongModeAndTwistLimits(t *testing.T) {
	m, _, _ := newTestManager(t, spatialmath.NewZeroPose())
	s, p := mustStart(t, m, testSessionCfg(ModePose))
	_, err := s.twist(r3.Vector{}, r3.Vector{})
	wantErr(t, err, "wrong mode: session is pose")

	m2, _, _ := newTestManager(t, spatialmath.NewZeroPose())
	tw, _ := mustStart(t, m2, testSessionCfg(ModeTwist))
	_, err = tw.target(p, "", true)
	wantErr(t, err, "wrong mode: session is twist")
	_, err = tw.twist(r3.Vector{X: 0.2}, r3.Vector{}) // 200 mm/s > 100 mm/s
	wantErr(t, err, "exceeds")
	_, err = tw.twist(r3.Vector{}, r3.Vector{Z: deg2rad(60)}) // 60 °/s > 45 °/s
	wantErr(t, err, "exceeds")
}

func TestSessionTwistKeepsAdvancedTarget(t *testing.T) {
	m, _, clk := newTestManager(t, spatialmath.NewZeroPose())
	s, p := mustStart(t, m, testSessionCfg(ModeTwist))
	for i := 0; i < 2; i++ {
		if _, err := s.twist(r3.Vector{X: 0.02}, r3.Vector{}); err != nil {
			t.Fatal(err)
		}
		step(t, s, clk, 25)
	}
	if d := s.status().TargetMM.X - p.Point().X; math.Abs(d-20) > 0.05 {
		t.Fatalf("target moved %.3f mm over two 0.5 s twists at 20 mm/s, want 20 (no snap-back)", d)
	}
}

func TestSessionHeldPoseConverges(t *testing.T) {
	m, _, clk := newTestManager(t, spatialmath.NewZeroPose())
	s, p := mustStart(t, m, testSessionCfg(ModePose))
	goal := spatialmath.NewPose(p.Point().Add(r3.Vector{Z: 20}), p.Orientation())
	for i := 0; i < 30; i++ {
		if _, err := s.target(goal, "", true); err != nil {
			t.Fatal(err)
		}
		step(t, s, clk, 5)
	}
	if st := s.status(); st.State != StateConverged {
		t.Fatalf("state = %s err = %.2f mm, want converged on a resent pose", st.State, st.ErrMM)
	}
}

func TestSessionDeadmanTimeout(t *testing.T) {
	for _, solver := range []string{SolverDLS, SolverQP} {
		t.Run(solver, func(t *testing.T) {
			m, sim, clk := newTestManager(t, spatialmath.NewZeroPose())
			cfg := testSessionCfg(ModePose)
			cfg.Loop.Solver = solver
			cfg.Timeout, cfg.Ramp = 100*time.Millisecond, 100*time.Millisecond
			s, p := mustStart(t, m, cfg)
			ticks := 0
			for !s.ended() && ticks < 100 {
				step(t, s, clk, 1)
				ticks++
			}
			// timeout + dt + ramp = 100 + 20 + 100 ms
			if elapsed := time.Duration(ticks) * 20 * time.Millisecond; elapsed > 220*time.Millisecond {
				t.Fatalf("dead-man stop took %v, want ≤ 220 ms", elapsed)
			}
			st := s.status()
			if st.State != StateStoppedTimeout || !sim.stopped {
				t.Fatalf("state = %s stopped = %v, want stopped_timeout and stopped", st.State, sim.stopped)
			}
			_, err := s.target(p, "", true)
			wantErr(t, err, "session ended: stopped_timeout")
			if m.busy("arm") {
				t.Fatal("arm still busy after the session ended")
			}
			if _, _, err := m.start(context.Background(), "arm", cfg); err != nil {
				t.Fatalf("restart after timeout: %v", err)
			}
		})
	}
}

func TestSessionWorldFrameMapping(t *testing.T) {
	rz := spatialmath.Quaternion(rotZ(math.Pi / 2))
	twb := spatialmath.NewPose(r3.Vector{X: 100, Y: 200, Z: 300}, &rz)
	m, _, clk := newTestManager(t, twb)
	cfg := testSessionCfg(ModePose)
	cfg.Frame = "world"
	s, pw := mustStart(t, m, cfg)
	fk, _ := ur5e(t).Transform(home)
	if pw.Point().Sub(spatialmath.Compose(twb, fk).Point()).Norm() > 1e-6 {
		t.Fatalf("start pose in world %v, want %v", pw.Point(), spatialmath.Compose(twb, fk).Point())
	}
	pb := spatialmath.NewPose(fk.Point().Add(r3.Vector{X: 10}), fk.Orientation())
	if _, err := s.target(spatialmath.Compose(twb, pb), "", true); err != nil {
		t.Fatal(err)
	}
	step(t, s, clk, 1) // Status.TargetMM is published by tick; v̂ = 0 on a first sample, so the target doesn't move
	if got := s.status().TargetMM; got.Sub(pb.Point()).Norm() > 1e-6 {
		t.Fatalf("base-frame target %v, want %v", got, pb.Point())
	}

	m2, _, clk2 := newTestManager(t, twb)
	cfg.Mode = ModeTwist
	s2, _ := mustStart(t, m2, cfg)
	step(t, s2, clk2, 1)
	before := s2.status().TargetMM
	if _, err := s2.twist(r3.Vector{X: 0.02}, r3.Vector{}); err != nil { // world +x = base −y under Rz(90°)
		t.Fatal(err)
	}
	step(t, s2, clk2, 40) // stay clear of the 1 s dead-man edge
	d := s2.status().TargetMM.Sub(before)
	if math.Abs(d.Y+16) > 0.05 || math.Abs(d.X) > 0.05 {
		t.Fatalf("base-frame target moved %v mm, want (0, −16, 0)", d)
	}
}

// Real clock and real Run goroutine: servo_stop ramps, stops the arm and ends the session.
func TestSessionStopRealClock(t *testing.T) {
	sim := &simIO{q: slices.Clone(home)}
	m := newServoManager(map[string]arm.Arm{"arm": fakeUR5e(t)}, &fakeFS{twb: spatialmath.NewZeroPose()}, logging.NewTestLogger(t))
	m.newIO = func(arm.Arm, float64) JointIO { return sim }
	s, _ := mustStart(t, m, testSessionCfg(ModePose))
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	st, err := s.stop(ctx)
	if err != nil {
		t.Fatal(err)
	}
	if st.State != StateStopped || !sim.stopped {
		t.Fatalf("state = %s stopped = %v, want stopped", st.State, sim.stopped)
	}
	_, err = s.stop(ctx)
	wantErr(t, err, "session ended: stopped")
	if err := m.close(ctx); err != nil {
		t.Fatal(err)
	}
}

func TestSessionCommandsRejectedWhileStopping(t *testing.T) {
	m, _, clk := newTestManager(t, spatialmath.NewZeroPose())
	cfg := testSessionCfg(ModePose)
	cfg.Timeout, cfg.Ramp = 100*time.Millisecond, 200*time.Millisecond
	s, p := mustStart(t, m, cfg)
	step(t, s, clk, 7) // past the 100 ms timeout, inside the 200 ms ramp
	if s.ended() {
		t.Fatal("session ended before the ramp finished")
	}
	_, err := s.target(p, "", true)
	wantErr(t, err, "session stopping: stopped_timeout")
	for i := 0; i < 20 && !s.ended(); i++ {
		step(t, s, clk, 1)
	}
	if !s.ended() || s.status().State != StateStoppedTimeout {
		t.Fatalf("ended = %v state = %s, want ended stopped_timeout", s.ended(), s.status().State)
	}

	m2, _, _ := newTestManager(t, spatialmath.NewZeroPose())
	tw, _ := mustStart(t, m2, testSessionCfg(ModeTwist))
	tw.beginStop(StateStopped) // what stop() does before it waits
	_, err = tw.twist(r3.Vector{X: 0.01}, r3.Vector{})
	wantErr(t, err, "session stopping: stopped")
}

func TestManagerCloseStopsLiveSession(t *testing.T) {
	sim := &simIO{q: slices.Clone(home)}
	m := newServoManager(map[string]arm.Arm{"arm": fakeUR5e(t)}, &fakeFS{twb: spatialmath.NewZeroPose()}, logging.NewTestLogger(t))
	m.newIO = func(arm.Arm, float64) JointIO { return sim }
	s, _ := mustStart(t, m, testSessionCfg(ModePose))
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	if err := m.close(ctx); err != nil {
		t.Fatal(err)
	}
	if st := s.status().State; !sim.stopped || st != StateStopped {
		t.Fatalf("stopped = %v state = %s, want stopped", sim.stopped, st)
	}
	_, _, err := m.start(ctx, "arm", testSessionCfg(ModePose))
	wantErr(t, err, "closing")
}

func TestSessionFaultDetail(t *testing.T) {
	sim := &simIO{q: slices.Clone(home), sendErr: errors.New("bus error")}
	m := newServoManager(map[string]arm.Arm{"arm": fakeUR5e(t)}, &fakeFS{twb: spatialmath.NewZeroPose()}, logging.NewTestLogger(t))
	m.newIO = func(arm.Arm, float64) JointIO { return sim }
	s, p := mustStart(t, m, testSessionCfg(ModePose))
	select {
	case <-s.done:
	case <-time.After(2 * time.Second):
		t.Fatal("session did not fault")
	}
	_, err := s.target(p, "", true)
	wantErr(t, err, "session ended: faulted: ")
	wantErr(t, err, "bus error")
}

func TestSessionFinishCancelIsStopped(t *testing.T) {
	m, _, _ := newTestManager(t, spatialmath.NewZeroPose())
	s, _ := mustStart(t, m, testSessionCfg(ModePose))
	s.finish(context.Canceled)
	if st := s.status().State; st != StateStopped {
		t.Fatalf("state = %s, want stopped", st)
	}
}

func TestStopFailureSurfaces(t *testing.T) {
	newM := func() (*servoManager, *session) {
		sim := &simIO{q: slices.Clone(home), stopErr: errors.New("stop failed")}
		m := newServoManager(map[string]arm.Arm{"arm": fakeUR5e(t)}, &fakeFS{twb: spatialmath.NewZeroPose()}, logging.NewTestLogger(t))
		m.newIO = func(arm.Arm, float64) JointIO { return sim }
		s, _ := mustStart(t, m, testSessionCfg(ModePose))
		return m, s
	}
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	_, s := newM()
	st, err := s.stop(ctx)
	wantErr(t, err, "stop failed")
	if st.State != StateStopped {
		t.Fatalf("state = %s, want stopped", st.State)
	}
	wantErr(t, s.endedErr(), "stop failed")
	m, _ := newM()
	wantErr(t, m.close(ctx), "stop failed")
	m, _ = newM()
	wantErr(t, m.stopArm(ctx, "arm"), "stop failed") // StopPlan path
}

func TestStopFailedKeepsJoinedStopError(t *testing.T) {
	if stopFailed(nil) || stopFailed(context.Canceled) {
		t.Fatal("nil or a bare cancel must not count as a stop failure")
	}
	if !stopFailed(errors.Join(context.Canceled, errors.New("stopping arm: bus error"))) {
		t.Fatal("a cancel joined with an arm.Stop failure must be reported")
	}
}

func TestSessionTwistReportsIgnoredRoll(t *testing.T) {
	m, _, clk := newTestManager(t, spatialmath.NewZeroPose())
	cfg := testSessionCfg(ModeTwist)
	cfg.Loop.Task = TaskPointing
	s, _ := mustStart(t, m, cfg)
	step(t, s, clk, 1) // populate ToolAxis
	a := s.loop.Status().ToolAxis
	st, err := s.twist(r3.Vector{}, a.Mul(deg2rad(10))) // pure roll about the tool axis
	if err != nil || !st.RollIgnored {
		t.Fatalf("roll twist: ignored=%v err=%v", st.RollIgnored, err)
	}
	st, err = s.twist(r3.Vector{X: 0.01}, r3.Vector{})
	if err != nil || st.RollIgnored {
		t.Fatalf("linear twist: ignored=%v err=%v", st.RollIgnored, err)
	}
}

// The roll check is in the base frame: the session-frame twist is rotated into it first.
func TestSessionTwistReportsIgnoredRollRotatedFrame(t *testing.T) {
	fk, err := ur5e(t).Transform(home)
	if err != nil {
		t.Fatal(err)
	}
	a0 := toolAxis(fk.Orientation().Quaternion())
	k := a0.Cross(r3.Vector{Z: 1})
	if k.Norm() < 1e-3 {
		k = a0.Cross(r3.Vector{X: 1})
	}
	rq := spatialmath.Quaternion(ExpRot(k.Normalize().Mul(math.Pi / 2))) // R ⟂ a: R·a ⟂ a and R⁻¹·a ⟂ a
	twb := spatialmath.NewPose(r3.Vector{X: 100}, &rq)
	for _, tc := range []struct {
		name    string
		toWorld bool // send R_wb·a (the tool axis in the session frame) instead of the raw base-frame a
		want    bool
	}{{"tool axis in session frame", true, true}, {"raw base-frame axis", false, false}} {
		m, _, clk := newTestManager(t, twb)
		cfg := testSessionCfg(ModeTwist)
		cfg.Frame = "world"
		cfg.Loop.Task = TaskPointing
		s, _ := mustStart(t, m, cfg)
		step(t, s, clk, 1)
		a := s.loop.Status().ToolAxis
		ang := a
		if tc.toWorld {
			ang = spatialmath.Compose(spatialmath.NewPose(r3.Vector{}, twb.Orientation()), spatialmath.NewPoseFromPoint(a)).Point()
		}
		st, err := s.twist(r3.Vector{}, ang.Mul(deg2rad(10)))
		if err != nil || st.RollIgnored != tc.want {
			t.Fatalf("%s: ignored=%v err=%v, want %v", tc.name, st.RollIgnored, err, tc.want)
		}
	}
}

// camPose is the test camera's pose in the arm's end-effector frame.
var camPose = spatialmath.NewPose(r3.Vector{X: 10, Y: -20, Z: 30}, &spatialmath.OrientationVectorDegrees{OX: 0.3, OZ: 1, Theta: 20})

// newFramedTestManager is newTestManager with a frame system holding the arm (a UR5e model frame named
// "arm"), a camera "cam" on its end effector, a camera "static" on world, and a second movable arm "arm2"
// with a camera "cam2" on it. NewFrameSystem adds the "<name>_origin" frames itself.
func newFramedTestManager(t *testing.T, cam spatialmath.Pose) (*servoManager, *simIO, *fakeClock) {
	t.Helper()
	m, sim, clk := newTestManager(t, spatialmath.NewZeroPose())
	model2, err := kinematics.ModelFromName(kinematics.UR5e, "arm2")
	if err != nil {
		t.Fatal(err)
	}
	link := func(parent string, p spatialmath.Pose, name string) *referenceframe.LinkInFrame {
		return referenceframe.NewLinkInFrame(parent, p, name, nil)
	}
	zero := spatialmath.NewZeroPose()
	m.fs.(*fakeFS).parts = []*referenceframe.FrameSystemPart{
		{FrameConfig: link(referenceframe.World, zero, "arm"), ModelFrame: ur5e(t)},
		{FrameConfig: link("arm", cam, "cam")},
		{FrameConfig: link(referenceframe.World, spatialmath.NewPoseFromPoint(r3.Vector{Z: 500}), "static")},
		{FrameConfig: link(referenceframe.World, zero, "arm2"), ModelFrame: model2},
		{FrameConfig: link("arm2", cam, "cam2")},
	}
	return m, sim, clk
}

func framedCfg() SessionConfig {
	cfg := testSessionCfg(ModePose)
	cfg.Frames = true
	return cfg
}

func loopTarget(s *session) spatialmath.Pose {
	s.loop.mu.Lock()
	defer s.loop.mu.Unlock()
	return s.loop.target
}

func TestSessionFrameTargetUsesMeasuredJoints(t *testing.T) {
	m, sim, clk := newFramedTestManager(t, camPose)
	s, _ := mustStart(t, m, framedCfg())
	startQ := slices.Clone(sim.q)
	sim.q[0] += deg2rad(1) // below resync_deg: the loop keeps its command, QMeas follows the arm
	step(t, s, clk, 1)
	qm := s.loop.Status().QMeas
	if maxAbsDiff(qm, startQ) < 1e-6 {
		t.Fatal("QMeas did not move")
	}
	pCam := spatialmath.NewPose(r3.Vector{X: 5, Y: 6, Z: 7}, &spatialmath.OrientationVectorDegrees{OZ: 1, Theta: 10})
	if _, err := s.target(pCam, "cam", true); err != nil {
		t.Fatal(err)
	}
	want := func(q []float64) spatialmath.Pose {
		fk, err := ur5e(t).Transform(q)
		if err != nil {
			t.Fatal(err)
		}
		return spatialmath.Compose(fk, spatialmath.Compose(camPose, pCam))
	}
	got := loopTarget(s)
	if !spatialmath.PoseAlmostCoincidentEps(got, want(qm), 1e-6) {
		t.Fatalf("target %v, want FK(QMeas) o cam o p = %v", got, want(qm))
	}
	if spatialmath.PoseAlmostCoincidentEps(got, want(startQ), 1e-3) {
		t.Fatal("target matches the start joints; the measured joints were not used")
	}
}

func TestSessionFrameRequiresFrames(t *testing.T) {
	m, _, _ := newFramedTestManager(t, camPose)
	s, p := mustStart(t, m, testSessionCfg(ModePose))
	_, err := s.target(p, "cam", true)
	wantErr(t, err, "frames")
}

func TestSessionUnknownFrame(t *testing.T) {
	m, _, _ := newFramedTestManager(t, camPose)
	s, p := mustStart(t, m, framedCfg())
	before := loopTarget(s)
	_, err := s.target(p, "nosuchcam", true)
	wantErr(t, err, "nosuchcam")
	if !spatialmath.PoseAlmostEqual(loopTarget(s), before) {
		t.Fatal("loop target changed after an unknown frame")
	}
}

func TestSessionFrameChainWithOtherMovableFrameRejected(t *testing.T) {
	m, _, _ := newFramedTestManager(t, camPose)
	s, p := mustStart(t, m, framedCfg())
	_, err := s.target(p, "cam2", true)
	wantErr(t, err, "movable")
	_, err2 := s.target(p, "cam2", true)
	if err2 == nil || err2.Error() != err.Error() {
		t.Fatalf("second call err = %v, want the cached %v", err2, err)
	}
	// static and world-parented cameras are fine
	if _, err := s.target(p, "static", true); err != nil {
		t.Fatal(err)
	}
}

func TestSessionTargetApplyFalse(t *testing.T) {
	m, _, _ := newFramedTestManager(t, camPose)
	s, p := mustStart(t, m, framedCfg())
	before := loopTarget(s)
	res, err := s.target(p, "cam", false)
	if err != nil {
		t.Fatal(err)
	}
	if res.Raw == nil || res.Target == nil || !spatialmath.PoseAlmostEqual(res.Raw, res.Target) {
		t.Fatalf("raw %v target %v, want equal and non-nil", res.Raw, res.Target)
	}
	if !spatialmath.PoseAlmostEqual(loopTarget(s), before) {
		t.Fatal("apply:false changed the loop target")
	}
	s.mu.Lock()
	seeded := s.filt.ema != nil
	s.mu.Unlock()
	if seeded {
		t.Fatal("apply:false advanced the filter")
	}
}

func TestSessionFilterFeedforwardOff(t *testing.T) {
	for _, ff := range []bool{false, true} {
		m, _, clk := newFramedTestManager(t, camPose)
		cfg := framedCfg()
		cfg.Filter = TargetFilter{EMAAlpha: 1, Feedforward: ff}
		s, p := mustStart(t, m, cfg)
		for i := 0; i < 2; i++ { // 2 mm in 20 ms = 100 mm/s
			q := spatialmath.NewPose(p.Point().Add(r3.Vector{X: 2 * float64(i)}), p.Orientation())
			if _, err := s.target(q, "", true); err != nil {
				t.Fatal(err)
			}
			clk.Advance(20 * time.Millisecond)
		}
		s.loop.mu.Lock()
		v := s.loop.vel
		s.loop.mu.Unlock()
		if got := math.Abs(v[0]); ff && got < 0.01 || !ff && got != 0 {
			t.Fatalf("feedforward %v: vel = %v", ff, v)
		}
	}
}

func TestSessionFrameTargetsMakeNoFrameSystemCalls(t *testing.T) {
	m, _, _ := newFramedTestManager(t, camPose)
	fsys := m.fs.(*fakeFS)
	s, p := mustStart(t, m, framedCfg())
	if n := fsys.cfgCalls.Load(); n != 1 {
		t.Fatalf("FrameSystemConfig calls at start = %d, want 1", n)
	}
	cfg0, tf0 := fsys.cfgCalls.Load(), fsys.tfCalls.Load()
	for i := 0; i < 5; i++ {
		if _, err := s.target(p, "cam", true); err != nil {
			t.Fatal(err)
		}
	}
	if fsys.cfgCalls.Load() != cfg0 || fsys.tfCalls.Load() != tf0 {
		t.Fatal("framed targets called the frame system (an RPC per target)")
	}
}

func fkOf(t *testing.T, q []float64) spatialmath.Pose {
	t.Helper()
	fk, err := ur5e(t).Transform(q)
	if err != nil {
		t.Fatal(err)
	}
	return fk
}

var pCamTest = spatialmath.NewPose(r3.Vector{X: 5, Y: 6, Z: 7}, &spatialmath.OrientationVectorDegrees{OZ: 1, Theta: 10})

func TestSessionFrameTargetFallsBackToStartQ(t *testing.T) {
	m, sim, _ := newFramedTestManager(t, camPose)
	s, _ := mustStart(t, m, framedCfg()) // no tick: QMeas is empty
	if _, err := s.target(pCamTest, "cam", true); err != nil {
		t.Fatal(err)
	}
	want := spatialmath.Compose(fkOf(t, sim.q), spatialmath.Compose(camPose, pCamTest))
	if got := loopTarget(s); !spatialmath.PoseAlmostCoincidentEps(got, want, 1e-6) {
		t.Fatalf("target %v, want %v", got, want)
	}
}

func TestSessionFrameWorldSessionFrameWithOffsetBase(t *testing.T) {
	rz := spatialmath.Quaternion(rotZ(math.Pi / 3))
	twb := spatialmath.NewPose(r3.Vector{X: 100, Y: 200, Z: 300}, &rz)
	m, sim, clk := newFramedTestManager(t, camPose)
	fsys := m.fs.(*fakeFS)
	fsys.twb = twb
	fsys.parts[0].FrameConfig = referenceframe.NewLinkInFrame(referenceframe.World, twb, "arm", nil)
	cfg := framedCfg()
	cfg.Frame = referenceframe.World
	s, _ := mustStart(t, m, cfg)
	sim.q[0] += deg2rad(1)
	step(t, s, clk, 1)
	qm := s.loop.Status().QMeas
	res, err := s.target(pCamTest, "cam", true)
	if err != nil {
		t.Fatal(err)
	}
	want := spatialmath.Compose(fkOf(t, qm), spatialmath.Compose(camPose, pCamTest))
	got := loopTarget(s)
	if !spatialmath.PoseAlmostCoincidentEps(got, want, 1e-6) {
		t.Fatalf("base-frame target %v, want %v", got, want)
	}
	if !spatialmath.PoseAlmostCoincidentEps(res.Target, spatialmath.Compose(twb, got), 1e-6) {
		t.Fatalf("res.Target %v, want twb o loop target", res.Target)
	}
}

func TestSessionPreviewDoesNotNeedLock(t *testing.T) {
	m, _, _ := newFramedTestManager(t, camPose)
	s, p := mustStart(t, m, framedCfg())
	s.mu.Lock()
	done := make(chan error, 1)
	go func() { _, err := s.target(p, "cam", false); done <- err }()
	select {
	case err := <-done:
		s.mu.Unlock()
		if err != nil {
			t.Fatal(err)
		}
	case <-time.After(2 * time.Second):
		s.mu.Unlock()
		t.Fatal("apply:false transform blocked on s.mu")
	}
}

func TestSessionConcurrentFramedTargets(t *testing.T) {
	m, _, _ := newFramedTestManager(t, camPose)
	s, p := mustStart(t, m, framedCfg())
	var wg sync.WaitGroup
	for g := 0; g < 4; g++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for i := 0; i < 50; i++ {
				if _, err := s.target(p, "cam", true); err != nil {
					t.Error(err)
					return
				}
			}
		}()
	}
	wg.Wait()
}

func TestSessionFloorClampReported(t *testing.T) {
	m, _, _ := newFramedTestManager(t, camPose)
	cfg := framedCfg()
	cfg.Filter = TargetFilter{EMAAlpha: 1, Feedforward: true, Floor: &FloorClamp{MinZMM: 0, TipMM: 100, MarginMM: 10}}
	s, _ := mustStart(t, m, cfg)
	low := spatialmath.NewPose(r3.Vector{X: 300, Y: 100, Z: -50}, &spatialmath.OrientationVectorDegrees{OZ: -1})
	res, err := s.target(low, "", true)
	if err != nil {
		t.Fatal(err)
	}
	if !res.Clamped || s.status().Clamps != 1 {
		t.Fatalf("clamped %v clamps %d, want true and 1", res.Clamped, s.status().Clamps)
	}
	if !(res.Target.Point().Z > res.Raw.Point().Z) {
		t.Fatalf("target z %v not above raw z %v", res.Target.Point().Z, res.Raw.Point().Z)
	}
}

// servo_start's point_velocities reaches the session's armIO (newIO runs per session).
func TestSessionPointVelocitiesOption(t *testing.T) {
	for _, off := range []bool{false, true} {
		m := newServoManager(map[string]arm.Arm{"arm": fakeUR5e(t)}, &fakeFS{twb: spatialmath.NewZeroPose()}, logging.NewTestLogger(t))
		var got *armIO
		m.newIO = func(a arm.Arm, hz float64) JointIO { got = NewArmIO(a, hz).(*armIO); return got }
		cfg := testSessionCfg(ModePose)
		cfg.NoPointVelocities = off
		mustStart(t, m, cfg)
		if got.pointVel == off {
			t.Errorf("NoPointVelocities=%v -> pointVel=%v", off, got.pointVel)
		}
	}
}

// The reply describes the new chunk, not the status from before the loop's next tick samples it.
func TestSessionTrajectoryReplyReportsNewChunk(t *testing.T) {
	m, _, clk := newTestManager(t, spatialmath.NewZeroPose())
	s, p := mustStart(t, m, testSessionCfg(ModePose))
	poses := make([]spatialmath.Pose, 10)
	for i := range poses {
		poses[i] = p
	}
	lead := 80 * time.Millisecond
	st, err := s.trajectory(poses, 100*time.Millisecond, 50*time.Millisecond, &lead, 0)
	if err != nil {
		t.Fatal(err)
	}
	if !st.Traj || st.TrajEnded || st.TrajLeadMS != 80 || st.TrajRemainingMS != 770 {
		t.Fatalf("traj %v ended %v lead %v remaining %v, want true false 80 770", st.Traj, st.TrajEnded, st.TrajLeadMS, st.TrajRemainingMS)
	}
	// A tick publishes the median; 3 ticks at 50 Hz is 60 ms.
	s.loop.mu.Lock()
	s.loop.lagSamples = []float64{3}
	s.loop.mu.Unlock()
	step(t, s, clk, 1)
	st, err = s.trajectory(poses, 100*time.Millisecond, 50*time.Millisecond, nil, 0)
	if err != nil {
		t.Fatal(err)
	}
	if st.TrajLeadMS != 60 || st.TrajRemainingMS != 790 {
		t.Fatalf("auto lead %v remaining %v, want 60 790", st.TrajLeadMS, st.TrajRemainingMS)
	}
}
