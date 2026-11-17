package diffik

import (
	"context"
	"errors"
	"slices"
	"strings"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	commonpb "go.viam.com/api/common/v1"
	motionpb "go.viam.com/api/service/motion/v1"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/arm/kinematics"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/spatialmath"
	"google.golang.org/protobuf/encoding/protojson"
)

// gripPose is the test gripper's pose in the arm's end-effector frame (the target station's gripper).
var gripPose = spatialmath.NewPose(r3.Vector{Z: 105}, &spatialmath.OrientationVectorDegrees{OZ: 1, Theta: 180})

// teleopFS serves a real frame system (arm, with a rigid gripper child) and defers arm_origin to fakeFS.
type teleopFS struct{ *fakeFS }

func (f teleopFS) FrameSystemConfig(context.Context) (*framesystem.Config, error) {
	return &framesystem.Config{Parts: []*referenceframe.FrameSystemPart{
		{FrameConfig: referenceframe.NewLinkInFrame(referenceframe.World, spatialmath.NewZeroPose(), "arm", nil), ModelFrame: ur5eModel()},
		{FrameConfig: referenceframe.NewLinkInFrame("arm", gripPose, "grip", nil)},
	}}, nil
}

func (f teleopFS) TransformPose(ctx context.Context, p *referenceframe.PoseInFrame, dst string,
	sup []*referenceframe.LinkInFrame,
) (*referenceframe.PoseInFrame, error) {
	if p.Parent() == "arm_origin" {
		return f.fakeFS.TransformPose(ctx, p, dst, sup)
	}
	cfg, _ := f.FrameSystemConfig(ctx)
	fsys, err := referenceframe.NewFrameSystem("t", cfg.Parts, nil)
	if err != nil {
		return nil, err
	}
	tf, err := fsys.Transform(referenceframe.NewZeroInputs(fsys).ToLinearInputs(), p, dst)
	if err != nil {
		return nil, err
	}
	return tf.(*referenceframe.PoseInFrame), nil
}

func ur5eModel() referenceframe.Model {
	m, err := kinematics.ModelFromName(kinematics.UR5e, "arm")
	if err != nil {
		panic(err)
	}
	return m
}

func newTeleopServo(t *testing.T, conf map[string]interface{}) (*diffIkServo, *servoManager, *fakeClock) {
	t.Helper()
	m, _, clk := newTestManager(t, spatialmath.NewZeroPose())
	m.fs = teleopFS{m.fs.(*fakeFS)}
	s := testServo(t, m)
	s.teleopConf = conf
	return s, m, clk
}

func startCmd(t *testing.T, comp string) map[string]interface{} {
	t.Helper()
	b, err := protojson.Marshal(&motionpb.MoveRequest{Name: "svc", ComponentName: comp})
	if err != nil {
		t.Fatal(err)
	}
	return map[string]interface{}{"teleop_start": string(b)}
}

func moveCmd(t *testing.T, comp string, p spatialmath.Pose) map[string]interface{} {
	t.Helper()
	b, err := protojson.Marshal(referenceframe.PoseInFrameToProtobuf(referenceframe.NewPoseInFrame(referenceframe.World, p)))
	if err != nil {
		t.Fatal(err)
	}
	return map[string]interface{}{"teleop_move": string(b), "component_name": comp, "seq": 1.0}
}

func teleStatus(t *testing.T, s *diffIkServo) map[string]interface{} {
	t.Helper()
	return do(t, s, map[string]interface{}{"teleop_status": true})["teleop_status"].(map[string]interface{})
}

func teleSession(t *testing.T, m *servoManager) *session {
	t.Helper()
	m.mu.Lock()
	defer m.mu.Unlock()
	return m.byArm["arm"]
}

func TestTeleopGripperMoveMapsToEndEffector(t *testing.T) {
	s, m, _ := newTeleopServo(t, nil)
	do(t, s, startCmd(t, "grip"))
	sess := teleSession(t, m)
	fk := loopTarget(sess)
	grip := spatialmath.Compose(fk, gripPose)
	do(t, s, moveCmd(t, "grip", grip))
	if d := spatialmath.PoseDelta(loopTarget(sess), fk); d.Point().Norm() > 1e-6 || spatialmath.OrientationBetween(loopTarget(sess).Orientation(), fk.Orientation()).AxisAngles().Theta > 1e-6 {
		t.Fatalf("hold move changed the target: %v", d)
	}
	up := spatialmath.NewPose(grip.Point().Add(r3.Vector{Z: 30}), grip.Orientation())
	do(t, s, moveCmd(t, "grip", up))
	got := loopTarget(sess).Point().Sub(fk.Point())
	if got.Sub(r3.Vector{Z: 30}).Norm() > 1e-6 {
		t.Fatalf("EE moved %v, want +30 mm z", got)
	}
}

func TestTeleopOnArmItself(t *testing.T) {
	s, m, _ := newTeleopServo(t, nil)
	do(t, s, startCmd(t, "arm"))
	sess := teleSession(t, m)
	fk := loopTarget(sess)
	p := spatialmath.NewPose(fk.Point().Add(r3.Vector{X: 5}), fk.Orientation())
	do(t, s, moveCmd(t, "arm", p))
	if loopTarget(sess).Point().Sub(p.Point()).Norm() > 1e-6 {
		t.Fatalf("target = %v, want %v", loopTarget(sess).Point(), p.Point())
	}
}

func TestTeleopErrors(t *testing.T) {
	s, _, _ := newTeleopServo(t, nil)
	ctx := context.Background()
	_, err := s.DoCommand(ctx, moveCmd(t, "ghost", spatialmath.NewZeroPose()))
	wantErr(t, err, `component "ghost" not registered in teleop pipeline`)
	_, err = s.DoCommand(ctx, startCmd(t, "nowhere"))
	wantErr(t, err, "not one of this service's arms")
	do(t, s, startCmd(t, "arm"))
	bad := moveCmd(t, "arm", spatialmath.NewZeroPose())
	b, _ := protojson.Marshal(&commonpb.PoseInFrame{ReferenceFrame: "cam", Pose: &commonpb.Pose{}})
	bad["teleop_move"] = string(b)
	_, err = s.DoCommand(ctx, bad)
	wantErr(t, err, "reference_frame")
	if _, err := s.DoCommand(ctx, map[string]interface{}{"teleop_stop": true}); err == nil {
		t.Fatal("teleop_stop without component_name: want error")
	}
}

// pump ticks sess on the fake clock until it ends, for operations that wait on the loop (stop).
func pump(sess *session, clk *fakeClock) {
	go func() {
		for dt := 1 / sess.loop.cfg.RateHz; !sess.ended(); {
			clk.Advance(time.Duration(dt * float64(time.Second)))
			if err := sess.loop.tick(context.Background(), dt); errors.Is(err, errRampDone) {
				sess.finish(sess.loop.stopErr)
				return
			}
			sess.loop.OnTick(sess.loop.Status())
			time.Sleep(time.Millisecond)
		}
	}()
}

func TestTeleopStopThenMoveAndRestart(t *testing.T) {
	s, m, clk := newTeleopServo(t, map[string]interface{}{"ramp_ms": 40.0})
	do(t, s, startCmd(t, "arm"))
	first := teleSession(t, m)
	pump(first, clk)
	do(t, s, startCmd(t, "arm")) // replaces
	second := teleSession(t, m)
	if !first.ended() || second == first || second.ended() {
		t.Fatal("second start should end the first session and run a new one")
	}
	pump(second, clk)
	do(t, s, map[string]interface{}{"teleop_stop": true, "component_name": "arm"})
	if !second.ended() {
		t.Fatal("teleop_stop should wait for the session to end")
	}
	_, err := s.DoCommand(context.Background(), moveCmd(t, "arm", spatialmath.NewZeroPose()))
	wantErr(t, err, "not registered in teleop pipeline")
	do(t, s, map[string]interface{}{"teleop_stop": true, "component_name": "arm"}) // unknown: not an error
	if st := teleStatus(t, s); st["error"] != "" {
		t.Fatalf("explicit stop reported an error: %v", st["error"])
	}
}

func TestTeleopBusyWithPlainServoSession(t *testing.T) {
	s, _, _ := newTeleopServo(t, nil)
	do(t, s, map[string]interface{}{"command": "servo_start", "component_name": "arm", "frame": "arm_origin"})
	_, err := s.DoCommand(context.Background(), startCmd(t, "arm"))
	wantErr(t, err, "busy")
}

func TestTeleopStatusIsTheDeadman(t *testing.T) {
	s, m, clk := newTeleopServo(t, map[string]interface{}{"timeout_ms": 100.0, "ramp_ms": 100.0})
	do(t, s, startCmd(t, "grip"))
	sess := teleSession(t, m)
	for i := 0; i < 25; i++ { // 500 ms of ticks with a status poll each: well past timeout_ms
		step(t, sess, clk, 1)
		if st := teleStatus(t, s); st["running"] != true || st["error"] != "" {
			t.Fatalf("poll %d: %v", i, st)
		}
	}
	for i := 0; i < 20 && !sess.ended(); i++ {
		step(t, sess, clk, 1)
	}
	if got := sess.status().State; got != StateStoppedTimeout {
		t.Fatalf("state = %s, want stopped_timeout", got)
	}
	st := teleStatus(t, s)
	if e, _ := st["error"].(string); e == "" || st["running"] != false {
		t.Fatalf("status after timeout = %v, want a non-empty error", st)
	}
	if st := teleStatus(t, s); st["error"] != "" {
		t.Fatalf("error repeated: %v", st["error"])
	}
	_, err := s.DoCommand(context.Background(), moveCmd(t, "grip", spatialmath.NewZeroPose()))
	wantErr(t, err, "not registered in teleop pipeline")
}

func TestTeleopConfigDefaultsReachSession(t *testing.T) {
	s, m, _ := newTeleopServo(t, map[string]interface{}{"timeout_ms": 321.0, "limits": map[string]interface{}{"linear_mm_s": 42.0}})
	do(t, s, startCmd(t, "arm"))
	sess := teleSession(t, m)
	if sess.timeout.Milliseconds() != 321 {
		t.Fatalf("timeout = %v", sess.timeout)
	}
	st := teleStatus(t, s)
	lim := st["sessions"].(map[string]interface{})["arm"].(map[string]interface{})["limits"].(map[string]interface{})
	if lim["linear_mm_s"] != 42.0 {
		t.Fatalf("limits = %v", lim)
	}
	if len(s.teleopConf) != 2 { // not mutated by the start
		t.Fatalf("teleopConf mutated: %v", s.teleopConf)
	}
}

func TestGetPoseTransforms(t *testing.T) {
	s, _, _ := newTeleopServo(t, nil)
	pif, err := s.GetPose(context.Background(), "grip", referenceframe.World, nil, nil)
	if err != nil {
		t.Fatal(err)
	}
	zero, _ := ur5eModel().Transform(make([]referenceframe.Input, 6))
	want := spatialmath.Compose(zero, gripPose).Point()
	if pif.Pose().Point().Sub(want).Norm() > 1e-6 || pif.Parent() != referenceframe.World {
		t.Fatalf("pose = %v in %s, want %v", pif.Pose().Point(), pif.Parent(), want)
	}
	if _, err := s.GetPose(context.Background(), "grip", "", nil, nil); err != nil { // empty destination is world
		t.Fatal(err)
	}
}

func TestTeleopStatusIdleCap(t *testing.T) {
	s, m, clk := newTeleopServo(t, map[string]interface{}{"timeout_ms": 100.0, "ramp_ms": 100.0, "teleop_idle_ms": 300.0})
	do(t, s, startCmd(t, "arm"))
	sess := teleSession(t, m)
	fk := loopTarget(sess)
	for i := 0; i < 10; i++ { // 200 ms: within the cap, polling alone keeps it alive past timeout_ms
		step(t, sess, clk, 1)
		teleStatus(t, s)
	}
	do(t, s, moveCmd(t, "arm", fk)) // a real move restarts the idle clock
	for i := 0; i < 14; i++ {       // 280 ms since the move
		step(t, sess, clk, 1)
		teleStatus(t, s)
	}
	if sess.ended() {
		t.Fatal("session ended inside the idle cap")
	}
	var reported string
	for i := 0; i < 30 && reported == ""; i++ { // past the cap polling no longer helps
		step(t, sess, clk, 1)
		reported = teleStatus(t, s)["error"].(string)
	}
	if got := sess.status().State; got != StateStoppedTimeout {
		t.Fatalf("state = %s, want stopped_timeout", got)
	}
	if !strings.Contains(reported, "arm: stopped_timeout") {
		t.Fatalf("error = %q", reported)
	}
}

func TestTeleopIdleMSMustBePositiveAndIsNotForwarded(t *testing.T) {
	s, _, _ := newTeleopServo(t, map[string]interface{}{"teleop_idle_ms": 0.0})
	_, err := s.DoCommand(context.Background(), startCmd(t, "arm"))
	wantErr(t, err, "teleop_idle_ms")
	s, m, _ := newTeleopServo(t, map[string]interface{}{"teleop_idle_ms": 5000.0})
	do(t, s, startCmd(t, "arm"))
	if got := teleSession(t, m).timeout; got != time.Second {
		t.Fatalf("default timeout = %v, want 1s", got)
	}
	if _, ok := s.teleopConf["teleop_idle_ms"]; !ok {
		t.Fatal("teleopConf was mutated")
	}
}

func TestTeleopStopWaitsForInFlightStart(t *testing.T) {
	s, _, _ := newTeleopServo(t, nil)
	s.startMu.Lock() // an in-flight start
	done := make(chan struct{})
	go func() {
		defer close(done)
		do(t, s, map[string]interface{}{"teleop_stop": true, "component_name": "arm"})
	}()
	select {
	case <-done:
		t.Fatal("teleop_stop returned while a start was in flight")
	case <-time.After(50 * time.Millisecond):
	}
	s.startMu.Unlock()
	<-done
}

func TestTeleopSwitchingComponentOnOneArmReplaces(t *testing.T) {
	s, m, clk := newTeleopServo(t, map[string]interface{}{"ramp_ms": 40.0})
	do(t, s, startCmd(t, "arm"))
	first := teleSession(t, m)
	pump(first, clk)
	do(t, s, startCmd(t, "grip"))
	if !first.ended() || teleSession(t, m) == first {
		t.Fatal("starting grip should replace the arm's teleop session")
	}
	if st := teleStatus(t, s); len(st["components"].([]interface{})) != 1 {
		t.Fatalf("components = %v", st["components"])
	}
}

func TestTeleopRejectsMovableIntermediateFrame(t *testing.T) {
	s, _, _ := newTeleopServo(t, nil)
	s.m.fs = movableFS{teleopFS{s.m.fs.(teleopFS).fakeFS}}
	_, err := s.DoCommand(context.Background(), startCmd(t, "tip"))
	wantErr(t, err, "movable")
	// the component's own DoF (gripper jaws) are fine: only frames between it and the arm must be rigid
	// (movableFS's TransformPose doesn't know "slide", so only armOf's verdict is checked here)
	if arm, err := s.armOf(context.Background(), "slide"); err != nil || arm != "arm" {
		t.Fatalf("armOf(slide) = %q, %v", arm, err)
	}
}

// movableFS hangs "tip" off a movable "slide" frame on the arm.
type movableFS struct{ teleopFS }

func (f movableFS) FrameSystemConfig(ctx context.Context) (*framesystem.Config, error) {
	cfg, _ := f.teleopFS.FrameSystemConfig(ctx)
	slide, _ := kinematics.ModelFromName(kinematics.UR5e, "slide")
	cfg.Parts = append(cfg.Parts,
		&referenceframe.FrameSystemPart{FrameConfig: referenceframe.NewLinkInFrame("arm", spatialmath.NewZeroPose(), "slide", nil), ModelFrame: slide},
		&referenceframe.FrameSystemPart{FrameConfig: referenceframe.NewLinkInFrame("slide", spatialmath.NewZeroPose(), "tip", nil)})
	return cfg, nil
}

func TestTeleopFaultedSessionReportedInStatus(t *testing.T) {
	sim := &simIO{q: slices.Clone(home), sendErr: errors.New("bus error")}
	m := newServoManager(map[string]arm.Arm{"arm": fakeUR5e(t)}, teleopFS{&fakeFS{twb: spatialmath.NewZeroPose()}}, logging.NewTestLogger(t))
	m.newIO = func(arm.Arm, float64) JointIO { return sim }
	s := testServo(t, m)
	do(t, s, startCmd(t, "arm"))
	sess := teleSession(t, m)
	select {
	case <-sess.done:
	case <-time.After(2 * time.Second):
		t.Fatal("session did not fault")
	}
	e := teleStatus(t, s)["error"].(string)
	if !strings.HasPrefix(e, "arm: faulted: ") || !strings.Contains(e, "bus error") {
		t.Fatalf("error = %q, want \"arm: faulted: ...bus error\"", e)
	}
	if teleStatus(t, s)["error"] != "" {
		t.Fatal("error repeated")
	}
}

func TestTeleopErrorsAccumulate(t *testing.T) {
	s, m, clk := newTeleopServo(t, map[string]interface{}{"timeout_ms": 100.0, "ramp_ms": 100.0})
	do(t, s, startCmd(t, "arm"))
	sess := teleSession(t, m)
	for i := 0; i < 30 && !sess.ended(); i++ {
		step(t, sess, clk, 1)
	}
	s.tmu.Lock()
	s.teleop["other"] = s.teleop["arm"] // a second ended entry
	s.tmu.Unlock()
	if e := teleStatus(t, s)["error"].(string); !strings.Contains(e, "; ") {
		t.Fatalf("error = %q, want both reasons joined", e)
	}
}

func TestSessionTouchDoesNotReviveStopping(t *testing.T) {
	m, _, clk := newTestManager(t, spatialmath.NewZeroPose())
	sess, _ := mustStart(t, m, testSessionCfg(ModePose))
	sess.beginStop(StateStopped)
	clk.Advance(time.Second)
	before := sess.lastCmd
	sess.touch()
	if !sess.lastCmd.Equal(before) {
		t.Fatal("touch refreshed a stopping session")
	}
}
