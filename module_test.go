package diffik

import (
	"context"
	"math"
	"slices"
	"strings"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	motion "go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/spatialmath"
	"google.golang.org/protobuf/types/known/structpb"
)

func testServo(t *testing.T, m *servoManager) *diffIkServo {
	t.Helper()
	return newServo(resource.NewName(motion.API, "servo"), logging.NewTestLogger(t), m)
}

func do(t *testing.T, s *diffIkServo, cmd map[string]interface{}) map[string]interface{} {
	t.Helper()
	r, err := s.DoCommand(context.Background(), cmd)
	if err != nil {
		t.Fatalf("%v: %v", cmd["command"], err)
	}
	if _, err := structpb.NewStruct(r); err != nil {
		t.Fatalf("%v response not structpb-convertible: %v", cmd["command"], err)
	}
	return r
}

func TestValidate(t *testing.T) {
	deps, _, err := Config{Arms: []string{"a", "b"}}.Validate("services.0")
	if err != nil || !slices.Equal(deps, []string{"a", "b"}) {
		t.Fatalf("deps = %v err = %v", deps, err)
	}
	for _, arms := range [][]string{nil, {"a", "a"}, {""}} {
		if _, _, err := (Config{Arms: arms}).Validate("services.0"); err == nil {
			t.Errorf("arms %q: want error", arms)
		}
	}
}

func TestServoStartDefaultsAndScalarJointLimit(t *testing.T) {
	m, _, clk := newTestManager(t, spatialmath.NewZeroPose())
	s := testServo(t, m)
	r := do(t, s, map[string]interface{}{"command": "servo_start", "component_name": "arm",
		"frame": "arm_origin", "limits": map[string]interface{}{"joint_deg_s": 60.0}})
	id, ok := r["session_id"].(string)
	if !ok || id == "" {
		t.Fatalf("session_id = %v", r["session_id"])
	}
	fk, _ := ur5e(t).Transform(home)
	pm := r["pose"].(map[string]interface{})
	if math.Abs(pm["x"].(float64)-fk.Point().X) > 1e-6 {
		t.Fatalf("pose x = %v, want %v", pm["x"], fk.Point().X)
	}
	sess, err := m.get(id)
	if err != nil {
		t.Fatal(err)
	}
	step(t, sess, clk, 1) // manual manager: one tick so status has a state
	st := do(t, s, map[string]interface{}{"command": "servo_status", "session_id": id})
	lim := st["limits"].(map[string]interface{})
	js := lim["joint_deg_s"].([]interface{})
	if len(js) != 6 || js[0].(float64) != 60 || lim["linear_mm_s"].(float64) != 100 || lim["angular_deg_s"].(float64) != 45 {
		t.Fatalf("limits = %v, want 6×60, 100, 45", lim)
	}
	if st["state"] != "converged" {
		t.Fatalf("state = %v", st["state"])
	}
}

func TestDoCommandErrors(t *testing.T) {
	m, _, _ := newTestManager(t, spatialmath.NewZeroPose())
	s := testServo(t, m)
	start := func(kv ...interface{}) map[string]interface{} {
		c := map[string]interface{}{"command": "servo_start", "component_name": "arm", "frame": "arm_origin"}
		for i := 0; i < len(kv); i += 2 {
			c[kv[i].(string)] = kv[i+1]
		}
		return c
	}
	for _, tc := range []struct {
		cmd  map[string]interface{}
		want string
	}{
		{map[string]interface{}{}, "command is required"},
		{map[string]interface{}{"command": "servo_fly"}, "unknown command"},
		{map[string]interface{}{"command": "servo_start"}, "component_name is required"},
		{start("component_name", "nope"), "not in this service's arms"},
		{start("frame", "arm"), "unsupported frame"},
		{start("mode", "fly"), "unsupported mode"},
		{start("timeout_ms", 0.0), "timeout_ms"},
		{start("timeout_ms", 20000.0), "timeout_ms"},
		{start("ramp_ms", -1.0), "ramp_ms"},
		{start("ramp_ms", 6000.0), "ramp_ms"},
		{start("rate_hz", 2000.0), "rate_hz"},
		{start("limits", map[string]interface{}{"joint_deg_s": []interface{}{math.NaN()}}), "joint_deg_s"},
		{start("limits", map[string]interface{}{"joint_deg_s2": "fast"}), "joint_deg_s2"},
		{start("task_weights", []interface{}{math.NaN(), 1.0, 1.0, 1.0, 1.0, 1.0}), "task_weights"},
		{start("rate_hz", math.NaN()), "rate_hz"},
		{start("rate_hz", "fast"), "rate_hz"},
		{start("point_velocities", "no"), "point_velocities"},
		{start("limits", map[string]interface{}{"linear_mm_s": -5.0}), "linear_mm_s"},
		{start("limits", "slow"), "limits"},
		{start("task_weights", []interface{}{1.0, 1.0, 1.0, 1.0, 1.0}), "task_weights"},
		{start("task_weights", []interface{}{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}), "task_weights"},
		{start("solver", "osqp"), "unsupported solver"},
		{start("posture", map[string]interface{}{}), `posture needs "solver": "qp"`},
		{start("solver", "qp", "posture", "x"), "posture must be an object"},
		{start("solver", "qp", "posture", map[string]interface{}{"weight": 0.0}), "posture.weight must be in (0, 0.01]"},
		{start("solver", "qp", "posture", map[string]interface{}{"weight": 0.1}), "posture.weight must be in (0, 0.01]"},
		{start("solver", "qp", "posture", map[string]interface{}{"gain": -1.0}), "posture.gain must be > 0"},
		{start("solver", "qp", "posture", map[string]interface{}{"joints_deg": []interface{}{1.0}}), "posture.joints_deg needs"},
		{start("solver", "qp", "posture", map[string]interface{}{"joints_deg": []interface{}{}}), "posture.joints_deg needs"},
		{start("solver", "qp", "posture", map[string]interface{}{"weigth": 1.0}), "unknown posture key"},
		{map[string]interface{}{"command": "servo_status"}, "session_id is required"},
		{map[string]interface{}{"command": "servo_status", "session_id": "nope"}, "unknown session_id"},
	} {
		if _, err := s.DoCommand(context.Background(), tc.cmd); err == nil || !strings.Contains(err.Error(), tc.want) {
			t.Errorf("%v: err = %v, want containing %q", tc.cmd, err, tc.want)
		}
	}

	id := do(t, s, start())["session_id"].(string)
	pose := func(kv ...interface{}) map[string]interface{} {
		p := map[string]interface{}{"x": 100.0, "y": 400.0, "z": 300.0, "o_x": 0.0, "o_y": 0.0, "o_z": 1.0, "theta": 0.0}
		for i := 0; i < len(kv); i += 2 {
			p[kv[i].(string)] = kv[i+1]
		}
		return p
	}
	for _, tc := range []struct {
		cmd  map[string]interface{}
		want string
	}{
		{start(), "busy"},
		{map[string]interface{}{"command": "servo_target", "session_id": id}, "pose is required"},
		{map[string]interface{}{"command": "servo_target", "session_id": id, "pose": pose("o_z", 0.0)}, "orientation"},
		{map[string]interface{}{"command": "servo_target", "session_id": id, "pose": pose("o_x", 1e200, "o_y", 1e200, "o_z", 1e200)}, "orientation"},
		{map[string]interface{}{"command": "servo_target", "session_id": id, "pose": pose("x", "a")}, "x"},
		{map[string]interface{}{"command": "servo_target", "session_id": id, "pose": pose("y", math.Inf(1))}, "y"},
		{map[string]interface{}{"command": "servo_twist", "session_id": id,
			"linear": []interface{}{0.0, 0.0, 0.0}, "angular": []interface{}{0.0, 0.0, 0.0}}, "wrong mode"},
		{map[string]interface{}{"command": "servo_twist", "session_id": id, "linear": []interface{}{0.0}}, "linear"},
	} {
		if _, err := s.DoCommand(context.Background(), tc.cmd); err == nil || !strings.Contains(err.Error(), tc.want) {
			t.Errorf("%v: err = %v, want containing %q", tc.cmd, err, tc.want)
		}
	}
	if _, err := s.Move(context.Background(), motion.MoveReq{ComponentName: "arm"}); err == nil || !strings.Contains(err.Error(), "busy") {
		t.Fatalf("Move on busy arm: err = %v", err)
	}
}

func TestPoseRoundTrip(t *testing.T) {
	n := 1 / math.Sqrt(3)
	in := map[string]interface{}{"x": 12.5, "y": -40.0, "z": 300.0, "o_x": n, "o_y": n, "o_z": n, "theta": 33.0}
	p, err := parsePose(in)
	if err != nil {
		t.Fatal(err)
	}
	out := poseMap(p)
	for k, v := range in {
		if math.Abs(out[k].(float64)-v.(float64)) > 1e-9 {
			t.Errorf("%s: %v → %v", k, v, out[k])
		}
	}
	// a non-unit orientation vector is normalised, not rejected
	in["o_x"], in["o_y"], in["o_z"] = 0.0, 0.0, 2.0
	if p, err = parsePose(in); err != nil || math.Abs(poseMap(p)["o_z"].(float64)-1) > 1e-9 {
		t.Fatalf("non-unit vector: %v %v", poseMap(p), err)
	}
}

func TestServoTwistUnitsAndStopViaDoCommandAndStopPlan(t *testing.T) {
	sim := &simIO{q: slices.Clone(home)}
	m := newServoManager(map[string]arm.Arm{"arm": fakeUR5e(t)}, &fakeFS{twb: spatialmath.NewZeroPose()}, logging.NewTestLogger(t))
	m.newIO = func(arm.Arm, float64) JointIO { return sim }
	s := testServo(t, m)
	id := do(t, s, map[string]interface{}{"command": "servo_start", "component_name": "arm",
		"frame": "arm_origin", "mode": "twist"})["session_id"].(string)
	// unit conversion at the boundary: limits are 100 mm/s and 45 °/s
	for _, tc := range []struct {
		lin, ang []interface{}
		ok       bool
	}{
		{[]interface{}{99.0, 0.0, 0.0}, []interface{}{0.0, 0.0, 0.0}, true},
		{[]interface{}{101.0, 0.0, 0.0}, []interface{}{0.0, 0.0, 0.0}, false},
		{[]interface{}{0.0, 0.0, 0.0}, []interface{}{0.0, 0.0, 44.0}, true},
		{[]interface{}{0.0, 0.0, 0.0}, []interface{}{0.0, 0.0, 46.0}, false},
	} {
		_, err := s.DoCommand(context.Background(), map[string]interface{}{"command": "servo_twist",
			"session_id": id, "linear": tc.lin, "angular": tc.ang})
		if (err == nil) != tc.ok {
			t.Fatalf("twist %v %v: err = %v, want ok = %v", tc.lin, tc.ang, err, tc.ok)
		}
	}
	if _, err := s.DoCommand(context.Background(), map[string]interface{}{"command": "servo_twist",
		"session_id": id, "linear": []interface{}{math.NaN(), 0.0, 0.0}, "angular": []interface{}{0.0, 0.0, 0.0}}); err == nil || !strings.Contains(err.Error(), "linear") {
		t.Fatalf("NaN twist err = %v", err)
	}
	do(t, s, map[string]interface{}{"command": "servo_twist", "session_id": id,
		"linear": []interface{}{20.0, 0.0, 0.0}, "angular": []interface{}{0.0, 0.0, 0.0}}) // mm/s
	time.Sleep(300 * time.Millisecond)
	fin := do(t, s, map[string]interface{}{"command": "servo_stop", "session_id": id})
	if fin["state"] != "stopped" || !sim.stopped {
		t.Fatalf("servo_stop state = %v stopped = %v", fin["state"], sim.stopped)
	}
	_, err := s.DoCommand(context.Background(), map[string]interface{}{"command": "servo_stop", "session_id": id})
	if err == nil || !strings.Contains(err.Error(), "session ended: stopped") {
		t.Fatalf("second stop err = %v", err)
	}

	sim.stopped = false
	id = do(t, s, map[string]interface{}{"command": "servo_start", "component_name": "arm",
		"frame": "arm_origin"})["session_id"].(string)
	if err := s.StopPlan(context.Background(), motion.StopPlanReq{ComponentName: "arm"}); err != nil {
		t.Fatal(err)
	}
	if st := do(t, s, map[string]interface{}{"command": "servo_status", "session_id": id}); st["state"] != "stopped" || !sim.stopped {
		t.Fatalf("after StopPlan state = %v stopped = %v", st["state"], sim.stopped)
	}
	if err := s.Close(context.Background()); err != nil {
		t.Fatal(err)
	}
}

func TestServoStartWorldFrameIntsAndLists(t *testing.T) {
	twb := spatialmath.NewPose(r3.Vector{X: 100}, spatialmath.NewZeroOrientation())
	m, _, clk := newTestManager(t, twb)
	s := testServo(t, m)
	js := []interface{}{10.0, 20.0, 30.0, 40.0, 50.0, 60.0}
	r := do(t, s, map[string]interface{}{"command": "servo_start", "component_name": "arm",
		"rate_hz": 50, "timeout_ms": 1000, "limits": map[string]interface{}{"joint_deg_s": js},
		"task_weights": []interface{}{2.0, 2.0, 2.0, 2.0, 2.0, 2.0}})
	fk, _ := ur5e(t).Transform(home)
	pm := r["pose"].(map[string]interface{})
	if math.Abs(pm["x"].(float64)-(fk.Point().X+100)) > 1e-6 {
		t.Fatalf("pose x = %v, want %v", pm["x"], fk.Point().X+100)
	}
	id := r["session_id"].(string)
	sess, err := m.get(id)
	if err != nil {
		t.Fatal(err)
	}
	step(t, sess, clk, 1)
	tgt := map[string]interface{}{}
	for k, v := range pm {
		tgt[k] = v
	}
	tgt["y"] = pm["y"].(float64) + 5
	tr := do(t, s, map[string]interface{}{"command": "servo_target", "session_id": id, "pose": tgt})
	for _, k := range []string{"state", "err_mm", "err_deg"} {
		if _, ok := tr[k]; !ok {
			t.Errorf("servo_target missing %q: %v", k, tr)
		}
	}
	st := do(t, s, map[string]interface{}{"command": "servo_status", "session_id": id})
	got := st["limits"].(map[string]interface{})["joint_deg_s"].([]interface{})
	if !slices.Equal(got, js) {
		t.Fatalf("joint_deg_s = %v, want %v", got, js)
	}
}

func TestServoStartTaskWeightsAndDamping(t *testing.T) {
	m, _, clk := newTestManager(t, spatialmath.NewZeroPose())
	s := testServo(t, m)
	start := func(kv ...interface{}) (map[string]interface{}, error) {
		c := map[string]interface{}{"command": "servo_start", "component_name": "arm", "frame": "arm_origin"}
		for i := 0; i < len(kv); i += 2 {
			c[kv[i].(string)] = kv[i+1]
		}
		return s.DoCommand(context.Background(), c)
	}
	for _, tc := range []struct {
		kv   []interface{}
		want string
	}{
		{[]interface{}{"task", "fly"}, "unsupported task"},
		{[]interface{}{"task_weights", []interface{}{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}, "task_weights"},
		{[]interface{}{"task_weights", []interface{}{1.0, 1.0, 1.0, -1.0, 1.0, 1.0}}, "task_weights"},
		{[]interface{}{"limits", map[string]interface{}{"singular_eps": 0.0}}, "singular_eps"},
		{[]interface{}{"limits", map[string]interface{}{"max_damping": -1.0}}, "max_damping"},
		{[]interface{}{"limits", map[string]interface{}{"tol_mm": 0.0}}, "tol_mm"},
		{[]interface{}{"limits", map[string]interface{}{"tol_deg": -1.0}}, "tol_deg"},
	} {
		if _, err := start(tc.kv...); err == nil || !strings.Contains(err.Error(), tc.want) {
			t.Errorf("%v: err = %v, want containing %q", tc.kv, err, tc.want)
		}
	}
	r, err := start("task", "pointing", "task_weights", []interface{}{1.0, 1.0, 1.0, 0.0, 0.0, 1.0},
		"limits", map[string]interface{}{"singular_eps": 0.02, "max_damping": 0.05})
	if err != nil {
		t.Fatal(err)
	}
	sess, err := m.get(r["session_id"].(string))
	if err != nil {
		t.Fatal(err)
	}
	step(t, sess, clk, 1)
	st := do(t, s, map[string]interface{}{"command": "servo_status", "session_id": r["session_id"]})
	lim := st["limits"].(map[string]interface{})
	if st["task"] != "pointing" || lim["singular_eps"] != 0.02 || lim["max_damping"] != 0.05 {
		t.Fatalf("status task=%v limits=%v", st["task"], lim)
	}
	if _, ok := st["pointing_residual_deg"].(float64); !ok {
		t.Fatalf("pointing_residual_deg missing: %v", st)
	}
}

func TestServoTwistRollIgnoredAndTaskStatus(t *testing.T) {
	m, _, clk := newTestManager(t, spatialmath.NewZeroPose())
	s := testServo(t, m)
	id := do(t, s, map[string]interface{}{"command": "servo_start", "component_name": "arm",
		"frame": "arm_origin", "mode": "twist", "task": "pointing"})["session_id"].(string)
	sess, err := m.get(id)
	if err != nil {
		t.Fatal(err)
	}
	step(t, sess, clk, 1)
	a := sess.loop.Status().ToolAxis.Mul(10) // deg/s about the tool axis
	r := do(t, s, map[string]interface{}{"command": "servo_twist", "session_id": id,
		"linear": []interface{}{0.0, 0.0, 0.0}, "angular": []interface{}{a.X, a.Y, a.Z}})
	if r["roll_ignored"] != true {
		t.Fatalf("roll twist response %v, want roll_ignored", r)
	}
	r = do(t, s, map[string]interface{}{"command": "servo_twist", "session_id": id,
		"linear": []interface{}{5.0, 0.0, 0.0}, "angular": []interface{}{0.0, 0.0, 0.0}})
	if _, ok := r["roll_ignored"]; ok {
		t.Fatalf("linear twist response %v, want no roll_ignored", r)
	}

	m, _, clk = newTestManager(t, spatialmath.NewZeroPose())
	s = testServo(t, m)
	id = do(t, s, map[string]interface{}{"command": "servo_start", "component_name": "arm",
		"frame": "arm_origin", "task": "pose"})["session_id"].(string)
	sess, err = m.get(id)
	if err != nil {
		t.Fatal(err)
	}
	step(t, sess, clk, 1)
	st := do(t, s, map[string]interface{}{"command": "servo_status", "session_id": id})
	if _, ok := st["pointing_residual_deg"]; st["task"] != "pose" || ok {
		t.Fatalf("pose status task=%v pointing_residual_deg present=%v", st["task"], ok)
	}
}

func TestServoStartTolLimits(t *testing.T) {
	m, _, clk := newTestManager(t, spatialmath.NewZeroPose())
	s := testServo(t, m)
	r := do(t, s, map[string]interface{}{"command": "servo_start", "component_name": "arm",
		"frame": "arm_origin", "limits": map[string]interface{}{"tol_mm": 3.5, "tol_deg": 4.0}})
	sess, err := m.get(r["session_id"].(string))
	if err != nil {
		t.Fatal(err)
	}
	step(t, sess, clk, 1)
	st := do(t, s, map[string]interface{}{"command": "servo_status", "session_id": r["session_id"]})
	lim := st["limits"].(map[string]interface{})
	if lim["tol_mm"] != 3.5 || lim["tol_deg"] != 4.0 {
		t.Fatalf("limits = %v, want tol_mm 3.5 tol_deg 4", lim)
	}
}

func TestServoFramedTargetReplyAndFilterConfig(t *testing.T) {
	m, _, _ := newFramedTestManager(t, camPose)
	m.now, m.manual = time.Now, false // a real Run loop, so Close can stop it
	s := testServo(t, m)
	filt := func(alpha float64) map[string]interface{} {
		return map[string]interface{}{"ema_alpha": alpha, "hold_mm": 1.0, "hold_deg": 0.5, "dwell_ms": 200.0,
			"floor": map[string]interface{}{"min_z_mm": -10.0, "tip_mm": 105.0, "margin_mm": 15.0}, "feedforward": false}
	}
	start := func(alpha float64) (map[string]interface{}, error) {
		return s.DoCommand(context.Background(), map[string]interface{}{"command": "servo_start", "component_name": "arm",
			"frame": "arm_origin", "frames": true, "target_filter": filt(alpha)})
	}
	if _, err := start(0); err == nil || !strings.Contains(err.Error(), "ema_alpha") {
		t.Fatalf("ema_alpha 0 err = %v", err)
	}
	r, err := start(0.5)
	if err != nil {
		t.Fatal(err)
	}
	id := r["session_id"].(string)
	pm := map[string]interface{}{"x": 5.0, "y": 6.0, "z": 7.0, "o_x": 0.0, "o_y": 0.0, "o_z": 1.0, "theta": 10.0}
	rep := do(t, s, map[string]interface{}{"command": "servo_target", "session_id": id, "pose": pm, "frame": "cam"})
	for _, k := range []string{"state", "err_mm", "err_deg", "target", "target_raw", "reach_deg", "pointing_residual_deg", "clamped"} {
		if _, ok := rep[k]; !ok {
			t.Errorf("servo_target reply lacks %q: %v", k, rep)
		}
	}
	st := do(t, s, map[string]interface{}{"command": "servo_status", "session_id": id})
	tf, ok := st["target_filter"].(map[string]interface{})
	if !ok || tf["ema_alpha"] != 0.5 || tf["feedforward"] != false {
		t.Errorf("servo_status target_filter = %v", st["target_filter"])
	}
	if _, ok := st["clamps"]; !ok {
		t.Errorf("servo_status lacks clamps: %v", st)
	}
	if _, err := s.DoCommand(context.Background(), map[string]interface{}{"command": "servo_target", "session_id": id,
		"pose": pm, "frame": "nosuchcam"}); err == nil {
		t.Fatal("unknown frame accepted")
	}
	if err := s.Close(context.Background()); err != nil {
		t.Fatal(err)
	}
}

func TestParseTargetFilter(t *testing.T) {
	def := DefaultTargetFilter()
	for _, tc := range []struct {
		name, errSub string
		in           interface{}
		check        func(TargetFilter) bool
	}{
		{"nil", "", nil, func(f TargetFilter) bool { return f == def }},
		{"empty", "", map[string]interface{}{}, func(f TargetFilter) bool { return f == def }},
		{"dwell", "", map[string]interface{}{"dwell_ms": 200}, func(f TargetFilter) bool { return f.Dwell == 200*time.Millisecond }},
		{"bogus", "unknown", map[string]interface{}{"bogus": 1}, nil},
		{"floor key", "unknown", map[string]interface{}{"floor": map[string]interface{}{"z": 1}}, nil},
		{"floor str", "floor", map[string]interface{}{"floor": "x"}, nil},
		{"floor no min_z", "min_z_mm is required", map[string]interface{}{"floor": map[string]interface{}{"tip_mm": 1}}, nil},
		{"ff str", "feedforward", map[string]interface{}{"feedforward": "no"}, nil},
	} {
		f, err := parseTargetFilter(tc.in)
		if tc.errSub != "" {
			if err == nil || !strings.Contains(err.Error(), tc.errSub) {
				t.Errorf("%s: err = %v, want containing %q", tc.name, err, tc.errSub)
			}
		} else if err != nil || !tc.check(f) {
			t.Errorf("%s: got %+v, %v", tc.name, f, err)
		}
	}
}

func TestStatusSolverFields(t *testing.T) {
	for _, solver := range []string{"dls", "qp"} {
		m, _, clk := newTestManager(t, spatialmath.NewZeroPose())
		s := testServo(t, m)
		r := do(t, s, map[string]interface{}{"command": "servo_start", "component_name": "arm",
			"frame": "arm_origin", "solver": solver})
		id := r["session_id"].(string)
		sess, err := m.get(id)
		if err != nil {
			t.Fatal(err)
		}
		step(t, sess, clk, 1)
		st := do(t, s, map[string]interface{}{"command": "servo_status", "session_id": id})
		if st["solver"] != solver {
			t.Errorf("%s: solver = %v", solver, st["solver"])
		}
		if q, ok := st["q_cmd_deg"].([]interface{}); !ok || len(q) != 6 {
			t.Errorf("%s: q_cmd_deg = %v", solver, st["q_cmd_deg"])
		}
		if _, ok := st["peak_accel_ratio"]; !ok {
			t.Errorf("%s: no peak_accel_ratio", solver)
		}
		for _, k := range []string{"qp_active", "qp_iters", "qp_fallbacks"} {
			if _, ok := st[k]; ok != (solver == "qp") {
				t.Errorf("%s: %s present = %v", solver, k, ok)
			}
		}
	}
}

// collManager is newTestManager with a frame system holding the arm and a table box beside it.
func collManager(t *testing.T) (*servoManager, *fakeFS) {
	t.Helper()
	m, _, _ := newTestManager(t, spatialmath.NewZeroPose())
	box, err := spatialmath.NewBox(spatialmath.NewZeroPose(), r3.Vector{X: 600, Y: 600, Z: 20}, "")
	if err != nil {
		t.Fatal(err)
	}
	f := m.fs.(*fakeFS)
	f.parts = []*referenceframe.FrameSystemPart{
		{FrameConfig: referenceframe.NewLinkInFrame(referenceframe.World, spatialmath.NewZeroPose(), "arm", nil), ModelFrame: ur5e(t)},
		{FrameConfig: referenceframe.NewLinkInFrame(referenceframe.World, spatialmath.NewPoseFromPoint(r3.Vector{X: 400, Z: -200}), "table", box)},
	}
	return m, f
}

func TestServoStartCollision(t *testing.T) {
	start := func(kv ...interface{}) map[string]interface{} {
		c := map[string]interface{}{"command": "servo_start", "component_name": "arm", "frame": "arm_origin", "solver": "qp"}
		for i := 0; i < len(kv); i += 2 {
			c[kv[i].(string)] = kv[i+1]
		}
		return c
	}
	obj := func(kv ...interface{}) map[string]interface{} {
		o := map[string]interface{}{}
		for i := 0; i < len(kv); i += 2 {
			o[kv[i].(string)] = kv[i+1]
		}
		return o
	}
	for _, c := range []struct {
		cmd  map[string]interface{}
		want string
	}{
		{start("solver", "dls", "world_state", obj()), `need "solver": "qp"`},
		{start("solver", "dls", "collision", true), `need "solver": "qp"`},
		{start("world_state", obj("bogus", 1.0)), "world_state"},
		{start("collision_specification", "x"), "collision_specification"},
		{start("collision_specification", []interface{}{obj("allows", []interface{}{obj("frame1", "nope", "frame2", "table")})}, "collision", true), "nope"},
		{start("collision_specification", []interface{}{obj("allows", []interface{}{obj("frame1", "nope", "frame2", "table")})}), "nope"},
		{start("collision", obj("d_min_mm", 0.0)), "d_min_mm"},
		{start("collision", obj("d_min_mm", 50.0, "d_act_mm", 50.0)), "d_act_mm"},
		{start("collision", obj("bogus", 1.0)), `unknown collision key "bogus"`},
	} {
		m, _ := collManager(t)
		_, err := testServo(t, m).DoCommand(context.Background(), c.cmd)
		if err == nil || !strings.Contains(err.Error(), c.want) {
			t.Errorf("%v: err = %v, want %q", c.cmd, err, c.want)
		}
	}

	m, f := collManager(t)
	s := testServo(t, m)
	id := do(t, s, start("collision", true))["session_id"]
	st := do(t, s, map[string]interface{}{"command": "servo_status", "session_id": id})
	if st["collision"] != "on" {
		t.Fatalf("collision = %v", st["collision"])
	}
	// +Inf (nothing in range yet) is omitted, so the key is absent until a pair is measured
	if d, ok := st["collision_min_mm"]; ok && math.IsInf(d.(float64), 0) {
		t.Fatalf("collision_min_mm = %v", d)
	}
	for _, k := range []string{"collision_pair", "collision_rows", "collision_relaxed", "collision_brakes", "collision_ignored", "collision_skipped"} {
		if _, ok := st[k]; !ok {
			t.Errorf("missing %s", k)
		}
	}
	if n := f.cfgCalls.Load(); n != 1 {
		t.Fatalf("FrameSystemConfig calls = %d, want 1", n)
	}

	// "collision": false alone is fine with dls.
	m, _ = collManager(t)
	do(t, testServo(t, m), start("solver", "dls", "collision", false))

	// Default is off, with no extra FrameSystemConfig call.
	m, f = collManager(t)
	s = testServo(t, m)
	id = do(t, s, start())["session_id"]
	st = do(t, s, map[string]interface{}{"command": "servo_status", "session_id": id})
	for k := range st {
		if strings.HasPrefix(k, "collision") {
			t.Errorf("unexpected key %s", k)
		}
	}
	if n := f.cfgCalls.Load(); n != 0 {
		t.Fatalf("FrameSystemConfig calls = %d, want 0", n)
	}
}

func TestStatusMapCollisionKeys(t *testing.T) {
	st := SessionStatus{Status: Status{Collision: "on", CollisionMinMM: math.Inf(1)}}
	m := statusMap(st)
	if _, ok := m["collision_min_mm"]; ok {
		t.Fatal("Inf collision_min_mm not omitted")
	}
	st.Collision = "off"
	m = statusMap(st)
	if _, ok := m["collision_rows"]; ok || m["collision"] != "off" {
		t.Fatalf("off status = %v", m)
	}
}

func TestValidateVisionServices(t *testing.T) {
	deps, _, err := Config{Arms: []string{"a"}, VisionServices: []string{"v1", "v2"}}.Validate("services.0")
	if err != nil || !slices.Equal(deps, []string{"a", "rdk:service:vision/v1", "rdk:service:vision/v2"}) {
		t.Fatalf("deps = %v err = %v", deps, err)
	}
	for _, vs := range [][]string{{""}, {"v", "v"}, {"a"}} {
		if _, _, err := (Config{Arms: []string{"a"}, VisionServices: vs}).Validate("p"); err == nil {
			t.Errorf("vision_services %q: want error", vs)
		}
	}
}

func liveManager(t *testing.T) (*servoManager, *fakeSrc) {
	t.Helper()
	m, _, _ := newFramedTestManager(t, camPose)
	src := &fakeSrc{props: vision.Properties{Detections3DSupported: true}}
	m.visions = map[string]LiveSource{"v": src}
	return m, src
}

func TestServoStartLiveObstacles(t *testing.T) {
	obj := func(kv ...interface{}) map[string]interface{} {
		o := map[string]interface{}{}
		for i := 0; i < len(kv); i += 2 {
			o[kv[i].(string)] = kv[i+1]
		}
		return o
	}
	good := func(kv ...interface{}) map[string]interface{} {
		l := obj("vision", "v", "camera", "cam")
		for i := 0; i < len(kv); i += 2 {
			l[kv[i].(string)] = kv[i+1]
		}
		return l
	}
	start := func(live interface{}, kv ...interface{}) map[string]interface{} {
		c := obj("command", "servo_start", "component_name", "arm", "frame", "arm_origin", "solver", "qp", "live_obstacles", live)
		for i := 0; i < len(kv); i += 2 {
			c[kv[i].(string)] = kv[i+1]
		}
		return c
	}
	vec := func(x, y, z float64) []interface{} { return []interface{}{x, y, z} }
	for _, c := range []struct {
		cmd  map[string]interface{}
		want string
	}{
		{start("x"), "live_obstacles must be an object"},
		{start(good(), "solver", "dls"), `needs "solver": "qp"`},
		{start(good(), "collision", false), "collision"},
		{start(obj("camera", "cam")), "live_obstacles.vision is required"},
		{start(obj("vision", "v")), "live_obstacles.camera is required"},
		{start(good("vision", "other")), "not in this service's vision_services"},
		{start(good("bogus", 1.0)), `unknown live_obstacles key "bogus"`},
		{start(good("rate_hz", "fast")), "live_obstacles.rate_hz"},
		{start(good("rate_hz", 100.0)), "live_obstacles.rate_hz"},
		{start(good("stale_ms", 50.0)), "live_obstacles.stale_ms"},
		{start(good("timeout_ms", 1e6)), "live_obstacles.timeout_ms"},
		{start(good("inflate_mm", -1.0)), "live_obstacles.inflate_mm"},
		{start(good("max", 65.0)), "live_obstacles.max"},
		{start(good("rate_hz", 0.0)), "live_obstacles.rate_hz must be > 0"},
		{start(good("stale_ms", 0.0)), "live_obstacles.stale_ms must be > 0"},
		{start(good("timeout_ms", 0.0)), "live_obstacles.timeout_ms must be > 0"},
		{start(good("max", 0.0)), "live_obstacles.max must be > 0"},
		{start(good("min_points", 0.0)), "live_obstacles.min_points must be > 0"},
		{start(good("max", 1e300)), "live_obstacles.max"},
		{start(good("max", 1.5)), "live_obstacles.max"},
		{start(good("min_points", -1.0)), "live_obstacles.min_points"},
		{start(good("workspace", obj("min_mm", vec(0, 0, 0)))), "workspace"},
		{start(good("workspace", obj("min_mm", vec(0, 0, 0), "max_mm", vec(1, 1, 1), "x", 1.0))), "workspace"},
		{start(good("workspace", obj("min_mm", vec(1, 1, 1), "max_mm", vec(0, 0, 0)))), "workspace"},
		{start(good("workspace", obj("min_mm", []interface{}{1.0}, "max_mm", vec(2, 2, 2)))), "live_obstacles.workspace.min_mm"},
		{start(good("extrude_to_z_mm", "low")), "live_obstacles.extrude_to_z_mm"},
		{start(good("workspace", obj("min_mm", vec(0, 0, 0), "max_mm", vec(1, 1, 1)), "extrude_to_z_mm", 5.0)), "extrude_to_z_mm"},
	} {
		m, _ := liveManager(t)
		_, err := testServo(t, m).DoCommand(context.Background(), c.cmd)
		if err == nil || !strings.Contains(err.Error(), c.want) {
			t.Errorf("%v: err = %v, want %q", c.cmd["live_obstacles"], err, c.want)
		}
	}

	m, src := liveManager(t)
	s := testServo(t, m)
	id := do(t, s, start(good("rate_hz", 20.0, "inflate_mm", 0.0, "max", 8,
		"workspace", obj("min_mm", vec(-1000, -1000, -500), "max_mm", vec(1000, 1000, 1000)))))["session_id"]
	sess, _ := m.get(id.(string))
	t.Cleanup(sess.cancel)
	if c := sess.poller.cfg; *c.InflateMM != 0 || c.Max != 8 || c.RateHz != 20 || c.WorkspaceMax.Z != 1000 || c.Camera != "cam" {
		t.Fatalf("live config = %+v", c)
	}
	if sess.poller.src != LiveSource(src) {
		t.Fatal("session did not get the vision service")
	}
	st := do(t, s, map[string]interface{}{"command": "servo_status", "session_id": id})
	if st["collision"] != "on" {
		t.Fatalf("collision = %v", st["collision"])
	}
	for _, k := range []string{"live_count", "live_age_ms", "live_updates", "live_errors", "live_self_dropped", "live_skipped",
		"live_overflow", "live_clipped", "live_backing_away", "live_pinched", "live_waiting", "live_method"} {
		if _, ok := st[k]; !ok {
			t.Errorf("missing %s", k)
		}
	}
	if st["live_method"] != "detections3d" {
		t.Fatalf("live_method = %v", st["live_method"])
	}
	if _, ok := st["live_last_error"]; ok {
		t.Fatal("live_last_error present with no error")
	}

}

func TestStatusMapLiveKeys(t *testing.T) {
	m := statusMap(SessionStatus{})
	for k := range m {
		if strings.HasPrefix(k, "live_") {
			t.Fatalf("unexpected %s", k)
		}
	}
	m = statusMap(SessionStatus{LiveMethod: "object_point_clouds", LiveLastError: "boom"})
	if m["live_last_error"] != "boom" || m["live_method"] != "object_point_clouds" {
		t.Fatalf("status = %v", m)
	}
}

func TestTeleopLiveObstacles(t *testing.T) {
	s, m, _ := newTeleopServo(t, map[string]interface{}{"solver": "qp",
		"live_obstacles": map[string]interface{}{"vision": "v", "camera": "cam"}})
	f := m.fs.(teleopFS)
	f.parts = []*referenceframe.FrameSystemPart{{FrameConfig: referenceframe.NewLinkInFrame("arm", camPose, "cam", nil)}}
	src := &fakeSrc{props: vision.Properties{Detections3DSupported: true}}
	m.visions = map[string]LiveSource{"v": src}
	do(t, s, startCmd(t, "arm"))
	sess := teleSession(t, m)
	t.Cleanup(sess.cancel)
	if sess.poller == nil || sess.poller.cfg.Camera != "cam" || sess.poller.src != LiveSource(src) {
		t.Fatalf("teleop session has no live poller: %+v", sess.poller)
	}
}

func TestServoStartGain(t *testing.T) {
	start := func(gain interface{}) (map[string]interface{}, error) {
		m, _, clk := newTestManager(t, spatialmath.NewZeroPose())
		s := testServo(t, m)
		c := map[string]interface{}{"command": "servo_start", "component_name": "arm", "frame": "arm_origin"}
		if gain != nil {
			c["gain"] = gain
		}
		r, err := s.DoCommand(context.Background(), c)
		if err != nil {
			return nil, err
		}
		sess, err := m.get(r["session_id"].(string))
		if err != nil {
			t.Fatal(err)
		}
		step(t, sess, clk, 1)
		return do(t, s, map[string]interface{}{"command": "servo_status", "session_id": r["session_id"]}), nil
	}
	for _, tc := range []struct {
		in   interface{}
		want float64
	}{{nil, 2}, {6.0, 6}} {
		st, err := start(tc.in)
		if err != nil {
			t.Fatal(err)
		}
		if g := st["limits"].(map[string]interface{})["gain"]; g != tc.want {
			t.Fatalf("gain %v: got %v, want %v", tc.in, g, tc.want)
		}
	}
	if _, err := start(0.0); err == nil || !strings.Contains(err.Error(), "gain") {
		t.Fatalf("gain 0 err = %v", err)
	}
}
