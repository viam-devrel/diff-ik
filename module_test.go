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
	"go.viam.com/rdk/resource"
	motion "go.viam.com/rdk/services/motion"
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
		{start("task_weights", []interface{}{math.NaN(), 1.0, 1.0, 1.0, 1.0, 1.0}), "task_weights"},
		{start("rate_hz", math.NaN()), "rate_hz"},
		{start("rate_hz", "fast"), "rate_hz"},
		{start("limits", map[string]interface{}{"linear_mm_s": -5.0}), "linear_mm_s"},
		{start("limits", "slow"), "limits"},
		{start("task_weights", []interface{}{1.0, 1.0, 1.0, 1.0, 1.0}), "task_weights"},
		{start("task_weights", []interface{}{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}), "task_weights"},
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
	m := newServoManager(map[string]arm.Arm{"arm": fakeUR5e(t)}, fakeFS{spatialmath.NewZeroPose()}, logging.NewTestLogger(t))
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
