package diffik

import (
	"context"
	"errors"
	"fmt"
	"maps"
	"strings"
	"time"

	commonpb "go.viam.com/api/common/v1"
	motionpb "go.viam.com/api/service/motion/v1"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"google.golang.org/protobuf/encoding/protojson"
	"google.golang.org/protobuf/proto"
)

// teleopEntry is one viam:teleop component bound to a servo session on its arm.
type teleopEntry struct {
	sess     *session
	tool     spatialmath.Pose // the arm's end effector expressed in the component's frame (rigid)
	idle     time.Duration    // teleop_status stops keeping the session alive this long after lastMove
	lastMove time.Time        // the last start or teleop_move; guarded by diffIkServo.tmu
}

// Defaults for the teleop config block: a session survives two missed 200 ms status polls, and status
// polling alone keeps it alive for at most a minute.
const (
	defaultTeleopTimeoutMS = 1000.0
	defaultTeleopIdleMS    = 60000.0
)

var teleopKeys = []string{"teleop_start", "teleop_move", "teleop_stop", "teleop_status"}

// isTeleop reports whether cmd is a viam:teleop command (they carry no "command" key).
func isTeleop(cmd map[string]interface{}) bool {
	for _, k := range teleopKeys {
		if _, ok := cmd[k]; ok {
			return true
		}
	}
	return false
}

func (s *diffIkServo) teleopCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if _, ok := cmd["teleop_start"]; ok {
		return nil, s.teleopStart(ctx, cmd["teleop_start"])
	}
	if _, ok := cmd["teleop_move"]; ok {
		return nil, s.teleopMove(cmd)
	}
	if _, ok := cmd["teleop_stop"]; ok {
		return nil, s.teleopStop(ctx, cmd)
	}
	return s.teleopStatus(), nil
}

// decodeProto reads a protojson message; viam:teleop sends it as a JSON string.
func decodeProto(v interface{}, msg proto.Message) error {
	b, ok := v.(string)
	if !ok {
		return fmt.Errorf("want a protojson string, got %T", v)
	}
	return protojson.UnmarshalOptions{DiscardUnknown: true}.Unmarshal([]byte(b), msg)
}

// armOf walks the component's parents in the frame system until it reaches one of this service's arms.
// Every frame in between must be static: the tool transform is computed once and assumed rigid.
func (s *diffIkServo) armOf(ctx context.Context, component string) (string, error) {
	cfg, err := s.m.fs.FrameSystemConfig(ctx)
	if err != nil {
		return "", fmt.Errorf("reading the frame system: %w", err)
	}
	parts := map[string]*referenceframe.FrameSystemPart{}
	for _, p := range cfg.Parts {
		parts[p.FrameConfig.Name()] = p
	}
	for cur, i := component, 0; i <= len(parts); i++ {
		if _, ok := s.m.arms[cur]; ok {
			return cur, nil
		}
		p, ok := parts[cur]
		if !ok {
			break
		}
		// the component's own DoF (e.g. gripper jaws) don't move its frame origin; frames between it and the arm must be rigid
		if cur != component && p.ModelFrame != nil && len(p.ModelFrame.DoF()) > 0 {
			return "", fmt.Errorf("frame %q between %q and its arm is movable; only rigid mounts are supported", cur, component)
		}
		cur = p.FrameConfig.Parent()
	}
	return "", fmt.Errorf("component %q is not one of this service's arms or attached to one", component)
}

func (s *diffIkServo) teleopStart(ctx context.Context, raw interface{}) error {
	var req motionpb.MoveRequest
	if err := decodeProto(raw, &req); err != nil {
		return fmt.Errorf("teleop_start: %w", err)
	}
	comp := req.GetComponentName()
	if comp == "" {
		return errors.New("teleop_start: component_name is required")
	}
	armName, err := s.armOf(ctx, comp)
	if err != nil {
		return err
	}
	tool := spatialmath.NewZeroPose()
	if comp != armName {
		pif, err := s.m.fs.TransformPose(ctx, referenceframe.NewPoseInFrame(armName, spatialmath.NewZeroPose()), comp, nil)
		if err != nil {
			return fmt.Errorf("locating %s's end effector in %s: %w", armName, comp, err)
		}
		tool = pif.Pose()
	}
	cmd := maps.Clone(s.teleopConf)
	if cmd == nil {
		cmd = map[string]interface{}{}
	}
	idleMS, err := optNum(cmd, "teleop_idle_ms", defaultTeleopIdleMS)
	if err != nil {
		return err
	}
	if !(idleMS > 0) {
		return fmt.Errorf("teleop_idle_ms must be > 0, got %v", idleMS)
	}
	delete(cmd, "teleop_idle_ms")
	if _, ok := cmd["timeout_ms"]; !ok {
		cmd["timeout_ms"] = defaultTeleopTimeoutMS
	}
	cmd["component_name"], cmd["frame"], cmd["mode"] = armName, referenceframe.World, string(ModePose)
	// ponytail: one mutex serialises all teleop starts and stops (and the replaced session's stop); per-component locks if it matters.
	s.startMu.Lock()
	defer s.startMu.Unlock()
	// Stop this component's old session and any other teleop session on the arm (a changed component name).
	cur := s.m.current(armName) // before tmu: current takes m.mu, which a servo_start holds across RPCs
	s.tmu.Lock()
	var olds []*teleopEntry
	for name, e := range s.teleop {
		if name == comp || e.sess == cur {
			olds = append(olds, e)
			delete(s.teleop, name)
		}
	}
	s.tmu.Unlock()
	for _, e := range olds {
		if !e.sess.ended() {
			_, _ = e.sess.stop(ctx) // an already-ended session is fine
		}
	}
	sess, _, err := s.startSession(ctx, cmd)
	if err != nil {
		return err
	}
	s.tmu.Lock()
	s.teleop[comp] = &teleopEntry{sess: sess, tool: tool, idle: time.Duration(idleMS * float64(time.Millisecond)), lastMove: s.m.now()}
	s.tmu.Unlock()
	return nil
}

// reap drops e once its session has ended and appends why to teleopErr. That string is pipeline-wide and
// report-once: whichever teleop_status call comes first consumes it. s.tmu must be held.
func (s *diffIkServo) reap(name string, e *teleopEntry) {
	if s.teleop[name] != e {
		return
	}
	delete(s.teleop, name)
	msg := fmt.Sprintf("%s: %s", name, strings.TrimPrefix(e.sess.endedErr().Error(), "session ended: "))
	if s.teleopErr != "" {
		msg = s.teleopErr + "; " + msg
	}
	s.teleopErr = msg
}

func (s *diffIkServo) notRegistered(name string, cause error) error {
	if cause != nil {
		return fmt.Errorf("component %q not registered in teleop pipeline: %w", name, cause)
	}
	return fmt.Errorf("component %q not registered in teleop pipeline", name)
}

func componentName(cmd map[string]interface{}) (string, error) {
	name, ok := cmd["component_name"].(string)
	if !ok || name == "" {
		return "", errors.New("component_name is required")
	}
	return name, nil
}

func (s *diffIkServo) teleopMove(cmd map[string]interface{}) error {
	name, err := componentName(cmd)
	if err != nil {
		return err
	}
	var pif commonpb.PoseInFrame
	if err := decodeProto(cmd["teleop_move"], &pif); err != nil {
		return fmt.Errorf("teleop_move: %w", err)
	}
	if f := pif.GetReferenceFrame(); f != "" && f != referenceframe.World {
		return fmt.Errorf("teleop_move: reference_frame must be %q, got %q", referenceframe.World, f)
	}
	s.tmu.Lock()
	e := s.teleop[name]
	if e == nil {
		s.tmu.Unlock()
		return s.notRegistered(name, nil)
	}
	if e.sess.ended() {
		s.reap(name, e)
		s.tmu.Unlock()
		return s.notRegistered(name, e.sess.endedErr())
	}
	e.lastMove = s.m.now()
	s.tmu.Unlock()
	p := referenceframe.ProtobufToPoseInFrame(&pif).Pose()
	if _, err := e.sess.target(spatialmath.Compose(p, e.tool), "", true); err != nil {
		return s.notRegistered(name, err)
	}
	return nil
}

// teleopStop ramp-stops the named component's session, or every teleop session when no component_name is
// given (as builtin does).
func (s *diffIkServo) teleopStop(ctx context.Context, cmd map[string]interface{}) error {
	name, _ := cmd["component_name"].(string)
	s.startMu.Lock() // so a stop can't slip between an in-flight start and its registration
	defer s.startMu.Unlock()
	s.tmu.Lock()
	var stops []*teleopEntry
	for n, e := range s.teleop {
		if name == "" || n == name {
			stops = append(stops, e)
			delete(s.teleop, n)
		}
	}
	s.tmu.Unlock()
	var errs []error
	for _, e := range stops {
		if !e.sess.ended() {
			_, err := e.sess.stop(ctx)
			errs = append(errs, err)
		}
	}
	return errors.Join(errs...)
}

// teleopStatus reports the teleop sessions. Polling it is the dead-man: viam:teleop sends no moves while the
// leader is still, so each call refreshes a live session's dead-man, but only while its last start or
// teleop_move is at most teleop_idle_ms old. If the module dies the polling stops, and if it keeps polling
// with no moves past the cap, timeout_ms ends the session (stopped_timeout), which the next call reports.
func (s *diffIkServo) teleopStatus() map[string]interface{} {
	s.tmu.Lock()
	defer s.tmu.Unlock()
	comps, sessions := []interface{}{}, map[string]interface{}{}
	var planMS float64
	for name, e := range s.teleop {
		if e.sess.ended() {
			s.reap(name, e)
			continue
		}
		if s.m.now().Sub(e.lastMove) <= e.idle {
			e.sess.touch()
		}
		st := e.sess.status()
		comps = append(comps, name)
		sessions[name] = statusMap(st)
		planMS = max(planMS, float64(st.LastTick)/float64(time.Millisecond))
	}
	errStr := s.teleopErr
	s.teleopErr = ""
	return map[string]interface{}{"teleop_status": map[string]interface{}{
		"running": len(comps) > 0, "components": comps, "error": errStr,
		"last_plan_ms": planMS, "sessions": sessions,
	}}
}
