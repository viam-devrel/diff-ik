package diffik

import (
	"context"
	"errors"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	motion "go.viam.com/rdk/services/motion"
)

var (
	Servo            = resource.NewModel("devrel", "diff-ik", "servo")
	errUnimplemented = errors.New("unimplemented")
)

func init() {
	resource.RegisterService(motion.API, Servo,
		resource.Registration[motion.Service, Config]{
			Constructor: newDiffIkServo,
		},
	)
}

type Config struct {
	/*
		Put config attributes here. There should be public/exported fields
		with a `json` parameter at the end of each attribute.

		Example config struct:
			type Config struct {
				Pin   string `json:"pin"`
				Board string `json:"board"`
				MinDeg *float64 `json:"min_angle_deg,omitempty"`
			}

		If your model does not need a config, replace Config in the init
		function with resource.NoNativeConfig
	*/
}

// Validate ensures all parts of the config are valid and important fields exist.
// Returns three values:
//  1. Required dependencies: other resources that must exist for this resource to work.
//  2. Optional dependencies: other resources that may exist but are not required.
//  3. An error if any Config fields are missing or invalid.
//
// The `path` parameter indicates
// where this resource appears in the machine's JSON configuration
// (for example, "components.0"). You can use it in error messages
// to indicate which resource has a problem.
//
// Note: Validate receives a copy of the config; mutations to it will do
// nothing. Fill in any default values in your resource's constructor function
// instead.
func (cfg Config) Validate(path string) ([]string, []string, error) {
	// Add config validation code here
	return nil, nil, nil
}

type diffIkServo struct {
	resource.AlwaysRebuild
	resource.Named

	name resource.Name

	logger logging.Logger
	cfg    Config

	cancelCtx  context.Context
	cancelFunc func()
}

func newDiffIkServo(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (motion.Service, error) {
	conf, err := resource.NativeConfig[Config](rawConf)
	if err != nil {
		return nil, err
	}

	return NewServo(ctx, deps, rawConf.ResourceName(), conf, logger)

}

func NewServo(ctx context.Context, deps resource.Dependencies, name resource.Name, conf Config, logger logging.Logger) (motion.Service, error) {

	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	s := &diffIkServo{
		name:       name,
		logger:     logger,
		cfg:        conf,
		cancelCtx:  cancelCtx,
		cancelFunc: cancelFunc,
	}
	return s, nil
}

func (s *diffIkServo) Name() resource.Name {
	return s.name
}

// Move is the primary method to move multiple components or any object to a specified location.
// Given a destination pose and a component, Move constructs a kinematic chain from goal to destination,
// solves it while adhering to constraints, and executes the movement to avoid collisions with the machine itself
// and other known objects. The above arguments are all grouped together in the MoveReq struct.
func (s *diffIkServo) Move(ctx context.Context, req motion.MoveReq) (bool, error) {
	return false, errUnimplemented
}

// MoveOnMap moves a base component to a destination Pose on a SLAM map and returns a unique ExecutionID.
// If the machine is already within PlanDeviationM of the goal, an error is returned.
// Monitor progress with `GetPlan()` and `ListPlanStatuses()`, and check the machine's position via the SLAM service.
// Designed for autonomous indoor navigation of rover bases.
func (s *diffIkServo) MoveOnMap(ctx context.Context, req motion.MoveOnMapReq) (motion.ExecutionID, error) {
	var executionIDRetVal motion.ExecutionID

	return executionIDRetVal, errUnimplemented
}

// MoveOnGlobe moves a base component to a destination GPS point(latitude, longitude and returns a unique ExecutionID.
// If the machine is already within PlanDeviationM of the goal, an error is returned.
// This non-blocking method uses a movement sensor to verify the location of the base.
// You can monitor progress with `GetPlan()` and `ListPlanStatuses()`. Designed for autonomous GPS navigation of rover bases.
func (s *diffIkServo) MoveOnGlobe(ctx context.Context, req motion.MoveOnGlobeReq) (motion.ExecutionID, error) {
	var executionIDRetVal motion.ExecutionID

	return executionIDRetVal, errUnimplemented
}

// GetPose returns the location and orientation of a component within a frame system.
// It returns a `PoseInFrame` describing the pose of the specified component relative to the specified destination frame.
// The `supplemental_transforms` argument can be used to augment the machine's existing frame system with additional frames.
// deprecated, use framesystem.Servce.GetPose
func (s *diffIkServo) GetPose(ctx context.Context, componentName string, destinationFrame string, supplementalTransforms []*referenceframe.LinkInFrame, extra map[string]interface{}) (*referenceframe.PoseInFrame, error) {
	return nil, errUnimplemented
}

// StopPlan stops a base component being moved by an in progress `MoveOnGlobe()` or `MoveOnMap()` call.
func (s *diffIkServo) StopPlan(ctx context.Context, req motion.StopPlanReq) error {
	return errUnimplemented
}

// ListPlanStatuses returns the statuses of plans created by `MoveOnGlobe()` or `MoveOnMap()` since the motion service initialized.
// It includes plans that are in progress or have changed state in the last 24 hours.
// All repeated fields are in chronological order.
func (s *diffIkServo) ListPlanStatuses(ctx context.Context, req motion.ListPlanStatusesReq) ([]motion.PlanStatusWithID, error) {
	return nil, errUnimplemented
}

// PlanHistory returns the plan history of the most recent `MoveOnGlobe()` or `MoveOnMap()` call by default.
// The history for earlier executions can be requested by providing an ExecutionID.
// It returns a result if the execution is active or has changed state in the last 24 hours and the machine has not reinitialized.
// Plans never change; replans always create new plans and replans share the ExecutionID of the previously executing plan.
func (s *diffIkServo) PlanHistory(ctx context.Context, req motion.PlanHistoryReq) ([]motion.PlanWithStatus, error) {
	return nil, errUnimplemented
}

// TempStreamArmJointPositions derives and paces a trajectory to armName from the targets it
// receives on targets, according to opts. It blocks until targets is closed and the derived
// trajectory has finished executing on the arm, or until ctx is canceled, whichever comes
// first. Only one call may be in flight for a given armName at a time; a concurrent call for
// the same arm returns an error immediately.
//
// If ctx is canceled, or the implementation fails on its own, it must stop the arm and return
// only once the arm has stopped; until it returns, a concurrent call for the same arm keeps
// returning an error.
//
// The implementation must not return nil before targets is closed: a nil return means every
// target was consumed and the trajectory derived from them has finished executing, leaving the arm at
// rest.
//
// The implementation writes a TempStreamResponse to responses for each acknowledgment it wants to send
// back, and the framework forwards each onto the wire. The framework, not the implementation,
// owns both channels: it writes and closes targets, and it closes responses after the
// implementation returns. So an implementation only reads targets and only writes responses,
// and must not close either.
func (s *diffIkServo) TempStreamArmJointPositions(ctx context.Context, armName string, opts motion.TempStreamOptions, targets <-chan []referenceframe.Input, responses chan<- motion.TempStreamResponse, extra map[string]interface{}) error {
	return errUnimplemented
}

func (s *diffIkServo) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, errUnimplemented
}

func (s *diffIkServo) Status(ctx context.Context) (map[string]interface{}, error) {
	return nil, errUnimplemented
}

func (s *diffIkServo) Close(context.Context) error {
	// Put close code here
	s.cancelFunc()
	return nil
}
