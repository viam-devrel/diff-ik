// Command cli is the Phase 0/1 harness: it connects to a machine, validates the arm's
// kinematics (validate) and runs servo-loop scenarios against it (run static|singular|circle).
package main

import (
	"context"
	"errors"
	"flag"
	"fmt"
	"math"
	"os"
	"os/signal"
	"slices"
	"strconv"
	"strings"
	"time"

	"diffik"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/rdk/services/vision"
	"go.viam.com/utils/rpc"
)

const usage = `usage: cli [flags] validate
       cli [flags] run static|singular|circle|hold|orbit
       cli [flags] probe
       cli [flags] pose
       cli clearance --csv RUN.csv --props PROPS.csv --arm NAME [--world-state F] [--address A] [--insecure]`

func main() {
	if err := realMain(); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}

func realMain() error {
	if len(os.Args) > 1 && os.Args[1] == "clearance" { // its own flags, after the subcommand
		return clearanceCmd(os.Args[2:])
	}
	address := flag.String("address", "localhost:8080", "machine address, e.g. <machine>.local.viam.cloud:8080 (or use --insecure for a plaintext local server)")
	insecure := flag.Bool("insecure", false, "dial without TLS (plaintext local server)")
	armName := flag.String("arm", "", "arm component name (required)")
	rate := flag.Float64("rate", 50, "loop rate in Hz")
	csvPath := flag.String("csv", "", "write one row per tick to this file (run only)")
	spread := flag.Float64("spread", 0.1, "validate: fraction of each joint's half-range to sample around the start pose")
	maxDegS := flag.Float64("max-joint-deg-s", 0, "run: per-joint velocity limit in deg/s (0 = loop default 90)")
	resyncDeg := flag.Float64("resync-deg", 0, "run: allowed divergence in deg between q_meas and where the arm should be given its measured lag, before resync (0 = loop default 3 ticks at max speed). The q_cmd - q_meas gap, and so a resync snap, can reach lag travel + this")
	maxLagMS := flag.Float64("max-lag-ms", 0, "run: longest arm lag the loop treats as lag; lag_ms is capped here and a bigger gap is divergence (0 = loop default 300, max 1000). If the arm never moves, the command can run this far ahead before the first resync")
	wrist := flag.Int("wrist", 4, "singular: index of the joint whose --singular-deg angle is singular (UR5e wrist: 4; SO-101 elbow: 2)")
	task := flag.String("task", "", "run: task mode, pose|pointing (default: pointing if the arm has < 6 joints, else pose)")
	singEps := flag.Float64("singular-eps", 0, "run: Chiaverini threshold ε (0 = loop default 0.04)")
	maxDamp := flag.Float64("max-damping", 0, "run: maximum damping λmax (0 = loop default 0.1)")
	singDeg := flag.Float64("singular-deg", 0, "singular: joint angle (deg) the scenario sweeps through, ±11.5° around it")
	minZ := flag.Float64("min-z-mm", 0, "floor guard: lowest allowed base-frame z (mm) for the end effector and jaw box; enabled whenever the flag is given (it can be 0 or negative)")
	tipMM := flag.Float64("tip-mm", 105, "floor guard: jaw tip distance along the tool z axis (mm)")
	settleTol := flag.Float64("settle-tol", 1e-4, "validate: max joint change (rad) between bracketing reads to accept a settled sample")
	hold := flag.Duration("hold", 0, "static: hold each target this long after convergence and report the drift")
	maxLin := flag.Float64("max-linear-mm-s", 0, "run: linear speed limit (0 = loop default 100)")
	maxAng := flag.Float64("max-angular-deg-s", 0, "run: angular speed limit (0 = loop default 45)")
	tolMM := flag.Float64("tol-mm", 0, "run: convergence position tolerance (0 = loop default 2)")
	tolDeg := flag.Float64("tol-deg", 0, "run: convergence angle tolerance (0 = loop default 2)")
	circleMMs := flag.Float64("circle-mm-s", 50, "circle: speed around the circle (mm/s)")
	leadMS := flag.Float64("lead-ms", 0, "run/probe: stamp each streamed point this far ahead (ms; 0 = diffik.DefaultLead, 5 ms; capped at one tick)")
	startJoints := flag.String("start-joints", "", "comma-separated joint angles (rad): run/validate/probe first make a guarded profiled move here")
	solver := flag.String("solver", "", "run: dls|qp (default dls)")
	postureJoint := flag.Int("posture-joint", 0, "run: joint index the posture offset moves along the self-motion (7-DoF arms)")
	postureDeg := flag.Float64("posture-deg", 0, "run: with --solver qp, posture q0 = the start joints moved this many degrees on --posture-joint along the self-motion (0 = q0 is the start joints)")
	postureW := flag.Float64("posture-weight", 0, "run: posture weight (0 = no posture task)")
	postureK := flag.Float64("posture-gain", 0, "run: posture gain 1/s (0 = default 1)")
	orbitDegS := flag.Float64("orbit-deg-s", 8, "orbit: rate the target pose rotates about base z, deg/s (negative reverses)")
	orbitDeg := flag.Float64("orbit-deg", 320, "orbit: total rotation, deg")
	maxDegS2 := flag.Float64("max-joint-deg-s2", 0, "run: per-joint acceleration limit in deg/s² (0 = the model's, else 360)")
	worldState := flag.String("world-state", "", "run: JSON file with a motion-service WorldState; turns collision avoidance on (needs --solver qp)")
	var allows [][2]string
	flag.Func("allow", "run: frame1:frame2 pair allowed to touch (repeatable; needs collision on)", func(s string) error {
		p, err := parseAllow(s)
		allows = append(allows, p)
		return err
	})
	collisionOn := flag.Bool("collision", false, "run: collision avoidance on without a world state (arm and mounted parts vs frame-system obstacles; needs --solver qp)")
	collisionOff := flag.Bool("collision-off", false, "run: collision avoidance off, even with --world-state")
	dMinMM := flag.Float64("d-min-mm", 0, "run: collision clearance to keep, mm (0 = default 10)")
	dActMM := flag.Float64("d-act-mm", 0, "run: distance at which the collision damper starts acting, mm (0 = default 100)")
	descendMM := flag.Float64("descend-mm", 100, "descend: how far the target moves down (-z, base frame), mm")
	descendMMs := flag.Float64("descend-mm-s", 20, "descend: speed of the target, mm/s")
	liveVision := flag.String("live-vision", "", "run: vision service to take live obstacles from (needs --solver qp; turns collision on)")
	liveCamera := flag.String("live-camera", "", "run: camera name passed to the vision service (and the frame for point-cloud objects)")
	liveRate := flag.Float64("live-rate-hz", 0, "run: live poll rate (0 = default 10)")
	liveStale := flag.Float64("live-stale-ms", 0, "run: stop if no detections arrive for this long (0 = default 500)")
	liveInflate := flag.Float64("live-inflate-mm", 0, "run: inflate each live obstacle by this much (default 10; 0 is allowed)")
	liveWorkspace := flag.String("live-workspace", "", "run: clip live obstacles to this world box, mm: x0,y0,z0,x1,y1,z1")
	liveExtrude := flag.Float64("live-extrude-z", 0, "run: extrude live obstacles down to this world z, mm (enabled whenever the flag is given)")
	flag.Parse()
	args := flag.Args()
	collision := (*collisionOn || *worldState != "" || *liveVision != "") && !*collisionOff
	var live *diffik.LiveConfig
	if *liveVision != "" {
		if *solver != diffik.SolverQP || *collisionOff {
			return errors.New("--live-vision needs --solver qp and collision on")
		}
		live = &diffik.LiveConfig{Vision: *liveVision, Camera: *liveCamera, RateHz: *liveRate, StaleMS: *liveStale}
		if *liveWorkspace != "" {
			var err error
			if live.WorkspaceMin, live.WorkspaceMax, err = parseWorkspace(*liveWorkspace); err != nil {
				return err
			}
		}
		flag.Visit(func(f *flag.Flag) {
			switch f.Name {
			case "live-extrude-z":
				live.ExtrudeToZ = liveExtrude
			case "live-inflate-mm":
				live.InflateMM = liveInflate
			}
		})
		if err := live.Validate(); err != nil {
			return fmt.Errorf("--live-*: %w", err)
		}
	} else if *liveCamera != "" || *liveRate != 0 || *liveStale != 0 || *liveWorkspace != "" || liveFlagSet("live-inflate-mm", "live-extrude-z") {
		return errors.New("--live-* flags need --live-vision")
	}
	if collision && *solver != diffik.SolverQP {
		return errors.New("collision avoidance needs --solver qp")
	}
	if len(allows) > 0 && !collision {
		return errors.New("--allow needs collision on (--collision or --world-state)")
	}
	if !(*dMinMM >= 0) || !(*dActMM >= 0) || math.IsInf(*dMinMM, 0) || math.IsInf(*dActMM, 0) || !(*descendMM > 0) || !(*descendMMs > 0) {
		return errors.New("--d-min-mm and --d-act-mm must be finite and >= 0; --descend-mm and --descend-mm-s must be > 0")
	}
	var ws *referenceframe.WorldState
	if *worldState != "" {
		b, err := os.ReadFile(*worldState)
		if err != nil {
			return err
		}
		if ws, err = diffik.ParseWorldState(b); err != nil {
			return fmt.Errorf("--world-state: %w", err)
		}
	}
	minZSet := false
	flag.Visit(func(f *flag.Flag) {
		if f.Name == "min-z-mm" {
			minZSet = true
		}
	})
	if minZSet && *startJoints == "" {
		return errors.New("--start-joints is required with --min-z-mm")
	}
	if !(*tipMM > 0) || !(*settleTol > 0) || !(*circleMMs > 0) || *hold < 0 || *maxLin < 0 || *maxAng < 0 || *tolMM < 0 || *tolDeg < 0 {
		return errors.New("--tip-mm, --settle-tol and --circle-mm-s must be > 0; --hold, --max-linear-mm-s, --max-angular-deg-s and --tol-mm must be >= 0")
	}
	if !(*spread > 0 && *spread <= 1) {
		return errors.New("--spread must be in (0, 1]")
	}
	if *task != "" && *task != diffik.TaskPose && *task != diffik.TaskPointing {
		return fmt.Errorf("--task must be %q or %q", diffik.TaskPose, diffik.TaskPointing) // before singular moves the arm
	}
	if *singEps < 0 || *maxDamp < 0 {
		return errors.New("--singular-eps and --max-damping must be >= 0")
	}
	if *orbitDegS == 0 || math.IsNaN(*orbitDegS) || math.IsInf(*orbitDegS, 0) || *maxDegS2 < 0 {
		return errors.New("--orbit-deg-s must be non-zero and --max-joint-deg-s2 >= 0")
	}
	if *solver != "" && *solver != diffik.SolverDLS && *solver != diffik.SolverQP {
		return fmt.Errorf("--solver must be %q or %q", diffik.SolverDLS, diffik.SolverQP)
	}
	if (*postureW > 0 || *postureDeg != 0) && *solver != diffik.SolverQP {
		return errors.New("--posture-weight and --posture-deg need --solver qp")
	}
	if !(*rate >= 1) {
		return errors.New("--rate must be >= 1")
	}
	validCmd := len(args) == 1 && (args[0] == "validate" || args[0] == "probe" || args[0] == "pose") ||
		len(args) == 2 && args[0] == "run" && (args[1] == "static" || args[1] == "singular" || args[1] == "circle" || args[1] == "hold" || args[1] == "orbit" || args[1] == "descend")
	if *armName == "" || !validCmd {
		return errors.New(usage)
	}

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt)
	defer stop()
	logger := logging.NewLogger("cli")

	var dial []rpc.DialOption
	if *insecure {
		dial = append(dial, rpc.WithInsecure())
	}
	if id, key := os.Getenv("VIAM_API_KEY_ID"), os.Getenv("VIAM_API_KEY"); id != "" {
		dial = append(dial, rpc.WithEntityCredentials(id, rpc.Credentials{Type: rpc.CredentialsTypeAPIKey, Payload: key}))
		if *insecure {
			dial = append(dial, rpc.WithAllowInsecureWithCredentialsDowngrade())
		}
	}
	opts := []client.RobotClientOption{client.WithDialOptions(dial...),
		// Sessions off: a lapsed session heartbeat makes viam-server call arm.Stop, which on the SO-101 speeds up a profiled move (spec 3c1).
		client.WithDisableSessions()}
	machine, err := client.New(ctx, *address, logger, opts...)
	if err != nil {
		return fmt.Errorf("connecting to %s: %w", *address, err)
	}
	defer machine.Close(context.Background())

	a, err := arm.FromProvider(machine, *armName)
	if err != nil {
		return err
	}
	model, err := a.Kinematics(ctx)
	if err != nil {
		return fmt.Errorf("getting kinematics: %w", err)
	}

	if *postureJoint < 0 || *postureJoint >= len(model.DoF()) {
		return fmt.Errorf("--posture-joint %d out of range for %d joints", *postureJoint, len(model.DoF()))
	}
	var guard *floorGuard
	if minZSet {
		guard = &floorGuard{model: model, minZ: *minZ, tipMM: *tipMM}
	}
	var sa arm.Arm = &safeArm{Arm: a, guard: guard}

	var req *diffik.CollisionRequest
	var parts []*referenceframe.FrameSystemPart
	if collision && len(args) == 2 { // run only
		fsCfg, err := machine.FrameSystemConfig(ctx)
		if err != nil {
			return fmt.Errorf("reading the frame system: %w", err)
		}
		parts = fsCfg.Parts
		req = &diffik.CollisionRequest{WorldState: ws, Allows: allows, DMinMM: *dMinMM, DActMM: *dActMM}
		// throwaway build: a bad --allow or a mesh fails before the arm moves
		q0, err := a.JointPositions(ctx, nil)
		if err != nil {
			return err
		}
		if _, err := diffik.NewCollisionModel(parts, model, *armName, *req, q0); err != nil {
			return err
		}
	}

	var liveSrc vision.Service
	if live != nil && len(args) == 2 {
		if liveSrc, err = vision.FromProvider(machine, live.Vision); err != nil {
			return fmt.Errorf("--live-vision: %w", err)
		}
	}

	var startQ []float64
	if live != nil && len(args) == 2 { // dry check before anything moves: properties call and camera frame
		q0, err := a.JointPositions(ctx, nil)
		if err != nil {
			return err
		}
		if err := diffik.CheckLive(ctx, liveSrc, parts, model, *armName, q0, *live); err != nil {
			return err
		}
	}
	if *startJoints != "" {
		if startQ, err = parseJoints(*startJoints, model.DoF()); err != nil {
			return err
		}
		fmt.Printf("moving to start joints %v (profiled move)\n", startQ)
		if err := sa.MoveToJointPositions(ctx, startQ, nil); err != nil {
			return err
		}
	}

	switch {
	case args[0] == "pose":
		return showPose(ctx, sa, model, guard, startQ, *hold)
	case args[0] == "validate":
		return validate(ctx, sa, model, *spread, *settleTol, guard)
	case args[0] == "probe" && len(args) == 1:
		return probe(ctx, sa, *rate, msDur(*leadMS))
	case args[0] == "run" && len(args) == 2:
		return run(ctx, sa, model, args[1], runOpts{
			rate: *rate, csvPath: *csvPath, wrist: *wrist, maxJointDegS: *maxDegS, maxLinear: *maxLin, maxAngular: *maxAng,
			resync: *resyncDeg, maxLagMS: *maxLagMS, tolMM: *tolMM, tolDeg: *tolDeg, task: *task, singEps: *singEps, maxDamp: *maxDamp, singDeg: *singDeg,
			hold: *hold, circleMMs: *circleMMs, guard: guard, lead: msDur(*leadMS),
			solver: *solver, postureJoint: *postureJoint, postureDeg: *postureDeg, postureW: *postureW, postureK: *postureK,
			descendMM: *descendMM, descendMMs: *descendMMs, armName: *armName, collision: req, parts: parts,
			orbitDegS: *orbitDegS, orbitDeg: *orbitDeg, maxJointDegS2: *maxDegS2, live: live, liveSrc: liveSrc,
		})
	default:
		return errors.New(usage)
	}
}

// parseJoints parses a comma-separated list of joint angles (rad), one per limit, each within its limits.
func parseJoints(s string, limits []referenceframe.Limit) ([]float64, error) {
	parts := strings.Split(s, ",")
	if len(parts) != len(limits) {
		return nil, fmt.Errorf("--start-joints has %d values, the arm has %d joints", len(parts), len(limits))
	}
	q := make([]float64, len(parts))
	for i, p := range parts {
		v, err := strconv.ParseFloat(strings.TrimSpace(p), 64)
		if err != nil {
			return nil, fmt.Errorf("--start-joints value %d: %w", i, err)
		}
		if !(v >= limits[i].Min && v <= limits[i].Max) {
			return nil, fmt.Errorf("--start-joints value %d: %v outside [%.3f, %.3f]", i, v, limits[i].Min, limits[i].Max)
		}
		q[i] = v
	}
	return q, nil
}

func msDur(ms float64) time.Duration { return time.Duration(ms * float64(time.Millisecond)) }

// parseAllow parses "frame1:frame2".
func parseAllow(s string) ([2]string, error) {
	i := strings.LastIndex(s, ":")
	if i < 0 {
		return [2]string{}, fmt.Errorf("--allow %q: want frame1:frame2", s)
	}
	a, b := s[:i], s[i+1:]
	if a == "" || b == "" {
		return [2]string{}, fmt.Errorf("--allow %q: want frame1:frame2", s)
	}
	return [2]string{a, b}, nil
}

// liveFlagSet reports whether any of the named flags was given.
func liveFlagSet(names ...string) (set bool) {
	flag.Visit(func(f *flag.Flag) {
		if slices.Contains(names, f.Name) {
			set = true
		}
	})
	return
}
