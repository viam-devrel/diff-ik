// Command cli is the Phase 0/1 harness: it connects to a machine, validates the arm's
// kinematics (validate) and runs servo-loop scenarios against it (run static|singular|circle).
package main

import (
	"context"
	"errors"
	"flag"
	"fmt"
	"os"
	"os/signal"
	"strconv"
	"strings"
	"time"

	"diffik"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/utils/rpc"
)

const usage = `usage: cli [flags] validate
       cli [flags] run static|singular|circle
       cli [flags] probe
       cli [flags] pose`

func main() {
	if err := realMain(); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}

func realMain() error {
	address := flag.String("address", "localhost:8080", "machine address, e.g. <machine>.local.viam.cloud:8080 (or use --insecure for a plaintext local server)")
	insecure := flag.Bool("insecure", false, "dial without TLS (plaintext local server)")
	armName := flag.String("arm", "", "arm component name (required)")
	rate := flag.Float64("rate", 50, "loop rate in Hz")
	csvPath := flag.String("csv", "", "write one row per tick to this file (run only)")
	spread := flag.Float64("spread", 0.1, "validate: fraction of each joint's half-range to sample around the start pose")
	maxDegS := flag.Float64("max-joint-deg-s", 0, "run: per-joint velocity limit in deg/s (0 = loop default 90)")
	resyncDeg := flag.Float64("resync-deg", 0, "run: q_cmd/q_meas divergence in deg before resync (0 = loop default 3 ticks at max speed)")
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
	flag.Parse()
	args := flag.Args()
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
	if !(*rate >= 1) {
		return errors.New("--rate must be >= 1")
	}
	validCmd := len(args) == 1 && (args[0] == "validate" || args[0] == "probe" || args[0] == "pose") ||
		len(args) == 2 && args[0] == "run" && (args[1] == "static" || args[1] == "singular" || args[1] == "circle")
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

	var guard *floorGuard
	if minZSet {
		guard = &floorGuard{model: model, minZ: *minZ, tipMM: *tipMM}
	}
	var sa arm.Arm = &safeArm{Arm: a, guard: guard}

	var startQ []float64
	if *startJoints != "" {
		if startQ, err = parseJoints(*startJoints, model.DoF()); err != nil {
			return err
		}
		fmt.Printf("moving to start joints %v (profiled, 30°/s on the SO-101 driver)\n", startQ)
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
			resync: *resyncDeg, tolMM: *tolMM, tolDeg: *tolDeg, task: *task, singEps: *singEps, maxDamp: *maxDamp, singDeg: *singDeg,
			hold: *hold, circleMMs: *circleMMs, guard: guard, lead: msDur(*leadMS),
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
