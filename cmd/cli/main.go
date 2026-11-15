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

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/utils/rpc"
)

const usage = `usage: cli [flags] validate
       cli [flags] run static|singular|circle`

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
	wrist := flag.Int("wrist", 4, "singular: index of the wrist joint whose zero is singular")
	flag.Parse()
	args := flag.Args()
	if !(*spread > 0 && *spread <= 1) {
		return errors.New("--spread must be in (0, 1]")
	}
	if *armName == "" || len(args) == 0 {
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
	opts := []client.RobotClientOption{client.WithDialOptions(dial...)}
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

	switch {
	case args[0] == "validate":
		return validate(ctx, a, model, *spread)
	case args[0] == "run" && len(args) == 2:
		return run(ctx, a, model, args[1], *rate, *csvPath, *wrist, *maxDegS, *resyncDeg)
	default:
		return errors.New(usage)
	}
}
