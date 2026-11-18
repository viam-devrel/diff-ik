// Command exportmodel writes an arm's kinematic model, as the machine serves it, to a JSON file
// for unit tests (e.g. testdata/panda.json).
package main

import (
	"context"
	"flag"
	"log"
	"os"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/utils/rpc"
	"google.golang.org/protobuf/encoding/protojson"
)

func main() {
	addr := flag.String("address", "localhost:8083", "plaintext machine address")
	name := flag.String("arm", "pick-arm", "arm component name")
	out := flag.String("out", "testdata/panda.json", "output file")
	flag.Parse()
	ctx := context.Background()
	m, err := client.New(ctx, *addr, logging.NewLogger("exportmodel"),
		client.WithDialOptions(rpc.WithInsecure()), client.WithDisableSessions())
	if err != nil {
		log.Fatal(err)
	}
	defer m.Close(ctx)
	a, err := arm.FromProvider(m, *name)
	if err != nil {
		log.Fatal(err)
	}
	model, err := a.Kinematics(ctx)
	if err != nil {
		log.Fatal(err)
	}
	// UnmarshalModelJSON reads the model config itself, not json.Marshal(model)'s wrapper
	b, err := protojson.Marshal(referenceframe.KinematicModelToProtobuf(model).GetModel())
	if err != nil {
		log.Fatal(err)
	}
	if err := os.WriteFile(*out, b, 0o644); err != nil {
		log.Fatal(err)
	}
}
