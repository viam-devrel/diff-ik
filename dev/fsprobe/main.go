// Command fsprobe prints the frame-system geometries and an arm's link geometries (collision Task 0).
package main

import (
	"context"
	"flag"
	"fmt"
	"log"
	"sort"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils/rpc"
)

func main() {
	addr := flag.String("address", "localhost:8083", "")
	name := flag.String("arm", "pick-arm", "")
	flag.Parse()
	ctx := context.Background()
	m, err := client.New(ctx, *addr, logging.NewLogger("probe"), client.WithDialOptions(rpc.WithInsecure()), client.WithDisableSessions())
	if err != nil {
		log.Fatal(err)
	}
	defer m.Close(ctx)
	cfg, err := m.FrameSystemConfig(ctx)
	if err != nil {
		log.Fatal(err)
	}
	fsys, err := referenceframe.NewFrameSystem("t", cfg.Parts, nil)
	if err != nil {
		log.Fatal(err)
	}
	zero := referenceframe.NewZeroInputs(fsys)
	geoms, err := referenceframe.FrameSystemGeometries(fsys, zero)
	if err != nil {
		log.Fatal(err)
	}
	names := []string{}
	for n := range geoms {
		names = append(names, n)
	}
	sort.Strings(names)
	for _, n := range names {
		f := fsys.Frame(n)
		for _, g := range geoms[n].Geometries() {
			c, _ := spatialmath.NewGeometryConfig(g)
			fmt.Printf("frame %-22s dof %d  %-28s %-8s %v\n", n, len(f.DoF()), g.Label(), c.Type, g.Pose().Point())
			if c.Type == spatialmath.MeshType {
				lo, hi := r3.Vector{X: 1e9, Y: 1e9, Z: 1e9}, r3.Vector{X: -1e9, Y: -1e9, Z: -1e9}
				for _, p := range g.ToPoints(1) {
					lo = r3.Vector{X: min(lo.X, p.X), Y: min(lo.Y, p.Y), Z: min(lo.Z, p.Z)}
					hi = r3.Vector{X: max(hi.X, p.X), Y: max(hi.Y, p.Y), Z: max(hi.Z, p.Z)}
				}
				fmt.Printf("    mesh bounds %.0f..%.0f  %.0f..%.0f  %.0f..%.0f\n", lo.X, hi.X, lo.Y, hi.Y, lo.Z, hi.Z)
			}
		}
	}
	for _, p := range cfg.Parts {
		dof := 0
		if p.ModelFrame != nil {
			dof = len(p.ModelFrame.DoF())
		}
		fmt.Printf("part %-22s parent %-12s model dof %d\n", p.FrameConfig.Name(), p.FrameConfig.Parent(), dof)
	}
	a, err := arm.FromProvider(m, *name)
	if err != nil {
		log.Fatal(err)
	}
	model, err := a.Kinematics(ctx)
	if err != nil {
		log.Fatal(err)
	}
	q, err := a.JointPositions(ctx, nil)
	if err != nil {
		log.Fatal(err)
	}
	lg, err := model.Geometries(q)
	if err != nil {
		log.Fatal(err)
	}
	ee, _ := model.Transform(q)
	fmt.Printf("live arm model %q, flange at %v\n", model.Name(), ee.Point())
	for _, g := range lg.Geometries() {
		c, _ := spatialmath.NewGeometryConfig(g)
		fmt.Printf("link %-28s %-8s %v\n", g.Label(), c.Type, g.Pose().Point())
	}
}
