// Command liveprobe checks vision services' 3D obstacles against known prop poses (live-obstacles Task 0).
package main

import (
	"context"
	"flag"
	"fmt"
	"log"
	"math"
	"sort"
	"strings"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/generic"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils/rpc"
)

type det struct {
	center r3.Vector // world mm
	dims   string
	label  string
	points int
}

func main() {
	addr := flag.String("address", "localhost:8084", "")
	svcs := flag.String("services", "obstacles-top:top-depth,segments-top:top-depth", "service:camera,...")
	poses := flag.String("poses", "500,150,81;600,-100,81;450,250,81", "intruder centre poses, world mm (x,y,z;...)")
	n := flag.Int("n", 20, "latency samples per service")
	wsMin := flag.Float64("ws-zmin", 20, "workspace clip: min world z (mm); the x/y range is the table")
	wsMax := flag.Float64("ws-zmax", 600, "workspace clip: max world z (mm)")
	extrude := flag.Float64("extrude-z", 20, "extrude each box down to this world z (mm); NaN disables")
	flag.Parse()
	ctx := context.Background()
	m, err := client.New(ctx, *addr, logging.NewLogger("probe"), client.WithDialOptions(rpc.WithInsecure()), client.WithDisableSessions())
	if err != nil {
		log.Fatal(err)
	}
	defer m.Close(ctx)
	world, err := generic.FromProvider(m, "sim-world")
	if err != nil {
		log.Fatal(err)
	}
	for _, sc := range strings.Split(*svcs, ",") {
		parts := strings.Split(sc, ":")
		svc, err := vision.FromProvider(m, parts[0])
		if err != nil {
			log.Fatal(err)
		}
		props, err := svc.GetProperties(ctx, nil)
		if err != nil {
			log.Fatal(err)
		}
		fmt.Printf("== %s on %s: detections3d %v object_pcds %v\n", parts[0], parts[1], props.Detections3DSupported, props.ObjectPCDsSupported)
		for _, ps := range strings.Split(*poses, ";") {
			var x, y, z float64
			fmt.Sscanf(ps, "%f,%f,%f", &x, &y, &z)
			if _, err := world.DoCommand(ctx, map[string]interface{}{"command": "set_prop_pose", "prop": "intruder",
				"position": []interface{}{x / 1000, y / 1000, z / 1000}, "orientation_wxyz": []interface{}{1, 0, 0, 0}}); err != nil {
				log.Fatal(err)
			}
			time.Sleep(700 * time.Millisecond)
			ds, err := detect(ctx, m, svc, parts[1], props.Detections3DSupported, *wsMin, *wsMax, *extrude)
			if err != nil {
				log.Fatal(err)
			}
			truth := r3.Vector{X: x, Y: y, Z: z}
			sort.Slice(ds, func(i, j int) bool { return ds[i].center.Sub(truth).Norm() < ds[j].center.Sub(truth).Norm() })
			fmt.Printf("  intruder at %v: %d detections\n", truth, len(ds))
			for i, d := range ds {
				if i >= 4 {
					break
				}
				fmt.Printf("    centre %6.1f %6.1f %6.1f  err %6.1f mm  dims %s  label %q  points %d\n", d.center.X, d.center.Y, d.center.Z, d.center.Sub(truth).Norm(), d.dims, d.label, d.points)
			}
		}
		var lat []time.Duration
		for i := 0; i < *n; i++ {
			t0 := time.Now()
			if _, err := detect(ctx, m, svc, parts[1], props.Detections3DSupported, *wsMin, *wsMax, *extrude); err != nil {
				log.Fatal(err)
			}
			lat = append(lat, time.Since(t0))
		}
		sort.Slice(lat, func(i, j int) bool { return lat[i] < lat[j] })
		fmt.Printf("  latency p50 %v p95 %v\n", lat[len(lat)/2], lat[int(math.Ceil(float64(len(lat))*0.95))-1])
	}
}

// clean clips world points to the table workspace, refits a world AABB and extrudes it (spec step 4b).
func clean(pts []r3.Vector, zmin, zmax, extrude float64) (r3.Vector, string, int, bool) {
	lo := r3.Vector{X: 1e9, Y: 1e9, Z: 1e9}
	hi := lo.Mul(-1)
	n := 0
	for _, p := range pts {
		if p.X < 200 || p.X > 1000 || p.Y < -600 || p.Y > 600 || p.Z < zmin || p.Z > zmax {
			continue
		}
		n++
		lo = r3.Vector{X: math.Min(lo.X, p.X), Y: math.Min(lo.Y, p.Y), Z: math.Min(lo.Z, p.Z)}
		hi = r3.Vector{X: math.Max(hi.X, p.X), Y: math.Max(hi.Y, p.Y), Z: math.Max(hi.Z, p.Z)}
	}
	if n < 10 {
		return r3.Vector{}, "", n, false
	}
	if !math.IsNaN(extrude) {
		lo.Z = extrude
	}
	d := hi.Sub(lo)
	return lo.Add(hi).Mul(0.5), fmt.Sprintf("clean %.0fx%.0fx%.0f (z %.0f..%.0f)", d.X, d.Y, d.Z, lo.Z, hi.Z), n, true
}

func detect(ctx context.Context, m *client.RobotClient, svc vision.Service, cam string, use3D bool, zmin, zmax, extrude float64) ([]det, error) {
	var out []det
	frameTF := map[string]spatialmath.Pose{}
	tf := func(frame string) (spatialmath.Pose, error) {
		if p, ok := frameTF[frame]; ok {
			return p, nil
		}
		pif, err := m.TransformPose(ctx, referenceframe.NewPoseInFrame(frame, spatialmath.NewZeroPose()), referenceframe.World, nil)
		if err != nil {
			return nil, err
		}
		frameTF[frame] = pif.Pose()
		return pif.Pose(), nil
	}
	worldPts := func(pc pointcloud.PointCloud, through spatialmath.Pose) []r3.Vector {
		var ps []r3.Vector
		pc.Iterate(0, 0, func(p r3.Vector, _ pointcloud.Data) bool {
			ps = append(ps, spatialmath.Compose(through, spatialmath.NewPoseFromPoint(p)).Point())
			return true
		})
		return ps
	}
	toWorld := func(p spatialmath.Pose, frame string) (r3.Vector, error) {
		pif, err := m.TransformPose(ctx, referenceframe.NewPoseInFrame(frame, p), referenceframe.World, nil)
		if err != nil {
			return r3.Vector{}, err
		}
		return pif.Pose().Point(), nil
	}
	if use3D {
		ds, err := svc.GetDetections3D(ctx, cam, nil)
		if err != nil {
			return nil, err
		}
		for _, d := range ds {
			if len(d.Transforms) == 0 {
				continue
			}
			root := d.Transforms[0]
			g := root.Geometry()
			if g == nil {
				continue
			}
			c, err := toWorld(spatialmath.Compose(root.Pose(), g.Pose()), root.Parent())
			if err != nil {
				return nil, err
			}
			pts := 0
			var wp []r3.Vector
			base, err := tf(root.Parent())
			if err != nil {
				return nil, err
			}
			for _, t := range d.Transforms[1:] {
				if pc, ok := t.Geometry().(pointcloud.PointCloud); ok {
					pts = pc.Size()
					wp = worldPts(pc, spatialmath.Compose(base, spatialmath.Compose(root.Pose(), t.Pose())))
				}
			}
			label := ""
			if len(d.Classifications) > 0 {
				label = d.Classifications[0].Label()
			}
			if cc, cd, _, ok := clean(wp, zmin, zmax, extrude); ok {
				out = append(out, det{cc, cd, label, pts})
			} else {
				_ = c
			}
		}
		return out, nil
	}
	objs, err := svc.GetObjectPointClouds(ctx, cam, nil)
	if err != nil {
		return nil, err
	}
	for _, o := range objs {
		if o.Geometry == nil {
			continue
		}
		c, err := toWorld(o.Geometry.Pose(), cam)
		if err != nil {
			return nil, err
		}
		pts := 0
		var wp []r3.Vector
		if o.PointCloud != nil {
			pts = o.PointCloud.Size()
			base, err := tf(cam)
			if err != nil {
				return nil, err
			}
			wp = worldPts(o.PointCloud, base)
		}
		_ = c
		if cc, cd, _, ok := clean(wp, zmin, zmax, extrude); ok {
			out = append(out, det{cc, cd, o.Geometry.Label(), pts})
		}
	}
	return out, nil
}
