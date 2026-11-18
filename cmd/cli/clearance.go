package main

import (
	"context"
	"encoding/csv"
	"errors"
	"flag"
	"fmt"
	"math"
	"os"
	"sort"
	"strconv"
	"strings"

	"diffik"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils/rpc"
)

// propSample is one props-CSV row: the prop's true pose in world (mm) at Unix time t.
type propSample struct {
	t    float64
	pos  r3.Vector
	q    [4]float64 // w, x, y, z
	size r3.Vector
}

type runRow struct {
	t float64
	q []float64
}

// readCSV returns the header's column index and the rows.
func readCSV(path string) (map[string]int, [][]string, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, nil, err
	}
	defer f.Close()
	recs, err := csv.NewReader(f).ReadAll()
	if err != nil {
		return nil, nil, fmt.Errorf("%s: %w", path, err)
	}
	if len(recs) < 2 {
		return nil, nil, fmt.Errorf("%s: no data rows", path)
	}
	col := map[string]int{}
	for i, h := range recs[0] {
		col[strings.TrimSpace(h)] = i
	}
	return col, recs[1:], nil
}

func num(rec []string, col map[string]int, name string) (float64, error) {
	i, ok := col[name]
	if !ok || i >= len(rec) {
		return 0, fmt.Errorf("missing column %q", name)
	}
	return strconv.ParseFloat(rec[i], 64)
}

func loadRun(path string, dof int) ([]runRow, error) {
	col, recs, err := readCSV(path)
	if err != nil {
		return nil, err
	}
	var out []runRow
	for n, rec := range recs {
		r := runRow{q: make([]float64, dof)}
		if r.t, err = num(rec, col, "epoch_s"); err != nil {
			return nil, fmt.Errorf("%s row %d: %w (was the run recorded before epoch_s existed?)", path, n+2, err)
		}
		for i := range r.q {
			if r.q[i], err = num(rec, col, "q_cmd_"+strconv.Itoa(i)); err != nil {
				return nil, fmt.Errorf("%s row %d: %w", path, n+2, err)
			}
		}
		out = append(out, r)
	}
	return out, nil
}

func loadProps(path string) (map[string][]propSample, error) {
	col, recs, err := readCSV(path)
	if err != nil {
		return nil, err
	}
	nameCol, ok := col["name"]
	if !ok {
		return nil, fmt.Errorf("%s: missing column %q", path, "name")
	}
	out := map[string][]propSample{}
	for n, rec := range recs {
		var v [12]float64
		for i, c := range []string{"t", "x", "y", "z", "qw", "qx", "qy", "qz", "sx", "sy", "sz"} {
			if v[i], err = num(rec, col, c); err != nil {
				return nil, fmt.Errorf("%s row %d: %w", path, n+2, err)
			}
		}
		s := propSample{t: v[0], pos: r3.Vector{X: v[1], Y: v[2], Z: v[3]}, q: [4]float64{v[4], v[5], v[6], v[7]}, size: r3.Vector{X: v[8], Y: v[9], Z: v[10]}}
		out[rec[nameCol]] = append(out[rec[nameCol]], s)
	}
	for _, ss := range out {
		sort.SliceStable(ss, func(i, j int) bool { return ss[i].t < ss[j].t })
	}
	return out, nil
}

// interp linearly interpolates a prop's pose at t (nlerp for the orientation); ok is false outside the logged span.
func interp(ss []propSample, t float64) (s propSample, ok bool) {
	if len(ss) == 0 || t < ss[0].t || t > ss[len(ss)-1].t {
		return s, false
	}
	i := sort.Search(len(ss), func(i int) bool { return ss[i].t >= t })
	b := ss[i]
	if i == 0 || b.t == t {
		return b, true
	}
	a := ss[i-1]
	f := (t - a.t) / (b.t - a.t)
	s = propSample{t: t, pos: a.pos.Add(b.pos.Sub(a.pos).Mul(f)), size: a.size}
	dot := 0.0
	for k := range a.q {
		dot += a.q[k] * b.q[k]
	}
	sign := 1.0
	if dot < 0 {
		sign = -1
	}
	norm := 0.0
	for k := range a.q {
		s.q[k] = a.q[k]*(1-f) + sign*b.q[k]*f
		norm += s.q[k] * s.q[k]
	}
	if norm = math.Sqrt(norm); norm > 0 {
		for k := range s.q {
			s.q[k] /= norm
		}
	}
	return s, true
}

func (s propSample) box(name string) (spatialmath.Geometry, error) {
	pose := spatialmath.NewPose(s.pos, &spatialmath.Quaternion{Real: s.q[0], Imag: s.q[1], Jmag: s.q[2], Kmag: s.q[3]})
	return spatialmath.NewBox(pose, s.size, name)
}

type clearMin struct {
	d    float64
	t    float64
	part string
	n    int // rows measured
}

// clearance replays every run row against each prop's true, uninflated box and returns the minimum per prop
// (by name) and overall (key "").
func clearance(rows []runRow, props map[string][]propSample, pm *diffik.PartsModel) (map[string]clearMin, error) {
	out := map[string]clearMin{"": {d: math.Inf(1)}}
	for name := range props {
		out[name] = clearMin{d: math.Inf(1)}
	}
	for _, r := range rows {
		for name, ss := range props {
			s, ok := interp(ss, r.t)
			if !ok {
				continue
			}
			g, err := s.box(name)
			if err != nil {
				return nil, fmt.Errorf("prop %s: %w", name, err)
			}
			d, part, err := pm.MinDistance(r.q, []spatialmath.Geometry{g.Transform(pm.WorldToBase())})
			if err != nil {
				return nil, err
			}
			for _, k := range []string{name, ""} {
				m := out[k]
				m.n++
				if d < m.d {
					m.d, m.t, m.part = d, r.t, part
				}
				out[k] = m
			}
		}
	}
	return out, nil
}

func clearanceCmd(args []string) error {
	fs := flag.NewFlagSet("clearance", flag.ContinueOnError)
	runCSV := fs.String("csv", "", "run CSV (with epoch_s and q_cmd_*)")
	propsCSV := fs.String("props", "", "props CSV from prop_mover.py")
	address := fs.String("address", "localhost:8080", "machine address")
	insecure := fs.Bool("insecure", false, "dial without TLS")
	armName := fs.String("arm", "", "arm component name")
	worldState := fs.String("world-state", "", "the run's --world-state JSON: its transforms (held objects, stand-in parts) are included")
	if err := fs.Parse(args); err != nil {
		return err
	}
	if *runCSV == "" || *propsCSV == "" || *armName == "" {
		return errors.New("usage: cli clearance --csv RUN.csv --props PROPS.csv --arm NAME [--world-state F] [--address A] [--insecure]")
	}
	props, err := loadProps(*propsCSV)
	if err != nil {
		return err
	}
	ctx := context.Background()
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
	machine, err := client.New(ctx, *address, logging.NewLogger("cli"), client.WithDialOptions(dial...), client.WithDisableSessions())
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
		return err
	}
	fsCfg, err := machine.FrameSystemConfig(ctx)
	if err != nil {
		return err
	}
	rows, err := loadRun(*runCSV, len(model.DoF()))
	if err != nil {
		return err
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
	pm, err := diffik.NewPartsModel(fsCfg.Parts, model, *armName, rows[0].q, ws)
	if err != nil {
		return err
	}
	for _, w := range pm.Warnings() {
		fmt.Println("warning (not measured):", w)
	}
	res, err := clearance(rows, props, pm)
	if err != nil {
		return err
	}
	fmt.Print(clearanceReport(res))
	return nil
}

func clearanceReport(res map[string]clearMin) string {
	var names []string
	for n := range res {
		if n != "" {
			names = append(names, n)
		}
	}
	sort.Strings(names)
	line := func(label string, m clearMin) string {
		if m.n == 0 {
			return fmt.Sprintf("%s: no run rows inside the prop's logged time span\n", label)
		}
		return fmt.Sprintf("%s: min clearance %.2f mm at t=%.3f (%s), %d rows\n", label, m.d, m.t, m.part, m.n)
	}
	var sb strings.Builder
	for _, n := range names {
		sb.WriteString(line(n, res[n]))
	}
	sb.WriteString(line("overall", res[""]))
	return sb.String()
}
