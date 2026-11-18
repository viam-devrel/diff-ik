# Panda in MuJoCo: QP solver A/B runs

Spec: `docs/superpowers/specs/2026-11-18-diff-ik-qp-solver-design.md`. Results: `docs/results/2026-11-qp-solver.md`.

## Prerequisites

- **viam-mujoco-sim,** branch `feat/custom-assets`, in `~/src/viam-mujoco-sim`.

## Start the cell

```
go build -o bin/diff-ik ./cmd/module
viam-server -config dev/panda-mujoco.json        # port 8083
```

The world takes about 30 s to build. The arm is `pick-arm`. Stop the SO-101 cells first, so only one MuJoCo process renders.

## Refresh the kinematic model

```
go run ./dev/exportmodel
```

This rewrites `testdata/panda.json` from the model the arm serves.

## Run limits

The sim arm's limits (30°/s and 60°/s²) don't reach the kinematic model. Every run passes
`--max-joint-deg-s 30 --max-joint-deg-s2 60`. The start joints, in radians, are `0,0,0,-1.5708,0,1.5708,-0.785`.

## A/B runs

Run each for `--solver dls` and `--solver qp`. Summaries print at the end; `--csv` saves the series.

Joint-limit orbit:

```
go run ./cmd/cli --address localhost:8083 --insecure --arm pick-arm --solver <s> --max-joint-deg-s 30 --max-joint-deg-s2 60 --start-joints 0,0,0,-1.5708,0,1.5708,-0.785 --orbit-deg-s 5 --orbit-deg 420 --csv .runs/qp-panda-orbit5-<s>.csv run orbit
```

Why these values: 8°/s for 320° stops about 0.9° short of joint 0's limit, and 420° exceeds what joints 0 and 2 can reach together, so the target ends up out of reach for both solvers.

Posture hold (`qp` only), first with q₀ at the start joints, then with q₀ moved 20°:

```
go run ./cmd/cli --address localhost:8083 --insecure --arm pick-arm --solver qp --max-joint-deg-s 30 --max-joint-deg-s2 60 --start-joints 0,-0.3,0,-2.0,0,1.8,0.785 --posture-weight 0.001 --hold 3s run hold
go run ./cmd/cli --address localhost:8083 --insecure --arm pick-arm --solver qp --max-joint-deg-s 30 --max-joint-deg-s2 60 --start-joints 0,-0.3,0,-2.0,0,1.8,0.785 --posture-weight 0.001 --posture-deg 20 --hold 3s run hold
```

Circle:

```
go run ./cmd/cli --address localhost:8083 --insecure --arm pick-arm --solver <s> --max-joint-deg-s 30 --max-joint-deg-s2 60 --start-joints 0,0,0,-1.5708,0,1.5708,-0.785 run circle
```
