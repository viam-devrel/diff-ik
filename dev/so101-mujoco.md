# SO-101 in MuJoCo: Phase 4 moving-tag sweep

Spec: `docs/superpowers/specs/2026-11-17-diff-ik-phase4-tag-sweep-design.md`. Results: `docs/results/2026-11-phase4-tag-sweep.md`.

## Prerequisites

- **viam-mujoco-sim,** branch `so101-asset`, in `~/src/viam-mujoco-sim-so101`.
- **`viam:apriltag` from source** in `~/src/apriltag` (`git clone https://github.com/viam-labs/apriltag`). The cell starts it through `dev/apriltag-run.sh`, because registry modules don't load from a local-only config.
- **The tag texture:** `dev/assets/tag36h11_0.png`. Regenerate it with `uv run dev/make_tag_png.py`.

## Start the cell

```
go build -o bin/diff-ik ./cmd/module
viam-server -config dev/so101-mujoco-tag.json        # port 8082
```

## Pre-position the arm (after every fresh start)

The wrist camera only sees the tag once the arm is over it. Use the scene camera to bring the arm over the tag at the circle's start point, (280, 0) mm in the arm frame. `tag_sweep.py` places the tag there itself, so you can run the sweep once with the scene camera, or move it there with a `set_prop_pose` call first. Then run:

```
uv run clients/python/tag_servo.py --address localhost:8082 --insecure --follower arm --camera scene-cam \
  --tracker scene-tags --axis down --hover-mm 150 --tip-mm 98 --joint-deg-s 90 --linear-mm-s 120 --duration 2
```

## Sweeps and report

```
uv run clients/python/tag_sweep.py --insecure --camera wrist-cam --tracker wrist-tags --csv .runs/p4-wrist
uv run clients/python/tag_sweep.py --insecure --camera scene-cam --tracker scene-tags --csv .runs/p4-scene
uv run clients/python/tag_sweep_report.py .runs/p4-wrist .runs/p4-scene --md .runs/p4-table.md
```

**Defaults** (`tag_sweep.py --help`):
- address `localhost:8082`, follower `arm`;
- `--axis down`, hover 150 mm, `tip_mm` 98;
- joint speed 90°/s, TCP speed 120 mm/s;
- speeds `10,20,40,80`, 3 laps, radius 60 mm, centre `220,0`.

A sweep takes about 3.6 minutes. Add `--ema-alpha 1` to turn off the client's target smoothing.
