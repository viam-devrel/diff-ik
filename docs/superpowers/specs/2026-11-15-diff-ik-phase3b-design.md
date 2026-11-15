# Diff-IK Phase 3b — SO-101 under MuJoCo physics: Design

Date: 2026-11-15
Builds on: `2026-11-15-diff-ik-phase3a-design.md`, and results in `docs/results/2026-11-phase3a.md`.
Sim-side spec: `viam-mujoco-sim/docs/superpowers/specs/2026-11-15-so101-asset-design.md`. It's a local file, because that repo ignores `docs/`. The sim work is on branch `so101-asset` in the worktree `~/src/viam-mujoco-sim-so101`, so the `~/src/viam-mujoco-sim` checkout that the UR5e machine runs stays on `main`.

## Goal

Run the Phase 3a validation against an SO-101 with physics and streaming lag: the bundled `asset: "so101"` in viam-mujoco-sim, which uses the upstream STS3215 servo dynamics. Then measure what the kinematic `so101:simulated` arm couldn't show.

## Scope

- **viam-mujoco-sim** (its own spec): the bundled `so101` arm and gripper.
- **diff-ik** (this spec, branch `diff-ik-phase3b`):
  - `dev/so101-mujoco.json`: a local, cloud-free config with `"network": {"bind_address": "localhost:8082"}`, so the MuJoCo UR5e machine on 8080 and the kinematic SO-101 sim (run on 8081 for 3a via a port override; `dev/so101-sim.json` defaults to 8080) are untouched. It contains:
    - `mujoco-sim` (local `~/src/viam-mujoco-sim-so101/run.sh`, the worktree) and `diff-ik` (local `bin/diff-ik`);
    - `sim-world`, a `table` obstacle, `arm` (`asset: so101`, `world: sim-world`, framed on world), `gripper` (`asset: so101`, `world: sim-world`, `mount: arm`, framed on `arm`), and `servo` (`arms: ["arm"]`).
  - A README line on running it with `--address localhost:8082 --insecure`.
  - `docs/results/2026-11-phase3b.md`.
- **No Go changes are expected.** If a run turns up a loop or surface bug, it gets fixed on this branch with a test, as in earlier phases.

## Validation runs

The runs match Phase 3a:
- pointing task;
- `--singular-eps 0.03 --max-damping 0.05`;
- start pose `q0a = (0, 0.3, 0.3, 0.5, 0)` rad (the asset's home keyframe);
- the loop's joint limit at 90°/s, with the sim arm at its default 180°/s.

Before each run, wait for any helper client's session to expire (Phase 3a finding).

| Run | Bar |
|---|---|
| `cli validate` | Jacobian ≤ 5% and FK ≤ 0.1 mm / 0.1°, both binding. With the fitted flange offset, the gap is set by the offset's 0.1 mm / 0.1° rounding, about 0.001 mm in the spec review. If FK misses by rounding alone, the write-up says so. |
| `run static --task pointing` | 10/10 converge to ≤ 2 mm / 2°; ≥ 49.5 Hz |
| `run singular --task pointing --wrist 2 --singular-deg -74` | reached; peak joint speed ≤ 1.1 × limit; ≤ 2 oscillations/s |
| `run circle --task pointing` (CLI, no feedforward) | baseline |
| Python bench `--task pointing` | smoke passes; dead-man ≤ 790 ms; circle mean ≤ 22.1 mm at `resync_deg` 15; the rest is baseline |
| Unreachable pointing via DoCommand (axis rotated 20° about base z) | `converged`; position ≤ 2 mm; residual within ±1° of `asin(sin20°·|a_xy|)` |

If a bar fails because of the physics model rather than the loop (for example the static 2 mm bar under force-limit or friction settling), the write-up says so and reports the measured floor. The bar is not quietly loosened.

## Measurements new in 3b

**What this sim can and can't show:**
- **Gravity sag:** none. The sim gravity-compensates every body, so sag is about zero by construction.
- **Servo dynamics:** the upstream STS3215 gains settle in about 70 ms, with about 1 ms of tracking lag, and upstream says they aren't a 1:1 match for the real servos.
- **So the measured lag comes from the sim's streaming pipeline,** like the UR5e's ~240 ms in Phase 1, **not from the SO-101's servos.** The 3c recommendations below are therefore sim numbers, and hardware has to re-derive them.
- **What it adds over 3a:** physical joint motion under force limits, friction and contacts, plus the sim's real streaming queue.

- **Streaming lag:** the shift between measured and commanded joints (cross-correlation over the static and circle CSVs, as in Phase 1), in ticks and ms.
- **Resyncs:** counts under the default `resync_deg` and under 15.
- **Stop shape:** travel after `servo_stop` from 50 mm/s, against the ideal ramp's ~5 mm (the Phase 2 UR5e cut the queue at 9.3 mm; Phase 3a's kinematic SO-101 ramped at 5.5 mm).
- **Settling:** steady-state position error after convergence, under the STS3215 force limits and friction. Sag is expected to be about 0 because of gravity compensation, so report it but don't read it as a hardware estimate.

## Results

`docs/results/2026-11-phase3b.md` gets a three-way table: SO-101 MuJoCo, against `so101:simulated` (3a), against UR5e MuJoCo (Phase 2). It also gives `resync_deg`, damping and joint-velocity values from the measured (sim) lag, labelled as sim-derived starting points for 3c.

## Out of scope

3c (hardware). Changes to the URDF importer.
