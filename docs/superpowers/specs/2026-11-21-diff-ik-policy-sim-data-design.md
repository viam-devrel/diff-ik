# Learned policy in sim, sub-project 1: scripted demonstrations and the dataset — Design

Date: 2026-11-21
Builds on:
- `servo_trajectory`, prototyped in `trajectory.go`, with results in `docs/results/2026-11-policy-streaming.md`;
- the Viam learning pipeline in sibling repos:
  - `viam:capture-control` (capture on demand);
  - `~/src/viam-sequence-to-lerobot` (a Viam sequence dataset export → LeRobot v3);
  - `~/src/viam-lerobot-training` (a Viam custom training script around `lerobot-train`);
  - `~/src/viam-vla-inference-service` (`viam-labs:vla:policy`);
- the mujoco-sim cells in `dev/`, and mujoco-sim itself (`~/src/viam-mujoco-sim`, release 0.20.0).

## The larger project, and where this fits

The goal is to train real visuomotor policies in simulation from a scripted expert, deploy them back into the same
simulation through diff-ik's `servo_trajectory`, and measure whether one command per chunk holds up under a trained
policy. The answer is API evidence first, and doubles as an end-to-end Viam pipeline demo.

Settled for the whole project:
- **Robot and task:** UR5e + Robotiq 2F-85 putting a red block in a bin, one instruction.
- **Policies:** ACT, diffusion and SmolVLA.
- **Training:** Viam custom training jobs, with local smoke runs.
- **Actions:** absolute end-effector pose plus gripper.
- **Deployment:** a runner in diff-ik first.
- **Quality bar:** at least 70% success before comparing deployment modes.

It's split into three sub-projects, each with its own spec and plan:

1. **This one: the data.** The cell, the scripted expert, Viam capture, the converter's `ee` action space, the
   dataset, and an end-to-end smoke test.
2. **ACT and the deployment comparison.** A training job, the full runner, and `servo_trajectory` against client
   playback.
3. **Latency extremes.** Diffusion, SmolVLA, the expiry behaviour, and the emulated-network repeat.

## Goal of this sub-project

A tested, reviewed dataset of scripted pick-to-bin demonstrations, captured through the Viam pipeline. Its layout must
match what `servo_trajectory` consumes, so nothing gets converted at deployment. The sub-project ends with a smoke test
that a policy trained on the data can drive the arm through `servo_trajectory`. Nothing in it is billed.

## Non-goals

- Training a good policy, cloud training, closed-loop evaluation, or the full runner (sub-projects 2 and 3).
- Randomizing visuals, lights or the bin between episodes, adding distractor objects, or expert noise with recovery
  demonstrations. These are the levers if sub-project 2 misses the 70% bar.
- Photoreal rendering. MuJoCo's rasterizer is the ceiling; Isaac Sim is out of scope.
- Changes to diff-ik's Go code. None are expected.

## Decisions

| Topic | Decision |
|---|---|
| mujoco-sim | Release **0.20.0**, from the registry, pinned. If the gripper reading needs the patch below, run a local build of 0.20.0 plus the patch instead, until it's released. |
| Machine | New cloud machine **`diff-ik-policy-sim`** in DevRelDemos / `test` (org `16518049-9dd3-479f-9644-c5d112aa42d8`, location `sr89pn2pma`), with viam-server on the laptop and its config mirrored in `dev/bc-pick-mujoco.json`. The user approved creating it and its API key. Credentials stay out of the repo. |
| Scene | UR5e on a 400 mm pedestal (as in `dev/ft-mujoco.json`), Robotiq 2F-85 (TCP frame z 155.8 mm), a table obstacle, the bundled bin (`meshes/bin.stl`, 150 × 150 × 80 mm) as a **fixed** obstacle, a red 40 mm cube prop `block` (about 50 g). The arm starts from `home_joints` (new in 0.20.0). |
| Cameras | `scene-cam`: fixed, front-above, **448 × 448**, the block's whole range and the bin in view. `wrist-cam`: on the arm, looking along the tool, 448 × 448, with `near_m` set so the jaws don't clip. `overview`: 1280 × 720, for people, not captured. |
| Render quality | `lighting` on the world: three lights (a warm directional key with shadows, a dim directional fill without, a spot rim), `headlight` 0.3, a gradient `skybox`, `shadow_size` 8192, `antialias` 8, `reflections` on. Textures: the bundled `textures/concrete.png` on the floor; a wood image on the table, generated once by a small script into `dev/assets/wood.png` (procedural grain, no external source; mujoco-sim bundles no wood texture); the bundled `textures/cardboard.png` on the bin. The block stays solid red. A before/after image pair (defaults against tuned, from `scene-cam` and `overview`) goes to the user for approval before capture. Visuals are fixed across episodes. |
| Expert | `clients/python/expert_pick.py`: one diff-ik **pose session** per episode (frame `arm_origin`, `gain` 8, `linear_mm_s` 300), streaming `servo_target` at 30 Hz along min-jerk segments. It reads the block's true pose from the world's `get_prop_pose`. |
| Flange, not TCP | A diff-ik session on `arm` servos the arm's kinematic end, the **flange**, and the arm's `EndPosition` is the flange too. The expert computes each grasp and place point for the gripper's tool point (TCP, 155.8 mm along the tool axis), then commands the flange pose (TCP minus 155.8 mm along tool z). The dataset state and action and every `servo_trajectory` pose are flange poses in `arm_origin`. A TCP pose is never written into the state or sent to the servo. |
| Expert phases | 1) Move to the randomized start pose, settle 0.5 s, `start_capture`. 2) Pre-grasp 100 mm above the block, yaw aligned to the block face nearest the tool's current yaw (never more than 45° of turn). 3) Descend to grasp height. 4) `grab`, then poll `IsHoldingSomething` until `meta.firm_grasp` (2 s time-out). 5) Lift 100 mm. 6) Move to 120 mm above the bin centre. 7) `open`. 8) Rise 50 mm, `stop_capture`. The last frame is the success state. |
| Randomization | Per episode, from a seed: block position uniform over a 250 × 200 mm table area clear of the bin, yaw uniform ±45°; start pose `home_joints` nudged ±30 mm and ±10° yaw (reached before capture starts); expert speed scale uniform 0.8–1.2. The same seed always gives the same scene. |
| Discards | An episode is discarded if the grasp times out, the block leaves the grasp, or the block isn't in the bin at the end. "Leaves the grasp" is detected by polling `IsHoldingSomething`'s `meta.firm_grasp` at about 5 Hz during the lift and the move to the bin (phases 5–6): false means discard. "In the bin": the block's centre is inside the bin's inner footprint, below its rim, and at rest. Capture is stopped, the sequence isn't added to the dataset, and the reason is logged. The discard rate goes in the dataset card. |
| Expert bar | At least 95% success over 20 seeds, without capture, before capture starts. |
| Capture | A `viam:capture-control` sensor at **10 Hz** over `scene-cam` and `wrist-cam` (`GetImages`), the arm (`JointPositions` and `EndPosition`) and the gripper (`DoCommand`), wired into the data manager, which syncs to the cloud. Each episode is `start_capture` with `{"tags": ["cmd:Pick up the red block and place it in the bin"]}`, then `stop_capture`, and becomes one sequence. |
| Gripper reading | The converter wants the gripper's opening, 0–100, from a `DoCommand` response, under a key (`--gripper-key`, default `position`), and stores it unscaled. mujoco-sim 0.20.0 exposes the jaw only in `IsHoldingSomething`'s `meta.jaw`, in radians, which capture can't feed to the converter. **Plan on the patch**: Task 0 only confirms the gap. Add `{"command": "position"}` to the mujoco-sim gripper, returning `{"position": 0–100 (100 = open), "input": 0 or 1}`, with a unit test, on a local branch `diff-ik-policy-sim` in `~/src/viam-mujoco-sim`. The user approved this change. |
| Datasets | Sequence dataset **`diff-ik-pick-v2`** (50 kept episodes, seeds from 1000) and **`diff-ik-pick-v2-eval`** (10 kept episodes, seeds from 9000, never trained on). A change to the expert's logic or its instruction means a new version. `-v1` held the 5 review episodes under the first instruction ("put the red block in the bin"); the user switched to a SmolVLA-style instruction after reviewing them, so capture restarts as `-v2`. |
| Capture check | On export, every frame needs every reading within 50 ms (the converter's tolerance). If more than 2% of frames miss, try 5 Hz first. The capture rate sets the action step everywhere downstream: `dt_ms` = 1000 / rate (100 at 10 Hz, 200 at 5 Hz). Failing that, use a local LeRobot dataset writer in the expert script that bypasses Viam capture, and record the switch. |
| Converter | In `~/src/viam-sequence-to-lerobot`, on a local branch `diff-ik-policy-sim`: **`--action-space ee`**. **State** (10 numbers): `[x, y, z]` in mm from the arm's `EndPosition` (in its base frame), the rotation matrix's first two rows (`r00 r01 r02 r10 r11 r12`), and the gripper opening 0–100. **Action:** the next frame's state, in the same layout. One arm, at most one gripper (handled as `joints` mode does). Frames whose next frame spans an alignment gap are skipped (as `delta-ee` does). Images are `observation.images.scene` and `observation.images.wrist`, resized to 224 by the existing `--image-size`. Tests are written first. README updated. |
| Smoke test | 1) Convert the first 5 episodes (`--action-space ee`). 2) Run `lerobot-train` locally: ACT, `chunk_size` 20, `n_action_steps` 20, 300 steps, on MPS (or CPU). The loss must fall. 3) Load the checkpoint in a `viam-labs:vla:policy` service on the sim machine (`device: mps`); `specs` must report the two image keys and the 10-number state. 4) `clients/python/policy_run.py --once`: one observation → `infer` → chunk to poses → one `servo_trajectory` (frame `arm_origin`, `dt_ms` = 1000 / capture rate, explicit `lead_ms`). The arm must move smoothly, toward poses inside the workspace. Task success isn't required. |
| Runner helpers | `policy_run.py` gets its pure helpers now, with tests: `state_vector(end_pose, gripper)` in the converter's layout, and `chunk_to_poses(actions)`, which rebuilds the rotation from the two rows (normalize, orthogonalize, third row as their cross product) into Viam's orientation vector. A round-trip test pins the layout against the converter's code path. The full runner (loop, gripper, evaluation) is sub-project 2. |
| Results | `docs/results/2026-11-policy-sim-data.md`: the cell (with the before/after renders), the expert bar, the capture check (skew), a dataset card (episodes, frames, lengths, discard rate and reasons, randomization ranges, seeds), the converter mode, and the smoke test. |

## Components

| Unit | Does | Depends on |
|---|---|---|
| `dev/bc-pick-mujoco.json` | The cell and services; mirrors the cloud config | mujoco-sim 0.20.0 (or the patched build), diff-ik `bin/diff-ik`, capture-control |
| `clients/python/expert_pick.py` | Scene sampling, the scripted pick-to-bin, the success check, capture start/stop, the discard log, adding kept sequences to the dataset | diff-ik `servo` (pose session), the world's prop commands, the gripper, the capture-control sensor, the data API |
| `clients/python/policy_run.py` (helpers and `--once`) | Layout helpers; the one-chunk smoke check | the policy service's `infer`, diff-ik `servo_trajectory` |
| mujoco-sim gripper `position` (only if needed) | The gripper opening 0–100 over `DoCommand` | the gripper model |
| `viam-sequence-to-lerobot --action-space ee` | The export → LeRobot v3 dataset in the `ee` layout | the export format |

## Data flow

```
expert_pick.py ─ servo_target 30 Hz ─► diff-ik ─► UR5e ─► MuJoCo 0.20.0
     │ start/stop_capture, sequences
     ▼
capture-control (10 Hz) ─► data manager ─► diff-ik-pick-v2 ─► viam dataset export
                                                 │
                     viam-seq-to-lerobot --action-space ee ─► LeRobot v3 (224 px)
                                                 │
                lerobot-train (local, 300 steps) ─► checkpoint ─► viam-labs:vla:policy
                                                 │
policy_run.py --once ─ infer ─► chunk ─► one servo_trajectory ─► diff-ik ─► UR5e
```

## Testing

- **Python clients:** a `test_*.py` of pure helpers each, runnable with `uv run`, in the style of the existing ones.
  - `test_expert_pick.py`: scene sampling (ranges; same seed, same scene), grasp yaw (never more than 45°), min-jerk
    segments (exact ends, monotonic progress), the in-bin check.
  - `test_policy_run.py`: the state layout, and `chunk_to_poses` round trips including noisy rotation rows.
- **Converter:** its own test suite, with new tests for `ee` (layout, rotation rows against a known pose, the gripper's
  0–100 value passed through unscaled with 100 = open, gap skipping, a tiny fixture export end to end).
- **mujoco-sim:** a unit test for the gripper `position` command, if it's added.
- **diff-ik:** no Go changes expected. If any happen, `go test ./...`, `go test -race ./...`, `go vet ./...` and
  `gofmt -l .` must be clean.

## Risks

- **Capture skew** with two 448 px cameras plus pose at 10 Hz on a laptop. Mitigated by the camera render thread in
  0.20.0, then by 5 Hz, then by the local writer.
- **The capture-control setup:** model names, attributes and the data manager wiring come from its docs and from
  `~/src/robot-demo-capture-controller`'s README. Task 0 confirms them against a running machine before the cell is
  final.
- **Adding sequences to a dataset from a script:** if the Python SDK can't do it, the expert prints each kept episode's
  time range and the dataset is filled with the `viam` CLI. Task 0 checks this.
- **The policy service on the Mac:** `viam-labs:vla:policy` must load a small local ACT checkpoint on MPS. If it can't,
  the smoke test calls the checkpoint through LeRobot directly, and the service question moves to sub-project 2.

## Done when

1. The cell runs on mujoco-sim 0.20.0 (or the patched build), and the user has approved the tuned visuals.
2. The expert succeeds at least 95% of the time over 20 seeds.
3. `diff-ik-pick-v2` has 50 kept episodes and `diff-ik-pick-v2-eval` has 10, with the dataset card written.
4. `--action-space ee` is tested, and the dataset converts and loads as a `LeRobotDataset` with sane statistics (no
   NaNs, gripper bimodal, position ranges matching the randomization).
5. The smoke test passes.
6. The results doc is written.

## Check-ins with the user

1. After Task 0: pipeline gaps, the gripper decision, and the render before/after.
2. At the 5-episode grid (last frames from both cameras), before the remaining capture.
3. At the end, before sub-project 2's brainstorm.
