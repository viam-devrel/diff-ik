# Learned policy in sim, sub-project 2: ACT and the deployment comparison — Design

Date: 2026-11-21
Builds on:
- sub-project 1: `docs/superpowers/specs/2026-11-21-diff-ik-policy-sim-data-design.md`, its results
  `docs/results/2026-11-policy-sim-data.md` (see "Notes for sub-project 2"), the cell `dev/bc-pick-mujoco.json`, and
  the clients `expert_pick.py`, `bc_pick.py`, `policy_layout.py`, `policy_run.py`;
- `servo_trajectory` (diff-ik v0.4.0) and the emulated-policy playback strategies in `clients/python/policy_stream.py`;
- sibling repos: `~/src/viam-sequence-to-lerobot` (branch `diff-ik-policy-sim`: multi-arm, `--grippers`,
  `--gripper-key`, `--action-space ee`), `~/src/viam-lerobot-training` (pins LeRobot `02655292` and the converter at
  `c60e8b3`), `~/src/viam-vla-inference-service` (`viam-labs:vla:policy` 0.3.0, bundles an older LeRobot), and
  `~/src/viam-mujoco-sim` (0.20.0 plus two local patches: gripper `position`, camera `captured_at`).

## Where this fits

The project trains visuomotor policies in mujoco-sim from a scripted expert and deploys them through diff-ik to test
whether one `servo_trajectory` per chunk holds up under a trained policy. Settled for the whole project: UR5e + 2F-85
pick-to-bin, one instruction ("Pick up the red block and place it in the bin"), absolute flange pose + gripper actions,
Viam training jobs, a runner in diff-ik, ≥70% success before comparing deployment modes.

1. Data (done).
2. **This one: ACT and the deployment comparison.**
3. Latency extremes: diffusion, SmolVLA, expiry behaviour, the emulated-network repeat.

## Goal

An ACT policy trained by a Viam training job on a fixed v3 dataset, served by `vla:policy`, that succeeds on ≥70% of
30 held-out seeds through `servo_trajectory`; then a comparison of three deployment modes (module `servo_trajectory`,
careful client playback, builtin `Move` per row) under sync and async inference.

## Non-goals

- Diffusion, SmolVLA, network emulation, expiry tests (sub-project 3).
- Visual, lighting or distractor randomization and recovery demonstrations, unless the gate misses (then they are the
  last fallback, and the user is asked first).
- Naive client playback (dropped from the comparison by the user).
- Releasing the mujoco-sim patches (separate work in that repo).

## Approach: two tracks

- **Track 1 (upstream):** PRs to the converter, the training script and `vla:policy` so a Viam training job can build
  the `ee` layout and its checkpoint loads in the module. Each PR is pushed only with the user's OK.
- **Track 2 (local, meanwhile):** capture v3, train ACT locally on MPS at the training pin, and build the runner and
  evaluation against that checkpoint. A local checkpoint near 70% proves the data before anything is billed.
- **After the merges:** a Viam training job on v3, deployed through `vla:policy` from the registry. Headline numbers
  come from this checkpoint only; local numbers are a cross-check.

## Decisions

### v3 dataset (`diff-ik-pick-v3`, `diff-ik-pick-v3-eval`)

| Topic | Decision |
|---|---|
| Gripper action | Convert with `--gripper-key input`: the gripper column is the latched 0/1 command (`grab` 1, `open` 0). The converter builds state and action from the same key (action = next state), so the state's gripper value is the latch too; the cameras show whether the block is held. The runner reads `input` from the gripper's `{"command": "position"}` reply for `state_vector`, and closes when the action is above 0.5, opens below. No converter change. |
| Tilt randomization | At each transit waypoint (start, pre-grasp, lift, above-bin) the expert tilts the tool up to 4° off vertical at a random azimuth, applied as a small rotation on top of the tool-down orientation with the waypoint's yaw (`theta`) unchanged. The grasp face choice and `theta` are computed as today, on the untilted pose. Grasp descent and release stay vertical. Waypoint poses carry a full orientation (scipy `Rotation`); segments interpolate position min-jerk and orientation by slerp on the same min-jerk phase (replacing the `(x, y, z, theta)`-only interpolation and `bc_pick.pose_map`'s fixed `o = (0, 0, -1)`). Tilts are drawn in `sample_scene` from the seed, so a seed always gives the same episode. Target: r02/r12 std about 0.03 (was 0.002/0.001). |
| Episodes | 100 training (seeds 2000–2099), 30 eval (seeds 9100–9129), captured in batches with the existing `add`. The eval set measures expert success and supplies the closed-loop layouts; it never trains. |
| Episode start | Expert and policy episodes both start at the seed's randomized start pose (servoed there, settle 0.5 s). The policy never starts from `home`, which removes the smoke test's 345 mm first-chunk jump; no warm-up logic. |
| Check before full capture | 10 expert episodes succeed 10/10 with tilt; local conversion's `meta/stats.json` shows the new r02/r12 std. User check-in. |
| Unchanged | Visuals, lights, cell, cameras (448 px, converted to 224), capture rate 10 Hz, the instruction. v2 and v2-eval are kept. |

### Training

| Topic | Decision |
|---|---|
| Converter PR | `viam-labs/viam-sequence-to-lerobot`: rebase branch `diff-ik-policy-sim` onto `origin/main` (`c60e8b3` squash-merged the `bimanual` commits the branch was built on), so the PR carries only the `ee` commits (`6eae2ce`, `03f5a1c`); re-run the tests after the rebase. |
| Training script PR | `viam-lerobot-training`: bump `ARG CONVERTER_REF` in `container/Dockerfile` to the merged commit; add `--action_space` to `model/args.py` (`--grippers` and `--gripper_key` exist; one arm only is fine); in `ee` mode the `cloud.py` preflight requires `(arm, "EndPosition")` instead of `JointPositions`. If the vla PR doesn't land, strip config fields the module's LeRobot rejects (`dtype`, `optimizer_fused`) at export. |
| Training container and script release | After the PR merges, per that repo's README (sections 4–5): an amd64 `docker buildx` build (needs the HF token secret), a push to the public image registry, `viam train containers register`, and a new training-script version upload. All of it is publishing: confirmed with the user first, and done by whoever owns those credentials if not us. |
| vla PR | `viam-vla-inference-service`: bump its LeRobot to `02655292`. If that drifts too far, fall back to the export strip and leave the bump as follow-up. |
| ACT settings | LeRobot defaults (ResNet-18, both cameras 224×224, state 10), `chunk_size` 20 (2 s at 10 Hz), `n_action_steps` 20, batch 8, about 20k steps, checkpoint every 5k. The runner, not the policy, decides how much of a chunk plays. |
| Local training | The existing Python 3.12 venv at `02655292`, on v3 converted locally with the converter branch (the same data the job sees). Pick the checkpoint by closed-loop success on 10 eval seeds, not loss. Served by the local `vla-policy` with the compat strip until the vla PR lands. |
| Cloud training | One Viam training job on `diff-ik-pick-v3`, same settings, confirmed with the user before submitting (billed); retries also need the user's OK. Output is a registry ML model package; `dev/bc-pick-mujoco.json` points `vla-policy` at it (fixes the machine-local path). If the cloud checkpoint is well short of the local one, compare the two configs before retraining. |
| Gate | The cloud checkpoint, module mode, sync cadence: ≥21 of 30 eval seeds. Fallbacks in order: more steps; more v3 episodes (new seeds); visual/recovery levers (ask first). |

### Runner (`clients/python/policy_run.py`, `policy_exec.py`, `policy_report.py`)

```
uv run clients/python/policy_run.py --mode module|client|move --cadence sync|async --replan-every 10 \
    --seeds 9100-9129 --out .runs/eval/<name>.jsonl
```

Episode loop:
1. Reset the world; lay out the block for the seed. The reset and `set_prop_pose` calls are inline in `expert_pick.Episode.run` today; they move into a function (`expert_pick.layout(seed)` plus a `place` helper) that both the expert and the runner call.
2. Servo to the seed's start pose, settle 0.5 s.
3. Observe: both cameras (resized to the specs' input shape), flange pose, jaw; stamp `t_obs`.
4. Infer (`vla-policy`, 5 s timeout): 20 rows of flange pose + gripper.
5. `executor.play(chunk, t_obs)`. Sync: wait until the chunk has played, then 3. Async: after `replan-every` rows
   (1 s), go to 3 while the chunk plays; the new chunk replaces the rest of the old one.
6. End: success (block inside the bin footprint, below the rim, at rest, gripper open), timeout (30 s), or failure.

Executors share `play(chunk, t_obs)` and `close()`:
- **module:** one pose session per episode; each chunk is one `servo_trajectory` with `dt_ms` 100, `age_ms` =
  now − `t_obs`, `blend_ms` 200, and `lead_ms` explicit until the lag fix below is released.
- **client:** the aligned + blend strategy from `policy_stream.py`, moved into a function both files import: a 30 Hz
  task samples the current chunk at its age-aligned time, crossfades 200 ms into a new chunk, sends `servo_target`.
- **move:** one blocking builtin `Move` per row; a new chunk replaces the queued rows. No session.

Gripper: in every mode the runner calls `grab`/`open` when the action column crosses 0.5. In module and client modes
it fires at that row's scheduled time shifted by the lead the arm uses; in move mode, when that row's `Move` starts.

Metrics, one JSONL row per episode: success, outcome reason, time to success, arm command messages per second,
`peak_accel_ratio` (module, client), jerk (RMS third difference of the measured flange position, sampled at 30 Hz by
a side task in every mode), row0-to-state distance per chunk, chunk-seam gap at each replacement, inference latency.
`policy_report.py` turns the JSONL files into the results tables.

### diff-ik module fixes (released together as v0.4.1)

1. The `servo_trajectory` reply reports the new chunk's own `lead_ms` and `remaining_ms` (it returns the stale
   pre-tick session status today).
2. The lag estimate (`lagTicks`) uses a median of recent lag samples instead of the jump-up / ease-down estimate that
   reads 40–60 ms high. It feeds both the trajectory auto-lead and `expected()` (the lag-compensated measured-joint
   prediction); both paths get tests. The servo doc's "send `lead_ms` explicitly" advice goes away.

Releasing v0.4.1 (merge to `main`, tag, push) is confirmed with the user at the time.

### Evaluation

| Stage | Checkpoint | Modes × cadence | Episodes |
|---|---|---|---|
| Local gate | best local | module × sync | 10 per candidate, then 30 |
| Cloud gate | cloud | module × sync | 30 (≥21 to pass) |
| Comparison | cloud | {module, client, move} × {sync, async} | 6 × 30 = 180 |

Reading it: a 30-episode success rate is good to about ±15 points, so differences under about 20 points aren't
called real. The main evidence is paired per-seed metrics (jerk, seam gap, row0-to-state, messages per second). The
question: does async `servo_trajectory` keep sync's success while being smoother than the careful client?

### Error handling

Per episode, recorded and the run continues: infer error or timeout (`infer`), session fault or `stopped_timeout`
(the state), `Move` planning error (`plan`), gripper error (`gripper`). The world resets and the next seed runs. The
run aborts only when the machine is unreachable or `vla-policy` specs don't match the layout (existing checks).

## Testing

Pure-function checks, `assert`-style like the other clients: async replacement (which rows survive a new chunk),
the gripper crossing schedule, the success predicate, the jerk metric, the shared playback function (and
`test_policy_stream.py` still passing). Go: one test per module fix. The expert's tilt sampling gets a check that
grasp and release waypoints stay vertical and transit tilt stays ≤4°.

## Deliverables

- This repo: expert tilt + episode-start changes; `policy_run.py`, `policy_exec.py`, `policy_report.py` and tests;
  the shared playback function; the two Go fixes (v0.4.1); `dev/bc-pick-mujoco.json` on the registry package;
  `docs/results/2026-11-policy-act.md`.
- Upstream: three PRs, each pushed with the user's OK; the training container build/register and script upload, with the user's OK.
- Viam: datasets `diff-ik-pick-v3` and `-v3-eval`, one training job (more only with OK), the model package.

## Check-ins

1. After v3's 10-episode check and stats, before the full capture.
2. Before each upstream PR is pushed, and before the training container is built, pushed and registered.
3. After the local gate.
4. Before each billed training job.
5. After the cloud gate.
6. At the end, with the comparison.

The `diff-ik-policy-sim` machine is restarted when needed and stopped at check-ins that wait on the user.
Credentials stay in the scratchpad, never printed or committed.
