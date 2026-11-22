# Learned policy in sim: expert-gated DAgger, round 1 — Design

Date: 2026-11-22
Builds on:
- sub-project 2 and its follow-ups: `docs/superpowers/specs/2026-11-21-diff-ik-policy-act-design.md`, results in
  `docs/results/2026-11-policy-act.md` (SmolVLA v2/v3/v4 at 20/22/23 of 30; the per-seed failure pattern; the
  wrist-camera occlusion in seeds 9101 and 9110);
- the cell `dev/bc-pick-mujoco.json` and the clients `bc_pick.py`, `expert_pick.py`, `policy_run.py`,
  `policy_exec.py`;
- `~/src/viam-lerobot-training` (PR #2 `early-stop-best`; local `lora-packaging-recipes` stacked on it);
- the registry model `diff-ik-pick-smolvla` version 4 (SmolVLA v4, 448 px, trained on `diff-ik-pick-v3-200`) in org
  DevRelDemos (`16518049-9dd3-479f-9644-c5d112aa42d8`).

## Why

SmolVLA stopped improving with more uniform data: 448 px and doubling the demonstrations to 198 episodes moved it from
20 to 22 to 23 of 30, within noise. Its failures cluster where the uniform demos are thin (the edges of the block
area). DAgger puts new data exactly in the states the policy reaches and fails from. The sim gives an oracle (true
block pose) and an expert that can finish the task, so corrections cost no human time.

## Goal

One DAgger round on SmolVLA v4 that answers: do expert corrections from the policy's own failure states improve it
more than the same amount of extra training on the existing data?

## Non-goals

- More than one round (decided after this round's gate).
- ACT or diffusion DAgger (ACT v2 was the fallback had fine-tuning from the registry checkpoint not been possible).
- Fixing the wrist-camera occlusion or the table texture: the cell stays as is so v4's 23/30 stays a valid baseline.
- Relabeling every visited state (classic DAgger), human-gated takeover, edge-oversampled uniform captures.

## Design

### 1. Takeover trigger: `clients/python/dagger_trigger.py` (new, pure)

`check(state, hist, cfg) -> str | None` returns the trigger kind or `None`. `state` holds the TCP position (world mm,
from `bc_pick.tcp_world(pose)`, see §2),
the block pose (world mm, from the sim), the gripper's commanded `input` (0/1), `holding` (firm grasp), and time since
the policy started. `hist` holds recent states (enough for the stall window). Kinds, checked in this order:

| kind | condition (defaults are CLI flags) |
|---|---|
| `drop` | holding in an earlier state, not holding now, and the block at rest (two consecutive 10 Hz samples, 0.1 s apart, under 1 mm apart) and not in the bin (`bc_pick.in_bin`). A block still falling after a release over the bin is not a drop. |
| `empty_grasp` | gripper commanded closed for ≥ 1.0 s without a firm grasp |
| `approach_stall` | not holding; TCP more than 15 mm from the grasp point (block centre + `GRASP_ABOVE_CENTER`), and its distance to it shrank by less than 10 mm over the last 2.0 s |
| `carry_stall` | holding; TCP more than 15 mm (xy) from the release point above the bin, and its distance to it shrank by less than 10 mm over the last 2.0 s |
| `timeout` | no success after 25 s |

The stall checks need a full 2 s window before they can fire, so a policy's slow start is not a stall. Time and
history start at the policy's `t0` (after the move to the start pose), not at the reset.

### 2. Expert from any state: `expert_pick.py` and `bc_pick.py`

Two new `bc_pick` helpers, so the expert and the trigger can read any pose the policy leaves, tilted or not:

- `six_tuple_from_pose(pose)`: the inverse of `pose_map`. It turns a Viam flange pose (arm frame, orientation vector)
  into the expert's `(x, y, z, theta, tx, ty)`: orientation = tool-down · rotvec(tx, ty, 0) applied as `pose_map`
  does, solved for theta and (tx, ty). Tested by round-tripping random tuples (tilt sizes in [`TILT_MIN_DEG`, 10°]) through `pose_map`, plus a tool-down
  pose that returns zero tilt (`pose_map` snaps tilts under `TILT_MIN_DEG` to tool-down).
- `tcp_world(pose)`: the TCP in world mm for any flange orientation (flange + R · (0, 0, `TCP_MM`) + `ARM_BASE`).
  Equals `flange_from_tcp`'s inverse for a tool-down pose (tested).

`finish_from` starts from `six_tuple_from_pose(servo_start's pose)`; its first segments return the tilt to the
expert's own values.

`Episode.run` is split so the scripted run from the start pose and the takeover share one tail:

- `finish_from(cur, holding, sc)`: if not holding, open the gripper, rise vertically to pregrasp height (clears the
  block and the bin), then the existing pregrasp → grasp → firm-grasp wait → lift → over-bin → release → retreat →
  in-bin check; if holding, lift (if below lift height) → over-bin → release → retreat → check.
- `run(seed, tags)` keeps its behaviour and its RNG draws: v2 seeds produce the same episodes as before.

### 3. Rollouts with takeover: `clients/python/dagger_run.py` (new)

- `policy_run.run_episode` gains an optional `stop` callback object. `run_episode` calls `stop.begin(t0)` when the
  policy starts (after `ex.start`), and `stop.check()` once per chunk (after `play` returns or the async replan wait
  ends). A non-`None` result is the trigger kind: it ends the loop like a timeout, and the row records it as
  `trigger`. Without the callback, behaviour is unchanged.
- `begin(t0)` starts a background task that samples the state for the trigger at 10 Hz (flange pose → `tcp_world`,
  block pose, gripper input, holding) on the side connection `success_task` already uses; `check()` evaluates
  `dagger_trigger.check` on the latest samples. The task is cancelled when the episode ends.
- `run_episode`'s `finally` already closes the policy executor (`servo_stop`), so `dagger_run` does not close it again.
  On a trigger, `dagger_run` then opens an expert `servo_start` session in pose
  mode from the current pose, starts capture on the `demo-capture` sensor with tags
  `cmd:<instruction>`, `episode:dagger1-<seed>`, `dagger:r1`, `trigger:<kind>`, `expert:v2-takeover`, and runs
  `finish_from`. Capture stops when the expert finishes or fails. Only the expert segment is captured.
- A rollout the policy finishes on its own is logged with no capture. A rollout that ends on an observe, infer or
  session error without a trigger is logged with its errors, with no takeover.
- Every rollout is a JSONL row (seed, outcome, trigger, `kept`, `t_start`, tags) in
  `.runs/policy-dagger/dagger1.jsonl`, in the format `expert_pick add` reads (captured rows carry `tags`, `kept` and `t_start`), so
  `expert_pick.py add --log .runs/policy-dagger/dagger1.jsonl` builds the dataset unchanged. A correction is `kept` only when the expert's in-bin check passes.
- Same flags as `policy_run` (module mode, sync cadence by default), plus the trigger thresholds, `--max-kept`
  (default 80) and `--seeds` (default 3000-3299).

### 4. Collection

- SmolVLA v4 served by `vla:policy`, module/sync, on seeds 3000 onward, the same uniform layout as training. Eval seeds
  9100–9129 stay held out.
- Stop at 80 kept corrections or 300 rollouts, whichever comes first.
- Dry run first: 5 rollouts with capture, then convert the sequences with the same converter settings as v3
  (`--action-space ee --gripper-key input`), and check the episode count, lengths, task text and that the first frame
  is the takeover state.
- Datasets: `diff-ik-pick-dagger-r1` (corrections only, via `expert_pick add`) and `diff-ik-pick-v3-200-dagger1`:
  a new sequence dataset filled by copying the 198 sequence ids of `diff-ik-pick-v3-200` (`6b0316ea770b27184e6eb985`)
  and of `diff-ik-pick-dagger-r1` with `add_sequences_to_dataset` (a small script in the scratchpad, or the MCP tool).

### 5. Fine-tuning from the registry checkpoint: viam-lerobot-training

- `--pretrained viam:<org_id>/<model_name>@<version>` is resolved right after argument parsing (before
  `train_flags` and `check_checkpoint`, which then see an ordinary local path): it downloads the registry ML model package with the job's
  `API_KEY` / `API_KEY_ID` environment variables (set by Viam training jobs: an org-scoped key with the submitter's
  permissions), unpacks it, finds the shallowest folder holding `config.json` (a `tokenizer/` folder can hold its own), and uses that path as the base checkpoint
  (`--policy.pretrained_path` for SmolVLA). Any other `--pretrained` value behaves as today.
- Missing credentials, an unknown package, or a package without `config.json` fail before the dataset conversion with
  a clear error.
- Unit test with a mocked package lookup and download. Branch on PR #2's branch; a PR needs the user's OK.
- **Local check before any billed job:** `viam packages export` v4, fine-tune one step locally from the unpacked folder
  on a small export of `diff-ik-pick-v3-200-dagger1`, and confirm (a) the weights load (loss at step 1 is near v4's
  final training loss, not the base model's), and (b) the normalization statistics come from the new dataset.

### 6. Training: two cloud jobs

Both from `viam:16518049-9dd3-479f-9644-c5d112aa42d8/diff-ik-pick-smolvla@4`, identical flags apart from the dataset:
448 px, 10k steps, `last` checkpoint, v4's batch size and SmolVLA's class-default schedule (no scheduler flags). LeRobot's
cosine scheduler scales itself when `--steps` is below its decay steps (warmup 1000, decay 30k, LR 1e-4 → 2.5e-6), so a
10k run gets about 333 warmup steps and reaches 2.5e-6 at 10k. Both jobs get the same schedule; confirm the scaling
from the scheduler's "Auto-scaling" log line in the local check (§5) or the job log.

- **A (DAgger):** `diff-ik-pick-v3-200-dagger1`.
- **B (control):** `diff-ik-pick-v3-200`.

Each job is a billed submission and needs the user's OK before it is submitted. Jobs are judged by their logs (the
Viam status bug can mark a running job FAILED).

### 7. Gate

- Both checkpoints gated module/sync on seeds 9100–9129 with `policy_run.py`, as for v4.
- Report: the success counts against v4's 23/30, the paired metrics, and the per-seed table for 9101, 9110, 9111, 9115
  and 9124.
- DAgger earns its keep if A beats B by more than noise (at 30 seeds, a gap of 4 or more successes). Otherwise the
  result is recorded as negative, with the trigger counts by kind.
- Results go into `docs/results/2026-11-policy-dagger.md`.

## Testing

- `dagger_trigger`: table tests for every kind, the order of precedence, and the 2 s warm-up (no stall fires before
  the window fills).
- `six_tuple_from_pose` and `tcp_world`: round-trip tests through `pose_map` and against `flange_from_tcp`.
- `drop`: a table case "released over the bin, block still falling" that must not fire.
- `--pretrained viam:`: unit test with a mocked package lookup and download, including the shallowest `config.json`.
- `finish_from`: a sim smoke run from forced mid-task states, one not holding (arm displaced from the block) and one
  holding (after a scripted grasp), both ending with the block in the bin; and a check that `run` still produces the
  v2 scene for a fixed seed.
- `run_episode` with a `stop` callback: a unit test with fake executor and policy objects that the loop ends with the
  trigger recorded.
- The 5-rollout dry run in step 4 before the full collection.

## Risks

- **Few triggers.** At 77% success, 300 rollouts give roughly 70 failures, so 80 kept corrections may need all 300
  seeds. The count is reported either way, and the round proceeds with what was collected.
- **Corrections are short and start mid-task.** Mixed with 198 full episodes, they are a small share of frames. Not
  reweighted in this round; noted if A ≈ B.
- **Normalization drift.** The new dataset's statistics differ slightly from v3-200's. The local check (step 5) confirms
  which statistics the fine-tune uses.
- **Takeover from the policy's servo session.** The handoff closes one session and opens another, so the arm briefly
  holds. That gap is not captured.
