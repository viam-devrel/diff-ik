# Learned policy in sim, sub-project 1: data — results

## Task 0: machine and pipeline checks

Date 2026-11-21. Laptop viam-server v1.17.0, `viam` CLI 1.17.0, viam-sdk 0.85+.

### Machine
- Machine `diff-ik-policy-sim`, id `25f31a32-828c-48eb-ad96-f2017532bd31`, part id `e731b4a1-9187-40d6-94a3-aa166b803114`,
  address `diff-ik-policy-sim-main.sr89pn2pma.viam.cloud`, machine-scoped API key `mujoco-sim-cell-20261121-002929`
  (key id `2285a021-5319-454e-8fe2-a8a7ec63b261`).
- viam-server must bind its own port: the other laptop machine holds `:8080`, so the config sets
  `network.bind_address` to `localhost:8087`.
- Modules resolved: `viam-labs:mujoco-sim` 0.20.0 (pinned); `viam:capture-control` `latest` resolved to **0.1.1**
  (pinned); `viam-labs:vla` `latest` resolved to **0.3.0**.

### Gripper `DoCommand` capture (Step 3): works with a fixed input
- `start_capture` at 10 Hz with the component-level `data_manager` service config
  (`additional_params.docommand_input = {"command": "measure"}`, `capture_frequency_hz: 0`) produced gripper
  `DoCommand` rows. Each row carries `additional_parameters.docommand_input` and
  `data.docommand_output.tcp_mm` (160.3), `jaw_gap_mm`, `finger_reach_mm`.
- The no-`additional_params` fallback was not needed, so what the module receives for an empty input was not tested.
- **Decision for Task 1:** the patched gripper answers `{"command": "position"}` with `{"position": 0-100, "input": ...}`,
  and the cell config sets `docommand_input: {"command": "position"}` on the gripper's `data_manager` service config.
  No need to answer `{}`.
- Side note: `measure` reports `tcp_mm` 160.3, not 155.8 (it is measured to the pad contact, so don't read it as
  the frame offset).

### Capture skew (Step 4): no skew at 10 Hz
- 22.1 s capture at 10 Hz tagged `probe:skew` (the stop landed early, not the planned 30 s): 222 readings in each
  of the 5 streams (`scene-cam`, `wrist-cam`, `arm/JointPositions`, `arm/EndPosition`, `gripper/DoCommand`).
- Per `scene-cam` frame, largest gap to the nearest reading of any other stream, on `time_requested`:
  **p95 = 1.0 ms, max 1.0 ms, 0.0% of frames over 50 ms.** Frame spacing mean 100.0 ms, max 101 ms.
- **Chosen rate: 10 Hz** (`dt_ms` = 100); 5 Hz repeat not needed.
- Caveat: gaps are on `time_requested` (the data manager's scheduling stamp). Binary rows have an empty
  `time_received`, so the real-render timing is not measured. Re-check on a real episode export.

### Sequences and datasets from a script (Step 5): the machine-scoped key is allowed
- With only the machine key: `data_client.list_sequences(organization_id=...)`, `create_dataset(type=DATASET_TYPE_SEQUENCE_DATA)`
  and `add_sequences_to_dataset` all succeed. `tabular_data_by_mql` and `binary_data_by_filter` also work.
  Sequences list with `part_id`, `sequence_tags`, `start_time`, `end_time`, `resources`, `dataset_ids`.
- The machine key is **not allowed** `delete_dataset` (PERMISSION_DENIED). The probe datasets were deleted with the
  CLI login instead. `ListSequences` has no tag filter, so filter client side by `part_id` and tags.
- `Filter` has no `tags`; use `tags_filter=TagsFilter(tags=[...])` for binary data.
- `viam dataset export --dataset-id ... --destination D` writes `D/<dataset-id>.zip` (not loose parquet files),
  containing `sequences.parquet` (`sequence_id, tags, start_at, end_at`), `tabular_data.parquet`
  (`sequence_id, timestamp, part_id, component_name, method_name, payload` as a JSON string) and
  `binary_data.parquet` (`sequence_id, timestamp, part_id, component_name, method_name, path, classifications,
  bounding_boxes`). The CLI also creates `D/binary_data/`.
- **BLOCKER-LEVEL FINDING: the export's `binary_data.parquet` had 0 rows and `binary_data/` was empty**, in three
  exports (the 22 s skew sequence, exported twice a few minutes apart; and a fresh 6 s 5 Hz sequence exported ~2 min
  after capture). `tabular_data.parquet` was complete (666 rows = 3 streams x 222). The images exist in the cloud
  (`binary_data_by_filter` returns 222 per camera with `method_name: GetImages`, `mime_type: image/jpeg`, and the
  sequence's `resources` list both cameras), and `viam data export binary filter` can download them. So sequence
  export drops the camera images here; the converter cannot work from this export until that is solved.

### Policy service (Step 6): loads on MPS; the suggested hub checkpoint doesn't
- `lerobot/act_aloha_sim_transfer_cube_human` on `mps`: state `failed`, `ProcessorMigrationError` ("requires
  migration to processor format ... `policy_preprocessor.json` not found"). It is a legacy-format checkpoint with the
  module's lerobot, so the cause is the checkpoint, not the device (`cpu` would fail the same way, not run).
- `lerobot/smolvla_base` on `mps`: `status` reached `ready`; `specs`: device `mps`, dtype bfloat16, policy_type
  `smolvla`, `image_feature_keys` `observation.images.camera1..3` (3 x 256 x 256), `observation.state` [6],
  `action_dim` 6, `n_action_steps` 50. So the service runs on MPS and needs a processor-format checkpoint (a
  locally trained ACT run saves those).
- Service removed afterwards; the `viam-labs:vla` 0.3.0 module is still in the config.

### Gaps / next
- Probe config on the machine: `sim-world` + `block`, `arm` (ur5e at z 400), `gripper` (2f85, z 155.8),
  `scene-cam`/`wrist-cam` 448 x 448, `demo-capture`, `data-manager`.

## Task 1: gripper reading and export fix

- mujoco-sim branch `diff-ik-policy-sim` (from 0.20.0), two commits: `{"command": "position"}` on the gripper
  (`{"position": 0-100, 100 = open, "input": 0|1}`; no `{}` alias) and `captured_at` on camera `get_images`
  (it returned an empty `ResponseMetadata`, so images looked like 1970 and fell outside the sequence window).
  Tests: gripper + camera 41 passed.
- The machine now runs the local module `/Users/nick.hehr/src/viam-mujoco-sim/run.sh` (a `local` module entry must not
  carry `module_id`); the gripper's `docommand_input` is `{"command": "position"}`.
- Export check (11 s capture at 10 Hz, tag `probe:export-fix`, exported ~1 min later): `binary_data.parquet` has
  **111 rows each for `scene-cam` and `wrist-cam`**, 222 image files under `binary_data/`; `tabular_data.parquet` has
  222 `arm` rows (JointPositions + EndPosition) and **111 `gripper` rows** with `docommand_output.position`
  (99.67 open, `input` 0.0). The fix works. The probe dataset was deleted with the CLI.
- Start/stop commands on `demo-capture` are `{"start_capture": true, "frequency_hz": N, "tags": [...]}` and `{"stop_capture": true}`.

## Task 2: cell, wood texture, renders

- `dev/make_wood_texture.py` first wrote `dev/assets/wood.png`, a regular procedural oak that read as stripes. After
  the user's review it was replaced by `dev/assets/wood-v2.png` (see "Visual fix" below).
- `dev/bc-pick-mujoco.json` has three machine-local absolute paths (the mujoco-sim `run.sh`, `bin/diff-ik`, and the
  table texture). Running the cell elsewhere means rewriting them.
- `dev/bc-pick-mujoco.json` is the cell: ur5e + 2F-85 on a pedestal, wood-textured table, cardboard bin, red 40 mm block
  at world (550, 50, 320), `scene-cam`/`wrist-cam` 448 x 448, `overview` 1280 x 720, capture-control 0.1.1,
  `viam-labs:vla` pinned 0.3.0, local mujoco-sim (patched build).
- **Network difference:** the repo file keeps `localhost:8089` (local-only run). The machine's cloud config keeps
  `localhost:8087` because the other laptop viam-server holds 8080; same config otherwise.
- **home_joints adjusted:** the plan's `[0, -1.5708, ...]` put the flange at (-492, -134, 488) in the arm frame (behind the
  arm, over the pedestal side). Joint 1 = 3.1416 rotates it to the table side. Readout after the change (arm frame):
  x 492.0, y 134.0, z 488.0, tool down (o = (0, 0, -1)). Gripper `position` after `Open`: 99.35.
- **Cameras:** no change needed. `scene-cam` frames the whole table top, the block and the bin. The block is visible in
  `wrist-cam` from home, so no servo move was needed.
- Renders (not committed), in `.runs/policy-sim/renders/`: `before-{scene-cam,wrist-cam,overview}.jpg` (no lighting
  block, no floor/table/bin textures) and `after-{scene-cam,wrist-cam,overview}.jpg` (tuned). The tuned config is
  restored on the machine.

### Visual fix (Task 2 follow-up)

- Wood: `dev/make_wood_texture.py` now writes `dev/assets/wood-v2.png` (multi-octave value noise, stretched along the grain,
  irregular ring spacing, five knots, light (170,135,95) / dark (105,75,48)). Renamed from `wood.png` so the sim does not
  serve a cached texture.
- Light: key light direction `[0.6, 0.15, -0.8]`, color `[0.82, 0.76, 0.68]` (was `[-0.4, 0.3, -1]`, `[0.95, 0.88, 0.78]`),
  so the arm casts a visible shadow on the table. Fill, rim, headlight, sky, shadow size and antialias unchanged.
- Renders: `.runs/policy-sim/renders/v2-{overview,scene-cam,wrist-cam}.jpg` (not committed).

### Check-in 2
The user reviewed the 5-episode grid (all successes) and switched the instruction to "Pick up the red block and place it in
the bin", so capture restarted as `diff-ik-pick-v2` (v1's 5 episodes are unused).

## Task 4: scripted expert and the expert bar

- `clients/python/expert_pick.py run` (no capture). First episode (seed 1000) kept with no constant changes; the plan's
  grasp height, speeds and timings worked as written. One code fix: the viam SDK's motion/gripper/etc. client modules must
  be imported before `RobotClient.at_address`, or `MotionClient.from_robot` raises `ResourceNotFoundError` (imports now at
  module top).
- `bc_pick.py` fixes (tests first): `minjerk_segment`'s last sample keeps theta continuous (`a[3] + dth`, no 360 deg jump
  across +-180); `yaw_from_wxyz` wraps to [-180, 180). `uv run clients/python/test_bc_pick.py` prints `ok`.
- **Expert bar: 20/20 kept (success 1.0) over seeds 1000-1019**; no failures, so no failure reasons by phase.
  Log: `.runs/policy-sim/expert-bar.jsonl` (untracked). `overview` after the run shows the block in the bin.
- Episode duration: about 12 s per episode wall-clock (seed 1000 alone: 14.5 s including connect and start; 20 episodes
  in about 4 min). Not logged per episode.

## Task 7: dataset captured, converted, checked

### Dataset card
- **Training:** dataset `diff-ik-pick-v2`, id `6b01b0a19d30069f0fae49f7`, **50 episodes**, seeds 1000-1049.
  **Eval:** dataset `diff-ik-pick-v2-eval`, id `6b01b0a2614c32cdd75e39a1`, **10 episodes**, seeds 9000-9009.
  (`diff-ik-pick-v1`, id `6b01a9dfdb1efb7ab4fd7ea0`, 5 episodes with the older instruction, is unused.)
- **Capture:** `expert_pick.py run --capture diff-ik-pick-v2` over seeds 1000-1064 (65 episodes) and
  `--capture diff-ik-pick-v2-eval` over 9000-9014 (15). **All 80 were kept (0 discarded)**, so no discard reasons;
  expert success 80/80 (the batch summary for 1005-1064 reads 60/60). Seeds 1050-1064 and 9010-9014 were captured as
  spare and not added to the datasets. Logs: `.runs/policy-sim/expert-diff-ik-pick-v2{,-eval}.jsonl` (untracked).
  The 50 / 10 subsets were added by trimming the logs and passing the trimmed copy to `add`:
  `head -50 .runs/policy-sim/expert-diff-ik-pick-v2.jsonl > v2-50.jsonl` and
  `head -10 .runs/policy-sim/expert-diff-ik-pick-v2-eval.jsonl > v2e-10.jsonl` (verified identical to the copies used),
  then `expert_pick.py add --log v2-50.jsonl ...` (and `--log v2e-10.jsonl` for eval), so the datasets hold exactly 50 / 10.
- **Instruction:** "Pick up the red block and place it in the bin" (`cmd:` tag). Tags per episode:
  `episode:<dataset>-<seed>`, `expert:v1`.
- **Randomization** (`bc_pick.sample_scene(seed)`): block centre world x 425-675 mm, y -60-140 mm; block yaw +-45 deg;
  start nudge from home +-30 mm per axis and +-10 deg yaw; expert speed scale 0.8-1.2. Bin fixed at world (550, -200).
- **Capture rate:** 10 Hz, `demo-capture` with `scene-cam`, `wrist-cam`, `arm` (JointPositions, EndPosition) and `gripper`
  DoCommand `position`; Task 0 measured no skew at 10 Hz (p95 1 ms). Episodes take about 12 s wall-clock each.
- **Export:** `viam dataset export` (images now present after the Task 1 fix): v2 wrote 9170 binary blobs (4585
  frames x 2 cameras), eval 1860.
- **Conversion** (from `~/src/viam-sequence-to-lerobot`, branch `diff-ik-policy-sim`):
  ```
  uv run viam-seq-to-lerobot .runs/policy-sim/export-v2 --camera scene-cam --camera wrist-cam --arm arm \
      --grippers gripper --action-space ee --image-size 224 --fps 10 --repo-id local/diff-ik-pick-v2 \
      --output-root .runs/policy-sim/lerobot-v2     # eval: export-v2-eval -> lerobot-v2-eval, repo-id ...-eval
  ```
  Result: **v2 50 episodes / 4535 frames, eval 10 episodes / 920 frames, 0 sequences skipped**, one task. 50 of the 4585
  exported frames per dataset are dropped (one per episode: the last frame has no next pose for an action).
  Outputs (untracked): `.runs/policy-sim/lerobot-v2`, `lerobot-v2-eval`.
- **Episode length:** v2 78-110 frames (median 90, 7.8-11.0 s); eval 79-104 (median 92).

### Sanity checks (`dev/check_policy_dataset.py`; state and action are 10-d: x, y, z, r00..r12, gripper)
Run: `cd ~/src/viam-sequence-to-lerobot && uv run python ~/src/diff-ik/dev/check_policy_dataset.py <dataset root>`.
- Counts and lengths: as above.
- NaNs in state or action: **0** in both datasets.
- Gripper: range 43.7-99.7. The closed value is ~44, not 0: the 40 mm block holds the 2F-85 open. v2: 61.3% of frames
  >= 90 (open), 36.1% in 40-50 (holding), **97.6% within 10 of 44 or of 100** (bimodal). Eval: 60.9% / 36.7% / 97.8%.
  The plan's "within 10 of 0 or 100" does not hold literally.
- Position ranges (arm frame, mm, flange): v2 x 430.5-675.5, y -199.7-163.1, z 84.2-517.7 (eval x 434.9-673.8,
  y -199.7-154.7, z 84.3-515.6). First frame with gripper < 60 (the grasp), v2: x 430.5-674.8, y -53.3-135.8,
  z 84.2-86.0 (eval x 434.9-673.1, y -9.9-130.0, z 84.3-86.1); the block area, flange z = 328 - 400 + 155.8 = 83.8 plus a
  few mm of lag. Last frame of every episode (release pose, both datasets): (550.5, -199.5, 306.0), above the bin
  (bin at world (550, -200)).
- `action[t][:3]` vs `state[t+1][:3]`: max difference 0 mm in both datasets. This holds by construction (the converter's
  action is the next state), so it checks the layout, not data quality.
- Images: `observation.images.scene_cam` and `observation.images.wrist_cam` frames are 3 x 224 x 224 (CHW) in both
  datasets; task string "Pick up the red block and place it in the bin".
- Real-episode skew re-check (Task 0 caveat) not done; frames were consumed at the camera clock with 0 dropped sequences.
- Note: torchcodec fails to load in the converter venv; `LeRobotDataset` falls back to pyav, which decoded fine.

### Follow-ups from review (commit 168f02b)
- `expert_pick.py add` is now idempotent (skips sequence ids already in the dataset; reports `added` and `already`); re-run
  on `diff-ik-pick-v1` gave `added 0, already 5`. `run` summary counts the episodes actually run.

### Notes for sub-project 2
- Image feature keys are `observation.images.scene_cam` / `observation.images.wrist_cam` (the converter derives them from
  the camera names); the spec's `scene` / `wrist` means these.
- The gripper action is ~100 open and ~44 holding, never near 0. A runner must treat values below ~70 as "close"; a 30/70
  hysteresis with "close below 30" would never close.
- action == next state is by construction (a layout check), not evidence of data quality.
- **Decide before training:** r02 and r12 are near-constant (std ~0.002 / 0.001 in `meta/stats.json`; the tool is always down),
  so mean/std normalization makes small tilts look 5-10 sigma out of distribution. Options: clamp the std in normalization, add
  slight orientation randomization to the expert, or check servo tilt.
- **Decide before training:** the gripper action is the next tick's measured jaw (ramps, ~44 when holding). The latched command
  `input` (0/1) is captured in the same DoCommand output, so `--gripper-key input` would give a clean binary action.
- The first chunk from home starts 200-345 mm from the state; plan a warm-up or blending for the first chunk.
- 50 episodes is thin for the randomization. 20 spare captured seeds (1050-1064, 9010-9014) exist; adding them changes v2's contents.
- A direct `infer` caller must resize images to the policy's input size (done in `policy_run.py` now).
- `dev/bc-pick-mujoco.json`'s `vla-policy` points at a gitignored local checkpoint, so a fresh clone shows that service failing.

### Smoke test (Task 9): local ACT -> `viam-labs:vla:policy` -> `servo_trajectory`
- **Training env:** `lerobot[training]` at 02655292 needs **Python 3.12** (the plan's 3.11 fails to resolve); venv
  built with `uv venv --python 3.12`. `lerobot-train` runs. The v2 dataset written by the converter's env **loaded
  as-is** in this env (no re-conversion needed).
- **Training:** ACT, chunk 20, n_action_steps 20, batch 8, 300 steps, MPS, ~45 s of training (50 s wall including
  startup). Loss step 25 **23.33** -> step 300 **2.99** (l1 0.78 -> 0.53). Checkpoint:
  `.runs/policy-sim/act-smoke/checkpoints/000300/pretrained_model` (config.json, model.safetensors,
  policy_{pre,post}processor.json and their normalizer safetensors, train_config.json), untracked.
- **Service:** `vla-policy` (`viam-labs:vla:policy`, vla 0.3.0, device mps). The first load failed with
  `DecodingError: The fields dtype, optimizer_fused are not valid for ACTConfig` (the module's bundled LeRobot is
  older than the training commit). Fix: a copy `checkpoints/000300-compat` whose `config.json` drops those two null
  fields; it then reached `ready`. `specs`: image keys `observation.images.scene_cam`, `observation.images.wrist_cam`
  (3x224x224), state `[10]`, action_dim 10, `n_action_steps` 20, no RTC. (Sub-project 2: pin training and module
  LeRobot versions together, or strip those fields after training.)
- **`policy_run.py --once`** (world reset first; via the cloud address with the API key): rows 20, row_width 10,
  `infer_s` 0.64, max step 7.4 mm, first pose (555.6, -71.9, 218.5), last (555.4, -78.7, 222.2), all inside the
  workspace; gripper column ~97 (open). Arm moved from (492.0, 134.0, 488.0) to (537.0, -15.6, 300.7), i.e. toward
  the chunk (still travelling when the session was stopped: the chunk is 2 s, the start is ~400 mm from it).
  `accepted` (since renamed `trajectory_reply_state`) "idle", status "running", `peak_accel_ratio` 1.0, no fault. A second run, watched in `overview`, showed
  the arm descending smoothly from home toward the block. Task success not expected.

### Review fixes and re-run (commit c99c1d6)
- `policy_run.py`: `accepted` renamed `trajectory_reply_state` (it is the session state when the call landed; a rejected
  chunk raises); the summary now also prints `lead_ms`, `remaining_ms`, `trajectory_ended`, the `servo_stop` reply state
  (`stop_state`), `row0_to_state_mm` and `row_last_to_end_mm`. The image-key assert is an explicit `SystemExit` that also
  checks `specs` `state_dim` and `action_dim` are 10; an unknown image mime type raises. `.runs/` is now in `.gitignore`.
- Why the cloud address: the insecure local dial (`--insecure` to `localhost:8087`) failed, so every run used
  `diff-ik-policy-sim-main.sr89pn2pma.viam.cloud` with the machine API key from the environment.
- Raw `--once` output (world reset first):
```json
{"rows": 20, "row_width": 10, "infer_s": 0.22598170908167958, "first": {"x": 555.5869750976562, "y": -71.89149475097656, "z": 218.45579528808594, "o_x": 0.00045088854810002433, "o_y": 0.00033252964844739216, "o_z": -0.9999998430617627, "theta": 27.549262871274024}, "last": {"x": 555.3858642578125, "y": -78.68677520751953, "z": 222.17494201660156, "o_x": 4.877292441941769e-05, "o_y": 0.0005731078954688373, "o_z": -0.9999998345842573, "theta": 24.1402116088017}, "max_step_mm": 7.4168779699635925, "gripper_col": [96.9, 97.0, 96.7, 96.5, 96.6], "start_xyz": [491.99782177812176, 134.0032471009216, 488.00036731899365], "end_xyz": [536.9462407740183, -15.058042091240765, 301.05189956584456], "row0_to_state_mm": 345.09491658597386, "row_last_to_end_mm": 103.00587276671077, "trajectory_reply_state": "idle", "lead_ms": 0.0, "remaining_ms": 0.0, "status_state": "running", "trajectory_ended": true, "stop_state": "stopped", "peak_accel_ratio": 1.0000000000000009}
```
  The first pose is **345 mm** from the observed state; the last pose is 103 mm from where the arm stopped (the arm was still
  travelling when the session ended). `lead_ms` and `remaining_ms` read 0.0 in the reply, and that is stale, not "the chunk was
  already due": `servo_trajectory`'s reply is the session status from before the loop's next tick samples the new chunk
  (session.go `trajectory` returns `s.status()`). Poll `servo_status` after a tick for real values. **diff-ik module fix for
  later:** the reply should report the new chunk's lead and remaining.

### Image resize fix and re-run
- The policy was trained on 224 x 224 images (the converter bicubic-resized the 448 x 448 JPEG frames), and the
  `viam-labs:vla:policy` service does not resize (only the vla controller does). `policy_run.py` sent the raw 448 px frames.
  It now reads `specs["input_features"][key]` (`[3, 224, 224]`; the numbers arrive as floats, so they are cast to int),
  decodes each frame with PIL, resizes with `Image.Resampling.BICUBIC`, re-encodes as JPEG (quality 95) and sends the
  resized height/width. `pillow` is in the script's PEP 723 dependencies.
- Raw `--once` output after the fix (world reset first):
```json
{"rows": 20, "row_width": 10, "infer_s": 0.06981862522661686, "first": {"x": 526.2438354492188, "y": 127.3753890991211, "z": 289.6628112792969, "o_x": 0.00027421354335713135, "o_y": -8.497370972979329e-06, "o_z": -0.9999999623673631, "theta": -85.72103851388643}, "last": {"x": 529.8375244140625, "y": 117.0950698852539, "z": 284.0704650878906, "o_x": 0.00010193396357379542, "o_y": -1.5096805078150586e-06, "o_z": -0.999999994803594, "theta": -84.21211245948105}, "max_step_mm": 7.144071406032352, "gripper_col": [105.7, 105.6, 105.5, 106.1, 105.5], "start_xyz": [491.99782185253866, 134.0032471039886, 488.00036671669], "end_xyz": [528.2601404377351, 118.5457964178278, 293.26250736846293], "row0_to_state_mm": 201.38148833961296, "row_last_to_end_mm": 9.438558627720509, "trajectory_reply_state": "idle", "lead_ms": 0.0, "remaining_ms": 0.0, "status_state": "running", "trajectory_ended": true, "stop_state": "stopped", "peak_accel_ratio": 1.0000000000000437}
```
  `row0_to_state_mm` fell from **345 to 201**, matching the offline home-frame check (199 mm), and the arm ended 9.4 mm from
  the chunk's last pose (was 103 mm). So the 448 px input was part of the earlier miss; the remaining gap is the undertrained
  300-step policy. The gripper column reads ~105 (an undertrained policy extrapolating past 100); harmless for `--once`,
  which does not command the gripper. `lead_ms`/`remaining_ms` 0.0 is the stale-reply issue above.

### Offline frame check (training frames through `infer`)
Three frames of `local/diff-ik-pick-v2` (JPEG-encoded images, state, task) sent to `vla-policy` `infer`; distances in mm
between the chunk's first row and the frame's state / dataset action (xyz):

| frame | row0 | state | \|row0 - state\| | \|row0 - action\| |
|---|---|---|---|---|
| 0 (start, home) | (553.3, 108.8, 305.4) | (483.2, 132.1, 490.1) | 198.9 | 198.6 |
| 40 (near grasp) | (620.0, 53.4, 86.8) | (619.7, 74.6, 86.9) | 21.2 | 21.3 |
| 1500 (mid-episode) | (529.5, -136.8, 216.8) | (517.1, 16.3, 184.3) | 157.0 | 152.0 |

Conclusion: the layout and frame are right. Predictions point to the right workspace region and the frame near the grasp
lands within 21 mm; the large misses are at home and mid-carry, where a 300-step, loss ~3 ACT policy is undertrained and
predicts a chunk that leads the state (a 20-row chunk at 10 Hz covers 2 s, so some lead is expected). No sign of a
layout or normalization mismatch (that would miss by hundreds of mm on training frames too). Three frames only. Script:
scratchpad `frame_check.py` (not committed).

## Task 10: cell summary, versions, tests, limits

### The cell
- `dev/bc-pick-mujoco.json`: ur5e + 2F-85, wood table, bin, red 40 mm block, `scene-cam` / `wrist-cam` 448 x 448; the
  before/after renders are in `.runs/policy-sim/renders/` (`before-*`, `after-*`, `v2-*`); `home_joints` changed (joint 1 =
  3.1416) as described in Task 2; cameras unchanged.

### Versions
- mujoco-sim: branch `diff-ik-policy-sim` from 0.20.0, commits `d395004` (gripper `position`) and `0b31747` (camera
  `captured_at`).
- Converter (`viam-sequence-to-lerobot`): branch `diff-ik-policy-sim` from `bimanual`, commits `6eae2ce` (`--action-space ee`)
  and `03f5a1c` (review follow-ups).
- `viam:capture-control` 0.1.1; `viam-labs:vla` 0.3.0; viam-server v1.17.0.
- Training LeRobot `02655292cdadd7dd2f8fc5daa31a7437b44354ee` (Python >= 3.12) vs the vla 0.3.0 module's bundled LeRobot
  (`ff7cc3de`, 0.6.2; see `~/src/viam-vla-inference-service/pyproject.toml`). Checkpoints from the former need `dtype` and
  `optimizer_fused` stripped from `config.json` to load in the latter.

### Tests (all green)
- `test_bc_pick.py`: ok; `test_policy_layout.py`: ok.
- Converter suite: 111 passed, 1 xfailed.
- mujoco-sim `tests/test_gripper.py` + `tests/test_camera.py` (`--tb=line`): 41 passed.
- diff-ik `go build ./...`: ok (no Go changes).

### Done when (spec)
1. Cell runs on the patched mujoco-sim; tuned visuals approved by the user for v2 after one fix pass (natural wood grain, key light angled for table shadows; Task 2).
2. Expert 20/20 over seeds 1000-1019 (>= 95%).
3. `diff-ik-pick-v2` 50 episodes, `diff-ik-pick-v2-eval` 10; card above.
4. `ee` mode tested; datasets load, 0 NaNs, gripper bimodal, ranges match the randomization.
5. Smoke test passes: trained, served, `--once` drove `servo_trajectory`; no fault.
6. This doc.

### Limits
- One scene (fixed bin, one block, one lighting), 50 expert episodes, sim only.
- The smoke policy (300 steps) is not expected to succeed; the offline check shows it is undertrained, not mis-laid-out.
- Gripper "closed" is ~44 (the block holds the jaws open), never near 0.
- Real-episode skew re-check not done; captured-at stamps are the data manager's `time_requested`.
- Cell config has machine-local absolute paths; the local insecure dial failed, so tooling goes through the cloud.
- The module's LeRobot and the training LeRobot must be pinned together (or the fields stripped).
- Sub-project 2 (closed-loop runner, training at scale) not started; the machine is left running.
