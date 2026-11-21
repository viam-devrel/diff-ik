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

- `dev/make_wood_texture.py` writes `dev/assets/wood.png` (procedural oak). The grain reads as wavy stripes at close range;
  on the table at render distance it passes as wood, but it is regular. Say if you want it more irregular.
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
