# Learned Policy in Sim, Sub-project 1 (Data) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or
> superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** a tested dataset of scripted UR5e pick-to-bin demonstrations, captured through the Viam pipeline. It's
converted to a LeRobot dataset in an absolute end-effector layout, and smoke-tested by a tiny policy driving the arm
through diff-ik's `servo_trajectory`.

**Architecture:**
- **Expert:** a Python script streams `servo_target` to a diff-ik pose session, using the block's true sim pose to
  pick it and drop it in a bin.
- **Capture:** a `viam:capture-control` sensor records each episode (two cameras, end-effector pose, gripper) as a
  Viam sequence, and kept sequences go into a sequence dataset.
- **Convert:** a new `--action-space ee` in `viam-sequence-to-lerobot` turns the export into a LeRobot v3 dataset.
- **Smoke test:** a 300-step local ACT run, served by `viam-labs:vla:policy`, drives one `servo_trajectory`.

**Tech stack:**
- Python 3.11 scripts run with `uv run` (PEP 723 headers); `viam-sdk`; numpy/scipy/Pillow;
- LeRobot (the commit pinned by `~/src/viam-lerobot-training`, `02655292c`);
- mujoco-sim 0.20.0; diff-ik (Go, this repo: `bin/diff-ik`, `servo_trajectory` in `trajectory.go`);
- Viam data API and CLI.

**Spec (source of truth; read it first):** `docs/superpowers/specs/2026-11-21-diff-ik-policy-sim-data-design.md`.

---

## Conventions (every task)

- **Repos and branches:**
  - diff-ik: `/Users/nick.hehr/src/diff-ik`, branch `diff-ik-policy-sim` (already checked out).
  - mujoco-sim: `/Users/nick.hehr/src/viam-mujoco-sim`, a new branch `diff-ik-policy-sim` from tag `0.20.0`.
  - Converter: `/Users/nick.hehr/src/viam-sequence-to-lerobot`, a new branch `diff-ik-policy-sim` from its
    **`bimanual`** branch (887e671, "Ask for gripper positions as a percentage"; checked out there now). `main`
    lacks the gripper support (`arm_components`, `grippers`, `gripper_value`, `EpisodeFrames.segments`) that `ee`
    builds on.

  Never push, never open PRs, never switch the user's other branches.
- **Uncommitted work in diff-ik:** the working tree holds uncommitted work from earlier sessions (`guard.go`,
  `force.go`, `trajectory.go`, `clients/python/*.py`, `dev/*.json`, `docs/results/*`, and edits to `loop.go`,
  `session.go`, `module.go`, `armio.go`, `task.go`, `README.md`, `devrel_diff-ik_servo.md`). **Never stage, commit,
  revert or reformat those.** Always `git add <exact new files>`, never `git add -A` or `git add .`. `bin/diff-ik` must
  be built from this working tree (`go build -o bin/diff-ik ./cmd/module`), because `servo_trajectory` exists only
  there.
- **Commit messages** end with:
  ```
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_012FAJrBytc88Rezs28MmrJ2
  ```
- **Shell:** zsh. It doesn't word-split `$VAR`: use functions, or `$=VAR`, in loops. Use `uv run <script.py>` for
  scripts with PEP 723 headers.
- **Viam:**
  - Org DevRelDemos is `16518049-9dd3-479f-9644-c5d112aa42d8`; location `test` is `sr89pn2pma`.
  - The user approved creating the machine `diff-ik-policy-sim` and its API key.
  - Credentials live only under `/private/tmp/claude-501/-Users-nick-hehr-src-diff-ik/e10380c1-756d-4aba-a5c0-b82042e5e9c3/scratchpad/diff-ik-policy-sim/`.
    Never print them, never commit them.
  - The Viam MCP tools (`create_machine`, `read_machine_config`, `set_machine_config`, `read_machine_api`,
    `run_docommand`, `read_machine_logs`) and the `viam` CLI (logged in) are available.
  - The `mujoco-sim-cell` skill's `cell.py` is at
    `/Users/nick.hehr/.claude/plugins/cache/viam-mujoco-sim/mujoco-sim/0.1.6/skills/mujoco-sim-cell/scripts/cell.py`
    (`creds`, `wait`).
- **Units:** poses in mm. The expert works in the arm's base frame (`arm_origin`); the arm stands at world
  (0, 0, 400) with no rotation, so arm = world − (0, 0, 400). Gripper opening 0–100, 100 = open. Joints for
  `home_joints` in radians.
- **Flange, not TCP** (spec): every servo pose and every dataset pose is the **flange**. The 2F-85's tool point is
  155.8 mm along the tool axis; with the tool pointing down, flange z = TCP z + 155.8.
- **Image feature names:** the converter names features after the cameras (`scene-cam` becomes
  `observation.images.scene_cam`, `wrist-cam` becomes `observation.images.wrist_cam`). The spec's
  `observation.images.scene`/`wrist` means these.
- **Python test files** follow `clients/python/test_peg.py`: a PEP 723 header, `sys.path.insert` of the script's
  folder, plain `assert`s, and `print("ok")`.

## File structure

| File | Responsibility |
|---|---|
| `viam-mujoco-sim/src/mujoco_module/models/gripper.py` (modify) | `{"command": "position"}` → `{"position", "input"}` |
| `viam-mujoco-sim/tests/test_gripper.py` (modify) | Test for it |
| `dev/make_wood_texture.py` (new) | Writes `dev/assets/wood.png` (procedural) |
| `dev/assets/wood.png` (new, generated) | Table texture |
| `dev/bc-pick-mujoco.json` (new) | The cell; mirrors the machine's cloud config |
| `clients/python/bc_pick.py` (new) | Shared cell constants and pure geometry (scene sampling, grasp yaw, min-jerk, in-bin, frames) |
| `clients/python/test_bc_pick.py` (new) | Tests for `bc_pick.py` |
| `clients/python/expert_pick.py` (new) | Runs expert episodes, capture start/stop, the episode log, `add` to a dataset |
| `viam-sequence-to-lerobot/src/viam_sequence_to_lerobot/{convert,cli}.py` (modify) | `--action-space ee` |
| `viam-sequence-to-lerobot/tests/{conftest,test_convert}.py` (modify) | `ee` fixture and tests |
| `viam-sequence-to-lerobot/README.md` (modify) | Document `ee` |
| `clients/python/policy_layout.py` (new) | `state_vector`, `chunk_to_poses` (the `ee` layout ↔ servo poses). The spec calls these "the runner helpers in `policy_run.py`"; they live in their own module so `policy_run.py` stays I/O. |
| `clients/python/test_policy_layout.py` (new) | Round trip, including against the converter's `pose.py` |
| `clients/python/policy_run.py` (new) | `--once` smoke: observe → `infer` → one `servo_trajectory` |
| `docs/results/2026-11-policy-sim-data.md` (new) | Results: Task 0 findings, renders, expert bar, capture check, dataset card, smoke test |

---

### Task 0: Machine and pipeline checks (no new code in the repos)

**Goal:** settle the spec's open points on a live machine before building. **Output:** a "Task 0" section in
`docs/results/2026-11-policy-sim-data.md` answering each question below. **Stop and report** if any blocker isn't
resolvable within the stated fallbacks.

- [ ] **Step 1: Create the machine and credentials.** Use MCP `create_machine` (location `sr89pn2pma`, name
  `diff-ik-policy-sim`), then:
  ```bash
  S=/private/tmp/claude-501/-Users-nick-hehr-src-diff-ik/e10380c1-756d-4aba-a5c0-b82042e5e9c3/scratchpad/diff-ik-policy-sim
  uv run /Users/nick.hehr/.claude/plugins/cache/viam-mujoco-sim/mujoco-sim/0.1.6/skills/mujoco-sim-cell/scripts/cell.py creds --machine-id <machine_id> --dir $S
  ```
  Expected: JSON with `viam_json`, `cell_json`, `address`, `part_id`, `key_name`, `key_id`. Record the machine id,
  part id, key name and key id (not the key) in the results doc. Start `viam-server -config $S/viam.json` in the
  background, logging to `$S/viam-server.log`.
- [ ] **Step 2: Minimal probe config.** With `set_machine_config` (revision from `read_machine_config`), write:
  - **modules:** `viam-labs:mujoco-sim` registry `0.20.0`, and `viam:capture-control` registry `latest`. Record the
    resolved version, then pin it;
  - a `world` with a `block` prop;
  - a `ur5e` arm at world (0, 0, 400);
  - a `robotiq_2f85` gripper (parent the arm, translation z 155.8), with data capture configured on the component:
    `"service_configs": [{"type": "data_manager", "attributes": {"capture_methods": [{"method": "DoCommand",
    "capture_frequency_hz": 0, "additional_params": {"docommand_input": {"command": "measure"}}}]}}]` (`measure`
    exists in 0.20.0, so this tests the parameter path before the patch);
  - two cameras at 448 × 448 (`scene-cam` fixed, `wrist-cam` parent the arm, translation y 60, z 30);
  - a capture-control sensor `demo-capture` with resources `scene-cam`/`GetImages`, `wrist-cam`/`GetImages`,
    `arm`/`JointPositions`, `arm`/`EndPosition`, `gripper`/`DoCommand`;
  - `data-manager` (`rdk:service:data_manager`, `rdk:builtin:builtin`) with
    `{"sync_interval_mins": 0.1, "capture_control_sensor": {"name": "demo-capture", "key": "overrides"}}`.

  Run `cell.py wait --cell $S/cell.json`. Expected: `{"ready": true, ...}`.
- [ ] **Step 3: Can capture-control record the gripper's `DoCommand` with its input?**
  1. Send `demo-capture` `{"start_capture": true, "frequency_hz": 10, "tags": ["probe:gripper"]}`, wait 5 s, then
     `{"stop_capture": true}` (MCP `run_docommand` with `resource_name: demo-capture`).
  2. Wait 60 s for sync, then `viam data export` (or the data API) for the part's `gripper` `DoCommand` readings in
     that window.
  3. Record whether the rows exist and hold `docommand_output.tcp_mm`.

  If they don't, repeat without `additional_params`, and record whether the module receives an empty `{}`. Decision
  for Task 1: which input the gripper must answer, `{"command": "position"}` and/or `{}`.
- [ ] **Step 4: Capture skew.** Capture 30 s at 10 Hz with the arm still, tagged `probe:skew`. Export the sequence
  (find its id by tag; see Step 5), and compute, per `scene-cam` frame, the largest timestamp gap to the nearest
  reading of every other stream. Record the p95, and the fraction of frames with any gap over 50 ms. If that fraction
  is over 2%, repeat at 5 Hz and record it.
- [ ] **Step 5: Sequences and datasets from a script.** Write a scratch script (in the scratchpad, not the repo) that:
  - makes a `ViamClient` with the machine's API key (from `$S/cell.json`: `key`, `key_id`);
  - calls `data_client.list_sequences(organization_id=...)` and pages through it;
  - filters to this part and the tag `probe:skew`;
  - creates a dataset `diff-ik-probe` with `type=DatasetType.DATASET_TYPE_SEQUENCE_DATA` (from
    `viam.proto.app.dataset`);
  - calls `add_sequences_to_dataset`.

  Record whether the machine-scoped key is allowed. **If it isn't, stop and ask the user**: don't create an org-level
  key on your own. Then run `viam dataset export --dataset-id <id> --destination <scratch dir>`, and record that the
  export has `sequences.parquet`, `tabular_data.parquet`, `binary_data.parquet` and `binary_data/`. Delete the probe
  dataset afterwards.
- [ ] **Step 6: The policy service on the Mac.** Add the `viam-labs:vla` registry module (latest; record the version)
  and a service `vla-policy` (`rdk:service:generic`, model `viam-labs:vla:policy`) with
  `{"model_hub_id": "lerobot/act_aloha_sim_transfer_cube_human", "device": "mps"}`. Poll `{"command": "status"}`
  until `ready` (or `failed`). Call `{"command": "specs"}` and record the image keys and state shape. Then remove the
  service. If it fails on MPS, try `"device": "cpu"` and record both outcomes.
- [ ] **Step 7: Record and commit.** Write the Task 0 section: machine ids, module versions, the gripper decision,
  the skew (and chosen capture rate), whether the SDK can add sequences, export layout, and policy service status.
  ```bash
  git add docs/results/2026-11-policy-sim-data.md
  git commit -m "Policy sim data Task 0: machine and pipeline checks"
  ```
  **Check-in:** the controller reports Task 0's findings to the user before Task 2's renders.

---

### Task 1: mujoco-sim gripper `position` command

**Files:** modify `/Users/nick.hehr/src/viam-mujoco-sim/src/mujoco_module/models/gripper.py` (`do_command`, about
line 201); test in `/Users/nick.hehr/src/viam-mujoco-sim/tests/test_gripper.py`.

- [ ] **Step 1: Branch.** `cd ~/src/viam-mujoco-sim && git checkout -b diff-ik-policy-sim 0.20.0`
- [ ] **Step 2: The failing test.** Append to `tests/test_gripper.py`. Component models need the **`running_sim`**
  fixture (a background-driven `SimManager` installed as the instance the models get), as
  `test_suction_model_reports_vacuum_and_keeps_it_on_stop` (about line 402) uses. The plain `sim` fixture with
  `sim.advance()` hangs here, because the models call `SimManager.get()` from worker threads. So the test waits on
  `is_moving()` instead of advancing time:
  ```python
  def test_position_command_reports_opening_percent(running_sim):
      import asyncio
      import time

      from viam.proto.app.robot import ComponentConfig
      from viam.utils import dict_to_struct

      from mujoco_module.models.arm import MujocoArm
      from mujoco_module.models.gripper import MujocoGripper
      from mujoco_module.models.world import MujocoWorld

      def config(name, attrs):
          return ComponentConfig(name=name, attributes=dict_to_struct(attrs))

      def settle(gripper, timeout=5.0):
          time.sleep(0.05)  # let the command reach the sim thread
          end = time.monotonic() + timeout
          while asyncio.run(gripper.is_moving()):
              assert time.monotonic() < end, "gripper still moving"
              time.sleep(0.02)

      MujocoWorld.new(config("sim-world", {"real_time": False}), {})
      MujocoArm.new(config("arm", {"world": "sim-world", "asset": "lite6"}), {})
      gripper = MujocoGripper.new(config("g", {"world": "sim-world", "arm": "arm"}), {})

      asyncio.run(gripper.open())
      settle(gripper)
      opened = asyncio.run(gripper.do_command({"command": "position"}))
      assert opened["position"] == pytest.approx(100.0, abs=1.0)
      assert opened["input"] == 0.0

      asyncio.run(gripper.grab())
      settle(gripper)
      closed = asyncio.run(gripper.do_command({"command": "position"}))
      assert closed["position"] == pytest.approx(0.0, abs=1.0)  # closed on nothing
      assert closed["input"] == 1.0
  ```
  If the closed-on-nothing check fails only on its 1% margin, loosen it to a few percent. Copy the model
  construction exactly from the suction test, if it differs: that test writes its own arm MJCF, so
  check that `{"asset": "lite6"}` builds there too, or reuse that test's arm setup with the Lite6 gripper.
- [ ] **Step 3: Run it to see it fail.** `uv run pytest tests/test_gripper.py -k position_command -q`. Expected:
  FAIL with `ValueError: unknown command` (not a hang; if it hangs, the fixture is wrong).
- [ ] **Step 4: Implement.** In `do_command`, before the `raise`:
  ```python
          if command.get("command") == "position" or (not command and self._answer_empty):
              return await asyncio.to_thread(self._position)
  ```
  and add the method to the class:
  ```python
      def _position(self) -> dict:
          """The jaw's opening as a percentage of its travel (100 = open), and the latched command."""
          handle = self._h()
          if handle.suction:
              return {"position": 0.0 if self._closed else 100.0, "input": 1.0 if self._closed else 0.0}
          span = handle.closed_q - handle.open_q
          pct = 100.0 * (handle.closed_q - handle.jaw()) / span if span else 100.0
          return {"position": float(min(100.0, max(0.0, pct))), "input": 1.0 if self._closed else 0.0}
  ```
  `self._answer_empty`: if Task 0 found that capture sends `{}`, set it to `True` in the constructor, with a comment
  ("data capture without a docommand_input sends {}"). Otherwise delete that clause, so the line is just
  `if command.get("command") == "position":`. Add the command to the gripper's DoCommand table in `README.md`:
  `{"command": "position"}` → `{"position": 0–100, 100 = open; "input": 0 or 1}`.
- [ ] **Step 5: Run the gripper tests.** `uv run pytest tests/test_gripper.py -q`. Expected: all pass.
- [ ] **Step 6: Build the local module.** `make run.sh` (or the repo's documented local-run step; check its
  `Makefile`). Record the path of `run.sh` for the cell.
- [ ] **Step 7: Commit (in mujoco-sim).**
  ```bash
  git add src/mujoco_module/models/gripper.py tests/test_gripper.py README.md
  git commit -m "feat: gripper position command (opening percent and latch) for capture"
  ```

---

### Task 2: The cell, the wood texture, and the renders

**Files:** create `dev/make_wood_texture.py`, `dev/assets/wood.png` (generated) and `dev/bc-pick-mujoco.json`.

- [ ] **Step 1: The texture generator.** Create `dev/make_wood_texture.py`:
  ```python
  # /// script
  # requires-python = ">=3.11"
  # dependencies = ["numpy", "pillow"]
  # ///
  """Write dev/assets/wood.png: a procedural light-oak grain, so the cell needs no downloaded texture.
  Run: uv run dev/make_wood_texture.py"""
  import os

  import numpy as np
  from PIL import Image

  rng = np.random.default_rng(7)
  n = 1024
  y, x = np.mgrid[0:n, 0:n] / n
  warp = 0.04 * np.sin(2 * np.pi * (3 * x + rng.random())) + 0.02 * np.sin(2 * np.pi * (11 * x + rng.random()))
  rings = np.sin(2 * np.pi * (38 * (y + warp)))
  grain = 0.5 + 0.35 * rings + 0.15 * rng.standard_normal((n, n)).cumsum(axis=1) / np.sqrt(np.arange(1, n + 1))
  grain = (grain - grain.min()) / (grain.max() - grain.min())
  light, dark = np.array([214, 178, 128]), np.array([150, 105, 62])
  img = (dark + (light - dark) * grain[..., None]).astype(np.uint8)
  os.makedirs(os.path.join(os.path.dirname(__file__), "assets"), exist_ok=True)
  Image.fromarray(img).save(os.path.join(os.path.dirname(__file__), "assets", "wood.png"))
  print("wrote dev/assets/wood.png")
  ```
  Run `uv run dev/make_wood_texture.py`. Expected: `wrote dev/assets/wood.png`. Look at it (Read the PNG): it should
  read as wood grain.
- [ ] **Step 2: The cell config.** Create `dev/bc-pick-mujoco.json`:
  - Use the mujoco-sim module from Task 1's local `run.sh`, or registry `0.20.0` if the patch wasn't needed.
  - Pin capture-control to Task 0's version.
  - If Task 0 chose the component-level `DoCommand` capture, set the gripper's `service_configs`
    `docommand_input` to `{"command": "position"}`. If Task 0 chose the empty-input route, drop `additional_params`.
  - The JSON's `network.bind_address` is `localhost:8089`, for a local-only run. The cloud config omits it.
  ```json
  {
   "modules": [
    {"type": "local", "name": "mujoco-sim", "executable_path": "/Users/nick.hehr/src/viam-mujoco-sim/run.sh"},
    {"type": "local", "name": "diff-ik", "executable_path": "/Users/nick.hehr/src/diff-ik/bin/diff-ik"},
    {"type": "registry", "name": "viam_capture-control", "module_id": "viam:capture-control", "version": "<TASK0_VERSION>"}
   ],
   "components": [
    {"name": "sim-world", "api": "rdk:component:generic", "model": "viam-labs:mujoco-sim:world",
     "attributes": {
      "floor_texture": "textures/concrete.png", "floor_texture_size": 2,
      "lighting": {
       "lights": [
        {"type": "directional", "direction": [-0.4, 0.3, -1], "color": [0.95, 0.88, 0.78], "shadows": true},
        {"type": "directional", "direction": [0.5, -0.4, -0.6], "color": [0.25, 0.28, 0.32], "shadows": false},
        {"type": "spot", "position": [-0.3, 0.9, 1.4], "direction": [0.6, -0.6, -0.8], "color": [0.5, 0.5, 0.55],
         "cutoff_deg": 35, "shadows": false}
       ],
       "headlight": 0.3, "skybox": [[0.62, 0.70, 0.80], [0.18, 0.18, 0.20]],
       "shadow_size": 8192, "antialias": 8, "reflections": true
      },
      "props": [{"name": "block", "type": "cube", "size": 0.04, "mass": 0.05, "color": [0.85, 0.08, 0.08],
                 "position": [0.55, 0.05, 0.32], "static_friction": 1.0}]
     }},
    {"name": "pedestal", "api": "rdk:component:generic", "model": "viam-labs:mujoco-sim:obstacle",
     "frame": {"parent": "world", "translation": {"x": 0, "y": 0, "z": 200}, "geometry": {"type": "box", "x": 200, "y": 200, "z": 400}},
     "attributes": {"world": "sim-world", "color": [0.35, 0.35, 0.38]}},
    {"name": "table", "api": "rdk:component:generic", "model": "viam-labs:mujoco-sim:obstacle",
     "frame": {"parent": "world", "translation": {"x": 550, "y": 0, "z": 150}, "geometry": {"type": "box", "x": 400, "y": 600, "z": 300}},
     "attributes": {"world": "sim-world", "color": [0.72, 0.58, 0.40], "texture": "/Users/nick.hehr/src/diff-ik/dev/assets/wood.png"}},
    {"name": "bin", "api": "rdk:component:generic", "model": "viam-labs:mujoco-sim:obstacle",
     "frame": {"parent": "world", "translation": {"x": 550, "y": -200, "z": 300}},
     "attributes": {"world": "sim-world", "mesh": "meshes/bin.stl", "color": [0.62, 0.48, 0.32], "texture": "textures/cardboard.png"}},
    {"name": "arm", "api": "rdk:component:arm", "model": "viam-labs:mujoco-sim:arm",
     "frame": {"parent": "world", "translation": {"x": 0, "y": 0, "z": 400}},
     "attributes": {"world": "sim-world", "asset": "ur5e", "max_joint_deg_s": 180, "max_joint_deg_s2": 360,
                    "home_joints": [0, -1.5708, 1.5708, -1.5708, -1.5708, 0]}},
    {"name": "gripper", "api": "rdk:component:gripper", "model": "viam-labs:mujoco-sim:gripper",
     "frame": {"parent": "arm", "translation": {"x": 0, "y": 0, "z": 155.8}},
     "attributes": {"world": "sim-world", "arm": "arm", "asset": "robotiq_2f85"},
     "service_configs": [{"type": "data_manager", "attributes": {"capture_methods": [
       {"method": "DoCommand", "capture_frequency_hz": 0, "additional_params": {"docommand_input": {"command": "position"}}}]}}]},
    {"name": "scene-cam", "api": "rdk:component:camera", "model": "viam-labs:mujoco-sim:camera",
     "frame": {"parent": "world", "translation": {"x": 1150, "y": -450, "z": 850}},
     "attributes": {"world": "sim-world", "target": [0.55, -0.03, 0.32], "fov_deg": 55, "width": 448, "height": 448}},
    {"name": "wrist-cam", "api": "rdk:component:camera", "model": "viam-labs:mujoco-sim:camera",
     "frame": {"parent": "arm", "translation": {"x": 0, "y": 70, "z": 40}},
     "attributes": {"world": "sim-world", "fov_deg": 75, "near_m": 0.02, "width": 448, "height": 448}},
    {"name": "overview", "api": "rdk:component:camera", "model": "viam-labs:mujoco-sim:camera",
     "frame": {"parent": "world", "translation": {"x": 1500, "y": -1100, "z": 1100}},
     "attributes": {"world": "sim-world", "target": [0.45, -0.05, 0.35], "width": 1280, "height": 720}},
    {"name": "demo-capture", "api": "rdk:component:sensor", "model": "viam:capture-control:capture-control-sensor",
     "attributes": {"resources": [
       {"resource_name": "scene-cam", "method": "GetImages"},
       {"resource_name": "wrist-cam", "method": "GetImages"},
       {"resource_name": "arm", "method": "JointPositions"},
       {"resource_name": "arm", "method": "EndPosition"},
       {"resource_name": "gripper", "method": "DoCommand"}]}}
   ],
   "services": [
    {"name": "servo", "api": "rdk:service:motion", "model": "devrel:diff-ik:servo", "attributes": {"arms": ["arm"]}},
    {"name": "world-state", "api": "rdk:service:world_state_store", "model": "viam-labs:mujoco-sim:world_state", "attributes": {"world": "sim-world"}},
    {"name": "data-manager", "api": "rdk:service:data_manager", "model": "rdk:builtin:builtin",
     "attributes": {"sync_interval_mins": 0.1, "capture_control_sensor": {"name": "demo-capture", "key": "overrides"}}}
   ],
   "network": {"bind_address": "localhost:8089"}
  }
  ```
- [ ] **Step 3: Deploy to the machine.** Rebuild `bin/diff-ik` (`go build -o bin/diff-ik ./cmd/module`). Write the
  config (without `network`) with `set_machine_config`, restart the machine's viam-server, run `cell.py wait`, and
  reset the world (`run_docommand` on `sim-world`, `{"command": "reset"}`). Then check:
  - `arm` `GetEndPosition` (MCP `read_machine_api`) shows the tool down (`o_z` ≈ −1) above the table: x ≈ 400–520,
    y ≈ 0–140 in the arm frame. If `home_joints` puts it elsewhere, adjust them and record the change;
  - the gripper's `position` command returns about 100 after `open`.
- [ ] **Step 4: Before/after renders.** Save `GetImages` from `scene-cam`, `wrist-cam` (with the block in view; send
  a `servo_target` over the block first if needed) and `overview` with the tuned config. Then write a copy with the
  world's `lighting`, `floor_texture` and the table and bin `texture` removed, and save the same three. Restore the
  tuned config. Check `scene-cam` frames the whole block area (x 425–675, y −60–140 world mm) and the bin. If it
  doesn't, adjust `scene-cam` (position, `target`, `fov_deg`) and record the change. Put both sets of images in
  `.runs/policy-sim/renders/` (not committed) and reference them in the results doc.
- [ ] **Step 5: Commit.**
  ```bash
  git add dev/make_wood_texture.py dev/assets/wood.png dev/bc-pick-mujoco.json docs/results/2026-11-policy-sim-data.md
  git commit -m "Policy sim data Task 2: pick cell, wood texture, tuned render"
  ```
  **Check-in:** the controller shows the user the before/after renders, and waits for approval of the visuals.

---

### Task 3: Cell geometry helpers (`bc_pick.py`), test first

**Files:** create `clients/python/bc_pick.py` and `clients/python/test_bc_pick.py`.

- [ ] **Step 1: Write the failing test,** `clients/python/test_bc_pick.py`:
  ```python
  # /// script
  # requires-python = ">=3.11"
  # ///
  """Pure-path checks for bc_pick.py. Run: uv run clients/python/test_bc_pick.py"""
  import math
  import os
  import sys

  sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
  from bc_pick import (  # noqa: E402
      BIN_XY, BLOCK_AREA, TCP_MM, angle_diff, flange_from_tcp, grasp_theta, in_bin, minjerk_segment, pose_map,
      sample_scene, world_to_arm, yaw_from_wxyz,
  )

  # scenes: deterministic, in range
  a, b = sample_scene(1003), sample_scene(1003)
  assert a == b and sample_scene(1004) != a
  for s in range(200):
      sc = sample_scene(s)
      (x0, x1), (y0, y1) = BLOCK_AREA
      assert x0 <= sc["block_xy"][0] <= x1 and y0 <= sc["block_xy"][1] <= y1
      assert -45 <= sc["block_yaw_deg"] <= 45 and 0.8 <= sc["speed"] <= 1.2
      assert all(abs(v) <= 30 for v in sc["nudge_xyz"]) and abs(sc["nudge_yaw_deg"]) <= 10

  # angles
  assert angle_diff(350, 10) == -20 and angle_diff(10, 350) == 20
  assert abs(yaw_from_wxyz([math.cos(math.radians(15)), 0, 0, math.sin(math.radians(15))]) - 30) < 1e-9
  # grasp yaw never turns more than 45 degrees from the current theta, and stays a cube-face alignment
  for cur in (180.0, 170.0, -170.0, 95.0):
      for yaw in range(-45, 46, 5):
          t = grasp_theta(yaw, cur)
          assert abs(angle_diff(t, cur)) <= 45 + 1e-9, (yaw, cur, t)
          assert abs(((180 - yaw - t) % 90 + 45) % 90 - 45) < 1e-9  # t = 180 - yaw (mod 90)

  # frames
  assert world_to_arm((550, 0, 320)) == (550, 0, -80)
  assert flange_from_tcp((550, 0, 328)) == (550, 0, 328 + TCP_MM)

  # min-jerk: exact ends, monotonic progress, count
  seg = minjerk_segment((0, 0, 0, 180), (100, 0, 0, 200), 1.0, 30)
  assert seg[0] == (0, 0, 0, 180) and seg[-1] == (100, 0, 0, 200) and len(seg) == 31
  assert all(p[0] <= q[0] for p, q in zip(seg, seg[1:]))
  wrap = minjerk_segment((0, 0, 0, 170), (0, 0, 0, -170), 1.0, 10)  # crosses 180: short way, +20 degrees
  assert abs(angle_diff(wrap[5][3], 180)) < 1e-6

  # in the bin: inside the inner footprint and below the rim
  bx, by = BIN_XY
  assert in_bin((bx, by, 320), BIN_XY) and in_bin((bx + 60, by - 60, 330), BIN_XY)
  assert not in_bin((bx + 80, by, 320), BIN_XY) and not in_bin((bx, by, 400), BIN_XY)

  assert pose_map((1, 2, 3, 45)) == {"x": 1, "y": 2, "z": 3, "o_x": 0, "o_y": 0, "o_z": -1, "theta": 45}
  print("ok")
  ```
- [ ] **Step 2: Run it to see it fail.** `uv run clients/python/test_bc_pick.py`. Expected: `ModuleNotFoundError: No
  module named 'bc_pick'`.
- [ ] **Step 3: Implement** `clients/python/bc_pick.py`:
  ```python
  """The pick-to-bin cell's constants and pure geometry, shared by expert_pick.py and policy_run.py.
  World frame in mm; the arm stands at world (0, 0, 400) with no rotation (dev/bc-pick-mujoco.json)."""
  import math
  import random

  ARM_BASE = (0.0, 0.0, 400.0)  # arm_origin in world, mm
  TABLE_TOP = 300.0
  BLOCK_MM = 40.0
  BIN_XY = (550.0, -200.0)  # the bin's frame: the centre of its underside, on the table
  BIN_INNER_HALF = 70.0  # 150 mm outside, 5 mm walls
  BIN_RIM_Z = TABLE_TOP + 80.0
  TCP_MM = 155.8  # 2F-85 tool point along the tool axis
  BLOCK_AREA = ((425.0, 675.0), (-60.0, 140.0))  # world x and y ranges for the block's centre
  GRASP_ABOVE_CENTER = 8.0  # TCP this far above the block's centre: closed fingertips clear the table
  PREGRASP_ABOVE = 100.0
  LIFT = 100.0
  PLACE_ABOVE_RIM = 120.0
  RETREAT = 50.0


  def sample_scene(seed: int) -> dict:
      """The episode's randomization (spec "Randomization"); the same seed gives the same scene."""
      r = random.Random(seed)
      (x0, x1), (y0, y1) = BLOCK_AREA
      return {"block_xy": (r.uniform(x0, x1), r.uniform(y0, y1)), "block_yaw_deg": r.uniform(-45, 45),
              "nudge_xyz": tuple(r.uniform(-30, 30) for _ in range(3)), "nudge_yaw_deg": r.uniform(-10, 10),
              "speed": r.uniform(0.8, 1.2)}


  def angle_diff(a: float, b: float) -> float:
      """a − b wrapped to [−180, 180)."""
      return (a - b + 180.0) % 360.0 - 180.0


  def yaw_from_wxyz(q) -> float:
      """Yaw (degrees about world z) of a quaternion [w, x, y, z] that rotates about z only."""
      return math.degrees(2 * math.atan2(q[3], q[0]))


  def grasp_theta(block_yaw_deg: float, current_theta_deg: float) -> float:
      """The tool's orientation-vector theta (tool pointing down) that lines the jaws up with a block face, choosing
      the face nearest the current theta. For o = (0, 0, −1), the tool's x axis points at 180° − theta in the world,
      and a cube repeats every 90°, so the candidates are 180 − yaw + 90k."""
      base = 180.0 - block_yaw_deg
      k = round(angle_diff(current_theta_deg, base) / 90.0)
      return current_theta_deg - angle_diff(current_theta_deg, base + 90.0 * k)


  def world_to_arm(p) -> tuple:
      return tuple(v - b for v, b in zip(p, ARM_BASE))


  def flange_from_tcp(tcp) -> tuple:
      """Tool pointing down: the flange is TCP_MM above the tool point."""
      return (tcp[0], tcp[1], tcp[2] + TCP_MM)


  def minjerk_segment(a, b, duration_s: float, hz: float) -> list:
      """Poses (x, y, z, theta) from a to b along a min-jerk profile, both ends included; theta turns the short way."""
      n = max(1, round(duration_s * hz))
      dth = angle_diff(b[3], a[3])
      out = []
      for i in range(n + 1):
          s = i / n
          m = s * s * s * (10 - 15 * s + 6 * s * s)
          out.append((a[0] + (b[0] - a[0]) * m, a[1] + (b[1] - a[1]) * m, a[2] + (b[2] - a[2]) * m, a[3] + dth * m))
      out[-1] = tuple(b)
      return out


  def in_bin(block_world_mm, bin_xy=BIN_XY) -> bool:
      """The block's centre is inside the bin's inner footprint and below its rim."""
      x, y, z = block_world_mm
      return abs(x - bin_xy[0]) <= BIN_INNER_HALF and abs(y - bin_xy[1]) <= BIN_INNER_HALF and z < BIN_RIM_Z


  def pose_map(p) -> dict:
      """A servo pose map for (x, y, z, theta) with the tool pointing down."""
      return {"x": p[0], "y": p[1], "z": p[2], "o_x": 0, "o_y": 0, "o_z": -1, "theta": p[3]}


  async def connect(address: str, insecure: bool, n: int = 1) -> list:
      """n robot connections (so streams don't queue behind each other): insecure for a local viam-server, else the
      API key in VIAM_API_KEY / VIAM_API_KEY_ID."""
      import os

      from viam.robot.client import RobotClient
      from viam.rpc.dial import DialOptions

      dial = (DialOptions(insecure=True, disable_webrtc=True) if insecure
              else DialOptions.with_api_key(os.environ["VIAM_API_KEY"], os.environ["VIAM_API_KEY_ID"]))
      return [await RobotClient.at_address(address, RobotClient.Options(dial_options=dial)) for _ in range(n)]
  ```
  `connect` is the one I/O helper here (its imports are inside it, so the tests still run without `viam-sdk`). The
  new clients use it instead of the untracked `touch_sweep.py`, so their commits stand alone.
- [ ] **Step 4: Run it to see it pass.** `uv run clients/python/test_bc_pick.py`. Expected: `ok`. If the
  `grasp_theta` assertions fail, fix the function, not the test: the two properties (never more than 45° of turn, and
  a cube-face alignment) are the contract.
- [ ] **Step 5: Commit.**
  ```bash
  git add clients/python/bc_pick.py clients/python/test_bc_pick.py
  git commit -m "Policy sim data Task 3: cell geometry helpers"
  ```

---

### Task 4: The expert episode (no capture), and the expert bar

**Files:** create `clients/python/expert_pick.py`. Its pure logic lives in `bc_pick.py`; this file is I/O.

- [ ] **Step 1: Write `clients/python/expert_pick.py`.** It connects with `bc_pick.connect` (Task 3), not
  `touch_sweep.connect`: `touch_sweep.py` is untracked, and this commit must stand alone. The `run`
  subcommand does what follows; `add` comes in Task 5.
  ```python
  # /// script
  # requires-python = ">=3.11"
  # dependencies = ["viam-sdk"]
  # ///
  """Scripted pick-to-bin expert for dev/bc-pick-mujoco.json (spec docs/superpowers/specs/2026-11-21-...-data-design.md).

    uv run clients/python/expert_pick.py run --address <addr> --seeds 1000-1019            # expert bar, no capture
    uv run clients/python/expert_pick.py run --address <addr> --seeds 1000-1049 --capture diff-ik-pick-v1
    uv run clients/python/expert_pick.py add --log .runs/policy-sim/expert-diff-ik-pick-v1.jsonl --dataset diff-ik-pick-v1
  """
  import argparse
  import asyncio
  import json
  import math
  import os
  import sys
  import time
  from datetime import datetime, timezone

  sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
  import bc_pick as C  # noqa: E402

  INSTRUCTION = "put the red block in the bin"
  TICK_HZ = 30
  BASE_MM_S = 150.0  # expert speed at scale 1


  class Episode:
      def __init__(self, robot_conns, a):
          from viam.components.arm import Arm
          from viam.components.generic import Generic
          from viam.components.gripper import Gripper
          from viam.components.sensor import Sensor
          from viam.services.motion import MotionClient

          r0, r1 = robot_conns
          self.a = a
          self.svc = MotionClient.from_robot(r0, a.service)
          self.world = Generic.from_robot(r1, a.world)
          self.gripper = Gripper.from_robot(r1, a.gripper)
          self.arm = Arm.from_robot(r1, a.arm)
          self.capture = Sensor.from_robot(r1, a.capture_sensor)
          self.sid = None

      async def target(self, p):
          await self.svc.do_command({"command": "servo_target", "session_id": self.sid, "pose": C.pose_map(p)})

      async def move(self, cur, goal, speed, check_grasp=False):
          """Stream a min-jerk segment from cur to goal (flange, arm frame, (x, y, z, theta)); returns goal."""
          dist = math.dist(cur[:3], goal[:3])
          dur = max(0.6, dist / (BASE_MM_S * speed), abs(C.angle_diff(goal[3], cur[3])) / 60.0)
          for i, p in enumerate(C.minjerk_segment(cur, goal, dur, TICK_HZ)):
              t = time.monotonic()
              await self.target(p)
              if check_grasp and i % 6 == 0:
                  hs = await self.gripper.is_holding_something()
                  if not hs.meta.get("firm_grasp"):
                      raise Discard("lost grasp")
              await asyncio.sleep(max(0.0, 1 / TICK_HZ - (time.monotonic() - t)))
          return goal

      async def hold(self, p, seconds):
          end = time.monotonic() + seconds
          while time.monotonic() < end:
              await self.target(p)
              await asyncio.sleep(0.1)

      async def block_world_mm(self):
          r = await self.world.do_command({"command": "get_prop_pose", "prop": "block"})
          pos, q = r["pose"]["position"], r["pose"]["orientation_wxyz"]
          return tuple(v * 1000 for v in pos), q

      async def run(self, seed, tags=None) -> dict:
          sc = C.sample_scene(seed)
          bx, by = sc["block_xy"]
          yaw = math.radians(sc["block_yaw_deg"])
          await self.world.do_command({"command": "reset"})
          await self.world.do_command({"command": "set_prop_pose", "prop": "block",
                                       "position": [bx / 1000, by / 1000, (C.TABLE_TOP + C.BLOCK_MM / 2) / 1000],
                                       "orientation_wxyz": [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]})
          await self.gripper.open()
          r = await self.svc.do_command({"command": "servo_start", "component_name": self.a.arm, "frame": "arm_origin",
                                         "mode": "pose", "gain": 8, "timeout_ms": 3000,
                                         "limits": {"linear_mm_s": 300, "angular_deg_s": 90}})
          self.sid = r["session_id"]
          p0 = r["pose"]
          cur = (p0["x"], p0["y"], p0["z"], p0["theta"])
          info = {"seed": seed, "scene": sc, "kept": False, "reason": ""}
          capturing = False
          try:
              if abs(p0["o_x"]) > 0.02 or abs(p0["o_y"]) > 0.02 or p0["o_z"] > -0.99:
                  # theta is only comparable across poses when the orientation vector is (0, 0, -1)
                  raise RuntimeError(f"start pose isn't tool-down: {p0}; fix home_joints (Task 2 Step 3)")
              n = sc["nudge_xyz"]
              start = (cur[0] + n[0], cur[1] + n[1], cur[2] + n[2], cur[3] + sc["nudge_yaw_deg"])
              cur = await self.move(cur, start, sc["speed"])
              await self.hold(cur, 0.5)
              if tags is not None:
                  await self.capture.do_command({"start_capture": True, "frequency_hz": self.a.rate_hz, "tags": tags})
                  capturing, info["t_start"] = True, datetime.now(timezone.utc).isoformat()
              (wx, wy, wz), q = await self.block_world_mm()
              theta = C.grasp_theta(C.yaw_from_wxyz(q), cur[3])
              grasp_tcp = C.world_to_arm((wx, wy, wz + C.GRASP_ABOVE_CENTER))
              pre = C.flange_from_tcp((grasp_tcp[0], grasp_tcp[1], grasp_tcp[2] + C.PREGRASP_ABOVE)) + (theta,)
              grasp = C.flange_from_tcp(grasp_tcp) + (theta,)
              cur = await self.move(cur, pre, sc["speed"])
              cur = await self.move(cur, grasp, sc["speed"] * 0.6)
              await self.hold(cur, 0.2)
              grab = asyncio.create_task(self.gripper.grab())
              await self.hold(cur, 0.6)
              await grab
              deadline = time.monotonic() + 2.0
              while not (await self.gripper.is_holding_something()).meta.get("firm_grasp"):
                  if time.monotonic() > deadline:
                      raise Discard("grasp timeout")
                  await self.hold(cur, 0.1)
              lift = (cur[0], cur[1], cur[2] + C.LIFT, cur[3])
              cur = await self.move(cur, lift, sc["speed"] * 0.8, check_grasp=True)
              bin_tcp = C.world_to_arm((C.BIN_XY[0], C.BIN_XY[1], C.BIN_RIM_Z + C.PLACE_ABOVE_RIM))
              over_bin = C.flange_from_tcp(bin_tcp) + (cur[3],)
              cur = await self.move(cur, over_bin, sc["speed"], check_grasp=True)
              await self.gripper.open()
              await self.hold(cur, 0.6)
              cur = await self.move(cur, (cur[0], cur[1], cur[2] + C.RETREAT, cur[3]), sc["speed"])
              await self.hold(cur, 0.4)
              b1, _ = await self.block_world_mm()
              await self.hold(cur, 0.3)
              b2, _ = await self.block_world_mm()
              if not (C.in_bin(b2) and math.dist(b1, b2) < 1.0):
                  raise Discard(f"block not in bin at rest: {tuple(round(v) for v in b2)}")
              info["kept"] = True
          except Discard as e:
              info["reason"] = str(e)
          finally:
              if capturing:
                  await self.capture.do_command({"stop_capture": True})
                  info["t_stop"] = datetime.now(timezone.utc).isoformat()
              try:
                  await self.svc.do_command({"command": "servo_stop", "session_id": self.sid})
              except Exception:
                  pass
          return info


  class Discard(Exception):
      pass


  def seed_range(s: str) -> list:
      lo, _, hi = s.partition("-")
      return list(range(int(lo), int(hi or lo) + 1))


  async def cmd_run(a):
      conns = await C.connect(a.address, a.insecure, 2)
      ep = Episode(conns, a)
      os.makedirs(".runs/policy-sim", exist_ok=True)
      log_path = f".runs/policy-sim/expert-{a.capture or 'bar'}.jsonl"
      kept = 0
      try:
          with open(log_path, "a") as log:
              for seed in seed_range(a.seeds):
                  tags = None
                  if a.capture:
                      tags = [f"cmd:{INSTRUCTION}", f"episode:{a.capture}-{seed}", "expert:v1"]
                  info = await ep.run(seed, tags)
                  if a.capture:
                      info["tags"] = tags
                  kept += info["kept"]
                  log.write(json.dumps(info) + "\n")
                  log.flush()
                  print(json.dumps({"seed": seed, "kept": info["kept"], "reason": info["reason"]}))
      finally:
          for c in conns:
              await c.close()
      n = len(seed_range(a.seeds))
      print(json.dumps({"episodes": n, "kept": kept, "success": kept / n, "log": log_path}))


  def main():
      ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
      sub = ap.add_subparsers(dest="cmd", required=True)
      run = sub.add_parser("run")
      run.add_argument("--address", required=True)
      run.add_argument("--insecure", action="store_true")
      run.add_argument("--seeds", required=True, help="e.g. 1000-1049")
      run.add_argument("--capture", default="", help="dataset name: capture each episode, tagged episode:<name>-<seed>")
      run.add_argument("--rate-hz", type=float, default=10.0)
      run.add_argument("--service", default="servo")
      run.add_argument("--arm", default="arm")
      run.add_argument("--gripper", default="gripper")
      run.add_argument("--world", default="sim-world")
      run.add_argument("--capture-sensor", default="demo-capture")
      a = ap.parse_args()
      if a.cmd == "run":
          asyncio.run(cmd_run(a))


  if __name__ == "__main__":
      main()
  ```
  Notes for the implementer:
  - `servo_start`'s `pose` is in the session frame (`arm_origin`). Its `theta` is the orientation-vector theta in
    degrees. If `o_z` isn't about −1 at `home_joints`, stop and fix `home_joints` (Task 2 Step 3).
  - Check `Gripper.is_holding_something()` in the installed SDK. If it returns a `HoldingStatus` without `.meta`,
    read `meta` the way the SDK exposes it, and keep the `firm_grasp` logic.
  - A start pose that isn't tool-down raises before any motion (its theta wouldn't match the targets'
    `o = (0, 0, -1)` convention).
- [ ] **Step 2: Smoke one episode.** Export the machine's key (from `$S/cell.json`, without printing it), then:
  ```bash
  uv run clients/python/expert_pick.py run --address <cell.json address> --seeds 1000
  ```
  Expected: one line `{"seed": 1000, "kept": true, ...}`. Watch `overview` (MCP `GetImages`) after it to confirm the
  block is in the bin. Fix grasp height, timing and approach until it works. Record any constant changes in
  `bc_pick.py`, and re-run `test_bc_pick.py`.
- [ ] **Step 3: The expert bar.** `uv run clients/python/expert_pick.py run --address <addr> --seeds 1000-1019`.
  Expected: `"success" >= 0.95`. If it's lower, read the reasons in `.runs/policy-sim/expert-bar.jsonl`, fix, and
  re-run all 20. Record the final success rate and the failure reasons by phase in the results doc.
- [ ] **Step 4: Commit.**
  ```bash
  git add clients/python/expert_pick.py clients/python/bc_pick.py clients/python/test_bc_pick.py docs/results/2026-11-policy-sim-data.md
  git commit -m "Policy sim data Task 4: scripted expert (expert bar met)"
  ```

---

### Task 5: Capture, add to a dataset, and the 5-episode review

**Files:** modify `clients/python/expert_pick.py` (add the `add` subcommand).

- [ ] **Step 1: The `add` subcommand.** Add to `expert_pick.py`:
  ```python
  async def cmd_add(a):
      """Add the log's kept episodes to a sequence dataset, creating it if needed; matched by episode tag."""
      from viam.app.viam_client import ViamClient
      from viam.proto.app.dataset import DatasetType
      from viam.rpc.dial import DialOptions

      kept = {}
      for line in open(a.log):
          row = json.loads(line)
          if row.get("kept") and row.get("tags"):
              kept[next(t for t in row["tags"] if t.startswith("episode:"))] = row["seed"]
      client = await ViamClient.create_from_dial_options(
          DialOptions.with_api_key(os.environ["VIAM_API_KEY"], os.environ["VIAM_API_KEY_ID"]))
      try:
          data = client.data_client
          found, token = {}, None
          while True:
              seqs, token = await data.list_sequences(organization_id=a.org_id, page_token=token)
              for s in seqs:
                  if s.part_id == a.part_id:
                      for t in s.sequence_tags:
                          if t in kept:
                              found[t] = s.id
              if not token:
                  break
          missing = sorted(set(kept) - set(found))
          ds = [d for d in await data.list_datasets_by_organization_id(a.org_id) if d.name == a.dataset]
          dataset_id = ds[0].id if ds else await data.create_dataset(
              name=a.dataset, organization_id=a.org_id, type=DatasetType.DATASET_TYPE_SEQUENCE_DATA)
          if found:
              await data.add_sequences_to_dataset(dataset_id=dataset_id, sequence_ids=list(found.values()))
          print(json.dumps({"dataset_id": dataset_id, "added": len(found), "missing": missing}))
      finally:
          client.close()
  ```
  and its parser: `add = sub.add_parser("add")` with `--log`, `--dataset`, `--org-id` (default
  `16518049-9dd3-479f-9644-c5d112aa42d8`) and `--part-id` (required), plus `elif a.cmd == "add":
  asyncio.run(cmd_add(a))`. If `ViamClient.close()` is a coroutine in the installed SDK, await it. If adding a
  sequence that's already in the dataset errors, filter those out first with `sequences_by_dataset_id`.
- [ ] **Step 2: Capture 5.**
  ```bash
  uv run clients/python/expert_pick.py run --address <addr> --seeds 1000-1004 --capture diff-ik-pick-v1
  ```
  Then wait 60 s for sync, and:
  ```bash
  uv run clients/python/expert_pick.py add --log .runs/policy-sim/expert-diff-ik-pick-v1.jsonl --dataset diff-ik-pick-v1 --part-id <part_id>
  ```
  Expected: `"added": <kept count>, "missing": []`. If episodes are missing, wait another 60 s and re-run `add`. It's
  idempotent by tag.
- [ ] **Step 3: Review grid.**
  1. `viam dataset export --dataset-id <id> --destination .runs/policy-sim/export-v1-5`.
  2. Write a scratch script (scratchpad) that picks, per sequence, the last `scene-cam` and last `wrist-cam` image
     from `binary_data/`, and makes one PNG grid: rows = episodes, columns = the two cameras.
  3. Save it to `.runs/policy-sim/review-5.png`.

  **Check-in:** the controller shows the grid to the user and waits. If they flag problems, fix the expert, bump the
  dataset to `diff-ik-pick-v2` (spec rule) and repeat this task.
- [ ] **Step 4: Commit.**
  ```bash
  git add clients/python/expert_pick.py
  git commit -m "Policy sim data Task 5: capture and dataset add"
  ```

---

### Task 6: Converter `--action-space ee`, test first

**Files (in `/Users/nick.hehr/src/viam-sequence-to-lerobot`):** modify
`src/viam_sequence_to_lerobot/convert.py`, `src/viam_sequence_to_lerobot/cli.py`, `tests/conftest.py`,
`tests/test_convert.py` and `README.md`. Setup: confirm `git branch --show-current` is `bimanual` at 887e671, then
`git checkout -b diff-ik-policy-sim bimanual`, then
`uv venv --python 3.11 && uv pip install -e ".[dev]"`, then `uv run pytest -q` must pass before you change anything.

- [ ] **Step 1: The fixture.** Append to `tests/conftest.py`:
  ```python
  EE_SEQ = "66666666-6666-6666-6666-666666666666"
  EE_ARM, EE_GRIPPER = "arm", "gripper"
  EE_TICKS = 12
  EE_GAP_TICK = 5  # no EndPosition reading: alignment drops this tick


  def ee_pose(tick: int) -> dict:
      return {"x": 100.0 + tick, "y": -50.0 + 2 * tick, "z": 80.0 - tick,
              "o_x": 0.0, "o_y": 0.0, "o_z": -1.0, "theta": 170.0 + 3 * tick}


  @pytest.fixture
  def ee_export(tmp_path: Path) -> Path:
      """One arm (EndPosition, tool pointing down) and one gripper (DoCommand position = 10 * tick), 12 ticks at
      10 Hz, one camera; tick 5 has no EndPosition reading."""
      export_dir = tmp_path / "ee"
      export_dir.mkdir()
      sequences = {"sequence_id": [EE_SEQ], "tags": [["cmd:put the red block in the bin"]],
                   "start_at": [BASE], "end_at": [BASE + timedelta(seconds=2)]}
      columns = ["sequence_id", "timestamp", "part_id", "component_name", "method_name"]
      tabular = {k: [] for k in [*columns, "payload"]}
      binary = {k: [] for k in [*columns, "path"]}

      def add(table, ts, component, method, key, value):
          for k, v in zip([*columns, key], [EE_SEQ, ts, "part-1", component, method, value]):
              table[k].append(v)

      for tick in range(EE_TICKS):
          ts = BASE + timedelta(seconds=0.1 * tick)
          rel = f"binary_data/{EE_SEQ}/cam/{tick:04d}.jpeg"
          _write_image(export_dir / rel, tick)
          add(binary, ts, CAMERA, "GetImages", "path", rel)
          if tick != EE_GAP_TICK:
              add(tabular, ts + timedelta(milliseconds=1), EE_ARM, "EndPosition", "payload",
                  json.dumps({"pose": ee_pose(tick)}))
          add(tabular, ts + timedelta(milliseconds=2), EE_GRIPPER, "DoCommand", "payload",
              json.dumps({"docommand_output": {"position": 10.0 * tick, "input": 0.0}}))

      pq.write_table(pa.table(sequences), export_dir / "sequences.parquet")
      pq.write_table(pa.table(tabular), export_dir / "tabular_data.parquet")
      pq.write_table(pa.table(binary), export_dir / "binary_data.parquet")
      return export_dir
  ```
- [ ] **Step 2: The failing tests.** Append to `tests/test_convert.py` (add `EE_*`, `ee_pose` and `CAMERA` to the
  conftest imports at the top, as the file imports other fixture constants; import `pose_rotation` from `.pose`):
  ```python
  def ee_config(ee_export, tmp_path, **overrides):
      defaults = dict(export_dir=ee_export, output_root=tmp_path / "ee_out", repo_id="test/ee",
                      camera_components=(CAMERA,), arm_components=(EE_ARM,), grippers=(EE_GRIPPER,),
                      action_space="ee", fps=10, min_frames=2)
      defaults.update(overrides)
      return ConversionConfig(**defaults)


  def test_build_episode_ee_layout_and_next_state_actions(ee_export, tmp_path):
      export = load_export(ee_export)
      episode = build_episode(export, export.sequences[0], ee_config(ee_export, tmp_path))
      # 12 ticks: tick 5 dropped (no pose); tick 4's next tick is 6 (a gap), so frame 4 is dropped; tick 11 is
      # only an action target.
      kept_ticks = [0, 1, 2, 3, 6, 7, 8, 9, 10]
      assert episode.states.shape == episode.actions.shape == (len(kept_ticks), 10)
      assert episode.segments == ((EE_ARM, 9), (EE_GRIPPER, 1))
      for row, tick in enumerate(kept_ticks):
          pose = ee_pose(tick)
          np.testing.assert_allclose(episode.states[row, :3], [pose["x"], pose["y"], pose["z"]], rtol=1e-6)
          rows = pose_rotation(pose).as_matrix()[:2, :].reshape(6)
          np.testing.assert_allclose(episode.states[row, 3:9], rows, atol=1e-6)
          assert episode.states[row, 9] == 10.0 * tick  # gripper passed through unscaled
          nxt = ee_pose(tick + 1)
          np.testing.assert_allclose(episode.actions[row, :3], [nxt["x"], nxt["y"], nxt["z"]], rtol=1e-6)
          assert episode.actions[row, 9] == 10.0 * (tick + 1)


  def test_config_ee_rejects_two_arms_and_action_arm(ee_export, tmp_path):
      with pytest.raises(ValueError, match="ee"):
          ee_config(ee_export, tmp_path, arm_components=("a", "b"), grippers=())
      with pytest.raises(ValueError, match="action_arm"):
          ee_config(ee_export, tmp_path, grippers=(), action_arm="leader")


  def test_build_episode_ee_without_gripper_is_nine_wide(ee_export, tmp_path):
      export = load_export(ee_export)
      episode = build_episode(export, export.sequences[0], ee_config(ee_export, tmp_path, grippers=()))
      assert episode.states.shape[1] == 9 and episode.segments == ((EE_ARM, 9),)


  def test_convert_end_to_end_ee(ee_export, tmp_path):
      from lerobot.datasets.lerobot_dataset import LeRobotDataset

      config = ee_config(ee_export, tmp_path)
      summary = convert(config)
      assert summary.episodes_written == 1 and summary.frames_written == 9
      dataset = LeRobotDataset(repo_id=config.repo_id, root=config.output_root)
      names = [*STATE_NAMES, "gripper"]
      assert dataset.meta.features["observation.state"]["names"] == names
      assert dataset.meta.features["action"]["names"] == names
      assert dataset[0]["observation.state"].shape == (10,) and dataset[0]["action"].shape == (10,)
  ```
- [ ] **Step 3: Run them to see them fail.** `uv run pytest tests/test_convert.py -k ee -q`. Expected: failures in
  `ConversionConfig` validation ("action_space must be 'joints' or 'delta-ee'").
- [ ] **Step 4: Implement in `convert.py`.**
  1. **Docstring:** add an `action_space="ee"` bullet: absolute `EndPosition` pose (9) plus, with a gripper, its
     `docommand_output.<gripper_key>` (1) → `observation.state`; the next tick's state → `action`; frames whose next
     tick was dropped are skipped, because the deployed chunk plays on a fixed step.
  2. **`__post_init__`:**
     - change the allowed set to `("joints", "delta-ee", "ee")` and its message to
       `"action_space must be 'joints', 'delta-ee' or 'ee', got ..."`;
     - after the `multi` checks add:
       ```python
               if self.action_space == "ee" and len(self.arm_components) > 1:
                   raise ValueError("ee supports a single arm, with at most one gripper")
       ```
       The existing `action_arm requires action_space='joints'` check covers `action_arm`.
  3. **`build_episode`:** set `pose_mode = config.action_space in ("delta-ee", "ee")` and use it in the `methods`
     line (`"EndPosition" if pose_mode else "JointPositions"`). Then add a branch between the `delta_ee` branch and
     the `else`:
     ```python
         elif config.action_space == "ee":
             arm = config.arm_components[0]
             try:
                 poses = np.stack([pose_state(r.payload) for r in aligned[arm]])
             except (KeyError, ValueError) as exc:
                 raise EpisodeSkip(f"unusable EndPosition reading: {exc}") from exc
             parts = [poses]
             segments.append((arm, 9))
             if config.grippers:
                 gripper = config.grippers[0]
                 parts.append(np.array([[gripper_value(r.payload, config.gripper_key, gripper)]
                                        for r in aligned[gripper]]))
                 segments.append((gripper, 1))
             rows = np.concatenate(parts, axis=1)
             # Absolute targets, but a deployed chunk plays one row per tick: a target across a dropped tick would
             # be played a tick early, so that frame is dropped (as delta-ee drops its deltas).
             frame_idx = [i for i in range(len(kept) - 1) if kept[i + 1] == kept[i] + 1]
             n_spanning = len(kept) - 1 - len(frame_idx)
             if n_spanning:
                 logger.info("Sequence %s: dropped %d/%d frames whose next tick spans a gap",
                             sequence.sequence_id, n_spanning, len(kept) - 1)
             if not frame_idx:
                 raise EpisodeSkip("no two consecutive aligned ticks to form an action")
             states = rows[frame_idx].astype(np.float32)
             actions = rows[[i + 1 for i in frame_idx]].astype(np.float32)
     ```
  4. **`convert()`, where the names are chosen** (about line 617): add before the `elif len(segments) == 1:`
     ```python
         elif config.action_space == "ee":
             state_names = action_names = [*STATE_NAMES, *(["gripper"] if config.grippers else [])]
     ```
- [ ] **Step 5: CLI.** In `cli.py`:
  - add `"ee"` to `--action-space`'s `choices`, and extend its help: "ee: state is the absolute EndPosition pose
    (9 dims) plus the gripper (1) when `--grippers` is set; action is the next frame's state, in the same layout";
  - in `--grippers`' help, say grippers also work with `ee` (one arm).
- [ ] **Step 6: Run all tests.** `uv run pytest -q`. Expected: all pass, the new `ee` tests included.
- [ ] **Step 7: README.** Under the `--action-space` list, add an `ee` bullet matching the docstring. It should say
  the state and action are 10 dims with a gripper (9 without), the gripper is unscaled 0–100, and gap-spanning frames
  are dropped. Mention it pairs with diff-ik's `servo_trajectory`, and that a client rebuilds the pose with
  `state_rotation` and `orientation_vector`.
- [ ] **Step 8: Commit (in the converter repo).**
  ```bash
  git add src/viam_sequence_to_lerobot/convert.py src/viam_sequence_to_lerobot/cli.py tests/conftest.py tests/test_convert.py README.md
  git commit -m "feat: --action-space ee (absolute end-effector pose plus gripper)"
  ```

---

### Task 7: Capture the dataset, convert, and the dataset card

- [ ] **Step 1: Training episodes.** Run `expert_pick.py run --seeds 1005-1069 --capture diff-ik-pick-v1` (more seeds
  than needed, for discards), then `add`. Re-run `run` with further seeds until the dataset holds **50 kept**. Count
  with `sequences_by_dataset_id(dataset_id, count_only=True)`.
- [ ] **Step 2: Eval episodes.** `run --seeds 9000-9014 --capture diff-ik-pick-v1-eval`, then `add` into
  `diff-ik-pick-v1-eval` until it holds **10 kept**.
- [ ] **Step 3: Export and convert.**
  ```bash
  viam dataset export --dataset-id <v1 id> --destination .runs/policy-sim/export-v1
  cd ~/src/viam-sequence-to-lerobot && uv run viam-seq-to-lerobot ~/src/diff-ik/.runs/policy-sim/export-v1 \
      --camera scene-cam --camera wrist-cam --arm arm --grippers gripper --action-space ee \
      --image-size 224 --fps <capture rate> --repo-id local/diff-ik-pick-v1 \
      --output-root ~/src/diff-ik/.runs/policy-sim/lerobot-v1
  ```
  Check the converter's `--help` for exact flag names (`--camera` repeats; the arm and output flags may be named
  differently) and adapt. Expected: 50 episodes written, and skips listed with reasons. Do the same for the eval
  export into `lerobot-v1-eval`.
- [ ] **Step 4: Sanity checks** (a scratch script, using `LeRobotDataset`). Record each in the results doc:
  - episode and frame counts, and the per-episode length range;
  - no NaNs in state or action;
  - the gripper column is bimodal: most values within 10 of 0 or 100;
  - position ranges match the block area and bin (arm frame);
  - for one episode, action[t][:3] equals state[t+1][:3] within 1e-4 mm, wherever frame t+1 exists;
  - images are 224 × 224 for both cameras.
- [ ] **Step 5: Dataset card.** In the results doc:
  - dataset names and ids; episodes kept and discarded, with reasons (from the JSONL logs);
  - frames, and the episode length range;
  - randomization ranges and seed ranges;
  - capture rate and skew (Task 0);
  - the conversion command.
- [ ] **Step 6: Commit.**
  ```bash
  git add docs/results/2026-11-policy-sim-data.md
  git commit -m "Policy sim data Task 7: dataset v1 captured and converted"
  ```

---

### Task 8: The layout helpers for the runner, test first

**Files:** create `clients/python/policy_layout.py` and `clients/python/test_policy_layout.py`.

- [ ] **Step 1: Write the failing test,** `clients/python/test_policy_layout.py`:
  ```python
  # /// script
  # requires-python = ">=3.11"
  # dependencies = ["numpy", "scipy", "viam-sdk"]
  # ///
  """Pure-path checks for policy_layout.py, including against the converter's own pose code.
  Run: uv run clients/python/test_policy_layout.py"""
  import os
  import sys

  import numpy as np

  sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
  from policy_layout import chunk_to_poses, state_vector  # noqa: E402

  pose = {"x": 512.3, "y": -40.1, "z": 95.0, "o_x": 0.02, "o_y": -0.03, "o_z": -0.999, "theta": 137.0}
  s = state_vector(pose, 63.0)
  assert len(s) == 10 and s[9] == 63.0 and s[:3] == [512.3, -40.1, 95.0]

  # round trip: state -> pose -> state
  poses, grip = chunk_to_poses([s])
  s2 = state_vector(poses[0], grip[0])
  np.testing.assert_allclose(s2, s, atol=1e-6)

  # noisy rotation rows (a policy's raw output) still give a valid pose
  noisy = list(s)
  noisy[3:9] = [v * 1.07 + 0.01 for v in noisy[3:9]]
  p, _ = chunk_to_poses([noisy])
  n = np.linalg.norm([p[0]["o_x"], p[0]["o_y"], p[0]["o_z"]])
  assert abs(n - 1) < 1e-6

  # the same layout as the converter's pose_state (the dataset's source of truth)
  conv = os.path.expanduser("~/src/viam-sequence-to-lerobot/src")
  if os.path.isdir(conv):
      sys.path.insert(0, conv)
      from viam_sequence_to_lerobot.pose import pose_state  # noqa: E402

      np.testing.assert_allclose(pose_state({"pose": pose}), s[:9], atol=1e-9)
  else:
      print("skipped the converter cross-check: ~/src/viam-sequence-to-lerobot not found")
  print("ok")
  ```
- [ ] **Step 2: Run it to see it fail.** `uv run clients/python/test_policy_layout.py`. Expected:
  `ModuleNotFoundError: No module named 'policy_layout'`. If importing the converter's `pose.py` fails on a missing
  dependency, add that dependency to the test's PEP 723 header.
- [ ] **Step 3: Implement** `clients/python/policy_layout.py`. Same math as the converter's `pose.py`
  (`pose_rotation`, `state_rotation`, `orientation_vector`), duplicated deliberately so the runner doesn't depend on
  the converter package; the test pins them together.
  ```python
  """The ee dataset layout <-> servo poses: [x, y, z (mm), r00 r01 r02 r10 r11 r12, gripper 0-100], flange in the arm's
  base frame (viam-sequence-to-lerobot --action-space ee)."""
  import numpy as np
  from scipy.spatial.transform import Rotation
  from viam.spatialmath import OrientationVector, RotationMatrix


  def _rotation(pose: dict) -> Rotation:
      q = OrientationVector(pose["o_x"], pose["o_y"], pose["o_z"], np.deg2rad(pose["theta"])).to_quaternion()
      return Rotation.from_matrix(np.asarray(q.to_rotation_matrix().elements, dtype=np.float64).reshape(3, 3))


  def state_vector(pose: dict, gripper: float) -> list:
      """A pose map (x, y, z mm; o_x, o_y, o_z, theta degrees) and the gripper opening as the 10-number state."""
      rows = _rotation(pose).as_matrix()[:2, :].reshape(6)
      return [float(pose["x"]), float(pose["y"]), float(pose["z"]), *map(float, rows), float(gripper)]


  def _rows_rotation(r) -> Rotation:
      a1, a2 = np.asarray(r[0:3], dtype=np.float64), np.asarray(r[3:6], dtype=np.float64)
      b1 = a1 / np.linalg.norm(a1)
      b2 = a2 - (b1 @ a2) * b1
      b2 /= np.linalg.norm(b2)
      return Rotation.from_matrix(np.stack([b1, b2, np.cross(b1, b2)]))


  def chunk_to_poses(rows) -> tuple:
      """Policy action rows (10 numbers each) as servo pose maps, plus the gripper column."""
      poses, grip = [], []
      for r in rows:
          m = RotationMatrix(_rows_rotation(r[3:9]).as_matrix().reshape(9).tolist())
          v = m.to_quaternion().to_orientation_vector()
          poses.append({"x": float(r[0]), "y": float(r[1]), "z": float(r[2]), "o_x": float(v.o_x),
                        "o_y": float(v.o_y), "o_z": float(v.o_z), "theta": float(np.degrees(v.theta))})
          grip.append(float(r[9]) if len(r) > 9 else float("nan"))
      return poses, grip
  ```
- [ ] **Step 4: Run it to see it pass.** `uv run clients/python/test_policy_layout.py`. Expected: `ok`, with no
  "skipped" line (the converter repo is present).
- [ ] **Step 5: Commit.**
  ```bash
  git add clients/python/policy_layout.py clients/python/test_policy_layout.py
  git commit -m "Policy sim data Task 8: ee layout helpers for the runner"
  ```

---

### Task 9: The end-to-end smoke test

- [ ] **Step 1: LeRobot environment.** In the scratchpad:
  ```bash
  uv venv --python 3.11 <scratch>/lerobot-venv
  VIRTUAL_ENV=<scratch>/lerobot-venv uv pip install "lerobot[training] @ git+https://github.com/huggingface/lerobot@02655292cdadd7dd2f8fc5daa31a7437b44354ee"
  ```
  That's the full SHA `~/src/viam-lerobot-training/requirements-lerobot.txt` pins; the training extra provides
  `lerobot-train`. Then check `<scratch>/lerobot-venv/bin/lerobot-train --help` runs. If the dataset written by the
  converter's environment won't load here, that file also pins a converter commit known to pair with this LeRobot;
  note any mismatch.
- [ ] **Step 2: Smoke dataset.** Convert only the first 5 episodes (export `.runs/policy-sim/export-v1-5` from Task 5)
  with the same converter command as Task 7, into `.runs/policy-sim/lerobot-smoke`.
- [ ] **Step 3: Train 300 steps.**
  ```bash
  <scratch>/lerobot-venv/bin/lerobot-train --dataset.repo_id=local/diff-ik-pick-smoke \
      --dataset.root=$HOME/src/diff-ik/.runs/policy-sim/lerobot-smoke --policy.type=act \
      --policy.chunk_size=20 --policy.n_action_steps=20 --steps=300 --batch_size=8 \
      --policy.device=mps --policy.push_to_hub=false --log_freq=25 --save_freq=300 \
      --output_dir=$HOME/src/diff-ik/.runs/policy-sim/act-smoke
  ```
  Flag names follow LeRobot at that commit; check `--help` and adapt. If MPS fails, use `--policy.device=cpu`.
  Expected: the loss at step 300 is below the loss at step 25. Record both, and the wall time. The checkpoint is under
  `.runs/policy-sim/act-smoke/checkpoints/*/pretrained_model`.
- [ ] **Step 4: Serve it.** On the machine add the `viam-labs:vla` module (Task 0's version) and a service
  `vla-policy` (`viam-labs:vla:policy`) with
  `{"model_path": "/Users/nick.hehr/src/diff-ik/.runs/policy-sim/act-smoke/checkpoints/<step>/pretrained_model", "device": "mps"}`.
  Poll `status` until `ready`. `specs` must list `observation.images.scene_cam`, `observation.images.wrist_cam` and a
  10-wide state.
- [ ] **Step 5: `policy_run.py --once`.** Create `clients/python/policy_run.py`:
  ```python
  # /// script
  # requires-python = ">=3.11"
  # dependencies = ["viam-sdk", "numpy", "scipy"]
  # ///
  """Policy runner for the pick-to-bin cell. Sub-project 1 has only --once: one observation, one infer, one
  servo_trajectory, to prove the layout and the command end to end.

    uv run clients/python/policy_run.py --address <addr> --once --lead-ms 200
  """
  import argparse
  import asyncio
  import base64
  import json
  import math
  import os
  import sys
  import time

  sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
  from policy_layout import chunk_to_poses, state_vector  # noqa: E402

  CAMERAS = {"observation.images.scene_cam": "scene-cam", "observation.images.wrist_cam": "wrist-cam"}


  async def once(a):
      from viam.components.arm import Arm
      from viam.components.camera import Camera
      from viam.components.gripper import Gripper
      from viam.services.generic import Generic
      from viam.services.motion import MotionClient

      from bc_pick import connect

      (robot,) = await connect(a.address, a.insecure, 1)
      try:
          arm, grip = Arm.from_robot(robot, a.arm), Gripper.from_robot(robot, a.gripper)
          policy, svc = Generic.from_robot(robot, a.policy), MotionClient.from_robot(robot, a.service)
          specs = await policy.do_command({"command": "specs"})
          keys = specs["image_feature_keys"]
          assert set(keys) == set(CAMERAS), f"policy wants {keys}, cell has {list(CAMERAS)}"
          t_obs = time.monotonic()
          images = {}
          for key, cam in CAMERAS.items():
              imgs, _ = await Camera.from_robot(robot, cam).get_images()
              img = imgs[0]
              enc = {"image/jpeg": "jpeg", "image/png": "png"}.get(getattr(img, "mime_type", "image/jpeg"), "jpeg")
              images[key] = {"encoding": enc, "data": base64.b64encode(img.data).decode(),
                             "height": img.height, "width": img.width, "channels": 3}
          p = await arm.get_end_position()
          g = (await grip.do_command({"command": "position"}))["position"]
          pose = {"x": p.x, "y": p.y, "z": p.z, "o_x": p.o_x, "o_y": p.o_y, "o_z": p.o_z, "theta": p.theta}
          state = state_vector(pose, g)
          r = await policy.do_command({"command": "infer", "images": images, "state": {"values": state},
                                       "task": "put the red block in the bin"})
          rows = r["actions"]["rows"]
          poses, gcol = chunk_to_poses(rows)
          sid = (await svc.do_command({"command": "servo_start", "component_name": a.arm, "frame": "arm_origin",
                                       "mode": "pose", "timeout_ms": 2000}))["session_id"]
          try:
              res = await svc.do_command({"command": "servo_trajectory", "session_id": sid, "poses": poses,
                                          "dt_ms": 1000 / a.rate_hz, "age_ms": (time.monotonic() - t_obs) * 1000,
                                          "lead_ms": a.lead_ms})
              await asyncio.sleep(len(poses) / a.rate_hz + 0.5)
              st = await svc.do_command({"command": "servo_status", "session_id": sid})
          finally:
              await svc.do_command({"command": "servo_stop", "session_id": sid})
          q = await arm.get_end_position()
          steps = [math.dist([a_["x"], a_["y"], a_["z"]], [b["x"], b["y"], b["z"]]) for a_, b in zip(poses, poses[1:])]
          print(json.dumps({
              "rows": len(rows), "row_width": len(rows[0]), "infer_s": r.get("server_s"),
              "first": poses[0], "last": poses[-1], "max_step_mm": max(steps) if steps else 0.0,
              "gripper_col": [round(v, 1) for v in gcol[:5]], "start_xyz": [p.x, p.y, p.z], "end_xyz": [q.x, q.y, q.z],
              "accepted": res.get("state"), "status_state": st.get("state"), "peak_accel_ratio": st.get("peak_accel_ratio")}))
      finally:
          await robot.close()


  def main():
      ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
      ap.add_argument("--address", required=True)
      ap.add_argument("--insecure", action="store_true")
      ap.add_argument("--once", action="store_true", required=True)
      ap.add_argument("--rate-hz", type=float, default=10.0, help="the capture rate: dt_ms = 1000 / rate")
      ap.add_argument("--lead-ms", type=float, default=200.0)
      ap.add_argument("--arm", default="arm")
      ap.add_argument("--gripper", default="gripper")
      ap.add_argument("--policy", default="vla-policy")
      ap.add_argument("--service", default="servo")
      asyncio.run(once(ap.parse_args()))


  if __name__ == "__main__":
      main()
  ```
  Reset the world, then run `uv run clients/python/policy_run.py --address <addr> --once`. Expected:
  - `rows` 20, `row_width` 10, `accepted` not an error;
  - `max_step_mm` under 30 (a 300-step policy is crude, but it shouldn't jump);
  - every pose inside the workspace (arm frame x 200–800, |y| < 450, z −150–400);
  - `end_xyz` moved toward the chunk's last pose;
  - no session fault.

  Watch `overview` during a second run to confirm the motion is smooth. Task success isn't expected.
- [ ] **Step 6: Commit.**
  ```bash
  git add clients/python/policy_run.py docs/results/2026-11-policy-sim-data.md
  git commit -m "Policy sim data Task 9: end-to-end smoke (local ACT -> vla:policy -> servo_trajectory)"
  ```

---

### Task 10: Results doc and finish

- [ ] **Step 1: Complete `docs/results/2026-11-policy-sim-data.md`:**
  - Task 0 findings;
  - the cell, with the before/after renders (paths under `.runs/policy-sim/renders/`) and any camera or
    `home_joints` adjustments;
  - the expert bar (rate and failure reasons);
  - the capture check;
  - the dataset card (Task 7);
  - the converter's `ee` mode (the branch and commit in `viam-sequence-to-lerobot`);
  - the mujoco-sim patch (branch and commit), if made;
  - the smoke test (training loss, wall time, `specs`, the `--once` output);
  - limits.
- [ ] **Step 2: Run every test.**
  - `for t in bc_pick policy_layout; do uv run clients/python/test_$t.py; done` → `ok` twice;
  - the converter suite: `cd ~/src/viam-sequence-to-lerobot && uv run pytest -q`;
  - if mujoco-sim was patched: `cd ~/src/viam-mujoco-sim && uv run pytest tests/test_gripper.py -q`.

  diff-ik's Go suite is unaffected (no Go changes). Run `go build ./...` as a check anyway.
- [ ] **Step 3: Commit.**
  ```bash
  git add docs/results/2026-11-policy-sim-data.md
  git commit -m "Policy sim data: results"
  ```
  **Check-in:** the controller reports to the user (done criteria from the spec, with evidence), and asks whether to
  keep the machine running, and whether to start sub-project 2's brainstorm.
