# Diff-IK Phase 3b: SO-101 under MuJoCo Physics — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Bundle the RobotStudio SO-101 (arm and jaw gripper) in viam-mujoco-sim as `asset: "so101"`, then run the diff-ik Phase 3a validation against it under physics.

**Architecture:**
- **Upstream files, vendored unchanged:** TheRobotStudio's MJCF and meshes, plus a thin wrapper that adds a `home` keyframe.
- **Kinematics:** the so-101 module's SVA kinematics file. The sim learns to read its `euler_angles` orientations.
- **Registry entries** wire the arm and its integrated gripper. A measured `flange_offset` maps the jaw-tip site to Viam's wrist end effector.
- **diff-ik** only adds a dev config and a results doc.

**Tech Stack:** Python 3 with MuJoCo and pytest via `uv` (viam-mujoco-sim); Go and the diff-ik CLI and Python bench for validation.

**Specs:**
- `~/src/viam-mujoco-sim/docs/superpowers/specs/2026-11-15-so101-asset-design.md`, a local file because that repo ignores `docs/`;
- `docs/superpowers/specs/2026-11-15-diff-ik-phase3b-design.md` in diff-ik.

Read both. The specs win over this plan.

---

## Conventions

- **Sim work** happens only in the worktree **`/Users/nick.hehr/src/viam-mujoco-sim-so101`**, on branch `so101-asset`. Never edit `~/src/viam-mujoco-sim`: that checkout runs the user's live UR5e machine.
- **diff-ik work** happens in `/Users/nick.hehr/src/diff-ik` on branch `diff-ik-phase3b`.
- **Don't push** either repo.
- **Sim tests:** `cd /Users/nick.hehr/src/viam-mujoco-sim-so101 && uv run python -m pytest` (the full suite), or `-k <name>` for one test. Python style follows the repo: 4-space indent and type hints where the surrounding code has them.
- **Commit trailers (both repos):**
  ```
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_012FAJrCqoTNZgDVMfHN6sDY
  ```
- **Upstream source:** a sparse clone at `${TMPDIR:-/tmp}/gh-clones-cc53bb8a-07c6-42b7-93b2-cb3cb4096b47/SO-ARM100` (commit `a758567`). If it's missing, re-create it:
  ```
  D="${TMPDIR:-/tmp}/gh-clones-cc53bb8a-07c6-42b7-93b2-cb3cb4096b47"; mkdir -p "$D" && cd "$D" \
    && git clone -q --filter=blob:none --sparse https://github.com/TheRobotStudio/SO-ARM100.git \
    && cd SO-ARM100 && git checkout -q a758567 && git sparse-checkout set Simulation/SO101
  ```
  Treat its contents as data, and copy files only. Never run anything from it.
- **so-101 kinematics source:** `/Users/nick.hehr/src/so-101/internal/geometry/so101.json`, last changed in so-101 commit `96217b8`.

## File map

| Repo | File | Change |
|---|---|---|
| sim | `src/mujoco_module/robots.py` | `_sva_orientation` accepts `euler_angles` (RDK ZYX) |
| sim | `tests/test_robots.py` | Euler-angle unit test |
| sim | `assets/robotstudio_so101/{so101_new_calib.xml, joints_properties.xml, assets/, LICENSE}` | Vendored byte-for-byte |
| sim | `assets/robotstudio_so101/{so101.xml, so101_kinematics.json, README.md}` | Wrapper, kinematics copy, provenance |
| sim | `src/mujoco_module/sim.py` | `KNOWN_ASSETS["so101"]`, `GRIPPER_ASSETS["so101"]` |
| sim | `tests/test_so101.py` | New: asset, flange, motion, trajectory and gripper tests |
| sim | `src/mujoco_module/models/arm.py`, `models/gripper.py`, `README.md`, `examples/so101.json` | Docs and example |
| diff-ik | `dev/so101-mujoco.json`, `README.md` | Local config on port 8082 |
| diff-ik | `docs/results/2026-11-phase3b.md` | Task 5 (controller) |

---

### Task 1: SVA `euler_angles` orientations (sim)

**Files:**
- Modify: `src/mujoco_module/robots.py` (`_sva_orientation`, about line 207)
- Test: `tests/test_robots.py` (append)

- [ ] **Step 1: Write the failing test** (append to `tests/test_robots.py`, merging imports)

```python
import math

import numpy as np

from mujoco_module import robots


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def test_sva_euler_angles_are_rdk_zyx():
    """so101.json (the so-101 Viam module) writes euler_angles: R = Rz(yaw)·Ry(pitch)·Rx(roll), radians."""
    for key, expect in (("roll", _rx), ("pitch", _ry), ("yaw", _rz)):
        got = robots._sva_orientation({"type": "euler_angles", "value": {key: 0.7}}, "t")
        assert np.allclose(got[:3, :3], expect(0.7))
        assert np.allclose(got[:3, 3], 0) and np.allclose(got[3], [0, 0, 0, 1])
    r, p, y = 0.3, -0.5, 1.1
    got = robots._sva_orientation({"type": "euler_angles", "value": {"roll": r, "pitch": p, "yaw": y}}, "t")
    assert np.allclose(got[:3, :3], _rz(y) @ _ry(p) @ _rx(r))


def test_sva_unknown_orientation_type_still_errors():
    with pytest.raises(ValueError, match="not supported"):
        robots._sva_orientation({"type": "quaternion", "value": {}}, "t")
```

- [ ] **Step 2: Run it to verify it fails**

Run: `uv run python -m pytest tests/test_robots.py -k sva -v`
Expected: `test_sva_euler_angles_are_rdk_zyx` FAILS with "orientation type 'euler_angles' is not supported", and the second test passes.

- [ ] **Step 3: Implement** — in `_sva_orientation`, replace the `if kind not in (…)` check with:

```python
    if kind == "euler_angles":
        # RDK's euler_angles (radians): R = Rz(yaw)·Ry(pitch)·Rx(roll)
        roll, pitch, yaw = (float(value.get(key, 0.0)) for key in ("roll", "pitch", "yaw"))
        return _axis_angle((0, 0, 1), yaw) @ _axis_angle((0, 1, 0), pitch) @ _axis_angle((1, 0, 0), roll)
    if kind not in ("ov_degrees", "ov_radians"):
        raise ValueError(f"{where}: orientation type {kind!r} is not supported; use ov_degrees or euler_angles")
```

- [ ] **Step 4: Run the tests**

Run: `uv run python -m pytest tests/test_robots.py -v`, then the full suite: `uv run python -m pytest -q`.
Expected: everything passes. If an existing test asserts the exact old message ("use ov_degrees"), the new message still contains that text.

- [ ] **Step 5: Commit**

```bash
git add src/mujoco_module/robots.py tests/test_robots.py
git commit -m "feat: SVA kinematics accept euler_angles orientations (RDK ZYX)"
```

---

### Task 2: Vendor the SO-101 and register the asset (sim)

**Files:**
- Create: `assets/robotstudio_so101/` (vendored files, wrapper, kinematics, README)
- Modify: `src/mujoco_module/sim.py` (`KNOWN_ASSETS`, `GRIPPER_ASSETS`)
- Create: `tests/test_so101.py`

- [ ] **Step 1: Vendor the files**

```bash
cd /Users/nick.hehr/src/viam-mujoco-sim-so101
SRC="${TMPDIR:-/tmp}/gh-clones-cc53bb8a-07c6-42b7-93b2-cb3cb4096b47/SO-ARM100"
test "$(git -C "$SRC" rev-parse --short HEAD)" = a758567 || echo "WARNING: upstream clone not at a758567"
mkdir -p assets/robotstudio_so101
cp "$SRC/Simulation/SO101/so101_new_calib.xml" "$SRC/Simulation/SO101/joints_properties.xml" assets/robotstudio_so101/
cp -R "$SRC/Simulation/SO101/assets" assets/robotstudio_so101/assets
cp "$SRC/LICENSE" assets/robotstudio_so101/LICENSE
cp /Users/nick.hehr/src/so-101/internal/geometry/so101.json assets/robotstudio_so101/so101_kinematics.json
cmp "$SRC/Simulation/SO101/so101_new_calib.xml" assets/robotstudio_so101/so101_new_calib.xml && echo byte-identical
```

Create `assets/robotstudio_so101/so101.xml`:

```xml
<!-- Wrapper for the vendored upstream SO-101 (see README.md): adds only a home keyframe.
     Lives beside so101_new_calib.xml so its meshdir="assets" resolves. -->
<mujoco model="so101">
  <include file="so101_new_calib.xml"/>
  <keyframe>
    <!-- 5 arm joints (rad) then the gripper; mid-workspace, free of self-contact -->
    <key name="home" qpos="0 0.3 0.3 0.5 0 0" ctrl="0 0.3 0.3 0.5 0 0"/>
  </keyframe>
</mujoco>
```

Create `assets/robotstudio_so101/README.md`:

```markdown
# SO-101 (RobotStudio) for viam-mujoco-sim

| File | Source |
|---|---|
| `so101_new_calib.xml`, `joints_properties.xml`, `assets/` (whole folder), `LICENSE` | TheRobotStudio/SO-ARM100 `Simulation/SO101` @ `a758567` (committed 2026-10-06; fetched 2026-11-15), Apache-2.0, byte-for-byte |
| `so101.xml` | Written here: includes `so101_new_calib.xml` and adds a `home` keyframe (`[0, 0.3, 0.3, 0.5, 0]` rad, gripper 0) |
| `so101_kinematics.json` | The so-101 Viam module (`devrel:so101-arm`) `internal/geometry/so101.json` @ `96217b8` (SVA, euler_angles) |

- **End effector:** Viam's end effector for this arm is the **wrist_roll axis** (so101.json's `tool`). The MJCF's `gripperframe` site is the jaw tip, about 98 mm beyond it. The registry's `flange_offset` maps one to the other. It was measured with the sim's flange check, and the residual is about 0.03 mm / 0.01° at zero, home, mixed and random poses, set by the offset's 0.1 mm / 0.1° rounding.
- **Servos:** the STS3215 servo parameters come from upstream (kp 998.22, kv 2.731, force ±3.35 N·m, friction and armature), which upstream adapted from Open Duck Mini. They aren't a 1:1 match for the real servos, and the sim gravity-compensates every body. So this asset has the real geometry, limits, friction and force limits, but not realistic gravity sag or servo lag.
- **Gripper:** the jaw is closed at −0.1745 rad, its lower limit, and open at 1.7453 rad. Grasping is untested.
- **Refreshing:** re-copy the four upstream items from a newer commit, then re-run `tests/test_so101.py`.
```

- [ ] **Step 2: Write the failing tests** (`tests/test_so101.py`)

```python
import json
import math
from pathlib import Path

import mujoco
import numpy as np
import pytest

from mujoco_module import robots
from mujoco_module.handles import ArmHandle, GripperHandle, JointTrajectoryPoint
from mujoco_module.sim import GRIPPER_ASSETS, KNOWN_ASSETS

ASSET = Path(__file__).parents[1] / "assets" / "robotstudio_so101"
HOME = [0.0, 0.3, 0.3, 0.5, 0.0]


def make_arm(sim, **attrs):
    # 100 mm riser: at home the jaw tip is level with the base, so on the floor it would touch
    return ArmHandle(sim, "arm", {"asset": "so101", "position": [0, 0, 0.1], **attrs})


def test_so101_home_touches_nothing(sim):
    make_arm(sim)
    mujoco.mj_forward(sim.model, sim.data)
    bodies = {
        (sim.model.body(sim.model.geom_bodyid[c.geom1]).name, sim.model.body(sim.model.geom_bodyid[c.geom2]).name)
        for c in sim.data.contact[: sim.data.ncon]
    }
    assert {p for p in bodies if any(n.startswith("arm/") for n in p)} == set()


def test_so101_starts_at_home(sim):
    assert make_arm(sim).get_joint_positions() == pytest.approx(HOME, abs=1e-6)


def test_so101_vendored_files_are_registered():
    entry = KNOWN_ASSETS["so101"]
    assert (Path(__file__).parents[1] / "assets" / entry["mjcf"]).exists()
    kin = json.loads((Path(__file__).parents[1] / "assets" / entry["kinematics_url"]).read_text())
    assert kin["name"] == "SO101" and len(kin["joints"]) == 5
    assert entry["flange_site"] == "gripperframe"
    # GetKinematics serves this file: it must be the so-101 module's own, unchanged
    assert (ASSET / "so101_kinematics.json").read_bytes() == Path(
        "/Users/nick.hehr/src/so-101/internal/geometry/so101.json"
    ).read_bytes()


@pytest.mark.parametrize(
    "q",
    [[0.0] * 5, HOME, [0.5, -0.5, 0.5, -0.5, 0.5], [0.7, -0.9, 1.1, -1.2, 2.0]],
    ids=["zero", "home", "mixed", "random"],
)
def test_so101_flange_matches_viam_kinematics(sim, q):
    """Residual of the MJCF flange (with flange_offset) against so101.json: if this fails,
    the motion service and diff-ik plan against the wrong arm."""
    arm = make_arm(sim)
    ids = arm._now()[1]
    sim.data.qpos[ids.qpos] = q
    mujoco.mj_kinematics(sim.model, sim.data)
    (x, y, z), quat = arm.get_end_pose()
    kin = robots.parse_kinematics((ASSET / "so101_kinematics.json").read_bytes(), urdf=False, label="so101")
    expected = kin.forward(list(q))[kin.end]
    got = robots.transform(quat=quat)
    got[:3, 3] = [x, y, z]
    metres, degrees = robots.pose_error(got, expected)
    assert metres < 1e-3 and degrees < 0.5, f"flange residual {metres * 1000:.3f} mm {degrees:.3f}°"


def test_so101_reaches_a_joint_target(sim):
    arm = make_arm(sim)
    target = [0.4, 0.1, 0.6, 0.2, -0.5]
    arm.set_joint_targets(target)
    done = arm.wait_for_joint_targets(target, math.radians(0.5))
    while not done.done() and sim.data.time < 10.0:
        sim.advance(0.01)
    assert done.done()
    assert arm.get_joint_positions() == pytest.approx(target, abs=math.radians(0.5))


def test_so101_trajectory_ends_at_last_point(sim):
    arm = make_arm(sim)
    mid = [0.2, 0.3, 0.3, 0.5, 0.0]
    end = [0.4, 0.2, 0.4, 0.4, 0.3]
    done = arm.start_trajectory(
        [JointTrajectoryPoint(0.0, tuple(HOME)), JointTrajectoryPoint(1.0, tuple(mid)), JointTrajectoryPoint(2.0, tuple(end))]
    )
    sim.advance(2.05)
    assert done.done()
    sim.advance(0.5)
    assert arm.get_joint_positions() == pytest.approx(end, abs=0.01)


def test_so101_gripper_opens_and_closes(sim):
    make_arm(sim)
    gripper = GripperHandle(sim, "gripper", "arm", {"asset": "so101"})
    assert (gripper.open_q, gripper.closed_q) == pytest.approx((1.74533, -0.17453), abs=1e-4)
    gripper.open_jaws()
    sim.advance(1.0)  # the jaw takes ~0.5 s to open and ~0.6-0.75 s to close
    assert gripper.jaw() == pytest.approx(gripper.open_q, abs=0.01)
    assert not gripper.is_moving()
    gripper.close_jaws()
    sim.advance(1.0)
    assert gripper.jaw() == pytest.approx(gripper.closed_q, abs=0.01)


def test_so101_gripper_asset_entry():
    assert GRIPPER_ASSETS["so101"] == {"open": 1.74533, "closed": -0.17453}
```

- [ ] **Step 3: Run them to verify they fail**

Run: `uv run python -m pytest tests/test_so101.py -v`
Expected: a KeyError for `"so101"`, or "unknown asset", because nothing is registered yet.

- [ ] **Step 4: Register the asset** in `src/mujoco_module/sim.py`. Add to `KNOWN_ASSETS`, after `"ur5e"`:

```python
    "so101": {
        # TheRobotStudio/SO-ARM100@a758567 Simulation/SO101 (Apache-2.0), wrapped by
        # so101.xml (adds a home keyframe); STS3215 servos from upstream
        "mjcf": "robotstudio_so101/so101.xml",
        # vendored from the so-101 module (devrel:so101-arm) internal/geometry/so101.json @ 96217b8
        "kinematics_url": "robotstudio_so101/so101_kinematics.json",
        # Viam's end effector is the wrist_roll axis; gripperframe is the jaw tip ~98 mm beyond it
        "flange_site": "gripperframe",
        "flange_offset": {
            "translation": {"x": -98.1, "y": 0.2, "z": 7.9},
            "orientation": {"o_x": 1, "o_y": 0, "o_z": 0, "theta": -2.8},
        },
    },
```

Add to `GRIPPER_ASSETS`:

```python
    # the SO-101's own jaw: ctrl -0.1745 (its lower limit) closes it, 1.7453 opens it;
    # set both, since the default (open = low) is the wrong way round for this jaw
    "so101": {"open": 1.74533, "closed": -0.17453},
```

- [ ] **Step 5: Run the tests, measuring the offset if the flange test fails**

Run: `uv run python -m pytest tests/test_so101.py -v`.

If `test_so101_flange_matches_viam_kinematics` fails, re-measure the offset:
1. Temporarily delete `flange_offset` from the entry and run `uv run python -m pytest tests/test_so101.py -k home -v`. Configuring fails with a message ending `try flange_offset: {...}`.
2. Copy that value into the entry and re-run.
3. Record the final value in `assets/robotstudio_so101/README.md`.

The spec review measured `{"translation": {"x": -98.1, "y": 0.2, "z": 7.9}, "orientation": {"o_x": 1, "o_y": 0, "o_z": 0, "theta": -2.8}}`, with a residual of about 0.03 mm / 0.01° (set by the offset's 0.1 mm / 0.1° rounding).

If the gripper's closed position stops short of −0.1745 because of contact, report the measured jaw value. Don't loosen the tolerance silently.

Then run the full suite: `uv run python -m pytest -q`. Expected: all pass.

- [ ] **Step 6: Commit**

```bash
git add assets/robotstudio_so101 src/mujoco_module/sim.py tests/test_so101.py
git commit -m "feat: bundled SO-101 arm and gripper (asset so101) from TheRobotStudio's MJCF"
```

---

### Task 3: Docs and example (sim)

**Files:**
- Modify: `src/mujoco_module/models/arm.py` (docstring `asset`), `src/mujoco_module/models/gripper.py` (docstring `asset`), `README.md`
- Create: `examples/so101.json`

- [ ] **Step 1: Docstrings**
  - `models/arm.py`: change `asset … a bundled arm, "lite6" (with its wide gripper) or "ur5e";` to also list `"so101"` (RobotStudio SO-101, 5-DOF, with its jaw).
  - `models/gripper.py`: add `"so101"`, the SO-101's own jaw, in its arm MJCF, to the `asset` list.

- [ ] **Step 2: README**
  - **Arm assets table** (after `ur5e`):
    `| \`so101\` | RobotStudio SO-101 (5-DOF) with its jaw gripper | \`[0, 17.2, 17.2, 28.6, 0]\` | [so-101](https://github.com/clintpurser/so-101) (\`internal/geometry/so101.json\`) |`
  - **Gripper table:**
    `| \`so101\` | The SO-101's own jaw, in its MJCF | \`so101\` | 98 mm (Viam's end effector is the wrist_roll axis; the jaw tip is ~98 mm beyond) | none served: give the frame a \`geometry\` |`
  - **A short "SO-101" note** under the arm table:
    - its servos come from upstream, and the sim gravity-compensates it, so it has real geometry and force limits but not realistic sag or servo lag;
    - it's 5-DOF, so a position-plus-pointing task suits it (the diff-ik servo's `pointing` mode);
    - provenance is in `assets/robotstudio_so101/README.md`;
    - **frame it at least 100 mm above any surface**, because at home the jaw tip is level with the base.

- [ ] **Step 3: `examples/so101.json`**, following `examples/ur5e-pick.json`'s shape:

```json
{
  "modules": [
    {"type": "local", "name": "mujoco-sim", "executable_path": "/ABSOLUTE/PATH/TO/viam-mujoco-sim/run.sh"}
  ],
  "components": [
    {"name": "sim-world", "api": "rdk:component:generic", "model": "viam-labs:mujoco-sim:world", "attributes": {}},
    {"name": "table", "api": "rdk:component:generic", "model": "viam-labs:mujoco-sim:obstacle",
     "attributes": {"world": "sim-world"},
     "frame": {"parent": "world", "translation": {"x": 250, "y": 0, "z": -10},
               "geometry": {"type": "box", "x": 600, "y": 600, "z": 20}}},
    {"name": "arm", "api": "rdk:component:arm", "model": "viam-labs:mujoco-sim:arm",
     "attributes": {"world": "sim-world", "asset": "so101"},
     "frame": {"parent": "world", "translation": {"x": 0, "y": 0, "z": 100}}},
    {"name": "gripper", "api": "rdk:component:gripper", "model": "viam-labs:mujoco-sim:gripper",
     "attributes": {"world": "sim-world", "mount": "arm", "asset": "so101"}, "frame": {"parent": "arm"}}
  ]
}
```

Check the obstacle attributes and frame geometry format against `examples/ur5e-pick.json`'s `table` and copy that shape exactly. The arm stands 100 mm above the table top: at home the jaw tip is level with the base, so an arm sitting on the table would put the jaw in it. The 100 mm also leaves about 36 mm of clearance for diff-ik's ±10° static targets.

- [ ] **Step 4: Verify** — run `uv run python -m pytest -q`; everything passes. Also check that `examples/so101.json` parses (`python3 -m json.tool`).

- [ ] **Step 5: Commit**

```bash
git add src/mujoco_module/models/arm.py src/mujoco_module/models/gripper.py README.md examples/so101.json
git commit -m "docs: so101 asset in arm/gripper docs and README; minimal SO-101 example"
```

---

### Task 4: diff-ik dev config (diff-ik)

**Files:**
- Create: `dev/so101-mujoco.json`
- Modify: `README.md` (the "Local SO-101 simulation" section)

- [ ] **Step 1: `dev/so101-mujoco.json`**

```json
{
  "network": {"bind_address": "localhost:8082"},
  "modules": [
    {"type": "local", "name": "mujoco-sim", "executable_path": "/Users/nick.hehr/src/viam-mujoco-sim-so101/run.sh"},
    {"type": "local", "name": "diff-ik", "executable_path": "/Users/nick.hehr/src/diff-ik/bin/diff-ik"}
  ],
  "components": [
    {"name": "sim-world", "api": "rdk:component:generic", "model": "viam-labs:mujoco-sim:world", "attributes": {}},
    {"name": "table", "api": "rdk:component:generic", "model": "viam-labs:mujoco-sim:obstacle",
     "attributes": {"world": "sim-world"},
     "frame": {"parent": "world", "translation": {"x": 250, "y": 0, "z": -10},
               "geometry": {"type": "box", "x": 600, "y": 600, "z": 20}}},
    {"name": "arm", "api": "rdk:component:arm", "model": "viam-labs:mujoco-sim:arm",
     "attributes": {"world": "sim-world", "asset": "so101"},
     "frame": {"parent": "world", "translation": {"x": 0, "y": 0, "z": 100}}},
    {"name": "gripper", "api": "rdk:component:gripper", "model": "viam-labs:mujoco-sim:gripper",
     "attributes": {"world": "sim-world", "mount": "arm", "asset": "so101"}, "frame": {"parent": "arm"}}
  ],
  "services": [
    {"name": "servo", "api": "rdk:service:motion", "model": "devrel:diff-ik:servo", "attributes": {"arms": ["arm"]}}
  ]
}
```

Keep the `table` shape identical to the one `examples/so101.json` ends up with in Task 3.

- [ ] **Step 2: README** — under "Local SO-101 simulation (Phase 3a)", add:

```markdown
For physics (Phase 3b), use the MuJoCo SO-101 from viam-mujoco-sim's `so101-asset` worktree:

    make -C ~/src/viam-mujoco-sim-so101 run.sh   # generated, gitignored
    viam-server -config dev/so101-mujoco.json   # serves plaintext localhost:8082
    go run ./cmd/cli --address localhost:8082 --insecure --arm arm --task pointing run static
```

- [ ] **Step 3: Verify and commit**

```bash
python3 -m json.tool dev/so101-mujoco.json > /dev/null
git add dev/so101-mujoco.json README.md
git commit -m "Dev config for the MuJoCo SO-101 (Phase 3b), port 8082"
```

---

### Task 5: Validation runs and results — controller only

The controller does this task itself, because it runs servers.

- [ ] **Step 1:** `make -C /Users/nick.hehr/src/viam-mujoco-sim-so101 run.sh`. `run.sh` is gitignored and generated, and the worktree has none. Then `go build -o bin/diff-ik ./cmd/module`. Start `viam-server -config dev/so101-mujoco.json` in the background and confirm `arm`, `gripper` and `servo` build. The first start creates the worktree's uv environment.
- [ ] **Step 2:** Confirm the arm starts at home (`q0a`). Before each run, wait for any helper client's session to expire, because of the Phase 3a session-safety finding.
- [ ] **Step 3:** Run the diff-ik spec's validation table:
  - `cli validate`;
  - `run static`, `run singular --wrist 2 --singular-deg -74` and `run circle`, all with `--task pointing --singular-eps 0.03 --max-damping 0.05 --csv .runs/p3b-<name>.csv`;
  - the Python bench `--task pointing --joint-deg-s 90`;
  - the unreachable-pointing DoCommand script below, which is the Phase 3a script with port 8082. Save it to the session scratchpad and run it with `uv run --with viam-sdk python -I <path>`:
    ```python
    import asyncio, math
    from viam.robot.client import RobotClient
    from viam.rpc.dial import DialOptions
    from viam.services.motion import MotionClient
    async def main():
        r = await RobotClient.at_address("127.0.0.1:8082", RobotClient.Options(dial_options=DialOptions(insecure=True, disable_webrtc=True)))
        svc = MotionClient.from_robot(r, "servo")
        s = await svc.do_command({"command": "servo_start", "component_name": "arm", "frame": "arm_origin", "task": "pointing"})
        sid, p = s["session_id"], s["pose"]
        a = math.radians(20)  # rotate the tool axis 20° about base z: a pure azimuth change
        tgt = dict(p, o_x=p["o_x"]*math.cos(a) - p["o_y"]*math.sin(a), o_y=p["o_x"]*math.sin(a) + p["o_y"]*math.cos(a))
        expected = math.degrees(math.asin(math.sin(a) * math.hypot(p["o_x"], p["o_y"])))
        for _ in range(30):
            await svc.do_command({"command": "servo_target", "session_id": sid, "pose": tgt})
            await asyncio.sleep(0.1)
        st = await svc.do_command({"command": "servo_status", "session_id": sid})
        print(f"state={st['state']} pos={st['err_mm']:.2f}mm pointing={st['err_deg']:.2f}° residual={st['pointing_residual_deg']:.2f}° expected={expected:.2f}°")
        await svc.do_command({"command": "servo_stop", "session_id": sid})
        await r.close()
    asyncio.run(main())
    ```
- [ ] **Step 4:** Measure the streaming lag by cross-correlating the static and circle CSVs (as in Phase 2). Record resyncs and the stop travel.
- [ ] **Step 5:** Write `docs/results/2026-11-phase3b.md`:
  - the three-way comparison table (SO-101 MuJoCo, `so101:simulated`, UR5e MuJoCo);
  - each bar with PASS/FAIL;
  - the "what this sim can and can't show" caveats;
  - sim-derived starting values for 3c.

  Commit it.
- [ ] **Step 6:** A final Opus review of both branches. Then superpowers:finishing-a-development-branch for each repo: diff-ik into `main`, and the sim branch as the user decides.
