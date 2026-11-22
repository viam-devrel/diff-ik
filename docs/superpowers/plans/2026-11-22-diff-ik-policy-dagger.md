# Expert-gated DAgger, round 1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Collect expert corrections from SmolVLA v4's own failure states in sim, fine-tune v4 on v3-200 plus the
corrections (and, as a control, on v3-200 alone), and gate both on seeds 9100–9129.

**Architecture:** A pure trigger module decides when the policy has gone wrong. `policy_run.run_episode` gains a stop
hook that a 10 Hz sampler drives. `expert_pick.Episode` gains `takeover`, which finishes the task from any pose and
captures only that segment. The training script learns `--pretrained viam:<org>/<name>@<version>`, so a cloud job can
start from the registry checkpoint.

**Tech Stack:** Python 3.11+, viam-sdk 0.85, numpy/scipy, mujoco-sim cell `dev/bc-pick-mujoco.json`,
viam-lerobot-training (LeRobot 02655292), Viam custom training jobs.

**Spec:** `docs/superpowers/specs/2026-11-22-diff-ik-policy-dagger-design.md`

## Conventions

- diff-ik Python checks are plain `assert` scripts with a PEP 723 header, run as
  `uv run clients/python/test_<name>.py` from the repo root, printing `ok`. No pytest, no fixtures (repo style; see
  `clients/python/test_bc_pick.py`).
- viam-lerobot-training uses pytest: `cd ~/src/viam-lerobot-training && .venv/bin/python -m pytest -m "not lerobot and not gpu" -q`.
- Commit messages: imperative, no prefix, ending with the attribution lines the controller gives.
- **Controller-only tasks** (marked ★) touch the live sim, Viam data, registry uploads, billed jobs or other repos'
  branches. The controller runs them, never an implementer. ★ tasks with **[check-in]** need the user's OK first.
- Machine: `diff-ik-policy-sim`, local viam-server, credentials in the scratchpad (`diff-ik-policy-sim/cell.json`,
  never printed). vla-policy serves SmolVLA v4 (`.runs/policy-act/smolvla-v4/pkg-compat`).
- Org: DevRelDemos `16518049-9dd3-479f-9644-c5d112aa42d8`. Container `slim-2`
  (`2e3137a1-6182-461d-ae18-37f200c62696`). Dataset `diff-ik-pick-v3-200` is `6b0316ea770b27184e6eb985`.

## File map

| File | Change |
|---|---|
| `clients/python/bc_pick.py` | `RELEASE_TCP`, `six_tuple_from_pose`, `tcp_world` |
| `clients/python/test_bc_pick.py` | round-trip and TCP tests |
| `clients/python/dagger_trigger.py` | new: `Cfg`, `check` |
| `clients/python/test_dagger_trigger.py` | new: table tests |
| `clients/python/expert_pick.py` | split `run` into `_start_session` / `_pick` / `_place` / `_close`; add `takeover` |
| `clients/python/policy_run.py` | `build_parser`, `parse_args(argv, parser)`, `build_context`, `run_episode(..., stop=None)` |
| `clients/python/test_policy_run_stop.py` | new: `run_episode` with fakes and a stop hook |
| `clients/python/dagger_run.py` | new: `Watch`, `takeover_tags`, `log_row`, `main_async` |
| `clients/python/test_dagger_run.py` | new: pure parts of `dagger_run` |
| `~/src/viam-lerobot-training/lerobot-training/model/viam_package.py` | new: `resolve_pretrained` |
| `~/src/viam-lerobot-training/lerobot-training/model/training.py` | resolve `--pretrained` before `check_checkpoint` |
| `~/src/viam-lerobot-training/tests/test_viam_package.py` | new |
| `docs/results/2026-11-policy-dagger.md` | new: results |

---

### Task 1: Pose helpers in `bc_pick.py`

**Files:**
- Modify: `clients/python/bc_pick.py` (constants block near line 18; new functions after `pose_map`, ~line 117)
- Test: `clients/python/test_bc_pick.py` (append)

- [ ] **Step 1: Write the failing tests** (append to `test_bc_pick.py`, before its final `print("ok")`; add
  `RELEASE_TCP, six_tuple_from_pose, tcp_world, ARM_BASE, PLACE_ABOVE_RIM, BIN_RIM_Z` to its import list)

```python
# six_tuple_from_pose inverts pose_map (tilts at or above the floor), and tool-down reads as zero tilt
r = random.Random(7)
for _ in range(200):
    mag, az = r.uniform(TILT_MIN_DEG + 0.01, 10.0), r.uniform(0, 360)
    t = (r.uniform(-200, 600), r.uniform(-400, 400), r.uniform(50, 500), r.uniform(-180, 180), *tilt_vector(mag, az))
    back = six_tuple_from_pose(pose_map(t))
    assert all(abs(a - b) < 1e-6 for a, b in zip(back[:3], t[:3])), (t, back)
    assert abs(angle_diff(back[3], t[3])) < 1e-4, (t, back)
    assert abs(back[4] - t[4]) < 1e-4 and abs(back[5] - t[5]) < 1e-4, (t, back)
down = six_tuple_from_pose(pose_map((10.0, 20.0, 30.0, 33.0)))
assert down[4] == 0.0 and down[5] == 0.0 and abs(angle_diff(down[3], 33.0)) < 1e-4, down
under = six_tuple_from_pose(pose_map((0.0, 0.0, 0.0, 10.0, 0.5, 0.0)))  # pose_map snaps it to tool-down
assert under[4] == 0.0 and under[5] == 0.0, under

# tcp_world: tool-down agrees with flange_from_tcp; any tilt keeps the TCP at TCP_MM from the flange
p = pose_map((100.0, 50.0, 300.0, 15.0))
tcp = tcp_world(p)
assert all(abs(a - b) < 1e-6 for a, b in zip(tcp, (100.0 + ARM_BASE[0], 50.0 + ARM_BASE[1], 300.0 - TCP_MM + ARM_BASE[2])))
assert all(abs(a - b) < 1e-6 for a, b in zip(flange_from_tcp(world_to_arm(tcp)), (100.0, 50.0, 300.0)))
pt = pose_map((100.0, 50.0, 300.0, 15.0, *tilt_vector(10.0, 45.0)))
d = [a - b - c for a, b, c in zip(tcp_world(pt), (100.0, 50.0, 300.0), ARM_BASE)]
assert abs(math.hypot(*d) - TCP_MM) < 1e-6 and d[2] < -0.98 * TCP_MM
assert RELEASE_TCP == (BIN_XY[0], BIN_XY[1], BIN_RIM_Z + PLACE_ABOVE_RIM)
```

- [ ] **Step 2: Run to verify it fails**

Run: `uv run clients/python/test_bc_pick.py`
Expected: `ImportError: cannot import name 'RELEASE_TCP'`

- [ ] **Step 3: Implement** in `bc_pick.py`

After `TILTED = ...` in the constants block:

```python
RELEASE_TCP = (BIN_XY[0], BIN_XY[1], BIN_RIM_Z + PLACE_ABOVE_RIM)  # where the expert releases (TCP, world mm)
```

After `pose_map`:

```python
def six_tuple_from_pose(pose: dict) -> tuple:
    """The inverse of pose_map: a flange pose map (arm frame) as (x, y, z, theta, tx, ty). The tilt is the smallest
    rotation about a horizontal axis that takes tool-down to the pose's tool axis; tilts under TILT_MIN_DEG read as
    tool-down, as pose_map writes them."""
    R = rotation(pose)
    z = R.apply([0.0, 0.0, 1.0])  # the tool axis in the arm frame
    axis = np.array([z[1], -z[0], 0.0])  # (0, 0, -1) x z: horizontal
    ang = math.degrees(math.atan2(np.linalg.norm(axis), -z[2]))
    tx = ty = 0.0
    if ang >= TILT_MIN_DEG:
        tx, ty = (float(v) for v in axis[:2] / np.linalg.norm(axis) * ang)
    tilt = Rotation.from_rotvec(np.radians([tx, ty, 0.0]))
    theta = pose_map_from((0.0, 0.0, 0.0), tilt.inv() * R)["theta"]  # tool-down after removing the tilt
    return (float(pose["x"]), float(pose["y"]), float(pose["z"]), theta, tx, ty)


def tcp_world(pose: dict) -> tuple:
    """The tool point in world mm for a flange pose map in the arm frame, any orientation."""
    off = rotation(pose).apply([0.0, 0.0, TCP_MM])
    return tuple(float(pose[k] + o + b) for k, o, b in zip("xyz", off, ARM_BASE))
```

- [ ] **Step 4: Run to verify it passes**

Run: `uv run clients/python/test_bc_pick.py`
Expected: `ok`. Also run `uv run clients/python/test_policy_layout.py` (shares the pose code): `ok`.

- [ ] **Step 5: Commit**

```bash
git add clients/python/bc_pick.py clients/python/test_bc_pick.py
git commit -m "bc_pick: invert pose_map and find the TCP for any flange pose"
```

---

### Task 2: Takeover triggers, `dagger_trigger.py`

**Files:**
- Create: `clients/python/dagger_trigger.py`
- Test: `clients/python/test_dagger_trigger.py`

A state is a dict: `t` (s since the policy started), `tcp` (world mm, 3-tuple), `block` (world mm, 3-tuple),
`grip_cmd` (1 when the gripper's latched `input` is closed, else 0), `holding` (firm grasp). `hist` is the list of
earlier states, oldest first, sampled at 10 Hz.

- [ ] **Step 1: Write the failing test** `clients/python/test_dagger_trigger.py`

```python
# /// script
# requires-python = ">=3.11"
# dependencies = ["viam-sdk", "numpy", "scipy"]
# ///
"""Table checks for dagger_trigger.py. Run: uv run clients/python/test_dagger_trigger.py"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bc_pick import BIN_XY, GRASP_ABOVE_CENTER, RELEASE_TCP  # noqa: E402
from dagger_trigger import Cfg, check  # noqa: E402

BLOCK = (500.0, 50.0, 320.0)
GRASP = (BLOCK[0], BLOCK[1], BLOCK[2] + GRASP_ABOVE_CENTER)
CFG = Cfg()


def st(t, tcp, block=BLOCK, grip=0, holding=False):
    return {"t": round(t, 3), "tcp": tcp, "block": block, "grip_cmd": grip, "holding": holding}


def run(states):
    """check() at every state with all earlier states as history; returns the list of results."""
    return [check(s, states[:i], CFG) for i, s in enumerate(states)]


def ticks(t0, t1):
    n = round((t1 - t0) * 10)
    return [t0 + i / 10 for i in range(n + 1)]


# approach_stall: static far from the block; nothing until the 2 s window fills
far = (GRASP[0] - 100, GRASP[1], GRASP[2] + 100)
out = run([st(t, far) for t in ticks(0.0, 2.0)])
assert out[:-1] == [None] * (len(out) - 1) and out[-1] == "approach_stall", out

# approaching at 50 mm/s: no stall
out = run([st(t, (far[0] + 50 * t, far[1], far[2])) for t in ticks(0.0, 1.9)] +
          [st(t, (far[0] + 50 * t, far[1], far[2])) for t in ticks(2.0, 2.0)])
assert out[-1] is None, out

# parked within 15 mm of the grasp point (closing on it): no stall
near = (GRASP[0] + 5, GRASP[1], GRASP[2])
assert run([st(t, near) for t in ticks(0.0, 3.0)])[-1] is None

# empty_grasp: closed at 5.0 with no firm grasp; fires at 6.0, not at 5.9
s = [st(t, near) for t in ticks(4.0, 4.9)] + [st(t, near, grip=1) for t in ticks(5.0, 6.0)]
out = run(s)
assert out[-2] is None and out[-1] == "empty_grasp", out

# grasped, then dropped: no empty_grasp while it falls, drop once it rests outside the bin
s = [st(t, near, grip=1) for t in ticks(5.0, 5.2)]
s += [st(t, (near[0], near[1], near[2] + 20 * (t - 5.3)), grip=1, holding=True) for t in ticks(5.3, 7.0)]
s += [st(7.1, (near[0], near[1], 400.0), block=(BLOCK[0], BLOCK[1], 360.0), grip=1),
      st(7.2, (near[0], near[1], 400.0), block=(BLOCK[0], BLOCK[1], 330.0), grip=1),
      st(7.3, (near[0], near[1], 400.0), block=(BLOCK[0], BLOCK[1], 320.0), grip=1),
      st(7.4, (near[0], near[1], 400.0), block=(BLOCK[0], BLOCK[1], 320.0), grip=1)]
out = run(s)
assert out[-4] is None and out[-3] is None and out[-2] is None and out[-1] == "drop", out[-5:]

# released over the bin: falling, then at rest in the bin; never a trigger
hold = (RELEASE_TCP[0], RELEASE_TCP[1], RELEASE_TCP[2])
inside = (BIN_XY[0], BIN_XY[1])
s = [st(t, hold, block=(*inside, 480.0), grip=1, holding=True) for t in ticks(10.0, 12.0)]
s += [st(12.1, hold, block=(*inside, 450.0)), st(12.2, hold, block=(*inside, 400.0)),
      st(12.3, hold, block=(*inside, 340.0)), st(12.4, hold, block=(*inside, 325.0)),
      st(12.5, hold, block=(*inside, 325.0))]
assert run(s)[-5:] == [None] * 5, run(s)[-5:]

# carry_stall: holding, parked away from the bin; a vertical lift is progress; hovering over the bin is not a stall
park = (400.0, 100.0, 450.0)
assert run([st(t, park, grip=1, holding=True) for t in ticks(0.0, 2.0)])[-1] == "carry_stall"
lift = [st(t, (400.0, 100.0, 330.0 + 50 * t), grip=1, holding=True) for t in ticks(0.0, 2.0)]
assert run(lift)[-1] is None
over = (RELEASE_TCP[0] + 30, RELEASE_TCP[1], RELEASE_TCP[2] - 40)
assert run([st(t, over, grip=1, holding=True) for t in ticks(0.0, 3.0)])[-1] is None

# timeout, and precedence (drop beats timeout)
s = [st(t, (far[0] + 50 * (t - 23.0), far[1], far[2])) for t in ticks(23.0, 25.0)]
assert run(s)[-1] == "timeout"
s = [st(24.8, near, holding=True, grip=1), st(24.9, near, block=(450.0, 0.0, 320.0), grip=1),
     st(25.0, near, block=(450.0, 0.0, 320.0), grip=1)]
assert run(s)[-1] == "drop"
print("ok")
```

- [ ] **Step 2: Run to verify it fails**

Run: `uv run clients/python/test_dagger_trigger.py`
Expected: `ModuleNotFoundError: No module named 'dagger_trigger'`

- [ ] **Step 3: Implement** `clients/python/dagger_trigger.py`

```python
"""Takeover triggers for expert-gated DAgger (spec docs/superpowers/specs/2026-11-22-diff-ik-policy-dagger-design.md).

A state is {"t": s since the policy started, "tcp": world mm, "block": world mm, "grip_cmd": 0 or 1,
"holding": firm grasp}; hist is the earlier states, oldest first (10 Hz)."""
import math
from dataclasses import dataclass

from bc_pick import GRASP_ABOVE_CENTER, RELEASE_TCP, in_bin


@dataclass(frozen=True)
class Cfg:
    stall_s: float = 2.0
    stall_mm: float = 10.0
    near_mm: float = 15.0
    release_xy_mm: float = 40.0
    empty_s: float = 1.0
    timeout_s: float = 25.0
    rest_mm: float = 1.0


def grasp_point(block) -> tuple:
    return (block[0], block[1], block[2] + GRASP_ABOVE_CENTER)


def _ref(state, hist, cfg):
    """The newest earlier state at least stall_s old; None while the window is filling."""
    for h in reversed(hist):
        if state["t"] - h["t"] >= cfg.stall_s - 1e-9:
            return h
    return None


def check(state, hist, cfg: Cfg = Cfg()):
    """The trigger kind, or None. Order: drop, empty_grasp, approach_stall, carry_stall, timeout."""
    prev = hist[-1] if hist else None
    at_rest = prev is not None and math.dist(prev["block"], state["block"]) < cfg.rest_mm
    if not state["holding"] and at_rest and not in_bin(state["block"]) and any(h["holding"] for h in hist):
        return "drop"
    if state["grip_cmd"] and not state["holding"]:
        since, held = state["t"], False
        for h in reversed(hist):
            if not h["grip_cmd"]:
                break
            since, held = h["t"], held or h["holding"]
        if not held and state["t"] - since >= cfg.empty_s - 1e-9:
            return "empty_grasp"
    ref = _ref(state, hist, cfg)
    if ref is not None:
        if not state["holding"] and at_rest and not in_bin(state["block"]):
            d = math.dist(state["tcp"], grasp_point(state["block"]))
            d0 = math.dist(ref["tcp"], grasp_point(ref["block"]))
            if d > cfg.near_mm and d0 - d < cfg.stall_mm:
                return "approach_stall"
        if state["holding"] and math.dist(state["tcp"][:2], RELEASE_TCP[:2]) > cfg.release_xy_mm:
            if math.dist(ref["tcp"], RELEASE_TCP) - math.dist(state["tcp"], RELEASE_TCP) < cfg.stall_mm:
                return "carry_stall"
    if state["t"] >= cfg.timeout_s - 1e-9:
        return "timeout"
    return None
```

- [ ] **Step 4: Run to verify it passes**

Run: `uv run clients/python/test_dagger_trigger.py`
Expected: `ok`. If a table case fails, fix the code, not the table, unless the case contradicts the spec's trigger
table (then report it).

- [ ] **Step 5: Commit**

```bash
git add clients/python/dagger_trigger.py clients/python/test_dagger_trigger.py
git commit -m "DAgger takeover triggers: drop, empty grasp, approach and carry stalls, timeout"
```

---

### Task 3: Expert takeover from any state, `expert_pick.py`

**Files:**
- Modify: `clients/python/expert_pick.py` (`Episode.run`, lines ~94–165; new methods on `Episode`)

No pure test: the refactor is checked by review (the call sequence of `run` must not change) and by the controller's
sim smoke in Task 7.

- [ ] **Step 1: Split `run` without changing its behaviour.** Move code, don't rewrite it:
  - `async def _start_session(self) -> dict`: the `servo_start` call (same arguments), sets `self.sid`, returns
    `r["pose"]`.
  - `async def _pick(self, cur, sc)`: from `(wx, wy, wz), q = await self.block_world_mm()` through the lift move;
    returns `cur`.
  - `async def _place(self, cur, sc)`: from `bin_tcp = ...` through the in-bin check (raises `Discard`).
  - `async def _close(self, info, capturing)`: the body of `run`'s `finally` (servo_stop, then stop_capture with the
    same warning, `kept=False`, `reason` and `abort` handling). Skip the `servo_stop` when `self.sid is None` (a
    `servo_start` that failed in `takeover`).
  - `run` becomes: scene, place, open, `p0 = await self._start_session()`, the same `cur`/`info`/tool-down check, the
    start move and hold, the capture start, `cur = await self._pick(cur, sc)`, `await self._place(cur, sc)`,
    `info["kept"] = True`, the same `except` clauses, `finally: await self._close(info, capturing)`.

- [ ] **Step 2: Add `takeover`** on `Episode`:

```python
    async def takeover(self, seed, tags=None) -> dict:
        """Finish the pick from wherever the policy left the arm (DAgger spec §2-3). Capture (when tags are given)
        starts before the first expert move, so the episode's first frame is the takeover state."""
        sc = C.sample_scene(seed)
        info = {"seed": seed, "scene": sc, "kept": False, "reason": ""}
        capturing = False
        self.sid = None
        try:
            cur = C.six_tuple_from_pose(await self._start_session())
            holding = bool((await self.gripper.is_holding_something()).meta.get("firm_grasp"))
            info["holding"] = holding
            if tags is not None:
                await self.capture.do_command({"start_capture": True, "frequency_hz": self.a.rate_hz, "tags": tags})
                capturing, info["t_start"] = True, datetime.now(timezone.utc).isoformat()
            if holding:
                over_z = C.flange_from_tcp(C.world_to_arm(C.RELEASE_TCP))[2]
                if cur[2] < over_z:  # rise straight up first, clear of the bin walls
                    cur = await self.move(cur, (cur[0], cur[1], over_z, cur[3], 0.0, 0.0), sc["speed"] * 0.8,
                                          check_grasp=True)
            else:
                await self.hold_during(cur, 0.6, self.gripper.open())
                (wx, wy, wz), _ = await self.block_world_mm()
                pre_tcp = (wx, wy, wz + C.GRASP_ABOVE_CENTER + C.PREGRASP_ABOVE)
                pre_z = C.flange_from_tcp(C.world_to_arm(pre_tcp))[2]
                if cur[2] < pre_z:  # rise straight up to pregrasp height, clear of the block
                    cur = await self.move(cur, (cur[0], cur[1], pre_z, *cur[3:]), sc["speed"])
                cur = await self._pick(cur, sc)
            await self._place(cur, sc)
            info["kept"] = True
        except Discard as e:
            info["reason"] = str(e)
        except Exception as e:
            info["reason"] = f"error: {e!r}"
        finally:
            await self._close(info, capturing)
        return info
```

- [ ] **Step 3: Check it still imports and the existing checks pass**

Run: `uv run clients/python/expert_pick.py --help` (prints usage) and `uv run clients/python/test_bc_pick.py` (`ok`).
Diff review: in `git diff`, every line removed from `run` must reappear unchanged in `_start_session`, `_pick`,
`_place` or `_close`.

- [ ] **Step 4: Commit**

```bash
git add clients/python/expert_pick.py
git commit -m "Expert: finish the pick from any state (takeover), sharing run's pick and place steps"
```

---

### Task 4: Stop hook in `policy_run.py`

**Files:**
- Modify: `clients/python/policy_run.py` (`run_episode` ~line 156; `main_async` ~line 243; `parse_args` ~line 296)
- Test: `clients/python/test_policy_run_stop.py` (new)

- [ ] **Step 1: Write the failing test** `clients/python/test_policy_run_stop.py`

```python
# /// script
# requires-python = ">=3.11"
# dependencies = ["viam-sdk", "numpy", "scipy", "pillow"]
# ///
"""run_episode with fakes: the stop hook ends the loop and the row records the trigger.
Run: uv run clients/python/test_policy_run_stop.py"""
import asyncio
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import policy_run  # noqa: E402

DOWN = {"x": 400.0, "y": 0.0, "z": 300.0, "o_x": 0.0, "o_y": 0.0, "o_z": -1.0, "theta": 0.0}
ROW = [400.0, 0.0, 300.0, 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0]


async def fake_observe(cams, arm, grip, specs):
    import time
    return time.monotonic(), {}, dict(DOWN), 0.0


async def idle(*_a, **_k):
    await asyncio.sleep(3600)


class World:
    async def do_command(self, cmd):
        return {}


class Grip:
    async def open(self):
        pass


class Policy:
    async def do_command(self, cmd):
        return {"actions": {"rows": [ROW] * 20}}


class Exec:
    def __init__(self):
        self.errors, self.msgs = [], 0

    async def start(self, seed):
        pass

    async def play(self, chunk):
        pass

    async def wait_done(self):
        pass

    async def close(self):
        return {"state": "stopped", "peak_accel_ratio": None, "msgs": 0}


class Stop:
    def __init__(self, results):
        self.results, self.t0, self.ended = list(results), None, False

    def begin(self, t0):
        self.t0 = t0

    def check(self):
        return self.results.pop(0) if self.results else None

    def end(self):
        self.ended = True


policy_run.observe, policy_run.success_task, policy_run.jerk_task = fake_observe, idle, idle
c = {"make_exec": Exec, "world": World(), "gripper": Grip(), "policy": Policy(), "specs": {}, "cams": {},
     "arm": None, "grip": None, "side_arm": None, "side_grip": None}

a = policy_run.parse_args(["--address", "x", "--out", os.devnull, "--timeout-s", "5"])
stop = Stop([None, "approach_stall"])
row = asyncio.run(policy_run.run_episode(a, c, 9100, stop=stop))
assert row["trigger"] == "approach_stall" and row["chunks"] == 2, row
assert stop.t0 is not None and stop.ended

a = policy_run.parse_args(["--address", "x", "--out", os.devnull, "--timeout-s", "0.3"])
row = asyncio.run(policy_run.run_episode(a, c, 9100))
assert row["trigger"] is None and row["outcome"] == "timeout", row

ap = policy_run.build_parser()
ap.add_argument("--extra-flag", default="x")
a = policy_run.parse_args(["--address", "x", "--out", os.devnull], parser=ap)
assert a.extra_flag == "x" and a.lead_ms == 100.0
print("ok")
```

- [ ] **Step 2: Run to verify it fails**

Run: `uv run clients/python/test_policy_run_stop.py`
Expected: `TypeError: run_episode() got an unexpected keyword argument 'stop'`

- [ ] **Step 3: Implement**

1. `run_episode(a, c, seed, stop=None)`. Add `trigger = None` with the other locals. Right after
   `t0, started = time.monotonic(), True`: `if stop is not None: stop.begin(t0)`. At the end of the loop body (after
   the sync `race` or the async `race`):

   ```python
                    if stop is not None and not ev.is_set():
                        trigger = stop.check()
                        if trigger:
                            break
   ```

   First line of the outer `finally`: `if stop is not None: stop.end()`. At the end, replace `return episode_row(...)`
   with `row = episode_row(...)`, `row["trigger"] = trigger`, `return row`.
2. `build_parser()`: everything in today's `parse_args` up to `a = ap.parse_args(argv)`, returning `ap`.
   `parse_args(argv=None, parser=None)`: `ap = parser or build_parser()`, then today's parsing and validation.
3. `async def build_context(a, servo_r, obs_r, side_r) -> dict`: the body of `main_async` from `policy, world = ...`
   through building `c` (the specs checks and `Executor` class settings included), returning `c`. Move the function-
   local imports (`Arm`, `Camera`, `Gripper`, `GenericComponent`, `Generic`, `MotionClient`) into `build_context` with
   it. `main_async` calls it. No test calls `build_context`; check it with `--once` in Task 7.

- [ ] **Step 4: Run to verify it passes**

Run: `uv run clients/python/test_policy_run_stop.py` and `uv run clients/python/test_policy_run.py`
Expected: `ok` and `ok`.

- [ ] **Step 5: Commit**

```bash
git add clients/python/policy_run.py clients/python/test_policy_run_stop.py
git commit -m "policy_run: stop hook per chunk, reusable parser and context"
```

---

### Task 5: Rollouts with takeover, `dagger_run.py`

**Files:**
- Create: `clients/python/dagger_run.py`
- Test: `clients/python/test_dagger_run.py`

- [ ] **Step 1: Write the failing test** `clients/python/test_dagger_run.py`

```python
# /// script
# requires-python = ">=3.11"
# dependencies = ["viam-sdk", "numpy", "scipy", "pillow"]
# ///
"""Pure checks for dagger_run.py. Run: uv run clients/python/test_dagger_run.py"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dagger_run import Watch, cfg_from_args, log_row, parse_args, takeover_tags  # noqa: E402
from dagger_trigger import Cfg  # noqa: E402

assert takeover_tags("dagger1", "r1", 3005, "drop") == [
    "cmd:Pick up the red block and place it in the bin", "episode:dagger1-3005", "dagger:r1", "trigger:drop",
    "expert:v2-takeover"]

row = {"seed": 3005, "outcome": "trigger", "trigger": "drop", "errors": [], "chunks": 4, "checkpoint": "v4",
       "jerk_rms": 1.0}
assert log_row(row) == {"seed": 3005, "outcome": "trigger", "trigger": "drop", "errors": [], "chunks": 4,
                        "checkpoint": "v4"}

a = parse_args(["--address", "x"])
assert a.seeds == "3000-3299" and a.max_kept == 80 and a.timeout_s == 40.0 and a.out.endswith("dagger1.jsonl")
assert a.capture_sensor == "demo-capture" and a.rate_hz == 10.0 and a.mode == "module" and a.cadence == "sync"
assert cfg_from_args(a) == Cfg()

w = Watch(None, None, None, Cfg())
assert w.check() is None
far = (400.0, 50.0, 428.0)
w.hist = [{"t": i / 10, "tcp": far, "block": (500.0, 50.0, 320.0), "grip_cmd": 0, "holding": False}
          for i in range(21)]
assert w.check() == "approach_stall"
print("ok")
```

- [ ] **Step 2: Run to verify it fails**

Run: `uv run clients/python/test_dagger_run.py`
Expected: `ModuleNotFoundError: No module named 'dagger_run'`

- [ ] **Step 3: Implement** `clients/python/dagger_run.py`

```python
# /// script
# requires-python = ">=3.11"
# dependencies = ["viam-sdk", "numpy", "scipy", "pillow"]
# ///
"""Expert-gated DAgger rollouts (spec docs/superpowers/specs/2026-11-22-diff-ik-policy-dagger-design.md): the policy
runs; on a trigger the scripted expert finishes from that state, and only the expert's segment is captured.

  uv run clients/python/dagger_run.py --address <addr> --seeds 3000-3299 --max-kept 80
  uv run clients/python/expert_pick.py add --log .runs/policy-dagger/dagger1.jsonl --dataset diff-ik-pick-dagger-r1 \\
      --part-id <part>
"""
import asyncio
import collections
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bc_pick  # noqa: E402
import dagger_trigger  # noqa: E402
import policy_run  # noqa: E402
from expert_pick import INSTRUCTION, Episode  # noqa: E402

LOG_KEYS = ("seed", "outcome", "trigger", "errors", "chunks", "checkpoint")


def takeover_tags(name, rnd, seed, kind) -> list:
    return [f"cmd:{INSTRUCTION}", f"episode:{name}-{seed}", f"dagger:{rnd}", f"trigger:{kind}", "expert:v2-takeover"]


def log_row(row) -> dict:
    return {k: row.get(k) for k in LOG_KEYS}


def cfg_from_args(a) -> dagger_trigger.Cfg:
    return dagger_trigger.Cfg(stall_s=a.stall_s, stall_mm=a.stall_mm, near_mm=a.near_mm,
                              release_xy_mm=a.release_xy_mm, empty_s=a.empty_s, timeout_s=a.trigger_timeout_s,
                              rest_mm=a.rest_mm)


class Watch:
    """run_episode's stop hook: samples the trigger state at 10 Hz from the policy's t0."""

    def __init__(self, arm, grip, world, cfg):
        self.arm, self.grip, self.world, self.cfg = arm, grip, world, cfg
        self.hist, self.task, self.t0 = [], None, None

    def begin(self, t0):
        self.t0, self.hist = t0, []
        self.task = asyncio.create_task(self._sample())

    async def _read(self) -> dict:
        p = await self.arm.get_end_position()
        pose = {"x": p.x, "y": p.y, "z": p.z, "o_x": p.o_x, "o_y": p.o_y, "o_z": p.o_z, "theta": p.theta}
        r = await self.world.do_command({"command": "get_prop_pose", "prop": "block"})
        g = (await self.grip.do_command({"command": "position"}))["input"]
        hs = await self.grip.is_holding_something()
        return {"t": time.monotonic() - self.t0, "tcp": bc_pick.tcp_world(pose),
                "block": tuple(v * 1000 for v in r["pose"]["position"]), "grip_cmd": int(g > 0.5),
                "holding": bool(hs.meta.get("firm_grasp"))}

    async def _sample(self):
        while True:
            t = time.monotonic()
            try:
                self.hist.append(await self._read())
            except Exception:  # noqa: BLE001 - a missed read is a missed sample
                pass
            await asyncio.sleep(max(0.0, 0.1 - (time.monotonic() - t)))

    def check(self):
        return dagger_trigger.check(self.hist[-1], self.hist[:-1], self.cfg) if self.hist else None

    def end(self):
        if self.task is not None:
            self.task.cancel()
            self.task = None


def parse_args(argv=None):
    ap = policy_run.build_parser()
    ap.add_argument("--name", default="dagger1", help="episode tag prefix: episode:<name>-<seed>")
    ap.add_argument("--round", default="r1", help="dagger:<round> tag")
    ap.add_argument("--max-kept", type=int, default=80)
    ap.add_argument("--capture-sensor", default="demo-capture")
    ap.add_argument("--rate-hz", type=float, default=10.0)
    d = dagger_trigger.Cfg()
    ap.add_argument("--stall-s", type=float, default=d.stall_s)
    ap.add_argument("--stall-mm", type=float, default=d.stall_mm)
    ap.add_argument("--near-mm", type=float, default=d.near_mm)
    ap.add_argument("--release-xy-mm", type=float, default=d.release_xy_mm)
    ap.add_argument("--empty-s", type=float, default=d.empty_s)
    ap.add_argument("--trigger-timeout-s", type=float, default=d.timeout_s)
    ap.add_argument("--rest-mm", type=float, default=d.rest_mm)
    ap.set_defaults(seeds="3000-3299", timeout_s=40.0, out=".runs/policy-dagger/dagger1.jsonl")
    return policy_run.parse_args(argv, parser=ap)


async def main_async(a):
    servo_r, obs_r, side_r = await bc_pick.connect(a.address, a.insecure, 3)
    try:
        c = await policy_run.build_context(a, servo_r, obs_r, side_r)
        ep = Episode((servo_r, side_r), a)
        watch = Watch(c["side_arm"], c["side_grip"], c["world"], cfg_from_args(a))
        os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
        n = kept = 0
        kinds, outcomes = collections.Counter(), []
        for seed in policy_run.seed_range(a.seeds):
            row = await policy_run.run_episode(a, c, seed, stop=watch)
            rec = log_row(row)
            if row["outcome"] == "trigger":  # a trigger with a failed close leaves the session live: no takeover
                kinds[row["trigger"]] += 1
                tags = takeover_tags(a.name, a.round, seed, row["trigger"])
                info = await ep.takeover(seed, tags)
                rec.update(info, tags=tags)
            n += 1
            kept += bool(rec.get("kept"))
            outcomes.append(row["outcome"])
            with open(a.out, "a") as f:
                f.write(json.dumps(rec) + "\n")
            print(f"seed {seed} {rec['outcome']} trigger={rec['trigger']} kept={rec.get('kept')} "
                  f"reason={rec.get('reason', '')!r} [{kept} kept / {n}]", flush=True)
            if rec.get("abort"):
                print("aborting: stop_capture failed", flush=True)
                break
            if kept >= a.max_kept:
                break
            if policy_run.should_abort(outcomes):
                print("aborting: the cell is not usable (repeated setup/close failures)", flush=True)
                break
        print(json.dumps({"rollouts": n, "kept": kept, "triggers": dict(kinds), "log": a.out}))
    finally:
        for r in (servo_r, obs_r, side_r):
            await r.close()


if __name__ == "__main__":
    asyncio.run(main_async(parse_args()))
```

- [ ] **Step 4: Run to verify it passes**

Run: `uv run clients/python/test_dagger_run.py`
Expected: `ok`. Also `uv run clients/python/dagger_run.py --help` prints usage.

- [ ] **Step 5: Commit**

```bash
git add clients/python/dagger_run.py clients/python/test_dagger_run.py
git commit -m "dagger_run: policy rollouts with expert takeover and capture of the correction"
```

---

### Task 6: `--pretrained viam:` in viam-lerobot-training

**Repo:** `~/src/viam-lerobot-training`. Start a branch from PR #2's branch:
`git switch -c pretrained-viam-package early-stop-best` (the checkout is on `lora-packaging-recipes` with a clean
tree; check `git status` first and stop if it is not clean). Do not push.

**Files:**
- Create: `lerobot-training/model/viam_package.py`
- Modify: `lerobot-training/model/training.py` (`main`, right before `check_checkpoint(args)`, ~line 108)
- Modify: `lerobot-training/model/args.py` (the `--pretrained` help text)
- Modify: `README.md` (document the `viam:` form next to `--pretrained`)
- Test: `tests/test_viam_package.py`

- [ ] **Step 1: Write the failing test** `tests/test_viam_package.py`

```python
import io
import json
import tarfile

import pytest

from model import viam_package as vp


def make_pkg(path, files):
    with tarfile.open(path, "w:gz") as tf:
        for name, text in files.items():
            data = text.encode()
            info = tarfile.TarInfo(name)
            info.size = len(data)
            tf.addfile(info, io.BytesIO(data))
    return path.as_uri()


def test_parse_ref():
    assert vp.parse_ref("viam:org-1/diff-ik-pick-smolvla@4") == ("org-1", "diff-ik-pick-smolvla", "4")
    for bad in ("viam:org-1/name", "viam:name@4", "viam:/name@4", "viam:org/@4", "viam:org/name@"):
        with pytest.raises(SystemExit):
            vp.parse_ref(bad)


def test_non_viam_values_pass_through(tmp_path):
    assert vp.resolve_pretrained(None, tmp_path) is None
    assert vp.resolve_pretrained("lerobot/smolvla_base", tmp_path) == "lerobot/smolvla_base"


def test_resolves_shallowest_config(tmp_path):
    url = make_pkg(tmp_path / "p.tar.gz", {"config.json": json.dumps({"type": "smolvla"}),
                                           "tokenizer/config.json": "{}", "model.safetensors": "x"})
    seen = []
    path = vp.resolve_pretrained("viam:org/name@4", tmp_path / "dl",
                                 url_for=lambda o, n, v: seen.append((o, n, v)) or url)
    assert seen == [("org", "name", "4")]
    assert json.loads((tmp_path / "dl" / "pkg" / "config.json").read_text())["type"] == "smolvla"
    assert path == str(tmp_path / "dl" / "pkg")


def test_package_without_config(tmp_path):
    url = make_pkg(tmp_path / "p.tar.gz", {"model.safetensors": "x"})
    with pytest.raises(SystemExit, match="config.json"):
        vp.resolve_pretrained("viam:org/name@4", tmp_path / "dl", url_for=lambda *_: url)


def test_needs_job_credentials(monkeypatch):
    monkeypatch.delenv("API_KEY", raising=False)
    monkeypatch.delenv("API_KEY_ID", raising=False)
    with pytest.raises(SystemExit, match="API_KEY"):
        vp.package_url("org", "name", "4")


def test_main_resolves_before_check_checkpoint(monkeypatch):
    from model import training
    order = []
    monkeypatch.setattr(training, "resolve_pretrained", lambda v, d: order.append("resolve") or "/tmp/base")
    monkeypatch.setattr(training, "check_checkpoint", lambda a: order.append(("check", a.pretrained)) or 1 / 0)
    monkeypatch.setattr(training.cloud, "check_components", lambda a: None)
    monkeypatch.setattr(training.cloud, "gpu_count_and_diagnostics", lambda: 0)
    monkeypatch.setattr(training.cloud, "choose_vcodec", lambda v: "h264")
    monkeypatch.setattr(training, "versions", lambda: {})
    argv = ["--binary_data_file=b", "--tabular_data_file=t", "--sequences_file=s", "--model_output_directory=/tmp/o",
            "--arm=arm", "--cameras=cam", "--policy=smolvla", "--pretrained=viam:org/name@4"]
    with pytest.raises(ZeroDivisionError):
        training.main(argv)
    assert order == ["resolve", ("check", "/tmp/base")]
```

Note: the last test stubs whatever `main` calls before `check_checkpoint`. Read `main` and adjust the stubs to match
(names above are from `training.py` lines 94–108); keep the assertion on the order.

- [ ] **Step 2: Run to verify it fails**

Run: `.venv/bin/python -m pytest tests/test_viam_package.py -q`
Expected: `ModuleNotFoundError: No module named 'model.viam_package'` (collection error).

- [ ] **Step 3: Implement** `lerobot-training/model/viam_package.py`

```python
"""--pretrained viam:<org_id>/<model_name>@<version>: a Viam registry ML model package as the base checkpoint.

Viam training jobs set API_KEY / API_KEY_ID: an org-scoped key with the submitter's permissions."""

from __future__ import annotations

import asyncio
import logging
import os
import tarfile
import urllib.request
from pathlib import Path

logger = logging.getLogger("lerobot-training")
PREFIX = "viam:"


def parse_ref(value: str) -> tuple[str, str, str]:
    org, slash, rest = value[len(PREFIX):].partition("/")
    name, at, version = rest.partition("@")
    if not (org and slash and name and at and version):
        raise SystemExit(f"--pretrained {value!r}: expected viam:<org_id>/<model_name>@<version>")
    return org, name, version


async def _package_url(org: str, name: str, version: str, key: str, key_id: str) -> str:
    from viam.app.viam_client import ViamClient
    from viam.proto.app.packages import GetPackageRequest, PackageServiceStub, PackageType
    from viam.rpc.dial import DialOptions

    client = await ViamClient.create_from_dial_options(DialOptions.with_api_key(key, key_id))
    try:
        stub = PackageServiceStub(client._channel)  # viam-sdk 0.85 has no packages client
        r = await stub.GetPackage(GetPackageRequest(id=f"{org}/{name}", version=version,
                                                    type=PackageType.PACKAGE_TYPE_ML_MODEL, include_url=True),
                                  metadata=client._metadata)
        return r.package.url
    finally:
        client.close()


def package_url(org: str, name: str, version: str) -> str:
    key, key_id = os.environ.get("API_KEY"), os.environ.get("API_KEY_ID")
    if not (key and key_id):
        raise SystemExit("--pretrained viam:... needs API_KEY and API_KEY_ID (Viam training jobs set them)")
    try:
        return asyncio.run(_package_url(org, name, version, key, key_id))
    except Exception as exc:  # noqa: BLE001 - any lookup failure stops the job before conversion
        raise SystemExit(f"cannot look up package {org}/{name}@{version}: {exc}") from exc


def checkpoint_dir(root: Path) -> Path:
    """The shallowest folder holding config.json (a tokenizer/ folder can hold its own)."""
    hits = sorted(root.rglob("config.json"), key=lambda p: len(p.relative_to(root).parts))
    if not hits:
        raise SystemExit(f"the package has no config.json under {root}")
    return hits[0].parent


def resolve_pretrained(value: str | None, dest: Path, url_for=package_url) -> str | None:
    """A viam: reference becomes a local checkpoint folder; any other value is returned unchanged."""
    if not value or not value.startswith(PREFIX):
        return value
    org, name, version = parse_ref(value)
    url = url_for(org, name, version)
    dest.mkdir(parents=True, exist_ok=True)
    archive = dest / "package.tar.gz"
    urllib.request.urlretrieve(url, archive)
    with tarfile.open(archive) as tf:
        tf.extractall(dest / "pkg", filter="data")
    path = checkpoint_dir(dest / "pkg")
    logger.info("base checkpoint %s from %s", path, value)
    return str(path)
```

Check `viam.proto.app.packages` exports `PackageType` (else import the enum from where the SDK defines it, e.g.
`PACKAGE_TYPE_ML_MODEL` at module level) and that the container's Python has `tarfile` `filter=` (3.11.4+). Report
if not.

In `training.py`: `from model.viam_package import resolve_pretrained`, and in `main` immediately before
`check_checkpoint(args)`:

```python
    # A registry checkpoint is downloaded first, so the flags and the pre-flight see a local path.
    args.pretrained = resolve_pretrained(args.pretrained, Path(tempfile.mkdtemp(prefix="base-")))
```

`--pretrained` help: `"override the curated base checkpoint: a hub id, a local path, or viam:<org_id>/<model>@<version> (a Viam registry ML model, downloaded with the job's API key)"`.
README: one short paragraph under the `--pretrained` docs with the fine-tune-from-a-registry-model example.

- [ ] **Step 4: Run to verify it passes**

Run: `.venv/bin/python -m pytest -m "not lerobot and not gpu" -q`
Expected: all pass, including the 6 new tests.

- [ ] **Step 5: Commit** (in `~/src/viam-lerobot-training`, on `pretrained-viam-package`)

```bash
git add lerobot-training/model/viam_package.py lerobot-training/model/training.py lerobot-training/model/args.py README.md tests/test_viam_package.py
git commit -m "--pretrained viam:<org>/<model>@<version>: fine-tune from a registry model"
```

---

### Task 7 ★: Sim smoke and a 5-rollout dry run

- [ ] **Step 0:** `uv run clients/python/policy_run.py --address <addr> --once` (checks `build_context` live).
- [ ] **Step 1:** Run all diff-ik checks: `for t in clients/python/test_bc_pick.py clients/python/test_dagger_trigger.py clients/python/test_policy_run.py clients/python/test_policy_run_stop.py clients/python/test_dagger_run.py clients/python/test_policy_exec.py clients/python/test_policy_layout.py; do uv run $t; done`: all `ok`.
- [ ] **Step 2: Expert regression.** `uv run clients/python/expert_pick.py run --address <addr> --seeds 1003-1005` (no
  capture): all `kept: true`.
- [ ] **Step 3: Forced takeovers.** A scratchpad script: for seed 3000, (a) reset, place the block, open a servo
  session, move the flange 60 mm sideways and 40 mm above grasp height with a 5° tilt, `servo_stop`, then
  `Episode.takeover(seed)` (no tags); (b) reset, run `_start_session` + the start move + `_pick`, `servo_stop`, then
  `takeover(seed)`. Both must return `kept: true`, with `holding` false and true respectively.
- [ ] **Step 4: Dry run with capture.** For each of seeds 9101, 9110, 9111, 9115, 9124 (`--seeds` takes N or LO-HI,
  so one call per seed): `uv run clients/python/dagger_run.py --address <addr> --seeds <seed> --name daggerdry --round dry --out .runs/policy-dagger/dry.jsonl`.
  Eval seeds are fine here: this data is thrown away. Expect at least 2 triggers.
- [ ] **Step 5: Convert check.** `expert_pick.py add --log .runs/policy-dagger/dry.jsonl --dataset diff-ik-pick-dagger-dry --part-id <part>`;
  `viam dataset export` it; convert with the v3 settings (scratchpad `convert_v3.sh`: `--camera scene-cam --camera wrist-cam --arm arm --grippers gripper --gripper-key input --action-space ee --image-size 448 --fps 10`);
  run `dev/check_policy_dataset.py`. Check: one episode per kept correction, task text is the instruction, the first
  frame's state equals the takeover pose (not the start pose), lengths 3–15 s. Then delete the dry dataset.
- [ ] **Step 6:** Fix anything found (through an implementer for code), then continue.

### Task 8 ★: Collection

- [ ] **Step 1:** `uv run clients/python/dagger_run.py --address <addr> --checkpoint-label smolvla-v4` in the
  background (defaults: seeds 3000–3299, max 80 kept), watched with Monitor on the log's summary lines.
- [ ] **Step 2:** On finish: report rollouts, policy successes, triggers by kind, kept corrections, failed takeovers
  and their reasons.

### Task 9 ★: Datasets

- [ ] **Step 1:** `expert_pick.py add --log .runs/policy-dagger/dagger1.jsonl --dataset diff-ik-pick-dagger-r1 --part-id <part>`;
  `missing` must be empty.
- [ ] **Step 2:** Create the sequence dataset `diff-ik-pick-v3-200-dagger1` and copy into it every sequence id of
  `6b0316ea770b27184e6eb985` and of `diff-ik-pick-dagger-r1` (`sequences_by_dataset_id` + `add_sequences_to_dataset`,
  scratchpad script). Check the count: 198 + kept.

### Task 10 ★: Local fine-tune check

- [ ] **Step 1:** `viam packages export --org-id=16518049-9dd3-479f-9644-c5d112aa42d8 --name=diff-ik-pick-smolvla --version=4 --type=ml_model --destination=<scratch>/v4pkg`, unpack.
- [ ] **Step 2:** Build a small sequence dataset `diff-ik-pick-check` from 5 v3-200 sequences plus 5 corrections,
  `viam dataset export` it to the scratchpad; run the training script locally
  (branch `pretrained-viam-package`, `.venv`) with the cloud args plus `--pretrained=<unpacked v4> --steps=2 --batch_size=2 --extra "--policy.device cpu --num_workers 0"`
  (or `mps`). Check:
  (a) the step-1 loss is well below the base model's step-1 loss on the same data (run once more with no
  `--pretrained` to get it) and near v4's final training loss;
  (b) the packaged checkpoint's normalization stats match the new dataset's stats, not v4's. If they come from v4's
  saved processor files instead, report it at the Task 11 check-in with the stat difference: the fix is either to
  accept v4's stats (the distributions nearly match) or to drop the processor files from the unpacked base before
  training;
  (c) the scheduler logs its auto-scaling for the step count.
- [ ] **Step 3:** If an org-readable key is at hand, call `viam_package.package_url` for
  `16518049-9dd3-479f-9644-c5d112aa42d8/diff-ik-pick-smolvla@4` locally. Otherwise the first cloud job is the test (it
  fails before the conversion if the lookup fails).

### Task 11 ★ [check-in]: Script version and the two jobs

- [ ] **Step 0:** Confirm that script version 4 was uploaded from `early-stop-best` (the `lora-packaging-recipes`
  commits stayed local), so version 5 from `pretrained-viam-package` rolls nothing back.
- [ ] **Step 1:** Show the user: the branch's diff, Task 10's results, and the two submit commands. Ask to (a) upload
  the branch as `lerobot-training` version 5 (`viam training-script upload`, as for versions 1–4, per the repo
  README), and (b) submit both jobs. Opening a PR for the branch is a separate question (it stacks on #2).
- [ ] **Step 2:** On approval, submit (the v4 command with these changes):

```bash
viam train submit custom from-registry --dataset-id=<v3-200-dagger1 id> --org-id=16518049-9dd3-479f-9644-c5d112aa42d8 --version=5 --script-name=16518049-9dd3-479f-9644-c5d112aa42d8:lerobot-training --container-id=2e3137a1-6182-461d-ae18-37f200c62696 --model-name=diff-ik-pick-smolvla-dagger --model-version=1 --args=arm=arm,grippers=gripper,gripper_key=input,action_space=ee,cameras="'scene-cam wrist-cam'",image_size=448,fps=10,policy=smolvla,steps=10000,batch_size=32,pretrained=viam:16518049-9dd3-479f-9644-c5d112aa42d8/diff-ik-pick-smolvla@4,extra="'--policy.n_action_steps 20'"
viam train submit custom from-registry --dataset-id=6b0316ea770b27184e6eb985 ... --model-name=diff-ik-pick-smolvla-ctrl --model-version=1 ... (identical args)
```

- [ ] **Step 3:** Watch both by their logs: the "base checkpoint ... from viam:" line, the auto-scaling line, the step
  rate, then the packaging. Judge by logs, not the job status.

### Task 12 ★: Gate and results

- [ ] **Step 1:** For each model: export the package, apply the vla compat strip (as for v4's `pkg-compat`), point
  vla-policy at it (`part_cfg.py`), then
  `uv run clients/python/policy_run.py --address <addr> --mode module --cadence sync --seeds 9100-9129 --timeout-s 30 --checkpoint-label <label> --out .runs/eval/<label>-gate.jsonl`.
- [ ] **Step 2:** `policy_report.py` on both plus `smolvla-v4-gate`; the per-seed table for 9101, 9110, 9111, 9115,
  9124.
- [ ] **Step 3:** Write `docs/results/2026-11-policy-dagger.md`: collection stats (rollouts, triggers by kind, kept),
  the two jobs, the gate table against v4, the verdict (A − B ≥ 4 successes = DAgger helped; otherwise negative), and
  next steps. Commit.
- [ ] **Step 4:** Update the memory note `diffik-policy-followups.md` (DAgger round 1 done, result) and report to the
  user. Ask before stopping the machine.
