# Policy ACT and the Deployment Comparison Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Train ACT on a fixed v3 pick-to-bin dataset (locally, then with a Viam training job), gate it at ≥70% success through `servo_trajectory`, then compare three deployment modes under sync and async inference.

**Architecture:** Two tracks. Track 2 (local) fixes the expert and data, builds the runner (`policy_run.py` + `policy_exec.py` + `policy_report.py`) and trains ACT on MPS. Track 1 (upstream) carries the converter's `ee` mode, the training script's `action_space`, and the LeRobot version alignment into viam-labs repos so a cloud job can train the same thing. Two small diff-ik Go fixes (median-lag auto-lead, an honest `servo_trajectory` reply) ship as v0.4.1.

**Tech Stack:** Go (diff-ik module), Python 3.11+ clients run with `uv run` (viam-sdk, numpy, scipy, pillow), LeRobot `02655292` in a Python 3.12 venv, Viam CLI (datasets, training), mujoco-sim 0.20.0 (local build with patches) on machine `diff-ik-policy-sim`.

**Spec:** `docs/superpowers/specs/2026-11-21-diff-ik-policy-act-design.md`. Read it before any task.

---

## Ground rules for every task

- Branch: `diff-ik-policy-sim` in `/Users/nick.hehr/src/diff-ik`. Commit each task. End commit messages with the attribution lines your harness gives you.
- Python checks are plain `assert` scripts with a PEP 723 header, run as `uv run clients/python/test_<name>.py`, printing `ok`. No pytest, no fixtures (repo style; see `clients/python/test_bc_pick.py`).
- Go: `go test -race ./...` passes and `gofmt -l .` prints nothing (ignore `.runs/`).
- Never print, log or commit credentials. Machine credentials live in the controller's scratchpad; tasks that need the machine are run by the controller, not by implementer subagents, unless the controller hands over a command.
- Never push, open PRs, upload, register, submit training jobs, tag or release. Those steps are marked **CONTROLLER + USER OK** and are done by the controller after the user approves.
- Machine address for clients: the cloud address `diff-ik-policy-sim-main.sr89pn2pma.viam.cloud` with `VIAM_API_KEY` / `VIAM_API_KEY_ID` in the environment (the local insecure dial fails; see the sub-project 1 results).

## File structure

| File | Responsibility |
|---|---|
| `loop.go`, `session.go`, `module.go` (modify) | median lag sample ring and `LagMedianMS`; auto-lead uses it; `servo_trajectory` reply computed from the new chunk |
| `trajectory_test.go`, `loop_test.go` (modify) | tests for both Go fixes |
| `devrel_diff-ik_servo.md`, `README.md` (modify) | drop "send `lead_ms` explicitly", document `lag_median_ms` |
| `clients/python/bc_pick.py` (modify) | tilts in `sample_scene`; 6-tuple poses; `minjerk_segment` and `pose_map` handle tilt |
| `clients/python/expert_pick.py` (modify) | `layout(seed)` / `place()` / `start_pose()` extracted; tilted transit waypoints |
| `clients/python/playback.py` (create) | `blended()`: the client crossfade selection shared by `policy_stream.py` and `policy_exec.py` |
| `clients/python/policy_exec.py` (create) | pure helpers (row survival, gripper events, jerk, success, chunk sampling) and the three executors |
| `clients/python/policy_run.py` (rewrite) | episode loop, cadence, metrics, JSONL out; keeps `--once` |
| `clients/python/policy_report.py` (create) | JSONL → markdown tables |
| `clients/python/test_playback.py`, `test_policy_exec.py`, `test_policy_report.py` (create), `test_bc_pick.py` (modify) | checks |
| `dev/bc-pick-mujoco.json` (modify, late) | `vla-policy` on the registry package |
| `docs/results/2026-11-policy-act.md` (create) | results |
| `~/src/viam-sequence-to-lerobot` | rebase branch for the PR |
| `~/src/viam-lerobot-training` | `--action_space`, `EndPosition` preflight, `CONVERTER_REF` |
| `~/src/viam-vla-inference-service` | LeRobot bump to `02655292` |

---

## Task 1: Median-lag auto-lead (Go)

**Files:** Modify `loop.go`; Test `trajectory_test.go`.

Context: `updateLag` (loop.go ~1031) finds `bestD`, the best-fit lag in ticks, then moves `lagTicks` (jumps up, eases down). `expected()` uses `lagTicks` and must keep doing so. `advance()` (loop.go ~566) sets `lead = lagTicks / RateHz` when `tr.autoLead`. Change only the auto-lead source.

- [ ] **Step 1: Write the failing tests** (append to `trajectory_test.go`; reuse its helpers `newTestLoop`, `runTicks`, `offsetTarget`, `home` as the existing `TestLoopFollowsTrajectory` does):

```go
// The auto-lead reads the median of recent best-fit lag samples, not the jump-up lagTicks; expected() keeps lagTicks.
func TestAutoLeadUsesMedianLag(t *testing.T) {
	l, _, m := newTestLoop(t, home, LoopConfig{})
	l.SetTarget(offsetTarget(t, m, home, r3.Vector{}, 0))
	runTicks(t, l, 2)
	l.lagTicks = 9                           // a spike raised the jump-up estimate
	l.lagSamples = []float64{3, 3, 4, 3, 9} // but most fits say 3
	p0, _, _ := FK(m, home)
	_ = p0
	tr := newTrajectory(l.now(), 100*time.Millisecond, []spatialmath.Pose{
		offsetTarget(t, m, home, r3.Vector{}, 0), offsetTarget(t, m, home, r3.Vector{X: 10}, 0)})
	tr.autoLead = true
	l.SetTrajectory(tr)
	runTicks(t, l, 1)
	want := 3 / l.cfg.RateHz * 1000
	if st := l.Status(); math.Abs(st.TrajLeadMS-want) > 1e-6 || math.Abs(st.LagMedianMS-want) > 1e-6 {
		t.Fatalf("lead %v ms, lag median %v ms, want %v", st.TrajLeadMS, st.LagMedianMS, want)
	}
	l.sent = [][]float64{{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}}
	if got := l.expected(); got[0] != 1 { // round(lagTicks = 9) ticks before the newest
		t.Fatalf("expected() used %v, want the command 9 ticks back", got)
	}
}

func TestMedian(t *testing.T) {
	for _, c := range []struct {
		in   []float64
		want float64
	}{{[]float64{3}, 3}, {[]float64{5, 1, 3}, 3}, {[]float64{4, 1, 3, 2}, 2.5}} {
		if got := median(c.in); got != c.want {
			t.Fatalf("median(%v) = %v, want %v", c.in, got, c.want)
		}
	}
}
```

Note: `runTicks` may itself call `updateLag` and append samples. If it does in this test (the target barely moves, so `updateLag` should return early), set `lagSamples` and `lagTicks` after the last `runTicks` before the assertion tick, or assert on what the ring holds. Adjust the test, not the semantics.

- [ ] **Step 2: Run** `go test -run 'TestAutoLeadUsesMedianLag|TestMedian' ./...` — expect compile failure (`lagSamples`, `LagMedianMS`, `median` undefined).

- [ ] **Step 3: Implement.**
  - Loop field next to `lagTicks`: `lagSamples []float64 // recent best-fit lags (ticks), newest last, at most lagSampleN; loop goroutine only`, and `const lagSampleN = 25` next to `lagAlpha`.
  - In `updateLag`, after computing `bestD` (past the early returns), and only when `moving` is true: `l.lagSamples = append(l.lagSamples, float64(bestD))`, trimmed to the last `lagSampleN`.
  - `func median(v []float64) float64` (sort a copy; even length → mean of the middle two).
  - `func (l *Loop) leadTicks() float64`: `median(l.lagSamples)` when non-empty, else `l.lagTicks`.
  - `advance()`: the auto-lead uses `l.leadTicks()` instead of `l.lagTicks`.
  - `Status.LagMedianMS float64 // median of recent best-fit lags, ms: the trajectory auto-lead` set where `LagMS` is set, from `leadTicks()`.
  - `module.go` `statusMap`: `out["lag_median_ms"] = st.LagMedianMS` next to `lag_ms`.

- [ ] **Step 4: Run** `go test -race ./... && gofmt -l . | grep -v .runs` — all pass, no files listed.

- [ ] **Step 5: Commit** `git add loop.go module.go trajectory_test.go && git commit -m "Trajectory auto-lead: median of recent lag fits (lag_median_ms)"`

## Task 2: `servo_trajectory` reply from the new chunk (Go)

**Files:** Modify `session.go` (`trajectory`, ~line 347), `devrel_diff-ik_servo.md`, `README.md`; Test: `session_test.go` (use its `newTestManager`, `mustStart` and `testSessionCfg` helpers).

- [ ] **Step 1: Failing test.** Start a manual pose session the way the existing session tests do, call `sess.trajectory(poses, 100ms, age=50ms, lead=&80ms, blend=0)` with 10 poses, and assert the returned status has `Traj == true`, `TrajLeadMS == 80`, `TrajRemainingMS` within 1 ms of `900 - 50 - 80 = 770`, `TrajEnded == false`. A second call with `lead == nil` must report `TrajLeadMS == loop.Status().LagMedianMS`.
- [ ] **Step 2: Run** it — fails (reply has `Traj == false`, lead 0).
- [ ] **Step 3: Implement** in `session.trajectory`, after `s.loop.SetTrajectory(tr)` and before returning: take `st := s.status()` (after `s.mu.Unlock()`, since `status()` locks), compute `lead := tr.lead` or, when `tr.autoLead`, `time.Duration(st.LagMedianMS * float64(time.Millisecond))`; set `st.Traj, st.TrajEnded = true, false`, `st.TrajLeadMS = float64(lead) / float64(time.Millisecond)`, `st.TrajRemainingMS = max(0, float64(tr.end().Sub(now.Add(lead))) / float64(time.Millisecond))`. No module.go change: the reply already reads these fields.
- [ ] **Step 4: Run** `go test -race ./...`.
- [ ] **Step 5: Docs.** In `devrel_diff-ik_servo.md` `servo_trajectory` table, `lead_ms` default text becomes: "the session's measured lag (median of recent fits, `lag_median_ms` in status)". Drop "The measured lag currently reads 40–60 ms high, so send it explicitly." Add `lag_median_ms` where status fields are listed. Returns line: "`lead_ms` and `remaining_ms` are the new chunk's." In `README.md`, if it mentions sending `lead_ms` explicitly or the high lag reading, update it the same way.
- [ ] **Step 6: Commit** `git commit -am "servo_trajectory reply reports the new chunk's lead and remaining"`

## Task 3: Tilted poses in `bc_pick.py`

**Files:** Modify `clients/python/bc_pick.py`, `clients/python/test_bc_pick.py`.

- [ ] **Step 1: Failing checks.** Change `test_bc_pick.py`'s header to `# dependencies = ["viam-sdk", "numpy", "scipy"]` and add:

```python
from bc_pick import TILT_MAX_DEG, tilt_vector  # noqa: E402  (add to the import list above)

# tilts: drawn after the v1 draws, so old seeds keep their scene
import random
r = random.Random(1003)
(x0, x1), (y0, y1) = BLOCK_AREA
v1 = {"block_xy": (r.uniform(x0, x1), r.uniform(y0, y1)), "block_yaw_deg": r.uniform(-45, 45),
      "nudge_xyz": tuple(r.uniform(-30, 30) for _ in range(3)), "nudge_yaw_deg": r.uniform(-10, 10),
      "speed": r.uniform(0.8, 1.2)}
sc = sample_scene(1003)
assert {k: sc[k] for k in v1} == v1
assert set(sc["tilts"]) == {"start", "pregrasp", "lift"}
for s in range(200):
    for tx, ty in sample_scene(s)["tilts"].values():
        assert math.hypot(tx, ty) <= TILT_MAX_DEG + 1e-9
assert sample_scene(7)["tilts"] == sample_scene(7)["tilts"]

# 6-tuple poses: (x, y, z, theta, tx, ty); untilted matches the old tool-down map exactly
assert pose_map((1, 2, 3, 45)) == {"x": 1, "y": 2, "z": 3, "o_x": 0, "o_y": 0, "o_z": -1, "theta": 45}
p = pose_map((1, 2, 3, 45, 0.0, 0.0))
assert abs(p["o_z"] + 1) < 1e-9 and abs(p["theta"] - 45) < 1e-6
p = pose_map((0, 0, 0, 180, 4.0, 0.0))  # 4 degrees about world x
assert abs(math.degrees(math.acos(-p["o_z"])) - 4.0) < 1e-6
assert tilt_vector(3.0, 90.0) == (0.0, 3.0) or max(abs(a - b) for a, b in zip(tilt_vector(3.0, 90.0), (0.0, 3.0))) < 1e-9

# segments carry tilt: exact ends, min-jerk between
seg = minjerk_segment((0, 0, 0, 180, 0.0, 0.0), (100, 0, 0, 180, 4.0, -2.0), 1.0, 30)
assert seg[0] == (0, 0, 0, 180, 0.0, 0.0) and seg[-1] == (100, 0, 0, 180, 4.0, -2.0) and len(seg) == 31
assert abs(seg[15][4] - 2.0) < 0.2  # halfway through min-jerk is halfway
assert minjerk_segment((0, 0, 0, 180), (1, 0, 0, 180), 1.0, 10)[-1] == (1, 0, 0, 180)  # 4-tuples still work
```

- [ ] **Step 2: Run** `uv run clients/python/test_bc_pick.py` — ImportError.
- [ ] **Step 3: Implement** in `bc_pick.py`:

```python
TILT_MAX_DEG = 4.0
TILTED = ("start", "pregrasp", "lift")  # grasp, above-bin (release) and retreat stay vertical


def tilt_vector(mag_deg: float, azimuth_deg: float) -> tuple:
    a = math.radians(azimuth_deg)
    return (mag_deg * math.cos(a), mag_deg * math.sin(a))
```

  - `sample_scene`: build the existing dict first (same draw order), then `d["tilts"] = {k: tilt_vector(r.uniform(0, TILT_MAX_DEG), r.uniform(0, 360)) for k in TILTED}`.
  - `minjerk_segment(a, b, ...)`: accept 4- or 6-tuples at either end, padding a 4-tuple with `0.0, 0.0`. The output has 6 elements if **either** end has 6 (so a tilt is never dropped when `cur` is a 4-tuple and the goal is a 6-tuple, or the reverse), else 4. Interpolate tx, ty linearly in `m` like x, y, z; `out[-1]` is `b` padded to the output length (with theta `a[3] + dth`, as now). Add a check: `minjerk_segment((0,0,0,180), (0,0,0,180,4.0,0.0), 1.0, 10)[-1] == (0,0,0,180,4.0,0.0)` and `len(minjerk_segment((0,0,0,180,4.0,0.0), (0,0,0,180), 1.0, 10)[0]) == 6`.
  - `pose_map(p)`: for a 4-tuple or a zero tilt return the exact tool-down map (keeps the existing assert). Otherwise build `R = Rotation.from_rotvec(np.radians([tx, ty, 0])) * down(theta)` where `down(theta)` is `policy_layout._rotation({"o_x": 0, "o_y": 0, "o_z": -1, "theta": theta})` (rename it `rotation` in `policy_layout.py`, keep `_rotation = rotation` so nothing else breaks), then convert to an orientation vector exactly as `policy_layout.chunk_to_poses` does (`RotationMatrix(...).to_quaternion().to_orientation_vector()`), theta in degrees. Import numpy/scipy/policy_layout inside `pose_map` so `bc_pick` stays importable without them for callers that never tilt.
- [ ] **Step 4: Run** `uv run clients/python/test_bc_pick.py` and `uv run clients/python/test_policy_layout.py` — both `ok`.
- [ ] **Step 5: Commit** `git commit -am "bc_pick: seed-drawn tool tilts and 6-tuple poses"`

## Task 4: Expert uses tilts; layout extracted

**Files:** Modify `clients/python/expert_pick.py`; Test: extend `clients/python/test_bc_pick.py` (pure parts only).

- [ ] **Step 1: Extract pure helpers into `bc_pick.py`** (so the runner can import them without the expert's SDK imports), with checks in `test_bc_pick.py`:

```python
def block_pose(seed: int) -> tuple:
    """World position (m) and orientation (wxyz) to set_prop_pose the block for this seed."""
    sc = sample_scene(seed)
    (bx, by), yaw = sc["block_xy"], math.radians(sc["block_yaw_deg"])
    return [bx / 1000, by / 1000, (TABLE_TOP + BLOCK_MM / 2) / 1000], [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]


def start_pose(p0: dict, seed: int) -> tuple:
    """The episode's start (flange, arm frame, 6-tuple): the session's first pose plus the seed's nudge and start tilt."""
    sc = sample_scene(seed)
    n, (tx, ty) = sc["nudge_xyz"], sc["tilts"]["start"]
    return (p0["x"] + n[0], p0["y"] + n[1], p0["z"] + n[2], p0["theta"] + sc["nudge_yaw_deg"], tx, ty)
```

  Checks: `block_pose(1003)` matches what `Episode.run` computed inline before (position from `block_xy`, z = 0.32 m); `start_pose({"x":0,"y":0,"z":0,"theta":180}, 1003)[4:] == tuple(sample_scene(1003)["tilts"]["start"])`.

- [ ] **Step 2: Rewire `expert_pick.Episode`.**
  - Add `async def place(self, seed)`: world `reset`, then `set_prop_pose` with `C.block_pose(seed)`. `run()` calls it instead of the inline calls.
  - `start = C.start_pose(p0, seed)` replaces the inline nudge tuple.
  - `pre` gets the pregrasp tilt: `pre = C.flange_from_tcp(...) + (theta, *sc["tilts"]["pregrasp"])`; `grasp = ... + (theta, 0.0, 0.0)`; `lift = (cur[0], cur[1], cur[2] + C.LIFT, cur[3], *sc["tilts"]["lift"])`; `over_bin = ... + (cur[3], 0.0, 0.0)`; retreat keeps `0.0, 0.0`.
  - `move()` already passes tuples through `C.minjerk_segment` and `C.pose_map`; make its `dur` use only `cur[:4]`/`goal[:4]` as now. `hold()` passes the 6-tuple to `target()` unchanged.
  - The `StartPoseError` check on `p0` (the session's first pose, at home) is unchanged: home is tool-down.
  - Capture tag `expert:v1` → `expert:v2` (tilted demos).
  - PEP 723 header: `dependencies = ["viam-sdk", "numpy", "scipy"]` (viam-sdk alone has no scipy; a tilted `pose_map` would raise, and `run()`'s `except Exception` would silently discard every episode).
- [ ] **Step 3: Run** `uv run clients/python/test_bc_pick.py` and `python3 -c "import ast,sys; ast.parse(open('clients/python/expert_pick.py').read())"`.
- [ ] **Step 4: Commit** `git commit -am "Expert: tilted transit waypoints; block layout and start pose shared via bc_pick"`

## Task 5: v3 check (CONTROLLER, machine)

- [ ] Restart `diff-ik-policy-sim` (viam-server with its scratchpad `viam.json`; rebuild `bin/diff-ik` first with `make` so Tasks 1–2 are in), wait until ready.
- [ ] `uv run clients/python/expert_pick.py run --seeds 9100-9109 ...` (no capture flag; same flags as sub-project 1's dry runs; the log goes to `.runs/policy-sim/expert-bar.jsonl` automatically). Expect 10/10 kept.
- [ ] Capture 10 tilted episodes into a throwaway dataset `diff-ik-pick-v3-probe`, export, convert locally with the converter branch (`--action-space ee --grippers gripper --gripper-key input --image-size 224`, other flags as in the sub-project 1 results), and read `meta/stats.json`: r02/r12 std near 0.03; gripper column only 0/1. Grab a `scene-cam` frame of a tilted pre-grasp for the user.
- [ ] **USER CHECK-IN 1:** report 10/10, the stats, the image. Stop the machine while waiting. Delete the probe dataset after approval.

## Task 6: Capture v3 (CONTROLLER, machine)

- [ ] Training: seeds 2000–2049, then 2050–2099, each with `--capture v3` and then `add --dataset diff-ik-pick-v3`. Eval: seeds 9100–9129 with `--capture v3-eval`, `add --dataset diff-ik-pick-v3-eval`.
- [ ] Discarded seeds are recorded, not replaced (the dataset is "kept episodes of these seeds"). If fewer than 95 training episodes survive, report before continuing.
- [ ] Export both datasets and convert locally (same flags as Task 5) into `.runs/policy-act/v3` and `.runs/policy-act/v3-eval`; run `dev/check_policy_dataset.py` on each from the converter env (`cd ~/src/viam-sequence-to-lerobot && uv run python /Users/nick.hehr/src/diff-ik/dev/check_policy_dataset.py ...`, as in the sub-project 1 results). `add` also needs `--part-id` (and the other flags it used in sub-project 1).
- [ ] Append a "v3 dataset" section to `docs/results/2026-11-policy-act.md` (create it with a title and date): counts, discards and reasons, stats for r02/r12 and the gripper, dataset IDs. Commit.

## Task 7: Shared playback selection

**Files:** Create `clients/python/playback.py`, `clients/python/test_playback.py`; Modify `clients/python/policy_stream.py`.

- [ ] **Step 1: Failing check** `test_playback.py`:

```python
# /// script
# requires-python = ">=3.11"
# ///
"""Checks for playback.py. Run: uv run clients/python/test_playback.py"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from playback import blended  # noqa: E402

at = lambda c, t: c.get(t)  # noqa: E731  a chunk is {time: action}
mix = lambda a, b, w: a + (b - a) * w  # noqa: E731
cur, prev = {1: 10.0}, {1: 0.0}
assert blended(cur, None, 1, 0.5, at, mix) == 10.0          # no previous chunk
assert blended(cur, prev, 1, 0.25, at, mix) == 2.5           # crossfade
assert blended(cur, prev, 1, 1.0, at, mix) == 10.0           # fade done
assert blended(cur, {}, 1, 0.25, at, mix) == 10.0            # previous chunk used up
assert blended({}, prev, 1, 0.25, at, mix) is None           # current chunk used up
print("ok")
```

- [ ] **Step 2: Run** — ImportError.
- [ ] **Step 3: Implement** `playback.py`:

```python
"""The client-side chunk selection shared by policy_stream.py and policy_exec.py."""


def blended(cur, prev, t, w, at, mix):
    """cur's action at time t, crossfaded from prev's by w in [0, 1]; None when cur has none at t.
    at(chunk, t) samples a chunk (None when used up); mix(a, b, w) is (1 − w)·a + w·b."""
    target = at(cur, t)
    if prev is not None and target is not None and w < 1:
        old = at(prev, t)
        if old is not None:
            target = mix(old, target, w)
    return target
```

- [ ] **Step 4: Use it in `policy_stream.py`'s client loop**: replace the inline `target = chunk_action(...)` + blend block with
  `w = min(1.0, (tick - state["switch_t"]) * 1000 / a.blend_ms) if a.strategy == "blend" else 1.0` and
  `target = blended(cur, state["prev"], ta if aligned else tick, w, lambda c, t: chunk_action(c, t, a.policy_hz, aligned, c["t_arrive"]), blend)` when `cur is not None`. For naive and aligned the previous behaviour is unchanged because `w == 1.0`. (For blend the old code sampled `prev` aligned; with `aligned` true for blend that is the same.)
- [ ] **Step 5: Run** `uv run clients/python/test_playback.py && uv run clients/python/test_policy_stream.py`.
- [ ] **Step 6: Commit** `git add clients/python/playback.py clients/python/test_playback.py clients/python/policy_stream.py && git commit -m "playback.blended: the client crossfade, shared"`

## Task 8: Runner pure helpers

**Files:** Create `clients/python/policy_exec.py` (helpers section), `clients/python/test_policy_exec.py`.

- [ ] **Step 1: Failing checks** `test_policy_exec.py` (header deps `numpy`, `scipy`, `viam-sdk`):

```python
import math, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from policy_exec import first_live_row, gripper_events, jerk_rms, sample_chunk, succeeded  # noqa: E402

# rows still ahead of now: row k is due at t_obs + k*dt
assert first_live_row(t_obs=10.0, dt=0.1, now=10.0) == 0
assert first_live_row(10.0, 0.1, 10.25) == 3
assert first_live_row(10.0, 0.1, 10.3) == 3   # exactly due counts as live (epsilon)

# gripper: an event at each 0.5 crossing, from the last commanded state, fired lead earlier
ev = gripper_events([0.0, 0.1, 0.9, 1.0, 0.2], t_obs=5.0, dt=0.1, closed=False, lead=0.05)
assert [(round(t, 3), c) for t, c in ev] == [(5.15, "grab"), (5.35, "open")]
assert gripper_events([0.9, 0.9], 0.0, 0.1, closed=True, lead=0.0) == []
assert gripper_events([0.9], 0.0, 0.1, closed=False, lead=0.0) == [(0.0, "grab")]

# jerk: a straight line at constant speed has none; a cubic x = t^3 (mm, s) has 6 mm/s^3
line = [(i / 30, i * 2.0, 0, 0) for i in range(60)]
assert jerk_rms(line) < 1e-6
cubic = [(i / 30, (i / 30) ** 3, 0, 0) for i in range(60)]
assert abs(jerk_rms(cubic) - 6.0) < 1e-6
assert math.isnan(jerk_rms(line[:3]))

# success: in the bin, at rest, gripper open
from bc_pick import BIN_XY
inside = (BIN_XY[0], BIN_XY[1], 320.0)
assert succeeded(inside, (inside[0] + 0.5, inside[1], 320.0), gripper_input=0.0)
assert not succeeded(inside, inside, gripper_input=1.0)                      # still holding
assert not succeeded(inside, (inside[0] + 5, inside[1], 320.0), 0.0)         # still moving
assert not succeeded((0, 0, 320.0), (0, 0, 320.0), 0.0)                      # not in the bin

# chunk sampling: linear position, slerp orientation, clamped at the ends
ch = {"t_obs": 0.0, "dt": 0.1, "poses": [
    {"x": 0, "y": 0, "z": 0, "o_x": 0, "o_y": 0, "o_z": -1, "theta": 0},
    {"x": 10, "y": 0, "z": 0, "o_x": 0, "o_y": 0, "o_z": -1, "theta": 90}]}
p = sample_chunk(ch, 0.05)
assert abs(p["x"] - 5) < 1e-9 and abs(p["o_z"] + 1) < 1e-6 and abs(abs(p["theta"]) - 45) < 1e-4
assert sample_chunk(ch, -1)["x"] == 0 and sample_chunk(ch, 5)["x"] == 10
print("ok")
```

- [ ] **Step 2: Run** — ImportError.
- [ ] **Step 3: Implement** in `policy_exec.py` (pure section at the top; SDK imports only inside the executor functions/classes of Task 9):
  - `first_live_row(t_obs, dt, now)`: `max(0, math.ceil((now - t_obs) / dt - 1e-9))`.
  - `gripper_events(gcol, t_obs, dt, closed, lead)`: walk rows; `want = v > 0.5`; on `want != closed` append `(t_obs + k*dt - lead, "grab" if want else "open")` and flip `closed`.
  - `jerk_rms(samples)`: samples `(t, x, y, z)` mm; NaN if fewer than 4; third difference of each axis divided by the mean dt cubed; RMS of the 3-D norm.
  - `succeeded(b1, b2, gripper_input)`: `in_bin(b2) and math.dist(b1, b2) < 1.0 and gripper_input < 0.5` (import `in_bin` from `bc_pick`).
  - `sample_chunk(chunk, t)`: `s = (t - t_obs)/dt` clamped to `[0, n-1]`; `k = floor`, `f = s - k`; position linear between rows k and k+1; orientation `Slerp` (scipy) between `policy_layout.rotation(pose_k)` and `pose_{k+1}`, converted back to an orientation-vector pose map as `chunk_to_poses` does.
  - Also add `mix_pose(a, b, w)`: linear position, slerp orientation (same conversion) — used with `playback.blended` in Task 9.
- [ ] **Step 4: Run** `uv run clients/python/test_policy_exec.py` → `ok`.
- [ ] **Step 5: Commit** `git add clients/python/policy_exec.py clients/python/test_policy_exec.py && git commit -m "policy_exec: pure helpers (live rows, gripper events, jerk, success, chunk sampling)"`

## Task 9: Executors

**Files:** Modify `clients/python/policy_exec.py`; Test: extend `test_policy_exec.py` with a fake service (no machine).

Interface (all three):

```python
class Executor:
    async def start(self, seed: int) -> None: ...            # servo_start, then servo to bc_pick.start_pose(p0, seed) (p0 = the servo_start reply pose), settle 0.5 s
    async def play(self, chunk: dict) -> None: ...           # chunk = {"t_obs", "dt", "poses", "grip"}; returns quickly
    def done_at(self) -> float: ...                          # monotonic time the current chunk finishes playing
    async def close(self) -> dict: ...                       # stop; returns {"state", "peak_accel_ratio", "msgs"}
```

Shared: every executor counts arm command messages (`self.msgs`), owns a `GripperScheduler` (an asyncio task that fires `gripper_events` at their times via `gripper.grab()` / `gripper.open()`, replaced wholesale on each `play`, tracking the last commanded state), and starts the episode the same way: `servo_start` (frame `arm_origin`, mode `pose`, `gain` 8, `timeout_ms` 3000, `limits {"linear_mm_s": 300, "angular_deg_s": 90}`), `p0` = its reply pose, `goal = bc_pick.start_pose(p0, seed)`, stream `bc_pick.minjerk_segment((p0["x"], p0["y"], p0["z"], p0["theta"]), goal, dur, 30)` as `servo_target`s at 30 Hz (`dur` as in `expert_pick.Episode.move`), hold 0.5 s.

- [ ] **ModuleExec(svc, gripper, arm_name, lead_ms, blend_ms=200):** keeps the start session. `play`: one `servo_trajectory` with `poses`, `dt_ms = dt*1000`, `age_ms = (now - t_obs)*1000`, `blend_ms`, and `lead_ms` unless `lead_ms is None` (auto); gripper events with `lead = reply["lead_ms"]/1000`. `close`: `servo_status` (for `peak_accel_ratio`) then `servo_stop`.
- [ ] **ClientExec(svc, gripper, arm_name, lead_ms, blend_ms=200):** keeps the start session; a 30 Hz task sends `servo_target` with `playback.blended(cur, prev, now + lead, w, sample_chunk, mix_pose)`, `w = min(1, (now - switch_t)/blend_s)`; past the chunk's end it holds the last row (`sample_chunk` clamps). `play` swaps `prev, cur` and resets `switch_t`.
- [ ] **MoveExec(motion, gripper, arm_name, servo_svc):** `start` uses a servo session (as above) then `servo_stop`s it. A worker task runs a queue of rows: for each, `MotionClient.move(component_name=arm_name, destination=PoseInFrame(reference_frame="arm_origin", pose=...))` (the builtin service, `MotionClient.from_robot(robot, "builtin")`), and fires that row's gripper command when the row's `Move` starts. `play` replaces the queue with rows from `first_live_row(t_obs, dt, now)`. `done_at` is "queue empty" (sync waits for it). No `peak_accel_ratio`.
- [ ] **Fake-service check** in `test_policy_exec.py`: a fake `svc.do_command` that records commands, a fake gripper; drive `ModuleExec.play` and `ClientExec` for 0.3 s with `asyncio.run`, assert one `servo_trajectory` with the expected `age_ms` ≥ 0 and `dt_ms` 100, and that `ClientExec` sent about 9 `servo_target`s whose poses move from row 0 toward row 1. Keep it small.
- [ ] Run `uv run clients/python/test_policy_exec.py`; commit `git commit -am "policy_exec: module, client and move executors"`.

## Task 10: The runner

**Files:** Rewrite `clients/python/policy_run.py` (keep `--once` working: it becomes `--seeds <one seed> --mode module --cadence sync` with one chunk); keep `test_policy_layout.py` passing.

CLI:

```
uv run clients/python/policy_run.py --address <addr> --mode module|client|move --cadence sync|async \
    --replan-every 10 --lead-ms 100|auto --seeds 9100-9129 --out .runs/eval/<name>.jsonl [--timeout-s 30] [--checkpoint-label <text>]
```

- [ ] Episode (per seed): world `reset` + `set_prop_pose(bc_pick.block_pose(seed))`; `gripper.open()`; `await executor.start(seed)`; then loop:
  1. Observe: both cameras via `get_images`, resized to `specs["input_features"][key]` (existing code); `arm.get_end_position()`; gripper `{"command": "position"}` → **`input`** (not `position`) into `state_vector(pose, input)`; `t_obs = time.monotonic()`.
  2. Infer with a 5 s `asyncio.wait_for`; `chunk_to_poses(rows)` → poses + gripper column; record `row0_to_state_mm`.
  3. Before `play`, if a chunk is playing, record the seam gap: `math.dist` of `sample_chunk(old, now)` and `sample_chunk(new, now)` positions.
  4. `executor.play(chunk)`.
  5. Sync: sleep until `executor.done_at()`. Async: sleep until `t_obs + replan_every * dt`.
  6. Success is checked by a side task (so it doesn't delay the async cadence): every 0.3 s it reads `get_prop_pose` and the gripper `input` and calls `succeeded(previous_read, this_read, input)`; a success ends the episode. Timeout ends it at `--timeout-s`.
  - A side task samples `arm.get_end_position()` at 30 Hz into `(t, x, y, z)` for `jerk_rms`.
  - Failures per spec "Error handling" (`infer`, session state, `plan`, `gripper`) end the episode with that `outcome`; the run continues with the next seed. Abort the run only if the machine is unreachable or `specs` don't match (existing checks).
- [ ] JSONL row per episode: `seed, mode, cadence, replan_every, lead_ms, success, outcome, t_success_s, msgs_per_s, peak_accel_ratio, jerk_rms, row0_to_state_mm (list), seam_gap_mm (list), infer_s (list), chunks, checkpoint` (`checkpoint` from `--checkpoint-label`, free text).
- [ ] Check with a tiny pure test where practical (e.g. the JSONL row builder); the episode loop is exercised on the machine in Task 12.
- [ ] Commit `git commit -am "policy_run: closed-loop episodes with sync/async cadence and three executors"`.

## Task 11: Report

**Files:** Create `clients/python/policy_report.py`, `clients/python/test_policy_report.py`.

- [ ] `policy_report.py <jsonl>...` prints one markdown table, a row per (checkpoint, mode, cadence): episodes, successes and rate, median `t_success_s`, median `jerk_rms`, median and p95 of all `seam_gap_mm`, median first-chunk `row0_to_state_mm`, mean `msgs_per_s`, median `peak_accel_ratio`, outcome counts. Plus a paired table per seed (success per mode) for the comparison.
- [ ] Check: feed three hand-written rows, assert the rate, medians and outcome counts in the output text.
- [ ] Commit.

## Task 12: Local ACT and the local gate (CONTROLLER, machine + MPS)

- [ ] Train in the existing 3.12 venv at LeRobot `02655292` on `.runs/policy-act/v3`:
  `lerobot-train --policy.type=act --dataset.repo_id=local/v3 --dataset.root=.runs/policy-act/v3 --policy.chunk_size=20 --policy.n_action_steps=20 --batch_size=8 --steps=20000 --save_freq=5000 --policy.device=mps --output_dir=.runs/policy-act/act-local` (exact flags as the sub-project 1 smoke run, which worked; run in the background, ~50 min).
- [ ] For each checkpoint (5k, 10k, 15k, 20k): make the `-compat` copy (drop `dtype`, `optimizer_fused` from `config.json`), point `vla-policy`'s `model_path` at it (machine config), and run `policy_run.py --mode module --cadence sync --lead-ms 100 --seeds 9100-9109`. Pick the best by successes.
- [ ] Run the best on all 30 eval seeds. Note `lag_median_ms` from `servo_status` during the run; that value becomes `--lead-ms` for the comparison.
- [ ] **Copycat check:** if episodes fail by never closing or closing late (the gripper action equals the latched state on most frames), look at the gripper column against the jaw state in the JSONL/a trace first; a runner-side fix (close when the action crosses, regardless of state) comes before more data or steps.
- [ ] Results section "Local ACT" (training loss, per-checkpoint 10-seed successes, 30-seed result, failure outcomes). Commit.
- [ ] **USER CHECK-IN 3** with the result. Stop the machine while waiting.

## Task 13: Converter PR branch (`~/src/viam-sequence-to-lerobot`)

- [ ] `git fetch origin && git switch diff-ik-policy-sim && git rebase --onto origin/main 887e671` (887e671 is the last unsquashed `bimanual` commit; afterwards `git log origin/main..` must list only the two `ee` commits, `feat: --action-space ee ...` and `ee review follow-ups ...`). Resolve conflicts against the squashed version.
- [ ] Run the repo's tests (see its README / `pytest.ini`); all pass.
- [ ] Write the PR title/body to the controller's scratchpad (what `ee` is: absolute flange pose as xyz + the first two rotation-matrix rows + gripper, why; tests).
- [ ] **CONTROLLER + USER OK:** push the branch and open the PR.

## Task 14: Training script PR (`~/src/viam-lerobot-training`)

- [ ] On a new branch `action-space-ee`:
  - `lerobot-training/model/args.py`: `p.add_argument("--action_space", default="joints", choices=["joints", "ee"])` (the converter's names; `ConversionConfig` rejects others, convert.py ~87). Pass it into the conversion config in `lerobot-training/model/training.py` (~line 54), where `gripper_key` is passed.
  - `cloud.py` preflight (~line 146): when `action_space == "ee"`, require `(arm, "EndPosition")` instead of `(arm, "JointPositions")`.
  - `container/Dockerfile`: `ARG CONVERTER_REF=<merged commit>` — leave a `TODO-after-merge` placeholder only in the PR description, and set the SHA once the converter PR merges.
  - README section 2 table: add `action_space`.
  - Export-time strip of `dtype` / `optimizer_fused` from the saved `config.json` **only if** Task 15 is abandoned.
- [ ] Tests for the args and preflight (repo's existing test style, run per its README "Development").
- [ ] **CONTROLLER + USER OK:** push, open the PR.

## Task 15: vla:policy LeRobot bump (`~/src/viam-vla-inference-service`)

- [ ] Branch `lerobot-02655292`: bump the LeRobot pin to `02655292cdadd7dd2f8fc5daa31a7437b44354ee`; fix import/API drift; run its tests. **CONTROLLER (machine):** load the local Task 12 checkpoint **without** the compat strip through the module run from source (point `vla-policy` at it) and run 3 eval seeds.
- [ ] If the drift is large (more than an afternoon), stop, report, and switch to the export strip in Task 14.
- [ ] **CONTROLLER + USER OK:** push, open the PR. A module release after merge is also publishing: ask.

## Task 16: Container, script and cloud training (CONTROLLER + USER OK at each step)

- [ ] After Tasks 13–14 merge: set `CONVERTER_REF`, then per `~/src/viam-lerobot-training/README.md` sections 4–5: `docker buildx` amd64 build with the HF token secret, push, `viam train containers register`, `viam training-script upload` (bump `--version`). The user may prefer to run these with their own credentials.
- [ ] Submit (exact, in the DevRelDemos org, which holds the dataset):

```
viam train submit custom from-registry --dataset-id=<diff-ik-pick-v3 id> --org-id=16518049-9dd3-479f-9644-c5d112aa42d8 \
  --model-name=diff-ik-pick-act --model-version=1 --version=<script version> \
  --script-name=16518049-9dd3-479f-9644-c5d112aa42d8:lerobot-training --container-id=<container id> \
  --args=arm=arm,grippers=gripper,gripper_key=input,action_space=ee,cameras="'scene-cam wrist-cam'",image_size=224,fps=10,policy=act,steps=20000,batch_size=8,extra="'--policy.chunk_size=20 --policy.n_action_steps=20'"
```

  Check the args against the README table before submitting: a job on the default `gripper_key=position` or `action_space=joint` is a wasted billed run.
- [ ] Watch `viam train logs`; on completion, deploy the model package to `vla-policy` (`dev/bc-pick-mujoco.json` and the machine config: the registry package instead of `model_path`), commit the config.
- [ ] Cloud gate: `policy_run.py --mode module --cadence sync --lead-ms <Task 12 value> --seeds 9100-9129 --checkpoint-label cloud`. ≥21/30 passes. If it lands well short of the local checkpoint, compare the two training configs (`train_metadata.json` against the local `train_config.json`) before anything else. Otherwise below 21: the spec's fallbacks, each with the user's OK.
- [ ] Results section "Cloud ACT". **USER CHECK-IN 5.**

## Task 17: v0.4.1 (CONTROLLER + USER OK)

- [ ] Cherry-pick or merge Tasks 1–2 onto `main` (they are the only Go changes), `go test -race ./...`, push `main`, tag `v0.4.1`, push the tag (deploy.yml publishes). Ask first; the user may prefer to release with sub-project 2's merge.

## Task 18: The comparison (CONTROLLER, machine)

- [ ] Six runs on the cloud checkpoint, seeds 9100–9129, `--lead-ms` from Task 12:
  `module sync`, `module async`, `client sync`, `client async`, `move sync`, `move async` (`--replan-every 10`), each to `.runs/eval/cmp-<mode>-<cadence>.jsonl`.
- [ ] `uv run clients/python/policy_report.py .runs/eval/cmp-*.jsonl` → tables into the results doc, read per the spec ("±15 points", paired metrics first). Add the per-mode observations (watch one episode of each in `overview`).
- [ ] Update `README.md`'s `servo_trajectory` paragraph with the headline (one or two sentences) and link the results.
- [ ] Commit. **USER CHECK-IN 6** with the tables and a recommendation. Stop the machine.
