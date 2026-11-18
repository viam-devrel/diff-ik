# Diff-IK Phase 4 (moving-tag visual servoing in MuJoCo) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Servo the MuJoCo SO-101 to follow an AprilTag that the sim slides around a circle at 10, 20, 40 and 80 mm/s. Do it once with a wrist camera and once with a scene camera, measuring tracking error against the tag's exact path.

**Architecture:**
- **Sim:** a sim cell config adds the textured tag prop, two sim cameras and two `viam:apriltag` pose trackers.
- **Runner:** `clients/python/tag_sweep.py` subclasses the existing `TagServo` client. It adds three things:
  - a mover that drives the prop through the world's `set_prop_pose` and logs ground truth;
  - a 50 Hz arm logger;
  - a summary file.
- **Report:** `clients/python/tag_sweep_report.py` turns the logs into a table for each speed.
- **Existing code:** `tag_servo.py` gets one refactor (`start_tracking()`, `tag_parser()`), and `servo_client.py` gets a `lag_ms` column.

**Tech Stack:**
- Python ≥ 3.11 scripts run with `uv run`; tests are plain `assert` scripts, as in the existing `clients/python/test_*.py`.
- `viam-sdk`.
- `opencv-contrib-python`, only for generating the tag PNG.
- viam-mujoco-sim, branch `so101-asset` (worktree `~/src/viam-mujoco-sim-so101`).
- `viam:apriltag`.
- Go only to build `bin/diff-ik`.

**Spec:** `docs/superpowers/specs/2026-11-17-diff-ik-phase4-tag-sweep-design.md`. Read it first.

**Branch:** `diff-ik-phase4` (already created; the spec commits are on it).

**Commit trailer** (every commit):
```
Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_012FAJr9jcJyz2f1AEiCkX3y
```

**Who does what:**
- **Code tasks for subagents:** Tasks 4–8.
- **Controller tasks:** Tasks 1–3 and 9–10, which run the sim and need judgement or the user.
- **Order:**
  1. Task 3 (the Task 0 checks) has to pass before Task 7's defaults are final. Tasks 4, 5, 6 and 8 don't depend on it and can start first.
  2. Task 7 needs Task 3's `tip_mm` and circle centre.
  3. Task 9 needs everything.

## File map

| File | Action | Responsibility |
|---|---|---|
| `dev/make_tag_png.py` | create | Generate `dev/assets/tag36h11_0.png` (10×10 cells, 1-cell white border) and check that it decodes |
| `dev/assets/tag36h11_0.png` | create (generated) | The tag texture |
| `dev/so101-mujoco-tag.json` | create | The sim cell: arm, gripper, table, tag prop, `wrist-cam`, `scene-cam`, the apriltag module and `wrist-tags`/`scene-tags`, and the `servo` service |
| `clients/python/servo_client.py` | modify | `FOLLOW_COLS` gains `lag_ms`; a pure `follow_row()` builds watchdog rows |
| `clients/python/test_servo_client.py` | create | `follow_row` checks |
| `clients/python/tag_servo.py` | modify | `start_tracking()` split out of `run()`; `tag_parser()` split out of `main()`; behaviour unchanged |
| `clients/python/tag_sweep.py` | create | `circle_state`, `circle_xy`, `TagSweep` (mover, arm logger, run/go, summary), CLI |
| `clients/python/test_tag_sweep.py` | create | Schedule checks |
| `clients/python/tag_sweep_report.py` | create | Per-speed analysis and a markdown table |
| `clients/python/test_tag_sweep_report.py` | create | Synthetic-log checks |
| `dev/so101-mujoco.md` | create | Runbook for Phase 4 |
| `docs/results/2026-11-phase4-tag-sweep.md` | create | Task 0 findings, results and findings |

---

### Task 1: Tag texture (controller)

**Files:** create `dev/make_tag_png.py`, generated `dev/assets/tag36h11_0.png`.

- [ ] **Step 1: Write the generator.**

```python
# /// script
# requires-python = ">=3.11"
# dependencies = ["opencv-contrib-python", "numpy"]
# ///
"""Phase 4 tag texture: tag36h11 ID 0 as a 10x10-cell PNG (the 8-cell black square plus a 1-cell white
border). On the 75 mm sim prop the black square is 60 mm, so tag_width_mm is 60.
Run: uv run dev/make_tag_png.py"""
import os

import cv2
import numpy as np

CELL = 80  # px per cell
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets", "tag36h11_0.png")

d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
black = cv2.aruco.generateImageMarker(d, 0, 8 * CELL, borderBits=1)  # 6x6 data + 1-cell black border
img = np.full((10 * CELL, 10 * CELL), 255, np.uint8)
img[CELL:9 * CELL, CELL:9 * CELL] = black
os.makedirs(os.path.dirname(OUT), exist_ok=True)
cv2.imwrite(OUT, img)

# self-check: it decodes as ID 0, and the black square is 80 % of the image
corners, ids, _ = cv2.aruco.ArucoDetector(d).detectMarkers(img)
assert ids is not None and ids.flatten().tolist() == [0], ids
side = np.linalg.norm(corners[0][0][0] - corners[0][0][1])
assert abs(side / img.shape[0] - 0.8) < 0.01, side
print("wrote", OUT, img.shape)
```

- [ ] **Step 2: Run it.** `uv run dev/make_tag_png.py` should print `wrote …/dev/assets/tag36h11_0.png (800, 800)`.
- [ ] **Step 3: Commit.** `git add dev/make_tag_png.py dev/assets/tag36h11_0.png && git commit -m "Phase 4: tag36h11 ID 0 texture (60 mm black square on a 75 mm prop)"`, with the trailer.

### Task 2: Sim cell config (controller)

**Files:** create `dev/so101-mujoco-tag.json`.

The arm's origin is at world (0, 0, 100) mm. The circle's default centre is (220, 0) mm in the arm frame, which is world (0.22, 0) m. The tag starts at the circle's angle-0 point, world (0.28, 0) m. The prop centre sits at world z = 2 mm: the box is 2 mm thick, and 1 mm of clearance stops it from snagging on the table.

- [ ] **Step 1: Write the config.**

```json
{
  "network": {"bind_address": "localhost:8082"},
  "modules": [
    {"type": "local", "name": "mujoco-sim", "executable_path": "/Users/nick.hehr/src/viam-mujoco-sim-so101/run.sh"},
    {"type": "local", "name": "diff-ik", "executable_path": "/Users/nick.hehr/src/diff-ik/bin/diff-ik"},
    {"type": "registry", "name": "viam_apriltag", "module_id": "viam:apriltag", "version": "0.2.1"}
  ],
  "components": [
    {"name": "sim-world", "api": "rdk:component:generic", "model": "viam-labs:mujoco-sim:world",
     "attributes": {"props": [
       {"name": "tag", "size": 0.075, "scale": [1, 1, 0.02667], "position": [0.28, 0, 0.002], "mass": 0.05,
        "color": [1, 1, 1], "texture": "/Users/nick.hehr/src/diff-ik/dev/assets/tag36h11_0.png"}]}},
    {"name": "table", "api": "rdk:component:generic", "model": "viam-labs:mujoco-sim:obstacle",
     "attributes": {"world": "sim-world"},
     "frame": {"parent": "world", "translation": {"x": 250, "y": 0, "z": -10},
               "geometry": {"type": "box", "x": 600, "y": 600, "z": 20}}},
    {"name": "arm", "api": "rdk:component:arm", "model": "viam-labs:mujoco-sim:arm",
     "attributes": {"world": "sim-world", "asset": "so101"},
     "frame": {"parent": "world", "translation": {"x": 0, "y": 0, "z": 100}}},
    {"name": "gripper", "api": "rdk:component:gripper", "model": "viam-labs:mujoco-sim:gripper",
     "attributes": {"world": "sim-world", "mount": "arm", "asset": "so101"}, "frame": {"parent": "arm"}},
    {"name": "wrist-cam", "api": "rdk:component:camera", "model": "viam-labs:mujoco-sim:camera",
     "attributes": {"world": "sim-world", "fov_deg": 60, "width": 640, "height": 480},
     "frame": {"parent": "arm", "translation": {"x": 0, "y": 60, "z": 30}}},
    {"name": "scene-cam", "api": "rdk:component:camera", "model": "viam-labs:mujoco-sim:camera",
     "attributes": {"world": "sim-world", "target": [0.22, 0, 0], "fov_deg": 60, "width": 1280, "height": 720},
     "frame": {"parent": "world", "translation": {"x": 220, "y": 318, "z": 318}}},
    {"name": "wrist-tags", "api": "rdk:component:pose_tracker", "model": "viam:apriltag:pose_tracker",
     "attributes": {"camera_name": "wrist-cam", "tag_family": "tag36h11", "tag_width_mm": 60}},
    {"name": "scene-tags", "api": "rdk:component:pose_tracker", "model": "viam:apriltag:pose_tracker",
     "attributes": {"camera_name": "scene-cam", "tag_family": "tag36h11", "tag_width_mm": 60}}
  ],
  "services": [
    {"name": "servo", "api": "rdk:service:motion", "model": "devrel:diff-ik:servo", "attributes": {"arms": ["arm"]}}
  ]
}
```

- [ ] **Step 2: Validate the JSON.** `python3 -m json.tool dev/so101-mujoco-tag.json > /dev/null` should print nothing.
- [ ] **Step 3: Commit.** `git add dev/so101-mujoco-tag.json && git commit -m "Phase 4: sim cell with tag prop, wrist and scene cameras, apriltag trackers"`, with the trailer.

### Task 3: Task 0 checks (controller; uses the sim, no new code committed)

Record each finding, with numbers, in a new `docs/results/2026-11-phase4-tag-sweep.md` under `## Task 0 findings`. Put probe scripts in the session scratchpad, not the repo. If a check fails and its fallback needs the sim repo changed, **stop and ask the user**.

- [ ] **Step 1: Build and start the cell.** `go build -o bin/diff-ik ./cmd/module`, then `viam-server -config dev/so101-mujoco-tag.json`, run in the background with its log to the scratchpad.
- [ ] **Step 2: apriltag on macOS (spec Task 0.1).**
  - **Check:** the server log shows `viam_apriltag` started and `wrist-tags`/`scene-tags` built.
  - **If the registry module fails to download or has no darwin build:**
    1. Clone `https://github.com/viam-labs/apriltag` (or the `viam` fork named in the registry entry) to `~/src/apriltag`.
    2. Follow its README to run from source, and replace the registry entry with `{"type": "local", "name": "viam_apriltag", "executable_path": "<its run script>"}`.
    3. Record which path worked.
- [ ] **Step 3: Prop motion at 50 Hz (Task 0.2).**
  - **Probe:** a short script connects with `--insecure` to `localhost:8082`. For 10 s, it calls `sim-world` `do_command({"command": "set_prop_pose", "prop": "tag", "position": [x, y, 0.002], "orientation_wxyz": [1, 0, 0, 0]})` at 50 Hz along a 60 mm circle at 40 mm/s.
  - **Readback:** between calls it reads `{"command": "get_prop_pose", "prop": "tag"}` and logs the commanded-versus-actual xy, z and tilt.
  - **Pass:** xy within 1 mm, z within 2 mm, tilt < 2°, and an achieved call rate ≥ 45 Hz.
  - **Fail:** stop and ask (a kinematic prop would be a sim change).
- [ ] **Step 4: Detection rates (Task 0.3).**
  - **Probe:** with the tag at rest at (0.28, 0), first move the arm so the wrist camera sees it. Do that by running `tag_servo.py` itself, as in Step 6.
  - **Measure:** call `get_poses(body_names=["0"])` on each tracker for 10 s and record successful detections per second, plus the `get_images` rate of each camera.
- [ ] **Step 5: Scene-camera coverage (Task 0.4).** Place the tag at 8 points round the circle with `set_prop_pose`. At each point, check that `scene-tags` returns tag `0` in 10 out of 10 polls. If not, move `scene-cam` closer along its line to the centre (300 mm, then 200 mm) and re-check. Step 6 repeats the occlusion check with the arm hovering over the tag: watch for drops there too.
- [ ] **Step 6: Reach and tip offset (Task 0.5).**
  - **`tip_mm`:** use 98 from the Phase 3b results (the jaw-tip site is 98 mm beyond Viam's end effector, with a 0.03 mm residual). Record it.
  - **Reach probe:** place the tag at the circle's points at 0°, 90°, 180° and 270° (arm-frame centre (220, 0), radius 60). At each, run

    ```
    uv run clients/python/tag_servo.py --address localhost:8082 --insecure --follower arm \
      --camera scene-cam --tracker scene-tags --axis down --hover-mm 150 --tip-mm 98 \
      --joint-deg-s 90 --linear-mm-s 120 --duration 5
    ```

    and confirm it reaches `track` and holds (the tag stays still).
  - **If it fails:** if a point is out of reach or a joint limit stalls it, move the centre (try (200, 0), then (240, 0)) and record the centre that works.
- [ ] **Step 7: Texture scale (Task 0.6).** With the tag at rest, read the tracker's pose of tag `0` through `robot.transform_pose(…, "arm_origin")` for both cameras. Compare it with the known tag-face centre: the commanded prop position plus 1 mm in z, minus the arm origin.
  - **Pass:** within 3 mm.
  - **If the depth is off by a constant factor:** record the factor and adjust `tag_width_mm` by it. The likeliest factor is √2: the registry docs call `tag_width_mm` "corner to corner", while the runbook says it's the square's edge. If √2 fits, the value is 84.9 and the runbook note should be corrected. Any other factor means the texture is scaled.
- [ ] **Step 8: Commit the findings.** `git add docs/results/2026-11-phase4-tag-sweep.md && git commit -m "Phase 4: Task 0 findings"` (plus any config change), with the trailer.

### Task 4: `lag_ms` in the follower CSV (subagent)

**Files:** modify `clients/python/servo_client.py` (`FOLLOW_COLS` at ~L17; `watchdog()` at ~L170), create `clients/python/test_servo_client.py`.

- [ ] **Step 1: Write the failing test** in `clients/python/test_servo_client.py`:

```python
# /// script
# requires-python = ">=3.11"
# ///
"""Pure-helper checks for servo_client.py. Run: uv run clients/python/test_servo_client.py"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from servo_client import FOLLOW_COLS, POSE_KEYS, follow_row  # noqa: E402

fp = dict(zip(POSE_KEYS, [1.0, 2.0, 3.0, 0.0, 0.0, -1.0, 0.0]))
r = follow_row(1.23456, fp)
assert len(r) == len(FOLLOW_COLS), (len(r), len(FOLLOW_COLS))
assert r[0] == "1.2346" and r[1] == "pose" and r[2] == 1.0
assert FOLLOW_COLS[-1] == "lag_ms" and r[-1] == ""
s = follow_row(2.0, fp, {"rate_hz": 50.0, "tick_p99_ms": 3.0, "overruns": 1, "resyncs": 2, "lag_ms": 42.0})
assert len(s) == len(FOLLOW_COLS) and s[1] == "status"
assert dict(zip(FOLLOW_COLS, s))["lag_ms"] == 42.0 and dict(zip(FOLLOW_COLS, s))["resyncs"] == 2
print("ok")
```

- [ ] **Step 2: Run it.** `uv run clients/python/test_servo_client.py` should fail with `ImportError: cannot import name 'follow_row'`.
- [ ] **Step 3: Implement.** In `servo_client.py`:

```python
FOLLOW_COLS = ["t", "kind", *[f"f_{k}" for k in POSE_KEYS], "rate_hz", "tick_p99_ms", "overruns", "resyncs", "lag_ms"]


def follow_row(t: float, fp: dict, st: dict | None = None) -> list:
    """One follower-CSV row: a "pose" row, or a "status" row from a servo_status reply st."""
    if st is None:
        return [f"{t:.4f}", "pose", *[fp[q] for q in POSE_KEYS], "", "", "", "", ""]
    return [f"{t:.4f}", "status", *[fp[q] for q in POSE_KEYS], st.get("rate_hz"), st.get("tick_p99_ms"),
            st.get("overruns"), st.get("resyncs"), st.get("lag_ms")]
```

  In `watchdog()`, replace the two inline `row = [...]` lists with `row = follow_row(tp, fp)` and `row = follow_row(tp, fp, st)`. Nothing else in `watchdog()` changes.
- [ ] **Step 4: Run.** `uv run clients/python/test_servo_client.py` prints `ok`. Then run every existing client test, which should all pass unchanged because readers use `csv.DictReader`:

  ```
  for f in clients/python/test_*.py; do uv run "$f" || echo "FAIL $f"; done
  ```
- [ ] **Step 5: Commit.** `git add clients/python/servo_client.py clients/python/test_servo_client.py && git commit -m "servo_client: lag_ms column in the follower CSV (follow_row)"`, with the trailer.

### Task 5: `start_tracking()` and `tag_parser()` in `tag_servo.py` (subagent)

**Files:** modify `clients/python/tag_servo.py` (`run()` at ~L295, `main()` at ~L338). This is a behaviour-preserving refactor.

- [ ] **Step 1: Split `run()`.** Add `start_tracking()` above `run()`. It holds everything from the floor check through the full-limit `start_session`. `run()` keeps the `--final-hover-mm` check and calls it:

```python
    async def start_tracking(self):
        """Floor check, the detection and watchdog tasks, the first tag, the half-limit engage session until
        settled, then the full-limit track session. Keep the tag still until this returns."""
        a = self.args
        fp = pose_dict(await self.follower2.get_end_position())  # the watchdog's first read may lag
        if lowest_z(fp, a.tip_mm) < a.min_z_mm:
            raise RuntimeError(f"follower is below the floor (lowest z {lowest_z(fp, a.tip_mm):.1f} mm); "
                               "park it at q_start first (runbook)")
        self.det_task = asyncio.create_task(self.detect())
        self.tasks = [asyncio.create_task(self.watchdog()), self.det_task]
        # the dead-man starts at servo_start and only applied targets refresh it: see the tag first
        t_end = time.monotonic() + 10
        while self.det is None and self.fault is None:
            if time.monotonic() > t_end:
                raise RuntimeError("no tag seen in 10 s")
            await asyncio.sleep(0.02)
        if self.fault:
            raise RuntimeError(self.fault)
        kw = {"frames": True, "target_filter": self.target_filter()}

        print("engaging at half the L2 limits…")
        await self.start_session("pointing", half=True, **kw)
        await self.stream("engage", self.settle(10, 5, 0.2, 30, "engage"))
        await self.stop_session()
        print("tracking. Ctrl-C to stop.")
        await self.start_session("pointing", half=False, **kw)

    async def run(self):
        a = self.args
        if a.final_hover_mm is not None and a.final_hover_mm >= a.hover_mm:
            raise RuntimeError("--final-hover-mm must be below --hover-mm")
        await self.start_tracking()
        end = math.inf
        # … the rest of the current run() from `if a.final_hover_mm is None:` onwards, unchanged
```

- [ ] **Step 2: Split `main()`.** Move the parser into `tag_parser(doc: str) -> argparse.ArgumentParser`: `base_parser(doc)`, the `set_defaults`, and every `add_argument` currently in `main()`. `main()` becomes `return run_client(TagServo, tag_parser(__doc__))`.
- [ ] **Step 3: Verify.**
  - `uv run clients/python/test_tag_servo.py` passes.
  - `uv run clients/python/tag_servo.py --help` lists the same flags as before.
  - `git diff` shows only the moves above.
- [ ] **Step 4: Commit.** `git add clients/python/tag_servo.py && git commit -m "tag_servo: start_tracking() and tag_parser() (no behaviour change)"`, with the trailer.

### Task 6: The circle schedule (subagent)

**Files:** create `clients/python/tag_sweep.py` (pure helpers only in this task) and `clients/python/test_tag_sweep.py`.

- [ ] **Step 1: Write the failing test** in `clients/python/test_tag_sweep.py`:

```python
# /// script
# requires-python = ">=3.11"
# ///
"""Schedule checks for tag_sweep.py. Run: uv run clients/python/test_tag_sweep.py"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tag_sweep import circle_state, circle_xy, total_time  # noqa: E402

V, LAPS, R, RAMP = [10.0, 20.0], 2, 60.0, 1.0
C = 2 * math.pi * R
T = total_time(V, LAPS, R, RAMP)
assert abs(T - (RAMP + LAPS * C / 10 + RAMP + LAPS * C / 20)) < 1e-9, T

# start: at rest at angle 0, ramping
s0 = circle_state(0.0, V, LAPS, R, RAMP)
assert s0["s"] == 0 and s0["speed"] == 0 and s0["seg"] == 0 and s0["lap"] == -1 and not s0["done"]
assert circle_xy(0.0, 220, 0, R) == (280.0, 0.0)

# continuous position and speed at every boundary
for tb in (RAMP, RAMP + LAPS * C / 10, 2 * RAMP + LAPS * C / 10):
    a, b = circle_state(tb - 1e-6, V, LAPS, R, RAMP), circle_state(tb + 1e-6, V, LAPS, R, RAMP)
    assert abs(a["s"] - b["s"]) < 1e-3 and abs(a["speed"] - b["speed"]) < 1e-2, (tb, a, b)

# holds at the segment speed; laps counted from the end of the ramp
m = circle_state(RAMP + 1.5 * C / 10, V, LAPS, R, RAMP)
assert m["speed"] == 10 and m["seg"] == 0 and m["seg_speed"] == 10 and m["lap"] == 1
r2 = circle_state(RAMP + LAPS * C / 10 + 0.5, V, LAPS, R, RAMP)
assert r2["seg"] == 1 and r2["lap"] == -1 and 10 < r2["speed"] < 20 and r2["seg_speed"] == 20

# done after the schedule, at rest
e = circle_state(T + 1, V, LAPS, R, RAMP)
assert e["done"] and e["speed"] == 0 and abs(e["s"] - (C * 2 * LAPS + RAMP * 5 + RAMP * 15)) < 1e-6, e

# a quarter turn lands at 90 degrees
x, y = circle_xy(C / 4, 220, 0, R)
assert abs(x - 220) < 1e-9 and abs(y - 60) < 1e-9
print("ok", round(T, 1), "s")
```

- [ ] **Step 2: Run it.** `uv run clients/python/test_tag_sweep.py` should fail with `ModuleNotFoundError: No module named 'tag_sweep'`.
- [ ] **Step 3: Implement** the start of `clients/python/tag_sweep.py`:

```python
# /// script
# requires-python = ">=3.11"
# dependencies = ["viam-sdk"]
# ///
"""Phase 4: follow an AprilTag that the MuJoCo sim slides around a circle at increasing speeds.

  uv run clients/python/tag_sweep.py --address localhost:8082 --insecure --camera wrist-cam --tracker wrist-tags \\
      --csv .runs/p4-wrist
Spec: docs/superpowers/specs/2026-11-17-diff-ik-phase4-tag-sweep-design.md. Runbook: dev/so101-mujoco.md.
"""
import math


def total_time(speeds: list, laps: int, radius_mm: float, ramp_s: float) -> float:
    c = 2 * math.pi * radius_mm
    return sum(ramp_s + laps * c / v for v in speeds)


def circle_state(t: float, speeds: list, laps: int, radius_mm: float, ramp_s: float) -> dict:
    """Where the tag is t s after the mover starts. Each speed: a ramp_s linear speed ramp from the previous
    speed (0 for the first), then laps full circles at that speed. Arc length s (mm) is integrated, so
    position and speed are continuous. lap is -1 in a ramp, else the completed laps at this speed."""
    c = 2 * math.pi * radius_mm
    s, prev, t0 = 0.0, 0.0, 0.0
    for i, v in enumerate(speeds):
        if t < t0 + ramp_s:
            d = t - t0
            return {"s": s + prev * d + (v - prev) * d * d / (2 * ramp_s), "speed": prev + (v - prev) * d / ramp_s,
                    "seg": i, "seg_speed": v, "lap": -1, "done": False}
        s += (prev + v) / 2 * ramp_s
        t0 += ramp_s
        hold = laps * c / v
        if t < t0 + hold:
            return {"s": s + v * (t - t0), "speed": v, "seg": i, "seg_speed": v,
                    "lap": int(v * (t - t0) // c), "done": False}
        s += laps * c
        t0 += hold
        prev = v
    return {"s": s, "speed": 0.0, "seg": len(speeds) - 1, "seg_speed": speeds[-1], "lap": laps, "done": True}


def circle_xy(s: float, cx: float, cy: float, radius_mm: float) -> tuple:
    """Arc length s (mm) from angle 0 to a point on the circle, counter-clockwise."""
    a = s / radius_mm
    return cx + radius_mm * math.cos(a), cy + radius_mm * math.sin(a)
```

- [ ] **Step 4: Run.** `uv run clients/python/test_tag_sweep.py` prints `ok … s`.
- [ ] **Step 5: Commit.** `git add clients/python/tag_sweep.py clients/python/test_tag_sweep.py && git commit -m "tag_sweep: circle speed schedule (circle_state, circle_xy)"`, with the trailer.

### Task 7: `TagSweep` runner (subagent, after Task 3)

**Files:** modify `clients/python/tag_sweep.py`.

Before starting, read the following:
- `clients/python/tag_servo.py`: `TagServo`, `start_tracking`, `stream`, `settle`, `tag_parser`;
- `clients/python/servo_client.py`: `open`, `close`, `release_extras`, `run_client`.

`TIP_MM` and `CENTRE` below are the values recorded in Task 3 Step 6, by default `98` and `"220,0"`.

- [ ] **Step 1: Add the class and CLI** below the helpers:

```python
import asyncio
import csv
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from servo_client import POSE_KEYS, pose_dict, run_client  # noqa: E402
from tag_servo import TagLost, TagServo, tag_parser  # noqa: E402

MOVER_HZ = 50
ARM_HZ = 50


class TagSweep(TagServo):
    def __init__(self, args):
        super().__init__(args)
        self.speeds = [float(v) for v in args.speeds.split(",")]
        self.cx, self.cy = (float(v) for v in args.centre.split(","))
        self.state: dict | None = None  # the mover's latest circle_state
        self.origin: tuple | None = None  # arm origin in world, mm
        self.mover_calls, self.mover_t0 = 0, None
        self._tagf = self._armf = None
        self.tag_csv = self.arm_csv = None
        self.world = None

    async def open_extras(self):
        await super().open_extras()
        from viam.components.generic import Generic
        from viam.proto.common import Pose, PoseInFrame

        a = self.args
        self.world = Generic.from_robot(self.robot2, a.world)
        o = (await self.robot.transform_pose(PoseInFrame(reference_frame=f"{a.follower}_origin", pose=Pose(o_z=1)),
                                             "world")).pose
        if abs(o.o_z - 1) > 1e-6 or abs(o.theta) > 1e-6:
            raise RuntimeError("the arm origin must be unrotated in world (only a translation is supported)")
        self.origin = (o.x, o.y, o.z)
        if a.csv:
            os.makedirs(os.path.dirname(a.csv) or ".", exist_ok=True)
            self._tagf = open(f"{a.csv}-tag.csv", "w", newline="")
            self._armf = open(f"{a.csv}-joints.csv", "w", newline="")
            self.tag_csv, self.arm_csv = csv.writer(self._tagf), csv.writer(self._armf)
            self.tag_csv.writerow(["t", "x", "y", "z", "speed", "seg", "seg_speed", "lap"])

    async def release_extras(self, step):
        for f in (self._tagf, self._armf):
            if f:
                f.close()

    # ---- the tag ------------------------------------------------------------------------------
    async def place(self, x: float, y: float):
        """Put the prop's centre at arm-frame (x, y) mm, resting flat; set_prop_pose wants world metres."""
        ox, oy, _ = self.origin
        await self.world.do_command({"command": "set_prop_pose", "prop": self.args.prop,
                                     "position": [(x + ox) / 1000, (y + oy) / 1000, self.args.prop_z_mm / 1000],
                                     "orientation_wxyz": [1, 0, 0, 0]})

    def face_z(self) -> float:
        """The tag face's z in the arm frame (mm): prop centre + half the 2 mm thickness, minus the origin."""
        return self.args.prop_z_mm + 1.0 - self.origin[2]

    async def mover(self):
        a = self.args
        start, n = time.monotonic(), 0
        self.mover_t0 = self.now()
        try:
            while True:
                st = circle_state(time.monotonic() - start, self.speeds, a.laps, a.radius_mm, a.ramp_s)
                x, y = circle_xy(st["s"], self.cx, self.cy, a.radius_mm)
                await self.place(x, y)
                self.state = st
                self.mover_calls += 1
                if self.tag_csv:
                    self.tag_csv.writerow([f"{self.now():.4f}", f"{x:.3f}", f"{y:.3f}", f"{self.face_z():.3f}",
                                           f"{st['speed']:.3f}", st["seg"], st["seg_speed"], st["lap"]])
                if st["done"]:
                    return
                n += 1
                await asyncio.sleep(max(0.0, start + n / MOVER_HZ - time.monotonic()))
        except asyncio.CancelledError:
            raise
        except Exception as e:
            self.fault = f"mover: {e}"

    async def arm_logger(self):
        """Joints (deg) and end-effector pose at 50 Hz: the report's jaw-tip path and joint accelerations."""
        start, n, header = time.monotonic(), 0, False
        try:
            while True:
                q = (await self.follower2.get_joint_positions()).values
                p = pose_dict(await self.follower2.get_end_position())
                if self.arm_csv:
                    if not header:
                        self.arm_csv.writerow(["t", *[f"q{i}" for i in range(len(q))], *POSE_KEYS])
                        header = True
                    self.arm_csv.writerow([f"{self.now():.4f}", *q, *[p[k] for k in POSE_KEYS]])
                n += 1
                await asyncio.sleep(max(0.0, start + n / ARM_HZ - time.monotonic()))
        except asyncio.CancelledError:
            raise
        except Exception as e:
            self.fault = f"arm logger: {e}"

    # ---- run ----------------------------------------------------------------------------------
    async def run(self):
        a = self.args
        await self.place(*circle_xy(0.0, self.cx, self.cy, a.radius_mm))
        await asyncio.sleep(0.5)  # let the prop settle and the trackers see it
        await self.start_tracking()
        self.tasks.append(asyncio.create_task(self.arm_logger()))
        await self.stream("track", self.settle(2, 2, 1, 30, "converge"))
        mv = asyncio.create_task(self.mover())
        self.tasks.append(mv)
        print(f"sweeping {a.speeds} mm/s, {a.laps} laps each, "
              f"{total_time(self.speeds, a.laps, a.radius_mm, a.ramp_s):.0f} s")
        await self.stream("sweep", lambda rep, t: mv.done())
        if self.fault:
            raise RuntimeError(self.fault)

    async def go(self):
        lost = None
        try:
            await self.run()
        except TagLost:
            lost = self.state["seg_speed"] if self.state else 0.0
            print(f"tag lost at {lost} mm/s")
        finally:
            self.write_summary(lost)

    def write_summary(self, lost):
        a = self.args
        if not a.csv:
            return
        dur = self.now() - self.mover_t0 if self.mover_t0 is not None else 0.0
        det_dur = self.now() - self.first_det_t if self.first_det_t is not None else 0.0
        out = {"args": vars(a), "arm_origin_mm": self.origin, "breaking_speed_mm_s": lost,
               "completed": bool(self.state and self.state["done"]), "fault": self.fault,
               "mover_hz": self.mover_calls / dur if dur > 0 else None,
               "accepted_det_hz": self.seq / det_dur if det_dur > 0 else None}
        with open(f"{a.csv}-summary.json", "w") as f:
            json.dump(out, f, indent=2)


def main() -> int:
    ap = tag_parser(__doc__)
    ap.set_defaults(address="localhost:8082", follower="arm", camera="wrist-cam", tracker="wrist-tags",
                    axis="down", hover_mm=150, tip_mm=TIP_MM, joint_deg_s=90, linear_mm_s=120)
    ap.add_argument("--speeds", default="10,20,40,80", help="mm/s, in order")
    ap.add_argument("--laps", type=int, default=3)
    ap.add_argument("--radius-mm", type=float, default=60)
    ap.add_argument("--centre", default=CENTRE, help="x,y mm in the arm frame")
    ap.add_argument("--ramp-s", type=float, default=1.0)
    ap.add_argument("--world", default="sim-world")
    ap.add_argument("--prop", default="tag")
    ap.add_argument("--prop-z-mm", type=float, default=2.0, help="world z of the prop centre")
    return run_client(TagSweep, ap)


if __name__ == "__main__":
    sys.exit(main())
```

  Replace `TIP_MM` and `CENTRE` with the literal values from Task 3, for example `tip_mm=98` and `default="220,0"`.
- [ ] **Step 2: Static checks.**
  - `uv run clients/python/test_tag_sweep.py` still prints `ok`.
  - `uv run clients/python/tag_sweep.py --help` lists the sweep flags with the sim defaults.
- [ ] **Step 3: Smoke run (controller, with the Task 3 cell running).**

  ```
  uv run clients/python/tag_sweep.py --insecure --speeds 10 --laps 1 --csv .runs/p4-smoke
  ```

  It should engage, converge, run one lap at 10 mm/s and exit 0. Then:
  - `.runs/p4-smoke-{tag,joints,ticks,follower}.csv` all have rows;
  - `.runs/p4-smoke-summary.json` has `"completed": true`;
  - the report lists drop reasons as well as the total (the results doc gives them in prose).
  - `mover_hz` ≥ 45.
- [ ] **Step 4: Commit.** `git add clients/python/tag_sweep.py && git commit -m "tag_sweep: TagSweep runner (mover, 50 Hz arm logger, summary)"`, with the trailer.

### Task 8: Sweep report (subagent)

**Files:** create `clients/python/tag_sweep_report.py` and `clients/python/test_tag_sweep_report.py`.

- [ ] **Step 1: Write the failing test** in `clients/python/test_tag_sweep_report.py`. It builds a synthetic 20 mm/s run in which the jaw tip follows the tag 100 ms late:

```python
# /// script
# requires-python = ">=3.11"
# ///
"""Synthetic-log checks for tag_sweep_report.py. Run: uv run clients/python/test_tag_sweep_report.py"""
import csv
import json
import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tag_sweep_report import accel_peak, analyze, markdown  # noqa: E402

R, V, CX, LAG, TIP = 60.0, 20.0, 220.0, 0.1, 100.0
C = 2 * math.pi * R
END = 2 * C / V  # two laps, no ramp


def tag_xy(t):
    a = V * max(t, 0.0) / R
    return CX + R * math.cos(a), R * math.sin(a)


pre = os.path.join(tempfile.mkdtemp(), "run")
with open(pre + "-tag.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "x", "y", "z", "speed", "seg", "seg_speed", "lap"])
    for k in range(int(END * 50)):
        t = k / 50
        x, y = tag_xy(t)
        w.writerow([t, x, y, -97, V, 0, V, int(V * t // C)])
with open(pre + "-joints.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "q0", "q1", "x", "y", "z", "o_x", "o_y", "o_z", "theta"])
    for k in range(int(END * 50)):
        t = k / 50
        x, y = tag_xy(t - LAG)
        w.writerow([t, 10 * math.sin(2 * math.pi * t), 0, x, y, 200, 0, 0, -1, 0])  # jaw tip at z = 100
with open(pre + "-ticks.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "kind", "phase", "state"])
    for k in range(int(END * 20)):
        w.writerow([k / 20, "det", "sweep", "running"])
    w.writerow([1.0, "drop", "sweep", "jump"])
with open(pre + "-follower.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "kind", "resyncs", "overruns", "lag_ms"])
    for k in range(int(END * 5)):
        w.writerow([k / 5, "status", k // 50, 0, 40.0])
        w.writerow([k / 5 + 0.1, "pose", "", "", ""])
with open(pre + "-summary.json", "w") as f:
    json.dump({"args": {"tip_mm": TIP, "camera": "wrist-cam"}, "breaking_speed_mm_s": None}, f)

res = analyze(pre)
(row,) = res["rows"]
assert row["speed"] == V and not row["lost"], row
assert abs(row["err"]["mean"] - V * LAG) < 0.2, row["err"]  # 2 mm behind on the circle
assert abs(row["lag_ms"] - 100) <= 10, row["lag_ms"]
assert row["err_at_lag"] < 0.3, row["err_at_lag"]
assert abs(row["det_hz"] - 20) < 1 and row["drops"] == 1 and row["drop_reasons"] == {"jump": 1}, row
assert row["resyncs"] == int(END * 5 - 1) // 50 and row["overruns"] == 0 and row["max_lag_ms"] == 40.0, row
peak = 10 * (2 * math.pi) ** 2  # 10 deg sine at 1 Hz
assert abs(row["peak_accel"] - peak) / peak < 0.1, row["peak_accel"]
assert abs(max(accel_peak([k / 50 for k in range(200)], [[5 * (k / 50) ** 2] for k in range(200)])) - 10) < 0.5
md = markdown([res])
assert "wrist-cam" in md and "| 20 |" in md, md
print("ok", {k: row[k] for k in ("lag_ms", "err_at_lag", "det_hz", "peak_accel")})
```

- [ ] **Step 2: Run it.** `uv run clients/python/test_tag_sweep_report.py` should fail with `ModuleNotFoundError: No module named 'tag_sweep_report'`.
- [ ] **Step 3: Implement** `clients/python/tag_sweep_report.py`:

```python
# /// script
# requires-python = ">=3.11"
# ///
"""Phase 4: a per-speed tracking table for tag_sweep.py runs.

  uv run clients/python/tag_sweep_report.py PREFIX [PREFIX ...] [--md OUT.md]
Reads PREFIX-tag.csv, -joints.csv, -ticks.csv, -follower.csv and -summary.json.
Spec: docs/superpowers/specs/2026-11-17-diff-ik-phase4-tag-sweep-design.md"""
import argparse
import collections
import json
import math
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from teleop_lag import _interp, _rows  # noqa: E402

LAGS = [k * 0.005 for k in range(-100, 101)]  # -0.5 .. +0.5 s


def jaw_tip(r: dict, tip_mm: float) -> tuple:
    """End-effector position plus tip_mm along the tool axis (Viam's orientation vector)."""
    n = math.sqrt(sum(float(r[k]) ** 2 for k in ("o_x", "o_y", "o_z")))
    return tuple(float(r[k]) + tip_mm * float(r[o]) / n for k, o in (("x", "o_x"), ("y", "o_y"), ("z", "o_z")))


def horiz_errors(ts, tips, tag_t, tag_xy, lag=0.0) -> list:
    """Horizontal mm from each jaw tip to the tag at t - lag; samples outside the tag log are skipped."""
    out = []
    for t, p in zip(ts, tips):
        u = t - lag
        if tag_t[0] <= u <= tag_t[-1]:
            x, y = _interp(tag_t, tag_xy, u)
            out.append(math.hypot(p[0] - x, p[1] - y))
    return out


def best_lag(ts, tips, tag_t, tag_xy) -> tuple:
    def mean_at(lag):
        e = horiz_errors(ts, tips, tag_t, tag_xy, lag)
        return statistics.fmean(e) if e else math.inf

    lag = min(LAGS, key=mean_at)
    return lag, mean_at(lag)


def stats(vs) -> dict:
    s = sorted(vs)
    return {"mean": statistics.fmean(s), "p95": s[min(len(s) - 1, int(0.95 * len(s)))], "max": s[-1]}


def accel_peak(ts, qs, win=5) -> list:
    """Peak |acceleration| per joint (units/s²): a win-sample moving average, then central second
    differences on the real timestamps."""
    h = win // 2
    t2 = ts[h:len(ts) - h]
    q2 = [[statistics.fmean(q[j] for q in qs[i - h:i + h + 1]) for j in range(len(qs[0]))]
          for i in range(h, len(qs) - h)]
    peak = [0.0] * len(qs[0])
    for i in range(1, len(t2) - 1):
        for j in range(len(peak)):
            v1 = (q2[i][j] - q2[i - 1][j]) / (t2[i] - t2[i - 1])
            v2 = (q2[i + 1][j] - q2[i][j]) / (t2[i + 1] - t2[i])
            peak[j] = max(peak[j], abs(v2 - v1) / ((t2[i + 1] - t2[i - 1]) / 2))
    return peak


def segments(tag) -> dict:
    """Per speed index: the speed, its whole window [t0, t1] and its steady window [s0, s1] (laps >= 1)."""
    segs = {}
    for r in tag:
        t, k, lap = float(r["t"]), int(r["seg"]), int(r["lap"])
        s = segs.setdefault(k, {"speed": float(r["seg_speed"]), "t0": t, "t1": t, "s0": None, "s1": None})
        s["t1"] = t
        if lap >= 1:
            s["s0"] = t if s["s0"] is None else s["s0"]
            s["s1"] = t
    return segs


def analyze(prefix: str) -> dict:
    with open(prefix + "-summary.json") as f:
        summ = json.load(f)
    tip = float(summ["args"]["tip_mm"])
    tag, arm = _rows(prefix + "-tag.csv"), _rows(prefix + "-joints.csv")
    ticks = _rows(prefix + "-ticks.csv")
    status = [r for r in _rows(prefix + "-follower.csv") if r["kind"] == "status"]
    tag_t = [float(r["t"]) for r in tag]
    tag_xy = [(float(r["x"]), float(r["y"])) for r in tag]
    at = [float(r["t"]) for r in arm]
    tips = [jaw_tip(r, tip) for r in arm]
    qcols = [c for c in arm[0] if c.startswith("q")]
    qs = [[float(r[c]) for c in qcols] for r in arm]
    rows = []
    for _, s in sorted(segments(tag).items()):
        row = {"speed": s["speed"], "lost": summ["breaking_speed_mm_s"] == s["speed"]}
        if s["s0"] is not None:
            idx = [i for i, t in enumerate(at) if s["s0"] <= t <= s["s1"]]
            ts, tp = [at[i] for i in idx], [tips[i] for i in idx]
            e = horiz_errors(ts, tp, tag_t, tag_xy)
            if e:
                row["err"] = stats(e)
            if len(ts) > 10:
                lag, el = best_lag(ts, tp, tag_t, tag_xy)
                row["lag_ms"], row["err_at_lag"] = round(lag * 1000), el

        def inwin(r):
            return s["t0"] <= float(r["t"]) <= s["t1"]

        dur = max(s["t1"] - s["t0"], 1e-9)
        row["det_hz"] = sum(1 for r in ticks if r["kind"] == "det" and inwin(r)) / dur
        drops = [r["state"] for r in ticks if r["kind"] == "drop" and inwin(r)]
        row["drops"], row["drop_reasons"] = len(drops), dict(collections.Counter(drops))
        st = [r for r in status if inwin(r)]
        if st:
            row["resyncs"] = int(float(st[-1]["resyncs"])) - int(float(st[0]["resyncs"]))
            row["overruns"] = int(float(st[-1]["overruns"])) - int(float(st[0]["overruns"]))
            lags = [float(r["lag_ms"]) for r in st if r["lag_ms"] not in ("", "None")]
            row["max_lag_ms"] = max(lags) if lags else None
        wq = [i for i, t in enumerate(at) if s["t0"] <= t <= s["t1"]]
        if len(wq) > 7:
            row["peak_accel"] = max(accel_peak([at[i] for i in wq], [qs[i] for i in wq]))
        rows.append(row)
    return {"prefix": prefix, "camera": summ["args"]["camera"],
            "breaking_speed_mm_s": summ["breaking_speed_mm_s"], "rows": rows}


def _f(v, nd=1):
    return "–" if v is None else f"{v:.{nd}f}"


def markdown(results: list) -> str:
    out = ["| Camera | Speed (mm/s) | Error mean / p95 / max (mm) | Lag (ms) | Error at lag (mm) | Detections (Hz) "
           "| Drops | Lost | Resyncs | Overruns | Max lag_ms | Peak joint accel (°/s²) |",
           "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for res in results:
        for r in res["rows"]:
            e = r.get("err")
            err = f"{_f(e['mean'])} / {_f(e['p95'])} / {_f(e['max'])}" if e else "–"
            out.append(f"| {res['camera']} | {r['speed']:g} | {err} | {r.get('lag_ms', '–')} | "
                       f"{_f(r.get('err_at_lag'))} | {_f(r['det_hz'])} | {r['drops']} | {'yes' if r['lost'] else 'no'} | "
                       f"{r.get('resyncs', '–')} | {r.get('overruns', '–')} | {_f(r.get('max_lag_ms'), 0)} | "
                       f"{_f(r.get('peak_accel'), 0)} |")
    return "\n".join(out)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("prefixes", nargs="+")
    ap.add_argument("--md", default=None, help="also write the table to this file")
    a = ap.parse_args()
    md = markdown([analyze(p) for p in a.prefixes])
    print(md)
    if a.md:
        with open(a.md, "w") as f:
            f.write(md + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 4: Run.** `uv run clients/python/test_tag_sweep_report.py` prints `ok {…}`.
- [ ] **Step 5: Commit.** `git add clients/python/tag_sweep_report.py clients/python/test_tag_sweep_report.py && git commit -m "tag_sweep_report: per-speed error, lag, detections, loop health, joint acceleration"`, with the trailer.

### Task 9: The runs, the report and the runbook (controller)

**Files:** create `dev/so101-mujoco.md`; extend `docs/results/2026-11-phase4-tag-sweep.md`.

- [ ] **Step 1: Wrist sweep.** The wrist camera must already see the tag at the circle's start (280, 0) before `start_tracking`'s 10 s "no tag seen" timeout. So after a fresh `viam-server` start, pre-position the arm over the tag with the scene camera: `uv run clients/python/tag_servo.py --address localhost:8082 --insecure --follower arm --camera scene-cam --tracker scene-tags --axis down --hover-mm 150 --tip-mm <TIP_MM> --joint-deg-s 90 --linear-mm-s 120 --duration 5` (the tag must be at rest at (280, 0); `tag_sweep.py` places it there at start, or place it with `set_prop_pose`). Then: `uv run clients/python/tag_sweep.py --insecure --camera wrist-cam --tracker wrist-tags --csv .runs/p4-wrist`.
- [ ] **Step 2: Scene sweep.** `uv run clients/python/tag_sweep.py --insecure --camera scene-cam --tracker scene-tags --csv .runs/p4-scene`.
- [ ] **Step 3: Report.** `uv run clients/python/tag_sweep_report.py .runs/p4-wrist .runs/p4-scene --md .runs/p4-table.md`.
- [ ] **Step 4: Check against the bars, per camera.**
  - mean error ≤ 5 mm at 20 mm/s;
  - no tag loss up to and including 40 mm/s;
  - ≤ 1 overrun per 10 s;
  - 80 mm/s is reported only.

  If a bar fails, report it as it is. Don't tune to pass without telling the user.
- [ ] **Step 5: Write the results doc.** It contains:
  - the setup;
  - the Task 0 findings;
  - the table;
  - the summary fields (`mover_hz`, `accepted_det_hz`, breaking speed);
  - findings:
    - which camera loses the tag first, and why;
    - how much of the error is lag (error versus error at lag);
    - peak joint acceleration against the 360°/s² limit;
    - anything sim-specific that flatters the result.
- [ ] **Step 6: Write `dev/so101-mujoco.md`.** It covers:
  - building `bin/diff-ik`;
  - starting `viam-server -config dev/so101-mujoco-tag.json`;
  - the apriltag module path that worked;
  - the pre-positioning step;
  - the two sweep commands and the report command, from Steps 1–3.
- [ ] **Step 7: Commit.** `git add dev/so101-mujoco.md docs/results/2026-11-phase4-tag-sweep.md && git commit -m "Phase 4 results: moving-tag sweep, wrist vs scene camera"`, with the trailer. The `.runs/` directory stays untracked.

### Task 10: Finish (controller)

- [ ] **Step 1: Run everything.**
  - `go test ./...`;
  - `for f in clients/python/test_*.py; do uv run "$f" || echo "FAIL $f"; done`.

  All should pass.
- [ ] **Step 2: Squash-merge.**
  1. `git switch main && git merge --squash diff-ik-phase4`;
  2. commit as `Diff-IK Phase 4: moving-tag visual servoing in MuJoCo (wrist and scene cameras)`, with the trailer.

  Don't push.
- [ ] **Step 3: Report to the user.** Give the results table and the findings. Offer an Evidence row for the scope doc (Google Doc and Claude Doc), to add only on their approval.
