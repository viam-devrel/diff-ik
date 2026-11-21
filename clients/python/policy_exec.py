"""Policy chunk execution: pure helpers (top), executors below."""
import asyncio
import contextlib
import math
import time

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Slerp, Rotation
from viam.proto.common import Pose, PoseInFrame

import bc_pick
import playback
from bc_pick import in_bin
from policy_layout import pose_map_from, rotation

# --- helpers ---


def first_live_row(t_obs: float, dt: float, now: float) -> int:
    """Index of the first chunk row not yet due at `now` (a row due exactly now counts as live)."""
    return max(0, math.ceil((now - t_obs) / dt - 1e-9))


def gripper_events(gcol, t_obs: float, dt: float, closed: bool, lead: float) -> list:
    """(time, "grab"|"open") at each 0.5 crossing from the last commanded state, fired `lead` s early
    for actuation latency."""
    out = []
    for k, v in enumerate(gcol):
        want = v > 0.5
        if want != closed:
            out.append((t_obs + k * dt - lead, "grab" if want else "open"))
            closed = want
    return out


def jerk_rms(samples) -> float:
    """RMS 3-D jerk (mm/s^3) of (t, x, y, z) samples; NaN under 4 distinct timestamps.

    Resampled evenly so timing jitter doesn't blow up the difference (cubic spline: linear interpolation's
    kinks swamp the third difference). Only comparable between runs at the same sample rate, because
    position noise dominates at 30 Hz."""
    a = np.asarray(samples, dtype=float).reshape(-1, 4)
    a = a[np.unique(a[:, 0], return_index=True)[1]]  # sorted, first sample of each timestamp
    if len(a) < 4:
        return float("nan")
    t = np.linspace(a[0, 0], a[-1, 0], len(a))
    dt = t[1] - t[0]
    j = np.diff(CubicSpline(a[:, 0], a[:, 1:])(t), n=3, axis=0) / dt**3
    return float(np.sqrt((np.linalg.norm(j, axis=1) ** 2).mean()))


def succeeded(b1, b2, gripper_input: float) -> bool:
    """Block in the bin, at rest (under 1 mm between two reads about 0.3 s apart), and the gripper released."""
    return in_bin(b2) and math.dist(b1, b2) < 1.0 and gripper_input < 0.5


def sample_chunk(chunk: dict, t: float) -> dict:
    """Pose at time t along a chunk: linear position, slerped orientation, clamped to the ends."""
    p, n = chunk["poses"], len(chunk["poses"])
    s = min(max((t - chunk["t_obs"]) / chunk["dt"], 0.0), n - 1)
    k = min(int(math.floor(s)), max(n - 2, 0))
    return mix_pose(p[k], p[min(k + 1, n - 1)], s - k)


def mix_pose(a: dict, b: dict, w: float) -> dict:
    """Pose w of the way from a to b (w=0 is a, w=1 is b), for crossfading between chunks."""
    R = Slerp([0.0, 1.0], Rotation.concatenate([rotation(a), rotation(b)]))([w])[0]
    return pose_map_from([a[k] + w * (b[k] - a[k]) for k in "xyz"], R)


# --- executors ---


TICK_HZ = 30
START_LIMITS = {"linear_mm_s": 300, "angular_deg_s": 90}


class Executor:
    """Shared: servo start, gripper scheduling, message count. Subclasses implement play / wait_done / close."""
    # The lead already makes the arm reach row k at its time; firing the gripper lead early closes it a full lag
    # before the arm arrives (measured 13-24 mm short at grab). True keeps the old behaviour for comparison.
    grip_early = False

    def __init__(self, svc, gripper, arm_name):
        self.svc, self.gripper, self.arm_name = svc, gripper, arm_name
        self.msgs, self.errors = 0, []
        self.closed = False  # last commanded gripper state
        self.sid = None
        self._gtask = None
        self._chunk = None
        self.goal = None
        self._gcmd = None

    # _cmd counts arm commands in self.msgs; raw svc.do_command (status, stop) does not.
    async def _cmd(self, command, **kw):
        self.msgs += 1
        return await self.svc.do_command({"command": command, **kw})

    async def start(self, seed: int, time_scale: float = 1.0) -> None:
        """servo_start, min-jerk to the seed's start pose at 30 Hz, hold 0.5 s. time_scale < 1 shortens the
        motion and hold (test-only hook; leave at 1.0 for real runs)."""
        r = await self._cmd("servo_start", component_name=self.arm_name, frame="arm_origin", mode="pose", gain=8,
                            timeout_ms=3000, limits=START_LIMITS)
        self.sid, p0 = r["session_id"], r["pose"]
        goal = self.goal = bc_pick.start_pose(p0, seed)
        cur = (p0["x"], p0["y"], p0["z"], p0["theta"])
        dur = max(0.6, math.dist(cur[:3], goal[:3]) / 150.0, abs(bc_pick.angle_diff(goal[3], cur[3])) / 60.0)
        for p in bc_pick.minjerk_segment(cur, goal, dur * time_scale, TICK_HZ):
            t = time.monotonic()
            await self._cmd("servo_target", session_id=self.sid, pose=bc_pick.pose_map(p))
            await asyncio.sleep(max(0.0, 1 / TICK_HZ - (time.monotonic() - t)))
        end = time.monotonic() + 0.5 * time_scale
        while time.monotonic() < end:
            await self._cmd("servo_target", session_id=self.sid, pose=bc_pick.pose_map(goal))
            await asyncio.sleep(0.1 * time_scale)
        self.msgs = 0

    def _schedule_gripper(self, chunk: dict, lead: float) -> None:
        """Replace the gripper task with one firing this chunk's events from the last commanded state."""
        if self._gtask:
            self._gtask.cancel()
        events = gripper_events(chunk["grip"], chunk["t_obs"], chunk["dt"], self.closed, lead)
        self._gtask = asyncio.create_task(self._fire(events))

    async def _fire(self, events):
        for t, what in events:
            await asyncio.sleep(max(0.0, t - time.monotonic()))
            try:
                self.closed = what == "grab"  # last commanded state, set before the await
                # shield + kept reference: cancelling the scheduler never aborts an in-flight grab/open, and
                # _stop can await it. A flip back mid-command can overlap two gripper commands: rare, harmless.
                self._gcmd = asyncio.ensure_future(self.gripper.grab() if what == "grab" else self.gripper.open())
                await asyncio.shield(self._gcmd)
            except Exception as e:  # noqa: BLE001 - recorded, not raised
                self.errors.append(("gripper", repr(e)))

    async def wait_done(self) -> None:
        c = self._chunk
        if c is None:
            return
        await asyncio.sleep(max(0.0, c["t_obs"] + (len(c["poses"]) - 1) * c["dt"] - time.monotonic()))

    async def _cancel(self, task) -> None:
        if task:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
            except Exception as e:  # noqa: BLE001 - a stream that died of a bug must not make close raise
                self.errors.append(("task", repr(e)))

    @staticmethod
    def _terminal(state) -> bool:
        return str(state).startswith("stopped") or state == "faulted"

    async def _stop(self) -> dict:
        await self._cancel(self._gtask)
        if self._gcmd and not self._gcmd.done():  # let an in-flight grab/open finish before the world resets
            try:
                await asyncio.wait_for(self._gcmd, 2.0)
            except Exception as e:  # noqa: BLE001
                self._gcmd.cancel()  # still pending after the wait: don't leave it running
                self.errors.append(("gripper", repr(e)))
        elif self._gcmd and not self._gcmd.cancelled() and self._gcmd.exception():
            err = ("gripper", repr(self._gcmd.exception()))
            if err not in self.errors:  # _fire may have recorded it already
                self.errors.append(err)
        st = await self.svc.do_command({"command": "servo_status", "session_id": self.sid})
        state, peak = st.get("state"), st.get("peak_accel_ratio")
        if not self._terminal(state):
            ended = any(k == "session" for k, _ in self.errors)  # module is ramping down: a stop would overwrite
            if not ended:
                try:
                    state = (await self.svc.do_command({"command": "servo_stop", "session_id": self.sid})).get("state")
                except Exception:  # noqa: BLE001 - "session ended/stopping: ..."
                    ended = True
            if ended:
                for _ in range(15):  # re-poll to the terminal state, bounded at 1.5 s
                    st = await self.svc.do_command({"command": "servo_status", "session_id": self.sid})
                    state = st.get("state")
                    if self._terminal(state):
                        break
                    await asyncio.sleep(0.1)
                else:  # the error didn't end the session (transient/validation): stop it, or the next start is "busy"
                    try:
                        state = (await self.svc.do_command(
                            {"command": "servo_stop", "session_id": self.sid})).get("state")
                    except Exception:  # noqa: BLE001
                        state = f"{state} (not terminal)"
        return {"state": state, "peak_accel_ratio": peak, "msgs": self.msgs}


class ModuleExec(Executor):
    """One servo_trajectory per chunk; the module blends, ages and leads."""

    def __init__(self, svc, gripper, arm_name, lead_ms, blend_ms=200):
        super().__init__(svc, gripper, arm_name)
        self.lead_ms, self.blend_ms = lead_ms, blend_ms

    async def play(self, chunk: dict) -> None:
        self._chunk = chunk
        kw = {} if self.lead_ms is None else {"lead_ms": self.lead_ms}  # None: module uses its measured lag
        try:
            r = await self._cmd("servo_trajectory", session_id=self.sid, poses=chunk["poses"],
                                dt_ms=chunk["dt"] * 1000, age_ms=(time.monotonic() - chunk["t_obs"]) * 1000,
                                blend_ms=self.blend_ms, **kw)
        except Exception as e:
            self.errors.append(("session", repr(e)))
            raise
        self._schedule_gripper(chunk, r["lead_ms"] / 1000 if self.grip_early else 0.0)

    async def close(self) -> dict:
        return await self._stop()


class ClientExec(Executor):
    """30 Hz servo_target streaming, crossfading from the previous chunk."""

    def __init__(self, svc, gripper, arm_name, lead_ms, blend_ms=200):
        super().__init__(svc, gripper, arm_name)
        self.lead, self.blend_s = lead_ms / 1000, blend_ms / 1000
        self.cur = self.prev = None
        self.switch_t = 0.0
        self._task = None

    async def start(self, seed: int, time_scale: float = 1.0) -> None:
        await super().start(seed, time_scale)
        self._task = asyncio.create_task(self._stream())

    async def _stream(self):
        while True:
            t = time.monotonic()
            if self.cur is None:
                pose = bc_pick.pose_map(self.goal)  # feed the dead-man until the first chunk
            else:
                w = 1.0 if self.blend_s <= 0 else min(1.0, (t - self.switch_t) / self.blend_s)
                pose = playback.blended(self.cur, self.prev, t + self.lead, w, sample_chunk, mix_pose)
            try:
                await self._cmd("servo_target", session_id=self.sid, pose=pose)
            except Exception as e:  # noqa: BLE001
                self.errors.append(("session", repr(e)))
                return
            await asyncio.sleep(max(0.0, 1 / TICK_HZ - (time.monotonic() - t)))

    def _check(self):
        if self._task.done():
            raise RuntimeError(f"servo stream ended: {self.errors[-1:]}")

    async def wait_done(self) -> None:
        self._check()
        await super().wait_done()

    async def play(self, chunk: dict) -> None:
        self._check()
        self.prev, self.cur, self.switch_t, self._chunk = self.cur, chunk, time.monotonic(), chunk
        self._schedule_gripper(chunk, self.lead if self.grip_early else 0.0)

    async def close(self) -> dict:
        await self._cancel(self._task)
        return await self._stop()


class MoveExec(Executor):
    """One blocking builtin-motion Move per row; the gripper fires as each row's Move starts."""

    def __init__(self, motion, gripper, arm_name, servo_svc):
        super().__init__(servo_svc, gripper, arm_name)
        self.motion = motion
        self.queue, self.busy = [], False
        self._wake = asyncio.Event()
        self._worker = None

    async def start(self, seed: int, time_scale: float = 1.0) -> None:
        await super().start(seed, time_scale)
        await self.svc.do_command({"command": "servo_stop", "session_id": self.sid})
        self._worker = asyncio.create_task(self._run())

    async def _run(self):
        while True:
            if not self.queue:
                self._wake.clear()
                await self._wake.wait()
                continue
            pose, g = self.queue.pop(0)
            self.busy = True
            try:
                if (g > 0.5) != self.closed:
                    try:
                        await (self.gripper.grab() if g > 0.5 else self.gripper.open())
                        self.closed = g > 0.5
                    except Exception as e:  # noqa: BLE001
                        self.errors.append(("gripper", repr(e)))
                self.msgs += 1
                await self.motion.move(component_name=self.arm_name, destination=PoseInFrame(
                    reference_frame="arm_origin", pose=Pose(**{k: pose[k] for k in
                                                               ("x", "y", "z", "o_x", "o_y", "o_z", "theta")})))
            except Exception as e:  # noqa: BLE001
                self.errors.append(("plan", repr(e)))
            finally:
                self.busy = False

    async def play(self, chunk: dict) -> None:
        k = first_live_row(chunk["t_obs"], chunk["dt"], time.monotonic())
        self.queue = list(zip(chunk["poses"], chunk["grip"]))[k:]
        self._wake.set()

    async def wait_done(self) -> None:
        while self.queue or self.busy:
            await asyncio.sleep(0.01)

    async def close(self) -> dict:
        await self._cancel(self._worker)
        return {"state": "done", "peak_accel_ratio": None, "msgs": self.msgs}
