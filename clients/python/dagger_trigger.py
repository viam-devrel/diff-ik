"""Takeover triggers for expert-gated DAgger (spec docs/superpowers/specs/2026-11-22-diff-ik-policy-dagger-design.md).

A state is {"t": s since the policy started, "tcp": world mm, "block": world mm, "grip_cmd": 0 or 1,
"holding": firm grasp}; hist is the earlier states, oldest first (10 Hz). hist must cover the whole rollout
since t0 (the drop scan and the stall windows rely on it; the caller keeps every sample). After a missed read,
"consecutive samples" may be 0.2-0.3 s apart, which only makes the checks more conservative."""
import math
from dataclasses import dataclass

from bc_pick import GRASP_ABOVE_CENTER, RELEASE_TCP, in_bin


@dataclass(frozen=True)
class Cfg:
    stall_s: float = 2.0
    stall_mm: float = 10.0
    near_mm: float = 15.0
    release_xy_mm: float = 40.0
    empty_s: float = 2.5
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


def _window(state, hist, ref):
    """The states from ref to now, inclusive."""
    return [h for h in hist if h["t"] >= ref["t"]] + [state]


def check(state, hist, cfg: Cfg = Cfg()):
    """The trigger kind, or None. Order: drop, empty_grasp, approach_stall, carry_stall, timeout."""
    prev = hist[-1] if hist else None
    at_rest = prev is not None and math.dist(prev["block"], state["block"]) < cfg.rest_mm
    if not state["holding"] and at_rest and not in_bin(state["block"]) and any(h["holding"] for h in hist) \
            and math.dist(state["tcp"], grasp_point(state["block"])) > cfg.near_mm:
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
        win = _window(state, hist, ref)
        if not any(h["holding"] for h in win) and at_rest and not in_bin(state["block"]):
            d = math.dist(state["tcp"], grasp_point(state["block"]))
            d0 = math.dist(ref["tcp"], grasp_point(ref["block"]))
            if d > cfg.near_mm and d0 - d < cfg.stall_mm:
                return "approach_stall"
        if all(h["holding"] for h in win) and math.dist(state["tcp"][:2], RELEASE_TCP[:2]) > cfg.release_xy_mm:
            if math.dist(ref["tcp"], RELEASE_TCP) - math.dist(state["tcp"], RELEASE_TCP) < cfg.stall_mm:
                return "carry_stall"
    if state["t"] >= cfg.timeout_s - 1e-9:
        return "timeout"
    return None
