# /// script
# requires-python = ">=3.11"
# dependencies = ["viam-sdk", "numpy", "scipy", "pillow"]
# ///
"""Pure-path checks for policy_run.py. Run: uv run clients/python/test_policy_run.py"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from policy_run import episode_row, outcome_from, parse_args, parse_lead, seed_range  # noqa: E402

assert seed_range("9100-9102") == [9100, 9101, 9102] and seed_range("7") == [7]

assert outcome_from([], "x", True, False) == "success"
assert outcome_from([], "x", False, False) == "timeout"
assert outcome_from([("infer", "e")], None, False, True) == "infer"
assert outcome_from([("session", "e")], "stopped_timeout", True, False) == "stopped_timeout"
assert outcome_from([("plan", "e")], "ok", False, False) == "plan"
assert outcome_from([("gripper", "e")], "ok", True, False) == "gripper"
assert outcome_from([("gripper", "e"), ("plan", "e")], "ok", False, False) == "plan"

row = episode_row(seed=9100, mode="module", cadence="async", replan_every=10, lead_ms=None, outcome="success",
                  t_success_s=12.5, duration_s=10.0, close={"state": "stopped", "peak_accel_ratio": 0.4, "msgs": 250},
                  jerk=float("nan"), row0_to_state_mm=(1.0, 2.0), seam_gap_mm=[3.0], infer_s=[0.1, 0.2], chunks=2,
                  checkpoint="ck", errors=[("gripper", "boom")])
assert row["success"] and row["msgs_per_s"] == 25.0 and row["jerk_rms"] is None
assert row["row0_to_state_mm"] == [1.0, 2.0] and row["errors"] == [["gripper", "boom"]] and row["lead_ms"] is None
json.dumps(row, allow_nan=False)
assert not episode_row(**{**{k: row[k] for k in ("seed", "mode", "cadence", "replan_every", "lead_ms", "chunks")},
                          "outcome": "timeout", "t_success_s": None, "duration_s": 0, "close": {"msgs": 0},
                          "jerk": 1.0, "row0_to_state_mm": [], "seam_gap_mm": [], "infer_s": [],
                          "checkpoint": "", "errors": []})["success"]

assert parse_lead("auto", "module") is None and parse_lead("150", "move") == 150.0
try:
    parse_lead("auto", "client")
    raise AssertionError("auto accepted outside module mode")
except ValueError:
    pass
a = parse_args(["--address", "x", "--mode", "client", "--lead-ms", "80", "--out", "o.jsonl"])
assert a.lead_ms == 80.0 and a.mode == "client"
o = parse_args(["--address", "x", "--once", "--mode", "move"])
assert o.mode == "module" and o.cadence == "sync" and o.out is None
print("ok")
