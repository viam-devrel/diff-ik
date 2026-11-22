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
