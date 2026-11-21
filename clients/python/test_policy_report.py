# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""Checks for policy_report.py. Run: uv run clients/python/test_policy_report.py"""
import json
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from policy_report import group, labels, load, paired, summary_row  # noqa: E402


def row(seed, mode, outcome, t, jerk, row0=(5.0,), seam=(1.0,), msgs=10.0, accel=None):
    return {"seed": seed, "mode": mode, "cadence": "async", "checkpoint": "ck", "success": outcome == "success",
            "outcome": outcome, "t_success_s": t, "jerk_rms": jerk, "row0_to_state_mm": list(row0),
            "seam_gap_mm": list(seam), "msgs_per_s": msgs, "peak_accel_ratio": accel}


rows = [
    row(0, "module", "success", 4.0, 100.0, seam=(1, 2, 3), accel=1.5),
    row(1, "module", "success", 6.0, 200.0, seam=(4,), accel=None),
    row(2, "module", "timeout", None, None),
    row(0, "client", "success", 5.0, 300.0),
    row(1, "client", "infer", None, 250.0),
    row(3, "client", "success", 9.0, None),
]
with tempfile.TemporaryDirectory() as d:
    p = os.path.join(d, "a.jsonl")
    with open(p, "w") as f:
        f.write("\n".join(json.dumps(r) for r in rows[:3]) + "\n\n")
    q = os.path.join(d, "b.jsonl")
    with open(q, "w") as f:
        f.write("\n".join(json.dumps(r) for r in rows[3:]) + "\n")
    assert load([p, q]) == rows

g = group(rows)
assert list(g) == [("ck", "client", "async"), ("ck", "module", "async")]
m = summary_row(g[("ck", "module", "async")])
assert m[0] == "3" and m[1] == "2 (67%)", m
assert m[2] == "5.00", m  # median of 4, 6
assert m[3] == "150", m  # null jerk ignored
assert m[4] == "2.0", m  # pooled seam 1,2,3,4,1
assert m[7] == "10.0" and m[8] == "1.50", m  # null accel ignored
assert m[9] == "success 2, timeout 1", m
c = summary_row(g[("ck", "client", "async")])
assert c[9] == "success 2, infer 1" and c[3] == "275", c

assert labels(g) == {k: f"{k[1]}/async" for k in g}
cells, lines = paired(g)
assert cells == {0: {"client/async": True, "module/async": True},
                 1: {"client/async": False, "module/async": True},
                 2: {"module/async": False},
                 3: {"client/async": True}}, cells
# shared seeds with both jerks: 0 (300-100) and 1 (250-200) -> median 125, client lower on 0/2
assert lines == ["client/async vs module/async: median Δjerk 125 (client/async lower on 0/2 seeds)"], lines
print("ok")
