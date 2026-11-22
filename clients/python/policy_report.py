#!/usr/bin/env python3
# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""Summarise policy_run JSONL rows. Run: uv run clients/python/policy_report.py <jsonl>..."""
import argparse
import itertools
import json
import sys
import statistics as st
from collections import Counter

DASH = "—"


def load(paths):
    rows = []
    for p in paths:
        with open(p) as f:
            rows += [json.loads(line) for line in f if line.strip()]
    return rows


def group(rows):
    """{(checkpoint, mode, cadence): [rows]}, sorted by key. Re-runs: first row per seed wins."""
    g, dup = {}, 0
    for r in rows:
        rs = g.setdefault((str(r.get("checkpoint") or "-"), r["mode"], r["cadence"]), [])
        if any(x["seed"] == r["seed"] for x in rs):
            dup += 1
        else:
            rs.append(r)
    if dup:
        print(f"warning: dropped {dup} duplicate-seed row(s), kept the first", file=sys.stderr)
    return dict(sorted(g.items()))


def _med(xs, fmt="{:.1f}"):
    return fmt.format(st.median(xs)) if xs else DASH


def _p95(xs):
    return "{:.1f}".format(st.quantiles(xs, n=20, method="inclusive")[-1]) if len(xs) > 1 else _med(xs)


def summary_row(rows):
    n = len(rows)
    ok = sum(bool(r["success"]) for r in rows)
    # seam percentiles pool all seams across episodes
    seam = [x for r in rows for x in r["seam_gap_mm"]]
    row0 = [r["row0_to_state_mm"][0] for r in rows if r["row0_to_state_mm"]]
    msgs = [r["msgs_per_s"] for r in rows if r["msgs_per_s"] is not None]
    nn = lambda k: [r[k] for r in rows if r[k] is not None]  # noqa: E731
    oc = Counter(r["outcome"] for r in rows)
    return [
        str(n),
        f"{ok} ({100 * ok / n:.0f}%)" if n else DASH,
        _med([r["t_success_s"] for r in rows if r["success"] and r["t_success_s"] is not None], "{:.2f}"),
        _med(nn("jerk_rms"), "{:.0f}"),
        _med(seam),
        _p95(seam),
        _med(row0),
        f"{st.mean(msgs):.1f}" if msgs else DASH,
        _med(nn("peak_accel_ratio"), "{:.2f}"),
        ", ".join(f"{k} {v}" for k, v in oc.most_common()),
    ]


HEADER = ["group", "episodes", "successes", "t_success med (s)", "jerk med", "seam med (mm)", "seam p95 (mm)",
          "row0 med (mm)", "msgs/s", "peak accel med", "outcomes"]


def labels(keys):
    """mode/cadence, prefixed by checkpoint only if checkpoints differ."""
    multi = len({k[0] for k in keys}) > 1
    return {k: "/".join(k if multi else k[1:]) for k in keys}


# (name, per-episode value or None, delta format)
METRICS = [
    ("jerk", lambda r: r["jerk_rms"], "{:.0f}"),
    ("seam", lambda r: st.median(r["seam_gap_mm"]) if r["seam_gap_mm"] else None, "{:.1f}"),
    ("row0", lambda r: r["row0_to_state_mm"][0] if r["row0_to_state_mm"] else None, "{:.1f}"),
    ("msgs/s", lambda r: r["msgs_per_s"], "{:.1f}"),
]


def paired(rows_by_group):
    """(cells {seed: {label: bool}}, metric lines). Seeds are matched across groups."""
    lab = labels(rows_by_group)
    by = {lab[k]: {r["seed"]: r for r in rs} for k, rs in rows_by_group.items()}
    cells = {s: {l: d[s]["success"] for l, d in by.items() if s in d} for s in sorted({s for d in by.values() for s in d})}
    lines = []
    for a, b in itertools.combinations(by, 2):
        for name, get, fmt in METRICS:
            pairs = [(get(by[a][s]), get(by[b][s])) for s in by[a].keys() & by[b].keys()]
            diffs = [x - y for x, y in pairs if x is not None and y is not None]
            if diffs:
                lines.append(f"{a} vs {b}: median Δ{name} {fmt.format(st.median(diffs))} "
                             f"({a} lower on {sum(d < 0 for d in diffs)}/{len(diffs)} seeds)")
    return cells, lines


def table(header, body):
    return "\n".join(["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
                     + ["| " + " | ".join(r) + " |" for r in body])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs="+")
    g = group(load(ap.parse_args().paths))
    lab = labels(g)
    print("## Summary\n")
    print(table(HEADER, [[lab[k]] + summary_row(rs) for k, rs in g.items()]))
    if len(g) >= 2:
        cells, lines = paired(g)
        names = list(lab.values())
        print("\n## Paired per seed\n")
        print(table(["seed"] + names, [[str(s)] + [("" if n not in c else "✓" if c[n] else "✗") for n in names]
                                       for s, c in cells.items()]))
        if lines:
            print("\n## Paired metrics (Δ = first − second; negative = first lower)\n")
            print("\n".join(f"- {l}" for l in lines))


if __name__ == "__main__":
    main()
