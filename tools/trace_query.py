#!/usr/bin/env python3
"""Ad-hoc query tool for localization validation trace logs.

Parses the CSV embedded between '=== LOCALIZATION TRACE CSV ===' and
'=== END LOCALIZATION TUNE LOG ===' in a validation_data/runN_*.txt file and
lets you filter/aggregate rows. Built for empirical validation of the
localization/odom/EKF/MCL stack against real-world runs.

Examples:
  trace_query.py FILE --accepted --cols time_ms,target_corr_y,applied_corr_y
  trace_query.py FILE --filter "correction_accepted==1 and motion_suppressed==1"
  trace_query.py FILE --stats applied_corr_y
  trace_query.py FILE --reject-bits          # decode correction_reject_mask histogram
The reject-mask bit names match src/lemlib/localization/localization.cpp.
"""
from __future__ import annotations
import argparse, math
from pathlib import Path

TRACE_MARKER = "=== LOCALIZATION TRACE CSV ==="
END_MARKER = "=== END LOCALIZATION TUNE LOG ==="

# Mirror of kReject* enum in src/lemlib/localization/localization.cpp:46-55
REJECT_BITS = {
    1 << 0: "turn_suppressed", 1 << 1: "sensors_stale", 1 << 2: "not_enough_sensors",
    1 << 3: "unstable_candidate", 1 << 4: "pose_delta", 1 << 5: "readiness",
    1 << 6: "nis", 1 << 7: "no_measurement", 1 << 8: "sensor_residual",
    1 << 9: "motion_suppressed",
}


def load(path: Path):
    rows, hdr, inc = [], None, False
    for line in Path(path).read_text(errors="replace").splitlines():
        s = line.strip()
        if s == TRACE_MARKER:
            inc = True; continue
        if s == END_MARKER:
            break
        if not inc:
            continue
        if hdr is None and s.startswith("time_ms,"):
            hdr = s.split(","); continue
        if hdr and s and not s.startswith("#"):
            parts = s.split(",")
            if len(parts) == len(hdr):
                rows.append(dict(zip(hdr, parts)))
    return hdr, rows


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return math.nan


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("file", type=Path)
    ap.add_argument("--accepted", action="store_true", help="only correction_accepted==1 rows")
    ap.add_argument("--cols", default=None, help="comma list of columns to print")
    ap.add_argument("--filter", default=None, help="python expr over numeric columns")
    ap.add_argument("--stats", default=None, help="column to summarize (min/max/mean/absmax)")
    ap.add_argument("--reject-bits", action="store_true", help="histogram of reject mask bits")
    ap.add_argument("--limit", type=int, default=0, help="max rows to print (0=all)")
    a = ap.parse_args()
    hdr, rows = load(a.file)
    print(f"# {a.file.name}: {len(rows)} trace rows, {len(hdr or [])} columns")

    sel = rows
    if a.accepted:
        sel = [r for r in sel if r.get("correction_accepted") == "1"]
    if a.filter:
        def ok(r):
            env = {k: num(v) for k, v in r.items()}
            try:
                return bool(eval(a.filter, {"__builtins__": {}}, env))
            except Exception:
                return False
        sel = [r for r in sel if ok(r)]
    print(f"# matched {len(sel)} rows")

    if a.reject_bits:
        hist = {name: 0 for name in REJECT_BITS.values()}
        masks = 0
        for r in rows:
            m = int(num(r.get("correction_reject_mask", 0)) or 0)
            if m:
                masks += 1
                for bit, name in REJECT_BITS.items():
                    if m & bit:
                        hist[name] += 1
        print(f"# rows with any reject bit: {masks}")
        for name, c in sorted(hist.items(), key=lambda kv: -kv[1]):
            print(f"  {name:20s} {c}")
        return 0

    if a.stats:
        vals = [num(r.get(a.stats)) for r in sel]
        vals = [v for v in vals if not math.isnan(v)]
        if vals:
            print(f"# {a.stats}: n={len(vals)} min={min(vals):.4f} max={max(vals):.4f} "
                  f"mean={sum(vals)/len(vals):.4f} absmax={max(abs(v) for v in vals):.4f}")
        else:
            print(f"# {a.stats}: no numeric values")
        return 0

    cols = a.cols.split(",") if a.cols else (hdr[:8] if hdr else [])
    print(" ".join(f"{c[:12]:>13}" for c in cols))
    out = sel if a.limit == 0 else sel[: a.limit]
    for r in out:
        print(" ".join(f"{r.get(c,'?')[:12]:>13}" for c in cols))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
