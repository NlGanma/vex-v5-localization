#!/usr/bin/env python3
"""Drift / localization-quality analysis over the validation_data traces.

Treats the recorded runs as a high-fidelity proxy for how the robot moves under
the algorithm and quantifies, from the logged corrections, how much raw odometry
drifts and how much the filtered odometry (with the boundary re-anchor) improves.

Method, in one line: each row logs THREE poses for the same instant --
  odom_only  : pure dead reckoning, never corrected      (raw-odom baseline)
  odom       : the live pose that receives bounded fixes  (shipped system)
  ekf        : the filter's best estimate                 (truth proxy at accepts)
The accepted corrections are sensor-gated (low NIS, >=3 sensors, sub-inch
residuals), so at an accept `ekf` is a trustworthy stand-in for ground truth.
Raw-odom error  ~= |ekf - odom_only|. The boundary re-anchor snaps odom to ekf
at each between-segment stop, so its error resets there and only re-accumulates
within a single segment. We reconstruct that counterfactual and compare.

Outputs (under report/data/):
  macros.tex            - \newcommand numbers consumed by the LaTeX report
  err_<run>.dat         - t[s], raw-odom error, boundary-system error, accept flag
  traj_<run>.dat        - odom_only x/y and ekf x/y (downsampled) for the map plot
  axis_<run>.dat        - per-accept |corr_x| vs |corr_y| (axis breakdown)
  bars.dat              - per-run raw vs filtered max error + improvement factor
"""
from __future__ import annotations
import json, math
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DATA = ROOT / "validation_data"
OUT = ROOT / "report" / "data"
TRACE_MARKER = "=== LOCALIZATION TRACE CSV ==="
END_MARKER = "=== END LOCALIZATION TUNE LOG ==="
BOUNDARY_CAP_IN = 2.5  # maxBoundaryCorrectionXY shipped in localization_config.cpp
MOTION_SUPPRESSED_BIT = 1 << 9

RUNS = [
    ("run1", "Sample route (moving fusion)", "run1_sample_route_moving_fusion_log.txt"),
    ("run2", "Square loop, relocalized start", "run2_normal_start_relocalize_log.txt"),
    ("run3", "Square + cross (full field)", "run3_square_cross_full_log.txt"),
    ("run4", "One sensor view blocked", "run4_bad_view_fallback_log.txt"),
    ("run5", "Forced weak start (pre-guard)", "run5_forced_start_fallback_log.txt"),
    ("run6", "After two-sensor guard", "run6_after_two_sensor_guard_log.txt"),
    ("run7", "Post-guard clean start", "run7_post_guard_clean_start_log.txt"),
]


def load(path: Path):
    rows, hdr, inc = [], None, False
    for line in path.read_text(errors="replace").splitlines():
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
    return rows


def f(r, k, d=0.0):
    try:
        return float(r.get(k, d))
    except (TypeError, ValueError):
        return d


def i(r, k, d=0):
    try:
        return int(float(r.get(k, d)))
    except (TypeError, ValueError):
        return d


def analyze(rows):
    """Distance-based drift model.

    `ekf - odom_only` is flat between sensor fixes (both dead-reckon the same
    deltas), so the booked divergence is a staircase that only steps at accepts;
    its final value is a (conservative, lower-bound) estimate of the cumulative
    raw-odom drift. We convert it to a per-inch drift RATE and replay it along the
    measured path: raw odom resets only at the start anchor (relocalization),
    while the boundary re-anchor additionally resets at every accept. The error
    bound is then geometric -- proportional to the longest UN-anchored stretch of
    path -- and the improvement factor (total path / longest segment) does not
    depend on the rate estimate at all.
    """
    if not rows:
        return None
    t0 = f(rows[0], "time_ms")
    # ---- pass 1: path, booked divergence, accept anchors ----
    base = []  # (t, path, rex, rey, accepted, ox, oy, ex, ey)
    path_len = 0.0
    prev_ox = prev_oy = None
    motion_rows = 0
    accepts = []           # (t, |cx|, |cy|, mag, nis, active) at rising edges
    anchor_paths = [0.0]   # path positions where error resets (start + accept edges)
    prev_acc = False
    for r in rows:
        t = (f(r, "time_ms") - t0) / 1000.0
        ox, oy = f(r, "odom_only_x"), f(r, "odom_only_y")
        ex, ey = f(r, "ekf_x"), f(r, "ekf_y")
        if prev_ox is not None:
            path_len += math.hypot(ox - prev_ox, oy - prev_oy)
        prev_ox, prev_oy = ox, oy
        if i(r, "correction_reject_mask") & MOTION_SUPPRESSED_BIT:
            motion_rows += 1
        acc = i(r, "correction_accepted") == 1
        rex, rey = ex - ox, ey - oy
        if acc and not prev_acc:  # rising edge = one boundary re-anchor
            anchor_paths.append(path_len)
            accepts.append((t, abs(f(r, "target_corr_x")), abs(f(r, "target_corr_y")),
                            math.hypot(f(r, "target_corr_x"), f(r, "target_corr_y")),
                            f(r, "last_nis"), i(r, "active_sensors")))
        prev_acc = acc
        base.append((t, path_len, rex, rey, acc, ox, oy, ex, ey))

    booked_drift = math.hypot(base[-1][2], base[-1][3])  # |ekf-odom_only| at end
    rate = booked_drift / path_len if path_len > 1e-6 else 0.0  # in per in
    # segment lengths between consecutive anchors, plus the final tail to run-end
    bounds = anchor_paths + [path_len]
    seg_lens = [bounds[k + 1] - bounds[k] for k in range(len(bounds) - 1)]
    max_seg = max(seg_lens) if seg_lens else path_len
    tail_seg = seg_lens[-1] if seg_lens else path_len  # last anchor -> end

    # ---- pass 2: modeled error ramps (raw vs boundary re-anchor) ----
    series = []
    last_anchor = 0.0
    ai = 0
    for (t, p, rex, rey, acc, ox, oy, ex, ey) in base:
        while ai < len(accepts) and p >= anchor_paths[ai + 1] - 1e-9:
            last_anchor = anchor_paths[ai + 1]
            ai += 1
        raw_model = rate * p                  # only the start is an anchor
        new_model = rate * (p - last_anchor)  # resets at every accept
        series.append((t, raw_model, new_model, 1 if acc else 0, ox, oy, ex, ey))

    raw_max = rate * path_len
    new_max = rate * max_seg
    raw_end = raw_max
    new_end = rate * tail_seg
    dur = series[-1][0] - series[0][0] if series else 0.0
    corr_mags = [a[3] for a in accepts]
    return {
        "rows": len(rows),
        "duration_s": dur,
        "path_len_in": path_len,
        "accept_rows": sum(1 for b in base if b[4]),
        "accept_events": len(accepts),
        "motion_fraction": motion_rows / len(rows) if rows else 0.0,
        "drift_rate": rate,
        "booked_drift": booked_drift,
        "max_seg_in": max_seg,
        "raw_end": raw_end,
        "raw_max": raw_max,
        "new_end": new_end,
        "new_max": new_max,
        "shipped_end": booked_drift,  # shipped pose tracks odom_only -> ~booked drift off truth
        "net_applied_in": math.hypot(f(rows[-1], "odom_x") - f(rows[-1], "odom_only_x"),
                                     f(rows[-1], "odom_y") - f(rows[-1], "odom_only_y")),
        "corr_mean": sum(corr_mags) / len(corr_mags) if corr_mags else 0.0,
        "corr_max": max(corr_mags) if corr_mags else 0.0,
        "improve_max": (path_len / max_seg) if max_seg > 1e-6 else float("nan"),
        "improve_end": (raw_end / new_end) if new_end > 1e-6 else float("nan"),
        "drift_per_100in": rate * 100.0,
        "series": series,
        "accepts": accepts,
    }


def count_events(rows):
    """Distinct accept clusters (a boundary re-anchor fires once per cluster)."""
    n, prev = 0, False
    for r in rows:
        a = i(r, "correction_accepted") == 1
        if a and not prev:
            n += 1
        prev = a
    return n


def downsample(series, target=420):
    if len(series) <= target:
        return series
    step = len(series) / target
    keep, idx = [], 0.0
    picked = set()
    while idx < len(series):
        picked.add(int(idx)); idx += step
    out = []
    for k, row in enumerate(series):
        if k in picked or row[3] == 1:  # always keep accept rows
            out.append(row)
    return out


def fmt(x, n=2):
    if isinstance(x, float) and (math.isnan(x) or math.isinf(x)):
        return "--"
    return f"{x:.{n}f}"


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    results = {}
    for key, label, fname in RUNS:
        rows = load(DATA / fname)
        m = analyze(rows)
        results[key] = (label, m)

    # ---- per-run plot data ----
    for key, (label, m) in results.items():
        if not m:
            continue
        ds = downsample(m["series"])
        with (OUT / f"err_{key}.dat").open("w") as fh:
            fh.write("t raw new accept\n")
            for (t, raw, new, acc, *_ ) in ds:
                fh.write(f"{t:.3f} {raw:.4f} {new:.4f} {acc}\n")
        with (OUT / f"traj_{key}.dat").open("w") as fh:
            fh.write("ox oy ex ey\n")
            for (t, raw, new, acc, ox, oy, ex, ey) in downsample(m["series"], 600):
                fh.write(f"{ox:.3f} {oy:.3f} {ex:.3f} {ey:.3f}\n")
        with (OUT / f"axis_{key}.dat").open("w") as fh:
            fh.write("idx cx cy mag\n")
            for n, a in enumerate(m["accepts"]):
                fh.write(f"{n} {a[1]:.4f} {a[2]:.4f} {a[3]:.4f}\n")

    # ---- bar-chart data (runs that actually exercised the sensors) ----
    with (OUT / "bars.dat").open("w") as fh:
        fh.write("label raw new factor\n")
        for key in ("run2", "run3"):
            label, m = results[key]
            fh.write(f"{key} {m['raw_max']:.4f} {m['new_max']:.4f} {m['improve_max']:.3f}\n")

    # ---- aggregate over the two evidence runs ----
    ev = [results[k][1] for k in ("run2", "run3")]
    agg_raw_max = max(x["raw_max"] for x in ev)
    agg_corr_max = max(x["corr_max"] for x in ev)
    agg_new_max = max(x["new_max"] for x in ev)
    agg_factor = agg_raw_max / agg_new_max if agg_new_max > 1e-6 else float("nan")
    tot_accepts = sum(x["accept_events"] for x in ev)
    tot_rows = sum(results[k][1]["rows"] for k, _, _ in RUNS if results[k][1])

    # ---- LaTeX macros ----
    def mac(name, val):
        return f"\\newcommand{{\\{name}}}{{{val}}}\n"

    with (OUT / "macros.tex").open("w") as fh:
        fh.write("% auto-generated by tools/drift_analysis.py -- do not edit\n")
        fh.write(mac("nRuns", "7"))
        fh.write(mac("nTotalRows", f"{tot_rows:,}"))
        fh.write(mac("nEvidenceAccepts", f"{tot_accepts}"))
        fh.write(mac("aggRawMax", fmt(agg_raw_max)))
        fh.write(mac("aggCorrMax", fmt(agg_corr_max)))
        fh.write(mac("aggNewMax", fmt(agg_new_max)))
        fh.write(mac("aggFactor", fmt(agg_factor, 1)))
        fh.write(mac("boundaryCap", fmt(BOUNDARY_CAP_IN, 1)))
        # maxCorrectionXY shipped in localization_config.cpp (per 10 ms cycle;
        # the dt-scale clamp allows at most 2x = 0.030 in, observed max 0.0255 in)
        fh.write(mac("contCap", "0.015"))
        words = {"run1": "RunOne", "run2": "RunTwo", "run3": "RunThree",
                 "run4": "RunFour", "run5": "RunFive", "run6": "RunSix", "run7": "RunSeven"}
        for key, (label, m) in results.items():
            if not m:
                continue
            K = words[key]  # letter-only macro stems (LaTeX names cannot contain digits)
            fh.write(mac(f"{K}Rows", f"{m['rows']}"))  # no comma: siunitx S-column groups it
            fh.write(mac(f"{K}Dur", fmt(m["duration_s"], 1)))
            fh.write(mac(f"{K}Path", fmt(m["path_len_in"], 0)))
            fh.write(mac(f"{K}Accepts", f"{m['accept_events']}"))
            fh.write(mac(f"{K}MotionPct", fmt(m["motion_fraction"] * 100, 0)))
            fh.write(mac(f"{K}RawMax", fmt(m["raw_max"])))
            fh.write(mac(f"{K}RawEnd", fmt(m["raw_end"])))
            fh.write(mac(f"{K}NewMax", fmt(m["new_max"])))
            fh.write(mac(f"{K}NewEnd", fmt(m["new_end"])))
            fh.write(mac(f"{K}ShippedEnd", fmt(m["shipped_end"])))
            fh.write(mac(f"{K}CorrMean", fmt(m["corr_mean"])))
            fh.write(mac(f"{K}CorrMax", fmt(m["corr_max"])))
            fh.write(mac(f"{K}NetApplied", fmt(m["net_applied_in"], 3)))
            fh.write(mac(f"{K}Factor", fmt(m["improve_max"], 1)))
            fh.write(mac(f"{K}DriftPer", fmt(m["drift_per_100in"], 1)))

    # ---- console summary ----
    print(f"{'run':6} {'rows':>5} {'dur_s':>6} {'path_in':>8} {'accepts':>7} "
          f"{'mot%':>5} {'raw_max':>8} {'new_max':>8} {'factor':>7} {'corr_max':>8}")
    for key, (label, m) in results.items():
        if not m:
            continue
        print(f"{key:6} {m['rows']:>5} {m['duration_s']:>6.1f} {m['path_len_in']:>8.0f} "
              f"{m['accept_events']:>7} {m['motion_fraction']*100:>5.0f} "
              f"{m['raw_max']:>8.2f} {m['new_max']:>8.2f} {m['improve_max']:>7.1f} "
              f"{m['corr_max']:>8.2f}")
    print(f"\nAGGREGATE (run2+run3): raw_max={agg_raw_max:.2f} in  new_max={agg_new_max:.2f} in  "
          f"factor={agg_factor:.1f}x  accepts={tot_accepts}")
    (OUT / "metrics.json").write_text(json.dumps(
        {k: {kk: vv for kk, vv in v[1].items() if kk not in ("series", "accepts")}
         for k, v in results.items() if v[1]}, indent=2))


if __name__ == "__main__":
    main()
