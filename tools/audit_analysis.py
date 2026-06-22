#!/usr/bin/env python3
"""Adversarial audit pass over validation_data + tune.txt.

Independent of drift_analysis.py: re-derives every report number from the raw
logs and cross-checks the firmware math (expected-distance raycast model, wall
solve, correction caps, gating invariants) against what the robot actually
logged. Prints findings; writes audit_metrics.json next to this script.
"""
from __future__ import annotations
import json, math, re, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DATA = ROOT / "validation_data"
TRACE_MARKER = "=== LOCALIZATION TRACE CSV ==="
END_MARKER = "=== END LOCALIZATION TUNE LOG ==="
MOTION_SUPPRESSED_BIT = 1 << 9
REJECT_BITS = {
    1 << 0: "turn_suppressed", 1 << 1: "sensors_stale", 1 << 2: "not_enough_sensors",
    1 << 3: "unstable_candidate", 1 << 4: "pose_delta", 1 << 5: "readiness",
    1 << 6: "nis", 1 << 7: "no_measurement", 1 << 8: "sensor_residual",
    1 << 9: "motion_suppressed",
}
SENSORS = ["front", "right", "back", "left"]

RUNS = [
    ("run1", "run1_sample_route_moving_fusion_log.txt"),
    ("run2", "run2_normal_start_relocalize_log.txt"),
    ("run3", "run3_square_cross_full_log.txt"),
    ("run4", "run4_bad_view_fallback_log.txt"),
    ("run5", "run5_forced_start_fallback_log.txt"),
    ("run6", "run6_after_two_sensor_guard_log.txt"),
    ("run7", "run7_post_guard_clean_start_log.txt"),
    ("tune", "../src/tune.txt"),
]


def fnum(v, d=float("nan")):
    try:
        return float(v)
    except (TypeError, ValueError):
        return d


def parse_log(path: Path):
    text = path.read_text(errors="replace").replace("\r", "")
    meta = {}
    for key in ("test_case", "status", "start_pose", "relocalize_attempted", "relocalize_success",
                "relocalize_status", "relocalize_pose", "relocalize_conf", "relocalize_residual",
                "relocalize_var", "checkpoint_count"):
        m = re.search(rf"^{key}=(.*)$", text, re.M)
        if m:
            meta[key] = m.group(1).strip()
    m = re.search(r"^relocalize_heading mode=(\S+) imu=([-\d.]+) hypotheses=(\d+)$", text, re.M)
    if m:
        meta["heading_mode"], meta["imu_heading_deg"], meta["hypotheses"] = m.group(1), float(m.group(2)), int(m.group(3))
    m = re.search(r"^relocalize_conf=([\d.]+) active=(\d+) scored=(\d+)", text, re.M)
    if m:
        meta["reloc_conf"], meta["reloc_active"], meta["reloc_scored"] = float(m.group(1)), int(m.group(2)), int(m.group(3))
    m = re.search(r"Compiled:\s+(.*)$", text, re.M)
    if m:
        meta["compiled"] = m.group(1).strip()

    # relocalize sensor snapshot
    sens = {}
    m = re.search(r"^relocalize_sensors F ([-\d.]+)/(-?\d+) R ([-\d.]+)/(-?\d+)$", text, re.M)
    if m:
        sens["front"] = (float(m.group(1)), int(m.group(2)))
        sens["right"] = (float(m.group(3)), int(m.group(4)))
    m = re.search(r"^relocalize_sensors B ([-\d.]+)/(-?\d+) L ([-\d.]+)/(-?\d+)$", text, re.M)
    if m:
        sens["back"] = (float(m.group(1)), int(m.group(2)))
        sens["left"] = (float(m.group(3)), int(m.group(4)))
    meta["reloc_sensors"] = sens

    # checkpoints: blocks "[N] name\nExp x y th\nRep x y th\nErr x y th\n..."
    checkpoints = []
    for cm in re.finditer(
            r"^\[(\d+)\] (.+)\nExp ([-\d. ]+)\nRep ([-\d. ]+)\nErr ([-\d. ]+)\nSeq (\d+)\n"
            r"F (.+)\nB (.+)\nA(\d+) C([-\d.]+) OK(\d)\n"
            r"Gate mask=(\d+) dxy=([-\d.]+) dth=([-\d.]+) stable=(\d+)/(\d+) inl=(\d+) res=([-\d.]+)/([-\d.]+)\n"
            r"Fus ([-\d. ]+)\nMCL ([-\d. ]+)\nNIS ([-\d.]+) ESS ([-\d.]+)", text, re.M):
        exp = [float(x) for x in cm.group(3).split()]
        rep = [float(x) for x in cm.group(4).split()]
        err = [float(x) for x in cm.group(5).split()]
        checkpoints.append({
            "idx": int(cm.group(1)), "name": cm.group(2).strip(), "exp": exp, "rep": rep, "err": err,
            "active": int(cm.group(9)), "conf": float(cm.group(10)), "accepted": int(cm.group(11)),
            "mask": int(cm.group(12)), "stable": cm.group(15) + "/" + cm.group(16),
            "inl": int(cm.group(17)), "nis": float(cm.group(22)),
        })
    meta["checkpoints"] = checkpoints

    # sensor model lines from the trace header
    sensor_model = {}
    for sm in re.finditer(r"^# sensor_model,(\d+),([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+),(-?\d+)$", text, re.M):
        sensor_model[int(sm.group(1))] = dict(
            dx=float(sm.group(2)), dy=float(sm.group(3)), dtheta_deg=float(sm.group(4)),
            min_range=float(sm.group(5)), max_range=float(sm.group(6)), min_conf=int(sm.group(7)))
    meta["sensor_model"] = sensor_model
    fb = re.search(r"^# field_bounds,([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+)$", text, re.M)
    if fb:
        meta["field"] = [float(fb.group(k)) for k in range(1, 7)]
    obstacles = []
    for om in re.finditer(r"^# obstacle,(\d+),(\w+),([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+)$", text, re.M):
        obstacles.append(dict(type=om.group(2), x=float(om.group(3)), y=float(om.group(4)),
                              radius=float(om.group(5)), halfW=float(om.group(6)), halfH=float(om.group(7))))
    meta["obstacles"] = obstacles

    # trace rows
    rows, hdr, inc = [], None, False
    for line in text.splitlines():
        s = line.strip()
        if s == TRACE_MARKER:
            inc = True
            continue
        if s == END_MARKER:
            break
        if not inc:
            continue
        if hdr is None and s.startswith("time_ms,"):
            hdr = s.split(",")
            continue
        if hdr and s and not s.startswith("#"):
            parts = s.split(",")
            if len(parts) == len(hdr):
                rows.append(dict(zip(hdr, parts)))
    return meta, hdr, rows


# ---------------- firmware math mirrors (for cross-validation) ----------------

def expected_distance(field, obstacles, sensors, pose, si):
    minX, maxX, minY, maxY = field[0], field[1], field[2], field[3]
    s = sensors[si]
    th = pose[2]
    sin_t, cos_t = math.sin(th), math.cos(th)
    sx = pose[0] + s["dx"] * cos_t + s["dy"] * sin_t
    sy = pose[1] - s["dx"] * sin_t + s["dy"] * cos_t
    hd = th + math.radians(s["dtheta_deg"])
    dx, dy = math.sin(hd), math.cos(hd)
    best = 1e9
    eps = 1e-6
    if abs(dx) > eps:
        for wall in (minX, maxX):
            t = (wall - sx) / dx
            y = sy + t * dy
            if t > 0 and minY <= y <= maxY:
                best = min(best, t)
    if abs(dy) > eps:
        for wall in (minY, maxY):
            t = (wall - sy) / dy
            x = sx + t * dx
            if t > 0 and minX <= x <= maxX:
                best = min(best, t)
    for ob in obstacles:
        if ob["type"] == "rect":
            rminX, rmaxX = ob["x"] - ob["halfW"], ob["x"] + ob["halfW"]
            rminY, rmaxY = ob["y"] - ob["halfH"], ob["y"] + ob["halfH"]
            tmin, tmax = -1e9, 1e9
            ok = True
            if abs(dx) < eps:
                if sx < rminX or sx > rmaxX:
                    ok = False
            else:
                t1, t2 = (rminX - sx) / dx, (rmaxX - sx) / dx
                tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
            if ok and abs(dy) < eps:
                if sy < rminY or sy > rmaxY:
                    ok = False
            elif ok:
                t1, t2 = (rminY - sy) / dy, (rmaxY - sy) / dy
                tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
            hit = -1.0
            if ok and tmax >= 0 and tmin <= tmax:
                hit = tmin if tmin >= 0 else tmax
            if hit > 0:
                best = min(best, hit)
        else:
            ddx, ddy = sx - ob["x"], sy - ob["y"]
            a = dx * dx + dy * dy
            b = 2 * (ddx * dx + ddy * dy)
            c = ddx * ddx + ddy * ddy - ob["radius"] ** 2
            disc = b * b - 4 * a * c
            if disc >= 0:
                r = math.sqrt(disc)
                for t in ((-b - r) / (2 * a), (-b + r) / (2 * a)):
                    if t > 0:
                        best = min(best, t)
    return -1.0 if best >= 1e8 else best


def classify_axis(heading):
    h = math.atan2(math.sin(heading), math.cos(heading))
    deg30 = math.radians(30)
    if abs(h) <= deg30:
        return "PosY"
    if abs(abs(h) - math.pi) <= deg30:
        return "NegY"
    if abs(h - math.pi / 2) <= deg30:
        return "PosX"
    if abs(h + math.pi / 2) <= deg30:
        return "NegX"
    return None


def solve_wall(field, sensors, snapshot, heading, min_conf_override=12, max_range_override=96.0):
    """Mirror of autonomous_localization::solveDirectWallPose best-subset=all-usable."""
    minX, maxX, minY, maxY = field[0], field[1], field[2], field[3]
    sin_t, cos_t = math.sin(heading), math.cos(heading)
    x_terms, y_terms, used = [], [], []
    for i, name in enumerate(SENSORS):
        dist, conf = snapshot.get(name, (-1.0, -1))
        s = sensors[i]
        min_conf = min(s["min_conf"], min_conf_override)
        max_range = max(s["max_range"], max_range_override)
        if dist < s["min_range"] or dist > max_range or dist <= 0:
            continue
        if conf >= 0 and conf < min_conf:
            continue
        shead = heading + math.radians(s["dtheta_deg"])
        axis = classify_axis(shead)
        if axis is None:
            continue
        sgx = s["dx"] * cos_t + s["dy"] * sin_t
        sgy = -s["dx"] * sin_t + s["dy"] * cos_t
        rdx, rdy = math.sin(shead), math.cos(shead)
        if axis == "PosX":
            x_terms.append(maxX - rdx * dist - sgx)
        elif axis == "NegX":
            x_terms.append(minX - rdx * dist - sgx)
        elif axis == "PosY":
            y_terms.append(maxY - rdy * dist - sgy)
        else:
            y_terms.append(minY - rdy * dist - sgy)
        used.append(name)
    if not x_terms or not y_terms:
        return None
    return (sum(x_terms) / len(x_terms), sum(y_terms) / len(y_terms), heading, used)


# ---------------- per-run analysis ----------------

def mcl_scan_groups(rows):
    """Group trace rows into MCL-scan clusters: a new scan when the latched MCL
    fields change. Returns list of (first_row_index, row)."""
    scans = []
    prev_key = None
    for k, r in enumerate(rows):
        key = (r.get("last_nis"), r.get("mcl_ess"), r.get("mcl_x"), r.get("mcl_y"),
               r.get("candidate_stable_scans"), r.get("correction_reject_mask"),
               r.get("correction_accepted"), r.get("active_sensors"))
        if key != prev_key:
            scans.append((k, r))
            prev_key = key
    return scans


def analyze_run(name, meta, rows):
    out = {"name": name, "meta": {k: v for k, v in meta.items() if k not in ("checkpoints", "sensor_model", "obstacles", "reloc_sensors", "field")}}
    if not rows:
        out["rows"] = 0
        return out
    t0 = fnum(rows[0]["time_ms"])
    dur = (fnum(rows[-1]["time_ms"]) - t0) / 1000.0

    # path length over odom_only
    path = 0.0
    px = py = None
    motion_rows = 0
    seq_gaps = []
    seq_prev = None
    stair_viol = 0.0
    div_prev = None
    accepts = []
    prev_acc = False
    max_applied = 0.0
    applied_total = 0.0
    cap_viol = 0
    div_series = []
    for k, r in enumerate(rows):
        ox, oy = fnum(r["odom_only_x"]), fnum(r["odom_only_y"])
        ex, ey = fnum(r["ekf_x"]), fnum(r["ekf_y"])
        if px is not None:
            path += math.hypot(ox - px, oy - py)
        px, py = ox, oy
        mask = int(fnum(r["correction_reject_mask"], 0))
        if mask & MOTION_SUPPRESSED_BIT:
            motion_rows += 1
        seq = int(fnum(r["odom_seq"], 0))
        if seq_prev is not None and seq > seq_prev + 3:
            seq_gaps.append((k, seq_prev, seq))
        seq_prev = seq
        acc = r.get("correction_accepted") == "1"
        div = math.hypot(ex - ox, ey - oy)
        div_series.append(((fnum(r["time_ms"]) - t0) / 1000.0, div, path, acc))
        if div_prev is not None and not acc:
            # staircase property: divergence should not change off-accept
            stair_viol = max(stair_viol, abs(div - div_prev))
        div_prev = div
        if acc and not prev_acc:
            accepts.append(dict(
                t=(fnum(r["time_ms"]) - t0) / 1000.0, path=path,
                cx=fnum(r["target_corr_x"]), cy=fnum(r["target_corr_y"]),
                mag=math.hypot(fnum(r["target_corr_x"]), fnum(r["target_corr_y"])),
                nis=fnum(r["last_nis"]), active=int(fnum(r["active_sensors"])),
                inl=int(fnum(r["candidate_inlier_sensors"])),
                mean_res=fnum(r["candidate_mean_residual"]), max_res=fnum(r["candidate_max_residual"]),
            ))
        prev_acc = acc
        ap = math.hypot(fnum(r["applied_corr_x"]), fnum(r["applied_corr_y"]))
        applied_total += ap
        max_applied = max(max_applied, ap)
        if ap > 0.0301:  # maxCorrectionXY * dtScale_max = 0.015*2 with float slop
            cap_viol += 1

    booked = div_series[-1][1]
    rate = booked / path if path > 1e-6 else 0.0
    anchor_paths = [0.0] + [a["path"] for a in accepts]
    bounds = anchor_paths + [path]
    seg_lens = [bounds[i + 1] - bounds[i] for i in range(len(bounds) - 1)]
    max_seg = max(seg_lens) if seg_lens else path
    tail = seg_lens[-1] if seg_lens else path

    # MCL scan-level stats
    scans = mcl_scan_groups(rows)
    n_scan = len(scans)
    acc_scans = sum(1 for _, r in scans if r.get("correction_accepted") == "1")
    mask_hist = {}
    nis_vals = []
    for _, r in scans:
        mask = int(fnum(r["correction_reject_mask"], 0))
        for bit, bname in REJECT_BITS.items():
            if mask & bit:
                mask_hist[bname] = mask_hist.get(bname, 0) + 1
        nis = fnum(r["last_nis"])
        if nis >= 0 and r.get("mcl_valid") == "1":
            nis_vals.append(nis)
    nis_vals.sort()

    def pct(p):
        if not nis_vals:
            return float("nan")
        idx = min(len(nis_vals) - 1, int(p * len(nis_vals)))
        return nis_vals[idx]

    # sensor stats on rows where used
    sensor_stats = {}
    for sname in SENSORS:
        used_res = [abs(fnum(r[f"{sname}_residual"])) for r in rows
                    if r.get(f"{sname}_used") == "1" and fnum(r[f"{sname}_expected"]) > 0]
        used_n = sum(1 for r in rows if r.get(f"{sname}_used") == "1")
        if used_res:
            used_res.sort()
            sensor_stats[sname] = dict(
                used_rows=used_n, med_abs_res=used_res[len(used_res) // 2],
                p90_abs_res=used_res[int(0.9 * (len(used_res) - 1))], max_abs_res=used_res[-1])
        else:
            sensor_stats[sname] = dict(used_rows=used_n)

    # cross-check expected-distance model on a sample of rows
    model_err = []
    field = meta.get("field")
    smodel = meta.get("sensor_model")
    if field and smodel:
        sensors = [smodel[i] for i in range(4)]
        step = max(1, len(rows) // 200)
        for r in rows[::step]:
            pose = (fnum(r["applied_x"]), fnum(r["applied_y"]), math.radians(fnum(r["applied_theta_deg"])))
            for si, sname in enumerate(SENSORS):
                logged = fnum(r[f"{sname}_expected"])
                if logged <= 0:
                    continue
                calc = expected_distance(field, meta.get("obstacles", []), sensors, pose, si)
                if calc > 0:
                    model_err.append(abs(calc - logged))
    model_err.sort()

    # end-state deltas
    last = rows[-1]
    end_odom_vs_only = math.hypot(fnum(last["odom_x"]) - fnum(last["odom_only_x"]),
                                  fnum(last["odom_y"]) - fnum(last["odom_only_y"]))
    end_ekf_vs_odom = math.hypot(fnum(last["ekf_x"]) - fnum(last["odom_x"]),
                                 fnum(last["ekf_y"]) - fnum(last["odom_y"]))

    out.update(dict(
        model_err_n=len(model_err),
        rows=len(rows), duration_s=round(dur, 2), path_in=round(path, 1),
        motion_pct=round(100.0 * motion_rows / len(rows), 1),
        scans=n_scan, accepted_scans=acc_scans, accept_events=len(accepts),
        accepts=accepts, reject_hist=mask_hist,
        nis_med=round(pct(0.5), 3) if nis_vals else None,
        nis_p90=round(pct(0.9), 3) if nis_vals else None,
        nis_max=round(nis_vals[-1], 3) if nis_vals else None,
        booked_drift_in=round(booked, 3), drift_rate_per100=round(rate * 100, 3),
        max_seg_in=round(max_seg, 1), tail_seg_in=round(tail, 1),
        raw_model_max=round(rate * path, 3), new_model_max=round(rate * max_seg, 3),
        new_model_end=round(rate * tail, 3),
        factor=round(path / max_seg, 2) if max_seg > 1e-6 else None,
        staircase_violation_in=round(stair_viol, 4),
        seq_gaps=seq_gaps[:5], n_seq_gaps=len(seq_gaps),
        max_applied_step_in=round(max_applied, 4), applied_total_in=round(applied_total, 2),
        cap_violations=cap_viol,
        end_net_applied_in=round(end_odom_vs_only, 3), end_ekf_minus_odom_in=round(end_ekf_vs_odom, 3),
        sensor_stats=sensor_stats,
        model_err_max=round(model_err[-1], 4) if model_err else None,
        model_err_med=round(model_err[len(model_err) // 2], 4) if model_err else None,
        checkpoints=meta.get("checkpoints", []),
    ))
    return out


def main():
    results = {}
    for key, fname in RUNS:
        p = (DATA / fname).resolve()
        if not p.exists():
            print(f"!! missing {p}")
            continue
        meta, hdr, rows = parse_log(p)
        res = analyze_run(key, meta, rows)
        results[key] = res

        m = res["meta"]
        print(f"\n=== {key} ({m.get('compiled','?')}) test_case={m.get('test_case','?')} "
              f"status={m.get('status','?')}")
        print(f"  reloc: status={m.get('relocalize_status')} pose={m.get('relocalize_pose')} "
              f"conf={m.get('reloc_conf')} active={m.get('reloc_active')} scored={m.get('reloc_scored')} "
              f"imu={m.get('imu_heading_deg')}")
        if res.get("rows", 0) == 0:
            print("  (no trace rows)")
            continue
        print(f"  rows={res['rows']} dur={res['duration_s']}s path={res['path_in']}in "
              f"motion%={res['motion_pct']} scans={res['scans']} acceptedScans={res['accepted_scans']} "
              f"events={res['accept_events']}")
        print(f"  booked_drift={res['booked_drift_in']}in rate/100in={res['drift_rate_per100']} "
              f"max_seg={res['max_seg_in']} factor={res['factor']} stair_viol={res['staircase_violation_in']}")
        print(f"  nis med/p90/max={res['nis_med']}/{res['nis_p90']}/{res['nis_max']} "
              f"max_applied_step={res['max_applied_step_in']} capViol={res['cap_violations']} "
              f"net_applied_end={res['end_net_applied_in']} ekf-odom_end={res['end_ekf_minus_odom_in']}")
        print(f"  seq_gaps={res['n_seq_gaps']} {res['seq_gaps']}")
        print(f"  reject_hist={res['reject_hist']}")
        print(f"  model_err med/max={res['model_err_med']}/{res['model_err_max']}")
        for s, st in res["sensor_stats"].items():
            print(f"    {s}: {st}")
        if res["accepts"]:
            for a in res["accepts"]:
                print(f"    accept t={a['t']:.2f}s mag={a['mag']:.2f} nis={a['nis']:.3f} "
                      f"active={a['active']} inl={a['inl']} res={a['mean_res']:.2f}/{a['max_res']:.2f}")
        cps = res.get("checkpoints", [])
        if cps:
            print(f"  checkpoints={len(cps)}")
            for c in cps:
                print(f"    [{c['idx']}] {c['name']}: err=({c['err'][0]:+.1f},{c['err'][1]:+.1f},{c['err'][2]:+.1f}deg) "
                      f"A{c['active']} nis={c['nis']:.2f} mask={c['mask']}")

        # wall-solve reproduction for wall_direct successes
        if m.get("relocalize_status") == "wall_direct" and res["meta"].get("relocalize_pose"):
            field = meta.get("field")
            smodel = meta.get("sensor_model")
            sens = meta.get("reloc_sensors", {})
            if field and smodel and sens:
                sensors = [smodel[i] for i in range(4)]
                imu = math.radians(meta.get("imu_heading_deg", 0.0))
                best = None
                for hd in (0.0, math.pi / 2, math.pi, -math.pi / 2):
                    sol = solve_wall(field, sensors, sens, hd)
                    if sol is None:
                        continue
                    pen = abs(math.remainder(hd - imu, 2 * math.pi))
                    if best is None or pen < best[1]:
                        best = (sol, pen)
                if best:
                    sol = best[0]
                    want = [float(x) for x in m["relocalize_pose"].split()]
                    res["wall_repro_err"] = round(math.hypot(sol[0] - want[0], sol[1] - want[1]), 4)
                    print(f"  wall-solve repro: ({sol[0]:.2f},{sol[1]:.2f}) used={sol[3]} "
                          f"vs logged ({want[0]:.1f},{want[1]:.1f}) "
                          f"d=({sol[0]-want[0]:+.2f},{sol[1]-want[1]:+.2f})")

    out = ROOT / "tools" / "audit_metrics.json"
    def clean(o):
        if isinstance(o, dict):
            return {k: clean(v) for k, v in o.items() if k != "checkpoints" or v}
        if isinstance(o, list):
            return [clean(v) for v in o]
        if isinstance(o, float) and (math.isnan(o) or math.isinf(o)):
            return None
        return o
    out.write_text(json.dumps(clean(results), indent=1, default=str))
    print(f"\nwrote {out}")

    emit_report_data(results)
    return 0


def emit_report_data(results):
    """Emit LaTeX macros and table-row fragments consumed by the report."""
    rd = ROOT / "report" / "data"
    rd.mkdir(parents=True, exist_ok=True)

    runs = [k for k, _ in RUNS if k in results and k != "tune"]
    ray_max = max((results[k].get("model_err_max") or 0.0) for k in results)
    ray_n = sum(results[k].get("model_err_n", 0) for k in results)
    wall_max = max((results[k].get("wall_repro_err") or 0.0) for k in results)
    stair_max = max((results[k].get("staircase_violation_in") or 0.0) for k in results)
    cap_obs = max((results[k].get("max_applied_step_in") or 0.0) for k in results)
    accepts = [(k, a) for k in runs for a in results[k].get("accepts", [])]
    mags = [a["mag"] for _, a in accepts]
    nises = [a["nis"] for _, a in accepts]
    run5_pose = [float(x) for x in results["run5"]["meta"]["relocalize_pose"].split()]
    run5_err = math.hypot(run5_pose[0] - (-27.0), run5_pose[1] - (-36.0))

    def cp_stats(key):
        cps = results[key].get("checkpoints", [])[1:]  # skip Start hold
        if not cps:
            return None
        exy = [max(abs(c["err"][0]), abs(c["err"][1])) for c in cps]
        return max(exy)

    tune_cps = results.get("tune", {}).get("checkpoints", [])[1:]
    tune_bias = sum(c["err"][2] for c in tune_cps) / len(tune_cps) if tune_cps else 0.0

    with (rd / "audit_macros.tex").open("w") as fh:
        def mac(name, val):
            fh.write(f"\\newcommand{{\\{name}}}{{{val}}}\n")
        fh.write("% auto-generated by tools/audit_analysis.py -- do not edit\n")
        mac("AuditRayMaxErr", f"{ray_max:.4f}")
        mac("AuditRayCount", f"{ray_n:,}")
        mac("AuditWallMaxErr", f"{wall_max:.2f}")
        mac("AuditStairMax", f"{stair_max:.4f}")
        mac("AuditCapObserved", f"{cap_obs:.4f}")
        mac("AuditCapBound", "0.030")
        mac("nValidatedFixes", f"{len(accepts)}")
        mac("AcceptMagMin", f"{min(mags):.2f}")
        mac("AcceptMagMax", f"{max(mags):.2f}")
        mac("AcceptNisMin", f"{min(nises):.3f}")
        mac("AcceptNisMax", f"{max(nises):.2f}")
        mac("RunFiveCommitErr", f"{run5_err:.0f}")
        mac("RunTwoCpMax", f"{cp_stats('run2'):.1f}")
        mac("RunThreeCpMax", f"{cp_stats('run3'):.1f}")
        mac("TuneSweepBias", f"{tune_bias:.1f}")
        mac("RunTwoBooked", f"{results['run2']['booked_drift_in']:.2f}")
        mac("RunThreeBooked", f"{results['run3']['booked_drift_in']:.2f}")

    for key in ("run2", "run3"):
        with (rd / f"cp_{key}_rows.tex").open("w") as fh:
            for c in results[key].get("checkpoints", []):
                nm = c["name"].replace("&", "\\&")
                fh.write(f"{c['idx']} & {nm} & {c['err'][0]:+.1f} & {c['err'][1]:+.1f} & "
                         f"{c['err'][2]:+.1f} & {c['active']} \\\\\n")

    with (rd / "accepts_rows.tex").open("w") as fh:
        for k, a in accepts:
            label = "2" if k == "run2" else "3"
            fh.write(f"{label} & {a['t']:.1f} & {a['mag']:.2f} & {a['nis']:.3f} & "
                     f"{a['active']} & {a['inl']} \\\\\n")

    with (rd / "rejects_run3.dat").open("w") as fh:
        fh.write("reason count\n")
        hist = results["run3"].get("reject_hist", {})
        order = ["motion_suppressed", "readiness", "not_enough_sensors", "unstable_candidate",
                 "no_measurement", "pose_delta", "sensor_residual", "sensors_stale", "turn_suppressed"]
        for name in order:
            if hist.get(name):
                fh.write(f"{name.replace('_', '-')} {hist[name]}\n")

    with (rd / "reloc_rows.tex").open("w") as fh:
        labels = {
            "run1": ("1", "sample route"), "run2": ("2", "square loop"),
            "run3": ("3", "square $+$ cross"), "run4": ("4", "blocked view"),
            "run5": ("5", "forced weak start"), "run6": ("6", "weak start, guarded"),
            "run7": ("7", "clean start, guarded"),
        }
        guard = {"run1": "pre", "run2": "pre", "run3": "pre", "run4": "pre",
                 "run5": "pre", "run6": "post", "run7": "post"}
        for k in runs:
            m = results[k]["meta"]
            num, desc = labels[k]
            status = str(m.get("relocalize_status", "?")).replace("_", "\\_")
            pose = m.get("relocalize_pose", "")
            ps = pose.split()
            posestr = f"({float(ps[0]):.1f}, {float(ps[1]):.1f})" if len(ps) >= 2 else "--"
            conf = m.get("reloc_conf", 0.0)
            scored = m.get("reloc_scored", 0)
            fh.write(f"{num} & {desc} & {guard[k]} & {status} & {scored} & "
                     f"{conf:.2f} & {posestr} \\\\\n")

    print(f"wrote report data fragments to {rd}")


if __name__ == "__main__":
    sys.exit(main())
