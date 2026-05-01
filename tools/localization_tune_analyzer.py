#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


TRACE_MARKER = "=== LOCALIZATION TRACE CSV ==="
SENSOR_NAMES = ["front", "right", "back", "left"]


@dataclass
class Pose:
    x: float
    y: float
    theta: float


@dataclass
class Obstacle:
    kind: str
    x: float
    y: float
    radius: float
    half_w: float
    half_h: float


@dataclass
class FieldConfig:
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    field_margin: float
    obstacle_margin: float
    obstacles: list[Obstacle]


@dataclass
class OdomModel:
    vertical_offset: float
    horizontal_offset: float
    track_width: float
    vertical_wheel_diameter: float
    vertical_gear_ratio: float
    horizontal_wheel_diameter: float
    horizontal_gear_ratio: float


@dataclass
class SensorModel:
    idx: int
    dx: float
    dy: float
    dtheta_deg: float
    min_range: float
    max_range: float
    min_confidence: int

    @property
    def dtheta_rad(self) -> float:
        return math.radians(self.dtheta_deg)


@dataclass
class OdomPair:
    row: dict[str, float | int]
    truth_local_x: float
    truth_local_y: float
    truth_delta_theta: float
    odom_delta_theta: float
    raw_delta_vertical: float
    raw_delta_horizontal: float


def wrap_angle(theta: float) -> float:
    return math.remainder(theta, 2.0 * math.pi)


def median(values: Iterable[float]) -> float | None:
    ordered = sorted(v for v in values if math.isfinite(v))
    if not ordered:
        return None
    mid = len(ordered) // 2
    if len(ordered) % 2 == 1:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


def downsample(values: list[dict[str, float | int]], limit: int) -> list[dict[str, float | int]]:
    if len(values) <= limit:
        return values
    step = max(1, len(values) // limit)
    return values[::step][:limit]


def parse_number(value: str) -> float | int | str:
    value = value.strip()
    if value == "":
        return value
    try:
        return int(value)
    except ValueError:
        pass
    try:
        return float(value)
    except ValueError:
        return value


def parse_trace(path: Path) -> tuple[FieldConfig | None, OdomModel | None, dict[int, SensorModel], list[dict[str, float | int]]]:
    text = path.read_text(encoding="utf-8")
    if TRACE_MARKER in text:
        text = text.split(TRACE_MARKER, 1)[1]

    comment_lines: list[str] = []
    csv_lines: list[str] = []
    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        if line.startswith("#"):
            comment_lines.append(line[1:].strip())
        else:
            csv_lines.append(line)

    if not csv_lines:
        raise ValueError("No CSV trace section found in log file")

    field: FieldConfig | None = None
    odom: OdomModel | None = None
    sensors: dict[int, SensorModel] = {}
    obstacles: list[Obstacle] = []

    for line in comment_lines:
        parts = [part.strip() for part in line.split(",")]
        if not parts:
            continue
        tag = parts[0]
        if tag == "field_bounds" and len(parts) >= 7:
            field = FieldConfig(
                min_x=float(parts[1]),
                max_x=float(parts[2]),
                min_y=float(parts[3]),
                max_y=float(parts[4]),
                field_margin=float(parts[5]),
                obstacle_margin=float(parts[6]),
                obstacles=obstacles,
            )
        elif tag == "odom_model" and len(parts) >= 8:
            odom = OdomModel(
                vertical_offset=float(parts[1]),
                horizontal_offset=float(parts[2]),
                track_width=float(parts[3]),
                vertical_wheel_diameter=float(parts[4]),
                vertical_gear_ratio=float(parts[5]),
                horizontal_wheel_diameter=float(parts[6]),
                horizontal_gear_ratio=float(parts[7]),
            )
        elif tag == "obstacle" and len(parts) >= 8:
            obstacles.append(
                Obstacle(
                    kind=parts[2],
                    x=float(parts[3]),
                    y=float(parts[4]),
                    radius=float(parts[5]),
                    half_w=float(parts[6]),
                    half_h=float(parts[7]),
                )
            )
        elif tag == "sensor_model" and len(parts) >= 8:
            idx = int(parts[1])
            sensors[idx] = SensorModel(
                idx=idx,
                dx=float(parts[2]),
                dy=float(parts[3]),
                dtheta_deg=float(parts[4]),
                min_range=float(parts[5]),
                max_range=float(parts[6]),
                min_confidence=int(parts[7]),
            )

    if field is not None:
        field.obstacles = obstacles

    reader = csv.DictReader(csv_lines)
    rows: list[dict[str, float | int]] = []
    for row in reader:
        parsed = {key: parse_number(value or "") for key, value in row.items()}
        rows.append(parsed)

    return field, odom, sensors, rows


def pose_from_row(row: dict[str, float | int], source: str) -> Pose:
    return Pose(
        x=float(row[f"{source}_x"]),
        y=float(row[f"{source}_y"]),
        theta=math.radians(float(row[f"{source}_theta_deg"])),
    )


def local_delta_from_poses(prev_pose: Pose, next_pose: Pose) -> tuple[float, float, float]:
    delta_theta = wrap_angle(next_pose.theta - prev_pose.theta)
    avg_heading = prev_pose.theta + 0.5 * delta_theta
    dx_global = next_pose.x - prev_pose.x
    dy_global = next_pose.y - prev_pose.y
    local_x = -dx_global * math.cos(avg_heading) + dy_global * math.sin(avg_heading)
    local_y = dx_global * math.sin(avg_heading) + dy_global * math.cos(avg_heading)
    return local_x, local_y, delta_theta


def pair_is_reliable(prev_row: dict[str, float | int], row: dict[str, float | int]) -> bool:
    if int(row["odom_seq"]) != int(prev_row["odom_seq"]) + 1:
        return False
    if int(row["odom_tel_seq"]) != int(row["odom_seq"]):
        return False
    if int(prev_row["odom_tel_seq"]) != int(prev_row["odom_seq"]):
        return False
    return (
        int(prev_row["mcl_valid"]) == 1
        and int(row["mcl_valid"]) == 1
        and int(prev_row["sensors_stale"]) == 0
        and int(row["sensors_stale"]) == 0
        and int(prev_row["active_sensors"]) >= 2
        and int(row["active_sensors"]) >= 2
    )


def build_odom_pairs(rows: list[dict[str, float | int]], pose_source: str) -> list[OdomPair]:
    pairs: list[OdomPair] = []
    for prev_row, row in zip(rows, rows[1:]):
        if not pair_is_reliable(prev_row, row):
            continue
        prev_pose = pose_from_row(prev_row, pose_source)
        current_pose = pose_from_row(row, pose_source)
        truth_local_x, truth_local_y, truth_delta_theta = local_delta_from_poses(prev_pose, current_pose)
        pairs.append(
            OdomPair(
                row=row,
                truth_local_x=truth_local_x,
                truth_local_y=truth_local_y,
                truth_delta_theta=truth_delta_theta,
                odom_delta_theta=math.radians(float(row["odom_delta_theta_deg"])),
                raw_delta_vertical=float(row["odom_selected_delta_vertical"]),
                raw_delta_horizontal=float(row["odom_selected_delta_horizontal"]),
            )
        )
    return pairs


def robust_offset_estimate(pairs: list[OdomPair], axis: str) -> tuple[float | None, int]:
    estimates: list[float] = []
    for pair in pairs:
        delta_theta = pair.odom_delta_theta
        if abs(delta_theta) < math.radians(2.0):
            continue
        turn_factor = 2.0 * math.sin(delta_theta / 2.0)
        if abs(turn_factor) < 1e-6:
            continue
        if axis == "horizontal":
            truth_local = pair.truth_local_x
            raw_delta = pair.raw_delta_horizontal
        else:
            truth_local = pair.truth_local_y
            raw_delta = pair.raw_delta_vertical
        estimates.append(truth_local / turn_factor - raw_delta / delta_theta)
    return median(estimates), len(estimates)


def robust_scale_estimate(pairs: list[OdomPair], axis: str) -> tuple[float | None, int]:
    estimates: list[float] = []
    for pair in pairs:
        if abs(pair.odom_delta_theta) > math.radians(1.0):
            continue
        if axis == "horizontal":
            truth_local = pair.truth_local_x
            raw_delta = pair.raw_delta_horizontal
        else:
            truth_local = pair.truth_local_y
            raw_delta = pair.raw_delta_vertical
        if abs(raw_delta) < 0.2:
            continue
        estimates.append(truth_local / raw_delta)
    return median(estimates), len(estimates)


def robust_turn_scale_estimate(pairs: list[OdomPair]) -> tuple[float | None, int]:
    estimates: list[float] = []
    for pair in pairs:
        if abs(pair.odom_delta_theta) < math.radians(2.0):
            continue
        estimates.append(pair.truth_delta_theta / pair.odom_delta_theta)
    return median(estimates), len(estimates)


def fit_lateral_bias_terms(
    pairs: list[OdomPair], horizontal_offset: float, horizontal_scale: float
) -> tuple[float | None, float | None, int]:
    sum_a = 0.0
    sum_b = 0.0
    sum_c = 0.0
    sum_d = 0.0
    sum_e = 0.0
    count = 0

    for pair in pairs:
        delta_theta = pair.odom_delta_theta
        if abs(delta_theta) < math.radians(1.0):
            continue
        raw_delta = pair.raw_delta_horizontal * horizontal_scale
        if abs(delta_theta) < 1e-6:
            model_local_x = raw_delta
        else:
            model_local_x = 2.0 * math.sin(delta_theta / 2.0) * (raw_delta / delta_theta + horizontal_offset)
        residual = pair.truth_local_x - model_local_x
        feature_a = delta_theta
        feature_b = abs(pair.truth_local_y) * delta_theta
        sum_a += feature_a * feature_a
        sum_b += feature_a * feature_b
        sum_c += feature_b * feature_b
        sum_d += feature_a * residual
        sum_e += feature_b * residual
        count += 1

    det = sum_a * sum_c - sum_b * sum_b
    if count < 10 or abs(det) < 1e-9:
        return None, None, count
    bias_per_rad = (sum_d * sum_c - sum_e * sum_b) / det
    bias_per_in_rad = (sum_e * sum_a - sum_d * sum_b) / det
    return bias_per_rad, bias_per_in_rad, count


def raycast_circle(sx: float, sy: float, dir_x: float, dir_y: float, obstacle: Obstacle) -> float:
    dx = sx - obstacle.x
    dy = sy - obstacle.y
    a = dir_x * dir_x + dir_y * dir_y
    b = 2.0 * (dx * dir_x + dy * dir_y)
    c = dx * dx + dy * dy - obstacle.radius * obstacle.radius
    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        return -1.0
    sqrt_disc = math.sqrt(disc)
    inv_2a = 1.0 / (2.0 * a)
    t1 = (-b - sqrt_disc) * inv_2a
    t2 = (-b + sqrt_disc) * inv_2a
    candidates = [t for t in (t1, t2) if t > 0.0]
    return min(candidates) if candidates else -1.0


def raycast_rect(sx: float, sy: float, dir_x: float, dir_y: float, obstacle: Obstacle) -> float:
    min_x = obstacle.x - obstacle.half_w
    max_x = obstacle.x + obstacle.half_w
    min_y = obstacle.y - obstacle.half_h
    max_y = obstacle.y + obstacle.half_h
    eps = 1e-6
    t_min = -1e9
    t_max = 1e9

    if abs(dir_x) < eps:
        if sx < min_x or sx > max_x:
            return -1.0
    else:
        tx1 = (min_x - sx) / dir_x
        tx2 = (max_x - sx) / dir_x
        t_min = max(t_min, min(tx1, tx2))
        t_max = min(t_max, max(tx1, tx2))

    if abs(dir_y) < eps:
        if sy < min_y or sy > max_y:
            return -1.0
    else:
        ty1 = (min_y - sy) / dir_y
        ty2 = (max_y - sy) / dir_y
        t_min = max(t_min, min(ty1, ty2))
        t_max = min(t_max, max(ty1, ty2))

    if t_max < 0.0 or t_min > t_max:
        return -1.0
    if t_min >= 0.0:
        return t_min
    if t_max >= 0.0:
        return t_max
    return -1.0


def expected_distance(field: FieldConfig, pose: Pose, sensor: SensorModel) -> float:
    sin_theta = math.sin(pose.theta)
    cos_theta = math.cos(pose.theta)

    sx = pose.x + sensor.dx * cos_theta + sensor.dy * sin_theta
    sy = pose.y - sensor.dx * sin_theta + sensor.dy * cos_theta

    heading = wrap_angle(pose.theta + sensor.dtheta_rad)
    dir_x = math.sin(heading)
    dir_y = math.cos(heading)
    best = 1e9
    eps = 1e-6

    if abs(dir_x) > eps:
        t1 = (field.min_x - sx) / dir_x
        y1 = sy + t1 * dir_y
        if t1 > 0.0 and field.min_y <= y1 <= field.max_y:
            best = min(best, t1)

        t2 = (field.max_x - sx) / dir_x
        y2 = sy + t2 * dir_y
        if t2 > 0.0 and field.min_y <= y2 <= field.max_y:
            best = min(best, t2)

    if abs(dir_y) > eps:
        t3 = (field.min_y - sy) / dir_y
        x3 = sx + t3 * dir_x
        if t3 > 0.0 and field.min_x <= x3 <= field.max_x:
            best = min(best, t3)

        t4 = (field.max_y - sy) / dir_y
        x4 = sx + t4 * dir_x
        if t4 > 0.0 and field.min_x <= x4 <= field.max_x:
            best = min(best, t4)

    for obstacle in field.obstacles:
        if obstacle.kind == "circle":
            hit = raycast_circle(sx, sy, dir_x, dir_y, obstacle)
        else:
            hit = raycast_rect(sx, sy, dir_x, dir_y, obstacle)
        if hit > 0.0:
            best = min(best, hit)

    return -1.0 if best >= 1e8 else best


def sensor_measurements(
    rows: list[dict[str, float | int]],
    field: FieldConfig,
    sensor: SensorModel,
    sensor_name: str,
    pose_source: str,
    sample_limit: int,
) -> list[tuple[Pose, float]]:
    usable: list[dict[str, float | int]] = []
    for row in rows:
        if int(row[f"{sensor_name}_used"]) != 1:
            continue
        if int(row["mcl_valid"]) != 1 or int(row["sensors_stale"]) != 0:
            continue
        usable.append(row)

    measurements: list[tuple[Pose, float]] = []
    for row in downsample(usable, sample_limit):
        pose = pose_from_row(row, pose_source)
        measured = float(row[f"{sensor_name}_dist"])
        predicted = expected_distance(field, pose, sensor)
        if predicted <= 0.0:
            continue
        measurements.append((pose, measured))
    return measurements


def sensor_objective(field: FieldConfig, sensor: SensorModel, measurements: list[tuple[Pose, float]]) -> tuple[float, float]:
    residuals: list[float] = []
    for pose, measured in measurements:
        predicted = expected_distance(field, pose, sensor)
        if predicted <= 0.0:
            continue
        residuals.append(measured - predicted)
    bias = median(residuals)
    if bias is None:
        return float("inf"), 0.0
    centered_errors = [abs(residual - bias) for residual in residuals]
    score = median(centered_errors)
    return float(score if score is not None else float("inf")), bias


def tune_sensor(
    field: FieldConfig,
    sensor: SensorModel,
    measurements: list[tuple[Pose, float]],
) -> tuple[SensorModel, float, float]:
    best = SensorModel(**sensor.__dict__)
    best_score, best_bias = sensor_objective(field, best, measurements)

    current_dx = best.dx
    current_dy = best.dy
    current_theta = best.dtheta_deg

    for step_xy, step_theta in ((0.5, 4.0), (0.25, 2.0), (0.125, 1.0)):
        improved = True
        while improved:
            improved = False
            candidates: list[SensorModel] = [best]
            for delta in (-step_xy, step_xy):
                candidates.append(
                    SensorModel(**{**best.__dict__, "dx": max(current_dx - 2.0, min(current_dx + 2.0, best.dx + delta))})
                )
                candidates.append(
                    SensorModel(**{**best.__dict__, "dy": max(current_dy - 2.0, min(current_dy + 2.0, best.dy + delta))})
                )
            for delta in (-step_theta, step_theta):
                candidates.append(
                    SensorModel(
                        **{
                            **best.__dict__,
                            "dtheta_deg": max(current_theta - 10.0, min(current_theta + 10.0, best.dtheta_deg + delta)),
                        }
                    )
                )

            for candidate in candidates:
                score, bias = sensor_objective(field, candidate, measurements)
                if score + 1e-6 < best_score:
                    best = candidate
                    best_score = score
                    best_bias = bias
                    improved = True
    return best, best_score, best_bias


def format_optional(value: float | None, precision: int = 4) -> str:
    if value is None or not math.isfinite(value):
        return "n/a"
    return f"{value:.{precision}f}"


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze a localization tune trace and suggest odom / sensor updates.")
    parser.add_argument("trace", type=Path, help="Path to localization_tune_latest.txt")
    parser.add_argument(
        "--pose-source",
        choices=("applied", "ekf", "mcl"),
        default="applied",
        help="Pose source used as the offline reference trajectory",
    )
    parser.add_argument(
        "--sensor-sample-limit",
        type=int,
        default=1500,
        help="Maximum number of per-sensor measurements to evaluate during bounded offset search",
    )
    args = parser.parse_args()

    field, odom_model, sensors, rows = parse_trace(args.trace)
    print(f"Trace rows: {len(rows)}")
    print(f"Pose source: {args.pose_source}")

    odom_pairs = build_odom_pairs(rows, args.pose_source)
    print(f"Reliable odom pairs: {len(odom_pairs)}")
    print()

    if odom_model is not None and odom_pairs:
        vertical_offset, vertical_offset_count = robust_offset_estimate(odom_pairs, "vertical")
        horizontal_offset, horizontal_offset_count = robust_offset_estimate(odom_pairs, "horizontal")
        vertical_scale, vertical_scale_count = robust_scale_estimate(odom_pairs, "vertical")
        horizontal_scale, horizontal_scale_count = robust_scale_estimate(odom_pairs, "horizontal")
        turn_scale, turn_scale_count = robust_turn_scale_estimate(odom_pairs)

        vertical_scale = 1.0 if vertical_scale is None else vertical_scale
        horizontal_scale = 1.0 if horizontal_scale is None else horizontal_scale
        lateral_bias_per_rad, lateral_bias_per_in_rad, lateral_bias_count = fit_lateral_bias_terms(
            odom_pairs,
            horizontal_offset if horizontal_offset is not None else odom_model.horizontal_offset,
            horizontal_scale,
        )

        suggested_vertical_offset = vertical_offset if vertical_offset is not None else odom_model.vertical_offset
        suggested_horizontal_offset = horizontal_offset if horizontal_offset is not None else odom_model.horizontal_offset

        print("Odom suggestions")
        print(
            f"  vertical offset:    current {odom_model.vertical_offset:.4f} -> suggested {suggested_vertical_offset:.4f}"
            f"  (turn samples {vertical_offset_count})"
        )
        print(
            f"  horizontal offset:  current {odom_model.horizontal_offset:.4f} -> suggested {suggested_horizontal_offset:.4f}"
            f"  (turn samples {horizontal_offset_count})"
        )
        print(
            f"  vertical scale:     current 1.0000 -> suggested {vertical_scale:.4f}"
            f"  (straight samples {vertical_scale_count})"
        )
        print(
            f"  horizontal scale:   current 1.0000 -> suggested {horizontal_scale:.4f}"
            f"  (straight samples {horizontal_scale_count})"
        )
        print(
            f"  turn scale:         current 1.0000 -> suggested {format_optional(turn_scale)}"
            f"  (turn samples {turn_scale_count})"
        )
        print(
            f"  lateral bias/rad:   current 0.0000 -> suggested {format_optional(lateral_bias_per_rad)}"
            f"  (fit samples {lateral_bias_count})"
        )
        print(
            f"  lateral bias/in*rad current 0.0000 -> suggested {format_optional(lateral_bias_per_in_rad)}"
            f"  (fit samples {lateral_bias_count})"
        )
        print(
            f"  vertical wheel dia: current {odom_model.vertical_wheel_diameter:.4f}"
            f" -> suggested {odom_model.vertical_wheel_diameter * vertical_scale:.4f}"
        )
        print(
            f"  horizontal wheel dia: current {odom_model.horizontal_wheel_diameter:.4f}"
            f" -> suggested {odom_model.horizontal_wheel_diameter * horizontal_scale:.4f}"
        )
        print()
    elif odom_model is None:
        print("Odom suggestions")
        print("  Missing odom metadata in the trace; re-run with the v2 logger to fit physical offsets.")
        print()

    if field is None or not sensors:
        print("Sensor suggestions")
        print("  Missing field or sensor metadata in the trace; re-run with the v2 logger to fit sensor offsets.")
        return 0

    print("Sensor suggestions")
    for idx, sensor_name in enumerate(SENSOR_NAMES):
        sensor = sensors.get(idx)
        if sensor is None:
            print(f"  {sensor_name}: missing metadata")
            continue
        measurements = sensor_measurements(rows, field, sensor, sensor_name, args.pose_source, args.sensor_sample_limit)
        if len(measurements) < 25:
            print(f"  {sensor_name}: not enough usable samples ({len(measurements)})")
            continue

        current_score, current_bias = sensor_objective(field, sensor, measurements)
        tuned_sensor, tuned_score, tuned_bias = tune_sensor(field, sensor, measurements)
        improvement = 0.0
        if math.isfinite(current_score) and current_score > 1e-6:
            improvement = 100.0 * (current_score - tuned_score) / current_score

        print(
            f"  {sensor_name}: bias {current_bias:+.3f} -> {tuned_bias:+.3f} in,"
            f" mae {current_score:.3f} -> {tuned_score:.3f} in, improvement {improvement:.1f}%"
        )
        print(
            f"    dx {sensor.dx:.3f} -> {tuned_sensor.dx:.3f},"
            f" dy {sensor.dy:.3f} -> {tuned_sensor.dy:.3f},"
            f" dtheta {sensor.dtheta_deg:.2f} -> {tuned_sensor.dtheta_deg:.2f} deg"
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
