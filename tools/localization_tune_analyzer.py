#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


TRACE_MARKER = "=== LOCALIZATION TRACE CSV ==="
END_MARKER = "=== END LOCALIZATION TUNE LOG ==="
SENSOR_NAMES = ["front", "right", "back", "left"]
SENSOR_DTHETA_SEARCH_BOUNDS_DEG = {
    0: 30.0, # front
    1: 10.0, # right
    2: 30.0, # back
    3: 10.0, # left
}
SENSOR_DX_LIMITS = {
    1: (0.0, None), # right sensor should stay on the robot's right side
    3: (None, 0.0), # left sensor should stay on the robot's left side
}
SENSOR_FIT_MIN_CONFIDENCE = 30
SENSOR_FIT_MIN_FIELD_EDGE_MARGIN = 10.0
GLOBAL_FIT_MIN_STARTS = 3
GLOBAL_FIT_MIN_START_SPAN_IN = 36.0
UNMODELED_OCCLUSION_RESIDUAL_IN = 7.0
UNMODELED_OCCLUSION_BIN_IN = 6.0
UNMODELED_OCCLUSION_MIN_HITS = 50
UNMODELED_OCCLUSION_MIN_CLUSTER_HITS = 12
STATIONARY_SWEEP_SOLVE_MIN_OBS = 60
STATIONARY_SWEEP_SOLVE_MAX_SCORE = 3.0
TRUSTED_RELOCALIZE_MAX_MEAN_RESIDUAL = 5.0
TRUSTED_RELOCALIZE_MAX_RESIDUAL = 12.0


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
class SensorFitQuality:
    heading_span_deg: float
    hit_labels: list[str]
    hit_search_edge: bool
    status: str


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
    if END_MARKER in text:
        text = text.split(END_MARKER, 1)[0]

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
    test_case_number: int | None = None
    relocalize_pose: Pose | None = None
    relocalize_success = False
    relocalize_status = ""
    relocalize_mean_residual = 0.0
    relocalize_max_residual = 0.0
    relocalize_scored_sensors = 0

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
        elif tag == "test_case" and len(parts) >= 2:
            try:
                test_case_number = int(parts[1])
            except ValueError:
                test_case_number = None
        elif tag == "relocalize" and len(parts) >= 7 and int(parts[1]) == 1:
            relocalize_success = int(parts[2]) == 1
            relocalize_status = parts[3]
            if relocalize_success:
                relocalize_pose = Pose(
                    x=float(parts[4]),
                    y=float(parts[5]),
                    theta=math.radians(float(parts[6])),
                )
            if len(parts) >= 11:
                relocalize_scored_sensors = int(parts[10])
        elif tag == "relocalize_residual" and len(parts) >= 3:
            relocalize_mean_residual = float(parts[1])
            relocalize_max_residual = float(parts[2])
            if len(parts) >= 4:
                relocalize_scored_sensors = int(parts[3])

    if field is not None:
        field.obstacles = obstacles

    reader = csv.DictReader(csv_lines)
    rows: list[dict[str, float | int]] = []
    for row in reader:
        parsed = {key: parse_number(value or "") for key, value in row.items()}
        parsed["_test_case_number"] = -1 if test_case_number is None else test_case_number
        parsed["_relocalize_success"] = 1 if relocalize_success else 0
        trusted_relocalize = (
            relocalize_success
            and relocalize_mean_residual <= TRUSTED_RELOCALIZE_MAX_MEAN_RESIDUAL
            and relocalize_max_residual <= TRUSTED_RELOCALIZE_MAX_RESIDUAL
            and relocalize_scored_sensors >= 3
        )
        # A wall_direct solve fixes heading from the IMU and solves x,y directly off
        # the walls. Unlike the heading-searching stationary sweep solver, it is not
        # subject to the square field's mirror ambiguity, so trust it as the geometry
        # anchor even with only 2 scored sensors as long as the direct residual is ~0.
        trusted_direct = (
            relocalize_success
            and relocalize_status == "wall_direct"
            and relocalize_mean_residual <= TRUSTED_RELOCALIZE_MAX_MEAN_RESIDUAL
            and relocalize_max_residual <= TRUSTED_RELOCALIZE_MAX_RESIDUAL
        )
        if test_case_number == 5 and relocalize_pose is not None and (trusted_relocalize or trusted_direct):
            parsed["stationary_start_x"] = relocalize_pose.x
            parsed["stationary_start_y"] = relocalize_pose.y
            parsed["stationary_heading_offset_deg"] = 0.0
        rows.append(parsed)

    return field, odom, sensors, rows


def parse_traces(
    paths: list[Path],
) -> tuple[FieldConfig | None, OdomModel | None, dict[int, SensorModel], list[dict[str, float | int]]]:
    field: FieldConfig | None = None
    odom: OdomModel | None = None
    sensors: dict[int, SensorModel] = {}
    rows: list[dict[str, float | int]] = []

    for trace_index, path in enumerate(paths):
        trace_field, trace_odom, trace_sensors, trace_rows = parse_trace(path)
        if trace_field is not None:
            field = trace_field
        if trace_odom is not None:
            odom = trace_odom
        if trace_sensors:
            sensors = trace_sensors
        if (
            trace_field is not None
            and trace_sensors
            and trace_rows
            and int(trace_rows[0].get("_test_case_number", -1)) == 5
            and "stationary_start_x" not in trace_rows[0]
        ):
            solved = solve_stationary_sweep_pose(trace_field, trace_sensors, trace_rows)
            if solved is not None:
                x, y, heading_offset_deg, score, median_abs, observation_count = solved
                for row in trace_rows:
                    row["stationary_start_x"] = x
                    row["stationary_start_y"] = y
                    row["stationary_heading_offset_deg"] = heading_offset_deg
                    row["stationary_sweep_score"] = score
                    row["stationary_sweep_median_abs"] = median_abs
                    row["stationary_sweep_observations"] = observation_count
        for row in trace_rows:
            row["_trace_index"] = trace_index
            row["_trace_path"] = str(path)
            rows.append(row)

    return field, odom, sensors, rows


def pose_from_row(row: dict[str, float | int], source: str) -> Pose:
    if source == "stationary_start":
        if "stationary_start_x" not in row:
            source = "odom_only" if "odom_only_x" in row else "applied"
        else:
            return Pose(
                x=float(row["stationary_start_x"]),
                y=float(row["stationary_start_y"]),
                theta=math.radians(
                    float(row["odom_only_theta_deg"]) + float(row.get("stationary_heading_offset_deg", 0.0))
                ),
            )
    if f"{source}_x" not in row:
        source = "odom_only" if "odom_only_x" in row else "applied"
    if source == "stationary_start":
        return Pose(
            x=float(row["stationary_start_x"]),
            y=float(row["stationary_start_y"]),
            theta=math.radians(float(row["odom_only_theta_deg"]) + float(row.get("stationary_heading_offset_deg", 0.0))),
        )
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


def sensor_ray(pose: Pose, sensor: SensorModel) -> tuple[float, float, float, float, float]:
    sin_theta = math.sin(pose.theta)
    cos_theta = math.cos(pose.theta)

    sx = pose.x + sensor.dx * cos_theta + sensor.dy * sin_theta
    sy = pose.y - sensor.dx * sin_theta + sensor.dy * cos_theta

    heading = wrap_angle(pose.theta + sensor.dtheta_rad)
    dir_x = math.sin(heading)
    dir_y = math.cos(heading)
    return sx, sy, dir_x, dir_y, heading


def expected_hit(field: FieldConfig, pose: Pose, sensor: SensorModel) -> tuple[float, str]:
    sx, sy, dir_x, dir_y, _ = sensor_ray(pose, sensor)
    best = 1e9
    label = "none"
    eps = 1e-6

    def consider(distance: float, hit_label: str) -> None:
        nonlocal best, label
        if 0.0 < distance < best:
            best = distance
            label = hit_label

    if abs(dir_x) > eps:
        t1 = (field.min_x - sx) / dir_x
        y1 = sy + t1 * dir_y
        if t1 > 0.0 and field.min_y <= y1 <= field.max_y:
            consider(t1, "min_x")

        t2 = (field.max_x - sx) / dir_x
        y2 = sy + t2 * dir_y
        if t2 > 0.0 and field.min_y <= y2 <= field.max_y:
            consider(t2, "max_x")

    if abs(dir_y) > eps:
        t3 = (field.min_y - sy) / dir_y
        x3 = sx + t3 * dir_x
        if t3 > 0.0 and field.min_x <= x3 <= field.max_x:
            consider(t3, "min_y")

        t4 = (field.max_y - sy) / dir_y
        x4 = sx + t4 * dir_x
        if t4 > 0.0 and field.min_x <= x4 <= field.max_x:
            consider(t4, "max_y")

    for obstacle in field.obstacles:
        if obstacle.kind == "circle":
            hit = raycast_circle(sx, sy, dir_x, dir_y, obstacle)
        else:
            hit = raycast_rect(sx, sy, dir_x, dir_y, obstacle)
        if hit > 0.0:
            consider(hit, "obstacle")

    return (-1.0, "none") if best >= 1e8 else (best, label)


def expected_distance(field: FieldConfig, pose: Pose, sensor: SensorModel) -> float:
    distance, _ = expected_hit(field, pose, sensor)
    return distance


def field_edge_margin(field: FieldConfig, pose: Pose, sensor: SensorModel) -> float:
    distance, hit_label = expected_hit(field, pose, sensor)
    if distance <= 0.0:
        return -1.0
    sx, sy, dir_x, dir_y, _ = sensor_ray(pose, sensor)
    hit_x = sx + distance * dir_x
    hit_y = sy + distance * dir_y
    if hit_label in ("min_x", "max_x"):
        return min(abs(hit_y - field.min_y), abs(hit_y - field.max_y))
    if hit_label in ("min_y", "max_y"):
        return min(abs(hit_x - field.min_x), abs(hit_x - field.max_x))
    return float("inf")


def solve_stationary_sweep_pose(
    field: FieldConfig,
    sensors: dict[int, SensorModel],
    rows: list[dict[str, float | int]],
) -> tuple[float, float, float, float, float, int] | None:
    observations: list[tuple[int, float, float]] = []
    for row in rows:
        if int(row.get("mcl_valid", 0)) != 1 or int(row.get("sensors_stale", 0)) != 0:
            continue
        heading = math.radians(float(row["odom_only_theta_deg"]))
        for idx, sensor_name in enumerate(SENSOR_NAMES):
            sensor = sensors.get(idx)
            if sensor is None:
                continue
            changed_key = f"{sensor_name}_changed"
            if changed_key in row and int(row[changed_key]) == 0:
                continue
            if int(row.get(f"{sensor_name}_used", 0)) != 1:
                continue
            confidence = int(row.get(f"{sensor_name}_conf", -1))
            if confidence >= 0 and confidence < max(sensor.min_confidence, SENSOR_FIT_MIN_CONFIDENCE):
                continue
            observations.append((idx, heading, float(row[f"{sensor_name}_dist"])))

    if len(observations) < STATIONARY_SWEEP_SOLVE_MIN_OBS:
        return None

    min_x = field.min_x + field.field_margin
    max_x = field.max_x - field.field_margin
    min_y = field.min_y + field.field_margin
    max_y = field.max_y - field.field_margin

    def valid_xy(x: float, y: float) -> bool:
        return min_x <= x <= max_x and min_y <= y <= max_y

    def score_pose(x: float, y: float, heading_offset: float) -> tuple[float, float, int]:
        if not valid_xy(x, y):
            return float("inf"), float("inf"), 0
        residuals: list[float] = []
        for idx, heading, measured in observations:
            sensor = sensors[idx]
            pose = Pose(x, y, wrap_angle(heading + heading_offset))
            predicted = expected_distance(field, pose, sensor)
            if predicted > 0.0:
                residuals.append(measured - predicted)
        if len(residuals) < STATIONARY_SWEEP_SOLVE_MIN_OBS:
            return float("inf"), float("inf"), len(residuals)

        abs_errors = sorted(abs(residual) for residual in residuals)
        trimmed = abs_errors[:max(1, int(len(abs_errors) * 0.70))]
        median_abs = median(abs_errors)
        if median_abs is None:
            return float("inf"), float("inf"), len(residuals)
        score = (sum(trimmed) / len(trimmed)) + (0.3 * median_abs) + (0.1 * trimmed[-1])
        return score, median_abs, len(residuals)

    best_score = float("inf")
    best: tuple[float, float, float, float, int] | None = None

    for step in (12.0, 6.0, 3.0, 1.5, 0.75):
        if best is None:
            xs = [min_x + i * step for i in range(int((max_x - min_x) / step) + 1)]
            ys = [min_y + i * step for i in range(int((max_y - min_y) / step) + 1)]
            heading_step = 15.0
            headings = [math.radians(-180.0 + i * heading_step) for i in range(int(360.0 / heading_step) + 1)]
        else:
            best_x, best_y, best_heading, _, _ = best
            xs = [best_x + i * step for i in range(-4, 5)]
            ys = [best_y + i * step for i in range(-4, 5)]
            heading_step = 5.0 if step >= 3.0 else 2.0 if step >= 1.5 else 1.0
            headings = [best_heading + math.radians(i * heading_step) for i in range(-4, 5)]

        for x in xs:
            for y in ys:
                if not valid_xy(x, y):
                    continue
                for heading_offset in headings:
                    score, median_abs, count = score_pose(x, y, heading_offset)
                    if score < best_score:
                        best_score = score
                        best = (x, y, heading_offset, median_abs, count)

    if best is None or best_score > STATIONARY_SWEEP_SOLVE_MAX_SCORE:
        return None

    x, y, heading_offset, median_abs, count = best
    return x, y, math.degrees(heading_offset), best_score, median_abs, count


def sensor_measurements(
    rows: list[dict[str, float | int]],
    field: FieldConfig,
    sensor: SensorModel,
    sensor_name: str,
    pose_source: str,
    sample_limit: int,
) -> list[tuple[Pose, float]]:
    usable: list[dict[str, float | int]] = []
    last_reading_key: tuple[float, int, int] | None = None
    for row in rows:
        changed_key = f"{sensor_name}_changed"
        if changed_key in row and int(row[changed_key]) == 0:
            continue

        # The trace is written faster than the distance sensors refresh. Only use
        # rows where this sensor's reading changed so a held sample is not fitted
        # against several different turn headings.
        reading_key = (
            float(row[f"{sensor_name}_dist"]),
            int(row[f"{sensor_name}_conf"]),
            int(row[f"{sensor_name}_used"]),
        )
        if reading_key == last_reading_key:
            continue
        last_reading_key = reading_key

        if int(row[f"{sensor_name}_used"]) != 1:
            continue
        confidence = int(row[f"{sensor_name}_conf"])
        if confidence >= 0 and confidence < max(sensor.min_confidence, SENSOR_FIT_MIN_CONFIDENCE):
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
        if field_edge_margin(field, pose, sensor) < SENSOR_FIT_MIN_FIELD_EDGE_MARGIN:
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
    abs_errors = [abs(residual) for residual in residuals]
    median_abs_error = median(abs_errors)
    mean_abs_error = sum(abs_errors) / len(abs_errors)
    if median_abs_error is None:
        return float("inf"), bias
    score = median_abs_error + (0.15 * mean_abs_error) + (0.10 * abs(bias))
    return float(score), bias


def clamp_sensor_dx(sensor: SensorModel, dx: float) -> float:
    lower, upper = SENSOR_DX_LIMITS.get(sensor.idx, (None, None))
    if lower is not None:
        dx = max(lower, dx)
    if upper is not None:
        dx = min(upper, dx)
    return dx


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
    theta_limit = SENSOR_DTHETA_SEARCH_BOUNDS_DEG.get(best.idx, 10.0)

    for step_xy, step_theta in ((0.5, 4.0), (0.25, 2.0), (0.125, 1.0), (0.0625, 0.5), (0.0625, 0.25), (0.0625, 0.1)):
        improved = True
        while improved:
            improved = False
            candidates: list[SensorModel] = [best]
            for delta in (-step_xy, step_xy):
                candidates.append(
                    SensorModel(
                        **{
                            **best.__dict__,
                            "dx": clamp_sensor_dx(
                                best, max(current_dx - 2.0, min(current_dx + 2.0, best.dx + delta))
                            ),
                        }
                    )
                )
                candidates.append(
                    SensorModel(**{**best.__dict__, "dy": max(current_dy - 2.0, min(current_dy + 2.0, best.dy + delta))})
                )
            for delta in (-step_theta, step_theta):
                candidates.append(
                    SensorModel(
                        **{
                            **best.__dict__,
                            "dtheta_deg": max(
                                current_theta - theta_limit, min(current_theta + theta_limit, best.dtheta_deg + delta)
                            ),
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


def tune_sensor_dtheta_only(
    field: FieldConfig,
    sensor: SensorModel,
    measurements: list[tuple[Pose, float]],
) -> tuple[float, float, float]:
    best_theta = sensor.dtheta_deg
    best_score, best_bias = sensor_objective(field, sensor, measurements)
    theta_limit = SENSOR_DTHETA_SEARCH_BOUNDS_DEG.get(sensor.idx, 10.0)

    for step_theta in (2.0, 1.0, 0.5, 0.25, 0.1):
        improved = True
        while improved:
            improved = False
            for delta in (-step_theta, step_theta):
                candidate_theta = max(
                    sensor.dtheta_deg - theta_limit,
                    min(sensor.dtheta_deg + theta_limit, best_theta + delta),
                )
                candidate = SensorModel(**{**sensor.__dict__, "dtheta_deg": candidate_theta})
                score, bias = sensor_objective(field, candidate, measurements)
                if score + 1e-6 < best_score:
                    best_theta = candidate_theta
                    best_score = score
                    best_bias = bias
                    improved = True

    return best_theta, best_score, best_bias


def circular_span_deg(angles: list[float]) -> float:
    if len(angles) < 2:
        return 0.0
    wrapped = sorted(angle % (2.0 * math.pi) for angle in angles)
    gaps = [wrapped[i + 1] - wrapped[i] for i in range(len(wrapped) - 1)]
    gaps.append((wrapped[0] + 2.0 * math.pi) - wrapped[-1])
    return math.degrees((2.0 * math.pi) - max(gaps))


def sensor_fit_quality(
    field: FieldConfig,
    original: SensorModel,
    tuned: SensorModel,
    measurements: list[tuple[Pose, float]],
    improvement: float,
) -> SensorFitQuality:
    headings: list[float] = []
    hit_labels: set[str] = set()
    for pose, _ in measurements:
        _, _, _, _, heading = sensor_ray(pose, tuned)
        headings.append(heading)
        _, label = expected_hit(field, pose, tuned)
        if label != "none":
            hit_labels.add(label)

    search_bound = SENSOR_DTHETA_SEARCH_BOUNDS_DEG.get(original.idx, 10.0)
    angle_delta = tuned.dtheta_deg - original.dtheta_deg
    hit_search_edge = abs(angle_delta) >= search_bound - 0.51
    heading_span = circular_span_deg(headings)
    ordered_hits = sorted(hit_labels)

    if hit_search_edge:
        status = "angle underconstrained: estimate reached the search edge"
    elif heading_span < 30.0 and len(ordered_hits) < 2:
        status = "angle weakly constrained: run the sensor-angle test"
    elif improvement < 5.0 and abs(angle_delta) < 0.5:
        status = "angle already fits this log"
    else:
        status = "angle constrained by this log"

    return SensorFitQuality(
        heading_span_deg=heading_span,
        hit_labels=ordered_hits,
        hit_search_edge=hit_search_edge,
        status=status,
    )


def format_optional(value: float | None, precision: int = 4) -> str:
    if value is None or not math.isfinite(value):
        return "n/a"
    return f"{value:.{precision}f}"


CORRECTION_REJECT_BITS = {
    1 << 0: "turn_suppressed",
    1 << 1: "sensors_stale",
    1 << 2: "not_enough_sensors",
    1 << 3: "unstable_candidate",
    1 << 4: "pose_delta",
    1 << 5: "readiness",
    1 << 6: "nis",
    1 << 7: "no_measurement",
    1 << 8: "sensor_residual",
    1 << 9: "motion_suppressed",
}


def summarize_trace_diagnostics(rows: list[dict[str, float | int]]) -> None:
    if not rows:
        return

    print("Fusion diagnostics")
    if "odom_only_x" in rows[0]:
        separations: list[float] = []
        final_dx = final_dy = final_dtheta = 0.0
        for row in rows:
            try:
                dx = float(row["applied_x"]) - float(row["odom_only_x"])
                dy = float(row["applied_y"]) - float(row["odom_only_y"])
                dtheta = math.remainder(
                    math.radians(float(row["applied_theta_deg"]) - float(row["odom_only_theta_deg"])),
                    2.0 * math.pi,
                )
            except (KeyError, TypeError, ValueError):
                continue
            separations.append(math.hypot(dx, dy))
            final_dx, final_dy, final_dtheta = dx, dy, math.degrees(dtheta)
        if separations:
            print(
                f"  odom-only delta final applied-odom=({final_dx:+.3f}, {final_dy:+.3f}, {final_dtheta:+.3f} deg),"
                f" max_xy={max(separations):.3f} in"
            )
        else:
            print("  odom-only delta unavailable")
    else:
        print("  odom-only baseline missing; re-run with the current logger to compare fusion vs pure odom.")

    if "correction_reject_mask" in rows[0]:
        accepted = 0
        accepted_by_sensors: dict[int, int] = {}
        accepted_theta_sum_by_sensors: dict[int, float] = {}
        accepted_theta_max_by_sensors: dict[int, float] = {}
        mask_counts: dict[int, int] = {}
        bit_counts: dict[int, int] = {}
        for row in rows:
            mask = int(row.get("correction_reject_mask", 0))
            if int(row.get("correction_accepted", 0)) == 1:
                accepted += 1
                active = int(row.get("active_sensors", 0))
                accepted_by_sensors[active] = accepted_by_sensors.get(active, 0) + 1
                try:
                    applied_theta = float(row.get("applied_corr_theta_deg", 0.0))
                except (TypeError, ValueError):
                    applied_theta = 0.0
                accepted_theta_sum_by_sensors[active] = accepted_theta_sum_by_sensors.get(active, 0.0) + applied_theta
                accepted_theta_max_by_sensors[active] = max(
                    accepted_theta_max_by_sensors.get(active, 0.0), abs(applied_theta)
                )
            if mask:
                mask_counts[mask] = mask_counts.get(mask, 0) + 1
                for bit in CORRECTION_REJECT_BITS:
                    if mask & bit:
                        bit_counts[bit] = bit_counts.get(bit, 0) + 1
        print(f"  corrections accepted rows={accepted}")
        if accepted_by_sensors:
            counts = ", ".join(f"A{active}={count}" for active, count in sorted(accepted_by_sensors.items()))
            print(f"  accepted by live sensors: {counts}")
            heading_details = ", ".join(
                f"A{active} sum_theta={accepted_theta_sum_by_sensors.get(active, 0.0):+.3f} deg "
                f"max_step={accepted_theta_max_by_sensors.get(active, 0.0):.3f} deg"
                for active in sorted(accepted_by_sensors)
            )
            print(f"  accepted heading correction: {heading_details}")
        if "candidate_inlier_sensors" in rows[0]:
            accepted_rows = [row for row in rows if int(row.get("correction_accepted", 0)) == 1]
            if accepted_rows:
                inliers = [int(row.get("candidate_inlier_sensors", 0)) for row in accepted_rows]
                max_residuals = [float(row.get("candidate_max_residual", 0.0)) for row in accepted_rows]
                print(
                    "  accepted candidate fit:"
                    f" inliers min/mean/max={min(inliers)}/{sum(inliers) / len(inliers):.2f}/{max(inliers)},"
                    f" max_residual max={max(max_residuals):.3f} in"
                )
        if bit_counts:
            details = ", ".join(
                f"{CORRECTION_REJECT_BITS[bit]}={count}" for bit, count in sorted(bit_counts.items())
            )
            print(f"  rejection bits: {details}")
        else:
            print("  rejection bits: none")
    else:
        print("  correction rejection masks missing; re-run with the current logger to diagnose gates.")
    print()


def nearest_wall(field: FieldConfig, x: float, y: float) -> str:
    distances = [
        (abs(x - field.min_x), "min_x"),
        (abs(x - field.max_x), "max_x"),
        (abs(y - field.min_y), "min_y"),
        (abs(y - field.max_y), "max_y"),
    ]
    return min(distances, key=lambda item: item[0])[1]


def summarize_unmodeled_occlusions(
    rows: list[dict[str, float | int]],
    field: FieldConfig,
    sensors: dict[int, SensorModel],
    pose_source: str,
) -> bool:
    clusters_by_trace: dict[int, dict[tuple[int, int], dict[str, object]]] = {}
    hit_counts_by_trace: dict[int, int] = {}

    for row in rows:
        trace_index = int(row.get("_trace_index", 0))
        for idx, sensor_name in enumerate(SENSOR_NAMES):
            sensor = sensors.get(idx)
            if sensor is None:
                continue
            changed_key = f"{sensor_name}_changed"
            if changed_key in row and int(row[changed_key]) == 0:
                continue
            if int(row.get(f"{sensor_name}_used", 0)) != 1:
                continue
            confidence = int(row.get(f"{sensor_name}_conf", -1))
            if confidence >= 0 and confidence < max(sensor.min_confidence, SENSOR_FIT_MIN_CONFIDENCE):
                continue
            if int(row.get("mcl_valid", 1)) != 1 or int(row.get("sensors_stale", 0)) != 0:
                continue

            pose = pose_from_row(row, pose_source)
            measured = float(row[f"{sensor_name}_dist"])
            predicted = expected_distance(field, pose, sensor)
            if predicted <= 0.0:
                continue
            residual = measured - predicted
            if residual > -UNMODELED_OCCLUSION_RESIDUAL_IN:
                continue

            sx, sy, dir_x, dir_y, _ = sensor_ray(pose, sensor)
            hit_x = sx + measured * dir_x
            hit_y = sy + measured * dir_y
            bin_x = int(round(hit_x / UNMODELED_OCCLUSION_BIN_IN) * UNMODELED_OCCLUSION_BIN_IN)
            bin_y = int(round(hit_y / UNMODELED_OCCLUSION_BIN_IN) * UNMODELED_OCCLUSION_BIN_IN)
            key = (bin_x, bin_y)

            trace_clusters = clusters_by_trace.setdefault(trace_index, {})
            cluster = trace_clusters.setdefault(
                key,
                {"count": 0, "sensors": set(), "residuals": [], "wall": nearest_wall(field, hit_x, hit_y)},
            )
            cluster["count"] = int(cluster["count"]) + 1
            cluster["sensors"].add(sensor_name)
            cluster["residuals"].append(residual)
            hit_counts_by_trace[trace_index] = hit_counts_by_trace.get(trace_index, 0) + 1

    flagged = False
    print("Unmodeled occlusion diagnostics")
    if not hit_counts_by_trace:
        print("  no clustered high-confidence early wall hits detected")
        print()
        return False

    for trace_index in sorted(hit_counts_by_trace):
        total_hits = hit_counts_by_trace[trace_index]
        clusters = sorted(
            clusters_by_trace.get(trace_index, {}).items(),
            key=lambda item: int(item[1]["count"]),
            reverse=True,
        )
        top_count = int(clusters[0][1]["count"]) if clusters else 0
        if total_hits >= UNMODELED_OCCLUSION_MIN_HITS and top_count >= UNMODELED_OCCLUSION_MIN_CLUSTER_HITS:
            flagged = True
            status = "likely unmodeled object"
        else:
            status = "below warning threshold"
        print(f"  trace #{trace_index}: early_hits={total_hits} top_cluster={top_count} status={status}")
        for (bin_x, bin_y), cluster in clusters[:3]:
            residuals = [float(value) for value in cluster["residuals"]]
            sensor_text = ",".join(sorted(str(sensor) for sensor in cluster["sensors"]))
            print(
                f"    hit_cluster x~{bin_x:+d} y~{bin_y:+d}"
                f" hits={int(cluster['count'])} sensors={sensor_text}"
                f" nearest_wall={cluster['wall']} median_res={median(residuals):+.2f} in"
            )
    if flagged:
        print("  status: exclude or model these traces before applying sensor-geometry constants")
    print()
    return flagged


def summarize_calibration_coverage(rows: list[dict[str, float | int]]) -> None:
    starts_by_trace: dict[int, Pose] = {}
    for row in rows:
        if "stationary_start_x" not in row or "stationary_start_y" not in row:
            continue
        trace_index = int(row.get("_trace_index", 0))
        if trace_index in starts_by_trace:
            continue
        starts_by_trace[trace_index] = Pose(
            x=float(row["stationary_start_x"]),
            y=float(row["stationary_start_y"]),
            theta=0.0,
        )

    print("Calibration coverage")
    if not starts_by_trace:
        print("  no stationary relocalized starts found; sensor geometry fit is not globally validated")
        print()
        return

    starts = list(starts_by_trace.items())
    xs = [pose.x for _, pose in starts]
    ys = [pose.y for _, pose in starts]
    x_span = max(xs) - min(xs)
    y_span = max(ys) - min(ys)
    start_text = ", ".join(f"#{trace_index}:{pose.x:+.1f},{pose.y:+.1f}" for trace_index, pose in starts[:8])
    if len(starts) > 8:
        start_text += ", ..."
    print(
        f"  stationary starts={len(starts)} x_span={x_span:.1f} in y_span={y_span:.1f} in"
        f" starts=[{start_text}]"
    )
    if len(starts) < GLOBAL_FIT_MIN_STARTS:
        print(
            "  status: location-specific only; aggregate more Test 5 logs before treating sensor geometry as field-wide"
        )
    elif x_span < GLOBAL_FIT_MIN_START_SPAN_IN or y_span < GLOBAL_FIT_MIN_START_SPAN_IN:
        print(
            "  status: weak field coverage; starts need wider X and Y spread before calling the fit global"
        )
    else:
        print("  status: field-wide coverage is adequate for global sensor-geometry fitting")
    print()


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze a localization tune trace and suggest odom / sensor updates.")
    parser.add_argument("trace", nargs="+", type=Path, help="Path(s) to localization_tune_latest.txt logs")
    parser.add_argument(
        "--pose-source",
        choices=("applied", "ekf", "mcl", "odom_only"),
        default="applied",
        help="Pose source used as the offline reference trajectory for odom diagnostics",
    )
    parser.add_argument(
        "--sensor-pose-source",
        choices=("applied", "ekf", "mcl", "odom_only", "stationary_start"),
        default=None,
        help="Pose source used for distance-sensor geometry fits; Test 5 defaults to stationary_start when available",
    )
    parser.add_argument(
        "--sensor-sample-limit",
        type=int,
        default=1500,
        help="Maximum number of per-sensor measurements to evaluate during bounded offset search",
    )
    args = parser.parse_args()

    field, odom_model, sensors, rows = parse_traces(args.trace)
    print(f"Trace files: {len(args.trace)}")
    print(f"Trace rows: {len(rows)}")
    print(f"Pose source: {args.pose_source}")
    sensor_pose_source = args.sensor_pose_source
    if sensor_pose_source is None:
        if any("stationary_start_x" in row for row in rows):
            sensor_pose_source = "stationary_start"
        else:
            sensor_pose_source = "odom_only" if rows and "odom_only_x" in rows[0] else args.pose_source
    print(f"Sensor pose source: {sensor_pose_source}")

    odom_pairs = build_odom_pairs(rows, args.pose_source)
    print(f"Reliable odom pairs: {len(odom_pairs)}")
    print()

    summarize_calibration_coverage(rows)
    summarize_trace_diagnostics(rows)
    occlusion_suspected = False
    if field is not None and sensors:
        occlusion_suspected = summarize_unmodeled_occlusions(rows, field, sensors, sensor_pose_source)

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
    if occlusion_suspected:
        print(
            "  WARNING: likely unmodeled occluder traces are present; do not apply these constants until those logs are excluded or modeled."
        )
    for idx, sensor_name in enumerate(SENSOR_NAMES):
        sensor = sensors.get(idx)
        if sensor is None:
            print(f"  {sensor_name}: missing metadata")
            continue
        measurements = sensor_measurements(rows, field, sensor, sensor_name, sensor_pose_source, args.sensor_sample_limit)
        if len(measurements) < 25:
            print(f"  {sensor_name}: not enough usable samples ({len(measurements)})")
            continue

        current_score, current_bias = sensor_objective(field, sensor, measurements)
        tuned_sensor, tuned_score, tuned_bias = tune_sensor(field, sensor, measurements)
        theta_only, theta_only_score, theta_only_bias = tune_sensor_dtheta_only(field, sensor, measurements)
        improvement = 0.0
        if math.isfinite(current_score) and current_score > 1e-6:
            improvement = 100.0 * (current_score - tuned_score) / current_score
        quality = sensor_fit_quality(field, sensor, tuned_sensor, measurements, improvement)
        search_bound = SENSOR_DTHETA_SEARCH_BOUNDS_DEG.get(sensor.idx, 10.0)
        hit_text = ",".join(quality.hit_labels) if quality.hit_labels else "none"

        print(
            f"  {sensor_name}: bias {current_bias:+.3f} -> {tuned_bias:+.3f} in,"
            f" score {current_score:.3f} -> {tuned_score:.3f},"
            f" samples {len(measurements)}, improvement {improvement:.1f}%"
        )
        print(
            f"    dx {sensor.dx:.3f} -> {tuned_sensor.dx:.3f},"
            f" dy {sensor.dy:.3f} -> {tuned_sensor.dy:.3f},"
            f" dtheta {sensor.dtheta_deg:.2f} -> {tuned_sensor.dtheta_deg:.2f} deg"
        )
        print(
            f"    angle fit: runtime uses the exact dtheta above; offline optimizer search bound ±{search_bound:.0f} deg,"
            f" heading span {quality.heading_span_deg:.1f} deg, predicted hits {hit_text}; {quality.status}"
        )
        print(
            f"    dtheta-only check {sensor.dtheta_deg:.2f} -> {theta_only:.2f} deg,"
            f" score {current_score:.3f} -> {theta_only_score:.3f}, bias {current_bias:+.3f} -> {theta_only_bias:+.3f}"
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
