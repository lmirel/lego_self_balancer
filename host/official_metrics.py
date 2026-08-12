"""Centering and settling metrics for official-reference telemetry."""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Mapping


def _rms(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values))


def _event_value(events: list[str], name: str, field: str) -> float | None:
    prefix = name + ","
    for event in events:
        if not event.startswith(prefix):
            continue
        for item in event.split(",")[1:]:
            if item.startswith(field + "="):
                return float(item.split("=", 1)[1])
    return None


def official_metrics(
    rows: list[Mapping[str, str]], events: list[str]
) -> dict[str, float | int | bool | None]:
    """Measure survival, centering drift, and oscillation-envelope change."""
    if not rows:
        raise ValueError("cannot analyse official reference without telemetry")

    times = [float(row["timestamp_ms"]) / 1000.0 for row in rows]
    angles = [float(row["relative_angle_deg"]) for row in rows]
    positions = [float(row["wheel_position_deg"]) for row in rows]
    speeds = [float(row["wheel_speed_dps"]) for row in rows]
    duties = [float(row["duty"]) for row in rows]
    rates = [float(row["gyro_x_dps"]) for row in rows]
    trims = [float(row.get("angle_trim_deg", 0.0)) for row in rows]
    desired_leans = [float(row.get("desired_lean_deg", 0.0)) for row in rows]
    window = max(1, len(rows) // 5)
    early_angles = angles[:window]
    late_angles = angles[-window:]
    late_positions = positions[-window:]
    elapsed = max(times[-1] - times[0], 1e-9)

    completed = any(event.startswith("TRIAL_COMPLETE,") for event in events)
    terminal_ms = None
    for event in events:
        fields = event.split(",")
        if fields[0] in ("TRIAL_COMPLETE", "FALLEN", "ABORTED"):
            terminal_ms = int(fields[1])
            break
    survival_s = terminal_ms / 1000.0 if terminal_ms is not None else times[-1]

    crossings = 0
    prior_sign = 0
    for angle in angles:
        sign = 1 if angle > 0 else -1 if angle < 0 else 0
        if sign and prior_sign and sign != prior_sign:
            crossings += 1
        if sign:
            prior_sign = sign

    early_rms = _rms(early_angles)
    late_rms = _rms(late_angles)
    return {
        "samples": len(rows),
        "completed": completed,
        "survival_s": round(survival_s, 6),
        "gyro_bias_dps": _event_value(events, "GYRO_BIAS", "dps"),
        "rms_angle_deg": round(_rms(angles), 6),
        "early_rms_angle_deg": round(early_rms, 6),
        "late_rms_angle_deg": round(late_rms, 6),
        "settling_ratio": round(late_rms / max(early_rms, 1e-9), 6),
        "mean_late_angle_deg": round(sum(late_angles) / len(late_angles), 6),
        "final_angle_trim_deg": round(trims[-1], 6),
        "max_abs_angle_trim_deg": round(max(abs(value) for value in trims), 6),
        "final_desired_lean_deg": round(desired_leans[-1], 6),
        "max_abs_desired_lean_deg": round(
            max(abs(value) for value in desired_leans), 6
        ),
        "zero_crossings": crossings,
        "final_wheel_position_deg": round(positions[-1], 6),
        "mean_late_wheel_position_deg": round(
            sum(late_positions) / len(late_positions), 6
        ),
        "wheel_position_drift_dps": round(
            (positions[-1] - positions[0]) / elapsed, 6
        ),
        "rms_wheel_speed_dps": round(_rms(speeds), 6),
        "max_abs_rate_dps": round(max(abs(value) for value in rates), 6),
        "rate_spike_samples": sum(abs(value) >= 50.0 for value in rates),
        "rms_duty": round(_rms(duties), 6),
        "max_abs_duty": round(max(abs(value) for value in duties), 6),
    }


def official_metrics_csv(
    telemetry_path: Path, events: list[str]
) -> dict[str, float | int | bool | None]:
    with telemetry_path.open(newline="", encoding="utf-8") as telemetry_file:
        return official_metrics(list(csv.DictReader(telemetry_file)), events)
