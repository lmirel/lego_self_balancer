"""Deterministic Phase 5 metrics and scoring for recorded trials."""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Iterable, Mapping


def _rms(values: Iterable[float]) -> float:
    values = list(values)
    return math.sqrt(sum(value * value for value in values) / len(values))


def _survival_seconds(
    events: list[str], rows: list[Mapping[str, str]], trial_duration_s: float
) -> tuple[float, bool]:
    for event in events:
        fields = event.split(",")
        if fields[0] == "TRIAL_COMPLETE":
            return int(fields[1]) / 1000.0, True
        if fields[0] in ("FALLEN", "ABORTED"):
            return int(fields[1]) / 1000.0, False
    if rows:
        return int(rows[-1]["timestamp_ms"]) / 1000.0, False
    return 0.0, False


def _recovered_toward_upright(rows: list[Mapping[str, str]], completed: bool) -> bool:
    """Require a real excursion to be arrested and brought near upright again."""
    if completed:
        return True
    excursion_seen = False
    for row in rows:
        error = abs(float(row["error_deg"]))
        if error >= 2.0:
            excursion_seen = True
        if excursion_seen and error <= 1.5:
            return True
    return False


def score_rows(
    rows: list[Mapping[str, str]],
    events: list[str],
    *,
    trial_duration_s: float,
    output_limit: float,
) -> dict[str, float | int | bool]:
    """Calculate reproducible metrics and a deliberately simple first score."""
    if not rows:
        raise ValueError("cannot score a trial without telemetry rows")

    errors = [float(row["error_deg"]) for row in rows]
    outputs = [float(row["output"]) for row in rows]
    loop_periods = [float(row["loop_dt_ms"]) for row in rows[1:]]
    if not loop_periods:
        loop_periods = [float(rows[0]["loop_dt_ms"])]
    survival_s, completed = _survival_seconds(events, rows, trial_duration_s)
    recovered_toward_upright = _recovered_toward_upright(rows, completed)

    zero_crossings = 0
    previous_sign = 0
    for error in errors:
        sign = 1 if error > 0 else -1 if error < 0 else 0
        if sign and previous_sign and sign != previous_sign:
            zero_crossings += 1
        if sign:
            previous_sign = sign

    saturated = sum(abs(output) >= output_limit * 0.99 for output in outputs)
    saturation_fraction = saturated / len(outputs)
    rms_error = _rms(errors)
    mean_error = sum(errors) / len(errors)
    motor_travel = 0.0
    rms_wheel_speed = 0.0
    rms_wheel_position = 0.0
    max_abs_wheel_position = 0.0
    if "wheel_speed_dps" in rows[0]:
        rms_wheel_speed = _rms(float(row["wheel_speed_dps"]) for row in rows)
    if "wheel_position_deg" in rows[0]:
        wheel_positions = [float(row["wheel_position_deg"]) for row in rows]
        rms_wheel_position = _rms(wheel_positions)
        max_abs_wheel_position = max(abs(value) for value in wheel_positions)
    if "left_angle_deg" in rows[0] and "right_angle_deg" in rows[0]:
        left_angles = [float(row["left_angle_deg"]) for row in rows]
        right_angles = [float(row["right_angle_deg"]) for row in rows]
        left_travel = sum(abs(after - before) for before, after in zip(left_angles, left_angles[1:]))
        right_travel = sum(abs(after - before) for before, after in zip(right_angles, right_angles[1:]))
        motor_travel = (left_travel + right_travel) / 2.0
    actuator_engaged = motor_travel >= 20.0
    no_actuation_penalty = 0.0 if actuator_engaged else 250.0
    no_recovery_penalty = 0.0 if recovered_toward_upright else 200.0

    # Survival dominates. Error and saturation provide understandable tie-breaks.
    score = (
        100.0 * survival_s
        - 10.0 * rms_error
        - 100.0 * saturation_fraction
        - no_actuation_penalty
        - no_recovery_penalty
    )

    return {
        "samples": len(rows),
        "initial_abs_angle_error_deg": round(abs(errors[0]), 6),
        "completed": completed,
        "survival_s": round(survival_s, 6),
        "rms_angle_error_deg": round(rms_error, 6),
        "mean_angle_error_deg": round(mean_error, 6),
        "max_abs_angle_error_deg": round(max(abs(error) for error in errors), 6),
        "rms_output": round(_rms(outputs), 6),
        "motor_travel_deg": round(motor_travel, 6),
        "rms_wheel_speed_dps": round(rms_wheel_speed, 6),
        "rms_wheel_position_deg": round(rms_wheel_position, 6),
        "max_abs_wheel_position_deg": round(max_abs_wheel_position, 6),
        "actuator_engaged": actuator_engaged,
        "recovered_toward_upright": recovered_toward_upright,
        "no_actuation_penalty": no_actuation_penalty,
        "no_recovery_penalty": no_recovery_penalty,
        "saturation_fraction": round(saturation_fraction, 6),
        "zero_crossings": zero_crossings,
        "average_loop_dt_ms": round(sum(loop_periods) / len(loop_periods), 6),
        "min_loop_dt_ms": min(loop_periods),
        "max_loop_dt_ms": max(loop_periods),
        "score": round(score, 6),
    }


def score_csv(
    telemetry_path: Path,
    events: list[str],
    *,
    trial_duration_s: float,
    output_limit: float,
) -> dict[str, float | int | bool]:
    with telemetry_path.open(newline="", encoding="utf-8") as telemetry_file:
        rows = list(csv.DictReader(telemetry_file))
    return score_rows(
        rows,
        events,
        trial_duration_s=trial_duration_s,
        output_limit=output_limit,
    )
