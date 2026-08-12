import math

import pytest

from host.scoring import score_rows


def row(timestamp, error, output, dt=10):
    return {
        "timestamp_ms": str(timestamp),
        "error_deg": str(error),
        "output": str(output),
        "loop_dt_ms": str(dt),
        "left_angle_deg": str(-timestamp),
        "right_angle_deg": str(timestamp),
    }


def test_completed_trial_metrics_and_score():
    rows = [
        row(0, -2, -20, 0),
        row(10, -1, -10),
        row(20, 1, 10),
        row(30, 2, 100),
    ]
    metrics = score_rows(
        rows,
        ["TRIAL_COMPLETE,5000"],
        trial_duration_s=5.0,
        output_limit=100.0,
    )

    assert metrics["completed"] is True
    assert metrics["survival_s"] == 5.0
    assert metrics["rms_angle_error_deg"] == pytest.approx(math.sqrt(2.5))
    assert metrics["mean_angle_error_deg"] == 0.0
    assert metrics["max_abs_angle_error_deg"] == 2.0
    assert metrics["saturation_fraction"] == 0.25
    assert metrics["actuator_engaged"] is True
    assert metrics["zero_crossings"] == 1
    assert metrics["average_loop_dt_ms"] == 10.0
    assert metrics["score"] == pytest.approx(500 - 10 * math.sqrt(2.5) - 25)


def test_fall_time_dominates_shorter_clean_trial():
    long_fallen = score_rows(
        [row(0, 4, 20, 0), row(3000, 4, 20)],
        ["FALLEN,3200,12.1"],
        trial_duration_s=5.0,
        output_limit=100.0,
    )
    short_clean = score_rows(
        [row(0, 0.1, 1, 0), row(1000, 0.1, 1)],
        ["FALLEN,1200,12.1"],
        trial_duration_s=5.0,
        output_limit=100.0,
    )

    assert long_fallen["score"] > short_clean["score"]


def test_empty_trial_is_rejected():
    with pytest.raises(ValueError, match="without telemetry"):
        score_rows([], [], trial_duration_s=5.0, output_limit=100.0)


def test_single_sample_fall_has_defined_timing_metrics():
    metrics = score_rows(
        [row(0, 3.793, 56.84, 0)],
        ["FALLEN,40,2.801,projected_error=-11.468"],
        trial_duration_s=5.0,
        output_limit=100.0,
    )

    assert metrics["samples"] == 1
    assert metrics["initial_abs_angle_error_deg"] == 3.793
    assert metrics["average_loop_dt_ms"] == 0.0
    assert metrics["min_loop_dt_ms"] == 0.0
    assert metrics["max_loop_dt_ms"] == 0.0
    assert metrics["recovered_toward_upright"] is False
    assert metrics["no_recovery_penalty"] == 200.0


def test_fallen_trial_can_demonstrate_recovery():
    rows = [row(0, -2.2, -40, 0)]
    rows.append(row(40, -1.0, -20))
    metrics = score_rows(
        rows,
        ["FALLEN,500,12.1"],
        trial_duration_s=5.0,
        output_limit=100.0,
    )

    assert metrics["recovered_toward_upright"] is True
    assert metrics["no_recovery_penalty"] == 0.0


def test_passive_survival_gets_no_actuation_penalty():
    rows = [row(0, 1, 4, 0), row(10, 1, 4)]
    rows[1]["left_angle_deg"] = "1"
    rows[1]["right_angle_deg"] = "1"
    metrics = score_rows(
        rows,
        ["TRIAL_COMPLETE,5000"],
        trial_duration_s=5.0,
        output_limit=100.0,
    )

    assert metrics["actuator_engaged"] is False
    assert metrics["no_actuation_penalty"] == 250.0
    assert metrics["score"] == 240.0
