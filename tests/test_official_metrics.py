import pytest

from host.official_metrics import official_metrics


def official_row(
    timestamp, angle, position, speed=0, duty=0, rate=0, trim=0, desired_lean=0
):
    return {
        "timestamp_ms": str(timestamp),
        "relative_angle_deg": str(angle),
        "angle_trim_deg": str(trim),
        "desired_lean_deg": str(desired_lean),
        "gyro_x_dps": str(rate),
        "wheel_position_deg": str(position),
        "wheel_speed_dps": str(speed),
        "duty": str(duty),
    }


def test_official_metrics_separate_centering_and_settling():
    rows = [
        official_row(0, -4, 0, -100, -40, -60),
        official_row(1000, 4, 20, 100, 40, 60),
        official_row(2000, -2, 30, -50, -20, -20),
        official_row(3000, 2, 35, 50, 20, 20),
        official_row(4000, -1, 40, -20, -10, -10),
        official_row(5000, 1, 50, 20, 10, 10),
    ]
    metrics = official_metrics(
        rows,
        ["GYRO_BIAS,dps=-0.1250,samples=160", "TRIAL_COMPLETE,5000"],
    )

    assert metrics["completed"] is True
    assert metrics["gyro_bias_dps"] == -0.125
    assert metrics["survival_s"] == 5.0
    assert metrics["settling_ratio"] == pytest.approx(0.25)
    assert metrics["final_wheel_position_deg"] == 50.0
    assert metrics["wheel_position_drift_dps"] == 10.0
    assert metrics["rate_spike_samples"] == 2
    assert metrics["final_angle_trim_deg"] == 0.0
    assert metrics["final_desired_lean_deg"] == 0.0


def test_official_metrics_reject_empty_trial():
    with pytest.raises(ValueError, match="without telemetry"):
        official_metrics([], [])
