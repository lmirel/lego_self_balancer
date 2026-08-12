import csv
from pathlib import Path

from host.tune import (
    OFFICIAL_REFERENCE_PROGRAM,
    TrialRecorder,
    choose_next_kd,
    is_unsafe_trial,
    is_valid_tuning_start,
    propose_candidate,
    render_official_reference,
    render_trial_files,
)


HEADER = (
    "timestamp_ms,angle_deg,error_deg,gyro_x_dps,filtered_gyro_x_dps,p_term,i_term,d_term,output,"
    "left_angle_deg,right_angle_deg,loop_dt_ms,state\n"
)


def test_recorder_preserves_raw_and_extracts_telemetry(tmp_path: Path):
    recorder = TrialRecorder(tmp_path)
    lines = [
        "Searching for any hub with Pybricks service...\n",
        "READY,hold robot near 87.14 deg and press CENTER\n",
        "ARMED,angle_deg=87.200,error_deg=0.060\n",
        HEADER,
        "0,87.200,0.060,1.00,1.00,0.45,0.00,0.20,0.65,0,0,0,RUNNING\n",
        "40,87.100,-0.040,-1.00,-0.25,-0.30,0.00,-0.20,-0.50,-1,1,10,RUNNING\n",
        "TRIAL_COMPLETE,5000\n",
        "TIMING,avg_ms=10.000,min_ms=9,max_ms=11,late=0\n",
        "STOPPED,state=COMPLETE\n",
    ]
    for line in lines:
        recorder.handle_line(line)
    recorder.close()

    assert recorder.telemetry_rows == 2
    assert recorder.terminal_event == "TRIAL_COMPLETE"
    assert (tmp_path / "hub-output.log").read_text() == "".join(lines)

    with (tmp_path / "telemetry.csv").open(newline="") as telemetry_file:
        rows = list(csv.DictReader(telemetry_file))
    assert [row["timestamp_ms"] for row in rows] == ["0", "40"]
    assert rows[1]["state"] == "RUNNING"


def test_recorder_captures_official_gyro_bias_event(tmp_path: Path):
    recorder = TrialRecorder(tmp_path)
    recorder.handle_line("GYRO_BIAS,dps=-0.1250,samples=160\n")
    recorder.close()

    assert recorder.events == ["GYRO_BIAS,dps=-0.1250,samples=160"]


def test_recorder_ignores_nontelemetry_and_malformed_rows(tmp_path: Path):
    recorder = TrialRecorder(tmp_path)
    recorder.handle_line(HEADER)
    recorder.handle_line("progress 100%\n")
    recorder.handle_line("1,too,few,columns\n")
    recorder.handle_line("FALLEN,1234,12.5\n")
    recorder.close()

    assert recorder.telemetry_rows == 0
    assert recorder.terminal_event == "FALLEN"


def test_only_early_or_saturated_falls_count_as_unsafe():
    ordinary_fall = {"survival_s": 1.24, "saturation_fraction": 0.0}
    early_fall = {"survival_s": 0.49, "saturation_fraction": 0.0}
    saturated_fall = {"survival_s": 1.0, "saturation_fraction": 0.25}

    assert not is_unsafe_trial("FALLEN", ordinary_fall)
    assert is_unsafe_trial("FALLEN", early_fall)
    assert is_unsafe_trial("FALLEN", saturated_fall)
    assert not is_unsafe_trial("TRIAL_COMPLETE", saturated_fall)


def test_tuning_start_must_remain_inside_arming_tolerance():
    assert is_valid_tuning_start({"initial_abs_angle_error_deg": 2.5}, 2.5)
    assert not is_valid_tuning_start({"initial_abs_angle_error_deg": 3.793}, 2.5)


def test_session_rendering_changes_only_session_gains(tmp_path: Path):
    program, config = render_trial_files(tmp_path, 8.25, 0.35, auto_arm=True)

    text = program.read_text()
    assert "KP = 8.25" in text
    assert "KI = 0.0" in text
    assert "KD = 0.35" in text
    assert "KW = 0.0" in text
    assert "KX = 0.0" in text
    assert "AUTO_ARM = True" in text
    assert "OUTPUT_LIMIT = 1000.0" in text
    assert "left_motor.run(LEFT_MOTOR_SIGN * output)" in text
    assert "right_motor.run(RIGHT_MOTOR_SIGN * output)" in text
    assert "speed_term = -KW * wheel_speed_dps" in text
    assert "position_term = -KX * wheel_position_deg" in text
    assert "PREDICTED_FALL_MIN_ERROR_DEG = 3.0" in text
    assert "GYRO_FILTER_ALPHA = 0.386" in text
    assert config["controller"]["kp"] == 8.25
    assert config["controller"]["kd"] == 0.35


def test_reference_controller_renders_discrete_visual_program_equations(tmp_path: Path):
    program, config = render_trial_files(
        tmp_path, 5.5, 4.0, ki=2.1, target_angle_deg=88.95,
        mode="REFERENCE", auto_arm=True,
    )
    text = program.read_text()

    assert 'CONTROLLER_MODE = "REFERENCE"' in text
    assert "TARGET_ANGLE_DEG = 88.95" in text
    assert "reference_integral += error_deg * REFERENCE_INTEGRAL_STEP" in text
    assert "reference_derivative = error_deg - reference_last_error" in text
    assert "REFERENCE_POWER_SCALE = 2.5" in text
    assert config["controller"]["mode"] == "REFERENCE"


def test_official_reference_preserves_core_pybricks_control_structure():
    text = OFFICIAL_REFERENCE_PROGRAM.read_text()

    assert "DT_MS = 5" in text
    assert "SPEED_WINDOW_MS = 200" in text
    assert "gyro_bias, upright_roll = wait_for_stable_countdown(hub, left, right)" in text
    assert "rate = raw_rate - gyro_bias" in text
    assert "relative_angle += rate * DT_MS / 1000.0" in text
    assert "ABSOLUTE_ANGLE_CORRECTION_TAU_S = 5.0" in text
    assert "absolute_angle = hub.imu.tilt()[1] - upright_roll" in text
    assert "RATE_GAIN = 0.018" in text
    assert "ANGLE_GAIN = 19.0" in text
    assert "POSITION_GAIN = 0.45" in text
    assert "SPEED_GAIN = 0.20" in text
    assert "angle_term = ANGLE_GAIN * relative_angle" in text
    assert "position_term = POSITION_GAIN * (position - commanded_position)" in text
    assert "STABLE_RATE_DPS = 3.0" in text
    assert "STABLE_WHEEL_STEP_DEG = 1" in text
    assert "NOMINAL_VOLTAGE_MV / battery_mv" in text
    assert "COMMAND_SPEED_DPS = 0.0" in text
    assert "COMMAND_TURN_DUTY = 0.0" in text
    assert "left.dc(LEFT_SIGN * (duty - COMMAND_TURN_DUTY))" in text
    assert "right.dc(RIGHT_SIGN * (duty + COMMAND_TURN_DUTY))" in text
    assert "DEADBAND_FADE_SPEED_DPS = 120.0" in text
    assert "duty = raw_duty + compensation" in text


def test_official_reference_renders_isolated_stationary_gain_pair():
    text = render_official_reference(
        0.4, 0.14, 200, 17.0, 12.0, 0.1, 10000, 10.0
    )

    assert "RATE_GAIN = 0.1" in text
    assert "ANGLE_GAIN = 17.0" in text
    assert "POSITION_GAIN = 0.4" in text
    assert "SPEED_GAIN = 0.14" in text
    assert "SPEED_WINDOW_MS = 200" in text
    assert "DEADBAND_COMPENSATION = 12.0" in text
    assert "TRIAL_DURATION_MS = 10000" in text
    assert "ABSOLUTE_ANGLE_CORRECTION_TAU_S = 10.0" in text
    assert "POSITION_GAIN = 0.45" not in text
    assert "SPEED_GAIN = 0.16" not in text


def test_candidate_is_a_bounded_d_step():
    config = {"controller": {"kp": 7.5, "kd": 0.2}}
    assert propose_candidate(config) == (7.5, 0.25)

    config["controller"]["kd"] = 0.5
    assert propose_candidate(config) == (7.5, 0.5)


def test_next_candidate_refines_or_continues():
    assert choose_next_kd(0.2, 0.25, 460, 470) == (
        0.225,
        "score did not beat the prior best; refine between both values",
    )
    assert choose_next_kd(0.2, 0.25, 480, 470) == (
        0.3,
        "score improved; continue the same bounded D direction",
    )
