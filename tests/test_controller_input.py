from pathlib import Path

from apps.ps_controller.control import (
    AxisProcessor,
    LowPass,
    SlewLimiter,
    apply_deadzone,
    apply_expo,
)
from apps.ps_controller.protocol import (
    format_command,
    format_config,
    is_newer_sequence,
)


def test_deadzone_is_zero_and_rescales_remaining_range():
    assert apply_deadzone(0.05, 0.08) == 0.0
    assert apply_deadzone(-0.08, 0.08) == 0.0
    assert apply_deadzone(1.0, 0.08) == 1.0
    assert apply_deadzone(-1.0, 0.08) == -1.0


def test_expo_preserves_sign_and_endpoints():
    assert apply_expo(1.0, 0.35) == 1.0
    assert apply_expo(-1.0, 0.35) == -1.0
    assert 0.0 < apply_expo(0.5, 0.35) < 0.5


def test_low_pass_moves_towards_sample():
    low_pass = LowPass(0.25)
    assert low_pass.update(1.0) == 0.25
    assert low_pass.update(1.0) == 0.4375


def test_slew_limiter_bounds_change_per_second():
    limiter = SlewLimiter(2.0)
    assert limiter.update(1.0, 0.1) == 0.2
    assert limiter.update(-1.0, 0.1) == 0.0


def test_axis_processor_suppresses_small_idle_noise():
    processor = AxisProcessor(deadzone=0.08)
    for _ in range(10):
        assert processor.update(0.04, 1 / 60) == 0.0


def test_ble_command_is_compact_and_clamps_values():
    command = format_command(0xFFFF, -2.0, 2.0)
    assert command == "C,ffff,-1.00,1.00"
    assert len((command + "\n").encode()) <= 20


def test_sequence_comparison_handles_wrap_and_rejects_stale_packets():
    assert is_newer_sequence(10, None)
    assert is_newer_sequence(11, 10)
    assert is_newer_sequence(0, 0xFFFF)
    assert not is_newer_sequence(10, 10)
    assert not is_newer_sequence(9, 10)


def test_runtime_limits_message_is_compact_and_bounded():
    config = format_config(300.0, 20.0)
    assert config == "S,300.0,20.0"
    assert len((config + "\n").encode()) <= 20

    import pytest

    assert format_config(1000.0, 8.0) == "S,1000.0,8.0"
    with pytest.raises(ValueError):
        format_config(0.0, 8.0)
    with pytest.raises(ValueError):
        format_config(float("inf"), 8.0)
    with pytest.raises(ValueError):
        format_config(300.0, 20.1)


def test_suspended_steering_test_is_low_duty_and_drive_independent():
    program = (
        Path(__file__).parents[1]
        / "apps" / "ps_controller" / "hub" / "steering_test.py"
    ).read_text()
    assert "MIN_TURN_DUTY = 30.0" in program
    assert "MAX_TURN_DUTY = 45.0" in program
    assert "TURN_NEUTRAL = 0.05" in program
    assert "last_sequence, ignored_drive, turn = command" in program
    assert "MAX_TURN_DUTY - MIN_TURN_DUTY" in program
    assert "left.dc(LEFT_SIGN * turn_duty)" in program
    assert "right.dc(RIGHT_SIGN * -turn_duty)" in program
    assert "WATCHDOG_MS = 250" in program
    assert "finally:\n    stop_motors(left, right)" in program


def test_remote_balance_preserves_locked_controller_and_bounds_commands():
    program = (
        Path(__file__).parents[1]
        / "apps" / "ps_controller" / "hub" / "balance_steering.py"
    ).read_text()
    for constant in (
        "DT_MS = 5",
        "SPEED_WINDOW_MS = 200",
        "RATE_GAIN = 0.018",
        "ANGLE_GAIN = 19.0",
        "POSITION_GAIN = 0.45",
        "SPEED_GAIN = 0.20",
        "REVERSAL_RELEASE_SPEED_DPS = 60.0",
        "REVERSAL_RELEASE_REFERENCE_DPS = 15.0",
        "LIFT_SPEED_DPS = 400.0",
        "LIFT_MAX_ANGLE_DEG = 8.0",
        "LIFT_CONFIRM_MS = 100",
        "TURN_REDUCED_SPEED_DPS = 500.0",
        "TURN_MINIMUM_SPEED_DPS = 700.0",
        "TURN_REDUCED_DUTY = 10.0",
        "TURN_MINIMUM_DUTY = 5.0",
        "BALANCE_DUTY_RESERVE = 5.0",
        "REFERENCE_ACCEL_DPS2 = 600.0",
        "REFERENCE_DECEL_MAX_DPS2 = 1800.0",
        "REFERENCE_DECEL_MIN_DPS2 = 200.0",
        "REFERENCE_DECEL_RAMP_SPEED_DPS = 400.0",
        "ANGLE_CORRECTION_TAU_S = 5.0",
        "DEADBAND_COMPENSATION = 8.0",
        "FALL_ANGLE_DEG = 12.0",
    ):
        assert constant in program
    assert "MAX_TURN_DUTY = 20.0" in program
    assert "MAX_DRIVE_SPEED_DPS = 300.0" in program
    assert "HARD_MAX_DRIVE_SPEED_DPS" not in program
    assert "HARD_MAX_TURN_DUTY = 20.0" in program
    assert "DRIVE_NEUTRAL = 0.03" in program
    assert "RUNAWAY_SPEED_DPS = 1200.0" in program
    assert "HIGH_SPEED_THRESHOLD_DPS = 400.0" in program
    assert "HIGH_SPEED_ANGLE_LIMIT_DEG = 10.5" in program
    assert "WATCHDOG_MS = 250" in program
    assert "COMMAND_REPORT_MS = 500" in program
    assert "CALIBRATE_RATE_DPS = 5.0" in program
    assert "CALIBRATE_WHEEL_STEP_DEG = 3" in program
    assert "CALIBRATE_WHEEL_DRIFT_DEG = 12" in program
    assert "CALIBRATE_MAX_ABS_PITCH_DEG = 15.0" in program
    assert "CALIBRATE_UPRIGHT_ROLL_DEG = 90.0" in program
    assert "CALIBRATE_ROLL_TOLERANCE_DEG = 8.0" in program
    assert "abs(pitch) <= CALIBRATE_MAX_ABS_PITCH_DEG" in program
    assert "abs(roll - CALIBRATE_UPRIGHT_ROLL_DEG)" in program
    assert "CALIBRATION_WAIT,pitch_deg=" in program
    assert "RELEASE_READY,gyro_bias_dps=" in program
    assert "hub.speaker.beep(700, 150)" in program
    assert "from pybricks.parameters import Axis, Button, Port, Side" in program
    assert "hub.display.orientation(up=Side.LEFT)" in program
    assert "for number in (3, 2, 1)" not in program
    assert "last_sequence, drive_command, turn_command = command" in program
    assert "requested_speed = drive_command * max_drive_speed_dps" in program
    assert "requested_speed * speed < 0.0" in program
    assert "requested_speed * reference_speed < 0.0" in program
    assert "abs(speed) > REVERSAL_RELEASE_SPEED_DPS" in program
    assert "or abs(reference_speed) > REVERSAL_RELEASE_REFERENCE_DPS" in program
    assert "elif drive_command != 0.0:" in program
    assert "reference_speed = move_toward(" in program
    assert "REFERENCE_ACCEL_DPS2 if building_speed else reference_decel_dps2" in program
    assert "decel_speed_fraction = min(" in program
    assert '"reference_decel_dps2={:.0f},"' in program
    assert "reference_speed * DT_MS / 1000.0" in program
    assert "reference_speed = 0.0" in program
    assert "stopping = drive_command == 0.0 or reversal_blocked" in program
    assert "commanded_position += reference_speed * DT_MS / 1000.0" in program
    assert "ANGLE_GAIN * relative_angle" in program
    assert "+ SPEED_GAIN * speed" in program
    assert "reason=runaway_speed" in program
    assert "reason=high_speed_angle" in program
    assert "drive_command == 0.0" in program
    assert "and reference_speed == 0.0" in program
    assert "lift_detect_ms += DT_MS" in program
    assert "reason=lifted" in program
    assert '"reference_speed_dps={:.1f},reversal_blocked={},"' in program
    assert "if abs(speed) >= TURN_MINIMUM_SPEED_DPS:" in program
    assert "elif abs(speed) >= TURN_REDUCED_SPEED_DPS:" in program
    assert "DUTY_LIMIT - BALANCE_DUTY_RESERVE - abs(duty)" in program
    assert "applied_turn_limit = min(speed_turn_limit, balance_turn_headroom)" in program
    assert "turn_duty = turn_command * applied_turn_limit" in program
    assert "CONTROL_CONFIG,max_drive_speed_dps=" in program
    assert "left.dc(LEFT_SIGN * (duty + turn_duty))" in program
    assert "right.dc(RIGHT_SIGN * (duty - turn_duty))" in program
    assert "commanded_position = 0.0" in program
    assert "WATCHDOG,commands_zeroed" in program
    assert "finally:\n    stop_motors(left, right)" in program


def test_balance_host_waits_until_hub_can_drain_stdin():
    host = (
        Path(__file__).parents[1]
        / "apps" / "ps_controller" / "run_balance_steering.py"
    ).read_text()
    assert 'if line.startswith("BALANCE_ACTIVE")' in host
    assert "balance_active.set()" in host
    assert "while not balance_active.is_set() and not stopped.is_set():" in host
    assert "hub.write_line" in host
    assert 'parser.add_argument("--drive-axis", type=int, default=1)' in host
    assert 'parser.add_argument("--turn-axis", type=int, default=2)' in host
    assert '"--stop-hub-on-exit", action="store_true"' in host
    assert "if args.stop_hub_on_exit:" in host
    assert "Host disconnected; hub balancing remains active." in host
