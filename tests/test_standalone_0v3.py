from pathlib import Path


ROOT = Path(__file__).parents[1]
APP = ROOT / "apps" / "0v3"


def test_0v3_package_contains_every_runtime_file():
    expected = {
        "README.md",
        "VERSION",
        "requirements.txt",
        "controller.py",
        "control.py",
        "protocol.py",
        "hub/main.py",
    }
    assert {
        str(path.relative_to(APP))
        for path in APP.rglob("*")
        if path.is_file() and "__pycache__" not in path.parts
    } == expected
    assert (APP / "VERSION").read_text().strip() == "0v3"


def test_0v3_host_is_self_contained_and_uses_locked_defaults():
    host = (APP / "controller.py").read_text()
    assert 'from control import AxisProcessor' in host
    assert 'from protocol import format_command, format_config' in host
    assert 'parent / "hub" / "main.py"' in host
    assert "apps.ps_controller" not in host
    assert 'parser.add_argument("--drive-axis", type=int, default=1)' in host
    assert 'parser.add_argument("--turn-axis", type=int, default=2)' in host
    assert 'parser.add_argument("--max-drive-speed-dps", type=float, default=300.0)' in host
    assert 'parser.add_argument("--max-turn-duty", type=float, default=20.0)' in host
    assert 'if line.startswith("BALANCE_ACTIVE")' in host


def test_0v3_hub_preserves_controller_and_safety_limits():
    hub = (APP / "hub" / "main.py").read_text()
    for constant in (
        "DT_MS = 5",
        "RATE_GAIN = 0.018",
        "ANGLE_GAIN = 19.0",
        "POSITION_GAIN = 0.45",
        "SPEED_GAIN = 0.20",
        "MAX_DRIVE_SPEED_DPS = 300.0",
        "MAX_TURN_DUTY = 20.0",
        "WATCHDOG_MS = 250",
        "FALL_ANGLE_DEG = 12.0",
        "RUNAWAY_SPEED_DPS = 750.0",
        "CALIBRATE_RATE_DPS = 5.0",
        "CALIBRATE_WHEEL_STEP_DEG = 3",
        "CALIBRATE_WHEEL_DRIFT_DEG = 12",
        "COUNTDOWN_RATE_DPS = 20.0",
        "COUNTDOWN_WHEEL_STEP_DEG = 6",
        "COUNTDOWN_WHEEL_DRIFT_DEG = 24",
    ):
        assert constant in hub
    assert "left.dc(LEFT_SIGN * (duty + turn_duty))" in hub
    assert "right.dc(RIGHT_SIGN * (duty - turn_duty))" in hub
    assert "finally:\n    stop_motors(left, right)" in hub
    assert "CALIBRATION_WAIT,rate_dps=" in hub
    assert "COUNTDOWN_CANCELLED,rate_dps=" in hub


def test_0v3_minimal_requirements_are_pinned():
    requirements = (APP / "requirements.txt").read_text().splitlines()
    assert requirements == ["pygame-ce==2.5.7", "pybricksdev==2.3.2"]
