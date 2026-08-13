from pathlib import Path


ROOT = Path(__file__).parents[1]
APP = ROOT / "apps" / "0v3p3"


def test_0v3p3_package_contains_every_runtime_and_architecture_file():
    expected = {
        "README.md",
        "VERSION",
        "requirements.txt",
        "controller.py",
        "control.py",
        "protocol.py",
        "hub/main.py",
        "ARCHITECTURE.md",
        "HOST_ARCHITECTURE.md",
        "HUB_ARCHITECTURE.md",
    }
    assert {
        str(path.relative_to(APP))
        for path in APP.rglob("*")
        if path.is_file() and "__pycache__" not in path.parts
    } == expected
    assert (APP / "VERSION").read_text().strip() == "0v3.3"


def test_0v3p3_host_is_self_contained_and_independent_on_exit():
    host = (APP / "controller.py").read_text()
    assert "apps.ps_controller" not in host
    assert "from control import AxisProcessor" in host
    assert "from protocol import format_command, format_config" in host
    assert 'parent / "hub" / "main.py"' in host
    assert 'parser.add_argument("--drive-axis", type=int, default=1)' in host
    assert 'parser.add_argument("--turn-axis", type=int, default=2)' in host
    assert '"--stop-hub-on-exit", action="store_true"' in host
    assert "if args.stop_hub_on_exit:" in host
    assert "Host disconnected; hub balancing remains active." in host


def test_0v3p3_hub_contains_current_control_and_safety_architecture():
    hub = (APP / "hub" / "main.py").read_text()
    for constant in (
        "DT_MS = 5",
        "RATE_GAIN = 0.018",
        "ANGLE_GAIN = 19.0",
        "POSITION_GAIN = 0.45",
        "SPEED_GAIN = 0.20",
        "REFERENCE_ACCEL_DPS2 = 600.0",
        "REFERENCE_DECEL_MAX_DPS2 = 1800.0",
        "REFERENCE_DECEL_MIN_DPS2 = 200.0",
        "REFERENCE_DECEL_RAMP_SPEED_DPS = 400.0",
        "TURN_REDUCED_SPEED_DPS = 500.0",
        "TURN_MINIMUM_SPEED_DPS = 700.0",
        "TURN_REDUCED_DUTY = 10.0",
        "TURN_MINIMUM_DUTY = 5.0",
        "BALANCE_DUTY_RESERVE = 5.0",
        "RUNAWAY_SPEED_DPS = 1200.0",
        "HIGH_SPEED_ANGLE_LIMIT_DEG = 10.5",
        "WATCHDOG_MS = 250",
        "COMMAND_REPORT_MS = 500",
        "LIFT_SPEED_DPS = 400.0",
        "LIFT_CONFIRM_MS = 100",
    ):
        assert constant in hub
    assert "hub.display.orientation(up=Side.LEFT)" in hub
    assert "reason=lifted" in hub
    assert "WATCHDOG,commands_zeroed" in hub
    assert "turn_duty = turn_command * applied_turn_limit" in hub
    assert "finally:\n    stop_motors(left, right)" in hub


def test_0v3p3_architecture_documents_describe_both_deployments():
    system = (APP / "ARCHITECTURE.md").read_text()
    host = (APP / "HOST_ARCHITECTURE.md").read_text()
    hub = (APP / "HUB_ARCHITECTURE.md").read_text()
    assert "Ownership boundaries" in system
    assert "Runtime command path" in system
    assert "Controller input processing" in host
    assert "Lifecycle behavior" in host
    assert "Drive controller" in hub
    assert "Balance controller" in hub
    assert "Turn controller and motor mixer" in hub
    assert "Safety supervisor" in hub


def test_0v3p3_requirements_are_pinned():
    assert (APP / "requirements.txt").read_text().splitlines() == [
        "pygame-ce==2.5.7",
        "pybricksdev==2.3.2",
    ]
