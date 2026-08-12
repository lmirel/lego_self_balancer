"""Compact host-to-hub game-controller command protocol."""

MAX_SEQUENCE = 0xFFFF
MAX_DRIVE_SPEED_DPS = 300.0
MAX_TURN_DUTY = 20.0


def clamp_command(value: float) -> float:
    return min(1.0, max(-1.0, value))


def format_command(sequence: int, drive: float, turn: float) -> str:
    """Format one command that fits a 20-byte BLE stdin packet."""
    sequence &= MAX_SEQUENCE
    return "C,{:x},{:.2f},{:.2f}".format(
        sequence, clamp_command(drive), clamp_command(turn)
    )


def format_config(max_drive_speed_dps: float, max_turn_duty: float) -> str:
    """Format runtime limits; the hub independently validates hard ceilings."""
    if not 0.0 < max_drive_speed_dps <= MAX_DRIVE_SPEED_DPS:
        raise ValueError("max drive speed must be in (0, 300] degrees/second")
    if not 0.0 <= max_turn_duty <= MAX_TURN_DUTY:
        raise ValueError("max turn duty must be in [0, 20]")
    return "S,{:.1f},{:.1f}".format(max_drive_speed_dps, max_turn_duty)


def is_newer_sequence(sequence: int, previous: int | None) -> bool:
    """Compare wrapping 16-bit sequence numbers."""
    if previous is None:
        return True
    distance = (sequence - previous) & MAX_SEQUENCE
    return 0 < distance < 0x8000
