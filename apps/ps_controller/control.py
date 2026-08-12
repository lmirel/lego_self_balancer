"""Pure input shaping used by the PS controller host application."""

from dataclasses import dataclass


def clamp(value: float, low: float = -1.0, high: float = 1.0) -> float:
    return min(high, max(low, value))


def apply_deadzone(value: float, deadzone: float) -> float:
    """Remove and rescale a symmetric joystick dead zone."""
    value = clamp(value)
    deadzone = clamp(deadzone, 0.0, 0.95)
    magnitude = abs(value)
    if magnitude <= deadzone:
        return 0.0
    scaled = (magnitude - deadzone) / (1.0 - deadzone)
    return scaled if value > 0 else -scaled


def apply_expo(value: float, amount: float) -> float:
    """Blend linear and cubic response; positive values soften the centre."""
    amount = clamp(amount, 0.0, 1.0)
    return (1.0 - amount) * value + amount * value * value * value


@dataclass
class LowPass:
    alpha: float
    value: float = 0.0

    def update(self, sample: float) -> float:
        alpha = clamp(self.alpha, 0.0, 1.0)
        self.value += alpha * (sample - self.value)
        return self.value


@dataclass
class SlewLimiter:
    units_per_second: float
    value: float = 0.0

    def update(self, target: float, dt: float) -> float:
        maximum_change = max(0.0, self.units_per_second * dt)
        change = clamp(target - self.value, -maximum_change, maximum_change)
        self.value = clamp(self.value + change)
        return self.value


class AxisProcessor:
    def __init__(
        self,
        deadzone: float = 0.08,
        expo: float = 0.35,
        filter_alpha: float = 0.25,
        slew_rate: float = 3.0,
    ) -> None:
        self.deadzone = deadzone
        self.expo = expo
        self.low_pass = LowPass(filter_alpha)
        self.slew = SlewLimiter(slew_rate)

    def update(self, raw: float, dt: float) -> float:
        shaped = apply_expo(apply_deadzone(raw, self.deadzone), self.expo)
        return self.slew.update(self.low_pass.update(shaped), dt)
