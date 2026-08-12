"""Standalone 0v2.2 stationary balancer for LEGO Robot Inventor 51515."""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Button, Port
from pybricks.pupdevices import Motor
from pybricks.tools import StopWatch, wait


DT_MS = 5
SPEED_WINDOW_MS = 200
WINDOW = SPEED_WINDOW_MS // DT_MS
RATE_GAIN = 0.018
ANGLE_GAIN = 19.0
POSITION_GAIN = 0.45
SPEED_GAIN = 0.20
ANGLE_CORRECTION_TAU_S = 5.0
NOMINAL_VOLTAGE_MV = 7200
DEADBAND_COMPENSATION = 8.0
DEADBAND_COMMAND_MIN = 3.0
DEADBAND_FADE_SPEED_DPS = 120.0
DUTY_LIMIT = 100.0
FALL_ANGLE_DEG = 12.0

LEFT_SIGN = -1
RIGHT_SIGN = 1
COMMAND_SPEED_DPS = 0.0
COMMAND_TURN_DUTY = 0.0


def stop_motors(left, right):
    left.stop()
    right.stop()


def wheel_step(left, right, previous_left, previous_right):
    left_angle = left.angle()
    right_angle = right.angle()
    return (
        max(abs(left_angle - previous_left), abs(right_angle - previous_right)),
        left_angle,
        right_angle,
    )


def calibrate(hub, left, right):
    """Wait for a still pose, count down, and return gyro/roll references."""
    stable_ms = 0
    rate_sum = 0.0
    roll_sum = 0.0
    samples = 0
    previous_left = left.angle()
    previous_right = right.angle()
    hub.display.char("H")

    while True:
        rate = hub.imu.angular_velocity(Axis.X)
        step, previous_left, previous_right = wheel_step(
            left, right, previous_left, previous_right
        )
        if abs(rate) <= 3.0 and step <= 1:
            stable_ms += 20
            rate_sum += rate
            roll_sum += hub.imu.tilt()[1]
            samples += 1
            if stable_ms >= 500:
                for number in (3, 2, 1):
                    hub.display.number(number)
                    hub.speaker.beep(500, 100)
                    for _ in range(45):
                        wait(20)
                        rate = hub.imu.angular_velocity(Axis.X)
                        step, previous_left, previous_right = wheel_step(
                            left, right, previous_left, previous_right
                        )
                        if abs(rate) <= 3.0 and step <= 1:
                            rate_sum += rate
                            roll_sum += hub.imu.tilt()[1]
                            samples += 1
                        if abs(rate) > 15.0 or step > 4:
                            hub.display.char("H")
                            stable_ms = 0
                            rate_sum = 0.0
                            roll_sum = 0.0
                            samples = 0
                            break
                    else:
                        continue
                    break
                else:
                    hub.display.off()
                    return rate_sum / samples, roll_sum / samples
        else:
            stable_ms = 0
            rate_sum = 0.0
            roll_sum = 0.0
            samples = 0
        wait(20)


hub = PrimeHub()
hub.system.set_stop_button(Button.BLUETOOTH)
left = Motor(Port.A)
right = Motor(Port.E)
stop_motors(left, right)

try:
    gyro_bias, upright_roll = calibrate(hub, left, right)
    left.reset_angle(0)
    right.reset_angle(0)

    position_buffer = [0.0] * WINDOW
    buffer_index = 0
    relative_angle = 0.0
    commanded_position = 0.0
    clock = StopWatch()
    next_loop_ms = 0

    while True:
        if Button.CENTER in hub.buttons.pressed():
            break

        rate = hub.imu.angular_velocity(Axis.X) - gyro_bias
        relative_angle += rate * DT_MS / 1000.0
        absolute_angle = hub.imu.tilt()[1] - upright_roll
        relative_angle += (absolute_angle - relative_angle) * (
            DT_MS / 1000.0 / ANGLE_CORRECTION_TAU_S
        )

        left_angle = left.angle()
        right_angle = right.angle()
        position = (LEFT_SIGN * left_angle + RIGHT_SIGN * right_angle) / 2.0
        speed = (position - position_buffer[buffer_index]) / SPEED_WINDOW_MS * 1000.0
        position_buffer[buffer_index] = position
        buffer_index = (buffer_index + 1) % WINDOW
        commanded_position += COMMAND_SPEED_DPS * DT_MS / 1000.0

        if abs(relative_angle) >= FALL_ANGLE_DEG:
            break

        raw_duty = (
            RATE_GAIN * rate
            + ANGLE_GAIN * relative_angle
            + POSITION_GAIN * (position - commanded_position)
            + SPEED_GAIN * speed
        ) * NOMINAL_VOLTAGE_MV / hub.battery.voltage()

        compensation = 0.0
        if abs(raw_duty) >= DEADBAND_COMMAND_MIN:
            fade = max(0.0, 1.0 - abs(speed) / DEADBAND_FADE_SPEED_DPS)
            compensation = DEADBAND_COMPENSATION * fade
            if raw_duty < 0:
                compensation = -compensation
        duty = max(-DUTY_LIMIT, min(DUTY_LIMIT, raw_duty + compensation))

        left.dc(LEFT_SIGN * (duty - COMMAND_TURN_DUTY))
        right.dc(RIGHT_SIGN * (duty + COMMAND_TURN_DUTY))

        next_loop_ms += DT_MS
        remaining = next_loop_ms - clock.time()
        if remaining > 0:
            wait(remaining)
finally:
    stop_motors(left, right)
    hub.display.off()
