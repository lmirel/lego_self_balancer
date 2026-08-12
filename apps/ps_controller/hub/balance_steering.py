"""0v3 balancer with watchdog-controlled drive and steering."""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Button, Port
from pybricks.pupdevices import Motor
from pybricks.tools import StopWatch, wait
from uselect import poll
from usys import stdin


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
MAX_TURN_DUTY = 20.0
MAX_DRIVE_SPEED_DPS = 300.0
HARD_MAX_DRIVE_SPEED_DPS = 300.0
HARD_MAX_TURN_DUTY = 20.0
DRIVE_NEUTRAL = 0.03
RUNAWAY_SPEED_DPS = 750.0
WATCHDOG_MS = 250
COMMAND_REPORT_MS = 200
MAX_LINE_LENGTH = 48


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


def is_newer_sequence(sequence, previous):
    if previous is None:
        return True
    distance = (sequence - previous) & 0xFFFF
    return 0 < distance < 0x8000


def parse_command(line, previous_sequence):
    fields = line.split(",")
    if len(fields) != 4 or fields[0] != "C":
        return None
    try:
        sequence = int(fields[1], 16)
        drive = float(fields[2])
        turn = float(fields[3])
    except ValueError:
        return None
    if sequence < 0 or sequence > 0xFFFF:
        return None
    if drive != drive or turn != turn:
        return None
    if drive < -1.0 or drive > 1.0 or turn < -1.0 or turn > 1.0:
        return None
    if not is_newer_sequence(sequence, previous_sequence):
        return None
    return sequence, drive, turn


def parse_config(line):
    fields = line.split(",")
    if len(fields) != 3 or fields[0] != "S":
        return None
    try:
        drive_limit = float(fields[1])
        turn_limit = float(fields[2])
    except ValueError:
        return None
    if drive_limit != drive_limit or turn_limit != turn_limit:
        return None
    if drive_limit <= 0.0 or drive_limit > HARD_MAX_DRIVE_SPEED_DPS:
        return None
    if turn_limit < 0.0 or turn_limit > HARD_MAX_TURN_DUTY:
        return None
    return drive_limit, turn_limit


hub = PrimeHub()
hub.system.set_stop_button(Button.BLUETOOTH)
left = Motor(Port.A)
right = Motor(Port.E)
stop_motors(left, right)

try:
    print("BALANCE_WAITING,max_drive_speed_dps={},max_turn_duty={}".format(
        MAX_DRIVE_SPEED_DPS, MAX_TURN_DUTY
    ))
    gyro_bias, upright_roll = calibrate(hub, left, right)
    left.reset_angle(0)
    right.reset_angle(0)

    position_buffer = [0.0] * WINDOW
    buffer_index = 0
    relative_angle = 0.0
    commanded_position = 0.0
    clock = StopWatch()
    next_loop_ms = 0
    next_report_ms = 0

    input_poll = poll()
    input_poll.register(stdin)
    input_buffer = ""
    last_sequence = None
    last_command_ms = None
    drive_command = 0.0
    turn_command = 0.0
    max_drive_speed_dps = MAX_DRIVE_SPEED_DPS
    max_turn_duty = MAX_TURN_DUTY
    watchdog_active = True
    print("BALANCE_ACTIVE,watchdog_ms={}".format(WATCHDOG_MS))

    while True:
        now_ms = clock.time()
        if Button.CENTER in hub.buttons.pressed():
            print("BALANCE_STOPPED,reason=center_button")
            break

        while input_poll.poll(0):
            character = stdin.read(1)
            if not character:
                break
            if character == "\n":
                line = input_buffer.rstrip("\r")
                input_buffer = ""
                config = parse_config(line)
                if config is not None:
                    max_drive_speed_dps, max_turn_duty = config
                    print(
                        "CONTROL_CONFIG,max_drive_speed_dps={:.1f},"
                        "max_turn_duty={:.1f}".format(
                            max_drive_speed_dps, max_turn_duty
                        )
                    )
                    continue
                command = parse_command(line, last_sequence)
                if command is not None:
                    last_sequence, drive_command, turn_command = command
                    last_command_ms = now_ms
                    if watchdog_active:
                        watchdog_active = False
                        print("CONTROL_ACTIVE,sequence={:x}".format(last_sequence))
            elif len(input_buffer) < MAX_LINE_LENGTH:
                input_buffer += character
            else:
                input_buffer = ""

        if last_command_ms is None or now_ms - last_command_ms > WATCHDOG_MS:
            drive_command = 0.0
            turn_command = 0.0
            if not watchdog_active:
                watchdog_active = True
                print("WATCHDOG,commands_zeroed")

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

        if abs(drive_command) <= DRIVE_NEUTRAL:
            drive_command = 0.0
        else:
            # Positive drive advances the position reference at a bounded rate.
            # Neutral leaves it unchanged so position feedback can brake and
            # hold. Never jump the reference to the measured wheel position.
            commanded_position += (
                drive_command * max_drive_speed_dps * DT_MS / 1000.0
            )

        if abs(relative_angle) >= FALL_ANGLE_DEG:
            print("BALANCE_STOPPED,reason=fallen,angle={:.2f}".format(
                relative_angle
            ))
            break
        if abs(speed) >= RUNAWAY_SPEED_DPS:
            print("BALANCE_STOPPED,reason=runaway_speed,speed={:.1f}".format(
                speed
            ))
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
        turn_duty = turn_command * max_turn_duty

        # Validated positive/right sign: left forward, right backward.
        left.dc(LEFT_SIGN * (duty + turn_duty))
        right.dc(RIGHT_SIGN * (duty - turn_duty))

        if now_ms >= next_report_ms:
            age_ms = -1 if last_command_ms is None else now_ms - last_command_ms
            print(
                "CONTROL_STATUS,drive={:.2f},drive_speed_dps={:.1f},turn={:.2f},"
                "turn_duty={:.2f},position={:.1f},target={:.1f},speed={:.1f},"
                "angle={:.2f},duty={:.1f},age_ms={},"
                "watchdog={}".format(
                    drive_command, drive_command * max_drive_speed_dps,
                    turn_command, turn_duty, position, commanded_position,
                    speed, relative_angle, duty,
                    age_ms, 1 if watchdog_active else 0,
                )
            )
            next_report_ms += COMMAND_REPORT_MS

        next_loop_ms += DT_MS
        remaining = next_loop_ms - clock.time()
        if remaining > 0:
            wait(remaining)
finally:
    stop_motors(left, right)
    hub.display.off()
    print("BALANCE_EXIT")
