"""Safety-wrapped adaptation of the official Pybricks Inventor balancer."""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Button, Port
from pybricks.pupdevices import Motor
from pybricks.tools import StopWatch, wait


LEFT_PORT = Port.A
RIGHT_PORT = Port.E
LEFT_SIGN = -1
RIGHT_SIGN = 1
BALANCE_AXIS = Axis.X

DT_MS = 5
SPEED_WINDOW_MS = 200
WINDOW = SPEED_WINDOW_MS // DT_MS
RATE_GAIN = 0.018
ANGLE_GAIN = 19.0
# Stationary full-state experiment. These two gains act on fixed encoder-zero
# position and wheel velocity and must be tuned together for station keeping.
POSITION_GAIN = 0.45
SPEED_GAIN = 0.20
NOMINAL_VOLTAGE_MV = 7200
DUTY_LIMIT = 100.0
ABSOLUTE_ANGLE_CORRECTION_TAU_S = 5.0
DEADBAND_COMPENSATION = 8.0
DEADBAND_COMMAND_MIN = 3.0
DEADBAND_FADE_SPEED_DPS = 120.0

STABLE_RATE_DPS = 3.0
STABLE_WHEEL_STEP_DEG = 1
STABLE_MS = 500
COUNTDOWN_SECONDS = 3
COUNTDOWN_RATE_LIMIT_DPS = 15.0
COUNTDOWN_WHEEL_STEP_DEG = 4
FALL_RELATIVE_ANGLE_DEG = 12.0
TRIAL_DURATION_MS = 0
TELEMETRY_PERIOD_MS = 40

# Controller-input boundary. Zero commands preserve stationary holding.
COMMAND_SPEED_DPS = 0.0
COMMAND_TURN_DUTY = 0.0


def stop_motors(left, right):
    left.stop()
    right.stop()


def wheel_step(left, right, previous_left, previous_right):
    left_angle = left.angle()
    right_angle = right.angle()
    step = max(abs(left_angle - previous_left), abs(right_angle - previous_right))
    return step, left_angle, right_angle


def wait_for_stable_countdown(hub, left, right):
    print("READY_OFFICIAL,hold robot at desired upright pose")
    stable_ms = 0
    bias_sum = 0.0
    bias_count = 0
    roll_sum = 0.0
    report_ms = 0
    previous_left = left.angle()
    previous_right = right.angle()
    while True:
        rate = hub.imu.angular_velocity(BALANCE_AXIS)
        step, previous_left, previous_right = wheel_step(
            left, right, previous_left, previous_right
        )
        if report_ms <= 0:
            print("WAITING_OFFICIAL,gyro_dps={:.2f},wheel_step_deg={}".format(
                rate, step
            ))
            report_ms = 500
        if abs(rate) <= STABLE_RATE_DPS and step <= STABLE_WHEEL_STEP_DEG:
            stable_ms += 20
            bias_sum += rate
            bias_count += 1
            roll_sum += hub.imu.tilt()[1]
            if stable_ms >= STABLE_MS:
                print("UPRIGHT_STABLE")
                for number in range(COUNTDOWN_SECONDS, 0, -1):
                    hub.display.number(number)
                    hub.speaker.beep(500, 100)
                    print("COUNTDOWN,{}".format(number))
                    for _ in range(45):
                        wait(20)
                        rate = hub.imu.angular_velocity(BALANCE_AXIS)
                        step, previous_left, previous_right = wheel_step(
                            left, right, previous_left, previous_right
                        )
                        if (
                            abs(rate) <= STABLE_RATE_DPS
                            and step <= STABLE_WHEEL_STEP_DEG
                        ):
                            bias_sum += rate
                            bias_count += 1
                            roll_sum += hub.imu.tilt()[1]
                        if (
                            abs(rate) > COUNTDOWN_RATE_LIMIT_DPS
                            or step > COUNTDOWN_WHEEL_STEP_DEG
                        ):
                            print("COUNTDOWN_CANCELLED,pose_moved")
                            hub.display.off()
                            stable_ms = 0
                            bias_sum = 0.0
                            bias_count = 0
                            roll_sum = 0.0
                            break
                    else:
                        continue
                    break
                else:
                    hub.display.off()
                    gyro_bias = bias_sum / max(1, bias_count)
                    upright_roll = roll_sum / max(1, bias_count)
                    print("GYRO_BIAS,dps={:.4f},samples={}".format(
                        gyro_bias, bias_count
                    ))
                    print("UPRIGHT_ROLL,deg={:.4f}".format(upright_roll))
                    return gyro_bias, upright_roll
        else:
            stable_ms = 0
            bias_sum = 0.0
            bias_count = 0
            roll_sum = 0.0
        wait(20)
        report_ms -= 20


hub = PrimeHub()
hub.system.set_stop_button(Button.BLUETOOTH)
left = Motor(LEFT_PORT)
right = Motor(RIGHT_PORT)
stop_motors(left, right)

state = "WAITING"
try:
    gyro_bias, upright_roll = wait_for_stable_countdown(hub, left, right)
    state = "RUNNING"

    left.reset_angle(0)
    right.reset_angle(0)
    position_buffer = [0.0] * WINDOW
    buffer_index = 0
    relative_angle = 0.0
    commanded_position = 0.0

    print(
        "TRIAL_STARTED,mode=OFFICIAL_REFERENCE,dt_ms={},speed_window_ms={},"
        "duration_ms={},gyro_bias_dps={:.4f}".format(
            DT_MS, SPEED_WINDOW_MS, TRIAL_DURATION_MS, gyro_bias
        )
    )
    print(
        "timestamp_ms,relative_angle_deg,absolute_angle_deg,gyro_x_dps,wheel_position_deg,"
        "commanded_position_deg,wheel_speed_dps,rate_term,angle_term,position_term,speed_term,"
        "battery_mv,duty,left_angle_deg,right_angle_deg,loop_dt_ms,state"
    )

    clock = StopWatch()
    previous_ms = 0
    next_loop_ms = 0
    next_telemetry_ms = 0
    timing_count = 0
    timing_sum = 0
    timing_min = 1000000
    timing_max = 0
    late_count = 0

    while True:
        now_ms = clock.time()
        loop_dt_ms = now_ms - previous_ms
        previous_ms = now_ms
        if timing_count:
            timing_sum += loop_dt_ms
            timing_min = min(timing_min, loop_dt_ms)
            timing_max = max(timing_max, loop_dt_ms)
        timing_count += 1

        if Button.CENTER in hub.buttons.pressed():
            state = "ABORTED"
            print("ABORTED,{},center_button".format(now_ms))
            break

        raw_rate = hub.imu.angular_velocity(BALANCE_AXIS)
        rate = raw_rate - gyro_bias
        relative_angle += rate * DT_MS / 1000.0
        absolute_angle = hub.imu.tilt()[1] - upright_roll
        relative_angle += (absolute_angle - relative_angle) * (
            DT_MS / 1000.0 / ABSOLUTE_ANGLE_CORRECTION_TAU_S
        )

        left_angle = left.angle()
        right_angle = right.angle()
        position = (LEFT_SIGN * left_angle + RIGHT_SIGN * right_angle) / 2.0
        speed = (position - position_buffer[buffer_index]) / SPEED_WINDOW_MS * 1000.0
        position_buffer[buffer_index] = position
        buffer_index = (buffer_index + 1) % WINDOW
        commanded_position += COMMAND_SPEED_DPS * DT_MS / 1000.0

        if abs(relative_angle) >= FALL_RELATIVE_ANGLE_DEG:
            state = "FALLEN"
            print("FALLEN,{},{:.3f}".format(now_ms, relative_angle))
            break
        if TRIAL_DURATION_MS > 0 and now_ms >= TRIAL_DURATION_MS:
            state = "COMPLETE"
            print("TRIAL_COMPLETE,{}".format(now_ms))
            break
        rate_term = RATE_GAIN * rate
        angle_term = ANGLE_GAIN * relative_angle
        position_term = POSITION_GAIN * (position - commanded_position)
        speed_term = SPEED_GAIN * speed
        battery_mv = hub.battery.voltage()
        raw_duty = (rate_term + angle_term + position_term + speed_term) * (
            NOMINAL_VOLTAGE_MV / battery_mv
        )
        compensation = 0.0
        if abs(raw_duty) >= DEADBAND_COMMAND_MIN:
            fade = max(0.0, 1.0 - abs(speed) / DEADBAND_FADE_SPEED_DPS)
            compensation = (
                DEADBAND_COMPENSATION * fade
                if raw_duty > 0 else -DEADBAND_COMPENSATION * fade
            )
        duty = raw_duty + compensation
        duty = max(-DUTY_LIMIT, min(DUTY_LIMIT, duty))

        left.dc(LEFT_SIGN * (duty - COMMAND_TURN_DUTY))
        right.dc(RIGHT_SIGN * (duty + COMMAND_TURN_DUTY))

        if now_ms >= next_telemetry_ms:
            print(
                "{},{:.3f},{:.3f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},"
                "{:.2f},{:.2f},{:.2f},{},{:.2f},{},{},{},{}".format(
                    now_ms, relative_angle, absolute_angle, rate, position,
                    commanded_position, speed,
                    rate_term, angle_term, position_term, speed_term,
                    battery_mv, duty, left_angle, right_angle, loop_dt_ms, state,
                )
            )
            next_telemetry_ms += TELEMETRY_PERIOD_MS

        next_loop_ms += DT_MS
        remaining = next_loop_ms - clock.time()
        if remaining > 0:
            wait(remaining)
        else:
            late_count += 1

    intervals = max(1, timing_count - 1)
    print("TIMING,avg_ms={:.3f},min_ms={},max_ms={},late={}".format(
        timing_sum / intervals, timing_min, timing_max, late_count
    ))
finally:
    stop_motors(left, right)
    print("STOPPED,state={}".format(state))
