"""Phase 2 hub-local P controller for the validated 51515 build.

This controller has no Bluetooth in the feedback path. Integral control is used
only when an evidence-gated PID trial is rendered by the host. Support the
robot before arming it. Press
CENTER once to start and again to abort. The Bluetooth button is an independent
firmware-level emergency stop in case the Python loop stops responding.
"""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Button, Port
from pybricks.pupdevices import Motor
from pybricks.tools import StopWatch, wait


# Mirrored from config.toml after Phase 1 hardware validation. Pybricks programs
# cannot directly load the Mac-side TOML file. Keep these values synchronized.
LEFT_MOTOR_PORT = Port.A
RIGHT_MOTOR_PORT = Port.E
LEFT_MOTOR_SIGN = -1
RIGHT_MOTOR_SIGN = 1
TARGET_ANGLE_DEG = 87.14

# P, gyro-based PD, and anti-windup PID are explicit modes. These values mirror
# config.toml; the staged host search keeps KI at zero until its final stage.
CONTROLLER_MODE = "PD"
KP = 120.0
KI = 0.0
KD = 0.0
KW = 0.0
KX = 0.0
REFERENCE_POWER_SCALE = 2.5
REFERENCE_INTEGRAL_STEP = 0.25
GYRO_FILTER_ALPHA = 0.386
AUTO_ARM = False
LOOP_PERIOD_MS = 10
TELEMETRY_PERIOD_MS = 40
OUTPUT_LIMIT = 1000.0
INTEGRAL_LIMIT = 20.0
FALL_ANGLE_DEG = 12.0
START_ANGLE_TOLERANCE_DEG = 2.5
UPRIGHT_GYRO_TOLERANCE_DPS = 5.0
UPRIGHT_STABLE_MS = 500
COUNTDOWN_ANGLE_LIMIT_DEG = 5.0
COUNTDOWN_GYRO_LIMIT_DPS = 30.0
COUNTDOWN_SECONDS = 3
PREDICTION_HORIZON_S = 0.15
PREDICTED_FALL_ANGLE_DEG = 10.0
PREDICTED_FALL_MIN_ERROR_DEG = 3.0
SATURATION_ABORT_MS = 250
TRIAL_DURATION_MS = 5000


def clamp(value, limit):
    return max(-limit, min(limit, value))


def stop_motors(left_motor, right_motor):
    # Coast is used for this first test to avoid drivetrain shock on a fall.
    left_motor.stop()
    right_motor.stop()


def pose_is_stable_upright(hub):
    angle_error = hub.imu.tilt()[1] - TARGET_ANGLE_DEG
    gyro_rate = hub.imu.angular_velocity(Axis.X)
    return (
        abs(angle_error) <= START_ANGLE_TOLERANCE_DEG
        and abs(gyro_rate) <= UPRIGHT_GYRO_TOLERANCE_DPS
    )


def pose_is_countdown_safe(hub):
    angle_error = hub.imu.tilt()[1] - TARGET_ANGLE_DEG
    gyro_rate = hub.imu.angular_velocity(Axis.X)
    return (
        abs(angle_error) <= COUNTDOWN_ANGLE_LIMIT_DEG
        and abs(gyro_rate) <= COUNTDOWN_GYRO_LIMIT_DPS
    )


def automatic_countdown(hub):
    """Wait for a stable upright pose, then count down without motor drive."""
    print("READY_AUTO,target_deg={:.2f},tolerance_deg={:.2f},stable_ms={}".format(
        TARGET_ANGLE_DEG, START_ANGLE_TOLERANCE_DEG, UPRIGHT_STABLE_MS
    ))
    stable_ms = 0
    report_ms = 0
    while True:
        angle = hub.imu.tilt()[1]
        gyro = hub.imu.angular_velocity(Axis.X)
        if report_ms <= 0:
            print("WAITING_AUTO,angle_deg={:.3f},error_deg={:.3f},gyro_dps={:.2f}".format(
                angle, angle - TARGET_ANGLE_DEG, gyro
            ))
            report_ms = 500
        if pose_is_stable_upright(hub):
            stable_ms += 20
            if stable_ms >= UPRIGHT_STABLE_MS:
                print("UPRIGHT_STABLE")
                for number in range(COUNTDOWN_SECONDS, 0, -1):
                    hub.display.number(number)
                    hub.speaker.beep(500, 100)
                    print("COUNTDOWN,{}".format(number))
                    for _ in range(45):
                        wait(20)
                        if not pose_is_countdown_safe(hub):
                            hub.display.off()
                            print("COUNTDOWN_CANCELLED,pose_moved")
                            stable_ms = 0
                            break
                    else:
                        continue
                    break
                else:
                    hub.display.off()
                    start_angle = hub.imu.tilt()[1]
                    print("ARMED_AUTO,angle_deg={:.3f},error_deg={:.3f}".format(
                        start_angle, start_angle - TARGET_ANGLE_DEG
                    ))
                    return
        else:
            stable_ms = 0
        wait(20)
        report_ms -= 20


def wait_until_armed(hub):
    if AUTO_ARM:
        automatic_countdown(hub)
        return

    while True:
        print("READY,hold robot near {:.2f} deg and press CENTER".format(
            TARGET_ANGLE_DEG
        ))
        while Button.CENTER not in hub.buttons.pressed():
            wait(20)

        # Do not begin driving until CENTER has been released. Its next press
        # aborts, and releasing first avoids interpreting the start as an abort.
        while Button.CENTER in hub.buttons.pressed():
            wait(20)

        start_angle_deg = hub.imu.tilt()[1]
        start_error_deg = start_angle_deg - TARGET_ANGLE_DEG
        if abs(start_error_deg) <= START_ANGLE_TOLERANCE_DEG:
            print("ARMED,angle_deg={:.3f},error_deg={:.3f}".format(
                start_angle_deg, start_error_deg
            ))
            return

        print("START_REJECTED,angle_deg={:.3f},error_deg={:.3f},limit_deg={:.3f}".format(
            start_angle_deg, start_error_deg, START_ANGLE_TOLERANCE_DEG
        ))


hub = PrimeHub()
# CENTER is handled by this program so it can start and stop a trial. Keep the
# Bluetooth button as a firmware-level stop that does not depend on this loop.
hub.system.set_stop_button(Button.BLUETOOTH)
left_motor = Motor(LEFT_MOTOR_PORT)
right_motor = Motor(RIGHT_MOTOR_PORT)
stop_motors(left_motor, right_motor)

state = "WAITING"
try:
    wait_until_armed(hub)
    state = "RUNNING"
    if CONTROLLER_MODE not in ("P", "PD", "PID", "REFERENCE"):
        raise ValueError("CONTROLLER_MODE must be P, PD, PID, or REFERENCE")

    print("TRIAL_STARTED,mode={},kp={},ki={},kd={},kw={},kx={},target_deg={},duration_ms={}".format(
        CONTROLLER_MODE, KP, KI, KD, KW, KX, TARGET_ANGLE_DEG, TRIAL_DURATION_MS
    ))
    print("timestamp_ms,angle_deg,error_deg,gyro_x_dps,filtered_gyro_x_dps,wheel_speed_dps,wheel_position_deg,p_term,i_term,d_term,speed_term,position_term,output,left_angle_deg,right_angle_deg,loop_dt_ms,state")

    clock = StopWatch()
    previous_loop_ms = 0
    next_loop_ms = 0
    next_telemetry_ms = 0
    timing_count = 0
    timing_sum_ms = 0
    timing_min_ms = 1000000
    timing_max_ms = 0
    late_count = 0
    saturation_started_ms = None
    integral = 0.0
    filtered_gyro_x_dps = None
    initial_left_angle_deg = left_motor.angle()
    initial_right_angle_deg = right_motor.angle()
    reference_integral = 0.0
    reference_last_error = 0.0

    while True:
        now_ms = clock.time()
        loop_dt_ms = now_ms - previous_loop_ms
        previous_loop_ms = now_ms

        if timing_count > 0:
            timing_sum_ms += loop_dt_ms
            timing_min_ms = min(timing_min_ms, loop_dt_ms)
            timing_max_ms = max(timing_max_ms, loop_dt_ms)
        timing_count += 1

        angle_deg = hub.imu.tilt()[1]
        gyro_x_dps = hub.imu.angular_velocity(Axis.X)
        if filtered_gyro_x_dps is None:
            filtered_gyro_x_dps = gyro_x_dps
        else:
            filtered_gyro_x_dps += GYRO_FILTER_ALPHA * (
                gyro_x_dps - filtered_gyro_x_dps
            )

        if Button.CENTER in hub.buttons.pressed():
            state = "ABORTED"
            stop_motors(left_motor, right_motor)
            print("ABORTED,{},center_button".format(now_ms))
            break

        # Validated convention: positive roll is a forward lean and positive
        # unified motor output drives both wheels forward. Therefore positive
        # forward angle error must produce positive output.
        error_deg = angle_deg - TARGET_ANGLE_DEG
        projected_error_deg = error_deg + gyro_x_dps * PREDICTION_HORIZON_S

        if abs(error_deg) > FALL_ANGLE_DEG:
            state = "FALLEN"
            stop_motors(left_motor, right_motor)
            print("FALLEN,{},{:.3f}".format(now_ms, error_deg))
            break

        moving_away_from_upright = error_deg * gyro_x_dps > 0
        if (
            abs(error_deg) >= PREDICTED_FALL_MIN_ERROR_DEG
            and moving_away_from_upright
            and abs(projected_error_deg) > PREDICTED_FALL_ANGLE_DEG
        ):
            state = "FALLEN"
            stop_motors(left_motor, right_motor)
            print("FALLEN,{},{:.3f},projected_error={:.3f}".format(
                now_ms, error_deg, projected_error_deg
            ))
            break

        if now_ms >= TRIAL_DURATION_MS:
            state = "COMPLETE"
            stop_motors(left_motor, right_motor)
            print("TRIAL_COMPLETE,{}".format(now_ms))
            break

        p_term = KP * error_deg
        # Positive gyro X means rotating forward. With the validated motor
        # convention, positive output moves the wheels forward to catch that
        # motion, so the stabilizing rate term has a positive sign here.
        d_term = KD * filtered_gyro_x_dps if CONTROLLER_MODE == "PD" else 0.0
        if CONTROLLER_MODE == "PID":
            d_term = KD * filtered_gyro_x_dps
            dt_s = (loop_dt_ms if loop_dt_ms > 0 else LOOP_PERIOD_MS) / 1000.0
            candidate_integral = clamp(
                integral + error_deg * dt_s, INTEGRAL_LIMIT
            )
            candidate_output = p_term + KI * candidate_integral + d_term
            # Conditional integration: accept while unsaturated, or when the
            # current error would pull an already saturated output back inward.
            if (
                abs(candidate_output) <= OUTPUT_LIMIT
                or candidate_output * error_deg < 0
            ):
                integral = candidate_integral
        i_term = KI * integral if CONTROLLER_MODE == "PID" else 0.0
        if CONTROLLER_MODE == "REFERENCE":
            # Faithful translation of the visual program. Its target-error and
            # movement signs are converted to this robot's validated convention,
            # so positive physical error still requests forward wheel motion.
            reference_integral += error_deg * REFERENCE_INTEGRAL_STEP
            reference_derivative = error_deg - reference_last_error
            reference_last_error = error_deg
            percent_to_dps = OUTPUT_LIMIT / 100.0
            reference_scale = REFERENCE_POWER_SCALE * percent_to_dps
            p_term = KP * error_deg * reference_scale
            i_term = KI * reference_integral * reference_scale
            d_term = KD * reference_derivative * reference_scale
        wheel_speed_dps = (
            LEFT_MOTOR_SIGN * left_motor.speed()
            + RIGHT_MOTOR_SIGN * right_motor.speed()
        ) / 2.0
        speed_term = -KW * wheel_speed_dps
        left_angle_deg = left_motor.angle()
        right_angle_deg = right_motor.angle()
        wheel_position_deg = (
            LEFT_MOTOR_SIGN * (left_angle_deg - initial_left_angle_deg)
            + RIGHT_MOTOR_SIGN * (right_angle_deg - initial_right_angle_deg)
        ) / 2.0
        position_term = -KX * wheel_position_deg
        # Output is requested wheel speed in degrees/second. Pybricks closes the
        # motor-speed loop and supplies the duty needed to overcome drivetrain
        # friction, so no application-level deadband kick is required.
        output = clamp(p_term + i_term + d_term + speed_term + position_term, OUTPUT_LIMIT)

        if abs(output) >= OUTPUT_LIMIT:
            if saturation_started_ms is None:
                saturation_started_ms = now_ms
            elif now_ms - saturation_started_ms >= SATURATION_ABORT_MS:
                state = "FALLEN"
                stop_motors(left_motor, right_motor)
                print("FALLEN,{},{:.3f},saturated_ms={}".format(
                    now_ms, error_deg, now_ms - saturation_started_ms
                ))
                break
        else:
            saturation_started_ms = None

        left_motor.run(LEFT_MOTOR_SIGN * output)
        right_motor.run(RIGHT_MOTOR_SIGN * output)

        if now_ms >= next_telemetry_ms:
            print("{},{:.3f},{:.3f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{},{},{},{}".format(
                now_ms,
                angle_deg,
                error_deg,
                gyro_x_dps,
                filtered_gyro_x_dps,
                wheel_speed_dps,
                wheel_position_deg,
                p_term,
                i_term,
                d_term,
                speed_term,
                position_term,
                output,
                left_angle_deg,
                right_angle_deg,
                loop_dt_ms,
                state,
            ))
            next_telemetry_ms += TELEMETRY_PERIOD_MS

        next_loop_ms += LOOP_PERIOD_MS
        remaining_ms = next_loop_ms - clock.time()
        if remaining_ms > 0:
            wait(remaining_ms)
        else:
            late_count += 1

    measured_intervals = max(1, timing_count - 1)
    print("TIMING,avg_ms={:.3f},min_ms={},max_ms={},late={}".format(
        timing_sum_ms / measured_intervals,
        timing_min_ms,
        timing_max_ms,
        late_count,
    ))
except Exception as error:
    state = "ABORTED"
    print("ERROR,{}".format(error))
    raise
finally:
    stop_motors(left_motor, right_motor)
    print("STOPPED,state={}".format(state))
