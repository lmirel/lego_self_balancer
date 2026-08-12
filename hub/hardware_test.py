"""Safely identify the drive motor and balance-sensor conventions.

Edit only the two port constants below while discovering the wiring. This is a
diagnostic program, not a controller. It never drives both motors together.
The hub's CENTER button retains its default role and aborts the program.
"""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Button, Port
from pybricks.pupdevices import Motor
from pybricks.tools import StopWatch, wait


# Motor mapping measured during Phase 1. These values are deliberately not read
# from the Mac-side TOML because this diagnostic must remain a single hub file.
LEFT_MOTOR_PORT = Port.A
RIGHT_MOTOR_PORT = Port.E

SAMPLE_PERIOD_MS = 100
MOTOR_TEST_DUTY = 30
MOTOR_TEST_MS = 500


def pulse(motor, label):
    """Apply one short, conservative positive-duty identification pulse."""
    angle_before = motor.angle()
    print("MOTOR_TEST_START,{},{},{}".format(label, MOTOR_TEST_DUTY, MOTOR_TEST_MS))
    try:
        motor.dc(MOTOR_TEST_DUTY)
        wait(MOTOR_TEST_MS)
    finally:
        motor.stop()
    angle_after = motor.angle()
    print("MOTOR_TEST_STOP,{},angle_before={},angle_after={},delta={}".format(
        label, angle_before, angle_after, angle_after - angle_before
    ))


hub = PrimeHub()
left_motor = Motor(LEFT_MOTOR_PORT)
right_motor = Motor(RIGHT_MOTOR_PORT)
clock = StopWatch()

print("INSTRUCTIONS,Support robot with wheels clear of people and surfaces")
print("INSTRUCTIONS,Move robot forward/backward by hand and watch pitch/roll/gyro")
print("INSTRUCTIONS,LEFT button pulses configured left motor; RIGHT pulses right")
print("INSTRUCTIONS,CENTER aborts at any time; each motor pulse is short and low power")
print("CONFIG,left_port={},right_port={}".format(LEFT_MOTOR_PORT, RIGHT_MOTOR_PORT))
print("timestamp_ms,pitch_deg,roll_deg,gyro_x_dps,gyro_y_dps,gyro_z_dps")

previous_buttons = set()
try:
    while True:
        pitch, roll = hub.imu.tilt()
        gyro_x = hub.imu.angular_velocity(Axis.X)
        gyro_y = hub.imu.angular_velocity(Axis.Y)
        gyro_z = hub.imu.angular_velocity(Axis.Z)
        print("{},{},{},{:.2f},{:.2f},{:.2f}".format(
            clock.time(), pitch, roll, gyro_x, gyro_y, gyro_z
        ))

        buttons = set(hub.buttons.pressed())
        newly_pressed = buttons - previous_buttons
        if Button.LEFT in newly_pressed:
            pulse(left_motor, "left")
        if Button.RIGHT in newly_pressed:
            pulse(right_motor, "right")
        previous_buttons = buttons
        wait(SAMPLE_PERIOD_MS)
finally:
    # Also covers Python exceptions. CENTER is the firmware-level stop button.
    left_motor.stop()
    right_motor.stop()
    print("STOPPED")
