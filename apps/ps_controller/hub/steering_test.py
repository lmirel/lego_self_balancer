"""Phase 3 suspended-wheel steering polarity test."""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Port
from pybricks.pupdevices import Motor
from pybricks.tools import StopWatch, wait
from uselect import poll
from usys import stdin


LEFT_SIGN = -1
RIGHT_SIGN = 1
MIN_TURN_DUTY = 30.0
MAX_TURN_DUTY = 45.0
TURN_NEUTRAL = 0.05
LOOP_MS = 5
WATCHDOG_MS = 250
REPORT_MS = 100
MAX_LINE_LENGTH = 48


def stop_motors(left, right):
    left.stop()
    right.stop()


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


hub = PrimeHub()
hub.system.set_stop_button(Button.BLUETOOTH)
left = Motor(Port.A)
right = Motor(Port.E)
stop_motors(left, right)

input_poll = poll()
input_poll.register(stdin)
clock = StopWatch()
buffer = ""
last_sequence = None
last_command_ms = None
turn = 0.0
watchdog_active = True
next_report_ms = 0
state = "RUNNING"

print("STEERING_READY,min_turn_duty={},max_turn_duty={},watchdog_ms={}".format(
    MIN_TURN_DUTY, MAX_TURN_DUTY, WATCHDOG_MS
))
hub.display.char("T")

try:
    while True:
        now_ms = clock.time()
        if Button.CENTER in hub.buttons.pressed():
            state = "CENTER_STOP"
            print("STEERING_STOPPED,reason=center_button")
            break

        while input_poll.poll(0):
            character = stdin.read(1)
            if not character:
                break
            if character == "\n":
                command = parse_command(buffer.rstrip("\r"), last_sequence)
                buffer = ""
                if command is None:
                    print("COMMAND_REJECTED")
                    continue
                last_sequence, ignored_drive, turn = command
                last_command_ms = now_ms
                if watchdog_active:
                    watchdog_active = False
                    print("STEERING_ACTIVE,sequence={:x}".format(last_sequence))
            elif len(buffer) < MAX_LINE_LENGTH:
                buffer += character
            else:
                buffer = ""
                print("COMMAND_REJECTED,line_too_long")

        if last_command_ms is None or now_ms - last_command_ms > WATCHDOG_MS:
            turn = 0.0
            if not watchdog_active:
                watchdog_active = True
                print("WATCHDOG,motors_stopped")

        if watchdog_active or abs(turn) <= TURN_NEUTRAL:
            stop_motors(left, right)
            turn_duty = 0.0
        else:
            magnitude = (abs(turn) - TURN_NEUTRAL) / (1.0 - TURN_NEUTRAL)
            turn_duty = MIN_TURN_DUTY + magnitude * (
                MAX_TURN_DUTY - MIN_TURN_DUTY
            )
            if turn < 0:
                turn_duty = -turn_duty
            # Positive stick/right command: left wheel forward, right backward.
            left.dc(LEFT_SIGN * turn_duty)
            right.dc(RIGHT_SIGN * -turn_duty)

        if now_ms >= next_report_ms:
            print(
                "STEERING_STATUS,turn={:.2f},duty={:.2f},drive_ignored={:.2f},"
                "watchdog={}".format(
                    turn, turn_duty, ignored_drive if last_sequence is not None else 0.0,
                    1 if watchdog_active else 0,
                )
            )
            next_report_ms += REPORT_MS
        wait(LOOP_MS)
finally:
    stop_motors(left, right)
    hub.display.off()
    print("STEERING_EXIT,state={}".format(state))
