"""Phase 2 BLE command receiver and watchdog test. Motors are never opened."""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Button
from pybricks.tools import StopWatch, wait
from uselect import poll
from usys import stdin


LOOP_MS = 5
WATCHDOG_MS = 250
REPORT_MS = 100
MAX_LINE_LENGTH = 48


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
input_poll = poll()
input_poll.register(stdin)
clock = StopWatch()

buffer = ""
last_sequence = None
last_command_ms = None
drive = 0.0
turn = 0.0
watchdog_active = True
next_report_ms = 0
state = "RUNNING"

print("LINK_READY,watchdog_ms={}".format(WATCHDOG_MS))
hub.display.char("L")

try:
    while True:
        now_ms = clock.time()
        if Button.CENTER in hub.buttons.pressed():
            state = "CENTER_STOP"
            print("LINK_STOPPED,reason=center_button")
            break

        # Drain every byte currently available without ever blocking the loop.
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
                last_sequence, drive, turn = command
                last_command_ms = now_ms
                if watchdog_active:
                    watchdog_active = False
                    print("LINK_ACTIVE,sequence={:x}".format(last_sequence))
            elif len(buffer) < MAX_LINE_LENGTH:
                buffer += character
            else:
                buffer = ""
                print("COMMAND_REJECTED,line_too_long")

        if last_command_ms is None or now_ms - last_command_ms > WATCHDOG_MS:
            drive = 0.0
            turn = 0.0
            if not watchdog_active:
                watchdog_active = True
                print("WATCHDOG,commands_zeroed")

        if now_ms >= next_report_ms:
            age_ms = -1 if last_command_ms is None else now_ms - last_command_ms
            sequence_text = "none" if last_sequence is None else "{:x}".format(last_sequence)
            print(
                "COMMAND_STATUS,sequence={},drive={:.2f},turn={:.2f},"
                "age_ms={},watchdog={}".format(
                    sequence_text, drive, turn, age_ms,
                    1 if watchdog_active else 0,
                )
            )
            next_report_ms += REPORT_MS

        wait(LOOP_MS)
finally:
    hub.display.off()
    print("LINK_EXIT,state={}".format(state))
