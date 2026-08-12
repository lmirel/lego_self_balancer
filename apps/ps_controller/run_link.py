#!/usr/bin/env python3
"""Run the Phase 2 PS controller to Pybricks BLE link test."""

import argparse
import asyncio
from pathlib import Path
import sys
import time

from control import AxisProcessor
from protocol import format_command


HERE = Path(__file__).resolve().parent
HUB_PROGRAM = HERE / "hub" / "link_test.py"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", help="Pybricks hub Bluetooth name or address")
    parser.add_argument("--drive-axis", type=int, default=1)
    parser.add_argument("--turn-axis", type=int, default=2)
    parser.add_argument("--deadzone", type=float, default=0.08)
    parser.add_argument("--expo", type=float, default=0.35)
    parser.add_argument("--filter-alpha", type=float, default=0.25)
    parser.add_argument("--slew-rate", type=float, default=3.0)
    parser.add_argument("--send-hz", type=int, default=50)
    parser.add_argument(
        "--watchdog-test", action="store_true",
        help="pause transmission once to prove the hub watchdog zeros commands",
    )
    return parser.parse_args()


async def output_reader(hub, stopped: asyncio.Event) -> None:
    while not stopped.is_set():
        line = await hub.read_line()
        print(line, flush=True)
        if line.startswith("LINK_STOPPED") or line.startswith("LINK_EXIT"):
            stopped.set()


async def command_sender(hub, joystick, pygame, args, stopped: asyncio.Event) -> None:
    drive_filter = AxisProcessor(
        args.deadzone, args.expo, args.filter_alpha, args.slew_rate
    )
    turn_filter = AxisProcessor(
        args.deadzone, args.expo, args.filter_alpha, args.slew_rate
    )
    interval = 1.0 / max(1, args.send_hz)
    previous = time.monotonic()
    sequence = 0
    next_report = previous
    started = previous
    watchdog_pause_reported = False

    while not stopped.is_set():
        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEREMOVED:
                await hub.write_line(format_command(sequence, 0.0, 0.0))
                raise RuntimeError("controller disconnected")

        now = time.monotonic()
        dt = max(0.0, min(0.1, now - previous))
        previous = now
        raw_drive = -joystick.get_axis(args.drive_axis)
        raw_turn = joystick.get_axis(args.turn_axis)
        drive = drive_filter.update(raw_drive, dt)
        turn = turn_filter.update(raw_turn, dt)
        elapsed = now - started
        watchdog_pause = args.watchdog_test and 2.0 <= elapsed < 2.6
        if watchdog_pause:
            if not watchdog_pause_reported:
                print("HOST_WATCHDOG_TEST,pausing_commands_ms=600")
                watchdog_pause_reported = True
        else:
            await hub.write_line(format_command(sequence, drive, turn))
            sequence = (sequence + 1) & 0xFFFF

        if now >= next_report:
            print("HOST_COMMAND,drive={:+.2f},turn={:+.2f}".format(drive, turn))
            next_report = now + 0.5
        await asyncio.sleep(interval)


async def run(args: argparse.Namespace) -> int:
    try:
        import pygame
        from pybricksdev.ble import find_device
        from pybricksdev.connections.pybricks import PybricksHubBLE
    except ImportError as error:
        print("Missing dependency: {}. Install requirements.txt first.".format(error))
        return 2

    pygame.init()
    pygame.joystick.init()
    hub = None
    try:
        if pygame.joystick.get_count() == 0:
            print("No controller found. Pair/connect the PS4 controller first.")
            return 1
        joystick = pygame.joystick.Joystick(0)
        required_axis = max(args.drive_axis, args.turn_axis)
        if required_axis >= joystick.get_numaxes():
            print("Controller does not provide requested axis {}.".format(required_axis))
            return 2
        print("CONTROLLER,{},axes={}".format(
            joystick.get_name(), joystick.get_numaxes()
        ))
        print("Searching for {}...".format(args.name or "any Pybricks hub"))
        device = await find_device(args.name)
        hub = PybricksHubBLE(device)
        await hub.connect()
        print("Connected. Downloading Phase 2 link test...")
        await hub.run(str(HUB_PROGRAM), wait=False, print_output=False, line_handler=True)

        stopped = asyncio.Event()
        reader = asyncio.create_task(output_reader(hub, stopped))
        sender = asyncio.create_task(command_sender(
            hub, joystick, pygame, args, stopped
        ))
        done, pending = await asyncio.wait(
            {reader, sender}, return_when=asyncio.FIRST_COMPLETED
        )
        stopped.set()
        for task in pending:
            task.cancel()
        for task in done:
            task.result()
        return 0
    finally:
        if hub is not None:
            try:
                await hub.stop_user_program()
            except Exception:
                pass
            await hub.disconnect()
        pygame.quit()


def main() -> int:
    args = parse_args()
    try:
        return asyncio.run(run(args))
    except KeyboardInterrupt:
        print("\nStopped by host; hub program stopped and commands are zero.")
        return 0
    except Exception as error:
        print("ERROR,{}".format(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
