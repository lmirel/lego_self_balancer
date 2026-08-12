#!/usr/bin/env python3
"""Run the Phase 3 suspended-wheel steering polarity test."""

import argparse
import asyncio
from pathlib import Path
import sys
import time

from control import AxisProcessor
from protocol import format_command


HUB_PROGRAM = Path(__file__).resolve().parent / "hub" / "steering_test.py"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--wheels-suspended", action="store_true",
        help="confirm both wheels are clear of the floor",
    )
    parser.add_argument("--name", help="Pybricks hub Bluetooth name or address")
    parser.add_argument("--drive-axis", type=int, default=1)
    parser.add_argument("--turn-axis", type=int, default=2)
    parser.add_argument("--send-hz", type=int, default=50)
    return parser.parse_args()


async def read_output(hub, stopped):
    while not stopped.is_set():
        line = await hub.read_line()
        print(line, flush=True)
        if line.startswith("STEERING_STOPPED") or line.startswith("STEERING_EXIT"):
            stopped.set()


async def send_commands(hub, joystick, pygame, args, stopped):
    drive_filter = AxisProcessor()
    turn_filter = AxisProcessor()
    interval = 1.0 / max(1, args.send_hz)
    previous = time.monotonic()
    sequence = 0
    while not stopped.is_set():
        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEREMOVED:
                await hub.write_line(format_command(sequence, 0.0, 0.0))
                raise RuntimeError("controller disconnected")
        now = time.monotonic()
        dt = max(0.0, min(0.1, now - previous))
        previous = now
        drive = drive_filter.update(-joystick.get_axis(args.drive_axis), dt)
        turn = turn_filter.update(joystick.get_axis(args.turn_axis), dt)
        await hub.write_line(format_command(sequence, drive, turn))
        sequence = (sequence + 1) & 0xFFFF
        await asyncio.sleep(interval)


async def run(args):
    if not args.wheels_suspended:
        print("Refusing motor test: support the robot with both wheels clear of "
              "the floor, then add --wheels-suspended.")
        return 2
    try:
        import pygame
        from pybricksdev.ble import find_device
        from pybricksdev.connections.pybricks import PybricksHubBLE
    except ImportError as error:
        print("Missing dependency: {}".format(error))
        return 2

    pygame.init()
    pygame.joystick.init()
    hub = None
    try:
        if pygame.joystick.get_count() == 0:
            print("No controller found.")
            return 1
        joystick = pygame.joystick.Joystick(0)
        print("CONTROLLER,{},axes={}".format(
            joystick.get_name(), joystick.get_numaxes()
        ))
        print("Keep wheels suspended. Drive axis 1 is ignored; move steering "
              "axis 2. CENTER or Ctrl-C stops.")
        device = await find_device(args.name)
        hub = PybricksHubBLE(device)
        await hub.connect()
        await hub.run(str(HUB_PROGRAM), wait=False, print_output=False, line_handler=True)
        stopped = asyncio.Event()
        reader = asyncio.create_task(read_output(hub, stopped))
        sender = asyncio.create_task(send_commands(
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


def main():
    try:
        return asyncio.run(run(parse_args()))
    except KeyboardInterrupt:
        print("\nStopped by host; hub program and motors stopped.")
        return 0
    except Exception as error:
        print("ERROR,{}".format(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
