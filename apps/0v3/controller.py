#!/usr/bin/env python3
"""Balance indefinitely with bounded PS4 drive and steering commands."""

import argparse
import asyncio
from pathlib import Path
import sys
import time

from control import AxisProcessor
from protocol import format_command, format_config


HUB_PROGRAM = Path(__file__).resolve().parent / "hub" / "main.py"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", help="Pybricks hub Bluetooth name or address")
    parser.add_argument("--drive-axis", type=int, default=1)
    parser.add_argument("--turn-axis", type=int, default=2)
    parser.add_argument("--max-drive-speed-dps", type=float, default=300.0)
    parser.add_argument("--max-turn-duty", type=float, default=20.0)
    parser.add_argument(
        "--use-stored-program", action="store_true",
        help="start the program already stored on the hub without rebuilding",
    )
    parser.add_argument("--send-hz", type=int, default=50)
    return parser.parse_args()


async def read_output(hub, stopped, balance_active):
    while not stopped.is_set():
        line = await hub.read_line()
        print(line, flush=True)
        if line.startswith("BALANCE_ACTIVE"):
            balance_active.set()
        if line.startswith("BALANCE_STOPPED") or line.startswith("BALANCE_EXIT"):
            stopped.set()


async def send_commands(hub, joystick, pygame, args, stopped, balance_active):
    drive_filter = AxisProcessor()
    turn_filter = AxisProcessor()
    interval = 1.0 / max(1, args.send_hz)
    previous = time.monotonic()
    sequence = 0
    while not balance_active.is_set() and not stopped.is_set():
        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEREMOVED:
                raise RuntimeError("controller disconnected during calibration")
        await asyncio.sleep(0.02)
    await hub.write_line(format_config(
        args.max_drive_speed_dps, args.max_turn_duty
    ))
    previous = time.monotonic()
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
    # Validate before controller discovery or Bluetooth connection.
    format_config(args.max_drive_speed_dps, args.max_turn_duty)
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
        print("Hold the robot upright for countdown and release as usual. "
              "Axis 1 drives; axis 2 steers. Begin with very small commands. "
              "CENTER or Ctrl-C stops.")
        print("Searching for {}...".format(args.name or "any Pybricks hub"))
        device = await find_device(args.name)
        hub = PybricksHubBLE(device)
        await hub.connect()
        if args.use_stored_program:
            print("Connected. Starting the program already stored on the hub...")
            program = None
        else:
            print("Connected. Downloading remote-control balancer...")
            program = str(HUB_PROGRAM)
        await hub.run(program, wait=False, print_output=False, line_handler=True)

        stopped = asyncio.Event()
        balance_active = asyncio.Event()
        reader = asyncio.create_task(read_output(hub, stopped, balance_active))
        sender = asyncio.create_task(send_commands(
            hub, joystick, pygame, args, stopped, balance_active
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
