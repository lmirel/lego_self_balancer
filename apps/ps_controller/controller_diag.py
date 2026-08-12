#!/usr/bin/env python3
"""Inspect and filter PS controller axes without connecting to the robot."""

import argparse
import time

from control import AxisProcessor


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--drive-axis", type=int, default=1,
                        help="reserved drive input axis (default: 1)")
    parser.add_argument("--turn-axis", type=int, default=2,
                        help="steering input axis (default: 2)")
    parser.add_argument("--deadzone", type=float, default=0.08)
    parser.add_argument("--expo", type=float, default=0.35)
    parser.add_argument("--filter-alpha", type=float, default=0.25)
    parser.add_argument("--slew-rate", type=float, default=3.0)
    parser.add_argument("--hz", type=int, default=60)
    parser.add_argument("--list", action="store_true",
                        help="list controllers and exit")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        import pygame
    except ImportError:
        print("pygame is not installed. Run: python -m pip install -r requirements.txt")
        return 2

    pygame.init()
    pygame.joystick.init()
    try:
        count = pygame.joystick.get_count()
        if count == 0:
            print("No controller found. Pair/connect it to macOS and try again.")
            return 1

        print(f"Found {count} controller(s):")
        for index in range(count):
            candidate = pygame.joystick.Joystick(index)
            print(f"  [{index}] {candidate.get_name()} "
                  f"axes={candidate.get_numaxes()} buttons={candidate.get_numbuttons()}")
        if args.list:
            return 0

        joystick = pygame.joystick.Joystick(0)
        required_axis = max(args.drive_axis, args.turn_axis)
        if required_axis >= joystick.get_numaxes():
            print(f"Controller has {joystick.get_numaxes()} axes; requested axis "
                  f"{required_axis}. Use --drive-axis/--turn-axis to correct it.")
            return 2

        drive = AxisProcessor(args.deadzone, args.expo,
                              args.filter_alpha, args.slew_rate)
        turn = AxisProcessor(args.deadzone, args.expo,
                             args.filter_alpha, args.slew_rate)
        interval = 1.0 / max(1, args.hz)
        previous = time.monotonic()
        next_print = previous
        print("Using right-stick vertical for drive and left-stick horizontal for turn.")
        print("Push each through its full range. Press Ctrl-C to finish.")
        print(" raw_drive raw_turn | drive  turn | all raw axes")

        while True:
            pygame.event.pump()
            if not joystick.get_init():
                print("\nController disconnected.")
                return 1

            now = time.monotonic()
            dt = max(0.0, min(0.1, now - previous))
            previous = now
            raw_drive = -joystick.get_axis(args.drive_axis)
            raw_turn = joystick.get_axis(args.turn_axis)
            drive_value = drive.update(raw_drive, dt)
            turn_value = turn.update(raw_turn, dt)
            if now >= next_print:
                all_axes = " ".join(
                    f"{axis}:{joystick.get_axis(axis):+.2f}"
                    for axis in range(joystick.get_numaxes())
                )
                print(f" {raw_drive:+.3f}    {raw_turn:+.3f}  | "
                      f"{drive_value:+.3f} {turn_value:+.3f} | {all_axes}")
                next_print = now + 0.1
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\nStopped.")
        return 0
    finally:
        pygame.quit()


if __name__ == "__main__":
    raise SystemExit(main())
