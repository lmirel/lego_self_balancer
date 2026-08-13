# Standalone PS controller balancer 0v3.3

This directory is a self-contained snapshot of the validated high-speed
PlayStation-controlled balancing robot. It does not import code from elsewhere
in the repository.

The Mac reads a PS4 controller through pygame and transmits normalized drive and
turn intent over BLE. The Robot Inventor/Prime Hub runs the 5 ms balance loop,
drive trajectory, adaptive braking, steering allocation, watchdog, and physical
safety logic locally.

Architecture:

- [System architecture](ARCHITECTURE.md)
- [Host architecture](HOST_ARCHITECTURE.md)
- [Robot hub architecture](HUB_ARCHITECTURE.md)

## Validated hardware

- LEGO MINDSTORMS Robot Inventor 51515 / Prime Hub with Pybricks firmware
- Left motor: port A, inverted logical polarity
- Right motor: port E, normal logical polarity
- Hub display mounted 90 degrees left
- PS4 drive axis: 1
- PS4 turn axis: 2

The controller gains and signs are specific to the tested tall robot. Revalidate
after changing construction, gearing, wheels, ports, or weight distribution.

## Package contents

```text
controller.py          Host entry point and BLE orchestration
control.py             Joystick dead zone, response, filtering, slew
protocol.py            Compact command/configuration protocol
hub/main.py            Complete Pybricks robot application
ARCHITECTURE.md        System design and responsibility boundaries
HOST_ARCHITECTURE.md   Host component details
HUB_ARCHITECTURE.md    Robot control and safety details
requirements.txt       Pinned host dependencies
VERSION                Release identifier
```

## Install Pybricks firmware

The standard LEGO firmware does not expose the Pybricks BLE service.

1. Open [Pybricks Code](https://code.pybricks.com/) in Chrome or Edge.
2. Choose **Tools → Install Pybricks Firmware**.
3. Connect the hub over USB and complete the installation.
4. Restart the hub and enable Bluetooth advertising.

Firmware installation or restoration can erase stored programs and settings.

## Host environment

Use Python 3.10 or newer:

```bash
cd /Users/mirel/Work/lego/robot/self-balancing/apps/0v3p3
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

Pair the PS4 controller in macOS Bluetooth settings. Disconnect the hub from
Pybricks Code or any other BLE application.

## Deploy and run

Place the robot on a level floor with generous travel and braking space. Hold it
upright with both sticks centered:

```bash
source .venv/bin/activate
python controller.py \
  --max-drive-speed-dps 800 \
  --max-turn-duty 20
```

The command compiles, downloads, and starts `hub/main.py`. If multiple hubs are
nearby, add `--name "YOUR_HUB_NAME"`.

Startup sequence:

1. The left-rotated display shows `H` while validating orientation/stillness.
2. The hub shows `R` and beeps when balancing begins.
3. Release at the beep.
4. Axis 1 drives; axis 2 turns.
5. The hub center button stops the program.

After one successful deployment, the stored program can be reused:

```bash
python controller.py --use-stored-program \
  --max-drive-speed-dps 800 \
  --max-turn-duty 20
```

Run without `--use-stored-program` after modifying or replacing hub code.

## Runtime behavior

- Host input is sent at 50 Hz.
- Hub balance runs every 5 ms.
- Telemetry is emitted at 2 Hz to limit BLE timing disturbance.
- Drive is a wheel-position trajectory rate, not direct duty.
- Acceleration is 600 degrees/second².
- Braking falls from 1800 degrees/second² near rest to 200 at 400 dps and stays
  at 200 above that speed.
- Turn authority is capped at 20 duty below 500 dps, 10 at 500–699, and 5 at
  700+, then reduced further when balance needs motor headroom.

High-speed travel needs substantial clear stopping distance. A requested speed
is not a strict measured-speed limit; transient overshoot is expected.

## Host-independent balancing

Ctrl-C or ordinary host termination disconnects without explicitly stopping the
hub program. After 250 ms without valid commands, the hub zeros drive and turn
but continues balancing.

Use the hub center button for the normal physical stop. For maintenance, this
option explicitly stops the hub program when the host exits:

```bash
python controller.py --stop-hub-on-exit
```

An uncatchable host kill cannot send a final packet, but the hub watchdog still
removes motion intent.

## Safety

- Absolute lean at 12 degrees: stop motors and exit.
- Speed at least 400 dps with lean at 10.5 degrees: stop motors and exit.
- Absolute wheel speed at 1200 dps: stop motors and exit.
- Neutral/reference-zero wheel speed above 400 dps for 100 ms while lean remains
  within 8 degrees: treat as lifted, stop motors, and exit.
- Missing commands for 250 ms: zero drive/turn and continue balancing.
- Any terminal path or exception stops both motors in `finally`.

After a lift stop, place the robot back on the floor and restart it. Automatic
in-hand restart is intentionally disabled.

## Troubleshooting

### Hub is not found

Confirm Pybricks firmware is installed, advertising is active, and Pybricks Code
is disconnected. Pass `--name` if needed.

### Controller is not found or axes differ

Pair the controller before launch. Override `--drive-axis` or `--turn-axis`
for a different SDL mapping.

### Display is rotated incorrectly

This release configures `Side.LEFT` globally. Change that single orientation
call in `hub/main.py` only if the physical hub mounting changes.

### Braking rocks before settling

The frozen position target and delayed 200 ms speed estimate can reverse duty
several times after the motion reference reaches zero. Keep adequate stopping
space; continuous or one-shot target rebasing experiments were rejected because
they worsened physical stability.

