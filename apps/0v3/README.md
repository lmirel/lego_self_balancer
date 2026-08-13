# Standalone PS controller balancer 0v3

This directory is a self-contained snapshot of the validated `0v3` application.
A Mac reads a PS4 controller with Pygame and sends drive/turn commands over BLE;
the Prime Hub runs the 200 Hz balance loop and enforces safety limits. Nothing in
this directory imports code from the rest of the repository.

## Validated hardware and controls

- LEGO MINDSTORMS Robot Inventor 51515 / Prime Hub with Pybricks firmware
- Left motor: port A, inverted polarity
- Right motor: port E, normal polarity
- Balance gyro: X axis; absolute-angle anchor: IMU roll
- PS4 controller drive: axis 1
- PS4 controller turn: axis 2
- Default maximum drive reference: 300 wheel degrees/second
- Default maximum turn differential: 20 duty

The gains and motor signs are specific to the tested tall robot. Revalidate them
after changing its construction, gearing, wheels, motor ports, or weight layout.

## Files

```text
controller.py     Mac entry point: controller input and BLE connection
control.py        dead zone, response curve, filtering, and slew limiting
protocol.py       compact command/config protocol and host-side bounds
hub/main.py       Pybricks balance, drive, steering, and safety program
requirements.txt minimal pinned host dependencies
VERSION          packaged application version
```

## 1. Install Pybricks firmware

The standard LEGO firmware does not expose the Pybricks BLE service.

1. Open [Pybricks Code](https://code.pybricks.com/) in Chrome or Edge.
2. Select **Tools → Install Pybricks Firmware**.
3. Connect the Prime/Inventor Hub over USB and complete the instructions.
4. Restart the hub and enable Bluetooth advertising.

Installing or restoring firmware can erase programs and settings stored on the
hub. The Robot Inventor 51515 uses the Prime Hub Pybricks firmware.

## 2. Prepare the host environment

Use Python 3.10 or newer. From this directory:

```bash
cd apps/0v3
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

Pair the PS4 controller in macOS Bluetooth settings, then verify that Pygame sees
it:

```bash
python -c "import pygame; pygame.init(); print(pygame.joystick.get_count())"
```

The result should be at least `1`. Disconnect the hub from Pybricks Code or any
other BLE application before starting the host app.

## 3. Prepare the robot

1. Connect the left motor to A and the right motor to E.
2. Use a level, non-slippery floor with clear travel and stopping space.
3. Keep cables, fingers, hair, and loose objects away from the wheels.
4. Keep a hand ready to catch the robot without restraining normal motion.
5. Start with both controller axes centered.

## 4. Deploy and run

The normal command compiles and downloads `hub/main.py`, starts it, and keeps the
BLE connection open for controller commands:

```bash
source .venv/bin/activate
python controller.py
```

If multiple Pybricks hubs are nearby:

```bash
python controller.py --name "YOUR_HUB_NAME"
```

Operation sequence:

1. Hold the robot motionless at its natural upright pose.
2. Keep holding while the hub displays `H` and counts down 3–2–1.
3. Release gently with drive and turn centered.
4. Use axis 1 to drive and axis 2 to turn.
5. Press the hub CENTER button or Ctrl-C for a normal stop.

The host intentionally waits for `BALANCE_ACTIVE` before sending commands, so
calibration cannot fill the hub's stdin buffer.

The startup pose is learned as the upright reference; there is no hard-coded
forward-lean angle. Automatic countdown begins after 500 ms within all three
stillness limits:

```text
gyro rate              <= 5 degrees/second
wheel step             <= 3 degrees per 20 ms sample
cumulative wheel drift <= 12 degrees
```

The countdown allows up to 20 degrees/second gyro rate, 6 degrees wheel step,
and 24 degrees cumulative drift before restarting. These tolerances allow a
natural upright hold without requiring the robot to lean forward, while the
cumulative limit prevents slow rolling from being accepted as stationary.
`CALIBRATION_WAIT` reports the three live measurements every 500 ms;
`COUNTDOWN_CANCELLED` reports the measurement that interrupted a countdown.

## Runtime parameters

The validated defaults are equivalent to:

```bash
python controller.py \
  --drive-axis 1 \
  --turn-axis 2 \
  --max-drive-speed-dps 300 \
  --max-turn-duty 20
```

The hub independently rejects drive limits above 300 degrees/second and turn
limits above 20 duty. Lower values can be useful for restricted spaces:

```bash
python controller.py --max-drive-speed-dps 180 --max-turn-duty 10
```

After one successful normal download, later parameter comparisons can start the
program already stored on the hub without rebuilding it:

```bash
python controller.py --use-stored-program \
  --max-drive-speed-dps 300 --max-turn-duty 20
```

Use `--use-stored-program` only when this app's `hub/main.py` is the program most
recently downloaded to the selected hub. Run normally again after modifying hub
code or installing different firmware.

## Safety behavior

- Hub CENTER: stops the program and motors.
- Host Ctrl-C or controller disconnect: stops the hub program and motors.
- Bluetooth firmware stop: stops the program.
- Missing commands for 250 ms: zeros drive/turn; stationary balancing continues.
- Relative lean at 12 degrees: stops the program and motors.
- Wheel speed at 750 degrees/second: stops the program and motors.
- Any hub exception: the `finally` block stops both motors.

A BLE link loss cannot deliver an explicit stop, so the hub-owned 250 ms
watchdog first removes motion commands while balance continues. Always keep a
hand ready to catch the robot.

## Controller and protocol

The locked balance constants are:

```text
loop period                 5 ms
wheel-speed window          200 ms
gyro-rate gain              0.018
angle gain                  19.0
position gain               0.45
wheel-speed gain            0.20
deadband compensation       8 duty
absolute-angle correction   5 s
```

The host samples and sends at 50 Hz. Input processing applies an 8% rescaled
dead zone, 35% cubic response blend, low-pass filtering, and a 3 units/second
slew limit. Commands are sequenced; the hub rejects duplicates and stale packets.
Drive advances the position reference and freezes it at neutral. Turn is applied
as a differential around balance duty.

At the validated 300 degrees/second setting, measured speed settled mostly near
280–350 degrees/second after an initial burst near 500. Peak observed duty was
about 62%; sustained lean was generally 8–10 degrees, with 11.25 degrees seen
during release against the 12-degree cutoff. This narrow release margin is why
`0v3` does not permit a higher drive limit.

## Troubleshooting

### No controller found

Pair and connect the controller in macOS Bluetooth settings before running the
app. Close other programs using it. If its axes differ, inspect them with a small
Pygame diagnostic or override `--drive-axis` and `--turn-axis`.

### Searching for a Pybricks hub times out

Confirm Pybricks—not standard LEGO—firmware is installed, restart the hub,
enable advertising, close Pybricks Code, move the hub closer, or pass `--name`.

### `unpack requires a buffer of 10 bytes`

The installed `pybricksdev` is too old for the hub firmware. Activate this app's
environment and reinstall `requirements.txt` with Python 3.10 or newer.

### GATT application error `0x81`

Use this packaged `controller.py`. It waits for calibration to finish before
writing commands. Older runners could fill stdin by transmitting during the
countdown.

### Countdown repeatedly restarts

Hold the chassis near its natural upright pose without forcing it forward. Keep
the wheels from rolling more than about 12 degrees during the initial half-second
stable window. Inspect `CALIBRATION_WAIT` and `COUNTDOWN_CANCELLED` to see whether
gyro rate, instantaneous wheel step, or cumulative wheel drift is the blocker.

### Motors or steering move in the wrong direction

Stop immediately. Confirm motor A is on the left and motor E is on the right.
Do not change gains to compensate for swapped ports or altered polarity.
