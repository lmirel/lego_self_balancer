# Standalone stationary balancer 0v2.2

This directory packages Git tag `0v2.2` as a standalone Pybricks app. The
filesystem name uses `0v2p2` because a plain identifier is convenient for app
folders; it means version `0v2.2`.

## Validated hardware

- LEGO MINDSTORMS Robot Inventor 51515 / Prime Hub running Pybricks firmware
- Left motor: port A, inverted polarity
- Right motor: port E, normal polarity
- Balance gyro axis: X
- Absolute-angle anchor: IMU roll

The constants are specific to the physically tested tall robot. Revalidate
ports, polarity, axis, and gains after changing its construction.

## 1. Install Pybricks firmware

The standard LEGO firmware does not provide the Pybricks Bluetooth service
required by this app.

1. Open [Pybricks Code](https://code.pybricks.com/) in Chrome or Edge.
2. Open **Tools → Install Pybricks Firmware**.
3. Select the Prime/Inventor Hub and connect it by micro-USB.
4. Follow the displayed firmware-install procedure.

The 51515 Inventor Hub uses the Prime Hub Pybricks firmware. Installing or
restoring firmware can erase programs and settings stored on the hub.

## 2. Prepare the Mac environment

Use Python 3.10 or newer. Do not use Apple's system Python 3.9; it can resolve
an obsolete `pybricksdev` version that is incompatible with current firmware.

From the repository root:

```bash
python3 --version
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

On the development Mac, Homebrew Python required its Homebrew Expat library to
be explicit. If Python reports a `pyexpat` missing-symbol error, run:

```bash
export DYLD_LIBRARY_PATH=/opt/homebrew/opt/expat/lib
```

Then activate the environment again if necessary:

```bash
source .venv/bin/activate
```

Verify the deployment tool:

```bash
pybricksdev --version
```

## 3. Prepare the robot

1. Connect the left motor to port A and the right motor to port E.
2. Put the robot on a level, non-slippery surface with room to move.
3. Keep hair, cables, fingers, and loose objects away from the wheels.
4. Keep a hand close enough to catch the robot without restraining it.
5. Turn on the hub and enable Bluetooth advertising.

## 4. Deploy and run

From the repository root with the project virtual environment active:

```bash
source .venv/bin/activate
.venv/bin/pybricksdev run ble apps/0v2p2/main.py
```

The command finds an advertising hub, transfers `main.py`, and starts it. If
multiple compatible hubs are nearby, specify the name assigned during firmware
installation:

```bash
.venv/bin/pybricksdev run ble --name YOUR_HUB_NAME apps/0v2p2/main.py
```

The equivalent command from the app directory is:

```bash
cd apps/0v2p2
../../.venv/bin/pybricksdev run ble main.py
```

Alternatively, open `main.py` in [Pybricks Code](https://code.pybricks.com/),
connect the hub, and use the editor's run/download controls.

## 5. Operate the app

1. Hold the robot motionless at its natural balance pose.
2. The hub displays `H` while waiting for a stable pose.
3. Keep holding it during the 3–2–1 countdown.
4. Release it gently after the countdown.
5. Press CENTER for a normal stop.

The Bluetooth button remains the Pybricks firmware stop. The motors also stop
if relative tilt reaches 12 degrees or an exception occurs. Keep a hand ready
to catch the robot.

To run again, let the previous command finish, return the robot upright, enable
Bluetooth advertising if needed, and repeat the deployment command.

## Troubleshooting

### `Searching for any hub with Pybricks service...` times out

- Confirm that Pybricks firmware—not standard LEGO firmware—is installed.
- Turn the hub off and on, then enable Bluetooth advertising again.
- Disconnect the hub from Pybricks Code or another application before using
  `pybricksdev`.
- Move the hub closer to the Mac.
- Use `--name` when multiple hubs are nearby.

### `unpack requires a buffer of 10 bytes`

The local `pybricksdev` is too old for the hub firmware. Recreate the virtual
environment with Python 3.10 or newer and reinstall `requirements.txt`.

### Countdown repeatedly restarts

The hub or wheels are moving too much for calibration. Hold the chassis and
wheels still at the intended balance pose until the countdown completes.

### Motors move in the wrong direction

Stop immediately. This app assumes left motor A with inverted polarity and
right motor E with normal polarity. Do not alter gains to compensate for a
wiring or polarity mismatch.

### Robot rocks instead of remaining perfectly still

Some bounded rocking is expected on the validated drivetrain because of motor
deadband, tire friction, and mechanical play. Version 0v2.2 prioritizes robust
balance and fixed-position behavior over perfectly motionless holding.

## Controller

The hub runs a 5 ms full-state feedback loop using:

```text
rate gain                 0.018
angle gain                19.0
fixed-position gain       0.45
wheel-speed gain          0.20
wheel-speed window        200 ms
deadband compensation     8.0
absolute-angle correction 5 s
```

This version is stationary: commanded speed and turn are both zero. The next
application layer may replace those values with game-controller commands.
