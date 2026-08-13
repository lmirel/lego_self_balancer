# PS controller host app

This app will drive and steer the `0v2p2` self-balancing robot from a PlayStation
controller connected to the Mac. The selected mapping uses axis 2 for steering
and axis 1 for drive. The first phase only inspects controller input;
it cannot connect to or move the robot.

See [ROADMAP.md](ROADMAP.md) for the staged implementation and safety gates.

## Set up

Pair the controller in macOS Bluetooth settings, then use the project's virtual
environment and install its dependencies:

```bash
cd /Users/mirel/Work/lego/robot/self-balancing
source .venv/bin/activate
python -m pip install -r requirements.txt
```

The dependency is `pygame-ce`, which is imported as `pygame` and has a macOS
Python 3.14 wheel. Do not install both `pygame` and `pygame-ce` in this virtual
environment.

## Phase 1: verify the controller

List controllers:

```bash
python apps/ps_controller/controller_diag.py --list
```

Run the input diagnostic:

```bash
python apps/ps_controller/controller_diag.py
```

The selected PS4 control assignment uses axis 2 for steering and axis 1 for
drive. Other controller mappings can
differ. The diagnostic displays every raw axis as `axis:value`; move each chosen
control separately and confirm only axis 1 or axis 2 changes as expected.
Positive axis 2 should produce positive `turn`; positive axis 1 should produce
positive `drive`. Both processed values should
settle at `0.000` when released.

If macOS exposes different axes, override either mapping, for example:

```bash
python apps/ps_controller/controller_diag.py --drive-axis 1 --turn-axis 2
```

No hub should be connected during this diagnostic. Stop it with Ctrl-C.

## Phase 2: verify the BLE command link

This test downloads a dedicated receiver to the hub. It does not import or open
the motors, so joystick movement cannot drive the wheels. The hub validates
sequenced commands, reports the values it received, and resets both commands to
zero if traffic stops for 250 ms.

Keep the robot switched on with Pybricks firmware and the PS4 controller paired,
then run:

```bash
python apps/ps_controller/run_link.py
```

To prove the watchdog path, run a second time with an automatic 600 ms command
pause after two seconds:

```bash
python apps/ps_controller/run_link.py --watchdog-test
```

During the pause, expect `WATCHDOG,commands_zeroed` and `watchdog=1`; after the
pause, expect a new `LINK_ACTIVE` and `watchdog=0`. This automatic pause happens
once per invocation.

If needed, select a named hub:

```bash
python apps/ps_controller/run_link.py --name "Pybricks Hub"
```

Expected output includes:

```text
LINK_READY,watchdog_ms=250
LINK_ACTIVE,sequence=...
HOST_COMMAND,drive=...,turn=...
COMMAND_STATUS,sequence=...,drive=...,turn=...,age_ms=...,watchdog=0
```

Move both sticks and compare `HOST_COMMAND` with `COMMAND_STATUS`. Press the
hub's CENTER button or Ctrl-C to finish. CENTER, Ctrl-C, controller disconnect,
BLE disconnect, and the hub watchdog are independent stop paths.

This phase deliberately does not test motors. Phase 3 begins with the robot
supported and its wheels suspended.

## Phase 3: suspended-wheel steering polarity

Support the robot securely with **both wheels clear of the floor**. This is not
a balancing test. The receiver opens the motors but ignores right-stick drive,
uses a 30% breakaway command increasing proportionally to a 45% maximum, and
retains the 250 ms watchdog. Releasing the stick still stops both motors rather
than applying the breakaway offset.

Run only after the wheels are suspended:

```bash
python apps/ps_controller/run_steering.py --wheels-suspended
```

Move steering axis 2. Expected wheel directions when looking at the robot
in its normal forward orientation:

| Stick | Left wheel | Right wheel | Intended yaw |
|---|---|---|---|
| Right | Forward | Backward | Right |
| Left | Backward | Forward | Left |
| Released | Stopped | Stopped | None |

Moving drive axis 1 must not affect either motor. Press CENTER or Ctrl-C to
stop. Report actual wheel directions before the steering command is integrated
with the balance controller.

Validated result: positive/right steering produced left-wheel forward and
right-wheel backward motion. Standalone unloaded movement began around 30--34%
duty. The 30% breakaway offset belongs only to this polarity test; it must not be
copied into the balancing controller's steering path.

## 0v3: balance with drive and steering

This release uses the locked `0v2p2` stationary balance gains, a smooth steering
differential capped at 20% duty, and bounded drive speed capped at 300 wheel
degrees/second (roughly 14.7 cm/s with 56 mm wheels). The standalone 30%
breakaway offset is not used.
Drive does not command motor duty directly. It advances the position target at
the configured wheel speed. Returning drive to neutral freezes that target so
position feedback can brake and hold; the target is never jumped to the measured
wheel position. A 1000 degrees/second runaway cutoff stops the trial before the
normal angle cutoff if wheel speed becomes extreme.

Place the robot on the floor, hold it at its normal upright release pose, and run:

```bash
python apps/ps_controller/run_remote.py
```

Axes and response limits are runtime parameters:

```bash
python apps/ps_controller/run_remote.py \
  --drive-axis 1 \
  --turn-axis 2 \
  --max-drive-speed-dps 300 \
  --max-turn-duty 20
```

The development app accepts any positive finite drive reference limit and rejects
turn limits above 20 duty. Defaults remain axes `1`/`2`, drive `300`, and turn
`20`. The 600 degrees/second² reference ramp prevents an instant jump. Physical
safety remains bounded by the 1000 degrees/second runaway stop, coupled
high-speed/lean stop, and 12-degree fall stop. The standalone `apps/0v3` package
retains its 300 ceiling.

Run normally once after any hub-code change so the new program is downloaded.
For subsequent limit/axis comparisons, start the stored copy and send new runtime
values without compiling or downloading again:

```bash
python apps/ps_controller/run_remote.py \
  --use-stored-program \
  --max-drive-speed-dps 300 \
  --max-turn-duty 20
```

Use `--use-stored-program` only after a successful normal run of this app; it
starts whichever program is currently stored on the selected hub.

Hold the robot still at its natural upright pose. After 500 ms of accepted
stillness, the hub displays `R` and emits one 700 Hz readiness beep. Release it
at the beep with both sticks centered; there is no countdown.
First confirm ordinary neutral balancing. Test steering again, then use only a
small axis 1 drive deflection and return it to neutral. Confirm the robot moves,
slows, and holds near its release location. Test the opposite direction only
after that passes. Keep a hand ready to catch it. CENTER, Ctrl-C, Bluetooth stop,
controller loss, and hard-fall detection stop the program. A 250 ms command
timeout zeros both commands, freezes the position target, and continues
stationary balancing.
The host waits for `BALANCE_ACTIVE` before transmitting, so calibration cannot
fill the hub's stdin buffer.

Startup learns the held pose as upright; it does not require a predefined forward
lean. The relaxed detector accepts up to 5 degrees/second gyro rate and 3 degrees
of wheel movement per 20 ms sample, while rejecting more than 12 degrees of
cumulative wheel drift. For this hub mounting, it also requires pitch within 15
degrees of zero and balancing-axis roll within 8 degrees of 90 degrees. A
stationary robot lying on its back (observed near zero pitch and -4 degrees roll)
therefore cannot redefine that pose as upright. `CALIBRATION_WAIT` reports both
tilt axes and all three motion measurements.

Initial acceptance is deliberately modest: correct direction, controlled low
speed, no continued travel at neutral, and no regression in steering or balance.
Validation progressed through 60, 90, 180, and 300 degrees/second. At the locked
300 setting, measured speed settled mostly around 280--350 degrees/second after
an initial burst near 500. Peak observed duty was about 62%; sustained lean was
typically 8--10 degrees, with 11.25 degrees observed during release against the
12-degree fall cutoff. Combined drive and turning at the 20-duty limit worked.
This is the `0v3` checkpoint; further speed work must first improve braking and
lean margin rather than raise the hard limit.

### Rejected speed-feed-forward experiment

A post-`0v3` experiment changed speed feedback to
`measured_speed - factor * commanded_speed`. Although factor `1.0` reduced
sustained lean, it consistently degraded stability and reversal behavior. In the
final test, measured speed reached `-755 degrees/second` and triggered the
runaway cutoff. The parameter and its experimental governor were removed. This
app again uses the validated `SPEED_GAIN * measured_speed` law. Do not resume
this feed-forward direction without a different controller model and explicit
new evidence.

### Development-only maximum-speed search

Only the drive limit is varied; the validated controller law stays unchanged.
There is no arbitrary development configuration ceiling. A coupled safety cutoff stops if
measured wheel speed is at least 400 degrees/second while lean reaches 10.5
degrees, in addition to the existing 12-degree fall and 1000 degrees/second
runaway limits.

The development controller is physically validated through 700 degrees/second.
At 700 it passed forward-to-neutral, reverse-to-neutral, direct reversal, and
combined steering. The deliberately gentle high-speed braking requires extra
stopping distance. Reproduce the locked baseline with:

```bash
python apps/ps_controller/run_remote.py \
  --max-drive-speed-dps 700 \
  --max-turn-duty 20
```

Run without `--use-stored-program` once because removing the configuration
ceiling requires downloading the updated hub code.

Testing established that 350 degrees/second was safe and steady 400-degree/second
travel was also healthy, but direct reversal at 400 failed because the position
reference changed direction while measured speed was still +425 degrees/second.
The development controller now freezes the reference during an opposing command
until measured speed falls below 60 degrees/second. `reversal_blocked=1` in
telemetry identifies this braking phase. This is an interlock around the locked
controller law, not the rejected speed-feed-forward or governor approach.

The first 450-degree/second test showed the interlock releasing correctly near
zero measured speed, but then applying the full opposite reference too abruptly:
measured speed overshot to 715 and then 752.5 degrees/second. The position
reference now ramps into any nonzero command at 600 degrees/second². Neutral
still freezes immediately for maximum braking; the ramp changes only entry into
motion and the post-interlock transition. Telemetry reports
`reference_speed_dps` separately from the raw stick request.

At 500, measured speed stayed mostly around 430--550 degrees/second with a brief
585 peak; lean stayed around 0.4--3.3 degrees and duty below about 68%, including
full turning.

At a 600-degree/second reference, stable straight travel briefly reached 770
degrees/second with only about 1 degree lean, so the development runaway cutoff
was raised from 750 to 1000. A high-speed reversal separately reached 10.52
degrees lean and correctly triggered the coupled angle cutoff. To begin braking
earlier as the stick is released, reference speed now reduces at 1800
degrees/second² while acceleration remains 600 degrees/second². The physical
angle cutoffs are unchanged.

The first 700-degree/second tests exposed that neutral, watchdog, and reversal
interlock still bypassed the deceleration rate by setting reference speed to zero
immediately. This froze the position target while measured speed remained
roughly 600--750 degrees/second, producing excessive braking lean; one watchdog
transition was followed by a 1607.5 degrees/second speed sample. These paths now
use a stopping trajectory: reference speed ramps toward zero at 1800
degrees/second² and the position target continues integrating that diminishing
speed. Reversal releases only when measured speed is below 60 and reference
speed below 15 degrees/second. Thus higher speed automatically receives more
stopping time and distance without weakening the angle safety cutoffs.

At 700, a fixed 1800 degrees/second² stop was still too aggressive: while
measured speed remained about 718--783 degrees/second, reference speed fell from
-628 to -268 in 200 ms and lean grew through 8.14 to 10.78 degrees. Deceleration
is now continuously speed-adaptive. The initial 600 degrees/second² high-speed
endpoint was still too aggressive relative to the robot's momentum, so the
current curve falls from 1800 degrees/second² near rest to 200 degrees/second²
at 700 degrees/second measured speed. Telemetry reports the active
`reference_decel_dps2`.

An attempted measured-speed stopping-position calculation was rejected: updating
that endpoint continuously created a moving target and immediate backward
runaway from neutral. The controller retains only the adaptive reference-speed
deceleration. Subsequent trials passed at 500 and 700 degrees/second, including
neutral stops, direct reversal, and steering. This is the `0v3.1` development
baseline; future lean-aware braking work starts after this checkpoint.
