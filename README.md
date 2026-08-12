# LEGO 51515 self-balancing robot

## Working baseline: 0v1

Version `0v1` is the first physically validated self-balancing baseline. It is
an adaptation of the official Pybricks state-feedback balancer rather than a
classical PID controller. On the tested Robot Inventor build it completed
multiple 15-second and 30-second unsupported trials and recovered from repeated
gentle nudges.

The locked controller in `hub/official_reference.py` uses:

- a 5 ms (200 Hz) raw motor-duty loop;
- relative tilt obtained by integrating bias-corrected gyro X rate;
- angle, wheel-position, and 300 ms wheel-speed feedback;
- gains `rate=0.018`, `angle=19.0`, `position=0.45`, `speed=0.176`;
- motor A on the left with sign `-1` and motor E on the right with sign `+1`;
- stationary gyro-bias calibration and a three-second arming countdown;
- a 12-degree hard fall stop, CENTER abort, Bluetooth firmware stop, and a
  30-second trial limit.

Run one catch-ready baseline trial from the repository root:

```bash
source .venv/bin/activate
python -m host.tune official-reference
```

Hold the robot motionless at its natural balance pose during calibration and
keep a hand ready to catch it. Trial output is recorded under `results/`, which
is intentionally excluded from source control.

The repository also retains the earlier phased PID experiments and hardware
identification tooling as development history.

## Mac setup

Use the project-local virtual environment; do not install these packages into
the system or Homebrew Python:

```bash
python3 --version  # must be Python 3.10 or newer
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

The checked-in lock was produced on Apple Silicon macOS with Python 3.14.5.
Current `pybricksdev` requires Python 3.10 or newer. In particular, do not build
the environment with Apple's `/usr/bin/python3` version 3.9: pip will silently
select an obsolete prerelease of `pybricksdev` which cannot communicate with
current hub firmware.

On this development Mac, Homebrew Python also needs its Homebrew Expat library
made explicit due to a local dynamic-linker mismatch. If Python reports a
`pyexpat` missing-symbol error, run this before creating/using the environment:

```bash
export DYLD_LIBRARY_PATH=/opt/homebrew/opt/expat/lib
```

## Install or restore hub firmware

The 51515 Inventor Hub and SPIKE Prime Hub use the same Pybricks firmware.

1. Open [Pybricks Code](https://code.pybricks.com/) in Chrome or Edge on the Mac.
2. Open **Tools → Install Pybricks Firmware**.
3. Select the Prime/Inventor hub, connect it by micro-USB, and follow the five
   on-screen steps, including entering firmware-update mode.
4. To return to LEGO firmware later, use **Tools → Restore official LEGO
   firmware** in the same app and follow its prompts.

These are the current official installation and recovery paths. Firmware
replacement can erase Pybricks programs/settings stored on the hub.

## Phase 0 smoke test

Turn on the hub and enable Bluetooth advertising. From the repository root:

```bash
source .venv/bin/activate
cd hub
pybricksdev run ble hello.py
```

If more than one hub is nearby, use its firmware-install name:

```bash
cd hub
pybricksdev run ble --name YOUR_HUB_NAME hello.py
```

Success is a green status light for one second and `HELLO,pybricks` in the Mac
terminal. The CLI remains attached until the program ends, so printed telemetry
is captured on standard output.

## Phase 1 hardware discovery

Before running this test, support the robot securely so both wheels are clear
of the floor, hands, hair, and loose parts. The program only uses a 30% duty,
500 ms pulse, one motor at a time. The CENTER button remains the immediate abort.
Each pulse logs the motor encoder's before/after angles and delta, so movement
can be confirmed even if it is difficult to see or feel.

Check `LEFT_MOTOR_PORT` and `RIGHT_MOTOR_PORT` at the top of
`hub/hardware_test.py`, then run:

```bash
source .venv/bin/activate
cd hub
pybricksdev run ble hardware_test.py | tee ../hardware-test.log
```

During the run:

1. Hold the chassis at its approximate true upright pose and record pitch/roll.
2. Lean it forward, then backward. Identify the changing tilt component and its
   sign; identify the matching X/Y/Z gyro rate and sign.
3. With wheels safely suspended, tap the hub LEFT button once. Observe which
   wheel moved and whether positive duty drives that wheel forward.
4. Tap RIGHT and record the same facts for the other configured motor.
5. Press CENTER to abort, then copy the measured ports, signs, balance axis,
   angle component, and upright angle into `config.toml`.
6. Set `hardware_validated = true` only after every value is confirmed.

If startup reports a device/port error, no motor is driven: correct the two port
constants and rerun. Do not proceed to a balancing controller until the Phase 1
measurements have been reviewed.

The terminal log is raw evidence. Keep it outside `results/` for now; structured
tuning sessions begin in a later roadmap phase.

## What remains hardware-dependent

Phase 0 and Phase 1 were demonstrated on the physical robot on 2026-08-11. The
measured configuration is motor A on the left (raw positive is backward), motor
E on the right (raw positive is forward), roll as the balance angle, X as the
matching gyro axis, and positive roll/gyro motion when leaning forward. The
measured upright reference is 87.14 degrees. These values are recorded in
`config.toml`; re-run hardware discovery after changing the physical build.

## Phase 2 minimal P controller

The controller runs entirely on the hub at a target period of 10 ms. Bluetooth
only receives decimated telemetry and is not part of the feedback path. The
commissioning gain is `Kp = 5.0`. A prior `Kp = 3.0` trial reached only 21%
duty and produced no meaningful encoder movement; the separate hardware test
demonstrated that approximately 30% duty overcomes this drivetrain's deadband.

Secure the robot where a fall cannot damage it or nearby objects. Keep hands,
hair, cables, and loose parts away from the wheels. Then run:

```bash
export DYLD_LIBRARY_PATH=/opt/homebrew/opt/expat/lib
source .venv/bin/activate
cd hub
PYTHONUNBUFFERED=1 pybricksdev run ble balancer.py | tee ../phase2-p.log
```

The motors remain stopped in `WAITING`. Hold the robot within 1 degree of the
87.14-degree target, press CENTER once, and release it to begin one five-second
trial. Starts outside this window are rejected without driving the motors. Adjust
the pose and press CENTER again. Press CENTER a second
time to abort normally. The Bluetooth button is an independent firmware-level
emergency stop if the Python loop becomes unresponsive. The motors also stop
when the angle error exceeds 12 degrees, the trial expires, or an exception
occurs.

For the first test, introduce only a very small forward lean. Both wheels must
drive forward: raw negative duty on left motor A and raw positive duty on right
motor E. If either direction is wrong, press CENTER again and return to Phase 1; do not
increase Kp. A successful commissioning run should contain `TRIAL_STARTED`,
telemetry rows, `FALLEN` or `TRIAL_COMPLETE`, a `TIMING` summary, and `STOPPED`.
Telemetry includes both motor encoder angles so commanded output can be compared
with actual wheel movement during commissioning.

Phase 2 was demonstrated on 2026-08-11 with `Kp = 5.0`. A five-second supported
trial completed without a safety fault. Left motor A moved from -73 to -330
degrees and right motor E moved from -7 to 306 degrees, confirming equivalent
forward wheel drive after polarity correction. Loop timing averaged 10.000 ms
(9--11 ms observed) with zero late iterations.

## Phase 3 gyro-based PD controller

`hub/balancer.py` supports explicit `P` and `PD` modes. The current Phase 3
candidate is `Kp = 7.5`, `Kd = 0.2`. A supported test at `Kp = 5.0` physically
validated that positive gyro X adds wheel drive in the forward catch direction,
but its peak 29% output was still near the measured drivetrain deadband. The next
unsupported trial at `Kp = 7.5`, `Kd = 0.2` lasted 3.25 seconds before a
controlled forward fall. Raising Kp to 10.0 reduced survival to 2.89 seconds and
caused output saturation, so Kp was returned to 7.5 and the bounded D search
began. The bounded comparison gave survival times of 2.90 seconds at Kd 0.0,
3.25 seconds at Kd 0.2, and 2.68 seconds at Kd 0.5. Kd 0.2 is restored for a
confirmation trial. The D term uses
the measured X-axis angular velocity directly; it does not estimate a derivative
from successive tilt samples. Integral control remained disabled during Phase 3;
the later staged search can enable bounded PID with conditional anti-windup only
after target-angle refinement leaves measurable bias.

Run it with the same safety setup and command used for Phase 2. The first CENTER
press/release starts; the second CENTER press aborts. For the first PD trial,
support the robot and introduce a small forward rotation. Positive gyro X must
produce a positive D term and additional forward wheel command. If it instead
amplifies the fall or drives opposite to the motion needed to catch the robot,
abort immediately and review the sign before changing either gain.

The telemetry now separates `p_term`, `d_term`, and clamped `output`. Do not tune
`Kd` or attempt unsupported balancing until this derivative sign test has been
reviewed from the physical response and recorded telemetry.

`PYTHONUNBUFFERED=1` is important when piping through `tee`: it makes READY,
arming rejection, trial state, and telemetry appear in the terminal immediately
instead of only after the process exits.

Phase 3 was demonstrated on 2026-08-11 at `Kp = 7.5`, `Kd = 0.2`. Positive
gyro X was physically verified to add forward catch drive. Unsupported trials
from controlled launches remained active for 3.25 and 3.05 seconds before safe
fall shutdown. In the confirmation run, starts at +3.443 and +1.938 degrees were
rejected; a +0.973-degree start was accepted. It ran for 3.05 seconds, stopped at
+12.202 degrees, averaged a 10.000 ms loop period (8--12 ms observed), and had
zero late iterations. This satisfies the incremental Phase 3 checkpoint but is
not a claim of sustained stable balance.

## Phase 4 recorded host telemetry

Run and record one human-confirmed physical trial from the repository root:

```bash
source .venv/bin/activate
python -m host.tune run
```

Use `--name HUB_NAME` when multiple Pybricks hubs are advertising. Output is
shown live. Each invocation creates `results/YYYY-MM-DD_HHMMSS/` containing:

- `session.json`: timestamps, configuration snapshot, events, outcome, and row count;
- `config.toml`: the exact trial configuration;
- `hub-output.log`: unmodified CLI/hub output;
- `telemetry.csv`: parsed, analysis-ready telemetry rows.

The host launches and records the program but never calculates motor commands.
The 100 Hz controller remains entirely on the hub. CENTER still starts/aborts a
trial and the Bluetooth button remains the independent firmware stop.

Phase 4 was demonstrated on 2026-08-11 in
`results/2026-08-11_174758/`. The host rejected seven out-of-window starts,
captured a complete five-second trial with return code 0, stored 125 valid
telemetry rows spanning 0--4960 ms, preserved the raw output, and saved a TOML
snapshot identical to the configuration embedded in `session.json`. Timestamps
were unique and monotonic. Hub timing averaged 10.000 ms (9--11 ms observed)
with zero late iterations.

## Phase 5 trial scoring

Every new `python -m host.tune run` session is scored automatically and gains a
`metrics.json`. Score an existing Phase 4 session with:

```bash
python -m host.analyse results/2026-08-11_174758
```

Metrics include survival time, completion, RMS and maximum angle error, RMS
output, saturation fraction, angle-error zero crossings, and measured loop
timing. The initial transparent score is:

```text
100 * survival_seconds
- 10 * RMS_angle_error_degrees
- 100 * saturation_fraction
```

Survival deliberately dominates. Raw hub output and telemetry remain unchanged,
so this formula can be revised later without rerunning physical trials.

Phase 5 was demonstrated on 2026-08-11. The complete five-second Phase 4 session
scored 468.634 with 3.137 degrees RMS error and no saturation. Applying the same
function to the real 3.05-second fallen Phase 3 trace scored 245.167 with 5.983
degrees RMS error. The score therefore ranks the clearly better physical run
higher while preserving both raw datasets for future rescoring.

## Phase 6 assisted tuner

Run one assisted tuning step from the repository root:

```bash
source .venv/bin/activate
python -m host.tune assist
```

The assistant proposes one nearby bounded Kp/Kd candidate and shows the current
and proposed values. It launches nothing unless the user types exactly `Y`.
That confirmation starts the normal hub workflow; CENTER is still required to
arm the physical trial. After the single trial ends, it records and scores the
session and prints one next proposal without launching it.
The pending proposal is saved in `results/assist-state.json`, so a later
invocation continues the refinement instead of repeating the previous candidate.

Each candidate gets a session-local `hub-program.py` and matching `config.toml`.
The canonical `hub/balancer.py` and root configuration are not rewritten during
a tuning trial. For a manually chosen, still single-trial override, use:

```bash
python -m host.tune run --kp 7.5 --kd 0.25
```

No command in Phase 6 launches repeated trials.

Phase 6 was demonstrated on 2026-08-11 with three separately confirmed
`Kp = 7.5`, `Kd = 0.25` trials. All completed five seconds and scored 463.026,
465.667, and 466.609, below the 468.634 prior best at Kd 0.2. This exposed and
fixed a carry-forward bug: the pending midpoint recommendation is now persisted,
and the next invocation proposes Kd 0.225 instead of repeating Kd 0.25.

## Phase 7 semi-automatic bounded search

Start a bounded session with one initial authorization:

```bash
source .venv/bin/activate
python -m host.tune semi-auto --max-trials 5
```

After typing exactly `Y` once, each hub program keeps the motors off until the
robot is within 2.5 degrees of upright and below 5 degrees/second for 0.5 seconds.
The terminal prints live angle/error/gyro diagnostics every 0.5 seconds. The hub
then beeps and displays 3, 2, 1. During countdown, a wider 5-degree/30-degree-per-
second envelope permits normal hand release; exceeding it cancels the countdown
and automatically returns to waiting. No CENTER press is needed to retry. After
a trial completes or falls, the Mac scores it, computes the
next bounded candidate, uploads it, and waits for the robot to be repositioned.
If the pose drifts outside the original 2.5-degree arming tolerance by the first
telemetry sample, the host retains the trial but rejects it as tuning evidence
and retries the same candidate.

Fall shutdown remains local to the hub. In addition to the 12-degree hard limit,
the controller stops when the actual error is already at least 3 degrees, it is
rotating farther from upright, and angle plus 150 ms of measured gyro motion
projects beyond 10 degrees. This gating prevents near-upright gyro spikes from
being classified as falls. It also stops when output remains saturated for 250
ms. CENTER aborts a
running trial, the Bluetooth button remains the firmware emergency stop, and the
host stops after at most the requested trials, any manual/error termination, or
three consecutive unsafe trials. An unsafe trial means a fall in under 0.5
seconds or a trial with at least 25% saturated output; ordinary controlled falls
remain valid tuning evidence and do not stop the search.

The validated drivetrain needs approximately 30% raw duty to begin moving, so
the balance controller now requests regulated wheel speed with Pybricks
`Motor.run()` instead. The firmware motor loop supplies the duty needed to
overcome friction and permits smooth low-speed reversals without application-
level start kicks. Controller output and its limit are degrees/second. Scoring
requires physical recovery, not just elapsed time or motor
travel: after reaching 2 degrees error, the robot must return within 1.5 degrees.
A parameter cannot be locked unless this happens in at
least two of its three winning confirmation trials; otherwise the search stops.

The search is stored under `results/search_YYYY-MM-DD_HHMMSS/` with a
`search.json`, `recommendation.json`, and a complete Phase 4-style directory for
each trial. Repositioning is manual; trial arming, countdown, scoring, gain
selection, and relaunch are automatic.

The persisted methodology in `results/staged-search-state.json` is:

Every parameter stage first explores each candidate once, then confirms its top
two candidates to three trials each. Selection uses the median score, after
which that parameter is locked for all later stages. A completed stage is never
reopened by a later noisy result.

1. P-only regulated-speed candidates now extend upward from the confirmed
   boundary: reuse three `Kp = 160` trials, then test `200, 240, 320`
   degrees/second per degree (`Ki = Kd = Kw = Kx = 0`), followed by
   midpoint refinement around the best coarse result, then Kp confirmation.
2. Freeze Kp; test `Kd = 0, 0.5, 1, 2`, then refine between the best value
   and its neighbor(s), then Kd confirmation. Ki remains zero.
   Including zero makes the confirmed P-only controller the baseline: a nonzero
   Kd is selected only when its median evidence is better. The three confirmed
   winning P trials are reused because PD with `Kd=0` has the identical control
   law; one additional poor release cannot replace that baseline.
   The D term uses a first-order gyro low-pass filter with `alpha = 0.386` at
   100 Hz (about a 10 Hz cutoff). Telemetry records raw and filtered gyro rates
   separately so noise reduction and phase lag remain inspectable.
   If motor-response delay followed by overshoot persists, run an explicit sign
   diagnostic before assuming the convention: compare `Kd=-1` and `Kd=+1` at
   `Kp=160` against the three-trial `Kd=0` baseline, with all other terms zero.
   If all Kd candidates fail while trials show consistent directional bias, run
   an early target diagnostic first. It compares offsets on both sides of the
   baseline (currently `86.74`, `86.94`, `87.14`, `87.34`, and `87.54`) at
   `Kp=160, Kd=0`, confirms the
   top two, locks the target, and only then restarts Kd.
   Early target ranking considers survival, RMS error, and directional bias;
   the gain-search recovery penalty must not by itself exclude a target that
   repeatedly crosses upright and materially reduces bias.
   A diagnostic target must also beat the three-trial confirmed baseline median;
   otherwise retain the baseline and stop rather than restart Kd in a loop.
3. Freeze PD; test target offsets of -0.4, -0.2, 0, +0.2, and +0.4 degrees, then
   refine around the best target, then target confirmation.
   This stage begins only when confirmed PD median survival reaches 3 seconds;
   otherwise the search stops because target optimization would mask an unstable
   controller rather than refine a working one.
4. If the best target trial has at most 0.5 degrees mean error, finish with
   `Ki = 0`. Otherwise test only `Ki = 0.01, 0.02, 0.05` in PID mode with a
   clamped integral and conditional integration while saturated; the top two Ki
   values are confirmed before selection.

When angle-only tuning remains below the survival gate, the next compact
extension is average physical wheel-speed damping. Encoder speeds are corrected
for the mirrored left motor, averaged, and applied as
`speed_term = -Kw * wheel_speed_dps`. Search `Kw = 0, 0.1, 0.2, 0.4`; reuse the
three confirmed P trials as the `Kw=0` baseline, confirm the top two, and stop to
reassess before adding another controller term.

If speed damping loses to zero, test average wheel-position return instead. The
trial-start encoder positions define zero; mirrored motor signs are corrected,
then `position_term = -Kx * wheel_position_deg` gently pulls accumulated travel
back without directly opposing instantaneous catch speed. Search
`Kx = 0, 0.1, 0.25, 0.5`, reuse the confirmed zero-feedback baseline, and
confirm before retaining the term.

A bounded session continues the saved stage on the next invocation. To discard
staged evidence and intentionally restart from P-only search, use:

```bash
python -m host.tune semi-auto --max-trials 5 --reset-search
```

## Visual-program reference controller

An isolated command reproduces the supplied block algorithm without changing
or advancing staged-search state:

```bash
python -m host.tune reference
```

It uses target `88.95`, `Kp=5.5`, `Ki=2.1`, `Kd=4`, discrete
`integral += error * 0.25`, `derivative = error - lastError`, and final
`result * 2.5%`. Since Pybricks `Motor.run()` accepts degrees/second, ±100% is
mapped to the configured ±1000°/s limit. Angle and motor signs are translated to
the validated physical convention. The run is catch-ready, auto-arms with the
normal countdown, retains all safety stops, and does not contaminate tuning
evidence.

## Official Pybricks balancer reference

The official Pybricks Robot Inventor balancer is adapted in an isolated,
safety-wrapped command:

```bash
python -m host.tune official-reference
```

It preserves the official 5 ms loop, gyro-integrated relative angle, 300 ms
encoder speed window, raw duty, battery compensation, and coupled gains
`0.018*rate + 19*angle + 0.45*position + 0.16*speed`. Ports, motor polarity, and
IMU axis are adapted to this validated build; steering and commanded travel are
removed. The integrated angle and encoders are zeroed immediately after the
stable countdown. Existing fall/abort safeguards remain active, and the command
does not alter staged-search state.
