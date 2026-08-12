# LEGO 51515 Self-Balancing Robot — Pybricks Tuning Project

## 1. Purpose

Build a small, reproducible Python-based control and tuning system for a LEGO MINDSTORMS Robot Inventor 51515 self-balancing two-wheel robot.

The current block-based PID tuning process is too slow and subjective. The goal is to move the real-time balance controller onto the hub using Pybricks and use a Mac-side Python tool to configure trials, collect telemetry, score results, and eventually automate gain tuning.

The project must be incremental: each roadmap phase must produce something directly testable and useful before adding more automation.

## 2. Hardware

Target hub:

- LEGO MINDSTORMS Robot Inventor 51515 hub / Prime-class hub supported by Pybricks.
- Two driven wheels, one motor per wheel.
- Current physical build is a tall inverted-pendulum robot with the hub above the axle.
- Existing LEGO program uses the hub IMU roll/tilt angle as the balance measurement.

Motor ports and polarity must be configurable, not hard-coded assumptions. Initial observed configuration should be captured in `config.toml` after the first hardware-identification test.

## 3. High-level architecture

The balancing loop MUST run locally on the LEGO hub. Bluetooth must not be in the real-time feedback path.

```text
Mac
┌──────────────────────────────────────────────┐
│ tune.py / CLI                               │
│                                              │
│ - configure trials                          │
│ - upload/start hub program                  │
│ - receive telemetry                         │
│ - score trials                              │
│ - suggest or automatically test gains       │
│ - save CSV/JSON results                     │
└───────────────────┬──────────────────────────┘
                    │ Bluetooth / Pybricks tooling
                    │ non-real-time control + telemetry only
┌───────────────────▼──────────────────────────┐
│ LEGO 51515 Hub                               │
│                                              │
│ hub/balancer.py                              │
│ - IMU sampling                               │
│ - fixed-period balance loop                  │
│ - P / PD / PID control                       │
│ - direct motor duty-cycle output             │
│ - fall detection / safety stop               │
│ - telemetry                                  │
└──────────────────────────────────────────────┘
```

## 4. Development environment on macOS

All host-side Python dependencies MUST be isolated from the system/Homebrew Python packages.

Use a project-local virtual environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install pybricksdev pybricks
```

Additional dependencies may be added only when actually required. Likely host dependencies later include:

```text
pybricksdev
pybricks              # API stubs/type assistance; firmware implements the real API
pytest
```

Do not globally `pip install` Pybricks tooling.

Commit a dependency lock/record appropriate to the chosen workflow, preferably `requirements.txt` initially for simplicity:

```bash
python -m pip freeze > requirements.txt
```

A fresh clone must be bootstrappable with a documented sequence such as:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Codex should not introduce Poetry, uv, Conda, Docker, or another environment manager unless there is a concrete need. Keep the project simple.

## 5. Repository layout

Initial target layout:

```text
lego-balancer/
├── spec.md
├── README.md
├── requirements.txt
├── .gitignore
├── config.toml
├── hub/
│   ├── balancer.py
│   └── hardware_test.py
├── host/
│   ├── tune.py
│   ├── transport.py
│   ├── scoring.py
│   └── analyse.py
├── tests/
│   ├── test_scoring.py
│   └── test_protocol.py
└── results/
    └── .gitkeep
```

Do not create every module up front merely to satisfy this tree. Add modules when the roadmap reaches them.

## 6. Real-time controller requirements

### 6.1 Control-loop location

The feedback loop MUST execute on the hub.

The Mac must never perform:

```text
read angle over BLE -> calculate PID -> send motor command over BLE
```

Bluetooth timing and OS scheduling must not affect balance stability.

### 6.2 Motor command

Use Pybricks regulated per-motor speed control (`Motor.run`) for balancing. The
firmware-level motor loop should overcome drivetrain friction; do not reproduce
that loop with discontinuous application-level duty compensation.

Pybricks exposes motor duty-cycle control using `Motor.dc(duty)`, with duty constrained to approximately `-100..100`.

The controller output must therefore be clamped:

```python
output = max(-100.0, min(100.0, output))
```

Both motors should receive an equivalent forward/backward balancing command after configurable motor polarity is applied.

### 6.3 IMU inputs

Use the Pybricks hub IMU.

The first implementation should expose and log both:

- tilt/roll angle in degrees;
- angular velocity around the relevant balance axis in degrees/second.

Pybricks provides IMU tilt/orientation functions and direct angular-velocity measurement. Prefer measured gyro angular velocity for the D term instead of estimating derivative solely from successive angle samples, once axis/sign have been validated.

### 6.4 Controller forms

Support these modes explicitly:

```text
P
PD
PID
```

Initial controller:

```text
error = target_angle - measured_angle
output = Kp * error
```

Preferred PD controller after gyro-axis validation:

```text
error = target_angle - measured_angle
output = Kp * error - Kd * angular_velocity
```

The sign of the D term must be derived from the chosen coordinate convention and verified experimentally. Do not assume the example sign is correct without a hardware sign test.

Fallback derivative implementation:

```text
d_error = (error - previous_error) / dt
output = Kp * error + Kd * d_error
```

This fallback may be used for comparison, but gyro angular velocity is preferred.

Integral control must be disabled by default.

When enabled:

```text
integral += error * dt
```

It MUST include anti-windup, e.g. an integral clamp and/or conditional integration while the output is saturated.

### 6.5 Loop timing

Use an explicit target loop period and measure actual elapsed time.

Initial target:

```text
100 Hz / 10 ms
```

Do not assume the hub will hit this exactly. Record actual `dt` and report:

- average loop period;
- minimum/maximum period;
- missed/late loop count if practical.

If 100 Hz is not sustainable, choose the fastest stable repeatable rate demonstrated by measurement.

## 7. Safety

Safety is mandatory before automatic tuning.

The hub controller must immediately stop motor drive when any of the following occurs:

- absolute angle error exceeds configurable fall threshold;
- requested test duration expires;
- host requests stop;
- program encounters an exception;
- controller is manually aborted using a suitable hub button mechanism where available.

Initial suggested fall threshold:

```text
10-15 degrees from target
```

This value is configurable and must be conservative during early testing.

On fall/abort:

```text
motor_left.stop() / brake()
motor_right.stop() / brake()
```

The choice between coast and brake should be tested and documented; safety and avoiding drivetrain shock take precedence over theoretical preference.

Automatic tuning MUST NOT run unattended. The user is expected to physically
catch/reposition the robot between trials. Repeated trials may use the validated
semi-automatic launch mechanism in section 14, but never remove the user from the
physical safety loop.

## 8. Hardware-validation program

Before writing PID tuning logic, implement `hub/hardware_test.py`.

It must establish:

1. left/right motor ports;
2. motor positive direction;
3. which IMU tilt component corresponds to forward/backward lean;
4. which IMU angular-velocity axis corresponds to that same motion;
5. sign convention:
   - lean forward angle sign;
   - forward gyro-rate sign;
   - motor duty sign that drives wheels forward;
6. approximate true upright angle.

The program should print clear measurements while the user manually moves the robot.

No PID code should be added until these signs are known.

## 9. Configuration

Use a human-readable file such as `config.toml` for hardware and safety parameters.

Example shape:

```toml
[hardware]
left_motor_port = "E"
right_motor_port = "A"
left_motor_sign = 1
right_motor_sign = -1
balance_axis = "X"
angle_component = "roll"
angle_sign = 1
gyro_sign = 1

[controller]
target_angle_deg = 88.95
kp = 15.0
ki = 0.0
kd = 0.0
loop_period_ms = 10
output_limit = 1000.0  # regulated wheel speed, degrees/second
integral_limit = 20.0

[safety]
fall_angle_deg = 12.0
trial_duration_s = 5.0
```

The exact ports and signs above are placeholders until hardware validation confirms them.

## 10. Telemetry

Each sample should ideally contain:

```text
timestamp_ms
trial_id
angle_deg
angle_error_deg
angular_velocity_dps
P_term
I_term
D_term
output_command
left_motor_speed_dps   # optional initially
right_motor_speed_dps  # optional initially
loop_dt_ms
state
```

Possible states:

```text
WAITING
RUNNING
FALLEN
COMPLETE
ABORTED
```

Do not require every telemetry field in Phase 1. Angle, gyro rate, output, state and time are sufficient initially.

Telemetry should be low-overhead and must not materially disturb the balance loop. If printing every loop harms timing, decimate telemetry, e.g. controller at 100 Hz and telemetry at 20-50 Hz.

## 11. Host/hub communication

Use supported Pybricks/pybricksdev BLE mechanisms rather than implementing the LEGO Bluetooth protocol from scratch.

The first host integration may use `pybricksdev` to upload/run the hub program and capture standard output if this is reliable enough.

A richer bidirectional protocol may be introduced later if needed.

Desired logical commands eventually include:

```text
PING
GET_STATUS
SET_GAINS kp ki kd
SET_TARGET angle
START_TRIAL trial_id duration
STOP
```

Desired hub events:

```text
READY
TRIAL_STARTED
TELEMETRY ...
TRIAL_COMPLETE ...
FALLEN ...
ERROR ...
```

Protocol must be versioned if/when a custom protocol is introduced.

Do not spend substantial time designing the protocol before basic upload/run/telemetry has been proven.

## 12. Trial scoring

Once telemetry exists, every trial can be assigned a score.

Primary objective: remain upright.

Useful metrics:

- time before fall;
- RMS angle error;
- maximum absolute angle error;
- RMS motor command;
- percentage of time output is saturated;
- number/amplitude of zero crossings or oscillations;
- settling behaviour near target;
- loop timing quality.

A first scoring function can be deliberately simple:

```text
score =
    + large reward for survival time
    - RMS angle error penalty
    - output saturation penalty
```

Do not over-engineer the score before real telemetry exists.

Raw trial data must always be retained so scoring logic can be changed later without rerunning every test.

## 13. Automatic tuning strategy

Automatic tuning is NOT Phase 1.

Once safe repeatable trials and telemetry work, implement conservative search rather than an opaque optimizer.

Suggested sequence:

### P search

- `Ki = 0`, `Kd = 0`.
- Search Kp over bounded candidates.
- Determine the region where the robot begins reliably arresting a small fall without immediate violent saturation.
- If the best candidate is the upper boundary but still does not balance,
  extend the bounded P range upward before introducing D or I. Require a
  visibly jittery/recovering P controller first; stop at violent response or
  repeated saturation.
- Express controller output as requested wheel speed in degrees/second and let
  the regulated motor API handle static and moving drivetrain friction.

### PD search

- Freeze Kp near the useful P region.
- Search Kd over a bounded range.
- Reward longer survival and reduced RMS angle error/oscillation.
- Include `Kd = 0` as the P-only baseline. Do not select a derivative term just
  because it is the least bad nonzero candidate; it must beat the baseline.
- Low-pass filter gyro input lightly before applying Kd, and retain raw plus
  filtered gyro values in telemetry. Keep cutoff high enough to avoid excessive
  phase delay; approximately 10 Hz at the 100 Hz controller rate is the initial
  bounded choice.
- When the observed motor lag and overshoot make rate-feedback phase uncertain,
  explicitly compare equal positive and negative Kd magnitudes against Kd=0.
  Do not infer the stabilizing sign solely from static sensor/motor conventions.

### Target-angle refinement

- Once PD can balance for several seconds, vary target angle in small increments (e.g. 0.05-0.1 deg) to reduce persistent directional drift.
- Require at least 3 seconds confirmed median survival before this stage. If the
  gate is missed, stop and revise the controller rather than tuning target angle.
- Exception: when every derivative candidate fails and repeated trials show a
  consistent directional bias, permit a bounded early target diagnostic using
  the locked Kp and `Kd=0`. Test offsets on both sides of the baseline rather
  than inferring adjustment direction from signed mean error alone, because
  target and body dynamics are coupled. Confirm its top two candidates before locking the
  diagnostic target, then restart Kd; do not proceed to Ki from this exception.
  Rank this diagnostic using survival, RMS error, and directional bias as well
  as recovery. Do not let the binary gain-search recovery penalty alone choose
  a target with demonstrably worse bias.
  Require the confirmed diagnostic winner to beat the confirmed original target
  baseline. Otherwise retain the baseline and stop; do not cycle back through
  Kd without new controller evidence.

### Integral tuning

- Only add Ki if a persistent bias remains that cannot reasonably be corrected by target-angle adjustment.
- Ki should be very small, bounded, and protected by anti-windup.

### Wheel-speed damping fallback

If confirmed angle-only control remains below the survival gate, add one compact
translation-damping term before increasing controller complexity. Sign-correct
the mirrored motor encoder speeds, average them, and subtract `Kw` times that
physical wheel speed from requested regulated wheel speed. Include `Kw=0` as
the confirmed baseline and tune only this new term while Kp, Kd, and target are
fixed.

If wheel-speed damping loses to its zero baseline, a bounded wheel-position
fallback may be tested. Define signed average displacement relative to both
encoders at trial start and subtract `Kx` times that displacement from requested
wheel speed. This should permit rapid catch motion while gently discouraging
unbounded translation. Tune only Kx against the same zero-feedback baseline.

Do not use machine learning, reinforcement learning, Bayesian optimization, or heavyweight optimization libraries in the initial project.

A claimed working visual-program controller may be translated as an isolated
reference experiment. Preserve its discrete integral and derivative equations,
target, gains, and final percent scaling; explicitly map percent output to the
selected Pybricks motor API units and translate signs using validated hardware
conventions. Do not merge reference results into staged tuning automatically.

An official Pybricks balancer may likewise be adapted as an isolated reference.
Preserve its loop period, gyro integration, encoder speed window, coupled state
feedback gains, raw-duty actuation, and voltage compensation. Adapt only known
hardware properties such as ports, polarity, and IMU axis; remove unrelated
steering/travel features and retain local safety stops.

A bounded grid/coarse-to-fine search is preferred because its behaviour is understandable and debuggable.

Each parameter must pass a confirmation gate before the next parameter is
tuned. Explore each coarse/refined candidate once, then run enough additional
trials to give the top two candidates three trials each. Select using median
score so one unusually good or bad hand release cannot determine the winner.
Lock the confirmed winner for all subsequent stages; later noisy results must
not reopen an earlier parameter stage. Revisit a locked stage only through an
explicitly started new search, not by oscillating automatically between gains.
Confirmation also requires demonstrated recovery, not merely longer survival or
wheel movement. After an excursion beyond 2 degrees, the robot must return
within 1.5 degrees in at least two of the winning candidate's three trials.
Oscillation is measured separately and is expected during P-only exploration;
Kd tuning should reduce it. If recovery does not occur, stop before locking
the parameter and correct the controller or drivetrain model first.

## 14. Human-in-the-loop trial workflow

The initial automatic tuner should assume this physical workflow:

```text
1. Host selects candidate gains.
2. Hub reports READY.
3. User positions robot approximately upright.
4. User explicitly starts the trial (keyboard or hub button).
5. Balance controller runs for N seconds or until fall.
6. Motors stop.
7. Host records/scorers result.
8. User repositions robot.
9. In manual mode, the next trial begins only after explicit confirmation.
   In semi-automatic mode, stable-upright detection and a cancellable 3-2-1
   countdown provide the per-trial launch gate after one bounded-session approval.
```

Default/manual workflows must not automatically launch repeated physical trials
without user confirmation. A bounded semi-automatic session is permitted after
one explicit initial authorization when all of the following are active: local
predictive fall shutdown, a hard trial limit, emergency stop, motors-off recovery,
stable-upright detection, an audible/visible countdown that cancels on movement,
and termination after repeated unsafe outcomes. The user still physically catches
and repositions the robot between trials.

Predictive fall detection must not trigger from an isolated high gyro reading
while the robot is near upright. It is eligible only after actual error reaches
3 degrees and the measured rotation is carrying the robot farther from upright;
the independent hard angle limit remains active.

## 15. Outputs

Store each tuning session under `results/`.

Example:

```text
results/
└── 2026-08-11_170212/
    ├── session.json
    ├── trials.csv
    ├── telemetry.csv
    └── recommendation.json
```

`recommendation.json` example:

```json
{
  "kp": 15.8,
  "ki": 0.0,
  "kd": 1.7,
  "target_angle_deg": 88.90,
  "evidence": {
    "best_trial_seconds": 8.4,
    "rms_angle_error_deg": 0.72
  }
}
```

Values shown are examples only.

## 16. CLI goals

Eventually provide a simple host CLI such as:

```bash
python -m host.tune check
python -m host.tune run --kp 15 --kd 0
python -m host.tune sweep-p --min 10 --max 25 --step 1
python -m host.tune sweep-pd
python -m host.analyse results/<session>
```

Do not implement all commands before the roadmap calls for them.

## 17. Roadmap

### Phase 0 — Environment and firmware

Exit criterion: Mac can use the isolated `.venv`, Pybricks firmware is installed, and a trivial hub Python program can be uploaded/run reliably.

Tasks:

- create repository and `.venv` workflow;
- install/pin Pybricks host tooling;
- document Pybricks firmware install and LEGO firmware restoration procedure;
- run a hello-world/hub-light program;
- confirm Mac BLE workflow.

### Phase 1 — Hardware discovery

Exit criterion: motor ports/polarity, balance angle axis, gyro axis/sign, and upright reference are known and recorded.

Deliverable:

```text
hub/hardware_test.py
config.toml
```

### Phase 2 — Minimal P controller

Exit criterion: a local hub loop reads angle and commands both motors using direct duty cycle, with safe fall shutdown and basic telemetry.

No host-side tuning automation yet.

Initial known empirical reference from the LEGO-block implementation:

```text
power-style controller around effective P gain ~15
Ki = 0
Kd = 0
```

This is only a starting reference; Pybricks direct duty control will have a different gain scale.

### Phase 3 — PD controller using gyro rate

Exit criterion: controller supports configurable Kp/Kd and measured gyro angular velocity; the robot can be manually tuned enough to spend meaningful time near upright.

Do not add I yet.

### Phase 4 — Reliable host telemetry

Exit criterion: Mac can launch a trial and store timestamped telemetry/results without affecting real-time hub control.

### Phase 5 — Trial scoring

Exit criterion: a recorded trial receives reproducible metrics and score, and the score can distinguish clearly good/bad runs.

### Phase 6 — Assisted tuner

Exit criterion: host proposes the next Kp/Kd candidate, waits for user confirmation, runs one test, scores it, and proposes the next candidate.

This is the first genuinely useful automatic-tuning milestone.

### Phase 7 — Automated bounded search

Exit criterion: user can initiate a tuning session and, while manually repositioning/confirming each trial, the software performs a coarse-to-fine Kp/Kd search, confirms each selected parameter from three trials using median score, locks completed stages, and produces a recommendation.

### Phase 8 — Target angle and optional I

Exit criterion: persistent drift can be minimized through target-angle optimization; Ki is added only if evidence shows it is useful.

### Phase 9 — Quality-of-life improvements

Optional:

- live terminal plot;
- CSV/PNG charts;
- saved named robot profiles;
- battery-voltage correlation;
- automatic identification of oscillation frequency;
- alternate D implementation comparison;
- motor speed feedback analysis.

These are explicitly secondary to achieving a stable controller.

## 18. Non-goals

Do not initially:

- create a web UI;
- use Docker for Mac Bluetooth access;
- control the real-time PID loop from the Mac;
- implement BLE from scratch;
- use reinforcement learning;
- create a generic robotics framework;
- build an elaborate schema/documentation system before runnable code exists;
- optimize steering, navigation, or remote driving before standing balance works;
- spend hours writing architecture without a working incremental artifact.

## 19. Engineering rules for Codex

1. Work incrementally and stop at each roadmap exit criterion.
2. Every phase must leave runnable/testable code.
3. Do not silently alter hardware assumptions; put them in config.
4. Never run motors without a fall/abort mechanism once control code is introduced.
5. Keep the hub feedback loop independent of BLE.
6. Prefer direct measurements and logs over visual guesses.
7. Preserve raw telemetry.
8. Tune one dimension at a time until automated search is introduced.
9. Avoid premature abstractions.
10. Update `README.md` with exact commands that were demonstrated to work.
11. If Pybricks API behaviour differs from this spec, verify against current official Pybricks documentation and adapt minimally rather than designing around assumptions.
12. Do not proceed to the next phase if the current phase's exit criterion is not demonstrated on hardware.

## 20. Immediate first task for Codex

Implement only Phase 0 and Phase 1.

The first useful checkpoint should be:

```text
$ source .venv/bin/activate
$ python ...
```

followed by a reliable way to run `hub/hardware_test.py` on the 51515 hub and observe/log:

```text
roll/pitch
angular velocity X/Y/Z
left/right motor direction
```

At that point, stop and review the measured axis/polarity results before implementing the balancing controller.
