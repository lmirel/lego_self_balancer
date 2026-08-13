# PS controller roadmap

The game-controller feature is a host application. Pygame reads the controller,
the host sends compact commands over the existing Pybricks Bluetooth connection,
and the hub blends those commands into the proven `0v2p2` balance controller.

Neutral drive means **hold the current location target**. It does not mean keep
moving, and it does not return to the location where the robot was launched.

## Command path

```text
PS controller -> pygame -> filtering -> BLE stdin -> watchdog -> balancer
                  host                    hub
```

The intended update rates are 60 Hz controller sampling and 50 Hz BLE commands.
The protocol will be one newline-terminated command:

```text
C,<sequence>,<drive>,<turn>
```

`drive` and `turn` are normalized to `-1.0..1.0`. The sequence number allows the
hub to ignore stale packets. The hub will reset both commands to zero if no valid
packet arrives for 250 ms.

## Phase 1 — controller diagnostic

- Discover and display connected controllers.
- Validate the selected drive and steering axes.
- Apply a rescaled dead zone, response curve, low-pass filtering, and slew limit.
- Print raw and processed values without connecting to or moving the robot.

Acceptance: both commands rest at zero, reach both signs smoothly, and no other
control unexpectedly changes them.

Status: complete. The selected mapping is drive axis 1 and turn axis 2.

## Phase 2 — BLE command link

- Run the hub program and controller reader in one host process using
  `pybricksdev`.
- Send commands at 50 Hz without blocking telemetry or the 200 Hz balance loop.
- Add sequence validation and a 250 ms hub-side watchdog.
- Make controller loss, host exit, malformed input, CENTER, and a hard fall all
  independently command zero and stop safely.

Acceptance: with wheels suspended, commands arrive continuously, stale commands
become zero, and disconnecting the controller cannot leave either motor driven.

Status: complete. Hardware validation confirmed normal command delivery, a
deliberate 600 ms host pause, hub-side zeroing after 250 ms, and automatic link
recovery when valid sequenced commands resumed.

## Phase 3 — steering only

- Feed turn into differential motor duty while drive remains fixed at zero.
- Begin with a deliberately small turn limit and add a turn slew limiter.
- Verify motor polarity with the wheels suspended, then test balancing turns.

Acceptance: left/right commands yaw in the expected direction without degrading
neutral balancing or bypassing fall detection.

Status: suspended-wheel polarity validation complete. Positive/right steering
made the left wheel run forward and the right wheel backward; reliable unloaded
motion began around 30--34% standalone duty. This breakaway offset is test-only.
Balance integration is complete with a smooth differential capped at 20% duty
and no breakaway step. On-floor testing confirmed that steering works well with
the selected axis 2 mapping.

## Phase 4 — low-speed drive

- Map right-stick vertical input to commanded wheel speed, not direct motor duty.
- Integrate commanded speed into the balancer's position target.
- Freeze that target when the stick returns to neutral.
- Increase the speed limit only after repeatable forward, reverse, and stop tests.

Acceptance: the robot travels in the requested direction, stops near its release
point after neutral, and never continues because of a stale host command.

Status: the lean/follow experiment exposed positive speed feedback when position
error was abruptly removed: speed rose to 962 degrees/second and duty saturated.
The revised controller uses a continuous positive position ramp and freezes it at
neutral without any target jump. A 750 degrees/second runaway cutoff was added.
Bidirectional driving and neutral target freeze were then validated at 60
degrees/second. Peak duty was about 32%, wheel speed about 185 degrees/second,
and lean about 8.35 degrees. A subsequent 90 degrees/second test, including
combined full steering, stayed around 5.8 degrees peak lean and 44% peak duty.
Tests then progressed through 180 and 300 degrees/second with all safety cutoffs
unchanged.
Drive/turn axes and limits are now runtime host parameters. A compact limits
message is validated against the locked 300 degrees/second drive ceiling and
20 duty turn. After one normal download, `--use-stored-program` permits repeated
comparisons without recompiling the hub program.

## Phase 5 — combined control and lock

- Test simultaneous drive and steering, reversals, nudges, and controller loss.
- Tune response curve, limits, and slew rates independently of balance gains.
- Record the controller mapping and selected constants.
- Tag the first repeatable remotely driven build only after the safety matrix and
  repeated neutral-balance runs pass.

Acceptance: predictable low-speed control, reliable neutral hold, safe timeout,
and no regression from the `0v2p2` standalone balancer.

Status: locked as `0v3`. Defaults are drive axis 1, turn axis 2, 300
degrees/second maximum drive, and 20 duty maximum turn. At full drive, measured
speed settled mostly around 280--350 degrees/second after an initial burst near
500; peak duty was about 62%, sustained lean typically 8--10 degrees, and release
reached 11.25 degrees against the 12-degree fall cutoff. Further speed increases
are deferred until braking and lean margin improve.

## Rejected post-0v3 speed-feed-forward experiment

The experiment replaced feedback on total wheel speed with feedback on
`measured_speed - factor * commanded_speed`. Factor `1.0` reduced sustained lean
but consistently worsened stability, especially through reversals. Adding a
speed governor and reversal interlock did not make the approach safe: the final
test reached `-755 degrees/second` and hit the runaway cutoff. The parameter,
telemetry, and governor were removed, restoring the validated `0v3` speed law.
This direction is closed unless a substantially different controller model is
introduced.

## Development-only maximum-speed search

Keep the validated controller law and 300 degrees/second default unchanged. The
development app has no arbitrary reference configuration ceiling. Each candidate
must pass forward/neutral, reverse/neutral, direct
reversal, and combined turning before advancing. A coupled cutoff stops at 10.5
degrees lean when wheel speed is already at least 400 degrees/second; the
independent 12-degree fall and 1200 degrees/second runaway cutoffs remain. The
standalone `apps/0v3` release retains its 300 ceiling.

Results: 350 degrees/second passed. Sustained 400-degree/second travel also
tracked safely around 330--435 degrees/second and roughly 4.5--6.4 degrees lean,
but direct reversal began moving the reference backward while measured speed was
still +425 degrees/second and fell at -12.17 degrees. A minimal reversal
interlock now freezes the position reference until opposing measured speed drops
below 60 degrees/second. It does not modify the locked speed feedback law.

Startup experimentation also removed the 3--2--1 countdown. After the 500 ms
stable-pose window, one readiness beep marks the instant balancing begins and the
operator should release the robot. Calibration additionally requires pitch
within 15 degrees of zero and balancing roll within 8 degrees of 90 degrees for
the observed hub mounting; stillness alone cannot arm the controller or redefine
the robot's lying-down pose as upright.

The first 450-degree/second reversal confirmed that the interlock stopped the
old direction safely, but its release jumped immediately to about 445
degrees/second reference. Measured speed overshot to 752.5 and hit the runaway
cutoff. Nonzero reference speed now ramps at 600 degrees/second² after startup or
interlock release. Neutral remains an immediate reference freeze so braking is
not weakened.

With transition ramping, 500 degrees/second passed. Measured speed stayed mostly
around 430--550 degrees/second with a brief 585 peak; lean stayed around 0.4--3.3
degrees and duty below about 68%, including full turn. Testing subsequently
progressed through 600 to 700 while the physical 1200-degree/second runaway and
angle cutoffs remained.

At a 600-degree/second reference, straight travel reached 770 degrees/second at
only about 1 degree lean; the former 750 runaway cutoff was therefore raised to
1000 for development. The reverse-to-forward transition instead reached 10.52
degrees lean at -567.5 degrees/second and correctly hit the coupled angle cutoff.
Reference deceleration is now 1800 degrees/second² versus 600 acceleration so a
stick release begins braking earlier. Speed-dependent steering remains deferred.

At 700 degrees/second, abrupt reference freezes on neutral, watchdog, and the
reversal interlock still demanded excessive braking lean. These paths now follow
a decelerating reference trajectory instead: reference speed approaches zero at
1800 degrees/second² while position continues integrating it. Reversal releases
only below 60 degrees/second measured speed and 15 degrees/second reference
speed. This gives stopping distance proportional to initial speed rather than
using the same instantaneous target freeze at every speed.

The first profiled 700-degree/second stop still used a fixed 1800
degrees/second² deceleration and exceeded the lean envelope while measured speed
remained near 718--783. Deceleration varies continuously with measured speed.
The original curve of 1800 at rest down to 600 at 700 degrees/second still
reduced the reference too quickly. Its high-speed endpoint became 200, but a
later exact-`0v3.1` run showed that the linear curve had already risen to 829
by 425 degrees/second and pushed braking lean through the safety boundary. The
curve now reaches 200 at 400 degrees/second and stays there above that speed.
Acceleration remains 600; stronger braking is reserved for the final low-speed
phase.

An 800-degree/second acceleration-inhibit experiment was rejected. Collapsing
the effective request to zero at high lean interacted with reversal detection
and repeatedly prevented a held command from establishing a reference. Temporary
loop counters also confirmed telemetry-induced deadline misses and were removed
after the reporting interval was reduced to 2 Hz.

At an 800-degree/second request, the robot remained at 0.03 degrees lean but
normal trajectory overshoot reached 1005 and tripped the former fixed 1000
cutoff. The emergency threshold is now 1200 while the high-speed angle cutoff is
unchanged. Timing telemetry also exposed 21 ms worst-case lateness and repeated
late-loop bursts around the long 5 Hz BLE report; reports are reduced to 2 Hz.

The next 700 test showed that rate adaptation by itself was insufficient: while
the reference reduced, an accumulated position-target lead still commanded about
90% forward duty and measured speed reached the 1000-degree/second cutoff. A
subsequent continuously recalculated stopping-position experiment was rejected
because it created a moving target and immediate backward runaway from neutral.
The next design must bound any target correction and must not activate from
ordinary stationary balancing.

After restoring neutral balance and lowering the adaptive curve's high-speed
endpoint to 200 degrees/second², 700 degrees/second passed forward and backward
neutral stops, direct reversal, and steering. Its longer braking distance is an
accepted property of the `0v3.1` baseline. Lean-aware braking is the next
development step and must preserve this checkpoint.

### TODO — speed-adaptive steering

Initial low-overhead implementation: stepped limits of 20/10/5 duty below
500/from 500/at 700 degrees/second, additionally capped each loop to reserve 5
duty points beyond the balance command. At -895 degrees/second the prior code
had balance duty saturated at -100 while still adding 17.4 turn duty, immediately
preceding the high-speed angle stop. Hardware validation of the stepped limiter
is pending.

The first hardware test of the 5-duty high-speed ceiling was stable but turned
too slowly. Raising it to 8 duty brought the high-speed stutter back, so that
experiment was rejected and the stable 5-duty ceiling restored. Future turn-rate
improvement must not come from increasing differential duty at high speed.

- Keep the requested turn response unchanged at low speed.
- Progressively reduce maximum turn duty above a measured-speed threshold rather
  than applying a sudden mode switch.
- Base adaptation on measured wheel speed, not requested drive, so overshoot and
  downhill/free-rolling motion are covered.
- Preserve steering direction, neutral dead zone, slew limiting, watchdog, and
  all existing balance safety stops.
- Make the threshold, minimum high-speed turn limit, and transition range runtime
  parameters only after conservative fixed values pass hardware testing.
- Validate straight travel first, then gentle high-speed arcs; do not begin with
  full-stick direction changes.

Acceptance: predictable low-speed turning is unchanged, high-speed steering
cannot consume enough differential duty to destabilize balance, the transition
is continuous, and combined drive/turn trials remain below the high-speed angle
and runaway cutoffs.
