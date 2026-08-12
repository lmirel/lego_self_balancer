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
message is validated against hub-owned ceilings of 300 degrees/second drive and
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
