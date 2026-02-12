# Bug Fix Verification Summary

Last updated: February 12, 2026

This file is the canonical bug record for `cubemars_servo_can`.
The test suite is treated as the source of truth.

## Validation Commands

```bash
UV_OFFLINE=1 UV_CACHE_DIR=.uv-cache uv run --frozen ruff check src tests
UV_OFFLINE=1 UV_CACHE_DIR=.uv-cache uv run --frozen pytest -q
```

Current validation result:

- `147 passed`
- Source coverage: `100%` (`633/633` statements)
- `ruff`: clean
- `black --check`: touched files formatted; full check is unstable/hangs in this environment

## Verified Bug Register (Sequential IDs)

Status legend:

- `fixed`: code changed and regression-tested
- `corrected`: prior claim was inaccurate and replaced with factual note

1. `BUG-001` Negative `dt` sign in async acceleration computation.

- Status: `fixed`
- Code: `src/cubemars_servo_can/servo_can.py`
- Tests: `TestDtCalculation`

2. `BUG-002` CAN payload length overflow (>8 bytes) not rejected.

- Status: `fixed`
- Code: `src/cubemars_servo_can/can_manager.py`
- Tests: `TestBufferOverflow`

3. `BUG-003` Integer buffer append helpers accepted out-of-range values.

- Status: `fixed`
- Code: `src/cubemars_servo_can/utils.py`
- Tests: `tests/test_utils.py`

4. `BUG-004` Motor velocity getter missed ERPM->rad/s conversion.

- Status: `fixed`
- Code: `src/cubemars_servo_can/servo_can.py`
- Tests: `TestVelocityUnitConversion`

5. `BUG-005` Velocity limit check compared mismatched units.

- Status: `fixed`
- Code: `src/cubemars_servo_can/servo_can.py`
- Tests: `TestVelocityLimitUnits`

6. `BUG-006` Position limit check compared mismatched units.

- Status: `fixed`
- Code: `src/cubemars_servo_can/servo_can.py`
- Tests: `TestPositionLimitUnits`

7. `BUG-007` Motor torque setter used incorrect gear-ratio mapping.

- Status: `fixed`
- Code: `src/cubemars_servo_can/servo_can.py`
- Tests: `TestTorqueFormula`

8. `BUG-008` Missing current limit enforcement.

- Status: `fixed`
- Code: `src/cubemars_servo_can/servo_can.py`
- Tests: `TestCurrentLimits`

9. `BUG-009` Missing torque limit enforcement.

- Status: `fixed`
- Code: `src/cubemars_servo_can/servo_can.py`
- Tests: `TestTorqueLimits`

10. `BUG-010` `check_can_connection()` always returned success.

- Status: `fixed`
- Code: `src/cubemars_servo_can/servo_can.py`
- Tests: `TestCheckCanConnection`

11. `BUG-011` Output/motor acceleration getters missed ERPM/s -> rad/s^2 conversion.

- Status: `fixed`
- Code: `src/cubemars_servo_can/servo_can.py`
- Tests: `TestAccelerationUnitConversion`

12. `BUG-012` CAN send failures could be silent to callers.

- Status: `fixed`
- Code: `src/cubemars_servo_can/can_manager.py`
- Tests: `TestCanErrorHandling`

13. `BUG-013` Motor torque getter multiplied by gear ratio instead of dividing.

- Status: `fixed`
- Code: `src/cubemars_servo_can/servo_can.py`
- Tests: `TestTorqueGetterUnits`

14. `BUG-014` Servo frame parser accepted non-8-byte payloads.

- Status: `fixed`
- Code: `src/cubemars_servo_can/can_manager.py`
- Tests: `test_parse_servo_message_invalid_length_raises`

15. `BUG-015` CAN manager singleton allowed silent channel mismatch reuse.

- Status: `fixed`
- Code: `src/cubemars_servo_can/can_manager.py`
- Tests: `TestSingletonBehavior`

16. `BUG-016` Position-velocity mode lacked explicit velocity/acceleration safety checks.

- Status: `fixed`
- Code: `src/cubemars_servo_can/servo_can.py`
- Tests: `TestPositionVelocityModeLimits`

17. `BUG-017` Current brake mode accepted negative current values.

- Status: `fixed`
- Code: `src/cubemars_servo_can/servo_can.py`
- Tests: `TestCurrentBrakeMode`

18. `BUG-018` Runtime constructor/destructor performed implicit privileged host shell calls.

- Status: `fixed`
- Code: `src/cubemars_servo_can/can_manager.py`
- Tests: `TestRemovedRuntimeConfiguration`

19. `BUG-019` CAN manager lacked deterministic shutdown path.

- Status: `fixed`
- Code: `src/cubemars_servo_can/can_manager.py`
- Tests: `TestListenerRegistrationLifecycle`

20. `BUG-020` Listener-thread motor faults raised off-thread instead of on control loop thread.

- Status: `fixed`
- Code: `src/cubemars_servo_can/servo_can.py`
- Tests: `TestErrorHandling`

21. `BUG-021` `__enter__` rollback path could leak state/resources on failure.

- Status: `fixed`
- Code: `src/cubemars_servo_can/servo_can.py`
- Tests: `TestContextManagerAndUpdateBranches`

22. `BUG-022` `__enter__` could leave motor powered if `_send_command()` failed before `_entered=True`.

- Status: `fixed`
- Code: `src/cubemars_servo_can/servo_can.py`
- Tests: `test_enter_failure_after_power_on_still_powers_off`

23. `BUG-023` CAN manager singleton could remain half-initialized if notifier setup failed.

- Status: `fixed`
- Code: `src/cubemars_servo_can/can_manager.py`
- Tests: `test_notifier_init_failure_resets_singleton`, `test_notifier_init_failure_swallows_shutdown_error`

24. `BUG-024` `close()` could leave singleton/resources partially active if notifier or bus shutdown raised.

- Status: `fixed`
- Code: `src/cubemars_servo_can/can_manager.py`
- Tests:
  `test_close_swallows_notifier_stop_error_and_continues`,
  `test_close_swallows_bus_shutdown_error_and_continues`

25. `BUG-025` Mutable default `log_vars` leaked custom log fields across motor instances.

- Status: `fixed`
- Code: `src/cubemars_servo_can/servo_can.py`
- Tests: `test_default_log_vars_are_isolated_per_instance`

26. `BUG-026` `check_can_connection()` accepted command-like low-byte matches as connection evidence.

- Status: `fixed`
- Code: `src/cubemars_servo_can/servo_can.py`, `src/cubemars_servo_can/can_manager.py`
- Tests: `test_check_connection_rejects_command_like_low_byte_id`, `test_command_like_low_byte_id_is_rejected`

27. `BUG-027` `check_can_connection()` could mark connected using non-telemetry frames (non-8-byte, unparseable, or power command loopback payloads).

- Status: `fixed`
- Code: `src/cubemars_servo_can/servo_can.py`
- Tests:
  `test_check_connection_ignores_non_8_byte_frames`,
  `test_check_connection_ignores_unparseable_frames`,
  `test_check_connection_rejects_power_on_echo`

28. `BUG-028` Listener path could parse non-status/non-telemetry frames (command-like IDs, non-8-byte, loopback command echoes).

- Status: `fixed`
- Code: `src/cubemars_servo_can/can_manager.py`
- Tests:
  `test_listener_ignores_command_like_low_byte_id`,
  `test_listener_ignores_non_8_byte_frame`,
  `test_listener_ignores_power_on_echo_on_exact_id`

29. `BUG-029` Thermal cutoff in `update()` could be evaluated against stale state from a previous cycle.

- Status: `fixed`
- Code: `src/cubemars_servo_can/servo_can.py`
- Tests: `test_update_does_not_latch_stale_overtemp_when_async_state_cooled`

30. `BUG-030` A transient single-sample over-temperature spike could force unnecessary shutdown behavior.

- Status: `fixed`
- Code: `src/cubemars_servo_can/servo_can.py`
- Tests:
  `test_invalid_overtemp_trip_count_raises`,
  `test_update_ignores_single_overtemp_spike_with_trip_count`,
  `test_update_raises_after_consecutive_overtemp_samples`

31. `BUG-031` Context-manager shutdown could hard-cut power at non-zero command, causing abrupt stop jerk.

- Status: `fixed`
- Code: `src/cubemars_servo_can/servo_can.py`
- Tests:
  `test_exit_soft_stops_velocity_before_power_off`,
  `test_exit_soft_stops_position_before_power_off`,
  `test_exit_soft_stops_position_velocity_before_power_off`,
  `test_soft_stop_duty_cycle_zeroes_and_sends_once`,
  `test_soft_stop_current_modes_zero_and_send_once`

32. `BUG-032` Velocity limit check could reject exact boundary commands due to strict comparison / float edge effects.

- Status: `fixed`
- Code: `src/cubemars_servo_can/servo_can.py`
- Tests:
  `test_velocity_limit_accepts_exact_max`,
  `test_velocity_limit_clamps_tiny_overage`

33. `BUG-033` Context-manager soft stop remains too abrupt on real inertia/load, causing residual vibration at shutdown.

- Status: `planned`
- Code target: `src/cubemars_servo_can/servo_can.py`
- Issue detail:
  Current best-effort shutdown ramp is short (~40ms in velocity mode), then `power_off()` hard-cuts drive. This can still generate mechanical jerk and audible/structural vibration on geared systems.
- Proposed solution:
  Add a configurable soft-stop profile in `CubeMarsServoCAN.__exit__`:
  (1) longer deceleration window with more ramp steps, (2) optional final `CURRENT_BRAKE` hold phase (bounded current + bounded duration), then (3) `power_off()`.
  Include safe defaults and user-tunable parameters.
- Planned validation:
  Add regression tests for command sequencing and timing behavior in shutdown path (velocity ramp profile, brake phase insertion, and final power-off ordering).
- Target window:
  Week of 2026-02-16.

34. `BUG-034` Over-temperature debounce can still allow motion commands during early hot samples, leading to start-then-abrupt-stop behavior and vibration.

- Status: `planned`
- Code target: `src/cubemars_servo_can/servo_can.py`
- Issue detail:
  With `overtemp_trip_count > 1`, the controller currently keeps sending user commands until the trip threshold is reached. On a hot motor, this can produce short motion bursts followed by forced stop, which feels abrupt and can excite vibration.
- Proposed solution:
  Add a pre-trip thermal guard in `update()`:
  (1) on first over-temp sample, suppress motion-producing commands immediately,
  (2) send a safe hold profile (zero velocity/torque with optional bounded brake hold),
  (3) keep raising hard fault once consecutive sample threshold is reached.
  Add a clear recovery policy (cooldown/hysteresis or explicit re-arm) so behavior is deterministic.
- Planned validation:
  Add tests that verify no new movement command is emitted once temperature exceeds threshold, confirm safe-hold command path, and confirm trip-count fault semantics remain intact.
- Target window:
  Week of 2026-02-16.

35. `BUG-035` Position command packing used inconsistent scaling between `SET_POS` and `SET_POS_SPD`.

- Status: `fixed`
- Code: `src/cubemars_servo_can/can_manager.py`
- Issue detail:
  `comm_can_set_pos()` used `int(pos * 1000000)` while `comm_can_set_pos_spd()` used `int(pos * 10000)`.
  This mismatch could produce large position overshoot when switching between position modes.
- Resolution:
  Standardized both position command paths to `int32(position * 10000)`.
- Tests:
  `test_set_pos_and_set_pos_spd_share_position_scaling`,
  `test_position_mode_sends_correct_command`

36. `BUG-036` Low-level current-brake command helper accepted invalid signed ranges.

- Status: `fixed`
- Code: `src/cubemars_servo_can/can_manager.py`
- Issue detail:
  The current-brake command API documented a `0..60A` domain but did not enforce it, which could allow undefined/ambiguous negative command emission if used directly.
- Resolution:
  Added explicit input validation in `comm_can_set_cb()` and reject values outside `0..60A`.
- Tests:
  `test_current_brake_rejects_negative_current`,
  `test_current_brake_rejects_over_60_amps`

37. `BUG-037` Telemetry-unit documentation around `ServoMotorState` was inconsistent with parser semantics.

- Status: `fixed`
- Code: `src/cubemars_servo_can/motor_state.py`, `src/cubemars_servo_can/can_manager.py`
- Issue detail:
  Internal motor state values are raw telemetry units (`deg_elec`, `ERPM`, `ERPM/s`), but docstrings suggested converted SI units in some places.
- Resolution:
  Updated state/parser docstrings and debug labels to clearly distinguish raw telemetry units from public converted getters.
- Tests: covered by existing parser/state tests (`TestStateParsing`, `test_motor_state_str_contains_fields`)

## Corrected Prior Inaccurate Claim

1. `CORR-001` Prior claim: temperature overflow at 127C due to `np.int16(data[6])`.

- Status: `corrected`
- Fact: `data[6]` is 0..255 and does not overflow at 127 or 255 in `int16`.
- Current parser uses `struct.unpack(">hhhBB", ...)`, where `B` decodes temperature as an unsigned byte.
- Tests: `TestTemperatureParsing`

## Regression and Functionality Retention

The following capabilities are explicitly regression-tested and retained:

- Context manager enter/exit power behavior
- Duty/current/current-brake/velocity/position/position-velocity command emission
- Telemetry getters (position, velocity, torque, acceleration, temperature)
- Safety limit checks (position, velocity, current, torque)
- CAN error surfacing and parser robustness

Notable mode-path tests:

- `test_current_brake_mode_sends_correct_command`
- `test_position_velocity_mode_sends_correct_command`

## Original Library Cross-Check

Inherited defects were verified against:

- `TMotorCANControl/src/TMotorCANControl/servo_can.py`

Confirmed inherited patterns included negative `dt`, unit mismatches in limit checks, weak connection check behavior, silent send failure behavior, and mutable-default argument leakage.
