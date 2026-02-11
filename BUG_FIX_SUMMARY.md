# Bug Fix Verification Summary

Last updated: February 11, 2026

This file is the canonical bug record for `cubemars_servo_can`.
The test suite is treated as the source of truth.

## Validation Commands

```bash
UV_CACHE_DIR=/tmp/uv-cache uv run pytest -q
UV_CACHE_DIR=/tmp/uv-cache uv run ruff check src tests
```

Current validation result:

- `93 passed`
- Source coverage: `100%` (`472/472` statements)
- `ruff`: clean

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

## Corrected Prior Inaccurate Claim

1. `CORR-001` Prior claim: temperature overflow at 127C due to `np.int16(data[6])`.

- Status: `corrected`
- Fact: `data[6]` is 0..255 and does not overflow at 127 or 255 in `int16`.
- Current parser uses `np.uint8`, which is explicit and correct for an unsigned byte.
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

Confirmed inherited patterns included negative `dt`, unit mismatches in limit checks, weak connection check behavior, and silent send failure behavior.
