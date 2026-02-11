# Changelog

## 0.1.2 - 2026-02-11

### Production Hardening

- Refactored CAN manager initialization to remove implicit privileged shell calls.
- Added explicit `CAN_Manager_servo.configure_socketcan(...)` helper for controlled interface setup.
- Added deterministic CAN manager shutdown via `CAN_Manager_servo.close()`.
- Added listener tracking plus `remove_motor(...)` lifecycle handling in CAN manager.
- Improved `CubeMarsServoCAN.__enter__` rollback behavior to avoid partial-entry resource leaks.
- Improved `CubeMarsServoCAN.__exit__` cleanup by resetting lifecycle/error state deterministically.
- Changed async motor fault handling to capture listener-thread faults and raise on `update()`.
- Improved `check_can_connection()` to reduce false positives from stale timestamped messages.

### Validation

- Test suite: `102 passed`
- Coverage: `100%` for `src/cubemars_servo_can/*`
- Lint: `ruff` clean

## 0.1.1 - 2026-02-11

### Fixed

- Corrected acceleration unit conversions (ERPM/s to rad/s^2) for output and motor getters.
- Corrected motor-side torque getter scaling by gear ratio.
- Added CAN send error propagation as `RuntimeError` instead of silent failure.
- Added strict 8-byte validation for parsed servo status frames.
- Added singleton guard for channel mismatch reuse.
- Added position-velocity safety checks for velocity limit and int16 bounds.
- Enforced non-negative current in current brake mode.

### Hardened

- Expanded tests for context manager CSV behavior, update warning/error branches, and mode error branches.
- Expanded tests for listener dispatch and debug output paths.
- Added exhaustive integer-range tests for all utility append helpers.
- Added dedicated tests for motor-state string representation.
- Removed implicit privileged host networking shell calls from runtime constructor/destructor.
- Added explicit `configure_socketcan(...)` helper for controlled interface setup.
- Added deterministic CAN manager shutdown via `close()`.
- Moved listener-thread motor fault surfacing to user `update()` thread.

### Validation

- Test suite: `102 passed`
- Coverage: `100%` for `src/cubemars_servo_can/*`
- Lint: `ruff` clean
