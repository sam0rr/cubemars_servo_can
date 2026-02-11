# Changelog

## 0.2.2 - 2026-02-11

### Fixed

- Hardened `CubeMarsServoCAN.__enter__` rollback to always attempt `power_off()` after any post-power-on failure (including `_send_command()` exceptions before `_entered=True`).
- Hardened CAN manager startup to reset singleton state and shut down the partially opened bus if `can.Notifier(...)` initialization fails.
- Hardened `CAN_Manager_servo.close()` to be best-effort: notifier/bus errors no longer block listener cleanup, closed-state finalization, or singleton reset.
- Replaced mutable default `log_vars` argument with per-instance list initialization to prevent cross-instance state leakage.

### Validation

- Test suite: `112 passed`
- Coverage: `100%` for `src/cubemars_servo_can/*` (`545/545` statements)
- Lint: `ruff` clean

## 0.2.1 - 2026-02-11

### Fixed

- Replaced fragile signed-int parsing in CAN status frame decode with deterministic `struct.unpack` handling.
- Prevented listener-thread parser exceptions from killing the notifier thread; errors are now surfaced on user `update()` calls.

### Dependencies

- Removed runtime `numpy` dependency.
- Replaced `np.pi` and `np.abs` usage with `math.pi` and built-in `abs`.

### Validation

- Test suite: `106 passed`
- Coverage: `100%` for `src/cubemars_servo_can/*`
- Lint: `ruff` clean

## 0.2.0 - 2026-02-11

### Breaking Changes

- Removed `CAN_Manager_servo.configure_socketcan(...)`.
- Removed `CAN_Manager_servo(..., auto_configure=..., bitrate=...)` initialization path.

### Hardening

- Added explicit CAN-interface open failure handling with actionable `RuntimeError`.
- Reset CAN manager singleton state when bus initialization fails, allowing clean retry.

### Documentation

- Standardized runtime guidance on root-managed boot-time interface setup (`systemd`).
- Clarified default channel behavior (`can0` by default; pass `can_channel` only for other interfaces such as `can1`).

### Validation

- Test suite: `100 passed`
- Coverage: `100%` for `src/cubemars_servo_can/*`
- Lint: `ruff` clean

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
