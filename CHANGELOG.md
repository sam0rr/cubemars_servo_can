# Changelog

## 0.2.7 - 2026-02-12

### Fixed

- Added a configurable shutdown soft-stop profile in `CubeMarsServoCAN.__exit__`:
  ramp duration/steps plus optional bounded current-brake hold phase before `power_off()`.
- Added pre-trip thermal guard behavior in `update()`:
  on first over-temperature sample, motion-producing commands are suppressed and safe hold commands are sent immediately.
- Added deterministic thermal guard cooldown policy with configurable hysteresis (`thermal_guard_cooldown_hysteresis_c`) to avoid hot/cool chatter.

### Validation

- Test suite: `168 passed`
- Coverage: `100%` for `src/cubemars_servo_can/*` (`723/723` statements)
- Lint: `ruff` clean
- Format: `black` clean on touched files

## 0.2.6 - 2026-02-12

### Fixed

- Aligned `SET_POS` command position scaling with `SET_POS_SPD` so both position paths encode `int32(position * 10000)`.
- Enforced non-negative/upper-bounded current in low-level `comm_can_set_cb(...)` (`0..60A`) to match current-brake mode semantics.
- Clarified raw telemetry units in parser and `ServoMotorState` docs (`deg_elec`, `ERPM`, `A`) to avoid unit confusion with public rad/rad-s getters.

### Validation

- Test suite: `147 passed`
- Coverage: `100%` for `src/cubemars_servo_can/*` (`633/633` statements)
- Lint: `ruff` clean
- Format: touched files formatted with `black` (full `black --check` is unstable/hangs in this environment)

## 0.2.5 - 2026-02-12

### Fixed

- Added best-effort pre-poweroff soft-stop handling in `CubeMarsServoCAN.__exit__` to reduce abrupt shutdown jerk:
  velocity ramp-down, duty/current zeroing, and position/position-velocity hold behavior.
- Fixed `update()` thermal safety ordering to evaluate over-temperature on freshly synchronized listener telemetry instead of stale prior-cycle state.
- Added configurable over-temperature debounce via `overtemp_trip_count` (must be `>= 1`) to avoid false trips from short telemetry spikes.
- Fixed velocity limit boundary handling to accept exact max-speed commands robustly and clamp tiny float overages at the configured cap.

### Changed

- Increased default `max_mosfett_temp` from `50.0` to `70.0`.

### Validation

- Test suite: `144 passed`
- Coverage: `100%` for `src/cubemars_servo_can/*` (`629/629` statements)
- Lint: `ruff` clean
- Format: `black --check` clean

## 0.2.4 - 2026-02-11

### Fixed

- Hardened connection detection to require a plausible status frame (valid status arbitration ID, 8-byte payload, parseable telemetry) instead of accepting any low-byte ID match.
- Added bounded connection probing (`max_messages`, `max_empty_polls`) to avoid unbounded scans on noisy buses while preserving compatibility with delayed first responses.
- Rejected command-like low-byte IDs during connection validation to prevent command traffic from being treated as device telemetry.
- Rejected local power on/off loopback payloads from being counted as connection evidence.
- Hardened listener dispatch to use the same status-ID classification and ignore non-8-byte / command-echo frames.

### Validation

- Test suite: `125 passed`
- Coverage: `100%` for `src/cubemars_servo_can/*` (`575/575` statements)
- Lint: `ruff` clean
- Format: `black --check` clean

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
