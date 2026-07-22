# Changelog

## 1.0.0 - 2026-07-22

### Breaking API

- Replaced `CubeMarsServoCAN` with the keyword-only `CubeMarsServoCan` API.
- Replaced string models and dictionary overrides with `MotorModel`, immutable
  `MotorConfig`, and immutable `ServoConfig`.
- Replaced mode-specific entry methods and duplicate getter/property aliases with
  `set_control_mode()`, canonical command methods, and explicit-unit properties.
- Removed all deprecated aliases, compatibility shims, and public low-level modules.

### Architecture and safety

- Replaced the process-wide one-channel singleton with locked transports keyed by
  SocketCAN channel, unique motor-ID registration, and reference-counted cleanup.
- Made construction side-effect free and context entry transactional.
- Added typed telemetry/command models, monotonic acceleration, structured logging,
  typed exceptions, explicit CSV fields, and deterministic zero-current shutdown.
- Retained and simplified the consecutive-sample thermal guard with cooldown
  hysteresis.

### Protocol corrections

- Corrected position-profile speed and acceleration scaling to 10 ERPM units.
- Decoded temperature as signed int8.
- Restricted status routing to exact extended function ID `0x29`.
- Corrected driver fault 6 and added fault 7.
- Removed MIT-mode FC/FD lifecycle frames from Servo-only operation.
- Tightened actuator current/torque presets while preserving the established
  position conversion pending hardware golden tests.

### Quality

- Enforced the existing immutable Ruff, mypy, Vulture, pytest, and 100% coverage
  policy across source, tests, and executable documentation examples.

Earlier pre-1.0 history remains available in Git.
