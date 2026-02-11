# Changelog

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

### Validation

- Test suite: `93 passed`
- Coverage: `100%` for `src/cubemars_servo_can/*`
- Lint: `ruff` clean
