# Migrating to 1.0

Version 1.0 intentionally has no deprecated aliases or compatibility shims.

| Before 1.0 | 1.0 |
| --- | --- |
| `CubeMarsServoCAN` | `CubeMarsServoCan` |
| string `motor_type` | `motor=MotorModel.AK80_9` |
| `motor_ID` | `motor_id` |
| constructor safety arguments | `servo_config=ServoConfig(...)` |
| `config_overrides` dictionary | `MotorConfig` or `dataclasses.replace()` |
| `enter_*_control()` | `set_control_mode(ControlMode.*)` |
| `set_zero_position()` | `set_origin(OriginMode.TEMPORARY)` |
| `set_*_radians_per_second()` | `set_output_velocity()` / `set_motor_velocity()` |
| generic `position`, `velocity`, `torque` | explicit read-only properties with units |
| `detach_listener()` / shared-manager close | one idempotent `close()` |

The old `can_manager`, `motor_state`, `servo_can`, and integer-buffer utility
modules were removed. Low-level protocol and transport modules are private.

Additional breaking corrections:

- Motor current configuration uses amperes.
- The `AKA60-6` current cap is 11.2 A, not its controller's 60 A electrical limit.
- Position-profile speed and acceleration now use the documented 10-ERPM units.
- Status temperature is signed.
- Only exact extended `0x29` status frames are routed.
- Error codes 6 and 7 use the current vendor meanings.
- Servo lifecycle no longer sends MIT-mode power frames.

Update imports and configuration as one atomic migration. Code that still imports
an old name should fail immediately so stale control paths cannot go unnoticed.
