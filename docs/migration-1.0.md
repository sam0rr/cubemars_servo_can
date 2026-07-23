# Migration to 1.0

Version 1.0 intentionally removes the pre-1.0 API. There are no deprecated
aliases or compatibility shims.

## Construction

Before:

```python
motor = CubeMarsServoCAN(
    motor_type="AK80-9",
    motor_ID=1,
    can_channel="can0",
)
```

Now:

```python
config = ServoConfig(
    motor=MotorModel.AK80_9,
    motor_id=1,
    can_channel="can0",
)
motor = CubeMarsServoCan(config)
```

## Modes and commands

| Pre-1.0                                         | 1.0                                            |
| ----------------------------------------------- | ---------------------------------------------- |
| `enter_velocity_control()`                      | `set_control_mode(ControlMode.VELOCITY)`       |
| `enter_current_control()`                       | `set_control_mode(ControlMode.Q_AXIS_CURRENT)` |
| `enter_position_control()`                      | `set_control_mode(ControlMode.POSITION)`       |
| `set_output_velocity_radians_per_second(value)` | `set_output_velocity(value)`                   |
| `set_motor_current_qaxis_amps(value)`           | `set_q_axis_current_amps(value)`               |
| `set_output_angle_radians(value)`               | `set_output_position(value)`                   |
| `set_zero_position()`                           | `set_origin(OriginMode.TEMPORARY)`             |

Use the explicit-unit telemetry properties documented in
[Usage](usage.md#telemetry).

## Configuration

Dictionary overrides and legacy scaled fields are gone. Use either a
`MotorModel` or a complete typed `MotorConfig` inside `ServoConfig`. To choose a
lower current or torque cap, pass it directly to `ServoConfig`; do not copy or
replace a built-in motor configuration.

## Lifecycle changes

- Construction performs no I/O.
- Context entry registers the motor, sends zero current, and waits for fresh
  exact `0x29` telemetry.
- Servo Mode does not send MIT Mode power-on or power-off frames.
- Context exit attempts zero current and releases only this motor's
  registration.
- CAN managers are shared per channel and close after the final motor exits.
