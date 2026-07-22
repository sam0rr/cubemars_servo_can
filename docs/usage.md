# Usage and safety

## Prepare SocketCAN

Bring the interface up outside the application. A typical systemd unit for the
Waveshare RS485 CAN HAT is:

```ini
[Unit]
Description=Bring up SocketCAN can0
After=network-pre.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/bin/sh -c 'ip link set can0 down || true'
ExecStart=/bin/sh -c 'ip link set can0 up type can bitrate 1000000 restart-ms 100'
ExecStart=/bin/sh -c 'ip link set can0 txqueuelen 65536'
ExecStop=/bin/sh -c 'ip link set can0 down'

[Install]
WantedBy=multi-user.target
```

Enable it once with `sudo systemctl enable --now can0.service`. The library never
runs privileged host commands.

## Control loop

Choose a mode, set its target, and call `update()` on every control-loop cycle.
Command setters update the pending target; `update()` synchronizes safety state,
sends the selected command, and writes an optional log row.

```python
"""Exercise the typed Servo Mode command API."""

from cubemars_servo_can import (
    ControlMode,
    CubeMarsServoCan,
    MotorModel,
    OriginMode,
    ServoConfig,
)

runtime = ServoConfig(can_channel="can0")

with CubeMarsServoCan(
    motor=MotorModel.AK80_9,
    motor_id=1,
    servo_config=runtime,
) as motor:
    motor.set_origin(OriginMode.TEMPORARY)

    motor.set_control_mode(ControlMode.DUTY_CYCLE)
    motor.set_duty_cycle(0.05)
    motor.update()

    motor.set_control_mode(ControlMode.Q_AXIS_CURRENT)
    motor.set_q_axis_current_amps(0.5)
    motor.set_output_torque(0.5)
    motor.update()

    motor.set_control_mode(ControlMode.CURRENT_BRAKE)
    motor.set_q_axis_current_amps(1.0)
    motor.update()

    motor.set_control_mode(ControlMode.VELOCITY)
    motor.set_output_velocity(1.0)
    motor.set_motor_velocity(9.0)
    motor.update()

    motor.set_control_mode(ControlMode.POSITION)
    motor.set_output_position(0.25)
    motor.set_motor_position(2.25)
    motor.update()

    motor.set_control_mode(ControlMode.POSITION_VELOCITY)
    motor.set_output_position(
        0.5,
        velocity_radians_per_second=1.0,
        acceleration_radians_per_second_squared=2.0,
    )
    motor.update()

    motor.set_control_mode(ControlMode.IDLE)
    motor.update()
```

All position and velocity methods use radians. Torque uses newton-metres and
current uses amperes. Methods prefixed with `set_output_` address the gearbox
output; methods prefixed with `set_motor_` address the motor side.

## Telemetry

After a valid status frame arrives, explicit-unit read-only properties expose:

- `output_position_radians`
- `output_velocity_radians_per_second`
- `output_acceleration_radians_per_second_squared`
- `output_torque_newton_meters`
- the corresponding `motor_*` properties
- `q_axis_current_amps`
- `temperature_celsius`
- `fault_code`

Telemetry is updated on the CAN notifier thread under a lock. Acceleration is
derived from consecutive monotonic timestamps.

## Safety behavior

- Commands outside configured position, ERPM, current, or torque limits raise
  `ValueError`; they are never silently clamped.
- Commands issued in the wrong mode raise `ControlModeError`.
- Driver fault codes raise `MotorFaultError` on the next `update()`.
- The first over-temperature sample activates a guard. Current-producing modes
  receive zero current, while position modes hold the latest reported position.
- The configured consecutive hot-sample count raises `MotorFaultError`.
- The guard clears only below the threshold minus the cooldown margin.
- Listener decoding failures are surfaced as `MotorConnectionError` from the user
  thread.
- Missing status telemetry raises `MotorConnectionError` before another command is
  sent; `telemetry_timeout_seconds` defaults to 1.5 seconds for 1 Hz uploads.
- Transient nonzero driver faults remain latched until `update()` consumes them.
- `close()` is idempotent and always attempts final zero current.

No `__del__` cleanup is used. Keep each motor in a `with` block, or call `close()`
explicitly in application-owned cleanup.

## Multiple motors and channels

Controllers using the same `can_channel` share one bus and notifier. Their motor
IDs must be unique. A different channel gets an independent transport. The bus is
closed when the last registered motor on that channel closes.
