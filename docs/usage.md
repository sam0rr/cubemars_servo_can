# Usage

---

## Before starting

Configure the actuator for Servo Mode, assign its CAN ID, and enable periodic
status upload using function ID `0x29`. Bring the SocketCAN interface up outside
the Python process. The library does not change host network configuration.

For a Raspberry Pi CAN HAT, a root-managed boot service can configure the
interface once:

```ini
[Unit]
Description=Bring up SocketCAN can0
After=network-pre.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/sbin/ip link set can0 up type can bitrate 1000000 restart-ms 100
ExecStop=/sbin/ip link set can0 down

[Install]
WantedBy=multi-user.target
```

Confirm the interface before running the application:

```bash
ip -details link show can0
```

---

## Lifecycle

`CubeMarsServoCan` is side-effect-free when constructed. Context entry acquires
the shared channel manager, registers the motor ID, sends zero current, and
waits for fresh status. Context exit attempts zero current and releases the
registration.

```python
from cubemars_servo_can import CubeMarsServoCan, MotorModel, ServoConfig

config = ServoConfig(
    motor=MotorModel.AK40_10,
    motor_id=2,
    can_channel="can0",
)

with CubeMarsServoCan(config) as motor:
    motor.update()
```

Use a regular update loop. Command setters stage values; `update()` checks
fresh telemetry and safety state before transmitting.

---

## Control modes

Select a mode, stage the matching command, and call `update()`.

### Velocity

```python
motor.set_control_mode(ControlMode.VELOCITY)
motor.set_output_velocity(2.0)
motor.update()
```

### Q-axis current

```python
motor.set_control_mode(ControlMode.Q_AXIS_CURRENT)
motor.set_q_axis_current_amps(1.5)
motor.update()
```

Q-axis current is limited to ±60 A and may be further restricted by the motor
configuration.

### Estimated torque

```python
motor.set_control_mode(ControlMode.Q_AXIS_CURRENT)
motor.set_output_torque(3.0)
motor.update()
```

Torque conversion is an ideal estimate:

`motor torque = current × Kt`

`output torque = motor torque × gear ratio`

It does not model gearbox loss, controller calibration, saturation, or
temperature. Measure and calibrate when torque accuracy matters.

### Position

```python
motor.set_control_mode(ControlMode.POSITION)
motor.set_output_position(0.5)
motor.update()
```

### Profiled position

```python
motor.set_control_mode(ControlMode.POSITION_VELOCITY)
motor.set_output_position(
    0.5,
    velocity_radians_per_second=2.0,
    acceleration_radians_per_second_squared=5.0,
)
motor.update()
```

### Current brake

```python
motor.set_control_mode(ControlMode.CURRENT_BRAKE)
motor.set_q_axis_current_amps(1.0)
motor.update()
```

Brake current is limited to the inclusive range from 0 A to 60 A and may be
further restricted by the motor configuration.

### Duty cycle

```python
motor.set_control_mode(ControlMode.DUTY_CYCLE)
motor.set_duty_cycle(0.1)
motor.update()
```

### Idle

`ControlMode.IDLE` sends zero q-axis current on each update.

---

## Origin

Origin operations are immediate and require an entered context:

```python
motor.set_origin(OriginMode.TEMPORARY)
motor.set_origin(OriginMode.PERSISTENT)
```

Only temporary (`0`) and persistent (`1`) operations from the current manual
are supported.

---

## Telemetry

After a successful context entry, properties expose the most recent decoded
sample:

```python
motor.update()

position = motor.output_position_radians
velocity = motor.output_velocity_radians_per_second
current = motor.q_axis_current_amps
temperature = motor.temperature_celsius
torque_estimate = motor.output_torque_newton_meters
```

Motor-shaft position, velocity, acceleration, and torque estimates are also
available through properties prefixed with `motor_`.

---

## Safety behavior

- Stale or malformed telemetry attempts zero current, then raises
  `MotorConnectionError`.
- A reported driver fault attempts zero current, then raises `MotorFaultError`.
- The first over-temperature sample suppresses motion. The configured
  consecutive count causes a typed thermal fault.
- Position modes hold the latest reported position during the pre-trip thermal
  guard; other modes send zero current.
- The guard clears only below the configured cooldown margin.
- `close()` is idempotent and attempts zero current before releasing resources.

These are software safeguards, not a replacement for hardware limits,
mechanical protection, an emergency stop, or commissioning tests.

---
