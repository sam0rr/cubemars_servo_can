# Usage Guide

This guide covers how to initialize the motor and use the various control modes available.

---

## Basic Initialization

The library uses a context manager (`with` block) to safely handle connection validation and shutdown.

Use one consistent no-`sudo` app runtime flow:

1. Configure your CAN interface (`can0`, `can1`, etc.) at boot with a root-managed service.
2. Run the Python app as a normal user.
3. Keep application code focused on motor control only.

Default interface behavior:

- `ServoConfig(...)` defaults to `can0`.
- You only need to pass `can_channel` when using another interface (for example `can1`).

### Raspberry Pi Notes (Waveshare RS485 CAN HAT)

Reference board and vendor docs:

- https://www.waveshare.com/wiki/RS485_CAN_HAT

Configure overlays in `/boot/firmware/config.txt`:

```ini
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25,spimaxfrequency=2000000
```

Reboot after editing `config.txt` so the overlay is applied:

```bash
sudo reboot
```

### Boot-Time Interface Setup (systemd)

Create `/etc/systemd/system/can0.service` (or `can1.service` if using `can1`):

```ini
[Unit]
Description=Bring up SocketCAN can0
After=network-pre.target
Before=network.target
Wants=network.target

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

Enable and verify:

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now can0.service
ip -details link show can0
```

This runs with root at boot, so your Python app can run without `sudo`.
If your interface is `can1`, replace `can0` with `can1` in both the service file and commands above.

### Application Example

```python
import time

from cubemars_servo_can import CubeMarsServoCan, MotorModel, ServoConfig

config = ServoConfig(
    motor=MotorModel.AK80_9,
    motor_id=1,
)

# Start motor control (interface must already be up via the boot service).
# can_channel defaults to "can0", so it can be omitted.
with CubeMarsServoCan(config) as motor:
    print("Motor connected!")

    # Control logic goes here.
    # ...

    time.sleep(1)
```

If your interface is not `can0`, pass it explicitly:

```python
config = ServoConfig(
    motor=MotorModel.AK80_9,
    motor_id=1,
    can_channel="can1",
)

with CubeMarsServoCan(config) as motor:
    ...
```

**Run it:**

```bash
uv run your_script.py
```

---

## Control Modes

You must explicitly select a control mode before staging a command for that mode.
Command setters stage values; `update()` checks fresh telemetry and safety state
before transmitting.

| Mode API                                          | Sent CAN command    |
| ------------------------------------------------- | ------------------- |
| `set_control_mode(ControlMode.DUTY_CYCLE)`        | `SET_DUTY`          |
| `set_control_mode(ControlMode.Q_AXIS_CURRENT)`    | `SET_CURRENT`       |
| `set_control_mode(ControlMode.CURRENT_BRAKE)`     | `SET_CURRENT_BRAKE` |
| `set_control_mode(ControlMode.VELOCITY)`          | `SET_RPM`           |
| `set_control_mode(ControlMode.POSITION)`          | `SET_POS`           |
| `set_control_mode(ControlMode.POSITION_VELOCITY)` | `SET_POS_SPD`       |

### 1. Position Mode (Most Common)

Moves the motor to a specific motor-shaft angle in radians.

```python
motor.set_control_mode(ControlMode.POSITION)

# Move the motor shaft to 180 degrees.
motor.set_motor_position(3.14)
motor.update()
```

### 2. Velocity Mode

Controls motor-shaft speed in radians per second.

```python
motor.set_control_mode(ControlMode.VELOCITY)

# Spin the motor shaft at 10 rad/s.
motor.set_motor_velocity(10.0)
motor.update()
```

### 3. Current Loop Mode (Torque)

Controls torque through q-axis current.

```python
motor.set_control_mode(ControlMode.Q_AXIS_CURRENT)

# Apply an estimated 0.5 Nm at the motor shaft.
motor.set_motor_torque(0.5)
motor.update()
```

Torque conversion is an ideal estimate:

`motor torque = current × Kt`

`output torque = motor torque × gear ratio`

It does not model gearbox loss, controller calibration, saturation, or
temperature. Measure and calibrate when torque accuracy matters.

### 4. Position-Velocity Mode (Trapezoidal)

Moves to an output position while respecting output-side velocity and
acceleration limits.

```python
motor.set_control_mode(ControlMode.POSITION_VELOCITY)
motor.set_output_position(
    3.14,
    velocity_radians_per_second=5.0,
    acceleration_radians_per_second_squared=10.0,
)
motor.update()
```

Use `set_motor_position()` with the same keyword arguments when the position,
velocity, and acceleration are expressed on the motor-shaft side.

### 5. Current Brake Mode

Applies a non-negative brake current to hold position.

```python
motor.set_control_mode(ControlMode.CURRENT_BRAKE)
motor.set_q_axis_current_amps(2.0)
motor.update()
```

### 6. Duty Cycle Mode

Controls normalized duty cycle directly.

```python
motor.set_control_mode(ControlMode.DUTY_CYCLE)
motor.set_duty_cycle(0.1)
motor.update()
```

### 7. Zeroing

```python
motor.set_origin(OriginMode.TEMPORARY)
motor.set_origin(OriginMode.PERSISTENT)
```

Origin operations are immediate and require an entered context. Temporary mode
lasts until power loss. Persistent mode saves the current physical position in
the motor's nonvolatile parameters; call it deliberately during setup rather
than from a repeated control loop.

### Idle

`ControlMode.IDLE` is the default and sends zero q-axis current on each update.

---

## Safety and Limits

- Position, velocity, current, and torque commands are checked against motor and
  Servo protocol limits.
- Position-velocity mode validates target velocity and acceleration before
  frame packing.
- Current brake mode rejects negative current.
- Invalid modes, non-finite values, and out-of-range values raise typed
  exceptions instead of being silently clamped.
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

### Runtime Safety Options

```python
config = ServoConfig(
    motor=MotorModel.AK80_9,
    motor_id=1,
    max_driver_temperature_celsius=70.0,
    overtemperature_trip_count=3,
    cooldown_margin_celsius=2.0,
    telemetry_timeout_seconds=0.1,
)
```

### Explicit Cleanup

The context manager is the recommended lifecycle. You can call `close()` to end
control early; repeated calls are safe, and context exit will not duplicate the
cleanup.

---

## Telemetry (Reading State)

After a successful context entry, properties expose the most recent decoded
sample. Call `update()` regularly so safety checks and commands run against
fresh telemetry.

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
