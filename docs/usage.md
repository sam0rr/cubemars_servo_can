# Usage Guide

This guide covers how to initialize the motor and use the various control modes available.

---

## Basic Initialization

The library uses a context manager (`with` block) to safely handle the connection, power-on sequence, and shutdown.

Use one consistent no-`sudo` app runtime flow:

1. Configure your CAN interface (`can0`, `can1`, etc.) at boot with a root-managed service.
2. Run the Python app as a normal user.
3. Keep application code focused on motor control only.

Default interface behavior:

- `CubeMarsServoCAN(...)` defaults to `can0`.
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
from cubemars_servo_can import CubeMarsServoCAN
import time

# 1. Start motor control (interface must already be up via boot service).
# can_channel defaults to "can0", so it can be omitted.
with CubeMarsServoCAN(motor_type='AK80-9', motor_ID=1) as motor:
    print("Motor Connected!")

    # 2. Control Logic goes here
    # ...

    time.sleep(1)
    # Motor automatically powers off when exiting this block
```

If your interface is not `can0`, pass it explicitly:

```python
with CubeMarsServoCAN(motor_type='AK80-9', motor_ID=1, can_channel='can1') as motor:
    ...
```

**Run it:**

```bash
uv run your_script.py
```

---

## Control Modes

You must explicitly enter a control mode before sending commands for that mode.

| Mode API                            | Sent CAN Command    |
| ----------------------------------- | ------------------- |
| `enter_duty_cycle_control()`        | `SET_DUTY`          |
| `enter_current_control()`           | `SET_CURRENT`       |
| `enter_current_brake_control()`     | `SET_CURRENT_BRAKE` |
| `enter_velocity_control()`          | `SET_RPM`           |
| `enter_position_control()`          | `SET_POS`           |
| `enter_position_velocity_control()` | `SET_POS_SPD`       |

### 1. Position Mode (Most Common)

Moves the motor to a specific angle (in radians).

```python
motor.enter_position_control()

# Move to 180 degrees (3.14 radians)
motor.set_motor_angle_radians(3.14)
motor.update()
```

### 2. Velocity Mode

Controls the motor speed (in rad/s).

```python
motor.enter_velocity_control()

# Spin at 10 rad/s (~95 RPM)
motor.set_motor_velocity_radians_per_second(10.0)
motor.update()
```

### 3. Current Loop Mode (Torque)

Controls the torque directly.

```python
motor.enter_current_control()

# Apply 0.5 Nm of torque
# The library automatically calculates the required current based on motor Kt
motor.set_motor_torque_newton_meters(0.5)
motor.update()
```

### 4. Position-Velocity Mode (Trapezoidal)

Moves to a position but respects velocity and acceleration limits. Useful for smooth movements.

```python
motor.enter_position_velocity_control()

# Target: 3.14 rad
# Max Speed: 5.0 rad/s
# Max Accel: 10.0 rad/s^2
motor.set_output_angle_radians(3.14, 5.0, 10.0)
motor.update()
```

### 5. Current Brake Mode

Applies brake current to hold position.

```python
motor.enter_current_brake_control()

# Brake current must be non-negative
motor.set_motor_current_qaxis_amps(2.0)
motor.update()
```

### 6. Duty Cycle Mode

Controls PWM directly. Mostly for testing.

```python
motor.enter_duty_cycle_control()
motor.set_duty_cycle_percent(0.1) # 10% power
motor.update()
```

### 7. Zeroing

Sets the current physical position as the new "0" (origin).

```python
motor.set_zero_position()
```

## Safety and Limits

- Position, velocity, current, and torque commands are checked against motor config limits.
- Position-velocity mode validates target velocity and acceleration before frame packing.
- Current brake mode rejects negative current values.
- `max_mosfett_temp` defaults to `70.0`.
- `update()` raises if temperature exceeds configured max for `overtemp_trip_count` consecutive updates (default `3`).
- While over-temperature pre-trip guard is active, `update()` sends safe hold commands (no new motion commands) until cooldown hysteresis clears.
- Motor faults reported from CAN listener are raised on the next `update()` call.
- `with CubeMarsServoCAN(...)` performs connection validation on entry.
- `__exit__` performs a configurable best-effort soft stop before `power_off()` using:
  `soft_stop_ramp_duration_s`, `soft_stop_ramp_steps`, `soft_stop_brake_hold_current_amps`, and `soft_stop_brake_hold_duration_s`.

## Telemetry (Reading State)

You can read the motor state at any time after calling `motor.update()`.

```python
motor.update()

print(f"Position: {motor.position:.3f} rad")
print(f"Velocity: {motor.velocity:.3f} rad/s")
print(f"Current:  {motor.current_qaxis:.3f} A")
print(f"Torque:   {motor.torque:.3f} Nm")
print(f"Temp:     {motor.temperature:.1f} °C")
```

---
