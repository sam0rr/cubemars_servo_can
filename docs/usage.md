# Usage Guide

This guide covers how to initialize the motor and use the various control modes available.

---

## Basic Initialization

The library uses a context manager (`with` block) to safely handle the connection, power-on sequence, and shutdown.

```python
from cubemars_servo_can import CubeMarsServoCAN
import time

# 1. Setup
# Ensure 'can_channel' matches your interface (e.g., 'can0', 'vcan0')
# Ensure 'motor_type' matches your hardware (AK80-9, AK10-9, etc.)
with CubeMarsServoCAN(motor_type='AK80-9', motor_ID=1, can_channel='can0') as motor:
    print("Motor Connected!")

    # 2. Control Logic goes here
    # ...

    time.sleep(1)
    # Motor automatically powers off when exiting this block
```

**Run it:**

```bash
sudo uv run your_script.py
```

---

## Control Modes

You must explicitly enter a control mode before sending commands for that mode.

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

### 5. Duty Cycle Mode

Controls PWM directly. Mostly for testing.

```python
motor.enter_duty_cycle_control()
motor.set_duty_cycle_percent(0.1) # 10% power
motor.update()
```

### 6. Zeroing

Sets the current physical position as the new "0" (origin).

```python
motor.set_zero_position()
```

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
