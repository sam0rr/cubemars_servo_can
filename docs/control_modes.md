# Control Modes

The library supports several control modes. You must switch to the appropriate mode before sending commands.

## 1. Duty Cycle Mode
Controls the PWM duty cycle directly. Useful for basic open-loop testing.

```python
motor.enter_duty_cycle_control()
motor.set_duty_cycle_percent(0.1) # 10%
motor.update()
```

## 2. Current Loop Mode
Controls the q-axis current (Torque control).

```python
motor.enter_current_control()
motor.set_motor_current_qaxis_amps(2.0) # 2.0 Amps
# OR
motor.set_motor_torque_newton_meters(0.5) # 0.5 Nm (Calculates required amps)
motor.update()
```

## 3. Velocity Mode
Controls the motor speed.

```python
motor.enter_velocity_control()
motor.set_motor_velocity_radians_per_second(10.0) # 10 rad/s
motor.update()
```

## 4. Position Mode
Moves the motor to a specific angle.

```python
motor.enter_position_control()
motor.set_motor_angle_radians(3.14) # 180 degrees
motor.update()
```

## 5. Position-Velocity Mode (Trapezoidal)
Moves to a position with velocity and acceleration limits.

```python
motor.enter_position_velocity_control()
# Target: 3.14 rad, Max Speed: 10 rad/s, Max Accel: 50 rad/s^2
motor.set_output_angle_radians(3.14, 10.0, 50.0) 
motor.update()
```

## 6. Zeroing
Sets the current position as the new zero (origin).

```python
motor.set_zero_position()
```
