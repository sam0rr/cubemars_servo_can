# Getting Started

## Prerequisites

### 1. Hardware
- A **CubeMars** servo motor (e.g., AK80-9, AK10-9).
- A **CANABLE** or compatible USB-CAN adapter.
- A 24V-48V Power Supply (depending on motor specs).

### 2. Software Dependencies
This library requires a Linux environment with SocketCAN support.

```bash
sudo apt-get install can-utils
```

### 3. Installation

Install the library using `pip` or `uv`:

```bash
pip install cubemars-servo-can
# or
uv pip install cubemars-servo-can
```

## Basic Connection

The library automatically manages the CAN interface (requires `sudo`).

```python
from cubemars_servo_can import CubeMarsServoCAN
import time

# Create the motor object
# This will automatically bring up the 'can0' interface
with CubeMarsServoCAN(motor_type='AK80-9', motor_ID=1, can_channel='can0') as motor:
    print("Motor Connected!")
    
    # Check telemetry
    print(f"Position: {motor.position} rad")
    print(f"Temperature: {motor.temperature} °C")
    
    # Enter a control mode before sending commands
    motor.enter_duty_cycle_control()
    
    # Send a command
    motor.set_duty_cycle_percent(0.1) # 10% duty cycle
    motor.update()
    
    time.sleep(1.0)
    
    # Stop
    motor.set_duty_cycle_percent(0.0)
    motor.update()
```

## Sudo Requirement

Because this library executes `ip link set` commands to configure the CAN bus bitrate (1Mbps), you must run your script with `sudo`:

```bash
sudo python my_script.py
```
