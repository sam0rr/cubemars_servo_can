# CubeMars Servo CAN

A simplified, robust, and modern Python library for controlling CubeMars AK-series actuators (e.g., AK40-10) via CAN bus in Servo Mode.

**This project is a streamlined, professional refactor of the [TMotorCANControl](https://github.com/neurobionics/TMotorCANControl) library.**

## Key Features

- **Pro-Grade Refactor:** Modular codebase with strict type hinting, linting, and solid architecture.
- **Zero Bloat:** Stripped of MIT mode and Serial control code to focus 100% on reliable Servo CAN operation.
- **Modern Packaging:** Built with `uv` and `pyproject.toml` for fast, reliable dependency management.
- **Advanced Configuration:** Safe defaults for AK-series motors with the ability to safely override parameters or define custom motors.

## Hardware Setup

**⚠️ You need a CAN link to use this library.**

We highly recommend the **Waveshare RS485 CAN HAT** for Raspberry Pi.

- [Purchase & Wiki Instructions](https://www.waveshare.com/wiki/RS485_CAN_HAT)

For motor wiring and power specifications, please **refer to the [official tutorial PDF](tutorial.pdf)** included in this repository.

## Documentation

- [**Usage Guide**](docs/usage.md): Detailed usage of Duty, Current, Velocity, and Position modes.
- [**Configuration Guide**](docs/configuration.md): How to change gear ratios, limits, or add custom motors.

## Quick Start

### 1. Install

Install directly from the repository using `uv` (recommended) or `pip`:

```bash
# Using uv (Recommended)
uv add git+https://github.com/sam0rr/cubemars_servo_can.git

# Using pip
pip install git+https://github.com/sam0rr/cubemars_servo_can.git
```

### 2. Run (Requires Sudo)

The library automatically manages the CAN interface (bringing `can0` up/down), which requires root privileges.

**Create a script (`main.py`):**

```python
from cubemars_servo_can import CubeMarsServoCAN
import time

# Initialize with 'with' block for safe power-on/off
# Ensure your CAN channel matches your hardware (default 'can0')
with CubeMarsServoCAN(motor_type='AK80-9', motor_ID=1, can_channel='can0') as motor:
    print("Motor Connected!")

    # 1. Enter Control Mode
    motor.enter_position_control()

    # 2. Set Command (180 degrees)
    motor.set_motor_angle_radians(3.14)

    # 3. Update (Send & Receive)
    motor.update()

    print(f"Position: {motor.position:.2f} rad")
    time.sleep(1)
```

**Run it:**

```bash
sudo uv run main.py
```

### 3. Upgrade

To update to the latest version of the library:

```bash
uv lock --upgrade
```

## Project Structure

The library uses a modern `src-layout` for robustness:

```
src/
└── cubemars_servo_can/
    ├── __init__.py      # Package entry point
    ├── servo_can.py     # Main high-level API (CubeMarsServoCAN)
    ├── can_manager.py   # Low-level CAN bus logic
    ├── motor_state.py   # Telemetry data structures
    ├── config.py        # Motor configuration & parameters
    ├── constants.py     # Protocol constants & Error codes
    └── utils.py         # Byte manipulation helpers
```

## Credits

Based on the original work by the [neurobionics](https://github.com/neurobionics/TMotorCANControl) team.
