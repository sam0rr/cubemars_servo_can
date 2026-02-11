# CubeMars Servo CAN

A simplified, robust, and modern Python library for controlling CubeMars AK-series actuators (e.g., AK40-10) via CAN bus in Servo Mode.

**This project is a streamlined fork/rewrite of the [TMotorCANControl](https://github.com/Umich-NASA-Big-Idea-Challenge/TMotorCANControl) library.**

## Why this exists?

The original `TMotorCANControl` library is goof but contains significant "bloat" for specific use cases:

- It bundles MIT Mode, Servo Mode, and Serial control all together.
- It includes extensive legacy demos, documentation files, and build artifacts.
- It uses older packaging standards.

**`cubemars_servo_can`** strips away everything except the core **Servo Mode via CAN** functionality, offering:

- **Zero Bloat:** Just the code you need to control your motors.
- **Modern Packaging:** Built with `uv` for fast, reliable dependency management.
- **Robust Defaults:** Pre-configured with safety parameters for AK40-10 motors (15A software limit, correct pole pairs).

## Installation & Integration

### 1. Adding to your Project

You can include this library in your own project's `pyproject.toml` or `requirements.txt`.

#### Using uv (Recommended)

If you are developing locally, add it as an editable dependency:

```bash
# If the folder is next to your project
uv add --editable ../cubemars_servo_can
```

#### Using pip

```bash
pip install -e /path/to/cubemars_servo_can
```

#### Manual `pyproject.toml`

If you are managing dependencies manually, add it to your `dependencies` list:

**For local development:**

```toml
[project]
dependencies = [
    "cubemars_servo_can @ file:///path/to/cubemars_servo_can",
]
```

**For production (once pushed to git):**

```toml
[project]
dependencies = [
    "cubemars_servo_can @ git+https://github.com/YOUR_USERNAME/cubemars_servo_can.git",
]
```

### 2. Standalone Setup

If you just want to run this library on its own:

```bash
git clone <your-repo-url>
cd cubemars_servo_can
uv sync
```

## Usage Example

```python
from cubemars_servo_can import TMotorManager_servo_can
import time

# Initialize the motor (e.g., AK40-10 with CAN ID 1)
# NOTE: Ensure your CAN interface (can0) is up before running
with TMotorManager_servo_can(motor_type='AK40-10', motor_ID=1) as motor:
    print("Motor Connected!")

    # 1. Enter Control Mode
    motor.enter_duty_cycle_control()

    # 2. Set Command (5% duty cycle)
    motor.set_duty_cycle_percent(0.05)

    # 3. Update Motor (Send command + Receive state)
    motor.update()

    # Print status
    print(f"Position: {motor.position} rad")
    print(f"Velocity: {motor.velocity} rad/s")
    print(f"Current:  {motor.current_qaxis} A")

    time.sleep(1)

    # The 'with' block automatically handles power-off and cleanup
```

## Supported Motors

Currently optimized for:

- **AK40-10** (KV170) - _Defaults: 15A limit, 7 pole pairs_
- _Inherited support for AK80-9 and AK10-9_

## Development

To run scripts or tests within the managed environment:

```bash
uv run python your_script.py
```

## Credits

Based on the work by the [Umich NASA Big Idea Challenge Team](https://github.com/Umich-NASA-Big-Idea-Challenge/TMotorCANControl).
