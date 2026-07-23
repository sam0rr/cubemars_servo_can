# CubeMars Servo CAN

A small, typed Python 3.13 library for controlling CubeMars AK and AKA
actuators over SocketCAN in Servo Mode.

Version 1.0 is intentionally forward-only. It exposes one configuration object,
one controller, explicit SI-unit names, and no deprecated aliases.

## Features

- Exact Servo Mode command/status codecs based on the current CubeMars manual.
- Built-in data for `AK10-9`, `AK40-10`, `AK80-9`, and `AKA60-6`.
- Conservative current, output-torque, temperature, and telemetry-age guards.
- One shared CAN manager per SocketCAN channel, with unique motor-ID routing.
- Context-managed lifecycle with deterministic zero-current cleanup.
- Strict typing, linting, dead-code checks, and 100% source coverage.

The library never configures network interfaces or runs privileged commands.
Bring `can0` (or another channel) up before starting the application.

## Install

```bash
uv add git+https://github.com/sam0rr/cubemars_servo_can.git
```

## Quick start

```python
import time

from cubemars_servo_can import (
    ControlMode,
    CubeMarsServoCan,
    MotorModel,
    ServoConfig,
)

config = ServoConfig(
    motor=MotorModel.AK80_9,
    motor_id=1,
)

with CubeMarsServoCan(config) as motor:
    motor.set_control_mode(ControlMode.VELOCITY)
    motor.set_output_velocity(1.0)

    for _ in range(100):
        motor.update()
        time.sleep(0.01)
```

The actuator must already be configured for Servo Mode, use the selected CAN
ID, and upload function `0x29` status frames. Context entry sends zero current
and requires a fresh exact status frame before returning.

## Public API

Import the supported API from `cubemars_servo_can`:

- `CubeMarsServoCan` and `ServoConfig`
- `MotorModel` and `MotorConfig`
- `ControlMode` and `OriginMode`
- the package-specific exception hierarchy

Modules whose names begin with `_` are implementation details and are not part
of the supported API.

## Documentation

- [Usage](docs/usage.md)
- [Configuration](docs/configuration.md)
- [Protocol](docs/protocol.md)
- [Migration to 1.0](docs/migration-1.0.md)
- [Changelog](CHANGELOG.md)
- [Verified bug and behavior register](BUG_FIX_SUMMARY.md)

Vendor firmware, parameter, and mechanical files remain under the existing
`AK40-10-firmware-and-parameters/` and
`AKA60-6-firmware-and-parameters/` directories.

## Development

```bash
uv sync
uv run ruff format --check .
uv run ruff check .
uv run mypy
uv run vulture
uv run pytest
uv build
```

The current 1.0 validation is software-only. Before production use, verify
direction, scaling, limits, fault handling, origin behavior, and shutdown on the
exact actuator, firmware, bus, load, and emergency-stop system.

## Credits

This project originated from
[TMotorCANControl](https://github.com/neurobionics/TMotorCANControl) and now
provides a focused Servo Mode API.
