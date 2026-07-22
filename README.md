# CubeMars Servo CAN

Typed Python control for CubeMars AK- and AKA-series actuators running CAN
Servo Mode. Version 1.0 is a forward-only API: legacy names, dictionary
configuration, MIT-mode lifecycle frames, and compatibility aliases are not
included.

## Highlights

- Python 3.13+ with a fully typed public API.
- Built-in safety profiles for `AK10-9`, `AK40-10`, `AK80-9`, and `AKA60-6`.
- Exact extended status-frame routing and vendor-specified wire scaling.
- One thread-safe CAN transport per SocketCAN channel, shared by unique motor IDs.
- Consecutive-sample thermal protection, fault translation, and zero-current exit.
- Optional CSV telemetry logging with explicit units.
- Strict Ruff, mypy, Vulture, and 100% source-coverage gates.

## Install

```bash
uv add cubemars-servo-can
```

The actuator must already be configured for Servo Mode, and the SocketCAN
interface must be up before the application starts.

## Quick start

```python
"""Run one AK80-9 in velocity mode."""

import logging

from cubemars_servo_can import ControlMode, CubeMarsServoCan, MotorModel

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

with CubeMarsServoCan(motor=MotorModel.AK80_9, motor_id=1) as motor:
    motor.set_control_mode(ControlMode.VELOCITY)
    motor.set_output_velocity(1.0)
    motor.update()
    logger.info("Output velocity: %.3f rad/s", motor.output_velocity_radians_per_second)
```

Construction is side-effect free. Entering the context opens the configured CAN
channel, registers the motor ID, sends zero-current Servo probes, and requires a
valid `0x29` status response. Exiting sends a final zero-current command and
releases the channel when its last motor closes.

## Documentation

- [Usage and safety](docs/usage.md)
- [Motor and runtime configuration](docs/configuration.md)
- [Servo protocol decisions](docs/protocol.md)
- [1.0 migration guide](docs/migration-1.0.md)
- [Changelog](CHANGELOG.md)
- [Verified bug register](BUG_FIX_SUMMARY.md)

The bundled [tutorial](tutorial.pdf) and vendor parameter directories remain the
hardware bring-up references. The protocol's position reference is not explicit
enough to justify changing the established high-level position conversion without
hardware-in-the-loop evidence; see the protocol notes before relying on absolute
multi-turn position.

## Development

Run the same immutable commands used by CI:

```bash
uv sync --locked
uv run ruff format --check .
uv run ruff check .
uv run mypy
uv run vulture
uv run pytest
```

Build validation uses `uv build --no-sources` followed by installation of the
wheel into a clean environment.

This project is derived from
[TMotorCANControl](https://github.com/neurobionics/TMotorCANControl).
