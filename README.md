# CubeMars Servo CAN

A simplified, robust, and modern Python library for controlling CubeMars AK-series and AKA-series actuators (for example `AK40-10` and `AKA60-6`) via CAN bus in Servo Mode.

---

## Key Features

- **Refactor:** Modular codebase with strict type hinting, linting, and solid architecture.
- **Zero Bloat:** Focused 100% on reliable Servo CAN operation.
- **Modern Packaging:** Built with `uv` and `pyproject.toml` for fast, reliable dependency management.
- **Advanced Configuration:** Safe defaults for AK-series motors with the ability to safely override parameters or define custom motors.
- **Built-in Motor Presets:** `AK10-9`, `AK80-9`, `AK40-10`, and `AKA60-6`.
- **Quality Gates:** Mock-based test suite with full source coverage (`src/cubemars_servo_can/*`).

---

## Hardware Setup

**You need a CAN link to use this library.**

We highly recommend the **Waveshare RS485 CAN HAT** for Raspberry Pi.

- [Purchase & Wiki Instructions](https://www.waveshare.com/wiki/RS485_CAN_HAT)

For motor wiring and initial configuration (setting the servo mode and CAN ID) please **refer to the [official tutorial PDF](tutorial.pdf)** included in this repository.

### Raspberry Pi + Waveshare RS485 CAN HAT

Keep hardware bring-up details in one place:

- Full Raspberry/Waveshare setup and single recommended runtime flow (boot-time interface bring-up for `can0`, `can1`, etc.): [Usage Guide](docs/usage.md#basic-initialization)

---

## Quick Start

### 1. Install

Install directly from the repository using `uv` or `pip`:

```bash
# Add as a dependency to your project (uv)
uv add git+https://github.com/sam0rr/cubemars_servo_can.git

# Install into the current Python environment (pip)
pip install git+https://github.com/sam0rr/cubemars_servo_can.git
```

---

### 2. Run

Use the usage guide for mode-by-mode examples:

- [**Usage Guide**](docs/usage.md)

---

### 3. Upgrade

Update to the latest repository version:

```bash
# If managed in a uv project dependency:
uv add --upgrade git+https://github.com/sam0rr/cubemars_servo_can.git

# If installed with pip:
pip install --upgrade git+https://github.com/sam0rr/cubemars_servo_can.git
```

---

## Documentation

- [**Usage Guide**](docs/usage.md): Detailed usage of Duty, Current, Velocity, and Position modes.
- [**Configuration Guide**](docs/configuration.md): How to change gear ratios, limits, or add custom motors.

---

## Included Vendor Files

- [`AK40-10-firmware-and-parameters`](AK40-10-firmware-and-parameters): Vendor firmware, parameter dumps, and CAD artifacts for AK40-10.
- [`AKA60-6-firmware-and-parameters`](AKA60-6-firmware-and-parameters): Official AKA60-6 firmware, parameter dumps, and CAD/manual support files.

---

## Development

To contribute to this library:

1. Clone the repository:

```bash
git clone https://github.com/sam0rr/cubemars_servo_can.git
cd cubemars_servo_can
```

2. Install dependencies:

```bash
uv sync
```

3. Run linting and formatting:

```bash
uv run ruff check . --fix
uv run ruff format .
```

4. Run type check:

```bash
uv run mypy
```

5. Run dead code check:

```bash
uv run vulture
```

6. Run tests:

```bash
uv run pytest
```

---
