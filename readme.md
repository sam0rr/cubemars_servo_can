# CubeMars Servo CAN

A simplified, robust, and modern Python library for controlling CubeMars AK-series actuators (e.g., AK40-10) via CAN bus in Servo Mode.

**This project is a streamlined, refactor of the [TMotorCANControl](https://github.com/neurobionics/TMotorCANControl) library.**

---

## Key Features

- **Refactor:** Modular codebase with strict type hinting, linting, and solid architecture.
- **Zero Bloat:** Stripped of MIT mode and Serial control code to focus 100% on reliable Servo CAN operation.
- **Modern Packaging:** Built with `uv` and `pyproject.toml` for fast, reliable dependency management.
- **Advanced Configuration:** Safe defaults for AK-series motors with the ability to safely override parameters or define custom motors.

---

## Hardware Setup

**You need a CAN link to use this library.**

We highly recommend the **Waveshare RS485 CAN HAT** for Raspberry Pi.

- [Purchase & Wiki Instructions](https://www.waveshare.com/wiki/RS485_CAN_HAT)

For motor wiring and initial configuration (setting the servo mode and CAN ID) please **refer to the [official tutorial PDF](tutorial.pdf)** included in this repository.

---

## Quick Start

### 1. Install

Install directly from the repository using `uv` (recommended) or `pip`:

```bash
# Using uv (Recommended)
uv add git+https://github.com/sam0rr/cubemars_servo_can.git

# Using pip
pip install git+https://github.com/sam0rr/cubemars_servo_can.git
```

---

### 3. Upgrade

To update to the latest version of the library:

```bash
uv lock --upgrade
```

---

## Documentation

- [**Usage Guide**](docs/usage.md): Detailed usage of Duty, Current, Velocity, and Position modes.
- [**Configuration Guide**](docs/configuration.md): How to change gear ratios, limits, or add custom motors.

---

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

---

## Development

To contribute to this library:

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/sam0rr/cubemars_servo_can.git
    cd cubemars_servo_can
    ```

2.  **Install dependencies:**
    ```bash
    uv sync
    ```

3.  **Run linters and formatters:**
    ```bash
    uv run black . && uv run ruff check .
    ```

---

## Credits

Based on the original work by the [neurobionics](https://github.com/neurobionics/TMotorCANControl) team.

---
