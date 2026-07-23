# Verified behavior and bug register

Last updated: July 23, 2026

Version 1.0 replaces the legacy implementation with a small forward-only API.
The regression suite is the source of truth for the verified software behavior.

## Current validation

```bash
uv run ruff format --check .
uv run ruff check .
uv run mypy
uv run vulture
uv run pytest
uv build
```

- `76 passed`
- `640/640` source statements covered
- Ruff clean
- mypy strict clean
- Vulture clean

Validation is software-only. Hardware commissioning remains required.

## Protocol corrections

1. Status frames use the exact extended ID `(0x29 << 8) | motor_id`.
2. Status payloads must contain exactly eight bytes.
3. Temperature is decoded as signed int8.
4. Fault `6` is MOSFET over-temperature and fault `7` is motor stall.
5. Position commands use signed int32 electrical degrees × 10,000.
6. Profile speed and acceleration use signed int16 values divided by ten.
7. Servo lifecycle does not emit MIT Mode `0xFC` or `0xFD` frames.

## Unit and configuration corrections

1. Output velocity converts ERPM using pole pairs and gear ratio.
2. Output position converts electrical degrees using degrees, pole pairs, and
   gear ratio.
3. Torque is clearly documented as an ideal current/Kt/ratio estimate.
4. Built-in model data separates hardware maxima from conservative defaults.
5. Custom motors use a complete typed `MotorConfig`; dictionary replacement and
   scaled compatibility fields are removed.

## Lifecycle and safety corrections

1. Construction performs no CAN or file I/O.
2. Context entry acquires resources transactionally and requires fresh status.
3. Duplicate motor IDs on one channel are rejected.
4. A channel manager closes only after its last motor is released.
5. Listener failures and motor faults surface on the control thread.
6. Fault, stale telemetry, and thermal trip paths attempt zero current first.
7. Pre-trip thermal behavior suppresses motion and uses cooldown hysteresis.
8. Cleanup is idempotent and best-effort without destructors or privileged
   commands.

## Hardware verification required

Before production release, verify on every supported actuator/firmware pair:

1. CAN bitrate, ID, Servo Mode, and `0x29` status upload configuration.
2. Position and velocity sign, scale, and wrap behavior.
3. Current and torque estimates against measured values.
4. Temporary and persistent origin behavior.
5. All driver faults, stale telemetry, thermal guard, emergency stop, and
   shutdown behavior under the real mechanical load.
