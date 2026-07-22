# Configuration

Configuration is immutable, keyword-only, slotted, and fully typed. There are no
dictionary override loaders in 1.0.

## Built-in motor profiles

Use `get_motor_config(MotorModel.AK80_9)` to inspect a built-in profile. The
supported models are `AK10_9`, `AK40_10`, `AK80_9`, and `AKA60_6`.

Safety-relevant corrections in 1.0 include:

- `AK40-10`: 7.3 A and 4.1 Nm actuator limits, 10:1 gearing, 14 pole pairs.
- `AKA60-6`: 11.2 A and 9.0 Nm actuator limits, 6:1 gearing, 14 pole pairs.
- `AK80-9`: the conservative 15 A cap is paired with a reachable 15 Nm output
  torque cap.
- Current limits are stored in amperes, not legacy centi-amp compatibility units.

## Runtime settings

```python
"""Configure one logged controller without legacy dictionaries."""

from pathlib import Path

from cubemars_servo_can import (
    CubeMarsServoCan,
    LogField,
    MotorModel,
    ServoConfig,
)

runtime = ServoConfig(
    can_channel="can1",
    max_driver_temperature_celsius=65.0,
    overtemperature_trip_count=3,
    cooldown_margin_celsius=3.0,
    connection_timeout_seconds=1.5,
    connection_probe_count=3,
    telemetry_timeout_seconds=1.5,
    csv_log_path=Path("motor-1.csv"),
    log_fields=(
        LogField.OUTPUT_POSITION_RADIANS,
        LogField.TEMPERATURE_CELSIUS,
    ),
)

motor = CubeMarsServoCan(
    motor=MotorModel.AK40_10,
    motor_id=1,
    servo_config=runtime,
)
motor.close()
```

The constructor does not open the CAN interface. CSV and CAN resources are
acquired only when the context is entered.

## Custom motors and adjusted limits

Construct `MotorConfig` directly, or use `dataclasses.replace()` to derive a
reviewable safety profile from a built-in model.

```python
"""Derive a stricter typed motor profile."""

from dataclasses import replace

from cubemars_servo_can import MotorModel, get_motor_config

base = get_motor_config(MotorModel.AKA60_6)
bench_profile = replace(
    base,
    model_name="AKA60-6-BENCH",
    min_current_amps=-3.0,
    max_current_amps=3.0,
    min_output_torque_newton_meters=-2.0,
    max_output_torque_newton_meters=2.0,
)
```

`MotorConfig` requires explicit output-position limits in radians, velocity
limits in ERPM, current limits in amperes, output-torque limits in newton-metres,
the effective motor torque constant, gear ratio, pole-pair count, and whether
persistent origin writes are allowed.

Torque conversion is:

```text
output torque = q-axis current × effective torque constant × gear ratio
```

Use actuator-specific measured calibration when the approximate torque estimate
is safety-critical.
