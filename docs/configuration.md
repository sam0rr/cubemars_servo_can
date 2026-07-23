# Configuration

Every controller receives exactly one immutable `ServoConfig`.

```python
from cubemars_servo_can import MotorModel, ServoConfig

config = ServoConfig(
    motor=MotorModel.AK80_9,
    motor_id=1,
    can_channel="can0",
    max_current_amps=10.0,
    max_output_torque_newton_meters=8.0,
    max_driver_temperature_celsius=70.0,
    telemetry_timeout_seconds=0.1,
)
```

Configuration is constructed explicitly. There are no dictionary overrides,
copy helpers, deprecated names, or compatibility aliases.

## Built-in motor data

| Model | Pole pairs | Gear ratio | Max ERPM | Kt (Nm/A) | Hardware current | Default current | Hardware output torque | Default output torque |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| AK10-9 | 21 | 9 | 60,000 | 0.160 | 31.9 A | 15.0 A | 53.0 Nm | 15.0 Nm |
| AK40-10 | 14 | 10 | 60,000 | 0.056 | 7.3 A | 7.3 A | 4.1 Nm | 4.0 Nm |
| AK80-9 | 21 | 9 | 32,000 | 0.095 | 28.0 A | 15.0 A | 22.0 Nm | 12.5 Nm |
| AKA60-6 | 14 | 6 | 50,000 | 0.11937 | 11.2 A | 11.2 A | 9.0 Nm | 8.0 Nm |

Hardware maxima are reference data, not default commands. An explicit override
may lower a default cap but cannot exceed the corresponding hardware maximum.

## Runtime fields

| Field | Default | Meaning |
| --- | --- | --- |
| `can_channel` | `"can0"` | Existing SocketCAN channel |
| `max_current_amps` | model default | Symmetric q-axis current cap |
| `max_output_torque_newton_meters` | model default | Symmetric ideal output-torque cap |
| `max_driver_temperature_celsius` | `70.0` | Thermal guard threshold |
| `overtemperature_trip_count` | `3` | Consecutive hot samples before fault |
| `cooldown_margin_celsius` | `2.0` | Hysteresis before the guard clears |
| `connection_timeout_seconds` | `1.5` | Context-entry fresh-status timeout |
| `telemetry_timeout_seconds` | `0.1` | Maximum telemetry age during update |

## Custom motor

Create a complete `MotorConfig`, then place it directly in `ServoConfig`:

```python
from cubemars_servo_can import MotorConfig, ServoConfig

motor_data = MotorConfig(
    model="CUSTOM-5",
    pole_pairs=14,
    gear_ratio=5.0,
    max_velocity_erpm=20_000.0,
    torque_constant_newton_meters_per_amp=0.1,
    hardware_max_current_amps=12.0,
    hardware_max_output_torque_newton_meters=6.0,
    default_max_current_amps=5.0,
    default_max_output_torque_newton_meters=2.5,
    supports_persistent_origin=False,
)

config = ServoConfig(
    motor=motor_data,
    motor_id=3,
)
```

All physical values and limits must be finite and positive. Conservative
defaults must not exceed the declared hardware maxima.
