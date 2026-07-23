# Motor Configuration

---

## Default Configurations

The library comes with built-in support for popular CubeMars motors:

- **AK80-9** (Default)
- **AK10-9**
- **AK40-10**
- **AKA60-6**

To use a specific motor:

```python
with CubeMarsServoCAN(motor_type='AK10-9', motor_ID=1) as motor:
    # Motor is loaded with AK10-9 specs (V_max=100k, Kt=0.16)
    pass
```

### AKA60-6 Preset Notes

The `AKA60-6` preset uses the real vendor files in
`AKA60-6-firmware-and-parameters/` plus the official published actuator specs.

- `GEAR_RATIO = 6`
- `NUM_POLE_PAIRS = 14`
- `V_max = 50000 ERPM` from `AKA60-6_V3_2_20250222.McParams`
- `Curr_max = 60.0 A` command cap in this library (`6000` in config storage units), from `AKA60-6_V3_2_20250222.McParams`
- `T_max = 9.0 Nm` from the official AKA60-6 KV80 actuator peak torque spec

Two fields still require interpretation for this library's torque API:

- `Kt_TMotor = 0.11937` comes from the published `KV80` motor constant.
- `Kt_actual = 0.134` is kept as the actuator-facing torque constant so the library's torque model reaches the published `9 Nm` peak torque at the published `11.2 A` peak current.

### AK40-10 Preset Notes

The `AK40-10` preset has been cross-checked against both:

- `AK40-10 KV170.McParams`
- the current official CubeMars `AK40-10 KV170` product spec page

The resulting library preset uses:

- `V_max = 60000 ERPM` from the vendor `.McParams`
- `NUM_POLE_PAIRS = 14` from the official product spec
- `Curr_max = 7.3 A` command cap (`730` in config storage units) from the official actuator peak-current spec
- `T_max = 4.1 Nm` from the official actuator peak-torque spec

`Kt_actual` is chosen so the published `4.1 Nm` peak torque maps to the published `7.3 A` peak current through the library torque model.

---

## Overriding Parameters

You may want to tweak a specific parameter (e.g., using a different gear ratio or limiting the max current further for safety). You can pass `config_overrides` to the constructor.

### Example: Changing Gear Ratio

```python
# Override the Gear Ratio to 50:1
my_config = {
    "GEAR_RATIO": 50.0
}

with CubeMarsServoCAN(motor_type='AK80-9', config_overrides=my_config) as motor:
    # Now motor.position will be calculated using 50.0 instead of 9.0
    pass
```

### Example: Defining a Custom Motor

If you are using a motor not in the default list, you can define it completely.
**Note:** You must provide ALL fields.

```python
custom_motor_specs = {
    "P_min": -12.5, "P_max": 12.5,
    "V_min": -50.0, "V_max": 50.0,
    # Current limits are in centi-amps for compatibility with TMotorCANControl.
    # Example: +/-1500 means +/-15.0A command range.
    "Curr_min": -1500.0, "Curr_max": 1500.0,
    "T_min": -5.0, "T_max": 5.0,
    "Kt_TMotor": 0.1,
    "Current_Factor": 0.59,
    "Kt_actual": 0.1,
    "GEAR_RATIO": 1.0, # Direct Drive
    "NUM_POLE_PAIRS": 14,
    "Use_derived_torque_constants": False
}

# Pass None or 'Custom' as motor_type (it will just use your overrides)
with CubeMarsServoCAN(motor_type='Custom', config_overrides=custom_motor_specs) as motor:
    pass
```

---

## Configuration Fields

| Field            | Type    | Description                                                                |
| ---------------- | ------- | -------------------------------------------------------------------------- |
| `P_min/max`      | `float` | Position limits. (Original library uses int32 mapping ~32000 to ~3200 deg) |
| `V_min/max`      | `float` | Velocity limits in Electrical RPM.                                         |
| `Curr_min/max`   | `float` | Current limits in centi-amps (e.g. `1500` = `15.0A`).                      |
| `T_min/max`      | `float` | Torque limits in Nm.                                                       |
| `Kt_TMotor`      | `float` | Torque constant from spec sheet.                                           |
| `Kt_actual`      | `float` | Calibrated torque constant.                                                |
| `GEAR_RATIO`     | `float` | Reduction ratio.                                                           |
| `NUM_POLE_PAIRS` | `int`   | Number of pole pairs.                                                      |

---

## Runtime Safety Options

These constructor parameters control fault behavior and shutdown behavior:

| Parameter             | Type    | Default | Behavior                                                                |
| --------------------- | ------- | ------- | ----------------------------------------------------------------------- |
| `max_mosfet_temp`     | `float` | `70.0`  | Temperature threshold in `update()`.                                    |
| `overtemp_trip_count` | `int`   | `3`     | Consecutive over-limit samples required before raising a thermal fault. |
| `cooldown_margin_c`   | `float` | `2.0`   | Required cooldown margin before thermal guard clears.                   |

Validation is strict:

- `overtemp_trip_count` must be at least `1`.

Context-manager shutdown policy:

- `__exit__` / `close()` send `SET_CURRENT 0.0A` as the final shutdown command.

---

## Notes

- The public API uses radians, rad/s, A, and Nm.
- Internally, some wire/config compatibility fields follow original TMotor scaling conventions.
