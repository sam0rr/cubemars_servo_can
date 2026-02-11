# Motor Configuration

---

## Default Configurations

The library comes with built-in support for popular CubeMars motors:

- **AK80-9** (Default)
- **AK10-9**
- **AK40-10**

To use a specific motor:

```python
with CubeMarsServoCAN(motor_type='AK10-9', motor_ID=1) as motor:
    # Motor is loaded with AK10-9 specs (V_max=100k, Kt=0.16)
    pass
```

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

## Notes

- The public API uses radians, rad/s, A, and Nm.
- Internally, some wire/config compatibility fields follow original TMotor scaling conventions.
