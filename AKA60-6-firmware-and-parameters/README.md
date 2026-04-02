# AKA60-6 Support Files

This folder contains the vendor files used to cross-check the `AKA60-6` preset.

Included files:

- `AKA60-6_V3_2_20250222.AppParams`
- `AKA60-6_V3_2_20250222.McParams`
- `AKA60-6_SE_V3_2_20250222.bin`
- `AKA60-6_SE_V3_2_20250222.hex`
- `AKA60-6.STEP`
- `Driver-AK54-4810-1C-A6.step`
- `aka60-6-robotic-actuator-2d-drawing.pdf`
- `aka-series-actuator-product-manual-v3-0-0.pdf`

Key values from `AKA60-6_V3_2_20250222.McParams`:

- `l_max_erpm = 50000`
- `l_current_max = 60`
- `l_current_min = -60`
- `foc_encoder_ratio = 14`
- `motor_poles = 14`

Key values from `AKA60-6_V3_2_20250222.AppParams`:

- `controller_id = 104`
- `send_can_status = 1`
- `send_can_status_rate_hz = 50`

Key values from the current official CubeMars AKA60-6 KV80 product page:

- `Reduction Ratio = 6:1`
- `Peak Torque = 9 Nm`
- `Peak Current = 11.2 A`
- `No-load Speed = 320/640 rpm` for `24/48V`

Library preset notes:

- `V_max` follows the vendor `.McParams` limit (`50000 ERPM`).
- `NUM_POLE_PAIRS = 14` matches the vendor encoder/motor-pole divisor.
- `Curr_max` follows the vendor controller current-loop limit (`60 A`) because that is what the shipped parameter file exposes.
- `T_max` is intentionally capped to the actuator's published peak torque (`9 Nm`).
