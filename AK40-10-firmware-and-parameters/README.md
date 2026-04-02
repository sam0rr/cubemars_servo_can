# AK40-10 Support Files

This folder contains the vendor files used to cross-check the `AK40-10` preset.

Included files:

- `AK40-10 KV170.AppParams`
- `AK40-10 KV170.McParams`
- `AK40-10 KV170.STEP`
- `AK40-10 KV170_V2_20250624.hex`
- `CMESC_SERVO_APP_BLDC_AK40-10_20250313.bin`
- `CMESC_MIT_APP_AK40_10_20250624.bin`

Key values from `AK40-10 KV170.McParams`:

- `l_max_erpm = 60000`
- `l_current_max = 35`
- `l_current_min = -35`
- `foc_encoder_ratio = 14`
- `motor_poles = 14`

Key values from `AK40-10 KV170.AppParams`:

- `controller_id = 69`
- `send_can_status = 0`
- `send_can_status_rate_hz = 50`

Key values from the current official CubeMars AK40-10 KV170 product page:

- `Reduction Ratio = 10:1`
- `Pole Pairs = 14`
- `Peak Torque = 4.1 Nm`
- `Peak Current = 7.3 A`
- `No-load Speed = 435 rpm`
- `Kt = 0.056 Nm/A`

Library preset notes:

- `V_max` follows the vendor `.McParams` limit (`60000 ERPM`).
- `NUM_POLE_PAIRS = 14` is required to make the controller ERPM limit line up with the official `435 rpm` no-load speed.
- `Curr_max` is intentionally capped to the actuator's published peak current (`7.3 A`) instead of the raw controller current-loop ceiling (`35 A`).
- `T_max` is intentionally capped to the actuator's published peak torque (`4.1 Nm`).

Operational note:

- The stored `AK40-10 KV170.AppParams` disables periodic CAN status upload (`send_can_status = 0`).
- This library expects status frames for `check_can_connection()` and normal `update()` telemetry flow.
- If you load that exact `.AppParams` onto hardware, enable CAN status upload in CubeMarsTool before using the library.
