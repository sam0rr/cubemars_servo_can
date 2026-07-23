# Servo Mode protocol

CubeMars Servo CAN uses extended arbitration IDs:

`arbitration ID = (function ID << 8) | motor ID`

The library accepts status only at function ID `0x29`, with an extended frame
and exactly eight payload bytes.

## Commands

| Function          |     ID | Payload                                                      |
| ----------------- | -----: | ------------------------------------------------------------ |
| Duty cycle        | `0x00` | signed int32, duty × 100,000                                 |
| Q-axis current    | `0x01` | signed int32, amps × 1,000                                   |
| Current brake     | `0x02` | signed int32, amps × 1,000                                   |
| Velocity          | `0x03` | signed int32 ERPM                                            |
| Position          | `0x04` | signed int32, electrical degrees × 10,000                    |
| Origin            | `0x05` | one byte: temporary `0`, persistent `1`                      |
| Position/velocity | `0x06` | int32 position, int16 velocity ÷ 10, int16 acceleration ÷ 10 |

All multi-byte integers are big-endian. Position/velocity frames contain eight
bytes. The `0xFC` and `0xFD` eight-byte frames used by MIT Mode are deliberately
absent from this Servo Mode lifecycle.

Electrical-position commands are limited to ±36,000 degrees. Current-brake
commands are limited to the inclusive range from 0 A to 60 A.

## Status

The `0x29` payload layout is:

| Bytes | Type          | Value                             |
| ----- | ------------- | --------------------------------- |
| 0–1   | signed int16  | electrical position × 0.1 degrees |
| 2–3   | signed int16  | velocity × 10 ERPM                |
| 4–5   | signed int16  | q-axis current × 0.01 A           |
| 6     | signed int8   | driver temperature in °C          |
| 7     | unsigned int8 | fault code                        |

Fault descriptions include MOSFET over-temperature (`6`) and motor stall
(`7`) according to the current driver manual.

## SI conversions

`output rad/s = ERPM × 2π / (60 × pole pairs × gear ratio)`

`output rad = electrical degrees × π / (180 × pole pairs × gear ratio)`

Torque is an ideal estimate:

`motor Nm = current A × Kt`

`output Nm = motor Nm × gear ratio`

Protocol internals live in `_protocol.py` and are intentionally private. The
supported interface uses the SI-unit methods and properties on
`CubeMarsServoCan`.
