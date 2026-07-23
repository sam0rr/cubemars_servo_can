# Servo Mode protocol

CubeMars Servo CAN uses extended arbitration IDs:

`arbitration ID = (function ID << 8) | motor ID`

The library accepts the real-time status function ID `0x29`, startup-state
function ID `0x09`, and exact motor-ID feedback emitted by supported controller
firmware. Feedback must use an extended frame and exactly eight payload bytes.
Short exact-ID frames are ignored because they can be local duty-command
loopback. Known eight-byte exact-ID command payloads are also rejected before
status decoding.

---

## Commands

| Function          |     ID | Payload                                                      |
| ----------------- | -----: | ------------------------------------------------------------ |
| Duty cycle        | `0x00` | signed int32, duty × 100,000                                 |
| Q-axis current    | `0x01` | signed int32, amps × 1,000                                   |
| Current brake     | `0x02` | signed int32, amps × 1,000                                   |
| Velocity          | `0x03` | signed int32 ERPM                                            |
| Position          | `0x04` | signed int32, motor-shaft degrees × 10,000                   |
| Origin            | `0x05` | one byte: temporary `0`, persistent `1`                      |
| Position/velocity | `0x06` | int32 position, int16 velocity ÷ 10, int16 acceleration ÷ 10 |

All multi-byte integers are big-endian. Position/velocity frames contain eight
bytes. The `0xFC` and `0xFD` eight-byte frames used by MIT Mode are deliberately
absent from this Servo Mode lifecycle.

Motor-shaft position commands are limited to ±36,000 degrees. Q-axis current is
limited to ±60 A, current brake to 0–60 A, and velocity-loop commands to
±100,000 ERPM.

---

## Status

The feedback payload layout is:

| Bytes | Type          | Value                              |
| ----- | ------------- | ---------------------------------- |
| 0–1   | signed int16  | motor-shaft position × 0.1 degrees |
| 2–3   | signed int16  | velocity × 10 ERPM                 |
| 4–5   | signed int16  | q-axis current × 0.01 A            |
| 6     | signed int8   | driver temperature in °C           |
| 7     | unsigned int8 | fault code                         |

Fault descriptions include MOSFET over-temperature (`6`) and motor stall
(`7`) according to the current driver manual.

---

## SI conversions

`output rad/s = ERPM × 2π / (60 × pole pairs × gear ratio)`

`output rad = motor-shaft degrees × π / (180 × gear ratio)`

Torque is an ideal estimate:

`motor Nm = current A × Kt`

`output Nm = motor Nm × gear ratio`

Protocol internals live in `_protocol.py` and are intentionally private. The
supported interface uses the SI-unit methods and properties on
`CubeMarsServoCan`.

---
