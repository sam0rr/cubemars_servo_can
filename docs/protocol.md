# Servo protocol decisions

The wire layer follows the current CubeMars AK Driver Servo Mode documentation.
Every command and status vector is regression-tested byte for byte.

## Arbitration IDs

The extended arbitration ID is `motor_id | (function_id << 8)`. Status telemetry
is accepted only from function ID `0x29`; an exact low-byte motor-ID match or an
unknown high byte is not treated as status.

## Wire values

| Function | Payload |
| --- | --- |
| Duty | signed int32, target × 100000 |
| Current / current brake | signed int32, amperes × 1000 |
| Velocity | signed int32 ERPM |
| Position | signed int32, position value × 10000 |
| Position-velocity | int32 position, int16 ERPM ÷ 10, int16 ERPM/s² ÷ 10 |
| Origin | one byte from `OriginMode` |
| Status | `>hhhbB`: position, velocity, current, signed temperature, fault |

Status scales are 0.1 position units, 10 ERPM, and 0.01 A. Fault code 6 is the
MOSFET over-temperature fault and code 7 is motor stall.

The library does not send the `FF FF FF FF FF FF FF FC/FD` MIT-mode enter/exit
frames. Connection probes and shutdown use Servo current command `0 A`.

## Position reference caveat

The vendor manual describes the wire position as degrees but does not unambiguously
identify the rotor/output reference for every supported firmware and actuator.
No hardware-in-the-loop rig was available for the 1.0 modernization. Therefore,
1.0 deliberately preserves the established high-level conversion factor of
`pi / pole_pairs`, with gearbox mapping performed by the output/motor API, rather
than guessing at a new absolute-position formula.

Before a later release changes this mapping, validate known positive and negative
commands against measured motor-shaft and output-shaft angles on every supported
model. Treat absolute multi-turn position as requiring application-level hardware
validation until that golden test exists.
