"""Pure CubeMars Servo Mode frame encoding and decoding."""

import math
import struct

from ._models import Telemetry
from .constants import OriginMode, _PacketId


def arbitration_id(*, motor_id: int, packet_id: _PacketId) -> int:
    """Build a 29-bit Servo Mode arbitration identifier."""
    _validate_motor_id(motor_id)
    return motor_id | (int(packet_id) << 8)


def status_arbitration_id(*, motor_id: int) -> int:
    """Return the only valid status arbitration identifier for a motor."""
    return arbitration_id(motor_id=motor_id, packet_id=_PacketId.STATUS)


def encode_duty_cycle(*, motor_id: int, duty_cycle: float) -> tuple[int, bytes]:
    """Encode a duty-cycle command."""
    return _scaled_int32_frame(
        motor_id=motor_id,
        packet_id=_PacketId.SET_DUTY,
        value=duty_cycle,
        scale=100_000.0,
    )


def encode_current(*, motor_id: int, current_amps: float) -> tuple[int, bytes]:
    """Encode a q-axis current command."""
    return _scaled_int32_frame(
        motor_id=motor_id,
        packet_id=_PacketId.SET_CURRENT,
        value=current_amps,
        scale=1_000.0,
    )


def encode_current_brake(*, motor_id: int, current_amps: float) -> tuple[int, bytes]:
    """Encode a non-negative current-brake command."""
    if current_amps < 0.0:
        raise ValueError("Brake current must not be negative")
    if current_amps > 60.0:
        raise ValueError("Brake current must not exceed the 60 A protocol limit")
    return _scaled_int32_frame(
        motor_id=motor_id,
        packet_id=_PacketId.SET_CURRENT_BRAKE,
        value=current_amps,
        scale=1_000.0,
    )


def encode_velocity(*, motor_id: int, velocity_erpm: float) -> tuple[int, bytes]:
    """Encode an electrical-RPM velocity command."""
    return _scaled_int32_frame(
        motor_id=motor_id,
        packet_id=_PacketId.SET_VELOCITY,
        value=velocity_erpm,
        scale=1.0,
    )


def encode_position(*, motor_id: int, position_units: float) -> tuple[int, bytes]:
    """Encode a Servo position command."""
    return _scaled_int32_frame(
        motor_id=motor_id,
        packet_id=_PacketId.SET_POSITION,
        value=position_units,
        scale=10_000.0,
    )


def encode_origin(*, motor_id: int, mode: OriginMode) -> tuple[int, bytes]:
    """Encode an origin command."""
    return (
        arbitration_id(motor_id=motor_id, packet_id=_PacketId.SET_ORIGIN),
        bytes((int(mode),)),
    )


def encode_position_velocity(
    *,
    motor_id: int,
    position_units: float,
    velocity_erpm: float,
    acceleration_erpm_per_second: float,
) -> tuple[int, bytes]:
    """Encode a profiled position command using the documented 10-ERPM units."""
    position = _scaled_integer(position_units, 10_000.0, bits=32)
    velocity = _scaled_integer(velocity_erpm, 0.1, bits=16)
    acceleration = _scaled_integer(acceleration_erpm_per_second, 0.1, bits=16)
    if acceleration < 0:
        raise ValueError("Profile acceleration must not be negative")
    return (
        arbitration_id(
            motor_id=motor_id,
            packet_id=_PacketId.SET_POSITION_VELOCITY,
        ),
        struct.pack(">ihh", position, velocity, acceleration),
    )


def decode_status(*, data: bytes, received_at_seconds: float) -> Telemetry:
    """Decode one eight-byte Servo status payload."""
    if len(data) != 8:
        raise ValueError(f"Servo status payload must contain 8 bytes, got {len(data)}")
    position, velocity, current, temperature, fault = struct.unpack(">hhhbB", data)
    return Telemetry(
        position_units=position * 0.1,
        velocity_erpm=velocity * 10.0,
        q_axis_current_amps=current * 0.01,
        temperature_celsius=float(temperature),
        fault_code=fault,
        received_at_seconds=received_at_seconds,
    )


def _scaled_int32_frame(
    *, motor_id: int, packet_id: _PacketId, value: float, scale: float
) -> tuple[int, bytes]:
    """Encode one finite scaled number in a signed 32-bit payload."""
    encoded = _scaled_integer(value, scale, bits=32)
    return arbitration_id(motor_id=motor_id, packet_id=packet_id), struct.pack(
        ">i", encoded
    )


def _scaled_integer(value: float, scale: float, *, bits: int) -> int:
    """Scale and range-check a finite signed integer."""
    if not math.isfinite(value):
        raise ValueError("Protocol values must be finite")
    encoded = int(value * scale)
    minimum = -(2 ** (bits - 1))
    maximum = (2 ** (bits - 1)) - 1
    if not minimum <= encoded <= maximum:
        raise ValueError(f"Scaled value {encoded} is outside signed int{bits} range")
    return encoded


def _validate_motor_id(motor_id: int) -> None:
    """Validate the one-byte controller identifier used by the protocol."""
    if not 0 <= motor_id <= 0xFF:
        raise ValueError("motor_id must be between 0 and 255")
