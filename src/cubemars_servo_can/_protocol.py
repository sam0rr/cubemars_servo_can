"""Pure CubeMars Servo Mode frame encoding and status decoding."""

import math
import struct
from dataclasses import dataclass

from ._motor_state import ServoTelemetry
from .constants import OriginMode, _PacketId

CURRENT_BRAKE_LIMIT_AMPS = 60.0
POSITION_LIMIT_DEGREES = 36_000.0


@dataclass(frozen=True, kw_only=True, slots=True)
class CanFrame:
    """One extended Classic CAN frame ready for transport."""

    arbitration_id: int
    data: bytes

    def __post_init__(self) -> None:
        """Validate Classic CAN and 29-bit identifier constraints."""
        if not 0 <= self.arbitration_id <= 0x1FFFFFFF:
            raise ValueError("arbitration_id must fit a 29-bit extended CAN ID")
        if len(self.data) > 8:
            raise ValueError("Classic CAN payloads cannot exceed 8 bytes")


def status_arbitration_id(*, motor_id: int) -> int:
    """Return the exact Servo status identifier for a motor."""
    return arbitration_id(motor_id=motor_id, packet_id=_PacketId.STATUS)


def encode_duty_cycle(*, motor_id: int, duty_cycle: float) -> CanFrame:
    """Encode a normalized duty-cycle command."""
    return scaled_int32_frame(
        motor_id=motor_id,
        packet_id=_PacketId.SET_DUTY,
        value=duty_cycle,
        scale=100_000.0,
    )


def encode_current(*, motor_id: int, current_amps: float) -> CanFrame:
    """Encode a q-axis current command."""
    return scaled_int32_frame(
        motor_id=motor_id,
        packet_id=_PacketId.SET_CURRENT,
        value=current_amps,
        scale=1_000.0,
    )


def encode_current_brake(*, motor_id: int, current_amps: float) -> CanFrame:
    """Encode a non-negative current-brake command."""
    if current_amps < 0.0:
        raise ValueError("brake current must not be negative")
    if current_amps > CURRENT_BRAKE_LIMIT_AMPS:
        raise ValueError("brake current exceeds the 60 A protocol limit")
    return scaled_int32_frame(
        motor_id=motor_id,
        packet_id=_PacketId.SET_CURRENT_BRAKE,
        value=current_amps,
        scale=1_000.0,
    )


def encode_velocity(*, motor_id: int, velocity_erpm: float) -> CanFrame:
    """Encode an electrical-RPM velocity command."""
    return scaled_int32_frame(
        motor_id=motor_id,
        packet_id=_PacketId.SET_VELOCITY,
        value=velocity_erpm,
        scale=1.0,
    )


def encode_position(*, motor_id: int, electrical_position_degrees: float) -> CanFrame:
    """Encode an electrical-position command in degrees."""
    validate_position(electrical_position_degrees)
    return scaled_int32_frame(
        motor_id=motor_id,
        packet_id=_PacketId.SET_POSITION,
        value=electrical_position_degrees,
        scale=10_000.0,
    )


def encode_origin(*, motor_id: int, mode: OriginMode) -> CanFrame:
    """Encode a temporary or persistent origin command."""
    return CanFrame(
        arbitration_id=arbitration_id(
            motor_id=motor_id,
            packet_id=_PacketId.SET_ORIGIN,
        ),
        data=bytes((int(mode),)),
    )


def encode_position_velocity(
    *,
    motor_id: int,
    electrical_position_degrees: float,
    velocity_erpm: float,
    acceleration_erpm_per_second: float,
) -> CanFrame:
    """Encode a profiled position command in the manual's 10-ERPM units."""
    validate_position(electrical_position_degrees)
    position = scaled_integer(electrical_position_degrees, 10_000.0, bits=32)
    velocity = scaled_integer(velocity_erpm, 0.1, bits=16)
    acceleration = scaled_integer(
        acceleration_erpm_per_second,
        0.1,
        bits=16,
    )
    if acceleration < 0:
        raise ValueError("profile acceleration must not be negative")
    return CanFrame(
        arbitration_id=arbitration_id(
            motor_id=motor_id,
            packet_id=_PacketId.SET_POSITION_VELOCITY,
        ),
        data=struct.pack(">ihh", position, velocity, acceleration),
    )


def decode_status(*, data: bytes, received_at_seconds: float) -> ServoTelemetry:
    """Decode one exact eight-byte Servo status payload."""
    if len(data) != 8:
        raise ValueError(f"Servo status payload must contain 8 bytes, got {len(data)}")
    position, velocity, current, temperature, fault = struct.unpack(">hhhbB", data)
    return ServoTelemetry(
        electrical_position_degrees=position * 0.1,
        velocity_erpm=velocity * 10.0,
        q_axis_current_amps=current * 0.01,
        temperature_celsius=float(temperature),
        fault_code=fault,
        received_at_seconds=received_at_seconds,
    )


def arbitration_id(*, motor_id: int, packet_id: _PacketId) -> int:
    """Build a 29-bit Servo Mode arbitration identifier."""
    validate_motor_id(motor_id)
    return motor_id | (int(packet_id) << 8)


def scaled_int32_frame(
    *,
    motor_id: int,
    packet_id: _PacketId,
    value: float,
    scale: float,
) -> CanFrame:
    """Encode one finite scaled value in a signed 32-bit payload."""
    encoded = scaled_integer(value, scale, bits=32)
    return CanFrame(
        arbitration_id=arbitration_id(motor_id=motor_id, packet_id=packet_id),
        data=struct.pack(">i", encoded),
    )


def scaled_integer(value: float, scale: float, *, bits: int) -> int:
    """Scale and range-check a finite signed integer."""
    if not math.isfinite(value):
        raise ValueError("protocol values must be finite")
    encoded = int(value * scale)
    minimum = -(2 ** (bits - 1))
    maximum = (2 ** (bits - 1)) - 1
    if not minimum <= encoded <= maximum:
        raise ValueError(f"scaled value {encoded} is outside signed int{bits} range")
    return encoded


def validate_position(electrical_position_degrees: float) -> None:
    """Keep electrical-position commands within the Servo protocol range."""
    if (
        not math.isfinite(electrical_position_degrees)
        or abs(electrical_position_degrees) > POSITION_LIMIT_DEGREES
    ):
        raise ValueError(
            "electrical position must be finite and within ±36,000 degrees"
        )


def validate_motor_id(motor_id: int) -> None:
    """Validate the one-byte controller identifier used by Servo Mode."""
    if (
        not isinstance(motor_id, int)
        or isinstance(motor_id, bool)
        or not 0 <= motor_id <= 0xFF
    ):
        raise ValueError("motor_id must be an integer between 0 and 255")
