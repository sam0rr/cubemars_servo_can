"""Golden-vector tests for the CubeMars Servo Mode protocol."""

import math
import struct

import pytest

from cubemars_servo_can import OriginMode, _protocol
from cubemars_servo_can.constants import _PacketId


@pytest.mark.parametrize(
    ("frame", "expected_id", "expected_data"),
    [
        (
            _protocol.encode_duty_cycle(motor_id=1, duty_cycle=0.5),
            0x001,
            struct.pack(">i", 50_000),
        ),
        (
            _protocol.encode_current(motor_id=2, current_amps=-1.25),
            0x102,
            struct.pack(">i", -1_250),
        ),
        (
            _protocol.encode_current_brake(motor_id=3, current_amps=2.5),
            0x203,
            struct.pack(">i", 2_500),
        ),
        (
            _protocol.encode_velocity(motor_id=4, velocity_erpm=-12_345.0),
            0x304,
            struct.pack(">i", -12_345),
        ),
        (
            _protocol.encode_position(motor_id=5, position_units=12.3456),
            0x405,
            struct.pack(">i", 123_455),
        ),
        (
            _protocol.encode_origin(motor_id=6, mode=OriginMode.PERSISTENT),
            0x506,
            b"\x01",
        ),
        (
            _protocol.encode_position_velocity(
                motor_id=7,
                position_units=1.5,
                velocity_erpm=1_230.0,
                acceleration_erpm_per_second=4_560.0,
            ),
            0x607,
            struct.pack(">ihh", 15_000, 123, 456),
        ),
    ],
)
def test_command_frames_match_vendor_golden_vectors(
    frame: tuple[int, bytes], expected_id: int, expected_data: bytes
) -> None:
    """Encode arbitration function IDs, scales, widths, and byte order exactly."""
    assert frame == (expected_id, expected_data)


def test_status_id_is_exact() -> None:
    """Accept only function 0x29 as the documented Servo status ID."""
    assert _protocol.status_arbitration_id(motor_id=0x2A) == 0x292A
    assert _protocol.arbitration_id(motor_id=1, packet_id=_PacketId.STATUS) == 0x2901


def test_decode_status_uses_signed_temperature() -> None:
    """Decode documented scales and preserve a negative signed temperature byte."""
    state = _protocol.decode_status(
        data=struct.pack(">hhhbB", -123, 45, -678, -12, 7),
        received_at_seconds=4.5,
    )
    assert state.position_units == pytest.approx(-12.3)
    assert state.velocity_erpm == 450.0
    assert state.q_axis_current_amps == pytest.approx(-6.78)
    assert state.temperature_celsius == -12.0
    assert state.fault_code == 7
    assert state.received_at_seconds == 4.5


def test_protocol_rejects_invalid_payloads_and_values() -> None:
    """Reject malformed, non-finite, negative-acceleration, and overflowing values."""
    with pytest.raises(ValueError, match="8 bytes"):
        _protocol.decode_status(data=b"short", received_at_seconds=0.0)
    with pytest.raises(ValueError, match="negative"):
        _protocol.encode_current_brake(motor_id=1, current_amps=-0.1)
    with pytest.raises(ValueError, match="finite"):
        _protocol.encode_current(motor_id=1, current_amps=math.inf)
    with pytest.raises(ValueError, match="int32"):
        _protocol.encode_position(motor_id=1, position_units=1_000_000.0)
    with pytest.raises(ValueError, match="int16"):
        _protocol.encode_position_velocity(
            motor_id=1,
            position_units=0.0,
            velocity_erpm=400_000.0,
            acceleration_erpm_per_second=0.0,
        )
    with pytest.raises(ValueError, match="negative"):
        _protocol.encode_position_velocity(
            motor_id=1,
            position_units=0.0,
            velocity_erpm=0.0,
            acceleration_erpm_per_second=-10.0,
        )


@pytest.mark.parametrize("motor_id", [-1, 256])
def test_protocol_rejects_invalid_motor_ids(motor_id: int) -> None:
    """Keep motor identifiers within their one-byte protocol field."""
    with pytest.raises(ValueError, match="between 0 and 255"):
        _protocol.encode_current(motor_id=motor_id, current_amps=0.0)
