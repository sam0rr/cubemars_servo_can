"""Golden-vector tests for the CubeMars Servo Mode protocol."""

import math
import struct
from typing import cast

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
            _protocol.encode_position(
                motor_id=5,
                motor_position_degrees=12.3456,
            ),
            0x405,
            struct.pack(">i", 123_455),
        ),
        (
            _protocol.encode_origin(
                motor_id=6,
                mode=OriginMode.PERSISTENT,
            ),
            0x506,
            b"\x01",
        ),
        (
            _protocol.encode_position_velocity(
                motor_id=7,
                motor_position_degrees=1.5,
                velocity_erpm=1_230.0,
                acceleration_erpm_per_second=4_560.0,
            ),
            0x607,
            struct.pack(">ihh", 15_000, 123, 456),
        ),
    ],
)
def test_command_frames_match_vendor_golden_vectors(
    frame: _protocol.CanFrame,
    expected_id: int,
    expected_data: bytes,
) -> None:
    """Encode function IDs, scales, widths, and byte order exactly."""
    assert frame.arbitration_id == expected_id
    assert frame.data == expected_data


def test_status_id_is_exact() -> None:
    """Accept only function 0x29 as the current documented status ID."""
    assert _protocol.status_arbitration_id(motor_id=0x2A) == 0x292A
    assert _protocol.arbitration_id(motor_id=1, packet_id=_PacketId.STATUS) == 0x2901


def test_decode_status_uses_documented_scales_and_signed_temperature() -> None:
    """Decode all eight bytes, including a negative signed temperature."""
    state = _protocol.decode_status(
        data=struct.pack(">hhhbB", -123, 45, -678, -12, 7),
        received_at_seconds=4.5,
    )
    assert state.motor_position_degrees == pytest.approx(-12.3)
    assert state.velocity_erpm == 450.0
    assert state.q_axis_current_amps == pytest.approx(-6.78)
    assert state.temperature_celsius == -12.0
    assert state.fault_code == 7
    assert state.received_at_seconds == 4.5
    assert state.acceleration_erpm_per_second == 0.0


def test_protocol_rejects_malformed_and_out_of_range_values() -> None:
    """Reject malformed, non-finite, negative, and overflowing inputs."""
    with pytest.raises(ValueError, match="8 bytes"):
        _protocol.decode_status(data=b"short", received_at_seconds=0.0)
    with pytest.raises(ValueError, match="negative"):
        _protocol.encode_current_brake(motor_id=1, current_amps=-0.1)
    with pytest.raises(ValueError, match="finite"):
        _protocol.encode_current_brake(motor_id=1, current_amps=math.nan)
    with pytest.raises(ValueError, match="60 A"):
        _protocol.encode_current_brake(motor_id=1, current_amps=60.1)
    with pytest.raises(ValueError, match="60"):
        _protocol.encode_current(motor_id=1, current_amps=-60.1)
    with pytest.raises(ValueError, match="finite"):
        _protocol.encode_current(motor_id=1, current_amps=math.inf)
    with pytest.raises(ValueError, match="100000"):
        _protocol.encode_velocity(motor_id=1, velocity_erpm=100_000.1)
    with pytest.raises(ValueError, match="finite"):
        _protocol.encode_duty_cycle(motor_id=1, duty_cycle=math.inf)
    with pytest.raises(ValueError, match="36,000"):
        _protocol.encode_position(
            motor_id=1,
            motor_position_degrees=1_000_000.0,
        )
    with pytest.raises(ValueError, match="36,000"):
        _protocol.encode_position_velocity(
            motor_id=1,
            motor_position_degrees=-36_000.1,
            velocity_erpm=0.0,
            acceleration_erpm_per_second=0.0,
        )
    with pytest.raises(ValueError, match="int16"):
        _protocol.encode_position_velocity(
            motor_id=1,
            motor_position_degrees=0.0,
            velocity_erpm=400_000.0,
            acceleration_erpm_per_second=0.0,
        )
    with pytest.raises(ValueError, match="negative"):
        _protocol.encode_position_velocity(
            motor_id=1,
            motor_position_degrees=0.0,
            velocity_erpm=0.0,
            acceleration_erpm_per_second=-10.0,
        )


@pytest.mark.parametrize("motor_id", [-1, 256, cast(int, True)])
def test_protocol_rejects_invalid_motor_ids(motor_id: int) -> None:
    """Keep motor identifiers within the one-byte protocol field."""
    with pytest.raises(ValueError, match="between 0 and 255"):
        _protocol.encode_current(motor_id=motor_id, current_amps=0.0)


def test_can_frame_validates_extended_identifier_and_payload_width() -> None:
    """Reject frames that cannot be sent as extended Classic CAN."""
    with pytest.raises(ValueError, match="29-bit"):
        _protocol.CanFrame(arbitration_id=0x20000000, data=b"")
    with pytest.raises(ValueError, match="8 bytes"):
        _protocol.CanFrame(arbitration_id=1, data=b"123456789")
