import pytest
from typing import List
from cubemars_servo_can.utils import (
    buffer_append_int16,
    buffer_append_uint16,
    buffer_append_int32,
    buffer_append_int64,
)


def test_buffer_append_int16() -> None:
    buffer: List[int] = []
    # Test positive value
    buffer_append_int16(buffer, 1234)
    assert buffer == [0x04, 0xD2]  # 1234 = 0x04D2

    # Test negative value (-1234 = 0xFB2E in 16-bit 2's complement)
    buffer = []
    buffer_append_int16(buffer, -1234)
    assert buffer == [0xFB, 0x2E]


def test_buffer_append_uint16() -> None:
    buffer: List[int] = []
    buffer_append_uint16(buffer, 65535)
    assert buffer == [0xFF, 0xFF]


def test_buffer_append_int32() -> None:
    buffer: List[int] = []
    val: int = 123456789
    # 123456789 = 0x075BCD15
    buffer_append_int32(buffer, val)
    assert buffer == [0x07, 0x5B, 0xCD, 0x15]


def test_buffer_append_int64() -> None:
    buffer: List[int] = []
    val: int = 1234567890123456789
    # 0x112210F47DE98115
    buffer_append_int64(buffer, val)
    assert buffer == [0x11, 0x22, 0x10, 0xF4, 0x7D, 0xE9, 0x81, 0x15]


# Integer overflow protection tests
def test_int16_overflow_raises_error() -> None:
    """Test that int16 overflow raises ValueError."""
    buffer: List[int] = []
    with pytest.raises(ValueError, match="out of int16 range"):
        buffer_append_int16(buffer, 40000)  # Exceeds int16 max (32767)


def test_int16_underflow_raises_error() -> None:
    """Test that int16 underflow raises ValueError."""
    buffer: List[int] = []
    with pytest.raises(ValueError, match="out of int16 range"):
        buffer_append_int16(buffer, -40000)  # Below int16 min (-32768)


def test_uint16_overflow_raises_error() -> None:
    """Test that uint16 overflow raises ValueError."""
    buffer: List[int] = []
    with pytest.raises(ValueError, match="out of uint16 range"):
        buffer_append_uint16(buffer, 70000)  # Exceeds uint16 max (65535)


def test_uint16_negative_raises_error() -> None:
    """Test that negative values for uint16 raises ValueError."""
    buffer: List[int] = []
    with pytest.raises(ValueError, match="out of uint16 range"):
        buffer_append_uint16(buffer, -1)


def test_int32_overflow_raises_error() -> None:
    """Test that int32 overflow raises ValueError."""
    buffer: List[int] = []
    with pytest.raises(ValueError, match="out of int32 range"):
        buffer_append_int32(buffer, 3000000000)  # Exceeds int32 max


def test_int32_valid_range() -> None:
    """Test that valid int32 values work correctly."""
    buffer: List[int] = []
    buffer_append_int32(buffer, 1000000)  # Within range
    assert len(buffer) == 4
    assert buffer == [0x00, 0x0F, 0x42, 0x40]  # 1000000 in big-endian
