"""
Utility functions for low-level byte manipulation and buffer management.
"""

from typing import List


def buffer_append_int16(buffer: List[int], number: int) -> None:
    """Appends a 16-bit signed integer to the buffer."""
    if not -32768 <= number <= 32767:
        raise ValueError(f"Value {number} out of int16 range (-32768 to 32767)")
    buffer.append((number >> 8) & 0xFF)
    buffer.append(number & 0xFF)


def buffer_append_uint16(buffer: List[int], number: int) -> None:
    """Appends a 16-bit unsigned integer to the buffer."""
    if not 0 <= number <= 65535:
        raise ValueError(f"Value {number} out of uint16 range (0 to 65535)")
    buffer.append((number >> 8) & 0xFF)
    buffer.append(number & 0xFF)


def buffer_append_int32(buffer: List[int], number: int) -> None:
    """Appends a 32-bit signed integer to the buffer."""
    if not -2147483648 <= number <= 2147483647:
        raise ValueError(
            f"Value {number} out of int32 range (-2147483648 to 2147483647)"
        )
    buffer.append((number >> 24) & 0xFF)
    buffer.append((number >> 16) & 0xFF)
    buffer.append((number >> 8) & 0xFF)
    buffer.append(number & 0xFF)


def buffer_append_uint32(buffer: List[int], number: int) -> None:
    """Appends a 32-bit unsigned integer to the buffer."""
    if not 0 <= number <= 4294967295:
        raise ValueError(f"Value {number} out of uint32 range (0 to 4294967295)")
    buffer.append((number >> 24) & 0xFF)
    buffer.append((number >> 16) & 0xFF)
    buffer.append((number >> 8) & 0xFF)
    buffer.append(number & 0xFF)


def buffer_append_int64(buffer: List[int], number: int) -> None:
    """Appends a 64-bit signed integer to the buffer."""
    if not -9223372036854775808 <= number <= 9223372036854775807:
        raise ValueError(f"Value {number} out of int64 range")
    buffer.append((number >> 56) & 0xFF)
    buffer.append((number >> 48) & 0xFF)
    buffer.append((number >> 40) & 0xFF)
    buffer.append((number >> 32) & 0xFF)
    buffer.append((number >> 24) & 0xFF)
    buffer.append((number >> 16) & 0xFF)
    buffer.append((number >> 8) & 0xFF)
    buffer.append(number & 0xFF)


def buffer_append_uint64(buffer: List[int], number: int) -> None:
    """Appends a 64-bit unsigned integer to the buffer."""
    if not 0 <= number <= 18446744073709551615:
        raise ValueError(f"Value {number} out of uint64 range")
    buffer.append((number >> 56) & 0xFF)
    buffer.append((number >> 48) & 0xFF)
    buffer.append((number >> 40) & 0xFF)
    buffer.append((number >> 32) & 0xFF)
    buffer.append((number >> 24) & 0xFF)
    buffer.append((number >> 16) & 0xFF)
    buffer.append((number >> 8) & 0xFF)
    buffer.append(number & 0xFF)
