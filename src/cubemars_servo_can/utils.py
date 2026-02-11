"""
Utility functions for low-level byte manipulation and buffer management.
"""

from typing import List


def buffer_append_int16(buffer: List[int], number: int) -> None:
    """Appends a 16-bit signed integer to the buffer."""
    buffer.append((number >> 8) & 0xFF)
    buffer.append(number & 0xFF)


def buffer_append_uint16(buffer: List[int], number: int) -> None:
    """Appends a 16-bit unsigned integer to the buffer."""
    buffer.append((number >> 8) & 0xFF)
    buffer.append(number & 0xFF)


def buffer_append_int32(buffer: List[int], number: int) -> None:
    """Appends a 32-bit signed integer to the buffer."""
    buffer.append((number >> 24) & 0xFF)
    buffer.append((number >> 16) & 0xFF)
    buffer.append((number >> 8) & 0xFF)
    buffer.append(number & 0xFF)


def buffer_append_uint32(buffer: List[int], number: int) -> None:
    """Appends a 32-bit unsigned integer to the buffer."""
    buffer.append((number >> 24) & 0xFF)
    buffer.append((number >> 16) & 0xFF)
    buffer.append((number >> 8) & 0xFF)
    buffer.append(number & 0xFF)


def buffer_append_int64(buffer: List[int], number: int) -> None:
    """Appends a 64-bit signed integer to the buffer."""
    buffer.append((number >> 56) & 0xFF)
    buffer.append((number >> 48) & 0xFF)
    buffer.append((number >> 40) & 0xFF)
    buffer.append((number >> 31) & 0xFF)
    buffer.append((number >> 24) & 0xFF)
    buffer.append((number >> 16) & 0xFF)
    buffer.append((number >> 8) & 0xFF)
    buffer.append(number & 0xFF)


def buffer_append_uint64(buffer: List[int], number: int) -> None:
    """Appends a 64-bit unsigned integer to the buffer."""
    buffer.append((number >> 56) & 0xFF)
    buffer.append((number >> 48) & 0xFF)
    buffer.append((number >> 40) & 0xFF)
    buffer.append((number >> 31) & 0xFF)
    buffer.append((number >> 24) & 0xFF)
    buffer.append((number >> 16) & 0xFF)
    buffer.append((number >> 8) & 0xFF)
    buffer.append(number & 0xFF)
