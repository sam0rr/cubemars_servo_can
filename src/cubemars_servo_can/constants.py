"""Public enumerations and internal protocol constants."""

from enum import IntEnum, StrEnum
from typing import Final


class ControlMode(StrEnum):
    """Available Servo Mode control strategies."""

    IDLE = "idle"
    DUTY_CYCLE = "duty_cycle"
    Q_AXIS_CURRENT = "q_axis_current"
    CURRENT_BRAKE = "current_brake"
    VELOCITY = "velocity"
    POSITION = "position"
    POSITION_VELOCITY = "position_velocity"


class OriginMode(IntEnum):
    """Origin operations documented by the Servo Mode protocol."""

    TEMPORARY = 0
    PERSISTENT = 1


class MotorModel(StrEnum):
    """Actuator models with built-in conversion factors and safety caps."""

    AK10_9 = "AK10-9"
    AK40_10 = "AK40-10"
    AK80_9 = "AK80-9"
    AKA60_6 = "AKA60-6"


class _PacketId(IntEnum):
    """Servo Mode function identifiers encoded in the arbitration ID."""

    SET_DUTY = 0x00
    SET_CURRENT = 0x01
    SET_CURRENT_BRAKE = 0x02
    SET_VELOCITY = 0x03
    SET_POSITION = 0x04
    SET_ORIGIN = 0x05
    SET_POSITION_VELOCITY = 0x06
    STARTUP_STATUS = 0x09
    STATUS = 0x29


_FAULT_DESCRIPTIONS: Final[dict[int, str]] = {
    0: "no fault",
    1: "over-temperature fault",
    2: "over-current fault",
    3: "over-voltage fault",
    4: "under-voltage fault",
    5: "encoder fault",
    6: "MOSFET over-temperature fault",
    7: "motor stall fault",
}
