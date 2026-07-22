"""Public enumerations used by CubeMars Servo CAN."""

from enum import Enum, IntEnum, StrEnum


class ControlMode(Enum):
    """Available Servo Mode control strategies."""

    IDLE = "idle"
    DUTY_CYCLE = "duty_cycle"
    Q_AXIS_CURRENT = "q_axis_current"
    CURRENT_BRAKE = "current_brake"
    VELOCITY = "velocity"
    POSITION = "position"
    POSITION_VELOCITY = "position_velocity"


class OriginMode(IntEnum):
    """Origin operations defined by the CubeMars Servo protocol."""

    TEMPORARY = 0
    PERSISTENT = 1
    RESTORE_DEFAULT = 2


class LogField(StrEnum):
    """Telemetry fields available in CSV logs."""

    OUTPUT_POSITION_RADIANS = "output_position_radians"
    OUTPUT_VELOCITY_RADIANS_PER_SECOND = "output_velocity_radians_per_second"
    Q_AXIS_CURRENT_AMPS = "q_axis_current_amps"
    TEMPERATURE_CELSIUS = "temperature_celsius"


class MotorModel(StrEnum):
    """CubeMars actuator models with built-in safety limits."""

    AK10_9 = "AK10-9"
    AK40_10 = "AK40-10"
    AK80_9 = "AK80-9"
    AKA60_6 = "AKA60-6"


class _PacketId(IntEnum):
    """Servo Mode command and status function identifiers."""

    SET_DUTY = 0x00
    SET_CURRENT = 0x01
    SET_CURRENT_BRAKE = 0x02
    SET_VELOCITY = 0x03
    SET_POSITION = 0x04
    SET_ORIGIN = 0x05
    SET_POSITION_VELOCITY = 0x06
    STATUS = 0x29


DEFAULT_LOG_FIELDS = (
    LogField.OUTPUT_POSITION_RADIANS,
    LogField.OUTPUT_VELOCITY_RADIANS_PER_SECOND,
    LogField.Q_AXIS_CURRENT_AMPS,
    LogField.TEMPERATURE_CELSIUS,
)

FAULT_DESCRIPTIONS = {
    0: "no fault",
    1: "over-temperature fault",
    2: "over-current fault",
    3: "over-voltage fault",
    4: "under-voltage fault",
    5: "encoder fault",
    6: "MOSFET over-temperature fault",
    7: "motor stall fault",
}
