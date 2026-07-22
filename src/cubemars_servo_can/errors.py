"""Exceptions raised by CubeMars Servo CAN."""


class CubeMarsServoCanError(Exception):
    """Base exception for package-specific failures."""


class CanConnectionError(CubeMarsServoCanError):
    """Raised when a CAN interface cannot be opened or used."""


class MotorConnectionError(CubeMarsServoCanError):
    """Raised when an actuator does not return valid status telemetry."""


class MotorFaultError(CubeMarsServoCanError):
    """Raised when the actuator reports a driver fault."""

    def __init__(self, *, motor_id: int, fault_code: int, description: str) -> None:
        """Initialize a motor fault with its protocol error code."""
        self.motor_id = motor_id
        self.fault_code = fault_code
        self.description = description
        super().__init__(
            f"Motor {motor_id} reported fault {fault_code}: {description}."
        )


class ControlModeError(CubeMarsServoCanError):
    """Raised when a command is incompatible with the selected mode."""
