"""Package-specific exceptions."""


class CubeMarsServoCanError(Exception):
    """Base class for expected library failures."""


class CanConnectionError(CubeMarsServoCanError):
    """Raised when a CAN interface cannot be opened or used."""


class MotorConnectionError(CubeMarsServoCanError):
    """Raised when an actuator does not provide usable fresh telemetry."""


class MotorFaultError(CubeMarsServoCanError):
    """Raised for a driver-reported or library-enforced motor fault."""

    def __init__(self, *, motor_id: int, fault_code: int, description: str) -> None:
        """Store structured fault details and build a readable message."""
        self.motor_id = motor_id
        self.fault_code = fault_code
        self.description = description
        super().__init__(
            f"Motor {motor_id} reported fault {fault_code}: {description}."
        )


class ControlModeError(CubeMarsServoCanError):
    """Raised when a command is incompatible with the selected control mode."""
