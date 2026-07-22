"""Internal command and telemetry models."""

from dataclasses import dataclass


@dataclass(frozen=True, kw_only=True, slots=True)
class Telemetry:
    """One decoded Servo status sample in protocol units."""

    position_units: float = 0.0
    velocity_erpm: float = 0.0
    q_axis_current_amps: float = 0.0
    temperature_celsius: float = 0.0
    fault_code: int = 0
    received_at_seconds: float = 0.0
    acceleration_erpm_per_second: float = 0.0


@dataclass(kw_only=True, slots=True)
class MotorCommand:
    """Mutable command targets in Servo protocol units."""

    position_units: float = 0.0
    velocity_erpm: float = 0.0
    q_axis_current_amps: float = 0.0
    duty_cycle: float = 0.0
    acceleration_erpm_per_second: float = 0.0
