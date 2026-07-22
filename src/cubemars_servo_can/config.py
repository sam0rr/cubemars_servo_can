"""Typed motor and runtime configuration."""

from dataclasses import dataclass
from pathlib import Path

from .constants import DEFAULT_LOG_FIELDS, LogField, MotorModel


@dataclass(frozen=True, kw_only=True, slots=True)
class MotorConfig:
    """Physical conversion factors and safety limits for one actuator model."""

    model_name: str
    min_output_position_radians: float
    max_output_position_radians: float
    min_velocity_erpm: float
    max_velocity_erpm: float
    min_current_amps: float
    max_current_amps: float
    min_output_torque_newton_meters: float
    max_output_torque_newton_meters: float
    effective_torque_constant_newton_meters_per_amp: float
    gear_ratio: float
    pole_pairs: int
    supports_persistent_origin: bool

    def __post_init__(self) -> None:
        """Reject unsafe or internally inconsistent configurations."""
        if not self.model_name.strip():
            raise ValueError("model_name must not be empty")
        ranges = (
            (
                self.min_output_position_radians,
                self.max_output_position_radians,
                "output position",
            ),
            (self.min_velocity_erpm, self.max_velocity_erpm, "velocity"),
            (self.min_current_amps, self.max_current_amps, "current"),
            (
                self.min_output_torque_newton_meters,
                self.max_output_torque_newton_meters,
                "output torque",
            ),
        )
        for minimum, maximum, label in ranges:
            if minimum >= maximum:
                raise ValueError(f"Minimum {label} must be less than maximum {label}")
        if self.effective_torque_constant_newton_meters_per_amp <= 0.0:
            raise ValueError("Torque constant must be positive")
        if self.gear_ratio <= 0.0:
            raise ValueError("gear_ratio must be positive")
        if self.pole_pairs <= 0:
            raise ValueError("pole_pairs must be positive")


@dataclass(frozen=True, kw_only=True, slots=True)
class ServoConfig:
    """CAN, thermal-safety, connection, and logging settings."""

    can_channel: str = "can0"
    max_driver_temperature_celsius: float = 70.0
    overtemperature_trip_count: int = 3
    cooldown_margin_celsius: float = 2.0
    connection_timeout_seconds: float = 0.25
    connection_probe_count: int = 3
    csv_log_path: Path | None = None
    log_fields: tuple[LogField, ...] = DEFAULT_LOG_FIELDS

    def __post_init__(self) -> None:
        """Validate runtime settings before any CAN resources are opened."""
        if not self.can_channel.strip():
            raise ValueError("can_channel must not be empty")
        if self.overtemperature_trip_count < 1:
            raise ValueError("overtemperature_trip_count must be at least 1")
        if self.cooldown_margin_celsius < 0.0:
            raise ValueError("cooldown_margin_celsius must not be negative")
        if self.connection_timeout_seconds <= 0.0:
            raise ValueError("connection_timeout_seconds must be positive")
        if self.connection_probe_count < 1:
            raise ValueError("connection_probe_count must be at least 1")
        if len(set(self.log_fields)) != len(self.log_fields):
            raise ValueError("log_fields must not contain duplicates")


_MOTOR_CONFIGS = {
    MotorModel.AK10_9: MotorConfig(
        model_name=MotorModel.AK10_9,
        min_output_position_radians=-531.965,
        max_output_position_radians=531.965,
        min_velocity_erpm=-100_000.0,
        max_velocity_erpm=100_000.0,
        min_current_amps=-15.0,
        max_current_amps=15.0,
        min_output_torque_newton_meters=-15.0,
        max_output_torque_newton_meters=15.0,
        effective_torque_constant_newton_meters_per_amp=0.206,
        gear_ratio=9.0,
        pole_pairs=21,
        supports_persistent_origin=True,
    ),
    MotorModel.AK40_10: MotorConfig(
        model_name=MotorModel.AK40_10,
        min_output_position_radians=-718.078,
        max_output_position_radians=718.078,
        min_velocity_erpm=-60_000.0,
        max_velocity_erpm=60_000.0,
        min_current_amps=-7.3,
        max_current_amps=7.3,
        min_output_torque_newton_meters=-4.1,
        max_output_torque_newton_meters=4.1,
        effective_torque_constant_newton_meters_per_amp=0.05616438356164384,
        gear_ratio=10.0,
        pole_pairs=14,
        supports_persistent_origin=True,
    ),
    MotorModel.AK80_9: MotorConfig(
        model_name=MotorModel.AK80_9,
        min_output_position_radians=-531.965,
        max_output_position_radians=531.965,
        min_velocity_erpm=-32_000.0,
        max_velocity_erpm=32_000.0,
        min_current_amps=-15.0,
        max_current_amps=15.0,
        min_output_torque_newton_meters=-15.0,
        max_output_torque_newton_meters=15.0,
        effective_torque_constant_newton_meters_per_amp=0.115,
        gear_ratio=9.0,
        pole_pairs=21,
        supports_persistent_origin=True,
    ),
    MotorModel.AKA60_6: MotorConfig(
        model_name=MotorModel.AKA60_6,
        min_output_position_radians=-1_196.797,
        max_output_position_radians=1_196.797,
        min_velocity_erpm=-50_000.0,
        max_velocity_erpm=50_000.0,
        min_current_amps=-11.2,
        max_current_amps=11.2,
        min_output_torque_newton_meters=-9.0,
        max_output_torque_newton_meters=9.0,
        effective_torque_constant_newton_meters_per_amp=0.134,
        gear_ratio=6.0,
        pole_pairs=14,
        supports_persistent_origin=True,
    ),
}


def get_motor_config(model: MotorModel) -> MotorConfig:
    """Return the immutable built-in configuration for ``model``."""
    if not isinstance(model, MotorModel):
        raise TypeError("model must be a MotorModel")
    return _MOTOR_CONFIGS[model]
