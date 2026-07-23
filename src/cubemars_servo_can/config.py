"""Immutable motor and runtime configuration."""

import math
from dataclasses import dataclass

from .constants import MotorModel


@dataclass(frozen=True, kw_only=True, slots=True)
class MotorConfig:
    """Physical data, hardware maxima, and conservative defaults for a motor."""

    model: str
    pole_pairs: int
    gear_ratio: float
    max_velocity_erpm: float
    torque_constant_newton_meters_per_amp: float
    hardware_max_current_amps: float
    hardware_max_output_torque_newton_meters: float
    default_max_current_amps: float
    default_max_output_torque_newton_meters: float
    supports_persistent_origin: bool = True

    def __post_init__(self) -> None:
        """Reject incomplete or internally inconsistent motor data."""
        if not self.model.strip():
            raise ValueError("model must not be empty")
        if self.pole_pairs <= 0:
            raise ValueError("pole_pairs must be positive")
        positive_values = (
            (self.gear_ratio, "gear_ratio"),
            (self.max_velocity_erpm, "max_velocity_erpm"),
            (
                self.torque_constant_newton_meters_per_amp,
                "torque_constant_newton_meters_per_amp",
            ),
            (self.hardware_max_current_amps, "hardware_max_current_amps"),
            (
                self.hardware_max_output_torque_newton_meters,
                "hardware_max_output_torque_newton_meters",
            ),
            (self.default_max_current_amps, "default_max_current_amps"),
            (
                self.default_max_output_torque_newton_meters,
                "default_max_output_torque_newton_meters",
            ),
        )
        for value, name in positive_values:
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if self.default_max_current_amps > self.hardware_max_current_amps:
            raise ValueError("default current limit exceeds the hardware maximum")
        if (
            self.default_max_output_torque_newton_meters
            > self.hardware_max_output_torque_newton_meters
        ):
            raise ValueError("default torque limit exceeds the hardware maximum")


@dataclass(frozen=True, kw_only=True, slots=True)
class ServoConfig:
    """Complete configuration for one high-level servo controller."""

    motor: MotorModel | MotorConfig
    motor_id: int
    can_channel: str = "can0"
    max_current_amps: float | None = None
    max_output_torque_newton_meters: float | None = None
    max_driver_temperature_celsius: float = 70.0
    overtemperature_trip_count: int = 3
    cooldown_margin_celsius: float = 2.0
    connection_timeout_seconds: float = 1.5
    telemetry_timeout_seconds: float = 0.1

    def __post_init__(self) -> None:
        """Validate settings before any CAN or file resources are acquired."""
        self._validate_identity()
        self._validate_runtime_values()
        motor_config = self.motor_config
        self._validate_limit(
            value=self.max_current_amps,
            hardware_maximum=motor_config.hardware_max_current_amps,
            name="max_current_amps",
        )
        self._validate_limit(
            value=self.max_output_torque_newton_meters,
            hardware_maximum=motor_config.hardware_max_output_torque_newton_meters,
            name="max_output_torque_newton_meters",
        )

    @property
    def motor_config(self) -> MotorConfig:
        """Resolve the selected built-in or custom motor data."""
        if isinstance(self.motor, MotorConfig):
            return self.motor
        return _MOTOR_CONFIGS[self.motor]

    @property
    def current_limit_amps(self) -> float:
        """Return the configured current cap or the conservative model default."""
        if self.max_current_amps is not None:
            return self.max_current_amps
        return self.motor_config.default_max_current_amps

    @property
    def output_torque_limit_newton_meters(self) -> float:
        """Return the configured torque cap or the conservative model default."""
        if self.max_output_torque_newton_meters is not None:
            return self.max_output_torque_newton_meters
        return self.motor_config.default_max_output_torque_newton_meters

    def _validate_identity(self) -> None:
        """Validate motor selection and bus addressing."""
        if not isinstance(self.motor, (MotorModel, MotorConfig)):
            raise TypeError("motor must be a MotorModel or MotorConfig")
        if (
            not isinstance(self.motor_id, int)
            or isinstance(self.motor_id, bool)
            or not 0 <= self.motor_id <= 0xFF
        ):
            raise ValueError("motor_id must be an integer between 0 and 255")
        if not self.can_channel.strip():
            raise ValueError("can_channel must not be empty")

    def _validate_runtime_values(self) -> None:
        """Validate thermal and timing settings."""
        if self.overtemperature_trip_count < 1:
            raise ValueError("overtemperature_trip_count must be at least 1")
        finite_values = (
            (
                self.max_driver_temperature_celsius,
                "max_driver_temperature_celsius",
            ),
            (self.cooldown_margin_celsius, "cooldown_margin_celsius"),
            (self.connection_timeout_seconds, "connection_timeout_seconds"),
            (self.telemetry_timeout_seconds, "telemetry_timeout_seconds"),
        )
        for value, name in finite_values:
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
        if self.cooldown_margin_celsius < 0.0:
            raise ValueError("cooldown_margin_celsius must not be negative")
        if self.connection_timeout_seconds <= 0.0:
            raise ValueError("connection_timeout_seconds must be positive")
        if self.telemetry_timeout_seconds <= 0.0:
            raise ValueError("telemetry_timeout_seconds must be positive")

    @staticmethod
    def _validate_limit(
        *, value: float | None, hardware_maximum: float, name: str
    ) -> None:
        """Validate an optional positive safety limit against hardware data."""
        if value is None:
            return
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        if value > hardware_maximum:
            raise ValueError(f"{name} exceeds the hardware maximum")


_MOTOR_CONFIGS: dict[MotorModel, MotorConfig] = {
    MotorModel.AK10_9: MotorConfig(
        model=MotorModel.AK10_9,
        pole_pairs=21,
        gear_ratio=9.0,
        max_velocity_erpm=60_000.0,
        torque_constant_newton_meters_per_amp=0.160,
        hardware_max_current_amps=31.9,
        hardware_max_output_torque_newton_meters=53.0,
        default_max_current_amps=15.0,
        default_max_output_torque_newton_meters=15.0,
    ),
    MotorModel.AK40_10: MotorConfig(
        model=MotorModel.AK40_10,
        pole_pairs=14,
        gear_ratio=10.0,
        max_velocity_erpm=60_000.0,
        torque_constant_newton_meters_per_amp=0.056,
        hardware_max_current_amps=7.3,
        hardware_max_output_torque_newton_meters=4.1,
        default_max_current_amps=7.3,
        default_max_output_torque_newton_meters=4.0,
    ),
    MotorModel.AK80_9: MotorConfig(
        model=MotorModel.AK80_9,
        pole_pairs=21,
        gear_ratio=9.0,
        max_velocity_erpm=32_000.0,
        torque_constant_newton_meters_per_amp=0.095,
        hardware_max_current_amps=28.0,
        hardware_max_output_torque_newton_meters=22.0,
        default_max_current_amps=15.0,
        default_max_output_torque_newton_meters=12.5,
    ),
    MotorModel.AKA60_6: MotorConfig(
        model=MotorModel.AKA60_6,
        pole_pairs=14,
        gear_ratio=6.0,
        max_velocity_erpm=50_000.0,
        torque_constant_newton_meters_per_amp=0.11937,
        hardware_max_current_amps=11.2,
        hardware_max_output_torque_newton_meters=9.0,
        default_max_current_amps=11.2,
        default_max_output_torque_newton_meters=8.0,
    ),
}
