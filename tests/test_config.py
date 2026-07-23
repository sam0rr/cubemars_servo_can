"""Tests for immutable, explicit 1.0 configuration."""

import math
from collections.abc import Callable
from typing import cast

import pytest

from cubemars_servo_can import MotorConfig, MotorModel, ServoConfig


def custom_motor() -> MotorConfig:
    """Build valid explicit custom motor data."""
    return MotorConfig(
        model="CUSTOM",
        pole_pairs=10,
        gear_ratio=5.0,
        max_velocity_erpm=20_000.0,
        torque_constant_newton_meters_per_amp=0.1,
        hardware_max_current_amps=20.0,
        hardware_max_output_torque_newton_meters=10.0,
        default_max_current_amps=10.0,
        default_max_output_torque_newton_meters=5.0,
    )


def invalid_motor(case: str) -> MotorConfig:
    """Build one deliberately invalid custom motor configuration."""
    return MotorConfig(
        model=cast(str, 1)
        if case == "model_type"
        else " "
        if case == "model"
        else "CUSTOM",
        pole_pairs=cast(int, True)
        if case == "pole_pairs_type"
        else 0
        if case == "pole_pairs"
        else 10,
        gear_ratio=cast(float, True)
        if case == "gear_ratio_type"
        else 0.0
        if case == "gear_ratio"
        else 5.0,
        max_velocity_erpm=math.nan if case == "velocity" else 20_000.0,
        torque_constant_newton_meters_per_amp=0.1,
        hardware_max_current_amps=5.0 if case == "current_limit" else 20.0,
        hardware_max_output_torque_newton_meters=(
            5.0 if case == "torque_limit" else 10.0
        ),
        default_max_current_amps=6.0 if case == "current_limit" else 10.0,
        default_max_output_torque_newton_meters=(
            6.0 if case == "torque_limit" else 5.0
        ),
    )


def test_builtin_models_match_documented_data_and_defaults() -> None:
    """Resolve every model through the single public ServoConfig object."""
    configs = {
        model: ServoConfig(motor=model, motor_id=1).motor_config for model in MotorModel
    }
    assert configs[MotorModel.AK10_9].hardware_max_current_amps == 31.9
    assert configs[MotorModel.AK40_10].default_max_output_torque_newton_meters == 4.0
    assert configs[MotorModel.AK80_9].torque_constant_newton_meters_per_amp == 0.095
    assert configs[MotorModel.AKA60_6].gear_ratio == 6.0
    assert [config.model for config in configs.values()] == [
        model.value for model in MotorModel
    ]


def test_custom_motor_and_explicit_caps_are_resolved_without_copy_helpers() -> None:
    """Accept a complete custom motor and explicit conservative limits."""
    motor = custom_motor()
    config = ServoConfig(
        motor=motor,
        motor_id=42,
        max_current_amps=8.0,
        max_output_torque_newton_meters=4.0,
    )
    assert config.motor_config is motor
    assert config.current_limit_amps == 8.0
    assert config.output_torque_limit_newton_meters == 4.0


def test_default_caps_and_runtime_defaults_are_conservative() -> None:
    """Use model defaults unless the caller explicitly chooses lower caps."""
    config = ServoConfig(motor=MotorModel.AK80_9, motor_id=1)
    assert config.current_limit_amps == 15.0
    assert config.output_torque_limit_newton_meters == 12.5
    assert config.can_channel == "can0"
    assert config.telemetry_timeout_seconds == 0.1


@pytest.mark.parametrize(
    ("case", "error_type", "message"),
    [
        ("model_type", TypeError, "model"),
        ("model", ValueError, "model"),
        ("pole_pairs_type", TypeError, "pole_pairs"),
        ("pole_pairs", ValueError, "pole_pairs"),
        ("gear_ratio_type", TypeError, "gear_ratio"),
        ("gear_ratio", ValueError, "gear_ratio"),
        ("velocity", ValueError, "max_velocity_erpm"),
        ("current_limit", ValueError, "current limit"),
        ("torque_limit", ValueError, "torque limit"),
    ],
)
def test_motor_config_rejects_invalid_physical_data(
    case: str,
    error_type: type[Exception],
    message: str,
) -> None:
    """Reject invalid physical values and defaults beyond hardware maxima."""
    with pytest.raises(error_type, match=message):
        invalid_motor(case)


@pytest.mark.parametrize(
    ("factory", "error_type", "message"),
    [
        (
            lambda: ServoConfig(
                motor=cast(MotorModel, "AK80-9"),
                motor_id=1,
            ),
            TypeError,
            "motor",
        ),
        (
            lambda: ServoConfig(motor=MotorModel.AK80_9, motor_id=cast(int, True)),
            TypeError,
            "motor_id",
        ),
        (
            lambda: ServoConfig(motor=MotorModel.AK80_9, motor_id=256),
            ValueError,
            "motor_id",
        ),
        (
            lambda: ServoConfig(
                motor=MotorModel.AK80_9,
                motor_id=1,
                can_channel=cast(str, 1),
            ),
            TypeError,
            "can_channel",
        ),
        (
            lambda: ServoConfig(
                motor=MotorModel.AK80_9,
                motor_id=1,
                can_channel=" ",
            ),
            ValueError,
            "can_channel",
        ),
        (
            lambda: ServoConfig(
                motor=MotorModel.AK80_9,
                motor_id=1,
                overtemperature_trip_count=0,
            ),
            ValueError,
            "overtemperature_trip_count",
        ),
        (
            lambda: ServoConfig(
                motor=MotorModel.AK80_9,
                motor_id=1,
                overtemperature_trip_count=cast(int, 1.5),
            ),
            TypeError,
            "integer",
        ),
        (
            lambda: ServoConfig(
                motor=MotorModel.AK80_9,
                motor_id=1,
                overtemperature_trip_count=cast(int, math.nan),
            ),
            TypeError,
            "integer",
        ),
        (
            lambda: ServoConfig(
                motor=MotorModel.AK80_9,
                motor_id=1,
                overtemperature_trip_count=True,
            ),
            TypeError,
            "integer",
        ),
        (
            lambda: ServoConfig(
                motor=MotorModel.AK80_9,
                motor_id=1,
                max_driver_temperature_celsius=cast(float, True),
            ),
            TypeError,
            "max_driver_temperature_celsius",
        ),
        (
            lambda: ServoConfig(
                motor=MotorModel.AK80_9,
                motor_id=1,
                max_driver_temperature_celsius=math.nan,
            ),
            ValueError,
            "max_driver_temperature_celsius",
        ),
        (
            lambda: ServoConfig(
                motor=MotorModel.AK80_9,
                motor_id=1,
                cooldown_margin_celsius=-1.0,
            ),
            ValueError,
            "cooldown_margin_celsius",
        ),
        (
            lambda: ServoConfig(
                motor=MotorModel.AK80_9,
                motor_id=1,
                connection_timeout_seconds=0.0,
            ),
            ValueError,
            "connection_timeout_seconds",
        ),
        (
            lambda: ServoConfig(
                motor=MotorModel.AK80_9,
                motor_id=1,
                telemetry_timeout_seconds=0.0,
            ),
            ValueError,
            "telemetry_timeout_seconds",
        ),
        (
            lambda: ServoConfig(
                motor=MotorModel.AK80_9,
                motor_id=1,
                max_current_amps=cast(float, True),
            ),
            TypeError,
            "max_current_amps",
        ),
        (
            lambda: ServoConfig(
                motor=MotorModel.AK80_9,
                motor_id=1,
                max_current_amps=math.inf,
            ),
            ValueError,
            "max_current_amps",
        ),
        (
            lambda: ServoConfig(
                motor=MotorModel.AK80_9,
                motor_id=1,
                max_output_torque_newton_meters=23.0,
            ),
            ValueError,
            "hardware maximum",
        ),
    ],
)
def test_servo_config_rejects_invalid_runtime_settings(
    factory: Callable[[], ServoConfig],
    error_type: type[Exception],
    message: str,
) -> None:
    """Reject invalid addressing, safety, and timing settings."""
    with pytest.raises(error_type, match=message):
        factory()
