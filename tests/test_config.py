"""Tests for immutable validated configuration."""

from dataclasses import replace
from pathlib import Path
from typing import cast

import pytest

from cubemars_servo_can import (
    LogField,
    MotorConfig,
    MotorModel,
    ServoConfig,
    get_motor_config,
)


def make_invalid_motor_config(case: str, base: MotorConfig) -> MotorConfig:
    """Build one deliberately invalid motor configuration."""
    if case == "model":
        return replace(base, model_name=" ")
    if case == "position":
        return replace(
            base,
            min_output_position_radians=1.0,
            max_output_position_radians=1.0,
        )
    if case == "velocity":
        return replace(base, min_velocity_erpm=1.0, max_velocity_erpm=0.0)
    if case == "current":
        return replace(base, min_current_amps=1.0, max_current_amps=0.0)
    if case == "torque":
        return replace(
            base,
            min_output_torque_newton_meters=1.0,
            max_output_torque_newton_meters=0.0,
        )
    if case == "constant":
        return replace(base, effective_torque_constant_newton_meters_per_amp=0.0)
    if case == "gear":
        return replace(base, gear_ratio=0.0)
    return replace(base, pole_pairs=0)


def make_invalid_servo_config(case: str, config: ServoConfig) -> ServoConfig:
    """Build one deliberately invalid runtime configuration."""
    if case == "channel":
        return replace(config, can_channel=" ")
    if case == "trips":
        return replace(config, overtemperature_trip_count=0)
    if case == "margin":
        return replace(config, cooldown_margin_celsius=-0.1)
    if case == "timeout":
        return replace(config, connection_timeout_seconds=0.0)
    if case == "probes":
        return replace(config, connection_probe_count=0)
    return replace(
        config,
        log_fields=(
            LogField.TEMPERATURE_CELSIUS,
            LogField.TEMPERATURE_CELSIUS,
        ),
    )


def test_all_builtin_models_have_safe_coherent_limits() -> None:
    """Return one immutable configuration for every public model."""
    configs = [get_motor_config(model) for model in MotorModel]
    assert [config.model_name for config in configs] == [
        model.value for model in MotorModel
    ]
    assert get_motor_config(MotorModel.AK40_10).max_current_amps == 7.3
    assert get_motor_config(MotorModel.AKA60_6).max_current_amps == 11.2
    assert get_motor_config(MotorModel.AK80_9).max_output_torque_newton_meters == 15.0
    assert get_motor_config(MotorModel.AK10_9).pole_pairs == 21


def test_get_motor_config_rejects_strings() -> None:
    """Require the new typed enum instead of preserving string aliases."""
    with pytest.raises(TypeError, match="MotorModel"):
        get_motor_config(cast(MotorModel, "AK80-9"))


@pytest.mark.parametrize(
    ("case", "message"),
    [
        ("model", "model_name"),
        ("position", "output position"),
        ("velocity", "velocity"),
        ("current", "current"),
        ("torque", "output torque"),
        ("constant", "Torque constant"),
        ("gear", "gear_ratio"),
        ("poles", "pole_pairs"),
    ],
)
def test_motor_config_validates_every_invariant(case: str, message: str) -> None:
    """Reject every unsafe custom motor configuration invariant."""
    base = get_motor_config(MotorModel.AK80_9)
    with pytest.raises(ValueError, match=message):
        make_invalid_motor_config(case, base)


def test_custom_motor_config_is_supported_directly() -> None:
    """Let callers create explicit typed configurations without dictionary loaders."""
    config = replace(
        get_motor_config(MotorModel.AK80_9),
        model_name="CUSTOM",
        supports_persistent_origin=False,
    )
    assert config.model_name == "CUSTOM"
    assert not config.supports_persistent_origin


def test_servo_config_defaults_are_typed_and_immutable() -> None:
    """Provide safe runtime defaults with an immutable logging selection."""
    config = ServoConfig(csv_log_path=Path("telemetry.csv"))
    assert config.can_channel == "can0"
    assert config.log_fields == tuple(LogField)
    assert config.csv_log_path == Path("telemetry.csv")


@pytest.mark.parametrize(
    ("case", "message"),
    [
        ("channel", "can_channel"),
        ("trips", "overtemperature_trip_count"),
        ("margin", "cooldown_margin_celsius"),
        ("timeout", "connection_timeout_seconds"),
        ("probes", "connection_probe_count"),
        ("logs", "duplicates"),
    ],
)
def test_servo_config_validates_every_invariant(case: str, message: str) -> None:
    """Reject invalid connection, thermal, and logging settings."""
    config = ServoConfig()
    with pytest.raises(ValueError, match=message):
        make_invalid_servo_config(case, config)
