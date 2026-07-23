"""Tests for the public API."""

import dataclasses
from dataclasses import FrozenInstanceError

import pytest

import cubemars_servo_can
from cubemars_servo_can import MotorConfig, MotorModel, ServoConfig


def test_exports_are_exact_and_sorted() -> None:
    """Expose exactly the names documented for 1.0."""
    expected = [
        "CanConnectionError",
        "ControlMode",
        "ControlModeError",
        "CubeMarsServoCan",
        "CubeMarsServoCanError",
        "MotorConfig",
        "MotorConnectionError",
        "MotorFaultError",
        "MotorModel",
        "OriginMode",
        "ServoConfig",
    ]
    assert cubemars_servo_can.__all__ == expected


def test_public_dataclasses_follow_project_policy() -> None:
    """Keep all public configuration dataclasses frozen, keyword-only, and slotted."""
    assert all(field.kw_only for field in dataclasses.fields(MotorConfig))
    assert all(field.kw_only for field in dataclasses.fields(ServoConfig))
    assert hasattr(MotorConfig, "__slots__")
    assert hasattr(ServoConfig, "__slots__")
    with pytest.raises(FrozenInstanceError):
        ServoConfig(motor=MotorModel.AK80_9, motor_id=1).__setattr__(
            "can_channel",
            "changed",
        )
