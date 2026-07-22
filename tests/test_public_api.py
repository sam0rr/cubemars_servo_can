"""Tests for the public API."""

import dataclasses
from dataclasses import FrozenInstanceError

import pytest

import cubemars_servo_can
from cubemars_servo_can import MotorConfig, ServoConfig


def test_exports_are_exact_and_sorted() -> None:
    """Expose only the forward-only names documented for 1.0."""
    expected = [
        "CanConnectionError",
        "ControlMode",
        "ControlModeError",
        "CubeMarsServoCan",
        "CubeMarsServoCanError",
        "LogField",
        "MotorConfig",
        "MotorConnectionError",
        "MotorFaultError",
        "MotorModel",
        "OriginMode",
        "ServoConfig",
        "get_motor_config",
    ]
    assert cubemars_servo_can.__all__ == expected
    assert not hasattr(cubemars_servo_can, "CubeMarsServoCAN")


def test_public_dataclasses_follow_project_policy() -> None:
    """Keep all public configuration dataclasses frozen, keyword-only, and slotted."""
    assert all(field.kw_only for field in dataclasses.fields(MotorConfig))
    assert all(field.kw_only for field in dataclasses.fields(ServoConfig))
    assert hasattr(MotorConfig, "__slots__")
    assert hasattr(ServoConfig, "__slots__")
    with pytest.raises(FrozenInstanceError):
        ServoConfig().__setattr__("can_channel", "changed")
