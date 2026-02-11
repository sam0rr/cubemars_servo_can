import pytest
from cubemars_servo_can.config import get_motor_config, MotorConfig


def test_default_ak80_9():
    config = get_motor_config("AK80-9")
    assert config.GEAR_RATIO == 9.0
    assert config.P_max == 32000.0
    assert config.Kt_actual == 0.115


def test_override_config():
    overrides = {"GEAR_RATIO": 50.0}
    config = get_motor_config("AK80-9", custom_config=overrides)
    assert config.GEAR_RATIO == 50.0
    assert config.P_max == 32000.0  # Should remain default


def test_custom_motor_success():
    custom_specs = {
        "P_min": -100.0,
        "P_max": 100.0,
        "V_min": -100.0,
        "V_max": 100.0,
        "Curr_min": -10.0,
        "Curr_max": 10.0,
        "T_min": -5.0,
        "T_max": 5.0,
        "Kt_TMotor": 0.1,
        "Current_Factor": 0.5,
        "Kt_actual": 0.1,
        "GEAR_RATIO": 1.0,
        "NUM_POLE_PAIRS": 10,
        "Use_derived_torque_constants": False,
    }
    config = get_motor_config("Custom", custom_config=custom_specs)
    assert config.P_max == 100.0


def test_custom_motor_missing_field():
    # Missing GEAR_RATIO
    custom_specs = {
        "P_min": -100.0,
        "P_max": 100.0,
        "V_min": -100.0,
        "V_max": 100.0,
        "Curr_min": -10.0,
        "Curr_max": 10.0,
        "T_min": -5.0,
        "T_max": 5.0,
        "Kt_TMotor": 0.1,
        "Current_Factor": 0.5,
        "Kt_actual": 0.1,
        # "GEAR_RATIO": 1.0, # MISSING
        "NUM_POLE_PAIRS": 10,
        "Use_derived_torque_constants": False,
    }
    with pytest.raises(ValueError, match="Missing required fields"):
        get_motor_config("Custom", custom_config=custom_specs)


def test_unknown_motor_no_config():
    with pytest.raises(ValueError, match="Unknown motor type"):
        get_motor_config("UnknownMotor")
