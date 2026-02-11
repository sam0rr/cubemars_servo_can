from dataclasses import dataclass
from typing import Dict, Any, Optional


@dataclass
class MotorConfig:
    """Configuration for a specific motor type."""

    P_min: float = -12.5
    P_max: float = 12.5
    V_min: float = -50.0
    V_max: float = 50.0
    Curr_min: float = -15.0
    Curr_max: float = 15.0
    T_min: float = -15.0
    T_max: float = 15.0
    Kt_TMotor: float = 0.1
    Current_Factor: float = 0.59
    Kt_actual: float = 0.1
    GEAR_RATIO: float = 1.0
    NUM_POLE_PAIRS: int = 21
    Use_derived_torque_constants: bool = False

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "MotorConfig":
        """Create a config object from a dictionary, filtering out unknown keys."""
        known_fields = cls.__annotations__.keys()
        filtered_data = {k: v for k, v in data.items() if k in known_fields}
        return cls(**filtered_data)


# Default definitions for known motors
DEFAULTS: Dict[str, Dict[str, Any]] = {
    "AK10-9": {
        "P_min": -32000,
        "P_max": 32000,
        "V_min": -100000,
        "V_max": 100000,
        "Curr_min": -1500,
        "Curr_max": 1500,
        "T_min": -15,
        "T_max": 15,
        "Kt_TMotor": 0.16,
        "Current_Factor": 0.59,
        "Kt_actual": 0.206,
        "GEAR_RATIO": 9.0,
        "NUM_POLE_PAIRS": 21,
        "Use_derived_torque_constants": False,
    },
    "AK80-9": {
        "P_min": -32000,
        "P_max": 32000,
        "V_min": -32000,
        "V_max": 32000,
        "Curr_min": -1500,
        "Curr_max": 1500,
        "T_min": -30,
        "T_max": 30,
        "Kt_TMotor": 0.091,
        "Current_Factor": 0.59,
        "Kt_actual": 0.115,
        "GEAR_RATIO": 9.0,
        "NUM_POLE_PAIRS": 21,
        "Use_derived_torque_constants": False,
    },
    "AK40-10": {
        "P_min": -32000,
        "P_max": 32000,
        "V_min": -60000,
        "V_max": 60000,
        "Curr_min": -1500,
        "Curr_max": 1500,
        "T_min": -19.6,
        "T_max": 19.6,
        "Kt_TMotor": 0.056,
        "Current_Factor": 0.59,
        "Kt_actual": 0.071,
        "GEAR_RATIO": 10.0,
        "NUM_POLE_PAIRS": 7,
        "Use_derived_torque_constants": False,
    },
}


def get_motor_config(
    motor_type: str, custom_config: Optional[Dict[str, Any]] = None
) -> MotorConfig:
    """Retrieve the configuration for a motor with optional overrides."""
    base_data = DEFAULTS.get(motor_type, {}).copy()

    if custom_config:
        base_data.update(custom_config)

    return MotorConfig.from_dict(base_data)
