from dataclasses import dataclass
from typing import Dict, Any, Optional


@dataclass
class MotorConfig:
    """
    Configuration for a specific motor type.
    """

    # Position limits (rad or deg, depending on context)
    # Original library sent these as int32, so -32000 corresponds to -3200 deg in some contexts
    P_min: float = -12.5  # -3200 deg
    P_max: float = 12.5  # 3200 deg

    # Velocity limits (RPM electrical speed or similar)
    V_min: float = -50.0  # -100000 rpm electrical speed
    V_max: float = 50.0  # 100000 rpm electrical speed

    # Current limits (Amps)
    Curr_min: float = -15.0  # -60A is the actual limit but set to -15A for safety
    Curr_max: float = 15.0  # 60A is the actual limit but set to 15A for safety

    # Torque limits (Nm)
    T_min: float = -15.0  # NM
    T_max: float = 15.0  # NM

    # Motor Constants
    Kt_TMotor: float = 0.1  # from TMotor website (actually 1/Kvll)
    Current_Factor: float = 0.59  # UNTESTED CONSTANT!
    Kt_actual: float = 0.1  # UNTESTED CONSTANT!

    # Gearbox and Pole Pairs
    GEAR_RATIO: float = 1.0
    NUM_POLE_PAIRS: int = 21

    # Usage flags
    Use_derived_torque_constants: bool = False  # true if you have a better model

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "MotorConfig":
        """
        Create a config object from a dictionary, filtering out unknown keys.
        """
        known_fields = cls.__annotations__.keys()
        filtered_data = {k: v for k, v in data.items() if k in known_fields}
        return cls(**filtered_data)


# Default definitions for known motors
# Values are taken from the original TMotorCANControl library
DEFAULTS: Dict[str, Dict[str, Any]] = {
    "AK10-9": {
        "P_min": -32000,  # -3200 deg
        "P_max": 32000,  # 3200 deg
        "V_min": -100000,  # -100000 rpm electrical speed
        "V_max": 100000,  # 100000 rpm electrical speed
        "Curr_min": -1500,  # -60A is the acutal limit but set to -15A
        "Curr_max": 1500,  # 60A is the acutal limit but set to 15A
        "T_min": -15,  # NM
        "T_max": 15,  # NM
        "Kt_TMotor": 0.16,  # from TMotor website (actually 1/Kvll)
        "Current_Factor": 0.59,  # UNTESTED CONSTANT!
        "Kt_actual": 0.206,  # UNTESTED CONSTANT!
        "GEAR_RATIO": 9.0,
        "NUM_POLE_PAIRS": 21,  # Assumed based on AK80-9 similarity
        "Use_derived_torque_constants": False,  # true if you have a better model
    },
    "AK80-9": {
        "P_min": -32000,  # -3200 deg
        "P_max": 32000,  # 3200 deg
        "V_min": -32000,  # -320000 rpm electrical speed
        "V_max": 32000,  # 320000 rpm electrical speed
        "Curr_min": -1500,  # -60A is the acutal limit but set to -15A
        "Curr_max": 1500,  # 60A is the acutal limit but set to 15A
        "T_min": -30,  # NM
        "T_max": 30,  # NM
        "Kt_TMotor": 0.091,  # from TMotor website (actually 1/Kvll)
        "Current_Factor": 0.59,
        "Kt_actual": 0.115,
        "GEAR_RATIO": 9.0,
        "NUM_POLE_PAIRS": 21,
        "Use_derived_torque_constants": False,  # true if you have a better model
    },
    "AK40-10": {
        "P_min": -32000,  # -3200 deg
        "P_max": 32000,  # 3200 deg
        "V_min": -60000,  # -60000 rpm electrical speed
        "V_max": 60000,  # 60000 rpm electrical speed
        "Curr_min": -1500,  # -15A safe limit
        "Curr_max": 1500,  # 15A safe limit
        "T_min": -19.6,  # NM
        "T_max": 19.6,  # NM
        "Kt_TMotor": 0.056,  # from TMotor website (actually 1/Kvll)
        "Current_Factor": 0.59,  # UNTESTED CONSTANT!
        "Kt_actual": 0.071,  # UNTESTED CONSTANT!
        "GEAR_RATIO": 10.0,
        "NUM_POLE_PAIRS": 7,
        "Use_derived_torque_constants": False,  # true if you have a better model
    },
}


def get_motor_config(
    motor_type: str, custom_config: Optional[Dict[str, Any]] = None
) -> MotorConfig:
    """
    Retrieve the configuration for a motor.

    Args:
        motor_type: The name of the motor (e.g., 'AK80-9').
        custom_config: A dictionary of overrides. If provided, these values
                       will replace the defaults.

    Returns:
        A MotorConfig object.
    """
    # Start with the default config for the type, or an empty dict if unknown
    base_data = DEFAULTS.get(motor_type, {}).copy()

    # Apply overrides if provided
    if custom_config:
        base_data.update(custom_config)

    return MotorConfig.from_dict(base_data)
