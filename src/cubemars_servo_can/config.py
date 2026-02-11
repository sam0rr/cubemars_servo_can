from dataclasses import dataclass
from typing import Dict, Any, Optional


@dataclass
class MotorConfig:
    """
    Configuration for a specific motor type.
    All fields are mandatory to ensure safe operation.
    """

    # Position limits (rad or deg, depending on context)
    # The original library uses int32 for position, where 32000 corresponds to ~3200 degrees.
    # Note: These values are checked against the absolute value of the position command.
    P_min: float
    P_max: float

    # Velocity limits (RPM electrical speed or similar)
    # The original library uses these values to clamp velocity commands.
    # Electrical RPM = Mechanical RPM * Pole Pairs
    V_min: float
    V_max: float

    # Current limits (Amps)
    # Note: The controller typically has a hard limit (e.g. 60A), but these software limits
    # are set lower for safety (e.g. 15A).
    # Checked against the command current.
    Curr_min: float
    Curr_max: float

    # Torque limits (Nm)
    # Used to clamp torque commands.
    T_min: float
    T_max: float

    # Motor Constants
    # Kt_TMotor: Torque constant provided by T-Motor website (actually 1/Kvll)
    # Current_Factor: Calibration factor for current control (Default: 0.59)
    # Kt_actual: The actual torque constant used for calculations
    Kt_TMotor: float
    Current_Factor: float
    Kt_actual: float

    # Gearbox and Pole Pairs
    # GEAR_RATIO: The mechanical gear reduction ratio (e.g. 9 for 9:1 reduction)
    # NUM_POLE_PAIRS: Number of magnet pole pairs in the rotor
    GEAR_RATIO: float
    NUM_POLE_PAIRS: int

    # Usage flags
    # Use_derived_torque_constants: Set to True if you have a better model for torque calculation.
    # Default is False (uses simple Kt * Current model).
    Use_derived_torque_constants: bool

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "MotorConfig":
        """
        Create a config object from a dictionary.
        Raises TypeError or KeyError if required fields are missing.
        """
        # Filter strictly for known fields, but then allow standard dataclass validation
        # to raise errors if anything is missing.
        known_fields = cls.__annotations__.keys()

        # Check for missing keys
        missing = [key for key in known_fields if key not in data]
        if missing:
            raise ValueError(
                f"Invalid Motor Configuration. Missing required fields: {missing}"
            )

        filtered_data = {k: v for k, v in data.items() if k in known_fields}
        return cls(**filtered_data)


# Default definitions for known motors
# Values are taken from the original TMotorCANControl library
DEFAULTS: Dict[str, Dict[str, Any]] = {
    "AK10-9": {
        "P_min": -32000.0,  # -3200 deg
        "P_max": 32000.0,  # 3200 deg
        "V_min": -100000.0,  # -100000 rpm electrical speed
        "V_max": 100000.0,  # 100000 rpm electrical speed
        "Curr_min": -1500.0,  # -60A is the acutal limit but set to -15A
        "Curr_max": 1500.0,  # 60A is the acutal limit but set to 15A
        "T_min": -15.0,  # NM
        "T_max": 15.0,  # NM
        "Kt_TMotor": 0.16,  # from TMotor website (actually 1/Kvll)
        "Current_Factor": 0.59,  # UNTESTED CONSTANT!
        "Kt_actual": 0.206,  # UNTESTED CONSTANT!
        "GEAR_RATIO": 9.0,
        "NUM_POLE_PAIRS": 21,  # Assumed based on AK80-9 similarity
        "Use_derived_torque_constants": False,  # true if you have a better model
    },
    "AK80-9": {
        "P_min": -32000.0,  # -3200 deg
        "P_max": 32000.0,  # 3200 deg
        "V_min": -32000.0,  # -320000 rpm electrical speed
        "V_max": 32000.0,  # 320000 rpm electrical speed
        "Curr_min": -1500.0,  # -60A is the acutal limit but set to -15A
        "Curr_max": 1500.0,  # 60A is the acutal limit but set to 15A
        "T_min": -30.0,  # NM
        "T_max": 30.0,  # NM
        "Kt_TMotor": 0.091,  # from TMotor website (actually 1/Kvll)
        "Current_Factor": 0.59,
        "Kt_actual": 0.115,
        "GEAR_RATIO": 9.0,
        "NUM_POLE_PAIRS": 21,
        "Use_derived_torque_constants": False,  # true if you have a better model
    },
    "AK40-10": {
        "P_min": -32000.0,  # -3200 deg
        "P_max": 32000.0,  # 3200 deg
        "V_min": -60000.0,  # -60000 rpm electrical speed
        "V_max": 60000.0,  # 60000 rpm electrical speed
        "Curr_min": -1500.0,  # -15A safe limit
        "Curr_max": 1500.0,  # 15A safe limit
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

    Raises:
        ValueError: If the motor type is unknown and no valid custom config is provided,
                    or if the resulting configuration is missing required fields.
    """
    # Start with the default config for the type
    if motor_type in DEFAULTS:
        base_data = DEFAULTS[motor_type].copy()
    elif custom_config:
        # If it's a new motor type, start empty and rely on custom_config to fill ALL fields
        base_data = {}
    else:
        raise ValueError(
            f"Unknown motor type '{motor_type}' and no custom config provided."
        )

    # Apply overrides if provided
    if custom_config:
        base_data.update(custom_config)

    return MotorConfig.from_dict(base_data)
