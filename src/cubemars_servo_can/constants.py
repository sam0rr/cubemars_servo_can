from enum import IntEnum
from typing import Dict, List, Final


class ControlMode(IntEnum):
    """
    Control states for the servo motor.
    Using IntEnum allows these to be directly used in integer comparisons/bitwise operations if needed.
    """

    DUTY_CYCLE = 0
    CURRENT_LOOP = 1
    CURRENT_BRAKE = 2
    VELOCITY = 3
    POSITION = 4
    SET_ORIGIN = 5
    POSITION_VELOCITY = 6
    IDLE = 7


# CAN Command IDs
# These are the sub-indices used in the CAN arbitration ID to specify the command type.
CAN_PACKET_ID: Final[Dict[str, int]] = {
    "SET_DUTY": 0,  # Duty cycle mode
    "SET_CURRENT": 1,  # Current loop mode
    "SET_CURRENT_BRAKE": 2,  # Current brake mode
    "SET_RPM": 3,  # Velocity mode
    "SET_POS": 4,  # Position loop mode
    "SET_ORIGIN_HERE": 5,  # Set origin mode
    "SET_POS_SPD": 6,  # Position velocity loop mode
}

# Error Codes
# Mapping from the motor's error byte to a human-readable description.
ERROR_CODES: Final[Dict[int, str]] = {
    0: "No Error",
    1: "Over temperature fault",
    2: "Over current fault",
    3: "Over voltage fault",
    4: "Under voltage fault",
    5: "Encoder fault",
    6: "Phase current unbalance fault (The hardware may be damaged)",
}

# Default variables to be logged to CSV
DEFAULT_LOG_VARIABLES: Final[List[str]] = [
    "motor_position",
    "motor_speed",
    "motor_current",
    "motor_temperature",
]
