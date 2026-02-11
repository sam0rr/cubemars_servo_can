from enum import Enum


class ControlMode(Enum):
    """
    Control states for the servo motor.
    """

    DUTY_CYCLE = 0
    CURRENT_LOOP = 1
    CURRENT_BRAKE = 2
    VELOCITY = 3
    POSITION = 4
    SET_ORIGIN = 5
    POSITION_VELOCITY = 6
    IDLE = 7


CAN_PACKET_ID = {
    "SET_DUTY": 0,  # Motor runs in duty cycle mode
    "SET_CURRENT": 1,  # Motor runs in current loop mode
    "SET_CURRENT_BRAKE": 2,  # Motor current brake mode operation
    "SET_RPM": 3,  # Motor runs in velocity mode
    "SET_POS": 4,  # Motor runs in position loop mode
    "SET_ORIGIN_HERE": 5,  # Set origin mode
    "SET_POS_SPD": 6,  # Position velocity loop mode
}

ERROR_CODES = {
    0: "No Error",
    1: "Over temperature fault",
    2: "Over current fault",
    3: "Over voltage fault",
    4: "Under voltage fault",
    5: "Encoder fault",
    6: "Phase current unbalance fault (The hardware may be damaged)",
}

# Default variables to be logged
DEFAULT_LOG_VARIABLES = [
    "motor_position",
    "motor_speed",
    "motor_current",
    "motor_temperature",
]
