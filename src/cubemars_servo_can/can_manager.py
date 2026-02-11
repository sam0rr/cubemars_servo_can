import can
import os
import numpy as np
from typing import List, Optional, TYPE_CHECKING

from .constants import CAN_PACKET_ID
from .motor_state import ServoMotorState
from .utils import buffer_append_int16, buffer_append_int32

if TYPE_CHECKING:
    from .servo_can import CubeMarsServoCAN


class MotorListener(can.Listener):
    """
    Python-can listener object, with handler to be called upon reception of a message on the CAN bus.
    """

    def __init__(self, canman: "CAN_Manager_servo", motor: "CubeMarsServoCAN") -> None:
        """
        Sets stores can manager and motor object references.

        Args:
            canman: The CanManager object to get messages from
            motor: The CubeMarsServoCAN object to update
        """
        self.canman = canman
        self.bus = canman.bus
        self.motor = motor

    def on_message_received(self, msg: can.Message) -> None:
        """
        Updates this listener's motor with the info contained in msg, if that message was for this motor.

        Args:
            msg: A python-can CAN message
        """
        data = bytes(msg.data)
        ID = msg.arbitration_id & 0x00000FF
        if ID == self.motor.ID:
            self.motor._update_state_async(self.canman.parse_servo_message(data))


class CAN_Manager_servo(object):
    """
    A class to manage the low level CAN communication protocols.
    This class is a Singleton to ensure only one CAN bus connection exists.
    """

    debug: bool = False
    """
    Set to true to display every message sent and recieved for debugging.
    """

    _instance: Optional["CAN_Manager_servo"] = None
    """
    Used to keep track of one instantiation of the class to make a singleton object.
    """

    # Instance attributes for type checking
    channel: str
    bus: can.interface.Bus
    notifier: can.Notifier

    def __new__(cls, channel: str = "can0") -> "CAN_Manager_servo":
        """
        Makes a singleton object to manage a socketcan_native CAN bus.

        Args:
            channel: The CAN channel name (e.g. 'can0', 'vcan0')
        """
        if cls._instance is None:
            cls._instance = super(CAN_Manager_servo, cls).__new__(cls)
            print("Initializing CAN Manager")

            # Save channel for later use
            cls._instance.channel = channel

            # Verify the CAN bus is currently down
            # Note: Requires sudo privileges
            os.system(f"sudo /sbin/ip link set {channel} down")
            # Start the CAN bus back up with high bitrate
            os.system(f"sudo /sbin/ip link set {channel} up type can bitrate 1000000")

            # Create a python-can bus object
            cls._instance.bus = can.interface.Bus(channel=channel, bustype="socketcan")
            # Create a python-can notifier object, which motors can later subscribe to
            cls._instance.notifier = can.Notifier(bus=cls._instance.bus, listeners=[])
            print("Connected on: " + str(cls._instance.bus))

        return cls._instance

    def __init__(self, channel: str = "can0") -> None:
        """
        All initialization happens in __new__
        """
        pass

    def __del__(self) -> None:
        """
        Shut down the CAN bus when the object is deleted.
        """
        if hasattr(self, "channel"):
            os.system(f"sudo /sbin/ip link set {self.channel} down")

    def add_motor(self, motor: "CubeMarsServoCAN") -> None:
        """
        Subscribe a motor object to the CAN bus to be updated upon message reception.

        Args:
            motor: The CubeMarsServoCAN object to be subscribed to the notifier
        """
        self.notifier.add_listener(MotorListener(self, motor))

    def send_servo_message(self, motor_id: int, data: List[int], data_len: int) -> None:
        """
        Sends a Servo Mode message to the motor, with a header of motor_id and data array of data.

        Args:
            motor_id: The CAN ID of the motor to send to.
            data: An array of integers or bytes of data to send.
            data_len: Length of the data array (usually 0 for these servo commands as data is in buffer)
        """
        DLC = data_len
        assert DLC <= 8, f"Data too long in message for motor {motor_id}"

        if self.debug:
            print(f"ID: {hex(motor_id)}   Data: [{', '.join(hex(d) for d in data)}]")

        message = can.Message(arbitration_id=motor_id, data=data, is_extended_id=True)

        try:
            self.bus.send(message)
            if self.debug:
                print(f"    Message sent on {self.bus.channel_info}")
        except can.CanError as e:
            if self.debug:
                print(f"    Message NOT sent: {e}")

    def power_on(self, motor_id: int) -> None:
        """
        Sends the power on code to motor_id.

        Args:
            motor_id: The CAN ID of the motor to send the message to.
        """
        self.send_servo_message(
            motor_id, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC], 0
        )

    def power_off(self, motor_id: int) -> None:
        """
        Sends the power off code to motor_id.

        Args:
            motor_id: The CAN ID of the motor to send the message to.
        """
        self.send_servo_message(
            motor_id, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD], 0
        )

    # --- Mode Setters ---

    def comm_can_set_duty(self, controller_id: int, duty: float) -> None:
        """
        Send a servo control message for duty cycle mode.
        Duty cycle mode: duty cycle voltage is specified for a given motor.

        Args:
            controller_id: CAN ID of the motor to send the message to
            duty: duty cycle (-1 to 1) to use
        """
        buffer: List[int] = []
        buffer_append_int32(buffer, int(duty * 100000.0))
        self.send_servo_message(
            controller_id | (CAN_PACKET_ID["SET_DUTY"] << 8),
            buffer,
            0,
        )

    def comm_can_set_current(self, controller_id: int, current: float) -> None:
        """
        Send a servo control message for current loop mode.
        Current loop mode: given the Iq current specified by the motor, the motor output torque = Iq * KT.

        Args:
            controller_id: CAN ID of the motor to send the message to
            current: current in Amps to use (-60 to 60)
        """
        buffer: List[int] = []
        buffer_append_int32(buffer, int(current * 1000.0))
        self.send_servo_message(
            controller_id | (CAN_PACKET_ID["SET_CURRENT"] << 8),
            buffer,
            0,
        )

    def comm_can_set_cb(self, controller_id: int, current: float) -> None:
        """
        Send a servo control message for current brake mode.
        Current brake mode: the motor is fixed at the current position by the specified brake current.

        Args:
            controller_id: CAN ID of the motor to send the message to
            current: current in Amps to use (0 to 60)
        """
        buffer: List[int] = []
        buffer_append_int32(buffer, int(current * 1000.0))
        self.send_servo_message(
            controller_id | (CAN_PACKET_ID["SET_CURRENT_BRAKE"] << 8),
            buffer,
            0,
        )

    def comm_can_set_rpm(self, controller_id: int, rpm: float) -> None:
        """
        Send a servo control message for velocity control mode.
        Velocity mode: the speed specified by the given motor.

        Args:
            controller_id: CAN ID of the motor to send the message to
            rpm: velocity in ERPM (-100000 to 100000)
        """
        buffer: List[int] = []
        buffer_append_int32(buffer, int(rpm))
        self.send_servo_message(
            controller_id | (CAN_PACKET_ID["SET_RPM"] << 8),
            buffer,
            0,
        )

    def comm_can_set_pos(self, controller_id: int, pos: float) -> None:
        """
        Send a servo control message for position control mode.
        Position mode: Given the specified position of the motor, the motor will run to the specified position.

        Args:
            controller_id: CAN ID of the motor to send the message to
            pos: desired position in degrees (sent as int32 * 1000000)
        """
        buffer: List[int] = []
        buffer_append_int32(buffer, int(pos * 1000000.0))
        self.send_servo_message(
            controller_id | (CAN_PACKET_ID["SET_POS"] << 8),
            buffer,
            0,
        )

    def comm_can_set_origin(self, controller_id: int, set_origin_mode: int) -> None:
        """
        Set the origin.

        Args:
            controller_id: CAN ID of the motor to send the message to
            set_origin_mode:
                0: setting the temporary origin (power failure elimination)
                1: setting the permanent zero point (automatic parameter saving)
                2: restoring the default zero point (automatic parameter saving)
        """
        buffer = [set_origin_mode]
        self.send_servo_message(
            controller_id | (CAN_PACKET_ID["SET_ORIGIN_HERE"] << 8),
            buffer,
            0,
        )

    def comm_can_set_pos_spd(
        self, controller_id: int, pos: float, spd: float, RPA: float
    ) -> None:
        """
        Send a servo control message for position control mode, with specified velocity and acceleration.
        This will be a trapezoidal speed profile.

        Args:
            controller_id: CAN ID of the motor to send the message to
            pos: desired position
            spd: desired max speed in ERPM
            RPA: desired acceleration
        """
        buffer: List[int] = []
        buffer_append_int32(buffer, int(pos * 10000.0))
        buffer_append_int16(buffer, int(spd))
        buffer_append_int16(buffer, int(RPA))
        self.send_servo_message(
            controller_id | (CAN_PACKET_ID["SET_POS_SPD"] << 8),
            buffer,
            0,
        )

    def parse_servo_message(self, data: bytes) -> ServoMotorState:
        """
        Unpack the servo message into a ServoMotorState object.

        Args:
            data: bytes of the message to be processed

        Returns:
            A ServoMotorState object representing the state based on the data received.
        """
        # using numpy to convert signed/unsigned integers
        pos_int = np.int16(data[0] << 8 | data[1])
        spd_int = np.int16(data[2] << 8 | data[3])
        cur_int = np.int16(data[4] << 8 | data[5])
        motor_pos = float(pos_int * 0.1)  # motor position
        motor_spd = float(spd_int * 10.0)  # motor speed
        motor_cur = float(cur_int * 0.01)  # motor current
        motor_temp = float(np.int16(data[6]))  # motor temperature
        motor_error = int(data[7])  # motor error mode

        if self.debug:
            print(f"  Position: {motor_pos}")
            print(f"  Velocity: {motor_spd}")
            print(f"  Current: {motor_cur}")
            print(f"  Temp: {motor_temp}")
            print(f"  Error: {motor_error}")

        return ServoMotorState(
            motor_pos, motor_spd, motor_cur, motor_temp, motor_error, 0.0
        )
