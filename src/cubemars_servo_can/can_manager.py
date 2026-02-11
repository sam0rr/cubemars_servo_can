import can
import subprocess
import numpy as np
from typing import List, Optional, TYPE_CHECKING, Dict

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
    _listeners: Dict[int, MotorListener]
    _closed: bool

    def __new__(
        cls,
        channel: str = "can0",
        auto_configure: bool = False,
        bitrate: int = 1000000,
    ) -> "CAN_Manager_servo":
        """
        Makes a singleton object to manage a socketcan_native CAN bus.

        Args:
            channel: The CAN channel name (e.g. 'can0', 'vcan0')
            auto_configure: Whether to run socketcan setup commands before opening the bus.
            bitrate: Bitrate used when auto-configuring socketcan.
        """
        if cls._instance is None:
            cls._instance = super(CAN_Manager_servo, cls).__new__(cls)
            print("Initializing CAN Manager")

            # Save channel for later use
            cls._instance.channel = channel

            if auto_configure:
                cls.configure_socketcan(channel=channel, bitrate=bitrate)

            # Create a python-can bus object
            cls._instance.bus = can.interface.Bus(channel=channel, bustype="socketcan")
            # Create a python-can notifier object, which motors can later subscribe to
            cls._instance.notifier = can.Notifier(bus=cls._instance.bus, listeners=[])
            cls._instance._listeners = {}
            cls._instance._closed = False
            print("Connected on: " + str(cls._instance.bus))

        elif channel != cls._instance.channel:
            raise RuntimeError(
                f"CAN manager already initialized on '{cls._instance.channel}', cannot reinitialize on '{channel}'"
            )

        return cls._instance

    def __init__(
        self,
        channel: str = "can0",
        auto_configure: bool = False,
        bitrate: int = 1000000,
    ) -> None:
        """
        All initialization happens in __new__
        """
        pass

    @staticmethod
    def configure_socketcan(
        channel: str = "can0", bitrate: int = 1000000, ip_tool: str = "/sbin/ip"
    ) -> None:
        """
        Configure a socketcan interface.

        This helper does not use sudo. Callers should run it with appropriate privileges.
        """
        try:
            subprocess.run([ip_tool, "link", "set", channel, "down"], check=True)
            subprocess.run(
                [ip_tool, "link", "set", channel, "up", "type", "can", "bitrate", str(bitrate)],
                check=True,
            )
        except FileNotFoundError as e:
            raise RuntimeError(f"Cannot find ip tool at '{ip_tool}'") from e
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"Failed to configure socketcan interface '{channel}'") from e

    def __del__(self) -> None:
        """
        Attempt to shut down resources when the object is deleted.
        """
        try:
            self.close()
        except Exception:
            # Never raise during GC.
            pass

    def close(self) -> None:
        """
        Stop notifier and shut down the CAN bus deterministically.
        """
        if not hasattr(self, "_closed") or self._closed:
            return

        if hasattr(self, "notifier"):
            self.notifier.stop()
        if hasattr(self, "bus"):
            self.bus.shutdown()

        self._listeners.clear()
        self._closed = True

        if CAN_Manager_servo._instance is self:
            CAN_Manager_servo._instance = None

    def add_motor(self, motor: "CubeMarsServoCAN") -> MotorListener:
        """
        Subscribe a motor object to the CAN bus to be updated upon message reception.

        Args:
            motor: The CubeMarsServoCAN object to be subscribed to the notifier
        """
        listener = MotorListener(self, motor)
        self.notifier.add_listener(listener)
        self._listeners[id(motor)] = listener
        return listener

    def remove_motor(self, motor: "CubeMarsServoCAN") -> None:
        """
        Remove a previously registered motor listener from the notifier.
        """
        listener = self._listeners.pop(id(motor), None)
        if listener is not None:
            self.notifier.remove_listener(listener)

    def send_servo_message(self, motor_id: int, data: List[int], data_len: int) -> None:
        """
        Sends a Servo Mode message to the motor, with a header of motor_id and data array of data.

        Args:
            motor_id: The CAN ID of the motor to send to.
            data: An array of integers or bytes of data to send.
            data_len: Length of the data array (usually 0 for these servo commands as data is in buffer)
        """
        # Validate data length - CAN messages can only have 0-8 bytes
        actual_len = len(data)
        if actual_len > 8:
            raise ValueError(
                f"Data buffer has {actual_len} bytes, but CAN max is 8 for motor {motor_id}"
            )

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
            raise RuntimeError(
                f"Failed to send CAN message to motor {motor_id}: {e}"
            ) from e

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
        Duty cycle mode: duty cycle voltage is specified for a given motor, similar to squarewave drive mode.

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
        Current loop mode: given the Iq current specified by the motor, the motor output torque = Iq * KT, so it can be used as a torque loop.

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
        Current brake mode: the motor is fixed at the current position by the specified brake current given by the motor (pay attention to the motor temperature when using).

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
        Position mode: Given the specified position of the motor, the motor will run to the specified position, (default speed 12000erpm acceleration 40000erpm).

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
        if len(data) != 8:
            raise ValueError(
                f"Servo status frame must be exactly 8 bytes, got {len(data)}"
            )

        # using numpy to convert signed/unsigned integers
        pos_int = np.int16(data[0] << 8 | data[1])
        spd_int = np.int16(data[2] << 8 | data[3])
        cur_int = np.int16(data[4] << 8 | data[5])
        motor_pos = float(pos_int * 0.1)  # motor position
        motor_spd = float(spd_int * 10.0)  # motor speed
        motor_cur = float(cur_int * 0.01)  # motor current
        motor_temp = float(np.uint8(data[6]))  # motor temperature (unsigned 8-bit)
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
