import can
import time
import csv
import traceback
import os
import warnings
from typing import List, Optional, Dict, Any, TextIO

import numpy as np

from .constants import CAN_PACKET_ID, ERROR_CODES, DEFAULT_LOG_VARIABLES, ControlMode
from .config import get_motor_config, MotorConfig
from .utils import buffer_append_int16, buffer_append_int32


class ServoMotorState:
    """Data structure to store and update motor states."""

    def __init__(
        self,
        position: float,
        velocity: float,
        current: float,
        temperature: float,
        error: int,
        acceleration: float,
    ) -> None:
        self.set_state(position, velocity, current, temperature, error, acceleration)

    def set_state(
        self,
        position: float,
        velocity: float,
        current: float,
        temperature: float,
        error: int,
        acceleration: float,
    ) -> None:
        self.position = position
        self.velocity = velocity
        self.current = current
        self.temperature = temperature
        self.error = error
        self.acceleration = acceleration

    def set_state_obj(self, other_motor_state: "ServoMotorState") -> None:
        self.position = other_motor_state.position
        self.velocity = other_motor_state.velocity
        self.current = other_motor_state.current
        self.temperature = other_motor_state.temperature
        self.error = other_motor_state.error
        self.acceleration = other_motor_state.acceleration

    def __str__(self) -> str:
        return f"Position: {self.position} | Velocity: {self.velocity} | Current: {self.current} | Temperature: {self.temperature} | Error: {self.error}"


class ServoCommand:
    """Data structure to store Servo command that will be sent upon update."""

    def __init__(
        self,
        position: float,
        velocity: float,
        current: float,
        duty: float,
        acceleration: float,
    ) -> None:
        self.position = position
        self.velocity = velocity
        self.current = current
        self.duty = duty
        self.acceleration = acceleration


class MotorListener(can.Listener):
    """Python-can listener object, with handler to be called upon reception of a message on the CAN bus."""

    def __init__(self, canman: "CAN_Manager_servo", motor: "CubeMarsServoCAN") -> None:
        self.canman = canman
        self.bus = canman.bus
        self.motor = motor

    def on_message_received(self, msg: can.Message) -> None:
        data = bytes(msg.data)
        ID = msg.arbitration_id & 0x00000FF
        if ID == self.motor.ID:
            self.motor._update_state_async(self.canman.parse_servo_message(data))


class CAN_Manager_servo(object):
    """A class to manage the low level CAN communication protocols."""

    debug: bool = False
    _instance: Optional["CAN_Manager_servo"] = None

    # Instance attributes for type checking
    channel: str
    bus: can.interface.Bus
    notifier: can.Notifier

    def __new__(cls, channel: str = "can0") -> "CAN_Manager_servo":
        if cls._instance is None:
            cls._instance = super(CAN_Manager_servo, cls).__new__(cls)
            print("Initializing CAN Manager")

            # Save channel for later use
            cls._instance.channel = channel

            # verify the CAN bus is currently down
            os.system(f"sudo /sbin/ip link set {channel} down")
            # start the CAN bus back up
            os.system(f"sudo /sbin/ip link set {channel} up type can bitrate 1000000")

            cls._instance.bus = can.interface.Bus(channel=channel, bustype="socketcan")
            cls._instance.notifier = can.Notifier(bus=cls._instance.bus, listeners=[])
            print("Connected on: " + str(cls._instance.bus))

        return cls._instance

    def __init__(self, channel: str = "can0") -> None:
        pass

    def __del__(self) -> None:
        if hasattr(self, "channel"):
            os.system(f"sudo /sbin/ip link set {self.channel} down")

    def add_motor(self, motor: "CubeMarsServoCAN") -> None:
        self.notifier.add_listener(MotorListener(self, motor))

    def send_servo_message(self, motor_id: int, data: List[int], data_len: int) -> None:
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
        self.send_servo_message(
            motor_id, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC], 0
        )

    def power_off(self, motor_id: int) -> None:
        self.send_servo_message(
            motor_id, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD], 0
        )

    # --- Mode Setters ---

    def comm_can_set_duty(self, controller_id: int, duty: float) -> None:
        buffer: List[int] = []
        buffer_append_int32(buffer, int(duty * 100000.0))
        self.send_servo_message(
            controller_id | (CAN_PACKET_ID["SET_DUTY"] << 8),
            buffer,
            0,
        )

    def comm_can_set_current(self, controller_id: int, current: float) -> None:
        buffer: List[int] = []
        buffer_append_int32(buffer, int(current * 1000.0))
        self.send_servo_message(
            controller_id | (CAN_PACKET_ID["SET_CURRENT"] << 8),
            buffer,
            0,
        )

    def comm_can_set_cb(self, controller_id: int, current: float) -> None:
        buffer: List[int] = []
        buffer_append_int32(buffer, int(current * 1000.0))
        self.send_servo_message(
            controller_id | (CAN_PACKET_ID["SET_CURRENT_BRAKE"] << 8),
            buffer,
            0,
        )

    def comm_can_set_rpm(self, controller_id: int, rpm: float) -> None:
        buffer: List[int] = []
        buffer_append_int32(buffer, int(rpm))
        self.send_servo_message(
            controller_id | (CAN_PACKET_ID["SET_RPM"] << 8),
            buffer,
            0,
        )

    def comm_can_set_pos(self, controller_id: int, pos: float) -> None:
        buffer: List[int] = []
        buffer_append_int32(buffer, int(pos * 1000000.0))
        self.send_servo_message(
            controller_id | (CAN_PACKET_ID["SET_POS"] << 8),
            buffer,
            0,
        )

    def comm_can_set_origin(self, controller_id: int, set_origin_mode: int) -> None:
        buffer = [set_origin_mode]
        self.send_servo_message(
            controller_id | (CAN_PACKET_ID["SET_ORIGIN_HERE"] << 8),
            buffer,
            0,
        )

    def comm_can_set_pos_spd(
        self, controller_id: int, pos: float, spd: float, RPA: float
    ) -> None:
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


class CubeMarsServoCAN:
    """The user-facing class that manages the motor."""

    def __init__(
        self,
        motor_type: str = "AK80-9",
        motor_ID: int = 1,
        max_mosfett_temp: float = 50.0,
        CSV_file: Optional[str] = None,
        log_vars: List[str] = DEFAULT_LOG_VARIABLES,
        can_channel: str = "can0",
        config_overrides: Optional[Dict[str, Any]] = None,
    ) -> None:
        """
        Args:
            motor_type: The type of motor being controlled, ie AK80-9.
            motor_ID: The CAN ID of the motor.
            max_mosfett_temp: temperature of the mosfett above which to throw an error, in Celsius
            CSV_file: A CSV file to output log info to. If None, no log will be recorded.
            log_vars: The variables to log as a python list.
            can_channel: The CAN channel to use (default "can0")
            config_overrides: Optional dictionary to override specific motor parameters.
        """
        self.type = motor_type
        self.ID = motor_ID
        self.csv_file_name = CSV_file
        self.max_temp = max_mosfett_temp

        # Load Configuration
        self.config: MotorConfig = get_motor_config(motor_type, config_overrides)

        print(f"Initializing device: {self.device_info_string()}")

        self._motor_state = ServoMotorState(0.0, 0.0, 0.0, 0.0, 0, 0.0)
        self._motor_state_async = ServoMotorState(0.0, 0.0, 0.0, 0.0, 0, 0.0)
        self._command = ServoCommand(0.0, 0.0, 0.0, 0.0, 0.0)
        self._control_state = ControlMode.IDLE

        self.radps_per_ERPM: float = 5.82e-04
        self.rad_per_Eang: float = np.pi / self.config.NUM_POLE_PAIRS

        self._entered = False
        self._start_time = time.time()
        self._last_update_time = self._start_time
        self._last_command_time: Optional[float] = None
        self._updated = False

        self.log_vars = log_vars
        self.LOG_FUNCTIONS = {
            "motor_position": self.get_motor_angle_radians,
            "motor_speed": self.get_motor_velocity_radians_per_second,
            "motor_current": self.get_current_qaxis_amps,
            "motor_temperature": self.get_temperature_celsius,
        }

        self._canman = CAN_Manager_servo(channel=can_channel)
        self._canman.add_motor(self)
        self.csv_file: Optional[TextIO] = None

    def __enter__(self) -> "CubeMarsServoCAN":
        print(f"Turning on control for device: {self.device_info_string()}")
        if self.csv_file_name is not None:
            with open(self.csv_file_name, "w") as fd:
                writer = csv.writer(fd)
                writer.writerow(["pi_time"] + self.log_vars)
            self.csv_file = open(self.csv_file_name, "a")
            # We don't need to call __enter__ on the file manually if we keep it open
            # But the original code was weird. We will just keep the handle.
            self.csv_writer = csv.writer(self.csv_file)

        self.power_on()
        self._send_command()
        self._entered = True
        if not self.check_can_connection():
            raise RuntimeError(f"Device not connected: {self.device_info_string()}")
        return self

    def __exit__(self, etype, value, tb) -> None:
        print(f"Turning off control for device: {self.device_info_string()}")
        self.power_off()

        if self.csv_file is not None:
            self.csv_file.close()

        if etype is not None:
            traceback.print_exception(etype, value, tb)

    def qaxis_current_to_TMotor_current(self, iq: float) -> float:
        return (
            iq
            * (self.config.GEAR_RATIO * self.config.Kt_TMotor)
            / self.config.Current_Factor
        )

    def _update_state_async(self, servo_state: ServoMotorState) -> None:
        if servo_state.error != 0:
            error_msg = ERROR_CODES.get(servo_state.error, "Unknown Error")
            raise RuntimeError(
                f"Driver board error for device: {self.device_info_string()}: {error_msg}"
            )

        now = time.time()
        dt = self._last_update_time - now
        self._last_update_time = now

        # Avoid division by zero if updates are instant
        if dt != 0:
            self._motor_state_async.acceleration = (
                servo_state.velocity - self._motor_state_async.velocity
            ) / dt

        self._motor_state_async.set_state_obj(servo_state)
        self._updated = True

    def update(self) -> None:
        if not self._entered:
            raise RuntimeError(
                f"Tried to update motor state before safely powering on for device: {self.device_info_string()}"
            )

        if self.get_temperature_celsius() > self.max_temp:
            raise RuntimeError(
                f"Temperature greater than {self.max_temp}C for device: {self.device_info_string()}"
            )

        now = time.time()
        if (self._last_command_time and (now - self._last_command_time) < 0.25) and (
            (now - self._last_update_time) > 0.1
        ):
            warnings.warn(
                f"State update requested but no data from motor. Delay longer after zeroing, decrease frequency, or check connection. {self.device_info_string()}",
                RuntimeWarning,
            )
        else:
            self._command_sent = False

        self._motor_state.set_state_obj(self._motor_state_async)
        # Apply Gear Ratio to position
        self._motor_state.position = self._motor_state.position / self.config.GEAR_RATIO

        self._send_command()

        if self.csv_file is not None:
            self.csv_writer.writerow(
                [self._last_update_time - self._start_time]
                + [self.LOG_FUNCTIONS[var]() for var in self.log_vars]
            )

        self._updated = False

    def _send_command(self) -> None:
        if self._control_state == ControlMode.DUTY_CYCLE:
            self._canman.comm_can_set_duty(self.ID, self._command.duty)
        elif self._control_state == ControlMode.CURRENT_LOOP:
            self._canman.comm_can_set_current(self.ID, self._command.current)
        elif self._control_state == ControlMode.CURRENT_BRAKE:
            self._canman.comm_can_set_cb(self.ID, self._command.current)
        elif self._control_state == ControlMode.VELOCITY:
            self._canman.comm_can_set_rpm(self.ID, self._command.velocity)
        elif self._control_state == ControlMode.POSITION:
            self._canman.comm_can_set_pos(self.ID, self._command.position)
        elif self._control_state == ControlMode.POSITION_VELOCITY:
            self._canman.comm_can_set_pos_spd(
                self.ID,
                self._command.position,
                self._command.velocity,
                self._command.acceleration,
            )
        elif self._control_state == ControlMode.IDLE:
            self._canman.comm_can_set_duty(self.ID, 0.0)
        else:
            raise RuntimeError(
                f"UNDEFINED STATE for device {self.device_info_string()}"
            )

        self._last_command_time = time.time()

    def power_on(self) -> None:
        self._canman.power_on(self.ID)
        self._updated = True

    def power_off(self) -> None:
        self._canman.power_off(self.ID)

    def set_zero_position(self) -> None:
        self._canman.comm_can_set_origin(self.ID, 1)
        self._last_command_time = time.time()

    # --- Getters ---

    def get_temperature_celsius(self) -> float:
        return self._motor_state.temperature

    def get_motor_error_code(self) -> int:
        return self._motor_state.error

    def get_current_qaxis_amps(self) -> float:
        return self._motor_state.current

    def get_output_angle_radians(self) -> float:
        return self._motor_state.position * self.rad_per_Eang

    def get_output_velocity_radians_per_second(self) -> float:
        return self._motor_state.velocity * self.radps_per_ERPM

    def get_output_acceleration_radians_per_second_squared(self) -> float:
        return self._motor_state.acceleration

    def get_output_torque_newton_meters(self) -> float:
        return (
            self.get_current_qaxis_amps()
            * self.config.Kt_actual
            * self.config.GEAR_RATIO
        )

    # --- Mode Setters ---

    def enter_duty_cycle_control(self) -> None:
        self._control_state = ControlMode.DUTY_CYCLE

    def enter_current_control(self) -> None:
        self._control_state = ControlMode.CURRENT_LOOP

    def enter_current_brake_control(self) -> None:
        self._control_state = ControlMode.CURRENT_BRAKE

    def enter_velocity_control(self) -> None:
        self._control_state = ControlMode.VELOCITY

    def enter_position_control(self) -> None:
        self._control_state = ControlMode.POSITION

    def enter_position_velocity_control(self) -> None:
        self._control_state = ControlMode.POSITION_VELOCITY

    def enter_idle_mode(self) -> None:
        self._control_state = ControlMode.IDLE

    # --- Command Setters ---

    def set_output_angle_radians(
        self, pos: float, vel: float = 0.0, acc: float = 0.0
    ) -> None:
        if np.abs(pos) >= self.config.P_max:
            raise RuntimeError(
                f"Cannot control using impedance mode for angles with magnitude greater than {self.config.P_max} rad!"
            )

        pos = pos / self.rad_per_Eang
        vel = vel / self.radps_per_ERPM
        acc = acc / self.radps_per_ERPM

        if self._control_state == ControlMode.POSITION_VELOCITY:
            self._command.position = pos
            self._command.velocity = vel
            self._command.acceleration = acc
        elif self._control_state == ControlMode.POSITION:
            self._command.position = pos
        else:
            raise RuntimeError(
                f"Attempted to send position command without entering position control {self.device_info_string()}"
            )

    def set_duty_cycle_percent(self, duty: float) -> None:
        if self._control_state != ControlMode.DUTY_CYCLE:
            raise RuntimeError(
                f"Attempted to send duty cycle command without gains for device {self.device_info_string()}"
            )
        else:
            if np.abs(duty) > 1:
                raise RuntimeError("Duty cycle cannot be greater than 100%!")
            self._command.duty = duty

    def set_output_velocity_radians_per_second(self, vel: float) -> None:
        if np.abs(vel) >= self.config.V_max:
            raise RuntimeError(
                f"Cannot control using speed mode for velocities with magnitude greater than {self.config.V_max} rad/s!"
            )

        if self._control_state != ControlMode.VELOCITY:
            raise RuntimeError(
                f"Attempted to send speed command without gains for device {self.device_info_string()}"
            )
        self._command.velocity = vel / self.radps_per_ERPM

    def set_motor_current_qaxis_amps(self, current: float) -> None:
        if self._control_state not in [
            ControlMode.CURRENT_LOOP,
            ControlMode.CURRENT_BRAKE,
        ]:
            raise RuntimeError(
                f"Attempted to send current command before entering current mode for device {self.device_info_string()}"
            )
        self._command.current = current

    def set_output_torque_newton_meters(self, torque: float) -> None:
        self.set_motor_current_qaxis_amps(
            torque / self.config.Kt_actual / self.config.GEAR_RATIO
        )

    # --- Motor-Side Wrappers ---

    def set_motor_torque_newton_meters(self, torque: float) -> None:
        self.set_output_torque_newton_meters(torque * self.config.Kt_actual)

    def set_motor_angle_radians(self, pos: float) -> None:
        self.set_output_angle_radians(pos / self.config.GEAR_RATIO, 0.0, 0.0)

    def set_motor_velocity_radians_per_second(self, vel: float) -> None:
        self.set_output_velocity_radians_per_second(vel / self.config.GEAR_RATIO)

    def get_motor_angle_radians(self) -> float:
        return self._motor_state.position * self.rad_per_Eang * self.config.GEAR_RATIO

    def get_motor_velocity_radians_per_second(self) -> float:
        return self._motor_state.velocity * self.config.GEAR_RATIO

    def get_motor_acceleration_radians_per_second_squared(self) -> float:
        return self._motor_state.acceleration * self.config.GEAR_RATIO

    def get_motor_torque_newton_meters(self) -> float:
        return self.get_output_torque_newton_meters() * self.config.GEAR_RATIO

    # --- String Representations ---

    def __str__(self) -> str:
        return (
            f"{self.device_info_string()} | "
            f"Position: {round(self.position, 3): 1f} rad | "
            f"Velocity: {round(self.velocity, 3): 1f} rad/s | "
            f"current: {round(self.current_qaxis, 3): 1f} A | "
            f"temp: {round(self.temperature, 0): 1f} C"
        )

    def device_info_string(self) -> str:
        return f"{self.type}  ID: {self.ID}"

    def check_can_connection(self) -> bool:
        if not self._entered:
            raise RuntimeError(
                "Tried to check_can_connection before entering motor control! Enter control using the __enter__ method, or instantiating the CubeMarsServoCAN in a with block."
            )
        listener = can.BufferedReader()
        self._canman.notifier.add_listener(listener)
        for i in range(10):
            self.power_on()
            time.sleep(0.001)
        return True

    # --- Properties ---

    temperature = property(
        get_temperature_celsius, doc="Temperature in Degrees Celsius"
    )

    error = property(get_motor_error_code, doc="Motor error code. 0 means no error.")

    current_qaxis = property(
        get_current_qaxis_amps,
        set_motor_current_qaxis_amps,
        doc="Q-axis current in amps",
    )

    position = property(
        get_output_angle_radians, set_output_angle_radians, doc="Output angle in rad"
    )

    velocity = property(
        get_output_velocity_radians_per_second,
        set_output_velocity_radians_per_second,
        doc="Output velocity in rad/s",
    )

    acceleration = property(
        get_output_acceleration_radians_per_second_squared,
        doc="Output acceleration in rad/s/s",
    )

    torque = property(
        get_output_torque_newton_meters,
        set_output_torque_newton_meters,
        doc="Output torque in Nm",
    )

    angle_motorside = property(
        get_motor_angle_radians, set_motor_angle_radians, doc="Motor-side angle in rad"
    )

    velocity_motorside = property(
        get_motor_velocity_radians_per_second,
        set_motor_velocity_radians_per_second,
        doc="Motor-side velocity in rad/s",
    )

    acceleration_motorside = property(
        get_motor_acceleration_radians_per_second_squared,
        doc="Motor-side acceleration in rad/s/s",
    )

    torque_motorside = property(
        get_motor_torque_newton_meters,
        set_motor_torque_newton_meters,
        doc="Motor-side torque in Nm",
    )


class ServoCommand:
    """Data structure to store Servo command that will be sent upon update"""

    def __init__(self, position, velocity, current, duty, acceleration):
        self.position = position
        self.velocity = velocity
        self.current = current
        self.duty = duty
        self.acceleration = acceleration


class MotorListener(can.Listener):
    """Python-can listener object, with handler to be called upon reception of a message on the CAN bus"""

    def __init__(self, canman, motor):
        self.canman = canman
        self.bus = canman.bus
        self.motor = motor

    def on_message_received(self, msg):
        data = bytes(msg.data)
        ID = msg.arbitration_id & 0x00000FF
        if ID == self.motor.ID:
            self.motor._update_state_async(self.canman.parse_servo_message(data))


class CAN_Manager_servo(object):
    """A class to manage the low level CAN communication protocols"""

    debug = False
    _instance = None

    def __new__(cls, channel="can0"):
        if not cls._instance:
            cls._instance = super(CAN_Manager_servo, cls).__new__(cls)
            print("Initializing CAN Manager")

            # Save channel for later use
            cls._instance.channel = channel

            # verify the CAN bus is currently down
            os.system(f"sudo /sbin/ip link set {channel} down")
            # start the CAN bus back up
            os.system(f"sudo /sbin/ip link set {channel} up type can bitrate 1000000")

            cls._instance.bus = can.interface.Bus(channel=channel, bustype="socketcan")
            cls._instance.notifier = can.Notifier(bus=cls._instance.bus, listeners=[])
            print("Connected on: " + str(cls._instance.bus))

        return cls._instance

    def __init__(self, channel="can0"):
        pass

    def __del__(self):
        if hasattr(self, "channel"):
            os.system(f"sudo /sbin/ip link set {self.channel} down")

    def add_motor(self, motor):
        self.notifier.add_listener(MotorListener(self, motor))

    def send_servo_message(self, motor_id, data, data_len):
        DLC = data_len
        assert DLC <= 8, "Data too long in message for motor " + str(motor_id)

        if self.debug:
            print(f"ID: {hex(motor_id)}   Data: [{', '.join(hex(d) for d in data)}]")

        message = can.Message(arbitration_id=motor_id, data=data, is_extended_id=True)

        try:
            self.bus.send(message)
            if self.debug:
                print("    Message sent on " + str(self.bus.channel_info))
        except can.CanError as e:
            if self.debug:
                print("    Message NOT sent: " + str(e))

    def power_on(self, motor_id):
        self.send_servo_message(
            motor_id, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC], 0
        )

    def power_off(self, motor_id):
        self.send_servo_message(
            motor_id, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD], 0
        )

    # --- Mode Setters ---

    def comm_can_set_duty(self, controller_id, duty):
        buffer = []
        buffer_append_int32(buffer, int(duty * 100000.0))
        self.send_servo_message(
            controller_id | (CAN_PACKET_ID["SET_DUTY"] << 8),
            buffer,
            0,
        )

    def comm_can_set_current(self, controller_id, current):
        buffer = []
        buffer_append_int32(buffer, int(current * 1000.0))
        self.send_servo_message(
            controller_id | (CAN_PACKET_ID["SET_CURRENT"] << 8),
            buffer,
            0,
        )

    def comm_can_set_cb(self, controller_id, current):
        buffer = []
        buffer_append_int32(buffer, int(current * 1000.0))
        self.send_servo_message(
            controller_id | (CAN_PACKET_ID["SET_CURRENT_BRAKE"] << 8),
            buffer,
            0,
        )

    def comm_can_set_rpm(self, controller_id, rpm):
        buffer = []
        buffer_append_int32(buffer, int(rpm))
        self.send_servo_message(
            controller_id | (CAN_PACKET_ID["SET_RPM"] << 8),
            buffer,
            0,
        )

    def comm_can_set_pos(self, controller_id, pos):
        buffer = []
        buffer_append_int32(buffer, int(pos * 1000000.0))
        self.send_servo_message(
            controller_id | (CAN_PACKET_ID["SET_POS"] << 8),
            buffer,
            0,
        )

    def comm_can_set_origin(self, controller_id, set_origin_mode):
        buffer = [set_origin_mode]
        self.send_servo_message(
            controller_id | (CAN_PACKET_ID["SET_ORIGIN_HERE"] << 8),
            buffer,
            0,
        )

    def comm_can_set_pos_spd(self, controller_id, pos, spd, RPA):
        buffer = []
        buffer_append_int32(buffer, int(pos * 10000.0))
        buffer_append_int16(buffer, int(spd))
        buffer_append_int16(buffer, int(RPA))
        self.send_servo_message(
            controller_id | (CAN_PACKET_ID["SET_POS_SPD"] << 8),
            buffer,
            0,
        )

    def parse_servo_message(self, data):
        # using numpy to convert signed/unsigned integers
        pos_int = np.int16(data[0] << 8 | data[1])
        spd_int = np.int16(data[2] << 8 | data[3])
        cur_int = np.int16(data[4] << 8 | data[5])
        motor_pos = float(pos_int * 0.1)  # motor position
        motor_spd = float(spd_int * 10.0)  # motor speed
        motor_cur = float(cur_int * 0.01)  # motor current
        motor_temp = np.int16(data[6])  # motor temperature
        motor_error = data[7]  # motor error mode

        if self.debug:
            print(f"  Position: {motor_pos}")
            print(f"  Velocity: {motor_spd}")
            print(f"  Current: {motor_cur}")
            print(f"  Temp: {motor_temp}")
            print(f"  Error: {motor_error}")

        return ServoMotorState(
            motor_pos, motor_spd, motor_cur, motor_temp, motor_error, 0
        )


class CubeMarsServoCAN:
    """
    The user-facing class that manages the motor.
    """

    def __init__(
        self,
        motor_type="AK80-9",
        motor_ID=1,
        max_mosfett_temp=50,
        CSV_file=None,
        log_vars=DEFAULT_LOG_VARIABLES,
        can_channel="can0",
        config_overrides=None,
    ):
        """
        Args:
            motor_type: The type of motor being controlled, ie AK80-9.
            motor_ID: The CAN ID of the motor.
            max_mosfett_temp: temperature of the mosfett above which to throw an error, in Celsius
            CSV_file: A CSV file to output log info to. If None, no log will be recorded.
            log_vars: The variables to log as a python list.
            can_channel: The CAN channel to use (default "can0")
            config_overrides: Optional dictionary to override specific motor parameters.
        """
        self.type = motor_type
        self.ID = motor_ID
        self.csv_file_name = CSV_file
        self.max_temp = max_mosfett_temp

        # Load Configuration
        self.config: MotorConfig = get_motor_config(motor_type, config_overrides)

        print(f"Initializing device: {self.device_info_string()}")

        self._motor_state = ServoMotorState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._motor_state_async = ServoMotorState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._command = ServoCommand(0.0, 0.0, 0.0, 0.0, 0.0)
        self._control_state = ControlMode.IDLE

        self.radps_per_ERPM = 5.82e-04
        self.rad_per_Eang = np.pi / self.config.NUM_POLE_PAIRS

        self._entered = False
        self._start_time = time.time()
        self._last_update_time = self._start_time
        self._last_command_time = None
        self._updated = False

        self.log_vars = log_vars
        self.LOG_FUNCTIONS = {
            "motor_position": self.get_motor_angle_radians,
            "motor_speed": self.get_motor_velocity_radians_per_second,
            "motor_current": self.get_current_qaxis_amps,
            "motor_temperature": self.get_temperature_celsius,
        }

        self._canman = CAN_Manager_servo(channel=can_channel)
        self._canman.add_motor(self)

    def __enter__(self):
        print("Turning on control for device: " + self.device_info_string())
        if self.csv_file_name is not None:
            with open(self.csv_file_name, "w") as fd:
                writer = csv.writer(fd)
                writer.writerow(["pi_time"] + self.log_vars)
            self.csv_file = open(self.csv_file_name, "a").__enter__()
            self.csv_writer = csv.writer(self.csv_file)
        self.power_on()
        self._send_command()
        self._entered = True
        if not self.check_can_connection():
            raise RuntimeError(
                "Device not connected: " + str(self.device_info_string())
            )
        return self

    def __exit__(self, etype, value, tb):
        print("Turning off control for device: " + self.device_info_string())
        self.power_off()

        if self.csv_file_name is not None:
            self.csv_file.__exit__(etype, value, tb)

        if etype is not None:
            traceback.print_exception(etype, value, tb)

    def qaxis_current_to_TMotor_current(self, iq):
        return (
            iq
            * (self.config.GEAR_RATIO * self.config.Kt_TMotor)
            / self.config.Current_Factor
        )

    def _update_state_async(self, servo_state):
        if servo_state.error != 0:
            raise RuntimeError(
                "Driver board error for device: "
                + self.device_info_string()
                + ": "
                + ERROR_CODES[servo_state.error]
            )

        now = time.time()
        dt = self._last_update_time - now
        self._last_update_time = now
        self._motor_state_async.acceleration = (
            servo_state.velocity - self._motor_state_async.velocity
        ) / dt
        self._motor_state_async.set_state_obj(servo_state)
        self._updated = True

    def update(self):
        if not self._entered:
            raise RuntimeError(
                "Tried to update motor state before safely powering on for device: "
                + self.device_info_string()
            )

        if self.get_temperature_celsius() > self.max_temp:
            raise RuntimeError(
                "Temperature greater than {}C for device: {}".format(
                    self.max_temp, self.device_info_string()
                )
            )

        now = time.time()
        # Logic to warn if updates are too slow
        if (self._last_command_time and (now - self._last_command_time) < 0.25) and (
            (now - self._last_update_time) > 0.1
        ):
            warnings.warn(
                "State update requested but no data from motor. Delay longer after zeroing, decrease frequency, or check connection. "
                + self.device_info_string(),
                RuntimeWarning,
            )
        else:
            self._command_sent = False

        self._motor_state.set_state_obj(self._motor_state_async)
        # Apply Gear Ratio to position
        self._motor_state.position = self._motor_state.position / self.config.GEAR_RATIO

        self._send_command()

        if self.csv_file_name is not None:
            self.csv_writer.writerow(
                [self._last_update_time - self._start_time]
                + [self.LOG_FUNCTIONS[var]() for var in self.log_vars]
            )

        self._updated = False

    def _send_command(self):
        if self._control_state == ControlMode.DUTY_CYCLE:
            self._canman.comm_can_set_duty(self.ID, self._command.duty)
        elif self._control_state == ControlMode.CURRENT_LOOP:
            self._canman.comm_can_set_current(self.ID, self._command.current)
        elif self._control_state == ControlMode.CURRENT_BRAKE:
            self._canman.comm_can_set_cb(self.ID, self._command.current)
        elif self._control_state == ControlMode.VELOCITY:
            self._canman.comm_can_set_rpm(self.ID, self._command.velocity)
        elif self._control_state == ControlMode.POSITION:
            self._canman.comm_can_set_pos(self.ID, self._command.position)
        elif self._control_state == ControlMode.POSITION_VELOCITY:
            self._canman.comm_can_set_pos_spd(
                self.ID,
                self._command.position,
                self._command.velocity,
                self._command.acceleration,
            )
        elif self._control_state == ControlMode.IDLE:
            self._canman.comm_can_set_duty(self.ID, 0.0)
        else:
            raise RuntimeError(
                "UNDEFINED STATE for device " + self.device_info_string()
            )

        self._last_command_time = time.time()

    def power_on(self):
        self._canman.power_on(self.ID)
        self._updated = True

    def power_off(self):
        self._canman.power_off(self.ID)

    def set_zero_position(self):
        self._canman.comm_can_set_origin(self.ID, 1)
        self._last_command_time = time.time()

    # --- Getters ---

    def get_temperature_celsius(self):
        return self._motor_state.temperature

    def get_motor_error_code(self):
        return self._motor_state.error

    def get_current_qaxis_amps(self):
        return self._motor_state.current

    def get_output_angle_radians(self):
        return self._motor_state.position * self.rad_per_Eang

    def get_output_velocity_radians_per_second(self):
        return self._motor_state.velocity * self.radps_per_ERPM

    def get_output_acceleration_radians_per_second_squared(self):
        return self._motor_state.acceleration

    def get_output_torque_newton_meters(self):
        return (
            self.get_current_qaxis_amps()
            * self.config.Kt_actual
            * self.config.GEAR_RATIO
        )

    # --- Mode Setters ---

    def enter_duty_cycle_control(self):
        self._control_state = ControlMode.DUTY_CYCLE

    def enter_current_control(self):
        self._control_state = ControlMode.CURRENT_LOOP

    def enter_current_brake_control(self):
        self._control_state = ControlMode.CURRENT_BRAKE

    def enter_velocity_control(self):
        self._control_state = ControlMode.VELOCITY

    def enter_position_control(self):
        self._control_state = ControlMode.POSITION

    def enter_position_velocity_control(self):
        self._control_state = ControlMode.POSITION_VELOCITY

    def enter_idle_mode(self):
        self._control_state = ControlMode.IDLE

    # --- Command Setters ---

    def set_output_angle_radians(self, pos, vel=0.0, acc=0.0):
        if np.abs(pos) >= self.config.P_max:
            raise RuntimeError(
                f"Cannot control using impedance mode for angles with magnitude greater than {self.config.P_max} rad!"
            )

        pos = pos / self.rad_per_Eang
        vel = vel / self.radps_per_ERPM
        acc = acc / self.radps_per_ERPM

        if self._control_state == ControlMode.POSITION_VELOCITY:
            self._command.position = pos
            self._command.velocity = vel
            self._command.acceleration = acc
        elif self._control_state == ControlMode.POSITION:
            self._command.position = pos
        else:
            raise RuntimeError(
                "Attempted to send position command without entering position control "
                + self.device_info_string()
            )

    def set_duty_cycle_percent(self, duty):
        if self._control_state != ControlMode.DUTY_CYCLE:
            raise RuntimeError(
                "Attempted to send duty cycle command without gains for device "
                + self.device_info_string()
            )
        else:
            if np.abs(duty) > 1:
                raise RuntimeError("Duty cycle cannot be greater than 100%!")
            self._command.duty = duty

    def set_output_velocity_radians_per_second(self, vel):
        if np.abs(vel) >= self.config.V_max:
            raise RuntimeError(
                f"Cannot control using speed mode for velocities with magnitude greater than {self.config.V_max} rad/s!"
            )

        if self._control_state != ControlMode.VELOCITY:
            raise RuntimeError(
                "Attempted to send speed command without gains for device "
                + self.device_info_string()
            )
        self._command.velocity = vel / self.radps_per_ERPM

    def set_motor_current_qaxis_amps(self, current):
        if self._control_state not in [
            ControlMode.CURRENT_LOOP,
            ControlMode.CURRENT_BRAKE,
        ]:
            raise RuntimeError(
                "Attempted to send current command before entering current mode for device "
                + self.device_info_string()
            )
        self._command.current = current

    def set_output_torque_newton_meters(self, torque):
        self.set_motor_current_qaxis_amps(
            torque / self.config.Kt_actual / self.config.GEAR_RATIO
        )

    # --- Motor-Side Wrappers ---

    def set_motor_torque_newton_meters(self, torque):
        self.set_output_torque_newton_meters(torque * self.config.Kt_actual)

    def set_motor_angle_radians(self, pos):
        self.set_output_angle_radians(pos / self.config.GEAR_RATIO, 0.0, 0.0)

    def set_motor_velocity_radians_per_second(self, vel):
        self.set_output_velocity_radians_per_second(vel / self.config.GEAR_RATIO)

    def get_motor_angle_radians(self):
        return self._motor_state.position * self.rad_per_Eang * self.config.GEAR_RATIO

    def get_motor_velocity_radians_per_second(self):
        return self._motor_state.velocity * self.config.GEAR_RATIO

    def get_motor_acceleration_radians_per_second_squared(self):
        return self._motor_state.acceleration * self.config.GEAR_RATIO

    def get_motor_torque_newton_meters(self):
        return self.get_output_torque_newton_meters() * self.config.GEAR_RATIO

    # --- String Representations ---

    def __str__(self):
        return (
            self.device_info_string()
            + " | Position: "
            + "{: 1f}".format(round(self.position, 3))
            + " rad | Velocity: "
            + "{: 1f}".format(round(self.velocity, 3))
            + " rad/s | current: "
            + "{: 1f}".format(round(self.current_qaxis, 3))
            + " A | temp: "
            + "{: 1f}".format(round(self.temperature, 0))
            + " C"
        )

    def device_info_string(self):
        return str(self.type) + "  ID: " + str(self.ID)

    def check_can_connection(self):
        if not self._entered:
            raise RuntimeError(
                "Tried to check_can_connection before entering motor control! Enter control using the __enter__ method, or instantiating the CubeMarsServoCAN in a with block."
            )
        listener = can.BufferedReader()
        self._canman.notifier.add_listener(listener)
        for i in range(10):
            self.power_on()
            time.sleep(0.001)
        return True

    # --- Properties ---

    temperature = property(
        get_temperature_celsius, doc="Temperature in Degrees Celsius"
    )

    error = property(get_motor_error_code, doc="Motor error code. 0 means no error.")

    current_qaxis = property(
        get_current_qaxis_amps,
        set_motor_current_qaxis_amps,
        doc="Q-axis current in amps",
    )

    position = property(
        get_output_angle_radians, set_output_angle_radians, doc="Output angle in rad"
    )

    velocity = property(
        get_output_velocity_radians_per_second,
        set_output_velocity_radians_per_second,
        doc="Output velocity in rad/s",
    )

    acceleration = property(
        get_output_acceleration_radians_per_second_squared,
        doc="Output acceleration in rad/s/s",
    )

    torque = property(
        get_output_torque_newton_meters,
        set_output_torque_newton_meters,
        doc="Output torque in Nm",
    )

    angle_motorside = property(
        get_motor_angle_radians, set_motor_angle_radians, doc="Motor-side angle in rad"
    )

    velocity_motorside = property(
        get_motor_velocity_radians_per_second,
        set_motor_velocity_radians_per_second,
        doc="Motor-side velocity in rad/s",
    )

    acceleration_motorside = property(
        get_motor_acceleration_radians_per_second_squared,
        doc="Motor-side acceleration in rad/s/s",
    )

    torque_motorside = property(
        get_motor_torque_newton_meters,
        set_motor_torque_newton_meters,
        doc="Motor-side torque in Nm",
    )
