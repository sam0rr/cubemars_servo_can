import can
import time
import csv
import traceback
import warnings
from typing import List, Optional, Dict, Any, TextIO

import numpy as np

from .constants import ERROR_CODES, DEFAULT_LOG_VARIABLES, ControlMode
from .config import get_motor_config, MotorConfig
from .motor_state import ServoMotorState, ServoCommand
from .can_manager import CAN_Manager_servo


class CubeMarsServoCAN:
    """
    The user-facing class that manages the motor. This class should be
    used in the context of a with as block, in order to safely enter/exit
    control of the motor.
    """

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
        Sets up the motor manager. Note the device will not be powered on by this method! You must
        call __enter__, mostly commonly by using a with block, before attempting to control the motor.

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
        """
        Used to safely power the motor on and begin the log file.
        """
        print(f"Turning on control for device: {self.device_info_string()}")
        if self.csv_file_name is not None:
            with open(self.csv_file_name, "w") as fd:
                writer = csv.writer(fd)
                writer.writerow(["pi_time"] + self.log_vars)
            self.csv_file = open(self.csv_file_name, "a")
            self.csv_writer = csv.writer(self.csv_file)

        self.power_on()
        self._send_command()
        self._entered = True
        if not self.check_can_connection():
            raise RuntimeError(f"Device not connected: {self.device_info_string()}")
        return self

    def __exit__(self, etype, value, tb) -> None:
        """
        Used to safely power the motor off and close the log file.
        """
        print(f"Turning off control for device: {self.device_info_string()}")
        self.power_off()

        if self.csv_file is not None:
            self.csv_file.close()

        if etype is not None:
            traceback.print_exception(etype, value, tb)

    def qaxis_current_to_TMotor_current(self, iq: float) -> float:
        """
        Convert Q-axis current to T-Motor current.
        """
        return (
            iq
            * (self.config.GEAR_RATIO * self.config.Kt_TMotor)
            / self.config.Current_Factor
        )

    def _update_state_async(self, servo_state: ServoMotorState) -> None:
        """
        This method is called by the handler every time a message is recieved on the bus
        from this motor, to store the most recent state information for later.

        Args:
            servo_state: the servo_state object with the updated motor state

        Raises:
            RuntimeError when device sends back an error code that is not 0 (0 meaning no error)
        """
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
        """
        This method is called by the user to synchronize the current state used by the controller/logger
        with the most recent message recieved, as well as to send the current command.
        """
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
        """
        Sends a command to the motor depending on what control mode the motor is in. This method
        is called by update(), and should only be called on its own if you don't want to update the motor state info.
        """
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
        """Powers on the motor."""
        self._canman.power_on(self.ID)
        self._updated = True

    def power_off(self) -> None:
        """Powers off the motor."""
        self._canman.power_off(self.ID)

    def set_zero_position(self) -> None:
        """Zeros the position"""
        self._canman.comm_can_set_origin(self.ID, 1)
        self._last_command_time = time.time()

    # --- Getters ---

    def get_temperature_celsius(self) -> float:
        """
        Returns:
            The most recently updated motor temperature in degrees C.
        """
        return self._motor_state.temperature

    def get_motor_error_code(self) -> int:
        """
        Returns:
            The most recently updated motor error code.
            Note the program should throw a runtime error before you get a chance to read
            this value if it is ever anything besides 0.
        """
        return self._motor_state.error

    def get_current_qaxis_amps(self) -> float:
        """
        Returns:
            The most recently updated qaxis current in amps
        """
        return self._motor_state.current

    def get_output_angle_radians(self) -> float:
        """
        Returns:
            The most recently updated output angle in radians
        """
        return self._motor_state.position * self.rad_per_Eang

    def get_output_velocity_radians_per_second(self) -> float:
        """
        Returns:
            The most recently updated output velocity in radians per second
        """
        return self._motor_state.velocity * self.radps_per_ERPM

    def get_output_acceleration_radians_per_second_squared(self) -> float:
        """
        Returns:
            The most recently updated output acceleration in radians per second per second
        """
        return self._motor_state.acceleration

    def get_output_torque_newton_meters(self) -> float:
        """
        Returns:
            the most recently updated output torque in Nm
        """
        return (
            self.get_current_qaxis_amps()
            * self.config.Kt_actual
            * self.config.GEAR_RATIO
        )

    # --- Mode Setters ---

    def enter_duty_cycle_control(self) -> None:
        """
        Must call this to enable sending duty cycle commands.
        """
        self._control_state = ControlMode.DUTY_CYCLE

    def enter_current_control(self) -> None:
        """
        Must call this to enable sending current commands.
        """
        self._control_state = ControlMode.CURRENT_LOOP

    def enter_current_brake_control(self) -> None:
        """
        Must call this to enable sending current brake commands.
        """
        self._control_state = ControlMode.CURRENT_BRAKE

    def enter_velocity_control(self) -> None:
        """
        Must call this to enable sending velocity commands.
        """
        self._control_state = ControlMode.VELOCITY

    def enter_position_control(self) -> None:
        """
        Must call this to enable position commands.
        """
        self._control_state = ControlMode.POSITION

    def enter_position_velocity_control(self) -> None:
        """
        Must call this to enable sending position commands with specified velocity and accleration limits.
        """
        self._control_state = ControlMode.POSITION_VELOCITY

    def enter_idle_mode(self) -> None:
        """
        Enter the idle state, where duty cycle is set to 0. (This is the default state.)
        """
        self._control_state = ControlMode.IDLE

    # --- Command Setters ---

    def set_output_angle_radians(
        self, pos: float, vel: float = 0.0, acc: float = 0.0
    ) -> None:
        """
        Update the current command to the desired position, when in position or position-velocity mode.
        Note, this does not send a command, it updates the CubeMarsServoCAN's saved command,
        which will be sent when update() is called.

        Args:
            pos: The desired output angle in rad
            vel: The desired speed to get there in rad/s (when in POSITION_VELOCITY mode)
            acc: The desired acceleration to get there in rad/s/s, ish (when in POSITION_VELOCITY mode)
        """
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
        """
        Used for duty cycle mode, to set desired duty cycle.
        Note, this does not send a command, it updates the CubeMarsServoCAN's saved command,
        which will be sent when update() is called.

        Args:
            duty: The desired duty cycle, (-1 to 1)
        """
        if self._control_state != ControlMode.DUTY_CYCLE:
            raise RuntimeError(
                f"Attempted to send duty cycle command without gains for device {self.device_info_string()}"
            )
        else:
            if np.abs(duty) > 1:
                raise RuntimeError(
                    "Cannot control using duty cycle mode for duty cycles greater than 100%!"
                )
            self._command.duty = duty

    def set_output_velocity_radians_per_second(self, vel: float) -> None:
        """
        Used for velocity mode to set output velocity command.
        Note, this does not send a command, it updates the CubeMarsServoCAN's saved command,
        which will be sent when update() is called.

        Args:
            vel: The desired output speed in rad/s
        """
        if np.abs(vel) >= self.config.V_max:
            raise RuntimeError(
                f"Cannot control using speed mode for angles with magnitude greater than {self.config.V_max} rad/s!"
            )

        if self._control_state != ControlMode.VELOCITY:
            raise RuntimeError(
                f"Attempted to send speed command without gains for device {self.device_info_string()}"
            )
        self._command.velocity = vel / self.radps_per_ERPM

    def set_motor_current_qaxis_amps(self, current: float) -> None:
        """
        Used for current mode to set current command.
        Note, this does not send a command, it updates the CubeMarsServoCAN's saved command,
        which will be sent when update() is called.

        Args:
            current: the desired current in amps.
        """
        if self._control_state not in [
            ControlMode.CURRENT_LOOP,
            ControlMode.CURRENT_BRAKE,
        ]:
            raise RuntimeError(
                f"Attempted to send current command before entering current mode for device {self.device_info_string()}"
            )
        self._command.current = current

    def set_output_torque_newton_meters(self, torque: float) -> None:
        """
        Used for current mode to set current, based on desired torque.
        If a more complicated torque model is available for the motor, that will be used.
        Otherwise it will just use the motor's torque constant.

        Args:
            torque: The desired output torque in Nm.
        """
        self.set_motor_current_qaxis_amps(
            torque / self.config.Kt_actual / self.config.GEAR_RATIO
        )

    # --- Motor-Side Wrappers ---

    def set_motor_torque_newton_meters(self, torque: float) -> None:
        """
        Wrapper of set_output_torque that accounts for gear ratio to control motor-side torque

        Args:
            torque: The desired motor-side torque in Nm.
        """
        self.set_output_torque_newton_meters(torque * self.config.Kt_actual)

    def set_motor_angle_radians(self, pos: float) -> None:
        """
        Wrapper for set_output_angle that accounts for gear ratio to control motor-side angle

        Args:
            pos: The desired motor-side position in rad.
        """
        self.set_output_angle_radians(pos / self.config.GEAR_RATIO, 0.0, 0.0)

    def set_motor_velocity_radians_per_second(self, vel: float) -> None:
        """
        Wrapper for set_output_velocity that accounts for gear ratio to control motor-side velocity

        Args:
            vel: The desired motor-side velocity in rad/s.
        """
        self.set_output_velocity_radians_per_second(vel / self.config.GEAR_RATIO)

    def get_motor_angle_radians(self) -> float:
        """
        Wrapper for get_output_angle that accounts for gear ratio to get motor-side angle

        Returns:
            The most recently updated motor-side angle in rad.
        """
        return self._motor_state.position * self.rad_per_Eang * self.config.GEAR_RATIO

    def get_motor_velocity_radians_per_second(self) -> float:
        """
        Wrapper for get_output_velocity that accounts for gear ratio to get motor-side velocity

        Returns:
            The most recently updated motor-side velocity in rad/s.
        """
        return self._motor_state.velocity * self.config.GEAR_RATIO

    def get_motor_acceleration_radians_per_second_squared(self) -> float:
        """
        Wrapper for get_output_acceleration that accounts for gear ratio to get motor-side acceleration

        Returns:
            The most recently updated motor-side acceleration in rad/s/s.
        """
        return self._motor_state.acceleration * self.config.GEAR_RATIO

    def get_motor_torque_newton_meters(self) -> float:
        """
        Wrapper for get_output_torque that accounts for gear ratio to get motor-side torque

        Returns:
            The most recently updated motor-side torque in Nm.
        """
        return self.get_output_torque_newton_meters() * self.config.GEAR_RATIO

    # --- String Representations ---

    def __str__(self) -> str:
        """Prints the motor's device info and current"""
        return (
            f"{self.device_info_string()} | "
            f"Position: {round(self.position, 3): 1f} rad | "
            f"Velocity: {round(self.velocity, 3): 1f} rad/s | "
            f"current: {round(self.current_qaxis, 3): 1f} A | "
            f"temp: {round(self.temperature, 0): 1f} C"
        )

    def device_info_string(self) -> str:
        """Prints the motor's ID and device type."""
        return f"{self.type}  ID: {self.ID}"

    def check_can_connection(self) -> bool:
        """
        Checks the motor's connection by attempting to send 10 startup messages.
        If it gets 10 replies, then the connection is confirmed.

        Returns:
            True if a connection is established and False otherwise.
        """
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
