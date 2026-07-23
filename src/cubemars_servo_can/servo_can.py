"""High-level, forward-only CubeMars Servo Mode controller."""

import logging
import math
import threading
import time
from types import TracebackType
from typing import Self

from ._can_manager import CanManager, CanManagerRegistry
from ._motor_state import ServoCommand, ServoTelemetry
from ._protocol import (
    CURRENT_LIMIT_AMPS,
    MOTOR_POSITION_LIMIT_DEGREES,
    PROFILE_VELOCITY_LIMIT_ERPM,
    VELOCITY_LIMIT_ERPM,
    CanFrame,
    encode_current,
    encode_current_brake,
    encode_duty_cycle,
    encode_origin,
    encode_position,
    encode_position_velocity,
    encode_velocity,
)
from .config import ServoConfig
from .constants import _FAULT_DESCRIPTIONS, ControlMode, OriginMode
from .errors import ControlModeError, MotorConnectionError, MotorFaultError

_LOGGER = logging.getLogger(__name__)


class CubeMarsServoCan:
    """Control one CubeMars actuator running Servo Mode over CAN."""

    def __init__(self, config: ServoConfig) -> None:
        """Create a controller without opening a CAN interface or file."""
        if not isinstance(config, ServoConfig):
            raise TypeError("config must be a ServoConfig")
        self.config = config
        self.motor_config = config.motor_config
        self._lock = threading.RLock()
        self._status_event = threading.Event()
        self._telemetry = ServoTelemetry()
        self._command = ServoCommand()
        self._control_mode = ControlMode.IDLE
        self._listener_error: Exception | None = None
        self._pending_fault_code: int | None = None
        self._manager: CanManager | None = None
        self._entered = False
        self._overtemperature_samples = 0
        self._thermal_guard_active = False

    def __enter__(self) -> Self:
        """Acquire CAN resources, register the motor, and verify fresh telemetry."""
        if self._entered:
            raise RuntimeError("the servo context is already entered")
        self._reset_session_state()
        manager: CanManager | None = None
        try:
            manager = CanManagerRegistry.acquire_registered(
                channel=self.config.can_channel,
                motor_id=self.config.motor_id,
                sink=self,
            )
            self._manager = manager
            self._entered = True
            if not self.check_connection():
                raise MotorConnectionError(
                    f"Motor {self.config.motor_id} did not return fresh "
                    "Servo status telemetry."
                )
            fault_code = self._consume_pending_fault()
            if fault_code is not None:
                raise self._motor_fault(fault_code)
        except Exception:
            self._entered = False
            if manager is not None:
                self._try_send_zero_current(manager)
                manager.unregister(motor_id=self.config.motor_id)
                CanManagerRegistry.release(manager=manager)
            self._manager = None
            self._clear_pending_events()
            raise
        _LOGGER.info(
            "Connected to %s (ID %d)",
            self.motor_config.model,
            self.config.motor_id,
        )
        return self

    def __exit__(
        self,
        _exception_type: type[BaseException] | None,
        _exception: BaseException | None,
        _traceback: TracebackType | None,
    ) -> None:
        """Close this controller without suppressing a context exception."""
        self.close()

    def __str__(self) -> str:
        """Return a compact representation of synchronized motor state."""
        return (
            f"{self.motor_config.model} (ID {self.config.motor_id}) | "
            f"position={self.output_position_radians:.3f} rad | "
            f"velocity={self.output_velocity_radians_per_second:.3f} rad/s | "
            f"current={self.q_axis_current_amps:.3f} A | "
            f"temperature={self.temperature_celsius:.1f} C"
        )

    @property
    def control_mode(self) -> ControlMode:
        """Return the currently selected command mode."""
        return self._control_mode

    @property
    def is_connected(self) -> bool:
        """Return whether this instance owns an active CAN registration."""
        return self._entered and self._manager is not None

    @property
    def temperature_celsius(self) -> float:
        """Return the most recently synchronized driver temperature."""
        return self._telemetry.temperature_celsius

    @property
    def fault_code(self) -> int:
        """Return the most recently synchronized driver fault code."""
        return self._telemetry.fault_code

    @property
    def q_axis_current_amps(self) -> float:
        """Return the most recently synchronized q-axis current."""
        return self._telemetry.q_axis_current_amps

    @property
    def output_position_radians(self) -> float:
        """Return gearbox-output position in radians."""
        return self._telemetry.motor_position_degrees * self._output_radians_per_degree

    @property
    def output_velocity_radians_per_second(self) -> float:
        """Return gearbox-output velocity in radians per second."""
        return self._telemetry.velocity_erpm * self._output_radians_per_second_per_erpm

    @property
    def output_acceleration_radians_per_second_squared(self) -> float:
        """Return gearbox-output acceleration in radians per second squared."""
        return (
            self._telemetry.acceleration_erpm_per_second
            * self._output_radians_per_second_per_erpm
        )

    @property
    def output_torque_newton_meters(self) -> float:
        """Estimate ideal output torque from current, Kt, and gear ratio."""
        return (
            self.q_axis_current_amps
            * self.motor_config.torque_constant_newton_meters_per_amp
            * self.motor_config.gear_ratio
        )

    @property
    def motor_position_radians(self) -> float:
        """Return motor-shaft position in radians."""
        return self.output_position_radians * self.motor_config.gear_ratio

    @property
    def motor_velocity_radians_per_second(self) -> float:
        """Return motor-shaft velocity in radians per second."""
        return self.output_velocity_radians_per_second * self.motor_config.gear_ratio

    @property
    def motor_acceleration_radians_per_second_squared(self) -> float:
        """Return motor-shaft acceleration in radians per second squared."""
        return (
            self.output_acceleration_radians_per_second_squared
            * self.motor_config.gear_ratio
        )

    @property
    def motor_torque_newton_meters(self) -> float:
        """Estimate ideal motor-shaft torque from reported current and Kt."""
        return (
            self.q_axis_current_amps
            * self.motor_config.torque_constant_newton_meters_per_amp
        )

    @property
    def _output_radians_per_degree(self) -> float:
        """Return the motor-degree to output-radian conversion."""
        return math.pi / (180.0 * self.motor_config.gear_ratio)

    @property
    def _output_radians_per_second_per_erpm(self) -> float:
        """Return the ERPM to output-radians-per-second conversion."""
        return (
            2.0
            * math.pi
            / (60.0 * self.motor_config.pole_pairs * self.motor_config.gear_ratio)
        )

    def set_control_mode(self, mode: ControlMode) -> None:
        """Select how the staged command is encoded by the next update."""
        if not isinstance(mode, ControlMode):
            raise TypeError("mode must be a ControlMode")
        self._control_mode = mode

    def set_origin(self, mode: OriginMode = OriginMode.TEMPORARY) -> None:
        """Send an immediate temporary or persistent origin operation."""
        self._require_entered()
        if not isinstance(mode, OriginMode):
            raise TypeError("mode must be an OriginMode")
        if (
            mode is OriginMode.PERSISTENT
            and not self.motor_config.supports_persistent_origin
        ):
            raise ValueError("this motor configuration forbids persistent origin")
        self._send_frame(
            encode_origin(motor_id=self.config.motor_id, mode=mode),
        )

    def set_duty_cycle(self, duty_cycle: float) -> None:
        """Stage normalized duty cycle in the inclusive range [-1, 1]."""
        self._require_mode(ControlMode.DUTY_CYCLE)
        self._check_range(
            value=duty_cycle,
            minimum=-1.0,
            maximum=1.0,
            label="Duty cycle",
            unit="",
        )
        self._command.duty_cycle = duty_cycle

    def set_q_axis_current_amps(self, current_amps: float) -> None:
        """Stage q-axis or braking current in amperes."""
        if self._control_mode not in {
            ControlMode.Q_AXIS_CURRENT,
            ControlMode.CURRENT_BRAKE,
        }:
            raise ControlModeError(
                "current commands require q-axis-current or current-brake mode"
            )
        maximum = min(self.config.current_limit_amps, CURRENT_LIMIT_AMPS)
        minimum = -maximum
        if self._control_mode is ControlMode.CURRENT_BRAKE:
            minimum = 0.0
        self._check_range(
            value=current_amps,
            minimum=minimum,
            maximum=maximum,
            label="Current",
            unit="A",
        )
        self._command.q_axis_current_amps = current_amps

    def set_output_position(
        self,
        position_radians: float,
        *,
        velocity_radians_per_second: float = 0.0,
        acceleration_radians_per_second_squared: float = 0.0,
    ) -> None:
        """Stage an output position and optional profile in SI units."""
        if self._control_mode not in {
            ControlMode.POSITION,
            ControlMode.POSITION_VELOCITY,
        }:
            raise ControlModeError(
                "position commands require position or position-velocity mode"
            )
        if not math.isfinite(position_radians):
            raise ValueError("position must be finite")
        motor_degrees = position_radians / self._output_radians_per_degree
        self._check_range(
            value=motor_degrees,
            minimum=-MOTOR_POSITION_LIMIT_DEGREES,
            maximum=MOTOR_POSITION_LIMIT_DEGREES,
            label="Motor position",
            unit="degrees",
        )
        velocity_erpm = self._command.velocity_erpm
        acceleration_erpm_per_second = self._command.acceleration_erpm_per_second
        if self._control_mode is ControlMode.POSITION_VELOCITY:
            velocity_erpm = self._output_velocity_to_erpm(
                velocity_radians_per_second,
                protocol_limit_erpm=PROFILE_VELOCITY_LIMIT_ERPM,
            )
            if (
                not math.isfinite(acceleration_radians_per_second_squared)
                or acceleration_radians_per_second_squared < 0.0
            ):
                raise ValueError("profile acceleration must be finite and non-negative")
            acceleration_erpm_per_second = (
                acceleration_radians_per_second_squared
                / self._output_radians_per_second_per_erpm
            )
            encode_position_velocity(
                motor_id=self.config.motor_id,
                motor_position_degrees=motor_degrees,
                velocity_erpm=velocity_erpm,
                acceleration_erpm_per_second=acceleration_erpm_per_second,
            )
        else:
            if (
                velocity_radians_per_second != 0.0
                or acceleration_radians_per_second_squared != 0.0
            ):
                raise ControlModeError(
                    "profile velocity and acceleration require position-velocity mode"
                )
            encode_position(
                motor_id=self.config.motor_id,
                motor_position_degrees=motor_degrees,
            )
        self._command.motor_position_degrees = motor_degrees
        self._command.velocity_erpm = velocity_erpm
        self._command.acceleration_erpm_per_second = acceleration_erpm_per_second

    def set_output_velocity(self, velocity_radians_per_second: float) -> None:
        """Stage gearbox-output velocity in radians per second."""
        self._require_mode(ControlMode.VELOCITY)
        self._command.velocity_erpm = self._output_velocity_to_erpm(
            velocity_radians_per_second,
            protocol_limit_erpm=VELOCITY_LIMIT_ERPM,
        )

    def set_output_torque(self, torque_newton_meters: float) -> None:
        """Stage ideal output torque by converting it to q-axis current."""
        limit = self.config.output_torque_limit_newton_meters
        self._check_range(
            value=torque_newton_meters,
            minimum=-limit,
            maximum=limit,
            label="Output torque",
            unit="Nm",
        )
        current_amps = torque_newton_meters / (
            self.motor_config.torque_constant_newton_meters_per_amp
            * self.motor_config.gear_ratio
        )
        self.set_q_axis_current_amps(current_amps)

    def set_motor_position(
        self,
        position_radians: float,
        *,
        velocity_radians_per_second: float = 0.0,
        acceleration_radians_per_second_squared: float = 0.0,
    ) -> None:
        """Stage motor-shaft position and optional motor-shaft profile."""
        self.set_output_position(
            position_radians / self.motor_config.gear_ratio,
            velocity_radians_per_second=(
                velocity_radians_per_second / self.motor_config.gear_ratio
            ),
            acceleration_radians_per_second_squared=(
                acceleration_radians_per_second_squared / self.motor_config.gear_ratio
            ),
        )

    def set_motor_velocity(self, velocity_radians_per_second: float) -> None:
        """Stage motor-shaft velocity in radians per second."""
        self.set_output_velocity(
            velocity_radians_per_second / self.motor_config.gear_ratio
        )

    def set_motor_torque(self, torque_newton_meters: float) -> None:
        """Stage ideal motor-shaft torque in newton-metres."""
        self.set_output_torque(torque_newton_meters * self.motor_config.gear_ratio)

    def check_connection(self) -> bool:
        """Send zero current and wait for a fresh exact Servo status frame."""
        self._require_entered()
        self._status_event.clear()
        self._send_frame(
            encode_current(motor_id=self.config.motor_id, current_amps=0.0),
        )
        return self._status_event.wait(self.config.connection_timeout_seconds)

    def update(self) -> None:
        """Enforce safety, send the selected command, and log fresh telemetry."""
        self._require_entered()
        with self._lock:
            listener_error = self._listener_error
            self._listener_error = None
            telemetry = self._telemetry
            fault_code = self._pending_fault_code
            self._pending_fault_code = None
        if listener_error is not None:
            self._try_send_zero_current()
            raise MotorConnectionError("failed to decode Servo telemetry") from (
                listener_error
            )
        if fault_code is not None:
            self._try_send_zero_current()
            raise self._motor_fault(fault_code)
        telemetry_age = time.monotonic() - telemetry.received_at_seconds
        if telemetry_age > self.config.telemetry_timeout_seconds:
            self._try_send_zero_current()
            raise MotorConnectionError(
                f"Motor {self.config.motor_id} telemetry is stale by "
                f"{telemetry_age:.3f} seconds."
            )
        if self._update_thermal_guard(telemetry.temperature_celsius):
            self._try_send_zero_current()
            raise MotorFaultError(
                motor_id=self.config.motor_id,
                fault_code=1,
                description=(
                    "temperature exceeded "
                    f"{self.config.max_driver_temperature_celsius:g} C for "
                    f"{self._overtemperature_samples} samples"
                ),
            )
        if self._thermal_guard_active:
            self._send_safe_command(telemetry)
        else:
            self._send_selected_command()

    def close(self) -> None:
        """Send zero current and release resources; repeated calls are safe."""
        manager = self._manager
        if manager is None:
            return
        self._try_send_zero_current(manager)
        self._entered = False
        manager.unregister(motor_id=self.config.motor_id)
        CanManagerRegistry.release(manager=manager)
        self._manager = None
        self._overtemperature_samples = 0
        self._thermal_guard_active = False
        self._clear_pending_events()

    def _accept_telemetry(self, telemetry: ServoTelemetry) -> None:
        """Store a sample and derive acceleration on the notifier thread."""
        with self._lock:
            elapsed = (
                telemetry.received_at_seconds - self._telemetry.received_at_seconds
            )
            acceleration = 0.0
            if self._telemetry.received_at_seconds > 0.0 and elapsed > 0.0:
                acceleration = (
                    telemetry.velocity_erpm - self._telemetry.velocity_erpm
                ) / elapsed
            self._telemetry = ServoTelemetry(
                motor_position_degrees=telemetry.motor_position_degrees,
                velocity_erpm=telemetry.velocity_erpm,
                q_axis_current_amps=telemetry.q_axis_current_amps,
                temperature_celsius=telemetry.temperature_celsius,
                fault_code=telemetry.fault_code,
                received_at_seconds=telemetry.received_at_seconds,
                acceleration_erpm_per_second=acceleration,
            )
            if telemetry.fault_code != 0 and self._pending_fault_code is None:
                self._pending_fault_code = telemetry.fault_code
            self._status_event.set()

    def _accept_listener_error(self, error: Exception) -> None:
        """Store a listener-thread failure for the next control-thread update."""
        with self._lock:
            self._listener_error = error

    def _send_selected_command(self) -> None:
        """Encode and send the command selected by the active mode."""
        if self._control_mode is ControlMode.IDLE:
            frame = encode_current(
                motor_id=self.config.motor_id,
                current_amps=0.0,
            )
        elif self._control_mode is ControlMode.DUTY_CYCLE:
            frame = encode_duty_cycle(
                motor_id=self.config.motor_id,
                duty_cycle=self._command.duty_cycle,
            )
        elif self._control_mode is ControlMode.Q_AXIS_CURRENT:
            frame = encode_current(
                motor_id=self.config.motor_id,
                current_amps=self._command.q_axis_current_amps,
            )
        elif self._control_mode is ControlMode.CURRENT_BRAKE:
            frame = encode_current_brake(
                motor_id=self.config.motor_id,
                current_amps=self._command.q_axis_current_amps,
            )
        elif self._control_mode is ControlMode.VELOCITY:
            frame = encode_velocity(
                motor_id=self.config.motor_id,
                velocity_erpm=self._command.velocity_erpm,
            )
        elif self._control_mode is ControlMode.POSITION:
            frame = encode_position(
                motor_id=self.config.motor_id,
                motor_position_degrees=self._command.motor_position_degrees,
            )
        elif self._control_mode is ControlMode.POSITION_VELOCITY:
            frame = encode_position_velocity(
                motor_id=self.config.motor_id,
                motor_position_degrees=self._command.motor_position_degrees,
                velocity_erpm=self._command.velocity_erpm,
                acceleration_erpm_per_second=(
                    self._command.acceleration_erpm_per_second
                ),
            )
        else:
            raise ControlModeError(f"unsupported control mode: {self._control_mode!r}")
        self._send_frame(frame)

    def _send_safe_command(self, telemetry: ServoTelemetry) -> None:
        """Hold position modes and zero current for all other thermal guards."""
        if self._control_mode is ControlMode.POSITION:
            frame = encode_position(
                motor_id=self.config.motor_id,
                motor_position_degrees=telemetry.motor_position_degrees,
            )
        elif self._control_mode is ControlMode.POSITION_VELOCITY:
            frame = encode_position_velocity(
                motor_id=self.config.motor_id,
                motor_position_degrees=telemetry.motor_position_degrees,
                velocity_erpm=0.0,
                acceleration_erpm_per_second=0.0,
            )
        else:
            frame = encode_current(
                motor_id=self.config.motor_id,
                current_amps=0.0,
            )
        self._send_frame(frame)

    def _update_thermal_guard(self, temperature_celsius: float) -> bool:
        """Update debounce and hysteresis state, returning whether it tripped."""
        limit = self.config.max_driver_temperature_celsius
        if temperature_celsius > limit:
            self._overtemperature_samples += 1
            self._thermal_guard_active = True
        else:
            self._overtemperature_samples = 0
            if temperature_celsius <= limit - self.config.cooldown_margin_celsius:
                self._thermal_guard_active = False
        return self._overtemperature_samples >= self.config.overtemperature_trip_count

    def _output_velocity_to_erpm(
        self,
        velocity_radians_per_second: float,
        *,
        protocol_limit_erpm: float,
    ) -> float:
        """Convert and validate an output velocity command."""
        erpm = velocity_radians_per_second / (self._output_radians_per_second_per_erpm)
        maximum = min(self.motor_config.max_velocity_erpm, protocol_limit_erpm)
        tolerance = 1e-6
        if not math.isfinite(erpm):
            raise ValueError("velocity must be finite")
        if erpm < -maximum - tolerance or erpm > maximum + tolerance:
            raise ValueError(
                f"Velocity {erpm:g} ERPM is outside [{-maximum:g}, {maximum:g}] ERPM"
            )
        return min(max(erpm, -maximum), maximum)

    def _send_frame(self, frame: CanFrame) -> None:
        """Send an encoded frame through the acquired channel manager."""
        manager = self._manager
        if manager is None:
            raise RuntimeError("the servo context has not been entered")
        manager.send(frame)

    def _try_send_zero_current(self, manager: CanManager | None = None) -> None:
        """Attempt a safe zero-current command without masking a primary error."""
        active_manager = manager if manager is not None else self._manager
        if active_manager is None:
            return
        try:
            active_manager.send(
                encode_current(
                    motor_id=self.config.motor_id,
                    current_amps=0.0,
                )
            )
        except Exception:
            _LOGGER.exception(
                "Failed to send zero-current command to motor %d",
                self.config.motor_id,
            )

    def _reset_session_state(self) -> None:
        """Discard state that must not cross context-manager sessions."""
        with self._lock:
            self._telemetry = ServoTelemetry()
            self._listener_error = None
            self._pending_fault_code = None
        self._status_event.clear()
        self._overtemperature_samples = 0
        self._thermal_guard_active = False

    def _clear_pending_events(self) -> None:
        """Clear listener events consumed by session cleanup."""
        with self._lock:
            self._listener_error = None
            self._pending_fault_code = None
        self._status_event.clear()

    def _consume_pending_fault(self) -> int | None:
        """Return and clear the oldest pending driver fault."""
        with self._lock:
            fault_code = self._pending_fault_code
            self._pending_fault_code = None
        return fault_code

    def _motor_fault(self, fault_code: int) -> MotorFaultError:
        """Build a structured exception for a driver-reported fault."""
        return MotorFaultError(
            motor_id=self.config.motor_id,
            fault_code=fault_code,
            description=_FAULT_DESCRIPTIONS.get(
                fault_code,
                "unknown driver fault",
            ),
        )

    def _require_entered(self) -> None:
        """Reject operations that require an acquired CAN manager."""
        if not self._entered or self._manager is None:
            raise RuntimeError("enter the servo context before using this operation")

    def _require_mode(self, expected: ControlMode) -> None:
        """Reject a command incompatible with the active mode."""
        if self._control_mode is not expected:
            raise ControlModeError(f"command requires {expected.value} mode")

    @staticmethod
    def _check_range(
        *,
        value: float,
        minimum: float,
        maximum: float,
        label: str,
        unit: str,
    ) -> None:
        """Reject non-finite values and values outside inclusive limits."""
        if not math.isfinite(value):
            raise ValueError(f"{label} must be finite")
        if not minimum <= value <= maximum:
            unit_suffix = f" {unit}" if unit else ""
            raise ValueError(
                f"{label} {value:g}{unit_suffix} is outside "
                f"[{minimum:g}, {maximum:g}]{unit_suffix}"
            )
