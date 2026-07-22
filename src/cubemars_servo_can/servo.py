"""High-level, forward-only CubeMars Servo Mode controller."""

import csv
import logging
import math
import threading
import time
from dataclasses import replace
from types import TracebackType
from typing import Protocol, Self, TextIO, cast

from . import _protocol
from ._models import MotorCommand, Telemetry
from ._transport import _ChannelTransport, _TransportRegistry
from .config import MotorConfig, ServoConfig, get_motor_config
from .constants import (
    FAULT_DESCRIPTIONS,
    ControlMode,
    LogField,
    MotorModel,
    OriginMode,
)
from .errors import ControlModeError, MotorConnectionError, MotorFaultError

_LOGGER = logging.getLogger(__name__)


class _CsvWriter(Protocol):
    """Typed subset of the writer returned by the ``csv`` module."""

    def writerow(self, _row: tuple[str | float | LogField, ...]) -> object:
        """Write one row to the CSV stream."""


class CubeMarsServoCan:
    """Control one CubeMars actuator running Servo Mode over CAN."""

    def __init__(
        self,
        *,
        motor: MotorModel | MotorConfig,
        motor_id: int,
        servo_config: ServoConfig | None = None,
    ) -> None:
        """Create a controller without opening a CAN interface.

        Args:
            motor: Built-in model or a complete custom motor configuration.
            motor_id: One-byte controller ID configured in the actuator.
            servo_config: Runtime CAN, safety, probe, and logging settings.
        """
        if not 0 <= motor_id <= 0xFF:
            raise ValueError("motor_id must be between 0 and 255")
        if not isinstance(motor, (MotorModel, MotorConfig)):
            raise TypeError("motor must be a MotorModel or MotorConfig")
        self.config = (
            get_motor_config(motor) if isinstance(motor, MotorModel) else motor
        )
        self.motor_id = motor_id
        self.servo_config = servo_config or ServoConfig()
        self._lock = threading.RLock()
        self._status_event = threading.Event()
        self._telemetry = Telemetry()
        self._command = MotorCommand()
        self._control_mode = ControlMode.IDLE
        self._listener_error: Exception | None = None
        self._pending_fault_code: int | None = None
        self._transport: _ChannelTransport | None = None
        self._entered = False
        self._overtemperature_samples = 0
        self._thermal_guard_active = False
        self._started_at_seconds = time.monotonic()
        self._log_file: TextIO | None = None
        self._log_writer: _CsvWriter | None = None

    def __enter__(self) -> Self:
        """Acquire CAN resources, register the motor, and verify telemetry."""
        if self._entered:
            raise RuntimeError("The servo context is already entered")
        self._reset_session_state()
        transport: _ChannelTransport | None = None
        try:
            transport = _TransportRegistry.acquire_registered(
                channel=self.servo_config.can_channel,
                motor_id=self.motor_id,
                sink=self,
            )
            self._transport = transport
            self._open_log()
            self._entered = True
            if not self.check_connection():
                raise MotorConnectionError(
                    f"Motor {self.motor_id} did not return Servo status telemetry."
                )
        except Exception:
            self._entered = False
            self._close_log()
            if transport is not None:
                transport.unregister(motor_id=self.motor_id)
                _TransportRegistry.release(transport=transport)
            self._transport = None
            self._clear_pending_events()
            raise
        _LOGGER.info("Connected to %s (ID %d)", self.config.model_name, self.motor_id)
        return self

    def __exit__(
        self,
        exception_type: type[BaseException] | None,
        exception: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        """Close this motor without suppressing an exception from the context."""
        del exception_type, exception, traceback
        self.close()

    @property
    def control_mode(self) -> ControlMode:
        """Return the currently selected command mode."""
        return self._control_mode

    @property
    def is_connected(self) -> bool:
        """Return whether this instance currently owns a CAN registration."""
        return self._entered and self._transport is not None

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
        """Return the output position using the established library conversion."""
        return (
            self._telemetry.position_units
            * self._radians_per_position_unit
            / self.config.gear_ratio
        )

    @property
    def output_velocity_radians_per_second(self) -> float:
        """Return output velocity in radians per second."""
        return self._telemetry.velocity_erpm * self._output_radians_per_second_per_erpm

    @property
    def output_acceleration_radians_per_second_squared(self) -> float:
        """Return output acceleration in radians per second squared."""
        return (
            self._telemetry.acceleration_erpm_per_second
            * self._output_radians_per_second_per_erpm
        )

    @property
    def output_torque_newton_meters(self) -> float:
        """Estimate output torque from the reported q-axis current."""
        return (
            self.q_axis_current_amps
            * self.config.effective_torque_constant_newton_meters_per_amp
            * self.config.gear_ratio
        )

    @property
    def motor_position_radians(self) -> float:
        """Return motor-side position in radians."""
        return self.output_position_radians * self.config.gear_ratio

    @property
    def motor_velocity_radians_per_second(self) -> float:
        """Return motor-side velocity in radians per second."""
        return self.output_velocity_radians_per_second * self.config.gear_ratio

    @property
    def motor_acceleration_radians_per_second_squared(self) -> float:
        """Return motor-side acceleration in radians per second squared."""
        return (
            self.output_acceleration_radians_per_second_squared * self.config.gear_ratio
        )

    @property
    def motor_torque_newton_meters(self) -> float:
        """Estimate motor-side torque from q-axis current."""
        return self.output_torque_newton_meters / self.config.gear_ratio

    @property
    def _radians_per_position_unit(self) -> float:
        """Return the established high-level position conversion factor."""
        return math.pi / self.config.pole_pairs

    @property
    def _output_radians_per_second_per_erpm(self) -> float:
        """Return the ERPM-to-output-radians-per-second conversion factor."""
        return 2.0 * math.pi / (60.0 * self.config.pole_pairs * self.config.gear_ratio)

    def set_control_mode(self, mode: ControlMode) -> None:
        """Select the interpretation of the next command sent by ``update``."""
        if not isinstance(mode, ControlMode):
            raise TypeError("mode must be a ControlMode")
        self._control_mode = mode

    def set_origin(self, mode: OriginMode = OriginMode.TEMPORARY) -> None:
        """Send an immediate origin operation."""
        self._require_entered()
        if not isinstance(mode, OriginMode):
            raise TypeError("mode must be an OriginMode")
        if (
            mode is not OriginMode.TEMPORARY
            and not self.config.supports_persistent_origin
        ):
            raise ValueError(
                "This motor configuration forbids persistent origin changes"
            )
        self._send_frame(_protocol.encode_origin(motor_id=self.motor_id, mode=mode))

    def set_duty_cycle(self, duty_cycle: float) -> None:
        """Set a normalized duty-cycle target in the inclusive range [-1, 1]."""
        self._require_mode(ControlMode.DUTY_CYCLE)
        if not -1.0 <= duty_cycle <= 1.0:
            raise ValueError("duty_cycle must be between -1 and 1")
        self._command.duty_cycle = duty_cycle

    def set_q_axis_current_amps(self, current_amps: float) -> None:
        """Set a q-axis or braking-current target in amperes."""
        if self._control_mode not in {
            ControlMode.Q_AXIS_CURRENT,
            ControlMode.CURRENT_BRAKE,
        }:
            raise ControlModeError(
                "Current commands require q-axis-current or current-brake mode."
            )
        if self._control_mode is ControlMode.CURRENT_BRAKE and current_amps < 0.0:
            raise ValueError("Current-brake mode requires a non-negative current")
        self._check_range(
            value=current_amps,
            minimum=self.config.min_current_amps,
            maximum=self.config.max_current_amps,
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
        """Set output position and optional profile limits in SI units."""
        if self._control_mode not in {
            ControlMode.POSITION,
            ControlMode.POSITION_VELOCITY,
        }:
            raise ControlModeError(
                "Position commands require position or position-velocity mode."
            )
        self._check_range(
            value=position_radians,
            minimum=self.config.min_output_position_radians,
            maximum=self.config.max_output_position_radians,
            label="Output position",
            unit="rad",
        )
        position_units = position_radians / self._radians_per_position_unit
        velocity_erpm = self._command.velocity_erpm
        acceleration_erpm_per_second = self._command.acceleration_erpm_per_second
        if self._control_mode is ControlMode.POSITION_VELOCITY:
            velocity_erpm = self._output_velocity_to_erpm(velocity_radians_per_second)
            if acceleration_radians_per_second_squared < 0.0:
                raise ValueError("Profile acceleration must not be negative")
            acceleration_erpm_per_second = (
                acceleration_radians_per_second_squared
                / self._output_radians_per_second_per_erpm
            )
        elif (
            velocity_radians_per_second != 0.0
            or acceleration_radians_per_second_squared != 0.0
        ):
            raise ControlModeError(
                "Profile velocity and acceleration require position-velocity mode."
            )
        self._command.position_units = position_units
        self._command.velocity_erpm = velocity_erpm
        self._command.acceleration_erpm_per_second = acceleration_erpm_per_second

    def set_output_velocity(self, velocity_radians_per_second: float) -> None:
        """Set output velocity in radians per second."""
        self._require_mode(ControlMode.VELOCITY)
        self._command.velocity_erpm = self._output_velocity_to_erpm(
            velocity_radians_per_second
        )

    def set_output_torque(self, torque_newton_meters: float) -> None:
        """Set output torque by converting it to a q-axis current target."""
        self._check_range(
            value=torque_newton_meters,
            minimum=self.config.min_output_torque_newton_meters,
            maximum=self.config.max_output_torque_newton_meters,
            label="Output torque",
            unit="Nm",
        )
        current = torque_newton_meters / (
            self.config.effective_torque_constant_newton_meters_per_amp
            * self.config.gear_ratio
        )
        self.set_q_axis_current_amps(current)

    def set_motor_position(
        self,
        position_radians: float,
        *,
        velocity_radians_per_second: float = 0.0,
        acceleration_radians_per_second_squared: float = 0.0,
    ) -> None:
        """Set a motor-side position and optional motor-side profile limits."""
        self.set_output_position(
            position_radians / self.config.gear_ratio,
            velocity_radians_per_second=(
                velocity_radians_per_second / self.config.gear_ratio
            ),
            acceleration_radians_per_second_squared=(
                acceleration_radians_per_second_squared / self.config.gear_ratio
            ),
        )

    def set_motor_velocity(self, velocity_radians_per_second: float) -> None:
        """Set motor-side velocity in radians per second."""
        self.set_output_velocity(velocity_radians_per_second / self.config.gear_ratio)

    def set_motor_torque(self, torque_newton_meters: float) -> None:
        """Set motor-side torque in newton-metres."""
        self.set_output_torque(torque_newton_meters * self.config.gear_ratio)

    def check_connection(self) -> bool:
        """Probe with zero current and wait for a fresh exact Servo status frame."""
        self._require_entered()
        self._status_event.clear()
        probe = _protocol.encode_current(motor_id=self.motor_id, current_amps=0.0)
        for _ in range(self.servo_config.connection_probe_count):
            self._send_frame(probe)
        return self._status_event.wait(self.servo_config.connection_timeout_seconds)

    def update(self) -> None:
        """Synchronize telemetry, enforce safety, send a command, and log a row."""
        self._require_entered()
        with self._lock:
            if self._listener_error is not None:
                error = self._listener_error
                self._listener_error = None
                raise MotorConnectionError(
                    "Failed to decode Servo telemetry"
                ) from error
            telemetry = self._telemetry
            fault_code = self._pending_fault_code
            self._pending_fault_code = None
        if fault_code is not None:
            raise MotorFaultError(
                motor_id=self.motor_id,
                fault_code=fault_code,
                description=FAULT_DESCRIPTIONS.get(fault_code, "unknown driver fault"),
            )
        telemetry_age = time.monotonic() - telemetry.received_at_seconds
        if telemetry_age > self.servo_config.telemetry_timeout_seconds:
            raise MotorConnectionError(
                f"Motor {self.motor_id} telemetry is stale by {telemetry_age:.3f} seconds."
            )
        self._update_thermal_guard(telemetry.temperature_celsius)
        if self._thermal_guard_active:
            self._send_safe_command(telemetry)
        else:
            self._send_selected_command()
        self._write_log_row(telemetry)

    def close(self) -> None:
        """Zero current and release this motor's resources; repeated calls are safe."""
        transport = self._transport
        if transport is None:
            return
        try:
            identifier, data = _protocol.encode_current(
                motor_id=self.motor_id,
                current_amps=0.0,
            )
            transport.send(arbitration_id=identifier, data=data)
        except Exception:
            _LOGGER.exception("Failed to send final zero-current command")
        finally:
            self._entered = False
            self._close_log()
            transport.unregister(motor_id=self.motor_id)
            _TransportRegistry.release(transport=transport)
            self._transport = None
            self._overtemperature_samples = 0
            self._thermal_guard_active = False
            self._clear_pending_events()

    def _accept_telemetry(self, telemetry: Telemetry) -> None:
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
            self._telemetry = replace(
                telemetry, acceleration_erpm_per_second=acceleration
            )
            if telemetry.fault_code and self._pending_fault_code is None:
                self._pending_fault_code = telemetry.fault_code
            self._status_event.set()

    def _accept_listener_error(self, error: Exception) -> None:
        """Store a listener-thread failure for the next user-thread update."""
        with self._lock:
            self._listener_error = error

    def _send_selected_command(self) -> None:
        """Encode and send the command selected by the current mode."""
        if self._control_mode is ControlMode.IDLE:
            frame = _protocol.encode_current(motor_id=self.motor_id, current_amps=0.0)
        elif self._control_mode is ControlMode.DUTY_CYCLE:
            frame = _protocol.encode_duty_cycle(
                motor_id=self.motor_id, duty_cycle=self._command.duty_cycle
            )
        elif self._control_mode is ControlMode.Q_AXIS_CURRENT:
            frame = _protocol.encode_current(
                motor_id=self.motor_id,
                current_amps=self._command.q_axis_current_amps,
            )
        elif self._control_mode is ControlMode.CURRENT_BRAKE:
            frame = _protocol.encode_current_brake(
                motor_id=self.motor_id,
                current_amps=self._command.q_axis_current_amps,
            )
        elif self._control_mode is ControlMode.VELOCITY:
            frame = _protocol.encode_velocity(
                motor_id=self.motor_id, velocity_erpm=self._command.velocity_erpm
            )
        elif self._control_mode is ControlMode.POSITION:
            frame = _protocol.encode_position(
                motor_id=self.motor_id,
                position_units=self._command.position_units,
            )
        elif self._control_mode is ControlMode.POSITION_VELOCITY:
            frame = _protocol.encode_position_velocity(
                motor_id=self.motor_id,
                position_units=self._command.position_units,
                velocity_erpm=self._command.velocity_erpm,
                acceleration_erpm_per_second=(
                    self._command.acceleration_erpm_per_second
                ),
            )
        else:
            raise ControlModeError(f"Unsupported control mode: {self._control_mode!r}")
        self._send_frame(frame)

    def _send_safe_command(self, telemetry: Telemetry) -> None:
        """Emit a non-driving command while the thermal guard is active."""
        if self._control_mode is ControlMode.POSITION:
            frame = _protocol.encode_position(
                motor_id=self.motor_id,
                position_units=telemetry.position_units,
            )
        elif self._control_mode is ControlMode.POSITION_VELOCITY:
            frame = _protocol.encode_position_velocity(
                motor_id=self.motor_id,
                position_units=telemetry.position_units,
                velocity_erpm=0.0,
                acceleration_erpm_per_second=0.0,
            )
        else:
            frame = _protocol.encode_current(motor_id=self.motor_id, current_amps=0.0)
        self._send_frame(frame)

    def _update_thermal_guard(self, temperature_celsius: float) -> None:
        """Apply consecutive-sample trip logic with cooldown hysteresis."""
        limit = self.servo_config.max_driver_temperature_celsius
        if temperature_celsius > limit:
            self._overtemperature_samples += 1
            self._thermal_guard_active = True
        else:
            self._overtemperature_samples = 0
            if temperature_celsius <= limit - self.servo_config.cooldown_margin_celsius:
                self._thermal_guard_active = False
        if (
            self._overtemperature_samples
            >= self.servo_config.overtemperature_trip_count
        ):
            raise MotorFaultError(
                motor_id=self.motor_id,
                fault_code=1,
                description=(
                    f"temperature exceeded {limit:g} C for "
                    f"{self._overtemperature_samples} samples"
                ),
            )

    def _output_velocity_to_erpm(self, velocity_radians_per_second: float) -> float:
        """Convert and validate an output velocity command."""
        erpm = velocity_radians_per_second / self._output_radians_per_second_per_erpm
        self._check_range(
            value=erpm,
            minimum=self.config.min_velocity_erpm,
            maximum=self.config.max_velocity_erpm,
            label="Velocity",
            unit="ERPM",
        )
        return erpm

    def _send_frame(self, frame: tuple[int, bytes]) -> None:
        """Send an encoded frame through the acquired transport."""
        transport = self._transport
        if transport is None:
            raise RuntimeError("The servo context has not been entered")
        identifier, data = frame
        transport.send(arbitration_id=identifier, data=data)

    def _open_log(self) -> None:
        """Open the optional CSV log and write its header."""
        path = self.servo_config.csv_log_path
        if path is None:
            return
        self._log_file = path.open("w", encoding="utf-8", newline="")
        writer = cast(_CsvWriter, csv.writer(self._log_file))
        writer.writerow(("elapsed_seconds", *self.servo_config.log_fields))
        self._log_writer = writer

    def _write_log_row(self, telemetry: Telemetry) -> None:
        """Write the selected telemetry fields when logging is enabled."""
        writer = self._log_writer
        if writer is None:
            return
        values = tuple(
            self._log_value(field, telemetry) for field in self.servo_config.log_fields
        )
        writer.writerow((time.monotonic() - self._started_at_seconds, *values))
        if self._log_file is not None:
            self._log_file.flush()

    def _log_value(self, field: LogField, telemetry: Telemetry) -> float:
        """Resolve one CSV field from a single immutable telemetry sample."""
        if field is LogField.OUTPUT_POSITION_RADIANS:
            return (
                telemetry.position_units
                * self._radians_per_position_unit
                / self.config.gear_ratio
            )
        if field is LogField.OUTPUT_VELOCITY_RADIANS_PER_SECOND:
            return telemetry.velocity_erpm * self._output_radians_per_second_per_erpm
        if field is LogField.Q_AXIS_CURRENT_AMPS:
            return telemetry.q_axis_current_amps
        if field is LogField.TEMPERATURE_CELSIUS:
            return telemetry.temperature_celsius
        raise ValueError(f"Unsupported log field: {field!r}")

    def _close_log(self) -> None:
        """Close and discard optional CSV resources."""
        log_file = self._log_file
        self._log_file = None
        self._log_writer = None
        if log_file is None:
            return
        try:
            log_file.close()
        except Exception:
            _LOGGER.exception("Failed to close the CSV telemetry log")

    def _reset_session_state(self) -> None:
        """Discard state that must never cross a context-manager session."""
        with self._lock:
            self._telemetry = Telemetry()
            self._listener_error = None
            self._pending_fault_code = None
        self._status_event.clear()
        self._overtemperature_samples = 0
        self._thermal_guard_active = False

    def _clear_pending_events(self) -> None:
        """Clear notifier events that have been consumed by session cleanup."""
        with self._lock:
            self._listener_error = None
            self._pending_fault_code = None
        self._status_event.clear()

    def _require_entered(self) -> None:
        """Reject operations that require an acquired transport."""
        if not self._entered or self._transport is None:
            raise RuntimeError("Enter the servo context before using this operation")

    def _require_mode(self, expected: ControlMode) -> None:
        """Reject a command incompatible with the active mode."""
        if self._control_mode is not expected:
            raise ControlModeError(f"Command requires {expected.value} mode")

    @staticmethod
    def _check_range(
        *, value: float, minimum: float, maximum: float, label: str, unit: str
    ) -> None:
        """Reject non-finite values and values outside inclusive safety limits."""
        if not math.isfinite(value):
            raise ValueError(f"{label} must be finite")
        if not minimum <= value <= maximum:
            raise ValueError(
                f"{label} {value:g} {unit} is outside [{minimum:g}, {maximum:g}] {unit}"
            )

    def __str__(self) -> str:
        """Return a compact representation of the synchronized motor state."""
        return (
            f"{self.config.model_name} (ID {self.motor_id}) | "
            f"position={self.output_position_radians:.3f} rad | "
            f"velocity={self.output_velocity_radians_per_second:.3f} rad/s | "
            f"current={self.q_axis_current_amps:.3f} A | "
            f"temperature={self.temperature_celsius:.1f} C"
        )
