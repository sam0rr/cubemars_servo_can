"""Behavioral tests for the forward-only high-level controller."""

import csv
import math
import struct
import time
from dataclasses import replace
from pathlib import Path
from typing import TextIO, cast

import pytest
from conftest import CanHarness

from cubemars_servo_can import (
    CanConnectionError,
    ControlMode,
    ControlModeError,
    CubeMarsServoCan,
    LogField,
    MotorConfig,
    MotorConnectionError,
    MotorFaultError,
    MotorModel,
    OriginMode,
    ServoConfig,
    get_motor_config,
)
from cubemars_servo_can._models import Telemetry

pytestmark = pytest.mark.usefixtures("can_harness")


class FailingLog:
    """Text-stream double whose close operation fails."""

    def close(self) -> None:
        """Raise a synthetic filesystem failure."""
        raise OSError("synthetic close failure")


def make_servo(
    *,
    motor: MotorModel | MotorConfig = MotorModel.AK80_9,
    motor_id: int = 1,
    servo_config: ServoConfig | None = None,
) -> CubeMarsServoCan:
    """Create a consistently configured test controller."""
    return CubeMarsServoCan(
        motor=motor,
        motor_id=motor_id,
        servo_config=servo_config
        or ServoConfig(
            can_channel="vcan0",
            connection_timeout_seconds=0.001,
        ),
    )


def enter_servo(servo: CubeMarsServoCan) -> CubeMarsServoCan:
    """Enter and return a controller for compact tests."""
    return servo.__enter__()


def test_constructor_is_side_effect_free_and_validated(can_harness: CanHarness) -> None:
    """Delay CAN acquisition and reject invalid public constructor values."""
    servo = make_servo()
    assert can_harness.bus is None
    assert servo.config is get_motor_config(MotorModel.AK80_9)
    assert servo.motor_id == 1
    assert not servo.is_connected
    assert servo.control_mode is ControlMode.IDLE
    servo.close()

    custom = replace(servo.config, model_name="CUSTOM")
    assert make_servo(motor=custom).config is custom
    with pytest.raises(ValueError, match="between 0 and 255"):
        make_servo(motor_id=-1)
    with pytest.raises(TypeError, match="MotorModel or MotorConfig"):
        CubeMarsServoCan(motor=cast(MotorModel, "AK80-9"), motor_id=1)


def test_context_lifecycle_probes_and_releases(can_harness: CanHarness) -> None:
    """Probe using zero current, prevent re-entry, and release deterministically."""
    servo = make_servo()
    assert enter_servo(servo) is servo
    assert servo.is_connected
    assert len(can_harness.sent) == 3
    assert all(message.arbitration_id == 0x101 for message in can_harness.sent)
    with pytest.raises(RuntimeError, match="already entered"):
        servo.__enter__()
    servo.__exit__(None, None, None)
    assert not servo.is_connected
    assert len(can_harness.sent) == 4
    servo.close()
    assert len(can_harness.sent) == 4


def test_context_exit_does_not_suppress_errors(can_harness: CanHarness) -> None:
    """Accept exception metadata without altering propagation semantics."""
    servo = enter_servo(make_servo())
    error = RuntimeError("application failure")
    servo.__exit__(RuntimeError, error, None)
    assert can_harness.bus is not None


def test_entry_rolls_back_when_motor_does_not_respond(can_harness: CanHarness) -> None:
    """Close all acquired resources when connection verification times out."""
    can_harness.respond_to_current = False
    servo = make_servo()
    with pytest.raises(MotorConnectionError, match="did not return"):
        servo.__enter__()
    assert not servo.is_connected
    assert can_harness.bus is not None
    assert can_harness.bus.shutdown_count == 1


def test_entry_retry_discards_malformed_frame_error(can_harness: CanHarness) -> None:
    """Keep listener failures scoped to the entry attempt that observed them."""
    can_harness.status_payload = b"bad"
    servo = make_servo()
    with pytest.raises(MotorConnectionError, match="did not return"):
        servo.__enter__()
    can_harness.status_payload = struct.pack(">hhhbB", 0, 0, 0, 25, 0)
    enter_servo(servo)
    servo.update()


def test_duplicate_motor_id_on_one_channel_is_rejected() -> None:
    """Prevent two controllers from consuming the same status stream."""
    first = enter_servo(make_servo())
    second = make_servo()
    with pytest.raises(CanConnectionError, match="already registered"):
        second.__enter__()
    assert first.is_connected
    first.close()


def test_check_connection_requires_an_entered_context() -> None:
    """Reject probes before transport acquisition."""
    servo = make_servo()
    with pytest.raises(RuntimeError, match="Enter the servo context"):
        servo.check_connection()
    with pytest.raises(RuntimeError, match="has not been entered"):
        servo._send_frame((0x101, b"\x00\x00\x00\x00"))


def test_close_logs_final_send_failures(
    can_harness: CanHarness, caplog: pytest.LogCaptureFixture
) -> None:
    """Release resources even if the final safety frame cannot be sent."""
    servo = enter_servo(make_servo())
    can_harness.fail_send = True
    servo.close()
    assert "final zero-current" in caplog.text
    assert not servo.is_connected


def test_close_releases_transport_when_log_close_fails(
    can_harness: CanHarness, caplog: pytest.LogCaptureFixture
) -> None:
    """Treat log errors as secondary to deterministic motor and CAN cleanup."""
    servo = enter_servo(make_servo())
    servo._log_file = cast(TextIO, FailingLog())
    servo.close()
    assert servo._transport is None
    assert can_harness.bus is not None
    assert can_harness.bus.shutdown_count == 1
    assert "Failed to close the CSV telemetry log" in caplog.text


def test_set_origin_is_immediate_and_typed(can_harness: CanHarness) -> None:
    """Send all supported origin modes and guard persistent configuration changes."""
    servo = make_servo()
    with pytest.raises(RuntimeError, match="Enter the servo context"):
        servo.set_origin()
    enter_servo(servo)
    servo.set_origin()
    servo.set_origin(OriginMode.PERSISTENT)
    servo.set_origin(OriginMode.RESTORE_DEFAULT)
    assert [message.arbitration_id for message in can_harness.sent[-3:]] == [
        0x501,
        0x501,
        0x501,
    ]
    assert [bytes(message.data) for message in can_harness.sent[-3:]] == [
        b"\x00",
        b"\x01",
        b"\x02",
    ]
    with pytest.raises(TypeError, match="OriginMode"):
        servo.set_origin(cast(OriginMode, 1))
    servo.close()

    restricted = replace(
        get_motor_config(MotorModel.AK80_9),
        supports_persistent_origin=False,
    )
    restricted_servo = enter_servo(make_servo(motor=restricted))
    with pytest.raises(ValueError, match="forbids"):
        restricted_servo.set_origin(OriginMode.PERSISTENT)


@pytest.mark.parametrize(
    ("mode", "configure", "expected_id", "expected_data"),
    [
        (ControlMode.IDLE, "idle", 0x101, struct.pack(">i", 0)),
        (ControlMode.DUTY_CYCLE, "duty", 0x001, struct.pack(">i", 25_000)),
        (ControlMode.Q_AXIS_CURRENT, "current", 0x101, struct.pack(">i", 1_500)),
        (ControlMode.CURRENT_BRAKE, "brake", 0x201, struct.pack(">i", 1_500)),
        (ControlMode.VELOCITY, "velocity", 0x301, struct.pack(">i", 1_000)),
        (ControlMode.POSITION, "position", 0x401, struct.pack(">i", 668_450)),
        (
            ControlMode.POSITION_VELOCITY,
            "profile",
            0x601,
            struct.pack(">ihh", 668_450, 100, 100),
        ),
    ],
)
def test_update_sends_every_control_mode(
    can_harness: CanHarness,
    mode: ControlMode,
    configure: str,
    expected_id: int,
    expected_data: bytes,
) -> None:
    """Dispatch each public mode to its exact protocol encoder."""
    servo = enter_servo(make_servo())
    servo.set_control_mode(mode)
    radians_per_erpm = 2.0 * math.pi / (60.0 * 21.0 * 9.0)
    if configure == "duty":
        servo.set_duty_cycle(0.25)
    elif configure in {"current", "brake"}:
        servo.set_q_axis_current_amps(1.5)
    elif configure == "velocity":
        servo.set_output_velocity(1_000.0 * radians_per_erpm)
    elif configure == "position":
        servo.set_output_position(10.0)
    elif configure == "profile":
        servo.set_output_position(
            10.0,
            velocity_radians_per_second=1_000.0 * radians_per_erpm,
            acceleration_radians_per_second_squared=1_000.0 * radians_per_erpm,
        )
    servo.update()
    assert can_harness.sent[-1].arbitration_id == expected_id
    assert bytes(can_harness.sent[-1].data) == expected_data


def test_set_control_mode_requires_enum() -> None:
    """Reject the removed integer/string mode conventions."""
    servo = make_servo()
    with pytest.raises(TypeError, match="ControlMode"):
        servo.set_control_mode(cast(ControlMode, "velocity"))


def test_duty_and_current_commands_enforce_modes_and_limits() -> None:
    """Reject unsafe duty, current, and braking targets."""
    servo = make_servo()
    with pytest.raises(ControlModeError, match="duty_cycle"):
        servo.set_duty_cycle(0.0)
    servo.set_control_mode(ControlMode.DUTY_CYCLE)
    with pytest.raises(ValueError, match="between -1 and 1"):
        servo.set_duty_cycle(1.1)

    with pytest.raises(ControlModeError, match="Current commands"):
        servo.set_q_axis_current_amps(0.0)
    servo.set_control_mode(ControlMode.CURRENT_BRAKE)
    with pytest.raises(ValueError, match="non-negative"):
        servo.set_q_axis_current_amps(-0.1)
    with pytest.raises(ValueError, match="outside"):
        servo.set_q_axis_current_amps(16.0)
    servo.set_control_mode(ControlMode.Q_AXIS_CURRENT)
    with pytest.raises(ValueError, match="finite"):
        servo.set_q_axis_current_amps(math.nan)


def test_position_commands_enforce_modes_profiles_and_ranges() -> None:
    """Validate position, profile velocity, and profile acceleration independently."""
    servo = make_servo()
    with pytest.raises(ControlModeError, match="Position commands"):
        servo.set_output_position(0.0)
    servo.set_control_mode(ControlMode.POSITION)
    previous_position = servo._command.position_units
    with pytest.raises(ValueError, match="outside"):
        servo.set_output_position(1_000.0)
    with pytest.raises(ControlModeError, match="Profile velocity"):
        servo.set_output_position(0.0, velocity_radians_per_second=1.0)
    assert servo._command.position_units == previous_position

    servo.set_control_mode(ControlMode.POSITION_VELOCITY)
    with pytest.raises(ValueError, match="Velocity"):
        servo.set_output_position(0.0, velocity_radians_per_second=100.0)
    with pytest.raises(ValueError, match="acceleration"):
        servo.set_output_position(
            0.0,
            acceleration_radians_per_second_squared=-1.0,
        )
    assert servo._command.position_units == previous_position


def test_velocity_torque_and_motor_wrappers() -> None:
    """Convert output and motor-side commands without compatibility aliases."""
    servo = make_servo()
    with pytest.raises(ControlModeError, match="velocity"):
        servo.set_output_velocity(0.0)
    servo.set_control_mode(ControlMode.VELOCITY)
    with pytest.raises(ValueError, match="Velocity"):
        servo.set_output_velocity(100.0)
    servo.set_motor_velocity(1.0)
    assert servo._command.velocity_erpm > 0.0

    servo.set_control_mode(ControlMode.Q_AXIS_CURRENT)
    with pytest.raises(ValueError, match="Output torque"):
        servo.set_output_torque(16.0)
    servo.set_output_torque(10.0)
    expected_current = 10.0 / (0.115 * 9.0)
    assert servo._command.q_axis_current_amps == pytest.approx(expected_current)
    servo.set_motor_torque(1.0)
    assert servo._command.q_axis_current_amps == pytest.approx(1.0 / 0.115)

    servo.set_control_mode(ControlMode.POSITION)
    servo.set_motor_position(9.0)
    assert servo._command.position_units == pytest.approx(1.0 / (math.pi / 21.0))


def test_telemetry_properties_and_acceleration() -> None:
    """Expose explicit-unit output and motor-side telemetry properties."""
    servo = make_servo()
    servo._accept_telemetry(
        Telemetry(
            position_units=21.0,
            velocity_erpm=1_000.0,
            q_axis_current_amps=2.0,
            temperature_celsius=30.0,
            fault_code=0,
            received_at_seconds=10.0,
        )
    )
    servo._accept_telemetry(
        Telemetry(
            position_units=42.0,
            velocity_erpm=1_100.0,
            q_axis_current_amps=2.0,
            temperature_celsius=31.0,
            fault_code=0,
            received_at_seconds=10.5,
        )
    )
    assert servo.output_position_radians == pytest.approx(2.0 * math.pi / 9.0)
    assert servo.motor_position_radians == pytest.approx(2.0 * math.pi)
    assert servo.motor_velocity_radians_per_second == pytest.approx(
        1_100.0 * 2.0 * math.pi / (60.0 * 21.0)
    )
    assert servo.output_velocity_radians_per_second == pytest.approx(
        servo.motor_velocity_radians_per_second / 9.0
    )
    assert servo.motor_acceleration_radians_per_second_squared == pytest.approx(
        200.0 * 2.0 * math.pi / (60.0 * 21.0)
    )
    assert servo.output_acceleration_radians_per_second_squared == pytest.approx(
        servo.motor_acceleration_radians_per_second_squared / 9.0
    )
    assert servo.output_torque_newton_meters == pytest.approx(2.0 * 0.115 * 9.0)
    assert servo.motor_torque_newton_meters == pytest.approx(2.0 * 0.115)
    assert servo.q_axis_current_amps == 2.0
    assert servo.temperature_celsius == 31.0
    assert servo.fault_code == 0
    assert "AK80-9 (ID 1)" in str(servo)


@pytest.mark.parametrize(
    ("fault_code", "description"),
    [(6, "MOSFET"), (7, "stall"), (99, "unknown")],
)
def test_update_surfaces_documented_and_unknown_faults(
    fault_code: int, description: str
) -> None:
    """Translate driver fault bytes to typed exceptions with corrected descriptions."""
    servo = enter_servo(make_servo())
    servo._accept_telemetry(Telemetry(fault_code=fault_code))
    with pytest.raises(MotorFaultError, match=description) as raised:
        servo.update()
    assert raised.value.motor_id == 1
    assert raised.value.fault_code == fault_code
    assert raised.value.description


def test_transient_fault_is_latched_until_update() -> None:
    """Never overwrite an unconsumed fault with a later healthy status sample."""
    servo = enter_servo(make_servo())
    now = time.monotonic()
    servo._accept_telemetry(Telemetry(fault_code=7, received_at_seconds=now))
    servo._accept_telemetry(Telemetry(received_at_seconds=now + 0.01))
    with pytest.raises(MotorFaultError, match="stall"):
        servo.update()
    servo.update()


def test_update_rejects_stale_telemetry_before_sending(
    can_harness: CanHarness,
) -> None:
    """Stop commanding when receive-side safety telemetry is no longer fresh."""
    servo = enter_servo(
        make_servo(
            servo_config=ServoConfig(
                can_channel="vcan0",
                connection_timeout_seconds=0.001,
                telemetry_timeout_seconds=0.1,
            )
        )
    )
    servo._accept_telemetry(Telemetry(received_at_seconds=time.monotonic() - 1.0))
    sent_before_update = len(can_harness.sent)
    with pytest.raises(MotorConnectionError, match="stale"):
        servo.update()
    assert len(can_harness.sent) == sent_before_update


def test_listener_errors_surface_on_user_thread() -> None:
    """Preserve notifier liveness and raise decoding failures from ``update``."""
    servo = enter_servo(make_servo())
    servo._accept_listener_error(ValueError("bad frame"))
    with pytest.raises(MotorConnectionError, match="decode"):
        servo.update()
    servo.update()


@pytest.mark.parametrize(
    ("mode", "expected_id"),
    [
        (ControlMode.IDLE, 0x101),
        (ControlMode.POSITION, 0x401),
        (ControlMode.POSITION_VELOCITY, 0x601),
    ],
)
def test_thermal_guard_sends_mode_appropriate_safe_frames(
    can_harness: CanHarness, mode: ControlMode, expected_id: int
) -> None:
    """Zero torque-producing modes and hold position modes before thermal trip."""
    config = ServoConfig(
        can_channel="vcan0",
        connection_timeout_seconds=0.001,
        max_driver_temperature_celsius=50.0,
        overtemperature_trip_count=3,
        cooldown_margin_celsius=2.0,
    )
    servo = enter_servo(make_servo(servo_config=config))
    servo.set_control_mode(mode)
    servo._accept_telemetry(
        Telemetry(
            position_units=12.0,
            temperature_celsius=51.0,
            received_at_seconds=time.monotonic(),
        )
    )
    servo.update()
    assert can_harness.sent[-1].arbitration_id == expected_id
    servo._accept_telemetry(
        Telemetry(
            position_units=12.0,
            temperature_celsius=49.0,
            received_at_seconds=time.monotonic(),
        )
    )
    servo.update()
    assert servo._thermal_guard_active
    servo._accept_telemetry(
        Telemetry(
            position_units=12.0,
            temperature_celsius=48.0,
            received_at_seconds=time.monotonic(),
        )
    )
    servo.update()
    assert not servo._thermal_guard_active


def test_thermal_guard_trips_after_consecutive_samples() -> None:
    """Raise a typed thermal fault only at the configured consecutive count."""
    servo = enter_servo(
        make_servo(
            servo_config=ServoConfig(
                can_channel="vcan0",
                connection_timeout_seconds=0.001,
                max_driver_temperature_celsius=50.0,
                overtemperature_trip_count=2,
            )
        )
    )
    servo._accept_telemetry(
        Telemetry(
            temperature_celsius=51.0,
            received_at_seconds=time.monotonic(),
        )
    )
    servo.update()
    servo._accept_telemetry(
        Telemetry(
            temperature_celsius=52.0,
            received_at_seconds=time.monotonic(),
        )
    )
    with pytest.raises(MotorFaultError, match="2 samples"):
        servo.update()


def test_csv_logging_writes_selected_explicit_unit_fields(
    tmp_path: Path,
) -> None:
    """Manage one UTF-8 CSV handle and flush each synchronized row."""
    path = tmp_path / "telemetry.csv"
    servo = enter_servo(
        make_servo(
            servo_config=ServoConfig(
                can_channel="vcan0",
                connection_timeout_seconds=0.001,
                csv_log_path=path,
            )
        )
    )
    servo.update()
    servo.close()
    with path.open(encoding="utf-8", newline="") as log_file:
        rows = list(csv.reader(log_file))
    assert rows[0] == ["elapsed_seconds", *LogField]
    assert len(rows) == 2
    assert float(rows[1][3]) == 3.0
    assert float(rows[1][4]) == 25.0


def test_csv_row_uses_one_immutable_telemetry_snapshot(tmp_path: Path) -> None:
    """Prevent notifier updates from mixing values within one CSV row."""
    path = tmp_path / "snapshot.csv"
    servo = enter_servo(
        make_servo(
            servo_config=ServoConfig(
                can_channel="vcan0",
                connection_timeout_seconds=0.001,
                csv_log_path=path,
            )
        )
    )
    snapshot = Telemetry(
        position_units=21.0,
        velocity_erpm=42.0,
        q_axis_current_amps=2.0,
        temperature_celsius=30.0,
        received_at_seconds=time.monotonic(),
    )
    servo._accept_telemetry(
        Telemetry(
            position_units=999.0,
            velocity_erpm=999.0,
            q_axis_current_amps=9.0,
            temperature_celsius=99.0,
            received_at_seconds=time.monotonic(),
        )
    )
    servo._write_log_row(snapshot)
    servo.close()
    with path.open(encoding="utf-8", newline="") as log_file:
        row = list(csv.reader(log_file))[1]
    assert float(row[1]) == pytest.approx(math.pi / 9.0)
    assert float(row[3]) == 2.0
    assert float(row[4]) == 30.0


def test_invalid_runtime_log_field_is_rejected(
    tmp_path: Path,
) -> None:
    """Keep an explicit failure for corrupted runtime enum state."""
    config = ServoConfig(
        can_channel="vcan0",
        connection_timeout_seconds=0.001,
        csv_log_path=tmp_path / "bad.csv",
        log_fields=(cast(LogField, "invalid"),),
    )
    servo = enter_servo(make_servo(servo_config=config))
    with pytest.raises(ValueError, match="Unsupported log field"):
        servo.update()


def test_corrupted_control_mode_is_rejected() -> None:
    """Fail explicitly if internal state is corrupted instead of sending a frame."""
    servo = enter_servo(make_servo())
    servo._control_mode = cast(ControlMode, object())
    with pytest.raises(ControlModeError, match="Unsupported"):
        servo.update()
