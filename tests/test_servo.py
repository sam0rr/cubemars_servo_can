"""Behavioral tests for the forward-only high-level controller."""

import math
import struct
import time
from typing import cast

import pytest
from conftest import CanHarness

from cubemars_servo_can import (
    CanConnectionError,
    ControlMode,
    ControlModeError,
    CubeMarsServoCan,
    MotorConfig,
    MotorConnectionError,
    MotorFaultError,
    MotorModel,
    OriginMode,
    ServoConfig,
)
from cubemars_servo_can._motor_state import ServoTelemetry
from cubemars_servo_can._protocol import CanFrame

pytestmark = pytest.mark.usefixtures("can_harness")


def make_servo(config: ServoConfig | None = None) -> CubeMarsServoCan:
    """Create a consistently configured test controller."""
    return CubeMarsServoCan(
        config
        or ServoConfig(
            motor=MotorModel.AK80_9,
            motor_id=1,
            can_channel="vcan0",
            connection_timeout_seconds=0.001,
            telemetry_timeout_seconds=1.0,
        )
    )


def enter_servo(servo: CubeMarsServoCan) -> CubeMarsServoCan:
    """Enter and return a controller for compact tests."""
    return servo.__enter__()


def restricted_motor() -> MotorConfig:
    """Return explicit custom data that forbids persistent origin changes."""
    return MotorConfig(
        model="CUSTOM",
        pole_pairs=21,
        gear_ratio=9.0,
        max_velocity_erpm=32_000.0,
        torque_constant_newton_meters_per_amp=0.095,
        hardware_max_current_amps=28.0,
        hardware_max_output_torque_newton_meters=22.0,
        default_max_current_amps=15.0,
        default_max_output_torque_newton_meters=12.5,
        supports_persistent_origin=False,
    )


def test_constructor_is_side_effect_free_and_typed(
    can_harness: CanHarness,
) -> None:
    """Delay all I/O until context entry and require one ServoConfig."""
    config = ServoConfig(
        motor=MotorModel.AK80_9,
        motor_id=1,
        can_channel="vcan0",
    )
    servo = make_servo(config)
    assert can_harness.bus is None
    assert servo.config is config
    assert servo.motor_config.model == "AK80-9"
    assert not servo.is_connected
    assert servo.control_mode is ControlMode.IDLE
    assert servo.temperature_celsius == 0.0
    assert servo.fault_code == 0
    servo.close()
    servo._try_send_zero_current()
    with pytest.raises(TypeError, match="ServoConfig"):
        CubeMarsServoCan(cast(ServoConfig, object()))


def test_context_lifecycle_probes_once_and_releases(
    can_harness: CanHarness,
) -> None:
    """Probe with zero current, prevent re-entry, and release deterministically."""
    servo = make_servo()
    assert enter_servo(servo) is servo
    assert servo.is_connected
    assert len(can_harness.sent) == 1
    assert can_harness.sent[0].arbitration_id == 0x101
    with pytest.raises(RuntimeError, match="already entered"):
        servo.__enter__()
    servo.__exit__(None, None, None)
    assert not servo.is_connected
    assert len(can_harness.sent) == 2
    servo.close()
    assert len(can_harness.sent) == 2


def test_context_exit_does_not_suppress_application_errors() -> None:
    """Accept exception metadata while leaving propagation to Python."""
    servo = enter_servo(make_servo())
    error = RuntimeError("application failure")
    servo.__exit__(RuntimeError, error, None)
    assert not servo.is_connected


def test_entry_rolls_back_on_timeout_malformed_status_and_retry(
    can_harness: CanHarness,
) -> None:
    """Release every resource after failed entry and allow a clean retry."""
    can_harness.respond_to_current = False
    servo = make_servo()
    with pytest.raises(MotorConnectionError, match="did not return"):
        servo.__enter__()
    assert not servo.is_connected
    assert can_harness.bus is not None
    assert can_harness.bus.shutdown_count == 1

    can_harness.respond_to_current = True
    can_harness.status_payload = b"bad"
    with pytest.raises(MotorConnectionError, match="did not return"):
        servo.__enter__()
    can_harness.status_payload = struct.pack(">hhhbB", 0, 0, 0, 25, 0)
    enter_servo(servo)
    servo.update()


def test_entry_rejects_fresh_motor_fault_and_rolls_back(
    can_harness: CanHarness,
) -> None:
    """Fail context entry safely when the first fresh status reports a fault."""
    can_harness.status_payload = struct.pack(">hhhbB", 0, 0, 0, 25, 7)
    servo = make_servo()
    with pytest.raises(MotorFaultError, match="stall"):
        servo.__enter__()
    assert not servo.is_connected


def test_duplicate_motor_id_on_one_channel_is_rejected() -> None:
    """Prevent two controllers from consuming the same status stream."""
    first = enter_servo(make_servo())
    second = make_servo()
    with pytest.raises(CanConnectionError, match="already registered"):
        second.__enter__()
    assert first.is_connected
    first.close()


def test_operations_requiring_context_are_rejected() -> None:
    """Reject immediate operations before CAN manager acquisition."""
    servo = make_servo()
    with pytest.raises(RuntimeError, match="enter the servo context"):
        servo.check_connection()
    with pytest.raises(RuntimeError, match="has not been entered"):
        servo._send_frame(CanFrame(arbitration_id=0x101, data=b""))
    with pytest.raises(RuntimeError, match="enter the servo context"):
        servo.set_origin()


def test_close_logs_secondary_send_failures(
    can_harness: CanHarness,
    caplog: pytest.LogCaptureFixture,
) -> None:
    """Release resources even when the final safety send fails."""
    servo = enter_servo(make_servo())
    can_harness.fail_send = True
    servo.close()
    assert servo._manager is None
    assert not servo.is_connected
    assert "Failed to send zero-current" in caplog.text


def test_set_origin_is_immediate_typed_and_capability_checked(
    can_harness: CanHarness,
) -> None:
    """Send only current-manual origin modes and enforce motor capability."""
    servo = enter_servo(make_servo())
    servo.set_origin()
    servo.set_origin(OriginMode.PERSISTENT)
    assert [message.arbitration_id for message in can_harness.sent[-2:]] == [
        0x501,
        0x501,
    ]
    assert [bytes(message.data) for message in can_harness.sent[-2:]] == [
        b"\x00",
        b"\x01",
    ]
    with pytest.raises(TypeError, match="OriginMode"):
        servo.set_origin(cast(OriginMode, 1))
    servo.close()

    restricted = enter_servo(
        make_servo(
            ServoConfig(
                motor=restricted_motor(),
                motor_id=1,
                can_channel="vcan0",
                connection_timeout_seconds=0.001,
            )
        )
    )
    with pytest.raises(ValueError, match="forbids"):
        restricted.set_origin(OriginMode.PERSISTENT)


@pytest.mark.parametrize(
    ("mode", "configure", "expected_id", "expected_data"),
    [
        (ControlMode.IDLE, "idle", 0x101, struct.pack(">i", 0)),
        (ControlMode.DUTY_CYCLE, "duty", 0x001, struct.pack(">i", 25_000)),
        (ControlMode.Q_AXIS_CURRENT, "current", 0x101, struct.pack(">i", 1_500)),
        (ControlMode.CURRENT_BRAKE, "brake", 0x201, struct.pack(">i", 1_500)),
        (ControlMode.VELOCITY, "velocity", 0x301, struct.pack(">i", 1_000)),
        (ControlMode.POSITION, "position", 0x401, struct.pack(">i", 120_000)),
        (
            ControlMode.POSITION_VELOCITY,
            "profile",
            0x601,
            struct.pack(">ihh", 120_000, 100, 100),
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
    """Dispatch every public mode through its exact protocol encoder."""
    servo = enter_servo(make_servo())
    servo.set_control_mode(mode)
    radians_per_erpm = 2.0 * math.pi / (60.0 * 21.0 * 9.0)
    radians_per_electrical_degree = math.pi / (180.0 * 21.0 * 9.0)
    if configure == "duty":
        servo.set_duty_cycle(0.25)
    elif configure in {"current", "brake"}:
        servo.set_q_axis_current_amps(1.5)
    elif configure == "velocity":
        servo.set_output_velocity(1_000.0 * radians_per_erpm)
    elif configure == "position":
        servo.set_output_position(12.0 * radians_per_electrical_degree)
    elif configure == "profile":
        servo.set_output_position(
            12.0 * radians_per_electrical_degree,
            velocity_radians_per_second=1_000.0 * radians_per_erpm,
            acceleration_radians_per_second_squared=1_000.0 * radians_per_erpm,
        )
    servo.update()
    assert can_harness.sent[-1].arbitration_id == expected_id
    assert bytes(can_harness.sent[-1].data) == expected_data


def test_control_mode_duty_and_current_validation() -> None:
    """Reject legacy mode values and unsafe duty or current targets."""
    servo = make_servo()
    with pytest.raises(TypeError, match="ControlMode"):
        servo.set_control_mode(cast(ControlMode, "velocity"))
    with pytest.raises(ControlModeError, match="duty_cycle"):
        servo.set_duty_cycle(0.0)
    servo.set_control_mode(ControlMode.DUTY_CYCLE)
    with pytest.raises(ValueError, match="outside"):
        servo.set_duty_cycle(1.1)

    with pytest.raises(ControlModeError, match="current commands"):
        servo.set_q_axis_current_amps(0.0)
    servo.set_control_mode(ControlMode.CURRENT_BRAKE)
    with pytest.raises(ValueError, match="outside"):
        servo.set_q_axis_current_amps(-0.1)
    with pytest.raises(ValueError, match="outside"):
        servo.set_q_axis_current_amps(16.0)
    servo.set_control_mode(ControlMode.Q_AXIS_CURRENT)
    with pytest.raises(ValueError, match="finite"):
        servo.set_q_axis_current_amps(math.nan)


def test_position_commands_validate_modes_profiles_and_protocol_ranges() -> None:
    """Validate position and profile inputs before mutating staged state."""
    servo = make_servo()
    with pytest.raises(ControlModeError, match="position commands"):
        servo.set_output_position(0.0)
    servo.set_control_mode(ControlMode.POSITION)
    previous_position = servo._command.electrical_position_degrees
    with pytest.raises(ValueError, match="finite"):
        servo.set_output_position(math.nan)
    with pytest.raises(ValueError, match="int32"):
        servo.set_output_position(1_000_000.0)
    with pytest.raises(ControlModeError, match="profile velocity"):
        servo.set_output_position(0.0, velocity_radians_per_second=1.0)
    assert servo._command.electrical_position_degrees == previous_position

    servo.set_control_mode(ControlMode.POSITION_VELOCITY)
    with pytest.raises(ValueError, match="Velocity"):
        servo.set_output_position(0.0, velocity_radians_per_second=100.0)
    with pytest.raises(ValueError, match="acceleration"):
        servo.set_output_position(
            0.0,
            acceleration_radians_per_second_squared=-1.0,
        )
    with pytest.raises(ValueError, match="acceleration"):
        servo.set_output_position(
            0.0,
            acceleration_radians_per_second_squared=math.nan,
        )
    with pytest.raises(ValueError, match="int16"):
        servo.set_output_position(
            0.0,
            acceleration_radians_per_second_squared=1_000.0,
        )
    assert servo._command.electrical_position_degrees == previous_position


def test_velocity_torque_and_motor_side_wrappers() -> None:
    """Convert explicit SI commands without compatibility aliases."""
    servo = make_servo()
    with pytest.raises(ControlModeError, match="velocity"):
        servo.set_output_velocity(0.0)
    servo.set_control_mode(ControlMode.VELOCITY)
    with pytest.raises(ValueError, match="Velocity"):
        servo.set_output_velocity(100.0)
    with pytest.raises(ValueError, match="finite"):
        servo.set_output_velocity(math.nan)
    servo.set_motor_velocity(1.0)
    assert servo._command.velocity_erpm > 0.0

    servo.set_control_mode(ControlMode.Q_AXIS_CURRENT)
    with pytest.raises(ValueError, match="Output torque"):
        servo.set_output_torque(13.0)
    servo.set_output_torque(10.0)
    expected_current = 10.0 / (0.095 * 9.0)
    assert servo._command.q_axis_current_amps == pytest.approx(expected_current)
    servo.set_motor_torque(1.0)
    assert servo._command.q_axis_current_amps == pytest.approx(1.0 / 0.095)

    servo.set_control_mode(ControlMode.POSITION)
    servo.set_motor_position(9.0)
    assert servo._command.electrical_position_degrees == pytest.approx(
        9.0 * 180.0 * 21.0 / math.pi
    )
    servo.set_control_mode(ControlMode.POSITION_VELOCITY)
    servo.set_motor_position(
        0.1,
        velocity_radians_per_second=0.2,
        acceleration_radians_per_second_squared=0.3,
    )


def test_velocity_accepts_exact_boundary_after_roundoff() -> None:
    """Clamp sub-micro-ERPM conversion noise at both model boundaries."""
    servo = make_servo(
        ServoConfig(
            motor=MotorModel.AKA60_6,
            motor_id=1,
            can_channel="vcan0",
        )
    )
    servo.set_control_mode(ControlMode.VELOCITY)
    factor = 2.0 * math.pi / (60.0 * 14.0 * 6.0)
    maximum = (50_000.0 / 14.0 / 60.0) * 2.0 * math.pi / 6.0
    servo.set_output_velocity(maximum)
    assert servo._command.velocity_erpm == 50_000.0
    servo.set_output_velocity(-maximum)
    assert servo._command.velocity_erpm == -50_000.0
    assert maximum / factor > 50_000.0


def test_telemetry_properties_and_acceleration() -> None:
    """Expose explicit-unit output and motor-shaft telemetry properties."""
    servo = make_servo()
    servo._accept_telemetry(
        ServoTelemetry(
            electrical_position_degrees=3_780.0,
            velocity_erpm=1_000.0,
            q_axis_current_amps=2.0,
            temperature_celsius=30.0,
            received_at_seconds=10.0,
        )
    )
    servo._accept_telemetry(
        ServoTelemetry(
            electrical_position_degrees=7_560.0,
            velocity_erpm=1_100.0,
            q_axis_current_amps=2.0,
            temperature_celsius=31.0,
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
    assert servo.output_torque_newton_meters == pytest.approx(2.0 * 0.095 * 9.0)
    assert servo.motor_torque_newton_meters == pytest.approx(2.0 * 0.095)
    assert servo.q_axis_current_amps == 2.0
    assert servo.temperature_celsius == 31.0
    assert servo.fault_code == 0
    assert "AK80-9 (ID 1)" in str(servo)


@pytest.mark.parametrize(
    ("fault_code", "description"),
    [(6, "MOSFET"), (7, "stall"), (99, "unknown")],
)
def test_update_surfaces_latched_driver_faults(
    can_harness: CanHarness,
    fault_code: int,
    description: str,
) -> None:
    """Translate fault bytes and attempt zero current before raising."""
    servo = enter_servo(make_servo())
    now = time.monotonic()
    servo._accept_telemetry(
        ServoTelemetry(
            fault_code=fault_code,
            received_at_seconds=now,
        )
    )
    servo._accept_telemetry(ServoTelemetry(received_at_seconds=now + 0.01))
    with pytest.raises(MotorFaultError, match=description) as raised:
        servo.update()
    assert can_harness.sent[-1].arbitration_id == 0x101
    assert bytes(can_harness.sent[-1].data) == b"\x00\x00\x00\x00"
    assert raised.value.motor_id == 1
    assert raised.value.fault_code == fault_code
    assert raised.value.description
    servo.update()


def test_update_rejects_stale_telemetry_after_safe_stop(
    can_harness: CanHarness,
) -> None:
    """Attempt zero current when receive-side safety telemetry is stale."""
    servo = enter_servo(make_servo())
    servo._accept_telemetry(ServoTelemetry(received_at_seconds=time.monotonic() - 2.0))
    sent_before_update = len(can_harness.sent)
    with pytest.raises(MotorConnectionError, match="stale"):
        servo.update()
    assert len(can_harness.sent) == sent_before_update + 1
    assert can_harness.sent[-1].arbitration_id == 0x101


def test_listener_errors_surface_on_control_thread_after_safe_stop(
    can_harness: CanHarness,
) -> None:
    """Keep notifier callbacks alive and surface decode failures in update."""
    servo = enter_servo(make_servo())
    servo._accept_listener_error(ValueError("bad frame"))
    with pytest.raises(MotorConnectionError, match="decode"):
        servo.update()
    assert can_harness.sent[-1].arbitration_id == 0x101
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
    can_harness: CanHarness,
    mode: ControlMode,
    expected_id: int,
) -> None:
    """Zero torque modes and hold both position modes before trip."""
    servo = enter_servo(
        make_servo(
            ServoConfig(
                motor=MotorModel.AK80_9,
                motor_id=1,
                can_channel="vcan0",
                connection_timeout_seconds=0.001,
                telemetry_timeout_seconds=1.0,
                max_driver_temperature_celsius=50.0,
                overtemperature_trip_count=3,
                cooldown_margin_celsius=2.0,
            )
        )
    )
    servo.set_control_mode(mode)
    servo._accept_telemetry(
        ServoTelemetry(
            electrical_position_degrees=12.0,
            temperature_celsius=51.0,
            received_at_seconds=time.monotonic(),
        )
    )
    servo.update()
    assert can_harness.sent[-1].arbitration_id == expected_id
    servo._accept_telemetry(
        ServoTelemetry(
            temperature_celsius=49.0,
            received_at_seconds=time.monotonic(),
        )
    )
    servo.update()
    assert servo._thermal_guard_active
    servo._accept_telemetry(
        ServoTelemetry(
            temperature_celsius=48.0,
            received_at_seconds=time.monotonic(),
        )
    )
    servo.update()
    assert not servo._thermal_guard_active


def test_thermal_guard_trips_after_consecutive_samples(
    can_harness: CanHarness,
) -> None:
    """Attempt zero current and raise at the configured sample count."""
    servo = enter_servo(
        make_servo(
            ServoConfig(
                motor=MotorModel.AK80_9,
                motor_id=1,
                can_channel="vcan0",
                connection_timeout_seconds=0.001,
                telemetry_timeout_seconds=1.0,
                max_driver_temperature_celsius=50.0,
                overtemperature_trip_count=2,
            )
        )
    )
    for temperature in (51.0, 52.0):
        servo._accept_telemetry(
            ServoTelemetry(
                temperature_celsius=temperature,
                received_at_seconds=time.monotonic(),
            )
        )
        if temperature == 51.0:
            servo.update()
    with pytest.raises(MotorFaultError, match="2 samples"):
        servo.update()
    assert can_harness.sent[-1].arbitration_id == 0x101


def test_corrupted_internal_control_mode_is_rejected() -> None:
    """Fail explicitly if internal state is corrupted instead of sending."""
    servo = enter_servo(make_servo())
    servo._control_mode = cast(ControlMode, object())
    with pytest.raises(ControlModeError, match="unsupported"):
        servo.update()
