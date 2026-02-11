import pytest
from typing import Generator, Dict, Any
from unittest.mock import MagicMock, patch
from cubemars_servo_can.servo_can import CubeMarsServoCAN


@pytest.fixture
def mock_can() -> Generator[Dict[str, Any], None, None]:
    """Fixture that mocks CAN bus interface."""
    with patch("cubemars_servo_can.can_manager.can") as mock_can_lib:
        mock_bus: MagicMock = MagicMock()
        mock_notifier: MagicMock = MagicMock()

        # Create a simple Message class to capture constructor arguments
        class MockMessage:
            def __init__(self, arbitration_id=None, data=None, is_extended_id=False):
                self.arbitration_id = arbitration_id
                self.data = data if data is not None else []
                self.is_extended_id = is_extended_id

        # Configure can.Message to return MockMessage instances
        mock_can_lib.Message.side_effect = MockMessage

        mock_can_lib.interface.Bus.return_value = mock_bus
        mock_can_lib.Notifier.return_value = mock_notifier

        yield {
            "can": mock_can_lib,
            "bus": mock_bus,
            "notifier": mock_notifier,
        }


def get_last_message(mock_can: Dict[str, Any]) -> Any:
    """Helper to get the last CAN message sent."""
    args: tuple = mock_can["bus"].send.call_args[0]
    return args[0]


class TestInitialization:
    """Tests for motor initialization."""

    def test_initialization_with_can_channel(self, mock_can: Dict[str, Any]) -> None:
        """Test that motor initializes with correct CAN channel."""
        CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1, can_channel="vcan0")
        mock_can["can"].interface.Bus.assert_called_with(
            channel="vcan0", bustype="socketcan"
        )

    def test_initialization_default_channel(self, mock_can: Dict[str, Any]) -> None:
        """Test that motor initializes with default can0 channel."""
        CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        mock_can["can"].interface.Bus.assert_called_with(
            channel="can0", bustype="socketcan"
        )

    def test_default_log_vars_are_isolated_per_instance(
        self, mock_can: Dict[str, Any]
    ) -> None:
        from cubemars_servo_can.constants import DEFAULT_LOG_VARIABLES

        motor1 = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor2 = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=2)

        motor1.log_vars.append("custom_metric")

        assert "custom_metric" not in motor2.log_vars
        assert "custom_metric" not in DEFAULT_LOG_VARIABLES


class TestEnterExit:
    """Tests for context manager enter/exit."""

    def test_enter_sets_power_on(self, mock_can: Dict[str, Any]) -> None:
        """Test that entering context sends power on command."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)

        with patch.object(motor, "check_can_connection", return_value=True):
            with motor:
                assert motor._entered is True

        # Power on (0xFC) and power off (0xFD) should be sent
        calls = mock_can["bus"].send.call_args_list
        assert len(calls) >= 2

    def test_exit_sends_power_off(self, mock_can: Dict[str, Any]) -> None:
        """Test that exiting context sends power off command."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)

        with patch.object(motor, "check_can_connection", return_value=True):
            with motor:
                pass

        # Verify send was called at least twice (power on and power off)
        # Note: Can't inspect message.data directly as can.Message is mocked
        assert mock_can["bus"].send.call_count >= 2


class TestControlModes:
    """Tests for all control modes."""

    def test_position_mode_sends_correct_command(
        self, mock_can: Dict[str, Any]
    ) -> None:
        """Test that position mode sends correctly formatted CAN message."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_position_control()

        # Set position to 90 degrees (pi/2 radians)
        motor.set_motor_angle_radians(1.5708)
        motor.update()

        message = get_last_message(mock_can)
        # Arbitration ID should be motor_id | (SET_POS << 8)
        # SET_POS = 4, so expected: 1 | (4 << 8) = 0x401
        assert message.arbitration_id == 0x401

        # Position is scaled by 1,000,000 and converted to int32
        # 1.5708 rad / rad_per_Eang * 1000000 = expected value
        # This verifies the command was actually formatted, not just sent
        assert len(message.data) == 4  # int32 is 4 bytes

    def test_velocity_mode_sends_correct_command(
        self, mock_can: Dict[str, Any]
    ) -> None:
        """Test that velocity mode sends correctly formatted CAN message."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_velocity_control()

        # Set velocity to 10 rad/s
        motor.set_motor_velocity_radians_per_second(10.0)
        motor.update()

        message = get_last_message(mock_can)
        # SET_RPM = 3, so expected: 1 | (3 << 8) = 0x301
        assert message.arbitration_id == 0x301
        assert len(message.data) == 4

    def test_current_mode_sends_correct_command(self, mock_can: Dict[str, Any]) -> None:
        """Test that current mode sends correctly formatted CAN message."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_current_control()

        # Set current to 5A
        motor.set_motor_current_qaxis_amps(5.0)
        motor.update()

        message = get_last_message(mock_can)
        # SET_CURRENT = 1, so expected: 1 | (1 << 8) = 0x101
        assert message.arbitration_id == 0x101
        assert len(message.data) == 4

    def test_duty_cycle_mode_sends_correct_command(
        self, mock_can: Dict[str, Any]
    ) -> None:
        """Test that duty cycle mode sends correctly formatted CAN message."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_duty_cycle_control()

        # Set duty to 50%
        motor.set_duty_cycle_percent(0.5)
        motor.update()

        message = get_last_message(mock_can)
        # SET_DUTY = 0, so expected: 1 | (0 << 8) = 0x001
        assert message.arbitration_id == 0x001
        assert len(message.data) == 4

    def test_current_brake_mode_sends_correct_command(
        self, mock_can: Dict[str, Any]
    ) -> None:
        """Test that current brake mode sends correctly formatted CAN message."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_current_brake_control()

        motor.set_motor_current_qaxis_amps(5.0)
        motor.update()

        message = get_last_message(mock_can)
        # SET_CURRENT_BRAKE = 2, so expected: 1 | (2 << 8) = 0x201
        assert message.arbitration_id == 0x201
        assert len(message.data) == 4

    def test_position_velocity_mode_sends_correct_command(
        self, mock_can: Dict[str, Any]
    ) -> None:
        """Test that position-velocity mode sends a full 8-byte command frame."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_position_velocity_control()

        target_pos = 1.0
        target_vel = 5.0
        target_acc = 10.0
        motor.set_output_angle_radians(target_pos, target_vel, target_acc)
        motor.update()

        message = get_last_message(mock_can)
        # SET_POS_SPD = 6, so expected: 1 | (6 << 8) = 0x601
        assert message.arbitration_id == 0x601
        assert len(message.data) == 8

        import struct

        pos_raw = struct.unpack(">i", bytes(message.data[0:4]))[0]
        vel_raw = struct.unpack(">h", bytes(message.data[4:6]))[0]
        acc_raw = struct.unpack(">h", bytes(message.data[6:8]))[0]

        expected_pos_raw = int((target_pos / motor.rad_per_Eang) * 10000.0)
        expected_vel_raw = int(target_vel / motor.radps_per_ERPM)
        expected_acc_raw = int(target_acc / motor.radps_per_ERPM)

        assert pos_raw == expected_pos_raw
        assert vel_raw == expected_vel_raw
        assert acc_raw == expected_acc_raw


class TestSafetyLimits:
    """Tests for safety limits and error handling."""

    def test_position_limit_raises_error(self, mock_can: Dict[str, Any]) -> None:
        """Test that exceeding position limit raises RuntimeError."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_position_control()

        # Try to set position beyond P_max (32000)
        # Must account for gear ratio: 32000 * 9.0 = 288000
        with pytest.raises(RuntimeError, match="Cannot control using position mode"):
            motor.set_motor_angle_radians(300000.0)

    def test_velocity_limit_raises_error(self, mock_can: Dict[str, Any]) -> None:
        """Test that exceeding velocity limit raises RuntimeError."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_velocity_control()

        # Try to set velocity beyond V_max (32000)
        # When setting motor velocity, it's divided by gear ratio before limit check
        # So we need to use a value that exceeds V_max * GEAR_RATIO
        with pytest.raises(RuntimeError, match="Cannot control using speed mode"):
            motor.set_motor_velocity_radians_per_second(350000.0)

    def test_duty_cycle_over_100_percent_raises_error(
        self, mock_can: Dict[str, Any]
    ) -> None:
        """Test that duty cycle > 100% raises RuntimeError."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_duty_cycle_control()

        with pytest.raises(
            RuntimeError,
            match="Cannot control using duty cycle mode for duty cycles greater than 100%",
        ):
            motor.set_duty_cycle_percent(1.5)

    def test_wrong_mode_raises_error(self, mock_can: Dict[str, Any]) -> None:
        """Test that sending position command in velocity mode raises error."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_velocity_control()

        with pytest.raises(
            RuntimeError,
            match="Attempted to send position command without entering position control",
        ):
            motor.set_motor_angle_radians(1.0)


class TestTorqueCalculation:
    """Tests for torque calculations."""

    def test_torque_to_current_conversion(self, mock_can: Dict[str, Any]) -> None:
        """Test that torque commands are converted to current correctly."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_current_control()

        # Set torque to 1.0 Nm
        motor.set_motor_torque_newton_meters(1.0)
        motor.update()

        message = get_last_message(mock_can)
        assert message.arbitration_id == 0x101  # Current mode

        # Verify current was calculated correctly:
        # Motor torque 1.0 Nm -> Output torque = 1.0 * 9.0 (GEAR_RATIO) = 9.0 Nm
        # Current = Output torque / Kt_actual / GEAR_RATIO = 9.0 / 0.115 / 9.0 = ~8.696 A
        # Current is sent as int32(current * 1000)
        import struct

        current_int = struct.unpack(">i", bytes(message.data))[0]
        expected_current = int((9.0 / 0.115 / 9.0) * 1000)
        assert abs(current_int - expected_current) < 10  # Allow small rounding error


class TestTelemetry:
    """Tests for telemetry and state reading."""

    def test_temperature_getter(self, mock_can: Dict[str, Any]) -> None:
        """Test that temperature property reads from state."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True

        # Manually set temperature in state
        motor._motor_state.temperature = 45.5

        assert motor.temperature == 45.5

    def test_error_getter(self, mock_can: Dict[str, Any]) -> None:
        """Test that error property reads from state."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True

        # Test no error
        motor._motor_state.error = 0
        assert motor.error == 0

        # Test over temperature error
        motor._motor_state.error = 1
        assert motor.error == 1


class TestDtCalculation:
    """Tests for dt calculation and acceleration computation."""

    def test_acceleration_calculation_with_positive_dt(
        self, mock_can: Dict[str, Any]
    ) -> None:
        """Test that dt is calculated correctly (positive) for acceleration."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True

        # Set initial state
        from cubemars_servo_can.motor_state import ServoMotorState

        motor._motor_state_async.velocity = 0.0
        motor._last_update_time = 0.0

        # Simulate a state update with known velocity change
        new_state = ServoMotorState(
            position=0.0,
            velocity=10.0,  # Changed by 10 rad/s
            current=0.0,
            temperature=25.0,
            error=0,
            acceleration=0.0,
        )

        # Use a time value significantly larger than 1e-9 to ensure dt is not too small
        # Set _last_update_time to a value that will give us dt = 1.0
        motor._last_update_time = 0.0

        with patch("time.time", return_value=1.0):  # 1 second later
            motor._update_state_async(new_state)

        # Acceleration should be positive: (10 - 0) / 1 = 10 rad/s^2
        assert motor._motor_state_async.acceleration == 10.0

    def test_acceleration_zero_on_instant_update(
        self, mock_can: Dict[str, Any]
    ) -> None:
        """Test that acceleration is zero when dt is very small (avoids division by zero)."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True

        from cubemars_servo_can.motor_state import ServoMotorState

        motor._motor_state_async.velocity = 5.0
        motor._last_update_time = 100.0

        new_state = ServoMotorState(
            position=0.0,
            velocity=10.0,
            current=0.0,
            temperature=25.0,
            error=0,
            acceleration=0.0,
        )

        # Update with almost no time passed (smaller than 1e-9 threshold)
        with patch("time.time", return_value=100.0000000005):  # 0.5 nanoseconds later
            motor._update_state_async(new_state)

        # Should be 0.0 to avoid division by very small dt
        assert motor._motor_state_async.acceleration == 0.0


class TestVelocityUnitConversion:
    """Tests for velocity unit conversion."""

    def test_motor_velocity_includes_conversion_factor(
        self, mock_can: Dict[str, Any]
    ) -> None:
        """Test that motor velocity includes radps_per_ERPM conversion."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True

        # Set raw ERPM in state
        motor._motor_state.velocity = 1000.0  # ERPM

        motor_velocity = motor.get_motor_velocity_radians_per_second()

        # Should be: ERPM * radps_per_ERPM * GEAR_RATIO
        # 1000 * 0.000582 * 9.0 = 5.238 rad/s
        expected = 1000.0 * 0.000582 * 9.0
        assert abs(motor_velocity - expected) < 0.01


class TestVelocityLimitUnits:
    """Tests for velocity limit checking in correct units."""

    def test_velocity_limit_in_correct_units(self, mock_can: Dict[str, Any]) -> None:
        """Test that velocity limit is compared in correct units (rad/s)."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_velocity_control()

        # V_max for AK80-9 is 32000 ERPM
        # Converted to rad/s: 32000 * 0.000582 = 18.624 rad/s
        # With gear ratio for motor-side: 18.624 * 9.0 = 167.6 rad/s
        v_max_rad_s = 32000 * 0.000582 * 9.0  # ~167.6 rad/s

        # Should raise error for velocity beyond limit
        with pytest.raises(RuntimeError, match="Cannot control using speed mode"):
            motor.set_motor_velocity_radians_per_second(v_max_rad_s + 10.0)

    def test_velocity_within_limit_passes(self, mock_can: Dict[str, Any]) -> None:
        """Test that velocity within limit doesn't raise error."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_velocity_control()

        # Should not raise error
        motor.set_motor_velocity_radians_per_second(50.0)  # Well within limit


class TestPositionLimitUnits:
    """Tests for position limit checking in correct units."""

    def test_position_limit_in_correct_units(self, mock_can: Dict[str, Any]) -> None:
        """Test that position limit is compared in correct units (radians)."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_position_control()

        # P_max for AK80-9 is 32000 (electrical units)
        # Converted to output radians: 32000 * (pi/21) / 9.0 = 531.9 rad
        # This is the output-side limit
        p_max_output_rad = 32000 * (3.14159 / 21) / 9.0  # ~531.9 rad

        # Should raise error for output position beyond limit
        # Note: set_output_angle_radians is called directly here
        with pytest.raises(RuntimeError, match="Cannot control using position mode"):
            motor.set_output_angle_radians(p_max_output_rad + 10.0)

    def test_position_within_limit_passes(self, mock_can: Dict[str, Any]) -> None:
        """Test that position within limit doesn't raise error."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_position_control()

        # Should not raise error (output angle within limit)
        motor.set_output_angle_radians(100.0)  # Well within limit of ~531.9 rad


class TestTorqueFormula:
    """Tests for torque calculation formula."""

    def test_motor_torque_correct_formula(self, mock_can: Dict[str, Any]) -> None:
        """Test that motor torque correctly accounts for gear ratio."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_current_control()

        # Set motor torque to 1.0 Nm
        motor.set_motor_torque_newton_meters(1.0)
        motor.update()

        message = get_last_message(mock_can)

        # Motor torque 1.0 Nm -> Output torque = 1.0 * 9.0 = 9.0 Nm
        # Current = 9.0 / 0.115 / 9.0 = 8.696 A
        import struct

        current_int = struct.unpack(">i", bytes(message.data))[0]
        expected_current = int((9.0 / 0.115 / 9.0) * 1000)

        assert abs(current_int - expected_current) < 10


class TestTorqueGetterUnits:
    """Tests for motor/output torque getters."""

    def test_motor_torque_getter_divides_by_gear_ratio(
        self, mock_can: Dict[str, Any]
    ) -> None:
        """Motor-side torque should not include gear ratio amplification."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True

        # Motor torque = Iq * Kt_actual
        motor._motor_state.current = 10.0
        expected_motor_torque = 10.0 * motor.config.Kt_actual
        expected_output_torque = expected_motor_torque * motor.config.GEAR_RATIO

        assert motor.get_output_torque_newton_meters() == pytest.approx(
            expected_output_torque
        )
        assert motor.get_motor_torque_newton_meters() == pytest.approx(
            expected_motor_torque
        )


class TestAccelerationUnitConversion:
    """Tests for acceleration unit conversion."""

    def test_acceleration_getters_convert_erpm_per_second(
        self, mock_can: Dict[str, Any]
    ) -> None:
        """Acceleration from telemetry is ERPM/s and must be converted to rad/s^2."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor._motor_state.acceleration = 1000.0  # ERPM/s

        expected_output_acc = 1000.0 * motor.radps_per_ERPM
        expected_motor_acc = expected_output_acc * motor.config.GEAR_RATIO

        assert (
            motor.get_output_acceleration_radians_per_second_squared()
            == pytest.approx(expected_output_acc)
        )
        assert (
            motor.get_motor_acceleration_radians_per_second_squared()
            == pytest.approx(expected_motor_acc)
        )


class TestCurrentLimits:
    """Tests for current limit enforcement."""

    def test_current_limit_enforced(self, mock_can: Dict[str, Any]) -> None:
        """Test that current limits are enforced."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_current_control()

        # Curr_max is 1500 (scaled by 100 in config), so limit is 15A
        with pytest.raises(RuntimeError, match="Current.*out of range"):
            motor.set_motor_current_qaxis_amps(20.0)  # Beyond 15A limit

    def test_current_within_limit_passes(self, mock_can: Dict[str, Any]) -> None:
        """Test that current within limit doesn't raise error."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_current_control()

        # Should not raise error (within 15A limit)
        motor.set_motor_current_qaxis_amps(10.0)


class TestCurrentBrakeMode:
    """Tests specific to current brake mode semantics."""

    def test_negative_brake_current_rejected(self, mock_can: Dict[str, Any]) -> None:
        """Brake mode should reject negative current commands."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_current_brake_control()

        with pytest.raises(RuntimeError, match="requires non-negative current"):
            motor.set_motor_current_qaxis_amps(-1.0)

    def test_non_negative_brake_current_allowed(self, mock_can: Dict[str, Any]) -> None:
        """Brake mode should allow non-negative current within limits."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_current_brake_control()
        motor.set_motor_current_qaxis_amps(5.0)


class TestPositionVelocityModeLimits:
    """Tests for position-velocity command limit checks."""

    def test_position_velocity_rejects_speed_beyond_limit(
        self, mock_can: Dict[str, Any]
    ) -> None:
        """Velocity limit checks should apply in position-velocity mode too."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_position_velocity_control()

        v_max_rad_s = motor.config.V_max * motor.radps_per_ERPM
        with pytest.raises(RuntimeError, match="position-velocity mode for velocity"):
            motor.set_output_angle_radians(1.0, v_max_rad_s + 0.1, 1.0)

    def test_position_velocity_rejects_acceleration_int16_overflow(
        self, mock_can: Dict[str, Any]
    ) -> None:
        """Acceleration must fit the int16 field used by the CAN frame."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_position_velocity_control()

        # Converts to ERPM/s far above int16 range.
        with pytest.raises(RuntimeError, match="outside int16 range"):
            motor.set_output_angle_radians(1.0, 1.0, 1e6)


class TestTorqueLimits:
    """Tests for torque limit enforcement."""

    def test_torque_limit_enforced(self, mock_can: Dict[str, Any]) -> None:
        """Test that torque limits are enforced."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_current_control()

        # T_max is 30 Nm for AK80-9
        with pytest.raises(RuntimeError, match="Torque.*out of range"):
            motor.set_output_torque_newton_meters(40.0)  # Beyond 30 Nm limit

    def test_torque_within_limit_passes(self, mock_can: Dict[str, Any]) -> None:
        """Test that torque within limit doesn't raise error."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_current_control()

        # Should not raise error (within 30 Nm limit and resulting current within 15A)
        # 10 Nm -> 10 / 0.115 / 9.0 = 9.66A (within 15A limit)
        motor.set_output_torque_newton_meters(10.0)


class TestCheckCanConnection:
    """Tests for CAN connection checking."""

    def test_check_connection_returns_false_when_no_response(
        self, mock_can: Dict[str, Any]
    ) -> None:
        """Test that check_can_connection returns False when no motor response."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True

        # Mock BufferedReader to return no messages
        with patch("cubemars_servo_can.servo_can.can.BufferedReader") as mock_reader:
            mock_instance = MagicMock()
            mock_instance.get_message.return_value = None
            mock_reader.return_value = mock_instance

            result = motor.check_can_connection()
            assert result is False

    def test_check_connection_returns_true_with_response(
        self, mock_can: Dict[str, Any]
    ) -> None:
        """Test that check_can_connection returns True when motor responds."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True

        # Create a mock message from the motor
        mock_msg = MagicMock()
        mock_msg.arbitration_id = 0x001  # Matches motor ID 1

        # Mock BufferedReader to return a message
        with patch("cubemars_servo_can.servo_can.can.BufferedReader") as mock_reader:
            mock_instance = MagicMock()
            mock_instance.get_message.side_effect = [mock_msg, None]
            mock_reader.return_value = mock_instance

            result = motor.check_can_connection()
            assert result is True


class TestErrorHandling:
    """Tests for error state handling."""

    def test_error_state_raised_on_update_thread(
        self, mock_can: Dict[str, Any]
    ) -> None:
        """Error frames are captured async and raised when user calls update()."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True

        # Create a mock state with an error
        from cubemars_servo_can.motor_state import ServoMotorState

        error_state = ServoMotorState(
            position=0.0,
            velocity=0.0,
            current=0.0,
            temperature=50.0,
            error=1,
            acceleration=0.0,  # Error 1 = Over temperature
        )

        motor._update_state_async(error_state)
        with pytest.raises(RuntimeError, match="Over temperature fault"):
            motor.update()

    def test_listener_error_raised_on_update_thread(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True

        motor._set_listener_error(ValueError("bad frame"))
        with pytest.raises(RuntimeError, match="CAN listener error"):
            motor.update()


class TestContextManagerAndUpdateBranches:
    """Tests for context manager, CSV logging, and update error paths."""

    def test_enter_with_csv_creates_writer(
        self, mock_can: Dict[str, Any], tmp_path
    ) -> None:
        csv_path = tmp_path / "motor_log.csv"
        motor: CubeMarsServoCAN = CubeMarsServoCAN(
            motor_type="AK80-9", motor_ID=1, CSV_file=str(csv_path)
        )

        with patch.object(motor, "check_can_connection", return_value=True):
            with motor:
                assert motor.csv_file is not None

        content = csv_path.read_text().strip().splitlines()
        assert content[0].startswith("pi_time,")

    def test_enter_raises_if_connection_check_fails(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        with patch.object(motor, "check_can_connection", return_value=False):
            with pytest.raises(RuntimeError, match="Device not connected"):
                motor.__enter__()
        assert motor._entered is False

    def test_enter_failure_after_power_on_still_powers_off(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)

        with patch.object(
            motor, "_send_command", side_effect=RuntimeError("send boom")
        ):
            with patch.object(motor, "power_off") as power_off:
                with pytest.raises(RuntimeError, match="send boom"):
                    motor.__enter__()
                power_off.assert_called_once()

        assert motor._entered is False

    def test_enter_failure_closes_csv_and_swallows_power_off_failure(
        self, mock_can: Dict[str, Any], tmp_path
    ) -> None:
        csv_path = tmp_path / "motor_log.csv"
        motor: CubeMarsServoCAN = CubeMarsServoCAN(
            motor_type="AK80-9", motor_ID=1, CSV_file=str(csv_path)
        )
        with patch.object(motor, "check_can_connection", return_value=False):
            with patch.object(
                motor, "power_off", side_effect=RuntimeError("off failed")
            ):
                with pytest.raises(RuntimeError, match="Device not connected"):
                    motor.__enter__()

        assert motor._entered is False
        assert motor.csv_file is None

    def test_exit_closes_csv_and_prints_traceback(
        self, mock_can: Dict[str, Any], tmp_path
    ) -> None:
        csv_path = tmp_path / "motor_log.csv"
        motor: CubeMarsServoCAN = CubeMarsServoCAN(
            motor_type="AK80-9", motor_ID=1, CSV_file=str(csv_path)
        )
        with patch.object(motor, "check_can_connection", return_value=True):
            motor.__enter__()

        err = RuntimeError("boom")
        with patch("cubemars_servo_can.servo_can.traceback.print_exception") as pe:
            motor.__exit__(RuntimeError, err, None)
            pe.assert_called_once()
        assert motor.csv_file is None
        assert motor._entered is False

    def test_update_raises_when_not_entered(self, mock_can: Dict[str, Any]) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        with pytest.raises(RuntimeError, match="before safely powering on"):
            motor.update()

    def test_update_raises_when_temperature_exceeds_limit(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor._motor_state.temperature = motor.max_temp + 1.0
        with pytest.raises(RuntimeError, match="Temperature greater than"):
            motor.update()

    def test_update_warns_on_stale_state(self, mock_can: Dict[str, Any]) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor._last_command_time = 9.9
        motor._last_update_time = 9.0
        with patch("time.time", return_value=10.0):
            with pytest.warns(
                RuntimeWarning, match="State update requested but no data"
            ):
                motor.update()

    def test_update_writes_csv_row(self, mock_can: Dict[str, Any], tmp_path) -> None:
        csv_path = tmp_path / "motor_log.csv"
        motor: CubeMarsServoCAN = CubeMarsServoCAN(
            motor_type="AK80-9", motor_ID=1, CSV_file=str(csv_path)
        )
        with patch.object(motor, "check_can_connection", return_value=True):
            motor.__enter__()

        motor._motor_state_async.position = 1.0
        motor._motor_state_async.velocity = 2.0
        motor._motor_state_async.current = 3.0
        motor._motor_state_async.temperature = 4.0
        motor.update()
        motor.__exit__(None, None, None)

        lines = csv_path.read_text().strip().splitlines()
        assert len(lines) >= 2


class TestMiscServoBranches:
    """Tests for remaining branch coverage in servo API."""

    def test_qaxis_current_to_tmotor_current(self, mock_can: Dict[str, Any]) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        iq = 2.0
        expected = (
            iq
            * (motor.config.GEAR_RATIO * motor.config.Kt_TMotor)
            / motor.config.Current_Factor
        )
        assert motor.qaxis_current_to_TMotor_current(iq) == pytest.approx(expected)

    def test_set_zero_position_sends_origin(self, mock_can: Dict[str, Any]) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        before = motor._last_command_time
        motor.set_zero_position()
        assert motor._last_command_time is not None
        assert motor._last_command_time != before

    def test_getters_cover_output_and_motor_units(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._motor_state.position = 10.0
        motor._motor_state.velocity = 20.0

        assert motor.get_output_angle_radians() == pytest.approx(
            10.0 * motor.rad_per_Eang
        )
        assert motor.get_output_velocity_radians_per_second() == pytest.approx(
            20.0 * motor.radps_per_ERPM
        )
        assert motor.get_motor_angle_radians() == pytest.approx(
            10.0 * motor.rad_per_Eang * motor.config.GEAR_RATIO
        )

    def test_enter_idle_mode_sets_state(self, mock_can: Dict[str, Any]) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor.enter_idle_mode()
        assert motor._control_state.name == "IDLE"

    def test_mode_specific_errors(self, mock_can: Dict[str, Any]) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_velocity_control()
        with pytest.raises(RuntimeError, match="position command"):
            motor.set_output_angle_radians(0.1)

        motor.enter_position_control()
        with pytest.raises(RuntimeError, match="duty cycle command"):
            motor.set_duty_cycle_percent(0.1)

        motor.enter_position_control()
        with pytest.raises(RuntimeError, match="speed command"):
            motor.set_output_velocity_radians_per_second(1.0)

        motor.enter_velocity_control()
        with pytest.raises(RuntimeError, match="current command before entering"):
            motor.set_motor_current_qaxis_amps(1.0)

    def test_position_velocity_rejects_speed_int16_overflow(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK40-10", motor_ID=1)
        motor._entered = True
        motor.enter_position_velocity_control()

        # For AK40-10, V_max allows up to ~34.9 rad/s output, enough to overflow int16 ERPM.
        with pytest.raises(RuntimeError, match="speed command .* outside int16 range"):
            motor.set_output_angle_radians(1.0, 30.0, 1.0)

    def test_send_command_invalid_control_state(self, mock_can: Dict[str, Any]) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._control_state = 999  # type: ignore[assignment]
        with pytest.raises(RuntimeError, match="UNDEFINED STATE"):
            motor._send_command()

    def test_str_representation(self, mock_can: Dict[str, Any]) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        text = str(motor)
        assert "ID:" in text
        assert "Position:" in text

    def test_check_can_connection_requires_entered(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        with pytest.raises(RuntimeError, match="before entering motor control"):
            motor.check_can_connection()

    def test_check_can_connection_ignores_stale_timestamped_messages(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True

        stale_msg = MagicMock()
        stale_msg.arbitration_id = 0x001
        stale_msg.timestamp = 1.0

        with patch("cubemars_servo_can.servo_can.can.BufferedReader") as mock_reader:
            mock_instance = MagicMock()
            mock_instance.get_message.side_effect = [stale_msg, None]
            mock_reader.return_value = mock_instance

            with patch("time.time", return_value=10.0):
                assert motor.check_can_connection() is False
