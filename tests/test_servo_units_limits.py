import pytest
from typing import Any, Dict
from unittest.mock import patch

from cubemars_servo_can.servo_can import CubeMarsServoCAN


def get_last_message(mock_can: Dict[str, Any]) -> Any:
    """Helper to get the last CAN message sent."""
    args: tuple = mock_can["bus"].send.call_args[0]
    return args[0]


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
