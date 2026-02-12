import pytest
import struct
from typing import Dict, Any
from unittest.mock import patch

from cubemars_servo_can.servo_can import CubeMarsServoCAN


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

    def test_invalid_overtemp_trip_count_raises(self, mock_can: Dict[str, Any]) -> None:
        with pytest.raises(ValueError, match="overtemp_trip_count must be >= 1"):
            CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1, overtemp_trip_count=0)

    @pytest.mark.parametrize(
        ("kwargs", "match"),
        [
            (
                {"thermal_guard_cooldown_hysteresis_c": -0.1},
                "thermal_guard_cooldown_hysteresis_c must be >= 0",
            ),
            (
                {"soft_stop_ramp_duration_s": -0.01},
                "soft_stop_ramp_duration_s must be >= 0",
            ),
            ({"soft_stop_ramp_steps": 0}, "soft_stop_ramp_steps must be >= 1"),
            (
                {"soft_stop_brake_hold_current_amps": -0.1},
                "soft_stop_brake_hold_current_amps must be >= 0",
            ),
            (
                {"soft_stop_brake_hold_current_amps": 60.1},
                "soft_stop_brake_hold_current_amps must be <= 60",
            ),
            (
                {"soft_stop_brake_hold_duration_s": -0.01},
                "soft_stop_brake_hold_duration_s must be >= 0",
            ),
        ],
    )
    def test_invalid_soft_stop_and_thermal_guard_parameters_raise(
        self, mock_can: Dict[str, Any], kwargs: Dict[str, float], match: str
    ) -> None:
        with pytest.raises(ValueError, match=match):
            CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1, **kwargs)


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

        assert len(message.data) == 4  # int32 is 4 bytes
        pos_raw = struct.unpack(">i", bytes(message.data))[0]
        expected_pos_raw = int(
            (1.5708 / motor.config.GEAR_RATIO / motor.rad_per_Eang) * 10000.0
        )
        assert pos_raw == expected_pos_raw

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

    def test_velocity_limit_accepts_exact_max(self, mock_can: Dict[str, Any]) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_velocity_control()

        v_max_rad_s = motor.config.V_max * motor.radps_per_ERPM
        motor.set_output_velocity_radians_per_second(v_max_rad_s)

        assert motor._command.velocity == pytest.approx(motor.config.V_max)

    def test_velocity_limit_clamps_tiny_overage(self, mock_can: Dict[str, Any]) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_velocity_control()

        v_max_rad_s = motor.config.V_max * motor.radps_per_ERPM
        tiny_over = motor.radps_per_ERPM * 5e-4
        motor.set_output_velocity_radians_per_second(v_max_rad_s + tiny_over)

        assert motor._command.velocity == pytest.approx(motor.config.V_max)

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
