import pytest
from typing import Any, Dict
from unittest.mock import MagicMock, patch

from cubemars_servo_can.servo_can import CubeMarsServoCAN


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
            mock_instance.get_message.side_effect = [stale_msg] + [None] * 120
            mock_reader.return_value = mock_instance

            with patch("time.time", return_value=10.0):
                assert motor.check_can_connection() is False
