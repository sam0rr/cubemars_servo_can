from typing import Any, Dict
from unittest.mock import MagicMock, patch

import pytest

from cubemars_servo_can.servo_can import CubeMarsServoCAN


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
        mock_msg.data = bytes([0] * 8)

        # Mock BufferedReader to return a message
        with patch("cubemars_servo_can.servo_can.can.BufferedReader") as mock_reader:
            mock_instance = MagicMock()
            mock_instance.get_message.side_effect = [mock_msg, None]
            mock_reader.return_value = mock_instance

            result = motor.check_can_connection()
            assert result is True

    def test_check_connection_accepts_compatible_non_command_id(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True

        compat_msg = MagicMock()
        compat_msg.arbitration_id = 0x901
        compat_msg.data = bytes([0] * 8)

        with patch("cubemars_servo_can.servo_can.can.BufferedReader") as mock_reader:
            mock_instance = MagicMock()
            mock_instance.get_message.side_effect = [compat_msg, None]
            mock_reader.return_value = mock_instance

            assert motor.check_can_connection() is True

    def test_check_connection_rejects_command_like_low_byte_id(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True

        spoof_msg = MagicMock()
        spoof_msg.arbitration_id = 0x601
        spoof_msg.timestamp = 11.0
        spoof_msg.data = bytes([0] * 8)

        with patch("cubemars_servo_can.servo_can.can.BufferedReader") as mock_reader:
            mock_instance = MagicMock()
            mock_instance.get_message.side_effect = [spoof_msg] + [None] * 120
            mock_reader.return_value = mock_instance

            with patch("time.time", return_value=10.0):
                assert motor.check_can_connection() is False

    def test_check_connection_rejects_power_on_echo(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True

        power_echo = MagicMock()
        power_echo.arbitration_id = 0x001
        power_echo.data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])

        with patch("cubemars_servo_can.servo_can.can.BufferedReader") as mock_reader:
            mock_instance = MagicMock()
            mock_instance.get_message.side_effect = [power_echo] + [None] * 120
            mock_reader.return_value = mock_instance

            assert motor.check_can_connection() is False

    def test_check_connection_ignores_non_8_byte_frames(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True

        short_frame = MagicMock()
        short_frame.arbitration_id = 0x001
        short_frame.data = bytes([0] * 7)

        with patch("cubemars_servo_can.servo_can.can.BufferedReader") as mock_reader:
            mock_instance = MagicMock()
            mock_instance.get_message.side_effect = [short_frame] + [None] * 120
            mock_reader.return_value = mock_instance

            assert motor.check_can_connection() is False

    def test_check_connection_ignores_unparseable_frames(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True

        bad_frame = MagicMock()
        bad_frame.arbitration_id = 0x001
        bad_frame.data = bytes([0] * 8)

        with patch("cubemars_servo_can.servo_can.can.BufferedReader") as mock_reader:
            mock_instance = MagicMock()
            mock_instance.get_message.side_effect = [bad_frame] + [None] * 120
            mock_reader.return_value = mock_instance

            with patch.object(
                motor._canman,
                "parse_servo_message",
                side_effect=ValueError("bad frame"),
            ):
                assert motor.check_can_connection() is False

    def test_check_connection_busy_stream_is_bounded(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True

        noisy_msg = MagicMock()
        noisy_msg.arbitration_id = 0x002
        noisy_msg.timestamp = 10.1
        noisy_msg.data = bytes([0] * 8)

        with patch("cubemars_servo_can.servo_can.can.BufferedReader") as mock_reader:
            mock_instance = MagicMock()
            mock_instance.get_message.return_value = noisy_msg
            mock_reader.return_value = mock_instance

            with patch("time.time", return_value=10.0):
                assert motor.check_can_connection() is False
            assert mock_instance.get_message.call_count == 200


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
