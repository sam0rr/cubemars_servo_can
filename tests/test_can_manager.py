"""Tests for CAN manager functionality."""

import pytest
from typing import Generator, Dict, Any
from unittest.mock import MagicMock, patch
from cubemars_servo_can.can_manager import CAN_Manager_servo


@pytest.fixture
def mock_can() -> Generator[Dict[str, Any], None, None]:
    """Fixture that mocks CAN bus interface."""
    with patch("cubemars_servo_can.can_manager.can") as mock_can_lib:
        mock_bus: MagicMock = MagicMock()
        mock_notifier: MagicMock = MagicMock()

        class MockMessage:
            def __init__(self, arbitration_id=None, data=None, is_extended_id=False):
                self.arbitration_id = arbitration_id
                self.data = data if data is not None else []
                self.is_extended_id = is_extended_id

        mock_can_lib.Message.side_effect = MockMessage
        mock_can_lib.interface.Bus.return_value = mock_bus
        mock_can_lib.Notifier.return_value = mock_notifier

        yield {"can": mock_can_lib, "bus": mock_bus, "notifier": mock_notifier}


class TestBufferOverflow:
    """Tests for buffer overflow protection."""

    def test_buffer_overflow_raises_error(self, mock_can: Dict[str, Any]) -> None:
        """Test that buffer with more than 8 bytes raises error."""
        can_manager = CAN_Manager_servo(channel="vcan0")

        # Try to send more than 8 bytes
        with pytest.raises(ValueError, match="Data buffer has 10 bytes"):
            can_manager.send_servo_message(
                motor_id=1,
                data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF],
                data_len=0,
            )

    def test_buffer_8_bytes_allowed(self, mock_can: Dict[str, Any]) -> None:
        """Test that buffer with exactly 8 bytes is allowed."""
        can_manager = CAN_Manager_servo(channel="vcan0")

        # Should not raise error
        can_manager.send_servo_message(
            motor_id=1,
            data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF],
            data_len=0,
        )


class TestTemperatureParsing:
    """Tests for temperature parsing."""

    def test_temperature_parsed_as_uint8(self, mock_can: Dict[str, Any]) -> None:
        """Test that temperature is parsed as unsigned 8-bit value."""
        can_manager = CAN_Manager_servo(channel="vcan0")

        # Create data with temperature byte = 0xFF (255 as unsigned, -1 as signed)
        data = bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00])

        state = can_manager.parse_servo_message(data)

        # Should be 255.0 (unsigned), not -1.0 (signed)
        assert state.temperature == 255.0

    def test_temperature_normal_range(self, mock_can: Dict[str, Any]) -> None:
        """Test that normal temperature values are parsed correctly."""
        can_manager = CAN_Manager_servo(channel="vcan0")

        # Temperature = 50°C
        data = bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x00])

        state = can_manager.parse_servo_message(data)

        assert state.temperature == 50.0


class TestStateParsing:
    """Tests for parsing incoming CAN messages."""

    def test_parse_servo_message(self, mock_can: Dict[str, Any]) -> None:
        """Test that incoming CAN messages are parsed correctly."""
        can_manager = CAN_Manager_servo(channel="vcan0")

        # Create mock data (position=100, velocity=50, current=5, temp=40, error=0)
        # Position: 100 * 10 = 1000 (0x03E8)
        # Velocity: 50 / 10 = 5 (0x0005)
        # Current: 5 / 0.01 = 500 (0x01F4)
        # Temp: 40 (0x28)
        # Error: 0
        data = bytes([0x03, 0xE8, 0x00, 0x05, 0x01, 0xF4, 0x28, 0x00])

        state = can_manager.parse_servo_message(data)

        assert state.position == 100.0  # 1000 * 0.1
        assert state.velocity == 50.0  # 5 * 10
        assert state.current == 5.0  # 500 * 0.01
        assert state.temperature == 40.0
        assert state.error == 0

    @pytest.mark.parametrize("frame_len", [7, 9])
    def test_parse_servo_message_invalid_length_raises(
        self, mock_can: Dict[str, Any], frame_len: int
    ) -> None:
        """Servo status frames must be exactly 8 bytes."""
        can_manager = CAN_Manager_servo(channel="vcan0")
        data = bytes([0x00] * frame_len)

        with pytest.raises(ValueError, match="exactly 8 bytes"):
            can_manager.parse_servo_message(data)


class TestCanErrorHandling:
    """Tests for CAN send error behavior."""

    def test_send_servo_message_raises_runtime_error_on_can_error(
        self, mock_can: Dict[str, Any]
    ) -> None:
        """CAN send failures should raise RuntimeError and not fail silently."""
        can_manager = CAN_Manager_servo(channel="vcan0")

        class DummyCanError(Exception):
            pass

        mock_can["can"].CanError = DummyCanError
        mock_can["bus"].send.side_effect = DummyCanError("send failed")

        with pytest.raises(RuntimeError, match="Failed to send CAN message"):
            can_manager.send_servo_message(motor_id=1, data=[0x01], data_len=0)


class TestSingletonBehavior:
    """Tests for singleton channel consistency."""

    def test_reinitializing_on_different_channel_raises(
        self, mock_can: Dict[str, Any]
    ) -> None:
        """A singleton initialized on one channel must not silently switch channels."""
        CAN_Manager_servo(channel="vcan0")
        with pytest.raises(RuntimeError, match="already initialized"):
            CAN_Manager_servo(channel="can0")
