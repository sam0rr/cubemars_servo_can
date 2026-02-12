"""Tests for CAN manager functionality."""

import struct
import pytest
from typing import Dict, Any
from unittest.mock import MagicMock, patch
from cubemars_servo_can.can_manager import CAN_Manager_servo, MotorListener
from cubemars_servo_can.motor_state import ServoMotorState


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

    def test_parse_servo_message_signed_fields(self, mock_can: Dict[str, Any]) -> None:
        """Signed int16 fields should decode without overflow on all supported Python versions."""
        can_manager = CAN_Manager_servo(channel="vcan0")

        # pos=-0.2 deg_elec (0xFFFE * 0.1), vel=-10.0 ERPM*10 (0xFFFF * 10), cur=-0.03A (0xFFFD * 0.01)
        data = bytes([0xFF, 0xFE, 0xFF, 0xFF, 0xFF, 0xFD, 0x19, 0x00])
        state = can_manager.parse_servo_message(data)

        assert state.position == -0.2
        assert state.velocity == -10.0
        assert state.current == -0.03
        assert state.temperature == 25.0
        assert state.error == 0

    @pytest.mark.parametrize(
        "raw_hi,raw_lo,expected",
        [
            (0x7F, 0xFF, 32767),
            (0x80, 0x00, -32768),
            (0xFF, 0xFF, -1),
        ],
    )
    def test_parse_servo_message_int16_edge_values(
        self,
        mock_can: Dict[str, Any],
        raw_hi: int,
        raw_lo: int,
        expected: int,
    ) -> None:
        """Signed 16-bit edge values should decode correctly and never overflow."""
        can_manager = CAN_Manager_servo(channel="vcan0")
        data = bytes([raw_hi, raw_lo, raw_hi, raw_lo, raw_hi, raw_lo, 0x00, 0x00])

        state = can_manager.parse_servo_message(data)

        assert state.position == expected * 0.1
        assert state.velocity == expected * 10.0
        assert state.current == expected * 0.01

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

    def test_interface_open_failure_raises_runtime_error(
        self, mock_can: Dict[str, Any]
    ) -> None:
        mock_can["can"].interface.Bus.side_effect = OSError("No such device")

        with pytest.raises(RuntimeError, match="Failed to open CAN interface 'can9'"):
            CAN_Manager_servo(channel="can9")

        assert CAN_Manager_servo._instance is None

    def test_notifier_init_failure_resets_singleton(
        self, mock_can: Dict[str, Any]
    ) -> None:
        mock_can["can"].Notifier.side_effect = RuntimeError("listener setup failed")

        with pytest.raises(RuntimeError, match="Failed to start CAN listener"):
            CAN_Manager_servo(channel="can0")

        assert CAN_Manager_servo._instance is None
        mock_can["bus"].shutdown.assert_called_once()

    def test_notifier_init_failure_swallows_shutdown_error(
        self, mock_can: Dict[str, Any]
    ) -> None:
        mock_can["can"].Notifier.side_effect = RuntimeError("listener setup failed")
        mock_can["bus"].shutdown.side_effect = RuntimeError("shutdown failed")

        with pytest.raises(RuntimeError, match="Failed to start CAN listener"):
            CAN_Manager_servo(channel="can0")

        assert CAN_Manager_servo._instance is None
        mock_can["bus"].shutdown.assert_called_once()


class TestMotorListener:
    """Tests for listener message dispatch."""

    def test_listener_updates_matching_motor(self, mock_can: Dict[str, Any]) -> None:
        can_manager = CAN_Manager_servo(channel="vcan0")
        motor = MagicMock()
        motor.ID = 5
        state = ServoMotorState(0.0, 0.0, 0.0, 25.0, 0, 0.0)
        can_manager.parse_servo_message = MagicMock(return_value=state)  # type: ignore[method-assign]

        listener = MotorListener(can_manager, motor)
        msg = MagicMock()
        msg.arbitration_id = 0x905  # lower byte = 0x05, non-command packet id
        msg.data = bytes([0] * 8)

        listener.on_message_received(msg)
        motor._update_state_async.assert_called_once_with(state)

    def test_listener_ignores_non_matching_motor(
        self, mock_can: Dict[str, Any]
    ) -> None:
        can_manager = CAN_Manager_servo(channel="vcan0")
        motor = MagicMock()
        motor.ID = 1
        listener = MotorListener(can_manager, motor)
        msg = MagicMock()
        msg.arbitration_id = 0x302  # lower byte = 0x02
        msg.data = bytes([0] * 8)

        listener.on_message_received(msg)
        motor._update_state_async.assert_not_called()

    def test_listener_ignores_command_like_low_byte_id(
        self, mock_can: Dict[str, Any]
    ) -> None:
        can_manager = CAN_Manager_servo(channel="vcan0")
        can_manager.parse_servo_message = MagicMock()  # type: ignore[method-assign]
        motor = MagicMock()
        motor.ID = 5
        listener = MotorListener(can_manager, motor)
        msg = MagicMock()
        msg.arbitration_id = 0x305  # lower byte match, packet id 0x03 (SET_RPM command)
        msg.data = bytes([0] * 8)

        listener.on_message_received(msg)
        motor._update_state_async.assert_not_called()
        can_manager.parse_servo_message.assert_not_called()  # type: ignore[attr-defined]

    def test_listener_ignores_non_8_byte_frame(self, mock_can: Dict[str, Any]) -> None:
        can_manager = CAN_Manager_servo(channel="vcan0")
        can_manager.parse_servo_message = MagicMock()  # type: ignore[method-assign]
        motor = MagicMock()
        motor.ID = 5
        listener = MotorListener(can_manager, motor)
        msg = MagicMock()
        msg.arbitration_id = 0x005
        msg.data = bytes([0] * 7)

        listener.on_message_received(msg)
        motor._update_state_async.assert_not_called()
        can_manager.parse_servo_message.assert_not_called()  # type: ignore[attr-defined]

    def test_listener_ignores_power_on_echo_on_exact_id(
        self, mock_can: Dict[str, Any]
    ) -> None:
        can_manager = CAN_Manager_servo(channel="vcan0")
        can_manager.parse_servo_message = MagicMock()  # type: ignore[method-assign]
        motor = MagicMock()
        motor.ID = 1
        listener = MotorListener(can_manager, motor)
        msg = MagicMock()
        msg.arbitration_id = 0x001
        msg.data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])

        listener.on_message_received(msg)
        motor._update_state_async.assert_not_called()
        can_manager.parse_servo_message.assert_not_called()  # type: ignore[attr-defined]

    def test_listener_surfaces_parse_errors_to_motor(
        self, mock_can: Dict[str, Any]
    ) -> None:
        can_manager = CAN_Manager_servo(channel="vcan0")
        can_manager.parse_servo_message = MagicMock(side_effect=ValueError("bad frame"))  # type: ignore[method-assign]

        motor = MagicMock()
        motor.ID = 1
        listener = MotorListener(can_manager, motor)

        msg = MagicMock()
        msg.arbitration_id = 0x001
        msg.data = bytes([0] * 8)

        listener.on_message_received(msg)
        motor._set_listener_error.assert_called_once()


class TestStatusIdClassification:
    def test_exact_motor_id_is_accepted(self) -> None:
        assert CAN_Manager_servo.is_status_arbitration_id(0x001, 0x001) is True

    def test_compatible_non_command_low_byte_id_is_accepted(self) -> None:
        assert CAN_Manager_servo.is_status_arbitration_id(0x901, 0x001) is True

    def test_command_like_low_byte_id_is_rejected(self) -> None:
        assert CAN_Manager_servo.is_status_arbitration_id(0x601, 0x001) is False

    def test_wrong_low_byte_id_is_rejected(self) -> None:
        assert CAN_Manager_servo.is_status_arbitration_id(0x902, 0x001) is False


class TestDebugBranches:
    """Tests for debug print branches."""

    def test_send_servo_message_debug_success_prints(
        self, mock_can: Dict[str, Any], capsys
    ) -> None:
        can_manager = CAN_Manager_servo(channel="vcan0")
        can_manager.debug = True
        can_manager.send_servo_message(motor_id=1, data=[0x12], data_len=0)
        out = capsys.readouterr().out
        assert "ID:" in out
        assert "Message sent on" in out
        can_manager.debug = False

    def test_send_servo_message_debug_error_prints(
        self, mock_can: Dict[str, Any], capsys
    ) -> None:
        can_manager = CAN_Manager_servo(channel="vcan0")
        can_manager.debug = True

        class DummyCanError(Exception):
            pass

        mock_can["can"].CanError = DummyCanError
        mock_can["bus"].send.side_effect = DummyCanError("send failed")

        with pytest.raises(RuntimeError, match="Failed to send CAN message"):
            can_manager.send_servo_message(motor_id=1, data=[0x12], data_len=0)

        out = capsys.readouterr().out
        assert "Message NOT sent" in out
        can_manager.debug = False

    def test_parse_servo_message_debug_prints(
        self, mock_can: Dict[str, Any], capsys
    ) -> None:
        can_manager = CAN_Manager_servo(channel="vcan0")
        can_manager.debug = True
        can_manager.parse_servo_message(bytes([0x00, 0x01, 0, 0, 0, 0, 0x2A, 0x00]))
        out = capsys.readouterr().out
        assert "Position (deg_elec):" in out
        assert "Temp:" in out
        can_manager.debug = False


class TestOriginCommand:
    """Tests for origin command wrapper."""

    def test_set_origin_command_format(self, mock_can: Dict[str, Any]) -> None:
        can_manager = CAN_Manager_servo(channel="vcan0")
        can_manager.comm_can_set_origin(controller_id=1, set_origin_mode=2)
        message = mock_can["bus"].send.call_args[0][0]
        assert message.arbitration_id == 0x501
        assert message.data == [2]


class TestCommandPacking:
    def test_set_pos_and_set_pos_spd_share_position_scaling(
        self, mock_can: Dict[str, Any]
    ) -> None:
        can_manager = CAN_Manager_servo(channel="vcan0")
        position = 123.456

        can_manager.comm_can_set_pos(controller_id=1, pos=position)
        pos_msg = mock_can["bus"].send.call_args_list[-1][0][0]

        can_manager.comm_can_set_pos_spd(
            controller_id=1, pos=position, spd=0.0, RPA=0.0
        )
        pos_spd_msg = mock_can["bus"].send.call_args_list[-1][0][0]

        pos_raw = struct.unpack(">i", bytes(pos_msg.data))[0]
        pos_spd_raw = struct.unpack(">i", bytes(pos_spd_msg.data[0:4]))[0]
        expected = int(position * 10000.0)

        assert pos_msg.arbitration_id == 0x401
        assert pos_spd_msg.arbitration_id == 0x601
        assert pos_raw == expected
        assert pos_spd_raw == expected

    def test_current_brake_rejects_negative_current(
        self, mock_can: Dict[str, Any]
    ) -> None:
        can_manager = CAN_Manager_servo(channel="vcan0")

        with pytest.raises(ValueError, match="non-negative"):
            can_manager.comm_can_set_cb(controller_id=1, current=-0.5)

    def test_current_brake_rejects_over_60_amps(self, mock_can: Dict[str, Any]) -> None:
        can_manager = CAN_Manager_servo(channel="vcan0")

        with pytest.raises(ValueError, match="<= 60A"):
            can_manager.comm_can_set_cb(controller_id=1, current=60.1)


class TestListenerRegistrationLifecycle:
    """Tests for listener add/remove and close behavior."""

    def test_remove_motor_unregisters_listener(self, mock_can: Dict[str, Any]) -> None:
        can_manager = CAN_Manager_servo(channel="vcan0")
        motor = MagicMock()
        listener = can_manager.add_motor(motor)

        can_manager.remove_motor(motor)
        mock_can["notifier"].remove_listener.assert_called_once_with(listener)

    def test_close_stops_notifier_and_bus(self, mock_can: Dict[str, Any]) -> None:
        can_manager = CAN_Manager_servo(channel="vcan0")
        can_manager.close()
        mock_can["notifier"].stop.assert_called_once()
        mock_can["bus"].shutdown.assert_called_once()
        assert can_manager._closed is True
        assert CAN_Manager_servo._instance is None

    def test_close_swallows_notifier_stop_error_and_continues(
        self, mock_can: Dict[str, Any]
    ) -> None:
        can_manager = CAN_Manager_servo(channel="vcan0")
        can_manager.add_motor(MagicMock())
        mock_can["notifier"].stop.side_effect = RuntimeError("stop failed")

        can_manager.close()

        mock_can["notifier"].stop.assert_called_once()
        mock_can["bus"].shutdown.assert_called_once()
        assert can_manager._listeners == {}
        assert can_manager._closed is True
        assert CAN_Manager_servo._instance is None

    def test_close_swallows_bus_shutdown_error_and_continues(
        self, mock_can: Dict[str, Any]
    ) -> None:
        can_manager = CAN_Manager_servo(channel="vcan0")
        can_manager.add_motor(MagicMock())
        mock_can["bus"].shutdown.side_effect = RuntimeError("shutdown failed")

        can_manager.close()

        mock_can["notifier"].stop.assert_called_once()
        mock_can["bus"].shutdown.assert_called_once()
        assert can_manager._listeners == {}
        assert can_manager._closed is True
        assert CAN_Manager_servo._instance is None

    def test_del_swallows_close_errors(self, mock_can: Dict[str, Any]) -> None:
        can_manager = CAN_Manager_servo(channel="vcan0")
        with patch.object(can_manager, "close", side_effect=RuntimeError("close boom")):
            # Should not raise
            can_manager.__del__()


class TestRemovedRuntimeConfiguration:
    """Tests for removed runtime socketcan configuration API."""

    def test_configure_socketcan_api_removed(self) -> None:
        assert not hasattr(CAN_Manager_servo, "configure_socketcan")
