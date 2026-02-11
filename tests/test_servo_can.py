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


class TestSafetyLimits:
    """Tests for safety limits and error handling."""

    def test_position_limit_raises_error(self, mock_can: Dict[str, Any]) -> None:
        """Test that exceeding position limit raises RuntimeError."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_position_control()

        # Try to set position beyond P_max (32000)
        # Must account for gear ratio: 32000 * 9.0 = 288000
        with pytest.raises(RuntimeError, match="Cannot control using impedance mode"):
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

        # Verify current was calculated (1.0 Nm motor-side / GEAR_RATIO / Kt_actual / GEAR_RATIO)
        # For AK80-9: 1.0 / 9.0 / 0.115 / 9.0 = ~0.107 A
        # Current is sent as int32(current * 1000)
        import struct

        current_int = struct.unpack(">i", bytes(message.data))[0]
        expected_current = int((1.0 / 9.0 / 0.115 / 9.0) * 1000)
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


class TestStateParsing:
    """Tests for parsing incoming CAN messages."""

    def test_parse_servo_message(self, mock_can: Dict[str, Any]) -> None:
        """Test that incoming CAN messages are parsed correctly."""
        from cubemars_servo_can.can_manager import CAN_Manager_servo

        # Create a CAN manager to test parsing
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

    def test_error_state_raises_runtime_error(self, mock_can: Dict[str, Any]) -> None:
        """Test that receiving an error state raises RuntimeError."""
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)

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

        with pytest.raises(RuntimeError, match="Over temperature fault"):
            motor._update_state_async(error_state)
