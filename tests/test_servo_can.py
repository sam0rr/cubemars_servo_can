import pytest
from unittest.mock import MagicMock, patch
from cubemars_servo_can.servo_can import CubeMarsServoCAN
from cubemars_servo_can.can_manager import CAN_Manager_servo
from cubemars_servo_can.constants import ControlMode


@pytest.fixture
def mock_can():
    """Fixture that mocks CAN bus interface."""
    with patch("cubemars_servo_can.can_manager.can") as mock_can_lib:
        # Setup mock bus and notifier
        mock_bus = MagicMock()
        mock_notifier = MagicMock()

        mock_can_lib.interface.Bus.return_value = mock_bus
        mock_can_lib.Notifier.return_value = mock_notifier

        yield {
            "can": mock_can_lib,
            "bus": mock_bus,
            "notifier": mock_notifier,
        }


def test_initialization(mock_can):
    # Should not raise
    motor = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1, can_channel="vcan0")

    # Check if bus was created
    mock_can["can"].interface.Bus.assert_called_with(
        channel="vcan0", bustype="socketcan"
    )


def test_enter_exit_context(mock_can):
    motor = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)

    # Mock check_can_connection to return True immediately to avoid delays/errors
    with patch.object(motor, "check_can_connection", return_value=True):
        with motor:
            assert motor._entered is True
            # Should have sent power on
            mock_can["bus"].send.assert_called()

    assert (
        motor._entered is True
    )  # _entered remains true, but connection is closed logic
    # In exit, power off is called
    assert mock_can["bus"].send.call_count >= 2


def test_send_position_command(mock_can):
    motor = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)

    # Mock internals to simulate entered state
    motor._entered = True
    motor.enter_position_control()

    # Set angle 3.14 rad
    motor.set_motor_angle_radians(3.14)

    motor.update()

    # Verify a message was sent
    args, _ = mock_can["bus"].send.call_args
    message = args[0]

    assert message.arbitration_id is not None


def test_safety_limits(mock_can):
    motor = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
    motor._entered = True
    motor.enter_position_control()

    # Try to set a huge angle beyond P_max (32000)
    # Note: set_motor_angle_radians divides by GEAR_RATIO (9.0) before checking limits
    # So we need to pass an angle > 32000 * 9.0 = 288000 to trigger the limit
    with pytest.raises(RuntimeError, match="Cannot control using impedance mode"):
        motor.set_motor_angle_radians(300000.0)
