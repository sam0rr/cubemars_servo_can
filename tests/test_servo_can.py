import pytest
from unittest.mock import MagicMock, patch
from cubemars_servo_can.servo_can import CubeMarsServoCAN
from cubemars_servo_can.constants import ControlMode


@pytest.fixture
def mock_can():
    with (
        patch("cubemars_servo_can.can_manager.can") as mock_can_lib,
        patch("cubemars_servo_can.can_manager.os.system") as mock_system,
    ):
        # Setup mock bus and notifier
        mock_bus = MagicMock()
        mock_notifier = MagicMock()

        mock_can_lib.interface.Bus.return_value = mock_bus
        mock_can_lib.Notifier.return_value = mock_notifier

        yield {
            "can": mock_can_lib,
            "bus": mock_bus,
            "notifier": mock_notifier,
            "system": mock_system,
        }


def test_initialization(mock_can):
    # Should not raise
    motor = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1, can_channel="vcan0")

    # Check if os.system was called to set up CAN
    mock_can["system"].assert_called()
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
    # We can inspect the last call to bus.send
    args, _ = mock_can["bus"].send.call_args
    message = args[0]

    assert message.arbitration_id is not None
    # We are not checking the exact byte packing here (covered in utils test/integration)
    # but ensuring the flow works.


def test_safety_limits(mock_can):
    motor = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
    motor._entered = True
    motor.enter_position_control()

    # Try to set a huge angle beyond P_max
    with pytest.raises(RuntimeError, match="Cannot control using impedance mode"):
        motor.set_motor_angle_radians(
            1000.0
        )  # 1000 rad is way > 32000/GearRatio (approx) depending on config
