import pytest
from unittest.mock import patch


@pytest.fixture(autouse=True)
def reset_can_manager_singleton():
    """Reset the CAN Manager singleton before and after each test."""
    from cubemars_servo_can.can_manager import CAN_Manager_servo

    # Reset before test
    CAN_Manager_servo._instance = None

    # Mock os.system during the test to prevent sudo calls
    with patch("cubemars_servo_can.can_manager.os.system") as mock_system:
        yield

    # Clean up: delete singleton instance after test (with os.system still mocked in fixture scope)
    if CAN_Manager_servo._instance is not None:
        instance = CAN_Manager_servo._instance
        CAN_Manager_servo._instance = None
        del instance
