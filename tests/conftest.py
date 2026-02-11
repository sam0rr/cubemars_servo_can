import pytest
import gc
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
        # After yield, we're still in the mock context
        # Clean up: delete singleton instance after test (with os.system still mocked)
        if CAN_Manager_servo._instance is not None:
            instance = CAN_Manager_servo._instance
            CAN_Manager_servo._instance = None
            del instance
            # Force garbage collection while mock is still active
            gc.collect()


@pytest.fixture(scope="session", autouse=True)
def cleanup_session():
    """Ensure cleanup at end of test session."""
    yield
    # After all tests complete, ensure any remaining singleton is cleaned
    # with os.system mocked to prevent sudo prompts
    with patch("cubemars_servo_can.can_manager.os.system"):
        from cubemars_servo_can.can_manager import CAN_Manager_servo

        if CAN_Manager_servo._instance is not None:
            CAN_Manager_servo._instance = None
        gc.collect()
