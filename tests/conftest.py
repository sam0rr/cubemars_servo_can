import gc
from collections.abc import Generator
from typing import Any
from unittest.mock import MagicMock, patch

import pytest


@pytest.fixture
def mock_can() -> Generator[dict[str, Any]]:
    """Fixture that mocks python-can bus and notifier primitives."""
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


@pytest.fixture(autouse=True)
def reset_can_manager_singleton() -> Generator[None]:
    """Reset the CAN Manager singleton before and after each test."""
    from cubemars_servo_can.can_manager import CAN_Manager_servo

    # Reset before test
    CAN_Manager_servo._instance = None

    yield

    # Deterministic cleanup after each test
    if CAN_Manager_servo._instance is not None:
        CAN_Manager_servo._instance.close()
    gc.collect()


@pytest.fixture(scope="session", autouse=True)
def cleanup_session() -> Generator[None]:
    """Ensure cleanup at end of test session."""
    yield
    from cubemars_servo_can.can_manager import CAN_Manager_servo

    if CAN_Manager_servo._instance is not None:
        CAN_Manager_servo._instance.close()
    gc.collect()
