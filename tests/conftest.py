import pytest
import gc
from typing import Generator


@pytest.fixture(autouse=True)
def reset_can_manager_singleton() -> Generator[None, None, None]:
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
def cleanup_session() -> Generator[None, None, None]:
    """Ensure cleanup at end of test session."""
    yield
    from cubemars_servo_can.can_manager import CAN_Manager_servo

    if CAN_Manager_servo._instance is not None:
        CAN_Manager_servo._instance.close()
    gc.collect()
