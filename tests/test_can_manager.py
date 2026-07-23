"""Tests for channel-scoped CAN ownership and exact status routing."""

import threading
from dataclasses import dataclass, field

import can
import pytest
from conftest import CanHarness

from cubemars_servo_can import CanConnectionError
from cubemars_servo_can._can_manager import (
    CanManager,
    CanManagerRegistry,
    MotorListener,
)
from cubemars_servo_can._motor_state import ServoTelemetry
from cubemars_servo_can._protocol import CanFrame


@dataclass(kw_only=True, slots=True)
class TelemetrySink:
    """Capture listener callbacks."""

    telemetry: list[ServoTelemetry] = field(default_factory=list)
    errors: list[Exception] = field(default_factory=list)

    def _accept_telemetry(self, telemetry: ServoTelemetry) -> None:
        """Capture decoded telemetry."""
        self.telemetry.append(telemetry)

    def _accept_listener_error(self, error: Exception) -> None:
        """Capture decoding failures."""
        self.errors.append(error)


def test_registry_shares_channel_and_closes_after_last_motor(
    can_harness: CanHarness,
) -> None:
    """Share one manager per channel and release it by registration count."""
    sink = TelemetrySink()
    first = CanManagerRegistry.acquire_registered(
        channel="vcan0",
        motor_id=1,
        sink=sink,
    )
    second = CanManagerRegistry.acquire_registered(
        channel="vcan0",
        motor_id=2,
        sink=sink,
    )
    assert first is second
    assert first.motor_count == 2
    first.unregister(motor_id=1)
    first.unregister(motor_id=99)
    CanManagerRegistry.release(manager=first)
    assert can_harness.bus is not None
    assert can_harness.bus.shutdown_count == 0
    first.unregister(motor_id=2)
    CanManagerRegistry.release(manager=first)
    assert can_harness.bus.shutdown_count == 1
    assert can_harness.notifier is not None
    assert can_harness.notifier.stop_count == 1
    first.close()
    assert can_harness.bus.shutdown_count == 1


def test_registry_opens_independent_channels(can_harness: CanHarness) -> None:
    """Use separate managers and buses for separate SocketCAN channels."""
    first = CanManagerRegistry.acquire_registered(
        channel="vcan0",
        motor_id=1,
        sink=TelemetrySink(),
    )
    first_bus = can_harness.bus
    second = CanManagerRegistry.acquire_registered(
        channel="vcan1",
        motor_id=1,
        sink=TelemetrySink(),
    )
    assert first is not second
    assert first_bus is not can_harness.bus


def test_manager_rejects_duplicate_closed_registration_and_send(
    can_harness: CanHarness,
) -> None:
    """Prevent ambiguous routing and operations on a closed manager."""
    manager = CanManager(channel="vcan0")
    sink = TelemetrySink()
    manager.register(motor_id=1, sink=sink)
    with pytest.raises(CanConnectionError, match="already registered"):
        manager.register(motor_id=1, sink=sink)
    manager.close()
    with pytest.raises(CanConnectionError, match="closed"):
        manager.register(motor_id=2, sink=sink)
    with pytest.raises(CanConnectionError, match="closed"):
        manager.send(CanFrame(arbitration_id=0x101, data=b""))
    assert can_harness.bus is not None


def test_manager_wraps_open_notifier_registration_and_send_failures(
    can_harness: CanHarness,
) -> None:
    """Translate python-can infrastructure failures to typed errors."""
    can_harness.fail_open = True
    with pytest.raises(CanConnectionError, match="open SocketCAN"):
        CanManager(channel="missing")

    can_harness.fail_open = False
    can_harness.fail_notifier = True
    can_harness.fail_shutdown = True
    with pytest.raises(CanConnectionError, match="start the CAN notifier"):
        CanManager(channel="vcan0")
    assert can_harness.bus is not None
    assert can_harness.bus.shutdown_count == 1

    can_harness.fail_shutdown = False
    can_harness.fail_notifier = False
    manager = CanManager(channel="vcan0")
    can_harness.fail_listener_addition = True
    with pytest.raises(CanConnectionError, match="Failed to register"):
        manager.register(motor_id=1, sink=TelemetrySink())
    can_harness.fail_listener_addition = False
    can_harness.fail_send = True
    with pytest.raises(CanConnectionError, match="Failed to send"):
        manager.send(CanFrame(arbitration_id=0x101, data=b"\x00"))


def test_failed_first_registration_rolls_back_new_manager(
    can_harness: CanHarness,
) -> None:
    """Close a new channel if its first listener cannot register."""
    can_harness.fail_listener_addition = True
    with pytest.raises(CanConnectionError, match="Failed to register"):
        CanManagerRegistry.acquire_registered(
            channel="vcan0",
            motor_id=1,
            sink=TelemetrySink(),
        )
    assert "vcan0" not in CanManagerRegistry._managers
    assert can_harness.bus is not None
    assert can_harness.bus.shutdown_count == 1


def test_cleanup_is_best_effort_and_failed_removal_deactivates_listener(
    can_harness: CanHarness,
    caplog: pytest.LogCaptureFixture,
) -> None:
    """Finish cleanup and stop dispatch when dependency cleanup fails."""
    manager = CanManager(channel="vcan0")
    sink = TelemetrySink()
    manager.register(motor_id=1, sink=sink)
    assert can_harness.notifier is not None
    listener = can_harness.notifier.listeners[0]
    can_harness.fail_listener_removal = True
    manager.unregister(motor_id=1)
    can_harness.emit(motor_id=1)
    assert not sink.telemetry
    can_harness.fail_notifier_stop = True
    can_harness.fail_shutdown = True
    manager.close()
    assert manager.motor_count == 0
    assert "Failed to remove motor" in caplog.text
    assert "Failed to stop CAN notifier" in caplog.text
    assert "Failed to close CAN bus" in caplog.text
    assert listener is not None


def test_listener_routes_only_explicit_extended_feedback(
    can_harness: CanHarness,
) -> None:
    """Accept known feedback IDs while rejecting standard and command frames."""
    sink = TelemetrySink()
    listener = MotorListener(motor_id=1, sink=sink)
    listener.on_message_received(
        can.Message(arbitration_id=0x0801, data=b"12345678", is_extended_id=True)
    )
    listener.on_message_received(
        can.Message(
            arbitration_id=0x2901,
            data=b"12345678",
            is_extended_id=False,
        )
    )
    assert not sink.telemetry
    assert not sink.errors

    for arbitration_id in (0x0001, 0x0901, 0x2901):
        listener.on_message_received(
            can.Message(
                arbitration_id=arbitration_id,
                data=can_harness.status_payload,
                is_extended_id=True,
            )
        )
    assert len(sink.telemetry) == 3
    assert all(sample.temperature_celsius == 25.0 for sample in sink.telemetry)

    listener.on_message_received(
        can.Message(
            arbitration_id=0x0601,
            data=b"12345678",
            is_extended_id=True,
        )
    )
    assert len(sink.telemetry) == 3
    assert not sink.errors

    listener.on_message_received(
        can.Message(arbitration_id=0x2901, data=b"bad", is_extended_id=True)
    )
    assert len(sink.errors) == 1
    listener.on_message_received(
        can.Message(
            arbitration_id=0x0001,
            data=b"duty",
            is_extended_id=True,
        )
    )
    assert len(sink.telemetry) == 3
    assert len(sink.errors) == 1
    for command in (
        bytes.fromhex("FF FF FF FF FF FF FF FC"),
        bytes.fromhex("FF FF FF FF FF FF FF FD"),
    ):
        listener.on_message_received(
            can.Message(
                arbitration_id=0x0001,
                data=command,
                is_extended_id=True,
            )
        )
    assert len(sink.telemetry) == 3
    assert len(sink.errors) == 1
    listener.deactivate()
    listener.on_message_received(
        can.Message(
            arbitration_id=0x2901,
            data=can_harness.status_payload,
            is_extended_id=True,
        )
    )
    assert len(sink.telemetry) == 3


def test_listener_deactivation_waits_for_in_flight_dispatch(
    can_harness: CanHarness,
) -> None:
    """Return from deactivation only after a cached sink dispatch finishes."""
    callback_started = threading.Event()
    release_callback = threading.Event()
    deactivation_started = threading.Event()
    deactivation_finished = threading.Event()

    class BlockingSink(TelemetrySink):
        def _accept_telemetry(self, telemetry: ServoTelemetry) -> None:
            callback_started.set()
            assert release_callback.wait(timeout=1.0)
            super()._accept_telemetry(telemetry)

    sink = BlockingSink()
    listener = MotorListener(motor_id=1, sink=sink)
    message = can.Message(
        arbitration_id=0x2901,
        data=can_harness.status_payload,
        is_extended_id=True,
    )
    dispatch_thread = threading.Thread(
        target=listener.on_message_received,
        args=(message,),
    )

    def deactivate() -> None:
        deactivation_started.set()
        listener.deactivate()
        deactivation_finished.set()

    dispatch_thread.start()
    assert callback_started.wait(timeout=1.0)
    deactivation_thread = threading.Thread(target=deactivate)
    deactivation_thread.start()
    assert deactivation_started.wait(timeout=1.0)
    assert not deactivation_finished.wait(timeout=0.01)
    release_callback.set()
    dispatch_thread.join(timeout=1.0)
    deactivation_thread.join(timeout=1.0)
    assert not dispatch_thread.is_alive()
    assert not deactivation_thread.is_alive()
    assert deactivation_finished.is_set()
    assert len(sink.telemetry) == 1
    listener.on_message_received(message)
    assert len(sink.telemetry) == 1
    assert not sink.errors
