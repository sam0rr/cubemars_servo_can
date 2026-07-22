"""Tests for channel-scoped CAN transport ownership and routing."""

from dataclasses import dataclass, field

import can
import pytest
from conftest import CanHarness

from cubemars_servo_can import CanConnectionError
from cubemars_servo_can._models import Telemetry
from cubemars_servo_can._transport import (
    _ChannelTransport,
    _MotorListener,
    _TransportRegistry,
)


@dataclass(kw_only=True, slots=True)
class TelemetrySink:
    """Capture listener callbacks."""

    telemetry: list[Telemetry] = field(default_factory=list)
    errors: list[Exception] = field(default_factory=list)

    def _accept_telemetry(self, telemetry: Telemetry) -> None:
        """Capture decoded telemetry."""
        self.telemetry.append(telemetry)

    def _accept_listener_error(self, error: Exception) -> None:
        """Capture decoding failures."""
        self.errors.append(error)


def test_registry_shares_channels_and_releases_last_motor(
    can_harness: CanHarness,
) -> None:
    """Share one bus per channel and close it only after its final unregister."""
    first = _TransportRegistry.acquire(channel="vcan0")
    second = _TransportRegistry.acquire(channel="vcan0")
    sink = TelemetrySink()
    assert first is second
    first.register(motor_id=1, sink=sink)
    first.register(motor_id=2, sink=sink)
    _TransportRegistry.release(transport=first)
    assert can_harness.bus is not None
    assert can_harness.bus.shutdown_count == 0
    first.unregister(motor_id=1)
    first.unregister(motor_id=2)
    first.unregister(motor_id=99)
    _TransportRegistry.release(transport=first)
    assert can_harness.bus.shutdown_count == 1
    assert can_harness.notifier is not None
    assert can_harness.notifier.stop_count == 1
    first.close()
    assert can_harness.bus.shutdown_count == 1


def test_registry_supports_multiple_channels(can_harness: CanHarness) -> None:
    """Open independent transports for independent SocketCAN channels."""
    first = _TransportRegistry.acquire(channel="vcan0")
    first_bus = can_harness.bus
    second = _TransportRegistry.acquire(channel="vcan1")
    assert first is not second
    assert first_bus is not can_harness.bus


def test_transport_rejects_duplicate_and_closed_registrations(
    can_harness: CanHarness,
) -> None:
    """Prevent ambiguous motor routing and use after close."""
    transport = _ChannelTransport(channel="vcan0")
    sink = TelemetrySink()
    transport.register(motor_id=1, sink=sink)
    with pytest.raises(CanConnectionError, match="already registered"):
        transport.register(motor_id=1, sink=sink)
    transport.close()
    with pytest.raises(CanConnectionError, match="closed"):
        transport.register(motor_id=2, sink=sink)
    assert can_harness.bus is not None


def test_transport_wraps_open_notifier_and_send_failures(
    can_harness: CanHarness,
) -> None:
    """Translate infrastructure failures to package-specific exceptions."""
    can_harness.fail_open = True
    with pytest.raises(CanConnectionError, match="open SocketCAN"):
        _ChannelTransport(channel="missing")

    can_harness.fail_open = False
    can_harness.fail_notifier = True
    with pytest.raises(CanConnectionError, match="start the CAN notifier"):
        _ChannelTransport(channel="vcan0")
    assert can_harness.bus is not None
    assert can_harness.bus.shutdown_count == 1

    can_harness.fail_notifier = False
    transport = _ChannelTransport(channel="vcan0")
    can_harness.fail_send = True
    with pytest.raises(CanConnectionError, match="Failed to send"):
        transport.send(arbitration_id=0x101, data=b"\x00")
    can_harness.fail_send = False
    with pytest.raises(ValueError, match="8 bytes"):
        transport.send(arbitration_id=0x101, data=b"123456789")
    transport.close()
    with pytest.raises(CanConnectionError, match="closed"):
        transport.send(arbitration_id=0x101, data=b"")


def test_transport_cleanup_is_best_effort(
    can_harness: CanHarness, caplog: pytest.LogCaptureFixture
) -> None:
    """Finish cleanup when listener, notifier, and bus cleanup operations fail."""
    transport = _ChannelTransport(channel="vcan0")
    transport.register(motor_id=1, sink=TelemetrySink())
    can_harness.fail_listener_removal = True
    transport.unregister(motor_id=1)
    can_harness.fail_notifier_stop = True
    can_harness.fail_shutdown = True
    transport.close()
    assert transport.motor_count == 0
    assert "Failed to remove motor" in caplog.text
    assert "Failed to stop the CAN notifier" in caplog.text
    assert "Failed to close the CAN bus" in caplog.text


def test_listener_routes_only_exact_extended_status(can_harness: CanHarness) -> None:
    """Ignore unrelated frames and report malformed matching status frames."""
    sink = TelemetrySink()
    listener = _MotorListener(motor_id=1, sink=sink)
    listener.on_message_received(
        can.Message(arbitration_id=1, data=b"12345678", is_extended_id=True)
    )
    listener.on_message_received(
        can.Message(arbitration_id=0x2901, data=b"12345678", is_extended_id=False)
    )
    assert not sink.telemetry
    assert not sink.errors

    listener.on_message_received(
        can.Message(arbitration_id=0x2901, data=b"bad", is_extended_id=True)
    )
    assert len(sink.errors) == 1

    listener.on_message_received(
        can.Message(
            arbitration_id=0x2901,
            data=can_harness.status_payload,
            is_extended_id=True,
        )
    )
    assert len(sink.telemetry) == 1
    assert sink.telemetry[0].temperature_celsius == 25.0
