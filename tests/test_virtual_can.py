"""Integration test using python-can's in-process virtual interface."""

import struct
import threading
from dataclasses import dataclass, field

import can
import pytest

from cubemars_servo_can._can_manager import CanManagerRegistry
from cubemars_servo_can._motor_state import ServoTelemetry


@dataclass(kw_only=True, slots=True)
class VirtualSink:
    """Capture status delivered by a real python-can notifier thread."""

    event: threading.Event = field(default_factory=threading.Event)
    telemetry: ServoTelemetry | None = None

    def _accept_telemetry(self, telemetry: ServoTelemetry) -> None:
        """Capture valid status."""
        self.telemetry = telemetry
        self.event.set()

    def _accept_listener_error(self, error: Exception) -> None:
        """Fail the integration wait if decoding unexpectedly fails."""
        raise AssertionError("Virtual CAN status failed to decode") from error


def test_status_round_trip_over_virtual_can(monkeypatch: pytest.MonkeyPatch) -> None:
    """Route a real virtual-bus status frame through Notifier and Listener."""
    virtual_channel = "cubemars-servo-can-integration"

    def open_virtual_bus(*, channel: str, interface: str) -> can.BusABC:
        """Map the production SocketCAN request onto a virtual test bus."""
        if channel != "vcan-test" or interface != "socketcan":
            raise ValueError("unexpected production bus request")
        return can.Bus(
            interface="virtual",
            channel=virtual_channel,
            receive_own_messages=False,
        )

    monkeypatch.setattr(can.interface, "Bus", open_virtual_bus)
    peer = can.Bus(
        interface="virtual",
        channel=virtual_channel,
        receive_own_messages=False,
    )
    sink = VirtualSink()
    manager = CanManagerRegistry.acquire_registered(
        channel="vcan-test",
        motor_id=3,
        sink=sink,
    )
    try:
        peer.send(
            can.Message(
                arbitration_id=0x2903,
                data=struct.pack(">hhhbB", 10, 20, 30, 40, 0),
                is_extended_id=True,
            )
        )
        assert sink.event.wait(1.0)
        assert sink.telemetry is not None
        assert sink.telemetry.motor_position_degrees == 1.0
        assert sink.telemetry.velocity_erpm == 200.0
    finally:
        manager.unregister(motor_id=3)
        CanManagerRegistry.release(manager=manager)
        peer.shutdown()
