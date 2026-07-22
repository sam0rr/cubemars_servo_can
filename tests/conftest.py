"""Typed CAN test doubles shared by the test suite."""

import struct
from collections.abc import Generator
from dataclasses import dataclass, field

import can
import pytest

from cubemars_servo_can._transport import _TransportRegistry


@dataclass(kw_only=True, slots=True)
class CanHarness:
    """Capture outbound frames and optionally generate valid status replies."""

    respond_to_current: bool = True
    fail_open: bool = False
    fail_notifier: bool = False
    fail_send: bool = False
    fail_shutdown: bool = False
    fail_listener_removal: bool = False
    fail_notifier_stop: bool = False
    status_payload: bytes = struct.pack(">hhhbB", 100, 20, 300, 25, 0)
    sent: list[can.Message] = field(default_factory=list)
    bus: "FakeBus | None" = None
    notifier: "FakeNotifier | None" = None

    def emit(self, *, motor_id: int = 1, data: bytes | None = None) -> None:
        """Deliver one exact extended Servo status frame to all listeners."""
        if self.notifier is None:
            raise RuntimeError("Notifier has not been created")
        message = can.Message(
            arbitration_id=(0x29 << 8) | motor_id,
            data=self.status_payload if data is None else data,
            is_extended_id=True,
        )
        self.notifier.emit(message)


class FakeBus:
    """Small typed replacement for a python-can bus."""

    def __init__(self, *, harness: CanHarness) -> None:
        """Bind captured operations to a harness."""
        self.harness = harness
        self.shutdown_count = 0

    def send(self, message: can.Message) -> None:
        """Capture a frame and synthesize a reply to current probes."""
        if self.harness.fail_send:
            raise can.CanError("synthetic send failure")
        self.harness.sent.append(message)
        if self.harness.respond_to_current and message.arbitration_id >> 8 == 0x01:
            self.harness.emit(motor_id=message.arbitration_id & 0xFF)

    def shutdown(self) -> None:
        """Record deterministic shutdown."""
        self.shutdown_count += 1
        if self.harness.fail_shutdown:
            raise RuntimeError("synthetic shutdown failure")


class FakeNotifier:
    """Small typed replacement for a python-can notifier."""

    def __init__(self, *, harness: CanHarness, listeners: list[can.Listener]) -> None:
        """Store listener registrations."""
        self.harness = harness
        self.listeners = listeners
        self.stop_count = 0

    def add_listener(self, listener: can.Listener) -> None:
        """Register a listener."""
        self.listeners.append(listener)

    def remove_listener(self, listener: can.Listener) -> None:
        """Remove a listener."""
        if self.harness.fail_listener_removal:
            raise RuntimeError("synthetic listener-removal failure")
        self.listeners.remove(listener)

    def emit(self, message: can.Message) -> None:
        """Deliver a message synchronously."""
        for listener in tuple(self.listeners):
            listener.on_message_received(message)

    def stop(self) -> None:
        """Record deterministic notifier shutdown."""
        self.stop_count += 1
        if self.harness.fail_notifier_stop:
            raise RuntimeError("synthetic notifier-stop failure")


@pytest.fixture
def can_harness(monkeypatch: pytest.MonkeyPatch) -> CanHarness:
    """Patch python-can construction with typed deterministic doubles."""
    harness = CanHarness()

    def open_bus(*, channel: str, interface: str) -> FakeBus:
        """Create the fake bus or raise the requested setup failure."""
        if harness.fail_open:
            raise OSError("synthetic open failure")
        if not channel or interface != "socketcan":
            raise ValueError("unexpected bus arguments")
        bus = FakeBus(harness=harness)
        harness.bus = bus
        return bus

    def open_notifier(bus: can.BusABC, listeners: list[can.Listener]) -> FakeNotifier:
        """Create the fake notifier or raise the requested setup failure."""
        del bus
        if harness.fail_notifier:
            raise RuntimeError("synthetic notifier failure")
        if harness.bus is None:
            raise ValueError("notifier was created before the bus")
        notifier = FakeNotifier(harness=harness, listeners=listeners)
        harness.notifier = notifier
        return notifier

    monkeypatch.setattr(can.interface, "Bus", open_bus)
    monkeypatch.setattr(can, "Notifier", open_notifier)
    return harness


@pytest.fixture(autouse=True)
def clean_transport_registry() -> Generator[None]:
    """Guarantee transport isolation and deterministic cleanup for every test."""
    for transport in tuple(_TransportRegistry._transports.values()):
        transport.close()
    _TransportRegistry._transports.clear()
    yield
    for transport in tuple(_TransportRegistry._transports.values()):
        transport.close()
    _TransportRegistry._transports.clear()
