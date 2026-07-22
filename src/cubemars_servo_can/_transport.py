"""Thread-safe shared CAN transports keyed by interface channel."""

import logging
import threading
import time
from typing import ClassVar, Protocol

import can

from . import _protocol
from ._models import Telemetry
from .errors import CanConnectionError

_LOGGER = logging.getLogger(__name__)


class _TelemetrySink(Protocol):
    """Consumer implemented by the high-level servo object."""

    def _accept_telemetry(self, telemetry: Telemetry) -> None:
        """Accept a status sample from the notifier thread."""

    def _accept_listener_error(self, error: Exception) -> None:
        """Accept a decoding failure from the notifier thread."""


class _MotorListener(can.Listener):
    """Route exact Servo status frames to one registered motor."""

    def __init__(self, *, motor_id: int, sink: _TelemetrySink) -> None:
        """Store the routing target."""
        self._status_id = _protocol.status_arbitration_id(motor_id=motor_id)
        self._sink = sink

    def on_message_received(self, msg: can.Message) -> None:
        """Decode a matching extended status frame and ignore every other frame."""
        if msg.arbitration_id != self._status_id or not msg.is_extended_id:
            return
        try:
            telemetry = _protocol.decode_status(
                data=bytes(msg.data), received_at_seconds=time.monotonic()
            )
        except (TypeError, ValueError) as error:
            self._sink._accept_listener_error(error)
            return
        self._sink._accept_telemetry(telemetry)


class _ChannelTransport:
    """One CAN bus and notifier shared by motors on the same channel."""

    def __init__(self, *, channel: str) -> None:
        """Open a SocketCAN bus and start its notifier."""
        self.channel = channel
        self._lock = threading.RLock()
        self._listeners: dict[int, _MotorListener] = {}
        self._quarantined_motor_ids: set[int] = set()
        self._closed = False
        try:
            self._bus = can.interface.Bus(channel=channel, interface="socketcan")
        except Exception as error:
            raise CanConnectionError(
                f"Failed to open SocketCAN interface {channel!r}."
            ) from error
        try:
            self._notifier = can.Notifier(self._bus, [])
        except Exception as error:
            try:
                self._bus.shutdown()
            except Exception:
                _LOGGER.exception(
                    "Failed to close the CAN bus after notifier startup failed"
                )
            raise CanConnectionError(
                f"Failed to start the CAN notifier on {channel!r}."
            ) from error

    @property
    def motor_count(self) -> int:
        """Return the number of active, non-quarantined motor identifiers."""
        with self._lock:
            return len(self._listeners) - len(self._quarantined_motor_ids)

    def register(self, *, motor_id: int, sink: _TelemetrySink) -> None:
        """Register one unique motor identifier."""
        with self._lock:
            if self._closed:
                raise CanConnectionError(f"CAN interface {self.channel!r} is closed.")
            if motor_id in self._listeners:
                raise CanConnectionError(
                    f"Motor ID {motor_id} is already registered on {self.channel!r}."
                )
            listener = _MotorListener(motor_id=motor_id, sink=sink)
            try:
                self._notifier.add_listener(listener)
            except Exception as error:
                raise CanConnectionError(
                    f"Failed to register motor ID {motor_id} on {self.channel!r}."
                ) from error
            self._listeners[motor_id] = listener

    def unregister(self, *, motor_id: int) -> None:
        """Remove a registered motor listener if present."""
        with self._lock:
            listener = self._listeners.get(motor_id)
            if listener is not None:
                try:
                    self._notifier.remove_listener(listener)
                except Exception:
                    self._quarantined_motor_ids.add(motor_id)
                    _LOGGER.exception(
                        "Failed to remove motor %d listener on %s",
                        motor_id,
                        self.channel,
                    )
                else:
                    del self._listeners[motor_id]
                    self._quarantined_motor_ids.discard(motor_id)

    def send(self, *, arbitration_id: int, data: bytes) -> None:
        """Send one extended CAN frame."""
        if len(data) > 8:
            raise ValueError("Classic CAN payloads cannot exceed 8 bytes")
        with self._lock:
            if self._closed:
                raise CanConnectionError(f"CAN interface {self.channel!r} is closed.")
            message = can.Message(
                arbitration_id=arbitration_id,
                data=data,
                is_extended_id=True,
            )
            try:
                self._bus.send(message)
            except can.CanError as error:
                raise CanConnectionError(
                    f"Failed to send a CAN frame on {self.channel!r}."
                ) from error
            _LOGGER.debug(
                "Sent Servo CAN frame id=%#x data=%s",
                arbitration_id,
                data.hex(" "),
            )

    def close(self) -> None:
        """Stop notification and close the underlying bus exactly once."""
        with self._lock:
            if self._closed:
                return
            try:
                self._notifier.stop()
            except Exception:
                _LOGGER.exception("Failed to stop the CAN notifier on %s", self.channel)
            try:
                self._bus.shutdown()
            except Exception:
                _LOGGER.exception("Failed to close the CAN bus on %s", self.channel)
            finally:
                self._listeners.clear()
                self._quarantined_motor_ids.clear()
                self._closed = True


class _TransportRegistry:
    """Own channel transports until their last motor is released."""

    _lock = threading.RLock()
    _transports: ClassVar[dict[str, _ChannelTransport]] = {}

    @classmethod
    def acquire(cls, *, channel: str) -> _ChannelTransport:
        """Return the existing channel transport or open a new one."""
        with cls._lock:
            transport = cls._transports.get(channel)
            if transport is None:
                transport = _ChannelTransport(channel=channel)
                cls._transports[channel] = transport
            return transport

    @classmethod
    def acquire_registered(
        cls, *, channel: str, motor_id: int, sink: _TelemetrySink
    ) -> _ChannelTransport:
        """Atomically acquire a channel and register a unique motor ID."""
        with cls._lock:
            transport = cls._transports.get(channel)
            created = transport is None
            if transport is None:
                transport = _ChannelTransport(channel=channel)
                cls._transports[channel] = transport
            try:
                transport.register(motor_id=motor_id, sink=sink)
            except Exception:
                if created:
                    del cls._transports[channel]
                    transport.close()
                raise
            return transport

    @classmethod
    def release(cls, *, transport: _ChannelTransport) -> None:
        """Close and discard an empty channel transport."""
        with cls._lock:
            if transport.motor_count != 0:
                return
            if cls._transports.get(transport.channel) is transport:
                del cls._transports[transport.channel]
                transport.close()
