"""Thread-safe shared CAN managers keyed by interface channel."""

import logging
import threading
import time
from typing import ClassVar, Protocol

import can

from ._motor_state import ServoTelemetry
from ._protocol import (
    CanFrame,
    decode_status,
    status_arbitration_id,
)
from .errors import CanConnectionError

_LOGGER = logging.getLogger(__name__)


class TelemetrySink(Protocol):
    """Consumer implemented by the high-level servo controller."""

    def _accept_telemetry(self, telemetry: ServoTelemetry) -> None:
        """Accept a status sample from the notifier thread."""

    def _accept_listener_error(self, error: Exception) -> None:
        """Accept a decode failure from the notifier thread."""


class MotorListener(can.Listener):
    """Route one motor's exact extended status frames to its sink."""

    def __init__(self, *, motor_id: int, sink: TelemetrySink) -> None:
        """Store immutable routing data."""
        self._status_id = status_arbitration_id(motor_id=motor_id)
        self._sink: TelemetrySink | None = sink

    def deactivate(self) -> None:
        """Stop dispatch even if notifier removal later fails."""
        self._sink = None

    def on_message_received(self, msg: can.Message) -> None:
        """Decode a matching status frame and ignore all other traffic."""
        sink = self._sink
        if (
            sink is None
            or msg.arbitration_id != self._status_id
            or not msg.is_extended_id
        ):
            return
        try:
            telemetry = decode_status(
                data=bytes(msg.data),
                received_at_seconds=time.monotonic(),
            )
        except (TypeError, ValueError) as error:
            sink._accept_listener_error(error)
            return
        sink._accept_telemetry(telemetry)


class CanManager:
    """Own one SocketCAN bus and notifier shared by a channel's motors."""

    def __init__(self, *, channel: str) -> None:
        """Open the bus and start notification transactionally."""
        self.channel = channel
        self._lock = threading.RLock()
        self._listeners: dict[int, MotorListener] = {}
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
                    "Failed to close CAN bus after notifier startup failure"
                )
            raise CanConnectionError(
                f"Failed to start the CAN notifier on {channel!r}."
            ) from error

    @property
    def motor_count(self) -> int:
        """Return the number of active motor registrations."""
        with self._lock:
            return len(self._listeners)

    def register(self, *, motor_id: int, sink: TelemetrySink) -> None:
        """Register one unique motor identifier on this channel."""
        with self._lock:
            if self._closed:
                raise CanConnectionError(f"CAN interface {self.channel!r} is closed.")
            if motor_id in self._listeners:
                raise CanConnectionError(
                    f"Motor ID {motor_id} is already registered on {self.channel!r}."
                )
            listener = MotorListener(motor_id=motor_id, sink=sink)
            try:
                self._notifier.add_listener(listener)
            except Exception as error:
                raise CanConnectionError(
                    f"Failed to register motor ID {motor_id} on {self.channel!r}."
                ) from error
            self._listeners[motor_id] = listener

    def unregister(self, *, motor_id: int) -> None:
        """Deactivate and remove a motor listener if it is registered."""
        with self._lock:
            listener = self._listeners.pop(motor_id, None)
            if listener is None:
                return
            listener.deactivate()
            try:
                self._notifier.remove_listener(listener)
            except Exception:
                _LOGGER.exception(
                    "Failed to remove motor %d listener on %s",
                    motor_id,
                    self.channel,
                )

    def send(self, frame: CanFrame) -> None:
        """Send one encoded extended Classic CAN frame."""
        with self._lock:
            if self._closed:
                raise CanConnectionError(f"CAN interface {self.channel!r} is closed.")
            message = can.Message(
                arbitration_id=frame.arbitration_id,
                data=frame.data,
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
                frame.arbitration_id,
                frame.data.hex(" "),
            )

    def close(self) -> None:
        """Stop notification and close the bus exactly once."""
        with self._lock:
            if self._closed:
                return
            for listener in self._listeners.values():
                listener.deactivate()
            try:
                self._notifier.stop()
            except Exception:
                _LOGGER.exception("Failed to stop CAN notifier on %s", self.channel)
            try:
                self._bus.shutdown()
            except Exception:
                _LOGGER.exception("Failed to close CAN bus on %s", self.channel)
            finally:
                self._listeners.clear()
                self._closed = True


class CanManagerRegistry:
    """Keep each channel manager alive until its final motor is released."""

    _lock: ClassVar[threading.RLock] = threading.RLock()
    _managers: ClassVar[dict[str, CanManager]] = {}

    @classmethod
    def acquire_registered(
        cls,
        *,
        channel: str,
        motor_id: int,
        sink: TelemetrySink,
    ) -> CanManager:
        """Atomically acquire a channel and register a unique motor ID."""
        with cls._lock:
            manager = cls._managers.get(channel)
            created = manager is None
            if manager is None:
                manager = CanManager(channel=channel)
                cls._managers[channel] = manager
            try:
                manager.register(motor_id=motor_id, sink=sink)
            except Exception:
                if created:
                    del cls._managers[channel]
                    manager.close()
                raise
            return manager

    @classmethod
    def release(cls, *, manager: CanManager) -> None:
        """Close and remove a channel manager once it has no motors."""
        with cls._lock:
            if manager.motor_count != 0:
                return
            if cls._managers.get(manager.channel) is manager:
                del cls._managers[manager.channel]
                manager.close()
