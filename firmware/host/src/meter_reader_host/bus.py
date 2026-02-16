"""Async CAN bus wrapper for the meter reader protocol."""

from __future__ import annotations

import asyncio
from collections.abc import AsyncGenerator
from contextlib import asynccontextmanager
from typing import TYPE_CHECKING

import can

from .protocol import MessageType, make_can_id, parse_can_id

if TYPE_CHECKING:
    pass


class MeterReaderBus:
    """Async context manager wrapping a python-can socketcan bus."""

    def __init__(self, channel: str = "can0", bitrate: int = 100_000) -> None:
        self._channel = channel
        self._bitrate = bitrate
        self._bus: can.Bus | None = None
        self._reader: can.AsyncBufferedReader | None = None
        self._notifier: can.Notifier | None = None

    async def __aenter__(self) -> "MeterReaderBus":
        loop = asyncio.get_event_loop()
        self._bus = can.Bus(
            interface="socketcan",
            channel=self._channel,
            bitrate=self._bitrate,
        )
        self._reader = can.AsyncBufferedReader()
        self._notifier = can.Notifier(self._bus, [self._reader], loop=loop)
        return self

    async def __aexit__(self, *_: object) -> None:
        if self._notifier is not None:
            self._notifier.stop()
        if self._bus is not None:
            self._bus.shutdown()

    def send(
        self,
        device_suffix: int,
        msg_type: MessageType | int,
        payload: bytes = b"",
    ) -> None:
        """Build and send a CAN extended frame to the given device."""
        if self._bus is None:
            raise RuntimeError("Bus not open")
        can_id = make_can_id(device_suffix, msg_type)
        msg = can.Message(
            arbitration_id=can_id,
            data=payload,
            is_extended_id=True,
        )
        self._bus.send(msg)

    async def receive(self) -> AsyncGenerator[tuple[MessageType | int, int, bytes], None]:
        """Async generator yielding (msg_type, device_suffix, data) for each frame."""
        if self._reader is None:
            raise RuntimeError("Bus not open")
        while True:
            msg: can.Message = await self._reader.get_message()
            if not msg.is_extended_id:
                continue
            msg_type, device_suffix = parse_can_id(msg.arbitration_id)
            yield msg_type, device_suffix, bytes(msg.data)

    async def receive_for(
        self, device_suffix: int
    ) -> AsyncGenerator[tuple[MessageType | int, bytes], None]:
        """Filtered async generator yielding (msg_type, data) for one device."""
        async for mt, suffix, data in self.receive():
            if suffix == device_suffix:
                yield mt, data


@asynccontextmanager
async def open_bus(channel: str = "can0", bitrate: int = 100_000):
    """Convenience async context manager for opening a MeterReaderBus."""
    bus = MeterReaderBus(channel=channel, bitrate=bitrate)
    async with bus:
        yield bus
