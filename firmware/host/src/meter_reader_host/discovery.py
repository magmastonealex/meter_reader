"""Device discovery by listening for Hello/Reset frames."""

from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING

from .protocol import MessageType

if TYPE_CHECKING:
    from .bus import MeterReaderBus


async def discover_devices(
    bus: "MeterReaderBus",
    timeout: float = 10.0,
) -> list[int]:
    """Listen for DeviceHello/DeviceReset frames and return unique device suffixes."""
    found: set[int] = set()
    try:
        async with asyncio.timeout(timeout):
            async for mt, suffix, _data in bus.receive():
                if mt in (MessageType.DeviceHello, MessageType.DeviceReset):
                    found.add(suffix)
    except TimeoutError:
        pass
    return sorted(found)
