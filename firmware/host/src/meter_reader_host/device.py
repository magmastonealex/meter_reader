"""High-level device abstraction for a single meter reader unit."""

from __future__ import annotations

import asyncio
import struct
from typing import TYPE_CHECKING

from .protocol import MagnetometerConfig, MessageType

if TYPE_CHECKING:
    from .bus import MeterReaderBus


class CommandError(Exception):
    """Raised when a command results in a warning or times out."""




class MeterReaderDevice:
    """Represents a single meter reader device on the CAN bus."""

    RESET_MAGIC = bytes([0xBA, 0xAD, 0xF0, 0x0D])

    def __init__(self, bus: "MeterReaderBus", device_suffix: int) -> None:
        self._bus = bus
        self.device_suffix = device_suffix

    async def wait_for_ack(
        self,
        cmd_code: int,
        timeout: float = 2.0,
    ) -> None:
        """Wait for a DeviceAck matching cmd_code, raise on warning or timeout."""
        async with asyncio.timeout(timeout):
            async for mt, data in self._bus.receive_for(self.device_suffix):
                if mt == MessageType.DeviceAck:
                    if len(data) >= 2 and data[1] == cmd_code:
                        return
                elif mt == MessageType.DeviceWarning:
                    code = data[1] if len(data) >= 2 else 0
                    raise CommandError(f"Device warning 0x{code:02X} in response to 0x{cmd_code:02X}")
                elif mt == MessageType.DeviceError:
                    code = data[1] if len(data) >= 2 else 0
                    raise CommandError(f"Device error 0x{code:02X}")

    async def get_config(self) -> MagnetometerConfig:
        """Query and return the current magnetometer configuration."""
        self._bus.send(self.device_suffix, MessageType.DeviceGetCurrentConfig)
        async with asyncio.timeout(2.0):
            async for mt, data in self._bus.receive_for(self.device_suffix):
                if mt == MessageType.DeviceGetCurrentConfigResp:
                    # Payload: [reserved, cfg_b3, cfg_b2, cfg_b1, cfg_b0]
                    if len(data) >= 5:
                        return MagnetometerConfig.from_bytes(data[1:5])
                    raise CommandError(f"Short GetCurrentConfigResp payload: {data.hex()}")
                elif mt == MessageType.DeviceWarning:
                    code = data[1] if len(data) >= 2 else 0
                    raise CommandError(f"Device warning 0x{code:02X} on GetCurrentConfig")

    async def set_config(self, config: MagnetometerConfig, timeout: float = 2.0) -> None:
        """Send a new configuration and wait for ack."""
        payload = config.to_bytes()
        self._bus.send(self.device_suffix, MessageType.DeviceReconfigure, payload)
        await self.wait_for_ack(int(MessageType.DeviceReconfigure), timeout=timeout)

    async def enter_debug_mode(self, high_sensitivity: bool = False) -> None:
        """Enter debug mode; streams DeviceRawData frames until exit."""
        payload = bytes([0x01, 0x01 if high_sensitivity else 0x00])
        self._bus.send(self.device_suffix, MessageType.DeviceDebugMode, payload)
        await self.wait_for_ack(int(MessageType.DeviceDebugMode))

    async def exit_debug_mode(self) -> None:
        """Exit debug mode and return to normal pulse counting."""
        self._bus.send(self.device_suffix, MessageType.DeviceDebugMode, bytes([0x00]))
        await self.wait_for_ack(int(MessageType.DeviceDebugMode))

    async def set_start_point(self, ticks: int) -> None:
        """Set the meter tick counter to an arbitrary value."""
        payload = struct.pack(">I", ticks)
        self._bus.send(self.device_suffix, MessageType.DeviceSetStartPoint, payload)
        await self.wait_for_ack(int(MessageType.DeviceSetStartPoint))

    async def reset(self) -> None:
        """Command the device to perform a full power-on reset."""
        self._bus.send(self.device_suffix, MessageType.DevicePleaseReset, self.RESET_MAGIC)
        await self.wait_for_ack(int(MessageType.DevicePleaseReset))

    async def _wait_fw_status(self, timeout: float = 5.0) -> tuple[int, int]:
        """Wait for a FwUpdateStatus response. Returns (status_code, next_seq)."""
        async with asyncio.timeout(timeout):
            async for mt, data in self._bus.receive_for(self.device_suffix):
                if mt == MessageType.FwUpdateStatus and len(data) >= 3:
                    status = data[0]
                    next_seq = struct.unpack(">H", data[1:3])[0]
                    return status, next_seq
        raise TimeoutError("No FwUpdateStatus received")

