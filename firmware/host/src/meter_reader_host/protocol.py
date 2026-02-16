"""CAN protocol definitions for the meter reader."""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum


class MessageType(IntEnum):
    # Device → Host
    DeviceError = 0x00
    DeviceAck = 0x01
    DeviceWarning = 0x02
    DeviceReset = 0x0F
    DeviceHello = 0x10
    DeviceRawData = 0x11
    DeviceMeterTicks = 0x12
    DeviceGetCurrentConfigResp = 0x19
    DeviceCANStatus = 0x1A
    # Host → Device
    DeviceDebugMode = 0x14
    DeviceReconfigure = 0x15
    DeviceSetStartPoint = 0x16
    DevicePleaseReset = 0x17
    DeviceGetCurrentConfig = 0x18


def make_can_id(device_suffix: int, msg_type: MessageType | int) -> int:
    """Encode a 29-bit extended CAN ID from device suffix and message type."""
    return (int(msg_type) << 16) | (device_suffix & 0xFFFF)


def parse_can_id(raw_id: int) -> tuple[MessageType, int]:
    """Decode a 29-bit extended CAN ID into (MessageType, device_suffix)."""
    msg_type_val = (raw_id >> 16) & 0xFF
    device_suffix = raw_id & 0xFFFF
    try:
        msg_type = MessageType(msg_type_val)
    except ValueError:
        msg_type = msg_type_val  # type: ignore[assignment]
    return msg_type, device_suffix


@dataclass
class MagnetometerConfig:
    """32-bit packed configuration bitfield for the MLX90394 sensor."""

    rising_val: int = 0       # i16: center point of waveform
    rising_delta: int = 100   # u9 (0–511): hysteresis half-width
    axis: int = 0             # 0=X, 1=Y, 2=Z
    high_sensitivity: bool = False
    rate: int = 6             # MLX rate code (ignored in normal op, always 100 Hz)

    @classmethod
    def from_u32(cls, value: int) -> "MagnetometerConfig":
        """Decode config from a 32-bit big-endian integer."""
        # rising_val: bits [31:16] as signed i16
        rising_val = struct.unpack(">h", struct.pack(">H", (value >> 16) & 0xFFFF))[0]
        # rising_delta: bits [15:7] (9-bit unsigned)
        rising_delta = (value >> 7) & 0x1FF
        # axis: bits [6:5]
        axis = (value >> 5) & 0x03
        # high_sensitivity: bit [4]
        high_sensitivity = bool((value >> 4) & 0x01)
        # rate: bits [3:0]
        rate = value & 0x0F
        return cls(
            rising_val=rising_val,
            rising_delta=rising_delta,
            axis=axis,
            high_sensitivity=high_sensitivity,
            rate=rate,
        )

    def to_u32(self) -> int:
        """Encode config to a 32-bit integer for transmission (big-endian)."""
        # rising_val as u16 bits [31:16]
        rv_u16 = struct.unpack(">H", struct.pack(">h", self.rising_val))[0]
        value = rv_u16 << 16
        value |= (self.rising_delta & 0x1FF) << 7
        value |= (self.axis & 0x03) << 5
        value |= (1 if self.high_sensitivity else 0) << 4
        value |= self.rate & 0x0F
        return value

    def to_bytes(self) -> bytes:
        """Encode config to 4 bytes big-endian."""
        return struct.pack(">I", self.to_u32())

    @classmethod
    def from_bytes(cls, data: bytes) -> "MagnetometerConfig":
        """Decode config from 4 bytes big-endian."""
        (value,) = struct.unpack(">I", data)
        return cls.from_u32(value)

    def axis_name(self) -> str:
        return ["X", "Y", "Z"][self.axis] if self.axis < 3 else f"?({self.axis})"

    def __str__(self) -> str:
        return (
            f"rising_val={self.rising_val}, rising_delta={self.rising_delta}, "
            f"axis={self.axis_name()}, high_sensitivity={self.high_sensitivity}, "
            f"rate={self.rate}"
        )


ERROR_CODES: dict[int, str] = {
    0x01: "I2C peripheral initialization failed",
    0x02: "Magnetometer config in flash is invalid/corrupt",
    0x03: "MLX90394 initialization failed after 10 retries",
    0x04: "I2C bus latch-up detected (transaction timed out)",
}

WARNING_CODES: dict[int, str] = {
    0x01: "Failed to reconfigure magnetometer for debug mode",
    0x02: "Failed to persist meter ticks to flash",
    0x03: "Payload length was not 4 bytes (SetStartPoint)",
    0x04: "Failed to persist new config to flash",
    0x05: "Payload length was not 4 bytes (Reconfigure)",
    0x10: "Failed to reconfigure magnetometer back to normal (debug exit)",
    0x11: "Stored config is invalid for MLX driver (debug exit)",
    0x80: "Failed to enqueue config response frame",
    0xAB: "Selected sensor axis is saturated; consider disabling high-sensitivity",
    0xCC: "Magnetometer data overflow (main loop too slow, reading missed)",
    0xFE: "Payload length was not 4 bytes (PleaseReset)",
}

RESET_CAUSES: dict[int, str] = {
    0x00: "Power-on reset",
    0x01: "Clock loss reset",
    0x02: "VBAT brownout",
    0x03: "Wakeup reset",
    0x04: "CPU reset",
    0x05: "Watchdog reset",
    0x06: "Software reset",
    0x07: "BOR reset",
}

AXIS_NAMES = {0: "X", 1: "Y", 2: "Z"}


