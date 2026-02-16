"""Textual TUI for the meter reader host."""

from __future__ import annotations

import asyncio
import struct
from dataclasses import dataclass, field
from datetime import datetime

import click
from textual import on, work
from textual.app import App, ComposeResult
from textual.containers import Container, Horizontal, ScrollableContainer, Vertical
from textual.message import Message
from textual.reactive import reactive
from textual.screen import ModalScreen, Screen
from textual.widgets import (
    Button,
    Checkbox,
    DataTable,
    Footer,
    Header,
    Input,
    Label,
    Select,
    Static,
)

from .bus import MeterReaderBus
from .device import CommandError, MeterReaderDevice
from .protocol import (
    AXIS_NAMES,
    ERROR_CODES,
    WARNING_CODES,
    MagnetometerConfig,
    MessageType,
    parse_can_id,
)


# ---------------------------------------------------------------------------
# Internal state
# ---------------------------------------------------------------------------


@dataclass
class DeviceState:
    suffix: int
    last_seen: datetime = field(default_factory=datetime.now)
    meter_active: bool = False
    tick_count: int | None = None
    config: MagnetometerConfig | None = None
    in_debug: bool = False


@dataclass
class RawSample:
    x: int
    y: int
    z: int
    status: int
    timestamp: datetime = field(default_factory=datetime.now)


# ---------------------------------------------------------------------------
# Messages (for cross-task communication)
# ---------------------------------------------------------------------------


class FrameReceived(Message):
    """A raw CAN frame has been received."""

    def __init__(self, msg_type: MessageType | int, suffix: int, data: bytes) -> None:
        super().__init__()
        self.msg_type = msg_type
        self.suffix = suffix
        self.data = data


class DeviceUpdated(Message):
    """A device's state has changed."""

    def __init__(self, suffix: int) -> None:
        super().__init__()
        self.suffix = suffix


# ---------------------------------------------------------------------------
# Config editor modal
# ---------------------------------------------------------------------------


class ConfigEditorScreen(ModalScreen[MagnetometerConfig | None]):
    """Modal dialog for editing device configuration."""

    BINDINGS = [("escape", "dismiss(None)", "Cancel")]

    def __init__(self, config: MagnetometerConfig, suffix: int) -> None:
        super().__init__()
        self._config = config
        self._suffix = suffix

    def compose(self) -> ComposeResult:
        with Container(id="config-dialog"):
            yield Label(f"Configure device 0x{self._suffix:04X}", id="config-title")
            with Vertical():
                yield Label("rising_val (signed integer):")
                yield Input(str(self._config.rising_val), id="rising-val", type="integer")
                yield Label("rising_delta (0–511):")
                yield Input(str(self._config.rising_delta), id="rising-delta", type="integer")
                yield Label("Axis:")
                yield Select(
                    [(name, str(i)) for i, name in AXIS_NAMES.items()],
                    value=str(self._config.axis),
                    id="axis-select",
                )
                yield Checkbox("High sensitivity", self._config.high_sensitivity, id="high-sens")
            with Horizontal(id="config-buttons"):
                yield Button("Apply", variant="primary", id="apply")
                yield Button("Cancel", id="cancel")

    @on(Button.Pressed, "#apply")
    def apply(self) -> None:
        try:
            rv = int(self.query_one("#rising-val", Input).value)
            rd = int(self.query_one("#rising-delta", Input).value)
            axis_val = self.query_one("#axis-select", Select).value
            axis = int(axis_val) if axis_val and axis_val != Select.BLANK else self._config.axis
            hs = self.query_one("#high-sens", Checkbox).value
            config = MagnetometerConfig(
                rising_val=rv,
                rising_delta=rd,
                axis=axis,
                high_sensitivity=hs,
                rate=self._config.rate,
            )
            self.dismiss(config)
        except (ValueError, TypeError) as e:
            self.notify(f"Invalid input: {e}", severity="error")

    @on(Button.Pressed, "#cancel")
    def cancel(self) -> None:
        self.dismiss(None)


# ---------------------------------------------------------------------------
# Device detail screen
# ---------------------------------------------------------------------------


class DeviceDetailScreen(Screen):
    """Detail view for a single device."""

    BINDINGS = [
        ("escape", "app.pop_screen()", "Back"),
        ("d", "toggle_debug", "Debug"),
        ("r", "reset_device", "Reset"),
    ]

    def __init__(self, suffix: int, app_ref: "MeterReaderApp") -> None:
        super().__init__()
        self._suffix = suffix
        self._app_ref = app_ref

    def compose(self) -> ComposeResult:
        yield Header()
        yield Label(f"Device 0x{self._suffix:04X}", id="detail-title")
        with Vertical(id="detail-body"):
            yield Static("", id="device-info")
            yield Static("", id="config-info")
            with Horizontal(id="detail-buttons"):
                yield Button("Toggle Debug", id="debug-btn")
                yield Button("Reconfigure", id="reconfig-btn")
                yield Button("Set Ticks", id="set-ticks-btn")
                yield Button("Reset Device", variant="error", id="reset-btn")
            yield Label("Raw Data (debug mode):")
            yield DataTable(id="raw-data-table")
        yield Footer()

    def on_mount(self) -> None:
        table = self.query_one("#raw-data-table", DataTable)
        table.add_columns("Time", "X", "Y", "Z", "Status")
        self._refresh_info()

    def _refresh_info(self) -> None:
        state = self._app_ref.devices.get(self._suffix)
        if state is None:
            return
        info = self.query_one("#device-info", Static)
        info.update(
            f"Last seen: {state.last_seen.strftime('%H:%M:%S')}  "
            f"Active: {'Yes' if state.meter_active else 'No'}  "
            f"Ticks: {state.tick_count if state.tick_count is not None else '?'}  "
            f"Debug: {'ON' if state.in_debug else 'off'}"
        )
        cfg_widget = self.query_one("#config-info", Static)
        if state.config:
            cfg_widget.update(f"Config: {state.config}")
        else:
            cfg_widget.update("Config: (unknown — press Reconfigure to fetch)")

    def on_frame_received(self, msg: FrameReceived) -> None:
        if msg.suffix != self._suffix:
            return
        self._refresh_info()
        if msg.msg_type == MessageType.DeviceRawData and len(msg.data) >= 7:
            x, y, z = struct.unpack(">hhh", msg.data[:6])
            status = msg.data[6]
            table = self.query_one("#raw-data-table", DataTable)
            table.add_row(
                datetime.now().strftime("%H:%M:%S.%f")[:-3],
                str(x), str(y), str(z), f"0x{status:02X}",
            )
            # Keep table small
            if table.row_count > 200:
                table.remove_row(table.rows[0].key)  # type: ignore[union-attr]

    @on(Button.Pressed, "#debug-btn")
    def toggle_debug(self) -> None:  # type: ignore[override]
        state = self._app_ref.devices.get(self._suffix)
        if state and state.in_debug:
            self._app_ref.run_command(self._suffix, "exit_debug")
        else:
            self._app_ref.run_command(self._suffix, "enter_debug")

    @on(Button.Pressed, "#reconfig-btn")
    def show_reconfig(self) -> None:
        state = self._app_ref.devices.get(self._suffix)
        config = state.config if state and state.config else MagnetometerConfig()

        async def _push() -> None:
            result = await self.app.push_screen_wait(
                ConfigEditorScreen(config, self._suffix)
            )
            if result is not None:
                self._app_ref.run_command(self._suffix, "set_config", result)

        self.run_worker(_push(), exclusive=True)

    @on(Button.Pressed, "#set-ticks-btn")
    def set_ticks(self) -> None:
        self.app.push_screen(SetTicksScreen(self._suffix, self._app_ref))

    @on(Button.Pressed, "#reset-btn")
    def reset_device(self) -> None:  # type: ignore[override]
        self._app_ref.run_command(self._suffix, "reset")


class SetTicksScreen(ModalScreen[None]):
    """Simple dialog to set the tick counter."""

    BINDINGS = [("escape", "dismiss(None)", "Cancel")]

    def __init__(self, suffix: int, app_ref: "MeterReaderApp") -> None:
        super().__init__()
        self._suffix = suffix
        self._app_ref = app_ref

    def compose(self) -> ComposeResult:
        with Container(id="ticks-dialog"):
            yield Label(f"Set ticks for 0x{self._suffix:04X}")
            yield Input(placeholder="Enter tick count", id="ticks-input", type="integer")
            with Horizontal():
                yield Button("Set", variant="primary", id="set-btn")
                yield Button("Cancel", id="cancel-btn")

    @on(Button.Pressed, "#set-btn")
    def do_set(self) -> None:
        try:
            val = int(self.query_one("#ticks-input", Input).value)
            self._app_ref.run_command(self._suffix, "set_ticks", val)
            self.dismiss(None)
        except ValueError:
            self.notify("Enter a valid integer", severity="error")

    @on(Button.Pressed, "#cancel-btn")
    def cancel(self) -> None:
        self.dismiss(None)


# ---------------------------------------------------------------------------
# Main TUI app
# ---------------------------------------------------------------------------


class MeterReaderApp(App):
    """Main TUI application."""

    CSS = """
    Screen {
        background: $surface;
    }
    #device-list {
        height: 100%;
        border: solid $primary;
    }
    #config-dialog, #ticks-dialog {
        width: 60;
        height: auto;
        background: $panel;
        border: solid $primary;
        padding: 1 2;
    }
    #config-title {
        text-align: center;
        text-style: bold;
    }
    #config-buttons, #detail-buttons {
        height: 3;
        align: center middle;
    }
    #detail-title {
        text-style: bold;
        padding: 1;
    }
    DataTable {
        height: 1fr;
    }
    """

    BINDINGS = [
        ("q", "quit", "Quit"),
        ("enter", "select_device", "Open"),
    ]

    def __init__(self, channel: str = "can0") -> None:
        super().__init__()
        self._channel = channel
        self.devices: dict[int, DeviceState] = {}
        self._bus: MeterReaderBus | None = None

    def compose(self) -> ComposeResult:
        yield Header()
        with ScrollableContainer():
            yield DataTable(id="device-list")
        yield Footer()

    def on_mount(self) -> None:
        table = self.query_one("#device-list", DataTable)
        table.add_columns("Suffix", "Last Seen", "Active", "Ticks", "Status")
        table.cursor_type = "row"
        self._start_bus()

    @work(thread=False)
    async def _start_bus(self) -> None:
        try:
            self._bus = MeterReaderBus(channel=self._channel)
            async with self._bus:
                async for mt, suffix, data in self._bus.receive():
                    self._handle_frame(mt, suffix, data)
                    self.post_message(FrameReceived(mt, suffix, data))
        except Exception as e:
            self.notify(f"Bus error: {e}", severity="error")

    def _handle_frame(
        self, mt: MessageType | int, suffix: int, data: bytes
    ) -> None:
        if suffix not in self.devices:
            self.devices[suffix] = DeviceState(suffix=suffix)

        state = self.devices[suffix]
        state.last_seen = datetime.now()

        if mt == MessageType.DeviceHello and len(data) >= 2:
            state.meter_active = bool(data[1])
        elif mt == MessageType.DeviceMeterTicks and len(data) >= 5:
            (state.tick_count,) = struct.unpack(">I", data[1:5])
        elif mt == MessageType.DeviceGetCurrentConfigResp and len(data) >= 5:
            state.config = MagnetometerConfig.from_bytes(data[1:5])

        self._refresh_table()

    def _refresh_table(self) -> None:
        table = self.query_one("#device-list", DataTable)
        # Rebuild rows
        table.clear()
        for suffix, state in sorted(self.devices.items()):
            table.add_row(
                f"0x{suffix:04X}",
                state.last_seen.strftime("%H:%M:%S"),
                "Yes" if state.meter_active else "No",
                str(state.tick_count) if state.tick_count is not None else "?",
                "DEBUG" if state.in_debug else "normal",
            )

    @on(DataTable.RowSelected, "#device-list")
    def open_device(self, event: DataTable.RowSelected) -> None:
        # Find device by row index
        sorted_suffixes = sorted(self.devices.keys())
        idx = event.cursor_row
        if 0 <= idx < len(sorted_suffixes):
            suffix = sorted_suffixes[idx]
            self.push_screen(DeviceDetailScreen(suffix, self))

    def run_command(self, suffix: int, cmd: str, *args: object) -> None:
        """Dispatch a device command as a background worker."""
        self.run_worker(self._do_command(suffix, cmd, *args), exclusive=False)

    async def _do_command(self, suffix: int, cmd: str, *args: object) -> None:
        if self._bus is None:
            self.notify("Bus not connected", severity="error")
            return
        device = MeterReaderDevice(self._bus, suffix)
        try:
            if cmd == "enter_debug":
                await device.enter_debug_mode()
                if suffix in self.devices:
                    self.devices[suffix].in_debug = True
                self.notify(f"0x{suffix:04X}: debug mode ON")
            elif cmd == "exit_debug":
                await device.exit_debug_mode()
                if suffix in self.devices:
                    self.devices[suffix].in_debug = False
                self.notify(f"0x{suffix:04X}: debug mode OFF")
            elif cmd == "set_config":
                (config,) = args
                await device.set_config(config)  # type: ignore[arg-type]
                if suffix in self.devices:
                    self.devices[suffix].config = config  # type: ignore[assignment]
                self.notify(f"0x{suffix:04X}: config updated")
            elif cmd == "set_ticks":
                (ticks,) = args
                await device.set_start_point(int(ticks))  # type: ignore[arg-type]
                self.notify(f"0x{suffix:04X}: ticks set to {ticks}")
            elif cmd == "reset":
                await device.reset()
                self.notify(f"0x{suffix:04X}: reset sent")
            elif cmd == "get_config":
                config = await device.get_config()
                if suffix in self.devices:
                    self.devices[suffix].config = config
                self._refresh_table()
        except (CommandError, TimeoutError) as e:
            self.notify(f"0x{suffix:04X} {cmd} failed: {e}", severity="error")


def main() -> None:
    """Entry point for meter-reader-tui."""

    @click.command()
    @click.option("--channel", default="can0", show_default=True, help="CAN interface")
    def _run(channel: str) -> None:
        app = MeterReaderApp(channel=channel)
        app.run()

    _run()
