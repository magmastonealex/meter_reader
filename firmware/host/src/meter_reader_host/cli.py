"""Click-based CLI for testing and scripting the meter reader."""

from __future__ import annotations

import asyncio
import struct
import sys
from datetime import datetime

import click

from .bus import MeterReaderBus
from .device import CommandError, MeterReaderDevice
from .discovery import discover_devices
from .protocol import (
    AXIS_NAMES,
    ERROR_CODES,
    MagnetometerConfig,
    MessageType,
    RESET_CAUSES,
    WARNING_CODES,
)


def _run(coro):
    """Run an async coroutine from a sync Click command."""
    return asyncio.run(coro)


@click.group()
def cli() -> None:
    """Meter reader host CLI."""


@cli.command()
@click.option("--channel", default="can0", show_default=True, help="CAN interface")
@click.option("--timeout", default=10.0, show_default=True, type=float, help="Discovery timeout (s)")
def discover(channel: str, timeout: float) -> None:
    """Discover devices by listening for Hello/Reset frames."""

    async def _run_discover() -> None:
        click.echo(f"Listening on {channel} for {timeout}s ...")
        async with MeterReaderBus(channel=channel) as bus:
            found = await discover_devices(bus, timeout=timeout)
        if found:
            click.echo(f"Found {len(found)} device(s):")
            for suffix in found:
                click.echo(f"  0x{suffix:04X}")
        else:
            click.echo("No devices found.")

    _run(_run_discover())


@cli.command("get-config")
@click.option("--channel", default="can0", show_default=True)
@click.option("--device", required=True, type=lambda x: int(x, 0), help="Device suffix (hex or dec)")
def get_config(channel: str, device: int) -> None:
    """Query and display the current device configuration."""

    async def _run_get() -> None:
        async with MeterReaderBus(channel=channel) as bus:
            dev = MeterReaderDevice(bus, device)
            try:
                config = await dev.get_config()
                click.echo(f"Config for 0x{device:04X}:")
                click.echo(f"  rising_val:       {config.rising_val}")
                click.echo(f"  rising_delta:     {config.rising_delta}")
                click.echo(f"  axis:             {config.axis_name()} ({config.axis})")
                click.echo(f"  high_sensitivity: {config.high_sensitivity}")
                click.echo(f"  rate:             {config.rate}")
            except (CommandError, TimeoutError) as e:
                click.echo(f"Error: {e}", err=True)
                sys.exit(1)

    _run(_run_get())


@cli.command("set-config")
@click.option("--channel", default="can0", show_default=True)
@click.option("--device", required=True, type=lambda x: int(x, 0))
@click.option("--rising-val", required=True, type=int, help="Center point (signed)")
@click.option("--rising-delta", required=True, type=int, help="Hysteresis (0–511)")
@click.option("--axis", required=True, type=int, help="0=X, 1=Y, 2=Z")
@click.option("--high-sensitivity", is_flag=True, default=False)
@click.option("--rate", default=6, show_default=True, type=int, help="Rate code (6=100Hz)")
def set_config(
    channel: str,
    device: int,
    rising_val: int,
    rising_delta: int,
    axis: int,
    high_sensitivity: bool,
    rate: int,
) -> None:
    """Set the device configuration."""

    async def _run_set() -> None:
        config = MagnetometerConfig(
            rising_val=rising_val,
            rising_delta=rising_delta,
            axis=axis,
            high_sensitivity=high_sensitivity,
            rate=rate,
        )
        async with MeterReaderBus(channel=channel) as bus:
            dev = MeterReaderDevice(bus, device)
            try:
                await dev.set_config(config)
                click.echo(f"Config applied to 0x{device:04X}: {config}")
            except (CommandError, TimeoutError) as e:
                click.echo(f"Error: {e}", err=True)
                sys.exit(1)

    _run(_run_set())


_AXIS_COLORS = {"X": "tab:red", "Y": "tab:green", "Z": "tab:blue"}


@cli.command("debug")
@click.option("--channel", default="can0", show_default=True)
@click.option("--device", required=True, type=lambda x: int(x, 0))
@click.option("--high-sensitivity", is_flag=True, default=False)
@click.option("--duration", default=30.0, show_default=True, type=float, help="Seconds to stream data")
@click.option("--off", is_flag=True, default=False, help="Send debug-off command and exit")
@click.option("--graph", is_flag=True, default=False, help="Show live matplotlib chart of all axes")
@click.option(
    "--check-config",
    is_flag=True,
    default=False,
    help="Fetch stored config, graph data with threshold lines to validate detection",
)
def debug(
    channel: str,
    device: int,
    high_sensitivity: bool,
    duration: float,
    off: bool,
    graph: bool,
    check_config: bool,
) -> None:
    """Enter debug mode and stream raw magnetometer data."""

    async def _run_debug() -> None:
        if check_config and off:
            raise click.UsageError("--check-config and --off are mutually exclusive")

        use_graph = graph or check_config
        WINDOW = 300  # rolling window: ~60 s at 5 Hz

        # Import matplotlib early so we fail before touching the bus
        plt = None
        if use_graph:
            try:
                import matplotlib.pyplot as _plt
                plt = _plt
            except ImportError:
                raise click.ClickException(
                    "matplotlib is required for --graph / --check-config. "
                    "Install it with: uv add matplotlib"
                )

        async with MeterReaderBus(channel=channel) as bus:
            dev = MeterReaderDevice(bus, device)
            try:
                if off:
                    await dev.exit_debug_mode()
                    click.echo(f"0x{device:04X}: debug mode OFF")
                    return

                # --check-config: fetch stored config to drive sensitivity + threshold display
                config = None
                if check_config:
                    click.echo(f"Fetching config from 0x{device:04X} ...")
                    config = await dev.get_config()
                    click.echo(f"  {config}")

                # Determine sensitivity to use
                actual_sens = config.high_sensitivity if config else high_sensitivity

                # Build the matplotlib figure
                fig = None
                ax = None
                lines: dict[str, object] = {}
                bufs: dict[str, list[int]] = {}
                if use_graph:
                    plt.ion()
                    fig, ax = plt.subplots(figsize=(10, 4))
                    ax.set_xlabel("Sample")
                    ax.set_ylabel("Value")

                    if check_config:
                        sel = AXIS_NAMES[config.axis]
                        ax.set_title(
                            f"Config validation — 0x{device:04X} — "
                            f"Axis {sel}, rising_val={config.rising_val}, "
                            f"\u0394={config.rising_delta}, "
                            f"{'high' if config.high_sensitivity else 'low'} sensitivity"
                        )
                        for name, color in _AXIS_COLORS.items():
                            if name == sel:
                                (lines[name],) = ax.plot(
                                    [], [], color=color, linewidth=1.5, label=name, zorder=3
                                )
                            else:
                                (lines[name],) = ax.plot(
                                    [], [], color=color, linewidth=0.8,
                                    linestyle=":", alpha=0.25,
                                    label=f"{name} (inactive)", zorder=2,
                                )
                            bufs[name] = []
                        upper = config.rising_val + config.rising_delta
                        lower = config.rising_val - config.rising_delta
                        ax.axhline(
                            config.rising_val, color="gray", linestyle="--",
                            linewidth=0.8, label=f"center ({config.rising_val})", zorder=1,
                        )
                        ax.axhline(
                            upper, color="limegreen", linestyle="--",
                            linewidth=1.0, label=f"trigger \u2191 ({upper})", zorder=1,
                        )
                        ax.axhline(
                            lower, color="tomato", linestyle="--",
                            linewidth=1.0, label=f"trigger \u2193 ({lower})", zorder=1,
                        )
                    else:
                        ax.set_title(f"Raw magnetometer data — device 0x{device:04X}")
                        for name, color in _AXIS_COLORS.items():
                            (lines[name],) = ax.plot([], [], color=color, label=name, linewidth=1)
                            bufs[name] = []

                    ax.legend(loc="upper right", fontsize=8)
                    fig.tight_layout()
                    plt.show(block=False)

                await dev.enter_debug_mode(high_sensitivity=actual_sens)
                click.echo(f"Debug mode ON for 0x{device:04X}. Streaming for {duration}s ...")
                if not use_graph:
                    click.echo("     X       Y       Z   Status")
                count = 0
                try:
                    async with asyncio.timeout(duration):
                        async for mt, data in bus.receive_for(device):
                            if mt == MessageType.DeviceRawData and len(data) >= 7:
                                x, y, z = struct.unpack(">hhh", data[:6])
                                status = data[6]
                                if not use_graph:
                                    click.echo(f"{x:7d} {y:7d} {z:7d}   0x{status:02X}")
                                else:
                                    for name, v in zip(("X", "Y", "Z"), (x, y, z)):
                                        bufs[name].append(v)
                                        if len(bufs[name]) > WINDOW:
                                            bufs[name] = bufs[name][-WINDOW:]
                                    count += 1
                                    # Redraw every 3 samples (~1.7 Hz at 5 Hz data rate)
                                    if count % 3 == 0:
                                        for name, line in lines.items():
                                            d = bufs[name]
                                            line.set_xdata(range(len(d)))
                                            line.set_ydata(d)
                                        ax.relim()
                                        ax.autoscale_view()
                                        fig.canvas.draw()
                                        fig.canvas.flush_events()
                                    if not plt.fignum_exists(fig.number):
                                        break  # user closed the window
                except TimeoutError:
                    pass
                await dev.exit_debug_mode()
                click.echo("Debug mode OFF.")
                if use_graph and plt.fignum_exists(fig.number):
                    click.echo("Close the chart window to exit.")
                    plt.ioff()
                    plt.show()
            except (CommandError, TimeoutError) as e:
                click.echo(f"Error: {e}", err=True)
                sys.exit(1)

    _run(_run_debug())


# Frame types shown by the monitor command; everything else is suppressed.
_MONITOR_TYPES = {
    MessageType.DeviceError,
    MessageType.DeviceWarning,
    MessageType.DeviceReset,
    MessageType.DeviceHello,
    MessageType.DeviceMeterTicks,
    MessageType.DeviceCANStatus,
}


def _format_monitor_frame(mt: MessageType | int, suffix: int, data: bytes) -> str | None:
    """Return a formatted monitor line for the given frame, or None to suppress."""
    if mt not in _MONITOR_TYPES:
        if isinstance(mt, MessageType):
            return None  # known but non-periodic (ack, raw data, config resp, …)
        # Truly unknown type — show raw so nothing is silently lost
        name = click.style(f"Unknown(0x{mt:02X})", fg="magenta")
        detail = data.hex()
        return f"[{datetime.now():%H:%M:%S}] 0x{suffix:04X}  {name:<20} {detail}"

    ts = f"[{datetime.now():%H:%M:%S}]"
    addr = f"0x{suffix:04X}"

    if mt == MessageType.DeviceHello:
        active = bool(data[1]) if len(data) >= 2 else False
        name = click.style(f"{'Hello':<14}", fg="green")
        detail = f"active={'yes' if active else 'no'}"

    elif mt == MessageType.DeviceMeterTicks:
        name = f"{'MeterTicks':<14}"
        if len(data) >= 5:
            (ticks,) = struct.unpack(">I", data[1:5])
            detail = f"ticks={ticks}"
        else:
            detail = f"short payload: {data.hex()}"

    elif mt == MessageType.DeviceCANStatus:
        if len(data) >= 3:
            tx_err, rx_err = data[1], data[2]
            has_errors = tx_err > 0 or rx_err > 0
            name = click.style(
                f"{'CANStatus':<14}", fg="yellow" if has_errors else None
            )
            detail = click.style(
                f"tx_err={tx_err}  rx_err={rx_err}",
                fg="yellow" if has_errors else None,
            )
        else:
            name = f"{'CANStatus':<14}"
            detail = f"short payload: {data.hex()}"

    elif mt == MessageType.DeviceReset:
        cause = data[1] if len(data) >= 2 else 0
        cause_str = RESET_CAUSES.get(cause, "unknown")
        name = click.style(f"{'Reset':<14}", fg="cyan")
        detail = click.style(f"cause=0x{cause:02X} ({cause_str})", fg="cyan")

    elif mt == MessageType.DeviceError:
        code = data[1] if len(data) >= 2 else 0
        desc = ERROR_CODES.get(code, "unknown error")
        name = click.style(f"{'Error':<14}", fg="red", bold=True)
        detail = click.style(f"0x{code:02X}: {desc}", fg="red", bold=True)

    elif mt == MessageType.DeviceWarning:
        code = data[1] if len(data) >= 2 else 0
        desc = WARNING_CODES.get(code, "unknown warning")
        name = click.style(f"{'Warning':<14}", fg="yellow")
        detail = click.style(f"0x{code:02X}: {desc}", fg="yellow")

    else:
        return None  # unreachable, but keeps mypy happy

    return f"{ts} {addr}  {name} {detail}"


@cli.command("monitor")
@click.option("--channel", default="can0", show_default=True)
@click.option(
    "--device",
    default=None,
    type=lambda x: int(x, 0),
    help="Filter to a specific device suffix (default: all devices)",
)
def monitor(channel: str, device: int | None) -> None:
    """Monitor the bus for periodic device messages. Ctrl-C to stop."""

    async def _run_monitor() -> None:
        async with MeterReaderBus(channel=channel) as bus:
            filt = f"0x{device:04X}" if device is not None else "all devices"
            click.echo(f"Monitoring {channel} ({filt}) — Ctrl-C to stop\n")
            async for mt, suffix, data in bus.receive():
                if device is not None and suffix != device:
                    continue
                line = _format_monitor_frame(mt, suffix, data)
                if line is not None:
                    click.echo(line)

    try:
        _run(_run_monitor())
    except KeyboardInterrupt:
        click.echo("\nStopped.")


def _print_axis_stats(
    mins: list[int], maxs: list[int], saturated: list[bool]
) -> None:
    click.echo(f"  {'Axis':<6} {'Min':>8} {'Max':>8} {'Range':>8}  Sat?")
    for i, name in AXIS_NAMES.items():
        sat_flag = "  *** SATURATED" if saturated[i] else ""
        click.echo(f"  {name:<6} {mins[i]:>8} {maxs[i]:>8} {maxs[i] - mins[i]:>8}{sat_flag}")


@cli.command("calibrate")
@click.option("--channel", default="can0", show_default=True)
@click.option("--device", required=True, type=lambda x: int(x, 0))
@click.option(
    "--duration",
    default=30.0,
    show_default=True,
    type=float,
    help="Seconds to collect data per sensitivity pass",
)
def calibrate(channel: str, device: int, duration: float) -> None:
    """Auto-calibrate by watching debug data to determine ideal sensor settings.

    Tries high sensitivity first, then falls back to low sensitivity if
    saturation or near-saturation is detected on any axis.  Reports recommended
    settings without applying them.
    """

    # Threshold below i16 max (32767) to flag "near saturation"
    NEAR_SAT = 30000

    async def _collect(
        bus: MeterReaderBus,
        dev: MeterReaderDevice,
        high_sensitivity: bool,
    ) -> tuple[list[int], list[int], list[bool], int]:
        """Enter debug mode, collect for duration, exit. Returns mins, maxs, saturated, count."""
        await dev.enter_debug_mode(high_sensitivity=high_sensitivity)
        mins = [32767, 32767, 32767]
        maxs = [-32768, -32768, -32768]
        saturated = [False, False, False]
        count = 0
        try:
            async with asyncio.timeout(duration):
                async for mt, data in bus.receive_for(dev.device_suffix):
                    if mt == MessageType.DeviceRawData and len(data) >= 7:
                        x, y, z = struct.unpack(">hhh", data[:6])
                        status = data[6]
                        for i, v in enumerate((x, y, z)):
                            mins[i] = min(mins[i], v)
                            maxs[i] = max(maxs[i], v)
                            if ((status >> i) & 1) or abs(v) >= NEAR_SAT:
                                saturated[i] = True
                        count += 1
        except TimeoutError:
            pass
        await dev.exit_debug_mode()
        return mins, maxs, saturated, count

    async def _run_cal() -> None:
        async with MeterReaderBus(channel=channel) as bus:
            dev = MeterReaderDevice(bus, device)
            try:
                # --- Phase 1: high sensitivity ---
                click.echo(f"Collecting {duration:.0f}s of data in high sensitivity mode ...")
                mins, maxs, sat, n = await _collect(bus, dev, high_sensitivity=True)
                click.echo(f"  {n} samples received")
                if n == 0:
                    click.echo("Error: no data received — is the device reachable?", err=True)
                    sys.exit(1)
                _print_axis_stats(mins, maxs, sat)
                high_sensitivity = True

                if any(sat):
                    sat_axes = [AXIS_NAMES[i] for i, s in enumerate(sat) if s]
                    click.echo(
                        f"  Saturation on {', '.join(sat_axes)} — retrying in low sensitivity mode"
                    )
                    # --- Phase 2: low sensitivity ---
                    click.echo(f"\nCollecting {duration:.0f}s of data in low sensitivity mode ...")
                    mins, maxs, sat, n = await _collect(bus, dev, high_sensitivity=False)
                    click.echo(f"  {n} samples received")
                    _print_axis_stats(mins, maxs, sat)
                    high_sensitivity = False
                else:
                    click.echo("  No saturation — using high sensitivity mode")

                # --- Pick best axis: largest peak-to-peak ---
                ranges = [maxs[i] - mins[i] for i in range(3)]
                best = int.__index__(ranges.index(max(ranges)))

                if ranges[best] < 50:
                    click.echo(
                        f"\nWarning: very small signal on all axes (max range {ranges[best]}). "
                        "Is the meter running? Results may be unreliable."
                    )

                rising_val = round((mins[best] + maxs[best]) / 2)
                rising_delta_raw = round(ranges[best] * 0.15)
                rising_delta = max(10, min(511, rising_delta_raw))
                delta_note = f"15% of {ranges[best]} p-p"
                if rising_delta != rising_delta_raw:
                    delta_note += f", clamped from {rising_delta_raw}"

                click.echo("\n--- Recommended configuration ---")
                click.echo(f"  axis:             {AXIS_NAMES[best]} ({best})")
                click.echo(f"  rising_val:       {rising_val}")
                click.echo(f"  rising_delta:     {rising_delta}  ({delta_note})")
                click.echo(f"  high_sensitivity: {high_sensitivity}")
                hs_flag = " --high-sensitivity" if high_sensitivity else ""
                click.echo(
                    f"\nTo apply:\n"
                    f"  meter-reader-cli set-config --device 0x{device:04X}"
                    f" --rising-val {rising_val}"
                    f" --rising-delta {rising_delta}"
                    f" --axis {best}"
                    f"{hs_flag}"
                )
            except (CommandError, TimeoutError) as e:
                click.echo(f"Error: {e}", err=True)
                sys.exit(1)

    _run(_run_cal())


@cli.command("set-ticks")
@click.option("--channel", default="can0", show_default=True)
@click.option("--device", required=True, type=lambda x: int(x, 0))
@click.option("--ticks", required=True, type=int, help="New tick counter value")
def set_ticks(channel: str, device: int, ticks: int) -> None:
    """Set the meter tick counter."""

    async def _run_set() -> None:
        async with MeterReaderBus(channel=channel) as bus:
            dev = MeterReaderDevice(bus, device)
            try:
                await dev.set_start_point(ticks)
                click.echo(f"0x{device:04X}: ticks set to {ticks}")
            except (CommandError, TimeoutError) as e:
                click.echo(f"Error: {e}", err=True)
                sys.exit(1)

    _run(_run_set())


@cli.command("reset")
@click.option("--channel", default="can0", show_default=True)
@click.option("--device", required=True, type=lambda x: int(x, 0))
def reset(channel: str, device: int) -> None:
    """Send a reset command to the device."""

    async def _run_reset() -> None:
        async with MeterReaderBus(channel=channel) as bus:
            dev = MeterReaderDevice(bus, device)
            try:
                await dev.reset()
                click.echo(f"0x{device:04X}: reset command sent")
            except (CommandError, TimeoutError) as e:
                click.echo(f"Error: {e}", err=True)
                sys.exit(1)

    _run(_run_reset())


