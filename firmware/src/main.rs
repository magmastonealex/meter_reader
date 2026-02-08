#![no_main]
#![no_std]

mod mlx;

use core::fmt;
use core::panic::PanicInfo;
use core::sync::atomic::{AtomicBool, Ordering, compiler_fence};

use embassy_futures::select::{Either, select};
use embassy_mspm0::can::frame::MCanFrame;
use embassy_mspm0::gpio::Input;
use embassy_mspm0::mode::Async;
use embassy_mspm0::gpio::Output;
use embassy_mspm0::pac::sysctl::vals::ResetcmdKey;
use embassy_sync::lazy_lock::LazyLock;

use embassy_mspm0::wwdt::{self, Watchdog};
use embassy_mspm0::{bind_interrupts, can};
use embassy_mspm0::i2c::{self, I2c as PeriphI2c};
use embassy_mspm0::peripherals::{I2C1, CANFD0};

use embassy_mspm0::uart::UartTx;

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embedded_hal_async::i2c::I2c;
use embedded_io_async::Write;

use embassy_sync::channel::{Channel, DynamicReceiver, DynamicSender};
use embassy_time::{Duration, Instant, Timer};

use embassy_executor::Spawner;

use embedded_storage_async::nor_flash::{NorFlash, ReadNorFlash};
use mlx::Mlx90394;

use embedded_can::{ExtendedId, Frame, Id, StandardId};

use defmt::{error, info, warn};

mod flashdriver;
mod configutils;

use defmt_rtt as _;

mod pulse_counter;

//use panic_probe as _;
use sequential_storage::cache::NoCache;
use sequential_storage::map::{MapConfig, MapStorage};

use crate::configutils::MagnetometerConfig;
use crate::mlx::MlxError;
use crate::pulse_counter::PulseCounter;

bind_interrupts!(struct Irqs {
    I2C1 => i2c::InterruptHandler<I2C1>;
    CANFD0 => can::InterruptHandler<CANFD0>;
});


fn uart_ll_write(buffer: &[u8]) {
    for &b in buffer {
        while ! embassy_mspm0::pac::UART2.stat().read().txfe() {}
        compiler_fence(Ordering::Release);
        embassy_mspm0::pac::UART2.txdata().write(|w| {
            w.set_data(b);
        });
    }

}

#[allow(dead_code)]
#[derive(Debug, defmt::Format, PartialEq, Eq)]
#[repr(u8)]
enum MessageType {
    /// A notification that a device reset for some reason.
    DeviceReset = 0x0F,

    /// The device has encountered a fatal error.
    DeviceError = 0x00,

    /// The device has taken your request into account.
    DeviceAck = 0x01,
    /// The device wasn't able to apply a config or debug mode that was requested.
    DeviceWarning = 0x02,
    
    /// A notification that a device is still alive.
    DeviceHello = 0x10,

    /// Sensor reading from this device?
    DeviceRawData = 0x11,
    
    /// Sensor reading from this device?
    DeviceMeterTicks = 0x12,

    /// If the device has encountered a pulse (indicating some flow) in the past minute.
    DeviceMeterActive = 0x13,

    /// Request device enter debug mode to dump data at 5hz.
    DeviceDebugMode = 0x14,

    DeviceReconfigure = 0x15,
    DeviceGetCurrentConfig = 0x18,
    DeviceGetCurrentConfigResp=0x19,

    DeviceSetStartPoint = 0x16,
    DevicePleaseReset = 0x17,
    DeviceCANStatus = 0x1a,
}

impl TryFrom<u8> for MessageType {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        info!("checking against: {:02x}", value);
        match value {
            0x0F => Ok(MessageType::DeviceReset),
            0x00 => Ok(MessageType::DeviceError),
            0x10 => Ok(MessageType::DeviceHello),
            0x11 => Ok(MessageType::DeviceRawData),
            0x12 => Ok(MessageType::DeviceMeterTicks),
            0x14 => Ok(MessageType::DeviceDebugMode),
            0x1a => Ok(MessageType::DeviceCANStatus),
            0x15 => Ok(MessageType::DeviceReconfigure),
            0x16 => Ok(MessageType::DeviceSetStartPoint),
            0x17 => Ok(MessageType::DevicePleaseReset),
            0x18 => Ok(MessageType::DeviceGetCurrentConfig),
            0x19 => Ok(MessageType::DeviceGetCurrentConfigResp),
            _ => Err(())
        }
    }
}

static DEVICE_ADDR_SUFFIX: LazyLock<u32> = LazyLock::new(|| {
    // addr prefix is the traceid masked to a 21 bit value, leaving the upper 8 bits of a CAN address free.
    // This allows arbitration to work based on message type, but also ensures the values are unique.
    let traceid: u32 = unsafe{ core::ptr::read_volatile((0x41C4_0000) as *const u32) };

    // TRM says that TRACEID is unique per part shipped,
    // though https://e2e.ti.com/support/microcontrollers/arm-based-microcontrollers-group/arm-based-microcontrollers/f/arm-based-microcontrollers-forum/1302805/lp-mspm0l1306-identifying-a-unique-id-for-each-part
    // casts some doubt on that. I suspect traceid is like a serial number but only within the actual unique dies, so would need to be qualified by deviceid/userid in more hardened situations
    info!("traceid: {:08x}", traceid);
    traceid & (0xFFFF)
});

fn get_can_id(device_suffix: u32, message_type: MessageType) -> ExtendedId {
    let typ: u8 = message_type as u8;
    //0017edf1
    //1FFFFFFF
    let extid = (device_suffix & 0xFFFF) | ((typ as u32) << 16);

    ExtendedId::new(extid).unwrap()
}

fn is_for_device(frame: &MCanFrame) -> bool {
    if let Id::Extended(e) = frame.id() {
        (e.as_raw() & 0xFFFF) == *DEVICE_ADDR_SUFFIX.get()
    } else {
        false
    }
}

fn get_message_type(frame: &MCanFrame) -> Option<MessageType> {
    if let Id::Extended(e) = frame.id() {
        info!("frame id for type: {:08x}", e.as_raw());
        match MessageType::try_from((e.as_raw() >> 16) as u8) {
            Ok(typ) => {
                Some(typ)
            },
            Err(_) => {
                None
            }
        }

    } else {
        None
    }
}

// This panic handler dumps stuff to UART.
// Note this is not particularly code-size efficient as it brings in a lot of formatting garbage just for panics (>> 10KB).
// This is fine for development where panics are possible, but should really be disabled in production releases.
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    static PANICKED: AtomicBool = AtomicBool::new(false);

    cortex_m::interrupt::disable();
    
    // Guard against infinite recursion, just in case.
    if !PANICKED.load(Ordering::Relaxed) {
        PANICKED.store(true, Ordering::Relaxed);

        let msg = info.message().as_str().unwrap_or("No panic message!");

        uart_ll_write("******Panicked!*********\r\n".as_bytes());
        uart_ll_write("Message: ".as_bytes());
        uart_ll_write(msg.as_bytes());
        uart_ll_write("\r\nLocation: ".as_bytes());
        if let Some(l) = info.location() {
            uart_ll_write("file: ".as_bytes());
            uart_ll_write(l.file().as_bytes());
        } else {
            uart_ll_write("No location info.\r\n".as_bytes());
        }

    }

    loop {
        // hope you remembered to set a watchdog.
        embassy_mspm0::pac::SYSCTL.resetlevel().write(|w| {
            w.set_level(embassy_mspm0::pac::sysctl::vals::ResetlevelLevel::POR);
        });
        embassy_mspm0::pac::SYSCTL.resetcmd().write(|w| {
            w.set_key(ResetcmdKey::KEY);
            w.set_go(true);
        });
    }
}

/// Hardfault handler.
///
/// Terminates the application and makes a semihosting-capable debug tool exit
/// with an error. This seems better than the default, which is to spin in a
/// loop.
#[cortex_m_rt::exception]
unsafe fn HardFault(_frame: &cortex_m_rt::ExceptionFrame) -> ! {
    panic!("Got a HardFault");
}

static OUTBOUND: Channel<ThreadModeRawMutex, MCanFrame, 10> = Channel::new();

static IS_METER_ACTIVE: AtomicBool = AtomicBool::new(false);

#[embassy_executor::task]
async fn periodic_hello(outgoing: DynamicSender<'static, MCanFrame>) {
    let id = get_can_id(*DEVICE_ADDR_SUFFIX.get(), MessageType::DeviceHello);
    loop {
        let mut data = [0x00u8; 2];
        if IS_METER_ACTIVE.load(Ordering::Relaxed) {
            data[1] = 0x01;
        }
        Timer::after_secs(5).await;
        let frame = MCanFrame::new(Id::Extended(id), &data).unwrap();
        match outgoing.try_send(frame) {
            Ok(_) => {},
            Err(_) => {
                warn!("failed to send hello - queue full?");
            }
        }
    }
}

#[embassy_executor::task]
async fn transmitter_mux(mut tx: can::CanTx<Async>, incoming: DynamicReceiver<'static, MCanFrame>) {
    loop {
        let frame = incoming.receive().await;
        tx.enqueue_frame(&frame).await.expect("no buserror possible");
    }
}

#[embassy_executor::task]
async fn receiver(mut rx: can::CanRx<Async>, outgoing: DynamicSender<'static, MCanFrame>) {
    loop {
        let mut frame = rx.get_frame().await.expect("no buserror possible right now");
        info!("Received frame: {}", frame);
        frame.set_id(Id::Standard(StandardId::new(0x0ab).unwrap()));
        info!("Sending reply... {}", frame);
        outgoing.send(frame).await;    
    }
}

#[embassy_executor::task]
async fn can_maintainer(mut status: can::CanStatus<'static>, outgoing: DynamicSender<'static, MCanFrame>) {
    let mut interval = 0;
    loop {
        Timer::after_secs(3).await;

        let errors = status.get_error_counters();

        info!("CAN error counters: {:?}", errors);

        if errors.bus_off {
            info!("Starting bus-off recovery");
            match status.recover() {
                Ok(_) => {
                    info!("Bus-off recovery completed.");
                }
                Err(e) => {
                    warn!("Bus-off recovery failed: {}", e);
                }
            }
        } else {
            interval += 1;
            if interval > 5 {
                interval = 0;
                try_send_canstats(errors.tec, errors.rec, &outgoing);
            }
        }
    }
}


async fn send_error_and_panic(reason: u8, outgoing: &DynamicSender<'_, MCanFrame>) -> ! {
    let frame = MCanFrame::new(Id::Extended(get_can_id(*DEVICE_ADDR_SUFFIX.get(), MessageType::DeviceError)), &[0x00, reason]).unwrap();
    match outgoing.try_send(frame) {
        Ok(_) => {},
        Err(_) => {
            panic!("init panic: {} - failed send", reason);
        }
    }
    Timer::after_millis(100).await;
    panic!("init panic: {}", reason);
}

fn try_send_warning(reason: u8, outgoing: &DynamicSender<'_, MCanFrame>) {
    let frame = MCanFrame::new(Id::Extended(get_can_id(*DEVICE_ADDR_SUFFIX.get(), MessageType::DeviceWarning)), &[0x00, reason]).unwrap();
    match outgoing.try_send(frame) {
        Ok(_) => {},
        Err(_) => {
            warn!("failed to send warning");
        }
    }
}

fn try_send_ack(reason: u8, outgoing: &DynamicSender<'_, MCanFrame>) {
    let frame = MCanFrame::new(Id::Extended(get_can_id(*DEVICE_ADDR_SUFFIX.get(), MessageType::DeviceAck)), &[0x00, reason]).unwrap();
    match outgoing.try_send(frame) {
        Ok(_) => {},
        Err(_) => {
            warn!("failed to send ack");
        }
    }
}

fn try_send_count(ticks: u32, outgoing: &DynamicSender<'_, MCanFrame>) {
    let mut dataframe = [0x00u8; 5];

    dataframe[1..5].copy_from_slice(&ticks.to_be_bytes());

    let frame = MCanFrame::new(Id::Extended(get_can_id(*DEVICE_ADDR_SUFFIX.get(), MessageType::DeviceMeterTicks)), &dataframe).unwrap();
    match outgoing.try_send(frame) {
        Ok(_) => {},
        Err(_) => {
            warn!("failed to send count");
        }
    }
}

fn try_send_canstats(tec: u8, rec: u8, outgoing: &DynamicSender<'_, MCanFrame>) {
    let mut dataframe = [0x00u8; 3];

    dataframe[1] = tec;
    dataframe[2] = rec;

    let frame = MCanFrame::new(Id::Extended(get_can_id(*DEVICE_ADDR_SUFFIX.get(), MessageType::DeviceCANStatus)), &dataframe).unwrap();
    match outgoing.try_send(frame) {
        Ok(_) => {},
        Err(_) => {
            warn!("failed to send count");
        }
    }
}

async fn wait_for_drdy(mag: &mut Mlx90394<'_, PeriphI2c<'_, Async>>) {
    loop {
        match mag.data_ready().await {
            Ok(v) => {
                if v {
                    return;
                }
            },
            Err(e) => {
                warn!("Got MLX error: {}", e);
                Timer::after_millis(100).await;
            }
        }

    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let periph = embassy_mspm0::init(Default::default());
    let mut led_output = Output::new(periph.PA25, embassy_mspm0::gpio::Level::Low);
    let mut led_output_2 = Output::new(periph.PA10, embassy_mspm0::gpio::Level::High);
    let mut mlx_drdy = Input::new(periph.PA24, embassy_mspm0::gpio::Pull::Up);
    let canstb = Output::new(periph.PA0, embassy_mspm0::gpio::Level::Low);

    
    // Configure UART first - we will use it in our panic handler.
    let mut config = embassy_mspm0::uart::Config::default();
    config.baudrate = 115200;
    let mut uart = UartTx::new_blocking(periph.UART2, periph.PA21, config).unwrap();

    uart.blocking_write("hello\r\n".as_bytes()).unwrap();


    let candriver = can::Can::new_async(periph.CANFD0, periph.PA27, periph.PA26, Irqs, can::Config::default()).unwrap();
    let (tx, mut rx, status) = candriver.split();

    spawner.spawn(transmitter_mux(tx, OUTBOUND.dyn_receiver()).unwrap());
    spawner.spawn(periodic_hello(OUTBOUND.dyn_sender()).unwrap());
    spawner.spawn(can_maintainer(status, OUTBOUND.dyn_sender()).unwrap());

    let outgoing = OUTBOUND.dyn_sender();

    // Send our "reset hello" indicating that we reset and why.
    let rstcause_raw = (embassy_mspm0::pac::SYSCTL.rstcause().read().0 & 0x0F) as u8;
    let frame = MCanFrame::new(Id::Extended(get_can_id(*DEVICE_ADDR_SUFFIX.get(), MessageType::DeviceReset)), &[0x00, rstcause_raw]).unwrap();
    outgoing.send(frame).await;

    // Now that we have sent our reset cause,
    // we can enable resetting upon a variety of non-recoverable faults...
    embassy_mspm0::pac::SYSCTL.systemcfg().write(|w| {
        w.set_flasheccrstdis(false);
        w.set_wwdtlp0rstdis(false);
        w.set_wwdtlp1rstdis(false);
    });

    // Enable our watchdog before we start initializing "risky" peripherals.
    let mut watchdog_cfg = wwdt::Config::default();
    watchdog_cfg.closed_window = wwdt::ClosedWindowPercentage::Zero;
    watchdog_cfg.timeout = wwdt::Timeout::Sec4;
    let mut watchdog = Watchdog::new(periph.WWDT0, watchdog_cfg);
    watchdog.pet();

    // Initialize i2c.
    let mut cfg = i2c::Config::default();
    cfg.bus_speed = i2c::BusSpeed::FastMode;
    let mut i2cinter = match i2c::I2c::new_async(periph.I2C1, periph.PA4, periph.PA3, Irqs, cfg) {
        Ok(drv) => drv,
        Err(e) => {
            error!("i2c config error: {}", e);

           send_error_and_panic(0x01, &outgoing).await;
        }
    };

    let controller = flashdriver::FlashController::new(flashdriver::take().unwrap(), 0x0001_E000, 0x0002_0000);
    let mut storage: MapStorage<u8, flashdriver::FlashController, NoCache> = MapStorage::<u8, _, _>::new(controller, const { MapConfig::new(0x0001E000..0x00020000) }, NoCache::new());

    //storage.store_item::<[u8; 3]>(&mut tmpbuf, &0x01, &[0x05, 0x10, 0xAB]).await.unwrap();
    let meter_ticks: u32 = configutils::get_value_or(&mut storage, configutils::ConfigKey::MeterTicks, 0u32).await;
    info!("Loaded meter ticks: {}", meter_ticks);
    let mut mag_config: MagnetometerConfig = configutils::get_bitfield_or(&mut storage, configutils::ConfigKey::MagnetometerConfig, MagnetometerConfig::default()).await;

    let mut pulse_counter = PulseCounter::new(mag_config.rising_val(), mag_config.rising_delta(), meter_ticks);

    try_send_count(pulse_counter.count(), &outgoing);

    let starting_config: mlx::Config = match (&mag_config).try_into() {
        Ok(cfg) => cfg,
        Err(_) => {
            error!("Config in flash is bad for mlx (flash corrupted!?)");
            send_error_and_panic(0x02, &outgoing).await;
        }
    };

    info!("Got starting MLX config: {}", mag_config);

    let mut attempt = 0;
    let mut mag = loop {
        let addr = if attempt % 2 == 0 {0x10} else {0x60};

        match select(
            Mlx90394::new_init(&mut i2cinter, addr, &starting_config),
            Timer::after_millis(250)
        ).await {
            Either::First(Ok(mlx)) => {
                break mlx
            }
            Either::First(Err(_)) => {
                attempt += 1;
                if attempt > 10 {
                    error!("Failed to initialize after multiple attempts - giving up");
                    send_error_and_panic(0x03, &outgoing).await;
                }
                info!("Failed. Trying again... {}", addr);
                watchdog.pet();
                Timer::after_millis(100).await;
            }
            Either::Second(_) => {
                error!("Detected bus latchup - not handling this right now.");
                uart.blocking_write("latchup\r\n".as_bytes()).unwrap();
                send_error_and_panic(0x04, &outgoing).await;
            }
        }
    };
    info!("Initialized MLX");
    watchdog.pet();

    // "debug mode" is triggered through a received CAN frame
    // and causes the device to report the values of every reading,
    // used for initial calibration and other debugging.
    let mut is_debug_mode = false;

    let mut last_pulse = Instant::now();
    let mut last_update = Instant::now();

    loop {
        //if mlx.data_ready().await.unwrap() {
            match select(rx.get_frame(), mlx_drdy.wait_for_low()).await {
                Either::First(Ok(frame)) if is_for_device(&frame) => {
                    match get_message_type(&frame) {
                        Some(MessageType::DeviceDebugMode) => {
                            if frame.data().len() >= 1 && frame.data()[0] == 0x01 {
                                is_debug_mode = true;
                                // reconfigure the magnetometer for debug mode - all axes, sensitivity depends on message, 5 hz.
                                let mut cfg = const {mlx::Config::new(0x02).unwrap()}; // 5 hz;
                                cfg.en_x = true;
                                cfg.en_y = true;
                                cfg.en_z = true;

                                if frame.data().len() > 1 && frame.data()[1] == 0x01 {
                                    cfg.high_sensitivity = true;
                                } else {
                                    cfg.high_sensitivity = false;
                                }
                                
                                if let Err(e) = mag.reconfigure(&cfg).await {
                                    warn!("Failed to reconfigure MLX... {}", e);
                                    try_send_warning(0x01, &outgoing);
                                } else {
                                    try_send_ack(MessageType::DeviceDebugMode as u8, &outgoing);
                                }
                            } else {
                                is_debug_mode = false;
                                match (&mag_config).try_into() {
                                    Ok(cfg) => {
                                        if let Err(e) = mag.reconfigure(&cfg).await {
                                            warn!("Failed to reconfigure MLX... {}", e);
                                            try_send_warning(0x10, &outgoing);
                                        } else {
                                            try_send_ack(MessageType::DeviceDebugMode as u8, &outgoing);
                                        }
                                    },
                                    Err(_) => {
                                        warn!("invalid mlx config - this is super bad!");
                                        try_send_warning(0x11, &outgoing);
                                    }
                                }
                            }
                        },
                        Some(MessageType::DeviceSetStartPoint) => {
                            if frame.data().len() == 4 {
                                let meter_ticks = u32::from_be_bytes(frame.data().try_into().unwrap());

                                pulse_counter.set_count(meter_ticks);

                                if let Err(e) = configutils::save_value(&mut storage, configutils::ConfigKey::MeterTicks, meter_ticks).await {
                                    try_send_warning(0x02, &outgoing);
                                    info!("Failed saving meter ticks: {}", e);
                                } else {
                                    try_send_ack(MessageType::DeviceSetStartPoint as u8, &outgoing);
                                    info!("Saved meter ticks: {}", meter_ticks);
                                }
                            } else {
                                try_send_warning(0x03, &outgoing);
                            }
                        },
                        Some(MessageType::DeviceReconfigure) => {
                            if frame.data().len() == 4 {
                                let tmp_config = u32::from_be_bytes(frame.data().try_into().unwrap());
                                
                                mag_config = MagnetometerConfig::from(tmp_config);
                                // ignore whatever rate was set, we'll always use 100Hz for real measurements.
                                mag_config.set_rate(6);

                                if let Err(e) = configutils::save_bitfield(&mut storage, configutils::ConfigKey::MagnetometerConfig, mag_config.clone()).await {
                                    try_send_warning(0x04, &outgoing);
                                    info!("Failed saving mag config: {}", e);
                                } else {
                                    try_send_ack(MessageType::DeviceReconfigure as u8, &outgoing);
                                    info!("Saved new config: {}", mag_config);
                                }
                            } else {
                                info!("Bad length for device reconfigure!");
                                try_send_warning(0x05, &outgoing);
                            }
                        }
                        Some(MessageType::DevicePleaseReset) => {
                            if frame.data().len() == 4 {
                                let data = frame.data();
                                if data[0] == 0xBA && data[1] == 0xAD && data[2] == 0xF0 && data[3] == 0x0D {
                                    try_send_ack(MessageType::DevicePleaseReset as u8, &outgoing);

                                    // give the ack a moment to get to the bus.
                                    Timer::after_millis(100).await;

                                    // reset the CPU. See you next time!
                                    embassy_mspm0::pac::SYSCTL.resetlevel().write(|w| {
                                        w.set_level(embassy_mspm0::pac::sysctl::vals::ResetlevelLevel::POR);
                                    });
                                    embassy_mspm0::pac::SYSCTL.resetcmd().write(|w| {
                                        w.set_key(ResetcmdKey::KEY);
                                        w.set_go(true);
                                    });
                                }
                            } else {
                                info!("Bad length for reset!");
                                try_send_warning(0xFE, &outgoing);
                            }
                        },
                        Some(MessageType::DeviceGetCurrentConfig) => {
                            let cfg = configutils::get_bitfield_or(&mut storage, configutils::ConfigKey::MagnetometerConfig, MagnetometerConfig::default()).await;
                            let mut dataframe = [0x00u8; 5];

                            dataframe[1..5].copy_from_slice(&cfg.0.to_be_bytes());

                            let frame = MCanFrame::new(Id::Extended(get_can_id(*DEVICE_ADDR_SUFFIX.get(), MessageType::DeviceGetCurrentConfigResp)), &dataframe).unwrap();
                            match outgoing.try_send(frame) {
                                Ok(_) => {},
                                Err(_) => {
                                    warn!("failed to send cfg");
                                    try_send_warning(0x80, &outgoing);
                                }
                            }
                        }
                        _ => {
                            info!("Ignoring frame of unknown / unhandled type")
                        }
                    }
                },
                Either::First(Ok(f)) => {
                    // not for us, ignore.
                    info!("Frame not for us: {}", f);
                }
                Either::First(Err(_)) => {
                    // ignore, won't ever happen
                }
                Either::Second(_) => {
                    // data is ready for reading from magnetometer
                    match mag.get_reading().await {
                        Ok(data) => {
                            
                            led_output.toggle();

                            let magdata = data.serialize();

                            if is_debug_mode {
                                info!("Got readings: {:?}", data);
                                // safety: data frame is 7 bytes, will always fit.
                                let frame = MCanFrame::new(Id::Extended(get_can_id(*DEVICE_ADDR_SUFFIX.get(), MessageType::DeviceRawData)), &magdata).unwrap();
                                match outgoing.try_send(frame) {
                                    Ok(_) => {},
                                    Err(_) => {info!("Failed to enqueue data frame - buffer full?")}
                                }
                            }

                            if data.overflowed {
                                warn!("Missed reading - loop too slow!");
                                try_send_warning(0xCC, &outgoing);
                            }

                            let (reading, saturated) = match mag_config.axis() {
                                0 => {
                                    (data.x, data.x_sat)
                                },
                                1 => {
                                    (data.y, data.y_sat)
                                },
                                _ => {
                                    (data.z, data.z_sat)
                                }
                            };

                            if saturated {
                                warn!("Saturated sensor - try using lower range!");
                                try_send_warning(0xAB, &outgoing);
                            } else {
                                match pulse_counter.update(reading) {
                                    Some(new_count) => {
                                        if new_count % 10 == 0 {
                                            // Save to flash every 1000 counts so we don't lose too much progress.
                                            if let Err(e) = configutils::save_value(&mut storage, configutils::ConfigKey::MeterTicks, new_count).await {
                                                try_send_warning(0x02, &outgoing);
                                                info!("Failed saving meter ticks: {}", e);
                                            }
                                        }
                                        if new_count % 2 == 0 {
                                            // publish to CAN every ~64 counts to avoid the bus being too noisy for high-flow.
                                            try_send_count(new_count, &outgoing);
                                        }
                                        last_pulse = Instant::now();
                                        // this doesn't really need to be atomic, but it makes the borrow checker happy.
                                        IS_METER_ACTIVE.store(true, Ordering::Relaxed);
                                        led_output_2.toggle();
                                    },
                                    None => {
                                        if last_pulse.elapsed() >= Duration::from_secs(20) {
                                            // this doesn't really need to be atomic, but it makes the borrow checker happy.
                                            if IS_METER_ACTIVE.load(Ordering::Relaxed) {
                                                IS_METER_ACTIVE.store(false, Ordering::Relaxed);
                                            }
                                        }
                                        if last_update.elapsed() >= Duration::from_secs(30) {
                                            try_send_count(pulse_counter.count(), &outgoing);
                                            last_update = Instant::now();
                                        }
                                    }
                                }
                            }

                        }
                        Err(e) => {
                            defmt::warn!("failed to read: {}", e);
                        } 
                    }
                }
            };
            
            // drdy should happen at 5hz minimum, so this should never fail!
            watchdog.pet();

    } 
}
