#![no_main]
#![no_std]

mod mlx;

use core::panic::PanicInfo;
use core::sync::atomic::{AtomicBool, Ordering, compiler_fence};

use embassy_mspm0::can::frame::MCanFrame;
use embassy_mspm0::gpio::Input;
use embassy_mspm0::mode::Async;
use embassy_mspm0::gpio::Output;

use embassy_mspm0::{bind_interrupts, can};
use embassy_mspm0::i2c::{self};
use embassy_mspm0::peripherals::{I2C1, CANFD0};

use embassy_mspm0::uart::UartTx;

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embedded_hal_async::i2c::{I2c, ErrorKind, Error as I2cError};
use embedded_io_async::Write;

use embassy_sync::channel::{Channel, DynamicReceiver, DynamicSender};
use embassy_time::{Duration, Timer};

use embassy_executor::Spawner;

use embedded_storage_async::nor_flash::{NorFlash, ReadNorFlash};
use mlx::Mlx90394;

use embedded_can::{Frame, Id, StandardId};

use defmt::{info, warn};

mod flashdriver;

use defmt_rtt as _;

use panic_probe as _;
use sequential_storage::cache::NoCache;
use sequential_storage::map::{MapConfig, MapStorage};

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

// This panic handler dumps stuff to UART.
// Note this is not particularly code-size efficient as it brings in a lot of formatting garbage just for panics (>> 10KB).
// This is fine for development where panics are possible, but should really be disabled in production releases.
/*#[panic_handler]
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
    }
}*/

/// Hardfault handler.
///
/// Terminates the application and makes a semihosting-capable debug tool exit
/// with an error. This seems better than the default, which is to spin in a
/// loop.
#[cortex_m_rt::exception]
unsafe fn HardFault(_frame: &cortex_m_rt::ExceptionFrame) -> ! {
    panic!("Got a HardFault");
}
/*
pub async fn i2c_scan<I>(i2c: &mut I) -> ()
where
    I: I2c,
    // We require the Error type to implement the embedded_hal Error trait
    // so we can distinguish between NACK (no device) and actual Bus Errors.
    I::Error: I2cError, 
{
    defmt::info!("Starting I2C Scan...");
    
    for addr in 0x01..0x7F {
        let mut tmp = [0u8; 1];
        match i2c.read(addr, &mut tmp).await {
            Ok(_) => {
                defmt::info!("Found device at address: {=u8:#04x}", addr);
            }
            Err(e) if matches!(e.kind(), ErrorKind::NoAcknowledge(_)) => {
                defmt::warn!("Error probing address {=u8:#04x}: bus error", addr);
            },
            Err(_) => {
                defmt::warn!("Other error probing address {=u8:#04x}", addr);
            }
        }
    }

    defmt::info!("I2C Scan complete.");
}
     */
static OUTBOUND: Channel<ThreadModeRawMutex, MCanFrame, 10> = Channel::new();

#[embassy_executor::task]
async fn periodic_hello(outgoing: DynamicSender<'static, MCanFrame>) {
    loop {
        Timer::after_secs(5).await;

        for number in 1u8..1 {
            let frame =
                can::frame::MCanFrame::new(Id::Standard(StandardId::new(0x123).unwrap()), &[0x12u8, 0x34, number])
                    .unwrap();
            outgoing.send(frame).await;
            info!("Sent hello frame!");
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
async fn can_maintainer(mut status: can::CanStatus<'static>) {
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
        }
    }
}



#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let periph = embassy_mspm0::init(Default::default());
    let mut led_output = Output::new(periph.PA25, embassy_mspm0::gpio::Level::Low);
    let mut led_output_2 = Output::new(periph.PA10, embassy_mspm0::gpio::Level::High);

    let canstb = Output::new(periph.PA0, embassy_mspm0::gpio::Level::Low);

    // TRM says that TRACEID is unique per part shipped,
    // though https://e2e.ti.com/support/microcontrollers/arm-based-microcontrollers-group/arm-based-microcontrollers/f/arm-based-microcontrollers-forum/1302805/lp-mspm0l1306-identifying-a-unique-id-for-each-part
    // casts some doubt on that. I suspect traceid is like a serial number but only within the actual unique dies, so would need to be qualified by deviceid/userid in more hardened situations
    let traceid: u32 = unsafe{ core::ptr::read_volatile((0x41C4_0000) as *const u32) };

    info!("traceid: {:08x}", traceid);

    let mut s2 = Input::new(periph.PA24, embassy_mspm0::gpio::Pull::Up);
    
    let tx = periph.PA21;

    let mut config = embassy_mspm0::uart::Config::default();
    config.baudrate = 115200;

    //let mut uart = Uart::new_blocking(periph.UART0, rx, tx, config).unwrap();
    let mut uart = UartTx::new_blocking(periph.UART2, tx, config).unwrap();

    uart.blocking_write("done\r\n".as_bytes()).unwrap();
    info!("Init completed");
    embassy_time::Timer::after(Duration::from_millis(100)).await;  

    //let mut i2cinter = i2c::I2c::new_async(periph.I2C1, periph.PA4, periph.PA3, Irqs, i2c::Config::default()).unwrap();
    let mut cfg = i2c::Config::default();
    cfg.bus_speed = i2c::BusSpeed::FastMode;
    let i2cinter = i2c::I2c::new_async(periph.I2C1, periph.PA4, periph.PA3, Irqs, cfg).unwrap();

    info!("was OK");
    
    //i2c_scan(&mut i2cinter).await;

    let controller = flashdriver::FlashController::new(flashdriver::take().unwrap(), 0x0001_E000, 0x0002_0000);



    // I think we have all (most) of the primitives we need.
    // Now to tie it together.
    // 
    
    //controller.erase(0x0001E000, 0x00020000).await.unwrap();

    let mut storage = MapStorage::<u8, _, _>::new(controller, const { MapConfig::new(0x0001E000..0x00020000) }, NoCache::new());

    let mut tmpbuf = [0x0u8; 80];
    let result = storage.fetch_item::<[u8; 3]>(&mut tmpbuf, &0x01).await;
    match result {
        Ok(Some(v)) => {
            defmt::info!("Got value: {:02x}", v);
        }
        Ok(None) => {
            defmt::warn!("No value found!");
        }
        Err(e) => {
            defmt::error!("Failed to fetch item: {}", e);
        }
    }

    storage.store_item::<[u8; 3]>(&mut tmpbuf, &0x01, &[0x05, 0x10, 0xAB]).await.unwrap();

    let candriver = can::Can::new_async(periph.CANFD0, periph.PA27, periph.PA26, Irqs, can::Config::default()).unwrap();
    let (tx, rx, status) = candriver.split();

    spawner.spawn(receiver(rx, OUTBOUND.dyn_sender()).unwrap());
    spawner.spawn(transmitter_mux(tx, OUTBOUND.dyn_receiver()).unwrap());
    spawner.spawn(periodic_hello(OUTBOUND.dyn_sender()).unwrap());
    spawner.spawn(can_maintainer(status).unwrap());

    let mut outgoing = OUTBOUND.dyn_sender();

    // or 0x10
    let mut mlx = Mlx90394::new_init(i2cinter, 0x10).await.unwrap();

    loop {
        /*if s2.is_low() {
            led_output_2.set_high();
        } else {
            led_output_2.set_low();
        }*/

        s2.wait_for_falling_edge().await;

        //if mlx.data_ready().await.unwrap() {
            led_output.toggle();
            match mlx.get_reading().await {
                Ok(data) => {
                    defmt::info!("Got readings: {:?}", data);
                    //let magdata = data.serialize();
                    //let frame =
                    //can::frame::MCanFrame::new(Id::Standard(StandardId::new(0x129).unwrap()), &magdata)
                    //    .unwrap();
                    //outgoing.send(frame).await;
                }
                Err(e) => {
                    defmt::warn!("failed to read: {}", e);
                } 
            }
            


            
        //}

    } 
}
