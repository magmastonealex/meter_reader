#![no_main]
#![no_std]

use core::panic::PanicInfo;
use core::ptr;
use core::sync::atomic::{AtomicBool, Ordering, compiler_fence};

use arbitrary_int::{u2, u4, u5, u7, u11, u29};
use cortex_m::{asm::nop};
use derive_mmio::Mmio;
use embassy_mspm0::gpio::Input;
use embassy_mspm0::interrupt::{self, InterruptExt};
use embassy_mspm0::interrupt::typelevel::Interrupt;
use embassy_mspm0::pac::canfd::{Canfd, vals as CanVals};
use embassy_mspm0::pac::sysctl::regs::{Syspllparam0, Syspllparam1};
use embassy_mspm0::{Config, gpio::Output};

use embassy_mspm0::{Peri, bind_interrupts, can};
use embassy_mspm0::i2c::{self};
use embassy_mspm0::peripherals::I2C1;

use embassy_mspm0::uart::{self, UartTx};
use embassy_mspm0::peripherals::UART0;

use embedded_hal_async::i2c::{I2c, ErrorKind, Error as I2cError};
use embedded_io_async::Write;

use embassy_executor::Spawner;

use embassy_time::{Duration};

use embassy_mspm0::pac::{IOMUX, SYSCTL};
use bitbybit::{bitenum, bitfield};


use defmt::{info, warn, error};

use defmt_rtt as _;
bind_interrupts!(struct Irqs {
    I2C1 => i2c::InterruptHandler<I2C1>;
});

//mod cancontroller;

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
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[derive(defmt::Format)]
pub enum CanError {
    ClockNotComingUp,
    CANNotRespondingReset,
    CANNotRespondingMem,
    CANNotResponding
}

fn setup_hfext() -> Result<(), CanError> {
    // Hack: Set up HFXT with the 25MHz oscillator
    // We'll use this for clocking the CAN peripheral
    // PA6 - PINCM11. Pin function 6 is HFCLK_IN.
    IOMUX.pincm(10).modify(|w| {
        w.set_pf(6);
        w.set_inena(true);
        w.set_pc(true);
    });
    SYSCTL.hsclken().modify(|w| {
        w.set_useexthfclk(true);
    });

    let mut cnt = 0;
    while !SYSCTL.clkstatus().read().hfclkgood() {
        if cnt > 1000 {
            return Err(CanError::ClockNotComingUp);
        }
        cnt += 1;
        cortex_m::asm::delay(1000);
    }

    Ok(())
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let periph = embassy_mspm0::init(Default::default());
    let mut led_output = Output::new(periph.PA25, embassy_mspm0::gpio::Level::Low);
    let mut led_output_2 = Output::new(periph.PA10, embassy_mspm0::gpio::Level::High);

    let mut canstb = Output::new(periph.PA0, embassy_mspm0::gpio::Level::Low);

    let s2 = Input::new(periph.PA24, embassy_mspm0::gpio::Pull::Up);
    
    let tx = periph.PA21;

    let mut config = embassy_mspm0::uart::Config::default();
    config.baudrate = 115200;

    //let mut uart = Uart::new_blocking(periph.UART0, rx, tx, config).unwrap();
    let mut uart = UartTx::new_blocking(periph.UART2, tx, config).unwrap();

    info!("Init completed");
    embassy_time::Timer::after(Duration::from_millis(100)).await;  

    let mut i2cinter = i2c::I2c::new_async(periph.I2C1, periph.PA4, periph.PA3, Irqs, i2c::Config::default()).unwrap();

    info!("was OK");
    
    i2c_scan(&mut i2cinter).await;

    // need to zero this somehow - unsafe?

    if let Err(e) = setup_hfext() {
        error!("Failed to configure hfclk: {:?}", e);
    } else {
        info!("CAN configured?");
    }

    let mut candriver = can::Can::new_blocking(periph.CANFD0, periph.PA27, periph.PA26, can::Config {
        functional_clock_rate: 25_000_000,
        clock_div: can::ClockDiv::DivBy1,
        accept_extended_ids: true,
        accept_remote_frames: false,
        timing: can::CanTimings::from_values(1, 30, 219, 30).unwrap(),
    }).unwrap();

    let mut interval = 0;
    loop {
        led_output.toggle();
        if s2.is_low() {
            led_output_2.toggle();
        }
        canstb.set_low();
        

        uart.blocking_write(b"ab\r\n").unwrap();

        //uart.blocking_write(b"abcd1334\r\n").unwrap();
        let ecr = embassy_mspm0::pac::CANFD0.mcan(0).ecr().read();
        let psr = embassy_mspm0::pac::CANFD0.mcan(0).psr().read();
        info!("ECR: {} {} {} {}", ecr.cel(), ecr.rp(), ecr.rec(), ecr.tec());
        info!("PSR: ep {} pxe {} lec {} bo {}", psr.ep(), psr.pxe(), psr.lec(), psr.bo());
        embassy_time::Timer::after(Duration::from_millis(100)).await;

        
    } 
}
