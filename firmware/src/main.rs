#![no_main]
#![no_std]

use cortex_m::{asm::nop};
use embassy_mspm0::{Config, gpio::Output};

use embassy_mspm0::{Peri, bind_interrupts};
use embassy_mspm0::i2c::{self};
use embassy_mspm0::peripherals::I2C0;

use embassy_mspm0::uart::{self, UartTx};
use embassy_mspm0::peripherals::UART0;

use embedded_hal_async::i2c::{I2c, ErrorKind, Error as I2cError};
use embedded_io_async::Write;

use embassy_executor::Spawner;

use embassy_time::{Duration};

// TODO(5) adjust HAL import
// use some_hal as _; // memory layout

use defmt::*;

use defmt_rtt as _;
use panic_probe as _;
/// Terminates the application and makes a semihosting-capable debug tool exit
/// with status code 0.
pub fn exit() -> ! {
    semihosting::process::exit(0);
}

bind_interrupts!(struct Irqs {
    I2C0 => i2c::InterruptHandler<I2C0>;
});

/// Hardfault handler.
///
/// Terminates the application and makes a semihosting-capable debug tool exit
/// with an error. This seems better than the default, which is to spin in a
/// loop.
#[cortex_m_rt::exception]
unsafe fn HardFault(_frame: &cortex_m_rt::ExceptionFrame) -> ! {
    semihosting::process::exit(1);
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

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let periph = embassy_mspm0::init(Default::default());
    let mut led_output = Output::new(periph.PA22, embassy_mspm0::gpio::Level::Low);
    
    let tx = periph.PA27;

    let mut config = embassy_mspm0::uart::Config::default();
    config.baudrate = 115200;


    //let mut uart = Uart::new_blocking(periph.UART0, rx, tx, config).unwrap();
    let mut uart = UartTx::new_blocking(periph.UART0, tx, config).unwrap();

    info!("Init completed");
    embassy_time::Timer::after(Duration::from_millis(100)).await;  

    let mut i2cinter = i2c::I2c::new_async(periph.I2C0, periph.PA11, periph.PA0, Irqs, i2c::Config::default()).unwrap();

    info!("was OK");
    
    i2c_scan(&mut i2cinter).await;

    loop {
        led_output.toggle();

        uart.blocking_write(b"ab\r\n").unwrap();

        //uart.blocking_write(b"abcd1334\r\n").unwrap();
        info!("testing");
        embassy_time::Timer::after(Duration::from_millis(100)).await;  
    } 
}
