#![no_main]
#![no_std]

use cortex_m::{asm::nop};
use embassy_mspm0::{Config, gpio::Output, uart::Uart};
use embassy_executor::Spawner;

use embassy_time::{Delay, Duration};
use embedded_hal::delay::DelayNs;

// TODO(5) adjust HAL import
// use some_hal as _; // memory layout


use panic_probe as _;
/// Terminates the application and makes a semihosting-capable debug tool exit
/// with status code 0.
pub fn exit() -> ! {
    semihosting::process::exit(0);
}

/// Hardfault handler.
///
/// Terminates the application and makes a semihosting-capable debug tool exit
/// with an error. This seems better than the default, which is to spin in a
/// loop.
#[cortex_m_rt::exception]
unsafe fn HardFault(_frame: &cortex_m_rt::ExceptionFrame) -> ! {
    semihosting::process::exit(1);
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let periph = embassy_mspm0::init(Default::default());

    let mut led_output = Output::new(periph.PA22, embassy_mspm0::gpio::Level::Low);
    
    let tx = periph.PA27;
    let rx = periph.PA26;

    let mut config = embassy_mspm0::uart::Config::default();
    config.baudrate = 115200;

    let mut uart = Uart::new_blocking(periph.UART0, rx, tx, config).unwrap();


    loop {
        led_output.toggle();

        uart.blocking_write(b"abcd1334\r\n").unwrap();
        
        Delay.delay_ms(1000);
    }
}
