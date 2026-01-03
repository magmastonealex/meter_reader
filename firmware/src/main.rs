#![no_main]
#![no_std]

use cortex_m::{asm::nop};
use embassy_mspm0::{Config, gpio::Output};


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

#[cortex_m_rt::entry]
fn main() -> ! {
    let periph = embassy_mspm0::init(Default::default());

    let mut led_output = Output::new(periph.PA22, embassy_mspm0::gpio::Level::Low);

    let mut cnt = 0;
    while cnt < 100 {
        cnt = cnt + 1;
        nop();
    }
    loop {
        led_output.toggle();
        Delay.delay_ms(100);
    }
}


// defmt-test 0.3.0 has the limitation that this `#[tests]` attribute can only be used
// once within a crate. the module can be in any file but there can only be at most
// one `#[tests]` module in this library crate
#[cfg(test)]
#[defmt_test::tests]
mod unit_tests {
    use defmt::assert;

    #[test]
    fn it_works() {
        assert!(true)
    }
}