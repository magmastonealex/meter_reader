#![no_main]
#![no_std]

use core::ptr;

use cortex_m::{asm::nop};
use embassy_mspm0::gpio::Input;
use embassy_mspm0::pac::canfd::{Canfd, vals as CanVals};
use embassy_mspm0::pac::sysctl::regs::{Syspllparam0, Syspllparam1};
use embassy_mspm0::{Config, gpio::Output};

use embassy_mspm0::{Peri, bind_interrupts};
use embassy_mspm0::i2c::{self};
use embassy_mspm0::peripherals::I2C1;

use embassy_mspm0::uart::{self, UartTx};
use embassy_mspm0::peripherals::UART0;

use embedded_hal::i2c::{I2c, ErrorKind, Error as I2cError};
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
    I2C1 => i2c::InterruptHandler<I2C1>;
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
        match i2c.read(addr, &mut tmp) {
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
    CANNotResponding
}

use embassy_mspm0::pac::{CANFD0, IOMUX, SYSCTL};

// We know this all works from C - EXCEPT for hfclkgood, which seemingly is always 0.
// Getting CAN revision data should also work and works fine from C.
// Is the SVD wrong, or are we doing something weird?
// Let's try the SVD in GDB and see what happens.

// svd/b SYSCTL SYSCTL_CLKSTATUS shows HFCLKGOOD = true???
fn do_can_stuff() -> Result<(), CanError> {

    // Hack: Set up HFXT with the 25MHz oscillator
    // We'll use this for clocking the CAN peripheral
    // sure hope it works!
    // PINCM is PINCM -1 for some reason????
    // PA6 - PINCMx 11. Pin function 6 is HFCLK_IN.
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

    // MCAN status?
    info!("MCAN status? {:x}", SYSCTL.sysstatus().read().mcan0ready());
    info!("clk status? {:x}", SYSCTL.clkstatus().read().hfclkgood());


    // "HFXT" is now working, and can be used as the async clock for CAN.
    // This appears to be the default state.
    //SYSCTL.genclkcfg().write(|w| {
    //    w.set_canclksrc(embassy_mspm0::pac::sysctl::vals::Canclksrc::HFCLK);
    //});
    //DL_GPIO_initPeripheralInputFunction(GPIO_HFCLKIN_IOMUX, GPIO_HFCLKIN_IOMUX_FUNC);
    //

    // Connect CAN_RX (PA27, PINCM60, PF 6, input.)
    IOMUX.pincm(59).modify(|w| {
        w.set_pf(6);
        w.set_inena(true);
        w.set_pc(true);
    });
    // Connect CAN_TX (PA26, PINCM59, PF 6)
    IOMUX.pincm(58).modify(|w| {
        w.set_pf(6);
        w.set_pc(true);
    });

    // Turn on the CAN peripheral!
    let can = embassy_mspm0::pac::CANFD0;
    can.pwren().write(|w| {
        w.set_enable(true);
        w.set_key(CanVals::PwrenKey::KEY);
    });
    cortex_m::asm::delay(16);

    can.ti_wrapper(0).msp(0).subsys_clken().write(|w| {
        w.set_clk_reqen(true);
    });

    SYSCTL.genclkcfg().modify(|w| {
        w.set_canclksrc(embassy_mspm0::pac::sysctl::vals::Canclksrc::HFCLK);
    });

    cnt = 0;
    while !can.ti_wrapper(0).processors(0).subsys_regs(0).subsys_stat().read().mem_init_done() {
        if cnt > 10000 {
            return Err(CanError::CANNotResponding);
        }
        cnt += 1;
        cortex_m::asm::delay(9000);
    }
    

    info!("MCAN status? {:x}", SYSCTL.sysstatus().read().mcan0ready());
    info!("CAN module data: {:x}", can.ti_wrapper(0).processors(0).subsys_regs(0).subsys_pid().read().scheme());
    info!("CAN module data: {:x}", can.ti_wrapper(0).processors(0).subsys_regs(0).subsys_pid().read().module_id());
    info!("CAN module data: {:x}", can.ti_wrapper(0).processors(0).subsys_regs(0).subsys_pid().read().major());
    info!("CAN module data: {:x}", can.ti_wrapper(0).processors(0).subsys_regs(0).subsys_pid().read().minor());

    info!("CAN release data: {:x}", can.mcan(0).crel().read().day());
    info!("CAN release data: {:x}", can.mcan(0).crel().read().mon());
    info!("CAN release data: {:x}", can.mcan(0).crel().read().year());
    info!("CAN release data: {:x}", can.mcan(0).crel().read().step());
    info!("CAN release data: {:x}", can.mcan(0).crel().read().substep());
    
    info!("CAN endian check: {:x}", can.mcan(0).endn().read().0);
    
    info!("CAN clken: {:x}", can.ti_wrapper(0).msp(0).subsys_clken().read().clk_reqen());

    info!("CAN is clocked? {:?}", can.ti_wrapper(0).msp(0).subsys_clksts().read().cclkdone());
    //can.ti_wrapper(0).msp(0).subsys_clken();
    

    // Read out some version data.
    
    info!("CAN version: {:x}", can.mcan(0).crel().read().rel());

    // Step by step per table 26-4.
    can.mcan(0).cccr().modify(|w| {
        w.set_init(true);
    });

    cnt = 0;
    while !can.mcan(0).cccr().read().init() {
        if cnt > 10000 {
            return Err(CanError::CANNotResponding);
        }
        cnt += 1;
        cortex_m::asm::delay(1000);
    }

    can.mcan(0).cccr().modify(|w| {
        w.set_cce(true);
    });

    // can freely change registers now.
    can.mcan(0).cccr().modify(|w| {
        w.set_fdoe(false); // classic CAN.
    });

    // Set nominal bit timing.

    // We have a 25MHz clock coming in. (40ns)
    // We have a target bitrate of 100 kbit/second (10uS = 10000ns)
    // We desire ~16 tq, but can tolerate more for now
    // (will tune using PLL later.)
    // let's say a prescaler of 10 gives us a can clock of 2.5MHz (400ns)
    // 400ns * 25 tq = 10000ns
    // so that leaves us with 25 tq per bit at 100kbit/second.
    // seg1 = 21, seg2 = 3 = sample point 88% (close enough to 87.5 for our work)
    
    // sjw = 2, consider bumping to 3.
    /*can.mcan(0).nbtp().write(|w| {
        w.set_nbrp(10-1);

        w.set_ntseg1(21-1); // per docs, 1 more than the programmed value is used.
        w.set_ntseg2(3-2);

        w.set_nsjw(2-1);
    });*/
    can.mcan(0).nbtp().write(|w| {
        w.set_nbrp(0);

        w.set_ntseg1(218); // per docs, 1 more than the programmed value is used.
        w.set_ntseg2(29);

        w.set_nsjw(29);
    });

    /*can.mcan(0).nbtp().write(|w| {
        w.set_nbrp(5-1);

        w.set_ntseg1(13-1); // per docs, 1 more than the programmed value is used.
        w.set_ntseg2(2-2);

        w.set_nsjw(2-1);
    });*/

    // Enter normal running state.
    can.mcan(0).cccr().modify(|w| {
        w.set_cce(false);
    });
    can.mcan(0).cccr().modify(|w| {
        w.set_init(false);
    });

    cnt = 0;
    while can.mcan(0).cccr().read().init() {
        if cnt > 10000 {
            return Err(CanError::CANNotResponding);
        }
        cnt += 1;
        cortex_m::asm::delay(1000);
    }

    // This should be enough to at least get acks?

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

    let mut i2cinter = i2c::I2c::new_blocking(periph.I2C1, periph.PA4, periph.PA3, i2c::Config::default()).unwrap();

    info!("was OK");
    
    i2c_scan(&mut i2cinter).await;

    if let Err(e) = do_can_stuff() {
        error!("Failed to configure CAN: {:?}", e);
    } else {
        info!("CAN configured?");
    }

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
        info!("testing: {} {} {} {}", ecr.cel(), ecr.rp(), ecr.rec(), ecr.tec());
        info!("PSR: ep {} pxe {} lec {} bo {}", psr.ep(), psr.pxe(), psr.lec(), psr.bo());
        embassy_time::Timer::after(Duration::from_millis(100)).await;  
    } 
}
