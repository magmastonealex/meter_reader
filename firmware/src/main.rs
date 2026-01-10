#![no_main]
#![no_std]

use core::panic::PanicInfo;
use core::ptr;
use core::sync::atomic::{AtomicBool, Ordering, compiler_fence};

use arbitrary_int::{u2, u4, u5, u7, u11, u29};
use cortex_m::{asm::nop};
use derive_mmio::Mmio;
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

use embassy_mspm0::pac::{CANFD0, IOMUX, SYSCTL};
use bitbybit::{bitenum, bitfield};


use defmt::{info, warn, error};

use defmt_rtt as _;
bind_interrupts!(struct Irqs {
    I2C1 => i2c::InterruptHandler<I2C1>;
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

/* Bosch MCAN Message RAM definitions - artisinal, hand-crafted, and super-duper hacky! */

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[derive(defmt::Format)]
pub enum CanError {
    ClockNotComingUp,
    CANNotRespondingReset,
    CANNotRespondingMem,
    CANNotResponding
}

#[bitenum(u2, exhaustive = true)]
enum StandardFilterType {
    RangeFilter = 0x0,
    DualID = 0x1,
    ClassicFilter = 0x2,
    FilterDisabled = 0x3,
}

#[bitenum(u3, exhaustive = true)]
enum StandardFilterConfig {
    NoActionDisableFilter = 0x0,
    StoreRXFIFO0 = 0x1,
    StoreRXFIFO1 = 0x2,
    RejectID = 0x3,
    SetPriority = 0x4,
    SetPriorityStoreFIFO0 = 0x5,
    SetPriorityStoreFIFO1 = 0x6,
    StoreRXBuffer = 0x7,
}

#[bitfield(u32, defmt_bitfields)]
struct McanFilter11 {
    #[bits(30..=31, rw)]
    sft: StandardFilterType,
    #[bits(27..=29, rw)]
    sfec: StandardFilterConfig,

    #[bits(16..=26, rw)]
    sfid1: u11,

    #[bits(11..=15, r)]
    rsvd:u5,

    #[bits(0..=10, rw)]
    sfid2: u11
}

#[bitfield(u32, defmt_bitfields)]
struct McanFilter29Hdr1 {
    #[bits(29..=31, rw)]
    efec: StandardFilterConfig,

    #[bits(0..=28, rw)]
    efid1: u29,
}

#[bitfield(u32, defmt_bitfields)]
struct McanFilter29Hdr2 {
    #[bits(30..=31, rw)]
    eft: StandardFilterType,
    #[bit(29)]
    res: bool,
    #[bits(0..=28, rw)]
    efid2: u29,
}

#[derive(defmt::Format)]
#[repr(C)]
struct McanFilter29 {
    f1: McanFilter29Hdr1,
    f2: McanFilter29Hdr2
}


#[bitfield(u32, defmt_bitfields)]
struct BufHeader1 {
    #[bit(31, rw)]
    esi: bool,

    #[bit(30, rw)]
    xtd: bool,

    #[bit(29, rw)]
    rtr: bool,

    #[bits(0..=28, rw)]
    id: u29
}
#[bitfield(u32, defmt_bitfields)]
struct RXBufHeader2 {
    #[bit(31, rw)]
    anmf: bool,

    #[bits(24..=30, rw)]
    fidx: u7,

    #[bits(22..=23, rw)]
    rsvd1: u2,

    #[bit(21, rw)]
    fdf: bool,

    #[bit(20, rw)]
    brs: bool,

    #[bits(16..=19, rw)]
    dlc: u4,

    #[bits(0..=15, rw)]
    rxts: u16
}

#[derive(defmt::Format)]
#[repr(C)]
struct McanRXFifoEntry {
    hdr1: BufHeader1,
    hdr2: RXBufHeader2,
    data: [u8; 8],
}

#[bitfield(u32, defmt_bitfields)]
struct TxBufHeader2 {
    #[bits(24..=31, rw)]
    marker: u8,

    #[bit(23, rw)]
    track_fifo: bool,

    #[bit(22, r)]
    rsvd: bool,

    #[bit(21, rw)]
    fdf: bool,

    #[bit(20, rw)]
    brs: bool,

    #[bits(16..=19, rw)]
    dlc: u4,

    #[bits(0..=15, rw)]
    rsvd2: u16
}

#[derive(defmt::Format)]
#[repr(C)]
struct McanTXEntry {
    hdr1: BufHeader1,
    hdr2: TxBufHeader2,
    data: [u8; 8],
}

#[bitfield(u32, defmt_bitfields)]
struct TxEventHeader1 {
    #[bit(31)]
    esi: bool,

    #[bit(30)]
    xtd: bool,
    #[bit(29)]
    rtr: bool,
    #[bits(0..=28)]
    id: u29
}
#[bitfield(u32, defmt_bitfields)]
struct TxEventHeader2 {
    #[bits(24..=31)]
    marker: u8,

    #[bits(22..=23)]
    event: u2,

    #[bit(21)]
    fdf: bool,

    #[bit(20)]
    brs: bool,

    #[bits(16..=19)]
    dlc: u4,

    #[bits(0..=15)]
    txts: u16
}

#[derive(defmt::Format)]
#[repr(C)]
struct McanTXEventEntry {
    hdr1: TxEventHeader1,
    hdr2: TxEventHeader2
}


// it would be nice if these could be generic across the various sizing dimensions, especially because you could enforce accuracy at compile time,
// but derive-mmio can't do this - so we fix the array sizes.
// It's undocumented by TI, but the message RAM lives at the beginning
// of the stated peripheral area in the memory map.
// This lives at 1079017472 = 0x40508000 on MSPM0G310x.
#[derive(Mmio)]
#[repr(C)]
struct McanMessageRAM {
    filters: [McanFilter11; 1],
    extended_filters: [McanFilter29; 0],
    rxfifo0: [McanRXFifoEntry; 10], 
    rxfifo1: [McanRXFifoEntry; 0],
    rxbuffers: [McanRXFifoEntry; 0],
    txevents: [McanTXEventEntry; 10],
    txfifo0: [McanTXEntry; 10],
}
#[derive(defmt::Format)]
struct MessageRamOffsets {
    filters: usize,
    extended_filters: usize,
    rxfifo0: usize,
    rxfifo1: usize,
    rxbuffers: usize,
    txevents: usize,
    txfifo: usize,
}

impl McanMessageRAM {
    const LENGTHS: MessageRamOffsets = {

        let num_filters = core::mem::offset_of!(McanMessageRAM, extended_filters) / core::mem::size_of::<McanFilter11>();
        let num_extended = (core::mem::offset_of!(McanMessageRAM, rxfifo0) - core::mem::offset_of!(McanMessageRAM, extended_filters))/core::mem::size_of::<McanFilter29>();
        let num_rxfifo0 = (core::mem::offset_of!(McanMessageRAM, rxfifo1) - core::mem::offset_of!(McanMessageRAM, rxfifo0))/core::mem::size_of::<McanRXFifoEntry>();
        let num_rxfifo1 = (core::mem::offset_of!(McanMessageRAM, rxbuffers) - core::mem::offset_of!(McanMessageRAM, rxfifo1))/core::mem::size_of::<McanRXFifoEntry>();
        let num_rxbuffers = (core::mem::offset_of!(McanMessageRAM, txevents) - core::mem::offset_of!(McanMessageRAM, rxbuffers))/core::mem::size_of::<McanTXEventEntry>();
        let num_txevents = (core::mem::offset_of!(McanMessageRAM, txfifo0) - core::mem::offset_of!(McanMessageRAM, txevents))/core::mem::size_of::<McanTXEventEntry>();
        let num_txfifo = (core::mem::size_of::<McanMessageRAM>() - core::mem::offset_of!(McanMessageRAM, txfifo0))/core::mem::size_of::<McanTXEntry>();

        if core::mem::size_of::<McanMessageRAM>() > 1024 {
            panic!("message RAM too large!");
        }

        MessageRamOffsets {
            filters: num_filters,
            extended_filters: num_extended,
            rxfifo0: num_rxfifo0,
            rxfifo1: num_rxfifo1,
            rxbuffers: num_rxbuffers,
            txevents: num_txevents,
            txfifo: num_txfifo
        }


    };
    const OFFSETS: MessageRamOffsets = {
        MessageRamOffsets {
            filters: 0,
            extended_filters: core::mem::offset_of!(McanMessageRAM, extended_filters)/4,
            rxfifo0: core::mem::offset_of!(McanMessageRAM, rxfifo0)/4,
            rxfifo1: core::mem::offset_of!(McanMessageRAM, rxfifo1)/4,
            rxbuffers: core::mem::offset_of!(McanMessageRAM, rxbuffers)/4,
            txevents: core::mem::offset_of!(McanMessageRAM, txevents)/4,
            txfifo: core::mem::offset_of!(McanMessageRAM, txfifo0)/4
        }
    };
}

// Hacky, awful code to test out the MCAN peripheral.
// This is clearly a small wrapper from TI around the Bosch MCAN IP.
fn do_can_stuff() -> Result<(), CanError> {

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

    info!("MCAN status? {:x}", SYSCTL.sysstatus().read().mcan0ready());
    info!("clk status? {:x}", SYSCTL.clkstatus().read().hfclkgood());

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

    /*
        // Performing this reset seems to lock up the peripheral - it will never complete memory initialization and all values will read as zeros.
        // It also seems to mess with the peripheral bus, as i2c seems to misbehave... avoid doing this right now.
        // I don't see any errata, but TI's examples don't seem to ever reset it?
        can.rstctl().write(|w| {
            w.set_resetstkyclr(true);
            w.set_resetassert(true);
            w.set_key(CanVals::ResetKey::KEY);
        });
    */
    
    // Allow the peripheral to keep running in debug halt mode.
    can.ti_wrapper(0).processors(0).subsys_regs(0).subsys_ctrl().modify(|w| {
        w.set_dbgsusp_free(true);
    });

    // this is NOT in the reference manual, but to get the peripheral to work,
    // you need to set a clock request for the peripheral in the "wrapper" around Bosch's MCAN IP.
    can.ti_wrapper(0).msp(0).subsys_clken().write(|w| {
        w.set_clk_reqen(true);
    });

    SYSCTL.genclkcfg().modify(|w| {
        w.set_canclksrc(embassy_mspm0::pac::sysctl::vals::Canclksrc::HFCLK);
    });
    
    cnt = 0;
    while can.ti_wrapper(0).processors(0).subsys_regs(0).subsys_stat().read().reset() {
        if cnt > 10000 {
            return Err(CanError::CANNotRespondingReset);
        }
        cnt += 1;
        cortex_m::asm::delay(9000);
    }

    cnt = 0;
    while !can.ti_wrapper(0).processors(0).subsys_regs(0).subsys_stat().read().mem_init_done() {
        if cnt > 10000 {
            return Err(CanError::CANNotRespondingMem);
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
    // I grabbed this from CCS SysConfig based on a 25MHz input clock.
    // At some point, calculate this properly.

    can.mcan(0).nbtp().write(|w| {
        w.set_nbrp(0);

        w.set_ntseg1(218);
        w.set_ntseg2(29);

        w.set_nsjw(29);
    });

    // Set global filter configuration.
    can.mcan(0).gfc().write(|w| {
        w.set_anfs(0x00); // accept non-matching 11-bit id frames into RX FIFO 0.
        w.set_anfe(0b10); // Reject extended frames for now.
        w.set_rrfs(true);
        w.set_rrfe(true);
    });

    // Configure message RAM sizes & start addresses.
    // 11 bit filters
    can.mcan(0).sidfc().write(|w| {
        w.set_lss(McanMessageRAM::LENGTHS.filters as u8); // one filter element.
        w.set_flssa(McanMessageRAM::OFFSETS.filters as u16); // start at beginning of RAM.
    });

    // 29 bit filters
    can.mcan(0).xidfc().write(|w| {
        w.set_lse(McanMessageRAM::LENGTHS.extended_filters as u8);
        w.set_flesa(McanMessageRAM::OFFSETS.extended_filters as u16); // no filter elements.
    });
    // RX FIFO 0
    can.mcan(0).rxf0c().write(|w| {
        w.set_f0om(false); // blocking mode - don't overwrite messages.
        w.set_f0wm(0); // no watermark interrupt.
        w.set_f0s(McanMessageRAM::LENGTHS.rxfifo0 as u8); // 10 elements in the FIFO.
        w.set_f0sa(McanMessageRAM::OFFSETS.rxfifo0 as u16); // we have 1 filter element.
    });
    // RX FIFO 1
    can.mcan(0).rxf1c().write(|w| {
        w.set_f1om(false);
        w.set_f1wm(0); // no watermark.
        w.set_f1s(McanMessageRAM::LENGTHS.rxfifo1 as u8); // no elements
        w.set_f1sa(McanMessageRAM::OFFSETS.rxfifo1 as u16); // start address is meaningless.
    });
    // RX Buffers
    can.mcan(0).rxbc().write(|w| {
        w.set_rbsa(McanMessageRAM::OFFSETS.rxbuffers as u16);
    });
    // Sizes for various RX elements.
    can.mcan(0).rxesc().write(|w| {
        w.set_rbds(0);
        w.set_f1ds(0);
        w.set_f0ds(0); // 8 byte size - TODO compute this :)
    });
    // TX Event FIFO
    can.mcan(0).txefc().write(|w| {
        w.set_efsa(McanMessageRAM::OFFSETS.txevents as u16); // 10 * 10 = 100 RX fifo size, started at address 1 = next adddress is 102.
        w.set_efs(McanMessageRAM::LENGTHS.txevents as u8);
        w.set_efwm(0);
    });
    // TX Buffers
    can.mcan(0).txbc().write(|w| {
        w.set_tfqm(false); // FIFO operation mode.
        w.set_tfqs(McanMessageRAM::LENGTHS.txfifo as u8); // No TX FIFO/queue.
        w.set_ndtb(0); // No dedicated transmit buffers.
        w.set_tbsa(McanMessageRAM::OFFSETS.txfifo as u16); // 102 + 20 = 122, 123 is next free index.
    });
    can.mcan(0).txesc().write(|w| {
        w.set_tbds(0); // max 8 byte data payloads in TX elements.
    });

    let mut z = McanFilter11::ZERO;
    z.set_sft(StandardFilterType::RangeFilter);
    z.set_sfec(StandardFilterConfig::StoreRXFIFO0);

    z.set_sfid1(u11::new(0x100));
    z.set_sfid2(u11::new(0x200));
    let mut msgram = unsafe {McanMessageRAM::new_mmio_at(0x4050_8000)};

    info!("Installing filter: {:?}", z);
    msgram.write_filters(0, z).expect("bad index writing filter?");
    
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

    // need to zero this somehow - unsafe?

    if let Err(e) = do_can_stuff() {
        error!("Failed to configure CAN: {:?}", e);
    } else {
        info!("CAN configured?");
    }

    let mut msgram = unsafe {McanMessageRAM::new_mmio_at(0x4050_8000)};


    info!("offsets calc 1: {}", McanMessageRAM::OFFSETS);
    info!("lengths:  {}", McanMessageRAM::LENGTHS);

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

        let fs = embassy_mspm0::pac::CANFD0.mcan(0).rxf0s().read();
        info!("RX FIFO fill level: {}, get index: {}, put index: {}, full: {}", fs.f0fl(), fs.f0gi(), fs.f0pi(), fs.f0f());

        let txfs = embassy_mspm0::pac::CANFD0.mcan(0).txfqs().read();
        info!("TX FIFO fill level: {}, get index: {}, put index: {}, full: {}", txfs.tffl(), txfs.tfgi(), txfs.tfqp(), txfs.tfqf());

        if fs.f0gi() != fs.f0pi() {
            let thismsg = msgram.read_rxfifo0(fs.f0gi() as usize).expect("out of bounds!?");

            info!("Got CAN message: {:?}", thismsg);

            embassy_mspm0::pac::CANFD0.mcan(0).rxf0a().write(|w| {
                w.set_f0ai(fs.f0gi());
            });


            if embassy_mspm0::pac::CANFD0.mcan(0).txfqs().read().tfqf() {
                info!("Could not reply - queue full!");
            } else {
                let write_idx = embassy_mspm0::pac::CANFD0.mcan(0).txfqs().read().tfqp();
                msgram.modify_txfifo0(write_idx as usize, |mut w| {
                    w.data[0] = 0xDE;
                    w.data[1] = 0xAD;
                    w.data[2] = 0xBE;
                    w.data[3] = 0xEF;
                    w.data[4] = thismsg.data[0];
                    w.data[5] = thismsg.data[1];
                    w.data[6] = thismsg.data[2];
                    w.data[7] = thismsg.data[3];
                    w.hdr2.set_dlc(u4::new(8));
                    w.hdr1.set_rtr(false);
                    w.hdr1.set_xtd(false);
                    w.hdr1.set_esi(false);
                    let id: u32 = 0x500u32 << 18;
                    w.hdr1.set_id(u29::new(id));

                    w.hdr2.set_marker(123);
                    w.hdr2.set_track_fifo(true);

                    w
                }).expect("write idx out of range?");

                let fifoentry = msgram.read_txfifo0(write_idx as usize);
                info!("Writing: idx {} {:?}", write_idx, fifoentry);

                embassy_mspm0::pac::CANFD0.mcan(0).txbar().write(|w| { 
                    w.0 = 1 << write_idx;
                });
            }
        }

        let event_fs = embassy_mspm0::pac::CANFD0.mcan(0).txefs().read();
        
        info!("TX event FIFO get index: {}, put index: {}, fill level: {}", event_fs.efgi(), event_fs.efpi(), event_fs.effl());
        if event_fs.efgi() != event_fs.efpi() {
            let thismsg = msgram.read_txevents(event_fs.efgi() as usize).expect("out of bounds!?");
            info!("Got TX event: {:?}", thismsg);
            embassy_mspm0::pac::CANFD0.mcan(0).txefa().write(|w| {
                w.set_efai(event_fs.efgi());
            });
        }

        
    } 
}
