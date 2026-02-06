#![allow(dead_code)]
// Hack for flashctl support before upstream support.
mod common;
mod flashctl;

use defmt::debug;
use embedded_storage_async::nor_flash::{ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash};
use flashctl::regs::{Cmdbyten, Cmdexec, Cmdweprotb};
use flashctl::vals::{Cmddone, Cmdpass};

use crate::flashdriver::flashctl::regs::Cmdtype;
use crate::flashdriver::flashctl::vals::Failverify;

// Don't expose this to parents.
// Just offer the FlashController struct,
// which a consumer can't directly create.
// You must use "take()" to get the one and only instance.
const FLASHCTL: flashctl::Flashctl = unsafe {
    flashctl::Flashctl::from_ptr(1074581504 as *mut _)
};

// Implement a PAC/HAL style `take` which provides just once instance of the FLASHCTL peripheral.
static mut FLASHCTL_TAKEN: bool = false;
pub fn take() -> Option<flashctl::Flashctl> {
    cortex_m::interrupt::free(|_f| {
        if unsafe {FLASHCTL_TAKEN} {
            None
        } else {
            unsafe { FLASHCTL_TAKEN = true; }
            Some(FLASHCTL)
        }
    })
}

// calculate the write-protect bits for 8k-size protection blocks (above 32k in the MAIN flash for our part.)
fn calc_wp_bits_8k(base_addr: u32) -> Option<Cmdweprotb>  {
    // 1024 byte sectors
    // 8 sector (8192 byte) erase protect range.
    if base_addr % 1024 != 0 {
        return None;
    }  

    // determine sector number:
    let secnum = base_addr >> 10;

    // determine which grouping of 8 that fits in:

    let group8 = secnum / 8;

    let mut inverse = 1u32<<group8;
    inverse >>= 4; // bit shift by 4. I think there's a datasheet bug (and maybe a C SDK bug?)
        // The docs say the first 4 bits are ignored.
        // But in practice, bit position 0 is actually "wp sector" 32?
        // Anyway, this is sketchy but works, so I'm going with it!
    inverse = !inverse;

    return Some(Cmdweprotb(inverse));
}

#[derive(Debug, Eq, PartialEq, defmt::Format)]
pub enum FlashError {
    /// The write or read was unaligned and could not be completed.
    Unaligned, 

    /// Internal verification of written / erased data failed - the flash is now
    /// in an inconsistent state.
    VerifyFail,

    /// An erase or write was requested in an area of flash which is out of range for this controller.
    OutOfBounds,

    /// Something bad happened internally in the driver and the flash controller rejected the operation.
    /// Value is the raw STATCMD with the top 16 (unused) bits masked off.
    Internal(u16)
}

impl NorFlashError for FlashError {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            FlashError::OutOfBounds => NorFlashErrorKind::OutOfBounds,
            FlashError::Unaligned => NorFlashErrorKind::NotAligned,
            FlashError::Internal(_) => NorFlashErrorKind::Other,
            FlashError::VerifyFail => NorFlashErrorKind::Other,
        }
    }
}

pub struct FlashController {
    flash: flashctl::Flashctl,
    start_addr: u32,
    end_addr: u32
}

// We'll implement the embedded_storage_async methods because we want to use sequential-storage,
// but they're not _actually_ async, especially because these operations block the CPU from fetching
// instructions anyways!
impl FlashController {
    /// Create a new FlashController instance.
    /// start_addr and end_addr are used as bounds for all writes/erases
    /// to protect application flash if desired.
    pub fn new(controller: flashctl::Flashctl, start_addr: u32, end_addr: u32) -> Self {
        assert!(end_addr > start_addr);
        FlashController {
            flash: controller,
            start_addr,
            end_addr
        }
    }

    const ERASESIZE: usize = 1024;
    const WRITE_WORDSIZE: usize = 8;
    const READ_WORDSIZE: usize = 1; // can byte-address for reading.

    // a ram function to perform the actual program operation.
    // TI's docs say that there is "undefined reads" if you try to read
    // from a flash bank while an operation is in progress,
    // unlike, say, ST, where flash reads just stall.
    // This seems to work in practice, but seems a bit sketchy.
    // Quote: The software sequence of setting the CMDEXEC bit and waiting for the CMDDONE response must be executed
    // from either the device SRAM or from a different flash bank from the bank that is being operated on, as the
    // flash controller will take control of the flash bank undergoing the operation. Reads to the flash bank that is being
    // operated on while the flash controller is executing the command are not predictable.
    #[unsafe(link_section = ".data")]
    #[inline(never)]
    #[unsafe(no_mangle)]
    extern "C" fn flashctl_pgm(&mut self) {
        self.flash.cmdexec().write_value(Cmdexec(0x01));

        while self.flash.statcmd().read().cmddone() != Cmddone::STATDONE {
        }
    }

    // generic flash operation.
    // Does not check alignment (as that can vary based on what you are doing.)
    fn ll_op(&mut self, addr: u32, data: [u32; 2], op: Cmdtype) -> Result<(), FlashError>{
        assert!(addr > 32768); // only above 32k supported for writes (write protect bits need to change.)

        // Calculate the write-protection bits we need to provide
        // to the controller to allow us to write in this 8k block of sectors.
        // Note we provide this function with the page of interest
        // by masking to only provide the top bits.
        let wp = calc_wp_bits_8k(addr & !0x7FFu32).unwrap();

        // Apply our write protection /  write-allow.
        self.flash.cmdweprotb().write_value(wp);

        // Where are we writing this value?
        self.flash.cmdaddr().write_value(addr);

        // Put our values in.
        self.flash.cmddata0().write_value(data[0]);
        self.flash.cmddata1().write_value(data[1]);

        // Which bytes of the data should we actually write?
        // For now we only support complete word writes.
        // Note that if you change this, you'll generate ECC errors if you read data without valid ECC set.
        // You can read from the non-ECC checked alias of flash if needed.
        self.flash.cmdbyten().write_value(Cmdbyten(0x1FF)); // DS is wrong - actually a 9 bit value not 8 bit.

        // make sure we generate ECC.
        self.flash.cmdctl().modify(|w| { // DS is wrong? or many fields missing from SVD.
            w.set_eccgenovr(false);
        });
        
        // What do we want to do? Program one word.
        self.flash.cmdtype().write_value(op);

        // these bits need to be done in a ram function to avoid corrupted reads.
        self.flashctl_pgm();

        // Grab our value of cmdstatus.
        let status = self.flash.statcmd().read();

        // Clear our registers as a bad bit flip might cause the flash-write-start bit to get set
        // and we'll have a very bad day.
        // Why a key wasn't required like it is for power and reset enable, I will never know.
        self.flash.cmdtype().write(|w| {
            w.set_command(flashctl::vals::Command::NOOP);
        });
        self.flash.cmddata0().write_value(0xFFFF_FFFF);
        self.flash.cmddata1().write_value(0xFFFF_FFFF);

        // Clear caches as we just modified flash.
        // TODO: It would be a huge performance win if we _didn't_ do this
        // but I don't see another way to have coherent reads right now.
        // This disable-enable approach is suggested in e2e here: https://e2e.ti.com/support/microcontrollers/arm-based-microcontrollers-group/arm-based-microcontrollers/f/arm-based-microcontrollers-forum/1266429/mspm0g3507-how-to-flush-cache-and-prefetch-logic
        embassy_mspm0::pac::CPUSS.ctl().modify(|w| {
            w.set_icache(embassy_mspm0::pac::cpuss::vals::Icache::DISABLE);
            w.set_prefetch(embassy_mspm0::pac::cpuss::vals::Prefetch::DISABLE);
        });
        embassy_mspm0::pac::CPUSS.ctl().modify(|w| {
            w.set_icache(embassy_mspm0::pac::cpuss::vals::Icache::ENABLE);
            w.set_prefetch(embassy_mspm0::pac::cpuss::vals::Prefetch::ENABLE);
        });

        // Now we can decide if the operation worked or not.
        // TODO: Fix the svd via a transform to avoid these useless enums and just get bool values.
        if status.cmdpass() == Cmdpass::STATPASS {
            return Ok(());
        } else if status.failverify() == Failverify::STATFAIL {
            return Err(FlashError::VerifyFail);
        } else {
            return Err(FlashError::Internal((status.0 & 0xFFFF) as u16));
        }
    }

    /// Write an entire flash word to a given (aligned!) address in flash.
    pub fn ll_write(&mut self, addr: u32, data: [u32; 2]) -> Result<(), FlashError> {
        if addr as usize % FlashController::WRITE_WORDSIZE != 0 {
            return Err(FlashError::Unaligned);
        }
        if addr < self.start_addr || addr+8 > self.end_addr {
            return Err(FlashError::OutOfBounds);
        }
        debug!("Performing write at 0x{:08X}", addr);

        let mut cmd = Cmdtype::default();
        cmd.set_command(flashctl::vals::Command::PROGRAM);
        cmd.set_size(flashctl::vals::Size::ONEWORD);
        self.ll_op(addr, data, cmd)
    }

    /// Erase a flash page/sector (1KiB size - must be aligned!)
    pub fn ll_erase(&mut self, addr: u32) -> Result<(), FlashError> {
        if addr as usize % FlashController::ERASESIZE != 0 {
            return Err(FlashError::Unaligned);
        }

        if addr < self.start_addr || addr+1024 > self.end_addr {
            return Err(FlashError::OutOfBounds);
        }

        debug!("Performing erase at 0x{:08X}", addr);

        let mut cmd = Cmdtype::default();
        cmd.set_command(flashctl::vals::Command::ERASE);
        cmd.set_size(flashctl::vals::Size::SECTOR);
        self.ll_op(addr, [0xFFFF_FFFFu32; 2], cmd)
    }

    /// "read" from flash (really just read from memory)
    /// This is just a safe wrapper around an unsafe read to the same address,
    /// useful for implementing the embedded-storage traits.
    /// also used to test "unaligned" read accesses.
    pub fn ll_read(&self, addr: u32, data: &mut [u8]) {
        debug!("Performing read at 0x{:08X}", addr);
        let flash_data = unsafe { core::slice::from_raw_parts(addr as *const u8, data.len()) };
        data.copy_from_slice(flash_data);
    }
}

impl ErrorType for FlashController {
    type Error = FlashError;
}

impl ReadNorFlash for FlashController {
    const READ_SIZE: usize = FlashController::READ_WORDSIZE;
    async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.ll_read(offset, bytes);
        Ok(())
    }
    fn capacity(&self) -> usize {
        //self.end_addr.saturating_sub(self.start_addr)  as usize
        // I think we need to "lie" here and report the entire flash size, even though we won't allow writes to all of it?
        return 131072;
    }
}

impl NorFlash for FlashController {
    const ERASE_SIZE: usize = FlashController::ERASESIZE;
    const WRITE_SIZE: usize = FlashController::WRITE_WORDSIZE;

    // The docs say this erases [from..to], i.e. non-inclusive of the last byte.
    async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        if to < from {
            return Err(FlashError::OutOfBounds);
        }
        if (to - from) % (Self::ERASE_SIZE as u32) != 0 {
            return Err(FlashError::Unaligned)
        }

        let num_pages = (to - from) / (Self::ERASESIZE as u32);
        for i in 0..num_pages {
            self.ll_erase(from + (Self::ERASESIZE as u32 * i))?;
        }

        Ok(())    
    }
    async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        if bytes.len() % Self::WRITE_WORDSIZE != 0 {
            return Err(FlashError::Unaligned);
        }

        let num_words = bytes.len() / Self::WRITE_SIZE;

        for i in 0..num_words {
            let start = i*8;
            let w1 = u32::from_le_bytes(bytes[start..start+4].try_into().unwrap());
            let w2 = u32::from_le_bytes(bytes[start+4..start+8].try_into().unwrap());

            self.ll_write(offset + (start as u32), [w1, w2])?;
        }

        Ok(())
    }
}