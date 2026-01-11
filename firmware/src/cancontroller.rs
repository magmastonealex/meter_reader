use embassy_hal_internal::PeripheralType;

use embassy_mspm0::interrupt::Interrupt;
use embassy_mspm0::mode::Mode;
use embassy_mspm0::pac::canfd::{Canfd as Regs, vals as CanVals};
use embassy_sync::waitqueue::AtomicWaker;

pub(crate) struct Info { // metadata/details about the specific instance of the peripheral in use.
    pub(crate) regs: Regs, // the registers for this specific instance
    pub(crate) interrupt: Interrupt, // which interrupt applies to this peripheral
}

pub(crate) struct State {
    // waker for when interesting things happen, I guess.
    pub(crate) waker: AtomicWaker,
}

// prevent external callers from creating instances of this.
pub(crate) trait SealedInstance {
    fn info() -> &'static Info;
    fn state() -> &'static State;
}

#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType {
    type Interrupt: embassy_mspm0::interrupt::typelevel::Interrupt;
}

// provide all of the details we need to use the CANFD0 peripheral.
// We would need another one of these for CANFD1, etc. which is why macros are usually used.
impl SealedInstance for embassy_mspm0::peripherals::CANFD0 {
    fn info() -> &'static Info {
        // too many types named Interrupt. There's an impl ... somewhere? in generated code impl Interrupt for CANFD0 which provides the IRQ constant enum value,
        // which we then use to reference the actual interrupt channel in other methods.
        // The type usage here is quite confusing to me.
        use embassy_mspm0::interrupt::typelevel::Interrupt; 

        const INFO: Info = Info {
            regs: embassy_mspm0::pac::CANFD0,
            interrupt: embassy_mspm0::interrupt::typelevel::CANFD0::IRQ,
        };

        &INFO
    }

    fn state() -> &'static State {
        static STATE: State = State {
            waker: AtomicWaker::new()
        };

        &STATE
    }
}

impl Instance for embassy_mspm0::peripherals::CANFD0 {
    type Interrupt = crate::interrupt::typelevel::CANFD0; // I'm still unclear why this type is needed _here_ too.
}

// the Blocking vs Async type parameters seem to be here to mostly avoid creating separate types? I'm not 100% clear on why that's preferrable.
// oh - unless there's truly common functionality between the two of them that's worth preserving? 

// I think the lifetime parameter here is done to ensure the struct can't outlive the pins?
// going to try eliding this for now and see what happens.
pub struct I2c<M: Mode> {
    info: &'static Info,
    state: &'static State,
    scl: Option<Peri<'d, AnyPin>>,
    sda: Option<Peri<'d, AnyPin>>,
    _phantom: PhantomData<M>,
}


// RX and TX pin traits - normally constructed via a macro, we'll do it manually to demonstrate functionality.
// These are effectively sealed because pf_num isn't public so can't be implemented by anyone else.
// we'll implement the correct combinations so the type system enforces you can't pass invalid pins in for each use case.
// pf_num is metadata we need anyways.
pub trait RxPin<T: Instance>: embassy_mspm0::gpio::Pin {
    fn pf_num(&self) -> u8;
}

pub trait TxPin<T: Instance>: embassy_mspm0::gpio::Pin {
    fn pf_num(&self) -> u8;
}

impl RxPin<embassy_mspm0::peripherals::CANFD0> for embassy_mspm0::peripherals::PA27 {
    fn pf_num(&self) -> u8 {
        6u8
    }
}

impl TxPin<embassy_mspm0::peripherals::CANFD0> for embassy_mspm0::peripherals::PA26 {
    fn pf_num(&self) -> u8 {
        6u8
    }
}