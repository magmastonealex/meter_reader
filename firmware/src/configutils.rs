//! Helper functions to load and save configuration values from a sequential-storage instance.

use embedded_storage_async::nor_flash::NorFlash;
use sequential_storage::{Error, cache::KeyCacheImpl, map::{MapStorage, Value}};

use crate::flashdriver::FlashError;
use bitfield::bitfield;
const MAX_KEY_PLUS_VALUE: usize = 0x80;

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq, Debug)]
    pub struct MagnetometerConfig(u32);
    u32;


    pub i16, rising_val, set_rising_val: 31, 16;

    pub u16, rising_delta, set_rising_delta: 15, 7; // 9 bits of delta representing tolerance before considering the value RISEN.

    pub u8, axis, set_axis: 6, 5;

    pub high_sensitivity, set_high_sensitivity: 4;
    /// rate, as determined by mlx90394 datasheet - Address 0x0E – CTRL1 Register.
    /// this is a crappy way to do this, but I'm getting tired of working on this project :)
    pub u8, rate, set_rate: 3, 0;
}

impl defmt::Format for MagnetometerConfig {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "MagCfg(rising: {}, delta: {}, axis: {}, hi-sens: {}, rate: {})", self.rising_val(), self.rising_delta(), self.axis(), self.high_sensitivity(), self.rate());
    }
}

impl From<u32> for MagnetometerConfig {
    fn from(value: u32) -> Self {
        MagnetometerConfig(value)
    }
}
impl From<MagnetometerConfig> for u32 {
    fn from(value: MagnetometerConfig) -> Self {
        value.0
    }
}

impl Default for MagnetometerConfig {
    fn default() -> Self {
        let mut this = MagnetometerConfig(0);
        this.set_axis(0);
        this.set_high_sensitivity(false);
        this.set_rate(2);

        this.set_rising_delta(100);

        this
    }
}

impl TryInto<crate::mlx::Config> for &MagnetometerConfig {
    type Error = ();
    fn try_into(self) -> Result<crate::mlx::Config, Self::Error> {
        let mut cfg = match crate::mlx::Config::new(self.rate()) {
            Some(cfg) => cfg,
            None => {
                return Err(());
            }
        };

        cfg.en_x = true;
        cfg.en_y = true;
        cfg.en_z = true;

        cfg.high_sensitivity = self.high_sensitivity();

        Ok(cfg)
    }
}

// hackity hack hack because I am tired of fighting with types.
// This will take anything which implements From<u32> and Into<u32> and fetch it from storage.
// it would be nice if this was generic across the actual integer width, but I am getting tired of this,
// and just want something which works :)
pub async fn get_bitfield_or<T, A, B>(
    storage: &mut MapStorage<u8, A, B>, 
    key: ConfigKey, 
    default: T
) -> T 
where 
    // We demand that T is valid for any lifetime, which should work fine for owned / primitive types.
    T: From<u32> + Into<u32>, 
    A: NorFlash, 
    B: KeyCacheImpl<u8>
{
    let value_found: u32 = get_value_or(storage, key, default.into()).await;

    value_found.into()
}

pub async fn save_bitfield<T, A, B>(
    storage: &mut MapStorage<u8, A, B>, 
    key: ConfigKey, 
    val: T
) -> Result<(), sequential_storage::Error<A::Error>> 
where 
    // We demand that T is valid for any lifetime, which should work fine for owned / primitive types.
    T: From<u32> + Into<u32>, 
    A: NorFlash, 
    B: KeyCacheImpl<u8>
{
    let value: u32 = val.into();

    save_value(storage, key, value).await
}

#[repr(u8)]
pub enum ConfigKey {
    MagnetometerConfig = 0x01,
    MeterTicks = 0x02,
}

pub async fn get_value_or<T, A, B>(
    storage: &mut MapStorage<u8, A, B>, 
    key: ConfigKey, 
    default: T
) -> T 
where 
    // We demand that T is valid for any lifetime, which should work fine for owned / primitive types.
    T: for<'a> Value<'a>, 
    A: NorFlash, 
    B: KeyCacheImpl<u8>
{
    // Now we CAN use stack memory, because T won't point to it
    let mut tmpbuf = [0x0u8; MAX_KEY_PLUS_VALUE];
    
    let result = storage.fetch_item::<T>(&mut tmpbuf, &(key as u8)).await;

    match result {
        Ok(Some(v)) => v,
        Ok(None) => {
            defmt::warn!("Did not find item, using default");
            default
        }
        Err(_) => {
            defmt::error!("Failed to fetch item, using default");
            default
        }
    }
}


pub async fn save_value<T, A, B>(
    storage: &mut MapStorage<u8, A, B>, 
    key: ConfigKey, 
    val: T
) -> Result<(), sequential_storage::Error<A::Error>>
where 
    T: for<'a> Value<'a>, 
    A: NorFlash, 
    B: KeyCacheImpl<u8>
{
    // Now we CAN use stack memory, because T won't point to it
    let mut tmpbuf = [0x0u8; MAX_KEY_PLUS_VALUE];
    storage.store_item(&mut tmpbuf, &(key as u8), &val).await
}