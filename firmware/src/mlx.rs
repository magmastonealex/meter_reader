#![allow(dead_code)]
use defmt::info;
use embassy_time::Timer;
use embedded_hal_async::i2c::{I2c};

#[derive(Debug, PartialEq, Eq, defmt::Format)]
pub struct MagReadings {
    pub x: i16,
    pub y: i16,
    pub z: i16,

    pub x_sat: bool,
    pub y_sat: bool,
    pub z_sat: bool,
    pub overflowed: bool
}

impl MagReadings {
    fn from_raw(main_data: [u8; 7]) -> Self {
        let mut data = MagReadings{
            x: i16::from_le_bytes(main_data[0..2].try_into().unwrap()),
            y: i16::from_le_bytes(main_data[2..4].try_into().unwrap()),
            z: i16::from_le_bytes(main_data[4..6].try_into().unwrap()),

            x_sat: false,
            y_sat: false,
            z_sat: false,
            overflowed: false
        };

        if (main_data[6] & 0x01) != 0 {
            data.x_sat = true;
        }
        if (main_data[6] & 0x02) != 0 {
            data.y_sat = true;
        }
        if (main_data[6] & 0x04) != 0 {
            data.y_sat = true;
        }
        if (main_data[6] & 0x08) != 0 {
            data.overflowed = true;
        }

        //defmt::info!("Readings X: {:016b} Y: {:b} Z: {:016b} D: {:08b}", data.x, data.y, data.z, main_data[6]);
        data
    }

    pub fn serialize(&self) -> [u8; 7] {
        let mut result = [0u8; 7];
        // X field (bytes 0-1)
        result[0..2].copy_from_slice(&self.x.to_be_bytes());

        // Y field (bytes 2-3)
        result[2..4].copy_from_slice(&self.y.to_be_bytes());

        // Z field (bytes 4-5)
        result[4..6].copy_from_slice(&self.z.to_be_bytes());

        // Status bitfield (byte 7)
        let mut status = 0u8;
        if self.x_sat { status |= 0x01; }
        if self.y_sat { status |= 0x02; }
        if self.z_sat { status |= 0x04; }
        if self.overflowed { status |= 0x08; }
        result[6] = status;

        result
    }
}

pub struct Mlx90394<'a, I2C> {
    i2c: &'a mut I2C,
    address: u8,
}

const REG_STAT1: u8 = 0x00;
const REG_X_LSB: u8 = 0x01;
const REG_X_MSB: u8 = 0x02;
const REG_Y_LSB: u8 = 0x03;
const REG_Y_MSB: u8 = 0x04;
const REG_Z_LSB: u8 = 0x05;
const REG_Z_MSB: u8 = 0x06;
const REG_STAT2: u8 = 0x07;
const REG_T_LSB: u8 = 0x08;
const REG_T_MSB: u8 = 0x09;
const REG_COMPANY_ID: u8 = 0x0A;
const REG_DEVICE_ID: u8 = 0x0B;
const REG_CTRL1: u8 = 0x0E;
const REG_CTRL2: u8 = 0x0F;
const REG_RESET: u8 = 0x11;
const REG_CTRL3: u8 = 0x14;
const REG_CTRL4: u8 = 0x15;
const REG_X_THR_LSB: u8 = 0x58;
const REG_X_THR_MSB: u8 = 0x59;
const REG_Y_THR_LSB: u8 = 0x5A;
const REG_Y_THR_MSB: u8 = 0x5B;
const REG_Z_THR_LSB: u8 = 0x5C;
const REG_Z_THR_MSB: u8 = 0x5D;

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum MlxError<E> {
    /// I2C communication error
    I2c(E),
    /// Magnetic sensor overflow on X axis
    OverflowX,
    /// Magnetic sensor overflow on Y axis
    OverflowY,
    /// Magnetic sensor overflow on Z axis
    OverflowZ,
    /// Invalid configuration
    InvalidConfig,
    /// Not the expected device.
    WrongDevice,
}
impl<E> From<E> for MlxError<E> {
    fn from(error: E) -> Self {
        MlxError::I2c(error)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct Config {
    pub high_sensitivity: bool,
    pub en_z: bool,
    pub en_x: bool,
    pub en_y: bool,
    rate: u8,
}

impl Config {
    pub const fn new(rate: u8) -> Option<Config> {
        if rate >= 15 || rate == 0 || rate == 1 || rate == 7 || rate == 8 || rate == 9 {
            return None;
        }

        Some(Config {
            high_sensitivity: false,
            en_x: true,
            en_y: true,
            en_z: true,
            rate: rate
        })
    }
}

impl<'a, I2C, E> Mlx90394<'a, I2C>
where
    I2C: I2c<Error = E>,
{   
    /// NOTE: a prior version of this driver accepted drdy input to await internally,
    /// but I don't think the mspm0 i2c driver handles a future being dropped due to a timeout very well.
    /// I want to be very careful in my "select" in the main loop so I only select on drdy or a CAN frame,
    /// and don't wind up interrupting an i2c transaction halfway through which would be... bad.
    pub async fn new_init(i2c: &'a mut I2C,address: u8, config: &Config) -> Result<Self, MlxError<E>> {
        let mut inst = Self {
            i2c,
            address
        };

        // Note: writing to the wrong sensor first causes misbehaviour. I think this might be an i2c driver issue, since
        // the bytes on the wire are "shifted" by one so to speak.
        // regardless, doing a read to detect devices instead of a write resolves it.
        let mut cidbuf: [u8; 2] = [0u8; 2];

        inst.read_registers_base(&mut cidbuf[0..1]).await?;

        inst.read_registers(REG_COMPANY_ID, &mut cidbuf).await?;

        if cidbuf[0] != 0x94 && cidbuf[1] != 0xAA {
            return Err(MlxError::WrongDevice)
        }

        inst.write_register(REG_RESET, 0x06).await?;
        
        // wait a bit of a while for it to come out of reset.
        Timer::after_millis(10).await;

        let stat1 = inst.read_register(REG_STAT1).await?;
        if stat1 & (1<<3) == 0 {
            // not reset?
            info!("not reset - something weird happened");
        }

        inst.reconfigure(config).await?;

        Ok(inst)
    }

    pub async fn reconfigure(&mut self, config: &Config) -> Result<(), MlxError<E>> {
        info!("Configuring MLX with: {}", config);
        // stop measurements.
        let ctrl1: u8 = 0; // disable everything.
        self.write_register(REG_CTRL1, ctrl1).await?;

        let mut ctrl2: u8  = 1 << 3; // Interrupt on INTB pin (will use for dready to avoid polling i2c.)
        if config.high_sensitivity {
            ctrl2 |= 1 << 7;  // CONFIG1 = 1 - configuration 2 for high sensitivity
        } else {
            ctrl2 |= 1 << 6; // CONFIG0 = 1 - configuration 1 for high range
        }

        self.write_register(REG_CTRL2, ctrl2).await?;

        let ctrl3:u8 = (1 << 7) | // enable oversampling for hall measurements
            (1 << 6) | // Enable oversampling for temperature measurements
            (6 << 3) | // Enable 4 filter taps for XY hall effect measurement - see https://github.com/zephyrproject-rtos/zephyr/blob/4f3478391a3c5e555dcd629f9e829378c17b0d62/drivers/sensor/melexis/mlx90394/mlx90394.c#L28 - this will take 490 us per axis.
            (6 << 0); // Enable 4 filter taps for temperature measurement.
        self.write_register(REG_CTRL3, ctrl3).await?;

        let ctrl4: u8 = (1 << 7) | // rsvd
            (0<<6) | // rsvd
            (1<<5) | // enable temp measurement
            (1<<4) | // rsvd
            (1<<3) | // drdy output on intb
            (6<<0); // 4 filter taps for Z hall effect measurement.

        self.write_register(REG_CTRL4, ctrl4).await?;

        // start continous measurements.
        let mut ctrl1: u8 = config.rate; // continous measurements.
        if config.en_z {
            ctrl1 |= 1<<6;
        }
        if config.en_x {
            ctrl1 |= 1<<4;
        }
        if config.en_y {
            ctrl1 |= 1<<5;
        }
            
        self.write_register(REG_CTRL1, ctrl1).await?;

        let reg1 = self.read_register(REG_CTRL1).await?;
        info!("register is: {:08x}", reg1);

        Ok(())
    }

    pub async fn data_ready(&mut self) -> Result<bool, MlxError<E>> {
        let mut stat1 = [0x00u8; 1];
        self.read_registers_base(&mut stat1).await?;

        if (stat1[0] & (1<<0)) != 0 {
            return Ok(true);
        } else {
            return Ok(false);
        }
    }

    pub async fn dump_data(&mut self, test: &mut [u8; 7]) -> Result<(), MlxError<E>> {
        self.read_registers(REG_X_LSB, test).await?;

        Ok(())
    }

    pub async fn get_reading(&mut self) -> Result<MagReadings, MlxError<E>> {
        let mut main_data = [0x0u8; 8];
        
        // this data is semi-garbage.
        // I wonder if it's being treated as a "direct read" somehow and starting from 0x00,
        // ^ yep - LA proves that there's a stop, then a delay, then a new start,
        // so the magnetometer misbehaves and doesn't reliably read correct data.
        // I think this is why reading the company ID also didn't always work reliably.
        self.read_registers_base(&mut main_data).await?;

        Ok(MagReadings::from_raw(main_data[1..8].try_into().unwrap()))
    }

    pub async fn get_temperature(&mut self) -> Result<i16, MlxError<E>> {
        let mut data = [0x0u8; 2];
        self.read_registers(REG_T_LSB, &mut data).await?;

        Ok(i16::from_be_bytes(data))
    }

    async fn write_register(&mut self, reg: u8, value: u8) -> Result<(), MlxError<E>> {
        self.i2c.write(self.address, &[reg, value]).await?;
        Ok(())
    }

    async fn read_register(&mut self, reg: u8) -> Result<u8, MlxError<E>> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(self.address, &[reg], &mut buf).await?;
        Ok(buf[0])
    }
    async fn read_registers_base(&mut self, buf: &mut [u8]) -> Result<(), MlxError<E>> {
        self.i2c.read(self.address, buf).await?;
        Ok(())
    }
    async fn read_registers(&mut self, start_reg: u8, buf: &mut [u8]) -> Result<(), MlxError<E>> {
        // note: write_read does a write, then a read, but there's not technically a true _repeated start_
        // which causes many devices to misbehave when reading registers. This is a driver bug in i2c.
        self.i2c.write_read(self.address, &[start_reg], buf).await?;
        Ok(())
    }
}