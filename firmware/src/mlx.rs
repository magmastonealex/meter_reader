use defmt::info;
use embedded_hal_async::i2c::{I2c, ErrorKind, Error as I2cError};

pub struct Mlx90394<I2C> {
    i2c: I2C,
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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


impl<I2C, E> Mlx90394<I2C>
where
    I2C: I2c<Error = E>,
{
    pub async fn new_init(i2c: I2C, address: u8) -> Result<Self, MlxError<E>> {
        let mut inst = Self {
            i2c,
            address
        };

        let mut cidbuf: [u8; 2] = [0u8; 2];

        inst.read_registers(REG_COMPANY_ID, &mut cidbuf).await?;

        if cidbuf[0] != 0x94 && cidbuf[1] != 0xAA {
            return Err(MlxError::WrongDevice)
        }

        inst.write_register(REG_RESET, 0x06).await?;


        let stat1 = inst.read_register(REG_STAT1).await?;
        if stat1 & (1<<3) == 0 {
            // not reset?
            info!("not reset - something weird happened");
        }

        let ctrl2:u8  = (1 << 6) | // CONFIG0 = 1 - configuration 1 for high range
            (1 << 3); // Interrupt on INTB pin (will use for dready to avoid polling i2c.)
        
        inst.write_register(REG_CTRL2, ctrl2).await?;


        let ctrl3:u8 = (1 << 7) | // enable oversampling for hall measurements
            (1 << 6) | // Enable oversampling for temperature measurements
            (6 << 3) | // Enable 4 filter taps for XY hall effect measurement - see https://github.com/zephyrproject-rtos/zephyr/blob/4f3478391a3c5e555dcd629f9e829378c17b0d62/drivers/sensor/melexis/mlx90394/mlx90394.c#L28 - this will take 490 us per axis.
            (6 << 0); // Enable 4 filter taps for temperature measurement.
        inst.write_register(REG_CTRL3, ctrl3).await?;

        let ctrl4: u8 = (1 << 7) | // rsvd
            (0<<6) | // rsvd
            (1<<5) | // enable temp measurement
            (1<<4) | // rsvd
            (1<<3) | // drdy output on intb
            (6<<0); // 4 filter taps for Z hall effect measurement.

        inst.write_register(REG_CTRL4, ctrl4).await?;

        // start continous measurements.
        let ctrl1: u8 = (1<<6) | // z axis enabled
            (1<<5) | // y axis enabled
            (1<<4) | // x axis enabled
            (2<<0); // 5 Hz continous measurements.
        inst.write_register(REG_CTRL1, ctrl1).await?;
        

        Ok(inst)
    }

    pub async fn data_ready(&mut self) -> Result<bool, MlxError<E>> {
        let stat1 = self.read_register(REG_STAT1).await?;
        if (stat1 & (1<<0)) != 0 {
            return Ok(true);
        } else {
            return Ok(false);
        }
    }

    pub async fn dump_data(&mut self, test: &mut [u8; 7]) -> Result<(), MlxError<E>> {
        self.read_registers(REG_X_LSB, test).await?;

        Ok(())
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

    async fn read_registers(&mut self, start_reg: u8, buf: &mut [u8]) -> Result<(), MlxError<E>> {
        self.i2c.write_read(self.address, &[start_reg], buf).await?;
        Ok(())
    }
}