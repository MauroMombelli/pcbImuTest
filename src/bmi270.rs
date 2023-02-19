use defmt::info;
use embassy_stm32::gpio::Output;
use embassy_stm32::spi::{Instance, Spi};
use embassy_time::{Duration, Instant, Timer};

use super::data::Data;

const WRITE: u8 = 0 << 7;
const READ: u8 = 1 << 7;

pub struct Bmi270 {
    cs: Output<'static, embassy_stm32::peripherals::PB7>,
}

impl Bmi270 {
    pub fn new(_cs: Output<'static, embassy_stm32::peripherals::PB7>) -> Bmi088 {
        Self {
            cs: _cs;
        }
    }

    pub async fn init<T: Instance, Tx, Rx>(&mut self, spi: &mut Spi<'_, T, Tx, Rx>) {
        //// STARTUP BMI088
        // as per DS, BMI088 acce need a CS rising edge
        // also gyro is in normal mode, acce in suspend.
        /* WARNING
            - In case of read operations, the SPI interface of the accelerometer part does not send the
        requested information directly after the master has send the corresponding register address,
        but sends a dummy byte first, whose content is not predictable. Only after this dummy byte the
        desired content is sent

            - multiple read does not require nothing special

        */

        let buf: [u8; 2] = [0x7E | WRITE, 0xB6]; // reset
        self.acce_cs.set_low();
        spi.blocking_write(&buf).ok();
        self.acce_cs.set_high();

        let buf: [u8; 2] = [0x14 | WRITE, 0xB6]; // reset
        self.gyro_cs.set_low();
        spi.blocking_write(&buf).ok();
        self.gyro_cs.set_high();

        Timer::after(Duration::from_millis(1)).await;

        // 0x00 ACC_CHIP_ID == 0x1E
        let mut buf: [u8; 3] = [0x00 | READ, 0xFF, 0xFF]; // turn on
        self.acce_cs.set_low();
        spi.blocking_transfer_in_place(&mut buf).ok();
        self.acce_cs.set_high();
        info!("acce WHOAMI {}", buf[2]);

        // 0x03: ACC_STATUS bit7 == data ready, 6:0 reserved

        // 0x40: ACC_CONF 7:4 badwith, keep default 0x0A 3:0 ODR 0x0B = 800Hz 0x0C = 1600Hz

        // 0x41: ACC_RANGE 1:0 0x00 = +-3g 0x01 +-6g (default)

        // 0x58: INT1_INT2_MAP_DATA bit 6 drdy on INT2, bit 2 drdy on INT1

        // 0x7C: ACC_PWR_CONF 0x00 = active, 0x03 suspend (default)
        /*
            let buf: [u8; 2] = [0x7C | WRITE, 0x00]; // activate
            acce_cs.set_low();
            spi.blocking_write(&buf).ok();
            acce_cs.set_high();
        */
        let mut buf: [u8; 3] = [0x7C | READ, 0xFF, 0xFF];
        self.acce_cs.set_low();
        spi.blocking_transfer_in_place(&mut buf).ok();
        self.acce_cs.set_high();
        info!("acce ACC_PWR_CONF {} should be 0", buf[2]);

        // 0x7D: ACC_PWR_CTRL 0x00 off (default) 0x04 on
        let buf: [u8; 2] = [0x7D | WRITE, 0x04]; // turn on
        self.acce_cs.set_low();
        spi.blocking_write(&buf).ok();
        self.acce_cs.set_high();

        Timer::after(Duration::from_millis(1)).await;

        let mut buf: [u8; 3] = [0x7D | READ, 0xFF, 0xFF];
        self.acce_cs.set_low();
        spi.blocking_transfer_in_place(&mut buf).ok();
        self.acce_cs.set_high();
        info!("acce ACC_PWR_CTRL {} should be 4", buf[2]);

        Timer::after(Duration::from_millis(100)).await;
        let mut buf: [u8; 3] = [0x00 | READ, 0xFF, 0xFF]; // turn on
        self.acce_cs.set_low();
        spi.blocking_transfer_in_place(&mut buf).ok();
        self.acce_cs.set_high();
        info!("acce WHOAMI {}", buf[2]);

        Timer::after(Duration::from_millis(100)).await;

        // 0x00: GYRO_CHIP_ID = 0x0F

        let mut buf: [u8; 2] = [0x00 | READ, 0x00]; // turn on
        self.gyro_cs.set_low();
        spi.blocking_transfer_in_place(&mut buf).ok();
        self.gyro_cs.set_high();
        info!("gyro WHOAMI {}", buf[1]);

        // 0x02 – 0x07: Rate data

        // 0x10: GYRO_BANDWIDTH 0x00 2000Hz 532Hz BW (default) 0x02 1000Hz 116Hz BW

        // 0x15: GYRO_INT_CTRL bit7 enable data ready int

        // 0x0F: GYRO_RANGE 0x00 +-2000deg/s (default) 0x02 = +-500 deg/s
    }

    pub async fn read_acce<T: Instance, Tx, Rx>(&mut self, spi: &mut Spi<'_, T, Tx, Rx>) -> Data {
        // 0x12 – 0x17: ACC data, X, Y, Z
        /*
        Accel_X_int16 = ACC_X_MSB * 256 + ACC_X_LSB
        Accel_Y_int16 = ACC_Y_MSB * 256 + ACC_Y_LSB
        Accel_Z_int16 = ACC_Z_MSB * 256 + ACC_Z_LSB

        Accel_X_in_mg = Accel_X_int16 / 32768 * 1000 * 2^(<0x41> + 1) * 1.5
        Accel_Y_in_mg = Accel_Y_int16 / 32768 * 1000 * 2^(<0x41> + 1) * 1.5
        Accel_Z_in_mg = Accel_Z_int16 / 32768 * 1000 * 2^(<0x41> + 1) * 1.5
        */
        let mut read: [u8; 8] = [0x12 | READ, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
        self.acce_cs.set_low();
        spi.blocking_transfer_in_place(&mut read).ok();
        self.acce_cs.set_high();

        Data {
            data: [
                (read[2] as u16 + (read[3] as u16 * 256)) as i16,
                (read[4] as u16 + (read[5] as u16 * 256)) as i16,
                (read[6] as u16 + (read[7] as u16 * 256)) as i16,
            ],
        }
    }

    pub async fn read_gyro<T: Instance, Tx, Rx>(&mut self, spi: &mut Spi<'_, T, Tx, Rx>) -> Data {
        // 0x02 – 0x07: Rate data X, Y, Z
        /*
        Rate_X: RATE_X_MSB * 256 + RATE_X_LSB
        Rate_Y: RATE_Y_MSB * 256 + RATE_Y_LSB
        Rate_Z: RATE_Z_MSB * 256 + RATE_Z_LSB
         */
        let mut read: [u8; 7] = [0x02 | READ, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
        self.gyro_cs.set_low();
        spi.blocking_transfer_in_place(&mut read).ok();
        self.gyro_cs.set_high();

        Data {
            data: [
                (read[1] as u16 + (read[2] as u16 * 256)) as i16,
                (read[3] as u16 + (read[4] as u16 * 256)) as i16,
                (read[5] as u16 + (read[6] as u16 * 256)) as i16,
            ],
        }
    }
}
