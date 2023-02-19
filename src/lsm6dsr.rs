use defmt::*;
use embassy_stm32::gpio::Output;
use embassy_stm32::spi::{Instance, Spi};
use embassy_time::{Duration, Instant, Timer};

use super::data::Data;

const WRITE: u8 = 0 << 7;
const READ: u8 = 1 << 7;

pub struct Lsm6 {
    cs: Output<'static, embassy_stm32::peripherals::PB6>,
}

impl Lsm6 {
    pub fn new(_cs: Output<'static, embassy_stm32::peripherals::PB6>) -> Lsm6 {
        Self { cs: _cs }
    }

    pub async fn init<T: Instance, Tx, Rx>(&mut self, spi: &mut Spi<'_, T, Tx, Rx>) {
        // 0x0F CHIP_ID == 0x6B
        let mut buf: [u8; 2] = [0x0F | READ, 0xFF];
        self.cs.set_low();
        spi.blocking_transfer_in_place(&mut buf).ok();
        self.cs.set_high();
        info!("Lsm6 WHOAMI {} (107)", buf[1]);

        // acce 833Hz, +-4g, 1st stage filter
        let buf: [u8; 2] = [0x10 | WRITE, 0x78];
        self.cs.set_low();
        spi.blocking_write(&buf).ok();
        self.cs.set_high();

        // gyro 833Hz, +-500dps
        let buf: [u8; 2] = [0x11 | WRITE, 0x74];
        self.cs.set_low();
        spi.blocking_write(&buf).ok();
        self.cs.set_high();
    }

    pub async fn read_acce<T: Instance, Tx, Rx>(&mut self, spi: &mut Spi<'_, T, Tx, Rx>) -> Data {
        let mut read: [u8; 7] = [0x28 | READ, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
        self.cs.set_low();
        spi.blocking_transfer_in_place(&mut read).ok();
        self.cs.set_high();

        Data {
            data: [
                (read[1] as u16 + (read[2] as u16 * 256)) as i16,
                (read[3] as u16 + (read[4] as u16 * 256)) as i16,
                (read[5] as u16 + (read[6] as u16 * 256)) as i16,
            ],
        }
    }

    pub async fn read_gyro<T: Instance, Tx, Rx>(&mut self, spi: &mut Spi<'_, T, Tx, Rx>) -> Data {
        let mut read: [u8; 7] = [0x22 | READ, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
        self.cs.set_low();
        spi.blocking_transfer_in_place(&mut read).ok();
        self.cs.set_high();

        Data {
            data: [
                (read[1] as u16 + (read[2] as u16 * 256)) as i16,
                (read[3] as u16 + (read[4] as u16 * 256)) as i16,
                (read[5] as u16 + (read[6] as u16 * 256)) as i16,
            ],
        }
    }
}
