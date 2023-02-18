#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::sync::atomic::{AtomicU32, Ordering};

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::interrupt;
use embassy_stm32::spi::Spi;
use embassy_stm32::time::Hertz;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};
use {defmt_rtt as _, panic_probe as _};

struct Data {
    data: [i16; 3],
}

static DATA_ACCE: Signal<ThreadModeRawMutex, Data> = Signal::new();
static DATA_GYRO: Signal<ThreadModeRawMutex, Data> = Signal::new();

// E13 -> LED

// acce: i2c, lsm303dlhc
// B6 and B7 -> i2c

// gyro: spi, l3gd20
// A5 A6 A7 -> SPI
// E3 -> CS

static GYRO_UPDATE: AtomicU32 = AtomicU32::new(0);
static ACCE_UPDATE: AtomicU32 = AtomicU32::new(0);
static CORE_UPDATE: AtomicU32 = AtomicU32::new(0);

static ASD: AtomicU32 = AtomicU32::new(0);

#[embassy_executor::task]
async fn read_sensors(
    mut spi: Spi<
        'static,
        embassy_stm32::peripherals::SPI1,
        embassy_stm32::peripherals::DMA1_CH3,
        embassy_stm32::peripherals::DMA1_CH2,
    >,
    mut acce_cs: Output<'static, embassy_stm32::peripherals::PE5>,
    mut gyro_cs: Output<'static, embassy_stm32::peripherals::PE4>,
    mut interrupt: ExtiInput<'static, embassy_stm32::peripherals::PE1>,
) {
    gyro_cs.set_high();
    acce_cs.set_high();

    Timer::after(Duration::from_millis(100)).await;

    const READ: u8 = 1 << 7;
    const WRITE: u8 = 0 << 7;
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

    // ACC_CHIP_ID == 0x1E
    let mut buf: [u8; 2] = [0x1E | READ, 0xFF]; // turn on
    acce_cs.set_low();
    spi.blocking_read(&mut buf).ok();
    acce_cs.set_high();

    // 0x03: ACC_STATUS bit7 == data ready, 6:0 reserved

    // 0x40: ACC_CONF 7:4 badwith, keep default 0x0A 3:0 ODR 0x0B = 800Hz 0x0C = 1600Hz

    // 0x41: ACC_RANGE 1:0 0x00 = +-3g 0x01 +-6g (default)

    // 0x58: INT1_INT2_MAP_DATA bit 6 drdy on INT2, bit 2 drdy on INT1

    // 0x7D: ACC_PWR_CTRL 0x00 off (default) 0x04 on
    let buf: [u8; 2] = [0x7D | WRITE, 0x04]; // turn on
    acce_cs.set_low();
    spi.blocking_write(&buf).ok();
    acce_cs.set_high();

    // 0x7C: ACC_PWR_CONF 0x00 = active, 0x03 suspend (default)
    let buf: [u8; 2] = [0x7C | WRITE, 0x00]; // activate
    acce_cs.set_low();
    spi.blocking_write(&buf).ok();
    acce_cs.set_high();

    Timer::after(Duration::from_millis(100)).await;
    let mut buf: [u8; 2] = [0x1E | READ, 0xFF]; // turn on
    acce_cs.set_low();
    spi.blocking_read(&mut buf).ok();
    acce_cs.set_high();
    info!("acce WHOAMI {}", buf[1]);

    // 0x00: GYRO_CHIP_ID = 0x0F

    let mut buf: [u8; 2] = [0x0F | READ, 0xFF]; // turn on
    gyro_cs.set_low();
    spi.blocking_read(&mut buf).ok();
    gyro_cs.set_high();
    info!("gyro WHOAMI {}", buf[1]);

    // 0x02 – 0x07: Rate data

    // 0x10: GYRO_BANDWIDTH 0x00 2000Hz 532Hz BW (default) 0x02 1000Hz 116Hz BW

    // 0x15: GYRO_INT_CTRL bit7 enable data ready int

    // 0x0F: GYRO_RANGE 0x00 +-2000deg/s (default) 0x02 = +-500 deg/s

    loop {
        Timer::after(Duration::from_millis(1)).await;
        // 0x12 – 0x17: ACC data, MSB_Z, LSB_Z, Y, X
        /*
        Accel_X_int16 = ACC_X_MSB * 256 + ACC_X_LSB
        Accel_Y_int16 = ACC_Y_MSB * 256 + ACC_Y_LSB
        Accel_Z_int16 = ACC_Z_MSB * 256 + ACC_Z_LSB

        Accel_X_in_mg = Accel_X_int16 / 32768 * 1000 * 2^(<0x41> + 1) * 1.5
        Accel_Y_in_mg = Accel_Y_int16 / 32768 * 1000 * 2^(<0x41> + 1) * 1.5
        Accel_Z_in_mg = Accel_Z_int16 / 32768 * 1000 * 2^(<0x41> + 1) * 1.5
        */
        let mut read: [u8; 8] = [0x12 | READ, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
        acce_cs.set_low();
        spi.blocking_transfer_in_place(&mut read).ok();
        acce_cs.set_high();

        let data = Data {
            data: [
                (read[4] as u16 + ((read[5] as u16) << 8)) as i16,
                (read[6] as u16 + ((read[7] as u16) << 8)) as i16,
                (read[2] as u16 + ((read[3] as u16) << 8)) as i16,
            ],
        };
        ACCE_UPDATE.fetch_add(1, Ordering::SeqCst);
        DATA_ACCE.signal(data);

        // 0x02 – 0x07: Rate data Z, Y, X
        /*
        Rate_X: RATE_X_MSB * 256 + RATE_X_LSB
        Rate_Y: RATE_Y_MSB * 256 + RATE_Y_LSB
        Rate_Z: RATE_Z_MSB * 256 + RATE_Z_LSB
         */

        let mut read: [u8; 7] = [0x02 | READ, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
        gyro_cs.set_low();
        spi.blocking_transfer_in_place(&mut read).ok();
        gyro_cs.set_high();

        let data = Data {
            data: [
                (read[5] as u16 + ((read[6] as u16) << 8)) as i16,
                (read[1] as u16 + ((read[2] as u16) << 8)) as i16,
                (read[3] as u16 + ((read[4] as u16) << 8)) as i16,
            ],
        };
        GYRO_UPDATE.fetch_add(1, Ordering::SeqCst);
        DATA_GYRO.signal(data);
    }
}

#[embassy_executor::task]
async fn read_gyro(
    mut spi: Spi<
        'static,
        embassy_stm32::peripherals::SPI1,
        embassy_stm32::peripherals::DMA1_CH3,
        embassy_stm32::peripherals::DMA1_CH2,
    >,
    mut nss: Output<'static, embassy_stm32::peripherals::PE3>,
    mut interrupt: ExtiInput<'static, embassy_stm32::peripherals::PE1>,
) {
    nss.set_high();

    const READ: u8 = 1 << 7;
    const WRITE: u8 = 0 << 7;
    const MULTI: u8 = 1 << 6;
    const SINGLE: u8 = 0 << 6;

    nss.set_low();
    let mut buf_id_spi: [u8; 2] = [0x0F | SINGLE | READ, 0x00];
    spi.blocking_transfer_in_place(&mut buf_id_spi).ok();
    nss.set_high();

    nss.set_low();
    let spi_gyro_enable_all: [u8; 2] = [0x20 | SINGLE | WRITE, 0xFF]; // 760Hz, 100 cut off, enable all
    spi.blocking_write(&spi_gyro_enable_all).ok();
    nss.set_high();

    nss.set_low();
    let spi_gyro_enable_all: [u8; 2] = [0x22 | SINGLE | WRITE, 0x08]; // enable data ready on DRDY/INT2
    spi.blocking_write(&spi_gyro_enable_all).ok();
    nss.set_high();

    nss.set_low();
    let spi_gyro_enable_all: [u8; 2] = [0x23 | SINGLE | WRITE, 0x10]; // continoous update, LSB, 500dps, 4 wire spi
    spi.blocking_write(&spi_gyro_enable_all).ok();
    nss.set_high();

    loop {
        interrupt.wait_for_high().await;
        let write = [0x28 | MULTI | READ, 0, 0, 0, 0, 0, 0];
        let mut read = [0; 7];
        nss.set_low();
        spi.transfer(&mut read, &write).await.ok();
        nss.set_high();

        GYRO_UPDATE.fetch_add(1, Ordering::SeqCst);

        let data = Data {
            data: [
                (read[1] as u16 + ((read[2] as u16) << 8)) as i16,
                (read[3] as u16 + ((read[4] as u16) << 8)) as i16,
                (read[5] as u16 + ((read[6] as u16) << 8)) as i16,
            ],
        };

        DATA_GYRO.signal(data);
    }
}

#[embassy_executor::task]
async fn read_acce(
    mut i2c: I2c<
        'static,
        embassy_stm32::peripherals::I2C1,
        embassy_stm32::peripherals::DMA1_CH6,
        embassy_stm32::peripherals::DMA1_CH7,
    >,
    mut interrupt: ExtiInput<'static, embassy_stm32::peripherals::PE4>,
) {
    let addr = 0x19;
    //accelerometer, enable all axis, low power mode, 400Hz
    let buf = [0x20, 0x77];
    i2c.blocking_write(addr, &buf);

    let buf = [0x22, 0x10]; //drdy int1
    i2c.blocking_write(addr, &buf);

    let request: [u8; 1] = [0x28 + 0x80]; // 0x80 mean multiread
    let mut read: [u8; 6] = [0; 6];
    loop {
        interrupt.wait_for_high().await;
        //i2c.write_read(addr, &request, &mut read).await.ok();
        let acce_read_future = i2c.write_read(addr, &request, &mut read);

        if embassy_time::with_timeout(Duration::from_secs(1), acce_read_future)
            .await
            .is_err()
        {
            info!("acce read timeout");
        }
        ACCE_UPDATE.fetch_add(1, Ordering::SeqCst);

        let data = Data {
            data: [
                (read[0] as u16 + ((read[1] as u16) << 8)) as i16,
                (read[2] as u16 + ((read[3] as u16) << 8)) as i16,
                (read[4] as u16 + ((read[5] as u16) << 8)) as i16,
            ],
        };

        DATA_ACCE.signal(data);
    }
}

#[embassy_executor::task]
async fn core() {
    let mut time_start_next_loop = Instant::now();
    let loop_desired_duration = Duration::from_millis(1);
    loop {
        if DATA_ACCE.signaled() && DATA_GYRO.signaled() {
            let acce = DATA_ACCE.wait().await;
            let gyro = DATA_GYRO.wait().await;

            if ASD.swap(0, Ordering::SeqCst) == 1 {
                info!("gyro: ({},{},{})", gyro.data[0], gyro.data[1], gyro.data[2]);
                info!("acce: ({},{},{})", acce.data[0], acce.data[1], acce.data[2]);
            }
        }

        CORE_UPDATE.fetch_add(1, Ordering::SeqCst);

        time_start_next_loop += loop_desired_duration;
        Timer::at(time_start_next_loop).await;
        //embassy_futures::yield_now().await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut clockConfigRcc = embassy_stm32::rcc::Config::default();
    clockConfigRcc.hse = Some(Hertz(8_000_000));
    clockConfigRcc.bypass_hse = false;
    clockConfigRcc.sysclk = Some(Hertz(72_000_000));
    clockConfigRcc.hclk = Some(Hertz(72_000_000));
    clockConfigRcc.pclk1 = Some(Hertz(36_000_000));
    clockConfigRcc.pclk2 = Some(Hertz(72_000_000));
    clockConfigRcc.pll48 = false;

    let mut clockConfig = embassy_stm32::Config::default();
    clockConfig.rcc = clockConfigRcc;
    let p = embassy_stm32::init(clockConfig);
    info!("Hello World!");

    let spi = Spi::new(
        p.SPI1,
        p.PA5,
        p.PA7,
        p.PA6,
        p.DMA1_CH3,
        p.DMA1_CH2,
        Hertz(10_000_000),
        embassy_stm32::spi::Config::default(),
    );
    //PE3 for onboard sensor

    let mut bmi270_cs = Output::new(p.PB7, Level::High, Speed::Low);
    let mut lsm_cs = Output::new(p.PB6, Level::High, Speed::Low);
    bmi270_cs.set_high();
    lsm_cs.set_high();

    let acce_cs = Output::new(p.PE5, Level::High, Speed::Low);
    let gyro_cs = Output::new(p.PE4, Level::High, Speed::Low);
    let gyro_interrupt = Input::new(p.PE1, Pull::Down);
    let gyro_interrupt = ExtiInput::new(gyro_interrupt, p.EXTI1);

    spawner
        .spawn(read_sensors(spi, acce_cs, gyro_cs, gyro_interrupt))
        .unwrap();
    /*
        let irq = interrupt::take!(I2C1_EV);
        let i2c = I2c::new(
            p.I2C1,
            p.PB6,
            p.PB7,
            irq,
            p.DMA1_CH6,
            p.DMA1_CH7,
            Hertz(300_000),
            Default::default(),
        );
    */
    //let acce_interrupt = Input::new(p.PE4, Pull::Down);
    //let acce_interrupt = ExtiInput::new(acce_interrupt, p.EXTI4);

    //spawner.spawn(read_acce(i2c, acce_interrupt)).unwrap();

    spawner.spawn(core()).unwrap();

    loop {
        Timer::after(Duration::from_secs(1)).await;
        let gyro_update = GYRO_UPDATE.swap(0, Ordering::SeqCst);
        let acce_update = ACCE_UPDATE.swap(0, Ordering::SeqCst);
        let core_update = CORE_UPDATE.swap(0, Ordering::SeqCst);
        info!(
            "gyro/s: {} acce/s: {} core/s: {}",
            gyro_update, acce_update, core_update
        );
        ASD.store(1, Ordering::SeqCst);
    }
}
