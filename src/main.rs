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
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};
use {defmt_rtt as _, panic_probe as _};

mod bmi088;
mod data;

use bmi088::Bmi088;
use data::Data;

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
    mut bmi088: Bmi088,
    //mut gyro_cs: Output<'static, embassy_stm32::peripherals::PB4>,
    //mut acce_cs: Output<'static, embassy_stm32::peripherals::PB5>,
) {
    bmi088.init(&mut spi);

    loop {
        Timer::after(Duration::from_millis(1)).await;

        let data = bmi088.read_acce(&mut spi).await;
        ACCE_UPDATE.fetch_add(1, Ordering::SeqCst);
        DATA_ACCE.signal(data);

        let data = bmi088.read_gyro(&mut spi).await;

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

struct Lsm {
    cs: Output<'static, embassy_stm32::peripherals::PB6>,
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut clock_config_rcc = embassy_stm32::rcc::Config::default();
    clock_config_rcc.hse = Some(Hertz(8_000_000));
    clock_config_rcc.bypass_hse = false;
    clock_config_rcc.sysclk = Some(Hertz(72_000_000));
    clock_config_rcc.hclk = Some(Hertz(72_000_000));
    clock_config_rcc.pclk1 = Some(Hertz(36_000_000));
    clock_config_rcc.pclk2 = Some(Hertz(72_000_000));
    clock_config_rcc.pll48 = false;

    let mut clockConfig = embassy_stm32::Config::default();
    clockConfig.rcc = clock_config_rcc;
    let p = embassy_stm32::init(clockConfig);
    info!("Hello World!");

    let mut spi = Spi::new(
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

    let lsm_cs = Output::new(p.PB6, Level::High, Speed::Low);

    /*
        let bmi088_acce_cs = Output::new(p.PB5, Level::High, Speed::Low);
        let bmi088_gyro_cs = Output::new(p.PB4, Level::High, Speed::Low);
    */
    let bmi088 = Bmi088::new(
        Output::new(p.PB4, Level::High, Speed::Low),
        Output::new(p.PB5, Level::High, Speed::Low),
    );

    // this is the onboard gyro
    let _gyro_nss = Output::new(p.PE3, Level::High, Speed::Low);
    let gyro_interrupt = Input::new(p.PE1, Pull::Down);
    let gyro_interrupt = ExtiInput::new(gyro_interrupt, p.EXTI1);

    spawner.spawn(read_sensors(spi, bmi088)).unwrap();
    // spawner.spawn(read_gyro(spi, _gyro_nss, gyro_interrupt)).unwrap();
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
