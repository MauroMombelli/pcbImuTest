#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::sync::atomic::{AtomicU32, Ordering};

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::interrupt;
use embassy_stm32::spi::Spi;
use embassy_stm32::time::Hertz;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};
use {defmt_rtt as _, panic_probe as _};

mod bmi088;
mod bmi270;
mod data;
mod lsm6dsr;

use bmi088::Bmi088;
use bmi270::Bmi270;
use data::Data;
use lsm6dsr::Lsm6;

// E13 -> LED

// acce: i2c, lsm303dlhc
// B6 and B7 -> i2c

// gyro: spi, l3gd20
// A5 A6 A7 -> SPI
// E3 -> CS
struct Imu {
    acce: Data,
    gyro: Data,
}
/*
static LSM6: Signal<ThreadModeRawMutex, Imu> = Signal::new();
static BMI270: Signal<ThreadModeRawMutex, Imu> = Signal::new();
static BMI088: Signal<ThreadModeRawMutex, Imu> = Signal::new();
*/
static SENSORS_DATA: Signal<ThreadModeRawMutex, [Imu; 3]> = Signal::new();

static READ_SENSOR_LOOP_S: AtomicU32 = AtomicU32::new(0);
static CORE_UPDATE: AtomicU32 = AtomicU32::new(0);

//#[embassy_executor::task]
async fn read_sensors(
    mut spi: Spi<
        'static,
        embassy_stm32::peripherals::SPI1,
        embassy_stm32::peripherals::DMA1_CH3,
        embassy_stm32::peripherals::DMA1_CH2,
    >,
    mut bmi088: Bmi088,
    mut bmi270: Bmi270,
    mut lsm6: Lsm6,
) {
    /*
    bmi088
    acce:
    range g: 3 6 12 24
    odr Hz: 12.5 25 50 100 200 400 800 1600
    gyro:
    range deg/s: 125 250 500 1000 2000
    odr Hz: 100 200 400 1000 2000

    bmi270
    acce:
    range g: 2 4 8 16
    odr Hz: 25 50 100 200 400 800 1600
    gyro:
    range deg/s: 125 250 500 1000 2000
    odr Hz: 25 50 100 200 400 800 1600 3200

    lsm6
    acce:
    range g: 2 4 8 16
    odr Hz: 1.6 12.5 26 52 104 208 416 833 1.66 3.33 6.66
    gyro:
    range deg/s: 250 500 1000 2000
    odr Hz: 12.5 26 52 104 208 416 833 1.66 3.33 6.66

     */
    info!("INIT");
    bmi088.init(&mut spi).await;
    bmi270.init(&mut spi).await;
    lsm6.init(&mut spi).await;

    info!("LOOP");

    let mut time_start_next_loop = Instant::now();
    let loop_desired_duration = Duration::from_millis(1);

    let mut count: u32 = 0;

    let output_delay = Duration::from_secs(1);
    let mut next_output = Instant::now() + output_delay;

    loop {
        time_start_next_loop += loop_desired_duration;
        Timer::at(time_start_next_loop).await;

        let data: [Imu; 3] = [
            Imu {
                acce: lsm6.read_acce(&mut spi).await,
                gyro: lsm6.read_gyro(&mut spi).await,
            },
            Imu {
                acce: bmi088.read_acce(&mut spi).await,
                gyro: bmi088.read_gyro(&mut spi).await,
            },
            Imu {
                acce: bmi270.read_acce(&mut spi).await,
                gyro: bmi270.read_gyro(&mut spi).await,
            },
        ];

        for (i, sensor) in data.iter().enumerate() {
            info!(
                "{},{},{},{},{},{},{}",
                i,
                sensor.gyro.data[0],
                sensor.gyro.data[1],
                sensor.gyro.data[2],
                sensor.acce.data[0],
                sensor.acce.data[1],
                sensor.acce.data[2],
            );
        }

        count += 1;

        if Instant::now() >= next_output {
            next_output += output_delay;

            info!("loop/s: {}", count);

            count = 0;
        }
        /*
        SENSORS_DATA.signal(data);
        READ_SENSOR_LOOP_S.fetch_add(1, Ordering::SeqCst);
        */
    }
}

async fn core() {
    /*
        let mut count: u32 = 0;

        let output_delay = Duration::from_secs(1);
        let mut next_output = Instant::now() + output_delay;
    */
    loop {
        let sensors = SENSORS_DATA.wait().await;

        for (i, sensor) in sensors.iter().enumerate() {
            info!(
                "{},{},{},{},{},{},{}",
                i,
                sensor.gyro.data[0],
                sensor.gyro.data[1],
                sensor.gyro.data[2],
                sensor.acce.data[0],
                sensor.acce.data[1],
                sensor.acce.data[2],
            );
        }
        /*
        count += 1;

        if Instant::now() >= next_output {
            next_output += output_delay;

            let read_sensors = READ_SENSOR_LOOP_S.swap(0, Ordering::SeqCst);
            info!("loop/s: {} read_sensors: {}", count, read_sensors);

            count = 0;
        }
        */
    }
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

    let mut clock_config = embassy_stm32::Config::default();
    clock_config.rcc = clock_config_rcc;
    let p = embassy_stm32::init(clock_config);
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

    let bmi270 = Bmi270::new(Output::new(p.PB7, Level::High, Speed::Low));

    let lsm6ds = Lsm6::new(Output::new(p.PB6, Level::High, Speed::Low));

    let bmi088 = Bmi088::new(
        Output::new(p.PB4, Level::High, Speed::Low),
        Output::new(p.PB5, Level::High, Speed::Low),
    );

    // this is the onboard gyro, keep it off
    let _gyro_nss = Output::new(p.PE3, Level::High, Speed::Low);
    let gyro_interrupt = Input::new(p.PE1, Pull::Down);
    let _gyro_interrupt = ExtiInput::new(gyro_interrupt, p.EXTI1);

    /*
    spawner.spawn(read_sensors(spi, bmi088, bmi270, lsm6ds)).unwrap();

    spawner.spawn(core()).unwrap();

    loop {
        Timer::after(Duration::from_secs(1)).await;
        let read_update = READ_SENSOR_LOOP_S.swap(0, Ordering::SeqCst);
        let core_update = CORE_UPDATE.swap(0, Ordering::SeqCst);
        info!("read_sensor/s: {} core/s: {}", read_update, core_update);
    }
    */
    read_sensors(spi, bmi088, bmi270, lsm6ds).await;
}
