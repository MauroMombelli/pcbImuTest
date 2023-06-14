#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::sync::atomic::{AtomicU32, Ordering};

use ahrs::{Ahrs, Madgwick};
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::spi::Spi;
use embassy_stm32::time::Hertz;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};
use nalgebra::Vector3;
use {defmt_rtt as _, panic_probe as _};

//mod drivers::bmi088;
//mod drivers::bmi270;
//mod drivers::lsm6dsr;
mod actuators;
mod sensors;

use actuators::stepper;
use sensors::data::Imu;
use sensors::{bmi088, bmi270, lsm6dsr};

// E13 -> LED

// acce: i2c, lsm303dlhc
// B6 and B7 -> i2c

// gyro: spi, l3gd20
// A5 A6 A7 -> SPI
// E3 -> CS

static SENSORS_DATA: Signal<ThreadModeRawMutex, [Imu; 3]> = Signal::new();

static READ_SENSOR_LOOP_S: AtomicU32 = AtomicU32::new(0);
static CORE_UPDATE: AtomicU32 = AtomicU32::new(0);
static SLEEP_UPDATE: AtomicU32 = AtomicU32::new(0);

#[embassy_executor::task]
async fn read_sensors(
    mut spi: Spi<
        'static,
        embassy_stm32::peripherals::SPI1,
        embassy_stm32::peripherals::DMA1_CH3,
        embassy_stm32::peripherals::DMA1_CH2,
    >,
    mut bmi088: bmi088::Bmi088,
    mut bmi270: bmi270::Bmi270,
    mut lsm6: lsm6dsr::Lsm6,
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

    loop {
        time_start_next_loop += loop_desired_duration;
        Timer::at(time_start_next_loop).await;

        let mut data: [Imu; 3] = [
            lsm6.read_imu(&mut spi).await,
            Imu {
                acce: bmi088.read_acce(&mut spi).await,
                gyro: bmi088.read_gyro(&mut spi).await,
            },
            bmi270.read_imu(&mut spi).await,
        ];

        //from sensor axis to body axis lsm6
        // no changes

        //from sensor axis to body axis bmi088
        let tmp = data[1].acce.data[0];
        data[1].acce.data[0] = -data[1].acce.data[1];
        data[1].acce.data[1] = tmp;
        let tmp = data[1].gyro.data[0];
        data[1].gyro.data[0] = -data[1].gyro.data[1];
        data[1].gyro.data[1] = tmp;

        //from sensor axis to body axis
        data[2].acce.data[0] *= -1.0;
        data[2].acce.data[1] *= -1.0;
        data[2].gyro.data[0] *= -1.0;
        data[2].gyro.data[1] *= -1.0;

        SENSORS_DATA.signal(data);
        READ_SENSOR_LOOP_S.fetch_add(1, Ordering::SeqCst);
    }
}

#[embassy_executor::task]
async fn flight_controller() {
    let output_delay = Duration::from_millis(100);
    let mut next_output = Instant::now() + output_delay;

    let mut ahrs = [
        Madgwick::new((1.0f32) / (256.0), 0.1f32),
        Madgwick::new((1.0f32) / (256.0), 0.1f32),
        Madgwick::new((1.0f32) / (256.0), 0.1f32),
    ];

    let mut stupid_int_x: [f32; 3] = [0.0, 0.0, 0.0];
    let mut stupid_int_y: [f32; 3] = [0.0, 0.0, 0.0];
    let mut stupid_int_z: [f32; 3] = [0.0, 0.0, 0.0];
    loop {
        let sensors = SENSORS_DATA.wait().await;

        for (i, sensor) in sensors.iter().enumerate() {
            let gyroscope = Vector3::new(sensor.gyro.data[0], sensor.gyro.data[1], sensor.gyro.data[2]);
            let accelerometer = Vector3::new(sensor.acce.data[0], sensor.acce.data[1], sensor.acce.data[2]);
            let quat = ahrs[i].update_imu(&gyroscope, &accelerometer);
            /* if Instant::now() >= next_output {
                if let Some(quat) = quat.ok() {
                    let (roll, pitch, yaw) = quat.euler_angles();
                    println!("sensor={} pitch={}, roll={}, yaw={}", i, pitch, roll, yaw);
                }
            } */
            stupid_int_x[i] += sensor.gyro.data[0];
            stupid_int_y[i] += sensor.gyro.data[1];
            stupid_int_z[i] += sensor.gyro.data[2];
        }

        if Instant::now() >= next_output {
            next_output += output_delay;
            for (i, sensor) in sensors.iter().enumerate() {
                println!(
                    "sensor={} ({}, {}, {}) ({}, {}, {})",
                    i,
                    sensor.gyro.data[0],
                    sensor.gyro.data[1],
                    sensor.gyro.data[2],
                    sensor.acce.data[0],
                    sensor.acce.data[1],
                    sensor.acce.data[2],
                );
            }
        }

        CORE_UPDATE.fetch_add(1, Ordering::SeqCst);
    }
}

#[embassy_executor::task]
async fn free_cpu_count() {
    loop {
        embassy_futures::yield_now().await;
        for i in 1..100 {
            SLEEP_UPDATE.fetch_add(1, Ordering::SeqCst);
        }
    }
}

#[embassy_executor::task]
async fn run_stepper(mut stepper: stepper::Stepper) {
    loop {
        stepper.step();
        Timer::after(Duration::from_millis(10)).await;
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

    let bmi270 = bmi270::Bmi270::new(Output::new(p.PB7, Level::High, Speed::Low));

    let lsm6ds = lsm6dsr::Lsm6::new(Output::new(p.PB6, Level::High, Speed::Low));

    let bmi088 = bmi088::Bmi088::new(
        Output::new(p.PB4, Level::High, Speed::Low),
        Output::new(p.PB5, Level::High, Speed::Low),
    );

    // this is the onboard gyro, keep it off
    let _gyro_nss = Output::new(p.PE3, Level::High, Speed::Low);
    let gyro_interrupt = Input::new(p.PE1, Pull::Down);
    let _gyro_interrupt = ExtiInput::new(gyro_interrupt, p.EXTI1);
    /*
        let stepper = stepper::Stepper::new(
            2038,
            Output::new(p.PD11, Level::Low, Speed::Low).degrade(),
            Output::new(p.PB13, Level::Low, Speed::Low).degrade(),
            Output::new(p.PB14, Level::Low, Speed::Low).degrade(),
            Output::new(p.PB12, Level::Low, Speed::Low).degrade(),
        );

        spawner.spawn(run_stepper(stepper)).unwrap();
    */

    spawner.spawn(free_cpu_count()).unwrap();

    spawner.spawn(flight_controller()).unwrap();

    spawner.spawn(read_sensors(spi, bmi088, bmi270, lsm6ds)).unwrap();

    loop {
        Timer::after(Duration::from_secs(1)).await;
        let read_update = READ_SENSOR_LOOP_S.swap(0, Ordering::SeqCst);
        let core_update = CORE_UPDATE.swap(0, Ordering::SeqCst);
        let sleep_update = SLEEP_UPDATE.swap(0, Ordering::SeqCst);
        info!(
            "read_sensor/s: {} core/s: {} sleep_update/s: {}",
            read_update, core_update, sleep_update
        );
    }

    /* let output_delay = Duration::from_secs(1);
    let mut next_output = Instant::now() + output_delay;
    let mut count = 0;
    loop {
        count += 1;

        if Instant::now() >= next_output {
            next_output += output_delay;
            info!("loop/s: {}", CORE_UPDATE.swap(0, Ordering::SeqCst));
            count = 0;
        }
    } */
}
