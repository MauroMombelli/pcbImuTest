#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m;
use cortex_m_rt::{entry, exception};

use cortex_m_semihosting::hprintln;

use stm32_hal2::{
    clocks::Clocks,
    gpio::{Pin, Port, PinMode, OutputType},
    i2c::I2c,
    spi::Spi,
    pac,
};
use stm32_hal2::clocks::*;

// monitor arm semihosting enable

// E13 -> LED

// acce: i2c, lsm303dlhc
// B6 and B7 -> i2c

// gyro: spi, l3gd20
// A5 A6 A7 -> SPI
// E3 -> CS

static SYSTICK_RELOAD: AtomicU32 = AtomicU32::new(0);

#[entry]
fn main() -> ! {
    // Set up CPU peripherals
    let mut cp = cortex_m::Peripherals::take().unwrap();

    // Set up microcontroller peripherals
    let dp = pac::Peripherals::take().unwrap();

    let clock_cfg = Clocks{
        input_src: InputSrc::Pll(PllSrc::Hse(8_000_000)), //8MHz external clock
        prediv: Prediv::Div1,
        pll_mul: PllMul::Mul9,
        usb_pre: UsbPrescaler::Div1_5,
        hclk_prescaler: HclkPrescaler::Div1,
        apb1_prescaler: ApbPrescaler::Div2,
        apb2_prescaler: ApbPrescaler::Div1,
        hse_bypass: false,
        security_system: false,
    };

    clock_cfg.setup().unwrap();

    cp.SYST.set_clock_source(cortex_m::peripheral::syst::SystClkSource::External);
    cp.SYST.set_reload(8_000-1); //1 isr == 1ms
    cp.SYST.clear_current();
    cp.SYST.enable_counter();
    cp.SYST.enable_interrupt();

    let mut led = Pin::new(Port::E, 13, PinMode::Output);
    led.set_high();

    /**************************************/

    let mut scl = Pin::new(Port::B, 6, PinMode::Alt(4));
    scl.output_type(OutputType::OpenDrain);
    //scl.set_high();

    let mut sda = Pin::new(Port::B, 7, PinMode::Alt(4));
    sda.output_type(OutputType::OpenDrain);
    //sda.set_high();

    let i2c_cfg = stm32_hal2::i2c::I2cConfig {
        speed: stm32_hal2::i2c::I2cSpeed::Fast400K, // Set to Fast mode, at 400Khz.
        nostretch: false,
        noise_filter: stm32_hal2::i2c::NoiseFilter::Analog,
        ..Default::default()
    };

    let mut i2c = I2c::new(dp.I2C1, i2c_cfg, &clock_cfg);

    let acce_addr = 0x19;
    //accelerometer, enable all axis

    let i2c_acce_enable_all_axis: [u8; 2] = [0x20, 0x77];
    i2c.write(acce_addr, &i2c_acce_enable_all_axis).ok();

    /**************************************/

    let mut _sck = Pin::new(Port::A, 5, PinMode::Alt(5));
    let mut _miso = Pin::new(Port::A, 6, PinMode::Alt(5));
    let mut _mosi = Pin::new(Port::A, 7, PinMode::Alt(5));
    let mut nss = Pin::new(Port::E, 3, PinMode::Output);
    nss.set_high();

    let spi_cfg = stm32_hal2::spi::SpiConfig {
        mode: stm32_hal2::spi::SpiMode::mode3(),
        comm_mode: stm32_hal2::spi::SpiCommMode::FullDuplex,
        slave_select: stm32_hal2::spi::SlaveSelect::Software,
        data_size: stm32_hal2::spi::DataSize::D8,
        fifo_reception_thresh: stm32_hal2::spi::ReceptionThresh::D8,
    };

    let mut spi = Spi::new(dp.SPI1, spi_cfg, stm32_hal2::spi::BaudRate::Div2);

    const READ: u8 = 1 << 7;
    const WRITE: u8 = 0 << 7;
    const MULTI: u8 = 1 << 6;
    const SINGLE: u8 = 0 << 6;

    /**************************************/

    nss.set_low();
    let mut buf_id_spi: [u8; 2] = [0x0F | SINGLE | READ, 0x00];
    spi.transfer(&mut buf_id_spi).ok();
    nss.set_high();

    nss.set_low();
    let spi_gyro_enable_all: [u8; 2] = [0x20 | SINGLE | WRITE, 0xFF]; // 760Hz, 100 cut off, enable all
    spi.write(&spi_gyro_enable_all).ok();
    nss.set_high();

    nss.set_low();
    let spi_gyro_enable_all: [u8; 2] = [0x23 | SINGLE | WRITE, 0x10]; // continoous update, LSB, 500dps, 4 wire spi
    spi.write(&spi_gyro_enable_all).ok();
    nss.set_high();


    let mut counter = 0;
    let mut last_value = 0;

    let mut gyro: [i16; 3] = [0; 3];
    let mut acce: [i16; 3] = [0; 3];

    loop {
        counter+=1;

        let buf_read_request_i2c: [u8; 1] = [0x28 + 0x80]; // 0x80 mean multiread
        let mut buf_read_i2c: [u8; 6] = [0; 6];
        i2c.write_read(acce_addr, &buf_read_request_i2c, &mut buf_read_i2c).ok();

        acce[0] = (buf_read_i2c[0] as u16 + ((buf_read_i2c[1] as u16) << 8)) as i16;
        acce[1] = (buf_read_i2c[2] as u16 + ((buf_read_i2c[3] as u16) << 8)) as i16;
        acce[2] = (buf_read_i2c[4] as u16 + ((buf_read_i2c[5] as u16) << 8)) as i16;

        nss.set_low();
        let mut buf_read_spi: [u8; 7] = [0x28 | MULTI | READ, 0,0,0,0,0,0];
        spi.transfer(&mut buf_read_spi).ok();
        nss.set_high();

        gyro[0] = (buf_read_spi[1] as u16 + ((buf_read_spi[2] as u16) << 8)) as i16;
        gyro[1] = (buf_read_spi[3] as u16 + ((buf_read_spi[4] as u16) << 8)) as i16;
        gyro[2] = (buf_read_spi[5] as u16 + ((buf_read_spi[6] as u16) << 8)) as i16;

        let value = SYSTICK_RELOAD.load(Ordering::SeqCst);
        if value - last_value >= 1000 { // every second
            last_value = value;
            hprintln!("loops/s: {} time: {} ms", counter, value);
            hprintln!("gyro: {:?} acce: {:?}", gyro, acce);
            counter = 0; // 1810 loop

            //toggle led
            if led.is_high(){led.set_low()}else{led.set_high()}
        }


    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

#[exception]
fn SysTick() -> () {
    SYSTICK_RELOAD.fetch_add(1, Ordering::SeqCst);
}
