#![no_std]
#![no_main]

use cortex_m::delay::Delay;
use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m;
use cortex_m_rt::{entry, exception};

use cortex_m_semihosting::hprintln;

use stm32_hal2::{
    clocks::Clocks,
    gpio::{Pin, Port, PinMode, OutputType},
    i2c::I2c,
    spi::Spi,
    pac::{self, interrupt},
};
use stm32_hal2::clocks::*;

// WIP DOES NOT WORK

// monitor arm semihosting enable

// E13 -> LED

// acce: i2c, lsm303dlhc
// B6 and B7 -> i2c

// gyro: spi, l3gd20
// A5 A6 A7 -> SPI
// E3 -> CS gyro

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

    //let clock_cfg = Clocks::default();
    clock_cfg.setup().unwrap();

    cp.SYST.set_clock_source(cortex_m::peripheral::syst::SystClkSource::External);
    cp.SYST.set_reload(8_000-1); //1 isr == 1ms
    cp.SYST.clear_current();
    cp.SYST.enable_counter();
    cp.SYST.enable_interrupt();

    let mut led = Pin::new(Port::E, 13, PinMode::Output);
    led.set_high();

    //let mut timer = Timer::new_tim3(dp.TIM3, 0.2, Default::default(), &clock_cfg);
    //timer.enable_interrupt(TimerInterrupt::Update);

    let mut dma = stm32_hal2::dma::Dma::new(dp.DMA1);

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

    /**************************************/

    let _sck = Pin::new(Port::A, 5, PinMode::Alt(4));
    let _miso = Pin::new(Port::A, 6, PinMode::Alt(4));
    let _mosi = Pin::new(Port::A, 7, PinMode::Alt(4));
    let mut nss = Pin::new(Port::E, 3, PinMode::Alt(4));

    nss.set_high();//always selected

    let spi_cfg = stm32_hal2::spi::SpiConfig::default();

    let mut spi = Spi::new(dp.SPI1, spi_cfg, stm32_hal2::spi::BaudRate::Div8);

    //SPI1_RX ch2
    //SP1_TX ch3

    /**************************************/

    //dma::mux(stm32_hal2::dma::DmaChannel::C6, stm32_hal2::dma::DmaInput::I2c1Tx, &mut dp.DMAMUX);
//    dma.mux(stm32_hal2::dma::DmaChannel::C6, stm32_hal2::dma::DmaInput::I2c1Tx, &mut dp.DMAMUX);
//    dma.mux(stm32_hal2::dma::DmaChannel::C7, stm32_hal2::dma::DmaInput::I2c1Rx, &mut dp.DMAMUX);
    let mut delay = Delay::new(cp.SYST, clock_cfg.systick());

    // Or, customize the config, including setting different preset speeds:


    //let mut delay = cortex_m::delay::Delay::new(cp.SYST, clock_cfg.systick());

    let addr = 0x19;
    //accelerometer, enable all axis

    let buf = [0x20, 0x77];
    unsafe { i2c.write_dma(addr, &buf, true, stm32_hal2::dma::DmaChannel::C6, Default::default(), &mut dma); }
    //delay.delay_ms(10);
  //  unsafe { i2c.write_dma(addr, &BUF, true, stm32_hal2::dma::DmaChannel::C6, Default::default(), &mut dma); }
    //i2c.write(addr, &BUF).ok();

    //hprintln!("START");

    let buf = [0x28 + 0x80]; // 0x80 mean multiread
    //let mut answer: [u8; 6] = [0; 6];
    let mut counter = 0;
    let mut last_value = 0;

    let mut buf_read_i2c: [u8; 6] = [0; 6];
    let mut buf_read_spi: [u8; 6] = [0; 6];

    unsafe{spi.read_dma(&mut buf_read_spi, stm32_hal2::dma::DmaChannel::C2, Default::default(), &mut dma);}
    unsafe { i2c.read_dma(addr, &mut buf_read_spi, stm32_hal2::dma::DmaChannel::C7, Default::default(), &mut dma); }
    loop {
        //delay.delay_ms(2);
        counter+=1;
        //i2c.write_read(addr, &BUF, &mut answer).ok();

        //if dma.transfer_is_complete(stm32_hal2::dma::DmaChannel::C6) {
        //    unsafe { i2c.write_dma(addr, &BUF, true, stm32_hal2::dma::DmaChannel::C6, Default::default(), &mut dma); }
        //}

//        i2c.read(addr, &mut answer).ok();

        //if dma.transfer_is_complete(stm32_hal2::dma::DmaChannel::C6) {
            //if dma.transfer_is_complete(stm32_hal2::dma::DmaChannel::C7) {
                //i2c.write(addr, &BUF).ok();
                //unsafe { i2c.read_dma(addr, &mut answer, stm32_hal2::dma::DmaChannel::C7, Default::default(), &mut dma); }
            //}
        //}

        if dma.transfer_is_complete(stm32_hal2::dma::DmaChannel::C2) {
            spi.stop_dma(stm32_hal2::dma::DmaChannel::C2, Default::default(), &mut dma);
            unsafe{spi.read_dma(&mut buf_read_spi, stm32_hal2::dma::DmaChannel::C2, Default::default(), &mut dma);}
        }


        let value = SYSTICK_RELOAD.load(Ordering::SeqCst);
        if value - last_value >= 1000 { // every second
            last_value = value;
            hprintln!("loops/s: {} time: {} ms", counter, value);
            counter = 0;

            //toggle led
            if led.is_high(){led.set_low()}else{led.set_high()}
        }


    }
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
/*
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}*/


#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

#[exception]
fn SysTick() -> () {
    SYSTICK_RELOAD.fetch_add(1, Ordering::SeqCst);
}

#[interrupt]
fn DMA1_CH6() {

}
