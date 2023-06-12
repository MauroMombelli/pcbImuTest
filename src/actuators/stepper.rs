use defmt::*;
use embassy_stm32::gpio::Output;

#[derive(Debug)]
enum Step {
    Step1,
    Step2,
    Step3,
    Step4,
}

pub struct Stepper {
    pin: [Output<'static, embassy_stm32::gpio::AnyPin>; 4],
    step_per_revolution: u16,
    step: Step,
}

impl Stepper {
    pub fn new(
        _step_per_revolution: u16,
        _in1: Output<'static, embassy_stm32::gpio::AnyPin>,
        _in2: Output<'static, embassy_stm32::gpio::AnyPin>,
        _in3: Output<'static, embassy_stm32::gpio::AnyPin>,
        _in4: Output<'static, embassy_stm32::gpio::AnyPin>,
    ) -> Stepper {
        let mut ret = Self {
            step_per_revolution: _step_per_revolution,
            pin: [_in1, _in2, _in3, _in4],
            step: Step::Step1,
        };
        ret.step();
        return ret;
    }

    pub fn step(&mut self) {
        //info!("STEP {:?}", self.step);
        match &self.step {
            Step::Step1 => {
                self.pin[0].set_high();
                self.pin[1].set_low();
                self.pin[2].set_high();
                self.pin[3].set_low();
                self.step = Step::Step2;
            }
            Step::Step2 => {
                self.pin[0].set_low();
                self.pin[1].set_high();
                self.pin[2].set_high();
                self.pin[3].set_low();
                self.step = Step::Step3;
            }
            Step::Step3 => {
                self.pin[0].set_low();
                self.pin[1].set_high();
                self.pin[2].set_low();
                self.pin[3].set_high();
                self.step = Step::Step4;
            }
            Step::Step4 => {
                self.pin[0].set_high();
                self.pin[1].set_low();
                self.pin[2].set_low();
                self.pin[3].set_high();
                self.step = Step::Step1;
            }
        }
    }
}
