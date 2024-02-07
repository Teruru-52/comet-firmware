mod types;

use core::{
    fmt::Write,
    marker::PhantomData,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};
use cortex_m::interrupt::free;
use spin::{Lazy, Mutex};
use stm32f4xx_hal::{
    pac,
    prelude::*,
    timer::{Event, SysDelay},
};

use types::{Led1, Led2, Led3, Led4, Led5, Led6, Led7, Led8};

pub static BAG: Lazy<Bag> = Lazy::new(|| Bag::new());
pub struct Bag {
    pub operator: Mutex<Operator>,
    clock: AtomicU32,
}

impl Bag {
    pub fn new() -> Self {
        // let dp = pac::Peripherals::take().unwrap();
        let cortex_m::Peripherals { SYST, mut NVIC, .. } = cortex_m::Peripherals::take().unwrap();
        let pac::Peripherals {
            RCC,
            GPIOA,
            GPIOB,
            GPIOC,
            GPIOH,
            ADC1,
            SPI1,
            TIM1,
            TIM2,
            TIM3,
            TIM4,
            TIM5,
            TIM9,
            FLASH,
            ..
        } = pac::Peripherals::take().unwrap();
        let rcc = RCC.constrain();

        let clocks = rcc
            .cfgr
            .hclk(96_000_000.Hz())
            .sysclk(96_000_000.Hz())
            .pclk1(48_000_000.Hz())
            .pclk2(48_000_000.Hz())
            .freeze();

        let mut delay = SYST.delay(&clocks);
        let gpioa = GPIOA.split();
        let gpiob = GPIOB.split();
        let gpioc = GPIOC.split();
        let gpioh = GPIOH.split();

        Bag {
            operator: Mutex::new(Operator {
                delay,
                led_back_left_red: gpiob.pb13.into_push_pull_output(),
                led_back_left_orange: gpiob.pb14.into_push_pull_output(),
                led_back_right_orange: gpioa.pa10.into_push_pull_output(),
                led_back_right_red: gpioa.pa11.into_push_pull_output(),
                led_front_right_outer: gpiob.pb5.into_push_pull_output(),
                led_front_right_inner: gpioc.pc14.into_push_pull_output(),
                led_front_left_inner: gpioh.ph1.into_push_pull_output(),
                led_front_left_outer: gpiob.pb12.into_push_pull_output(),
            }),
            clock: AtomicU32::new(0),
        }
    }
}

pub struct Operator {
    delay: SysDelay,
    led_back_left_red: Led1,
    led_back_left_orange: Led2,
    led_back_right_orange: Led3,
    led_back_right_red: Led4,
    led_front_right_outer: Led5,
    led_front_right_inner: Led6,
    led_front_left_inner: Led7,
    led_front_left_outer: Led8,
}

impl Operator {
    pub fn delay_ms(&mut self, ms: u32) {
        self.delay.delay_ms(ms);
    }

    pub fn led_on_red(&mut self) {
        self.led_back_left_red.set_high();
        self.led_back_right_red.set_high();
    }

    pub fn led_off_red(&mut self) {
        self.led_back_left_red.set_low();
        self.led_back_right_red.set_low();
    }

    pub fn led_on_orange(&mut self) {
        self.led_back_left_orange.set_high();
        self.led_back_right_orange.set_high();
    }

    pub fn led_off_orange(&mut self) {
        self.led_back_left_orange.set_low();
        self.led_back_right_orange.set_low();
    }

    pub fn led_on_white(&mut self) {
        self.led_front_right_outer.set_high();
        self.led_front_right_inner.set_high();
        self.led_front_left_inner.set_high();
        self.led_front_left_outer.set_high();
    }

    pub fn led_off_white(&mut self) {
        self.led_front_right_outer.set_low();
        self.led_front_right_inner.set_low();
        self.led_front_left_inner.set_low();
        self.led_front_left_outer.set_low();
    }

    pub fn led_on_all(&mut self) {
        self.led_back_left_red.set_high();
        self.led_back_left_orange.set_high();
        self.led_back_right_orange.set_high();
        self.led_back_right_red.set_high();
        self.led_front_right_outer.set_high();
        self.led_front_right_inner.set_high();
        self.led_front_left_inner.set_high();
        self.led_front_left_outer.set_high();
    }

    pub fn led_off_all(&mut self) {
        self.led_back_left_red.set_low();
        self.led_back_left_orange.set_low();
        self.led_back_right_orange.set_low();
        self.led_back_right_red.set_low();
        self.led_front_right_outer.set_low();
        self.led_front_right_inner.set_low();
        self.led_front_left_inner.set_low();
        self.led_front_left_outer.set_low();
    }
}
