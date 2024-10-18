mod types;

use core::{
    fmt::Write,
    marker::PhantomData,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};
use cortex_m::interrupt::free;
use spin::{Lazy, Mutex};
use stm32f4xx_hal::{
    interrupt,
    nb::block,
    pac,
    prelude::*,
    qei::Qei,
    timer::{Event, Flag, SysDelay},
};

use types::{ControlTimer, Led1, Led2, Led3, Led4, Led5, Led6, Led7, Led8, SensorTimer};

pub static BAG: Lazy<Bag> = Lazy::new(|| Bag::new());

pub fn tick_on() {
    use cortex_m::peripheral::NVIC;
    use interrupt::TIM5;
    // use interrupt::{TIM3, TIM5};

    free(|_cs| {
        // NVIC::unpend(TIM3);
        NVIC::unpend(TIM5);
        unsafe {
            cortex_m::interrupt::enable();
            // NVIC::unmask(TIM3);
            NVIC::unmask(TIM5);
        }
    });
}

pub fn tick_off() {
    use cortex_m::peripheral::NVIC;
    use interrupt::TIM5;
    // use interrupt::{TIM3, TIM5};

    free(|_cs| {
        cortex_m::interrupt::disable();
        // NVIC::mask(TIM3);
        // NVIC::pend(TIM3);
        NVIC::mask(TIM5);
        NVIC::pend(TIM5);
    });
}

pub struct Bag {
    pub operator: Mutex<Operator>,
    // pub pollar: Mutex<Pollar>,
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

        let mut control_timer = TIM5.counter::<1_000_000>(&clocks);
        control_timer.start(200.millis()).unwrap();

        // let mut sensor_timer = TIM3.counter::<1_000_000>(&clocks);
        // sensor_timer.start(2000.millis()).unwrap();

        control_timer.listen(Event::Update);
        // sensor_timer.listen(Event::Update);

        unsafe {
            NVIC.set_priority(interrupt::TIM5, 0);
            // NVIC.set_priority(interrupt::TIM3, 1);
        }

        Bag {
            operator: Mutex::new(Operator {
                control_timer,
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
            // pollar: Mutex::new(Pollar {
            // timer: sensor_timer,
            // led_back_left_orange: gpiob.pb14.into_push_pull_output(),
            // led_back_right_orange: gpioa.pa10.into_push_pull_output(),
            // }),  
            clock: AtomicU32::new(0),
        }
    }
}

pub struct Operator {
    control_timer: ControlTimer,
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
    fn tick_off_routine(&mut self) {
        tick_off();
        BAG.clock.store(0, Ordering::SeqCst);
    }

    pub fn clear_interrupt(&mut self) {
        self.control_timer.clear_flags(Flag::Update);
    }

    pub fn control(&mut self) {
        BAG.clock.fetch_add(1, Ordering::SeqCst);
        self.led_back_left_red.toggle();
        self.led_back_right_red.toggle();
        self.led_front_right_outer.toggle();
    }

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

    pub fn turn_on_panic_led(&mut self) {
        self.led_back_left_orange.set_high();
        self.led_back_right_orange.set_high();
    }
}

pub struct Pollar {
    timer: SensorTimer,
    // led_back_left_orange: Led2,
    // led_back_right_orange: Led3,
}

impl Pollar {
    pub fn poll(&mut self) {
        // macro_rules! poll {}self.tick_off_routine();
    }

    pub fn clear_interrupt(&mut self) {
        self.timer.clear_flags(Flag::Update);
    }
}
