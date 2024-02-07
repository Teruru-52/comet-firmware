#[allow(unused)]
use stm32f4xx_hal::{
    gpio::{
        gpioa::{PA0, PA1, PA10, PA11, PA12, PA13, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9},
        gpiob::{
            PB0, PB1, PB10, PB11, PB12, PB13, PB14, PB15, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9,
        },
        gpioc::{PC13, PC14, PC15},
        gpioh::{PH0, PH1},
        Alternate, Analog, OpenDrain, Output, PushPull,
    },
    pac::{ADC1, SPI1, TIM1, TIM10, TIM11, TIM2, TIM3, TIM4, TIM5, TIM9},
};

pub type Led1 = PB13<Output<PushPull>>;
pub type Led2 = PB14<Output<PushPull>>;
pub type Led3 = PA10<Output<PushPull>>;
pub type Led4 = PA11<Output<PushPull>>;
pub type Led5 = PB5<Output<PushPull>>;
pub type Led6 = PC14<Output<PushPull>>;
pub type Led7 = PH1<Output<PushPull>>;
pub type Led8 = PB12<Output<PushPull>>;
