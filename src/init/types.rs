#[allow(unused)]
use sensors2::{encoder::MA702GQ, imu::ICM20600, motor::Motor, speaker::Speaker, tof::VL6180X};
use stm32f4xx_hal::{
    gpio::{
        gpioa::{PA0, PA1, PA10, PA11, PA12, PA13, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9},
        gpiob::{
            PB0, PB1, PB10, PB11, PB12, PB13, PB14, PB15, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9,
        },
        gpioc::{PC13, PC14, PC15},
        gpioh::{PH0, PH1},
        Alternate,
        Analog,
        OpenDrain,
        Output,
        PushPull,
        // Alternate, Analog, OpenDrain, Output, PushPull, Pin,
    },
    pac::{ADC1, SPI1, TIM1, TIM10, TIM11, TIM2, TIM3, TIM4, TIM5, TIM9},
    qei::Qei,
    spi::TransferModeNormal,
    timer::{
        counter::Counter,
        pwm::{PwmChannel, PwmHz},
    },
};

pub type Led1 = PB13<Output<PushPull>>;
pub type Led2 = PB14<Output<PushPull>>;
pub type Led3 = PA10<Output<PushPull>>;
pub type Led4 = PA11<Output<PushPull>>;
pub type Led5 = PB5<Output<PushPull>>;
pub type Led6 = PC14<Output<PushPull>>;
pub type Led7 = PH1<Output<PushPull>>;
pub type Led8 = PB12<Output<PushPull>>;

// pub type Speaker = PwmHz<TIM3, 1, PB4<Alternate<2>>>;
// pub type Speaker = PwmHz<TIM3, ChannelBuilder<TIM3, 1, PB4<Alternate<2>>>>;
// pub type Speaker_ = Speaker<PwmChannel<TIM3, 1>>;

// pub type LeftEncoder = MA702GQ<Qei<TIM4>>;
// MA702GQ<Qei<TIM4, (PB6<Alternate<PushPull, 2>>, PB7<Alternate<PushPull, 2>>)>>;

// pub type RightEncoder = MA702GQ<Qei<TIM2>>;
// MA702GQ<Qei<TIM2, (PA0<Alternate<PushPull, 1>>, PA1<Alternate<PushPull, 1>>)>>;

pub type Imu = ICM20600<PB15<Output<PushPull>>>;

// pub type Spi = stm32f4xx_hal::spi::Spi<
//     SPI1,
//     (
//         PA5<Alternate<5, PushPull>>,
//         PA6<Alternate<5, PushPull>>,
//         PA7<Alternate<5, PushPull>>,
//     ),
//     TransferModeNormal,
// >;

pub type ControlTimer = Counter<TIM5, 1_000_000>; // interrupt
pub type SensorTimer = Counter<TIM9, 1_000_000>; // interrupt
