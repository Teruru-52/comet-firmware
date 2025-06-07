#[allow(unused)]
use sensors2::{
    encoder::AS5055A, encoder::MA702GQ, imu::ICM20600, infrared::Infrared, motor::Motor,
    speaker::Speaker,
};
use stm32f4xx_hal::{
    adc::Adc,
    gpio::{
        gpioa::{PA0, PA1, PA10, PA11, PA12, PA13, PA15, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9},
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
    pac::{ADC1, SPI2, TIM1, TIM10, TIM11, TIM2, TIM3, TIM4, TIM5, TIM9},
    qei::Qei,
    spi::TransferModeNormal,
    timer::{
        counter::Counter,
        pwm::{PwmChannel, PwmHz},
    },
};

pub type Led1 = PB2<Output<PushPull>>;
pub type Led2 = PB10<Output<PushPull>>;
pub type Led3 = PA10<Output<PushPull>>;
pub type Led4 = PA11<Output<PushPull>>;
pub type Led5 = PA15<Output<PushPull>>;
pub type Led6 = PB3<Output<PushPull>>;
pub type Led7 = PB4<Output<PushPull>>;
pub type Led8 = PC13<Output<PushPull>>;
pub type Led9 = PC14<Output<PushPull>>;
pub type Led10 = PC15<Output<PushPull>>;

pub type Speaker_ = Speaker<PwmChannel<TIM9, 1>>;

pub type Voltmeter = sensors2::voltmeter::Voltmeter<ADC1, PA2<Analog>>;

pub type LeftMotor = Motor<PwmChannel<TIM4, 2>, PwmChannel<TIM4, 3>>;

pub type RightMotor = Motor<PwmChannel<TIM4, 1>, PwmChannel<TIM4, 0>>;

// pub type LeftEncoder = AS5055A<PA9<Output<PushPull>>>;
pub type LeftEncoder =
    MA702GQ<Qei<TIM5, (PA0<Alternate<PushPull, 2>>, PA1<Alternate<PushPull, 2>>)>>;

// pub type RightEncoder = AS5055A<PA12<Output<PushPull>>>;
pub type RightEncoder =
    MA702GQ<Qei<TIM1, (PA8<Alternate<PushPull, 1>>, PA9<Alternate<PushPull, 1>>)>>;

pub type Imu = ICM20600<PB12<Output<PushPull>>>;

pub type Spi = stm32f4xx_hal::spi::Spi<
    SPI2,
    (
        PB13<Alternate<PushPull, 5>>,
        PB14<Alternate<PushPull, 5>>,
        PB15<Alternate<PushPull, 5>>,
    ),
    TransferModeNormal,
>;

pub struct Infrareds {
    pub front_left: Infrared<ADC1, PA7<Analog>>,
    pub front_right: Infrared<ADC1, PA5<Analog>>,
    pub side_left: Infrared<ADC1, PB0<Analog>>,
    pub side_right: Infrared<ADC1, PA4<Analog>>,
}

pub type ControlTimer = Counter<TIM2, 1_000_000>; // interrupt
