mod trajectory;
mod types;

use core::{
    fmt::Write,
    marker::PhantomData,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};
use cortex_m::interrupt::free;
use embedded_hal::{adc, blocking::serial::write};
use heapless::{Deque, Vec};
#[allow(unused_imports)]
use micromath::F32Ext;
use mousecore2::{
    control::{
        ControlParameters, Controller, NavigationController, SupervisoryController, Target, Tracker,
    },
    estimate::{AngleState, Estimator, LengthState, SensorValue, State},
    solve::{
        run::{shortest_path, EdgeKind, Node, Posture as RunPosture},
        search::{
            Commander, Coordinate, Posture as SearchPosture, SearchState, Searcher, WallState,
        },
    },
    wall::{Pose, PoseConverter, WallDetector, Walls},
};
use sensors2::{
    encoder::AS5055A, encoder::MA702GQ, imu::ICM20600, infrared::Infrared, motor::Motor,
    speaker::Speaker,
};
use spin::{Lazy, Mutex};
use stm32f4xx_hal::{
    adc::{config::AdcConfig, Adc},
    flash::{FlashExt, LockedFlash},
    i2c::Pins,
    interrupt,
    nb::block,
    pac::{self, tim2, tim3::cnt},
    prelude::*,
    qei::Qei,
    timer::{Event, SysDelay},
};
use uom::si::{
    acceleration::meter_per_second_squared,
    angle::{degree, revolution},
    angular_acceleration::degree_per_second_squared,
    angular_jerk::degree_per_second_cubed,
    angular_velocity::degree_per_second,
    electric_potential::volt,
    f32::{
        Acceleration, Angle, AngularAcceleration, AngularJerk, AngularVelocity, ElectricPotential,
        Frequency, Jerk, Length, Time, Velocity,
    },
    frequency::hertz,
    jerk::meter_per_second_cubed,
    length::{meter, millimeter},
    time::second,
    velocity::meter_per_second,
};

use trajectory::{Command, TrajectoryConfig, TrajectoryManager};
use types::{
    ControlTimer, Imu, Infrareds, Led1, Led10, Led2, Led3, Led4, Led5, Led6, Led7, Led8, Led9,
    LeftEncoder, LeftMotor, RightEncoder, RightMotor, Speaker_, Spi, Voltmeter,
};

const W: u8 = 32;
const START_LENGTH: Length = Length {
    value: 0.02,
    units: PhantomData,
    dimension: PhantomData,
};
const SENSOR_STDDEV: Length = Length {
    value: 0.04,
    units: PhantomData,
    dimension: PhantomData,
};
const WHEEL_RADIUS: Length = Length {
    value: 0.00707,
    units: PhantomData,
    dimension: PhantomData,
};
const PATH_MAX: usize = 32 * 32;
const FRONT_OFFSET: Length = Length {
    value: 0.01,
    dimension: PhantomData,
    units: PhantomData,
};
const SQUARE_WIDTH: Length = Length {
    value: 0.09,
    dimension: PhantomData,
    units: PhantomData,
};
const VOLTAGE_LIMIT: ElectricPotential = ElectricPotential {
    value: 3.3,
    dimension: PhantomData,
    units: PhantomData,
};
static INIT_STATE: Lazy<State> = Lazy::new(|| State {
    x: LengthState {
        x: Length::new::<millimeter>(45.0),
        ..Default::default()
    },
    y: LengthState {
        x: Length::new::<millimeter>(45.0),
        ..Default::default()
    },
    theta: AngleState {
        x: Angle::new::<degree>(90.0),
        ..Default::default()
    },
});
static RUN_GOALS: Lazy<[Node<W>; 4]> = Lazy::new(|| {
    use RunPosture::*;
    [(2, 0, South), (2, 0, North), (2, 0, East), (2, 0, West)]
        .map(|(x, y, pos)| Node::new(x, y, pos).unwrap())
});
static SEARCH_GOALS: Lazy<[Coordinate<W>; 2]> =
    Lazy::new(|| [(3, 0), (2, 1)].map(|(x, y)| Coordinate::new(x, y).unwrap()));

pub static BAG: Lazy<Bag> = Lazy::new(|| Bag::new());

pub fn tick_on() {
    use cortex_m::peripheral::NVIC;
    use interrupt::{TIM1_BRK_TIM9, TIM2};

    free(|_cs| {
        NVIC::unpend(TIM1_BRK_TIM9);
        NVIC::unpend(TIM2);
        unsafe {
            cortex_m::interrupt::enable();
            NVIC::unmask(TIM1_BRK_TIM9);
            NVIC::unmask(TIM2);
        }
    });
}

pub fn tick_off() {
    use cortex_m::peripheral::NVIC;
    use interrupt::{TIM1_BRK_TIM9, TIM2};

    free(|_cs| {
        cortex_m::interrupt::disable();
        NVIC::mask(TIM1_BRK_TIM9);
        NVIC::pend(TIM1_BRK_TIM9);
        NVIC::mask(TIM2);
        NVIC::pend(TIM2);
    });
}

struct Linear {
    slope: f32,
    intercept: Length,
}

impl Default for Linear {
    fn default() -> Self {
        Self {
            slope: 1.0,
            intercept: Default::default(),
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug, Default)]
struct InfraredData<T> {
    front_right: T,
    front_left: T,
    side_right: T,
    side_left: T,
}

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
enum InfraredSensorPos {
    FrontRight,
    FrontLeft,
    SideRigth,
    SideLeft,
}

#[derive(Clone, Copy, Debug)]
enum Mode {
    Idle(IdleMode),
    Search { run_number: usize },
    Return { run_number: usize },
    Run { run_number: usize },
    Select,
    TransIdent,
    RotIdent,
}

#[derive(Clone, Copy, Debug)]
enum IdleMode {
    Nop = 0,
    Search,
    Run,
    AddSearch,
    // TransIdent,
    // RotIdent,
}

impl IdleMode {
    const N: u8 = 4;
}

struct Selector {
    angle: Angle,
}

impl Selector {
    fn proceed(&mut self, relative: Angle) {
        self.angle += relative;
    }

    fn mode(&self) -> IdleMode {
        match (self.angle.get::<revolution>() * 3.0).floor() as u8 % IdleMode::N {
            0 => IdleMode::Nop,
            1 => IdleMode::Search,
            2 => IdleMode::Run,
            3 => IdleMode::AddSearch,
            // 3 => IdleMode::TransIdent,
            // 4 => IdleMode::RotIdent,
            _ => unreachable!(),
        }
    }
}

impl From<IdleMode> for Selector {
    fn from(mode: IdleMode) -> Self {
        Self {
            angle: Angle::new::<revolution>((mode as u8 as f32 + 0.5) / 3.0),
        }
    }
}

pub struct Bag {
    pub operator: Mutex<Operator>,
    pub pollar: Mutex<Pollar>,
    pub solver: Mutex<Solver>,
    pub walls: Mutex<Walls<W>>,
    state: Mutex<SearchState<W>>,
    pub commander: Mutex<Option<Commander<W>>>,
    is_search_finish: AtomicBool,
    voltage: Mutex<ElectricPotential>,
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
            SPI2,
            TIM1,
            TIM2,
            TIM3,
            TIM4,
            TIM5,
            TIM9,
            TIM10,
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

        let period = Time::new::<second>(0.001);

        let tracker = Tracker::builder()
            .period(period)
            .zeta(1.0)
            .b(1.0)
            .xi_threshold(Velocity::new::<meter_per_second>(0.2))
            .build();
        let navigator = NavigationController::builder()
            .gain(120.0)
            .dgain(40.0)
            .build();
        let supervisor = SupervisoryController::builder()
            .avoidance_distance(Length::new::<millimeter>(25.0))
            .margin(60.0)
            .square_width(SQUARE_WIDTH)
            .build();
        let controller = Controller::builder()
            .trans_params(ControlParameters {
                kp: 4.8497,
                ki: 29.5783,
                kd: 0.005,
                model_k: 1.865,
                model_t1: 0.4443,
            })
            .rot_params(ControlParameters {
                kp: 0.21134,
                ki: 2.9317,
                kd: 0.0,
                model_k: 82.39,
                model_t1: 0.2855,
            })
            .period(period)
            .build();
        let detector = WallDetector::<W>::default();
        let estimator = Estimator::builder()
            .period(period)
            .slip_angle_const(Acceleration::new::<meter_per_second_squared>(80.0))
            .build();
        let state = INIT_STATE.clone();
        let manager = TrajectoryConfig::builder()
            .search_velocity(Velocity::new::<meter_per_second>(0.4))
            .run_slalom_velocity(Velocity::new::<meter_per_second>(0.6))
            .v_max(Velocity::new::<meter_per_second>(2.0))
            .a_max(Acceleration::new::<meter_per_second_squared>(4.0))
            .j_max(Jerk::new::<meter_per_second_cubed>(200.0))
            .spin_v_max(AngularVelocity::new::<degree_per_second>(1440.0))
            .spin_a_max(AngularAcceleration::new::<degree_per_second_squared>(
                14400.0,
            ))
            .spin_j_max(AngularJerk::new::<degree_per_second_cubed>(57600.0))
            .period(period)
            .square_width(SQUARE_WIDTH)
            .front_offset(FRONT_OFFSET)
            .initial_pose(Pose {
                x: state.x.x,
                y: state.y.x,
                theta: state.theta.x,
            })
            .build()
            .into();

        let mut control_timer = TIM2.counter::<1_000_000>(&clocks);
        control_timer.start(1.millis()).unwrap();

        // SPI settings
        let spi_pins = (
            gpiob.pb13.into_alternate(),
            gpiob.pb14.into_alternate(),
            gpiob.pb15.into_alternate(),
        );
        let mut spi = SPI2.spi(
            spi_pins,
            stm32f4xx_hal::hal::spi::MODE_1,
            // stm32f4xx_hal::hal::spi::MODE_3,
            1_562_500.Hz(),
            &clocks,
        );
        // sensors using SPI
        let imu = {
            let mut cs = gpiob.pb12.into_push_pull_output();
            cs.set_high();

            ICM20600::new(&mut spi, cs, &mut delay, &mut control_timer)
        };

        // let left_encoder = {
        //     let mut cs = gpioa.pa9.into_push_pull_output();
        //     cs.set_high();

        //     AS5055A::new(&mut spi, cs, &mut delay, &mut control_timer)
        // };
        // let right_encoder = {
        //     let mut cs = gpioa.pa12.into_push_pull_output();
        //     cs.set_high();

        //     AS5055A::new(&mut spi, cs, &mut delay, &mut control_timer)
        // };
        let left_encoder = {
            let pins = (gpioa.pa0.into_alternate(), gpioa.pa1.into_alternate());
            let qei = Qei::new(TIM5, pins);
            let encoder = MA702GQ::new(qei);
            encoder
        };
        let right_encoder = {
            let pins = (gpioa.pa8.into_alternate(), gpioa.pa9.into_alternate());
            let qei = Qei::new(TIM1, pins);
            let encoder = MA702GQ::new(qei);
            encoder
        };

        let (spi, spi_pins) = spi.release();
        let spi = spi.spi(
            spi_pins,
            stm32f4xx_hal::hal::spi::MODE_1,
            // stm32f4xx_hal::hal::spi::MODE_3,
            7_500_000.Hz(),
            &clocks,
        );

        let adc1 = Mutex::new(Adc::adc1(ADC1, true, AdcConfig::default()));
        let voltmeter = {
            let pa2 = gpioa.pa2.into_analog();
            Voltmeter::new(
                // adc1,
                pa2,
                Time::new::<second>(0.005),
                Frequency::new::<hertz>(1.0),
                2.0,
            )
        };
        let voltage = voltmeter.voltage();

        let (mut pwm_s, mut pwm_f) = {
            let pins = (gpioa.pa6.into_alternate(), gpiob.pb1.into_alternate());

            TIM3.pwm_hz(pins, 10.kHz(), &clocks).split()
        };
        let (infrared_fl, infrared_fr, infrared_sl, infrared_sr) = {
            let pa7 = gpioa.pa7.into_analog();
            let pa5 = gpioa.pa5.into_analog();
            let pb0 = gpiob.pb0.into_analog();
            let pa4 = gpioa.pa4.into_analog();
            pwm_f.enable();
            pwm_s.enable();
            (
                Infrared::new(pa7, 0.3),
                Infrared::new(pa5, 0.3),
                Infrared::new(pb0, 0.3),
                Infrared::new(pa4, 0.3),
            )
        };

        let (left_motor, right_motor) = {
            let (mut pwm1, mut pwm2, mut pwm3, mut pwm4) = {
                let pins = (
                    gpiob.pb6.into_alternate(),
                    gpiob.pb7.into_alternate(),
                    gpiob.pb8.into_alternate(),
                    gpiob.pb9.into_alternate(),
                );

                TIM4.pwm_hz(pins, 30.kHz(), &clocks).split()
            };
            pwm1.enable();
            pwm2.enable();
            pwm3.enable();
            pwm4.enable();
            (Motor::new(pwm3, pwm4), Motor::new(pwm2, pwm1))
        };
        let speaker = {
            let mut pwm = {
                let pin = gpioa.pa3.into_alternate();
                TIM9.pwm_hz(pin, 1_000.Hz(), &clocks).split()
            };
            pwm.enable();
            Speaker::new(pwm)
        };

        // let mut sensor_timer = TIM9.counter::<1_000_000>(&clocks);
        // sensor_timer.start(1000.micros()).unwrap();

        control_timer.listen(Event::Update);
        // sensor_timer.listen(Event::Update);

        unsafe {
            NVIC.set_priority(interrupt::TIM2, 0);
            NVIC.set_priority(interrupt::TIM1_BRK_TIM9, 1);
        }

        Bag {
            operator: Mutex::new(Operator {
                tracker,
                navigator,
                supervisor,
                detector,
                estimator,
                controller,
                state,
                manager,
                mode: Mode::Idle(IdleMode::Search),
                selector: Selector::from(IdleMode::Search),

                stddev: Length::default(),
                converter: PoseConverter::default(),

                control_timer,
                spi,
                imu,
                left_encoder,
                right_encoder,
                voltage,
                left_motor,
                right_motor,
                speaker,

                delay,
                led_back_left1: gpiob.pb2.into_push_pull_output(),
                led_back_left2: gpiob.pb10.into_push_pull_output(),
                led_back_left3: gpioa.pa10.into_push_pull_output(),
                led_back_left4: gpioa.pa11.into_push_pull_output(),
                led_back_left5: gpioa.pa15.into_push_pull_output(),
                led_back_right1: gpiob.pb3.into_push_pull_output(),
                led_back_right2: gpiob.pb4.into_push_pull_output(),
                led_back_right3: gpioc.pc13.into_push_pull_output(),
                // led_back_right4: gpioc.pc14.into_push_pull_output(),
                // led_back_right5: gpioc.pc15.into_push_pull_output(),
                ident_data: Vec::new(),
                flash: LockedFlash::new(FLASH),
                past_states: Deque::new(),
            }),
            pollar: Mutex::new(Pollar {
                // timer: control_timer,
                // timer: sensor_timer,
                led_back_right4: gpioc.pc14.into_push_pull_output(),
                led_back_right5: gpioc.pc15.into_push_pull_output(),
                adc1,
                voltmeter,
                infrareds: Infrareds {
                    front_left: infrared_fl,
                    front_right: infrared_fr,
                    side_left: infrared_sl,
                    side_right: infrared_sr,
                },
                pwm_f,
                pwm_s,
            }),
            solver: Mutex::new(Solver::new()),
            walls: Mutex::new(Walls::new()),
            state: Mutex::new(
                SearchState::new(Coordinate::new(0, 1).unwrap(), SearchPosture::North).unwrap(),
            ),
            commander: Mutex::new(None),
            is_search_finish: AtomicBool::new(false),
            voltage: Mutex::new(voltage),
            clock: AtomicU32::new(0),
        }
    }
}

pub struct Operator {
    tracker: Tracker,
    navigator: NavigationController,
    #[allow(unused)]
    supervisor: SupervisoryController,
    controller: Controller,
    estimator: Estimator,
    detector: WallDetector<W>,
    state: State,
    manager: TrajectoryManager,
    mode: Mode,
    selector: Selector,

    stddev: Length,
    #[allow(unused)]
    converter: PoseConverter<W>,

    control_timer: ControlTimer,
    spi: Spi,
    imu: Imu,
    left_encoder: LeftEncoder,
    right_encoder: RightEncoder,
    left_motor: LeftMotor,
    right_motor: RightMotor,
    voltage: ElectricPotential,
    speaker: Speaker_,

    delay: SysDelay,
    led_back_left1: Led1,
    led_back_left2: Led2,
    led_back_left3: Led3,
    led_back_left4: Led4,
    led_back_left5: Led5,
    led_back_right1: Led6,
    led_back_right2: Led7,
    led_back_right3: Led8,
    // led_back_right4: Led9,
    // led_back_right5: Led10,

    // record for system identification
    #[allow(unused)]
    ident_data: Vec<f32, 2_000>,
    flash: LockedFlash,

    past_states: Deque<State, 100>,
}

impl Operator {
    fn tick_off_routine(&mut self) {
        tick_off();
        BAG.clock.store(0, Ordering::SeqCst);
    }

    pub fn clear_interrupt(&mut self) {
        // self.control_timer.clear_flags(Flag::Update); // 0.18.0
        self.control_timer.clear_interrupt(Event::Update); // 0.12.0
    }

    fn load_walls_from_flash(&self) -> Walls<W> {
        let bytes = self.flash.read();
        let offset = 16 * 1024;
        Walls::try_from(&bytes[offset..offset + 512]).unwrap()
    }

    fn store_walls_to_flash(&mut self, walls: &Walls<W>) {
        let mut flash = self.flash.unlocked();
        flash.erase(1).unwrap();
        flash.program(16 * 1024, walls.as_bytes().iter()).unwrap();
    }

    pub fn init(&mut self) {
        self.reset_state();
    }

    pub fn control(&mut self) {
        BAG.clock.fetch_add(1, Ordering::SeqCst);
        // match self.mode {
        //     Mode::Idle(mode) => self.control_idle(mode),
        //     Mode::Search { run_number } => self.control_search(run_number),
        //     Mode::Return { run_number } => self.control_return(run_number),
        //     Mode::Run { run_number } => self.control_run(run_number),
        //     Mode::Select => self.control_select(),
        //     Mode::TransIdent => self.control_trans_ident(),
        //     Mode::RotIdent => self.control_rot_ident(),
        // }

        // test imu
        // self.estimate();
        // let angular_velocity = self.state.theta.v;
        // let theta = self.state.theta.x.value;

        // test motor
        if let Some(voltage) = BAG.voltage.try_lock() {
            self.voltage = *voltage;
            // self.left_motor
            //     .apply(ElectricPotential::new::<volt>(2.0), self.voltage);
            // self.right_motor
            //     .apply(ElectricPotential::new::<volt>(2.0), self.voltage);
        }

        // test encoder
        // let left_angle = block!(self.left_encoder.angle(&mut self.spi)).unwrap();
        // let right_angle = block!(self.right_encoder.angle(&mut self.spi)).unwrap();
        // let left_distance =
        //     block!(self.left_encoder.dist_angle(&mut self.spi)).unwrap() * WHEEL_RADIUS;
        // let right_distance =
        //     block!(self.right_encoder.dist_angle(&mut self.spi)).unwrap() * WHEEL_RADIUS;
        // need to multiply by wheel radius
        let left_angle = -block!(self.left_encoder.angle()).unwrap();
        let right_angle = -block!(self.right_encoder.angle()).unwrap();

        static mut cnt_1kHz: u16 = 0;
        unsafe {
            cnt_1kHz = (cnt_1kHz + 1) % 1000;
            if cnt_1kHz == 0 {
                self.led_back_left1.toggle();
                self.led_back_left2.toggle();
                self.led_back_left3.toggle();
                self.led_back_left4.toggle();
                self.led_back_left5.toggle();
                self.led_back_right1.toggle();
                self.led_back_right2.toggle();
                self.led_back_right3.toggle();
            }
            if cnt_1kHz % 200 == 0 {
                // writeln!(jlink_rtt::Output::new(), "{:?}", self.voltage).ok();
                // writeln!(
                //     jlink_rtt::Output::new(),
                //     "{:?}, {:?}",
                //     theta,
                //     angular_velocity
                // )
                // .ok();
                writeln!(
                    jlink_rtt::Output::new(),
                    "left: {:?}, right: {:?}",
                    left_angle,
                    right_angle
                )
                .ok();
                // writeln!(
                //     jlink_rtt::Output::new(),
                //     "left: {:?}, right: {:?}",
                //     left_distance,
                //     right_distance
                // )
                // .ok();
            }
        }
    }

    #[allow(unused)]
    fn control_rot_ident(&mut self) {
        let voltage = self.voltage;
        self.left_motor
            .apply(ElectricPotential::new::<volt>(-0.3), voltage);
        self.right_motor
            .apply(ElectricPotential::new::<volt>(0.3), voltage);
        self.estimate();
        if self.ident_data.push(self.state.theta.v.value).is_err() {
            self.tick_off_routine();
            let mut out = jlink_rtt::Output::new();
            for &v in &self.ident_data {
                writeln!(out, "{}", v).ok();
            }
            self.ident_data.clear();
            self.mode = Mode::Idle(IdleMode::Search);
            tick_on();
        }
    }

    #[allow(unused)]
    fn control_trans_ident(&mut self) {
        let voltage = self.voltage;
        self.left_motor
            .apply(ElectricPotential::new::<volt>(0.4), voltage);
        self.right_motor
            .apply(ElectricPotential::new::<volt>(0.4), voltage);
        self.estimate();
        let state = &self.state;
        let v = state.x.v * state.theta.x.value.cos() + state.y.v * state.theta.x.value.sin();
        if self.ident_data.push(v.value).is_err() {
            self.tick_off_routine();
            let mut out = jlink_rtt::Output::new();
            for &v in &self.ident_data {
                writeln!(out, "{}", v).ok();
            }
            self.ident_data.clear();
            self.mode = Mode::Idle(IdleMode::Search);
            tick_on();
        }
    }

    fn is_switch_on(&mut self) -> bool {
        const SELECT_ACCEL: Acceleration = Acceleration {
            value: 5.0,
            units: PhantomData,
            dimension: PhantomData,
        };
        block!(self.imu.translational_acceleration(&mut self.spi))
            .unwrap()
            .abs()
            > SELECT_ACCEL
    }

    fn is_switch_off(&mut self) -> bool {
        const SELECT_ACCEL: Acceleration = Acceleration {
            value: 2.0,
            units: PhantomData,
            dimension: PhantomData,
        };
        block!(self.imu.translational_acceleration(&mut self.spi))
            .unwrap()
            .abs()
            < SELECT_ACCEL
    }

    fn display_number_by_led(&mut self, n: u8) {
        let n = n & 3;
        if n & 1 == 1 {
            self.led_back_right1.set_high();
        } else {
            self.led_back_right1.set_low();
        }
        if (n >> 1) & 1 == 1 {
            self.led_back_right2.set_high();
        } else {
            self.led_back_right2.set_low();
        }
    }

    fn control_select(&mut self) {
        let mode = self.selector.mode();
        if self.is_switch_on() {
            self.tick_off_routine();
            while !self.is_switch_off() {
                block!(self.control_timer.wait()).unwrap();
            }
            self.mode = Mode::Idle(mode);
            self.display_number_by_led(0);
            tick_on();
            return;
        }
        self.display_number_by_led(mode as u8);
        // self.selector
        //     .proceed(-block!(self.right_encoder.angle()).unwrap());
    }

    fn init_run(&mut self, start: Node<W>, is_goal: impl Fn(&Node<W>) -> bool + Copy) {
        use RunPosture::*;

        let (_, path, npos) = IntoIterator::into_iter([North, East, South, West])
            .filter_map(|pos| {
                shortest_path(
                    Node::new(start.x(), start.y(), pos).unwrap(),
                    is_goal,
                    |coord| {
                        !matches!(
                            BAG.walls.lock().wall_state(coord),
                            WallState::Checked { exists: false }
                        )
                    },
                    |kind| match kind {
                        EdgeKind::Straight(x) => *x as u16 * 10,
                        EdgeKind::StraightDiagonal(x) => *x as u16 * 7,
                        EdgeKind::Slalom45 => 12,
                        EdgeKind::Slalom90 => 15,
                        EdgeKind::Slalom135 => 20,
                        EdgeKind::Slalom180 => 25,
                        EdgeKind::SlalomDiagonal90 => 15,
                    },
                )
                .map(|(path, cost)| (cost, path, pos))
            })
            .min_by_key(|v| v.0)
            .unwrap();

        let pos_to_u8 = |pos| {
            use RunPosture::*;
            match pos {
                North => 0u8,
                East => 1,
                South => 2,
                West => 3,
                _ => unreachable!(),
            }
        };

        let pos = pos_to_u8(start.posture());
        let npos = pos_to_u8(npos);

        let init_angle = Angle::new::<degree>(match (4 + npos - pos) % 4 {
            0 => 0.0,
            1 => -90.0,
            2 => 180.0,
            3 => 90.0,
            _ => unreachable!(),
        });
        self.manager.into_run(path, init_angle);
    }

    fn control_idle(&mut self, mode: IdleMode) {
        if self.is_switch_on() {
            self.tick_off_routine();
            // wait until switch off
            while !self.is_switch_off() {
                block!(self.control_timer.wait()).unwrap();
            }
            self.mode = Mode::Select;
            tick_on();
            return;
        }
        // if let Some(mut que) = BAG.tof_queue.try_lock() {
        // while let Some((distance, pos, _)) = que.pop() {
        //     if pos == TofPosition::Right && distance < START_LENGTH {
        //         self.tick_off_routine();
        //         self.reset_state();
        //         self.mode = match mode {
        //             IdleMode::Nop => self.mode,
        //             IdleMode::Search => {
        //                 BAG.is_search_finish.store(false, Ordering::SeqCst);
        //                 self.manager.into_search();
        //                 Mode::Search { run_number: 4 }
        //             }
        //             IdleMode::Run => {
        //                 use RunPosture::*;

        //                 *BAG.walls.lock() = self.load_walls_from_flash();
        //                 BAG.is_search_finish.store(true, Ordering::SeqCst);
        //                 self.init_run(Node::new(0, 0, North).unwrap(), |node| {
        //                     RUN_GOALS.iter().any(|goal| node == goal)
        //                 });
        //                 Mode::Run { run_number: 0 }
        //             }
        //             IdleMode::AddSearch => {
        //                 *BAG.walls.lock() = self.load_walls_from_flash();
        //                 BAG.is_search_finish.store(false, Ordering::SeqCst);
        //                 self.manager.into_search();
        //                 Mode::Search { run_number: 0 }
        //             } // IdleMode::TransIdent => {
        //               //     self.ident_data.clear();
        //               //     Mode::TransIdent
        //               // }
        //               // IdleMode::RotIdent => {
        //               //     self.ident_data.clear();
        //               //     Mode::RotIdent
        //               // }
        //         };
        //         tick_on();
        //     }
        // }
        // }
    }

    fn control_search(&mut self, run_number: usize) {
        self.estimate();

        if let Some(command) = self.manager.command() {
            match command {
                Command::Track(target) => self.track(&target),
                Command::Operation => {
                    self.tick_off_routine();
                    self.store_walls_to_flash(&*BAG.walls.lock());
                    tick_on();
                }
            }
        } else {
            if !BAG.is_search_finish.load(Ordering::SeqCst) {
                self.manager.set_emergency(Pose::from_search_state(
                    BAG.state.lock().clone(),
                    SQUARE_WIDTH,
                    FRONT_OFFSET,
                ));
                match self.manager.command().unwrap() {
                    Command::Track(target) => self.track(&target),
                    _ => unreachable!(),
                }
            } else if self.manager.set_final(Pose::from_search_state(
                BAG.state.lock().clone(),
                SQUARE_WIDTH,
                FRONT_OFFSET,
            )) {
                match self.manager.command().unwrap() {
                    Command::Track(target) => self.track(&target),
                    _ => unreachable!(),
                }
            } else {
                self.tick_off_routine();
                let rev_start = Node::new(0, 0, RunPosture::South).unwrap();
                let state = BAG.state.lock();
                let x = state.x();
                let y = state.y();
                let (x, y, pos) = match state.posture() {
                    SearchPosture::North => (x, y + 1, RunPosture::North),
                    SearchPosture::East => (x + 1, y, RunPosture::East),
                    SearchPosture::South => (x, y - 1, RunPosture::South),
                    SearchPosture::West => (x - 1, y, RunPosture::West),
                };
                self.init_run(Node::new(x, y, pos).unwrap(), |node| &rev_start == node);
                self.mode = Mode::Return { run_number };
                self.store_walls_to_flash(&*BAG.walls.lock());
                tick_on();
                return;
            }
        };

        self.correct(true);

        if self.manager.is_full() {
            return;
        }

        match (
            BAG.commander.try_lock(),
            BAG.walls.try_lock(),
            BAG.state.try_lock(),
        ) {
            (Some(mut commander), Some(walls), Some(mut state)) => {
                match commander
                    .as_ref()
                    .map(|commander| commander.next_coordinate(|coord| walls.wall_state(coord)))
                {
                    Some(Ok(Some(next_coord))) => {
                        let pose =
                            Pose::from_search_state::<W>(state.clone(), SQUARE_WIDTH, FRONT_OFFSET);
                        let dir = state.update(&next_coord).unwrap();
                        self.manager.set(pose, dir);
                        commander.take();
                    }
                    Some(Ok(None)) => (),
                    Some(Err(err)) => unreachable!("{:?}", err),
                    _ => (),
                }
            }
            _ => (),
        }
    }

    fn control_run(&mut self, run_number: usize) {
        self.estimate();
        self.correct(false);
        match self.manager.command() {
            Some(Command::Track(target)) => self.track(&target),
            None => {
                use RunPosture::*;

                self.tick_off_routine();
                let goal = Node::new(0, 0, South).unwrap();
                let start = self.manager.last_node().unwrap();
                self.init_run(start, |node| goal == *node);
                self.mode = Mode::Return { run_number };
                tick_on();
            }
            _ => unreachable!(),
        }
    }

    fn control_return(&mut self, run_number: usize) {
        self.estimate();
        self.correct(false);
        if let Some(command) = self.manager.command() {
            match command {
                Command::Track(target) => self.track(&target),
                _ => unreachable!(),
            }
        } else {
            use RunPosture::*;

            self.tick_off_routine();
            self.init_run(Node::new(0, 0, South).unwrap(), |node| {
                RUN_GOALS.iter().any(|goal| node == goal)
            });
            self.mode = if run_number > 0 {
                Mode::Run {
                    run_number: run_number - 1,
                }
            } else {
                Mode::Idle(IdleMode::Nop)
            };
            tick_on();
        }
    }

    fn estimate(&mut self) {
        // estimate
        let sensor_value = {
            SensorValue {
                // left_distance: -block!(self.left_encoder.dist_angle()) * WHEEL_RADIUS,
                // right_distance: -block!(self.right_encoder.dist_angle()) * WHEEL_RADIUS,
                // dammy
                left_distance: Length::new::<millimeter>(0.0),
                right_distance: Length::new::<millimeter>(0.0),
                translational_acceleration: block!(self
                    .imu
                    .translational_acceleration(&mut self.spi))
                .unwrap(),
                angular_velocity: block!(self.imu.angular_velocity(&mut self.spi)).unwrap(),
            }
        };
        self.estimator.estimate(&mut self.state, &sensor_value);
        // if self.past_states.is_full() {
        //     self.past_states.pop_back();
        // }
        // self.past_states.push_front(self.state.clone()).unwrap();
        // self.stddev += Length::new::<millimeter>(0.005);
    }

    fn track(&mut self, target: &Target) {
        let input = self.navigator.navigate(&self.state, target);
        // let input = self.supervisor.supervise(&input, &self.state);
        let (control_target, control_state) = self.tracker.track(&self.state, target, &input);
        let vol = self.controller.control(&control_target, &control_state);

        const THRES: ElectricPotential = ElectricPotential {
            value: 40.0,
            dimension: PhantomData,
            units: PhantomData,
        };

        // fail safe for input voltage
        if THRES < vol.left.abs() || THRES < vol.right.abs() {
            panic!("fail safe!");
        }

        // update voltage
        if let Some(voltage) = BAG.voltage.try_lock() {
            self.voltage = *voltage;
        }
        self.left_motor.apply(vol.left, self.voltage);
        self.right_motor.apply(vol.right, self.voltage);
    }

    // TODO: Remove `with_detect` and separate this method into detect and correct
    fn correct(&mut self, with_detect: bool) {
        // match (BAG.tof_queue.try_lock(), BAG.walls.try_lock()) {
        //     (Some(mut que), Some(mut walls)) => {
        //         let current_clock = BAG.clock.load(Ordering::SeqCst);
        //         while let Some((distance, pos, clock)) = que.pop() {
        //             if current_clock < clock {
        //                 continue;
        //             }
        //             let diff = (current_clock - clock) as usize;
        //             let config = match pos {
        //                 TofPosition::Front => &self.tof_configs.front,
        //                 TofPosition::Right => &self.tof_configs.right,
        //                 TofPosition::Left => &self.tof_configs.left,
        //             };
        //             let distance = config.1.slope * distance + config.1.intercept;
        //             let state = if let Some(state) = self.past_states.iter().nth(diff) {
        //                 state
        //             } else {
        //                 return;
        //             };
        //             let cos_th = state.theta.x.value.cos();
        //             let sin_th = state.theta.x.value.sin();
        //             let pose = Pose {
        //                 x: state.x.x + config.0.x * cos_th - config.0.y * sin_th,
        //                 y: state.y.x + config.0.x * sin_th + config.0.y * cos_th,
        //                 theta: self.state.theta.x + config.0.theta,
        //             };

        //             if with_detect {
        //                 if let Some((coord, wall_state)) =
        //                     self.detector
        //                         .detect_and_update(&distance, &SENSOR_STDDEV, &pose)
        //                 {
        //                     walls.update(&coord, &wall_state);
        //                 }
        //             }

        //             const DISTANCE_TH: Length = Length {
        //                 value: 0.05,
        //                 units: PhantomData,
        //                 dimension: PhantomData,
        //             };
        //             if let Some(info) = self.converter.convert(&pose) {
        //                 if matches!(
        //                     walls.wall_state(&info.coord),
        //                     WallState::Checked { exists: true }
        //                 ) && distance < DISTANCE_TH
        //                 {
        //                     let sensor_var = SENSOR_STDDEV * SENSOR_STDDEV;
        //                     let var = self.stddev * self.stddev;
        //                     let k = (var / (var + sensor_var)).get::<uom::si::ratio::ratio>();

        //                     let distance_diff = k * (info.existing_distance - distance);
        //                     let cos_th = pose.theta.value.cos();
        //                     let sin_th = pose.theta.value.sin();
        //                     self.state.x.x += distance_diff * cos_th;
        //                     self.state.y.x += distance_diff * sin_th;
        //                     self.stddev = Length::new::<meter>((var * (1.0 - k)).value.sqrt());
        //                 }
        //             }
        //         }
        //     }
        //     _ => (),
        // }
    }

    pub fn stop(&mut self) {
        self.left_motor.apply(Default::default(), self.voltage);
        self.right_motor.apply(Default::default(), self.voltage);
    }

    fn reset_state(&mut self) {
        self.imu
            .init(&mut self.spi, &mut self.delay, &mut self.control_timer);
        // reset encoder
        block!(self.right_encoder.angle()).unwrap();
        block!(self.left_encoder.angle()).unwrap();
        self.state = INIT_STATE.clone();
    }

    pub fn beep(&mut self) {
        self.speaker.apply(0.4);
        self.delay_ms(50);
        self.speaker.apply(0.0);
    }

    pub fn turn_on_speaker(&mut self) {
        self.speaker.apply(0.2);
    }

    pub fn check_battery(&mut self) {
        writeln!(jlink_rtt::Output::new(), "Voltage: {:?}", self.voltage).ok();
        if self.voltage > VOLTAGE_LIMIT + ElectricPotential::new::<volt>(0.2) {
            self.led_back_left1.set_high();
        }
        if self.voltage > VOLTAGE_LIMIT + ElectricPotential::new::<volt>(0.4) {
            self.led_back_left2.set_high();
        }
        if self.voltage > VOLTAGE_LIMIT + ElectricPotential::new::<volt>(0.6) {
            self.led_back_left3.set_high();
        }
        if self.voltage > VOLTAGE_LIMIT + ElectricPotential::new::<volt>(0.8) {
            self.led_back_left4.set_high();
        }
        // if self.voltage < VOLTAGE_LIMIT {
        //     panic!("Low voltage!: {:?}", self.voltage);
        // }
        self.delay_ms(1000);
        self.led_off_all();
    }

    pub fn delay_ms(&mut self, ms: u32) {
        self.delay.delay_ms(ms);
    }

    pub fn led_on_all(&mut self) {
        self.led_back_left1.set_high();
        self.led_back_left2.set_high();
        self.led_back_left3.set_high();
        self.led_back_left4.set_high();
        self.led_back_left5.set_high();
        self.led_back_right1.set_high();
        self.led_back_right2.set_high();
        self.led_back_right3.set_high();
        // self.led_back_right4.set_high();
        // self.led_back_right5.set_high();
    }

    pub fn led_off_all(&mut self) {
        self.led_back_left1.set_low();
        self.led_back_left2.set_low();
        self.led_back_left3.set_low();
        self.led_back_left4.set_low();
        self.led_back_left5.set_low();
        self.led_back_right1.set_low();
        self.led_back_right2.set_low();
        self.led_back_right3.set_low();
        // self.led_back_right4.set_low();
        // self.led_back_right5.set_low();
    }

    pub fn turn_on_panic_led(&mut self) {
        self.led_back_right1.set_high();
    }
}

pub struct Pollar {
    // timer: ControlTimer,
    // timer: SensorTimer,
    led_back_right4: Led9,
    led_back_right5: Led10,
    adc1: Mutex<Adc<stm32f4xx_hal::pac::ADC1>>,
    voltmeter: Voltmeter,
    infrareds: Infrareds,
    pwm_f: stm32f4xx_hal::timer::pwm::PwmChannel<stm32f4xx_hal::pac::TIM3, 3>,
    pwm_s: stm32f4xx_hal::timer::pwm::PwmChannel<stm32f4xx_hal::pac::TIM3, 0>,
}

impl Pollar {
    pub fn init(&mut self) {
        self.check_battery();
        self.infrareds.front_left.init(&mut self.pwm_f);
        self.infrareds.front_right.init(&mut self.pwm_f);
        self.infrareds.side_left.init(&mut self.pwm_s);
        self.infrareds.side_right.init(&mut self.pwm_s);
    }

    pub fn poll(&mut self) {
        // macro_rules! poll {}self.tick_off_routine();
        self.voltmeter.update_voltage(&mut self.adc1);
        self.infrareds.front_left.update_value(&mut self.adc1);
        self.infrareds.front_right.update_value(&mut self.adc1);
        self.infrareds.side_left.update_value(&mut self.adc1);
        self.infrareds.side_right.update_value(&mut self.adc1);

        let voltage = self.voltmeter.voltage();
        // if voltage < VOLTAGE_LIMIT {
        //     panic!("Low voltage!: {:?}", voltage);
        // }

        static mut cnt_1kHz: u16 = 0;
        unsafe {
            cnt_1kHz = (cnt_1kHz + 1) % 1000;
            if cnt_1kHz == 0 {
                self.led_back_right4.toggle();
                self.led_back_right5.toggle();
                writeln!(jlink_rtt::Output::new(), "Voltage: {:?}", voltage).ok();
                writeln!(
                    jlink_rtt::Output::new(),
                    "fl: {:?}, fr: {:?}, sl: {:?}, sr: {:?}",
                    self.infrareds.front_left.value(),
                    self.infrareds.front_right.value(),
                    self.infrareds.side_left.value(),
                    self.infrareds.side_right.value()
                )
                .ok();
            }
        }
        *BAG.voltage.lock() = voltage;
    }

    pub fn check_battery(&mut self) {
        self.voltmeter.init(&mut self.adc1);
        self.voltmeter.update_voltage(&mut self.adc1);
        BAG.operator.lock().check_battery();
    }

    pub fn clear_interrupt(&mut self) {
        // self.timer.clear_flags(Flag::Update);
        // self.timer.clear_interrupt(Event::Update);
    }
}

pub struct Solver {
    searcher: Searcher<W>,
}

impl Solver {
    pub fn new() -> Self {
        Self {
            searcher: Searcher::new(Coordinate::new(0, 1).unwrap(), &*SEARCH_GOALS),
        }
    }

    pub fn search(&mut self) {
        // search
        if BAG.is_search_finish.load(Ordering::SeqCst) {
            return;
        }
        {
            let lock = BAG.commander.lock();
            if lock.is_some() {
                return;
            }
            core::mem::drop(lock);
        }

        let walls = {
            let lock = BAG.walls.lock();
            let walls = lock.clone();
            core::mem::drop(lock);
            walls
        };
        let coord = {
            let lock = BAG.state.lock();
            let coord = lock.coordinate().clone();
            core::mem::drop(lock);
            coord
        };
        match self
            .searcher
            .search(&coord, |coord| walls.wall_state(coord))
        {
            Ok(Some(next)) => {
                BAG.commander.lock().replace(next);
            }
            Ok(None) => {
                BAG.is_search_finish.store(true, Ordering::SeqCst);
            }
            Err(err) => unreachable!("{:?}", err),
        }
    }
}
