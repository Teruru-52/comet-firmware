#![no_std]
#![no_main]

mod init;

// pick a panicking behavior
// use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// use panic_abort as _; // requires nightly
// use panic_itm as _; // logs messages over ITM; requires ITM support
// use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use core::fmt::Write;
use core::panic::PanicInfo;

// use cortex_m::asm;
use cortex_m::interrupt::free;
use cortex_m_rt::entry;
use jlink_rtt::Output;
use spin::Lazy;
use stm32f4xx_hal::interrupt;

use init::{tick_on, BAG};

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    let mut out = Output::new();
    writeln!(out, "{:?}", info).ok();
    let mut operator = BAG.operator.lock();
    operator.turn_on_panic_led();
    operator.turn_on_speaker();
    loop {}
}

// #[interrupt]
// fn TIM1_BRK_TIM9() {
//     let mut pollar = BAG.pollar.lock();
//     pollar.poll();
//     pollar.clear_interrupt();
// }

#[interrupt]
fn TIM2() {
    free(|_| {
        let mut pollar = BAG.pollar.lock();
        pollar.poll();
        pollar.clear_interrupt();
        let mut operator = BAG.operator.lock();
        operator.control();
        operator.clear_interrupt();
    });
}

#[entry]
fn main() -> ! {
    // initialization
    Lazy::force(&BAG);

    BAG.operator.lock().delay_ms(50_u32);

    BAG.pollar.lock().init();
    let mut out = Output::new();
    writeln!(out, "start!").ok();
    BAG.operator.lock().beep();
    tick_on();

    loop {
        // BAG.operator.lock().led_off_all();
        // BAG.operator.lock().delay_ms(1000_u32);
        // BAG.operator.lock().led_on_all();
        // BAG.operator.lock().delay_ms(1000_u32);
    }
}
