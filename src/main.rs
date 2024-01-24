#![no_std]
#![no_main]

// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
                     // use panic_abort as _; // requires nightly
                     // use panic_itm as _; // logs messages over ITM; requires ITM support
                     // use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use cortex_m_rt::entry;
// use jlink_rtt::Output;
use stm32f4::stm32f411;

#[entry]
fn main() -> ! {
    let perip = stm32f411::Peripherals::take().unwrap();

    // perip.RCC.ahbenr.modify(|_, w| w.iopben().set_bit());
    perip.RCC.ahb1enr.modify(|_, w| w.gpioben().enabled());

    let gpiob = &perip.GPIOB;
    gpiob.moder.modify(|_, w| w.moder13().output());

    loop {
        gpiob.bsrr.write(|w| w.bs13().set());
        // gpiob.bsrr.write(|w| w.br13().reset());
    }
}
