#![no_std]
#![no_main]

mod init;

// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
                     // use panic_abort as _; // requires nightly
                     // use panic_itm as _; // logs messages over ITM; requires ITM support
                     // use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use cortex_m::asm;
use cortex_m_rt::entry;
// use jlink_rtt::Output;
use init::BAG;
use spin::Lazy;

#[entry]
fn main() -> ! {
    // initialization
    Lazy::force(&BAG);

    loop {
        BAG.operator.lock().led_on_red();
        BAG.operator.lock().led_off_orange();
        BAG.operator.lock().led_off_white();
        BAG.operator.lock().delay_ms(1000_u32);
        BAG.operator.lock().led_off_red();
        BAG.operator.lock().led_on_orange();
        BAG.operator.lock().led_on_white();
        BAG.operator.lock().delay_ms(1000_u32);
    }
}
