#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_reset as _; // panic handler

use cortex_m;
use cortex_m_rt::entry;
use stm32f4xx_hal as hal;

use crate::hal::{prelude::*, stm32};

#[entry]
fn main() -> ! {
    if let (Some(dp), Some(cp)) = (
        stm32::Peripherals::take(),
        cortex_m::peripheral::Peripherals::take(),
    ) {
        let gpioc = dp.GPIOC.split();
        let mut led = gpioc.pc1.into_push_pull_output();

        // Set up the system clock. We want to run at 48MHz for this one.
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.mhz()).freeze();

        // Create a delay abstraction based on SysTick
        let mut delay = hal::delay::Delay::new(cp.SYST, clocks);

        loop {
            // On for 1s, off for 1s.
            led.set_high().unwrap();
            delay.delay_ms(1000_u32);
            led.set_low().unwrap();
            delay.delay_ms(1000_u32);
        }
    }

    loop {}
}
