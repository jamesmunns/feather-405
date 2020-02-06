#![no_main]
#![no_std]

use panic_persist as _; // panic handler

use cortex_m;
use cortex_m_rt::{entry, pre_init};
use stm32f4xx_hal as hal;

use crate::hal::{prelude::*, stm32};
use usb_device::prelude::*;

use usbd_serial::{SerialPort, USB_CLASS_CDC};


use stm32f4xx_hal::usb::{Peripheral, UsbBus};
static mut EP_MEMORY: [u32; 1024] = [0; 1024];

use embedded_hal::digital::v2::OutputPin;

enum TogCount {
    On(usize),
    Off(usize),
}

#[entry]
fn main() -> ! {
    if let (Some(dp), Some(cp)) = (
        stm32::Peripherals::take(),
        cortex_m::peripheral::Peripherals::take(),
    ) {
        let gpioc = dp.GPIOC.split();
        let gpioa = dp.GPIOA.split();
        let mut led = gpioc.pc1.into_push_pull_output();

        // Set up the system clock. We want to run at 48MHz for this one.
        let rcc = dp.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(12.mhz())
            .sysclk(48.mhz())
            .pclk1(24.mhz())
            .require_pll48clk()
            .freeze();

        let usb = Peripheral {
            usb_global: dp.OTG_FS_GLOBAL,
            usb_device: dp.OTG_FS_DEVICE,
            usb_pwrclk: dp.OTG_FS_PWRCLK,
            pin_dm: gpioa.pa11.into_alternate_af10(),
            pin_dp: gpioa.pa12.into_alternate_af10(),
        };

        let usb_bus = UsbBus::new(usb, unsafe { &mut EP_MEMORY });

        let mut serial = SerialPort::new(&usb_bus);

        let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27DD))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(USB_CLASS_CDC)
            .build();

        // Create a delay abstraction based on SysTick
        let mut delay = hal::delay::Delay::new(cp.SYST, clocks);

        let mut tog = TogCount::Off(0);
        led.set_low().ok();
        let mut togs = 0;

        'reset: loop {
            //////////////////////////////////////////////////////////////////
            // If the USB port is idle, blink to show we are bored
            //////////////////////////////////////////////////////////////////
            if !usb_dev.poll(&mut [&mut serial]) {
                use TogCount::*;
                tog = match tog {
                    Off(n) if n >= 200_000 => {
                        led.set_high().ok();
                        togs += 1;
                        On(0)
                    }
                    On(n) if n >= 200_000 => {
                        led.set_low().ok();
                        togs += 1;
                        Off(0)
                    }
                    On(n) => On(n + 1),
                    Off(n) => Off(n + 1),
                };

                //////////////////////////////////////////////////////////////
                // After a while, just crash and reboot so we can press the
                // DFU button on reboot
                //////////////////////////////////////////////////////////////
                assert!(togs <= 30);

                continue;
            } else {
                //////////////////////////////////////////////////////////////
                // When active, reset the idle counter
                //////////////////////////////////////////////////////////////
                tog = TogCount::Off(0);
                led.set_low().ok();
            }

            let mut buf = [0u8; 64];

            //////////////////////////////////////////////////////////////////
            // Loopback as upper case, use 'z' to force a reboot of the device
            //////////////////////////////////////////////////////////////////
            match serial.read(&mut buf) {
                Ok(count) if count > 0 => {
                    led.set_high().ok(); // Turn on

                    // Echo back in upper case
                    for c in buf[0..count].iter_mut() {
                        if *c == b'z' {
                            break 'reset;
                        }

                        if b'a' <= *c && *c <= b'z' {
                            *c &= !0x20;
                        }
                    }

                    let mut write_offset = 0;
                    while write_offset < count {
                        match serial.write(&buf[write_offset..count]) {
                            Ok(len) if len > 0 => {
                                write_offset += len;
                            }
                            _ => {}
                        }
                    }
                }
                _ => {}
            }

            led.set_low().ok(); // Turn off
        }
    }

    panic!("!");
}

// NOTE: This is much more reliable when you enable `inline-asm` in cortex-m
#[pre_init]
unsafe fn before_main() {
    extern "C" {
        static mut _panic_dump_start: u8;
    }

    use cortex_m::register::msp;

    let start_ptr = &mut _panic_dump_start as *mut u8;

    // Panic-persist sets a flag to the start of the dump region
    // when a panic occurs
    if 0x0FACADE0 == core::ptr::read_unaligned(start_ptr.cast::<usize>()) {
        // Clear the flag
        start_ptr.cast::<usize>().write_unaligned(0x00000000);

        // The DFU bootloader's reset vector and initial stack pointer
        const SYSMEM_MSP: u32 = 0x1fff0000;
        const SYSMEM_RESET: u32 = 0x1fff0004;

        let dfu_msp = core::ptr::read(SYSMEM_MSP as *const u32);
        let putter: *const fn() = SYSMEM_RESET as *const fn();

        msp::write(dfu_msp);
        (*putter)();
    }
}
