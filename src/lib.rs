//! Rust library for TinyCircuits' Thumby.
//! 
//! This crate provides a basic abstraction of the Thumby hardware, including
//! the display, audio, and input.
//! 
//! # Examples
//! 
//! Examples can be found in [the examples folder](https://github.com/jngo102/thumby-rs/blob/master/examples).
//! 
#![no_std]
#![deny(missing_docs)]
#![deny(warnings)]
#![deny(unstable_features)]
#![deny(unused_import_braces)]
#![deny(unused_qualifications)]
#![deny(rustdoc::broken_intra_doc_links)]
#![deny(rustdoc::missing_doc_code_examples)]

mod audio;
mod input;
mod ssd1306;

use audio::Audio;
use input::Input;
use rp2040_hal::{
    gpio::{FunctionSpi, Pins},
    pac::{CorePeripherals, Peripherals},
    pwm::Slices,
    Sio
};
use ssd1306::SSD1306;
pub use audio::music;
pub use ssd1306::color;

/// The main Thumby data structure.
pub struct Thumby {
    /// Data structure to interface with the Thumby's piezo-electric speaker.
    pub audio: Audio,
    /// Data structure to interface with the Thumby's OLED screen.
    pub display: SSD1306,
    /// Data structure to interface with the Thumby's input D-pad and buttons.
    pub input: Input,
}

impl Thumby {
    /// Create a new Thumby instance.
    pub fn new() -> Self {
        let mut pac = Peripherals::take().expect("Failed to take Peripherals singleton");
        let core = CorePeripherals::take().expect("Failed to take core peripherals");

        let sio = Sio::new(pac.SIO);

        let pins = Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        let pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);
        let mut pwm = pwm_slices.pwm6;
        pwm.enable();
        let audio = Audio::new(pwm, pins.gpio28);
        let _sck = pins.gpio18.into_mode::<FunctionSpi>();
        let _sda = pins.gpio19.into_mode::<FunctionSpi>();
        let cs = pins.gpio16.into_push_pull_output();
        let dc = pins.gpio17.into_push_pull_output();
        let rst = pins.gpio20.into_push_pull_output();
        let mut display = SSD1306::new(
            cs,
            dc,
            rst,
            pac.WATCHDOG,
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            pac.SPI0,
            pac.RESETS,
            pac.TIMER,
            core.SYST,
        );
        display.enable_grayscale();

        let input = Input::new(
            pins.gpio3.into_pull_up_input(),
            pins.gpio5.into_pull_up_input(),
            pins.gpio4.into_pull_up_input(),
            pins.gpio6.into_pull_up_input(),
            pins.gpio27.into_pull_up_input(),
            pins.gpio24.into_pull_up_input(),
        );

        Self {
            audio,
            display,
            input,
        }
    }

    /// Wait for a given number of milliseconds.
    pub fn wait_ms(&mut self, ms: u32) {
        self.display.wait_ms(ms);
    }

    /// Update the Thumby's state.
    pub fn update(&mut self) {
        self.display.update();
        self.input.update();
    }
}
