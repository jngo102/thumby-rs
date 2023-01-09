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
#![deny(unsafe_code)]
#![deny(unstable_features)]
#![deny(unused_import_braces)]
#![deny(unused_qualifications)]
#![deny(rustdoc::broken_intra_doc_links)]
#![deny(rustdoc::missing_doc_code_examples)]

mod audio;
mod input;

use ssd1306::prelude::DisplayConfig;
use ::ssd1306::{
    mode::BufferedGraphicsMode, prelude::SPIInterface, rotation::DisplayRotation::Rotate0,
    size::DisplaySize72x40, Ssd1306,
};
use audio::Audio;
use cortex_m::delay::Delay;
use embedded_hal::spi::MODE_0;
use fugit::RateExtU32;
use input::Input;
use rp2040_hal::{
    clocks,
    gpio::{bank0::*, FunctionSpi, Pin, Pins, PushPullOutput},
    pac::{CorePeripherals, Peripherals, SPI0},
    pwm::Slices,
    spi::Enabled,
    Clock, Sio, Spi, Watchdog,
};
pub use audio::music;

type Ssd1306Thumby = Ssd1306<
    SPIInterface<Spi<Enabled, SPI0, 8>, Pin<Gpio17, PushPullOutput>, Pin<Gpio16, PushPullOutput>>,
    DisplaySize72x40,
    BufferedGraphicsMode<DisplaySize72x40>,
>;

/// The main Thumby data structure.
pub struct Thumby {
    /// Data structure to interface with the Thumby's piezo-electric speaker.
    pub audio: Audio,
    /// Data structure to interface with the Thumby's OLED screen.
    pub display: Ssd1306Thumby,
    /// Data structure to interface with the Thumby's input D-pad and buttons.
    pub input: Input,
    delay: Delay,
}

impl Thumby {
    /// Create a new Thumby instance.
    pub fn new() -> Self {
        let mut pac = Peripherals::take().expect("Failed to take Peripherals singleton");
        let core = CorePeripherals::take().expect("Failed to take core peripherals");

        let sio = Sio::new(pac.SIO);

        let mut watchdog = Watchdog::new(pac.WATCHDOG);

        // Configure the clocks
        let clocks = clocks::init_clocks_and_plls(
            12_000_000u32,
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog,
        )
        .ok()
        .expect("Failed to initialize clocks");

        let mut delay = Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

        let pins = Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        let pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);
        let pwm = pwm_slices.pwm6;
        let audio = Audio::new(pwm, pins.gpio28);

        let _sck = pins.gpio18.into_mode::<FunctionSpi>();
        let _sda = pins.gpio19.into_mode::<FunctionSpi>();
        let cs = pins.gpio16.into_push_pull_output();
        let dc = pins.gpio17.into_push_pull_output();
        let mut rst = pins.gpio20.into_push_pull_output();
        let spi = Spi::<_, _, 8>::new(pac.SPI0).init(
            &mut pac.RESETS,
            clocks.peripheral_clock.freq(),
            115_200u32.Hz(),
            &MODE_0,
        );
        let interface = SPIInterface::new(spi, dc, cs);
        let mut display =
            Ssd1306::new(interface, DisplaySize72x40, Rotate0).into_buffered_graphics_mode();
        display
            .reset(&mut rst, &mut delay)
            .ok()
            .expect("Failed to reset display");
        display.init().ok().expect("Failed to initialise display");

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
            delay,
        }
    }

    /// Wait for a given number of milliseconds.
    pub fn wait_ms(&mut self, ms: u32) {
        self.delay.delay_ms(ms);
    }

    /// Update the Thumby's state.
    pub fn update(&mut self) {
        self.input.update();
    }
}
