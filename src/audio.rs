use embedded_hal::PwmPin;
use rp2040_hal::{
    gpio::{bank0::Gpio28, Pin, PinId},
    pwm::{FreeRunning, Pwm6, Slice},
};

/// Module containing musical note declarations
pub mod music {
    /// 4th octave C note
    pub const C: f32 = 261.63;
    /// 4th octave D note
    pub const D: f32 = 293.66;
    /// 4th octave E note
    pub const E: f32 = 329.63;
    /// 4th octave F note
    pub const F: f32 = 349.23;
    /// 4th octave G note
    pub const G: f32 = 392.00;
    /// 4th octave A note
    pub const A: f32 = 440.00;
    /// 4th octave B note
    pub const B: f32 = 493.88;
    /// Rest note
    pub const REST: f32 = 0.0;
}

const XTAL_FREQ: f32 = 12_000_000.0;

fn calc_note(freq: f32) -> u16 {
    (XTAL_FREQ / 40.0 / freq) as u16
}

pub struct Audio {
    pwm: Slice<Pwm6, FreeRunning>,
}

impl Audio {
    pub fn new(
        mut pwm: Slice<Pwm6, FreeRunning>,
        audio_pin: Pin<Gpio28, <Gpio28 as PinId>::Reset>,
    ) -> Self {
        pwm.set_div_frac(40);
        pwm.set_div_int(40);
        let channel = &mut pwm.channel_a;
        channel.output_to(audio_pin);
        Self { pwm }
    }

    /// Start playing audio from the speaker.
    ///
    /// # Arguments
    ///
    /// `freq` - The audio's frequency
    pub fn play(&mut self, freq: f32) {
        let top = calc_note(freq);
        self.pwm.channel_a.set_duty(top / 2);
        self.pwm.set_top(top);
    }

    /// Stop playing audio from the speaker.
    pub fn stop(&mut self) {
        self.pwm.channel_a.set_duty(0);
    }
}
