use rp2040_hal::gpio::{bank0::*, Pin, PinId, PullUpInput};
use embedded_hal::digital::v2::InputPin;

pub struct Input {
    pub left: Button<Gpio3>,
    pub right: Button<Gpio5>,
    pub up: Button<Gpio4>,
    pub down: Button<Gpio6>,
    pub a: Button<Gpio27>,
    pub b: Button<Gpio24>,
}

impl Input {
    pub fn new(
        left_pin: Pin<Gpio3, PullUpInput>,
        right_pin: Pin<Gpio5, PullUpInput>,
        up_pin: Pin<Gpio4, PullUpInput>,
        down_pin: Pin<Gpio6, PullUpInput>,
        a_pin: Pin<Gpio27, PullUpInput>,
        b_pin: Pin<Gpio24, PullUpInput>,
    ) -> Self {
        let left = Button::new(left_pin);
        let right = Button::new(right_pin);
        let up = Button::new(up_pin);
        let down = Button::new(down_pin);
        let a = Button::new(a_pin);
        let b = Button::new(b_pin);

        Self {
            left,
            right,
            up,
            down,
            a,
            b,
        }
    }

    pub fn update(&mut self) {
        self.left.update();
        self.right.update();
        self.up.update();
        self.down.update();
        self.a.update();
        self.b.update();
    }
}

pub struct Button<I> where I: PinId {
    pin: Pin<I, PullUpInput>,
    state: bool,
    prev_state: bool,
}

impl<I> Button<I> where I: PinId {
    pub fn new(pin: Pin<I, PullUpInput>) -> Self {
        Self {
            pin,
            state: false,
            prev_state: false
        }
    }

    pub fn update(&mut self) {
        self.prev_state = self.state;
        self.state = self.pin.is_low().ok().expect("Failed to read button state");
    }

    /// Whether a button has just been pressed.
    pub fn just_pressed(&self) -> bool {
        self.state && !self.prev_state
    }

    /// Whether a button has just been released.
    pub fn just_released(&self) -> bool {
        !self.state && self.prev_state
    }

    /// Whether a button is currently pressed.
    pub fn pressed(&self) -> bool {
        self.state
    }

    /// Whether a button is currently released.
    pub fn released(&self) -> bool {
        !self.state
    }
}