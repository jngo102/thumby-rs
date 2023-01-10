#![no_std]
#![no_main]

use thumby::{color::Color, Thumby};
// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// Entry point to our bare-metal application.
///
/// The `#[rp2040_hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
#[rp2040_hal::entry]
fn main() -> ! {
    let mut thumby = Thumby::new();
    thumby.audio.play(thumby::music::C);
    thumby.wait_ms(500);
    thumby.display.draw_filled_rectangle(16, 16, 72 - 32, 40 - 32, Color::DarkGray);
    thumby.audio.play(thumby::music::D);
    thumby.wait_ms(500);
    thumby.display.draw_filled_rectangle(0, 0, 72, 40, Color::White);
    thumby.audio.play(thumby::music::E);
    thumby.wait_ms(500);

    loop {
        if thumby.input.a.pressed() || thumby.input.b.pressed() {
            thumby.audio.play(thumby::music::C);
        } else {
            thumby.audio.stop();
        }
    }
}