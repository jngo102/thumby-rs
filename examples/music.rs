#![no_std]
#![no_main]

use cortex_m::asm;
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

use thumby::{
    music::*,
    Thumby,
};

const TWINKLE_TWINKLE: [f32; 48] = [
    C, C, G, G, A, A, G, REST,
    F, F, E, E, D, D, C, REST,
    G, G, F, F, E, E, D, REST,
    G, G, F, F, E, E, D, REST,
    C, C, G, G, A, A, G, REST,
    F, F, E, E, D, D, C, REST,
];

/// Entry point to our bare-metal application.
///
/// The `#[rp2040_hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
#[rp2040_hal::entry]
fn main() -> ! {
    let mut thumby = Thumby::new();

    for note in TWINKLE_TWINKLE {
        thumby.audio.play(note);
        thumby.wait_ms(500);
        thumby.audio.stop();
        thumby.wait_ms(100);
    }

    loop {
        asm::wfe();
    }
}