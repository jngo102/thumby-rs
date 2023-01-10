use color::Color;
use cortex_m::{asm, delay::Delay, peripheral::SYST, prelude::_embedded_hal_blocking_spi_Write};
use embedded_hal::{digital::v2::OutputPin, spi::MODE_0};
use fugit::RateExtU32;
use libm::*;
use rp2040_hal::{
    clocks,
    gpio::{bank0::*, Pin, PushPullOutput},
    multicore::{Multicore, Stack},
    pac::*,
    spi::Enabled,
    timer::{Instant, Timer},
    Clock, Sio, Spi, Watchdog,
};

const STACK_SIZE: usize = 2048;

const FRAME_TIME_US: u32 = 4709;
const PRE_FRAME_TIME_US: u32 = 883;

const WIDTH: usize = 72;
const HEIGHT: usize = 40;
const BUFF_SZ: usize = WIDTH * HEIGHT / 8;
const BUFF_INT_SZ: usize = BUFF_SZ / 4;

static mut CORE1_STACK: Stack<STACK_SIZE> = Stack::new();

static mut STATE: State = State {
    thread: ThreadState::Stopped,
    copy_buffers: 0,
    pending_cmd: 0,
    contrast: 0,
    invert: false,
};
static mut BUFFER: [u8; BUFF_SZ] = [0; BUFF_SZ];
static mut SHADING: [u8; BUFF_SZ] = [0; BUFF_SZ];
static mut SUBFRAMES: [[u8; BUFF_SZ]; 3] = [[0; BUFF_SZ]; 3];
static PRE_FRAME_CMDS: [u8; 4] = [0xA8, 0, 0xD3, 52];
static POST_FRAME_CMDS: [u8; 4] = [0xD3, HEIGHT as u8 + (64 - 57), 0xA8, 57 - 1];
static mut POST_FRAME_ADJ: [[u8; 2]; 3] = [[0x81, 0]; 3];
static mut POST_FRAME_ADJ_SRC: [u8; 3] = [0; 3];
static mut PENDING_CMDS: [u8; 8] = [0; 8];

/// Module containing the data structure for the Thumby's display colors
pub mod color {
    /// Data structure containing the color options for the Thumby's OLED display
    #[derive(Copy, Clone)]
    pub enum Color {
        /// Black color
        Black,
        /// White color
        White,
        /// Dark gray color
        DarkGray,
        /// Light gray color
        LightGray,
    }
}

#[derive(PartialEq)]
enum ThreadState {
    Stopped,
    Starting,
    Running,
    Stopping,
}

struct State {
    thread: ThreadState,
    copy_buffers: u8,
    pending_cmd: u8,
    contrast: u8,
    invert: bool,
}

pub struct SSD1306 {
    spi: Spi<Enabled, SPI0, 8>,
    cs: Pin<Gpio16, PushPullOutput>,
    dc: Pin<Gpio17, PushPullOutput>,
    rst: Pin<Gpio20, PushPullOutput>,
    delay: Delay,
    timer: Timer,

    initialised: bool,
    brightness: u32,
    last_update_end: Instant,
    frame_rate: u8,
}

impl SSD1306 {
    pub fn new(
        cs: Pin<Gpio16, PushPullOutput>,
        dc: Pin<Gpio17, PushPullOutput>,
        rst: Pin<Gpio20, PushPullOutput>,
        watcher: WATCHDOG,
        xosc: XOSC,
        clks: CLOCKS,
        pll_sys: PLL_SYS,
        pll_usb: PLL_USB,
        spi0: SPI0,
        mut resets: RESETS,
        tmr: TIMER,
        syst: SYST,
    ) -> Self {
        let mut watchdog = Watchdog::new(watcher);

        // Configure the clocks
        let clocks = clocks::init_clocks_and_plls(
            12_000_000u32,
            xosc,
            clks,
            pll_sys,
            pll_usb,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .expect("Failed to initialize clocks");

        let spi = Spi::<_, _, 8>::new(spi0).init(
            &mut resets,
            clocks.peripheral_clock.freq(),
            (100 * 1000 * 1000).Hz(),
            &MODE_0,
        );

        let timer = Timer::new(tmr, &mut resets);

        let delay = cortex_m::delay::Delay::new(syst, clocks.system_clock.freq().to_Hz());

        let last_update_end = timer.get_counter();
        Self {
            spi,
            cs,
            dc,
            rst,
            delay,
            timer,

            initialised: false,
            brightness: 127,
            last_update_end,
            frame_rate: 60,
        }
    }

    pub fn reset(&mut self) {
        self.rst.set_high().unwrap();
        self.delay.delay_ms(1);
        self.rst.set_low().unwrap();
        self.delay.delay_ms(10);
        self.rst.set_high().unwrap();
        self.delay.delay_ms(10);
    }

    pub fn init(&mut self) {
        self.brightness(self.brightness);
        self.dc.set_low().unwrap();
        if self.initialised {
            unsafe {
                if STATE.thread == ThreadState::Stopped {
                    self.spi.write(&[0xA8, 0, 0xD3, 52]).unwrap();
                    self.delay.delay_us(FRAME_TIME_US * 3);
                    self.spi.write(&[0xA8, HEIGHT as u8 - 1, 0xD3, 0]).unwrap();
                    if STATE.invert {
                        self.spi.write(&[0xA6 | 1]).unwrap();
                    }
                } else {
                    self.spi.write(&[0xAE, 0xA8, 0, 0xD3, 0, 0xAF]).unwrap();
                }
            }
            return;
        }

        self.reset();
        self.cs.set_low().unwrap();
        self.spi
            .write(&[
                0xAE, 0x20, 0x00, 0x40, 0xA1, 0xA8, 63, 0xC8, 0xD3, 0, 0xDA, 0x12, 0xD5, 0xF0,
                0xD9, 0x11, 0xDB, 0x20, 0x81, 0x7F, 0xA4, 0xA6, 0x8D, 0x14, 0xAD, 0x30, 0xAF,
            ])
            .unwrap();
        self.dc.set_high().unwrap();
        let zero32 = [0; 32];
        for _ in 0..32 {
            self.spi.write(&zero32).unwrap();
        }
        self.dc.set_low().unwrap();
        self.spi.write(&[0x21, 28, 99, 0x22, 0, 4]).unwrap();
        self.initialised = true;
    }

    pub fn enable_grayscale(&mut self) {
        if unsafe { STATE.thread == ThreadState::Running } {
            return;
        }

        let mut pac = unsafe { Peripherals::steal() };
        let mut sio = Sio::new(pac.SIO);

        unsafe { STATE.thread = ThreadState::Starting };
        self.init();

        let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
        let core1 = &mut mc.cores()[1];
        core1
            .spawn(unsafe { &mut CORE1_STACK.mem }, display_thread)
            .unwrap();

        // while unsafe { STATE.thread != ThreadState::Running } {
        //     asm::wfe();
        // }
    }

    pub fn disable_grayscale(&mut self) {
        unsafe {
            if STATE.thread != ThreadState::Running {
                return;
            }
            STATE.thread = ThreadState::Stopping;
            while STATE.thread != ThreadState::Stopped {
                asm::wfe();
            }
        }
        self.init();
        self.show();
        self.brightness(self.brightness);
    }

    pub fn write_cmd(&mut self, cmd: &[u8]) {
        unsafe {
            if STATE.thread == ThreadState::Running {
                if cmd.len() > PENDING_CMDS.len() {
                    panic!(
                        "Cannot send more than {} bytes using write_cmd()",
                        PENDING_CMDS.len()
                    );
                }
                let mut i = 0;
                while i < cmd.len() {
                    PENDING_CMDS[i] = cmd[i];
                    i += 1;
                }
                while i < PENDING_CMDS.len() {
                    PENDING_CMDS[i] = 0x3E;
                    i += 1;
                }
                STATE.pending_cmd = 1;
                // while STATE.pending_cmd > 0 {
                //     asm::wfe();
                // }
            } else {
                self.dc.set_low().unwrap();
                self.spi.write(cmd).unwrap();
            }
        }
    }

    pub fn power_off(&mut self) {
        self.write_cmd(&[0xAE]);
    }

    pub fn power_on(&mut self) {
        self.write_cmd(&[0xAF]);
    }

    pub fn invert(&mut self, invert: bool) {
        unsafe {
            STATE.invert = invert;
            STATE.copy_buffers = 1;
            let invert = if invert { 1 } else { 0 };
            if STATE.thread != ThreadState::Running {
                self.write_cmd(&[0xA6 | invert]);
            }
        }
    }

    pub fn show(&mut self) {
        unsafe {
            if STATE.thread == ThreadState::Running {
                STATE.copy_buffers = 1;
                // while STATE.copy_buffers != 0 {
                //     asm::wfe();
                // }
            } else {
                self.dc.set_high().unwrap();
                self.spi.write(&BUFFER).unwrap();
            }
        }
    }

    pub fn show_async(&mut self) {
        unsafe {
            if STATE.thread == ThreadState::Running {
                STATE.copy_buffers = 1;
            } else {
                self.show();
            }
        }
    }

    pub fn set_fps(&mut self, fps: u8) {
        self.frame_rate = fps;
    }

    pub fn update(&mut self) {
        self.show();
        if self.frame_rate > 0 {
            let frame_time_ms = 1000 / self.frame_rate as i64;
            let mut frame_time_remaining = frame_time_ms
                - (self.timer.get_counter() - self.last_update_end).to_millis() as i64;
            while frame_time_remaining > 0 {
                frame_time_remaining = frame_time_ms
                    - (self.timer.get_counter() - self.last_update_end).to_millis() as i64;
            }
        }
        self.last_update_end = self.timer.get_counter();
    }

    pub fn brightness(&mut self, mut c: u32) {
        if c > 127 {
            c = 127;
        }
        let cc = floor(sqrt(f64::from(c << 17))) as u32;
        unsafe {
            POST_FRAME_ADJ_SRC[0] = (((cc * 30) >> 12) + 6) as u8;
            POST_FRAME_ADJ_SRC[1] = (((cc * 72) >> 12) + 14) as u8;
            let c3 = (cc * 340 >> 12) + 20;
            POST_FRAME_ADJ_SRC[2] = if c3 < 255 { c3 as u8 } else { 255 };

            if STATE.thread == ThreadState::Running {
                STATE.contrast = 1;
            } else {
                POST_FRAME_ADJ[0][1] = POST_FRAME_ADJ_SRC[0];
                POST_FRAME_ADJ[1][1] = POST_FRAME_ADJ_SRC[1];
                POST_FRAME_ADJ[2][1] = POST_FRAME_ADJ_SRC[2];
                self.write_cmd(&[0x81, c as u8]);
                self.brightness = c;
            }
        }
    }

    pub fn fill(&mut self, color: Color) {
        let color = color as u8;
        let f1 = if (color & 1) > 0 { 255 } else { 0 };
        let f2 = if (color & 2) > 0 { 255 } else { 0 };
        for i in 0..BUFF_INT_SZ {
            unsafe {
                BUFFER[i] = f1;
                SHADING[i] = f2;
            }
        }
    }

    pub fn draw_filled_rectangle(
        &mut self,
        x: u8,
        y: u8,
        mut width: u8,
        mut height: u8,
        color: Color,
    ) {
        if x >= WIDTH as u8 || y >= HEIGHT as u8 {
            return;
        }
        let mut x2 = x + width;
        let y2 = y + height;
        if x2 > WIDTH as u8 {
            x2 = WIDTH as u8;
            width = WIDTH as u8 - x
        }
        if y2 > HEIGHT as u8 {
            height = HEIGHT as u8 - y;
        }

        let mut o = (y >> 3) * WIDTH as u8;
        let mut oe = o + x2;
        o += x;
        let strd = WIDTH as u8 - width;

        let color = color as u8;
        let c1 = (color & 8) > 0;
        let c2 = (color & 2) > 0;
        let v1 = if c1 { 0xFF } else { 0 };
        let v2 = if c2 { 0xFF } else { 0 };

        let yb = y & 7;
        let ybh = 8 - yb;
        let mut m: u8;
        if height <= ybh {
            m = ((1 << height) - 1) << yb;
        } else {
            m = 0xFF << yb;
        }
        let mut im = 255 - m;
        while o < oe {
            unsafe {
                if c1 {
                    BUFFER[o as usize] |= m as u8;
                } else {
                    BUFFER[o as usize] &= im as u8;
                }
                if c2 {
                    SHADING[o as usize] |= m as u8;
                } else {
                    SHADING[o as usize] &= im as u8;
                }
            }
            o += 1;
        }
        height -= ybh;
        while height >= 8 {
            o += strd;
            oe += WIDTH as u8;
            while o < oe {
                unsafe {
                    BUFFER[o as usize] = v1;
                    SHADING[o as usize] = v2;
                }
                o += 1;
            }
            height -= 8;
        }
        if height > 0 {
            o += strd;
            oe += WIDTH as u8;
            m = (1 << height) - 1;
            im = 255 - m;
            while o < oe {
                unsafe {
                    if c1 {
                        BUFFER[o as usize] |= m as u8;
                    } else {
                        BUFFER[o as usize] &= im as u8;
                    }
                    if c2 {
                        SHADING[o as usize] |= m as u8;
                    } else {
                        SHADING[o as usize] &= im as u8;
                    }
                    o += 1;
                }
            }
        }
    }

    pub fn draw_rectangle(&mut self, x: u8, y: u8, width: u8, height: u8, color: Color) {
        self.draw_filled_rectangle(x, y, width, 1, color);
        self.draw_filled_rectangle(x, y, 1, height, color);
        self.draw_filled_rectangle(x, y + height - 1, width, 1, color);
        self.draw_filled_rectangle(x + width - 1, y, 1, height, color);
    }

    pub fn set_pixel(&mut self, x: usize, y: usize, color: Color) {
        if x >= WIDTH || y >= HEIGHT {
            return;
        }
        let color = color as u8;
        let o = (y >> 3) * WIDTH + x;
        let m = 1 << (y & 7);
        let im = 255 - m;
        unsafe {
            if (color & 1) > 0 {
                BUFFER[o] |= m;
            } else {
                BUFFER[o] &= im;
            }

            if (color & 2) > 0 {
                SHADING[o] |= m;
            } else {
                SHADING[o] &= im;
            }
        }
    }

    pub fn draw_line(&mut self, mut x0: u8, mut y0: u8, mut x1: u8, mut y1: u8, color: Color) {
        if x0 == x1 {
            self.draw_filled_rectangle(x0, y0, 1, y1 - y0, color);
            return;
        }
        if y0 == y1 {
            self.draw_filled_rectangle(x0, y0, x1 - x0, 1, color);
            return;
        }
        let mut dx = (x1 - x0) as i16;
        let mut dy = (y1 - y0) as i16;
        let mut sx: i8 = 1;
        if dy < 0 {
            (x0, x1) = (x1, x0);
            (y0, y1) = (y1, y0);
            dy = -dy;
            dx = -dx;
        }
        if dx < 0 {
            dx = -dx;
            sx = -1;
        }
        let mut x = x0;
        let mut y = y0;

        let mut o = (y >> 3) * WIDTH as u8 + x;
        let mut m = 1 << (y & 7);
        let mut im = 255 - m;
        let color = color as u8;
        let c1 = color & 1;
        let c2 = color & 2;

        let mut err: i16;
        if dx > dy {
            err = dx >> 1;
            x1 += 1;
            while x != x1 {
                if x < WIDTH as u8 && y < HEIGHT as u8 {
                    unsafe {
                        if c1 > 0 {
                            BUFFER[o as usize] |= m;
                        } else {
                            BUFFER[o as usize] &= im;
                        }
                        if c2 > 0 {
                            SHADING[o as usize] |= m;
                        } else {
                            SHADING[o as usize] &= im;
                        }
                    }
                }
                err -= dy;
                if err < 0 {
                    y += 1;
                    m <<= 1;
                    if (m as u16 & 0x100) > 0 {
                        o += WIDTH as u8;
                        m = 1;
                        im = 0xFE;
                    } else {
                        im = 255 - m;
                    }
                    err += dx;
                }
                x += sx as u8;
                o += sx as u8;
            }
        } else {
            err = dy >> 1;
            y1 += 1;
            while y != y1 {
                if x < WIDTH as u8 && y < HEIGHT as u8 {
                    unsafe {
                        if c1 > 0 {
                            BUFFER[o as usize] |= m;
                        } else {
                            BUFFER[o as usize] &= im;
                        }
                        if c2 > 0 {
                            SHADING[o as usize] |= m;
                        } else {
                            SHADING[o as usize] &= im;
                        }
                    }
                }
                err -= dx;
                if err < 0 {
                    x += sx as u8;
                    o += sx as u8;
                    err += dy;
                }
                y += 1;
                m <<= 1;
                if (m as u16 & 0x100) > 0 {
                    o += WIDTH as u8;
                    m = 1;
                    im = 0xFE;
                } else {
                    im = 255 - m;
                }
            }
        }
    }

    pub fn wait_ms(&mut self, ms: u32) {
        self.delay.delay_ms(ms);
    }
}

fn display_thread() -> ! {
    unsafe {
        let pac = Peripherals::steal();
        let spi0 = pac.SPI0;
        let sio = pac.SIO;
        let tmr = pac.TIMER;

        STATE.thread = ThreadState::Running;
        {
            let mut pac = Peripherals::steal();
            let sio = Sio::new(pac.SIO);
            let pins = Pins::new(
                pac.IO_BANK0,
                pac.PADS_BANK0,
                sio.gpio_bank0,
                &mut pac.RESETS,
            );
            let pwm_slices = rp2040_hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
            let mut pwm = pwm_slices.pwm6;
            pwm.enable();
            let mut audio = crate::audio::Audio::new(pwm, pins.gpio28);
            if STATE.thread != ThreadState::Running {
                audio.play(crate::music::C);
            } else {
                audio.play(crate::music::B);
            }
        }
        while STATE.thread == ThreadState::Running {
            let mut framebuffer = 0;
            while framebuffer < 3 {
                let mut time_out = tmr.timerawl.read().bits() + PRE_FRAME_TIME_US;
                sio.gpio_out_xor.write(|w| w.bits(1 << 17));
                let mut i = 0;
                while i < 4 {
                    while (spi0.sspsr.read().bits() & 2) == 0 {}
                    spi0.sspdr.write(|w| w.bits(PRE_FRAME_CMDS[i].into()));
                    i += 1;
                }
                while (spi0.sspsr.read().bits() & 4) == 4 {
                    // i = spi0.sspdr.read().bits() as usize;
                }
                while (spi0.sspsr.read().bits() & 0x10) == 0x10 {}
                while (spi0.sspsr.read().bits() & 4) == 4 {
                    // i = spi0.sspdr.read().bits() as usize;
                }

                sio.gpio_out_clr.write(|w| w.bits(1 << 17));
                i = 0;
                let spibuff = SUBFRAMES[framebuffer];
                while i < 360 {
                    while (spi0.sspsr.read().bits() & 2) == 0 {}
                    spi0.sspdr.write(|w| w.bits(spibuff[i] as u32));
                    i += 1;
                }
                while (spi0.sspsr.read().bits() & 4) == 4 {
                    // i = spi0.sspdr.read().bits() as usize;
                }
                while (spi0.sspsr.read().bits() & 0x10) == 0x10 {}
                while (spi0.sspsr.read().bits() & 4) == 4 {
                    // i = spi0.sspdr.read().bits() as usize;
                }

                sio.gpio_out_xor.write(|w| w.bits(1 << 17));
                i = 0;
                let spibuff = POST_FRAME_ADJ[framebuffer];
                while i < 2 {
                    while (spi0.sspsr.read().bits() & 2) == 0 {}
                    spi0.sspdr.write(|w| w.bits(spibuff[i].into()));
                    i += 1;
                }
                while (spi0.sspsr.read().bits() & 4) == 4 {
                    // i = spi0.sspdr.read().bits() as usize;
                }
                while (spi0.sspsr.read().bits() & 0x10) == 0x10 {}
                while (spi0.sspsr.read().bits() & 4) == 4 {
                    // i = spi0.sspdr.read().bits() as usize;
                }

                while tmr.timerawl.read().bits() < time_out {}

                time_out = tmr.timerawl.read().bits() + FRAME_TIME_US;

                i = 0;
                while i < 4 {
                    while (spi0.sspsr.read().bits() & 2) == 0 {}
                    spi0.sspdr.write(|w| w.bits(POST_FRAME_CMDS[i].into()));
                    i += 1;
                }
                i = 0;
                let spibuff = POST_FRAME_ADJ[framebuffer];
                while i < 2 {
                    while (spi0.sspsr.read().bits() & 2) == 0 {}
                    spi0.sspdr.write(|w| w.bits(spibuff[i].into()));
                    i += 1;
                }
                while (spi0.sspsr.read().bits() & 4) == 4 {
                    // i = spi0.sspdr.read().bits() as usize;
                }
                while (spi0.sspsr.read().bits() & 0x10) == 0x10 {}
                while (spi0.sspsr.read().bits() & 4) == 4 {
                    // i = spi0.sspdr.read().bits() as usize;
                }

                if framebuffer == 2 {
                    if STATE.copy_buffers != 0 {
                        i = 0;
                        let inv = if STATE.invert { -1 } else { 0 };
                        while i < BUFF_INT_SZ {
                            let v1 = BUFFER[i] as i8 ^ inv;
                            let v2 = SHADING[i] as i8;
                            SUBFRAMES[0][i] = (v1 | v2) as u8;
                            SUBFRAMES[1][i] = v1 as u8;
                            SUBFRAMES[2][i] = (v1 & (v1 ^ v2)) as u8;
                            i += 1;
                        }
                        STATE.copy_buffers = 0;
                    }
                    if STATE.contrast != 0 {
                        POST_FRAME_ADJ[0][1] = POST_FRAME_ADJ_SRC[0];
                        POST_FRAME_ADJ[1][1] = POST_FRAME_ADJ_SRC[1];
                        POST_FRAME_ADJ[2][1] = POST_FRAME_ADJ_SRC[2];
                        STATE.contrast = 0;
                    } else if STATE.pending_cmd != 0 {
                        i = 0;
                        while i < 8 {
                            while (spi0.sspsr.read().bits() & 2) == 0 {}
                            spi0.sspdr.write(|w| w.bits(PENDING_CMDS[i].into()));
                            i += 1;
                        }
                        while (spi0.sspsr.read().bits() & 4) == 4 {
                            // i = spi0.sspdr.read().bits() as usize;
                        }
                        while (spi0.sspsr.read().bits() & 0x10) == 0x10 {}
                        while (spi0.sspsr.read().bits() & 4) == 4 {
                            // i = spi0.sspdr.read().bits() as usize;
                        }
                        STATE.pending_cmd = 0;
                    }
                }

                while tmr.timerawl.read().bits() < time_out {}

                framebuffer += 1;
            }
        }

        STATE.thread = ThreadState::Stopped;
    }

    loop {
        asm::wfe();
    }
}
