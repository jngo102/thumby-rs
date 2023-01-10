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

#[derive(Clone, Copy, Debug, PartialEq)]
enum ThreadState {
    Stopped,
    Starting,
    Running,
    Stopping,
}

#[derive(Clone, Copy, Debug)]
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
    state: State,
    buffer: [u8; BUFF_SZ],
    shading: [u8; BUFF_SZ],
    subframes: [[u8; BUFF_SZ]; 3],
    pre_frame_cmds: [u8; 4],
    post_frame_cmds: [u8; 4],
    post_frame_adj: [[u8; 2]; 3],
    post_frame_adj_src: [u8; 3],
    pending_cmds: [u8; 8],
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
            state: State {
                thread: ThreadState::Stopped,
                copy_buffers: 0,
                pending_cmd: 0,
                contrast: 0,
                invert: false,
            },
            buffer: [0; BUFF_SZ],
            shading: [0; BUFF_SZ],
            subframes: [[0; BUFF_SZ]; 3],
            pre_frame_cmds: [0xA8, 0, 0xD3, 52],
            post_frame_cmds: [0xD3, HEIGHT as u8 + (64 - 57), 0xA8, 57 - 1],
            post_frame_adj: [[0x81, 0]; 3],
            post_frame_adj_src: [0; 3],
            pending_cmds: [0; 8],
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
            if self.state.thread == ThreadState::Stopped {
                self.spi.write(&[0xA8, 0, 0xD3, 52]).unwrap();
                self.delay.delay_us(FRAME_TIME_US * 3);
                self.spi.write(&[0xA8, HEIGHT as u8 - 1, 0xD3, 0]).unwrap();
                if self.state.invert {
                    self.spi.write(&[0xA6 | 1]).unwrap();
                }
            } else {
                self.spi.write(&[0xAE, 0xA8, 0, 0xD3, 0, 0xAF]).unwrap();
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
        if self.state.thread == ThreadState::Running {
            return;
        }

        let mut pac = unsafe { Peripherals::steal() };
        let mut sio = Sio::new(pac.SIO);

        self.state.thread = ThreadState::Starting;
        self.init();

        let mut state = self.state.clone();
        let buffer = self.buffer.clone();
        let shading = self.shading.clone();
        let mut subframes = self.subframes.clone();
        let pre_frame_cmds = self.pre_frame_cmds.clone();
        let post_frame_cmds = self.post_frame_cmds.clone();
        let mut post_frame_adj = self.post_frame_adj.clone();
        let post_frame_adj_src = self.post_frame_adj_src.clone();
        let pending_cmds = self.pending_cmds.clone();
        let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
        let core1 = &mut mc.cores()[1];
        core1
            .spawn(unsafe { &mut CORE1_STACK.mem }, move || {
                state.thread = ThreadState::Running;

                loop {
                    display_thread(
                        &mut state,
                        &buffer,
                        &shading,
                        &mut subframes,
                        &pre_frame_cmds,
                        &post_frame_cmds,
                        &mut post_frame_adj,
                        &post_frame_adj_src,
                        &pending_cmds,
                    );
                }
            })
            .unwrap();

        // while self.state.thread != ThreadState::Running {
        //     self.state = state;
        //     self.subframes = subframes;
        //     self.post_frame_adj = post_frame_adj;
        // }
    }

    pub fn disable_grayscale(&mut self) {
        if self.state.thread != ThreadState::Running {
            return;
        }
        self.state.thread = ThreadState::Stopping;
        while self.state.thread != ThreadState::Stopped {
            asm::wfe();
        }
        self.init();
        self.show();
        self.brightness(self.brightness);
    }

    pub fn write_cmd(&mut self, cmd: &[u8]) {
        if self.state.thread == ThreadState::Running {
            if cmd.len() > self.pending_cmds.len() {
                panic!(
                    "Cannot send more than {} bytes using write_cmd()",
                    self.pending_cmds.len()
                );
            }
            let mut i = 0;
            while i < cmd.len() {
                self.pending_cmds[i] = cmd[i];
                i += 1;
            }
            while i < self.pending_cmds.len() {
                self.pending_cmds[i] = 0x3E;
                i += 1;
            }
            self.state.pending_cmd = 1;
            while self.state.pending_cmd > 0 {
                asm::wfe();
            }
        } else {
            self.dc.set_low().unwrap();
            self.spi.write(cmd).unwrap();
        }
    }

    pub fn power_off(&mut self) {
        self.write_cmd(&[0xAE]);
    }

    pub fn power_on(&mut self) {
        self.write_cmd(&[0xAF]);
    }

    pub fn invert(&mut self, invert: bool) {
        self.state.invert = invert;
        self.state.copy_buffers = 1;
        let invert = if invert { 1 } else { 0 };
        if self.state.thread != ThreadState::Running {
            self.write_cmd(&[0xA6 | invert]);
        }
    }

    pub fn show(&mut self) {
        if self.state.thread == ThreadState::Running {
            self.state.copy_buffers = 1;
            // while self.state.copy_buffers != 0 {
            //     asm::wfe();
            // }
        } else {
            self.dc.set_high().unwrap();
            self.spi.write(&self.buffer).unwrap();
        }
    }

    pub fn show_async(&mut self) {
        if self.state.thread == ThreadState::Running {
            self.state.copy_buffers = 1;
        } else {
            self.show();
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
        self.post_frame_adj_src[0] = (((cc * 30) >> 12) + 6) as u8;
        self.post_frame_adj_src[1] = (((cc * 72) >> 12) + 14) as u8;
        let c3 = (cc * 340 >> 12) + 20;
        self.post_frame_adj_src[2] = if c3 < 255 { c3 as u8 } else { 255 };

        if self.state.thread == ThreadState::Running {
            self.state.contrast = 1;
        } else {
            self.post_frame_adj[0][1] = self.post_frame_adj_src[0];
            self.post_frame_adj[1][1] = self.post_frame_adj_src[1];
            self.post_frame_adj[2][1] = self.post_frame_adj_src[2];
            self.write_cmd(&[0x81, c as u8]);
            self.brightness = c;
        }
    }

    pub fn fill(&mut self, color: Color) {
        let color = color as u8;
        let f1 = if (color & 1) > 0 { 255 } else { 0 };
        let f2 = if (color & 2) > 0 { 255 } else { 0 };
        for i in 0..BUFF_INT_SZ {
            self.buffer[i] = f1;
            self.shading[i] = f2;
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
            if c1 {
                self.buffer[o as usize] |= m as u8;
            } else {
                self.buffer[o as usize] &= im as u8;
            }
            if c2 {
                self.shading[o as usize] |= m as u8;
            } else {
                self.shading[o as usize] &= im as u8;
            }
            o += 1;
        }
        height -= ybh;
        while height >= 8 {
            o += strd;
            oe += WIDTH as u8;
            while o < oe {
                self.buffer[o as usize] = v1;
                self.shading[o as usize] = v2;
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
                if c1 {
                    self.buffer[o as usize] |= m as u8;
                } else {
                    self.buffer[o as usize] &= im as u8;
                }
                if c2 {
                    self.shading[o as usize] |= m as u8;
                } else {
                    self.shading[o as usize] &= im as u8;
                }
                o += 1;
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
        if (color & 1) > 0 {
            self.buffer[o] |= m;
        } else {
            self.buffer[o] &= im;
        }

        if (color & 2) > 0 {
            self.shading[o] |= m;
        } else {
            self.shading[o] &= im;
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
                    if c1 > 0 {
                        self.buffer[o as usize] |= m;
                    } else {
                        self.buffer[o as usize] &= im;
                    }
                    if c2 > 0 {
                        self.shading[o as usize] |= m;
                    } else {
                        self.shading[o as usize] &= im;
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
                    if c1 > 0 {
                        self.buffer[o as usize] |= m;
                    } else {
                        self.buffer[o as usize] &= im;
                    }
                    if c2 > 0 {
                        self.shading[o as usize] |= m;
                    } else {
                        self.shading[o as usize] &= im;
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

fn display_thread(
    state: &mut State,
    buffer: &[u8; BUFF_SZ],
    shading: &[u8; BUFF_SZ],
    subframes: &mut [[u8; BUFF_SZ]; 3],
    pre_frame_cmds: &[u8; 4],
    post_frame_cmds: &[u8; 4],
    post_frame_adj: &mut [[u8; 2]; 3],
    post_frame_adj_src: &[u8; 3],
    pending_cmds: &[u8; 8],
) {
    let pac = unsafe { Peripherals::steal() };
    let spi0 = pac.SPI0;
    let sio = pac.SIO;
    let tmr = pac.TIMER;

    let mut framebuffer = 0;
    while framebuffer < 3 {
        let mut time_out = tmr.timerawl.read().bits() + PRE_FRAME_TIME_US;
        sio.gpio_out_xor.write(|w| unsafe { w.bits(1 << 17) });
        let mut i = 0;
        while i < 4 {
            while (spi0.sspsr.read().bits() & 2) == 0 {}
            spi0.sspdr
                .write(|w| unsafe { w.bits(pre_frame_cmds[i].into()) });
            i += 1;
        }
        while (spi0.sspsr.read().bits() & 4) == 4 {
            // i = spi0.sspdr.read().bits() as usize;
        }
        while (spi0.sspsr.read().bits() & 0x10) == 0x10 {}
        while (spi0.sspsr.read().bits() & 4) == 4 {
            // i = spi0.sspdr.read().bits() as usize;
        }

        sio.gpio_out_clr.write(|w| unsafe { w.bits(1 << 17) });
        i = 0;
        let spibuff = subframes[framebuffer];
        while i < 360 {
            while (spi0.sspsr.read().bits() & 2) == 0 {}
            spi0.sspdr.write(|w| unsafe { w.bits(spibuff[i] as u32) });
            i += 1;
        }
        while (spi0.sspsr.read().bits() & 4) == 4 {
            // i = spi0.sspdr.read().bits() as usize;
        }
        while (spi0.sspsr.read().bits() & 0x10) == 0x10 {}
        while (spi0.sspsr.read().bits() & 4) == 4 {
            // i = spi0.sspdr.read().bits() as usize;
        }

        sio.gpio_out_xor.write(|w| unsafe { w.bits(1 << 17) });
        i = 0;
        let spibuff = post_frame_adj[framebuffer];
        while i < 2 {
            while (spi0.sspsr.read().bits() & 2) == 0 {}
            spi0.sspdr.write(|w| unsafe { w.bits(spibuff[i].into()) });
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
            spi0.sspdr
                .write(|w| unsafe { w.bits(post_frame_cmds[i].into()) });
            i += 1;
        }
        i = 0;
        let spibuff = post_frame_adj[framebuffer];
        while i < 2 {
            while (spi0.sspsr.read().bits() & 2) == 0 {}
            spi0.sspdr.write(|w| unsafe { w.bits(spibuff[i].into()) });
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
            if state.copy_buffers != 0 {
                i = 0;
                let inv = if state.invert { -1 } else { 0 };
                while i < BUFF_INT_SZ {
                    let v1 = buffer[i] as i8 ^ inv;
                    let v2 = shading[i] as i8;
                    subframes[0][i] = (v1 | v2) as u8;
                    subframes[1][i] = v1 as u8;
                    subframes[2][i] = (v1 & (v1 ^ v2)) as u8;
                    i += 1;
                }
                state.copy_buffers = 0;
            }
            if state.contrast != 0 {
                post_frame_adj[0][1] = post_frame_adj_src[0];
                post_frame_adj[1][1] = post_frame_adj_src[1];
                post_frame_adj[2][1] = post_frame_adj_src[2];
                state.contrast = 0;
            } else if state.pending_cmd != 0 {
                i = 0;
                while i < 8 {
                    while (spi0.sspsr.read().bits() & 2) == 0 {}
                    spi0.sspdr
                        .write(|w| unsafe { w.bits(pending_cmds[i].into()) });
                    i += 1;
                }
                while (spi0.sspsr.read().bits() & 4) == 4 {
                    // i = spi0.sspdr.read().bits() as usize;
                }
                while (spi0.sspsr.read().bits() & 0x10) == 0x10 {}
                while (spi0.sspsr.read().bits() & 4) == 4 {
                    // i = spi0.sspdr.read().bits() as usize;
                }
                state.pending_cmd = 0;
            }
        }

        while tmr.timerawl.read().bits() < time_out {}

        framebuffer += 1;
    }
}
