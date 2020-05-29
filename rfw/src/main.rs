#![no_std]
#![no_main]

use panic_ramdump as _;
use cortex_m_rt::entry;
use cortex_m;

use tinytga::Tga;

use stm32f7xx_hal::{
    self as hal,
    device::{self, LTDC, DMA2D},
    ltdc::{DisplayConfig, Layer, DisplayController, SupportedWord, PixelFormat},
    gpio::Speed,
    time::U32Ext,
    rcc::Rcc,
    spi::{self, Spi, NoMiso},
    delay::Delay,
    prelude::*,
};

use embedded_graphics::{
    prelude::*,
    drawable::Pixel,
    geometry::Size,
    pixelcolor::{Rgb888, RgbColor},
    style::{PrimitiveStyle, Styled},
    image::Image,
    DrawTarget,
    primitives,
    egcircle, egrectangle, egtext,
    fonts::Font12x16,
    primitive_style, text_style,

};

const WIDTH: u16 = 320;
const HEIGHT: u16 = 272;

const FB_GRAPHICS_SIZE: usize = (WIDTH as usize) * (HEIGHT as usize);
static mut FB_LAYER1: [u32; FB_GRAPHICS_SIZE] = [0; FB_GRAPHICS_SIZE];

pub const SCREEN_CONFIG: DisplayConfig = DisplayConfig {
    active_width: WIDTH,
    active_height: HEIGHT,
    h_back_porch: 20,
    h_front_porch: 10,
    h_sync: 10,
    v_back_porch: 2,
    v_front_porch: 4,
    v_sync: 2,
    frame_rate: 30,
    h_sync_pol: false,
    v_sync_pol: false,
    no_data_enable_pol: false,
    pixel_clock_pol: false,
};

pub struct LinetimeDisplay<T: 'static + SupportedWord> {
    pub controller: DisplayController<T>,
}

impl<T: 'static + SupportedWord> LinetimeDisplay<T> {
    pub fn new(ltdc: LTDC, dma2d: DMA2D) -> LinetimeDisplay<T> {
        let controller = DisplayController::new(
            ltdc,
            dma2d,
            PixelFormat::ARGB8888,
            SCREEN_CONFIG,
            None,
        );

        LinetimeDisplay {
            controller,
        }
    }
}

impl DrawTarget<Rgb888> for LinetimeDisplay<u32> {
    type Error = core::convert::Infallible;

    /// Draw a `Pixel` that has a color defined
    fn draw_pixel(&mut self, pixel: Pixel<Rgb888>) -> Result<(), Self::Error> {
        let Pixel(coord, color) = pixel;
        let value: u32 = (color.b() as u32 & 0xFF)
            | ((color.g() as u32 & 0xFF) << 8)
            | ((color.r() as u32 & 0xFF) << 16)
            | ((0xFFu32) << 24);

        // TODO : draw pixel
        self.controller
            .draw_pixel(Layer::L1, coord.x as usize, coord.y as usize, value);
        Ok(())
    }

    /// Draw a hardware accelerated (by DMA2D) rectangle
    /*
    fn draw_rectangle(&mut self, item: &Styled<primitives::Rectangle, PrimitiveStyle<Rgb888>>) -> Result<(), Self::Error> {
        if item.style.stroke_color.is_none() {
            let top_left = (item.primitive.top_left.x as usize, item.primitive.top_left.y as usize);
            let bottom_right = (item.primitive.bottom_right.x as usize, item.primitive.bottom_right.y as usize);
            let color = match item.style.fill_color {
                Some(c) =>  {
                    (c.b() as u32 & 0xFF) |
                    ((c.g() as u32 & 0xFF) << 8) |
                    ((c.r() as u32 & 0xFF) << 16) |
                    (0xFF << 24)
                },
                None => 0u32
            };

            // Note(unsafe) because transfert might not be before an other write
            // to the buffer occurs. However, such Register -> Buffer transfert
            // is so fast that such issue does not occur
            // TODO : use safer DMA api when the embedde-hal DMA traits will be stabilised
            unsafe {self.controller.draw_rectangle(Layer::L1, top_left, bottom_right, color);}
        } else {
            self.draw_iter(item).unwrap();
        }

        Ok(())
    }
    */

    /// Return the sise of the screen
    fn size(&self) -> Size {
        Size::new(480, 272)
    }
}

struct ILI9342<'a, I: spi::Instance, P: spi::Pins<I>> {
    spi: &'a mut Spi<I, P, spi::Enabled<u8>>,
    rst: &'a mut hal::gpio::gpioe::PE5<hal::gpio::Output<hal::gpio::PushPull>>,
    cs: &'a mut hal::gpio::gpioe::PE4<hal::gpio::Output<hal::gpio::PushPull>>,
    dcx: &'a mut hal::gpio::gpioe::PE3<hal::gpio::Output<hal::gpio::PushPull>>,
    delay: &'a mut Delay,
}

impl <'a, I: spi::Instance, P: spi::Pins<I>> ILI9342<'a, I, P>
{
    pub fn init(&mut self) {

        // Hardware reset
        self.rst.set_low().ok();
        self.delay.delay_ms(20u16);
        self.rst.set_high().ok();
        self.delay.delay_ms(120u16);

        // Assert CS
        self.cs.set_low().ok();

        // Software reset
        self.write_index(0x01);
        self.delay.delay_ms(20u16);

        /* Display off */
        self.write_index(0x28);

        /* Enable extended commands. */
        self.write_index(0xC8);
            self.write_data(0xFF);
            self.write_data(0x93);
            self.write_data(0x42);

        /* Memory access control */
        self.write_index(0x36);
            self.write_data(0x18);

        /* Pixel format */
        self.write_index(0x3A);
            self.write_data(0x66);

        /* RGB */
        self.write_index(0xB0);
            self.write_data(0x40);

        /* Display Waveform Cycle 1 (normal/full colour mode) */
        self.write_index(0xB1);
            self.write_data(0x00); /* DIVA = clock division ratio = 1/1 */
            self.write_data(0x10); /* RTNA = clocks per line      = 16  */

        /* Display Function Control.
         * The fouth parameter PCDIV is timing critical and very poorly documented.
         * It appears to be a single 6-bit value for PCDIV, rather than the
         * 4-bit PCDIVH and 4-bit PCDIVL that appears elsewhere in the
         * documentation (from an older controller perhaps).
         * Valid values don't seem to follow what you'd expect from the datasheet
         * documentation either. Needs changing when you change DOTCLK, which is
         * the TFT LCD clock, which is HSE /M *SAIN /SAIR /SAIDIVR.
         * For DOTCLK=2.5MHz (SAIDIVR=16), use 0x02 to get ~28fps.
         * For DOTCLK=5.0MHz (SAIDIVR= 8), use 0x01 to get ~56fps.
         * If you get weird glitches or timing issues, try changing this first.
         */
        self.write_index(0xB6);
            self.write_data(0x0A);
            self.write_data(0xC2);
            self.write_data(0x27);
            self.write_data(0x01);

        /* Interface control */
        self.write_index(0xF6);
            self.write_data(0x01);
            self.write_data(0x00);
            self.write_data(0x06);

        /* Power control */
        self.write_index(0xC0);
            self.write_data(0x10);
            self.write_data(0x10);
        self.write_index(0xc1);
            self.write_data(0x36);

        /* VCOM */
        self.write_index(0xc5);
            self.write_data(0xc3);

        /* Gamma */
        self.write_index(0xE0);
            self.write_data(0x00);
            self.write_data(0x05);
            self.write_data(0x08);
            self.write_data(0x02);
            self.write_data(0x1A);
            self.write_data(0x0C);
            self.write_data(0x42);
            self.write_data(0x7A);
            self.write_data(0x54);
            self.write_data(0x08);
            self.write_data(0x0D);
            self.write_data(0x0C);
            self.write_data(0x23);
            self.write_data(0x25);
            self.write_data(0x0F);
        self.write_index(0xE1);
            self.write_data(0x00);
            self.write_data(0x29);
            self.write_data(0x2F);
            self.write_data(0x03);
            self.write_data(0x0F);
            self.write_data(0x05);
            self.write_data(0x42);
            self.write_data(0x55);
            self.write_data(0x53);
            self.write_data(0x06);
            self.write_data(0x0F);
            self.write_data(0x0C);
            self.write_data(0x38);
            self.write_data(0x3A);
            self.write_data(0x0F);

        /* Sleep out */
        self.write_index(0x11);
        self.delay.delay_ms(120u16);

        /* Display on */
        self.write_index(0x29);

        self.cs.set_high().ok();
    }

    fn write_index(&mut self, idx: u8) {
        self.dcx.set_low().ok();
        self.spi.write(&[idx]).unwrap();
    }

    fn write_data(&mut self, data: u8) {
        self.dcx.set_high().ok();
        self.spi.write(&[data]).unwrap();
    }
}

#[entry]
fn main() -> ! {
    let dp = device::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let mut rcc: Rcc = dp.RCC.constrain();

    // Set up pins
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    let gpiod = dp.GPIOD.split();
    let gpioe = dp.GPIOE.split();
    let gpiog = dp.GPIOG.split();

    // vsync, hsync, dotclk, enable
    gpioa.pa4.into_alternate_af14().set_speed(Speed::VeryHigh);
    gpioc.pc6.into_alternate_af14().set_speed(Speed::VeryHigh);
    gpioe.pe14.into_alternate_af14().set_speed(Speed::VeryHigh);
    gpioe.pe13.into_alternate_af14().set_speed(Speed::VeryHigh);

    // R7-2, G7-2, B7-2
    gpioe.pe15.into_alternate_af14().set_speed(Speed::VeryHigh);
    gpiob.pb1.into_alternate_af14().set_speed(Speed::VeryHigh);
    gpioa.pa12.into_alternate_af14().set_speed(Speed::VeryHigh);
    gpioa.pa11.into_alternate_af14().set_speed(Speed::VeryHigh);
    gpiob.pb0.into_alternate_af14().set_speed(Speed::VeryHigh);
    gpioc.pc10.into_alternate_af14().set_speed(Speed::VeryHigh);
    gpiog.pg8.into_alternate_af14().set_speed(Speed::VeryHigh);
    gpioc.pc7.into_alternate_af14().set_speed(Speed::VeryHigh);
    gpiob.pb11.into_alternate_af14().set_speed(Speed::VeryHigh);
    gpiob.pb10.into_alternate_af14().set_speed(Speed::VeryHigh);
    gpioe.pe11.into_alternate_af14().set_speed(Speed::VeryHigh);
    gpioa.pa6.into_alternate_af14().set_speed(Speed::VeryHigh);
    gpiob.pb9.into_alternate_af14().set_speed(Speed::VeryHigh);
    gpiob.pb8.into_alternate_af14().set_speed(Speed::VeryHigh);
    gpioa.pa3.into_alternate_af14().set_speed(Speed::VeryHigh);
    gpioe.pe12.into_alternate_af14().set_speed(Speed::VeryHigh);
    gpiod.pd10.into_alternate_af14().set_speed(Speed::VeryHigh);
    gpiog.pg10.into_alternate_af14().set_speed(Speed::VeryHigh);

    // RST, DCX, SDA, SCL, CS (SPI4)
    let mut lcd_rst = gpioe.pe5.into_push_pull_output();
    let mut lcd_dcx = gpioe.pe3.into_push_pull_output();
    let lcd_mosi = gpioe.pe6.into_alternate_af5();
    let lcd_sclk = gpioe.pe2.into_alternate_af5();
    let mut lcd_cs = gpioe.pe4.into_push_pull_output();

    // Backlight
    let mut backlight = gpiod.pd12.into_push_pull_output();
    backlight.set_high().ok();

    // Sounder
    let mut sounder_sd = gpiob.pb2.into_push_pull_output();
    sounder_sd.set_low().ok();

    // Onboard green LED
    let mut led = gpiob.pb7.into_push_pull_output();

    // SPI4
    let no_miso = NoMiso{};
    let mut spi = Spi::new(dp.SPI4, (lcd_sclk, no_miso, lcd_mosi)).enable::<u8>(
        &mut rcc,
        spi::ClockDivider::DIV8,
        spi::Mode {
            polarity: spi::Polarity::IdleLow,
            phase: spi::Phase::CaptureOnFirstTransition,
        },
    );

    let clocks = rcc
        .cfgr
        .sysclk(216.mhz())
        .hclk(216.mhz())
        .freeze();
    let mut delay = Delay::new(cp.SYST, clocks);

    let mut ili = ILI9342 {
        spi: &mut spi, rst: &mut lcd_rst, cs: &mut lcd_cs, dcx: &mut lcd_dcx,
        delay: &mut delay,
    };
    ili.init();

    let mut display = LinetimeDisplay::new(dp.LTDC, dp.DMA2D);
    display
        .controller
        .config_layer(Layer::L1, unsafe { &mut FB_LAYER1 }, PixelFormat::ARGB8888);

    display.controller.enable_layer(Layer::L1);
    display.controller.reload();

    let display = &mut display;

    egrectangle!(
        top_left = (40, 80),
        bottom_right = (80, 136),
        style = primitive_style!(fill_color = Rgb888::new(0xFF, 0x00, 0x18))
    ).draw(display).ok();

    egrectangle!(
        top_left = (80, 80),
        bottom_right = (120, 136),
        style = primitive_style!(fill_color = Rgb888::new(0xFF, 0xa5, 0x2C))
    ).draw(display).ok();

    egrectangle!(
        top_left = (120, 80),
        bottom_right = (160, 136),
        style = primitive_style!(fill_color = Rgb888::new(0xFF, 0xFF, 0x41))
    ).draw(display).ok();

    egrectangle!(
        top_left = (160, 80),
        bottom_right = (200, 136),
        style = primitive_style!(fill_color = Rgb888::new(0x00, 0x80, 0x18))
    ).draw(display).ok();

    egrectangle!(
        top_left = (200, 80),
        bottom_right = (240, 136),
        style = primitive_style!(fill_color = Rgb888::new(0x00, 0x00, 0xF9))
    ).draw(display).ok();

    egrectangle!(
        top_left = (240, 80),
        bottom_right = (280, 136),
        style = primitive_style!(fill_color = Rgb888::new(0x86, 0x00, 0x7D))
    ).draw(display).ok();

    egrectangle!(
        top_left = (40, 90),
        bottom_right = (280, 126),
        style = primitive_style!(fill_color = Rgb888::new(0x00, 0x00, 0xFF))
    ).draw(display).ok();

    egtext!(
        text = "Hello Rust!",
        top_left = (100, 100),
        style = text_style!(font = Font12x16, text_color = RgbColor::WHITE)
    ).draw(display).ok();

    let tga = Tga::from_slice(include_bytes!(concat!(
        env!("CARGO_MANIFEST_DIR"), "/rust-pride.tga"))).unwrap();
    let image: Image<Tga, Rgb888> = Image::new(&tga, Point::new(120, 150));
    image.draw(display).ok();

    loop {
        led.set_high().unwrap();
        delay.delay_ms(500u16);
        led.set_low().unwrap();
        delay.delay_ms(500u16);
    }
}
