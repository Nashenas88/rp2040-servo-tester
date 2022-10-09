#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::mono_font::{MonoFont, MonoTextStyleBuilder};
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::text::{Alignment, Baseline, Text, TextStyle, TextStyleBuilder};
use embedded_hal::adc::OneShot;
use embedded_hal::timer::CountDown;
use fugit::{Duration, RateExtU32};
use micromath::F32Ext;
use panic_probe as _;
use rp2040_hal::clocks::{init_clocks_and_plls, Clock, SystemClock};
use rp2040_hal::dma::{Channel, ChannelIndex, DMAExt, CH0, CH1};
use rp2040_hal::gpio::{DynPin, Error as GpioError, FunctionConfig, FunctionPio0};
use rp2040_hal::pac::{interrupt, PIO0};
use rp2040_hal::pio::{PIOExt, StateMachineIndex, UninitStateMachine, PIO, SM0};
use rp2040_hal::sio::Sio;
use rp2040_hal::watchdog::Watchdog;
use rp2040_hal::{entry, pac};
use servo_pio::calibration::{map_float, AngularCalibration, Calibration};
use servo_pio::pwm_cluster::{dma_interrupt, GlobalState, GlobalStates, Handler};
use servo_pio::servo_cluster::{ServoCluster, ServoClusterBuilderError};
use ssd1306::mode::DisplayConfig;
use ssd1306::rotation::DisplayRotation;
use ssd1306::size::{DisplaySize, DisplaySize128x32};
use ssd1306::Ssd1306;

#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

// We only want to test 1 servo.
const NUM_SERVOS: usize = 1;
const NUM_CHANNELS: usize = 1;
// Number of readings to average over to stabilize.
const SAMPLES: u8 = 50;

// Aliases to easily manage swapping out screens.
type ScreenSize = DisplaySize128x32;
const SCREEN_SIZE: ScreenSize = DisplaySize128x32;

// Delay between each servo update.
type Dur = Duration<u32, 1, 1_000_000>;
const MOVEMENT_DELAY: Dur = Dur::millis(10);

// Text rendering properties.
const FONT: MonoFont = FONT_10X20;
// We want the text centered on the screen.
const TEXT_STYLE: TextStyle = TextStyleBuilder::new()
    .alignment(Alignment::Center)
    .baseline(Baseline::Middle)
    .build();
const POSITION: Point = Point {
    x: <ScreenSize as DisplaySize>::WIDTH as i32 / 2,
    y: <ScreenSize as DisplaySize>::HEIGHT as i32 / 2,
};

static mut STATE1: Option<GlobalState<CH0, CH1, PIO0, SM0>> = {
    const NONE_HACK: Option<GlobalState<CH0, CH1, PIO0, SM0>> = None;
    NONE_HACK
};
static mut GLOBALS: GlobalStates<NUM_CHANNELS> = {
    const NONE_HACK: Option<&'static mut dyn Handler> = None;
    GlobalStates {
        states: [NONE_HACK; NUM_CHANNELS],
    }
};

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let _core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // Configure the Timer peripheral in count-down mode.
    let timer = rp2040_hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut count_down = timer.count_down();

    let pins = rp2040_hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Update pins depending on your board/setup.
    let sda_pin = pins.gpio2;
    let scl_pin = pins.gpio3;
    let servo_pin = pins.gpio22;
    let mut potentiometer_pin = pins.gpio26.into_floating_input();

    // Setup i2c for display.
    let i2c1 = rp2040_hal::I2C::i2c1(
        pac.I2C1,
        sda_pin.into_mode(),
        scl_pin.into_mode(),
        400.kHz(),
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
    );

    // Create the I2C display interface:
    let interface = ssd1306::I2CDisplayInterface::new(i2c1);
    let mut display = Ssd1306::new(interface, SCREEN_SIZE, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let character_style = MonoTextStyleBuilder::new()
        .font(&FONT)
        .text_color(BinaryColor::On)
        .build();

    // Setup adc for potentiometer.
    let mut adc = rp2040_hal::Adc::new(pac.ADC, &mut pac.RESETS);

    // Setup servo cluster.
    let (mut pio0, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let dma = pac.DMA.split(&mut pac.RESETS);

    let mut servo_cluster = match build_servo_cluster(
        &mut pio0,
        sm0,
        (dma.ch0, dma.ch1),
        [servo_pin.into_mode::<FunctionPio0>().into()],
        clocks.system_clock,
        unsafe { &mut STATE1 },
    ) {
        Ok(cluster) => cluster,
        Err(e) => {
            defmt::error!("Failed to build servo cluster: {:?}", e);
            #[allow(clippy::empty_loop)]
            loop {}
        }
    };

    // Get the servo index.
    let [servo] = servo_cluster.servos();

    // Unmask the DMA interrupt so the handler can start running. This can only
    // be done after the servo cluster has been built.
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::DMA_IRQ_0);
    }

    // Buffer for writing formatting data into. Used to display pulse readings.
    let mut buf = [0u8; 12];

    let mut pulse;

    loop {
        // Start the count down so we can try to maintain a stable loop.
        count_down.start(MOVEMENT_DELAY);

        // Read the potentiometer value, averaging a few samples to get rid of noise.
        pulse = 0.0;
        for _ in 0..SAMPLES {
            let reading = adc.read(&mut potentiometer_pin).unwrap();
            pulse += map_float(reading, 0.0, 4096.0, 500.0, 2500.0);
        }
        pulse /= SAMPLES as f32;

        // Set the pulse on the servo cluster.
        servo_cluster.set_pulse(servo, pulse, true);

        let pulse = pulse.round() as u16;

        // Write the servo value to the display.
        display.clear();
        let s: &str = write_to::show(&mut buf, format_args!("Pulse: {:04}", pulse)).unwrap();
        let text = Text::with_text_style(s, POSITION, character_style, TEXT_STYLE);
        text.draw(&mut display).unwrap();
        display.flush().unwrap();

        // Wait for the count_down to finish.
        let _ = nb::block!(count_down.wait());
    }
}

#[derive(Format)]
enum BuildError {
    Gpio(GpioError),
    Build(ServoClusterBuilderError),
}

fn build_servo_cluster<C1, C2, P, SM>(
    pio: &mut PIO<P>,
    sm: UninitStateMachine<(P, SM)>,
    dma_channels: (Channel<C1>, Channel<C2>),
    servo_pins: [DynPin; NUM_SERVOS],
    #[cfg(feature = "debug_pio")] side_set_pin: DynPin,
    system_clock: SystemClock,
    state: &'static mut Option<GlobalState<C1, C2, P, SM>>,
) -> Result<ServoCluster<1, P, SM, AngularCalibration>, BuildError>
where
    C1: ChannelIndex,
    C2: ChannelIndex,
    P: PIOExt + FunctionConfig,
    SM: StateMachineIndex,
{
    let mut calibration = Calibration::new().unwrap();
    // Turn off limits so we can manually test for the limits.
    calibration.limit_upper = false;
    calibration.limit_lower = false;

    ServoCluster::builder(pio, sm, dma_channels, unsafe { &mut GLOBALS })
        .pins(servo_pins)
        .map_err(BuildError::Gpio)?
        .calibrations([Some(calibration)])
        .pwm_frequency(50.0)
        .build(&system_clock, state)
        .map_err(BuildError::Build)
}

#[interrupt]
fn DMA_IRQ_0() {
    critical_section::with(|_| {
        // Safety: we're within a critical section, so nothing else will modify global_state.
        dma_interrupt(unsafe { &mut GLOBALS });
    });
}

pub mod write_to {
    use core::cmp::min;
    use core::fmt;

    pub struct WriteTo<'a> {
        buffer: &'a mut [u8],
        // on write error (i.e. not enough space in buffer) this grows beyond
        // `buffer.len()`.
        used: usize,
    }

    impl<'a> WriteTo<'a> {
        pub fn new(buffer: &'a mut [u8]) -> Self {
            WriteTo { buffer, used: 0 }
        }

        pub fn as_str(self) -> Option<&'a str> {
            if self.used <= self.buffer.len() {
                // only successful concats of str - must be a valid str.
                use core::str::from_utf8_unchecked;
                Some(unsafe { from_utf8_unchecked(&self.buffer[..self.used]) })
            } else {
                None
            }
        }
    }

    impl<'a> fmt::Write for WriteTo<'a> {
        fn write_str(&mut self, s: &str) -> fmt::Result {
            if self.used > self.buffer.len() {
                return Err(fmt::Error);
            }
            let remaining_buf = &mut self.buffer[self.used..];
            let raw_s = s.as_bytes();
            let write_num = min(raw_s.len(), remaining_buf.len());
            remaining_buf[..write_num].copy_from_slice(&raw_s[..write_num]);
            self.used += raw_s.len();
            if write_num < raw_s.len() {
                Err(fmt::Error)
            } else {
                Ok(())
            }
        }
    }

    pub fn show<'a>(buffer: &'a mut [u8], args: fmt::Arguments) -> Result<&'a str, fmt::Error> {
        let mut w = WriteTo::new(buffer);
        fmt::write(&mut w, args)?;
        w.as_str().ok_or(fmt::Error)
    }
}
