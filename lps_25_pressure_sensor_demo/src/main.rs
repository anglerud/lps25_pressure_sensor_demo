//! Barometer with an LCD display
//!
//! This needs an LPS25 pressure sensor and an I2C-enabled HD44780 LCD display hooked up to PB6 and
//! PB7. See the README.md in this repo for how to wire this all up.
//!
//! This code was started from the `blue_pill_base` repo which you can read about in the README as
//! well.

#![no_std]
#![cfg_attr(not(doc), no_main)]
#![feature(alloc_error_handler)]

// This gives us `alloc` support, so we can use the format macro.
use core::alloc::Layout;
extern crate alloc;
use alloc_cortex_m::CortexMHeap;

// This is the "Real Time Terminal" support for the debugger. I'm using an ST-Link V2 clone.
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};

// The Blue Pill's HAL crate imports.
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use stm32f1xx_hal::{delay, i2c, pac, prelude::*};

// Shared-bus lets us use multiple I2C devices on the same I2C bus.
use shared_bus;

// 16x2 LED screen.
use hd44780_driver::{Cursor, CursorBlink, Display, DisplayMode, HD44780};

// Pressure sensor - LPS25 - breakout board by Adafruit
use lps25hb::interface::{i2c::I2cAddress, I2cInterface};
use lps25hb::*;

const LCD_I2C_ADDRESS: u8 = 0x27;

// Temperature (and humidity) sensor.
use aht20_driver;
// use aht20;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

#[entry]
fn main() -> ! {
    // We need to initialize the allocator BEFORE using it - so we do that first.
    let start = cortex_m_rt::heap_start() as usize;
    let size = 256; // in bytes
    unsafe { ALLOCATOR.init(start, size) }

    // Init buffers for debug printing
    rtt_init_print!();
    // Get access to the core peripherals from the cortex-m crate
    let cp = cortex_m::Peripherals::take().unwrap();
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut delay = delay::Delay::new(cp.SYST, clocks);

    // Acquire the GPIOC peripheral
    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);

    // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the function
    // in order to configure the port. For pins 0-7, crl should be passed instead.
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    // Set up I2C
    let afio = dp.AFIO.constrain(&mut rcc.apb2);
    let mut mapr = afio.mapr;
    let mut apb = rcc.apb1;
    let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
    let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);

    let mode = i2c::Mode::Standard { frequency: 40.hz() };
    let start_timeout_us: u32 = 10000;
    let start_retries: u8 = 5;
    let addr_timeout_us: u32 = 10000;
    let data_timeout_us: u32 = 10000;

    let i2c = i2c::BlockingI2c::i2c1(
        dp.I2C1,
        (scl, sda),
        &mut mapr,
        mode,
        clocks,
        &mut apb,
        start_timeout_us,
        start_retries,
        addr_timeout_us,
        data_timeout_us,
    );
    // This shared bus lets us pass the I2C bus to multiple devices.
    let i2c_bus = shared_bus::BusManagerSimple::new(i2c);

    // Initialize the LCD panel.
    let mut lcd = HD44780::new_i2c(i2c_bus.acquire_i2c(), LCD_I2C_ADDRESS, &mut delay).unwrap();
    lcd.reset(&mut delay).unwrap();
    lcd.clear(&mut delay).unwrap();
    lcd.set_display_mode(
        DisplayMode {
            display: Display::On,
            cursor_visibility: Cursor::Visible,
            cursor_blink: CursorBlink::On,
        },
        &mut delay,
    )
    .unwrap();

    // configure I2C interface for the LPS25HB driver.
    let i2c_interface = I2cInterface::init(i2c_bus.acquire_i2c(), I2cAddress::SA0_VCC);
    // create a new LPS25 instance with the I2C interface
    let mut lps25hb = LPS25HB::new(i2c_interface);

    lps25hb.sensor_on(true).unwrap();
    // enable Block Data Update
    lps25hb.bdu_enable(true).unwrap();
    lps25hb.set_datarate(ODR::_1Hz).unwrap();

    // Configure the ahl20 temperature and humidity sensor
    // FIXME: OK, we need to write our own because:
    //   * we can't share the delay - we need to take a reference to delay instead
    //   * the driver is agpl, but it's a library... not a good match.
    //   * it's blocking - see if we can't make a non-blocking one, see PR to driver.
    //   * it'll be fun! You've only done this guided before.
    // let mut aht20_dev = aht20_driver::AHT20::new(i2c_bus.acquire_i2c(), aht20_driver::SENSOR_ADDRESS, &mut delay);
    // aht20_dev.init().unwrap();
    // let status = aht20_dev.check_status().unwrap();
    // rprintln!("aht20 status: {:?}, ready: {}, calibrated: {}", status, status.is_ready(), status.is_calibrated());
    // let measurement = aht20_dev.measure().unwrap();
    // rprintln!("measurement: {:?}", measurement);

    loop {
        // Read temperature and pressure.  We can't rely on the lps25 temperature sensor for actual
        // temperature readings.  It shows 19.2C when the CO2 sensor says 25.9, and the DK internal
        // sensor says 27.25. ST's website only advertises it as a pressure sensor and doesn't
        // mention the thermometer at all. I think it's just there to switch between different
        // profiles based on very coarse-grained temperature bands.
        // We print out the read temperature, but don't display it on the LCD panel.
        //
        // Here, we can get a measurement if we create, then destroy the sensor! That works because
        // then we release the delay device. This really sucks. We *could* take the delay as a
        // function parameter, rather than embed it in the aht20? That way we'd not hold on to
        // it. Is that why the lcd takes it as a param? Might well be!
        // Commit this way, then try to move the delay into a function param.
        let mut aht20_dev = aht20_driver::AHT20::new(
            i2c_bus.acquire_i2c(),
            aht20_driver::SENSOR_ADDRESS,
            &mut delay,
        );
        aht20_dev.init().unwrap();
        let aht20_measurement = aht20_dev.measure().unwrap();
        rprintln!("aht20_measurement: {:?}", aht20_measurement);
        aht20_dev.destroy();

        let press = lps25hb.read_pressure().unwrap();
        let lps25_temp = lps25hb.read_temperature().unwrap();

        lcd.clear(&mut delay).unwrap();
        let hpa_str = alloc::format!("{:.1} hPa", press);
        let temp_str = alloc::format!("{:.2}C", aht20_measurement.temperature);

        lcd.write_str(&hpa_str, &mut delay).unwrap();
        lcd.set_cursor_pos(40, &mut delay).unwrap(); // Move to 2nd row.
        lcd.write_str(&temp_str, &mut delay).unwrap();

        rprintln!("pressure (lps25): {:.1} hPa", press);
        rprintln!("temperature (lps25): {:.2}C", lps25_temp);
        rprintln!("temperature (aht20): {:.2}C", aht20_measurement.temperature);

        // Blink the Blue Pill's onboard LED to show liveness.
        delay.delay_ms(1_000_u16);
        led.set_high().unwrap();
        delay.delay_ms(1_000_u16);
        led.set_low().unwrap();
    }
}

#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    loop {}
}
