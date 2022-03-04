//! Barometer with an LCD display
//!
//! This needs an LPS25 pressure sensor and an I2C-enabled HD44780 LCD display hooked up to PB6 and
//! PB7. See the README.md in this repo for how to wire this all up.
//!
//! This code was started from the `blue_pill_base` repo which you can read about in the README as
//! well.
#![no_std]
#![no_main]

// Global logger using "Real Time Terminal" support for the debugger. I'm using an ST-Link V2 clone.
// This goes via Knurlin-rs' defmt.
use defmt_rtt as _;
use panic_probe as _;

// The Blue Pill's HAL crate imports.
use cortex_m_rt::entry;
use stm32f1xx_hal::{i2c, pac, prelude::*};

// Strings, in a no_std env
use core::fmt::Write;
use arrayvec::ArrayString;

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

#[entry]
fn main() -> ! {
    // Get access to the core peripherals from the cortex-m crate
    let cp = cortex_m::Peripherals::take().unwrap();
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut delay = cp.SYST.delay(&clocks);

    // Acquire the GPIOC peripheral
    let mut gpiob = dp.GPIOB.split();
    let mut gpioc = dp.GPIOC.split();

    // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the function
    // in order to configure the port. For pins 0-7, crl should be passed instead.
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    // Set up I2C
    let mut afio = dp.AFIO.constrain();
    let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
    let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);

    let start_timeout_us: u32 = 10000;
    let start_retries: u8 = 5;
    let addr_timeout_us: u32 = 10000;
    let data_timeout_us: u32 = 10000;

    let i2c = i2c::BlockingI2c::i2c1(
        dp.I2C1,
        (scl, sda),
        &mut afio.mapr,
        i2c::Mode::Standard { frequency: 40.Hz() },
        clocks,
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

    // Configure the aht20 temperature and humidity sensor
    let mut aht20_dev =
        aht20_driver::AHT20::new(i2c_bus.acquire_i2c(), aht20_driver::SENSOR_ADDRESS);
    // TODO: error handling, this is your own driver, you can do it!
    let mut aht20 = aht20_dev.init(&mut delay).unwrap();

    // This is just a real-life verifiation that soft_reset does not misbehave.
    // It is not required at all.
    let _ = aht20.soft_reset(&mut delay).unwrap();

    loop {
        // TODO: error handling on measure.
        let aht20_measurement = aht20.measure(&mut delay).unwrap();
        let lps25_pressure = lps25hb.read_pressure().unwrap();
        let lps25_temperature = lps25hb.read_temperature().unwrap();

        lcd.clear(&mut delay).unwrap();

        let mut hpa_buf = ArrayString::<16>::new(); // The display is 16 char wide.
        let mut temp_buf = ArrayString::<16>::new(); // The display is 16 char wide.
       
        if let Err(_) = write!(&mut hpa_buf, "{:.1}hPa", lps25_pressure) {
            defmt::error!("Failed to write to pressure buffer");
        }
        if let Err(_) = write!(&mut temp_buf, "{:.2}C", aht20_measurement.temperature) {
            defmt::error!("Failed to write to temperature buffer");
        }

        lcd.write_str(&hpa_buf, &mut delay).unwrap();
        lcd.set_cursor_pos(40, &mut delay).unwrap(); // Move to 2nd row.
        lcd.write_str(&temp_buf, &mut delay).unwrap();

        defmt::info!("pressure (lps25): {=f32} hPa", lps25_pressure);
        defmt::info!("temperature (lps25): {=f32}C", lps25_temperature);
        defmt::info!("temperature (aht20): {=f32}C", aht20_measurement.temperature);
        defmt::info!("humidity (aht20): {=f32}%", aht20_measurement.humidity);
        defmt::info!("--");

        // Blink the Blue Pill's onboard LED to show liveness.
        delay.delay_ms(1_000_u16);
        led.set_high();
        delay.delay_ms(1_000_u16);
        led.set_low();
    }
}
