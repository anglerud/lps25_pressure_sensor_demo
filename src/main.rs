//! Blinks an LED
//!
//! This assumes that a LED is connected to pc13 as is the case on the blue pill board.
//!
//! Note: Without additional hardware, PC13 should not be used to drive an LED, see page 5.1.2 of
//! the reference manual for an explanation. This is not an issue on the blue pill.

#![deny(unsafe_code)]
#![no_std]
#![cfg_attr(not(doc), no_main)]

use rtt_target::{rprintln, rtt_init_print};
use panic_rtt_target as _;

use nb::block;

use stm32f1xx_hal::{
    delay::Delay, // Maybe not needed
    i2c,
    prelude::*,
    pac,
    timer::Timer,
};
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;

// Pressure sensor - LPS25
use lps25hb::*;
use lps25hb::interface::{I2cInterface,
    i2c::I2cAddress};
use lps25hb::sensor::*;
use lps25hb::register::*;


#[entry]
fn main() -> ! {
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

    // Acquire the GPIOC peripheral
    // FIXME: Probably only some of these are neded.
    // let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);
    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);

    // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the function
    // in order to configure the port. For pins 0-7, crl should be passed instead.
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    // Configure the syst timer to trigger an update every second
    let mut timer = Timer::syst(cp.SYST, &clocks).start_count_down(1.hz());

    // Set up i2c
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


    // configure I2C interface for the LPS25HB driver.
    let i2c_interface = I2cInterface::init(i2c, I2cAddress::SA0_VCC);
    // create a new LPS25 instance with the I2C interface
    let mut lps25hb = LPS25HB::new(i2c_interface);

    // turn the sensor on
    lps25hb.sensor_on(true).unwrap();

    // enable Block Data Update
    lps25hb.bdu_enable(true).unwrap();

    // set data rate to 7Hz
    lps25hb.set_datarate(ODR::_1Hz).unwrap();



    rprintln!("Hello, Rust!");
    // Wait for the timer to trigger an update and change the state of the LED
    let mut i = 0;
    loop {
        block!(timer.wait()).unwrap();
        led.set_high().unwrap();
        block!(timer.wait()).unwrap();
        led.set_low().unwrap();

        // read temperature and pressure
        let temp = lps25hb.read_temperature().unwrap();
        let press = lps25hb.read_pressure().unwrap();

        let id = lps25hb.get_device_id().unwrap();

        rprintln!("temperature: {:.1} C, pressure: {:.1} hPa", temp, press);
        rprintln!("my name is: {}", id);


        i += 1;
        rprintln!("Hello again; I have blinked {} times.", i);
        if i == 10 {
            panic!("Yow, 10 times is enough!");
        }
    }
}
