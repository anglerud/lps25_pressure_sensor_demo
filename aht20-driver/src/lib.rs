#![cfg_attr(not(test), no_std)]
//! SCD30 driver and
//! https://github.com/adafruit/Adafruit_CircuitPython_AHTx0/blob/main/adafruit_ahtx0.py inspration
//! AHT20 Datasheet: https://cdn-learn.adafruit.com/assets/assets/000/091/676/original/AHT20-datasheet-2020-4-16.pdf?1591047915
//!
//! This is how I believe how we get a measurement. This does not include the Command:Calibrate or
//! Command::SoftReset. For Calibrate, I image you go to Command::Init in this chart. For SoftReset
//! go to Start.
//! XXX: TO figure out - after TriggerMeasurement - do we just read a byte
//!      which then comes back with a status - and continue to wait... or do we
//!      issue CheckStatus? The docs seem to indicate that we just keep reading
//!      bytes until Busy goes to no and then read another 6 bytes?
//! NOTE: That Initialize takes 2 bytes parameters: 0x08 0x00,
//!       and TriggerMeasurement takes 2 bytes parameters: 0x33 0x00
//! NOTE: For blog - need to run with --release.
//!       OR I guess we could set up a debug profile?
//! NOTE: For blog - some of the datasheet is like poetry - the "Initialization" command has a
//!       definition "Keep main engine".
//!
//! A successful TriggerMeasurement command lets you read 6 bytes of data back.
//! 2 bytes humidity data, 1 byte humidity temperature, 2 bytes temperature data then 1 byte of
//! CRC.
//!
//! To calculate Relative Humidity: (humidity data / 2**20) * 100
//! To calculate Temp in C: (temp data / 2**20) * 200 - 50
//!
//! Note that there is no mention of what "Humidity temperature" is used for.
//!
//! ```text
//!           Start (Power on)
//!                  │
//!                  ▼
//!              Wait 40 ms
//!                  │
//!                  ▼
//!   Command::CheckStatus (0x71)   ◄───    Wait 10 ms
//!                  │                           ▲
//!                  ▼                           │
//!          Status::Calibrated ──► No ──► Command::Initialize (0xBE)
//!                  │
//!                  ▼
//!                 Yes
//!                  │
//!                  ▼
//! Command::TriggerMeasurement (0xAC)   ◄─┐
//!                  │                     │
//!                  ▼                     │
//!             Wait 80 ms                 │
//!                  │                     │
//!                  ▼                     │
//!   Command::CheckStatus (0x71) ◄──┐     │
//!                  │               │     │
//!                  ▼               │     │
//!             Status::Busy  ───►  Yes    │
//!                  │                     │
//!                  ▼                     │
//!                 No                     │
//!                  │                     │
//!                  ▼                     │
//!             Read 6 bytes               │
//!                  │                     │
//!                  ▼                     │
//!             Calculate CRC              │
//!                  │                     │
//!                  ▼                     │
//!               CRC good ─► No  ─────────┘
//!                  │
//!                  ▼
//!                 Yes
//!                  │
//!                  ▼
//!        Calc Humidity and Temp
//! ```

// TODO:
// * check initialized in more places and return an error
//   if not. New variant in error enum for that (also test it).
//   NO - split driver struct into two and make it impossible
//   to get wrong.
// * Docstrings
// * Transfer notes out into blog article
// * split into independent crate and push to staging repo
// * push 0.0.1 to real repo
// * use in the lps25_demo... app.
// * push 1.0.0 to real repo
// * write blog
// * submit driver and blog to embedded awesome
// * submit driver and blog to /r/rust
// * submit driver and blog to rust discourse
// NOTE for blog: praise embedded-hal-mock, and especially the
//      i2c mocking - really easy, clear error messages.
// NOTE: we could make init return a new struct which has the
//       measure methods on, and *remove* the measure methods
//       from the AHT20 struct. That way you *couldn't* call
//       the wrong methods at all. Makes tracking initialized
//       much easier! I like that a lot.
use crc_any::CRCu8;
use embedded_hal::blocking::delay::{DelayMs, DelayUs};
use embedded_hal::blocking::i2c;

// TODO: delete
// use rtt_target::rprintln;

/// AHT20 I2C address
pub const SENSOR_ADDRESS: u8 = 0b0011_1000; // This is I2C address 0x38;

///  
pub enum Command {
    CheckStatus = 0b0111_0001, // 0x71, Get a byte of status word. You can use this
    //       on startup to check if you need to send the
    //       Initialize command. Check Status::Calibrated.
    //       If the calibration status bit (4th bit) is not
    //       1, you need to send Command::Initialize.
    Initialize = 0b1011_1110, // 0xBE, Section 5.3, page 8, Table 9
    //       This command takes two bytes of parameter.
    //       0b0000_1000 (0x08), then 0b0000_0000 (0x00).
    Calibrate = 0b1110_0001, // 0xE1, Calibrate - or return calibration status.
    //       Status will be Status::Calibrated, where
    //       bit4 indicates calibrated. If it's 0, it's not.
    TriggerMeasurement = 0b1010_1100, // 0xAC, Section 5.3, page 8, Table 9
    //       This command takes two bytes of parameter.
    //       0b00110011 (0x33), then 0b0000_0000 (0x00).
    //       Wait 80ms for the measurement. You'll get a
    //       status byte back. Check the status
    //       for Status::Busy to be 0. If it is, then read
    //       6 bytes (5 data plus a byte of CRC).
    //       QUESTION: At this point we can *send* a byte
    //       of CRC to get it verified? Secion 5.4.4 ?
    SoftReset = 0b1011_1010, // 0xBA, Section 5.3, page 8, Table 9.
                             //       Also see Section 5.5. This takes 20ms or less
                             //       to complete.
}

// TODO: Use https://github.com/unrelentingtech/ldc1x1x/blob/trunk/src/data.rs#L63 pattern to
// represent the status! That looks really nice. So make the struct a tuple with one 8-bit word.
// Then functions to get the status! data_ready() -> bool, and calibrated() -> bool for example.
// const STATUS_BUSY: u8 0x80  // Status bit for busy
pub enum Status {
    Busy = 0b1000_0000, // Status bit for busy - 8th bit enabled. 1<<7, 0x80
    //   1 is Busy measuring. 0 is "Free in dormant state"
    //   Table 10, page 8 of the datasheet.
    Calibrated = 0b0000_1000, // Status bit for calibrated - 4th bit enabled. 1<<4, 0x08.
                              //   1 is Calibrated, 0 is uncalibrated. If 0, send
                              //   Command::Initialize (0xBE).
                              //   Table 10, page 8 of the datasheet.
}

// TODO: create a struct that represents the measurement,
// QUESTION: Make  AHL20 take a Delay? A reference to a delay possibly.
// NOTE: for blog - learning about the nb crate - a bit surprising. It's on the
//       front page of the hal docs, but I'd not really seen it in examples or
//       crates I'd read before.
//       Example doesn't really show how to use it in a driver - how would I
//       wait? The spec for this says to wait Nms - but how do I enforce that
//       in my driver? I don't want to just make it a comment.
//       Er, like... do I keep state in the struct, then return wouldblock or
//       something? How do I time?
//       - The docs are very brief, and I think aimed at someone who is already
//         an experienced embedded dev, or already knows the ecosystem.
//       - For example "Application specific errors can be put inside the Other variant in the
//       nb::Error enum." - what? I don't understand this at all.
//       OK, so there isn't much code in there at all, but that still leaves me
//       a bit mystified about how it's intended to be used.
// NOTE: OK, so yes I think in order to use nb I'd have to store an 'is_reading' and
//       `is_waiting` bool for a state machine, and a timestamp from a timer or delay or
//       something.
//       Then the caller would have to call nb::block!() and busy-spin or whatever.
// For blog: point out that I don't understand how this would work if not blocking
// really, and how that'd turn into async ever. I've probably not understood.
// I guess we could use timer::CountDown
// OK, and finally - we're using blocking::i2c - so we don't need the non-blocking
// mode.
// In fact, I don't even see an implementation for i2c in stm32f1xx_hal that isn't
// i2c.blocking. So, is it even possible? All in all, being pointed to the `nb` crate
// right away took me on a very confusing journey - the end result which is... I think,
// that at least with the hal implementation I'm going to use this with, there isn't
// support to do i2c non-blocking anyway, so I don't need to try.
// TODO: Create a new crate for this! Then refer to that crate as a git or path link
//       in the demo's crate manifest.
//
// TODO: Use timer::CountDown, and the timer::CountDownTimer<SYST> (reference) (concrete)
//       for the pausing - and then block!(cdt.wait())
//       Must definitely be a reference so that we don't go owning the timer or syst.
//       BUT, that seems like an easy way to do this?
// NOTE: for blog - also mention that the distinction between a Delay and a CountDown
//       is kinda fuzzy.
//       https://docs.rs/embedded-hal/latest/embedded_hal/timer/trait.CountDown.html
//       No, actually it's clear:
//       https://docs.rs/embedded-hal/latest/embedded_hal/blocking/delay/index.html
//       delay is *only* blocking. As we're going to be only blocking, I guess we should
//       go for this one. After all, only blocking i2c is available.
// NOTE: for blog - OK, so after I've gone off and read about nb and become confused,
//       there's actually some useful stuff on the embedded-hal front page about using
//       nb in async - including a little loop that polls forever!
//       The examples use the try_nb! macro, and I can't see that in the code or docs?
//       Where does that come from?
// INSIGHT: it's really valuable to know that the hal stuff is *very* split between
//          blocking and non-blocking. That makes it a lot easier to understand - and should
//          probably be pointed out earlier. Also that the non-blocking story appears to
//          be very confused - and also incomplete. For example, there's no implementation
//          of anything but blocking for the hal I'm using.

// TODO: docstring
#[derive(Debug, Clone, Copy)]
pub struct SensorStatus(pub u8);

impl SensorStatus {
    // Not sure this is needed.
    // could also be called from_status_byte?
    pub fn new(status: u8) -> Self {
        SensorStatus(status)
    }

    /// Check if the sensor is ready to have data read from it.
    /// After issuing a sensor read, you must check is_ready before reading the
    /// result. The read() function takes care of this wait and check.
    pub fn is_ready(self) -> bool {
        // The busy bit should be 0 (not busy) for the sensor to report ready.
        (self.0 & Status::Busy as u8) == 0
    }

    pub fn is_calibrated(self) -> bool {
        // The calibrated bit should be set.
        (self.0 & Status::Calibrated as u8) != 0
    }
}

// TODO: docstring
#[derive(Debug, Clone, Copy)]
pub struct SensorReading {
    pub humidity: f32,
    pub temperature: f32,
}

impl SensorReading {
    pub fn from_bytes(sensor_data: [u8; 5]) -> Self {
        // NOTE: for the blog article. TOTALLY not clear how to get two values out of 5 bytes. The
        // middle byte was noted as "Humidity temperature", which I assumed was a third value of
        // some kind and ended up googling about what that could mean with no results. Nowhere
        // in the docs was a "humidity temperature" mentioned, but it sort of made sense to me
        // that it could be a concept as humidity and temperature have a relationship.
        // BUT - in actuality it's a byte that gets *split*. It took a lot of poking aimlessly at
        // it and getting nonsense results until section 6 "Signal Transformation" gave a clue -
        // which is the dividing by 2^20 - as in "20 bits" two bytes (16 bits) plus a half byte (a
        // nibble)! This is absolutely not clear from the datasheet, and it feels like solving a
        // cryptic puzzle rather than reading a data sheet!
        // Being a complete beginner at this - it also took some trial and error to generate
        // a valid f32 from these at all! Masking out the top four bits in the split byte was
        // somethig which took me some time to realize I had to do. All makes sense in retrospect,
        // but definitely not immediately.
        // I also spent a bunch of time messing around with f32::from_be_bytes and from_bits
        // after googling around, but turns out that wasn't needed.
        // I spent a lot of time in python just poking at this. At first I had:
        // let humidity_bytes: &[u8] = &sensor_data[..2];
        // let humidity_temperature_byte: u8 = sensor_data[2];
        // let temperature_bytes: &[u8] = &sensor_data[3..5];
        // clearly wrong - especially that "humidity_temperature_byte"

        // Our five bytes of sensor data is split into 20 bits (two and a half bytes) humidity and
        // 20 bits temperature. We'll have to spend a bit of time splitting the middle byte up.
        let humidity_bytes: &[u8] = &sensor_data[..2];
        let split_byte: u8 = sensor_data[2];
        let temperature_bytes: &[u8] = &sensor_data[3..5];

        // We have a byte that might look like 0x0101_1010, we want only the first four bits, (the
        // 0101) to be at the end of the byte. So we shift them four right and end up with
        // 0x0000_0101. These 4 bits go at the very end of our 20-bit humidity value.
        // In the final 32-bit value they're these ones: 0x0000_0000_0000_0000_0000_0000_0000_1111
        let right_bits_humidity: u32 = (split_byte >> 4).into();
        // In the final 32-bit value they're these ones: 0x0000_0000_0000_1111_1111_0000_0000_0000
        let left_bits_humidity: u32 = (humidity_bytes[0] as u32) << 12;
        // In the final 32-bit value they're these ones: 0x0000_0000_0000_0000_0000_1111_1111_0000
        let middle_bits_humidity: u32 = (humidity_bytes[0] as u32) << 4;
        // We combine them to form the complete 20 bits: 0x0000_0000_0000_1111_1111_1111_1111_1111
        let humidity_val: u32 = left_bits_humidity | middle_bits_humidity | right_bits_humidity;

        // From section 6.1 "Relative humidity transformation" here is how we turn this into
        // a relative humidity percantage value.
        let humidity_percent = (humidity_val as f32) / ((1 << 20) as f32) * 100.0;

        // With that same example byte - we want to keep only the last four bits this time, so we
        // mask the first four and end up with 0x0000_1010. These bits end up at the very start of
        // our 20-bit temperature value. In the final 32-bit value they're these ones:
        // 0x0000_0000_0000_1111_0000_0000_0000_0000 To get them into their final position - we'll
        // left-shift them by 16 positions.
        let split_byte_temperature: u32 = (split_byte & 0b0000_1111).into();
        // We need to fill the rightmost 20 bits, starting with our split byte
        // In the final 32-bit value they're these ones: 0x0000_0000_0000_1111_0000_0000_0000_0000
        let left_bits_temp: u32 = (split_byte_temperature << 16).into();
        // In the final 32-bit value they're these ones: 0x0000_0000_0000_0000_1111_1111_0000_0000
        let middle_bits_temp: u32 = (temperature_bytes[0] as u32) << 8;
        // And just for symmetry...
        // In the final 32-bit value they're these ones: 0x0000_0000_0000_0000_0000_0000_1111_1111
        let right_bits_temp: u32 = temperature_bytes[1] as u32;
        // We combine them to form the complete 20 bits: 0x0000_0000_0000_1111_1111_1111_1111_1111
        let temperature_val: u32 = left_bits_temp | middle_bits_temp | right_bits_temp;

        // From section 6.2 "Temperature transformation" here is how we turn this into
        // a temprature in °C.
        let temperature_celcius = (temperature_val as f32) / ((1 << 20) as f32) * 200.0 - 50.0;

        SensorReading {
            humidity: humidity_percent,
            temperature: temperature_celcius,
        }
    }
}

/// A driver error
#[derive(Debug, PartialEq)]
pub enum Error<E> {
    /// I2C bus error
    I2c(E),
    /// CRC validation failed
    InvalidCrc,
}

// NOTE: for blog. This was my initial definition, but this doesn't
//   work because you can't use the delay functions in more than
//   one place even if you send in a borrow. The delay's functions
//   take &mut self, thus we need to make this a mutable reference.
//   It's a bit of a shame, this is my first use of an explicit
//   lifetime, and the functions all taking (&mut delay) looks
//   pretty ugly! The LCD driver I use took this approach, and has
//   clearly noticed the same.
// /// An AHT20 sensor on the I2C bus `I`, with a Delay device `D`.
// pub struct AHT20<'a, I, D>
// where
//     I: i2c::Read + i2c::Write,
//     D: DelayUs<u16> + DelayMs<u16>,
// {
//     i2c: I,
//     address: u8,
//     delay: &'a mut D,
//     initialized: bool,
// }

// TODO: Create AHT20Initialized, have it
//       take the AHT20 struct? That reduces dupliateion.
//       But we might have to make some things public?
//       *or* do we? Can one return a private struct?
//       Find out and try it! OTOH having more than one
//       public stuct doesn't seem like a problem.
//       I think the above approach if we can.
//       The goal is to make an API that one can't get wrong,
//       such as measuring before init. It'll make error handling
//       easier as well.
//
//       OR shall we destroy
//       the AHT20 and create a *new* struct with the
//       newly liberated i2c instance?
//       That might have to duplicate check_status?

/// An AHT20 sensor on the I2C bus `I`.
pub struct AHT20<I>
where
    I: i2c::Read + i2c::Write,
{
    i2c: I,
    address: u8,
    initialized: bool,
}

impl<E, I> AHT20<I>
where
    I: i2c::Read<Error = E> + i2c::Write<Error = E>,
{
    /// Initializes the SCD30 driver.
    /// This consumes the I2C bus `I`
    pub fn new(i2c: I, address: u8) -> Self {
        AHT20 {
            i2c: i2c,
            address: address,
            initialized: false,
        }
    }

    /// Run the AHT20 init and calibration routines.
    ///
    /// This must be called before any other methods except `check_status`. This method will take
    /// *at least* 40ms to return.
    ///
    /// ```text
    ///          Start (Power on)
    ///                 │
    ///                 ▼
    ///             Wait 40 ms
    ///                 │
    ///                 ▼
    ///  Command::CheckStatus (0x71)   ◄───    Wait 10 ms
    ///                 │                           ▲
    ///                 ▼                           │
    ///         Status::Calibrated ──► No ──► Command::Initialize (0xBE)
    ///                 │
    ///                 ▼
    ///                Yes
    /// ```
    pub fn init(&mut self, delay: &mut (impl DelayUs<u16> + DelayMs<u16>)) -> Result<(), Error<E>> {
        delay.delay_ms(40_u16);

        while !self.check_status()?.is_calibrated() {
            self.send_initialize()?;
            delay.delay_ms(10_u16);
        }

        self.initialized = true;

        Ok(())
    }

    // Send CheckStatus, read one byte back.
    // TODO: better docstring
    pub fn check_status(&mut self) -> Result<SensorStatus, Error<E>> {
        let command: [u8; 1] = [Command::CheckStatus as u8];
        let mut read_buffer = [0u8; 1];

        self.i2c.write(self.address, &command).map_err(Error::I2c)?;
        self.i2c
            .read(self.address, &mut read_buffer)
            .map_err(Error::I2c)?;

        let status_byte = read_buffer[0];
        Ok(SensorStatus::new(status_byte))
    }

    // Initialize = 0b1011_1110, // 0xBE, Section 5.3, page 8, Table 9
    //       This command takes two bytes of parameter.
    //       0b0000_1000 (0x08), then 0b0000_0000 (0x00).
    // TODO: better docstring
    pub fn send_initialize(&mut self) -> Result<(), Error<E>> {
        // Send CheckStatus, read one byte back.
        let command: [u8; 3] = [
            Command::Initialize as u8,
            // Two parameters as described in the datasheet. There is no indication what these
            // parameters mean, just that they should be provided. There is also no returned
            // value.
            0b0000_1000, // 0x08
            0b0000_0000, // 0x00
        ];

        self.i2c.write(self.address, &command).map_err(Error::I2c)?;

        Ok(())
    }

    // TriggerMeasurement = 0b1010_1100, // 0xAC, Section 5.3, page 8, Table 9
    //       This command takes two bytes of parameter.
    //       0b00110011 (0x33), then 0b0000_0000 (0x00).
    // TODO: better docstring.
    pub fn send_trigger_measurement(&mut self) -> Result<(), Error<E>> {
        // Send TriggerMeasurement, read nothing back.
        let command: [u8; 3] = [
            Command::TriggerMeasurement as u8,
            // Two parameters as described in the datasheet. There is no indication what these
            // parameters mean, just that they should be provided. There is no returned value.
            // To get the measurement, see [measure](measure).
            0b0011_0011, // 0x33
            0b0000_0000, // 0x00
        ];

        self.i2c.write(self.address, &command).map_err(Error::I2c)?;

        Ok(())
    }

    // TODO: better docstring.
    // SoftReset = 0b1011_1010, // 0xBA, Section 5.3, page 8, Table 9.
    //                          //       Also see Section 5.5. This takes 20ms or less
    //                          //       to complete.
    pub fn send_soft_reset(&mut self, delay: &mut (impl DelayUs<u16> + DelayMs<u16>)) -> Result<(), Error<E>> {
        // Send SoftReset, read nothing back, wait 20ms.
        let command: [u8; 1] = [ Command::SoftReset as u8, ];

        self.i2c.write(self.address, &command).map_err(Error::I2c)?;
        // The datasheet in section 5.5 says there is a guarantee that the reset time does
        // not exceed 20ms.
        delay.delay_ms(20_u16);

        Ok(())
    }

    /// Measure temperature (and humidity) once.
    ///
    /// Command::TriggerMeasurement (0xAC)   ◄─┐
    ///                  │                     │
    ///                  ▼                     │
    ///             Wait 80 ms                 │
    ///                  │                     │
    ///                  ▼                     │
    ///   Command::CheckStatus (0x71) ◄──┐     │
    ///                  │               │     │
    ///                  ▼               │     │
    ///             Status::Busy  ───►  Yes    │
    ///                  │                     │
    ///                  ▼                     │
    ///                 No                     │
    ///                  │                     │
    ///                  ▼                     │
    ///             Read 6 bytes               │
    ///                  │                     │
    ///                  ▼                     │
    ///             Calculate CRC              │
    ///                  │                     │
    ///                  ▼                     │
    ///               CRC good ─► No  ─────────┘
    ///                  │
    ///                  ▼
    ///                 Yes
    ///                  │
    ///                  ▼
    ///        Calc Humidity and Temp
    /// TODO: Update doctring.
    pub fn measure(
        &mut self,
        delay: &mut (impl DelayUs<u16> + DelayMs<u16>),
    ) -> Result<SensorReading, Error<E>> {
        loop {
            let measurement_result = self.measure_once(delay);
            match measurement_result {
                Ok(sb) => {
                    return Ok(SensorReading::from_bytes([
                        sb[0], sb[1], sb[2], sb[3], sb[4],
                    ]))
                }
                Err(Error::InvalidCrc) => return Err(Error::InvalidCrc), // Q: Also log? How?
                Err(other) => return Err(other),
            }
        }
    }

    // TODO: docstring
    pub fn measure_once(
        &mut self,
        delay: &mut (impl DelayUs<u16> + DelayMs<u16>),
    ) -> Result<[u8; 5], Error<E>> {
        self.send_trigger_measurement()?;
        delay.delay_ms(80_u16);

        // Wait for measurement to be ready
        while !self.check_status()?.is_ready() {
            delay.delay_ms(1_u16);
        }

        // So, the datasheet is quite clear that we read back 7 bytes total. The first byte is a
        // status byte - which seems very redundant, as we have just been checking the status
        // above. However, it's there - but we can just ignore it. We can't get rid of it though
        // as it's part of the CRCd data. OK, being CRCd is an advantage though!
        // NOTE: for the blog - At first I only did the data part, and stripped out the
        //       status byte, thus got the wrong CRC value.
        // TODO OR: doublecheck the status byte - check for ready and calibrated again?
        // NOTE: So the CRC covers the status byte, which is nice I guess. So it could be
        //       a little more reliable? So I think we should use it.
        let mut read_buffer = [0u8; 7];
        self.i2c
            .read(self.address, &mut read_buffer)
            .map_err(Error::I2c)?;

        let data: &[u8] = &read_buffer[..6];
        let crc_byte: u8 = read_buffer[6];

        let crc = compute_crc(data);
        // TODO: delete
        // rprintln!("data: {:?}", data);
        // rprintln!("crc_byte: {:?}", crc_byte);
        // rprintln!("crc: {:?}", crc);
        if crc_byte != crc {
            return Err(Error::InvalidCrc);
        }

        // This is a little awkward, copying the bytes out, but it works.
        // Note that we're dropping the first byte, which is status, and byte
        // 7 which is the CRC.
        // Q: If this was longer, how should we do this? We don't want to copy out
        //    like 31 bytes like this. NOTE: Ask question in blog.
        Ok([data[1], data[2], data[3], data[4], data[5]])
    }

    /// Destroys this driver and releases the I2C bus `I`
    pub fn destroy(self) -> Self {
        self
    }
}

/// compute_crc uses the CRCu8 algoritm from crc-any. The parameter choice makes this a
/// "CRC-8-Dallas/Maxim".
///
/// The CRC invocation takes some parameters, which we get from the datasheet:
/// https://cdn-learn.adafruit.com/assets/assets/000/091/676/original/AHT20-datasheet-2020-4-16.pdf?1591047915
/// Section 5.4.4:
///
/// > CRC initial vaue is 0xFF, crc8 check polynomial CRC[7:0]=1+x**4 + x**5 + x**8
/// NOTE for blog. "Those are certainly some words". I had no idea how to go from that
///   description to parameters to put into the crc-any crate's functions. As I frequently
///   do when faced with something I don't know, I started on Wikipedia.
///
/// https://en.wikipedia.org/wiki/Cyclic_redundancy_check#Polynomial_representations_of_cyclic_redundancy_checks
/// You can find it in the table on wikipedia, under "CRC-8-Dallas/Maxim", 1-Wire bus.
///
/// NOTE: for blog. That wasn't actually that helpful. I found a likely magic value, but no
///   idea why. A little more googling led me to an article which actually explains it.
/// This article explains how we get from `CRC[7:0]=1 + x**4 + x**5 + x**8` to `0x31` as the hex
/// representation: http://www.sunshine2k.de/articles/coding/crc/understanding_crc.html#ch72
///
/// The **N is the Nth bit (zero indexed).
/// > The most significant bit [(x**8)] is left out in the hexadecimal representation
/// So that the leaves bit 0 (the +1 we do), 4, 5
///
/// So that gives us:
///
/// ```python
/// >>> hex(0x00110001)
/// '0x31'
/// ```
///
/// And, confirming that's right, this is what Knurling's test driver crate uses.
/// https://github.com/knurling-rs/test-driver-crate-example/blob/main/src/lib.rs#L59
/// which indicates this is either an I2C thing, or a common driver default as CRC parameters.
/// NOTE: for blog. The above.
fn compute_crc(bytes: &[u8]) -> u8 {
    // Poly (0x31), bits (8), initial (0xff), final_xor (0x00), reflect (flase).
    let mut crc = CRCu8::create_crc(0x31, 8, 0xff, 0x00, false);
    crc.digest(bytes);
    crc.get_crc()
}

// NOTE: for readme - show how to run the tests.
// NOTE: also set up github actions to run these tests?
#[cfg(test)]
mod tests {
    use super::{Error, AHT20, SENSOR_ADDRESS};
    // NOTE: For blog. Not having access to `use` things from the top
    // of this file took a while to figure out. It's probably in the
    // book, but so used to interior scopes having access to things from
    // the outside.
    use embedded_hal_mock::delay::MockNoop as MockDelay;
    use embedded_hal_mock::i2c::Mock as I2cMock;
    use embedded_hal_mock::i2c::Transaction;

    /// Test SensorStatus reporting being ready.
    #[test]
    fn sensorstatus_is_ready() {
        let status = super::SensorStatus::new(0x00);
        assert_eq!(status.is_ready(), true);
    }

    /// Test SensorStatus reporting being busy.
    #[test]
    fn sensorstatus_is_not_ready() {
        // 8th bit being 1 signifies "busy"
        // Equiv to 0x01 << 7, or 128 (dec) or 0x80 (hex)
        let status = super::SensorStatus::new(0b1000_0000);
        assert_eq!(status.is_ready(), false);
    }

    /// Test SensorStatus reporting being calibrated.
    #[test]
    fn sensorstatus_is_calibrated() {
        // 4th bit being 1 signifies the sensor being calibrated.
        // Equiv to 0x01 << 3, or 8 (dec) or 0x08
        let status = super::SensorStatus::new(0b0000_1000);
        assert_eq!(status.is_calibrated(), true);
    }

    /// Test SensorStatus reporting being uncalibrated.
    #[test]
    fn sensorstatus_is_not_calibrated() {
        let status = super::SensorStatus::new(0b0000_0000);
        assert_eq!(status.is_calibrated(), false);
    }

    /// Test creating new AHT20 sensors.
    ///
    /// Test that we can create multiple AHT20 devices. We test this because it's one of the
    /// measures of success for this driver.
    #[test]
    fn aht20_new() {
        // In the real app we'd used shared-bus to share the i2c bus between the two drivers, but
        // I think this is fine for a test.
        let mock_i2c_1 = I2cMock::new(&[]);
        let mock_i2c_2 = I2cMock::new(&[]);

        let _aht20_1 = AHT20::new(mock_i2c_1, SENSOR_ADDRESS);
        let _aht20_2 = AHT20::new(mock_i2c_2, SENSOR_ADDRESS);
    }

    /// Test sending the CheckStatus i2c command, and read a status byte back.
    #[test]
    fn check_status() {
        let expectations = vec![
            Transaction::write(SENSOR_ADDRESS, vec![super::Command::CheckStatus as u8]),
            // 4th bit being 1 signifies the sensor being calibrated.
            // Equiv to 0x01 << 3, or 8 (dec) or 0x08
            Transaction::read(SENSOR_ADDRESS, vec![0b0000_1000]),
        ];
        let mock_i2c = I2cMock::new(&expectations);

        let mut aht20 = AHT20::new(mock_i2c, SENSOR_ADDRESS);
        let status = aht20.check_status().unwrap();
        assert_eq!(status.is_calibrated(), true);

        let mut mock = aht20.destroy().i2c;
        mock.done(); // verify expectations
    }

    /// Test sending the i2c Initialize command.
    #[test]
    fn send_initialize() {
        let expectations = vec![Transaction::write(
            SENSOR_ADDRESS,
            vec![
                super::Command::Initialize as u8,
                0b0000_1000, // 0x08
                0b0000_0000, // 0x00
            ],
        )];
        let mock_i2c = I2cMock::new(&expectations);
        // let mut mock_delay = MockDelay::new();

        let mut aht20 = AHT20::new(mock_i2c, SENSOR_ADDRESS);
        aht20.send_initialize().unwrap();

        let mut mock = aht20.destroy().i2c;
        mock.done(); // verify expectations
    }

    /// Initialize sensor, with the sensor reporting calibrated immediately.
    ///
    /// No call to send_initialize will be required.
    #[test]
    fn init_with_calibrated_sensor() {
        // This test has check_status return an already calibrated sensor. This means
        // that send_initialize is not called.
        let expectations = vec![
            Transaction::write(SENSOR_ADDRESS, vec![super::Command::CheckStatus as u8]),
            // 4th bit being 1 signifies the sensor being calibrated.
            // Equiv to 0x01 << 3, or 8 (dec) or 0x08
            Transaction::read(SENSOR_ADDRESS, vec![0b0000_1000]),
        ];
        let mock_i2c = I2cMock::new(&expectations);
        let mut mock_delay = MockDelay::new();

        let mut aht20 = AHT20::new(mock_i2c, SENSOR_ADDRESS);
        assert_eq!(aht20.initialized, false);
        aht20.init(&mut mock_delay).unwrap();

        assert_eq!(aht20.initialized, true);
        let mut mock = aht20.destroy().i2c;
        mock.done(); // verify expectations
    }

    /// Initialize sensor, with a report of an uncalibrated sensor.
    ///
    /// The sensor will report being uncalibrated once, then after initialization the sensor will
    /// report being calibrated.
    #[test]
    fn init_with_uncalibrated_sensor() {
        // This test has check_status return an uncalibrated sensor. With that, a call
        // to send_initialize is done to initialize and calibrate the sensor. A second
        // call to check_status verifies the new calibrated status.
        let expectations = vec![
            // The first two transactions are check_status
            Transaction::write(SENSOR_ADDRESS, vec![super::Command::CheckStatus as u8]),
            // 4th bit being 0 signifies the sensor not being calibrated.
            Transaction::read(SENSOR_ADDRESS, vec![0b0000_0000]),
            // This is send_initialize
            Transaction::write(
                SENSOR_ADDRESS,
                vec![
                    super::Command::Initialize as u8,
                    0b0000_1000, // 0x08
                    0b0000_0000, // 0x00
                ],
            ),
            // One more check_status will be called, this time with the 4th bit set
            // to 1 - signifying the sensor is now calibrated and we can finish the init.
            Transaction::write(SENSOR_ADDRESS, vec![super::Command::CheckStatus as u8]),
            Transaction::read(SENSOR_ADDRESS, vec![0b0000_1000]),
        ];
        let mock_i2c = I2cMock::new(&expectations);
        let mut mock_delay = MockDelay::new();

        let mut aht20 = AHT20::new(mock_i2c, SENSOR_ADDRESS);
        assert_eq!(aht20.initialized, false);
        aht20.init(&mut mock_delay).unwrap();

        assert_eq!(aht20.initialized, true);
        let mut mock = aht20.destroy().i2c;
        mock.done(); // verify expectations
    }

    /// Test sending the i2c SoftReset command.
    #[test]
    fn send_soft_reset() {
        let expectations = vec![Transaction::write(
            SENSOR_ADDRESS,
            vec![ super::Command::SoftReset as u8, ],
        )];
        let mock_i2c = I2cMock::new(&expectations);
        let mut mock_delay = MockDelay::new();

        let mut aht20 = AHT20::new(mock_i2c, SENSOR_ADDRESS);
        aht20.send_soft_reset(&mut mock_delay).unwrap();

        let mut mock = aht20.destroy().i2c;
        mock.done(); // verify expectations
    }

    /// Test sending the i2c TriggerMeasurement command.
    #[test]
    fn send_trigger_measurement() {
        let expectations = vec![Transaction::write(
            SENSOR_ADDRESS,
            vec![
                super::Command::TriggerMeasurement as u8,
                0b0011_0011, // 0x33
                0b0000_0000, // 0x00
            ],
        )];
        let mock_i2c = I2cMock::new(&expectations);

        let mut aht20 = AHT20::new(mock_i2c, SENSOR_ADDRESS);
        aht20.send_trigger_measurement().unwrap();

        let mut mock = aht20.destroy().i2c;
        mock.done(); // verify expectations
    }

    /// Measure once, sensor reports ready at once.
    ///
    /// No wait is needed in this scenario.
    #[test]
    fn measure_once_immediately_ready() {
        let expectations = vec![
            // send_trigger_measurement
            Transaction::write(
                SENSOR_ADDRESS,
                vec![
                    super::Command::TriggerMeasurement as u8,
                    0b0011_0011, // 0x33
                    0b0000_0000, // 0x00
                ],
            ),
            // check_status called. 4th bit set to to 1 - signifying the sensor is calibrated 8th
            // bit set to 0 (not busy), signalling that a measurement is ready for us to read.
            Transaction::write(SENSOR_ADDRESS, vec![super::Command::CheckStatus as u8]),
            Transaction::read(SENSOR_ADDRESS, vec![0b0000_1000]),
            // We can now read 7 bytes. status byte, 5 data bytes, crc byte.
            // These are taken from a run of the sensor.
            Transaction::read(
                SENSOR_ADDRESS,
                vec![
                    0b0001_1100, //  28, 0x1c - ready, calibrated, and some mystery flags.
                    //             bit 8 set to 0 is ready. bit 4 set is calibrated. bit 5
                    //             and 3 are described as 'reserved'.
                    0b0110_0101, // 101, 0x65 - first byte of humidity value
                    0b1011_0100, // 180, 0xb4 - second byte of humidity vaue
                    0b0010_0101, //  37, 0x25 - split byte. 4 bits humidity, 4 bits temperature.
                    0b1100_1101, // 205, 0xcd - first full byte of temperature.
                    0b0010_0110, //  38, 0x26 - second full byte of temperature.
                    0b1100_0110, // 198, 0xc6 - CRC
                ],
            ),
        ];
        let mock_i2c = I2cMock::new(&expectations);
        let mut mock_delay = MockDelay::new();

        let mut aht20 = AHT20::new(mock_i2c, SENSOR_ADDRESS);
        aht20.measure_once(&mut mock_delay).unwrap();

        let mut mock = aht20.destroy().i2c;
        mock.done(); // verify expectations
    }

    /// Measure once, with a wait inserted.
    ///
    /// We signal via check_status that a wait should be inserted before another attempt to read
    /// data from the sensor is made.
    #[test]
    fn measure_once_wait_once() {
        let expectations = vec![
            // send_trigger_measurement
            Transaction::write(
                SENSOR_ADDRESS,
                vec![
                    super::Command::TriggerMeasurement as u8,
                    0b0011_0011, // 0x33
                    0b0000_0000, // 0x00
                ],
            ),
            // check_status called. 4th bit set to to 1 - signifying the sensor is calibrated 8th
            // bit set to 1 (busy), signalling that we should wait for the sensor.
            Transaction::write(SENSOR_ADDRESS, vec![super::Command::CheckStatus as u8]),
            Transaction::read(SENSOR_ADDRESS, vec![0b1000_1000]),
            // Next time round, we say that the sensor is good to go.
            Transaction::write(SENSOR_ADDRESS, vec![super::Command::CheckStatus as u8]),
            Transaction::read(SENSOR_ADDRESS, vec![0b0000_1000]),
            // We can now read 7 bytes. status byte, 5 data bytes, crc byte.
            // These are taken from a run of the sensor.
            Transaction::read(
                SENSOR_ADDRESS,
                vec![
                    0b0001_1100, //  28, 0x1c - ready, calibrated, and some mystery flags.
                    //             bit 8 set to 0 is ready. bit 4 set is calibrated. bit 5
                    //             and 3 are described as 'reserved'.
                    0b0110_0101, // 101, 0x65 - first byte of humidity value
                    0b1011_0100, // 180, 0xb4 - second byte of humidity vaue
                    0b0010_0101, //  37, 0x25 - split byte. 4 bits humidity, 4 bits temperature.
                    0b1100_1101, // 205, 0xcd - first full byte of temperature.
                    0b0010_0110, //  38, 0x26 - second full byte of temperature.
                    0b1100_0110, // 198, 0xc6 - CRC
                ],
            ),
        ];
        let mock_i2c = I2cMock::new(&expectations);
        let mut mock_delay = MockDelay::new();

        let mut aht20 = AHT20::new(mock_i2c, SENSOR_ADDRESS);
        aht20.measure_once(&mut mock_delay).unwrap();

        let mut mock = aht20.destroy().i2c;
        mock.done(); // verify expectations
    }

    // Single measurement pass with bad CRC.
    //
    // Intentionally corrupt the read data to make sure we get a CRC error.
    #[test]
    fn measure_once_bad_crc() {
        let expectations = vec![
            // send_trigger_measurement
            Transaction::write(
                SENSOR_ADDRESS,
                vec![
                    super::Command::TriggerMeasurement as u8,
                    0b0011_0011, // 0x33
                    0b0000_0000, // 0x00
                ],
            ),
            // Check status, and  we say that the sensor is good to go.
            Transaction::write(SENSOR_ADDRESS, vec![super::Command::CheckStatus as u8]),
            Transaction::read(SENSOR_ADDRESS, vec![0b0000_1000]),
            // We can now read 7 bytes. status byte, 5 data bytes, crc byte.
            // These are taken from a run of the sensor.
            Transaction::read(
                SENSOR_ADDRESS,
                vec![
                    0b0001_1100, //  28, 0x1c - ready, calibrated, and some mystery flags.
                    //             bit 8 set to 0 is ready. bit 4 set is calibrated. bit 5
                    //             and 3 are described as 'reserved'.
                    0b0110_0101, // 101, 0x65 - first byte of humidity value
                    0b1011_0100, // 180, 0xb4 - second byte of humidity vaue
                    0b0010_0101, //  37, 0x25 - split byte. 4 bits humidity, 4 bits temperature.
                    0b1100_1101, // 205, 0xcd - first full byte of temperature.
                    0b0010_0111, //  39, 0x27 - second full byte of temperature.
                    //  NOTE: This should be 38, 0x26, but is intentionally corrupted
                    //        so that the CRC won't match. Last bit flipped from 0 to 1.
                    0b1100_0110, // 198, 0xc6 - CRC
                ],
            ),
        ];
        let mock_i2c = I2cMock::new(&expectations);
        let mut mock_delay = MockDelay::new();

        // test and verify
        let mut aht20 = AHT20::new(mock_i2c, SENSOR_ADDRESS);
        match aht20.measure_once(&mut mock_delay) {
            Ok(_) => panic!("CRC is wrong and measure_once should not pass."),
            Err(err_type) => assert_eq!(err_type, Error::InvalidCrc),
        }

        let mut mock = aht20.destroy().i2c;
        mock.done(); // verify expectations
    }

    /// Test a measurement.
    ///
    /// This uses data from an actual sensor run.
    #[test]
    fn measure() {
        // setup
        let expectations = vec![
            // send_trigger_measurement
            Transaction::write(
                SENSOR_ADDRESS,
                vec![
                    super::Command::TriggerMeasurement as u8,
                    0b0011_0011, // 0x33
                    0b0000_0000, // 0x00
                ],
            ),
            // check_status - with ready bit set to 'ready' (off)
            Transaction::write(SENSOR_ADDRESS, vec![super::Command::CheckStatus as u8]),
            Transaction::read(SENSOR_ADDRESS, vec![0b0000_1000]),
            // We can now read 7 bytes. status byte, 5 data bytes, crc byte.
            // These are taken from a run of the sensor.
            Transaction::read(
                SENSOR_ADDRESS,
                vec![
                    0b0001_1100, //  28, 0x1c - ready, calibrated, and some mystery flags.
                    //             bit 8 set to 0 is ready. bit 4 set is calibrated. bit 5
                    //             and 3 are described as 'reserved'.
                    0b0110_0101, // 101, 0x65 - first byte of humidity value
                    0b1011_0100, // 180, 0xb4 - second byte of humidity vaue
                    0b0010_0101, //  37, 0x25 - split byte. 4 bits humidity, 4 bits temperature.
                    0b1100_1101, // 205, 0xcd - first full byte of temperature.
                    0b0010_0110, //  38, 0x26 - second full byte of temperature.
                    0b1100_0110, // 198, 0xc6 - CRC
                ],
            ),
        ];
        let mock_i2c = I2cMock::new(&expectations);
        let mut mock_delay = MockDelay::new();

        // test
        let mut aht20 = AHT20::new(mock_i2c, SENSOR_ADDRESS);
        let measurement = aht20.measure(&mut mock_delay).unwrap();

        // verification
        let mut mock = aht20.destroy().i2c;
        mock.done(); // verify expectations

        // Temp was ~22.5C and humidity ~40% when above data taken.
        assert!(measurement.temperature > 22.0 && measurement.temperature < 23.0);
        assert!(measurement.humidity > 39.0 && measurement.humidity < 41.0);
    }

    #[test]
    fn crc_correct() {
        // Example from the Interface Specification document.
        assert_eq!(super::compute_crc(&[0xBE, 0xEF]), 0x92);
    }

    #[test]
    fn crc_wrong() {
        // Changed example from the Interface Specification document. This should not match - the
        // bytes going in are changed from the known good values, but the expected result is the
        // same.
        assert_ne!(super::compute_crc(&[0xFF, 0xFF]), 0x92);
    }
}
