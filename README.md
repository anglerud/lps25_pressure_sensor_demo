This is a base program for the blue pill, with the st-link v2 programmer
and cargo-run.

Run with:

    $ cargo run --bin blue_pill_base --release

Next up is bringing in the lsp25 sensor and trying to get that to read

I2C on the bluepill, using SCL1 + SDA1 which is PB6 + PB7 - which I got from
the pinout diagram and chart on microcontrollerslab.com's [blue pill pinout -
peripherals programming
features](https://microcontrollerslab.com/stm32f103c8t6-blue-pill-pinout-peripherals-programming-features/)
article.

This is connected to the Adafruit LPS25 according to the [LPS25 pressure sensor
- pinouts](https://learn.adafruit.com/adafruit-lps25-pressure-sensor/pinouts)
article on adafruit.com:


* SCK - I2C clock pin, connect to your microcontroller's I2C clock line. This
  pin is level shifted so you can use 3-5V logic, and there's a 10K pullup on
  this pin.
* SDI - I2C data pin, connect to your microcontroller's I2C data line. This pin
  is level shifted so you can use 3-5V logic, and there's a 10K pullup on this
  pin.

