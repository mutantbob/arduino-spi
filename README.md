As of 2022-Mar the `Spi` implementation in `atmega-hal` is hardcoded to use one channel-select pin.
(i.e. d10/PB2 on an Arduino Uno) 

https://github.com/Rahix/avr-hal/blob/e897783816437a677aa577ddfdaa34e9a1e86d96/mcu/atmega-hal/src/spi.rs#L24

https://github.com/Rahix/avr-hal/blob/e897783816437a677aa577ddfdaa34e9a1e86d96/avr-hal-generic/src/spi.rs#L122


Unfortunately, this precludes the use of SPI peripherals using other channel select pins.

One example is the Arduino Ethernet Shield v1 which includes a w5100 chip attached to channel select 10, and an SD card reader attached to channel select 4.   They share the SCLK, MOSI, and MISO pins (which is a feature of SPI).
Another SPI peripheral is the OV5642 shield which can be wired to use any free pin for its channel select.

This crate provides an alternate SPI crate for Rust which is basically the code from `avr-hal-generic` `spi.rs` rewritten to separate the SCLK/MOSI/MISO pins from the channel select.

The primary feature is the `SpiTransaction` class which brings the 4 SPI pins (as borrowed mutable refs) together and pulls the channel select pin low until the `SpiTransaction`  object is `drop()`-ped.

As an example of how to use this crate consider this implementation of the OV5642 FIFO-burst read functionality:

```Rust
pub struct BurstReader<'a, P: PinOps> {
    spi: SpiTransaction0<'a, P>,
}

impl<'a, P: NumberedPin + PinOps> BurstReader<'a, P> {
    pub fn new(cam: &'a mut ArduCamOV5642<P>, spi: &'a mut Spi0) -> Self {
        let mut spi = spi.begin_transaction(&mut cam.cs_pin);
        spi.duplex_transfer(raw::BURST_FIFO_READ as u8); // tell the camera to start sending FIFO bytes as fast as we can read them
        BurstReader { spi }
    }
    pub fn next(&mut self) -> u8 {
        self.spi.duplex_transfer(0)
    }
}
```