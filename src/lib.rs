#![no_std]

//! SPI Implementation
//use crate::port;
use avr_hal_generic::port;
use core::marker::PhantomData;
use embedded_hal::spi;
use void;

/// Oscillator Clock Frequency division options.
///
/// The bus speed is calculated by dividing the IO clock by the prescaler:
///
/// ```text
/// F_sck = CLK_io / Prescaler
/// ```
///
/// Please note that the overall transfer speed might be lower due to software overhead while
/// sending / receiving.
///
/// | Prescale | 16 MHz Clock | 8 MHz Clock |
/// | --- | --- | --- |
/// | `OscfOver2` | 8 MHz | 4 MHz |
/// | `OscfOver4` | 4 MHz | 2 MHz |
/// | `OscfOver8` | 2 MHz | 1 MHz |
/// | `OscfOver16` | 1 MHz | 500 kHz |
/// | `OscfOver32` | 500 kHz | 250 kHz |
/// | `OscfOver64` | 250 kHz | 125 kHz |
/// | `OscfOver128` | 125 kHz | 62.5 kHz |
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SerialClockRate {
    OscfOver2,
    OscfOver4,
    OscfOver8,
    OscfOver16,
    OscfOver32,
    OscfOver64,
    OscfOver128,
}

/// Order of data transmission, either MSB first or LSB first
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DataOrder {
    MostSignificantFirst,
    LeastSignificantFirst,
}

/// Settings to pass to Spi.
///
/// Easiest way to initialize is with
/// `Settings::default()`.  Otherwise can be instantiated with alternate
/// settings directly.
#[derive(Clone, PartialEq, Eq)]
pub struct Settings {
    pub data_order: DataOrder,
    pub clock: SerialClockRate,
    pub mode: spi::Mode,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            data_order: DataOrder::MostSignificantFirst,
            clock: SerialClockRate::OscfOver4,
            mode: spi::MODE_1,
        }
    }
}

/// Internal trait for low-level SPI peripherals
///
/// This trait defines the common interface for all SPI peripheral variants.  It is used as an
/// intermediate abstraction ontop of which the [`Spi`] API is built.  **Prefer using the
/// [`Spi`] API instead of this trait.**
pub trait SpiOps<H, SCLK, MOSI, MISO> {
    fn raw_setup(&mut self, settings: &Settings);
    fn raw_release(&mut self);

    fn raw_check_iflag(&self) -> bool;
    fn raw_read(&self) -> u8;
    fn raw_write(&mut self, byte: u8);
}

/// Wrapper for the CS pin
///
/// Used to contain the chip-select pin during operation to prevent its mode from being
/// changed from Output. This is necessary because the SPI state machine would otherwise
/// reset itself to SPI slave mode immediately. This wrapper can be used just like an
/// output pin, because it implements all the same traits from embedded-hal.
pub struct ChipSelectPin<CSPIN>(port::Pin<port::mode::Output, CSPIN>);

impl<CSPIN: port::PinOps> embedded_hal::digital::v2::OutputPin for ChipSelectPin<CSPIN> {
    type Error = crate::void::Void;
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.0.set_low();
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.0.set_high();
        Ok(())
    }
}

impl<CSPIN: port::PinOps> embedded_hal::digital::v2::StatefulOutputPin for ChipSelectPin<CSPIN> {
    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(self.0.is_set_low())
    }
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self.0.is_set_high())
    }
}

impl<CSPIN: port::PinOps> embedded_hal::digital::v2::ToggleableOutputPin for ChipSelectPin<CSPIN> {
    type Error = crate::void::Void;
    fn toggle(&mut self) -> Result<(), Self::Error> {
        self.0.toggle();
        Ok(())
    }
}

/// Behavior for a SPI interface.
///
/// Stores the SPI peripheral for register access.  In addition, it takes
/// ownership of the MOSI and MISO pins to ensure they are in the correct mode.
/// Instantiate with the `new` method.
pub struct Spi<H, SPI, SCLKPIN, MOSIPIN, MISOPIN> {
    p: SPI,
    sclk: port::Pin<port::mode::Output, SCLKPIN>,
    mosi: port::Pin<port::mode::Output, MOSIPIN>,
    miso: port::Pin<port::mode::Input, MISOPIN>,
    write_in_progress: bool,
    _h: PhantomData<H>,
}

impl<H, SPI, SCLKPIN, MOSIPIN, MISOPIN> Spi<H, SPI, SCLKPIN, MOSIPIN, MISOPIN>
where
    SPI: SpiOps<H, SCLKPIN, MOSIPIN, MISOPIN>,
    SCLKPIN: port::PinOps,
    MOSIPIN: port::PinOps,
    MISOPIN: port::PinOps,
{
    /// Instantiate an SPI with the registers, SCLK/MOSI/MISO/CS pins, and settings,
    /// with the internal pull-up enabled on the MISO pin.
    ///
    /// The pins are not actually used directly, but they are moved into the struct in
    /// order to enforce that they are in the correct mode, and cannot be used by anyone
    /// else while SPI is active.  CS is placed into a `ChipSelectPin` instance and given
    /// back so that its output state can be changed as needed.
    pub fn new(
        p: SPI,
        sclk: port::Pin<port::mode::Output, SCLKPIN>,
        mosi: port::Pin<port::mode::Output, MOSIPIN>,
        miso: port::Pin<port::mode::Input<port::mode::PullUp>, MISOPIN>,
        settings: Settings,
    ) -> Self {
        let mut spi = Self {
            p,
            sclk,
            mosi,
            miso: miso.forget_imode(),
            write_in_progress: false,
            _h: PhantomData,
        };
        spi.p.raw_setup(&settings);
        spi
    }

    /// Instantiate an SPI with the registers, SCLK/MOSI/MISO/CS pins, and settings,
    /// with an external pull-up on the MISO pin.
    ///
    /// The pins are not actually used directly, but they are moved into the struct in
    /// order to enforce that they are in the correct mode, and cannot be used by anyone
    /// else while SPI is active.
    pub fn with_external_pullup(
        p: SPI,
        sclk: port::Pin<port::mode::Output, SCLKPIN>,
        mosi: port::Pin<port::mode::Output, MOSIPIN>,
        miso: port::Pin<port::mode::Input<port::mode::Floating>, MISOPIN>,
        settings: Settings,
    ) -> Self {
        let mut spi = Self {
            p,
            sclk,
            mosi,
            miso: miso.forget_imode(),
            write_in_progress: false,
            _h: PhantomData,
        };
        spi.p.raw_setup(&settings);
        spi
    }

    /// Reconfigure the SPI peripheral after initializing
    pub fn reconfigure(&mut self, settings: Settings) -> nb::Result<(), crate::void::Void> {
        // wait for any in-flight writes to complete
        self.flush()?;
        self.p.raw_setup(&settings);
        Ok(())
    }

    /// Disable the SPI device and release ownership of the peripheral
    /// and pins.  Instance can no-longer be used after this is
    /// invoked.
    pub fn release(
        mut self,
    ) -> (
        SPI,
        port::Pin<port::mode::Output, SCLKPIN>,
        port::Pin<port::mode::Output, MOSIPIN>,
        port::Pin<port::mode::Input, MISOPIN>,
    ) {
        self.p.raw_release();
        (self.p, self.sclk, self.mosi, self.miso)
    }

    fn flush(&mut self) -> nb::Result<(), void::Void> {
        if self.write_in_progress {
            if self.p.raw_check_iflag() {
                self.write_in_progress = false;
            } else {
                return Err(nb::Error::WouldBlock);
            }
        }
        Ok(())
    }

    pub fn begin_transaction<'a, CSPIN: port::PinOps>(
        &'a mut self,
        channel_select: &'a mut port::Pin<port::mode::Output, CSPIN>,
    ) -> SpiTransaction<'a, H, SPI, SCLKPIN, MOSIPIN, MISOPIN, CSPIN> {
        SpiTransaction::new(self, channel_select)
    }
}

pub struct SpiTransaction<'a, H, SPI, SCLKPIN, MOSIPIN, MISOPIN, CSPIN: port::PinOps> {
    spi: &'a mut Spi<H, SPI, SCLKPIN, MOSIPIN, MISOPIN>,
    channel_select: &'a mut port::Pin<port::mode::Output, CSPIN>,
}

impl<'a, H, SPI, SCLKPIN, MOSIPIN, MISOPIN, CSPIN>
    SpiTransaction<'a, H, SPI, SCLKPIN, MOSIPIN, MISOPIN, CSPIN>
where
    SPI: SpiOps<H, SCLKPIN, MOSIPIN, MISOPIN>,
    CSPIN: port::PinOps,
{
    pub fn new(
        spi: &'a mut Spi<H, SPI, SCLKPIN, MOSIPIN, MISOPIN>,
        channel_select: &'a mut port::Pin<port::mode::Output, CSPIN>,
    ) -> Self {
        channel_select.set_low(); // start talking to the peripheral
        SpiTransaction {
            spi,
            channel_select,
        }
    }

    fn receive(&mut self) -> u8 {
        self.spi.p.raw_read()
    }

    fn write(&mut self, byte: u8) {
        self.spi.write_in_progress = true;
        self.spi.p.raw_write(byte);
    }
}

impl<'a, H, SPI, SCLKPIN, MOSIPIN, MISOPIN, CSPIN: port::PinOps> Drop
    for SpiTransaction<'a, H, SPI, SCLKPIN, MOSIPIN, MISOPIN, CSPIN>
{
    fn drop(&mut self) {
        // stop talking to the peripheral
        self.channel_select.set_high();
    }
}

impl<'a, H, SPI, SCLKPIN, MOSIPIN, MISOPIN, CSPIN>
    SpiTransaction<'a, H, SPI, SCLKPIN, MOSIPIN, MISOPIN, CSPIN>
where
    SPI: SpiOps<H, SCLKPIN, MOSIPIN, MISOPIN>,
    SCLKPIN: port::PinOps,
    MOSIPIN: port::PinOps,
    MISOPIN: port::PinOps,
    CSPIN: port::PinOps,
{
    pub fn duplex_transfer(&mut self, to_send: u8) -> u8 {
        nb::block!(self.spi.flush()).unwrap();
        self.write(to_send);
        nb::block!(self.spi.flush()).unwrap();
        self.receive()
    }
}

/// FullDuplex trait implementation, allowing this struct to be provided to
/// drivers that require it for operation.  Only 8-bit word size is supported
/// for now.
impl<'a, H, SPI, SCLKPIN, MOSIPIN, MISOPIN, CSPIN> spi::FullDuplex<u8>
    for SpiTransaction<'a, H, SPI, SCLKPIN, MOSIPIN, MISOPIN, CSPIN>
where
    SPI: SpiOps<H, SCLKPIN, MOSIPIN, MISOPIN>,
    SCLKPIN: port::PinOps,
    MOSIPIN: port::PinOps,
    MISOPIN: port::PinOps,
    CSPIN: port::PinOps,
{
    type Error = void::Void;

    /// Sets up the device for transmission and sends the data
    fn send(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        self.spi.flush()?;
        self.write(byte);
        Ok(())
    }

    /// Reads and returns the response in the data register
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.spi.flush()?;
        Ok(self.receive())
    }
}

/// Default Transfer trait implementation. Only 8-bit word size is supported for now.
impl<'a, H, SPI, SCLKPIN, MOSIPIN, MISOPIN, CSPIN>
    embedded_hal::blocking::spi::transfer::Default<u8>
    for SpiTransaction<'a, H, SPI, SCLKPIN, MOSIPIN, MISOPIN, CSPIN>
where
    SPI: SpiOps<H, SCLKPIN, MOSIPIN, MISOPIN>,
    SCLKPIN: port::PinOps,
    MOSIPIN: port::PinOps,
    MISOPIN: port::PinOps,
    CSPIN: port::PinOps,
{
}

/// Default Write trait implementation. Only 8-bit word size is supported for now.
impl<'a, H, SPI, SCLKPIN, MOSIPIN, MISOPIN, CSPIN> embedded_hal::blocking::spi::write::Default<u8>
    for SpiTransaction<'a, H, SPI, SCLKPIN, MOSIPIN, MISOPIN, CSPIN>
where
    SPI: SpiOps<H, SCLKPIN, MOSIPIN, MISOPIN>,
    SCLKPIN: port::PinOps,
    MOSIPIN: port::PinOps,
    MISOPIN: port::PinOps,
    CSPIN: port::PinOps,
{
}

/// Implement traits for a SPI interface
#[macro_export]
macro_rules! impl_spi {
    (
        hal: $HAL:ty,
        peripheral: $SPI:ty,
        sclk: $sclkpin:ty,
        mosi: $mosipin:ty,
        miso: $misopin:ty,
    ) => {
        impl $crate::SpiOps<$HAL, $sclkpin, $mosipin, $misopin> for $SPI {
            /// Sets up the control/status registers with the right settings for this secondary device
            fn raw_setup(&mut self, settings: &Settings) {
                use embedded_hal::spi;

                // set up control register
                self.spcr.write(|w| {
                    // enable SPI
                    w.spe().set_bit();
                    // Set to primary mode
                    w.mstr().set_bit();
                    // set up data order control bit
                    match settings.data_order {
                        DataOrder::MostSignificantFirst => w.dord().clear_bit(),
                        DataOrder::LeastSignificantFirst => w.dord().set_bit(),
                    };
                    // set up polarity control bit
                    match settings.mode.polarity {
                        spi::Polarity::IdleHigh => w.cpol().set_bit(),
                        spi::Polarity::IdleLow => w.cpol().clear_bit(),
                    };
                    // set up phase control bit
                    match settings.mode.phase {
                        spi::Phase::CaptureOnFirstTransition => w.cpha().clear_bit(),
                        spi::Phase::CaptureOnSecondTransition => w.cpha().set_bit(),
                    };
                    // set up clock rate control bit
                    match settings.clock {
                        SerialClockRate::OscfOver2 => w.spr().fosc_4_2(),
                        SerialClockRate::OscfOver4 => w.spr().fosc_4_2(),
                        SerialClockRate::OscfOver8 => w.spr().fosc_16_8(),
                        SerialClockRate::OscfOver16 => w.spr().fosc_16_8(),
                        SerialClockRate::OscfOver32 => w.spr().fosc_64_32(),
                        SerialClockRate::OscfOver64 => w.spr().fosc_64_32(),
                        SerialClockRate::OscfOver128 => w.spr().fosc_128_64(),
                    }
                });
                // set up 2x clock rate status bit
                self.spsr.write(|w| match settings.clock {
                    SerialClockRate::OscfOver2 => w.spi2x().set_bit(),
                    SerialClockRate::OscfOver4 => w.spi2x().clear_bit(),
                    SerialClockRate::OscfOver8 => w.spi2x().set_bit(),
                    SerialClockRate::OscfOver16 => w.spi2x().clear_bit(),
                    SerialClockRate::OscfOver32 => w.spi2x().set_bit(),
                    SerialClockRate::OscfOver64 => w.spi2x().clear_bit(),
                    SerialClockRate::OscfOver128 => w.spi2x().clear_bit(),
                });
            }

            /// Disable the peripheral
            fn raw_release(&mut self) {
                self.spcr.write(|w| w.spe().clear_bit());
            }

            /// Check the interrupt flag to see if the write has completed
            fn raw_check_iflag(&self) -> bool {
                self.spsr.read().spif().bit_is_set()
            }

            /// Read a byte from the data register
            fn raw_read(&self) -> u8 {
                self.spdr.read().bits()
            }

            /// Write a byte to the data register, which begins transmission
            /// automatically.
            fn raw_write(&mut self, byte: u8) {
                self.spdr.write(|w| unsafe { w.bits(byte) });
            }
        }
    };
}
//use arduino_hal::hal::Atmega;

impl_spi! {
    hal: arduino_hal::hal::Atmega,
    peripheral:
    //atmega_hal::pac::
    arduino_hal::pac::SPI,
    sclk: arduino_hal::hal::port::PB5,
    mosi: arduino_hal::hal::port::PB3,
    miso: arduino_hal::hal::port::PB4,
}

pub type Spi0 = Spi<
    arduino_hal::hal::Atmega,
    arduino_hal::pac::SPI,
    arduino_hal::hal::port::PB5,
    arduino_hal::hal::port::PB3,
    arduino_hal::hal::port::PB4,
>;

pub type SpiTransaction0<'a, CS> = SpiTransaction<
    'a,
    arduino_hal::hal::Atmega,
    arduino_hal::pac::SPI,
    arduino_hal::hal::port::PB5,
    arduino_hal::hal::port::PB3,
    arduino_hal::hal::port::PB4,
    CS,
>;
