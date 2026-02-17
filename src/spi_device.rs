//! Adapter from [`embedded_hal_async::spi::SpiDevice`] to this crate's [`Spi`] trait.
//!
//! Use this when your SPI peripheral is a **device** (with its own CS and possibly a shared bus)
//! rather than a raw bus. For example, with [embassy-embedded-hal] you can share one SPI bus
//! across multiple devices; each device gets a `SpiDevice` (bus + CS). Wrap that `SpiDevice` in
//! [`SpiDeviceAdapter`] to use it with [`Cat25040`].
//!
//! [embassy-embedded-hal]: https://crates.io/crates/embassy-embedded-hal
//!
//! # Example (embassy-embedded-hal shared SPI)
//!
//! ```ignore
//! use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
//! use embassy_sync::mutex::Mutex;
//! use cat25040::{spi_device::SpiDeviceAdapter, Cat25040, ...};
//!
//! static SPI_BUS: StaticCell<Mutex<NoopRawMutex, embassy_stm32::spi::Spi<...>>> = StaticCell::new();
//! let spi_bus = SPI_BUS.init(Mutex::new(spi));
//! let eeprom_cs = Output::new(p.PA12, Level::High, Speed::VeryHigh);
//! let eeprom_device = SpiDevice::new(spi_bus, eeprom_cs);
//! let adapter = SpiDeviceAdapter::new(eeprom_device);
//! let mut eeprom = Cat25040::new(adapter, EmbassyDelay);
//! ```

use crate::{Cat25040Error, Spi};
use embedded_hal_async::spi::{Operation, SpiDevice};

/// Wraps an [`embedded_hal_async::spi::SpiDevice`] and implements this crate's [`Spi`] trait.
///
/// Use with [embassy-embedded-hal]'s shared `SpiDevice` when multiple devices share one SPI bus.
#[derive(Debug)]
pub struct SpiDeviceAdapter<D> {
    device: D,
}

impl<D> SpiDeviceAdapter<D> {
    /// Create an adapter from any async SPI device (e.g. shared bus + CS).
    pub fn new(device: D) -> Self {
        Self { device }
    }
}

impl<D> SpiDeviceAdapter<D>
where
    D: SpiDevice<u8>,
{
    fn map_err(_: D::Error) -> Cat25040Error {
        Cat25040Error::Spi
    }
}

#[allow(async_fn_in_trait)]
impl<D> Spi for SpiDeviceAdapter<D>
where
    D: SpiDevice<u8>,
{
    async fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Cat25040Error> {
        self.device
            .transaction(operations)
            .await
            .map_err(SpiDeviceAdapter::<D>::map_err)
    }
}
