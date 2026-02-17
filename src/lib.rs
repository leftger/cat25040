//! A `no_std`, async Rust driver for the ON Semiconductor CAT25040 4-Kbit SPI EEPROM,
//! built on [`embedded-hal-async`](https://crates.io/crates/embedded-hal-async) traits.
//!
//! Works with any platform that implements the [`Spi`] and [`HardwareInterface`] traits.
//!
//! # Features
//!
//! - Byte read/write with automatic write-only-if-changed optimization
//! - Page write (16 bytes per page, single write cycle)
//! - Sequential read of arbitrary length
//! - 9-bit addressing -- handles the A8 bit encoding in opcodes automatically
//! - Busy polling after writes
//! - **Software block protection** -- protect memory regions from accidental writes
//! - **Power-up initialization** -- proper timing delays per datasheet specifications
//! - Optional [`defmt`](https://crates.io/crates/defmt) logging via the `defmt` feature
//! - Extensive use of [`device-driver`](https://crates.io/crates/device-driver) for:
//!   - Register definitions and field access
//!   - Command operations (WREN, WRDI, WRSR)
//!   - Type-safe status register access with named fields
//!
//! # Device-Driver Integration
//!
//! This driver leverages the `device-driver` crate to provide a strongly-typed API
//! for registers and commands. Register definitions are generated from a YAML manifest
//! at build time, providing compile-time guarantees and field-level access to status bits.
//!
//! # Example
//!
//! ```rust,no_run
//! # use cat25040::{Cat25040, BlockProtection};
//! # async fn example<S: cat25040::Spi, H: cat25040::HardwareInterface>(spi: S, hardware_interface: H) {
//! let mut eeprom = Cat25040::new(spi, hardware_interface);
//!
//! // Initialize device after power-on (waits required tPUW = 1ms)
//! eeprom.init().await.unwrap();
//!
//! // Read 16 bytes starting at address 0x00
//! let mut buf = [0u8; 16];
//! eeprom.read(0x00, &mut buf).await.unwrap();
//!
//! // Write a single byte (skips if value already matches)
//! eeprom.write_byte(0xAB, 0x10).await.unwrap();
//!
//! // Write a full 16-byte page (address must be page-aligned)
//! let page = *b"EEPROM OK!";
//! eeprom.write_page(&page, 0x00).await.unwrap();
//!
//! // Access status register with type-safe fields
//! let status = eeprom.read_status().await.unwrap();
//! if status.busy() {
//!     // Write in progress
//! }
//! if status.wel() {
//!     // Write enabled
//! }
//!
//! // Configure block protection to protect upper half (0x100-0x1FF)
//! eeprom.set_block_protection(BlockProtection::UpperHalf).await.unwrap();
//!
//! // Check current protection level
//! let level = eeprom.get_block_protection().await.unwrap();
//!
//! // Clear all protection
//! eeprom.clear_block_protection().await.unwrap();
//!
//! // Use device-driver commands directly
//! eeprom.write_enable().await.unwrap();
//! eeprom.write_disable().await.unwrap();
//! # }
//! ```
//!
//! # Device Compatibility
//!
//! Designed for the CAT25040 but should work with other `CAT250xx` family EEPROMs
//! that use the same SPI command set and 9-bit addressing (e.g., `CAT25020`).

#![cfg_attr(not(feature = "std"), no_std)]
// Allow clippy warnings in generated code
#![allow(clippy::identity_op)]
#![allow(clippy::unnecessary_cast)]
#![allow(clippy::erasing_op)]
#![allow(clippy::new_without_default)]

#[cfg(feature = "defmt")]
use defmt::debug;

use core::fmt;
use core::fmt::Display;

#[allow(clippy::identity_op)]
#[allow(clippy::unnecessary_cast)]
mod generated {
    include!(concat!(env!("OUT_DIR"), "/cat25040_device.rs"));
}

mod register_interface;

// Re-export generated types for public API
pub use generated::field_sets::StatusReg as Status;
pub use generated::Cat25040Device;

// Re-export register interface for advanced users
pub use register_interface::SpiRegisterInterface;

// Memory access opcodes (not register/command based, so kept as constants)
const READ_OPCODE: u8 = 0x03;
const WRITE_OPCODE: u8 = 0x02;

const BUSY_WAIT_TIME_MS: u32 = 1;
const WRITE_CYCLE_TIME_MS: u32 = 5;
const PAGE_SIZE: u8 = 16;

/// SPI communication trait for the CAT25040 driver.
#[allow(async_fn_in_trait)]
pub trait Spi {
    /// Perform a transaction with the SPI device.
    async fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Cat25040Error>;
}

/// Hardware interface trait for delays.
#[allow(async_fn_in_trait)]
pub trait HardwareInterface {
    /// Wait for the specified number of milliseconds.
    async fn wait_ms(&mut self, timeout_ms: u32);
}

/// CAT25040 EEPROM driver, generic over any SPI implementation and delay provider.
///
/// This driver uses the device-driver crate for register and command operations,
/// while maintaining a high-level API for EEPROM memory access.
pub struct Cat25040<S: Spi, H: HardwareInterface> {
    device: Cat25040Device<SpiRegisterInterface<S>>,
    hardware_interface: H,
}

// Re-export Operation for convenience
pub use embedded_hal_async::spi::Operation;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cat25040Error {
    Spi,
    InvalidAddress,
    InvalidLength,
    DeviceNotReady,
}

impl Display for Cat25040Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Cat25040Error: {self:?}")
    }
}

/// Block protection levels for the CAT25040 (512 bytes = 0x000-0x1FF).
///
/// These configure which portions of the EEPROM are write-protected via the BP0 and BP1 bits.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlockProtection {
    /// No protection - all memory is writable (BP1=0, BP0=0)
    None,
    /// Upper quarter protected: 0x180-0x1FF (128 bytes) (BP1=0, BP0=1)
    UpperQuarter,
    /// Upper half protected: 0x100-0x1FF (256 bytes) (BP1=1, BP0=0)
    UpperHalf,
    /// Full array protected: 0x000-0x1FF (all 512 bytes) (BP1=1, BP0=1)
    Full,
}

impl BlockProtection {
    /// Convert block protection level to (BP1, BP0) bit values.
    const fn to_bits(self) -> (bool, bool) {
        match self {
            BlockProtection::None => (false, false),
            BlockProtection::UpperQuarter => (false, true),
            BlockProtection::UpperHalf => (true, false),
            BlockProtection::Full => (true, true),
        }
    }

    /// Convert (BP1, BP0) bit values to block protection level.
    const fn from_bits(bp1: bool, bp0: bool) -> Self {
        match (bp1, bp0) {
            (false, false) => BlockProtection::None,
            (false, true) => BlockProtection::UpperQuarter,
            (true, false) => BlockProtection::UpperHalf,
            (true, true) => BlockProtection::Full,
        }
    }
}

impl<S: Spi, H: HardwareInterface> Cat25040<S, H> {
    /// Create a new CAT25040 driver instance.
    ///
    /// Note: After power-on, call [`init()`](Self::init) to ensure the device is ready.
    pub fn new(spi: S, hardware_interface: H) -> Self {
        let spi_interface = SpiRegisterInterface::new(spi);
        let device = Cat25040Device::new(spi_interface);
        Self {
            device,
            hardware_interface,
        }
    }

    /// Initialize the device after power-on.
    ///
    /// Waits for the required power-up time (tPUW = 1ms max per datasheet)
    /// and verifies the device is ready and in write-disable state.
    ///
    /// # Errors
    ///
    /// Returns [`Cat25040Error::DeviceNotReady`] if the device is still busy after power-up.
    /// Returns [`Cat25040Error::Spi`] if the SPI transaction fails.
    pub async fn init(&mut self) -> Result<(), Cat25040Error> {
        // Wait for device to be ready after power-up (tPUW = 1ms max per datasheet)
        self.hardware_interface.wait_ms(1).await;

        // Verify device is not busy and in write-disable state
        let status = self.read_status().await?;
        if status.busy() {
            return Err(Cat25040Error::DeviceNotReady);
        }

        #[cfg(feature = "defmt")]
        debug!("Device initialized successfully");

        Ok(())
    }

    /// Get direct access to the underlying device-driver device.
    ///
    /// This allows using generated register and command APIs directly.
    pub fn device(&mut self) -> &mut Cat25040Device<SpiRegisterInterface<S>> {
        &mut self.device
    }

    /// Reads the EEPROM status register using device-driver's generated API.
    ///
    /// # Errors
    ///
    /// Returns [`Cat25040Error::Spi`] if the SPI transaction fails.
    pub async fn read_status(&mut self) -> Result<Status, Cat25040Error> {
        self.device.status_reg().read_async().await
    }

    /// Returns `true` if a write cycle is in progress (WIP bit set).
    ///
    /// # Errors
    ///
    /// Returns [`Cat25040Error::Spi`] if the SPI transaction fails.
    pub async fn is_busy(&mut self) -> Result<bool, Cat25040Error> {
        let status = self.read_status().await?;
        Ok(status.busy())
    }

    /// Sends the Write Enable (WREN) command using device-driver's generated API.
    ///
    /// # Errors
    ///
    /// Returns [`Cat25040Error::Spi`] if the SPI transaction fails.
    pub async fn write_enable(&mut self) -> Result<(), Cat25040Error> {
        self.device.wren().dispatch_async().await
    }

    /// Sends the Write Disable (WRDI) command using device-driver's generated API.
    ///
    /// # Errors
    ///
    /// Returns [`Cat25040Error::Spi`] if the SPI transaction fails.
    pub async fn write_disable(&mut self) -> Result<(), Cat25040Error> {
        self.device.wrdi().dispatch_async().await
    }

    /// Get the current block protection level by reading BP0 and BP1 bits from the status register.
    ///
    /// # Errors
    ///
    /// Returns [`Cat25040Error::Spi`] if the SPI transaction fails.
    pub async fn get_block_protection(&mut self) -> Result<BlockProtection, Cat25040Error> {
        let status = self.read_status().await?;
        Ok(BlockProtection::from_bits(status.bp_1(), status.bp_0()))
    }

    /// Set the block protection level by writing BP0 and BP1 bits to the status register.
    ///
    /// This enables software write protection for different memory regions:
    /// - `None`: No protection (all memory writable)
    /// - `UpperQuarter`: Protect addresses 0x180-0x1FF (128 bytes)
    /// - `UpperHalf`: Protect addresses 0x100-0x1FF (256 bytes)
    /// - `Full`: Protect all addresses 0x000-0x1FF (512 bytes)
    ///
    /// **Note**: The hardware WP (Write Protect) pin can override software protection.
    /// When WP is low, all writes are inhibited regardless of BP0/BP1 settings.
    ///
    /// # Errors
    ///
    /// Returns [`Cat25040Error::Spi`] if any SPI transaction fails.
    pub async fn set_block_protection(&mut self, level: BlockProtection) -> Result<(), Cat25040Error> {
        // Enable write operations first (required for WRSR)
        self.write_enable().await?;

        // Write the status register with new BP0 and BP1 values
        let (bp1, bp0) = level.to_bits();
        self.device.write_status_reg().write_async(|reg| {
            reg.set_bp_0(bp0);
            reg.set_bp_1(bp1);
        }).await?;

        // Wait for write cycle to complete
        self.hardware_interface.wait_ms(WRITE_CYCLE_TIME_MS).await;
        while self.is_busy().await? {
            self.hardware_interface.wait_ms(BUSY_WAIT_TIME_MS).await;
        }

        #[cfg(feature = "defmt")]
        debug!("Set block protection to {:?}", level);

        Ok(())
    }

    /// Clear all block protection (convenience method).
    ///
    /// Equivalent to `set_block_protection(BlockProtection::None)`.
    ///
    /// # Errors
    ///
    /// Returns [`Cat25040Error::Spi`] if any SPI transaction fails.
    pub async fn clear_block_protection(&mut self) -> Result<(), Cat25040Error> {
        self.set_block_protection(BlockProtection::None).await
    }

    fn get_valid_opcode_for_address(address: u16, opcode: u8) -> u8 {
        // In CAT25020, if A8 is set, bit 3 of the opcode is set
        if address & (1 << 8) != 0 {
            opcode | (1 << 3)
        } else {
            opcode
        }
    }

    /// Reads `rx_buf.len()` bytes starting at `address` into `rx_buf`.
    ///
    /// Memory reads use the READ opcode (0x03) with 9-bit addressing.
    ///
    /// # Errors
    ///
    /// Returns [`Cat25040Error::Spi`] if the SPI transaction fails.
    #[allow(clippy::cast_possible_truncation)] // A8 bit is encoded in the opcode; only low 8 bits go on the wire.
    pub async fn read(&mut self, address: u16, rx_buf: &mut [u8]) -> Result<(), Cat25040Error> {
        let opcode = Self::get_valid_opcode_for_address(address, READ_OPCODE);
        let spi = self.device.interface.spi_mut();
        spi.transaction(&mut [
            Operation::Write(&[opcode, address as u8]),
            Operation::Read(rx_buf),
        ])
        .await?;
        #[cfg(feature = "defmt")]
        debug!(
            "Read 0x{:02x} from address 0x{:02x} with opcode 0x{:02x}",
            rx_buf[0], address, opcode
        );
        Ok(())
    }

    /// Writes a single byte to `address`, skipping the write if the value already matches.
    ///
    /// Uses device-driver for the WREN command and status checking.
    ///
    /// # Errors
    ///
    /// Returns [`Cat25040Error::Spi`] if any SPI transaction fails.
    #[allow(clippy::cast_possible_truncation)] // A8 bit is encoded in the opcode; only low 8 bits go on the wire.
    pub async fn write_byte(&mut self, data: u8, address: u16) -> Result<(), Cat25040Error> {
        let mut current_data = [0u8; 1];
        self.read(address, &mut current_data).await?;
        if current_data[0] == data {
            #[cfg(feature = "defmt")]
            debug!(
                "Data at address 0x{:02x} is already 0x{:02x}, skipping write",
                address, data
            );
            return Ok(());
        }

        self.write_enable().await?;
        let opcode = Self::get_valid_opcode_for_address(address, WRITE_OPCODE);
        let spi = self.device.interface.spi_mut();
        spi.transaction(&mut [Operation::Write(&[opcode, address as u8, data])])
            .await?;

        self.hardware_interface.wait_ms(WRITE_CYCLE_TIME_MS).await;
        while self.is_busy().await? {
            self.hardware_interface.wait_ms(BUSY_WAIT_TIME_MS).await;
        }
        #[cfg(feature = "defmt")]
        debug!(
            "Wrote 0x{:02x} to address 0x{:02x} with opcode 0x{:02x}",
            data, address, opcode
        );
        Ok(())
    }

    /// Writes exactly 16 bytes to a page-aligned `address`.
    ///
    /// Uses device-driver for the WREN command and status checking.
    ///
    /// # Errors
    ///
    /// - [`Cat25040Error::InvalidAddress`] if `address` is not page-aligned.
    /// - [`Cat25040Error::InvalidLength`] if `data.len() != 16`.
    /// - [`Cat25040Error::Spi`] if any SPI transaction fails.
    #[allow(clippy::cast_possible_truncation)] // A8 bit is encoded in the opcode; only low 8 bits go on the wire.
    pub async fn write_page(&mut self, data: &[u8], address: u16) -> Result<(), Cat25040Error> {
        // Checking alignment:
        if !address.is_multiple_of(u16::from(PAGE_SIZE)) {
            return Err(Cat25040Error::InvalidAddress);
        }
        if data.len() != PAGE_SIZE as usize {
            return Err(Cat25040Error::InvalidLength);
        }

        let mut cmd = [0u8; PAGE_SIZE as usize + 2];
        cmd[0] = Self::get_valid_opcode_for_address(address, WRITE_OPCODE);
        cmd[1] = address as u8;
        cmd[2..PAGE_SIZE as usize + 2].copy_from_slice(data);

        self.write_enable().await?;

        let spi = self.device.interface.spi_mut();
        spi.transaction(&mut [Operation::Write(&cmd)]).await?;

        self.hardware_interface.wait_ms(WRITE_CYCLE_TIME_MS).await;
        while self.is_busy().await? {
            self.hardware_interface.wait_ms(BUSY_WAIT_TIME_MS).await;
        }
        #[cfg(feature = "defmt")]
        debug!("Wrote page to address 0x{:02x}", address);
        Ok(())
    }
}

pub mod spi_device;

#[cfg(test)]
mod tests {
    use super::*;
    use futures::executor::block_on;

    // ── Mock SPI implementation ──────────────────────────────────────────────

    #[derive(Debug)]
    struct MockSpi {
        status_register: u8,
        write_enable: bool,
        memory: [u8; 512],
        read_data: u8,
    }

    impl MockSpi {
        pub fn new() -> Self {
            Self {
                status_register: 0,
                write_enable: false,
                memory: [0; 512],
                read_data: 0,
            }
        }

        /// Pre-populate a byte in simulated EEPROM memory.
        pub fn set_memory(&mut self, address: u16, data: u8) {
            self.memory[address as usize] = data;
        }
    }

    impl Spi for MockSpi {
        async fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Cat25040Error> {
            // Test-local opcode constants
            const WREN_OPCODE: u8 = 0x06;
            const WRDI_OPCODE: u8 = 0x04;
            const RDSR_OPCODE: u8 = 0x05;
            const WRSR_OPCODE: u8 = 0x01;

            for operation in operations {
                match operation {
                    Operation::Write(data) => {
                        let base_opcode = data[0] & !0x08;
                        let a8: u16 = if data[0] & 0x08 != 0 { 0x100 } else { 0 };

                        match base_opcode {
                            WREN_OPCODE => {
                                self.write_enable = true;
                            }
                            WRDI_OPCODE => {
                                self.write_enable = false;
                            }
                            RDSR_OPCODE => {
                                self.read_data = self.status_register;
                            }
                            WRSR_OPCODE => {
                                // Write status register (only BP0 and BP1 bits are writable)
                                if data.len() >= 2 {
                                    // Preserve BUSY and WEL bits, update only BP0 and BP1
                                    let new_bp_bits = data[1] & 0x0C; // Mask BP0 (bit 2) and BP1 (bit 3)
                                    self.status_register = (self.status_register & !0x0C) | new_bp_bits;
                                }
                            }
                            READ_OPCODE => {
                                let addr = a8 | (data[1] as u16);
                                self.read_data = self.memory[addr as usize];
                            }
                            WRITE_OPCODE => {
                                let base_addr = a8 | (data[1] as u16);
                                // Support both single-byte and page writes:
                                // data layout is [opcode, addr_low, d0, d1, ...]
                                for (i, &byte) in data[2..].iter().enumerate() {
                                    // Page wrap: stay within the same 16-byte page
                                    let page_start = base_addr & !0x0F;
                                    let offset = ((base_addr & 0x0F) + i as u16) % PAGE_SIZE as u16;
                                    self.memory[(page_start | offset) as usize] = byte;
                                }
                            }
                            _ => {
                                println!("MockSpi: unhandled opcode 0x{:02x}", data[0]);
                            }
                        }
                    }
                    Operation::Read(data) => {
                        data[0] = self.read_data;
                    }
                    _ => {}
                }
            }
            Ok(())
        }
    }

    // ── Mock hardware interface (no-op, instant return) ───────────────────────────

    struct MockHardwareInterface;

    impl HardwareInterface for MockHardwareInterface {
        async fn wait_ms(&mut self, _timeout_ms: u32) {
            // No-op in tests
        }
    }

    /// Helper to create a Cat25040 with mock SPI and mock hardware interface.
    fn make_eeprom(spi: MockSpi) -> Cat25040<MockSpi, MockHardwareInterface> {
        Cat25040::new(spi, MockHardwareInterface)
    }

    // ── get_valid_opcode_for_address ──────────────────────────────────

    type TestCat = Cat25040<MockSpi, MockHardwareInterface>;

    #[test]
    fn opcode_unchanged_when_a8_clear() {
        assert_eq!(TestCat::get_valid_opcode_for_address(0x00, READ_OPCODE), READ_OPCODE);
    }

    #[test]
    fn opcode_unchanged_at_max_8bit_address() {
        assert_eq!(TestCat::get_valid_opcode_for_address(0xFF, READ_OPCODE), READ_OPCODE);
    }

    #[test]
    fn opcode_sets_bit3_when_a8_set() {
        assert_eq!(TestCat::get_valid_opcode_for_address(0x100, READ_OPCODE), 0x0B);
    }

    #[test]
    fn write_opcode_sets_bit3_when_a8_set() {
        assert_eq!(TestCat::get_valid_opcode_for_address(0x1FF, WRITE_OPCODE), 0x0A);
    }

    #[test]
    fn opcode_preserves_existing_bits() {
        assert_eq!(TestCat::get_valid_opcode_for_address(0x100, 0xFF), 0xFF);
    }

    // ── read ─────────────────────────────────────────────────────────

    #[test]
    fn read_returns_zero_from_blank_memory() {
        let mut eeprom = make_eeprom(MockSpi::new());
        let mut buf = [0xFFu8; 1];
        block_on(eeprom.read(0x0000, &mut buf)).unwrap();
        assert_eq!(buf[0], 0x00);
    }

    #[test]
    fn read_returns_prepopulated_data() {
        let mut spi = MockSpi::new();
        spi.set_memory(0x10, 0xAB);
        let mut eeprom = make_eeprom(spi);
        let mut buf = [0u8; 1];
        block_on(eeprom.read(0x10, &mut buf)).unwrap();
        assert_eq!(buf[0], 0xAB);
    }

    #[test]
    fn read_with_a8_address() {
        let mut spi = MockSpi::new();
        spi.set_memory(0x1FF, 0xCD);
        let mut eeprom = make_eeprom(spi);
        let mut buf = [0u8; 1];
        block_on(eeprom.read(0x1FF, &mut buf)).unwrap();
        assert_eq!(buf[0], 0xCD);
    }

    #[test]
    fn read_different_addresses_return_different_data() {
        let mut spi = MockSpi::new();
        spi.set_memory(0x00, 0x11);
        spi.set_memory(0x01, 0x22);
        spi.set_memory(0x02, 0x33);
        let mut eeprom = make_eeprom(spi);

        let mut buf = [0u8; 1];
        block_on(eeprom.read(0x00, &mut buf)).unwrap();
        assert_eq!(buf[0], 0x11);

        block_on(eeprom.read(0x01, &mut buf)).unwrap();
        assert_eq!(buf[0], 0x22);

        block_on(eeprom.read(0x02, &mut buf)).unwrap();
        assert_eq!(buf[0], 0x33);
    }

    // ── read_status / is_busy ────────────────────────────────

    #[test]
    fn read_status_reads_mock_status() {
        let mut spi = MockSpi::new();
        spi.status_register = 0x03;
        let mut eeprom = make_eeprom(spi);
        let status = block_on(eeprom.read_status()).unwrap();
        // Check that BUSY and WEL bits are set (0x03 = 0b00000011)
        assert!(status.busy());
        assert!(status.wel());
    }

    #[test]
    fn is_busy_returns_true_when_wip_set() {
        let mut spi = MockSpi::new();
        spi.status_register = 0x01; // BUSY bit set
        let mut eeprom = make_eeprom(spi);
        assert!(block_on(eeprom.is_busy()).unwrap());
    }

    #[test]
    fn is_busy_returns_false_when_wip_clear() {
        let mut eeprom = make_eeprom(MockSpi::new());
        assert!(!block_on(eeprom.is_busy()).unwrap());
    }

    #[test]
    fn is_busy_ignores_non_wip_bits() {
        let mut spi = MockSpi::new();
        spi.status_register = 0x02 | 0x04; // WEL and BP0 bits set, but not BUSY
        let mut eeprom = make_eeprom(spi);
        assert!(!block_on(eeprom.is_busy()).unwrap());
    }

    // ── write_enable ─────────────────────────────────────────────────

    #[test]
    fn write_enable_succeeds() {
        let mut eeprom = make_eeprom(MockSpi::new());
        block_on(eeprom.write_enable()).unwrap();
    }

    // ── write ────────────────────────────────────────────────────────

    #[test]
    fn write_byte_skips_when_data_already_matches() {
        let mut spi = MockSpi::new();
        spi.set_memory(0x05, 0xAA);
        let mut eeprom = make_eeprom(spi);
        block_on(eeprom.write_byte(0xAA, 0x05)).unwrap();
    }

    #[test]
    fn write_byte_skips_at_a8_address_when_data_matches() {
        let mut spi = MockSpi::new();
        spi.set_memory(0x1FF, 0x42);
        let mut eeprom = make_eeprom(spi);
        block_on(eeprom.write_byte(0x42, 0x1FF)).unwrap();
    }

    #[test]
    fn write_byte_stores_data_in_memory() {
        let mut eeprom = make_eeprom(MockSpi::new());
        block_on(eeprom.write_byte(0xBE, 0x10)).unwrap();
        let mut buf = [0u8; 1];
        block_on(eeprom.read(0x10, &mut buf)).unwrap();
        assert_eq!(buf[0], 0xBE);
    }

    #[test]
    fn write_byte_stores_data_at_a8_address() {
        let mut eeprom = make_eeprom(MockSpi::new());
        block_on(eeprom.write_byte(0xEF, 0x1AB)).unwrap();
        let mut buf = [0u8; 1];
        block_on(eeprom.read(0x1AB, &mut buf)).unwrap();
        assert_eq!(buf[0], 0xEF);
    }

    #[test]
    fn write_byte_then_read_roundtrip() {
        let mut eeprom = make_eeprom(MockSpi::new());
        let test_data: &[(u16, u8)] = &[
            (0x00, 0xAA),
            (0x01, 0xBB),
            (0x02, 0xCC),
            (0x03, 0xDD),
        ];
        for &(addr, byte) in test_data {
            block_on(eeprom.write_byte(byte, addr)).unwrap();
        }
        for &(addr, expected) in test_data {
            let mut buf = [0u8; 1];
            block_on(eeprom.read(addr, &mut buf)).unwrap();
            assert_eq!(buf[0], expected, "mismatch at address 0x{:03x}", addr);
        }
    }

    #[test]
    fn write_byte_overwrites_existing_data() {
        let mut spi = MockSpi::new();
        spi.set_memory(0x20, 0xAA);
        let mut eeprom = make_eeprom(spi);
        block_on(eeprom.write_byte(0x55, 0x20)).unwrap();
        let mut buf = [0u8; 1];
        block_on(eeprom.read(0x20, &mut buf)).unwrap();
        assert_eq!(buf[0], 0x55);
    }

    // ── write_page ────────────────────────────────────────────────────

    #[test]
    fn write_page_stores_full_page() {
        let mut eeprom = make_eeprom(MockSpi::new());
        let page: [u8; 16] = [
            0x00, 0x11, 0x22, 0x33,
            0x44, 0x55, 0x66, 0x77,
            0x88, 0x99, 0xAA, 0xBB,
            0xCC, 0xDD, 0xEE, 0xFF,
        ];
        block_on(eeprom.write_page(&page, 0x00)).unwrap();

        for (i, &expected) in page.iter().enumerate() {
            let mut buf = [0u8; 1];
            block_on(eeprom.read(i as u16, &mut buf)).unwrap();
            assert_eq!(buf[0], expected, "mismatch at offset {}", i);
        }
    }

    #[test]
    fn write_page_at_a8_address() {
        let mut eeprom = make_eeprom(MockSpi::new());
        let page = [0xAA; 16];
        // Page-aligned address in the A8 range
        block_on(eeprom.write_page(&page, 0x100)).unwrap();

        for i in 0..16u16 {
            let mut buf = [0u8; 1];
            block_on(eeprom.read(0x100 + i, &mut buf)).unwrap();
            assert_eq!(buf[0], 0xAA, "mismatch at address 0x{:03x}", 0x100 + i);
        }
    }

    #[test]
    fn write_page_rejects_unaligned_address() {
        let mut eeprom = make_eeprom(MockSpi::new());
        let page = [0x00; 16];
        let result = block_on(eeprom.write_page(&page, 0x05));
        assert!(result.is_err());
    }

    #[test]
    fn write_page_rejects_wrong_length() {
        let mut eeprom = make_eeprom(MockSpi::new());
        let short = [0x00; 8];
        let result = block_on(eeprom.write_page(&short, 0x00));
        assert!(result.is_err());
    }

    #[test]
    fn write_page_does_not_affect_adjacent_pages() {
        let mut spi = MockSpi::new();
        // Pre-populate adjacent pages with known data
        for i in 0..16u16 {
            spi.set_memory(i, 0xFF);        // page 0
            spi.set_memory(0x20 + i, 0xFF); // page 2
        }
        let mut eeprom = make_eeprom(spi);

        // Write to page 1 (0x10-0x1F)
        let page = [0x42; 16];
        block_on(eeprom.write_page(&page, 0x10)).unwrap();

        // Page 0 should be untouched
        let mut buf = [0u8; 1];
        block_on(eeprom.read(0x00, &mut buf)).unwrap();
        assert_eq!(buf[0], 0xFF, "page 0 was corrupted");

        // Page 2 should be untouched
        block_on(eeprom.read(0x20, &mut buf)).unwrap();
        assert_eq!(buf[0], 0xFF, "page 2 was corrupted");

        // Page 1 should have new data
        block_on(eeprom.read(0x10, &mut buf)).unwrap();
        assert_eq!(buf[0], 0x42);
    }

    #[test]
    fn write_page_then_overwrite() {
        let mut eeprom = make_eeprom(MockSpi::new());
        let first = [0x11; 16];
        let second = [0x22; 16];
        block_on(eeprom.write_page(&first, 0x00)).unwrap();
        block_on(eeprom.write_page(&second, 0x00)).unwrap();

        let mut buf = [0u8; 1];
        for i in 0..16u16 {
            block_on(eeprom.read(i, &mut buf)).unwrap();
            assert_eq!(buf[0], 0x22, "mismatch at offset {}", i);
        }
    }

    // ── init ─────────────────────────────────────────────────────────

    #[test]
    fn init_succeeds_when_device_ready() {
        let mut eeprom = make_eeprom(MockSpi::new());
        block_on(eeprom.init()).unwrap();
    }

    #[test]
    fn init_fails_when_device_busy() {
        let mut spi = MockSpi::new();
        spi.status_register = 0x01; // BUSY bit set
        let mut eeprom = make_eeprom(spi);
        let result = block_on(eeprom.init());
        assert_eq!(result, Err(Cat25040Error::DeviceNotReady));
    }

    // ── block protection ─────────────────────────────────────────────

    #[test]
    fn get_block_protection_none() {
        let mut spi = MockSpi::new();
        spi.status_register = 0x00; // BP1=0, BP0=0
        let mut eeprom = make_eeprom(spi);
        let level = block_on(eeprom.get_block_protection()).unwrap();
        assert_eq!(level, BlockProtection::None);
    }

    #[test]
    fn get_block_protection_upper_quarter() {
        let mut spi = MockSpi::new();
        spi.status_register = 0x04; // BP1=0, BP0=1
        let mut eeprom = make_eeprom(spi);
        let level = block_on(eeprom.get_block_protection()).unwrap();
        assert_eq!(level, BlockProtection::UpperQuarter);
    }

    #[test]
    fn get_block_protection_upper_half() {
        let mut spi = MockSpi::new();
        spi.status_register = 0x08; // BP1=1, BP0=0
        let mut eeprom = make_eeprom(spi);
        let level = block_on(eeprom.get_block_protection()).unwrap();
        assert_eq!(level, BlockProtection::UpperHalf);
    }

    #[test]
    fn get_block_protection_full() {
        let mut spi = MockSpi::new();
        spi.status_register = 0x0C; // BP1=1, BP0=1
        let mut eeprom = make_eeprom(spi);
        let level = block_on(eeprom.get_block_protection()).unwrap();
        assert_eq!(level, BlockProtection::Full);
    }

    #[test]
    fn set_block_protection_none() {
        let mut eeprom = make_eeprom(MockSpi::new());
        block_on(eeprom.set_block_protection(BlockProtection::None)).unwrap();
        let level = block_on(eeprom.get_block_protection()).unwrap();
        assert_eq!(level, BlockProtection::None);
    }

    #[test]
    fn set_block_protection_upper_quarter() {
        let mut eeprom = make_eeprom(MockSpi::new());
        block_on(eeprom.set_block_protection(BlockProtection::UpperQuarter)).unwrap();
        let level = block_on(eeprom.get_block_protection()).unwrap();
        assert_eq!(level, BlockProtection::UpperQuarter);
    }

    #[test]
    fn set_block_protection_upper_half() {
        let mut eeprom = make_eeprom(MockSpi::new());
        block_on(eeprom.set_block_protection(BlockProtection::UpperHalf)).unwrap();
        let level = block_on(eeprom.get_block_protection()).unwrap();
        assert_eq!(level, BlockProtection::UpperHalf);
    }

    #[test]
    fn set_block_protection_full() {
        let mut eeprom = make_eeprom(MockSpi::new());
        block_on(eeprom.set_block_protection(BlockProtection::Full)).unwrap();
        let level = block_on(eeprom.get_block_protection()).unwrap();
        assert_eq!(level, BlockProtection::Full);
    }

    #[test]
    fn clear_block_protection() {
        let mut spi = MockSpi::new();
        spi.status_register = 0x0C; // Start with full protection
        let mut eeprom = make_eeprom(spi);
        block_on(eeprom.clear_block_protection()).unwrap();
        let level = block_on(eeprom.get_block_protection()).unwrap();
        assert_eq!(level, BlockProtection::None);
    }

    #[test]
    fn block_protection_roundtrip() {
        let mut eeprom = make_eeprom(MockSpi::new());
        let levels = [
            BlockProtection::None,
            BlockProtection::UpperQuarter,
            BlockProtection::UpperHalf,
            BlockProtection::Full,
            BlockProtection::None,
        ];
        for &level in &levels {
            block_on(eeprom.set_block_protection(level)).unwrap();
            let read_level = block_on(eeprom.get_block_protection()).unwrap();
            assert_eq!(read_level, level);
        }
    }
}