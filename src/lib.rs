//! A `no_std`, async Rust driver for the ON Semiconductor CAT25040 4-Kbit SPI EEPROM,
//! built on [`embedded-hal-async`](https://crates.io/crates/embedded-hal-async) traits.
//!
//! Works with any platform that implements
//! [`SpiDevice`](embedded_hal_async::spi::SpiDevice) and
//! [`DelayNs`](embedded_hal_async::delay::DelayNs) -- no framework lock-in.
//!
//! # Features
//!
//! - Byte read/write with automatic write-only-if-changed optimization
//! - Page write (16 bytes per page, single write cycle)
//! - Sequential read of arbitrary length
//! - 9-bit addressing -- handles the A8 bit encoding in opcodes automatically
//! - Busy polling after writes
//! - Optional [`defmt`](https://crates.io/crates/defmt) logging via the `defmt` feature
//!
//! # Example
//!
//! ```rust,no_run
//! # async fn example(spi_device: impl embedded_hal_async::spi::SpiDevice,
//! #                  delay: impl embedded_hal_async::delay::DelayNs) {
//! use cat25040::Cat25040;
//!
//! let mut eeprom = Cat25040::new(spi_device, delay);
//!
//! // Read 16 bytes starting at address 0x00
//! let mut buf = [0u8; 16];
//! eeprom.read(0x00, &mut buf).await.unwrap();
//!
//! // Write a single byte (skips if value already matches)
//! eeprom.write_byte(0xAB, 0x10).await.unwrap();
//!
//! // Write a full 16-byte page (address must be page-aligned)
//! let page = *b"ION-C EEPROM OK!";
//! eeprom.write_page(&page, 0x00).await.unwrap();
//! # }
//! ```
//!
//! # Device Compatibility
//!
//! Designed for the CAT25040 but should work with other CAT250xx family EEPROMs
//! that use the same SPI command set and 9-bit addressing (e.g., CAT25020).

#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(feature = "defmt")]
use defmt::{debug, info};

use core::fmt;
use core::fmt::Display;
use embedded_hal_async::spi::SpiDevice;
use embedded_hal_async::spi::Operation;
use embedded_hal_async::delay::DelayNs;

const WREN_OPCODE: u8 = 0x06;
const WRDI_OPCODE: u8 = 0x04;
const RDSR_OPCODE: u8 = 0x05;
const WRSR_OPCODE: u8 = 0x01;
const READ_OPCODE: u8 = 0x03;
const WRITE_OPCODE: u8 = 0x02;

const STATUS_REGISTER_BUSY: u8 = 1 << 0;  
const STATUS_REGISTER_WRITE_ENABLE_LATCH: u8 = 1 << 1;
const STATUS_REGISTER_BLOCK_PROTECT_0: u8 = 1 << 2;
const STATUS_REGISTER_BLOCK_PROTECT_1: u8 = 1 << 3;

const BUSY_WAIT_TIME_MS: u32 = 1;
const WRITE_CYCLE_TIME_MS: u32 = 5;
const PAGE_SIZE: u8 = 16;

/// CAT25040 EEPROM driver, generic over any SPI device and delay provider.
pub struct Cat25040<SPI: SpiDevice, D: DelayNs> {
    spi: SPI,
    delay: D,
}

#[derive(Debug)]
pub enum Cat25040Error {
    Spi,
    InvalidAddress,
    InvalidLength,
}

impl Display for Cat25040Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Cat25040Error: {:?}", self)
    }
}

impl<SPI: SpiDevice, D: DelayNs> Cat25040<SPI, D> {
    pub fn new(spi: SPI, delay: D) -> Self {
        Self { spi, delay }
    }

    pub async fn get_status_register(&mut self, status_buf: &mut [u8]) -> Result<(), Cat25040Error> {
        self.spi.transaction(
            &mut [
                Operation::Write(&mut [RDSR_OPCODE]),
                Operation::Read(status_buf)]).await.map_err(|_| Cat25040Error::Spi)?;
        Ok(())
    }

    pub async fn is_busy(&mut self) -> Result<bool, Cat25040Error> {
        let mut status_buf  = [0u8; 1];
        self.get_status_register(&mut status_buf).await?;

        Ok(status_buf[0] & STATUS_REGISTER_BUSY != 0)
    }

    pub async fn write_enable(&mut self) -> Result<(), Cat25040Error> {
        self.spi.transaction(
            &mut [
                Operation::Write(&mut [WREN_OPCODE]),
            ]).await.map_err(|_| Cat25040Error::Spi)?;
        Ok(())
    }

    fn get_valid_opcode_for_address(address: u16, opcode: u8) -> u8 {
        // In CAT25020, if A8 is set, bit 3 of the opcode is set
        if address & (1 << 8) != 0 {
            opcode | (1 << 3)
        } else {
            opcode
        }
    }

    pub async fn read(&mut self, address: u16, rx_buf: &mut [u8]) -> Result<(), Cat25040Error> {        
        let opcode = Self::get_valid_opcode_for_address(address, READ_OPCODE);
        self.spi.transaction(
            &mut [
                Operation::Write(&mut [opcode, address as u8]),
                Operation::Read(rx_buf)]).await.map_err(|_| Cat25040Error::Spi)?;
        #[cfg(feature = "defmt")]
        debug!("Read 0x{:02x} from address 0x{:02x} with opcode 0x{:02x}", rx_buf[0], address, opcode);
        Ok(())
    }

    pub async fn write_byte(&mut self, data: u8, address: u16) -> Result<(), Cat25040Error> {
        let mut current_data = [0u8; 1];
        self.read(address, &mut current_data).await?;
        if current_data[0] == data {
            #[cfg(feature = "defmt")]
            debug!("Data at address 0x{:02x} is already 0x{:02x}, skipping write", address, data);
            return Ok(());
        }
        
        self.write_enable().await?;
        let opcode = Self::get_valid_opcode_for_address(address, WRITE_OPCODE);
        self.spi.transaction(
            &mut [
                Operation::Write(&mut [opcode, address as u8, data])]).await.map_err(|_| Cat25040Error::Spi)?;
        self.delay.delay_ms(WRITE_CYCLE_TIME_MS).await;
        while self.is_busy().await? {
            self.delay.delay_ms(BUSY_WAIT_TIME_MS).await;
        }
        #[cfg(feature = "defmt")]
        debug!("Wrote 0x{:02x} to address 0x{:02x} with opcode 0x{:02x}", data, address, opcode);
        Ok(())
    }

    pub async fn write_page(&mut self, data: &[u8], address: u16) -> Result<(), Cat25040Error> {
        // Checking alignment:
        if address % PAGE_SIZE as u16 != 0 {
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

        self.spi.transaction(
            &mut [
                Operation::Write(&mut cmd)]).await.map_err(|_| Cat25040Error::Spi)?;


        self.delay.delay_ms(WRITE_CYCLE_TIME_MS).await;
        while self.is_busy().await? {
            self.delay.delay_ms(BUSY_WAIT_TIME_MS).await;
        }
        #[cfg(feature = "defmt")]
        debug!("Wrote page to address 0x{:02x}", address);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use std::convert::Infallible;

    use super::*;
    use embedded_hal_async::spi::*;
    use embedded_hal_async::spi::Operation;
    use futures::executor::block_on;

    // ── Mock SPI device ──────────────────────────────────────────────

    #[derive(Debug)]
    struct MockSpiDevice {
        status_register: u8,
        write_enable: bool,
        memory: [u8; 512],
        read_data: u8,
    }

    impl MockSpiDevice {
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

        pub fn get_memory(&self, address: u16) -> u8 {
            self.memory[address as usize]
        }
    }

    impl ErrorType for MockSpiDevice {
        type Error = Infallible;
    }

    impl SpiDevice for MockSpiDevice {
        async fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
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
                                println!("MockSpiDevice: unhandled opcode 0x{:02x}", data[0]);
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

    // ── Mock delay (no-op, instant return) ───────────────────────────

    struct MockDelay;

    impl DelayNs for MockDelay {
        async fn delay_ns(&mut self, _ns: u32) {
            // No-op in tests
        }
    }

    /// Helper to create a Cat25040 with mock SPI and mock delay.
    fn make_eeprom(spi: MockSpiDevice) -> Cat25040<MockSpiDevice, MockDelay> {
        Cat25040::new(spi, MockDelay)
    }

    // ── get_valid_opcode_for_address ──────────────────────────────────

    type TestCat = Cat25040<MockSpiDevice, MockDelay>;

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
        let mut eeprom = make_eeprom(MockSpiDevice::new());
        let mut buf = [0xFFu8; 1];
        block_on(eeprom.read(0x0000, &mut buf)).unwrap();
        assert_eq!(buf[0], 0x00);
    }

    #[test]
    fn read_returns_prepopulated_data() {
        let mut spi = MockSpiDevice::new();
        spi.set_memory(0x10, 0xAB);
        let mut eeprom = make_eeprom(spi);
        let mut buf = [0u8; 1];
        block_on(eeprom.read(0x10, &mut buf)).unwrap();
        assert_eq!(buf[0], 0xAB);
    }

    #[test]
    fn read_with_a8_address() {
        let mut spi = MockSpiDevice::new();
        spi.set_memory(0x1FF, 0xCD);
        let mut eeprom = make_eeprom(spi);
        let mut buf = [0u8; 1];
        block_on(eeprom.read(0x1FF, &mut buf)).unwrap();
        assert_eq!(buf[0], 0xCD);
    }

    #[test]
    fn read_different_addresses_return_different_data() {
        let mut spi = MockSpiDevice::new();
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

    // ── get_status_register / is_busy ────────────────────────────────

    #[test]
    fn get_status_register_reads_mock_status() {
        let mut spi = MockSpiDevice::new();
        spi.status_register = 0x03;
        let mut eeprom = make_eeprom(spi);
        let mut status = [0u8; 1];
        block_on(eeprom.get_status_register(&mut status)).unwrap();
        assert_eq!(status[0], 0x03);
    }

    #[test]
    fn is_busy_returns_true_when_wip_set() {
        let mut spi = MockSpiDevice::new();
        spi.status_register = STATUS_REGISTER_BUSY;
        let mut eeprom = make_eeprom(spi);
        assert!(block_on(eeprom.is_busy()).unwrap());
    }

    #[test]
    fn is_busy_returns_false_when_wip_clear() {
        let mut eeprom = make_eeprom(MockSpiDevice::new());
        assert!(!block_on(eeprom.is_busy()).unwrap());
    }

    #[test]
    fn is_busy_ignores_non_wip_bits() {
        let mut spi = MockSpiDevice::new();
        spi.status_register = STATUS_REGISTER_WRITE_ENABLE_LATCH
            | STATUS_REGISTER_BLOCK_PROTECT_0;
        let mut eeprom = make_eeprom(spi);
        assert!(!block_on(eeprom.is_busy()).unwrap());
    }

    // ── write_enable ─────────────────────────────────────────────────

    #[test]
    fn write_enable_succeeds() {
        let mut eeprom = make_eeprom(MockSpiDevice::new());
        block_on(eeprom.write_enable()).unwrap();
    }

    // ── write ────────────────────────────────────────────────────────

    #[test]
    fn write_byte_skips_when_data_already_matches() {
        let mut spi = MockSpiDevice::new();
        spi.set_memory(0x05, 0xAA);
        let mut eeprom = make_eeprom(spi);
        block_on(eeprom.write_byte(0xAA, 0x05)).unwrap();
    }

    #[test]
    fn write_byte_skips_at_a8_address_when_data_matches() {
        let mut spi = MockSpiDevice::new();
        spi.set_memory(0x1FF, 0x42);
        let mut eeprom = make_eeprom(spi);
        block_on(eeprom.write_byte(0x42, 0x1FF)).unwrap();
    }

    #[test]
    fn write_byte_stores_data_in_memory() {
        let mut eeprom = make_eeprom(MockSpiDevice::new());
        block_on(eeprom.write_byte(0xBE, 0x10)).unwrap();
        let mut buf = [0u8; 1];
        block_on(eeprom.read(0x10, &mut buf)).unwrap();
        assert_eq!(buf[0], 0xBE);
    }

    #[test]
    fn write_byte_stores_data_at_a8_address() {
        let mut eeprom = make_eeprom(MockSpiDevice::new());
        block_on(eeprom.write_byte(0xEF, 0x1AB)).unwrap();
        let mut buf = [0u8; 1];
        block_on(eeprom.read(0x1AB, &mut buf)).unwrap();
        assert_eq!(buf[0], 0xEF);
    }

    #[test]
    fn write_byte_then_read_roundtrip() {
        let mut eeprom = make_eeprom(MockSpiDevice::new());
        let test_data: &[(u16, u8)] = &[
            (0x00, 0x49),  // 'I'
            (0x01, 0x4F),  // 'O'
            (0x02, 0x4E),  // 'N'
            (0x03, 0x21),  // '!'
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
        let mut spi = MockSpiDevice::new();
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
        let mut eeprom = make_eeprom(MockSpiDevice::new());
        let page: [u8; 16] = [
            0x49, 0x4F, 0x4E, 0x21, // "ION!"
            0xDE, 0xAD, 0xBE, 0xEF,
            0xCA, 0xFE, 0xBA, 0xBE,
            0x00, 0x11, 0x22, 0x33,
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
        let mut eeprom = make_eeprom(MockSpiDevice::new());
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
        let mut eeprom = make_eeprom(MockSpiDevice::new());
        let page = [0x00; 16];
        let result = block_on(eeprom.write_page(&page, 0x05));
        assert!(result.is_err());
    }

    #[test]
    fn write_page_rejects_wrong_length() {
        let mut eeprom = make_eeprom(MockSpiDevice::new());
        let short = [0x00; 8];
        let result = block_on(eeprom.write_page(&short, 0x00));
        assert!(result.is_err());
    }

    #[test]
    fn write_page_does_not_affect_adjacent_pages() {
        let mut spi = MockSpiDevice::new();
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
        let mut eeprom = make_eeprom(MockSpiDevice::new());
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
}