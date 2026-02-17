//! Register interface implementation for CAT25040 SPI communication.
//!
//! This module bridges the device-driver crate's RegisterInterface traits
//! with the CAT25040's SPI protocol.

use crate::{Cat25040Error, Operation, Spi};
use device_driver::{AsyncCommandInterface, AsyncRegisterInterface, CommandInterface, RegisterInterface};

/// Wrapper that implements device-driver's RegisterInterface for our SPI trait.
pub struct SpiRegisterInterface<S: Spi> {
    spi: S,
}

impl<S: Spi> SpiRegisterInterface<S> {
    /// Create a new SPI register interface wrapper.
    pub fn new(spi: S) -> Self {
        Self { spi }
    }

    /// Unwrap and return the underlying SPI interface.
    pub fn into_inner(self) -> S {
        self.spi
    }

    /// Get a mutable reference to the underlying SPI interface.
    pub fn spi_mut(&mut self) -> &mut S {
        &mut self.spi
    }
}

impl<S: Spi> RegisterInterface for SpiRegisterInterface<S> {
    type AddressType = u8;
    type Error = Cat25040Error;

    fn read_register(
        &mut self,
        _address: Self::AddressType,
        _size_bits: u32,
        _data: &mut [u8],
    ) -> Result<(), Self::Error> {
        // For CAT25040, the address IS the opcode for register reads (e.g., 0x05 for status)
        // We need to use async in a sync context, which isn't directly possible
        // This is a limitation - we'll implement the async version primarily
        unimplemented!("Use AsyncRegisterInterface instead for async operations")
    }

    fn write_register(
        &mut self,
        _address: Self::AddressType,
        _size_bits: u32,
        _data: &[u8],
    ) -> Result<(), Self::Error> {
        unimplemented!("Use AsyncRegisterInterface instead for async operations")
    }
}

impl<S: Spi> AsyncRegisterInterface for SpiRegisterInterface<S> {
    type AddressType = u8;
    type Error = Cat25040Error;

    async fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        // For CAT25040, address is the opcode (e.g., 0x05 for Read Status)
        self.spi
            .transaction(&mut [Operation::Write(&[address]), Operation::Read(data)])
            .await
    }

    async fn write_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        // For CAT25040, address is the opcode (e.g., 0x01 for Write Status)
        let mut buf = [0u8; 9]; // Max: 1 byte opcode + 8 bytes data
        buf[0] = address;
        let len = 1 + data.len();
        buf[1..len].copy_from_slice(data);

        self.spi
            .transaction(&mut [Operation::Write(&buf[..len])])
            .await
    }
}

impl<S: Spi> CommandInterface for SpiRegisterInterface<S> {
    type AddressType = u8;
    type Error = Cat25040Error;

    fn dispatch_command(
        &mut self,
        _address: Self::AddressType,
        _size_bits_in: u32,
        _input: &[u8],
        _size_bits_out: u32,
        _output: &mut [u8],
    ) -> Result<(), Self::Error> {
        unimplemented!("Use AsyncCommandInterface instead for async operations")
    }
}

impl<S: Spi> AsyncCommandInterface for SpiRegisterInterface<S> {
    type AddressType = u8;
    type Error = Cat25040Error;

    async fn dispatch_command(
        &mut self,
        address: Self::AddressType,
        _size_bits_in: u32,
        input: &[u8],
        _size_bits_out: u32,
        output: &mut [u8],
    ) -> Result<(), Self::Error> {
        // For CAT25040 commands:
        // - Commands like WREN/WRDI have no input/output (empty slices)
        // - Just send the opcode
        if input.is_empty() && output.is_empty() {
            // Simple command with no data
            self.spi
                .transaction(&mut [Operation::Write(&[address])])
                .await
        } else {
            // Commands with input/output would go here if needed
            unimplemented!("Commands with input/output not yet supported")
        }
    }
}
