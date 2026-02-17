# cat25040

A `no_std`, async Rust driver for the **ON Semiconductor CAT25040** (4-Kbit SPI EEPROM), built on [`embedded-hal-async`](https://crates.io/crates/embedded-hal-async) traits.

Works with any platform that implements the driver's `Spi` and `HardwareInterface` traits — no framework lock-in.

## Features

- **Byte read/write** with automatic write-only-if-changed optimization
- **Page write** (16 bytes per page, single write cycle)
- **Sequential read** of arbitrary length
- **9-bit addressing** — handles the A8 bit encoding in opcodes automatically
- **Busy polling** after writes with configurable delay
- **Software block protection** — protect memory regions (quarter, half, or full array)
- **Power-up initialization** — proper timing delays per datasheet (tPUW = 1ms)
- **Type-safe register access** — uses [`device-driver`](https://crates.io/crates/device-driver) crate for compile-time safe register operations
- **YAML-defined device model** — register and command definitions in declarative YAML
- Optional `defmt` logging (enable the `defmt` feature)

## Usage

```rust
use cat25040::{Cat25040, BlockProtection};

// Provide your SPI and delay implementations via custom traits
let mut eeprom = Cat25040::new(spi, hardware_interface);

// Initialize after power-on (waits required tPUW = 1ms)
eeprom.init().await.unwrap();

// Read 16 bytes starting at address 0x00
let mut buf = [0u8; 16];
eeprom.read(0x00, &mut buf).await.unwrap();

// Write a single byte (skips if value already matches)
eeprom.write_byte(0xAB, 0x10).await.unwrap();

// Write a full 16-byte page (address must be page-aligned)
let page = *b"EEPROM data here";
eeprom.write_page(&page, 0x00).await.unwrap();

// Type-safe status register access
let status = eeprom.read_status().await.unwrap();
if status.busy() {
    // Write in progress
}
if status.wel() {
    // Write enable latch is set
}

// Protect upper half of memory (addresses 0x100-0x1FF)
eeprom.set_block_protection(BlockProtection::UpperHalf).await.unwrap();

// Check current protection level
let level = eeprom.get_block_protection().await.unwrap();

// Clear all protection
eeprom.clear_block_protection().await.unwrap();

// Send write enable/disable commands
eeprom.write_enable().await.unwrap();
eeprom.write_disable().await.unwrap();
```

## API

| Method | Description |
|---|---|
| `new(spi, hardware_interface)` | Create a new driver instance |
| `init()` | Initialize device after power-on (waits tPUW = 1ms) |
| `read(address, buf)` | Read `buf.len()` bytes starting at `address` |
| `write_byte(data, address)` | Write a single byte (skips if unchanged) |
| `write_page(data, address)` | Write exactly 16 bytes to a page-aligned address |
| `read_status()` | Read the EEPROM status register (returns type-safe `Status`) |
| `is_busy()` | Check if a write cycle is in progress (WIP bit) |
| `write_enable()` | Send the WREN command (uses device-driver) |
| `write_disable()` | Send the WRDI command (uses device-driver) |
| `get_block_protection()` | Get current block protection level |
| `set_block_protection(level)` | Set block protection (None, UpperQuarter, UpperHalf, Full) |
| `clear_block_protection()` | Clear all block protection (convenience method) |
| `device()` | Access underlying device-driver device for advanced usage |

## Architecture

### Device-Driver Integration

This driver uses the [`device-driver`](https://crates.io/crates/device-driver) crate for type-safe register and command operations. Key components:

**YAML Device Model** (`src/cat25040.yaml`):
- Defines registers (STATUS_REG) with named fields (BUSY, WEL, BP0, BP1)
- Defines commands (WREN, WRDI, WRSR) with opcodes
- Auto-generates type-safe Rust code at build time

**SpiRegisterInterface** (`src/register_interface.rs`):
- Bridges the driver's `Spi` trait with device-driver's `RegisterInterface` and `CommandInterface` traits
- Handles opcode-based register access specific to CAT25040
- Enables compile-time verification of register operations

**Generated Device API**:
- `Cat25040Device` provides the low-level device API
- `Status` (alias for `StatusReg`) provides type-safe field access
- Commands like `wren()` and `wrdi()` are generated from YAML

### Custom Traits

The driver defines its own traits to avoid framework lock-in:

**`Spi` trait** - Wraps `embedded_hal_async::spi::SpiDevice`:
```rust
pub trait Spi {
    async fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Cat25040Error>;
}
```

**`HardwareInterface` trait** - Provides delay functionality:
```rust
pub trait HardwareInterface {
    async fn wait_ms(&mut self, timeout_ms: u32);
}
```

These traits can be easily implemented for any platform. See the `spi_device` module for an example implementation using `embedded-hal-async` directly.

## Block Protection

The CAT25040 supports software-controlled write protection for different memory regions using the BP0 and BP1 bits in the status register:

| Level | Protected Addresses | Protected Size | Description |
|-------|---------------------|----------------|-------------|
| `None` | None | 0 bytes | No protection (BP1=0, BP0=0) |
| `UpperQuarter` | 0x180-0x1FF | 128 bytes | Upper 1/4 protected (BP1=0, BP0=1) |
| `UpperHalf` | 0x100-0x1FF | 256 bytes | Upper 1/2 protected (BP1=1, BP0=0) |
| `Full` | 0x000-0x1FF | 512 bytes | Entire array protected (BP1=1, BP0=1) |

**Hardware Override**: The WP (Write Protect) pin can override software protection. When WP is low, all writes are inhibited regardless of BP0/BP1 settings. When not using the WP pin, tie it to VCC.

**Use Cases**:
- Protect calibration data, configuration, or factory settings from accidental overwrites
- Reserve upper memory for read-only parameters while allowing writes to lower addresses
- Implement multi-level security by combining hardware (WP pin) and software protection

## Cargo Features

- **`std`** (default) — enables standard library support for testing
- **`defmt`** — enables debug logging via `defmt`

## Hardware Pins

The CAT25040 has several hardware control pins that affect driver behavior:

- **CS (Chip Select)**: Managed by your SPI implementation. CS must go high to start internal write cycles.
- **WP (Write Protect)**: Hardware write protection. When low, all writes are inhibited regardless of software BP0/BP1 settings. Tie to VCC when not used.
- **HOLD**: Pauses SPI communication when taken low (while SCK is low). Allows the master to service higher-priority interrupts. Tie to VCC when not used.
- **SCK, SI, SO**: Standard SPI signals (clock, data in, data out).

**Important**: This driver does not directly control WP or HOLD pins. These are hardware features that should be managed by your platform's GPIO implementation if needed.

## Power-On Timing

After power-on, the device requires up to 1ms before it's ready for operations (tPUW per datasheet). Always call `init()` after creating the driver instance:

```rust
let mut eeprom = Cat25040::new(spi, hardware_interface);
eeprom.init().await.unwrap(); // Waits 1ms and verifies device is ready
```

## Implementing for Your Platform

To use this driver, implement the `Spi` and `HardwareInterface` traits for your platform:

```rust
use cat25040::{Cat25040, Spi, HardwareInterface, Operation, Cat25040Error};
use embedded_hal_async::spi::SpiDevice;
use embedded_hal_async::delay::DelayNs;

// Wrapper for SpiDevice
struct MySpi<S: SpiDevice> {
    spi: S,
}

impl<S: SpiDevice> Spi for MySpi<S> {
    async fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Cat25040Error> {
        self.spi.transaction(operations).await.map_err(|_| Cat25040Error::Spi)
    }
}

// Wrapper for DelayNs
struct MyHardwareInterface<D: DelayNs> {
    delay: D,
}

impl<D: DelayNs> HardwareInterface for MyHardwareInterface<D> {
    async fn wait_ms(&mut self, timeout_ms: u32) {
        self.delay.delay_ms(timeout_ms).await;
    }
}

// Use with your platform (e.g., embassy)
let spi = MySpi { spi: spi_device };
let hw = MyHardwareInterface { delay: embassy_time::Delay };
let mut eeprom = Cat25040::new(spi, hw);
```

The `spi_device` module in this crate provides a ready-to-use implementation for platforms using `embedded-hal-async`.

## Device Compatibility

Designed for the CAT25040 but should work with other CAT250xx family EEPROMs that use the same SPI command set and 9-bit addressing (e.g., CAT25020). The driver handles the A8 address bit encoding automatically by setting bit 3 of the opcode when addressing the upper 256 bytes.

## Project Structure

```
src/
├── lib.rs                    # Main driver implementation
├── cat25040.yaml             # Device model (registers, commands)
├── register_interface.rs     # SpiRegisterInterface implementation
└── spi_device.rs             # Example SPI trait implementations
build.rs                      # Code generation from YAML
```

The build script uses `device-driver-generation` to generate Rust code from the YAML manifest. Generated code is placed in `target/<profile>/build/cat25040-*/out/cat25040_device.rs`.

## Testing

```sh
cargo test
```

The test suite includes 26 tests covering:
- 9-bit address encoding (A8 bit in opcode)
- Read/write operations at various addresses
- Page writes with alignment validation
- Status register access with type-safe fields
- Block protection configuration
- Power-up initialization

Tests use mock `Spi` and `HardwareInterface` implementations — no hardware required.

## License

See the repository root for license information.
