# cat25040

A `no_std`, async Rust driver for the **ON Semiconductor CAT25040** (4-Kbit SPI EEPROM), built on [`embedded-hal-async`](https://crates.io/crates/embedded-hal-async) traits.

Works with any platform that implements `embedded_hal_async::spi::SpiDevice` and `embedded_hal_async::delay::DelayNs` — no framework lock-in.

## Features

- **Byte read/write** with automatic write-only-if-changed optimization
- **Page write** (16 bytes per page, single write cycle)
- **Sequential read** of arbitrary length
- **9-bit addressing** — handles the A8 bit encoding in opcodes automatically
- **Busy polling** after writes with configurable delay
- Optional `defmt` logging (enable the `defmt` feature)

## Usage

```rust
use cat25040::Cat25040;

// Any SpiDevice + DelayNs implementation works.
// Example with embassy:
let mut eeprom = Cat25040::new(spi_device, embassy_time::Delay);

// Read 16 bytes starting at address 0x00
let mut buf = [0u8; 16];
eeprom.read(0x00, &mut buf).await.unwrap();

// Write a single byte (skips if value already matches)
eeprom.write_byte(0xAB, 0x10).await.unwrap();

// Write a full 16-byte page (address must be page-aligned)
let page = *b"EEPROM OK!";
eeprom.write_page(&page, 0x00).await.unwrap();
```

## API

| Method | Description |
|---|---|
| `new(spi, delay)` | Create a new driver instance |
| `read(address, buf)` | Read `buf.len()` bytes starting at `address` |
| `write_byte(data, address)` | Write a single byte (skips if unchanged) |
| `write_page(data, address)` | Write exactly 16 bytes to a page-aligned address |
| `get_status_register(buf)` | Read the EEPROM status register |
| `is_busy()` | Check if a write cycle is in progress (WIP bit) |
| `write_enable()` | Send the WREN command |

## Cargo Features

- **`std`** (default) — enables standard library support for testing
- **`defmt`** — enables debug logging via `defmt`

## Device Compatibility

Designed for the CAT25040 but should work with other CAT250xx family EEPROMs that use the same SPI command set and 9-bit addressing (e.g., CAT25020). Adjust `PAGE_SIZE` if your device differs.

## Testing

```sh
cargo test
```

Tests use a mock `SpiDevice` and mock `DelayNs` — no hardware required.

## License

See the repository root for license information.
