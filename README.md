# embassy-bmp280

[![Crates.io](https://img.shields.io/crates/v/embassy-bmp280.svg)](https://crates.io/crates/embassy-bmp280)
[![Documentation](https://img.shields.io/docsrs/embassy-bmp280/latest.svg)](https://docs.rs/embassy-bmp280)
[![License](https://img.shields.io/crates/l/embassy-bmp280.svg)](LICENSE)

Async `no_std` driver for the Bosch BMP280 pressure and temperature sensor.

The project follows the same philosophy as [`embassy-gy-bmi160`](https://crates.io/crates/embassy-gy-bmi160):
native async operation, no heap allocation, no `unsafe`, and support for shared I2C buses.

---

## Version 0.3.0

- Added RP2350 RISC-V/Hazard3 backend support.
- Embassy dependencies are now selected through explicit upstream or RISC-V features.
- Added an explicit Embassy time backend and sync backend for each supported architecture.

### Dependencies

For ARM/Cortex-M targets:

```toml
[dependencies]
embassy-bmp280 = "0.3"
embedded-hal-async = "1.0"
embassy-sync = "0.8"
embassy-time = "0.5"
```

For the RP2350 RISC-V/Hazard3 target:

```toml
[dependencies]
embassy-bmp280 = { version = "0.3", default-features = false, features = ["time-riscv", "sync-riscv"] }
embedded-hal-async = "1.0"
embassy-sync-riscv = "0.1"
embassy-time-riscv = "0.1"
embassy-time-driver-riscv = "0.1"
```

---

## Features

- `#![forbid(unsafe_code)]`: memory safety enforced at compile time
- Native async operation through `embedded-hal-async`
- Software reset and chip-ID verification at startup
- Factory OTP calibration read and application
- Integer-only temperature and pressure compensation
- Typed error handling with `Bmp280Error<E>`
- Compatible with `embassy-embedded-hal` shared I2C buses
- Optional `float` feature for `f32` conversions
- ARM/Cortex-M and RP2350 RISC-V/Hazard3 Embassy backends

---

## Installation

### ARM/Cortex-M

```toml
[dependencies]
embassy-bmp280 = "0.3"
embassy-sync = "0.8"
embassy-time = "0.5"
embedded-hal-async = "1.0"
```

### RP2350 RISC-V/Hazard3

```toml
[dependencies]
embassy-bmp280 = { version = "0.3", default-features = false, features = ["time-riscv", "sync-riscv"] }
embassy-sync-riscv = "0.1"
embassy-time-riscv = "0.1"
embassy-time-driver-riscv = "0.1"
embedded-hal-async = "1.0"
```

To enable `f32` conversion methods:

```toml
embassy-bmp280 = { version = "0.3", features = ["float"] }
```

For RISC-V with floating-point conversions:

```toml
embassy-bmp280 = {
    version = "0.3",
    default-features = false,
    features = ["time-riscv", "sync-riscv", "float"]
}
```

---

## Wiring (Pico 2 / RP2350)

| BMP280 pin | Connection | Notes |
|------------|------------|-------|
| VIN | 3.3 V or 5 V | Check the voltage markings on the module |
| GND | Common ground | |
| SCL | Example: GP5 | Pull-up is often included on the module |
| SDA | Example: GP4 | Pull-up is often included on the module |

---

## Usage

### Initialization

```rust
use embassy_bmp280::{Bmp280, Bmp280Address, Bmp280Config, PowerMode};

// Default address
let mut bmp =
    Bmp280::new(i2c_device, Bmp280Address::Default, Bmp280Config::default()).await?;
```

`Bmp280::new` performs the following steps:

1. Software reset of the sensor, followed by the 3 ms datasheet delay
2. Chip-ID verification (`0x58` or `0x60`)
3. Reading of 24 bytes of OTP calibration data
4. Configuration of oversampling and operating mode

### Reading measurements

```rust
let data = bmp.read().await?;

// Temperature in centidegrees Celsius: 2315 = 23.15 °C
let temp_cdeg = data.temperature_cdeg;

// Pressure in Pascals × 256 (Q24.8 format)
let press_raw = data.pressure_pa256;
let press_pa = press_raw / 256; // Pascals
let press_hpa = press_raw as f32 / (256.0 * 100.0); // hPa
```

### With the `float` feature

```rust
// Enable features = ["float"] in Cargo.toml
let temp_c = data.temperature_celsius(); // f32, for example 23.15
let press_hpa = data.pressure_hpa(); // f32, for example 1013.25
```

### Custom configuration

```rust
use embassy_bmp280::{Bmp280Config, OversamplingTemp, OversamplingPress, PowerMode};

let config = Bmp280Config {
    temp_os: OversamplingTemp::X4,
    press_os: OversamplingPress::X4,
    mode: PowerMode::Normal,
};

let mut bmp =
    Bmp280::new(i2c_device, Bmp280Address::Default, config).await?;
```

| `OversamplingTemp` / `OversamplingPress` | Resolution | Typical current |
|------------------------------------------|------------|-----------------|
| `Skip` | Disabled | — |
| `X1` | 16-bit | Minimum |
| `X2` | 17-bit | |
| `X4` | 18-bit | |
| `X8` | 19-bit | |
| `X16` | 20-bit | Maximum |

### Alternative address

```rust
let mut bmp =
    Bmp280::new(i2c, Bmp280Address::Secondary, Bmp280Config::default()).await?;

// Or change it dynamically after rediscovery
bmp.set_address(Bmp280Address::Custom(0x77));
```

### Sleep mode

```rust
// Save power between acquisitions
bmp.set_mode(PowerMode::Sleep).await?;

// Resume without reconfiguring oversampling
bmp.set_mode(PowerMode::Normal).await?;
```

### Error handling

```rust
use embassy_bmp280::Bmp280Error;

match Bmp280::new(i2c, Bmp280Address::Default, Bmp280Config::default()).await {
    Ok(bmp) => {
        // Sensor ready
    }

    Err(Bmp280Error::InvalidChipId(id)) => {
        // Unexpected component on the bus; the received ID is available for debugging
        defmt::error!("Unexpected chip ID: 0x{:02X}", id);
    }

    Err(Bmp280Error::InvalidCalibration) => {
        // Corrupted OTP or defective component
        defmt::error!("Invalid calibration");
    }

    Err(Bmp280Error::I2c(e)) => {
        // Bus error (NACK, timeout, arbitration lost...)
        defmt::error!("I2C error: {:?}", e);
    }
}
```

---

## Global signals

`BMP280_SIGNAL` allows the latest measurement to be shared between tasks without explicit shared memory.

```rust
use embassy_bmp280::signals::BMP280_SIGNAL;

// Producer task
BMP280_SIGNAL.signal(data);

// Consumer task (for example: logger, radio, or display)
#[embassy_executor::task]
async fn logger_task() {
    loop {
        let data = BMP280_SIGNAL.wait().await;
        // data.temperature_cdeg, data.pressure_pa256
    }
}
```

---

## Project architecture

```text
embassy-bmp280/
├── Cargo.toml
└── src/
    ├── lib.rs           ← Main driver and public types
    ├── error.rs         ← Bmp280Error<E>
    ├── calibration.rs   ← CalibrationData and Bosch algorithms
    └── signals.rs       ← BMP280_SIGNAL (CriticalSectionRawMutex)
```

### Module responsibilities

- `calibration.rs` isolates the integer Bosch arithmetic and is testable independently from I2C.
- `error.rs` exposes a dedicated error type so `?` works throughout the driver.
- `signals.rs` provides an interrupt-safe inter-task broadcast channel.

---

## Public API

### `Bmp280`

| Method | Description |
|--------|-------------|
| `Bmp280::new(i2c, addr, config).await` | Creates, initializes, and configures the driver |
| `bmp.read().await` | Reads and returns compensated `Bmp280Data` |
| `bmp.set_mode(mode).await` | Changes mode without changing oversampling |
| `bmp.set_address(addr)` | Dynamically updates the I2C address |

### `Bmp280Address`

| Variant | Value |
|---------|-------|
| `Default` | `0x76` |
| `Secondary` | `0x77` |
| `Custom(u8)` | User-defined value |

### `Bmp280Data`

| Field | Type | Value |
|-------|------|-------|
| `temperature_cdeg` | `i32` | °C × 100 (for example, `2315` = 23.15 °C) |
| `pressure_pa256` | `u32` | Pa × 256, Q24.8 format |
| `temperature_celsius()` ¹ | `f32` | Degrees Celsius |
| `pressure_hpa()` ¹ | `f32` | Hectopascals |

¹ Available only with `features = ["float"]`.

### `Bmp280Error<E>`

| Variant | Cause |
|---------|-------|
| `I2c(E)` | I2C bus error (NACK, timeout, etc.) |
| `InvalidChipId(u8)` | Read ID differs from `0x58` / `0x60` |
| `InvalidCalibration` | Corrupted OTP (`dig_T1 == 0`) |

---

## Build targets

RISC-V RP2350:

```sh
cargo check \
  --target riscv32imac-unknown-none-elf \
  --no-default-features \
  --features time-riscv,sync-riscv
```

ARM Cortex-M:

```sh
cargo check --target thumbv7em-none-eabihf
```

## 🛡️ License

Licensed under either of:

    MIT license (http://opensource.org/licenses/MIT)

    Apache License, Version 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

at your option.

Copyright (C) 2026 Jorge Andre Castro
