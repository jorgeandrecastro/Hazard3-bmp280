// Copyright (C) 2026 Jorge Andre Castro
// GPL-2.0-or-later

//! # embassy-bmp280
//!
//! Async `no_std` driver for the Bosch BMP280 pressure and temperature sensor.
//!
//! ## Features
//! - Native async support through `embedded-hal-async`
//! - OTP calibration read at startup
//! - Temperature and pressure compensation using the official Bosch algorithm
//! - Shared I2C bus support (`embassy-embedded-hal`)
//! - Zero `unsafe`
//!
//! ## Minimal example
//! ```rust,ignore
//! let mut bmp =
//!     Bmp280::new(i2c_device, Bmp280Address::Default, Bmp280Config::default()).await?;
//! let data = bmp.read().await?;
//! // data.temperature_cdeg : i32 (°C × 100, e.g. 2315 = 23.15 °C)
//! // data.pressure_pa256   : u32 (Pa × 256, Q24.8 format)
//! ```

#![no_std]
#![forbid(unsafe_code)]

// ============================================================
// EMBASSY TIME BACKEND SELECTION
// ============================================================

// RISC-V: expose embassy-time-riscv under the standard crate name.
#[cfg(feature = "time-riscv")]
extern crate embassy_time_riscv as embassy_time;

// Prevent selecting both backends simultaneously.
#[cfg(all(feature = "time-upstream", feature = "time-riscv"))]
compile_error!(
    "Features \"time-upstream\" and \"time-riscv\" are mutually exclusive."
);

// Require exactly one backend.
#[cfg(not(any(feature = "time-upstream", feature = "time-riscv")))]
compile_error!(
    "Select one Embassy time backend: \"time-upstream\" or \"time-riscv\"."
);

#[cfg(feature = "sync-upstream")]
extern crate embassy_sync;

#[cfg(feature = "sync-riscv")]
extern crate embassy_sync_riscv as embassy_sync;

pub mod calibration;
pub mod error;
pub mod signals;

use calibration::CalibrationData;
pub use error::Bmp280Error;

use embassy_time::Timer;
use embedded_hal_async::i2c::I2c;

// Registers
const REG_CALIB_START:  u8 = 0x88;
const REG_CHIP_ID:      u8 = 0xD0;
const REG_RESET:        u8 = 0xE0;
const REG_CTRL_MEAS:    u8 = 0xF4;
const REG_CONFIG:       u8 = 0xF5;
const REG_PRESS_MSB:    u8 = 0xF7;

const CHIP_ID_BMP280:   u8 = 0x58;
const CHIP_ID_BME280:   u8 = 0x60; // compatible variant
const RESET_WORD:       u8 = 0xB6;

// I2C address

/// BMP280 I2C address.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Bmp280Address {
    /// SDO connected to GND → 0x76 (factory default).
    Default,
    /// SDO connected to VCC → 0x77.
    Secondary,
    /// Custom address.
    Custom(u8),
}

impl From<Bmp280Address> for u8 {
    fn from(a: Bmp280Address) -> u8 {
        match a {
            Bmp280Address::Default    => 0x76,
            Bmp280Address::Secondary  => 0x77,
            Bmp280Address::Custom(v)  => v,
        }
    }
}

// Configuration
/// Temperature oversampling.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum OversamplingTemp {
    Skip  = 0b000,
    X1    = 0b001,
    X2    = 0b010,
    X4    = 0b011,
    X8    = 0b100,
    X16   = 0b101,
}

/// Pressure oversampling.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum OversamplingPress {
    Skip  = 0b000,
    X1    = 0b001,
    X2    = 0b010,
    X4    = 0b011,
    X8    = 0b100,
    X16   = 0b101,
}

/// Operating mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum PowerMode {
    /// No measurements. Useful for saving power.
    Sleep  = 0b00,
    /// One measurement, then return to sleep.
    Forced = 0b01,
    /// Continuous measurements (recommended with signals).
    Normal = 0b11,
}

/// Complete sensor configuration.
#[derive(Debug, Clone, Copy)]
pub struct Bmp280Config {
    pub temp_os:  OversamplingTemp,
    pub press_os: OversamplingPress,
    pub mode:     PowerMode,
}

impl Default for Bmp280Config {
    /// Balance between performance and consumption: ×1 oversampling, Normal mode.
    fn default() -> Self {
        Self {
            temp_os:  OversamplingTemp::X1,
            press_os: OversamplingPress::X1,
            mode:     PowerMode::Normal,
        }
    }
}

// Output data

/// Compensated measurement from the BMP280.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Bmp280Data {
    /// Temperature in centidegrees Celsius (for example, `2315` = 23.15 °C).
    pub temperature_cdeg: i32,
    /// Pressure in Pascals × 256: Q24.8 format.
    /// Divide by 256 to obtain Pascals.
    pub pressure_pa256: u32,
}

impl Bmp280Data {
    /// Temperature in degrees Celsius (f32).
    /// 
    /// Note: Requires hardware FPU support on the target.
    /// No external libm dependency is required for these simple divisions.
    #[cfg(feature = "float")]
    pub fn temperature_celsius(&self) -> f32 {
        self.temperature_cdeg as f32 / 100.0
    }

    /// Pressure in hPa (f32). 1 hPa = 100 Pa.
    #[cfg(feature = "float")]
    pub fn pressure_hpa(&self) -> f32 {
        (self.pressure_pa256 as f32) / (256.0 * 100.0)
    }
}

// Driver
pub struct Bmp280<I2C> {
    i2c:   I2C,
    addr:  u8,
    calib: CalibrationData,
}

impl<I2C: I2c> Bmp280<I2C> {
    /// Creates and initializes the driver.
    ///
    /// Performs the following steps in order:
    /// 1. Software reset of the sensor
    /// 2. Chip ID verification (0x58 or 0x60)
    /// 3. Reading of the 24 OTP calibration bytes
    /// 4. Application of the configuration (oversampling + mode)
    ///
    /// # Errors
    /// - [`Bmp280Error::I2c`]: communication failure
    /// - [`Bmp280Error::InvalidChipId`]: unexpected component on the bus
    /// - [`Bmp280Error::InvalidCalibration`]: corrupted OTP (dig_T1 == 0)
    pub async fn new(
        i2c:    I2C,
        addr:   Bmp280Address,
        config: Bmp280Config,
    ) -> Result<Self, Bmp280Error<I2C::Error>> {
        let mut driver = Self {
            i2c,
            addr: addr.into(),
            calib: CalibrationData::default(),
        };

        driver.soft_reset().await?;
        driver.verify_chip_id().await?;
        driver.read_calibration().await?;
        driver.apply_config(config).await?;

        Ok(driver)
    }

    // Private
    async fn write_reg(
        &mut self,
        reg: u8,
        val: u8,
    ) -> Result<(), Bmp280Error<I2C::Error>> {
        self.i2c
            .write(self.addr, &[reg, val])
            .await
            .map_err(Bmp280Error::I2c)
    }

    async fn read_reg(
        &mut self,
        reg: u8,
    ) -> Result<u8, Bmp280Error<I2C::Error>> {
        let mut buf = [0u8; 1];
        self.i2c
            .write_read(self.addr, &[reg], &mut buf)
            .await
            .map_err(Bmp280Error::I2c)?;
        Ok(buf[0])
    }

    async fn read_bytes<const N: usize>(
        &mut self,
        reg: u8,
    ) -> Result<[u8; N], Bmp280Error<I2C::Error>> {
        let mut buf = [0u8; N];
        self.i2c
            .write_read(self.addr, &[reg], &mut buf)
            .await
            .map_err(Bmp280Error::I2c)?;
        Ok(buf)
    }

    async fn soft_reset(&mut self) -> Result<(), Bmp280Error<I2C::Error>> {
        self.write_reg(REG_RESET, RESET_WORD).await?;
        // The BMP280 needs approximately 2 ms after reset before it is ready to communicate.
        Timer::after_millis(3).await;
        Ok(())
    }

    async fn verify_chip_id(&mut self) -> Result<(), Bmp280Error<I2C::Error>> {
        let id = self.read_reg(REG_CHIP_ID).await?;
        if id != CHIP_ID_BMP280 && id != CHIP_ID_BME280 {
            return Err(Bmp280Error::InvalidChipId(id));
        }
        Ok(())
    }

    async fn read_calibration(&mut self) -> Result<(), Bmp280Error<I2C::Error>> {
        let raw: [u8; 24] = self.read_bytes::<24>(REG_CALIB_START).await?;
        let calib = CalibrationData::from_raw(&raw);

        // Sanity check: dig_T1 cannot be zero on a valid sensor.
        if calib.dig_t1 == 0 {
            return Err(Bmp280Error::InvalidCalibration);
        }

        self.calib = calib;
        Ok(())
    }

    async fn apply_config(
        &mut self,
        cfg: Bmp280Config,
    ) -> Result<(), Bmp280Error<I2C::Error>> {
        // ctrl_meas: [7:5] osrs_t | [4:2] osrs_p | [1:0] mode
        let ctrl = ((cfg.temp_os  as u8) << 5)
                 | ((cfg.press_os as u8) << 2)
                 |  (cfg.mode     as u8);
        self.write_reg(REG_CTRL_MEAS, ctrl).await?;

        // config: IIR filter disabled, standby time 0.5 ms
        self.write_reg(REG_CONFIG, 0x00).await?;
        Ok(())
    }

    // Public

    /// Reads and returns a compensated measurement (temperature and pressure).
    ///
    /// Reads 6 bytes in burst mode from 0xF7, applies the Bosch compensation
    /// algorithm, and returns [`Bmp280Data`].
    ///
    /// # Errors
    /// - [`Bmp280Error::I2c`]: read failure
    pub async fn read(&mut self) -> Result<Bmp280Data, Bmp280Error<I2C::Error>> {
        // Burst read: press[2:0], then temp[2:0] (datasheet order)
        let raw: [u8; 6] = self.read_bytes::<6>(REG_PRESS_MSB).await?;

        let raw_p = ((raw[0] as u32) << 12)
                  | ((raw[1] as u32) <<  4)
                  | ((raw[2] as u32) >>  4);

        let raw_t = ((raw[3] as u32) << 12)
                  | ((raw[4] as u32) <<  4)
                  | ((raw[5] as u32) >>  4);

        let (temperature_cdeg, t_fine) = self.calib.compensate_temperature(raw_t);
        let pressure_pa256 = self.calib
            .compensate_pressure(raw_p, t_fine)
            .unwrap_or(0);

        Ok(Bmp280Data { temperature_cdeg, pressure_pa256 })
    }

    /// Changes the I2C address dynamically (for example, after rediscovery).
    pub fn set_address(&mut self, addr: Bmp280Address) {
        self.addr = addr.into();
    }

    /// Changes the operating mode without reconfiguring oversampling.
    ///
    /// Useful for switching to `Sleep` during low-power periods.
    pub async fn set_mode(
        &mut self,
        mode: PowerMode,
    ) -> Result<(), Bmp280Error<I2C::Error>> {
        let current = self.read_reg(REG_CTRL_MEAS).await?;
        let updated = (current & 0b1111_1100) | (mode as u8);
        self.write_reg(REG_CTRL_MEAS, updated).await
    }
}