// Copyright (C) 2026 Jorge Andre Castro
// license "MIT OR Apache-2.0"

//! BMP280 driver error types.

/// Possible BMP280 driver errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Bmp280Error<E> {
    /// Underlying I2C communication error.
    I2c(E),
    /// The read chip ID does not match BMP280/BME280 (0x58 or 0x60).
    /// Contains the unexpected received ID.
    InvalidChipId(u8),
    /// The calibration data is invalid (for example, dig_T1 == 0).
    InvalidCalibration,
}

impl<E> From<E> for Bmp280Error<E> {
    fn from(e: E) -> Self {
        Self::I2c(e)
    }
}