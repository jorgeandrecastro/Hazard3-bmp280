// Copyright (C) 2026 Jorge Andre Castro
// license  "MIT OR Apache-2.0"

//! Global asynchronous signals for sharing BMP280 data.
//!
//! Uses `CriticalSectionRawMutex`: safe to access from interrupts.

#![forbid(unsafe_code)]

use crate::Bmp280Data;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};

/// Latest published measurement (compensated temperature and pressure).
pub static BMP280_SIGNAL: Signal<CriticalSectionRawMutex, Bmp280Data> = Signal::new();