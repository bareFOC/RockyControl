#![cfg_attr(not(feature = "std"), no_std)]
//! Board-agnostic `#![no_std]` Rocky Humanoid MCU firmware core (`rk-fw-core`).
//!
//! Shared identically between:
//! - `firmware/rk-rp2350` (Raspberry Pi RP2350 Cortex-M33F + `embassy-rp` + `embassy-usb`)
//! - `firmware/rk-esp32p4` (Espressif ESP32-P4 RISC-V + `esp-hal` + `embassy-usb`)

#[cfg(feature = "copper")]
extern crate alloc;

pub mod config;
pub mod model_store;
pub mod servo_bus;
pub mod status_led;
pub mod transport;

#[cfg(feature = "copper")]
pub mod copper;
