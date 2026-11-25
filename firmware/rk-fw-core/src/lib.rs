#![cfg_attr(not(feature = "std"), no_std)]
//! Board-agnostic `#![no_std]` Rocky Humanoid MCU firmware core (`rk-fw-core`).
//!
//! Shared identically between:
//! - `firmware/rk-rp2350` (Raspberry Pi RP2350 Cortex-M33F + `embassy-rp` + `embassy-usb`)
//! - `firmware/rk-esp32p4` (Espressif ESP32-P4 RISC-V + `esp-hal` + `embassy-usb`)

pub mod config;
pub mod control_task;
pub mod copper;
pub mod imu;
pub mod joint_plant;
pub mod lsm6dsv;
pub mod model_store;
#[cfg(feature = "std")]
pub mod replay;
pub mod servo_bus;
pub mod status_led;
pub mod transport;

// Host+MCU single-source control application entry points.
pub use control_task::{RobotIo, control_task, step_control_tick};
pub use copper::{ControlTask, ControlTick, current_led_mode};
#[cfg(feature = "std")]
pub use replay::{
    CopperListTick, CopperReplayLog, ReplayVerification, verify_deterministic_replay,
};
