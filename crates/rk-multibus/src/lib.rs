#![no_std]
//! Shared BareFOC register-pointer servo transport for Rocky Mk1.
//!
//! This crate is the single translation layer between Rocky's
//! [`rk_control::RawServoCommand`] / [`rk_control::RawServoFeedback`] and the
//! canonical BareFOC control table served by `barefoc-master`. Firmware and the
//! software simulator both use the APIs here so they exercise the exact same
//! wire encoding: a two-byte big-endian register pointer followed by
//! little-endian values, with reads implemented as pointer latch + burst read.
//!
//! The upstream `barefoc-master` crate is consumed read-only via a path
//! dependency with its `defmt` feature disabled (bareFOC uses defmt 0.3,
//! Rocky firmware uses defmt 1.0).
//!
//! # Layout
//!
//! - [`joint_map`]: node addressing (`0x20 + joint`), register mapping, unit
//!   conversions (`f32` amperes ↔ N·m / mA), and mode mapping.
//! - [`transport`]: `embedded_hal::i2c::I2c` adapter over the external PIO I3C
//!   transaction boundary, including register-pointer latch state.
//! - [`health`]: per-joint timeout / degraded-mask accounting, ported from
//!   `rk-fw-core`'s old I3C tracker with the bespoke codec removed.

#[cfg(test)]
extern crate std;

pub mod health;
pub mod joint_map;
pub mod transport;

// Canonical BareFOC vocabulary, re-exported so downstream crates have one
// import path and cannot drift to a different register profile.
pub use barefoc_master::{
    ClientError, ControlTableClient, I2cTransport, OperatingMode, Transport, TransportError,
    BROADCAST_ID, CONTROL_TABLE_SIZE, MAX_I2C_WRITE_FRAME, REG_GOAL_CURRENT, REG_GOAL_POSITION,
    REG_GOAL_VELOCITY, REG_OPERATING_MODE, REG_PRESENT_CURRENT, REG_PRESENT_POSITION,
    REG_PRESENT_VELOCITY, REG_TORQUE_ENABLE,
};

pub use health::{
    MultibusBusTracker, MAX_CONSECUTIVE_TIMEOUTS,
};
pub use joint_map::{
    bind_joint, decode_feedback_burst, decode_joint_command, encode_feedback_burst, joint_node_addr,
    read_joint_feedback, write_joint_command, JointCommand, JointFeedback,
    FEEDBACK_BURST_LEN, FEEDBACK_BURST_REG, MAX_SERVO_JOINTS,
    REG_PRESENT_TEMP_MCU, REG_PRESENT_TEMP_MOTOR, REG_PRESENT_TEMPERATURE,
};
pub use rk_robot_profile::{JOINT_NODE_BASE, TorqueConstants};
pub use transport::{
    PioError, PioI3c, PioI3cBus,
};
