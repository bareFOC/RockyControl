#![no_std]
//! URDF-free compile-time striker biped profile for embedded firmware targets.

pub use rk_robot_profile::RobotProfile;

/// Compiled 8-DoF striker biped profile (no URDF XML or mesh bytes linked).
pub const PROFILE: &RobotProfile = &rk_robot_profile::generated::striker_biped::PROFILE;
