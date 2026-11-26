#![no_std]
//! URDF-free compile-time humanoid profile for embedded firmware targets.

pub use rk_robot_profile::RobotProfile;

/// Compiled 28-DoF humanoid profile (no URDF XML or mesh bytes linked).
pub const PROFILE: &RobotProfile = &rk_robot_profile::generated::humanoid::PROFILE;
