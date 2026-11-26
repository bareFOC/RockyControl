#![cfg_attr(not(feature = "std"), no_std)]
//! Pure `#![no_std]` compile-time robot profile definitions (`rk-robot-profile`).
//!
//! Single source of truth for robot kinematics, link masses, leg geometry,
//! coupled-joint ratios, IMU mounting metadata, and multi-bus servo endpoints,
//! shared identically across MCU firmware, control kernel, and simulator.

use rk_proto::MAX_JOINTS;

pub mod generated;

/// Default effective torque constant (0.9 N·m per ampere).
pub const DEFAULT_NM_PER_AMP: f32 = 0.9;
/// Default max joint torque limit (N·m).
pub const DEFAULT_MAX_TORQUE_NM: f32 = 1.8;
/// Default max joint velocity (rad/s).
pub const DEFAULT_MAX_VELOCITY_RAD_S: f32 = 8.0;
/// First BareFOC node address used by servo joints.
pub const JOINT_NODE_BASE: u8 = 0x20;

/// Parameterized 6-DoF bipedal leg geometry in meters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LegGeometry {
    /// Upper leg (hip pitch -> knee pitch) length in meters.
    pub thigh_len: f32,
    /// Lower leg (knee pitch -> ankle pitch) length in meters.
    pub shin_len: f32,
    /// Combined vertical offset of hip-roll->hip-pitch + ankle-pitch->ankle-roll in meters.
    pub hip_ankle_offset: f32,
    /// Lateral distance from pelvis centerline to one hip axis in meters.
    pub half_hip_spacing: f32,
    /// Nominal standing hip-to-ankle height in meters (slight combat knee bend).
    pub nominal_height: f32,
    /// Minimum safe crouch height in meters.
    pub min_height: f32,
    /// Maximum reachable hip-pitch-to-ankle-pitch chord (`< thigh_len + shin_len`).
    pub max_reach: f32,
    /// Maximum sagittal support foot offset `dxi` in meters.
    pub max_dx: f32,
    /// Maximum lateral support foot offset `dyi` in meters.
    pub max_dy: f32,
    /// Maximum sagittal swing foot offset `dxis` in meters.
    pub max_swing_dx: f32,
    /// Maximum lateral swing foot offset `dyis` in meters.
    pub max_swing_dy: f32,
    /// Peak sinusoidal swing foot clearance in meters (`fhMax`).
    pub max_foot_lift: f32,
    /// Maximum lateral pelvis recovery sway in meters (`swMax`).
    pub max_sway: f32,
}

impl LegGeometry {
    /// Exact Kondo KHR-3HV / ai2001 reference dimensions from `main.c` (in meters).
    pub const fn khr_3hv() -> Self {
        Self {
            thigh_len: 0.065,
            shin_len: 0.065,
            hip_ankle_offset: 0.0645,
            half_hip_spacing: 0.0215,
            nominal_height: 0.185,
            min_height: 0.140,
            max_reach: 0.129,
            max_dx: 0.045,
            max_dy: 0.045,
            max_swing_dx: 0.055,
            max_swing_dy: 0.055,
            max_foot_lift: 0.032,
            max_sway: 0.030,
        }
    }

    /// 28-DoF humanoid leg dimensions (from `humanoid.urdf`: thigh 0.088 + knee 0.020 = 0.108 m,
    /// shin 0.090 m, hip_ankle_offset = 0.016 m, half_hip_spacing = 0.026 m).
    pub const fn humanoid() -> Self {
        Self {
            thigh_len: 0.108,
            shin_len: 0.090,
            hip_ankle_offset: 0.016,
            half_hip_spacing: 0.026,
            nominal_height: 0.195,
            min_height: 0.145,
            max_reach: 0.196,
            max_dx: 0.050,
            max_dy: 0.045,
            max_swing_dx: 0.070,
            max_swing_dy: 0.060,
            max_foot_lift: 0.035,
            max_sway: 0.025,
        }
    }
}

/// Mass properties of one link.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LinkMass {
    /// Human-readable link name (matches the URDF `<link name="...">`).
    pub name: &'static str,
    /// Index of the parent link in `links`; self-index for the pelvis root.
    pub parent: u8,
    /// Actuator joint index in `0..joint_count`; `None` for fixed/root links.
    pub joint_index: Option<u8>,
    /// Local COM offset in the link's own frame, from `<inertial><origin xyz>`.
    pub com: [f32; 3],
    /// Link mass in kilograms, from `<inertial><mass value>`.
    pub mass: f32,
}

/// A joint's fixed transform and actuator mapping relative to its parent link.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointKin {
    /// Axis unit vector in the **parent** link frame, from URDF `<axis xyz>`.
    pub axis: [f32; 3],
    /// Joint origin translation in the **parent** link frame, from `<origin xyz>`.
    pub origin: [f32; 3],
    /// Actuator index in `0..joint_count` that drives this URDF joint, or `None` for fixed/root.
    pub actuator_index: Option<u8>,
    /// Scale factor from actuator angle to URDF joint angle (`1.0` for 1:1, `±0.5` for coupled half-joints).
    pub mimic_scale: f32,
}

/// Per-joint torque constants used to convert between N·m commands and amperes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TorqueConstants {
    /// Effective N·m per ampere of motor current, indexed by joint.
    pub nm_per_amp: [f32; MAX_JOINTS],
}

impl Default for TorqueConstants {
    fn default() -> Self {
        Self {
            nm_per_amp: [DEFAULT_NM_PER_AMP; MAX_JOINTS],
        }
    }
}

impl TorqueConstants {
    /// Uniform torque constant for every joint.
    pub const fn uniform(nm_per_amp: f32) -> Self {
        Self {
            nm_per_amp: [nm_per_amp; MAX_JOINTS],
        }
    }

    fn constant(&self, joint: usize) -> f32 {
        let k = self.nm_per_amp[joint.min(MAX_JOINTS - 1)];
        if k.is_finite() && k > 0.0 {
            k
        } else {
            0.0
        }
    }

    /// Convert a commanded torque in N·m to BareFOC goal current in amperes.
    pub fn nm_to_amps(&self, joint: usize, torque_nm: f32) -> f32 {
        let k = self.constant(joint);
        if k == 0.0 { 0.0 } else { torque_nm / k }
    }

    /// Convert a BareFOC current reading in amperes to N·m.
    pub fn amps_to_nm(&self, joint: usize, current_amps: f32) -> f32 {
        self.constant(joint) * current_amps
    }

    /// Convert N·m to Rocky's `u16` milliampere feedback unit.
    pub fn nm_to_milliamps(&self, joint: usize, torque_nm: f32) -> u16 {
        let ma = self.nm_to_amps(joint, torque_nm) * 1000.0_f32;
        if !ma.is_finite() {
            return 0;
        }
        let mag = if ma < 0.0 { -ma } else { ma };
        if mag >= u16::MAX as f32 {
            u16::MAX
        } else {
            (mag + 0.5_f32) as u16
        }
    }

    /// Convert Rocky's milliampere feedback unit back to N·m.
    pub fn milliamps_to_nm(&self, joint: usize, current_ma: u16) -> f32 {
        self.amps_to_nm(joint, (current_ma as f32) * 0.001_f32)
    }
}

/// Hardware I3C bus + node address for one actuator.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct ServoEndpoint {
    /// I3C bus index (`0..4`).
    pub bus_id: u8,
    /// BareFOC node address on that bus.
    pub node_addr: u8,
}

impl ServoEndpoint {
    /// Default single-bus endpoint (`bus_id = 0`, `node_addr = 0x20 + joint`).
    pub const fn default_for_joint(joint: usize) -> Self {
        Self {
            bus_id: 0,
            node_addr: JOINT_NODE_BASE.saturating_add(joint as u8),
        }
    }
}

/// Coupled double-joint ratio metadata (anatomical total bend -> motor/URDF half-joint).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CoupledJointMap {
    /// Multiplier applied to anatomical calibrated angle to obtain raw motor angle
    /// (`1.0` for 1:1 joints, `0.5` for 1:2 coupled double-knees/elbows).
    pub motor_to_joint_ratio: [f32; MAX_JOINTS],
}

impl Default for CoupledJointMap {
    fn default() -> Self {
        Self {
            motor_to_joint_ratio: [1.0; MAX_JOINTS],
        }
    }
}

/// Optional anatomical role -> actuator index mapping for a robot profile.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct JointIndices {
    pub waist_yaw: Option<u8>,
    pub waist_pitch: Option<u8>,
    pub waist_roll: Option<u8>,
    pub head_yaw: Option<u8>,
    pub head_pitch: Option<u8>,
    pub head_roll: Option<u8>,
    pub l_shoulder_pitch: Option<u8>,
    pub l_shoulder_roll: Option<u8>,
    pub l_shoulder_yaw: Option<u8>,
    pub l_elbow_pitch: Option<u8>,
    pub r_shoulder_pitch: Option<u8>,
    pub r_shoulder_roll: Option<u8>,
    pub r_shoulder_yaw: Option<u8>,
    pub r_elbow_pitch: Option<u8>,
    pub l_hip_yaw: Option<u8>,
    pub l_hip_roll: Option<u8>,
    pub l_hip_pitch: Option<u8>,
    pub l_knee_pitch: Option<u8>,
    pub l_ankle_pitch: Option<u8>,
    pub l_ankle_roll: Option<u8>,
    pub l_toe_pitch: Option<u8>,
    pub r_hip_yaw: Option<u8>,
    pub r_hip_roll: Option<u8>,
    pub r_hip_pitch: Option<u8>,
    pub r_knee_pitch: Option<u8>,
    pub r_ankle_pitch: Option<u8>,
    pub r_ankle_roll: Option<u8>,
    pub r_toe_pitch: Option<u8>,
}

const fn matches_opt(opt: Option<u8>, idx: usize) -> bool {
    match opt {
        Some(v) => v as usize == idx,
        None => false,
    }
}

impl JointIndices {
    pub const fn empty() -> Self {
        Self {
            waist_yaw: None,
            waist_pitch: None,
            waist_roll: None,
            head_yaw: None,
            head_pitch: None,
            head_roll: None,
            l_shoulder_pitch: None,
            l_shoulder_roll: None,
            l_shoulder_yaw: None,
            l_elbow_pitch: None,
            r_shoulder_pitch: None,
            r_shoulder_roll: None,
            r_shoulder_yaw: None,
            r_elbow_pitch: None,
            l_hip_yaw: None,
            l_hip_roll: None,
            l_hip_pitch: None,
            l_knee_pitch: None,
            l_ankle_pitch: None,
            l_ankle_roll: None,
            l_toe_pitch: None,
            r_hip_yaw: None,
            r_hip_roll: None,
            r_hip_pitch: None,
            r_knee_pitch: None,
            r_ankle_pitch: None,
            r_ankle_roll: None,
            r_toe_pitch: None,
        }
    }

    pub const fn is_left_leg(&self, idx: usize) -> bool {
        matches_opt(self.l_hip_yaw, idx)
            || matches_opt(self.l_hip_roll, idx)
            || matches_opt(self.l_hip_pitch, idx)
            || matches_opt(self.l_knee_pitch, idx)
            || matches_opt(self.l_ankle_pitch, idx)
            || matches_opt(self.l_ankle_roll, idx)
            || matches_opt(self.l_toe_pitch, idx)
    }

    pub const fn is_right_leg(&self, idx: usize) -> bool {
        matches_opt(self.r_hip_yaw, idx)
            || matches_opt(self.r_hip_roll, idx)
            || matches_opt(self.r_hip_pitch, idx)
            || matches_opt(self.r_knee_pitch, idx)
            || matches_opt(self.r_ankle_pitch, idx)
            || matches_opt(self.r_ankle_roll, idx)
            || matches_opt(self.r_toe_pitch, idx)
    }

    pub const fn is_leg_joint(&self, idx: usize) -> bool {
        self.is_left_leg(idx) || self.is_right_leg(idx)
    }

    /// Sagittal-plane leg joints driven by the SLIP running torque overlay (including toes).
    pub const fn is_slip_leg_joint(&self, idx: usize) -> bool {
        matches_opt(self.l_hip_pitch, idx)
            || matches_opt(self.l_knee_pitch, idx)
            || matches_opt(self.l_ankle_pitch, idx)
            || matches_opt(self.l_toe_pitch, idx)
            || matches_opt(self.r_hip_pitch, idx)
            || matches_opt(self.r_knee_pitch, idx)
            || matches_opt(self.r_ankle_pitch, idx)
            || matches_opt(self.r_toe_pitch, idx)
    }
}

/// Complete compile-time robot profile shared by firmware, kernel, and simulator.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RobotProfile {
    pub name: &'static str,
    pub joint_count: usize,
    pub urdf_joint_count: usize,
    pub joint_indices: JointIndices,
    pub links: &'static [LinkMass],
    pub joint_kin: &'static [JointKin],
    pub total_mass_kg: f32,
    pub inv_total_mass: f32,
    pub pelvis_index: usize,
    pub left_foot_index: Option<usize>,
    pub right_foot_index: Option<usize>,
    pub left_toe_index: Option<usize>,
    pub right_toe_index: Option<usize>,
    pub imu_link_index: usize,
    pub imu_mounting_rpy: [f32; 3],
    pub z_up: bool,
    pub motor_to_joint_ratio: [f32; MAX_JOINTS],
    pub leg_geometry: LegGeometry,
    pub foot_half_length_m: f32,
    pub foot_margin_m: f32,
    pub h_com_m: f32,
    pub torque_constants: TorqueConstants,
    pub max_torque_nm: [f32; MAX_JOINTS],
    pub max_velocity_rad_s: [f32; MAX_JOINTS],
    pub servo_endpoints: [ServoEndpoint; MAX_JOINTS],
    pub default_limits: [[f32; 2]; MAX_JOINTS],
}

pub const fn default_endpoints() -> [ServoEndpoint; MAX_JOINTS] {
    let mut out = [ServoEndpoint {
        bus_id: 0,
        node_addr: 0,
    }; MAX_JOINTS];
    let mut i = 0;
    while i < MAX_JOINTS {
        out[i] = ServoEndpoint::default_for_joint(i);
        i += 1;
    }
    out
}
