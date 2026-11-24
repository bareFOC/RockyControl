#![cfg_attr(not(feature = "std"), no_std)]
//! Layer 1 — Pure `#![no_std]` Safety, Walking & UVC Vertical Balance Kernel (`rk-control`).
//!
//! Enforces:
//! - 100 ms deadman watchdog (`WATCHDOG_TIMEOUT_MS`)
//! - Per-joint software stops (`[min_rad, max_rad]`)
//! - Slew-rate velocity clamp (`omega_base * speed_limit_pct / 100 * dt`)
//! - Calibration zero-offset & sign-inversion mapping
//! - Bumpless `Teach -> Drive` target handoff & safe Reset-to-Teach invariant
//! - Geometric UVC (Upper Body Vertical Control) & 6-DoF analytic leg IK (`uvc.rs`)
//! - Protective `FallTuck` -> `EStop` (`TILT_FALL_CUTOFF`) safety latch
//!
//! Strictly single-precision `f32` everywhere for RP2350 Cortex-M33 FPU numeric parity.

pub mod estimator;
pub mod uvc;

use core::f32::consts::{FRAC_PI_2, PI, TAU};

pub use estimator::{DcmSwingHint, KinematicEstimator};
use rk_proto::{
    BalanceGains, BalanceTelemetry, ConfigCmd, ControlMode, FaultFlags, HostToMcu, ImuState,
    JointCommandMode, MAX_JOINTS, McuToHost, PROTOCOL_VERSION, WATCHDOG_TIMEOUT_MS, WalkCmd,
    classify_fall, fall_orientation, uvc_phase,
};
use serde::{Deserialize, Serialize};
pub use uvc::{LegGeometry, LegIkAngles, UvcController, UvcPhase, UvcStepOutput, solve_leg_ik};

pub const DEFAULT_BASE_SLEW_RAD_PER_SEC: f32 = 2.0;
pub const BALANCE_SLEW_RAD_PER_SEC: f32 = 5.5;
pub const OVER_CURRENT_TRIP_MA: u16 = 1950;
pub const OVER_TEMP_TRIP_C: u8 = 78;
pub const UNDER_VOLTAGE_TRIP_MV: u16 = 10_500;

// Common upper-body joint indices (identical in 17-DoF and 21-DoF Rocky Mk1 URDFs).
const J_WAIST_YAW: usize = 0;
const J_L_SHOULDER_PITCH: usize = 3;
const J_L_SHOULDER_ROLL: usize = 4;
const J_R_SHOULDER_PITCH: usize = 6;
const J_R_SHOULDER_ROLL: usize = 7;

// 21-DoF Rocky Mk1 6-DoF leg indices (matches assets/sample_humanoid/rocky_mk1.urdf).
const J21_L_HIP_YAW: usize = 9;
const J21_L_HIP_ROLL: usize = 10;
const J21_L_HIP_PITCH: usize = 11;
const J21_L_KNEE_PITCH: usize = 12;
const J21_L_ANKLE_PITCH: usize = 13;
const J21_L_ANKLE_ROLL: usize = 14;
const J21_R_HIP_YAW: usize = 15;
const J21_R_HIP_ROLL: usize = 16;
const J21_R_HIP_PITCH: usize = 17;
const J21_R_KNEE_PITCH: usize = 18;
const J21_R_ANKLE_PITCH: usize = 19;
const J21_R_ANKLE_ROLL: usize = 20;

// Legacy 17-DoF 4-DoF leg fallback indices (graceful degradation for older 4-DoF URDFs).
const J17_L_HIP_PITCH: usize = 9;
const J17_L_HIP_ROLL: usize = 10;
const J17_L_KNEE_PITCH: usize = 11;
const J17_L_ANKLE_PITCH: usize = 12;
const J17_R_HIP_PITCH: usize = 13;
const J17_R_HIP_ROLL: usize = 14;
const J17_R_KNEE_PITCH: usize = 15;
const J17_R_ANKLE_PITCH: usize = 16;

/// Deterministic `#![no_std]` single-precision sine accurate to < 2e-5 rad over all angles.
#[inline]
pub fn sin_f32(mut x: f32) -> f32 {
    if !x.is_finite() {
        return 0.0;
    }
    let k = ((x + PI) / TAU) as i32;
    x -= (k as f32) * TAU;
    if x > PI {
        x -= TAU;
    } else if x < -PI {
        x += TAU;
    }
    if x > FRAC_PI_2 {
        x = PI - x;
    } else if x < -FRAC_PI_2 {
        x = -PI - x;
    }
    let x2 = x * x;
    x * (1.0_f32
        + x2 * (-0.166_666_67_f32
            + x2 * (0.008_333_334_f32 + x2 * (-0.000_198_412_7_f32 + x2 * 0.000_002_755_732_f32))))
}

/// Deterministic `#![no_std]` single-precision cosine.
#[inline]
pub fn cos_f32(x: f32) -> f32 {
    sin_f32(x + FRAC_PI_2)
}

pub const DEFAULT_MAX_TORQUE_NM: f32 = 1.8;
pub const DEFAULT_MAX_VELOCITY_RAD_S: f32 = 8.0;

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub struct JointCalibration {
    pub zero_offset_rad: f32,
    pub inverted: bool,
    pub min_rad: f32,
    pub max_rad: f32,
    /// Per-joint physical peak torque/current limit ceiling in N·m (enforced downstream).
    pub max_torque_nm: f32,
    /// Per-joint physical peak angular velocity ceiling in rad/s (enforced downstream).
    pub max_velocity_rad_s: f32,
}

impl Default for JointCalibration {
    fn default() -> Self {
        Self {
            zero_offset_rad: 0.0,
            inverted: false,
            min_rad: -1.57,
            max_rad: 1.57,
            max_torque_nm: DEFAULT_MAX_TORQUE_NM,
            max_velocity_rad_s: DEFAULT_MAX_VELOCITY_RAD_S,
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub struct RawServoFeedback {
    pub raw_angle_rad: [f32; MAX_JOINTS],
    pub velocity_rad_s: [f32; MAX_JOINTS],
    pub current_ma: [u16; MAX_JOINTS],
    pub temp_c: [u8; MAX_JOINTS],
    pub bus_voltage_mv: u16,
    pub bus_error: bool,
    pub imu: ImuState,
    /// Cumulative I3C servo bus timeout count reported by the bus driver.
    pub i3c_timeout_count: u16,
    /// Active IMU source validity flags (`estimator_valid::LSM6DSV_OK` / `BNO055_OK`).
    pub imu_source_mask: u32,
}

impl Default for RawServoFeedback {
    fn default() -> Self {
        Self {
            raw_angle_rad: [0.0; MAX_JOINTS],
            velocity_rad_s: [0.0; MAX_JOINTS],
            current_ma: [0; MAX_JOINTS],
            temp_c: [28; MAX_JOINTS],
            bus_voltage_mv: 12_400,
            bus_error: false,
            imu: ImuState::default(),
            i3c_timeout_count: 0,
            imu_source_mask: rk_proto::estimator_valid::LSM6DSV_OK,
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub struct RawServoCommand {
    pub torque_mask: u32,
    pub raw_target_rad: [f32; MAX_JOINTS],
    pub command_mode: [u8; MAX_JOINTS],
    pub raw_velocity_rad_s: [f32; MAX_JOINTS],
    pub torque_nm: [f32; MAX_JOINTS],
}

impl Default for RawServoCommand {
    fn default() -> Self {
        Self {
            torque_mask: 0,
            raw_target_rad: [0.0; MAX_JOINTS],
            command_mode: [JointCommandMode::Position as u8; MAX_JOINTS],
            raw_velocity_rad_s: [0.0; MAX_JOINTS],
            torque_nm: [DEFAULT_MAX_TORQUE_NM; MAX_JOINTS],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct KernelStepOutput {
    pub servo_cmd: RawServoCommand,
    pub telemetry: McuToHost,
}

#[inline]
pub fn raw_to_calibrated(raw_rad: f32, cal: &JointCalibration) -> f32 {
    let sign = if cal.inverted { -1.0_f32 } else { 1.0_f32 };
    sign * (raw_rad - cal.zero_offset_rad)
}

#[inline]
pub fn calibrated_to_raw(cal_rad: f32, cal: &JointCalibration) -> f32 {
    let sign = if cal.inverted { -1.0_f32 } else { 1.0_f32 };
    (sign * cal_rad) + cal.zero_offset_rad
}

#[derive(Clone, Debug)]
pub struct DirectServoKernel {
    joint_count: usize,
    base_slew_rad_per_sec: f32,
    calibration: [JointCalibration; MAX_JOINTS],
    active_mode: ControlMode,
    latched_faults: FaultFlags,
    last_valid_seq: u32,
    last_valid_rx_ms: Option<u32>,
    last_host_speed_pct: u8,
    last_host_torque_mask: u32,
    last_host_target_rad: [f32; MAX_JOINTS],
    last_host_command_mode: [u8; MAX_JOINTS],
    last_host_velocity_rad_s: [f32; MAX_JOINTS],
    last_host_torque_limit_nm: [f32; MAX_JOINTS],
    applied_target_rad: [f32; MAX_JOINTS],
    initialized_targets: bool,
    flash_commit_count: u32,
    // MCU-native geometric UVC balance, kinematic DCM estimator & walking controller
    balance_gains: BalanceGains,
    last_walk_cmd: WalkCmd,
    uvc: UvcController,
    estimator: KinematicEstimator,
    uvc_blend: f32,
    /// Orientation captured at the most recent `TILT_FALL_CUTOFF` latch
    /// (`fall_orientation::*` wire code). Persists while the fault is latched.
    fall_orientation: u8,
    /// Cumulative MCU wire drop counters forwarded into `McuToHost`.
    rx_crc_errors: u16,
    rx_version_mismatches: u16,
}

impl DirectServoKernel {
    pub fn new(joint_count: usize, initial_cal: &[JointCalibration]) -> Self {
        let clamped_count = joint_count.clamp(1, MAX_JOINTS);
        let mut calibration = [JointCalibration::default(); MAX_JOINTS];
        for (dst, src) in calibration.iter_mut().zip(initial_cal.iter()) {
            *dst = *src;
        }
        Self {
            joint_count: clamped_count,
            base_slew_rad_per_sec: DEFAULT_BASE_SLEW_RAD_PER_SEC,
            calibration,
            active_mode: ControlMode::Teach,
            latched_faults: FaultFlags::empty(),
            last_valid_seq: 0,
            last_valid_rx_ms: None,
            last_host_speed_pct: 55,
            last_host_torque_mask: 0,
            last_host_target_rad: [0.0; MAX_JOINTS],
            last_host_command_mode: [JointCommandMode::Position as u8; MAX_JOINTS],
            last_host_velocity_rad_s: [0.0; MAX_JOINTS],
            last_host_torque_limit_nm: [DEFAULT_MAX_TORQUE_NM; MAX_JOINTS],
            applied_target_rad: [0.0; MAX_JOINTS],
            initialized_targets: false,
            flash_commit_count: 0,
            balance_gains: BalanceGains::default(),
            last_walk_cmd: WalkCmd::default(),
            uvc: UvcController::new(LegGeometry::rocky_mk1()),
            estimator: KinematicEstimator::new(),
            uvc_blend: 0.0,
            fall_orientation: fall_orientation::UNKNOWN,
            rx_crc_errors: 0,
            rx_version_mismatches: 0,
        }
    }

    pub fn record_rx_crc_error(&mut self) {
        self.rx_crc_errors = self.rx_crc_errors.saturating_add(1);
    }

    pub fn record_rx_version_mismatch(&mut self) {
        self.rx_version_mismatches = self.rx_version_mismatches.saturating_add(1);
    }

    pub fn set_wire_error_counts(&mut self, rx_crc_errors: u16, rx_version_mismatches: u16) {
        self.rx_crc_errors = rx_crc_errors;
        self.rx_version_mismatches = rx_version_mismatches;
    }

    pub fn joint_count(&self) -> usize {
        self.joint_count
    }

    pub fn active_mode(&self) -> ControlMode {
        self.active_mode
    }

    pub fn calibration(&self) -> &[JointCalibration] {
        &self.calibration[..self.joint_count]
    }

    pub fn balance_gains(&self) -> BalanceGains {
        self.balance_gains
    }

    pub fn flash_commit_count(&self) -> u32 {
        self.flash_commit_count
    }

    pub fn step(
        &mut self,
        now_ms: u32,
        dt_sec: f32,
        host_cmd: Option<&HostToMcu>,
        feedback: &RawServoFeedback,
    ) -> KernelStepOutput {
        let dt = dt_sec.clamp(0.0001_f32, 0.1_f32);

        // 1. Convert raw encoder angles -> calibrated joint space
        let mut measured_rad = [0.0_f32; MAX_JOINTS];
        for i in 0..self.joint_count {
            measured_rad[i] = raw_to_calibrated(feedback.raw_angle_rad[i], &self.calibration[i]);
        }

        if !self.initialized_targets {
            self.applied_target_rad = measured_rad;
            self.last_host_target_rad = measured_rad;
            self.initialized_targets = true;
        }

        // 2. Ingest host frame if present
        if let Some(cmd) = host_cmd {
            let is_fresh = match self.last_valid_rx_ms {
                None => true,
                Some(_) => cmd.seq != self.last_valid_seq,
            };
            if is_fresh {
                self.last_valid_seq = cmd.seq;
                self.last_valid_rx_ms = Some(now_ms);
            }

            let mut cleared_faults_this_tick = false;
            match cmd.config {
                ConfigCmd::None => {}
                ConfigCmd::SetZeroOffset { joint, zero_rad } => {
                    let idx = joint as usize;
                    if idx < self.joint_count && zero_rad.is_finite() {
                        self.calibration[idx].zero_offset_rad = zero_rad;
                        measured_rad[idx] =
                            raw_to_calibrated(feedback.raw_angle_rad[idx], &self.calibration[idx]);
                        self.applied_target_rad[idx] = measured_rad[idx];
                        self.last_host_target_rad[idx] = measured_rad[idx];
                    }
                }
                ConfigCmd::SetInvert { joint, inverted } => {
                    let idx = joint as usize;
                    if idx < self.joint_count {
                        self.calibration[idx].inverted = inverted;
                        measured_rad[idx] =
                            raw_to_calibrated(feedback.raw_angle_rad[idx], &self.calibration[idx]);
                        self.applied_target_rad[idx] = measured_rad[idx];
                        self.last_host_target_rad[idx] = measured_rad[idx];
                    }
                }
                ConfigCmd::SetSoftLimits {
                    joint,
                    min_rad,
                    max_rad,
                } => {
                    let idx = joint as usize;
                    if idx < self.joint_count && min_rad.is_finite() && max_rad.is_finite() {
                        let (lo, hi) = if min_rad <= max_rad {
                            (min_rad, max_rad)
                        } else {
                            (max_rad, min_rad)
                        };
                        self.calibration[idx].min_rad = lo;
                        self.calibration[idx].max_rad = hi;
                    }
                }
                ConfigCmd::SetBalanceGains(g) => {
                    if g.uvc_gain_pitch.is_finite()
                        && g.uvc_gain_roll.is_finite()
                        && g.tilt_deadband_rad.is_finite()
                        && g.gyro_damp_pitch.is_finite()
                        && g.gyro_damp_roll.is_finite()
                        && g.arm_assist.is_finite()
                        && g.fall_cutoff_rad.is_finite()
                        && g.dcm_step_gain.is_finite()
                        && g.foot_half_length_m.is_finite()
                        && g.foot_margin_m.is_finite()
                        && g.ukemi_tilt_rad.is_finite()
                        && g.ukemi_torque_scale.is_finite()
                        && g.compliance_scale.is_finite()
                    {
                        let foot_half = g.foot_half_length_m.clamp(0.015, 0.12);
                        let fall_cutoff = g.fall_cutoff_rad.clamp(0.30, 1.35);
                        self.balance_gains = BalanceGains {
                            uvc_gain_pitch: g.uvc_gain_pitch.clamp(0.0, 1.5),
                            uvc_gain_roll: g.uvc_gain_roll.clamp(0.0, 1.5),
                            tilt_deadband_rad: g.tilt_deadband_rad.clamp(0.005, 0.15),
                            gyro_damp_pitch: g.gyro_damp_pitch.clamp(0.0, 0.5),
                            gyro_damp_roll: g.gyro_damp_roll.clamp(0.0, 0.5),
                            arm_assist: g.arm_assist.clamp(0.0, 1.5),
                            fall_cutoff_rad: fall_cutoff,
                            dcm_step_gain: g.dcm_step_gain.clamp(0.0, 1.5),
                            foot_half_length_m: foot_half,
                            foot_margin_m: g.foot_margin_m.clamp(0.010, foot_half),
                            ukemi_tilt_rad: g
                                .ukemi_tilt_rad
                                .clamp(0.25, (fall_cutoff - 0.03).max(0.25)),
                            ukemi_torque_scale: g.ukemi_torque_scale.clamp(0.05, 0.50),
                            compliance_scale: g.compliance_scale.clamp(0.10, 1.0),
                        };
                    }
                }
                ConfigCmd::CommitFlash => {
                    self.flash_commit_count = self.flash_commit_count.saturating_add(1);
                }
                ConfigCmd::ClearFaults => {
                    // Explicit Reset E-Stop: returns strictly to Teach (never directly to Drive)
                    self.latched_faults = FaultFlags::empty();
                    self.active_mode = ControlMode::Teach;
                    self.applied_target_rad = measured_rad;
                    self.last_host_target_rad = measured_rad;
                    self.uvc.reset();
                    self.estimator.reset();
                    self.uvc_blend = 0.0;
                    self.fall_orientation = fall_orientation::UNKNOWN;
                    cleared_faults_this_tick = true;
                }
            }

            self.last_host_speed_pct = cmd.speed_limit_pct.clamp(1, 100);
            self.last_host_torque_mask = cmd.torque_mask;
            self.last_walk_cmd = cmd.walk;
            for i in 0..self.joint_count {
                if cmd.target_rad[i].is_finite() {
                    self.last_host_target_rad[i] = cmd.target_rad[i];
                }
                self.last_host_command_mode[i] =
                    JointCommandMode::from_wire(cmd.command_mode[i]).to_wire();
                if cmd.velocity_rad_s[i].is_finite() {
                    self.last_host_velocity_rad_s[i] = cmd.velocity_rad_s[i];
                }
                if cmd.torque_limit_nm[i].is_finite() {
                    self.last_host_torque_limit_nm[i] = cmd.torque_limit_nm[i];
                }
            }

            if cmd.mode == ControlMode::EStop {
                self.active_mode = ControlMode::EStop;
                self.latched_faults.insert(FaultFlags::ESTOP_ACTIVE);
            } else if !cleared_faults_this_tick && self.active_mode != ControlMode::EStop {
                if cmd.mode == ControlMode::Drive && self.active_mode == ControlMode::Teach {
                    // Enforce MCU-side bumpless sync on Teach -> Drive
                    self.applied_target_rad = measured_rad;
                    self.uvc.reset();
                    self.uvc_blend = 0.0;
                }
                self.active_mode = cmd.mode;
            }
        }

        // 3. Enforce 100 ms Deadman Watchdog
        if let Some(last_rx) = self.last_valid_rx_ms {
            if now_ms.saturating_sub(last_rx) > WATCHDOG_TIMEOUT_MS {
                self.active_mode = ControlMode::EStop;
                self.latched_faults.insert(FaultFlags::WATCHDOG_TIMEOUT);
            }
        }

        // 4. Evaluate hardware health advisories
        let mut transient_faults = FaultFlags::empty();
        if feedback.bus_error {
            transient_faults.insert(FaultFlags::SERVO_BUS_ERROR);
        }
        if feedback.bus_voltage_mv < UNDER_VOLTAGE_TRIP_MV {
            transient_faults.insert(FaultFlags::UNDER_VOLTAGE);
        }
        for i in 0..self.joint_count {
            if feedback.current_ma[i] >= OVER_CURRENT_TRIP_MA {
                transient_faults.insert(FaultFlags::OVER_CURRENT);
            }
            if feedback.temp_c[i] >= OVER_TEMP_TRIP_C {
                transient_faults.insert(FaultFlags::OVER_TEMP);
            }
        }

        // 5. Run Kinematic CoM / DCM / ZMP Estimator & evaluate Ukemi pre-fall relaxation
        let imu = feedback.imu;
        let max_tilt = imu.pitch_rad.abs().max(imu.roll_rad.abs());
        let ukemi_active = self.active_mode == ControlMode::Drive
            && max_tilt >= self.balance_gains.ukemi_tilt_rad
            && max_tilt < self.balance_gains.fall_cutoff_rad;

        let (estimator_telem, dcm_hint) = self.estimator.step(
            dt,
            self.joint_count,
            &measured_rad,
            imu,
            self.uvc.geometry().nominal_height,
            self.uvc.contact_mask(),
            feedback.imu_source_mask,
            ukemi_active,
            self.balance_gains,
        );

        // 5b. Compute MCU-native UVC upper-body balance + locomotion overlay
        let mut desired_rad = self.last_host_target_rad;
        let mut balance_telem = BalanceTelemetry {
            hip_height_m: self.uvc.geometry().nominal_height,
            ..Default::default()
        };

        let humanoid_legs_available = self.joint_count >= 17;
        let has_6dof_legs = self.joint_count >= 21;

        if self.active_mode == ControlMode::Drive && humanoid_legs_available {
            let walk = self.last_walk_cmd;
            let uvc_engaged = walk.balance_enabled || walk.walk_enabled;
            let tilt_exceeded = max_tilt >= self.balance_gains.fall_cutoff_rad;

            if uvc_engaged || tilt_exceeded || self.uvc.phase() == UvcPhase::FallTuck {
                let uvc_out =
                    self.uvc
                        .step_with_dcm(dt, imu, walk, self.balance_gains, dcm_hint);

                if uvc_out.fall_cutoff_ready {
                    // Protective tuck complete -> drop torque & latch TILT_FALL_CUTOFF
                    self.active_mode = ControlMode::EStop;
                    self.latched_faults.insert(FaultFlags::TILT_FALL_CUTOFF);
                    // Prefer the orientation latched by the UVC tuck; fall back to
                    // classifying the current sample directly if unavailable.
                    self.fall_orientation = if uvc_out.fall_orientation != fall_orientation::UNKNOWN
                    {
                        uvc_out.fall_orientation
                    } else {
                        classify_fall(&imu).to_wire()
                    };
                } else {
                    // Bumpless ramp into UVC leg targets (instant when actively catching a shove or tucking)
                    let target_blend = 1.0_f32;
                    let ramp_rate = if uvc_out.in_fall_tuck
                        || uvc_out.phase == UvcPhase::ActiveCatchStep
                    {
                        24.0_f32
                    } else {
                        8.0_f32
                    };
                    self.uvc_blend += (target_blend - self.uvc_blend) * (dt * ramp_rate).clamp(0.0, 1.0);
                    let b = self.uvc_blend;
                    let lerp = |a: f32, t: f32| a + (t - a) * b;

                    if has_6dof_legs {
                        desired_rad[J21_L_HIP_YAW] = lerp(desired_rad[J21_L_HIP_YAW], uvc_out.left_leg.hip_yaw);
                        desired_rad[J21_L_HIP_ROLL] = lerp(desired_rad[J21_L_HIP_ROLL], uvc_out.left_leg.hip_roll);
                        desired_rad[J21_L_HIP_PITCH] = lerp(desired_rad[J21_L_HIP_PITCH], uvc_out.left_leg.hip_pitch);
                        desired_rad[J21_L_KNEE_PITCH] = lerp(desired_rad[J21_L_KNEE_PITCH], uvc_out.left_leg.knee_pitch);
                        desired_rad[J21_L_ANKLE_PITCH] = lerp(desired_rad[J21_L_ANKLE_PITCH], uvc_out.left_leg.ankle_pitch);
                        desired_rad[J21_L_ANKLE_ROLL] = lerp(desired_rad[J21_L_ANKLE_ROLL], uvc_out.left_leg.ankle_roll);

                        desired_rad[J21_R_HIP_YAW] = lerp(desired_rad[J21_R_HIP_YAW], uvc_out.right_leg.hip_yaw);
                        desired_rad[J21_R_HIP_ROLL] = lerp(desired_rad[J21_R_HIP_ROLL], uvc_out.right_leg.hip_roll);
                        desired_rad[J21_R_HIP_PITCH] = lerp(desired_rad[J21_R_HIP_PITCH], uvc_out.right_leg.hip_pitch);
                        desired_rad[J21_R_KNEE_PITCH] = lerp(desired_rad[J21_R_KNEE_PITCH], uvc_out.right_leg.knee_pitch);
                        desired_rad[J21_R_ANKLE_PITCH] = lerp(desired_rad[J21_R_ANKLE_PITCH], uvc_out.right_leg.ankle_pitch);
                        desired_rad[J21_R_ANKLE_ROLL] = lerp(desired_rad[J21_R_ANKLE_ROLL], uvc_out.right_leg.ankle_roll);
                    } else {
                        // Graceful degradation for legacy 17-DoF 4-DoF-leg URDFs
                        desired_rad[J17_L_HIP_PITCH] = lerp(desired_rad[J17_L_HIP_PITCH], uvc_out.left_leg.hip_pitch);
                        desired_rad[J17_L_HIP_ROLL] = lerp(desired_rad[J17_L_HIP_ROLL], uvc_out.left_leg.hip_roll);
                        desired_rad[J17_L_KNEE_PITCH] = lerp(desired_rad[J17_L_KNEE_PITCH], uvc_out.left_leg.knee_pitch);
                        desired_rad[J17_L_ANKLE_PITCH] = lerp(desired_rad[J17_L_ANKLE_PITCH], uvc_out.left_leg.ankle_pitch);

                        desired_rad[J17_R_HIP_PITCH] = lerp(desired_rad[J17_R_HIP_PITCH], uvc_out.right_leg.hip_pitch);
                        desired_rad[J17_R_HIP_ROLL] = lerp(desired_rad[J17_R_HIP_ROLL], uvc_out.right_leg.hip_roll);
                        desired_rad[J17_R_KNEE_PITCH] = lerp(desired_rad[J17_R_KNEE_PITCH], uvc_out.right_leg.knee_pitch);
                        desired_rad[J17_R_ANKLE_PITCH] = lerp(desired_rad[J17_R_ANKLE_PITCH], uvc_out.right_leg.ankle_pitch);
                    }

                    // Waist coupling + upper-body arm counter-spread (`armCont`)
                    desired_rad[J_WAIST_YAW] += uvc_out.waist_yaw * b;
                    desired_rad[J_L_SHOULDER_PITCH] += uvc_out.l_shoulder_pitch_delta * b;
                    desired_rad[J_L_SHOULDER_ROLL] += uvc_out.l_shoulder_roll_delta * b;
                    desired_rad[J_R_SHOULDER_PITCH] += uvc_out.r_shoulder_pitch_delta * b;
                    desired_rad[J_R_SHOULDER_ROLL] += uvc_out.r_shoulder_roll_delta * b;
                }

                balance_telem = BalanceTelemetry {
                    gait_phase_rad: uvc_out.gait_phase_rad,
                    pitch_corr_rad: uvc_out.pitch_corr_rad,
                    roll_corr_rad: uvc_out.roll_corr_rad,
                    balance_active: walk.balance_enabled && !uvc_out.fall_cutoff_ready,
                    walk_active: uvc_out.walk_active && !uvc_out.fall_cutoff_ready,
                    uvc_phase: uvc_out.phase.to_wire(),
                    support_foot: uvc_out.support_foot,
                    foot_dx_m: uvc_out.foot_dx_m,
                    foot_dy_m: uvc_out.foot_dy_m,
                    hip_height_m: uvc_out.hip_height_m,
                    fall_orientation: fall_orientation::UNKNOWN,
                };
            } else {
                self.uvc.reset();
                self.uvc_blend = 0.0;
                balance_telem.uvc_phase = uvc_phase::IDLE;
            }
        } else {
            self.uvc.reset();
            self.uvc_blend = 0.0;
            balance_telem.uvc_phase = uvc_phase::IDLE;
        }

        balance_telem.fall_orientation = self.fall_orientation;

        // 6. Compute torque mask, soft-stopped slew-clamped targets, and explicit physical
        //    torque / velocity ceilings (Safety ALWAYS downstream of balance & host commands).
        let mut torque_state_mask: u32 = 0;
        let mut raw_target_rad = [0.0_f32; MAX_JOINTS];
        let mut cmd_mode_out = [JointCommandMode::Position as u8; MAX_JOINTS];
        let mut raw_vel_out = [0.0_f32; MAX_JOINTS];
        let mut torque_nm_out = [0.0_f32; MAX_JOINTS];
        let mut measured_vel_rad_s = [0.0_f32; MAX_JOINTS];
        for i in 0..self.joint_count {
            let sign = if self.calibration[i].inverted {
                -1.0_f32
            } else {
                1.0_f32
            };
            measured_vel_rad_s[i] = sign * feedback.velocity_rad_s[i];
        }

        if self.active_mode == ControlMode::Drive {
            let valid_mask = if self.joint_count >= 32 {
                u32::MAX
            } else {
                (1u32 << self.joint_count) - 1
            };
            torque_state_mask = self.last_host_torque_mask & valid_mask;
            let speed_scale = (self.last_host_speed_pct as f32) * 0.01_f32;
            let base_slew = if balance_telem.balance_active
                || balance_telem.walk_active
                || balance_telem.uvc_phase == uvc_phase::FALL_TUCK
            {
                BALANCE_SLEW_RAD_PER_SEC
            } else {
                self.base_slew_rad_per_sec
            };
            let max_delta = base_slew * speed_scale * dt;
            let compliance = self.balance_gains.compliance_scale.clamp(0.10_f32, 1.0_f32);
            let ukemi_scale = self
                .balance_gains
                .ukemi_torque_scale
                .clamp(0.05_f32, 0.50_f32);

            for i in 0..self.joint_count {
                let cal = &self.calibration[i];
                let enabled = ((torque_state_mask >> i) & 1) == 1;
                let sign = if cal.inverted { -1.0_f32 } else { 1.0_f32 };
                let max_t = cal.max_torque_nm.max(0.05_f32);
                let max_v = cal.max_velocity_rad_s.max(0.5_f32);
                let mode = JointCommandMode::from_wire(self.last_host_command_mode[i]);
                cmd_mode_out[i] = mode.to_wire();

                if !enabled {
                    self.applied_target_rad[i] = measured_rad[i];
                    raw_vel_out[i] = 0.0;
                    torque_nm_out[i] = 0.0;
                } else {
                    let mut desired = desired_rad[i];
                    if desired < cal.min_rad {
                        desired = cal.min_rad;
                        transient_faults.insert(FaultFlags::LIMIT_CLAMPED);
                    } else if desired > cal.max_rad {
                        desired = cal.max_rad;
                        transient_faults.insert(FaultFlags::LIMIT_CLAMPED);
                    }

                    let current = self.applied_target_rad[i];
                    let diff = desired - current;
                    if diff > max_delta {
                        self.applied_target_rad[i] = current + max_delta;
                    } else if diff < -max_delta {
                        self.applied_target_rad[i] = current - max_delta;
                    } else {
                        self.applied_target_rad[i] = desired;
                    }

                    // Enforce explicit per-joint velocity clamp
                    let req_vel = self.last_host_velocity_rad_s[i];
                    let clamped_vel = req_vel.clamp(-max_v, max_v);
                    raw_vel_out[i] = sign * clamped_vel;

                    // Enforce explicit per-joint torque/current ceiling + software compliance + ukemi
                    let effective_ceiling = if ukemi_active {
                        (max_t * ukemi_scale).min(max_t * compliance)
                    } else {
                        max_t * compliance
                    };
                    let req_t = self.last_host_torque_limit_nm[i];
                    torque_nm_out[i] = match mode {
                        JointCommandMode::Position | JointCommandMode::Velocity => {
                            req_t.clamp(0.0_f32, effective_ceiling)
                        }
                        JointCommandMode::Torque => {
                            sign * req_t.clamp(-effective_ceiling, effective_ceiling)
                        }
                    };
                }
                raw_target_rad[i] =
                    calibrated_to_raw(self.applied_target_rad[i], &self.calibration[i]);
            }
        } else {
            // Teach or EStop: Torque strictly OFF, applied target tracks measured
            for i in 0..self.joint_count {
                self.applied_target_rad[i] = measured_rad[i];
                raw_target_rad[i] = calibrated_to_raw(measured_rad[i], &self.calibration[i]);
                raw_vel_out[i] = 0.0;
                torque_nm_out[i] = 0.0;
            }
        }

        let telemetry = McuToHost {
            protocol_version: PROTOCOL_VERSION,
            seq_ack: self.last_valid_seq,
            uptime_ms: now_ms,
            active_mode: self.active_mode,
            faults: self.latched_faults | transient_faults,
            torque_state_mask,
            measured_rad,
            applied_target_rad: self.applied_target_rad,
            velocity_rad_s: measured_vel_rad_s,
            current_ma: feedback.current_ma,
            temp_c: feedback.temp_c,
            bus_voltage_mv: feedback.bus_voltage_mv,
            imu,
            balance: balance_telem,
            estimator: estimator_telem,
            rx_crc_errors: self.rx_crc_errors,
            rx_version_mismatches: self.rx_version_mismatches,
            i3c_timeout_count: feedback.i3c_timeout_count,
        };

        KernelStepOutput {
            servo_cmd: RawServoCommand {
                torque_mask: torque_state_mask,
                raw_target_rad,
                command_mode: cmd_mode_out,
                raw_velocity_rad_s: raw_vel_out,
                torque_nm: torque_nm_out,
            },
            telemetry,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nostd_trig_matches_std_f32() {
        for deg in -720..=720 {
            let rad = (deg as f32).to_radians();
            assert!(
                (sin_f32(rad) - rad.sin()).abs() < 2e-4,
                "sin mismatch at {deg} deg"
            );
            assert!(
                (cos_f32(rad) - rad.cos()).abs() < 2e-4,
                "cos mismatch at {deg} deg"
            );
        }
    }

    #[test]
    fn watchdog_trips_after_100ms_and_zeroes_torque() {
        let cal = [JointCalibration::default(); 4];
        let mut kernel = DirectServoKernel::new(4, &cal);
        let fb = RawServoFeedback::default();

        let drive_cmd = HostToMcu {
            seq: 1,
            mode: ControlMode::Drive,
            torque_mask: 0b1111,
            speed_limit_pct: 50,
            ..Default::default()
        };

        let out1 = kernel.step(10, 0.01, Some(&drive_cmd), &fb);
        assert_eq!(out1.telemetry.active_mode, ControlMode::Drive);
        assert_eq!(out1.servo_cmd.torque_mask, 0b1111);

        let out2 = kernel.step(110, 0.01, None, &fb);
        assert_eq!(out2.telemetry.active_mode, ControlMode::Drive);

        let out3 = kernel.step(111, 0.01, None, &fb);
        assert_eq!(out3.telemetry.active_mode, ControlMode::EStop);
        assert_eq!(out3.servo_cmd.torque_mask, 0);
        assert!(out3.telemetry.faults.contains(FaultFlags::WATCHDOG_TIMEOUT));

        let reset_cmd = HostToMcu {
            seq: 2,
            mode: ControlMode::Drive,
            torque_mask: 0b1111,
            config: ConfigCmd::ClearFaults,
            ..Default::default()
        };
        let out4 = kernel.step(120, 0.01, Some(&reset_cmd), &fb);
        assert_eq!(out4.telemetry.active_mode, ControlMode::Teach);
        assert_eq!(out4.servo_cmd.torque_mask, 0);
        assert!(!out4.telemetry.faults.contains(FaultFlags::WATCHDOG_TIMEOUT));
    }

    #[test]
    fn slew_rate_clamps_90_degree_step_at_20_percent() {
        let cal = [JointCalibration {
            min_rad: -2.0,
            max_rad: 2.0,
            ..Default::default()
        }; 2];
        let mut kernel = DirectServoKernel::new(2, &cal);
        let fb = RawServoFeedback::default();

        let mut cmd = HostToMcu {
            seq: 1,
            mode: ControlMode::Drive,
            torque_mask: 0b01,
            speed_limit_pct: 20,
            ..Default::default()
        };
        cmd.target_rad[0] = core::f32::consts::FRAC_PI_2;

        // First tick dt = 0.05s -> max_delta = 2.0 * 0.20 * 0.05 = 0.02 rad
        let out = kernel.step(50, 0.05, Some(&cmd), &fb);
        assert!((out.telemetry.applied_target_rad[0] - 0.02_f32).abs() < 1e-5);
    }

    #[test]
    fn soft_limits_clamp_and_flag_advisory() {
        let cal = [JointCalibration {
            min_rad: -0.5,
            max_rad: 0.5,
            ..Default::default()
        }; 1];
        let mut kernel = DirectServoKernel::new(1, &cal);
        let fb = RawServoFeedback::default();

        let mut cmd = HostToMcu {
            seq: 1,
            mode: ControlMode::Drive,
            torque_mask: 1,
            speed_limit_pct: 100,
            ..Default::default()
        };
        cmd.target_rad[0] = 1.5;

        let mut last = kernel.step(0, 0.1, Some(&cmd), &fb);
        for tick in 1..20 {
            cmd.seq = tick + 1;
            last = kernel.step(tick * 50, 0.05, Some(&cmd), &fb);
        }
        assert!((last.telemetry.applied_target_rad[0] - 0.5_f32).abs() < 1e-5);
        assert!(last.telemetry.faults.contains(FaultFlags::LIMIT_CLAMPED));
    }

    #[test]
    fn uvc_balance_steps_to_catch_tilt_and_tucks_before_fall_cutoff() {
        let cal = [JointCalibration {
            min_rad: -2.2,
            max_rad: 2.2,
            ..Default::default()
        }; 21];
        let mut kernel = DirectServoKernel::new(21, &cal);

        let mut fb = RawServoFeedback::default();
        fb.imu.pitch_rad = 0.22; // Leaning forward ~12.6 deg
        fb.imu.roll_rad = -0.15; // Leaning left -> Left leg catches

        let mut cmd = HostToMcu {
            seq: 1,
            mode: ControlMode::Drive,
            torque_mask: (1u32 << 21) - 1,
            speed_limit_pct: 100,
            walk: WalkCmd {
                walk_enabled: false,
                balance_enabled: true,
                ..Default::default()
            },
            ..Default::default()
        };

        let mut out = kernel.step(10, 0.01, Some(&cmd), &fb);
        for tick in 2..=12 {
            cmd.seq = tick;
            out = kernel.step(tick * 10, 0.01, Some(&cmd), &fb);
        }
        assert_eq!(out.telemetry.active_mode, ControlMode::Drive);
        assert!(out.telemetry.balance.balance_active);
        assert_eq!(out.telemetry.balance.uvc_phase, uvc_phase::ACTIVE_CATCH_STEP);
        assert_eq!(out.telemetry.balance.support_foot, 1, "left foot catches -roll lean");
        assert!(out.telemetry.balance.foot_dx_m > 0.02, "foot must step forward under forward tilt");
        assert!(out.telemetry.balance.foot_dy_m < -0.01, "left foot must step left under -roll");

        // Exceed fall cutoff -> protective FallTuck then EStop with TILT_FALL_CUTOFF
        fb.imu.pitch_rad = 0.90;
        let mut fall = out;
        for tick in 13..=25 {
            cmd.seq = tick;
            fall = kernel.step(tick * 10, 0.01, Some(&cmd), &fb);
        }
        assert_eq!(fall.telemetry.active_mode, ControlMode::EStop);
        assert_eq!(fall.servo_cmd.torque_mask, 0);
        assert!(fall.telemetry.faults.contains(FaultFlags::TILT_FALL_CUTOFF));
    }

    #[test]
    fn fall_cutoff_captures_classified_orientation_in_telemetry() {
        let cal = [JointCalibration {
            min_rad: -2.2,
            max_rad: 2.2,
            ..Default::default()
        }; 21];

        let cases: [(f32, f32, u8); 4] = [
            (1.0, 0.0, fall_orientation::PRONE),
            (-1.0, 0.0, fall_orientation::SUPINE),
            (0.0, 1.0, fall_orientation::SIDE_RIGHT),
            (0.0, -1.0, fall_orientation::SIDE_LEFT),
        ];

        for (pitch, roll, expected) in cases {
            let mut kernel = DirectServoKernel::new(21, &cal);
            let mut fb = RawServoFeedback::default();
            fb.imu = ImuState {
                available: true,
                pitch_rad: pitch,
                roll_rad: roll,
                gravity: [0.0, -9.81, 0.0],
                ..Default::default()
            };
            let mut cmd = HostToMcu {
                mode: ControlMode::Drive,
                torque_mask: (1u32 << 21) - 1,
                walk: WalkCmd {
                    balance_enabled: true,
                    ..Default::default()
                },
                ..Default::default()
            };

            let mut out = kernel.step(10, 0.01, Some(&cmd), &fb);
            for tick in 2..=20 {
                cmd.seq = tick;
                out = kernel.step(tick * 10, 0.01, Some(&cmd), &fb);
            }
            assert!(
                out.telemetry.faults.contains(FaultFlags::TILT_FALL_CUTOFF),
                "pitch={pitch} roll={roll} must latch a fall"
            );
            assert_eq!(
                out.telemetry.balance.fall_orientation, expected,
                "pitch={pitch} roll={roll} orientation mismatch"
            );
        }
    }

    #[test]
    fn uvc_walking_alternates_legs_and_advances_phase() {
        let cal = [JointCalibration {
            min_rad: -2.2,
            max_rad: 2.2,
            ..Default::default()
        }; 21];
        let mut kernel = DirectServoKernel::new(21, &cal);
        let fb = RawServoFeedback::default();
        let mut cmd = HostToMcu {
            seq: 1,
            mode: ControlMode::Drive,
            torque_mask: (1u32 << 21) - 1,
            speed_limit_pct: 100,
            walk: WalkCmd {
                walk_enabled: true,
                balance_enabled: true,
                vx: 0.85,
                ..Default::default()
            },
            ..Default::default()
        };

        let mut last = kernel.step(10, 0.01, Some(&cmd), &fb);
        let mut max_knee_diff = 0.0_f32;
        for tick in 2..=60 {
            cmd.seq = tick;
            last = kernel.step(tick * 10, 0.01, Some(&cmd), &fb);
            let diff = (last.telemetry.applied_target_rad[J21_L_KNEE_PITCH]
                - last.telemetry.applied_target_rad[J21_R_KNEE_PITCH])
                .abs();
            max_knee_diff = max_knee_diff.max(diff);
        }
        assert!(last.telemetry.balance.walk_active);
        assert!(last.telemetry.balance.gait_phase_rad > 0.2);
        assert!(
            max_knee_diff > 0.08,
            "left and right knees must alternate during UVC walking (max_diff={max_knee_diff})"
        );
    }

    #[test]
    fn calibration_offset_and_invert_round_trip() {
        let cal = JointCalibration {
            zero_offset_rad: 0.35,
            inverted: true,
            min_rad: -1.0,
            max_rad: 1.0,
            ..Default::default()
        };
        let raw = 0.85_f32;
        let cal_val = raw_to_calibrated(raw, &cal);
        assert!((cal_val - (-0.50_f32)).abs() < 1e-6);
        assert!((calibrated_to_raw(cal_val, &cal) - raw).abs() < 1e-6);
    }

    #[test]
    fn kinematic_dcm_triggers_step_outside_2_8cm_margin() {
        let cal = [JointCalibration {
            min_rad: -2.2,
            max_rad: 2.2,
            ..Default::default()
        }; 21];
        let mut kernel = DirectServoKernel::new(21, &cal);
        let mut cmd = HostToMcu {
            seq: 1,
            mode: ControlMode::Drive,
            torque_mask: (1u32 << 21) - 1,
            speed_limit_pct: 100,
            walk: WalkCmd {
                balance_enabled: true,
                ..Default::default()
            },
            ..Default::default()
        };

        // Quiet upright stance -> DCM inside ±2.8 cm foot margin
        let mut fb = RawServoFeedback::default();
        let quiet = kernel.step(5, 0.005, Some(&cmd), &fb);
        assert!(
            quiet.telemetry.estimator.foot_margin_m > 0.020,
            "upright margin must be near +0.028 m, got {}",
            quiet.telemetry.estimator.foot_margin_m
        );

        // Apply forward pitch velocity impulse while angle is still small (inside static deadband)
        // -> Capture Point xi = c + c_dot / omega leaves ±2.8 cm and triggers ActiveCatchStep.
        fb.imu.pitch_rad = 0.025;
        fb.imu.pitch_rate_rad_s = 1.6;
        let mut pushed = quiet;
        for tick in 2..=12 {
            cmd.seq = tick;
            pushed = kernel.step(tick * 5, 0.005, Some(&cmd), &fb);
        }
        assert!(
            pushed.telemetry.estimator.foot_margin_m < 0.0,
            "DCM must leave ±2.8 cm margin under forward velocity impulse (margin={})",
            pushed.telemetry.estimator.foot_margin_m
        );
        assert_eq!(
            pushed.telemetry.balance.uvc_phase,
            uvc_phase::ACTIVE_CATCH_STEP,
            "DCM breach must trigger ActiveCatchStep"
        );
    }

    #[test]
    fn ukemi_drops_torque_ceiling_before_fall_cutoff() {
        let cal = [JointCalibration {
            min_rad: -2.2,
            max_rad: 2.2,
            max_torque_nm: 1.8,
            ..Default::default()
        }; 21];
        let mut kernel = DirectServoKernel::new(21, &cal);
        let mut cmd = HostToMcu {
            seq: 1,
            mode: ControlMode::Drive,
            torque_mask: (1u32 << 21) - 1,
            torque_limit_nm: [1.8; MAX_JOINTS],
            walk: WalkCmd {
                balance_enabled: true,
                ..Default::default()
            },
            ..Default::default()
        };

        // Nominal stance -> full torque ceiling (1.8 N·m)
        let mut fb = RawServoFeedback::default();
        let nom = kernel.step(10, 0.01, Some(&cmd), &fb);
        assert!(!nom.telemetry.estimator.ukemi_active);
        assert!((nom.servo_cmd.torque_nm[11] - 1.8).abs() < 1e-4);

        // Tilt crosses ukemi_tilt_rad (0.56 rad ~32 deg) while below fall_cutoff_rad (0.65 rad)
        fb.imu.pitch_rad = 0.60;
        cmd.seq = 2;
        let ukemi = kernel.step(20, 0.01, Some(&cmd), &fb);
        assert_eq!(ukemi.telemetry.active_mode, ControlMode::Drive);
        assert!(ukemi.telemetry.estimator.ukemi_active);
        let expected_ceiling = 1.8 * BalanceGains::default().ukemi_torque_scale;
        assert!(
            ukemi.servo_cmd.torque_nm[11] <= expected_ceiling + 1e-4,
            "ukemi must soften torque ceiling to <= {expected_ceiling}, got {}",
            ukemi.servo_cmd.torque_nm[11]
        );
    }

    #[test]
    fn physical_torque_and_velocity_clamps_enforced() {
        let cal = [JointCalibration {
            max_torque_nm: 1.2,
            max_velocity_rad_s: 5.0,
            ..Default::default()
        }; 4];
        let mut kernel = DirectServoKernel::new(4, &cal);
        let fb = RawServoFeedback::default();

        let mut cmd = HostToMcu {
            seq: 1,
            mode: ControlMode::Drive,
            torque_mask: 0b1111,
            ..Default::default()
        };
        cmd.command_mode[0] = JointCommandMode::Position.to_wire();
        cmd.torque_limit_nm[0] = 99.0; // Oversized torque limit -> clamped to 1.2
        cmd.velocity_rad_s[0] = 40.0; // Oversized velocity -> clamped to 5.0
        cmd.command_mode[1] = JointCommandMode::Torque.to_wire();
        cmd.torque_limit_nm[1] = -99.0; // Negative torque command -> clamped to -1.2

        let out = kernel.step(10, 0.01, Some(&cmd), &fb);
        assert!((out.servo_cmd.torque_nm[0] - 1.2).abs() < 1e-5);
        assert!((out.servo_cmd.raw_velocity_rad_s[0] - 5.0).abs() < 1e-5);
        assert!((out.servo_cmd.torque_nm[1] - (-1.2)).abs() < 1e-5);
    }

    #[test]
    fn v1_valid_mask_marks_grf_and_momentum_unavailable() {
        let cal = [JointCalibration::default(); 21];
        let mut kernel = DirectServoKernel::new(21, &cal);
        // COM/DCM are IMU-derived: provide a live sensor with a valid source tag so the
        // estimator is allowed to publish those channels for this validity-mask test.
        let mut fb = RawServoFeedback::default();
        fb.imu.available = true;
        fb.imu_source_mask = rk_proto::estimator_valid::LSM6DSV_OK;
        let cmd = HostToMcu {
            seq: 1,
            mode: ControlMode::Teach,
            ..Default::default()
        };
        let out = kernel.step(10, 0.01, Some(&cmd), &fb);
        let mask = out.telemetry.estimator.valid_mask;
        assert_ne!(mask & rk_proto::estimator_valid::COM, 0);
        assert_ne!(mask & rk_proto::estimator_valid::DCM, 0);
        assert_ne!(mask & rk_proto::estimator_valid::LSM6DSV_OK, 0);
        assert_eq!(mask & rk_proto::estimator_valid::GRF, 0);
        assert_eq!(mask & rk_proto::estimator_valid::MOMENTUM, 0);
    }

    #[test]
    fn imu_outage_invalidates_com_dcm_until_sensor_returns() {
        let cal = [JointCalibration::default(); 21];
        let mut kernel = DirectServoKernel::new(21, &cal);
        let cmd = HostToMcu {
            seq: 1,
            mode: ControlMode::Teach,
            ..Default::default()
        };
        let mut fb = RawServoFeedback::default();
        // Sensor absent: no channel derived from live attitude may be marked valid.
        fb.imu.available = false;
        let outage = kernel.step(10, 0.01, Some(&cmd), &fb);
        assert_eq!(outage.telemetry.estimator.valid_mask, 0);

        // Sensor returns with a BNO055 source tag: COM/DCM and that source become valid.
        fb.imu.available = true;
        fb.imu_source_mask = rk_proto::estimator_valid::BNO055_OK;
        let live = kernel.step(11, 0.01, Some(&cmd), &fb);
        let mask = live.telemetry.estimator.valid_mask;
        assert_ne!(mask & rk_proto::estimator_valid::COM, 0);
        assert_ne!(mask & rk_proto::estimator_valid::DCM, 0);
        assert_ne!(mask & rk_proto::estimator_valid::BNO055_OK, 0);
        assert_eq!(mask & rk_proto::estimator_valid::LSM6DSV_OK, 0);
        assert_eq!(mask & rk_proto::estimator_valid::GRF, 0);
        assert_eq!(mask & rk_proto::estimator_valid::MOMENTUM, 0);
    }
}
