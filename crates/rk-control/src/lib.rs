#![cfg_attr(not(feature = "std"), no_std)]
//! Layer 1 — Pure `#![no_std]` Safety, Walking & Vertical Balance Kernel (`rk-control`).
//!
//! Enforces:
//! - 100 ms deadman watchdog (`WATCHDOG_TIMEOUT_MS`)
//! - Per-joint software stops (`[min_rad, max_rad]`)
//! - Slew-rate velocity clamp (`omega_base * speed_limit_pct / 100 * dt`)
//! - Calibration zero-offset & sign-inversion mapping
//! - Bumpless `Teach -> Drive` target handoff & safe Reset-to-Teach invariant
//! - MCU-native bipedal walking gait oscillator (`WalkCmd`)
//! - MCU-native 2-axis IMU upper-body vertical stabilization & tilt fall-cutoff latch
//!
//! Strictly single-precision `f32` everywhere for RP2350 Cortex-M33 FPU numeric parity.

use core::f32::consts::{FRAC_PI_2, PI, TAU};

use rk_proto::{
    BalanceGains, BalanceTelemetry, ConfigCmd, ControlMode, FaultFlags, HostToMcu, ImuState,
    MAX_JOINTS, McuToHost, WATCHDOG_TIMEOUT_MS, WalkCmd,
};
use serde::{Deserialize, Serialize};

pub const DEFAULT_BASE_SLEW_RAD_PER_SEC: f32 = 2.0;
pub const BALANCE_SLEW_RAD_PER_SEC: f32 = 4.2;
pub const OVER_CURRENT_TRIP_MA: u16 = 1950;
pub const OVER_TEMP_TRIP_C: u8 = 78;
pub const UNDER_VOLTAGE_TRIP_MV: u16 = 10_500;

// Standard Rocky 17-DoF humanoid joint indices (matches assets/sample_humanoid/rocky_mk1.urdf).
const J_WAIST_YAW: usize = 0;
const J_L_SHOULDER_PITCH: usize = 3;
const J_L_SHOULDER_ROLL: usize = 4;
const J_R_SHOULDER_PITCH: usize = 6;
const J_R_SHOULDER_ROLL: usize = 7;
const J_L_HIP_PITCH: usize = 9;
const J_L_HIP_ROLL: usize = 10;
const J_L_KNEE_PITCH: usize = 11;
const J_L_ANKLE_PITCH: usize = 12;
const J_R_HIP_PITCH: usize = 13;
const J_R_HIP_ROLL: usize = 14;
const J_R_KNEE_PITCH: usize = 15;
const J_R_ANKLE_PITCH: usize = 16;

/// Deterministic `#![no_std]` single-precision sine accurate to < 2e-5 rad over all angles.
#[inline]
pub fn sin_f32(mut x: f32) -> f32 {
    if !x.is_finite() {
        return 0.0;
    }
    // Range-reduce to [-PI, PI]
    let k = ((x + PI) / TAU) as i32;
    x -= (k as f32) * TAU;
    if x > PI {
        x -= TAU;
    } else if x < -PI {
        x += TAU;
    }
    // Fold into [-PI/2, PI/2]
    if x > FRAC_PI_2 {
        x = PI - x;
    } else if x < -FRAC_PI_2 {
        x = -PI - x;
    }
    // 7th-order minimax/Taylor polynomial on [-PI/2, PI/2]
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

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub struct JointCalibration {
    pub zero_offset_rad: f32,
    pub inverted: bool,
    pub min_rad: f32,
    pub max_rad: f32,
}

impl Default for JointCalibration {
    fn default() -> Self {
        Self {
            zero_offset_rad: 0.0,
            inverted: false,
            min_rad: -1.57,
            max_rad: 1.57,
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub struct RawServoFeedback {
    pub raw_angle_rad: [f32; MAX_JOINTS],
    pub current_ma: [u16; MAX_JOINTS],
    pub temp_c: [u8; MAX_JOINTS],
    pub bus_voltage_mv: u16,
    pub bus_error: bool,
    pub imu: ImuState,
}

impl Default for RawServoFeedback {
    fn default() -> Self {
        Self {
            raw_angle_rad: [0.0; MAX_JOINTS],
            current_ma: [0; MAX_JOINTS],
            temp_c: [28; MAX_JOINTS],
            bus_voltage_mv: 12_400,
            bus_error: false,
            imu: ImuState::default(),
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub struct RawServoCommand {
    pub torque_mask: u32,
    pub raw_target_rad: [f32; MAX_JOINTS],
}

impl Default for RawServoCommand {
    fn default() -> Self {
        Self {
            torque_mask: 0,
            raw_target_rad: [0.0; MAX_JOINTS],
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
    applied_target_rad: [f32; MAX_JOINTS],
    initialized_targets: bool,
    flash_commit_count: u32,
    // MCU-native walking oscillator & upper-body vertical balance state
    balance_gains: BalanceGains,
    last_walk_cmd: WalkCmd,
    gait_phase_rad: f32,
    walk_blend: f32,
    pitch_int: f32,
    roll_int: f32,
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
            applied_target_rad: [0.0; MAX_JOINTS],
            initialized_targets: false,
            flash_commit_count: 0,
            balance_gains: BalanceGains::default(),
            last_walk_cmd: WalkCmd::default(),
            gait_phase_rad: 0.0,
            walk_blend: 0.0,
            pitch_int: 0.0,
            roll_int: 0.0,
        }
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

            // Apply configuration command
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
                    if g.kp_pitch.is_finite()
                        && g.kd_pitch.is_finite()
                        && g.kp_roll.is_finite()
                        && g.kd_roll.is_finite()
                        && g.ankle_ratio.is_finite()
                        && g.arm_assist.is_finite()
                        && g.fall_cutoff_rad.is_finite()
                    {
                        self.balance_gains = BalanceGains {
                            kp_pitch: g.kp_pitch.clamp(0.0, 4.0),
                            kd_pitch: g.kd_pitch.clamp(0.0, 1.5),
                            kp_roll: g.kp_roll.clamp(0.0, 4.0),
                            kd_roll: g.kd_roll.clamp(0.0, 1.5),
                            ankle_ratio: g.ankle_ratio.clamp(0.0, 1.0),
                            arm_assist: g.arm_assist.clamp(0.0, 1.5),
                            fall_cutoff_rad: g.fall_cutoff_rad.clamp(0.30, 1.35),
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
                    self.pitch_int = 0.0;
                    self.roll_int = 0.0;
                    self.walk_blend = 0.0;
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
            }

            if cmd.mode == ControlMode::EStop {
                self.active_mode = ControlMode::EStop;
                self.latched_faults.insert(FaultFlags::ESTOP_ACTIVE);
            } else if !cleared_faults_this_tick && self.active_mode != ControlMode::EStop {
                if cmd.mode == ControlMode::Drive && self.active_mode == ControlMode::Teach {
                    // Enforce MCU-side bumpless sync on Teach -> Drive
                    self.applied_target_rad = measured_rad;
                    self.pitch_int = 0.0;
                    self.roll_int = 0.0;
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

        // 4. Evaluate hardware health advisories & tilt fall-cutoff latch
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

        let imu = feedback.imu;
        let tilt_exceeded = imu.pitch_rad.abs() > self.balance_gains.fall_cutoff_rad
            || imu.roll_rad.abs() > self.balance_gains.fall_cutoff_rad;
        if self.active_mode == ControlMode::Drive && self.joint_count >= 17 && tilt_exceeded {
            self.active_mode = ControlMode::EStop;
            self.latched_faults.insert(FaultFlags::TILT_FALL_CUTOFF);
        }

        // 5. Compute MCU-native walking gait + upper-body vertical balance overlay
        let mut desired_rad = self.last_host_target_rad;
        let mut balance_telem = BalanceTelemetry {
            gait_phase_rad: self.gait_phase_rad,
            pitch_corr_rad: 0.0,
            roll_corr_rad: 0.0,
            balance_active: false,
            walk_active: false,
        };

        if self.active_mode == ControlMode::Drive && self.joint_count >= 17 {
            let walk = self.last_walk_cmd;
            let vx = walk.vx.clamp(-1.0, 1.0);
            let vy = walk.vy.clamp(-1.0, 1.0);
            let yaw = walk.yaw_rate.clamp(-1.0, 1.0);
            let crouch = walk.stance_crouch.clamp(0.0, 1.0);
            let step_h = walk.step_height.clamp(0.1, 1.0);
            let cadence = walk.cadence_hz.clamp(0.6, 2.8);

            // Smooth bumpless ramp for walk mode engagement
            let target_blend = if walk.walk_enabled { 1.0_f32 } else { 0.0_f32 };
            let blend_rate = (dt * 4.5_f32).clamp(0.0, 1.0);
            self.walk_blend += (target_blend - self.walk_blend) * blend_rate;

            let speed_mag = (vx * vx + vy * vy + yaw * yaw).min(1.0);
            if walk.walk_enabled && speed_mag > 0.002 {
                self.gait_phase_rad += TAU * cadence * dt;
                if self.gait_phase_rad >= TAU {
                    self.gait_phase_rad -= TAU;
                }
            } else if self.walk_blend > 0.01 {
                // Gently return gait phase to double-support stance (0 or PI) when stationary
                let dist_to_zero = if self.gait_phase_rad > PI {
                    self.gait_phase_rad - TAU
                } else {
                    self.gait_phase_rad
                };
                self.gait_phase_rad -= dist_to_zero * (dt * 6.0_f32).clamp(0.0, 1.0);
                if self.gait_phase_rad < 0.0 {
                    self.gait_phase_rad += TAU;
                }
            }

            // Stage A: Phase-locked bipedal walk oscillator (blended cleanly onto legs + subtle counter-swing)
            if self.walk_blend > 0.005 {
                let b = self.walk_blend;
                let motion_scale = if speed_mag > 0.002 {
                    (speed_mag * 2.2_f32).min(1.0)
                } else {
                    0.0_f32
                };

                let base_hip = -(0.12_f32 + crouch * 0.36_f32);
                let base_knee = 2.0_f32 * (-base_hip);
                let base_ankle = base_hip;

                let phi_l = self.gait_phase_rad;
                let phi_r = self.gait_phase_rad + PI;

                let sl = sin_f32(phi_l);
                let cl = cos_f32(phi_l);
                let sr = sin_f32(phi_r);
                let cr = cos_f32(phi_r);

                // Smooth cycloidal swing foot lift (only positive half-wave)
                let lift_l = if sl > 0.0 { sl * sl } else { 0.0 } * step_h * motion_scale;
                let lift_r = if sr > 0.0 { sr * sr } else { 0.0 } * step_h * motion_scale;

                let stride_amp = 0.34_f32 * vx + 0.10_f32 * yaw;
                let stride_amp_r = 0.34_f32 * vx - 0.10_f32 * yaw;

                let l_hip_pitch = base_hip - lift_l * 0.38_f32 - cl * stride_amp;
                let l_knee_pitch = base_knee + lift_l * 0.72_f32 + (cl * stride_amp).abs() * 0.12;
                let l_ankle_pitch = base_ankle - lift_l * 0.30_f32 + cl * stride_amp * 0.65;

                let r_hip_pitch = base_hip - lift_r * 0.38_f32 - cr * stride_amp_r;
                let r_knee_pitch = base_knee + lift_r * 0.72_f32 + (cr * stride_amp_r).abs() * 0.12;
                let r_ankle_pitch = base_ankle - lift_r * 0.30_f32 + cr * stride_amp_r * 0.65;

                // Natural lateral weight transfer + strafe command on hip roll
                let weight_shift = sl * 0.095_f32 * motion_scale;
                let strafe_l = 0.04_f32 + weight_shift + vy * (0.14_f32 + 0.10_f32 * sl);
                let strafe_r = -0.04_f32 + weight_shift + vy * (0.14_f32 - 0.10_f32 * sr);

                let lerp = |a: f32, target: f32| a + (target - a) * b;
                desired_rad[J_L_HIP_PITCH] = lerp(desired_rad[J_L_HIP_PITCH], l_hip_pitch);
                desired_rad[J_L_HIP_ROLL] = lerp(desired_rad[J_L_HIP_ROLL], strafe_l);
                desired_rad[J_L_KNEE_PITCH] = lerp(desired_rad[J_L_KNEE_PITCH], l_knee_pitch);
                desired_rad[J_L_ANKLE_PITCH] = lerp(desired_rad[J_L_ANKLE_PITCH], l_ankle_pitch);

                desired_rad[J_R_HIP_PITCH] = lerp(desired_rad[J_R_HIP_PITCH], r_hip_pitch);
                desired_rad[J_R_HIP_ROLL] = lerp(desired_rad[J_R_HIP_ROLL], strafe_r);
                desired_rad[J_R_KNEE_PITCH] = lerp(desired_rad[J_R_KNEE_PITCH], r_knee_pitch);
                desired_rad[J_R_ANKLE_PITCH] = lerp(desired_rad[J_R_ANKLE_PITCH], r_ankle_pitch);

                // Subtle torso & arm counter-swing when actively walking
                if motion_scale > 0.01 {
                    let waist_swing = cl * vx * 0.16_f32 + yaw * 0.22_f32;
                    let arm_swing = cl * vx * 0.26_f32;
                    desired_rad[J_WAIST_YAW] += waist_swing * b;
                    desired_rad[J_L_SHOULDER_PITCH] += arm_swing * b;
                    desired_rad[J_R_SHOULDER_PITCH] -= arm_swing * b;
                }
                balance_telem.walk_active = walk.walk_enabled;
            }

            // Stage B: MCU-Native 2-Axis Upper-Body Vertical Stabilizer (IMU PD + Anti-Windup)
            if walk.balance_enabled {
                let g = self.balance_gains;
                let pitch_err = -imu.pitch_rad;
                let roll_err = -imu.roll_rad;

                self.pitch_int = (self.pitch_int + pitch_err * dt).clamp(-0.14_f32, 0.14_f32);
                self.roll_int = (self.roll_int + roll_err * dt).clamp(-0.14_f32, 0.14_f32);

                let pitch_corr = (g.kp_pitch * pitch_err - g.kd_pitch * imu.pitch_rate_rad_s
                    + 0.28_f32 * self.pitch_int)
                    .clamp(-0.60_f32, 0.60_f32);
                let roll_corr = (g.kp_roll * roll_err - g.kd_roll * imu.roll_rate_rad_s
                    + 0.28_f32 * self.roll_int)
                    .clamp(-0.48_f32, 0.48_f32);

                let hip_share = 1.0_f32 - 0.50_f32 * g.ankle_ratio;
                let ankle_share = 0.70_f32 * g.ankle_ratio;
                let knee_comp = 0.16_f32 * pitch_corr.abs();

                // Pitch stabilization: hip-pitch uprighting + ankle strategy + slight knee compliance
                desired_rad[J_L_HIP_PITCH] += pitch_corr * hip_share;
                desired_rad[J_R_HIP_PITCH] += pitch_corr * hip_share;
                desired_rad[J_L_ANKLE_PITCH] += pitch_corr * ankle_share;
                desired_rad[J_R_ANKLE_PITCH] += pitch_corr * ankle_share;
                desired_rad[J_L_KNEE_PITCH] += knee_comp;
                desired_rad[J_R_KNEE_PITCH] += knee_comp;

                // Roll stabilization: hip-roll lateral counter-tilt + upper-body arm windmill assist
                desired_rad[J_L_HIP_ROLL] += roll_corr;
                desired_rad[J_R_HIP_ROLL] += roll_corr;

                if g.arm_assist > 0.01 {
                    let arm_roll = roll_corr * g.arm_assist * 1.15_f32;
                    let arm_pitch = pitch_corr * g.arm_assist * 0.70_f32;
                    desired_rad[J_L_SHOULDER_ROLL] += arm_roll;
                    desired_rad[J_R_SHOULDER_ROLL] += arm_roll;
                    desired_rad[J_L_SHOULDER_PITCH] -= arm_pitch;
                    desired_rad[J_R_SHOULDER_PITCH] -= arm_pitch;
                }

                balance_telem.pitch_corr_rad = pitch_corr;
                balance_telem.roll_corr_rad = roll_corr;
                balance_telem.balance_active = true;
            } else {
                self.pitch_int = 0.0;
                self.roll_int = 0.0;
            }
            balance_telem.gait_phase_rad = self.gait_phase_rad;
        } else {
            self.pitch_int = 0.0;
            self.roll_int = 0.0;
            self.walk_blend = 0.0;
        }

        // 6. Compute torque mask & soft-stopped, slew-clamped servo targets (Safety always downstream)
        let mut torque_state_mask: u32 = 0;
        let mut raw_target_rad = [0.0_f32; MAX_JOINTS];

        if self.active_mode == ControlMode::Drive {
            let valid_mask = if self.joint_count >= 32 {
                u32::MAX
            } else {
                (1u32 << self.joint_count) - 1
            };
            torque_state_mask = self.last_host_torque_mask & valid_mask;
            let speed_scale = (self.last_host_speed_pct as f32) * 0.01_f32;
            let base_slew = if balance_telem.balance_active || balance_telem.walk_active {
                BALANCE_SLEW_RAD_PER_SEC
            } else {
                self.base_slew_rad_per_sec
            };
            let max_delta = base_slew * speed_scale * dt;

            for i in 0..self.joint_count {
                let enabled = ((torque_state_mask >> i) & 1) == 1;
                if !enabled {
                    // Track measured while joint torque is off
                    self.applied_target_rad[i] = measured_rad[i];
                } else {
                    let cal = &self.calibration[i];
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
                }
                raw_target_rad[i] =
                    calibrated_to_raw(self.applied_target_rad[i], &self.calibration[i]);
            }
        } else {
            // Teach or EStop: Torque strictly OFF, applied target tracks measured
            for i in 0..self.joint_count {
                self.applied_target_rad[i] = measured_rad[i];
                raw_target_rad[i] = calibrated_to_raw(measured_rad[i], &self.calibration[i]);
            }
        }

        let telemetry = McuToHost {
            seq_ack: self.last_valid_seq,
            uptime_ms: now_ms,
            active_mode: self.active_mode,
            faults: self.latched_faults | transient_faults,
            torque_state_mask,
            measured_rad,
            applied_target_rad: self.applied_target_rad,
            current_ma: feedback.current_ma,
            temp_c: feedback.temp_c,
            bus_voltage_mv: feedback.bus_voltage_mv,
            imu,
            balance: balance_telem,
        };

        KernelStepOutput {
            servo_cmd: RawServoCommand {
                torque_mask: torque_state_mask,
                raw_target_rad,
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
    fn upper_body_vertical_balance_corrects_tilt_and_latches_fall_cutoff() {
        let cal = [JointCalibration {
            min_rad: -2.0,
            max_rad: 2.0,
            ..Default::default()
        }; 17];
        let mut kernel = DirectServoKernel::new(17, &cal);

        let mut fb = RawServoFeedback::default();
        fb.imu.pitch_rad = 0.22; // Leaning forward ~12.6 deg
        fb.imu.roll_rad = -0.15;

        let cmd = HostToMcu {
            seq: 1,
            mode: ControlMode::Drive,
            torque_mask: (1u32 << 17) - 1,
            speed_limit_pct: 100,
            walk: WalkCmd {
                walk_enabled: false,
                balance_enabled: true,
                ..Default::default()
            },
            ..Default::default()
        };

        let out = kernel.step(10, 0.01, Some(&cmd), &fb);
        assert_eq!(out.telemetry.active_mode, ControlMode::Drive);
        assert!(out.telemetry.balance.balance_active);
        assert!(out.telemetry.balance.pitch_corr_rad < -0.1, "must counter forward pitch");
        assert!(out.telemetry.balance.roll_corr_rad > 0.05, "must counter negative roll");
        assert!(out.telemetry.applied_target_rad[J_L_HIP_PITCH] < 0.0);
        assert!(out.telemetry.applied_target_rad[J_L_HIP_ROLL] > 0.0);

        // Exceed 42 deg fall cutoff -> latches TILT_FALL_CUTOFF & forces EStop with zero torque
        fb.imu.pitch_rad = 0.90;
        let mut cmd2 = cmd;
        cmd2.seq = 2;
        let fall = kernel.step(20, 0.01, Some(&cmd2), &fb);
        assert_eq!(fall.telemetry.active_mode, ControlMode::EStop);
        assert_eq!(fall.servo_cmd.torque_mask, 0);
        assert!(fall.telemetry.faults.contains(FaultFlags::TILT_FALL_CUTOFF));
    }

    #[test]
    fn walking_oscillator_advances_phase_and_drives_alternating_legs() {
        let cal = [JointCalibration {
            min_rad: -2.2,
            max_rad: 2.2,
            ..Default::default()
        }; 17];
        let mut kernel = DirectServoKernel::new(17, &cal);
        let fb = RawServoFeedback::default();
        let mut cmd = HostToMcu {
            seq: 1,
            mode: ControlMode::Drive,
            torque_mask: (1u32 << 17) - 1,
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
        for tick in 2..=60 {
            cmd.seq = tick;
            last = kernel.step(tick * 10, 0.01, Some(&cmd), &fb);
        }
        assert!(last.telemetry.balance.walk_active);
        assert!(last.telemetry.balance.gait_phase_rad > 0.5);
        assert!(
            (last.telemetry.applied_target_rad[J_L_KNEE_PITCH]
                - last.telemetry.applied_target_rad[J_R_KNEE_PITCH])
                .abs()
                > 0.05,
            "left and right knees must alternate during walking"
        );
    }

    #[test]
    fn calibration_offset_and_invert_round_trip() {
        let cal = JointCalibration {
            zero_offset_rad: 0.35,
            inverted: true,
            min_rad: -1.0,
            max_rad: 1.0,
        };
        let raw = 0.85_f32;
        let cal_val = raw_to_calibrated(raw, &cal);
        assert!((cal_val - (-0.50_f32)).abs() < 1e-6);
        assert!((calibrated_to_raw(cal_val, &cal) - raw).abs() < 1e-6);
    }
}
