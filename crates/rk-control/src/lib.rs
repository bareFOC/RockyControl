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

pub mod cbf;
pub mod com_fk;
pub mod estimator;
pub mod impact;
pub mod slip;
pub mod uvc;

use core::f32::consts::{FRAC_PI_2, PI, TAU};

use libm::sqrtf;
use rk_robot_profile::{RobotProfile, generated};

pub use cbf::{
    CbfConfig, project_capture_point, project_com_velocity, project_tilt_rate, project_tilt_torque,
    shield_torque,
};
pub use estimator::{DcmSwingHint, KinematicEstimator};
pub use impact::{
    ImpactConfig, ImpactDetector, ImpactOutput, ImpactSample, MAX_IMPACT_JOINTS,
    gravity_torque_nm,
};
use rk_proto::{
    BalanceGains, BalanceTelemetry, ConfigCmd, ControlMode, FaultFlags, HostToMcu,
    ImpactWireConfig, ImuState, JointCommandMode, MAX_JOINTS, McuToHost, PROTOCOL_VERSION,
    RunningConfig, UvcGaitConfig, WATCHDOG_TIMEOUT_MS, WalkCmd, classify_fall, estimator_valid,
    fall_orientation, uvc_phase,
};
use serde::{Deserialize, Serialize};
pub use slip::{FlightDetector, SlipConfig, SlipController, SlipInputs, SlipOutput, SlipPhase};
pub use uvc::{LegIkAngles, UvcController, UvcPhase, UvcStepOutput, solve_leg_ik};

pub const DEFAULT_BASE_SLEW_RAD_PER_SEC: f32 = 2.0;
pub const BALANCE_SLEW_RAD_PER_SEC: f32 = 5.5;
pub const OVER_CURRENT_TRIP_MA: u16 = 1950;
pub const OVER_TEMP_TRIP_C: u8 = 78;
pub const UNDER_VOLTAGE_TRIP_MV: u16 = 10_500;

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

// `impact.rs` intentionally has no wire dependency; keep its fixed capacity in
// lockstep with the protocol's joint array at compile time.
const _: [(); 1] = [(); (impact::MAX_IMPACT_JOINTS == MAX_JOINTS) as usize];

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
    #[serde(with = "rk_proto::serde_array")]
    pub raw_angle_rad: [f32; MAX_JOINTS],
    #[serde(with = "rk_proto::serde_array")]
    pub velocity_rad_s: [f32; MAX_JOINTS],
    #[serde(with = "rk_proto::serde_array")]
    pub current_ma: [u16; MAX_JOINTS],
    #[serde(with = "rk_proto::serde_array")]
    pub temp_c: [u8; MAX_JOINTS],
    pub bus_voltage_mv: u16,
    pub bus_error: bool,
    pub imu: ImuState,
    /// Cumulative I3C servo bus timeout count reported by the bus driver.
    pub i3c_timeout_count: u16,
    /// Active IMU source validity flags (`estimator_valid::LSM6DSV_OK` / `BNO055_OK`).
    pub imu_source_mask: u32,
    /// Bit 0 = Right sole loaded, bit 1 = Left sole loaded (same wire convention as
    /// `EstimatorTelemetry::contact_mask`). Meaningless unless `contact_valid_mask`
    /// asserts `estimator_valid::CONTACT_SENSED`.
    pub contact_mask: u8,
    /// Normal ground reaction in Newtons indexed `[right, left]`. Meaningless
    /// unless `contact_valid_mask` asserts `estimator_valid::GRF`.
    pub grf_n: [f32; 2],
    /// Validity mask for the sensed contact channels (`estimator_valid::CONTACT_SENSED`
    /// gates `contact_mask`, `estimator_valid::GRF` gates `grf_n`). Hardware without
    /// a contact sensor leaves this clear, so the kernel keeps its phase fallback.
    pub contact_valid_mask: u32,
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
            contact_mask: 0,
            grf_n: [0.0; 2],
            contact_valid_mask: 0,
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub struct RawServoCommand {
    pub torque_mask: u64,
    #[serde(with = "rk_proto::serde_array")]
    pub raw_target_rad: [f32; MAX_JOINTS],
    #[serde(with = "rk_proto::serde_array")]
    pub command_mode: [u8; MAX_JOINTS],
    #[serde(with = "rk_proto::serde_array")]
    pub raw_velocity_rad_s: [f32; MAX_JOINTS],
    #[serde(with = "rk_proto::serde_array")]
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
pub fn raw_to_calibrated_with_ratio(raw_rad: f32, cal: &JointCalibration, ratio: f32) -> f32 {
    let sign = if cal.inverted { -1.0_f32 } else { 1.0_f32 };
    let inv_ratio = if ratio != 0.0_f32 { 1.0_f32 / ratio } else { 1.0_f32 };
    sign * (raw_rad - cal.zero_offset_rad) * inv_ratio
}

#[inline]
pub fn calibrated_to_raw_with_ratio(cal_rad: f32, cal: &JointCalibration, ratio: f32) -> f32 {
    let sign = if cal.inverted { -1.0_f32 } else { 1.0_f32 };
    (sign * cal_rad * ratio) + cal.zero_offset_rad
}

#[inline]
pub fn raw_to_calibrated(raw_rad: f32, cal: &JointCalibration) -> f32 {
    raw_to_calibrated_with_ratio(raw_rad, cal, 1.0_f32)
}

#[inline]
pub fn calibrated_to_raw(cal_rad: f32, cal: &JointCalibration) -> f32 {
    calibrated_to_raw_with_ratio(cal_rad, cal, 1.0_f32)
}

#[derive(Clone, Debug)]
pub struct DirectServoKernel {
    profile: &'static RobotProfile,
    joint_count: usize,
    base_slew_rad_per_sec: f32,
    calibration: [JointCalibration; MAX_JOINTS],
    active_mode: ControlMode,
    latched_faults: FaultFlags,
    last_valid_seq: u32,
    last_valid_rx_ms: Option<u32>,
    last_host_speed_pct: u8,
    last_host_torque_mask: u64,
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
    /// Last sensed loaded-foot mask. Held through an airborne sensed tick so a
    /// brief loss of sole contact does not move the capture-point origin to the
    /// pelvis (which would command a foot placement under the falling body).
    last_support_mask: u8,
    uvc_blend: f32,
    /// Orientation captured at the most recent `TILT_FALL_CUTOFF` latch
    /// (`fall_orientation::*` wire code). Persists while the fault is latched.
    fall_orientation: u8,
    /// Cumulative MCU wire drop counters forwarded into `McuToHost`.
    rx_crc_errors: u16,
    rx_version_mismatches: u16,
    // P3 running overlay: SLIP template + scalar CBF safety shield. The flag
    // is OFF by default so every v1 build keeps its exact walking/balance path.
    /// Authoritative runtime gate for the SLIP running overlay.
    running_enabled: bool,
    /// Stateful SLIP controller (holds its own sanitized configuration).
    slip: SlipController,
    // P4 combat impact detection. `torque_capable` is OFF by default: v1 runs
    // the current-spike heuristic, v2 firmware opts into the momentum observer.
    /// Stateful impact detector (momentum observer or v1 current heuristic).
    impact: ImpactDetector,
}

impl DirectServoKernel {
    pub fn new(joint_count: usize, initial_cal: &[JointCalibration]) -> Self {
        let profile = if joint_count == 8 {
            &generated::striker_biped::PROFILE
        } else {
            &generated::humanoid::PROFILE
        };
        let mut kernel = Self::new_with_profile(profile, initial_cal);
        kernel.joint_count = joint_count.clamp(1, MAX_JOINTS);
        kernel
    }

    pub fn new_with_profile(
        profile: &'static RobotProfile,
        initial_cal: &[JointCalibration],
    ) -> Self {
        let clamped_count = profile.joint_count.clamp(1, MAX_JOINTS);
        let mut calibration = [JointCalibration::default(); MAX_JOINTS];
        for (i, dst) in calibration.iter_mut().enumerate() {
            dst.min_rad = profile.default_limits[i][0];
            dst.max_rad = profile.default_limits[i][1];
            dst.max_torque_nm = profile.max_torque_nm[i];
            dst.max_velocity_rad_s = profile.max_velocity_rad_s[i];
        }
        for (dst, src) in calibration.iter_mut().zip(initial_cal.iter()) {
            *dst = *src;
        }
        let mut gains = BalanceGains::default();
        gains.h_com_m = profile.h_com_m;
        gains.foot_half_length_m = profile.foot_half_length_m;
        gains.foot_margin_m = profile.foot_margin_m;
        Self {
            profile,
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
            balance_gains: gains,
            last_walk_cmd: WalkCmd::default(),
            uvc: UvcController::new_for_profile(profile.leg_geometry, profile.z_up),
            estimator: KinematicEstimator::new(),
            last_support_mask: 0b11,
            uvc_blend: 0.0,
            fall_orientation: fall_orientation::UNKNOWN,
            rx_crc_errors: 0,
            rx_version_mismatches: 0,
            running_enabled: false,
            slip: SlipController::new(SlipConfig::from_nominal_height(
                profile.leg_geometry.nominal_height,
            )),
            impact: ImpactDetector::new(ImpactConfig::default()),
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

    pub const fn profile(&self) -> &'static RobotProfile {
        self.profile
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

    pub fn set_balance_gains(&mut self, gains: BalanceGains) {
        self.balance_gains = gains;
    }

    pub const fn uvc_gait_config(&self) -> UvcGaitConfig {
        self.uvc.gait_config()
    }

    pub fn set_uvc_gait_config(&mut self, cfg: UvcGaitConfig) {
        self.uvc.set_gait_config(cfg);
    }

    pub const fn impact_wire_config(&self) -> ImpactWireConfig {
        let c = self.impact.config();
        ImpactWireConfig {
            torque_capable: c.torque_capable,
            k_o: c.k_o,
            threshold_nm: c.threshold_nm,
            debounce_ticks: c.debounce_ticks,
            refractory_s: c.refractory_s,
            current_spike_ma: c.current_spike_ma,
            ma_per_nm: c.ma_per_nm,
        }
    }

    pub fn set_impact_wire_config(&mut self, cfg: ImpactWireConfig) {
        self.impact.set_config(ImpactConfig {
            torque_capable: cfg.torque_capable,
            k_o: cfg.k_o,
            threshold_nm: cfg.threshold_nm,
            debounce_ticks: cfg.debounce_ticks,
            refractory_s: cfg.refractory_s,
            current_spike_ma: cfg.current_spike_ma,
            ma_per_nm: cfg.ma_per_nm,
        });
    }

    pub fn set_calibration(&mut self, cal: &[JointCalibration]) {
        let n = self.joint_count.min(cal.len());
        self.calibration[..n].copy_from_slice(&cal[..n]);
    }

    pub fn flash_commit_count(&self) -> u32 {
        self.flash_commit_count
    }

    /// True when the SLIP running overlay is active. Always `false` by default.
    pub const fn running_enabled(&self) -> bool {
        self.running_enabled
    }

    /// Current SLIP running phase (useful for telemetry added by the wire work).
    pub const fn slip_phase(&self) -> SlipPhase {
        self.slip.phase()
    }

    /// Debounced flight flag of the running detector.
    pub const fn slip_in_flight(&self) -> bool {
        self.slip.in_flight()
    }

    /// Enables or disables the running overlay. Disabling also resets its
    /// internal phase/debounce/landing state so re-enabling starts cleanly.
    pub fn set_running_enabled(&mut self, enabled: bool) {
        self.running_enabled = enabled;
        self.slip.config.enabled = enabled;
        if !enabled {
            self.slip.reset();
        }
    }

    /// Replaces the SLIP parameter set. `cfg.enabled` is authoritative, so
    /// this also toggles the kernel gate.
    pub fn set_slip_config(&mut self, cfg: SlipConfig) {
        self.running_enabled = cfg.enabled;
        self.slip = SlipController::new(cfg);
    }

    /// Read-only copy of the active SLIP parameters.
    pub const fn slip_config(&self) -> SlipConfig {
        self.slip.config
    }

    /// True when the torque-capable momentum observer replaces the v1
    /// current-spike heuristic. Always `false` by default.
    pub const fn torque_capable(&self) -> bool {
        self.impact.config().torque_capable
    }

    /// Read-only copy of the active impact-detector parameters.
    pub const fn impact_config(&self) -> ImpactConfig {
        *self.impact.config()
    }

    /// Enables/disables the hardware gate. Changing the gate resets the
    /// detector's baselines so no stale sample can trigger a false impact.
    pub fn set_torque_capable(&mut self, enabled: bool) {
        self.impact.set_torque_capable(enabled);
    }

    /// Replaces the complete impact parameter set (including the gate).
    pub fn set_impact_config(&mut self, cfg: ImpactConfig) {
        self.impact.set_config(cfg);
    }

    /// Debounced impact flag from the most recent tick (before fall/ukemi gating).
    pub const fn impact_detected(&self) -> bool {
        self.impact.impact_active()
    }

    /// Impact magnitude in N·m-equivalent from the most recent tick.
    pub const fn impact_magnitude_nm(&self) -> f32 {
        self.impact.last_magnitude_nm()
    }

    /// Maps the wire `RunningConfig` (Group 2) onto the kernel's local
    /// `SlipConfig`. The running crate stays `rk-proto`-agnostic; only this
    /// integrator knows the wire layout. Invalid numeric fields force the
    /// gate off. State is preserved when gains are unchanged.
    fn apply_running_config(&mut self, wire: RunningConfig) {
        if !wire.all_finite() {
            if self.running_enabled {
                self.set_running_enabled(false);
            }
            return;
        }
        let cfg = SlipConfig {
            enabled: wire.enabled,
            mass_kg: wire.mass_kg,
            spring_k_n_per_m: wire.leg_spring_n_m,
            rest_length_m: wire.leg_rest_length_m,
            target_hop_m: wire.hop_height_m,
            cbf_kappa: wire.cbf_kappa,
            landing_k0_n_per_m: wire.landing_stiffness_touchdown_n_m,
            landing_k1_n_per_m: wire.landing_stiffness_nominal_n_m,
            landing_damping_n_s_per_m: wire.landing_damping_n_s_m,
            landing_ramp_s: wire.landing_ramp_s,
            ..Default::default()
        };
        if cfg != self.slip.config {
            self.set_slip_config(cfg);
        }
    }

    pub fn step(
        &mut self,
        now_ms: u32,
        dt_sec: f32,
        host_cmd: Option<&HostToMcu>,
        feedback: &RawServoFeedback,
    ) -> KernelStepOutput {
        let dt = dt_sec.clamp(0.0001_f32, 0.1_f32);

        // 1. Convert raw encoder angles / rates -> calibrated joint space
        let mut measured_rad = [0.0_f32; MAX_JOINTS];
        let mut measured_vel_rad_s = [0.0_f32; MAX_JOINTS];
        for i in 0..self.joint_count {
            let ratio = self.profile.motor_to_joint_ratio[i];
            let inv_ratio = if ratio != 0.0_f32 { 1.0_f32 / ratio } else { 1.0_f32 };
            measured_rad[i] =
                raw_to_calibrated_with_ratio(feedback.raw_angle_rad[i], &self.calibration[i], ratio);
            let sign = if self.calibration[i].inverted {
                -1.0_f32
            } else {
                1.0_f32
            };
            measured_vel_rad_s[i] = sign * feedback.velocity_rad_s[i] * inv_ratio;
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
                        measured_rad[idx] = raw_to_calibrated_with_ratio(
                            feedback.raw_angle_rad[idx],
                            &self.calibration[idx],
                            self.profile.motor_to_joint_ratio[idx],
                        );
                        self.applied_target_rad[idx] = measured_rad[idx];
                        self.last_host_target_rad[idx] = measured_rad[idx];
                    }
                }
                ConfigCmd::SetInvert { joint, inverted } => {
                    let idx = joint as usize;
                    if idx < self.joint_count {
                        self.calibration[idx].inverted = inverted;
                        measured_rad[idx] = raw_to_calibrated_with_ratio(
                            feedback.raw_angle_rad[idx],
                            &self.calibration[idx],
                            self.profile.motor_to_joint_ratio[idx],
                        );
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
                        && g.h_com_m.is_finite()
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
                            h_com_m: g.h_com_m.clamp(0.12, 0.80),
                            ukemi_tilt_rad: g
                                .ukemi_tilt_rad
                                .clamp(0.25, (fall_cutoff - 0.03).max(0.25)),
                            ukemi_torque_scale: g.ukemi_torque_scale.clamp(0.05, 0.50),
                            compliance_scale: g.compliance_scale.clamp(0.10, 1.0),
                        };
                    }
                }
                ConfigCmd::SetUvcGait(g) => {
                    if g.all_finite() {
                        self.uvc.set_gait_config(g);
                    }
                }
                ConfigCmd::SetActuatorLimits {
                    joint,
                    max_torque_nm,
                    max_velocity_rad_s,
                } => {
                    let idx = joint as usize;
                    if idx < self.joint_count
                        && max_torque_nm.is_finite()
                        && max_velocity_rad_s.is_finite()
                    {
                        let hw_max_t = self.profile.max_torque_nm[idx].max(0.05_f32);
                        let hw_max_v = self.profile.max_velocity_rad_s[idx].max(0.5_f32);
                        self.calibration[idx].max_torque_nm =
                            max_torque_nm.clamp(0.05_f32, hw_max_t);
                        self.calibration[idx].max_velocity_rad_s =
                            max_velocity_rad_s.clamp(0.5_f32, hw_max_v);
                    }
                }
                ConfigCmd::SetImpactConfig(c) => {
                    if c.all_finite() {
                        self.set_impact_wire_config(c);
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
                    self.slip.reset();
                    self.impact.reset();
                    self.fall_orientation = fall_orientation::UNKNOWN;
                    cleared_faults_this_tick = true;
                }
            }

            self.last_host_speed_pct = cmd.speed_limit_pct.clamp(1, 100);
            self.last_host_torque_mask = cmd.torque_mask;
            self.last_walk_cmd = cmd.walk;
            self.apply_running_config(cmd.walk.running);
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
                    self.impact.reset();
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

        // Share UVC's calibrated attitude with the estimator at a 1-tick lag:
        // UVC runs *after* the estimator in this tick, so its bias accessors
        // expose the previous tick's calibration. The validated 21-DoF FK path
        // derives CoM from joint encoders only (no IMU tilt), but the legacy
        // proxy fallback for non-21-DoF rigs is attitude-dependent and must see
        // the bias-compensated sample. IMU availability/source bits are kept
        // untouched so validity gating is unchanged.
        let (comp_pitch, comp_roll) = self.uvc.bias_compensated_attitude(&imu);
        let mut estimator_imu = imu;
        if imu.available {
            estimator_imu.pitch_rad = comp_pitch;
            estimator_imu.roll_rad = comp_roll;
        }

        // Sensed contact/GRF closes the support loop when a sensor publishes it.
        // Hardware (and any replay log) with the validity bits clear keeps the
        // exact phase-derived fallback path, including zero GRF telemetry.
        let contact_sensed =
            feedback.contact_valid_mask & estimator_valid::CONTACT_SENSED != 0;
        let grf_valid = contact_sensed && feedback.contact_valid_mask & estimator_valid::GRF != 0;
        // A sole counts as loaded only when it actually carries ground reaction.
        // The plant's boolean contact can stay true through touchdown transitions
        // with ~0 N; using GRF removes that flicker from the capture origin.
        const MIN_LOADED_GRF_N: f32 = 1.0_f32;
        let sensed_mask = if contact_sensed {
            (if feedback.grf_n[0] > MIN_LOADED_GRF_N { 0b01 } else { 0 })
                | (if feedback.grf_n[1] > MIN_LOADED_GRF_N { 0b10 } else { 0 })
        } else {
            0
        };
        let phase_mask = self.uvc.contact_mask();
        // The estimator needs a support *origin* even during a brief flight
        // tick. Hold the last loaded sole rather than snapping to the pelvis;
        // telemetry still reports the raw sensed mask (including flight).
        // While walking, a sensed sole that disagrees with the gait's support
        // foot is a foot that has not landed yet: use the phase support for the
        // capture origin so the swing trajectory cannot chase a stale sole.
        let support_mask = if contact_sensed && sensed_mask != 0 {
            // A sensed sole that disagrees with the controller's support foot is
            // a sole still in transition (swing/landing), not a stable origin.
            // Keep the commanded support for the capture origin; telemetry still
            // reports the raw sensed mask. This holds both while walking and
            // while a catch step is settling after a shove.
            if sensed_mask != phase_mask
                && (self.last_walk_cmd.walk_enabled || max_tilt < 0.15_f32)
            {
                phase_mask
            } else {
                self.last_support_mask = sensed_mask;
                sensed_mask
            }
        } else if contact_sensed {
            self.last_support_mask
        } else {
            phase_mask
        };
        let reported_mask = if contact_sensed {
            // Telemetry reports the sensor's raw contact mask; only the capture
            // origin uses the GRF-refined / phase-gated support selection.
            feedback.contact_mask & 0b11
        } else {
            support_mask
        };
        let sensed_grf_n = if contact_sensed {
            feedback.grf_n
        } else {
            [0.0_f32; 2]
        };

        let (mut estimator_telem, dcm_hint) = self.estimator.step(
            self.profile,
            dt,
            self.joint_count,
            &measured_rad,
            &measured_vel_rad_s,
            estimator_imu,
            self.uvc.geometry().nominal_height,
            support_mask,
            sensed_grf_n,
            grf_valid,
            self.last_walk_cmd.walk_enabled,
            feedback.imu_source_mask,
            ukemi_active,
            self.balance_gains,
        );
        // Publish exactly the sensed mask the kernel consumed this tick. During
        // flight this is 0 even though the estimator holds the last origin.
        estimator_telem.contact_mask = reported_mask;

        // 5b. Compute MCU-native UVC upper-body balance + locomotion overlay
        let pelvis_imu = self.estimator.pelvis_imu();
        let mut desired_rad = self.last_host_target_rad;
        let mut balance_telem = BalanceTelemetry {
            hip_height_m: self.uvc.geometry().nominal_height,
            ..Default::default()
        };

        let ji = &self.profile.joint_indices;
        let humanoid_legs_available = self.joint_count >= 17
            && ji.l_hip_pitch.is_some()
            && ji.l_knee_pitch.is_some()
            && ji.r_hip_pitch.is_some()
            && ji.r_knee_pitch.is_some();

        if self.active_mode == ControlMode::Drive && humanoid_legs_available {
            let walk = self.last_walk_cmd;
            let uvc_engaged = walk.balance_enabled || walk.walk_enabled;
            let tilt_exceeded = pelvis_imu
                .pitch_rad
                .abs()
                .max(pelvis_imu.roll_rad.abs())
                >= self.balance_gains.fall_cutoff_rad
                || max_tilt >= self.balance_gains.fall_cutoff_rad;

            if uvc_engaged || tilt_exceeded || self.uvc.phase() == UvcPhase::FallTuck {
                let uvc_out = self.uvc.step_with_dcm(
                    dt,
                    pelvis_imu,
                    walk,
                    self.balance_gains,
                    dcm_hint,
                    estimator_telem,
                );

                if uvc_out.fall_cutoff_ready {
                    // Protective tuck complete -> drop torque & latch TILT_FALL_CUTOFF
                    self.active_mode = ControlMode::EStop;
                    self.latched_faults.insert(FaultFlags::TILT_FALL_CUTOFF);
                    self.fall_orientation = if uvc_out.fall_orientation != fall_orientation::UNKNOWN
                    {
                        uvc_out.fall_orientation
                    } else {
                        classify_fall(&pelvis_imu).to_wire()
                    };
                } else {
                    let target_blend = 1.0_f32;
                    let ramp_rate =
                        if uvc_out.in_fall_tuck || uvc_out.phase == UvcPhase::ActiveCatchStep {
                            24.0_f32
                        } else {
                            8.0_f32
                        };
                    self.uvc_blend +=
                        (target_blend - self.uvc_blend) * (dt * ramp_rate).clamp(0.0, 1.0);
                    let b = self.uvc_blend;
                    let set_lerp = |dst: &mut [f32; MAX_JOINTS], opt: Option<u8>, t: f32| {
                        if let Some(idx) = opt {
                            let i = idx as usize;
                            if i < self.joint_count {
                                dst[i] += (t - dst[i]) * b;
                            }
                        }
                    };
                    let add_delta = |dst: &mut [f32; MAX_JOINTS], opt: Option<u8>, d: f32| {
                        if let Some(idx) = opt {
                            let i = idx as usize;
                            if i < self.joint_count {
                                dst[i] += d * b;
                            }
                        }
                    };

                    set_lerp(&mut desired_rad, ji.l_hip_yaw, uvc_out.left_leg.hip_yaw);
                    set_lerp(&mut desired_rad, ji.l_hip_roll, uvc_out.left_leg.hip_roll);
                    set_lerp(&mut desired_rad, ji.l_hip_pitch, uvc_out.left_leg.hip_pitch);
                    set_lerp(&mut desired_rad, ji.l_knee_pitch, uvc_out.left_leg.knee_pitch);
                    set_lerp(&mut desired_rad, ji.l_ankle_pitch, uvc_out.left_leg.ankle_pitch);
                    set_lerp(&mut desired_rad, ji.l_ankle_roll, uvc_out.left_leg.ankle_roll);
                    set_lerp(&mut desired_rad, ji.l_toe_pitch, uvc_out.left_leg.toe_pitch);

                    set_lerp(&mut desired_rad, ji.r_hip_yaw, uvc_out.right_leg.hip_yaw);
                    set_lerp(&mut desired_rad, ji.r_hip_roll, uvc_out.right_leg.hip_roll);
                    set_lerp(&mut desired_rad, ji.r_hip_pitch, uvc_out.right_leg.hip_pitch);
                    set_lerp(&mut desired_rad, ji.r_knee_pitch, uvc_out.right_leg.knee_pitch);
                    set_lerp(&mut desired_rad, ji.r_ankle_pitch, uvc_out.right_leg.ankle_pitch);
                    set_lerp(&mut desired_rad, ji.r_ankle_roll, uvc_out.right_leg.ankle_roll);
                    set_lerp(&mut desired_rad, ji.r_toe_pitch, uvc_out.right_leg.toe_pitch);

                    // Waist coupling + upper-body arm counter-spread (`armCont`)
                    add_delta(&mut desired_rad, ji.waist_yaw, uvc_out.waist_yaw);
                    add_delta(&mut desired_rad, ji.waist_pitch, uvc_out.waist_pitch);
                    add_delta(&mut desired_rad, ji.waist_roll, uvc_out.waist_roll);
                    add_delta(&mut desired_rad, ji.l_shoulder_pitch, uvc_out.l_shoulder_pitch_delta);
                    add_delta(&mut desired_rad, ji.l_shoulder_roll, uvc_out.l_shoulder_roll_delta);
                    add_delta(&mut desired_rad, ji.r_shoulder_pitch, uvc_out.r_shoulder_pitch_delta);
                    add_delta(&mut desired_rad, ji.r_shoulder_roll, uvc_out.r_shoulder_roll_delta);
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
                    running_active: false,
                    flight_detected: false,
                    running_com_height_m: 0.0,
                    // Filled in by the impact stage below (after torque control).
                    impact_detected: false,
                    impact_magnitude_nm: 0.0,
                    // World-frame position/velocity are published by the plant
                    // backend (rk-sim), not by the hardware-agnostic kernel.
                    ..Default::default()
                };
            } else {
                self.uvc.reset();
                self.uvc_blend = 0.0;
                balance_telem.uvc_phase = uvc_phase::IDLE;
            }
        } else {
            self.uvc.reset();
            self.uvc_blend = 0.0;
            self.slip.reset();
            balance_telem.uvc_phase = uvc_phase::IDLE;
        }

        balance_telem.fall_orientation = self.fall_orientation;

        // 5c. P3 running overlay (SLIP + CBF). Gated OFF by default: with the
        // flag false, `slip_active` stays false and the step-6 loop below is
        // byte-for-byte identical to v1 behavior.
        let mut slip_active = false;
        let mut slip_torque_nm = 0.0_f32;
        if self.running_enabled
            && self.slip.config.enabled
            && self.active_mode == ControlMode::Drive
            && humanoid_legs_available
        {
            let cfg = self.slip.config;
            let geom = self.uvc.geometry();
            let lk = ji.l_knee_pitch.unwrap_or(0) as usize;
            let rk = ji.r_knee_pitch.unwrap_or(0) as usize;
            let knee_rad = 0.5_f32 * (measured_rad[lk] + measured_rad[rk]);
            let hip_height = (geom.nominal_height - geom.thigh_len * (knee_rad - 0.35_f32))
                .clamp(geom.min_height, geom.nominal_height + 0.06_f32);
            let leg_length_m = cfg.rest_length_m * (hip_height / geom.nominal_height);
            let leg_length_rate_m_s = -0.5_f32
                * (measured_vel_rad_s[lk] + measured_vel_rad_s[rk])
                * geom.thigh_len;

            let mut leg_current_ma = 0.0_f32;
            for i in 0..self.joint_count {
                if ji.is_slip_leg_joint(i) {
                    leg_current_ma += feedback.current_ma[i] as f32;
                }
            }
            let specific_force_m_s2 = sqrtf(
                imu.gravity[0] * imu.gravity[0]
                    + imu.gravity[1] * imu.gravity[1]
                    + imu.gravity[2] * imu.gravity[2],
            );

            let slip_out = self.slip.step(SlipInputs {
                dt_s: dt,
                leg_length_m,
                leg_length_rate_m_s,
                leg_current_ma,
                specific_force_m_s2,
                imu_available: imu.available,
            });

            // Scalar CBF shield: a command that would violate the torso-tilt,
            // CoM-velocity or support-relative capture-point safe set is
            // minimally modified before the existing torque ceiling is applied.
            let tilt = if imu.pitch_rad.abs() >= imu.roll_rad.abs() {
                imu.pitch_rad
            } else {
                imu.roll_rad
            };
            let tilt_rate = if imu.pitch_rad.abs() >= imu.roll_rad.abs() {
                imu.pitch_rate_rad_s
            } else {
                imu.roll_rate_rad_s
            };
            // `foot_margin_m` is the estimator's *support-relative* signed
            // remaining margin: margin - |xi_rel - p|. Converting it back to
            // a distance keeps the CBF capture set relative to support rather
            // than to the world origin (which would falsely trip on a CoM
            // offset inherent to a crouched pose).
            let capture_err = (0.028_f32 - estimator_telem.foot_margin_m).max(0.0_f32);
            let cbf_cfg = CbfConfig {
                kappa: cfg.cbf_kappa,
                ..Default::default()
            };
            slip_torque_nm = shield_torque(
                slip_out.joint_torque_nm,
                tilt,
                tilt_rate,
                estimator_telem.com_vel_m_s[2],
                capture_err,
                &cbf_cfg,
                4.0_f32, // approximate tilt authority: rad/s² per N·m
                dt,
            );
            slip_active = true;
            balance_telem.running_active = true;
            balance_telem.flight_detected = slip_out.in_flight;
            let up_idx = if self.profile.z_up { 2 } else { 1 };
            balance_telem.running_com_height_m = estimator_telem.com_pos_m[up_idx];
            // Mark the running telemetry channels valid for consumers.
            estimator_telem.valid_mask |= rk_proto::estimator_valid::RUNNING;
        }

        // 6. Compute torque mask, soft-stopped slew-clamped targets, and explicit physical
        //    torque / velocity ceilings (Safety ALWAYS downstream of balance & host commands).
        let mut torque_state_mask: u64 = 0;
        let mut raw_target_rad = [0.0_f32; MAX_JOINTS];
        let mut cmd_mode_out = [JointCommandMode::Position as u8; MAX_JOINTS];
        let mut raw_vel_out = [0.0_f32; MAX_JOINTS];
        let mut torque_nm_out = [0.0_f32; MAX_JOINTS];

        if self.active_mode == ControlMode::Drive {
            let valid_mask = if self.joint_count >= 64 {
                u64::MAX
            } else {
                (1u64 << self.joint_count) - 1
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
                    let ratio = self.profile.motor_to_joint_ratio[i];
                    raw_vel_out[i] = sign * clamped_vel * ratio;

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

                    // P3 running overlay: the SLIP/CBF torque replaces the host
                    // torque on leg joints only (including toes), still subject to
                    // the very same effective ceiling (compliance/ukemi) computed above.
                    if slip_active && ji.is_slip_leg_joint(i) {
                        cmd_mode_out[i] = JointCommandMode::Torque.to_wire();
                        raw_vel_out[i] = 0.0;
                        torque_nm_out[i] =
                            sign * slip_torque_nm.clamp(-effective_ceiling, effective_ceiling);
                    }
                }
                raw_target_rad[i] = calibrated_to_raw_with_ratio(
                    self.applied_target_rad[i],
                    &self.calibration[i],
                    self.profile.motor_to_joint_ratio[i],
                );
            }
        } else {
            // Teach or EStop: Torque strictly OFF, applied target tracks measured
            for i in 0..self.joint_count {
                self.applied_target_rad[i] = measured_rad[i];
                raw_target_rad[i] = calibrated_to_raw_with_ratio(
                    measured_rad[i],
                    &self.calibration[i],
                    self.profile.motor_to_joint_ratio[i],
                );
                raw_vel_out[i] = 0.0;
                torque_nm_out[i] = 0.0;
            }
        }

        // 6b. P4 combat impact detection. Runs *after* the safety/torque stage
        // so the momentum observer sees exactly the torque applied this tick.
        // The result is telemetry only: fall/ukemi logic retains priority and
        // no servo command is modified by an impact.
        let impact_out = if self.active_mode == ControlMode::Drive {
            self.impact.step(self.profile, &ImpactSample {
                dt_s: dt,
                joint_count: self.joint_count,
                angles: &measured_rad,
                velocities: &measured_vel_rad_s,
                torque_nm: &torque_nm_out,
                current_ma: &feedback.current_ma,
            })
        } else {
            self.impact.reset();
            ImpactOutput::default()
        };
        let fall_priority =
            ukemi_active || self.latched_faults.contains(FaultFlags::TILT_FALL_CUTOFF);
        balance_telem.impact_detected = impact_out.impact && !fall_priority;
        balance_telem.impact_magnitude_nm = impact_out.magnitude_nm;
        estimator_telem.valid_mask |= rk_proto::estimator_valid::IMPACT;

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

    const J21_L_HIP_PITCH: usize = generated::humanoid::PROFILE.joint_indices.l_hip_pitch.unwrap() as usize;
    const J21_L_KNEE_PITCH: usize = generated::humanoid::PROFILE.joint_indices.l_knee_pitch.unwrap() as usize;
    const J21_L_ANKLE_PITCH: usize = generated::humanoid::PROFILE.joint_indices.l_ankle_pitch.unwrap() as usize;
    const J21_R_HIP_PITCH: usize = generated::humanoid::PROFILE.joint_indices.r_hip_pitch.unwrap() as usize;
    const J21_R_KNEE_PITCH: usize = generated::humanoid::PROFILE.joint_indices.r_knee_pitch.unwrap() as usize;
    const J21_R_ANKLE_PITCH: usize = generated::humanoid::PROFILE.joint_indices.r_ankle_pitch.unwrap() as usize;

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
        let mut kernel = DirectServoKernel::new(28, &cal);

        let mut fb = RawServoFeedback::default();
        fb.imu.pitch_rad = 0.22; // Leaning forward ~12.6 deg
        fb.imu.roll_rad = -0.15; // Leaning left -> Left leg catches

        let mut cmd = HostToMcu {
            seq: 1,
            mode: ControlMode::Drive,
            torque_mask: (1u64 << 21) - 1,
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
        assert_eq!(
            out.telemetry.balance.uvc_phase,
            uvc_phase::ACTIVE_CATCH_STEP
        );
        assert_eq!(
            out.telemetry.balance.support_foot, 1,
            "left foot catches -roll lean"
        );
        assert!(
            out.telemetry.balance.foot_dx_m > 0.02,
            "foot must step forward under forward tilt"
        );
        assert!(
            out.telemetry.balance.foot_dy_m < -0.01,
            "left foot must step left under -roll"
        );

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
            let mut kernel = DirectServoKernel::new(28, &cal);
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
                torque_mask: (1u64 << 21) - 1,
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
        let mut kernel = DirectServoKernel::new(28, &cal);
        let fb = RawServoFeedback::default();
        let mut cmd = HostToMcu {
            seq: 1,
            mode: ControlMode::Drive,
            torque_mask: (1u64 << 21) - 1,
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
        }; 28];
        let mut kernel = DirectServoKernel::new(28, &cal);
        let mut cmd = HostToMcu {
            seq: 1,
            mode: ControlMode::Drive,
            torque_mask: (1u64 << 28) - 1,
            speed_limit_pct: 100,
            walk: WalkCmd {
                balance_enabled: true,
                ..Default::default()
            },
            ..Default::default()
        };

        // Quiet upright stance -> DCM inside foot margin
        let mut fb = RawServoFeedback::default();
        let quiet = kernel.step(5, 0.005, Some(&cmd), &fb);
        assert!(
            quiet.telemetry.estimator.foot_margin_m > 0.015,
            "upright margin must be positive, got {}",
            quiet.telemetry.estimator.foot_margin_m
        );

        // Apply a joint-velocity impulse on both hip pitch joints (11, 17)
        // while angles remain near zero. The mass-weighted CoM Jacobian turns
        // this into a support-relative CoM velocity, so Capture Point
        // xi = c_rel + v_rel / omega leaves the ±2.8 cm margin and triggers
        // ActiveCatchStep. IMU rate alone no longer drives DCM (analytic FK).
        fb.velocity_rad_s[J21_L_HIP_PITCH] = -8.0;
        fb.velocity_rad_s[J21_R_HIP_PITCH] = -8.0;
        fb.imu.available = true; // keep estimator valid bits asserted
        let mut pushed = quiet;
        for tick in 2..=25 {
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
        let mut kernel = DirectServoKernel::new(28, &cal);
        let mut cmd = HostToMcu {
            seq: 1,
            mode: ControlMode::Drive,
            torque_mask: (1u64 << 21) - 1,
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
        let mut kernel = DirectServoKernel::new(28, &cal);
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
        let mut kernel = DirectServoKernel::new(28, &cal);
        let cmd = HostToMcu {
            seq: 1,
            mode: ControlMode::Teach,
            ..Default::default()
        };
        let mut fb = RawServoFeedback::default();
        // Sensor absent: no channel derived from live attitude may be marked valid.
        fb.imu.available = false;
        let outage = kernel.step(10, 0.01, Some(&cmd), &fb);
        // Impact detection is current-based, so its validity bit is independent
        // of IMU availability; all IMU-derived channels remain invalid.
        assert_eq!(
            outage.telemetry.estimator.valid_mask,
            rk_proto::estimator_valid::IMPACT
        );

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

    // ---- Group 1 (P3): SLIP running overlay + CBF shield -----------------

    #[test]
    fn running_disabled_by_default_leaves_servo_output_unchanged() {
        let cal = [JointCalibration {
            min_rad: -2.2,
            max_rad: 2.2,
            ..Default::default()
        }; 21];
        let mut kernel = DirectServoKernel::new(28, &cal);
        assert!(!kernel.running_enabled());
        assert_eq!(kernel.slip_phase(), SlipPhase::Stance);

        let fb = RawServoFeedback::default();
        let mut cmd = HostToMcu {
            seq: 1,
            mode: ControlMode::Drive,
            torque_mask: (1u64 << 21) - 1,
            torque_limit_nm: [1.8; MAX_JOINTS],
            walk: WalkCmd {
                balance_enabled: false,
                ..Default::default()
            },
            ..Default::default()
        };
        cmd.command_mode[11] = JointCommandMode::Torque.to_wire();
        cmd.torque_limit_nm[11] = 0.7;

        let out = kernel.step(10, 0.005, Some(&cmd), &fb);
        assert!(!out.telemetry.balance.running_active);
        assert!(!out.telemetry.balance.flight_detected);
        // Host torque passes through verbatim when the gate is off.
        assert_eq!(
            out.servo_cmd.command_mode[11],
            JointCommandMode::Torque.to_wire()
        );
        assert!((out.servo_cmd.torque_nm[11] - 0.7_f32).abs() < 1e-6);

        // Explicitly disabling twice is a no-op and yields identical commands.
        let mut disabled = DirectServoKernel::new(21, &cal);
        disabled.set_running_enabled(false);
        let out2 = disabled.step(10, 0.005, Some(&cmd), &fb);
        assert_eq!(out.servo_cmd.torque_nm, out2.servo_cmd.torque_nm);
        assert_eq!(out.servo_cmd.command_mode, out2.servo_cmd.command_mode);
        assert_eq!(out.servo_cmd.raw_target_rad, out2.servo_cmd.raw_target_rad);
    }

    #[test]
    fn running_enabled_modulates_leg_torque_behind_the_gate() {
        let cal = [JointCalibration {
            min_rad: -2.2,
            max_rad: 2.2,
            ..Default::default()
        }; 28];
        let mut kernel = DirectServoKernel::new(28, &cal);
        kernel.set_running_enabled(true);
        assert!(kernel.running_enabled());

        let mut fb = RawServoFeedback::default();
        fb.imu.available = true;
        fb.imu.gravity = [0.0, 0.0, -9.81];
        fb.raw_angle_rad[J21_L_HIP_PITCH] = -0.35;
        fb.raw_angle_rad[J21_L_KNEE_PITCH] = 0.35;
        fb.raw_angle_rad[J21_L_ANKLE_PITCH] = -0.35;
        fb.raw_angle_rad[J21_R_HIP_PITCH] = -0.35;
        fb.raw_angle_rad[J21_R_KNEE_PITCH] = 0.35;
        fb.raw_angle_rad[J21_R_ANKLE_PITCH] = -0.35;

        let mut cmd = HostToMcu {
            seq: 1,
            mode: ControlMode::Drive,
            torque_mask: (1u64 << 28) - 1,
            torque_limit_nm: [1.8; MAX_JOINTS],
            walk: WalkCmd {
                balance_enabled: false,
                running: RunningConfig {
                    enabled: true,
                    ..Default::default()
                },
                ..Default::default()
            },
            ..Default::default()
        };
        // Host torque is deliberately lower than the SLIP thrust so the two
        // paths produce distinguishable commands.
        cmd.torque_limit_nm[J21_L_KNEE_PITCH] = 0.7;

        let out = kernel.step(10, 0.005, Some(&cmd), &fb);
        assert!(out.telemetry.balance.running_active);
        assert!(!out.telemetry.balance.flight_detected);
        assert_eq!(kernel.slip_phase(), SlipPhase::Stance);
        // Leg joints switch to torque mode and receive SLIP thrust.
        assert_eq!(
            out.servo_cmd.command_mode[J21_L_KNEE_PITCH],
            JointCommandMode::Torque.to_wire()
        );
        let leg_torque = out.servo_cmd.torque_nm[J21_L_KNEE_PITCH];
        assert!(
            leg_torque > 0.0_f32,
            "compressed leg must push off: {leg_torque}"
        );
        assert!(
            leg_torque <= 1.8_f32 + 1e-6,
            "existing torque ceiling must hold: {leg_torque}"
        );
        // A non-leg joint keeps the host's command mode/value.
        cmd.seq = 2;
        assert_eq!(out.servo_cmd.raw_velocity_rad_s[J21_L_KNEE_PITCH], 0.0);

        // Replaying the same tick with the gate off restores host behavior.
        let mut off_cmd = cmd;
        off_cmd.walk.running.enabled = false;
        let mut off = DirectServoKernel::new(21, &cal);
        let out_off = off.step(10, 0.005, Some(&off_cmd), &fb);
        assert!(!out_off.telemetry.balance.running_active);
        assert_ne!(out_off.servo_cmd.torque_nm[J21_L_KNEE_PITCH], leg_torque);
    }

    #[test]
    fn cbf_shield_reduces_running_torque_when_tilt_violates_the_safe_set() {
        let cal = [JointCalibration {
            min_rad: -2.2,
            max_rad: 2.2,
            ..Default::default()
        }; 28];
        let mut base_fb = RawServoFeedback::default();
        base_fb.imu.available = true;
        base_fb.imu.gravity = [0.0, 0.0, -9.81];
        base_fb.raw_angle_rad[J21_L_HIP_PITCH] = -0.35;
        base_fb.raw_angle_rad[J21_L_KNEE_PITCH] = 0.35;
        base_fb.raw_angle_rad[J21_L_ANKLE_PITCH] = -0.35;
        base_fb.raw_angle_rad[J21_R_HIP_PITCH] = -0.35;
        base_fb.raw_angle_rad[J21_R_KNEE_PITCH] = 0.35;
        base_fb.raw_angle_rad[J21_R_ANKLE_PITCH] = -0.35;

        let cmd = HostToMcu {
            seq: 1,
            mode: ControlMode::Drive,
            torque_mask: (1u64 << 28) - 1,
            torque_limit_nm: [1.8; MAX_JOINTS],
            walk: WalkCmd {
                balance_enabled: false,
                running: RunningConfig {
                    enabled: true,
                    ..Default::default()
                },
                ..Default::default()
            },
            ..Default::default()
        };

        let mut upright = DirectServoKernel::new(28, &cal);
        upright.set_running_enabled(true);
        let quiet = upright.step(10, 0.005, Some(&cmd), &base_fb);
        let torque_upright = quiet.servo_cmd.torque_nm[J21_L_KNEE_PITCH];

        let mut leaning_fb = base_fb;
        // 0.499 rad is inside ukemi (0.56) but beyond the 0.5 rad CBF limit.
        leaning_fb.imu.pitch_rad = 0.499;
        let mut leaning = DirectServoKernel::new(28, &cal);
        leaning.set_running_enabled(true);
        let tilted = leaning.step(10, 0.005, Some(&cmd), &leaning_fb);
        let torque_tilted = tilted.servo_cmd.torque_nm[J21_L_KNEE_PITCH];

        assert!(
            torque_upright > 0.0_f32,
            "baseline thrust required: {torque_upright}"
        );
        assert!(
            torque_tilted < torque_upright,
            "CBF must minimally reduce the command: {torque_tilted} >= {torque_upright}"
        );
    }

    // ---- Group 3 (P4): hardware-gated impact detection -------------------

    fn impact_test_kernel() -> DirectServoKernel {
        let cal = [JointCalibration {
            min_rad: -2.2_f32,
            max_rad: 2.2_f32,
            ..Default::default()
        }; 21];
        DirectServoKernel::new(21, &cal)
    }

    fn impact_test_cmd() -> HostToMcu {
        HostToMcu {
            seq: 1,
            mode: ControlMode::Drive,
            torque_mask: (1u64 << 21) - 1,
            torque_limit_nm: [1.8_f32; MAX_JOINTS],
            walk: WalkCmd {
                balance_enabled: false,
                ..Default::default()
            },
            ..Default::default()
        }
    }

    #[test]
    fn current_spike_detects_impact_without_changing_servo_commands() {
        let mut kernel = impact_test_kernel();
        assert!(!kernel.torque_capable(), "v1 gate must default to OFF");
        assert!(!kernel.impact_config().torque_capable);

        let mut cmd = impact_test_cmd();
        let mut fb = RawServoFeedback::default();
        for c in fb.current_ma.iter_mut() {
            *c = 200;
        }

        let quiet = kernel.step(5, 0.005_f32, Some(&cmd), &fb);
        assert!(!quiet.telemetry.balance.impact_detected);
        assert_ne!(
            quiet.telemetry.estimator.valid_mask & rk_proto::estimator_valid::IMPACT,
            0,
            "impact telemetry must be marked valid on the v1 path"
        );

        fb.current_ma[11] = 800; // +600 mA -> 0.54 N·m-equivalent
        cmd.seq = 2;
        let first = kernel.step(10, 0.005_f32, Some(&cmd), &fb);
        assert!(!first.telemetry.balance.impact_detected, "2-tick debounce");

        cmd.seq = 3;
        let hit = kernel.step(15, 0.005_f32, Some(&cmd), &fb);
        assert!(hit.telemetry.balance.impact_detected);
        assert!(hit.telemetry.balance.impact_magnitude_nm > 0.3_f32);
        assert!(kernel.impact_detected());

        // A second kernel seeing the same command but no current spike must
        // produce byte-identical servo commands: detection is telemetry-only.
        let mut control = impact_test_kernel();
        let mut quiet_fb = fb;
        quiet_fb.current_ma[11] = 200;
        let reference = control.step(15, 0.005_f32, Some(&cmd), &quiet_fb);
        assert_eq!(hit.servo_cmd.torque_nm, reference.servo_cmd.torque_nm);
        assert_eq!(
            hit.servo_cmd.raw_target_rad,
            reference.servo_cmd.raw_target_rad
        );
        assert_eq!(hit.servo_cmd.command_mode, reference.servo_cmd.command_mode);
        assert_eq!(
            hit.servo_cmd.raw_velocity_rad_s,
            reference.servo_cmd.raw_velocity_rad_s
        );
    }

    #[test]
    fn momentum_observer_detects_injected_torque_behind_the_gate() {
        let mut kernel = impact_test_kernel();
        kernel.set_torque_capable(true);
        assert!(kernel.torque_capable());

        let angles = [0.0_f32; MAX_JOINTS];
        let mut hold = [0.0_f32; MAX_IMPACT_JOINTS];
        gravity_torque_nm(&generated::humanoid::PROFILE, &angles, &mut hold);

        let mut cmd = impact_test_cmd();
        for i in 0..MAX_JOINTS {
            cmd.command_mode[i] = JointCommandMode::Torque.to_wire();
            cmd.torque_limit_nm[i] = -hold[i.min(MAX_IMPACT_JOINTS - 1)];
        }
        // Inject +0.6 N·m on hip pitch while the plant does not accelerate:
        // that is exactly an unmodeled external wrench at the joint.
        cmd.torque_limit_nm[J21_L_HIP_PITCH] += 0.6_f32;

        let fb = RawServoFeedback::default();
        let mut impact_seen = false;
        for tick in 1..=8 {
            cmd.seq = tick;
            let out = kernel.step(tick * 5, 0.005_f32, Some(&cmd), &fb);
            if out.telemetry.balance.impact_detected {
                impact_seen = true;
                assert!(
                    out.telemetry.balance.impact_magnitude_nm > 0.20_f32,
                    "magnitude must exceed the 0.2 N·m floor"
                );
                break;
            }
        }
        assert!(
            impact_seen,
            "momentum observer must flag injected external torque"
        );

        // Same gravity-compensated command without injection stays quiet.
        let mut quiet_kernel = impact_test_kernel();
        quiet_kernel.set_torque_capable(true);
        let mut quiet_cmd = cmd;
        quiet_cmd.torque_limit_nm[J21_L_HIP_PITCH] -= 0.6_f32;
        for tick in 1..=8 {
            quiet_cmd.seq = tick;
            let out = quiet_kernel.step(tick * 5, 0.005_f32, Some(&quiet_cmd), &fb);
            assert!(
                !out.telemetry.balance.impact_detected,
                "gravity-compensated quiet stance must not flag impact at tick {tick}"
            );
        }
    }

    #[test]
    fn impact_flag_never_latches_fall_cutoff_and_yields_to_ukemi() {
        let mut kernel = impact_test_kernel();
        let mut cmd = HostToMcu {
            seq: 1,
            mode: ControlMode::Drive,
            torque_mask: (1u64 << 21) - 1,
            walk: WalkCmd {
                balance_enabled: true,
                ..Default::default()
            },
            ..Default::default()
        };
        let mut fb = RawServoFeedback::default();
        fb.imu.available = true;
        fb.imu.pitch_rad = 0.10_f32; // well below ukemi/fall thresholds
        for c in fb.current_ma.iter_mut() {
            *c = 200;
        }

        // Seed the current baseline with a quiet sample first.
        let mut out = kernel.step(5, 0.005_f32, Some(&cmd), &fb);
        fb.current_ma[11] = 800;
        for tick in 2..=3 {
            cmd.seq = tick;
            out = kernel.step(tick * 5, 0.005_f32, Some(&cmd), &fb);
        }
        assert!(out.telemetry.balance.impact_detected);
        assert_eq!(out.telemetry.active_mode, ControlMode::Drive);
        assert!(!out.telemetry.faults.contains(FaultFlags::TILT_FALL_CUTOFF));
        assert!(!out.telemetry.faults.contains(FaultFlags::ESTOP_ACTIVE));

        // Ukemi takes priority: the impact channel is suppressed while the
        // pre-fall torque relaxation is active, but the fall logic is intact.
        fb.imu.pitch_rad = 0.60_f32;
        cmd.seq = 5;
        let ukemi = kernel.step(25, 0.005_f32, Some(&cmd), &fb);
        assert!(ukemi.telemetry.estimator.ukemi_active);
        assert!(!ukemi.telemetry.balance.impact_detected);
        assert_eq!(ukemi.telemetry.active_mode, ControlMode::Drive);
    }

    #[test]
    fn runtime_tuning_cmds_update_uvc_limits_and_impact_with_clamping() {
        let mut kernel = DirectServoKernel::new(21, &[JointCalibration::default(); MAX_JOINTS]);
        let fb = RawServoFeedback::default();

        let mut gait = kernel.uvc_gait_config();
        gait.max_foot_lift = 0.045;
        gait.step_duration_s = 0.25;
        gait.toe_push_off_gain = 0.30;
        gait.waist_balance_gain = 0.50;
        gait.arm_swing_gain = 1.40;
        let cmd1 = HostToMcu {
            seq: 1,
            config: ConfigCmd::SetUvcGait(gait),
            ..Default::default()
        };
        kernel.step(5, 0.005, Some(&cmd1), &fb);
        assert_eq!(kernel.uvc_gait_config(), gait);

        let hw_max_t = kernel.profile().max_torque_nm[2];
        let hw_max_v = kernel.profile().max_velocity_rad_s[2];
        let cmd2 = HostToMcu {
            seq: 2,
            config: ConfigCmd::SetActuatorLimits {
                joint: 2,
                max_torque_nm: 999.0,
                max_velocity_rad_s: 0.1,
            },
            ..Default::default()
        };
        kernel.step(10, 0.005, Some(&cmd2), &fb);
        assert_eq!(kernel.calibration()[2].max_torque_nm, hw_max_t);
        assert_eq!(kernel.calibration()[2].max_velocity_rad_s, 0.5_f32.min(hw_max_v));

        let ic = ImpactWireConfig {
            torque_capable: true,
            k_o: 120.0,
            threshold_nm: 0.35,
            debounce_ticks: 4,
            refractory_s: 0.12,
            current_spike_ma: 350.0,
            ma_per_nm: 1200.0,
        };
        let cmd3 = HostToMcu {
            seq: 3,
            config: ConfigCmd::SetImpactConfig(ic),
            ..Default::default()
        };
        kernel.step(15, 0.005, Some(&cmd3), &fb);
        assert_eq!(kernel.impact_wire_config(), ic);
    }
}
