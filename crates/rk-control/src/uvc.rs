//! Pure `#![no_std]` Geometric UVC (Upper Body Vertical Control) & Analytic 6-DoF Leg IK.
//!
//! Ported and generalized from Dr. Guero's ai2001 KHR UVC algorithm (`https://ai2001.ifdef.jp/`):
//! - Zero heap allocations, strictly single-precision `f32` via `libm` for RP2350 Cortex-M33F parity.
//! - Closed-form 6-DoF leg IK (`solve_leg_ik`) guaranteeing flat soles (`hip_pitch + knee_pitch + ankle_pitch == 0`
//!   and `hip_roll + ankle_roll == 0`).
//! - Full 6-stage UVC state machine (`CrouchInit` -> `CalibrateImu` -> `ReadyWatch` -> `ActiveCatchStep` ->
//!   `PostStepSettle` -> `ReturnToNeutral` -> `RecoverySettle`) plus protective `FallTuck` before cutoff.
//! - Rate-independent `dt` scaling normalized to the 100 Hz (`10 ms`) ai2001 reference tick so
//!   200 Hz RP2350 firmware and browser/desktop simulation share identical physical time constants.

use core::f32::consts::PI;

use libm::{acosf, asinf, atan2f, cosf, logf, sinf, sqrtf};
use rk_proto::{
    BalanceGains, EstimatorTelemetry, ImuState, WalkCmd, classify_fall, estimator_valid,
    fall_orientation, uvc_phase,
};
use rk_robot_profile::LegGeometry;

use crate::estimator::DcmSwingHint;

/// Reference tick duration of ai2001 `main.c` (`10 ms` = 100 Hz).
const REF_TICK_SEC: f32 = 0.010_f32;
/// Duration of protective servo tuck pose before torque drop (`40 ms`).
const FALL_TUCK_DURATION_SEC: f32 = 0.040_f32;
/// Maximum knee-triangle half-angle (`60 deg` = `PI / 3`, matching ai2001 `k > 60*PI/180`).
const MAX_HALF_KNEE_RAD: f32 = 1.047_197_6_f32;
/// Gravitational acceleration for the LIPM natural frequency `ω = sqrt(g / h_com)`.
const GRAVITY_M_S2: f32 = 9.81_f32;
/// Shortest/longest support half-cycle allowed by step-time adaptation (seconds).
/// The lower bound keeps the 200 Hz gait clock below one π boundary per tick;
/// the upper bound prevents a near-zero cadence from freezing the walk.
const STEP_DUR_MIN_SEC: f32 = 0.16_f32;
const STEP_DUR_MAX_SEC: f32 = 0.45_f32;

/// Khadiv capture-error step-time adaptation, shared by the catch-step timer and
/// the walking gait clock so both advance from the same law:
/// `T = clamp(0.8 · ln(1 + margin / |ξ − p|) / ω, T_min, T_nom)`.
///
/// `nom` is the undisturbed half-cycle duration; without a capture breach it is
/// returned unchanged (clamped to the admissible range). A breach shortens the
/// half-cycle toward `T_min` so recovery/landing arrives sooner.
fn adapt_step_duration(
    capture_valid: bool,
    capture_error: f32,
    margin: f32,
    omega: f32,
    nom: f32,
) -> f32 {
    let nom = nom.clamp(STEP_DUR_MIN_SEC, STEP_DUR_MAX_SEC);
    if capture_valid && capture_error > margin {
        let ratio = margin / capture_error.max(1e-6_f32);
        (0.8_f32 * logf(1.0_f32 + ratio) / omega).clamp(STEP_DUR_MIN_SEC, nom)
    } else {
        nom
    }
}

/// Solved single-leg 6/7-DoF joint angles in URDF sign conventions (radians).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LegIkAngles {
    pub hip_yaw: f32,
    pub hip_roll: f32,
    pub hip_pitch: f32,
    pub knee_pitch: f32,
    pub ankle_pitch: f32,
    pub ankle_roll: f32,
    pub toe_pitch: f32,
    /// Clamped height if the requested `(x, y, h)` exceeded `max_reach`.
    pub clamped_h: f32,
}

/// Closed-form 6-DoF analytic leg IK (`footCont` from ai2001).
///
/// - `x`: foot sagittal offset relative to hip (`+x` = forward).
/// - `y_out`: foot lateral offset outward from pelvis centerline (`+y_out` = outward).
/// - `h`: vertical distance from hip-roll pivot to ankle-roll pivot.
/// - `is_left`: `true` for Left leg (`+X` side), `false` for Right leg (`-X` side).
///
/// Guarantees:
/// - `hip_pitch + knee_pitch + ankle_pitch == 0.0` (foot sole stays strictly horizontal in pitch).
/// - `hip_roll + ankle_roll == 0.0` (foot sole stays strictly flat in roll).
pub fn solve_leg_ik(
    x: f32,
    y_out: f32,
    h: f32,
    hip_yaw: f32,
    is_left: bool,
    geom: &LegGeometry,
) -> LegIkAngles {
    let safe_h = h.max(geom.min_height * 0.5);
    let r_lat = sqrtf(y_out * y_out + safe_h * safe_h);
    let lateral_h = (r_lat - geom.hip_ankle_offset).max(0.02_f32);
    let roll_ang = atan2f(y_out, safe_h);

    let mut d = sqrtf(x * x + lateral_h * lateral_h);
    let mut clamped_h = safe_h;
    if d > geom.max_reach {
        d = geom.max_reach;
        let rem_sq = (d * d - x * x).max(0.0004_f32);
        let max_lat_h = sqrtf(rem_sq) + geom.hip_ankle_offset;
        let h_sq = (max_lat_h * max_lat_h - y_out * y_out).max(0.0004_f32);
        clamped_h = sqrtf(h_sq).max(geom.min_height);
    }

    let l1 = geom.thigh_len;
    let l2 = geom.shin_len;
    let d_safe = d.clamp(
        (l1 - l2).abs() + 0.01_f32,
        (l1 + l2 - 0.0005_f32).min(geom.max_reach),
    );

    let x_ang = asinf((x / d_safe).clamp(-0.999_f32, 0.999_f32));
    let cos_hip =
        ((l1 * l1 + d_safe * d_safe - l2 * l2) / (2.0_f32 * l1 * d_safe)).clamp(-1.0_f32, 1.0_f32);
    let cos_knee =
        ((l1 * l1 + l2 * l2 - d_safe * d_safe) / (2.0_f32 * l1 * l2)).clamp(-1.0_f32, 1.0_f32);

    let hip_bend = acosf(cos_hip).min(MAX_HALF_KNEE_RAD);
    let knee_bend = (PI - acosf(cos_knee)).clamp(0.0_f32, 2.0_f32 * MAX_HALF_KNEE_RAD);
    let ankle_bend = (knee_bend - hip_bend).max(0.0_f32);

    // Rocky URDF convention: hip_pitch < 0 forward, knee_pitch > 0 bend, ankle_pitch < 0 dorsiflexion
    let hip_pitch = -(hip_bend + x_ang);
    let knee_pitch = knee_bend;
    let ankle_pitch = -(ankle_bend - x_ang);

    let (hip_roll, ankle_roll) = if is_left {
        (roll_ang, -roll_ang)
    } else {
        (-roll_ang, roll_ang)
    };

    LegIkAngles {
        hip_yaw,
        hip_roll,
        hip_pitch,
        knee_pitch,
        ankle_pitch,
        ankle_roll,
        toe_pitch: 0.0,
        clamped_h,
    }
}

/// Active UVC state machine phase.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum UvcPhase {
    #[default]
    Idle,
    CrouchInit,
    CalibrateImu,
    ReadyWatch,
    ActiveCatchStep,
    PostStepSettle,
    ReturnToNeutral,
    RecoverySettle,
    FallTuck,
}

impl UvcPhase {
    pub const fn to_wire(self) -> u8 {
        match self {
            Self::Idle => uvc_phase::IDLE,
            Self::CrouchInit => uvc_phase::CROUCH_INIT,
            Self::CalibrateImu => uvc_phase::CALIBRATE_IMU,
            Self::ReadyWatch => uvc_phase::READY_WATCH,
            Self::ActiveCatchStep => uvc_phase::ACTIVE_CATCH_STEP,
            Self::PostStepSettle => uvc_phase::POST_STEP_SETTLE,
            Self::ReturnToNeutral => uvc_phase::RETURN_TO_NEUTRAL,
            Self::RecoverySettle => uvc_phase::RECOVERY_SETTLE,
            Self::FallTuck => uvc_phase::FALL_TUCK,
        }
    }
}

/// Output produced by a single deterministic tick of [`UvcController`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct UvcStepOutput {
    pub left_leg: LegIkAngles,
    pub right_leg: LegIkAngles,
    pub waist_yaw: f32,
    pub waist_pitch: f32,
    pub waist_roll: f32,
    pub l_shoulder_pitch_delta: f32,
    pub l_shoulder_roll_delta: f32,
    pub r_shoulder_pitch_delta: f32,
    pub r_shoulder_roll_delta: f32,
    /// Equivalent pitch/roll correction signals (radians) for telemetry & ankle damping.
    pub pitch_corr_rad: f32,
    pub roll_corr_rad: f32,
    /// Continuous walk/step phase in `[0, TAU)` for UI/sim diagnostics.
    pub gait_phase_rad: f32,
    pub phase: UvcPhase,
    /// Support foot (`0 = Right`, `1 = Left`).
    pub support_foot: u8,
    /// Support foot sagittal offset (`dxi`, meters; `+` forward).
    pub foot_dx_m: f32,
    /// Signed lateral foot offset (meters; `+` right, `-` left).
    pub foot_dy_m: f32,
    /// Current hip-to-ankle height (`auto_h`, meters).
    pub hip_height_m: f32,
    /// True during protective tuck before torque drop.
    pub in_fall_tuck: bool,
    /// True once protective tuck duration completes -> caller must drop torque & latch `TILT_FALL_CUTOFF`.
    pub fall_cutoff_ready: bool,
    /// Orientation classified from the IMU when the protective tuck engaged
    /// (`fall_orientation::*` wire code), forwarded into telemetry on cutoff latch.
    pub fall_orientation: u8,
    pub walk_active: bool,
}

/// Pure `#![no_std]` geometric UVC controller mirroring Dr. Guero's ai2001 state machine.
#[derive(Clone, Debug)]
pub struct UvcController {
    geom: LegGeometry,
    z_up: bool,
    phase: UvcPhase,
    phase_elapsed_sec: f32,
    /// `0 = Right` support foot, `1 = Left` support foot.
    support_foot: u8,
    /// Support foot sagittal offset (`dxi`, meters; `+` forward).
    dxi: f32,
    /// Support foot outward lateral offset (`dyi`, meters; `>= 0`).
    dyi: f32,
    /// Swing foot sagittal offset (`dxis`, meters).
    dxis: f32,
    /// Swing foot outward lateral offset (`dyis`, meters).
    dyis: f32,
    /// Active hip height (`autoH`, meters).
    auto_h: f32,
    /// Lateral pelvis sway (`sw`, meters).
    sway: f32,
    sway_max: f32,
    /// Waist/hip yaw coupling (`wt`, radians) and walk swing offset (`wk`, meters).
    waist_yaw: f32,
    wk: f32,
    /// Auto-zeroed IMU attitude bias (radians).
    pitch_bias: f32,
    roll_bias: f32,
    cal_sum_pitch: f32,
    cal_sum_roll: f32,
    cal_samples: u32,
    /// Filtered step progress phase for telemetry.
    gait_phase_rad: f32,
    /// Continuous gait clock in `[0, 2π)`. Advanced by `π/step_dur · dt`
    /// whenever `walk.walk_enabled`, where `step_dur` is the Khadiv-adapted
    /// support half-cycle (nominal `0.5/cadence` when undisturbed); its
    /// half-cycle selects the support foot (right in `[0, π)`, left in
    /// `[π, 2π)`). Phase-locked to the synthetic catch-step phase on
    /// `ActiveCatchStep` entry.
    phase_accum_rad: f32,
    /// Smoothly blends leg stance when UVC engages.
    initialized: bool,
    /// Snapshot of initial offsets when entering `ReturnToNeutral`.
    ret_start_dxi: f32,
    ret_start_dyi: f32,
    ret_start_dxis: f32,
    ret_start_dyis: f32,
    ret_start_h: f32,
    ret_start_wt: f32,
}

impl UvcController {
    pub const fn new(geom: LegGeometry) -> Self {
        Self::new_for_profile(geom, false)
    }

    pub const fn new_for_profile(geom: LegGeometry, z_up: bool) -> Self {
        Self {
            geom,
            z_up,
            phase: UvcPhase::Idle,
            phase_elapsed_sec: 0.0,
            support_foot: 0,
            dxi: 0.0,
            dyi: 0.0,
            dxis: 0.0,
            dyis: 0.0,
            auto_h: geom.nominal_height,
            sway: 0.0,
            sway_max: 0.0,
            waist_yaw: 0.0,
            wk: 0.0,
            pitch_bias: 0.0,
            roll_bias: 0.0,
            cal_sum_pitch: 0.0,
            cal_sum_roll: 0.0,
            cal_samples: 0,
            gait_phase_rad: 0.0,
            phase_accum_rad: 0.0,
            initialized: false,
            ret_start_dxi: 0.0,
            ret_start_dyi: 0.0,
            ret_start_dxis: 0.0,
            ret_start_dyis: 0.0,
            ret_start_h: geom.nominal_height,
            ret_start_wt: 0.0,
        }
    }

    pub const fn geometry(&self) -> &LegGeometry {
        &self.geom
    }

    pub const fn phase(&self) -> UvcPhase {
        self.phase
    }

    /// Swing-foot landing target `(dxis, dyis)` in meters for the most recent tick.
    ///
    /// Exposed for deterministic replay/unit tests so the capture law can be inspected
    /// directly without inferring it through the leg IK. The stance hip is still reported
    /// by [`UvcStepOutput::foot_dx_m`] / [`UvcStepOutput::foot_dy_m`] and is never written
    /// by the capture law.
    pub const fn swing_targets(&self) -> (f32, f32) {
        (self.dxis, self.dyis)
    }

    /// Auto-zeroed sagittal IMU bias in radians (clamped to `[-0.08, 0.08]`).
    ///
    /// Exposed so the kernel can share the calibrated attitude with the estimator
    /// with a one-tick lag (the estimator runs before UVC in each control tick).
    pub const fn pitch_bias(&self) -> f32 {
        self.pitch_bias
    }

    /// Auto-zeroed lateral IMU bias in radians (clamped to `[-0.08, 0.08]`).
    pub const fn roll_bias(&self) -> f32 {
        self.roll_bias
    }

    /// Bias-compensated torso attitude `(pitch_rad, roll_rad)` derived from an
    /// IMU sample using the most recently calibrated bias.
    ///
    /// The estimator runs before UVC in each tick, so this reflects the bias
    /// computed on the previous tick — the intended 1-tick-lagged sharing from
    /// the P1 design. No state-machine refactor is required.
    pub fn bias_compensated_attitude(&self, imu: &ImuState) -> (f32, f32) {
        (
            imu.pitch_rad - self.pitch_bias,
            imu.roll_rad - self.roll_bias,
        )
    }

    /// Foot contact bitmask (`bit 0 = Right stance`, `bit 1 = Left stance`) for air-gating CoM velocity.
    pub const fn contact_mask(&self) -> u8 {
        match self.phase {
            UvcPhase::FallTuck => 0b00,
            UvcPhase::ActiveCatchStep | UvcPhase::ReturnToNeutral => {
                if self.support_foot == 0 {
                    0b01
                } else {
                    0b10
                }
            }
            _ => 0b11,
        }
    }

    /// Resets UVC state cleanly on `Teach`, `EStop`, or `ClearFaults`.
    pub fn reset(&mut self) {
        self.phase = UvcPhase::Idle;
        self.phase_elapsed_sec = 0.0;
        self.support_foot = 0;
        self.dxi = 0.0;
        self.dyi = 0.0;
        self.dxis = 0.0;
        self.dyis = 0.0;
        self.auto_h = self.geom.nominal_height;
        self.sway = 0.0;
        self.sway_max = 0.0;
        self.waist_yaw = 0.0;
        self.wk = 0.0;
        self.cal_sum_pitch = 0.0;
        self.cal_sum_roll = 0.0;
        self.cal_samples = 0;
        self.gait_phase_rad = 0.0;
        self.phase_accum_rad = 0.0;
        self.initialized = false;
    }

    /// Executes one deterministic `#![no_std]` UVC control cycle without external DCM modulation.
    pub fn step(
        &mut self,
        dt: f32,
        imu: ImuState,
        walk: WalkCmd,
        gains: BalanceGains,
    ) -> UvcStepOutput {
        self.step_with_dcm(
            dt,
            imu,
            walk,
            gains,
            DcmSwingHint::default(),
            EstimatorTelemetry::default(),
        )
    }

    /// Executes one deterministic `#![no_std]` UVC control cycle with P1a strict DOF partitioning:
    /// - Stance torso verticality -> UVC I-only (`dxi`/`dyi` spherical integration).
    /// - High-frequency vibration damping -> decoupled ankle rate-gyro P-term.
    /// - Capturability / stepping -> the estimator's capture point drives swing-foot landing
    ///   (`dxis`/`dyis`) ONLY; the stance hip is never touched by the capture law.
    ///
    /// `estimator` must carry a valid `estimator_valid::DCM` channel for the capture law to run;
    /// otherwise the legacy `DcmSwingHint` fallback is used. The lossy hint is deliberately
    /// never reverse-engineered back into a capture point.
    pub fn step_with_dcm(
        &mut self,
        dt: f32,
        imu: ImuState,
        walk: WalkCmd,
        gains: BalanceGains,
        dcm: DcmSwingHint,
        estimator: EstimatorTelemetry,
    ) -> UvcStepOutput {
        let dt = dt.clamp(0.0005_f32, 0.05_f32);
        let tick_scale = dt / REF_TICK_SEC;

        // Target nominal height modulated by combat crouch slider
        let crouch = walk.stance_crouch.clamp(0.0_f32, 1.0_f32);
        let target_h = (self.geom.nominal_height
            - crouch * (self.geom.nominal_height - self.geom.min_height) * 0.45_f32)
            .clamp(self.geom.min_height, self.geom.nominal_height);

        // 1. Protective Fall-Tuck detection (ai2001 `detAng`) — always active whenever Drive + legs present
        let fall_exceeded = imu.pitch_rad.abs() >= gains.fall_cutoff_rad
            || imu.roll_rad.abs() >= gains.fall_cutoff_rad;
        if fall_exceeded && self.phase != UvcPhase::FallTuck {
            self.phase = UvcPhase::FallTuck;
            self.phase_elapsed_sec = 0.0;
        }

        if self.phase == UvcPhase::FallTuck {
            self.phase_elapsed_sec += dt;
            let tuck_leg = solve_leg_ik(0.0, 0.015, self.geom.min_height, 0.0, true, &self.geom);
            let tuck_leg_r = solve_leg_ik(0.0, 0.015, self.geom.min_height, 0.0, false, &self.geom);
            return UvcStepOutput {
                left_leg: tuck_leg,
                right_leg: tuck_leg_r,
                waist_yaw: 0.0,
                waist_pitch: 0.0,
                waist_roll: 0.0,
                l_shoulder_pitch_delta: -0.45,
                l_shoulder_roll_delta: -0.15,
                r_shoulder_pitch_delta: -0.45,
                r_shoulder_roll_delta: 0.15,
                pitch_corr_rad: 0.0,
                roll_corr_rad: 0.0,
                gait_phase_rad: self.gait_phase_rad,
                phase: UvcPhase::FallTuck,
                support_foot: self.support_foot,
                foot_dx_m: self.dxi,
                foot_dy_m: self.signed_dy(),
                hip_height_m: self.geom.min_height,
                in_fall_tuck: true,
                fall_cutoff_ready: self.phase_elapsed_sec >= FALL_TUCK_DURATION_SEC,
                fall_orientation: classify_fall(&imu).to_wire(),
                walk_active: false,
            };
        }

        // 2. Initialize state machine when entering Drive with balance or walk enabled
        if !self.initialized || self.phase == UvcPhase::Idle {
            self.initialized = true;
            self.auto_h = target_h;
            self.phase = UvcPhase::CrouchInit;
            self.phase_elapsed_sec = 0.0;
            self.cal_sum_pitch = 0.0;
            self.cal_sum_roll = 0.0;
            self.cal_samples = 0;
        }

        // 3. Bias-compensated tilt & radial deadband (matching ai2001 `uvc()`)
        let raw_pitch = imu.pitch_rad - self.pitch_bias;
        let raw_roll = imu.roll_rad - self.roll_bias;
        let (pitch_eff, roll_eff) = apply_radial_deadband(
            raw_pitch,
            raw_roll,
            gains.tilt_deadband_rad.clamp(0.005_f32, 0.15_f32),
        );

        let vx = walk.vx.clamp(-1.0_f32, 1.0_f32);
        let vy = walk.vy.clamp(-1.0_f32, 1.0_f32);
        let yaw_cmd = walk.yaw_rate.clamp(-1.0_f32, 1.0_f32);
        let walk_vel_mag = sqrtf(vx * vx + vy * vy + yaw_cmd * yaw_cmd);
        let wants_walk = walk.walk_enabled && walk_vel_mag > 0.04_f32;
        let dcm_step_trigger = walk.balance_enabled && dcm.margin_breached;
        let tilt_trigger = walk.balance_enabled
            && (roll_eff.abs() > 0.001_f32 || pitch_eff.abs() > 0.004_f32 || dcm_step_trigger);

        // Capture-point landing law (swing foot only). ξ and ċ come straight from the
        // estimator's authoritative telemetry — never reconstructed from the lossy hint.
        //   p = ξ_rel + K_v * (v_des - v_com) / ω,  ω = sqrt(g / h_com),  v_des = v_cmd * 0.5 m/s
        // Default gains give h_com = 0.72 m -> ω ≈ 3.69 rad/s, matching the
        // FK-derived sole-to-CoM height of the bundled model. K_v = dcm_step_gain.
        let h_com = if gains.h_com_m.is_finite() {
            gains.h_com_m.clamp(0.12_f32, 0.80_f32)
        } else {
            0.25_f32
        };
        let omega = sqrtf(GRAVITY_M_S2 / h_com);
        let capture_valid = estimator.valid_mask & estimator_valid::DCM != 0;
        let (capture_sag, capture_lat, capture_error) = if capture_valid {
            let k_v = gains.dcm_step_gain.clamp(0.0_f32, 1.5_f32);
            let xi_sag = estimator.dcm_xy_m[1];
            let xi_lat = estimator.dcm_xy_m[0];
            let (sag_vel_idx, lat_vel_idx) = if self.z_up { (0, 1) } else { (2, 0) };
            let p_sag = xi_sag + k_v * (vx * 0.5_f32 - estimator.com_vel_m_s[sag_vel_idx]) / omega;
            let p_lat = xi_lat + k_v * (vy * 0.5_f32 - estimator.com_vel_m_s[lat_vel_idx]) / omega;
            let err_sag = xi_sag - p_sag;
            let err_lat = xi_lat - p_lat;
            (p_sag, p_lat, sqrtf(err_sag * err_sag + err_lat * err_lat))
        } else {
            (0.0_f32, 0.0_f32, 0.0_f32)
        };

        let step_h_scale = walk.step_height.clamp(0.1_f32, 1.0_f32);
        let cadence = walk.cadence_hz.clamp(0.6_f32, 2.8_f32);
        let walking_clock = walk.walk_enabled;
        // Nominal support half-cycle = one gait half-cycle at the commanded cadence.
        // The catch-step timer keeps its ai2001 18-tick @ 10 ms = 0.18 s duration,
        // while the walking clock always uses the cadence-derived duration even
        // if velocity is too small to actually walk.
        let walk_step_dur_nom = (0.5_f32 / cadence).clamp(STEP_DUR_MIN_SEC, STEP_DUR_MAX_SEC);
        let step_dur_nom = if wants_walk {
            walk_step_dur_nom
        } else {
            0.18_f32
        };
        // Khadiv step-time adaptation: a capture error beyond the usable foot
        // margin shortens both the timer-driven recovery step and the walking
        // half-cycle. The clock remains the single source of truth for walking
        // phase; `step_dur` scales how fast its π boundary arrives.
        //
        // During steady walking a non-zero Raibert velocity correction is
        // normal and must NOT saturate the clock at the minimum half-cycle
        // (that swaps support before the lagged actuator lands the swing foot,
        // producing flight). Walking therefore adapts only on a genuine
        // support-polygon breach signalled by the estimator; catch steps keep
        // the un-gated Khadiv response to their own capture error.
        let adapt_step = !walking_clock || dcm.margin_breached;
        let foot_half = gains.foot_half_length_m.clamp(0.015_f32, 0.12_f32);
        let margin = if gains.foot_margin_m.is_finite() {
            gains.foot_margin_m.clamp(0.010_f32, foot_half)
        } else {
            0.028_f32
        };
        let step_dur = if adapt_step {
            adapt_step_duration(capture_valid, capture_error, margin, omega, step_dur_nom)
        } else {
            step_dur_nom
        };
        let walk_step_dur = if adapt_step {
            // Even on a breach, keep at least 85% of the nominal half-cycle:
            // the lagged actuator needs that tail to plant the swing sole
            // before the π boundary swaps support.
            adapt_step_duration(capture_valid, capture_error, margin, omega, walk_step_dur_nom)
                .max(0.85_f32 * walk_step_dur_nom)
        } else {
            walk_step_dur_nom
        };

        // Gait clock: `π/step_dur · dt` advances exactly π per undisturbed
        // half-cycle (nominal cadence), and faster when a capture breach
        // shortens `step_dur`. The accumulator is the single source of truth
        // for walking phase and runs through every state while walking is
        // engaged. `dt ≤ 0.05` and `step_dur ≥ 0.16` bound one increment to
        // `< π`, so a single subtraction wraps into `[0, 2π)`.
        const TWO_PI: f32 = 2.0_f32 * PI;
        if walking_clock {
            self.phase_accum_rad += (PI / walk_step_dur) * dt;
            if self.phase_accum_rad >= TWO_PI {
                self.phase_accum_rad -= TWO_PI;
            }
        }

        let mut swing_lift_m = 0.0_f32;

        // 4. Advance 6-stage ai2001 UVC state machine
        self.phase_elapsed_sec += dt;
        match self.phase {
            UvcPhase::Idle => {
                self.phase = UvcPhase::CrouchInit;
                self.phase_elapsed_sec = 0.0;
            }
            UvcPhase::CrouchInit => {
                // Smoothly adopt nominal combat crouch height
                let alpha = (dt * 12.0_f32).clamp(0.0_f32, 1.0_f32);
                self.auto_h += (target_h - self.auto_h) * alpha;
                if tilt_trigger || wants_walk {
                    // Immediate catch response if disturbed during startup
                    self.enter_catch_step(raw_roll, vy, wants_walk);
                } else if self.phase_elapsed_sec >= 0.08_f32 {
                    self.phase = UvcPhase::CalibrateImu;
                    self.phase_elapsed_sec = 0.0;
                    self.cal_sum_pitch = 0.0;
                    self.cal_sum_roll = 0.0;
                    self.cal_samples = 0;
                }
            }
            UvcPhase::CalibrateImu => {
                // Auto-zero IMU bias when quiet (matching ai2001 mode 720)
                if tilt_trigger || wants_walk {
                    self.enter_catch_step(raw_roll, vy, wants_walk);
                } else {
                    if imu.pitch_rad.abs() < 0.12_f32
                        && imu.roll_rad.abs() < 0.12_f32
                        && imu.pitch_rate_rad_s.abs() < 0.25_f32
                        && imu.roll_rate_rad_s.abs() < 0.25_f32
                    {
                        self.cal_sum_pitch += imu.pitch_rad;
                        self.cal_sum_roll += imu.roll_rad;
                        self.cal_samples = self.cal_samples.saturating_add(1);
                    }
                    if self.phase_elapsed_sec >= 0.12_f32 {
                        if self.cal_samples >= 4 {
                            let inv = 1.0_f32 / (self.cal_samples as f32);
                            self.pitch_bias = (self.cal_sum_pitch * inv).clamp(-0.08_f32, 0.08_f32);
                            self.roll_bias = (self.cal_sum_roll * inv).clamp(-0.08_f32, 0.08_f32);
                        }
                        self.phase = UvcPhase::ReadyWatch;
                        self.phase_elapsed_sec = 0.0;
                    }
                }
            }
            UvcPhase::ReadyWatch => {
                // Smoothly track requested combat crouch height while watching IMU
                let alpha = (dt * 8.0_f32).clamp(0.0_f32, 1.0_f32);
                self.auto_h += (target_h - self.auto_h) * alpha;
                self.dxi *= (1.0_f32 - 8.0_f32 * dt).max(0.0_f32);
                self.dyi *= (1.0_f32 - 8.0_f32 * dt).max(0.0_f32);
                self.dxis *= (1.0_f32 - 8.0_f32 * dt).max(0.0_f32);
                self.dyis *= (1.0_f32 - 8.0_f32 * dt).max(0.0_f32);
                self.sway *= (1.0_f32 - 8.0_f32 * dt).max(0.0_f32);
                self.waist_yaw *= (1.0_f32 - 8.0_f32 * dt).max(0.0_f32);

                // Slow zero-drift tracking when strictly stationary inside deadband
                if raw_pitch.abs() < gains.tilt_deadband_rad * 0.5_f32
                    && raw_roll.abs() < gains.tilt_deadband_rad * 0.5_f32
                    && imu.pitch_rate_rad_s.abs() < 0.08_f32
                    && imu.roll_rate_rad_s.abs() < 0.08_f32
                {
                    let drift = (0.04_f32 * dt).clamp(0.0_f32, 0.005_f32);
                    self.pitch_bias =
                        (self.pitch_bias + raw_pitch * drift).clamp(-0.08_f32, 0.08_f32);
                    self.roll_bias = (self.roll_bias + raw_roll * drift).clamp(-0.08_f32, 0.08_f32);
                }

                if tilt_trigger || wants_walk {
                    self.enter_catch_step(raw_roll, vy, wants_walk);
                }
            }
            UvcPhase::ActiveCatchStep => {
                let progress = if walking_clock {
                    // The free-running phase clock drives support swaps: which
                    // half of the cycle the accumulator occupies is the stance
                    // foot, and progress is the position within that half.
                    let phase_support = if self.phase_accum_rad >= PI {
                        1_u8
                    } else {
                        0_u8
                    };
                    if phase_support != self.support_foot {
                        // Half-cycle boundary crossed -> foot landing. Apply
                        // the same CoP bleed as the timer-driven landing path.
                        self.support_foot = phase_support;
                        self.phase_elapsed_sec = 0.0;
                        self.dxi *= 0.65_f32;
                        self.dyi *= 0.65_f32;
                        self.dxis = -self.dxi;
                        self.dyis = self.dyi;
                    }
                    let p = if self.support_foot == 0 {
                        self.phase_accum_rad / PI
                    } else {
                        (self.phase_accum_rad - PI) / PI
                    };
                    p.clamp(0.0_f32, 1.0_f32)
                } else {
                    (self.phase_elapsed_sec / step_dur).clamp(0.0_f32, 1.0_f32)
                };
                self.gait_phase_rad = if walking_clock {
                    self.phase_accum_rad
                } else if self.support_foot == 0 {
                    progress * PI
                } else {
                    PI + progress * PI
                };

                // Core ai2001 geometric spherical UVC projection with Capture-Point rate lead
                if walk.balance_enabled {
                    self.integrate_uvc_spherical(
                        pitch_eff,
                        roll_eff,
                        imu.pitch_rate_rad_s,
                        imu.roll_rate_rad_s,
                        gains,
                        tick_scale,
                        dt,
                    );
                }

                // Superpose commanded walking stride (`WalkCmd`) onto UVC foot landing target
                if wants_walk {
                    let target_walk_x = vx * self.geom.max_dx * 0.68_f32;
                    let lat_sign = if self.support_foot == 0 {
                        -1.0_f32
                    } else {
                        1.0_f32
                    };
                    let target_walk_y = (vy * lat_sign * self.geom.max_dy * 0.55_f32).max(0.0_f32);
                    let walk_blend = (dt * 7.5_f32).clamp(0.0_f32, 1.0_f32);
                    self.dxi = (self.dxi + (target_walk_x - self.dxi) * walk_blend)
                        .clamp(-self.geom.max_dx, self.geom.max_dx);
                    self.dyi = (self.dyi + (target_walk_y - self.dyi) * walk_blend)
                        .clamp(0.0_f32, self.geom.max_dy);
                }

                // Swing-foot landing ONLY. With a valid DCM channel the capture target
                // (ξ + K_v·(v_des − v_com)/ω) is authoritative; the legacy hint is a fallback.
                // `dxi`/`dyi` (stance hip) are never written here — UVC remains I-only.
                let swing_lat_sign = if (self.support_foot == 0) ^ self.z_up {
                    -1.0_f32
                } else {
                    1.0_f32
                };
                if capture_valid {
                    self.dxis = capture_sag.clamp(-self.geom.max_swing_dx, self.geom.max_swing_dx);
                    // Right-positive world lateral -> outward-from-pelvis on the swing side.
                    // Bidirectional: a negative capture offset places the swing sole
                    // inward instead of clamping it to the nominal hip line. That
                    // removes the one-sided lateral bias that used to push the CoP
                    // away from the body every step.
                    self.dyis = (capture_lat * swing_lat_sign)
                        .clamp(-self.geom.max_swing_dy, self.geom.max_swing_dy);
                } else {
                    let dcm_swing_y = dcm.swing_dy_m * swing_lat_sign;
                    self.dxis = (-self.dxi + dcm.swing_dx_m)
                        .clamp(-self.geom.max_swing_dx, self.geom.max_swing_dx);
                    self.dyis = (self.dyi + dcm_swing_y)
                        .clamp(-self.geom.max_swing_dy, self.geom.max_swing_dy);
                }
                // Keep the swing sole from crossing the stance sole even when
                // the capture law asks for a large inward placement.
                let anti_collide = self.dyi + self.dyis;
                if anti_collide < 0.0_f32 {
                    self.dyis -= anti_collide;
                }

                // Swing foot clearance envelope (dwell-shaped below)
                let lift_scale = if wants_walk {
                    step_h_scale
                } else {
                    // Scale catch-step lift with disturbance magnitude so tiny corrections don't stomp
                    let offset_norm = (sqrtf(self.dxi * self.dxi + self.dyi * self.dyi)
                        / self.geom.max_dx)
                        .clamp(0.28_f32, 1.0_f32);
                    step_h_scale * offset_norm
                };
                // Toe-off / heel-strike dwell: sin²(π·p) keeps the swing sole
                // near the ground while the support hand-off happens (p≈0/1)
                // but retains the same mid-swing peak clearance. The old pure
                // sine lifted too early and landed too late, so the plant saw
                // 0b00 flight ticks at every steady support swap.
                let lift_s = sinf(PI * progress);
                swing_lift_m = self.geom.max_foot_lift * lift_scale * lift_s * lift_s;

                // Pelvis waist/hip yaw twist (`feetCont1`: wt = 0.5 * atan2(dxi, dyi + half_hip))
                let wt_geom = 0.35_f32 * atan2f(self.dxi, self.dyi + self.geom.half_hip_spacing);
                self.waist_yaw = (wt_geom + yaw_cmd * 0.24_f32).clamp(-0.45_f32, 0.45_f32);
                self.wk = (self.dxi.abs() * 0.33_f32).min(self.geom.max_dx * 0.33_f32);

                // Step landing -> swap support/swing foot. While walking the
                // accumulator already performed the swap at the half-cycle
                // boundary above, so the timer-driven path is for catch steps only.
                if progress >= 1.0_f32 && !walking_clock {
                    self.support_foot ^= 1;
                    // Preserve landed CoP shift in the lean direction across support swap
                    let prev_dxi = self.dxi;
                    let prev_dyi = self.dyi;
                    self.dxi = prev_dxi * 0.65_f32;
                    self.dyi = prev_dyi * 0.65_f32;
                    self.dxis = -self.dxi;
                    self.dyis = self.dyi;

                    if wants_walk
                        || (walk.balance_enabled
                            && (pitch_eff.abs() > 0.015_f32 || roll_eff.abs() > 0.015_f32))
                    {
                        // Continue dynamic stepping until upright or walk released
                        self.phase = UvcPhase::ActiveCatchStep;
                        self.phase_elapsed_sec = 0.0;
                    } else {
                        self.phase = UvcPhase::PostStepSettle;
                        self.phase_elapsed_sec = 0.0;
                    }
                }
            }
            UvcPhase::PostStepSettle => {
                // Settle planted recovery stance (`mode 750`, ~0.22 s) while bleeding residual CoP offset
                let relax = (1.0_f32 - 5.0_f32 * dt).max(0.0_f32);
                self.dxi *= relax;
                self.dyi *= relax;
                self.dxis *= relax;
                self.dyis *= relax;
                if wants_walk
                    || (walk.balance_enabled
                        && (pitch_eff.abs() > gains.tilt_deadband_rad * 0.8_f32
                            || roll_eff.abs() > gains.tilt_deadband_rad * 0.8_f32))
                {
                    self.enter_catch_step(raw_roll, vy, wants_walk);
                } else if self.phase_elapsed_sec >= 0.22_f32 {
                    // Prepare ReturnToNeutral sway amplitude from residual foot offset
                    let resid = sqrtf(0.5_f32 * self.dxi * self.dxi + self.dyi * self.dyi);
                    self.sway_max = (resid * 0.65_f32).min(self.geom.max_sway);
                    self.ret_start_dxi = self.dxi;
                    self.ret_start_dyi = self.dyi;
                    self.ret_start_dxis = self.dxis;
                    self.ret_start_dyis = self.dyis;
                    self.ret_start_h = self.auto_h;
                    self.ret_start_wt = self.waist_yaw;
                    self.phase = UvcPhase::ReturnToNeutral;
                    self.phase_elapsed_sec = 0.0;
                }
            }
            UvcPhase::ReturnToNeutral => {
                // Smoothly pull feet back under hips (`mode 760`, ~0.42 s)
                let ret_dur = 0.42_f32;
                let p = (self.phase_elapsed_sec / ret_dur).clamp(0.0_f32, 1.0_f32);
                let remain = 1.0_f32 - p;
                self.dxi = self.ret_start_dxi * remain;
                self.dyi = self.ret_start_dyi * remain;
                self.dxis = self.ret_start_dxis * remain;
                self.dyis = self.ret_start_dyis * remain;
                self.auto_h = self.ret_start_h + (target_h - self.ret_start_h) * p;
                self.waist_yaw = self.ret_start_wt * remain;
                self.sway = self.sway_max * sinf(PI * p);
                swing_lift_m = self.geom.max_foot_lift * 0.35_f32 * sinf(PI * p);

                if wants_walk
                    || (walk.balance_enabled
                        && (pitch_eff.abs() > gains.tilt_deadband_rad
                            || roll_eff.abs() > gains.tilt_deadband_rad))
                {
                    self.enter_catch_step(raw_roll, vy, wants_walk);
                } else if p >= 1.0_f32 {
                    self.dxi = 0.0;
                    self.dyi = 0.0;
                    self.dxis = 0.0;
                    self.dyis = 0.0;
                    self.sway = 0.0;
                    self.waist_yaw = 0.0;
                    self.phase = UvcPhase::RecoverySettle;
                    self.phase_elapsed_sec = 0.0;
                }
            }
            UvcPhase::RecoverySettle => {
                // Final post-return stabilization dwell (`mode 770`, ~0.20 s)
                let alpha = (dt * 10.0_f32).clamp(0.0_f32, 1.0_f32);
                self.auto_h += (target_h - self.auto_h) * alpha;
                if wants_walk || tilt_trigger {
                    self.enter_catch_step(raw_roll, vy, wants_walk);
                } else if self.phase_elapsed_sec >= 0.20_f32 {
                    self.phase = UvcPhase::ReadyWatch;
                    self.phase_elapsed_sec = 0.0;
                }
            }
            UvcPhase::FallTuck => {}
        }

        // TODO (needs rk-proto): while `walk_enabled && !margin_breached` the
        // machine is really WALKING, but rk-proto has no distinct wire code and
        // `UvcStepOutput` cannot carry one. Consumers must use the existing
        // `balance.walk_active` field to disambiguate from a genuine catch step;
        // rk-sim's `WalkQuality.catch_step_fraction` does exactly that.
        // Telemetry always exposes the continuous accumulator while walking,
        // even in non-stepping states, so the phase never jumps between states.
        if walking_clock {
            self.gait_phase_rad = self.phase_accum_rad;
        }

        // 5. Solve 6-DoF Analytic Leg IK for support and swing legs
        let sup_x = self.dxi;
        let sup_y = (self.dyi - self.sway).max(0.0_f32);
        let sup_h = self
            .auto_h
            .clamp(self.geom.min_height, self.geom.nominal_height);

        let swg_x = self.dxis;
        let swg_y = (self.dyis + self.sway).clamp(-self.geom.max_swing_dy, self.geom.max_swing_dy);
        let swg_h = (self.auto_h - swing_lift_m)
            .clamp(self.geom.min_height * 0.85_f32, self.geom.nominal_height);

        let (mut left_leg, mut right_leg) = if self.support_foot == 1 {
            // Left is support, Right is swing
            (
                solve_leg_ik(sup_x, sup_y, sup_h, -self.waist_yaw, true, &self.geom),
                solve_leg_ik(swg_x, swg_y, swg_h, self.waist_yaw, false, &self.geom),
            )
        } else {
            // Right is support, Left is swing
            (
                solve_leg_ik(swg_x, swg_y, swg_h, -self.waist_yaw, true, &self.geom),
                solve_leg_ik(sup_x, sup_y, sup_h, self.waist_yaw, false, &self.geom),
            )
        };

        // 6. Direct ankle gyro rate damping + micro-stance compliance (isolated from stance UVC I-term)
        let mut pitch_corr = 0.0_f32;
        let mut roll_corr = 0.0_f32;
        if walk.balance_enabled {
            // Stronger direct ankle P-term: the lumped plant integrates the same
            // correction as a torso restoring moment, so a weak P term leaves a
            // permanent lean during steady walking (tens of milliradians).
            let damp_p = (-gains.gyro_damp_pitch * imu.pitch_rate_rad_s - 0.80_f32 * raw_pitch)
                .clamp(-0.35_f32, 0.35_f32);
            let damp_r = (-gains.gyro_damp_roll * imu.roll_rate_rad_s - 0.90_f32 * raw_roll)
                .clamp(-0.30_f32, 0.30_f32);

            left_leg.ankle_pitch += damp_p;
            right_leg.ankle_pitch += damp_p;
            left_leg.ankle_roll += damp_r;
            right_leg.ankle_roll += damp_r;

            pitch_corr = damp_p;
            roll_corr = damp_r;
        }

        // Toe push-off on stance foot & dorsiflexion on swing foot during active stepping.
        let toe_push = if wants_walk || walk.walk_enabled {
            0.18_f32 * sinf(self.gait_phase_rad).max(0.0_f32)
        } else {
            0.0_f32
        };
        if self.support_foot == 1 {
            left_leg.toe_pitch += toe_push;
            right_leg.toe_pitch -= 0.5_f32 * toe_push;
        } else {
            right_leg.toe_pitch += toe_push;
            left_leg.toe_pitch -= 0.5_f32 * toe_push;
        }

        // Waist pitch/roll balance assist (applied when profile has waist_pitch/waist_roll).
        let waist_pitch = 0.35_f32 * pitch_corr;
        let waist_roll = 0.35_f32 * roll_corr;

        // 7. Arm counter-spread (`armCont` proportional to lateral step offset + sagittal lean)
        let arm_gain = gains.arm_assist.clamp(0.0_f32, 1.5_f32);
        let lat_ratio = (self.dyis / self.geom.max_swing_dy.max(0.01_f32)).clamp(0.0_f32, 1.0_f32);
        let spread = arm_gain * (0.48_f32 * lat_ratio + 0.32_f32 * raw_roll.abs());
        let sag_swing = arm_gain * (0.45_f32 * raw_pitch);

        UvcStepOutput {
            left_leg,
            right_leg,
            waist_yaw: self.waist_yaw,
            waist_pitch,
            waist_roll,
            l_shoulder_pitch_delta: -sag_swing,
            l_shoulder_roll_delta: spread,
            r_shoulder_pitch_delta: -sag_swing,
            r_shoulder_roll_delta: -spread,
            pitch_corr_rad: pitch_corr,
            roll_corr_rad: roll_corr,
            gait_phase_rad: self.gait_phase_rad,
            phase: self.phase,
            support_foot: self.support_foot,
            foot_dx_m: self.dxi,
            foot_dy_m: self.signed_dy(),
            hip_height_m: self.auto_h,
            in_fall_tuck: false,
            fall_cutoff_ready: false,
            fall_orientation: fall_orientation::UNKNOWN,
            walk_active: wants_walk || walk.walk_enabled,
        }
    }

    fn enter_catch_step(&mut self, raw_roll: f32, vy: f32, wants_walk: bool) {
        if raw_roll.abs() > 0.01_f32 {
            // Lean right (+roll) -> Right foot (0) catches outward right; Lean left (-roll) -> Left foot (1)
            self.support_foot = if raw_roll >= 0.0_f32 { 0 } else { 1 };
        } else if wants_walk && vy.abs() > 0.05_f32 {
            self.support_foot = if vy > 0.0_f32 { 1 } else { 0 };
        }
        self.phase = UvcPhase::ActiveCatchStep;
        self.phase_elapsed_sec = 0.0;
        // Phase-lock the free-running clock to the synthetic catch-step phase:
        // a catch step always starts at progress 0, so its accumulator value is
        // the start of the selected support foot's half-cycle (0 or π). This
        // makes `gait_phase_rad` continuous across the state/clock handoff.
        self.phase_accum_rad = if self.support_foot == 0 { 0.0_f32 } else { PI };
    }

    /// Core ai2001 spherical-projection UVC integration (`uvc()` in `main.c`) with
    /// Capture-Point rate lead (LIPM phase-plane damping): `theta + k * theta_dot`.
    /// The rate lead arrests torso angular momentum so repeated catch-step support
    /// swaps cannot accumulate a growing lean; the ankle P-term handles high-frequency
    /// damping separately and DCM modulates the swing foot only.
    #[allow(clippy::too_many_arguments)]
    fn integrate_uvc_spherical(
        &mut self,
        pitch_eff: f32,
        roll_eff: f32,
        pitch_rate: f32,
        roll_rate: f32,
        gains: BalanceGains,
        tick_scale: f32,
        dt: f32,
    ) {
        let g_p = gains.uvc_gain_pitch.clamp(0.0_f32, 1.5_f32);
        let g_r = gains.uvc_gain_roll.clamp(0.0_f32, 1.5_f32);

        // Centering bleed prevents integrator windup and creates a smooth Capture-Point lag
        let bleed = (1.0_f32 - 4.5_f32 * dt).clamp(0.70_f32, 1.0_f32);
        self.dxi *= bleed;
        self.dyi *= bleed;

        // Capture-Point lead: arrests torso angular momentum without sagittal overshoot
        let pitch_lead = pitch_eff + 0.30_f32 * pitch_rate;
        let roll_lead = roll_eff + 0.26_f32 * roll_rate;

        // Positive roll_t grows `dyi` outward on the active support foot
        let signed_roll = if self.support_foot == 0 {
            roll_lead
        } else {
            -roll_lead
        };
        // Normalized to 10 ms ai2001 tick so BalanceGains::default() gives deadbeat CoP tracking
        let roll_t = (g_r * 0.28_f32 * signed_roll * tick_scale).clamp(-0.14_f32, 0.14_f32);
        let pitch_t = (g_p * 0.28_f32 * pitch_lead * tick_scale).clamp(-0.14_f32, 0.14_f32);

        // Lateral spherical projection: rotate (dyi - sway, auto_h) by roll_t
        let h_safe = self.auto_h.max(self.geom.min_height);
        let k_lat = atan2f(self.dyi - self.sway, h_safe);
        let cos_k_lat = cosf(k_lat).max(0.25_f32);
        let kl_lat = h_safe / cos_k_lat;
        let ks_lat = (k_lat + roll_t).clamp(-0.75_f32, 0.75_f32);
        self.dyi = (kl_lat * sinf(ks_lat) + self.sway).clamp(0.0_f32, self.geom.max_dy);
        let h_after_lat = (kl_lat * cosf(ks_lat)).max(self.geom.min_height);

        // Sagittal spherical projection: rotate (dxi, auto_h) by pitch_t
        let k_sag = atan2f(self.dxi, h_after_lat);
        let cos_k_sag = cosf(k_sag).max(0.25_f32);
        let kl_sag = h_after_lat / cos_k_sag;
        let ks_sag = (k_sag + pitch_t).clamp(-0.75_f32, 0.75_f32);
        self.dxi = (kl_sag * sinf(ks_sag)).clamp(-self.geom.max_dx, self.geom.max_dx);
        self.auto_h = (kl_sag * cosf(ks_sag)).clamp(self.geom.min_height, self.geom.nominal_height);
    }

    #[inline]
    fn signed_dy(&self) -> f32 {
        if self.support_foot == 0 {
            self.dyi
        } else {
            -self.dyi
        }
    }
}

/// Applies ai2001 radial deadband (`fw = sqrt(pitch^2 + roll^2) > deadband`).
#[inline]
pub fn apply_radial_deadband(pitch: f32, roll: f32, deadband: f32) -> (f32, f32) {
    let mag = sqrtf(pitch * pitch + roll * roll);
    if mag > deadband && mag > 1e-6_f32 {
        let scale = (mag - deadband) / mag;
        (pitch * scale, roll * scale)
    } else {
        (0.0_f32, 0.0_f32)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Undisturbed gait-clock increment under the P3 law `π/step_dur · dt`.
    /// For cadences whose nominal half-cycle is inside `[0.16, 0.45]` this is
    /// exactly the historic `2π·cadence·dt` increment.
    fn nominal_walk_phase_inc(cadence: f32, dt: f32) -> f32 {
        (PI / (0.5_f32 / cadence).clamp(STEP_DUR_MIN_SEC, STEP_DUR_MAX_SEC)) * dt
    }

    fn test_biped_geom() -> LegGeometry {
        LegGeometry {
            thigh_len: 0.32,
            shin_len: 0.32,
            hip_ankle_offset: 0.06,
            half_hip_spacing: 0.095,
            nominal_height: 0.62,
            min_height: 0.46,
            max_reach: 0.635,
            max_dx: 0.16,
            max_dy: 0.15,
            max_swing_dx: 0.22,
            max_swing_dy: 0.22,
            max_foot_lift: 0.095,
            max_sway: 0.075,
        }
    }

    #[test]
    fn analytic_6dof_leg_ik_keeps_sole_strictly_parallel_on_khr_and_rocky() {
        for geom in [LegGeometry::khr_3hv(), LegGeometry::humanoid(), test_biped_geom()] {
            let xs = [
                -geom.max_dx,
                -geom.max_dx * 0.4,
                0.0,
                geom.max_dx * 0.5,
                geom.max_dx,
            ];
            let ys = [0.0, geom.max_dy * 0.5, geom.max_dy];
            let hs = [
                geom.min_height,
                (geom.min_height + geom.nominal_height) * 0.5,
                geom.nominal_height,
            ];
            for &x in &xs {
                for &y in &ys {
                    for &h in &hs {
                        for is_left in [true, false] {
                            let ik = solve_leg_ik(x, y, h, 0.12, is_left, &geom);
                            let pitch_sum = ik.hip_pitch + ik.knee_pitch + ik.ankle_pitch;
                            let roll_sum = ik.hip_roll + ik.ankle_roll;
                            assert!(
                                pitch_sum.abs() < 1e-4,
                                "sole pitch must stay parallel (sum={pitch_sum})"
                            );
                            assert!(
                                roll_sum.abs() < 1e-4,
                                "sole roll must stay flat (sum={roll_sum})"
                            );
                            assert!(ik.knee_pitch >= 0.0, "knee must never hyperextend");
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn bias_compensated_attitude_subtracts_calibrated_bias() {
        let ctrl = UvcController::new(test_biped_geom());
        assert_eq!(ctrl.pitch_bias(), 0.0);
        assert_eq!(ctrl.roll_bias(), 0.0);
        let imu = ImuState {
            pitch_rad: 0.06,
            roll_rad: -0.04,
            ..Default::default()
        };
        assert_eq!(ctrl.bias_compensated_attitude(&imu), (0.06, -0.04));
    }

    #[test]
    fn radial_deadband_suppresses_small_imu_noise_and_passes_shoves() {
        let (p0, r0) = apply_radial_deadband(0.02, 0.02, 0.033);
        assert_eq!(p0, 0.0);
        assert_eq!(r0, 0.0);

        let (p1, r1) = apply_radial_deadband(0.10, 0.0, 0.033);
        assert!((p1 - 0.067).abs() < 1e-4);
        assert_eq!(r1, 0.0);
    }

    #[test]
    fn uvc_projects_foot_forward_under_forward_lean_and_triggers_fall_tuck() {
        let mut ctrl = UvcController::new(test_biped_geom());
        let gains = BalanceGains::default();
        let walk = WalkCmd::default();

        // Lean forward ~12 deg -> must enter ActiveCatchStep and step foot forward (+dxi)
        let imu_lean = ImuState {
            available: true,
            pitch_rad: 0.21,
            roll_rad: 0.09,
            yaw_rad: 0.0,
            pitch_rate_rad_s: 0.8,
            roll_rate_rad_s: 0.3,
            yaw_rate_rad_s: 0.0,
            gravity: [0.0, -9.81, 0.0],
        };
        let mut out = ctrl.step(0.005, imu_lean, walk, gains);
        for _ in 0..20 {
            out = ctrl.step(0.005, imu_lean, walk, gains);
        }
        assert_eq!(out.phase, UvcPhase::ActiveCatchStep);
        assert!(
            out.foot_dx_m > 0.02,
            "foot must project forward under forward lean"
        );
        assert!(
            out.foot_dy_m > 0.005,
            "right foot must project right under +roll lean"
        );
        assert!(!out.in_fall_tuck);

        // Exceed fall cutoff -> enters protective FallTuck for 40 ms before cutoff ready
        let imu_fall = ImuState {
            available: true,
            pitch_rad: 0.85,
            roll_rad: 0.0,
            yaw_rad: 0.0,
            pitch_rate_rad_s: 2.0,
            roll_rate_rad_s: 0.0,
            yaw_rate_rad_s: 0.0,
            gravity: [0.0, -9.81, 0.0],
        };
        let tuck0 = ctrl.step(0.01, imu_fall, walk, gains);
        assert_eq!(tuck0.phase, UvcPhase::FallTuck);
        assert!(tuck0.in_fall_tuck);
        assert!(!tuck0.fall_cutoff_ready);

        let mut tuck_end = tuck0;
        for _ in 0..6 {
            tuck_end = ctrl.step(0.01, imu_fall, walk, gains);
        }
        assert!(
            tuck_end.fall_cutoff_ready,
            "cutoff must fire after 40 ms tuck"
        );
    }

    /// Group 1 (P3): phase advances by exactly one fixed-dt increment every
    /// tick while walking, including across 2π wraps and support swaps. With
    /// no capture telemetry the Khadiv law falls back to nominal cadence.
    #[test]
    fn walking_gait_phase_advances_continuously() {
        let mut ctrl = UvcController::new(test_biped_geom());
        let dt = 0.005_f32;
        let cadence = 1.55_f32;
        let walk = WalkCmd {
            walk_enabled: true,
            vx: 0.5,
            cadence_hz: cadence,
            ..Default::default()
        };
        let imu = ImuState {
            available: true,
            ..Default::default()
        };
        let gains = BalanceGains::default();
        let expected_inc = nominal_walk_phase_inc(cadence, dt);
        assert!(
            (expected_inc - 2.0_f32 * PI * cadence * dt).abs() < 1e-6_f32,
            "undisturbed cadence must still advance at 2π·cadence·dt"
        );
        let mut prev: Option<f32> = None;
        let mut wraps = 0;
        for _ in 0..400 {
            let out = ctrl.step(dt, imu, walk, gains);
            assert!(out.walk_active);
            assert!(out.gait_phase_rad >= 0.0_f32 && out.gait_phase_rad < 2.0_f32 * PI);
            if let Some(p) = prev {
                let mut delta = out.gait_phase_rad - p;
                if delta < 0.0_f32 {
                    delta += 2.0_f32 * PI;
                    wraps += 1;
                }
                assert!(
                    (delta - expected_inc).abs() < 1e-4_f32,
                    "phase must advance by exactly one increment (delta={delta}, expected={expected_inc})"
                );
            }
            prev = Some(out.gait_phase_rad);
        }
        assert!(wraps > 0, "2 s at 1.55 Hz must wrap the accumulator");
    }

    /// Group 1 (P2): the accumulator half-cycle selects the support foot with
    /// a π offset, and support alternates as the clock crosses each boundary.
    #[test]
    fn support_foot_is_pi_offset_from_walking_phase() {
        let mut ctrl = UvcController::new(test_biped_geom());
        let dt = 0.005_f32;
        let walk = WalkCmd {
            walk_enabled: true,
            vx: 0.4,
            cadence_hz: 1.55,
            ..Default::default()
        };
        let imu = ImuState {
            available: true,
            ..Default::default()
        };
        let gains = BalanceGains::default();
        let mut swaps = 0;
        let mut prev_support = None;
        for _ in 0..400 {
            let out = ctrl.step(dt, imu, walk, gains);
            if out.phase == UvcPhase::ActiveCatchStep {
                if out.support_foot == 0 {
                    assert!(
                        out.gait_phase_rad < PI + 1e-4_f32,
                        "right support must occupy the [0, π) half, got {}",
                        out.gait_phase_rad
                    );
                } else {
                    assert!(
                        out.gait_phase_rad >= PI - 1e-4_f32,
                        "left support must occupy the [π, 2π) half, got {}",
                        out.gait_phase_rad
                    );
                }
                if let Some(p) = prev_support {
                    if p != out.support_foot {
                        swaps += 1;
                    }
                }
                prev_support = Some(out.support_foot);
            }
        }
        assert!(
            swaps >= 4,
            "support must alternate repeatedly at π offsets, got {swaps} swaps"
        );
    }

    /// P3: with `walk_enabled`, a capture-point breach shortens the gait
    /// half-cycle so the next support swap arrives earlier than nominal cadence.
    #[test]
    fn walking_capture_breach_shortens_step() {
        let dt = 0.005_f32;
        let cadence = 1.55_f32;
        let walk = WalkCmd {
            walk_enabled: true,
            balance_enabled: true,
            vx: 0.5,
            cadence_hz: cadence,
            ..Default::default()
        };
        let imu = ImuState {
            available: true,
            ..Default::default()
        };
        let gains = BalanceGains::default();
        let omega = sqrtf(GRAVITY_M_S2 / gains.h_com_m);
        let v_des = walk.vx * 0.5_f32;

        // Same valid capture telemetry in both runs; the disturbance adds a
        // genuine support-polygon breach plus a large CoM velocity error.
        let nominal = capture_telemetry(v_des / omega, v_des, true);
        let breach = capture_telemetry(0.0, 2.0_f32, true);
        let calm_hint = DcmSwingHint::default();
        let breach_hint = DcmSwingHint {
            margin_breached: true,
            ..Default::default()
        };

        let first_left_support = |telem: EstimatorTelemetry, hint: DcmSwingHint| -> usize {
            let mut ctrl = UvcController::new(test_biped_geom());
            for tick in 1..=400 {
                let out = ctrl.step_with_dcm(dt, imu, walk, gains, hint, telem);
                if out.phase == UvcPhase::ActiveCatchStep && out.support_foot == 1 {
                    return tick;
                }
            }
            panic!("walking never swapped to left support");
        };

        let nominal_ticks = first_left_support(nominal, calm_hint);
        let breach_ticks = first_left_support(breach, breach_hint);
        eprintln!("support swap: nominal={nominal_ticks} ticks, breach={breach_ticks} ticks");
        assert!(
            breach_ticks + 8 < nominal_ticks,
            "capture breach must advance the support swap by >= 40 ms ({breach_ticks} vs {nominal_ticks})"
        );
        // Nominal cadence must still be correct when undisturbed. A walking
        // breach shortens the half-cycle but keeps 85% of nominal so the
        // lagged actuator can plant the swing foot before the swap.
        // Tick granularity is one increment, so allow one tick of slack.
        assert!(
            (nominal_ticks as f32 * nominal_walk_phase_inc(cadence, dt) - PI).abs()
                < 2.0_f32 * nominal_walk_phase_inc(cadence, dt),
            "undisturbed support swap must land at π, got tick {nominal_ticks}"
        );
        let shortened_dur =
            0.85_f32 * (0.5_f32 / cadence).clamp(STEP_DUR_MIN_SEC, STEP_DUR_MAX_SEC);
        let shortening_s = (nominal_ticks - breach_ticks) as f32 * dt;
        assert!(
            shortening_s > 0.0_f32 && shortening_s <= (0.5_f32 / cadence - shortened_dur) + dt,
            "breach shortening {shortening_s:.3}s must be early but no shorter than the walking settling floor"
        );
    }

    /// Group 1 (P2): entering a catch step phase-locks the accumulator to the
    /// synthetic catch-step phase (progress 0 -> 0 or π) with no jump, and the
    /// very next tick continues advancing from that locked value.
    #[test]
    fn catch_step_entry_phase_locks_without_discontinuity() {
        let mut ctrl = UvcController::new(test_biped_geom());
        let dt = 0.005_f32;
        let cadence = 1.55_f32;
        let walk = WalkCmd {
            walk_enabled: true,
            vx: 0.3,
            cadence_hz: cadence,
            ..Default::default()
        };
        // +roll lean selects right support via `enter_catch_step`.
        let imu = ImuState {
            available: true,
            roll_rad: 0.05,
            ..Default::default()
        };
        let gains = BalanceGains::default();
        let mut entry = None;
        for _ in 0..40 {
            let out = ctrl.step(dt, imu, walk, gains);
            if out.phase == UvcPhase::ActiveCatchStep {
                entry = Some(out);
                break;
            }
        }
        let entry = entry.expect("walking must engage a catch step");
        assert_eq!(
            entry.support_foot, 0,
            "+roll lean must select right support"
        );
        let locked = if entry.support_foot == 0 { 0.0_f32 } else { PI };
        assert!(
            (entry.gait_phase_rad - locked).abs() < 1e-5,
            "catch-step entry must lock phase to synthetic progress 0 (got {}, expected {locked})",
            entry.gait_phase_rad
        );

        let next = ctrl.step(dt, imu, walk, gains);
        let expected_inc = nominal_walk_phase_inc(cadence, dt);
        let delta = if next.gait_phase_rad >= entry.gait_phase_rad {
            next.gait_phase_rad - entry.gait_phase_rad
        } else {
            next.gait_phase_rad + 2.0_f32 * PI - entry.gait_phase_rad
        };
        assert!(
            (delta - expected_inc).abs() < 1e-4_f32,
            "phase after entry must advance by one increment, not reset (delta={delta})"
        );
    }

    // ---- P2 capture-law tests (valid `estimator_valid::DCM` telemetry required) ----

    fn capture_telemetry(xi_sag: f32, v_sag: f32, dcm_valid: bool) -> EstimatorTelemetry {
        EstimatorTelemetry {
            dcm_xy_m: [0.0, xi_sag],
            com_vel_m_s: [0.0, 0.0, v_sag],
            valid_mask: if dcm_valid {
                estimator_valid::COM | estimator_valid::DCM
            } else {
                estimator_valid::COM
            },
            ..Default::default()
        }
    }

    /// Enter `ActiveCatchStep` with a forward lean using *invalid* telemetry so the
    /// capture law cannot influence the setup phase.
    fn enter_catch(ctrl: &mut UvcController) -> (ImuState, WalkCmd, BalanceGains) {
        let imu = ImuState {
            available: true,
            pitch_rad: 0.20,
            ..Default::default()
        };
        let walk = WalkCmd {
            balance_enabled: true,
            vx: 0.5,
            ..Default::default()
        };
        let gains = BalanceGains::default();
        let out = ctrl.step_with_dcm(
            0.005,
            imu,
            walk,
            gains,
            DcmSwingHint::default(),
            EstimatorTelemetry::default(),
        );
        assert_eq!(out.phase, UvcPhase::ActiveCatchStep);
        (imu, walk, gains)
    }

    /// Speed tracking: at steady state (v_com == v_des == vx*0.5) the swing-foot
    /// landing target equals the capture point ξ = v_des/ω.
    #[test]
    fn capture_law_tracks_commanded_speed_via_swing_foot_only() {
        let mut ctrl = UvcController::new(test_biped_geom());
        let (imu, walk, gains) = enter_catch(&mut ctrl);

        let omega = sqrtf(GRAVITY_M_S2 / gains.h_com_m);
        let v_des = walk.vx.clamp(-1.0, 1.0) * 0.5_f32;
        let telem = capture_telemetry(v_des / omega, v_des, true);
        let out = ctrl.step_with_dcm(0.005, imu, walk, gains, DcmSwingHint::default(), telem);

        let (dxis, _) = ctrl.swing_targets();
        assert!(
            (dxis - v_des / omega).abs() < 1e-3_f32,
            "swing target must track v_des/ω={} (got {dxis})",
            v_des / omega
        );
        assert_eq!(out.phase, UvcPhase::ActiveCatchStep);
    }

    /// Disturbance recovery: a forward CoM velocity shove (ξ = v_com/ω) moves the
    /// landing target forward relative to steady state, but the K_v velocity term
    /// pulls it back toward the CoM instead of blindly following ξ.
    #[test]
    fn capture_law_recovers_from_forward_velocity_disturbance() {
        let omega = sqrtf(GRAVITY_M_S2 / BalanceGains::default().h_com_m);
        let v_des = 0.25_f32;
        let shove_v = 1.0_f32;

        let mut steady = UvcController::new(test_biped_geom());
        let (imu, walk, gains) = enter_catch(&mut steady);
        steady.step_with_dcm(
            0.005,
            imu,
            walk,
            gains,
            DcmSwingHint::default(),
            capture_telemetry(v_des / omega, v_des, true),
        );
        let (steady_dxis, _) = steady.swing_targets();

        let mut pushed = UvcController::new(test_biped_geom());
        let (imu, walk, gains) = enter_catch(&mut pushed);
        pushed.step_with_dcm(
            0.005,
            imu,
            walk,
            gains,
            DcmSwingHint::default(),
            capture_telemetry(shove_v / omega, shove_v, true),
        );
        let (pushed_dxis, _) = pushed.swing_targets();

        assert!(
            pushed_dxis > steady_dxis,
            "forward shove must place the foot farther forward ({pushed_dxis} vs {steady_dxis})"
        );
        assert!(
            pushed_dxis < shove_v / omega,
            "velocity feedback must keep the landing target behind the disturbed ξ"
        );
        assert!(pushed_dxis <= pushed.geometry().max_swing_dx);
    }

    /// Khadiv timing: a capture error beyond the usable foot margin shortens the
    /// timer-driven step; the same clock must otherwise advance at the nominal rate.
    #[test]
    fn khadiv_timing_shortens_step_when_capture_error_exceeds_margin() {
        let advance_phase = |telem: EstimatorTelemetry| -> f32 {
            let mut ctrl = UvcController::new(test_biped_geom());
            let (imu, walk, gains) = enter_catch(&mut ctrl);
            let mut phase = UvcPhase::Idle;
            for _ in 0..12 {
                phase = ctrl
                    .step_with_dcm(0.005, imu, walk, gains, DcmSwingHint::default(), telem)
                    .phase;
            }
            assert_eq!(phase, UvcPhase::ActiveCatchStep);
            // Non-walking telemetry reports progress*π, so larger phase == shorter T.
            ctrl.gait_phase_rad
        };

        // ξ=0, v_com=0.25 -> p≈0.026 m, error < 0.028 m margin -> nominal T=0.18 s.
        let nominal_phase = advance_phase(capture_telemetry(0.0, 0.25, true));
        // ξ=0, v_com=2.0 -> p≈-0.18 m, error > margin -> T clamps to T_min=0.16 s.
        let shortened_phase = advance_phase(capture_telemetry(0.0, 2.0, true));
        assert!(
            shortened_phase > nominal_phase + 0.02_f32,
            "large capture error must shorten the step (phase {shortened_phase} vs {nominal_phase})"
        );
    }

    /// Strict DOF partitioning: valid capture telemetry changes only the swing
    /// target; the stance hip integrator is bit-identical to the invalid-DCM case,
    /// and the capture law is gated off when `estimator_valid::DCM` is absent.
    #[test]
    fn capture_law_leaves_stance_hip_untouched_and_respects_dcm_validity_gate() {
        let mut valid = UvcController::new(test_biped_geom());
        let mut invalid = UvcController::new(test_biped_geom());
        let (imu, walk, gains) = enter_catch(&mut valid);
        let _ = enter_catch(&mut invalid);

        // v_com == v_des -> capture error 0, so step timing is identical too.
        let telem = capture_telemetry(0.0, 0.25, true);
        let gated = capture_telemetry(0.0, 0.25, false);
        let mut phase_valid = UvcPhase::Idle;
        let mut phase_invalid = UvcPhase::Idle;
        let mut out_valid = UvcStepOutput::default();
        let mut out_invalid = UvcStepOutput::default();
        for _ in 0..10 {
            out_valid =
                valid.step_with_dcm(0.005, imu, walk, gains, DcmSwingHint::default(), telem);
            out_invalid =
                invalid.step_with_dcm(0.005, imu, walk, gains, DcmSwingHint::default(), gated);
            phase_valid = out_valid.phase;
            phase_invalid = out_invalid.phase;
        }
        assert_eq!(phase_valid, UvcPhase::ActiveCatchStep);
        assert_eq!(phase_invalid, UvcPhase::ActiveCatchStep);

        assert_eq!(
            out_valid.foot_dx_m, out_invalid.foot_dx_m,
            "capture law must never write the stance sagittal hip"
        );
        assert_eq!(
            out_valid.foot_dy_m, out_invalid.foot_dy_m,
            "capture law must never write the stance lateral hip"
        );
        assert_ne!(
            valid.swing_targets().0,
            invalid.swing_targets().0,
            "DCM validity gate must switch the capture law on/off"
        );
    }
}
