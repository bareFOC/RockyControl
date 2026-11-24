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

use libm::{acosf, asinf, atan2f, cosf, sinf, sqrtf};
use rk_proto::{BalanceGains, ImuState, WalkCmd, classify_fall, fall_orientation, uvc_phase};

use crate::estimator::DcmSwingHint;

/// Reference tick duration of ai2001 `main.c` (`10 ms` = 100 Hz).
const REF_TICK_SEC: f32 = 0.010_f32;
/// Duration of protective servo tuck pose before torque drop (`40 ms`).
const FALL_TUCK_DURATION_SEC: f32 = 0.040_f32;
/// Maximum knee-triangle half-angle (`60 deg` = `PI / 3`, matching ai2001 `k > 60*PI/180`).
const MAX_HALF_KNEE_RAD: f32 = 1.047_197_6_f32;

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
    /// Matches bundled `assets/sample_humanoid/rocky_mk1.urdf` (21-DoF, 6-DoF legs).
    pub const fn rocky_mk1() -> Self {
        Self {
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
            max_swing_dx: 0.070,
            max_swing_dy: 0.070,
            max_foot_lift: 0.035,
            max_sway: 0.025,
        }
    }
}

/// Solved single-leg 6-DoF joint angles in Rocky Mk1 URDF sign conventions (radians).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LegIkAngles {
    pub hip_yaw: f32,
    pub hip_roll: f32,
    pub hip_pitch: f32,
    pub knee_pitch: f32,
    pub ankle_pitch: f32,
    pub ankle_roll: f32,
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
    let d_safe = d.clamp((l1 - l2).abs() + 0.01_f32, (l1 + l2 - 0.0005_f32).min(geom.max_reach));

    let x_ang = asinf((x / d_safe).clamp(-0.999_f32, 0.999_f32));
    let cos_hip = ((l1 * l1 + d_safe * d_safe - l2 * l2) / (2.0_f32 * l1 * d_safe))
        .clamp(-1.0_f32, 1.0_f32);
    let cos_knee = ((l1 * l1 + l2 * l2 - d_safe * d_safe) / (2.0_f32 * l1 * l2))
        .clamp(-1.0_f32, 1.0_f32);

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
        Self {
            geom,
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
        self.step_with_dcm(dt, imu, walk, gains, DcmSwingHint::default())
    }

    /// Executes one deterministic `#![no_std]` UVC control cycle with P1a strict DOF partitioning:
    /// - Stance torso verticality -> UVC I-only (`dxi`/`dyi` spherical integration).
    /// - High-frequency vibration damping -> decoupled ankle rate-gyro P-term.
    /// - Capturability / stepping -> `dcm` modulates swing-foot landing (`dxis`/`dyis`) ONLY.
    pub fn step_with_dcm(
        &mut self,
        dt: f32,
        imu: ImuState,
        walk: WalkCmd,
        gains: BalanceGains,
        dcm: DcmSwingHint,
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

        let step_h_scale = walk.step_height.clamp(0.1_f32, 1.0_f32);
        let cadence = walk.cadence_hz.clamp(0.6_f32, 2.8_f32);
        // ai2001 catch-step duration: 18 ticks @ 10 ms = 0.18 s; walk cadence modulates step speed
        let step_dur = if wants_walk {
            (0.5_f32 / cadence).clamp(0.16_f32, 0.45_f32)
        } else {
            0.18_f32
        };

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
                    self.pitch_bias = (self.pitch_bias + raw_pitch * drift).clamp(-0.08_f32, 0.08_f32);
                    self.roll_bias = (self.roll_bias + raw_roll * drift).clamp(-0.08_f32, 0.08_f32);
                }

                if tilt_trigger || wants_walk {
                    self.enter_catch_step(raw_roll, vy, wants_walk);
                }
            }
            UvcPhase::ActiveCatchStep => {
                let progress = (self.phase_elapsed_sec / step_dur).clamp(0.0_f32, 1.0_f32);
                self.gait_phase_rad = if self.support_foot == 0 {
                    progress * PI
                } else {
                    PI + progress * PI
                };

                // Core ai2001 geometric spherical UVC projection (strictly I-only on stance torso)
                if walk.balance_enabled {
                    self.integrate_uvc_spherical(pitch_eff, roll_eff, gains, tick_scale, dt);
                }

                // Superpose commanded walking stride (`WalkCmd`) onto UVC foot landing target
                if wants_walk {
                    let target_walk_x = vx * self.geom.max_dx * 0.68_f32;
                    let lat_sign = if self.support_foot == 0 { -1.0_f32 } else { 1.0_f32 };
                    let target_walk_y = (vy * lat_sign * self.geom.max_dy * 0.55_f32).max(0.0_f32);
                    let walk_blend = (dt * 7.5_f32).clamp(0.0_f32, 1.0_f32);
                    self.dxi = (self.dxi + (target_walk_x - self.dxi) * walk_blend)
                        .clamp(-self.geom.max_dx, self.geom.max_dx);
                    self.dyi = (self.dyi + (target_walk_y - self.dyi) * walk_blend)
                        .clamp(0.0_f32, self.geom.max_dy);
                }

                // Mirror swing foot (`uvcSub()`) + Kinematic DCM swing-foot placement modulation ONLY
                // (never double-drives the stance hip integrator `dxi`/`dyi`).
                let swing_lat_sign = if self.support_foot == 0 { -1.0_f32 } else { 1.0_f32 };
                let dcm_swing_y = (dcm.swing_dy_m * swing_lat_sign).max(0.0_f32);
                self.dxis = (-self.dxi + dcm.swing_dx_m)
                    .clamp(-self.geom.max_swing_dx, self.geom.max_swing_dx);
                self.dyis = (self.dyi + dcm_swing_y).clamp(0.0_f32, self.geom.max_swing_dy);
                let anti_collide = self.dyi + self.dyis;
                if anti_collide < 0.0_f32 {
                    self.dyis -= anti_collide;
                }

                // Sinusoidal swing foot clearance (`fh = fhMax * sin(PI * progress)`)
                let lift_scale = if wants_walk {
                    step_h_scale
                } else {
                    // Scale catch-step lift with disturbance magnitude so tiny corrections don't stomp
                    let offset_norm =
                        (sqrtf(self.dxi * self.dxi + self.dyi * self.dyi) / self.geom.max_dx)
                            .clamp(0.28_f32, 1.0_f32);
                    step_h_scale * offset_norm
                };
                swing_lift_m = self.geom.max_foot_lift * lift_scale * sinf(PI * progress);

                // Pelvis waist/hip yaw twist (`feetCont1`: wt = 0.5 * atan2(dxi, dyi + half_hip))
                let wt_geom = 0.35_f32 * atan2f(self.dxi, self.dyi + self.geom.half_hip_spacing);
                self.waist_yaw = (wt_geom + yaw_cmd * 0.24_f32).clamp(-0.45_f32, 0.45_f32);
                self.wk = (self.dxi.abs() * 0.33_f32).min(self.geom.max_dx * 0.33_f32);

                // Step landing -> swap support/swing foot
                if progress >= 1.0_f32 {
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

        // 5. Solve 6-DoF Analytic Leg IK for support and swing legs
        let sup_x = self.dxi;
        let sup_y = (self.dyi - self.sway).max(0.0_f32);
        let sup_h = self.auto_h.clamp(self.geom.min_height, self.geom.nominal_height);

        let swg_x = self.dxis;
        let swg_y = (self.dyis + self.sway).max(0.0_f32);
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

        // 6. Decoupled ankle rate-gyro P-term damping (strictly isolated from stance hip UVC I-term)
        let mut pitch_corr = 0.0_f32;
        let mut roll_corr = 0.0_f32;
        if walk.balance_enabled {
            let damp_p = (-gains.gyro_damp_pitch * imu.pitch_rate_rad_s).clamp(-0.35_f32, 0.35_f32);
            let damp_r = (-gains.gyro_damp_roll * imu.roll_rate_rad_s).clamp(-0.30_f32, 0.30_f32);

            left_leg.ankle_pitch += damp_p;
            right_leg.ankle_pitch += damp_p;
            left_leg.ankle_roll += damp_r;
            right_leg.ankle_roll += damp_r;

            pitch_corr = damp_p;
            roll_corr = damp_r;
        }

        // 7. Arm counter-spread (`armCont` proportional to lateral step offset + sagittal lean)
        let arm_gain = gains.arm_assist.clamp(0.0_f32, 1.5_f32);
        let lat_ratio = (self.dyis / self.geom.max_swing_dy.max(0.01_f32)).clamp(0.0_f32, 1.0_f32);
        let spread = arm_gain * (0.48_f32 * lat_ratio + 0.32_f32 * raw_roll.abs());
        let sag_swing = arm_gain * (0.45_f32 * raw_pitch);

        UvcStepOutput {
            left_leg,
            right_leg,
            waist_yaw: self.waist_yaw,
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
    }

    /// Core ai2001 spherical-projection UVC integration (`uvc()` in `main.c`) — strictly I-only
    /// on stance torso tilt (`pitch_eff`, `roll_eff`) with anti-windup bleed so it never fights
    /// the decoupled ankle rate-gyro P-term or DCM swing-foot placement.
    fn integrate_uvc_spherical(
        &mut self,
        pitch_eff: f32,
        roll_eff: f32,
        gains: BalanceGains,
        tick_scale: f32,
        dt: f32,
    ) {
        let g_p = gains.uvc_gain_pitch.clamp(0.0_f32, 1.5_f32);
        let g_r = gains.uvc_gain_roll.clamp(0.0_f32, 1.5_f32);

        // Anti-windup centering bleed prevents stance integrator runaway
        let bleed = (1.0_f32 - 4.5_f32 * dt).clamp(0.70_f32, 1.0_f32);
        self.dxi *= bleed;
        self.dyi *= bleed;

        // Positive roll_t grows `dyi` outward on the active support foot
        let signed_roll = if self.support_foot == 0 {
            roll_eff
        } else {
            -roll_eff
        };
        // Strictly I-only attitude projection normalized to 10 ms ai2001 tick
        let roll_t = (g_r * 0.28_f32 * signed_roll * tick_scale).clamp(-0.14_f32, 0.14_f32);
        let pitch_t = (g_p * 0.28_f32 * pitch_eff * tick_scale).clamp(-0.14_f32, 0.14_f32);

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

    #[test]
    fn analytic_6dof_leg_ik_keeps_sole_strictly_parallel_on_khr_and_rocky() {
        for geom in [LegGeometry::khr_3hv(), LegGeometry::rocky_mk1()] {
            let xs = [-geom.max_dx, -geom.max_dx * 0.4, 0.0, geom.max_dx * 0.5, geom.max_dx];
            let ys = [0.0, geom.max_dy * 0.5, geom.max_dy];
            let hs = [geom.min_height, (geom.min_height + geom.nominal_height) * 0.5, geom.nominal_height];
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
        let mut ctrl = UvcController::new(LegGeometry::rocky_mk1());
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
        assert!(out.foot_dx_m > 0.02, "foot must project forward under forward lean");
        assert!(out.foot_dy_m > 0.005, "right foot must project right under +roll lean");
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
        assert!(tuck_end.fall_cutoff_ready, "cutoff must fire after 40 ms tuck");
    }
}
