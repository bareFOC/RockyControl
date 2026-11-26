//! Pure `#![no_std]` mass-weighted Center-of-Mass, analytic CoM velocity,
//! support-relative Divergent Component of Motion (DCM / Capture Point), and
//! ZMP estimator for Rocky Mk1 (`rk-control`).
//!
//! Designed for strict RP2350 Cortex-M33F parity:
//! - Strictly single-precision `f32` via `libm`, zero heap allocations.
//! - For 21-DoF Rocky Mk1, CoM position and velocity come from the shared
//!   mass-weighted FK ([`crate::com_fk`]) driven by joint encoders and measured
//!   joint rates. No finite differencing of position, no IMU gyro extrapolation.
//! - The capture point is expressed **relative to the current support origin**
//!   (left stance foot, right stance foot, or double-support midpoint from
//!   `contact_mask`), using a fixed configurable CoM height `h_com_m` for
//!   ω = sqrt(g / h_com). `h_com_m` comes from [`BalanceGains::h_com_m`].
//! - Encodes the 7 cm printed foot (`foot_half_length_m = 0.035 m`) with ±2.8 cm
//!   (`foot_margin_m = 0.028 m`) usable ankle/CoP authority before kinematic DCM
//!   triggers a recovery step.
//! - Strict DOF partitioning: outputs [`DcmSwingHint`] exclusively for swing-foot
//!   placement modulation — never double-driving the stance hip UVC integrator.
//! - Explicit `valid_mask` (`estimator_valid::*`): v1 sets `COM | DCM | imu_source` only
//!   when the IMU is live; `GRF` and `MOMENTUM` are never set so disabled sensors never
//!   read as `0.0 N` free-fall, and a sensor dropout never publishes stale kinematics.
//!
//! # Fallback proxy
//!
//! For joint counts other than 21 (legacy 17-DoF URDFs, test rigs), the
//! estimator keeps the historical IMU-tilt proxy path for position and a
//! finite-difference + gyro-blend velocity reconstruction. This path is
//! deterministic but is **not** the validated physical CoM path.

use libm::{atan2f, sqrtf};
use rk_proto::{BalanceGains, EstimatorTelemetry, ImuState, MAX_JOINTS, estimator_valid};
use rk_robot_profile::RobotProfile;

use crate::com_fk::{compute_state, mat_mul, mat_vec, rot_from_rpy};

const GRAVITY_M_S2: f32 = 9.81_f32;
/// Reference 200 Hz tick duration (5 ms) for EMA velocity filter.
const REF_EST_DT_SEC: f32 = 0.005_f32;
const VEL_EMA_ALPHA_200HZ: f32 = 0.30_f32;
/// Clamping bounds for the configurable CoM height (matches the MCU config clamp).
const H_COM_MIN_M: f32 = 0.12_f32;
const H_COM_MAX_M: f32 = 0.80_f32;

/// Swing-foot placement modulation computed by [`KinematicEstimator`] for [`crate::UvcController`].
///
/// Strictly partitioned to the swing foot — never injected into the stance hip integrator.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DcmSwingHint {
    /// True when the Capture Point `xi` leaves the encoded ±2.8 cm foot margin.
    pub margin_breached: bool,
    /// Sagittal swing-foot landing offset modulation (`+` = forward, meters).
    pub swing_dx_m: f32,
    /// Lateral swing-foot landing offset modulation (`+` = right, `-` = left, meters).
    pub swing_dy_m: f32,
}

/// Support origin identifier derived from `contact_mask`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Support {
    Left,
    Right,
    Double,
    None,
}

impl Support {
    fn from_mask(mask: u8) -> Self {
        match mask & 0b11 {
            0b01 => Self::Left,
            0b10 => Self::Right,
            0b11 => Self::Double,
            _ => Self::None,
        }
    }
}

/// Pure `#![no_std]` kinematic state estimator running at 200 Hz on RP2350 & sim.
#[derive(Clone, Debug)]
pub struct KinematicEstimator {
    initialized: bool,
    prev_com: [f32; 3],
    com_vel: [f32; 3],
    prev_vel: [f32; 3],
    com_acc: [f32; 3],
    prev_support: Support,
    /// Last known support origin/velocity in the pelvis frame. Held during a
    /// sensed flight tick so the capture point stays referenced to the sole
    /// that was actually carrying the body instead of snapping to the pelvis.
    last_support_pos: [f32; 3],
    last_support_vel: [f32; 3],
    /// Filtered support origin. A support swap re-targets both legs at once, so
    /// the raw FK foot origin jumps laterally by tens of centimetres; feeding
    /// that jump straight into ξ produces a spurious catch-step trigger. An
    /// asymmetric EMA (slow on swap, fast while settled) keeps the capture
    /// origin continuous without hiding a genuine, sustained support change.
    support_pos_f: [f32; 3],
    support_vel_f: [f32; 3],
    support_init: bool,
    /// Support-relative CoM from the *midpoint of both soles*. The gait's
    /// alternating single-foot origin jumps when legs are re-targeted; the
    /// midpoint is continuous and is the effective support polygon while
    /// walking, so it is used for the walking capturability margin only.
    com_mid_pos: [f32; 3],
    com_mid_vel: [f32; 3],
    last_pelvis_imu: ImuState,
    last_telemetry: EstimatorTelemetry,
}

impl Default for KinematicEstimator {
    fn default() -> Self {
        Self::new()
    }
}

impl KinematicEstimator {
    pub const fn new() -> Self {
        Self {
            initialized: false,
            prev_com: [0.0, 0.48, 0.0],
            com_vel: [0.0; 3],
            prev_vel: [0.0; 3],
            com_acc: [0.0; 3],
            prev_support: Support::Double,
            last_support_pos: [0.0; 3],
            last_support_vel: [0.0; 3],
            support_pos_f: [0.0; 3],
            support_vel_f: [0.0; 3],
            support_init: false,
            com_mid_pos: [0.0; 3],
            com_mid_vel: [0.0; 3],
            last_pelvis_imu: ImuState {
                available: false,
                pitch_rad: 0.0,
                roll_rad: 0.0,
                yaw_rad: 0.0,
                pitch_rate_rad_s: 0.0,
                roll_rate_rad_s: 0.0,
                yaw_rate_rad_s: 0.0,
                gravity: [0.0, -9.81, 0.0],
            },
            last_telemetry: EstimatorTelemetry {
                com_pos_m: [0.0, 0.48, 0.0],
                com_vel_m_s: [0.0; 3],
                dcm_xy_m: [0.0; 2],
                zmp_xy_m: [0.0; 2],
                foot_margin_m: 0.028,
                grf_left_n: 0.0,
                grf_right_n: 0.0,
                contact_mask: 0b11,
                ukemi_active: false,
                valid_mask: 0,
            },
        }
    }

    pub fn reset(&mut self) {
        self.initialized = false;
        self.com_vel = [0.0; 3];
        self.prev_vel = [0.0; 3];
        self.com_acc = [0.0; 3];
        self.prev_support = Support::Double;
        self.last_support_pos = [0.0; 3];
        self.last_support_vel = [0.0; 3];
        self.support_init = false;
        self.com_mid_pos = [0.0; 3];
        self.com_mid_vel = [0.0; 3];
    }

    pub const fn last_telemetry(&self) -> EstimatorTelemetry {
        self.last_telemetry
    }

    pub const fn pelvis_imu(&self) -> ImuState {
        self.last_pelvis_imu
    }

    /// Executes one deterministic kinematic estimation tick and returns `(telemetry, swing_hint)`.
    ///
    /// `measured_vel_rad_s` is the calibrated joint-rate array (same joint
    /// indexing as `measured_rad`). For full-body legged profiles it drives the
    /// analytic mass-weighted CoM Jacobian in [`crate::com_fk`], and transforms
    /// chest-mounted IMU readings back to the pelvis frame.
    #[allow(clippy::too_many_arguments)]
    pub fn step(
        &mut self,
        profile: &RobotProfile,
        dt: f32,
        joint_count: usize,
        measured_rad: &[f32; MAX_JOINTS],
        measured_vel_rad_s: &[f32; MAX_JOINTS],
        imu: ImuState,
        hip_height_m: f32,
        contact_mask: u8,
        grf_n: [f32; 2],
        grf_valid: bool,
        walk_enabled: bool,
        imu_source_mask: u32,
        ukemi_active: bool,
        gains: BalanceGains,
    ) -> (EstimatorTelemetry, DcmSwingHint) {
        let dt = dt.clamp(0.0005_f32, 0.05_f32);
        let support = Support::from_mask(contact_mask);
        let support_swapped = support != self.prev_support;

        let use_fk = profile.joint_indices.l_hip_pitch.is_some()
            && joint_count >= profile.joint_count
            && !profile.links.is_empty();

        // 1. Support-relative CoM position & velocity + chest-IMU pelvis compensation.
        let (com_pos, raw_vel, pelvis_imu): ([f32; 3], [f32; 3], ImuState) = if use_fk {
            // Validated mass-weighted FK path: joint encoders + joint rates only.
            let fk = compute_state(profile, measured_rad, measured_vel_rad_s);
            let pelvis_imu = de_rotate_imu(imu, fk.imu_rot_from_pelvis);
            // Continuous double-support reference used only for the walking
            // capturability margin (the reported DCM stays support-relative).
            let mid_pos = [
                0.5_f32 * (fk.left_foot[0] + fk.right_foot[0]),
                0.5_f32 * (fk.left_foot[1] + fk.right_foot[1]),
                0.5_f32 * (fk.left_foot[2] + fk.right_foot[2]),
            ];
            let mid_vel = [
                0.5_f32 * (fk.left_foot_vel[0] + fk.right_foot_vel[0]),
                0.5_f32 * (fk.left_foot_vel[1] + fk.right_foot_vel[1]),
                0.5_f32 * (fk.left_foot_vel[2] + fk.right_foot_vel[2]),
            ];
            for axis in 0..3 {
                self.com_mid_pos[axis] = fk.com[axis] - mid_pos[axis];
                self.com_mid_vel[axis] = fk.com_vel[axis] - mid_vel[axis];
            }
            let (support_pos, support_vel) = match support {
                Support::Left => (fk.left_foot, fk.left_foot_vel),
                Support::Right => (fk.right_foot, fk.right_foot_vel),
                Support::Double => (
                    [
                        0.5_f32 * (fk.left_foot[0] + fk.right_foot[0]),
                        0.5_f32 * (fk.left_foot[1] + fk.right_foot[1]),
                        0.5_f32 * (fk.left_foot[2] + fk.right_foot[2]),
                    ],
                    [
                        0.5_f32 * (fk.left_foot_vel[0] + fk.right_foot_vel[0]),
                        0.5_f32 * (fk.left_foot_vel[1] + fk.right_foot_vel[1]),
                        0.5_f32 * (fk.left_foot_vel[2] + fk.right_foot_vel[2]),
                    ],
                ),
                // Airborne: keep the last loaded origin. Snapping to the pelvis
                // would make ξ jump to zero and command a foot placement under
                // the body at the exact moment the swing foot must still reach.
                Support::None => (self.last_support_pos, self.last_support_vel),
            };
            if support != Support::None {
                self.last_support_pos = support_pos;
                self.last_support_vel = support_vel;
            }
            // Smooth the support origin so a simultaneous leg re-target at a
            // swap cannot inject a tens-of-centimetres jump into ξ. Track quickly
            // while settled, slowly right after a swap.
            if !self.support_init {
                self.support_pos_f = support_pos;
                self.support_vel_f = support_vel;
                self.support_init = true;
            } else {
                // Standing balance can flicker between soles while a catch step
                // settles, so move the origin slowly. Walking keeps the origin
                // close to the gait-clock support (fast tracking) — filtering it
                // more heavily desynchronises the swing target and causes flight.
                let alpha = if support_swapped {
                    if walk_enabled { 0.25_f32 } else { 0.12_f32 }
                } else if walk_enabled {
                    0.9_f32
                } else {
                    (18.0_f32 * dt).clamp(0.0_f32, 0.9_f32)
                };
                for axis in 0..3 {
                    self.support_pos_f[axis] +=
                        alpha * (support_pos[axis] - self.support_pos_f[axis]);
                    self.support_vel_f[axis] +=
                        alpha * (support_vel[axis] - self.support_vel_f[axis]);
                }
            }
            let support_pos = self.support_pos_f;
            let support_vel = self.support_vel_f;
            // Support origin is expressed in the pelvis frame; subtracting it
            // gives ground/support-relative position and velocity.
            (
                [
                    fk.com[0] - support_pos[0],
                    fk.com[1] - support_pos[1],
                    fk.com[2] - support_pos[2],
                ],
                [
                    fk.com_vel[0] - support_vel[0],
                    fk.com_vel[1] - support_vel[1],
                    fk.com_vel[2] - support_vel[2],
                ],
                pelvis_imu,
            )
        } else {
            // Legacy proxy fallback (non-FK / <17-DoF rigs): IMU-tilt CoM proxy with
            // finite-difference + gyro-blend velocity.
            let (p, v) = proxy_state(measured_rad, imu, hip_height_m);
            (p, v, imu)
        };
        self.last_pelvis_imu = pelvis_imu;

        // 2. Optional EMA smoothing of the analytic velocity. On a support swap
        //    we snap to the fresh value instead of blending, so the change in
        //    support origin cannot leak into a spurious velocity spike.
        if !self.initialized || support_swapped {
            self.prev_com = com_pos;
            self.com_vel = raw_vel;
            self.prev_vel = raw_vel;
            self.com_acc = [0.0; 3];
            self.initialized = true;
        } else {
            let alpha = (VEL_EMA_ALPHA_200HZ * (dt / REF_EST_DT_SEC)).clamp(0.05_f32, 0.85_f32);
            if contact_mask == 0 {
                // Airborne: bleed velocity gently during fall/lift.
                let decay = (1.0_f32 - 8.0_f32 * dt).clamp(0.0_f32, 1.0_f32);
                for axis in 0..3 {
                    self.com_vel[axis] *= decay;
                    self.com_acc[axis] *= decay;
                }
            } else {
                let inv_dt = 1.0_f32 / dt;
                for axis in 0..3 {
                    self.prev_vel[axis] = self.com_vel[axis];
                    self.com_vel[axis] += alpha * (raw_vel[axis] - self.com_vel[axis]);
                    let raw_a = ((self.com_vel[axis] - self.prev_vel[axis]) * inv_dt)
                        .clamp(-12.0_f32, 12.0_f32);
                    self.com_acc[axis] += (alpha * 0.5_f32) * (raw_a - self.com_acc[axis]);
                }
            }
            self.prev_com = com_pos;
        }
        self.prev_support = support;
        let com_pos = com_pos; // use raw support-relative position (not smoothed)
        // TODO(P2 cross-file, coordinating agent): `uvc.rs:527-530` still clamps
        // h_com to 0.60 with a 0.25 fallback. With the new 0.72 default UVC's ω
        // disagrees with this estimator's ω. That file is not owned by this P2
        // agent; it needs clamp `[0.12, 0.80]` and fallback `0.72` (plus the
        // stale comment at uvc.rs:526) to restore a single source of truth.
        let h_com = if gains.h_com_m.is_finite() {
            gains.h_com_m.clamp(H_COM_MIN_M, H_COM_MAX_M)
        } else {
            0.25_f32
        };

        // 3. LIPM natural frequency & support-relative Capture Point.
        //    omega = sqrt(g / h_com),  xi = c_rel + c_dot_rel / omega
        //    REP-103 (`z_up = true`): X (0) is sagittal, Y (1) is lateral, Z (2) is up.
        let lat_idx = if profile.z_up { 1 } else { 0 };
        let sag_idx = if profile.z_up { 0 } else { 2 };
        let omega = sqrtf(GRAVITY_M_S2 / h_com);
        let inv_omega = 1.0_f32 / omega.max(0.5_f32);
        let dcm_x = com_pos[lat_idx] + self.com_vel[lat_idx] * inv_omega;
        let dcm_z = com_pos[sag_idx] + self.com_vel[sag_idx] * inv_omega;

        // 4. Kinematic ZMP estimate from fixed h_com, clamped to the physical
        //    foot half-length.
        let foot_half = gains.foot_half_length_m.clamp(0.015_f32, 0.12_f32);
        let zmp_x = (com_pos[lat_idx] - (h_com / GRAVITY_M_S2) * self.com_acc[lat_idx])
            .clamp(-foot_half * 2.5_f32, foot_half * 2.5_f32);
        let zmp_z = (com_pos[sag_idx] - (h_com / GRAVITY_M_S2) * self.com_acc[sag_idx])
            .clamp(-foot_half, foot_half);

        // 5. Support polygon margin against 7 cm foot (±2.8 cm usable sagittal ankle margin).
        let usable_margin = gains.foot_margin_m.clamp(0.010_f32, foot_half);
        let lat_margin = if contact_mask == 0b11 {
            // Double support widens lateral polygon across both hips
            usable_margin * 2.4_f32
        } else {
            usable_margin
        };
        // While walking, the capture region is the current sole *plus the next
        // reachable swing-foot landing*. A capture point past the sole edge is
        // normal mid-stance locomotion and must not latch a permanent step
        // trigger; only a point beyond the next step's reach is uncapturable.
        // Standing balance keeps the strict support-foot margin so a shove is
        // still reported as a breach.
        let next_step_reach = profile.leg_geometry.max_swing_dx;
        let hip_half_spacing = profile.leg_geometry.half_hip_spacing;
        let (sag_remaining, lat_remaining) = if walk_enabled {
            let mid_dcm_x = self.com_mid_pos[lat_idx] + self.com_mid_vel[lat_idx] * inv_omega;
            let mid_dcm_z = self.com_mid_pos[sag_idx] + self.com_mid_vel[sag_idx] * inv_omega;
            (
                (usable_margin + next_step_reach) - mid_dcm_z.abs(),
                (lat_margin + next_step_reach + hip_half_spacing) - mid_dcm_x.abs(),
            )
        } else {
            (usable_margin - dcm_z.abs(), lat_margin - dcm_x.abs())
        };
        let signed_margin = sag_remaining.min(lat_remaining);

        // 6. Swing-foot landing modulation (strictly swing-foot only when margin breached)
        // TODO(P2 cross-file, coordinating agent): `uvc.rs` keeps the robot in
        // `UvcPhase::ActiveCatchStep` for the entire walk (`wants_walk` re-enters
        // it from ReadyWatch/PostStepSettle, and the walking clock has no exit
        // path), so `catch_step_fraction` stays 1.0 even with a healthy margin.
        // Needed uvc.rs change: add a dedicated Walking phase (or report a
        // `uvc_phase::WALKING` wire code while `walk_enabled && !margin_breached`)
        // and gate `enter_catch_step` on an actual support-polygon breach or a
        // tilted torso instead of `wants_walk` alone.
        let margin_breached = signed_margin < 0.0_f32;
        let dcm_gain = gains.dcm_step_gain.clamp(0.0_f32, 1.5_f32);
        let swing_dx_m = if dcm_z.abs() > usable_margin {
            let excess = dcm_z - dcm_z.signum() * usable_margin;
            (excess * dcm_gain).clamp(-0.18_f32, 0.18_f32)
        } else {
            0.0_f32
        };
        let swing_dy_m = if dcm_x.abs() > lat_margin {
            let excess = dcm_x - dcm_x.signum() * lat_margin;
            (excess * dcm_gain).clamp(-0.14_f32, 0.14_f32)
        } else {
            0.0_f32
        };

        // 7. Explicit channel validity bitmask: MOMENTUM is never set on v1, and
        //    GRF is set only when the kernel received a valid sensed contact
        //    snapshot. CoM & DCM are published only while the IMU is live; a
        //    dropout must not publish stale kinematic state as authoritative.
        let imu_bits = if imu.available {
            let masked =
                imu_source_mask & (estimator_valid::LSM6DSV_OK | estimator_valid::BNO055_OK);
            if masked != 0 {
                masked
            } else {
                estimator_valid::LSM6DSV_OK
            }
        } else {
            0
        };
        let mut valid_mask = if imu.available {
            estimator_valid::COM | estimator_valid::DCM | imu_bits
        } else {
            0
        };
        // Feedback GRF is `[right, left]`; telemetry keeps the named channels.
        let (grf_left_n, grf_right_n) = if grf_valid {
            valid_mask |= estimator_valid::GRF;
            (grf_n[1], grf_n[0])
        } else {
            (0.0_f32, 0.0_f32)
        };

        let telemetry = EstimatorTelemetry {
            com_pos_m: com_pos,
            com_vel_m_s: self.com_vel,
            dcm_xy_m: [dcm_x, dcm_z],
            zmp_xy_m: [zmp_x, zmp_z],
            foot_margin_m: signed_margin,
            grf_left_n,
            grf_right_n,
            contact_mask,
            ukemi_active,
            valid_mask,
        };
        self.last_telemetry = telemetry;

        (
            telemetry,
            DcmSwingHint {
                margin_breached,
                swing_dx_m,
                swing_dy_m,
            },
        )
    }
}

/// Transforms an IMU reading measured in a torso/chest link back into the
/// pelvis frame using `R_pelvis_to_imu` from [`crate::com_fk`].
///
/// When `r_pelvis_to_imu` is identity (pelvis-mounted IMU), `imu` is returned
/// unchanged.
pub fn de_rotate_imu(imu: ImuState, r_pelvis_to_imu: [[f32; 3]; 3]) -> ImuState {
    if !imu.available {
        return imu;
    }
    let is_identity = (r_pelvis_to_imu[0][0] - 1.0_f32).abs() < 1e-6_f32
        && (r_pelvis_to_imu[1][1] - 1.0_f32).abs() < 1e-6_f32
        && (r_pelvis_to_imu[2][2] - 1.0_f32).abs() < 1e-6_f32
        && r_pelvis_to_imu[0][1].abs() < 1e-6_f32
        && r_pelvis_to_imu[0][2].abs() < 1e-6_f32
        && r_pelvis_to_imu[1][2].abs() < 1e-6_f32;
    if is_identity {
        return imu;
    }

    let r_world_to_imu = rot_from_rpy([imu.roll_rad, imu.pitch_rad, imu.yaw_rad]);
    let r_imu_to_pelvis = [
        [r_pelvis_to_imu[0][0], r_pelvis_to_imu[1][0], r_pelvis_to_imu[2][0]],
        [r_pelvis_to_imu[0][1], r_pelvis_to_imu[1][1], r_pelvis_to_imu[2][1]],
        [r_pelvis_to_imu[0][2], r_pelvis_to_imu[1][2], r_pelvis_to_imu[2][2]],
    ];
    let r_world_to_pelvis = mat_mul(r_world_to_imu, r_imu_to_pelvis);

    let roll_rad = atan2f(r_world_to_pelvis[1][2], r_world_to_pelvis[2][2]);
    let cos_p_mag = sqrtf(
        r_world_to_pelvis[1][2] * r_world_to_pelvis[1][2]
            + r_world_to_pelvis[2][2] * r_world_to_pelvis[2][2],
    );
    let pitch_rad = atan2f(-r_world_to_pelvis[0][2], cos_p_mag);
    let yaw_rad = atan2f(r_world_to_pelvis[0][1], r_world_to_pelvis[0][0]);

    let rates = mat_vec(
        r_pelvis_to_imu,
        [imu.roll_rate_rad_s, imu.pitch_rate_rad_s, imu.yaw_rate_rad_s],
    );
    let gravity = mat_vec(r_pelvis_to_imu, imu.gravity);

    ImuState {
        available: imu.available,
        pitch_rad,
        roll_rad,
        yaw_rad,
        pitch_rate_rad_s: rates[1],
        roll_rate_rad_s: rates[0],
        yaw_rate_rad_s: rates[2],
        gravity,
    }
}

/// Legacy IMU-tilt proxy for non-21-DoF rigs. Returns support-relative (i.e.
/// ground-frame) CoM position and a finite-difference + gyro velocity estimate.
fn proxy_state(
    measured_rad: &[f32; MAX_JOINTS],
    imu: ImuState,
    hip_height_m: f32,
) -> ([f32; 3], [f32; 3]) {
    use libm::{cosf, sinf};

    let h_hip = hip_height_m.clamp(0.14_f32, 0.85_f32);
    let torso_lever = h_hip * 0.34_f32;
    let base_com_y = (h_hip * 0.82_f32 + 0.06_f32).max(0.15_f32);

    // Upper-body kinematic shift from shoulder pitch/roll (indices 3,4 left; 6,7 right).
    let (arm_dx, arm_dz) = {
        let l_sp = measured_rad[3];
        let l_sr = measured_rad[4];
        let r_sp = measured_rad[6];
        let r_sr = measured_rad[7];
        let dx = 0.022_f32 * (sinf(r_sr) - sinf(l_sr));
        let dz = -0.024_f32 * (sinf(l_sp) + sinf(r_sp));
        (dx, dz)
    };

    let sin_p = sinf(imu.pitch_rad.clamp(-1.45_f32, 1.45_f32));
    let cos_p = cosf(imu.pitch_rad.clamp(-1.45_f32, 1.45_f32));
    let sin_r = sinf(imu.roll_rad.clamp(-1.45_f32, 1.45_f32));
    let cos_r = cosf(imu.roll_rad.clamp(-1.45_f32, 1.45_f32));

    let lever = base_com_y * 0.65_f32 + torso_lever;
    let com_x = lever * sin_r + arm_dx;
    let com_z = lever * sin_p + arm_dz;
    let com_y = (base_com_y * (cos_p * cos_r).max(0.25_f32)).max(0.12_f32);

    // Gyro lever projection gives an instantaneous velocity estimate for the
    // legacy path (no joint rates available).
    let gyro_vx = (lever * imu.roll_rate_rad_s * cos_r).clamp(-3.0_f32, 3.0_f32);
    let gyro_vz = (lever * imu.pitch_rate_rad_s * cos_p).clamp(-3.0_f32, 3.0_f32);

    ([com_x, com_y, com_z], [gyro_vx, 0.0_f32, gyro_vz])
}

#[cfg(test)]
mod tests {
    use super::*;
    use rk_robot_profile::generated::humanoid::PROFILE as HUMANOID_PROFILE;

    fn imu_upright() -> ImuState {
        ImuState {
            available: true,
            ..Default::default()
        }
    }

    fn gains() -> BalanceGains {
        BalanceGains {
            foot_half_length_m: 0.035,
            foot_margin_m: 0.028,
            dcm_step_gain: 1.0,
            ..Default::default()
        }
    }

    /// Helper for the common "no sensed contact, standing balance" test case.
    fn step_quiet(
        est: &mut KinematicEstimator,
        angles: &[f32; MAX_JOINTS],
        vel: &[f32; MAX_JOINTS],
        imu: ImuState,
        contact_mask: u8,
        imu_source_mask: u32,
        gains: BalanceGains,
    ) -> (EstimatorTelemetry, DcmSwingHint) {
        est.step(
            &HUMANOID_PROFILE,
            0.005,
            HUMANOID_PROFILE.joint_count,
            angles,
            vel,
            imu,
            0.4,
            contact_mask,
            [0.0; 2],
            false,
            false,
            imu_source_mask,
            false,
            gains,
        )
    }

    #[test]
    fn fk_path_zero_pose_keeps_dcm_inside_margin() {
        let mut est = KinematicEstimator::new();
        let angles = [0.0_f32; MAX_JOINTS];
        let vel = [0.0_f32; MAX_JOINTS];
        let (telem, _) =
            step_quiet(&mut est, &angles, &vel, imu_upright(), 0b11, 0, gains());
        assert!(
            telem.foot_margin_m > 0.0,
            "quiet 21-DoF stance must be inside the support margin: {}",
            telem.foot_margin_m
        );
    }

    #[test]
    fn joint_velocity_impulse_pushes_support_relative_dcm_outside_margin() {
        let mut est = KinematicEstimator::new();
        let angles = [0.0_f32; MAX_JOINTS];
        let mut vel = [0.0_f32; MAX_JOINTS];
        // Forward hip-pitch rate (negative Z CoM velocity in pelvis frame)
        // must produce a support-relative DCM excursion.
        let l_hp = HUMANOID_PROFILE.joint_indices.l_hip_pitch.unwrap() as usize;
        let r_hp = HUMANOID_PROFILE.joint_indices.r_hip_pitch.unwrap() as usize;
        vel[l_hp] = -1.5_f32;
        vel[r_hp] = -1.5_f32;
        let mut last = telem_of(&mut est, 0, &angles, &vel);
        for tick in 1..=20 {
            last = telem_of(&mut est, tick, &angles, &vel);
        }
        assert!(
            last.foot_margin_m < 0.0,
            "joint-rate impulse must breach the support margin: {}",
            last.foot_margin_m
        );
    }

    fn telem_of(
        est: &mut KinematicEstimator,
        tick: u32,
        angles: &[f32; MAX_JOINTS],
        vel: &[f32; MAX_JOINTS],
    ) -> EstimatorTelemetry {
        let (t, _) = step_quiet(est, angles, vel, imu_upright(), 0b11, tick, gains());
        t
    }

    #[test]
    fn support_swap_does_not_spike_com_velocity() {
        let mut est = KinematicEstimator::new();
        let angles = [0.0_f32; MAX_JOINTS];
        let mut vel = [0.0_f32; MAX_JOINTS];
        vel[11] = 0.5_f32;
        vel[17] = -0.3_f32;
        // Warm up in double support.
        for tick in 0..20 {
            let _ = telem_of(&mut est, tick, &angles, &vel);
        }
        let before = est.last_telemetry().com_vel_m_s;
        // Switch to left stance (contact_mask 0b01).
        let (after, _) =
            step_quiet(&mut est, &angles, &vel, imu_upright(), 0b01, 20, gains());
        for axis in 0..3 {
            let delta = (after.com_vel_m_s[axis] - before[axis]).abs();
            assert!(
                delta < 0.5_f32,
                "support swap must not spike CoM velocity on axis {axis}: {delta}"
            );
        }
    }

    #[test]
    fn h_com_gain_controls_dcm_frequency_and_is_clamped() {
        let angles = [0.0_f32; MAX_JOINTS];
        let mut vel = [0.0_f32; MAX_JOINTS];
        let l_hp = HUMANOID_PROFILE.joint_indices.l_hip_pitch.unwrap() as usize;
        let r_hp = HUMANOID_PROFILE.joint_indices.r_hip_pitch.unwrap() as usize;
        vel[l_hp] = -0.8_f32;
        vel[r_hp] = -0.8_f32;
        let dcm_for = |h: f32| {
            let mut est = KinematicEstimator::new();
            let mut g = gains();
            g.h_com_m = h;
            let (telem, _) = step_quiet(&mut est, &angles, &vel, imu_upright(), 0b11, 0, g);
            telem.dcm_xy_m
        };
        let low = dcm_for(0.12);
        let high = dcm_for(0.80);
        // Larger h_com -> smaller omega -> larger v/omega sagittal excursion.
        assert!(
            high[1].abs() > low[1].abs(),
            "higher CoM height must increase support-relative DCM excursion: {} vs {}",
            high[1],
            low[1]
        );
        // Non-finite or out-of-range values are clamped, never propagate NaN.
        let clamped = dcm_for(f32::NAN);
        assert!(clamped.iter().all(|v| v.is_finite()));
        let clamped_hi = dcm_for(100.0);
        assert!(clamped_hi.iter().all(|v| v.is_finite()));
        assert_eq!(clamped_hi, dcm_for(0.80));
    }

    #[test]
    fn proxy_fallback_still_works_for_non_21_joint_counts() {
        let mut est = KinematicEstimator::new();
        let angles = [0.0_f32; MAX_JOINTS];
        let vel = [0.0_f32; MAX_JOINTS];
        let mut imu = imu_upright();
        imu.pitch_rad = 0.05;
        let (telem, _) = est.step(
            &HUMANOID_PROFILE,
            0.005, 17, &angles, &vel, imu, 0.4, 0b11, [0.0; 2], false, false, 0, false, gains(),
        );
        assert!(telem.valid_mask & estimator_valid::COM != 0);
    }

    #[test]
    fn sensed_contact_and_grf_are_published_with_validity() {
        let mut est = KinematicEstimator::new();
        let angles = [0.0_f32; MAX_JOINTS];
        let vel = [0.0_f32; MAX_JOINTS];
        let (telem, _) = est.step(
            &HUMANOID_PROFILE,
            0.005,
            HUMANOID_PROFILE.joint_count,
            &angles,
            &vel,
            imu_upright(),
            0.4,
            0b10, // sensed left stance; a stale phase mask would be 0b01
            [20.0_f32, 7.0_f32], // `[right, left]` GRF
            true,
            false,
            0,
            false,
            gains(),
        );
        assert_eq!(telem.contact_mask, 0b10);
        assert_eq!(telem.grf_right_n, 20.0);
        assert_eq!(telem.grf_left_n, 7.0);
        assert_ne!(telem.valid_mask & estimator_valid::GRF, 0);

        // Clear validity: the kernel supplies the fallback mask and zero GRF,
        // and the GRF channel must stay invalid (never read as 0 N free-fall).
        let (invalid, _) = est.step(
            &HUMANOID_PROFILE,
            0.005,
            HUMANOID_PROFILE.joint_count,
            &angles,
            &vel,
            imu_upright(),
            0.4,
            0b11,
            [0.0; 2],
            false,
            false,
            0,
            false,
            gains(),
        );
        assert_eq!(invalid.contact_mask, 0b11);
        assert_eq!((invalid.grf_left_n, invalid.grf_right_n), (0.0, 0.0));
        assert_eq!(invalid.valid_mask & estimator_valid::GRF, 0);
    }

    #[test]
    fn walking_margin_includes_next_step_reach_but_stance_breach_does_not() {
        let angles = [0.0_f32; MAX_JOINTS];
        let mut vel = [0.0_f32; MAX_JOINTS];
        let l_hp = HUMANOID_PROFILE.joint_indices.l_hip_pitch.unwrap() as usize;
        let r_hp = HUMANOID_PROFILE.joint_indices.r_hip_pitch.unwrap() as usize;
        vel[l_hp] = -1.5_f32;
        vel[r_hp] = -1.5_f32;
        let standing = {
            let mut est = KinematicEstimator::new();
            step_quiet(&mut est, &angles, &vel, imu_upright(), 0b11, 0, gains()).0
        };
        let walking = {
            let mut est = KinematicEstimator::new();
            let (telem, _) = est.step(
                &HUMANOID_PROFILE,
                0.005,
                HUMANOID_PROFILE.joint_count,
                &angles,
                &vel,
                imu_upright(),
                0.4,
                0b11,
                [0.0; 2],
                false,
                true, // walk_enabled
                0,
                false,
                gains(),
            );
            telem
        };
        assert!(
            standing.foot_margin_m < 0.0,
            "stationary disturbance must still breach the support footprint"
        );
        assert!(
            walking.foot_margin_m > 0.0_f32 && walking.foot_margin_m > standing.foot_margin_m + 0.05_f32,
            "steady walking must be capturable with the next-step region: {} vs {}",
            walking.foot_margin_m,
            standing.foot_margin_m
        );
    }

    #[test]
    fn default_h_com_matches_fk_sole_to_com_height_at_nominal_crouch() {
        use crate::com_fk::compute_state;
        use crate::uvc::solve_leg_ik;
        use rk_proto::WalkCmd;

        let geom = HUMANOID_PROFILE.leg_geometry;
        let ji = &HUMANOID_PROFILE.joint_indices;
        let crouch = WalkCmd::default().stance_crouch.clamp(0.0_f32, 1.0_f32);
        let hip_h = (geom.nominal_height
            - crouch * (geom.nominal_height - geom.min_height) * 0.45_f32)
            .clamp(geom.min_height, geom.nominal_height);
        let left = solve_leg_ik(0.0_f32, 0.0_f32, hip_h, 0.0_f32, true, &geom);
        let right = solve_leg_ik(0.0_f32, 0.0_f32, hip_h, 0.0_f32, false, &geom);
        let mut angles = [0.0_f32; MAX_JOINTS];
        angles[ji.l_hip_yaw.unwrap() as usize] = left.hip_yaw;
        angles[ji.l_hip_roll.unwrap() as usize] = left.hip_roll;
        angles[ji.l_hip_pitch.unwrap() as usize] = left.hip_pitch;
        angles[ji.l_knee_pitch.unwrap() as usize] = left.knee_pitch;
        angles[ji.l_ankle_pitch.unwrap() as usize] = left.ankle_pitch;
        angles[ji.l_ankle_roll.unwrap() as usize] = left.ankle_roll;
        angles[ji.r_hip_yaw.unwrap() as usize] = right.hip_yaw;
        angles[ji.r_hip_roll.unwrap() as usize] = right.hip_roll;
        angles[ji.r_hip_pitch.unwrap() as usize] = right.hip_pitch;
        angles[ji.r_knee_pitch.unwrap() as usize] = right.knee_pitch;
        angles[ji.r_ankle_pitch.unwrap() as usize] = right.ankle_pitch;
        angles[ji.r_ankle_roll.unwrap() as usize] = right.ankle_roll;

        let fk = compute_state(&HUMANOID_PROFILE, &angles, &[0.0_f32; MAX_JOINTS]);
        let sole_z = 0.5_f32 * (fk.left_foot[2] + fk.right_foot[2]);
        let sole_to_com_m = fk.com[2] - sole_z;
        assert!(
            (HUMANOID_PROFILE.h_com_m - sole_to_com_m).abs() < 0.08_f32,
            "profile h_com_m {} must match FK sole-to-CoM {:.4} m",
            HUMANOID_PROFILE.h_com_m,
            sole_to_com_m
        );
    }

    #[test]
    fn chest_imu_compensates_for_waist_pitch() {
        // Pelvis is level in world, while waist_pitch tilts chest by +0.35 rad.
        // Chest IMU reads pitch = +0.35 rad; de_rotate_imu must recover ~0.0 rad.
        let waist_pitch = 0.35_f32;
        let r_pelvis_to_imu = rot_from_rpy([0.0, waist_pitch, 0.0]);
        let chest_imu = ImuState {
            available: true,
            pitch_rad: waist_pitch,
            roll_rad: 0.0,
            yaw_rad: 0.0,
            ..Default::default()
        };
        let pelvis_imu = de_rotate_imu(chest_imu, r_pelvis_to_imu);
        assert!(
            pelvis_imu.pitch_rad.abs() < 1e-5_f32,
            "de-rotated pelvis pitch should be ~0, got {}",
            pelvis_imu.pitch_rad
        );
        assert!(
            pelvis_imu.roll_rad.abs() < 1e-5_f32,
            "de-rotated pelvis roll should be ~0, got {}",
            pelvis_imu.roll_rad
        );
    }
}
