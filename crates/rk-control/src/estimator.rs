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

use libm::sqrtf;
use rk_proto::{BalanceGains, EstimatorTelemetry, ImuState, MAX_JOINTS, estimator_valid};

use crate::com_fk::compute_state;

const GRAVITY_M_S2: f32 = 9.81_f32;
/// Reference 200 Hz tick duration (5 ms) for EMA velocity filter.
const REF_EST_DT_SEC: f32 = 0.005_f32;
const VEL_EMA_ALPHA_200HZ: f32 = 0.30_f32;
/// Clamping bounds for the configurable CoM height (matches the MCU config clamp).
const H_COM_MIN_M: f32 = 0.12_f32;
const H_COM_MAX_M: f32 = 0.60_f32;
/// Joint count required for the validated mass-weighted FK path.
const FK_JOINT_COUNT: usize = 21;

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
    }

    pub const fn last_telemetry(&self) -> EstimatorTelemetry {
        self.last_telemetry
    }

    /// Executes one deterministic kinematic estimation tick and returns `(telemetry, swing_hint)`.
    ///
    /// `measured_vel_rad_s` is the calibrated joint-rate array (same joint
    /// indexing as `measured_rad`). For 21-DoF robots it drives the analytic
    /// mass-weighted CoM Jacobian in [`crate::com_fk`].
    #[allow(clippy::too_many_arguments)]
    pub fn step(
        &mut self,
        dt: f32,
        joint_count: usize,
        measured_rad: &[f32; MAX_JOINTS],
        measured_vel_rad_s: &[f32; MAX_JOINTS],
        imu: ImuState,
        hip_height_m: f32,
        contact_mask: u8,
        imu_source_mask: u32,
        ukemi_active: bool,
        gains: BalanceGains,
    ) -> (EstimatorTelemetry, DcmSwingHint) {
        let dt = dt.clamp(0.0005_f32, 0.05_f32);
        let support = Support::from_mask(contact_mask);
        let support_swapped = support != self.prev_support;

        // 1. Support-relative CoM position & velocity.
        let (com_pos, raw_vel): ([f32; 3], [f32; 3]) = if joint_count == FK_JOINT_COUNT {
            // Validated mass-weighted FK path: joint encoders + joint rates only.
            let fk = compute_state(measured_rad, measured_vel_rad_s);
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
                Support::None => ([0.0; 3], [0.0; 3]),
            };
            // Support origin is expressed in the pelvis frame; subtracting it
            // gives ground/support-relative position and velocity. Because
            // both fields are continuous in joint state, a support swap does
            // not introduce a velocity spike.
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
            )
        } else {
            // Legacy proxy fallback (non-21-DoF rigs): IMU-tilt CoM proxy with
            // finite-difference + gyro-blend velocity.
            proxy_state(measured_rad, imu, hip_height_m)
        };

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
        let h_com = if gains.h_com_m.is_finite() {
            gains.h_com_m.clamp(H_COM_MIN_M, H_COM_MAX_M)
        } else {
            0.25_f32
        };

        // 3. LIPM natural frequency & support-relative Capture Point.
        //    omega = sqrt(g / h_com),  xi = c_rel + c_dot_rel / omega
        let omega = sqrtf(GRAVITY_M_S2 / h_com);
        let inv_omega = 1.0_f32 / omega.max(0.5_f32);
        let dcm_x = com_pos[0] + self.com_vel[0] * inv_omega;
        let dcm_z = com_pos[2] + self.com_vel[2] * inv_omega;

        // 4. Kinematic ZMP estimate from fixed h_com, clamped to the physical
        //    7 cm printed foot half-length (±3.5 cm).
        let foot_half = gains.foot_half_length_m.clamp(0.015_f32, 0.12_f32);
        let zmp_x = (com_pos[0] - (h_com / GRAVITY_M_S2) * self.com_acc[0])
            .clamp(-foot_half * 2.5_f32, foot_half * 2.5_f32);
        let zmp_z = (com_pos[2] - (h_com / GRAVITY_M_S2) * self.com_acc[2])
            .clamp(-foot_half, foot_half);

        // 5. Support polygon margin against 7 cm foot (±2.8 cm usable sagittal ankle margin).
        let usable_margin = gains.foot_margin_m.clamp(0.010_f32, foot_half);
        let lat_margin = if contact_mask == 0b11 {
            // Double support widens lateral polygon across both hips
            usable_margin * 2.4_f32
        } else {
            usable_margin
        };
        let sag_remaining = usable_margin - dcm_z.abs();
        let lat_remaining = lat_margin - dcm_x.abs();
        let signed_margin = sag_remaining.min(lat_remaining);

        // 6. Swing-foot landing modulation (strictly swing-foot only when margin breached)
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

        // 7. Explicit channel validity bitmask: v1 NEVER sets GRF or MOMENTUM!
        //    CoM & DCM are published only while the IMU is live; a dropout must
        //    not publish stale kinematic state as authoritative.
        let imu_bits = if imu.available {
            let masked = imu_source_mask & (estimator_valid::LSM6DSV_OK | estimator_valid::BNO055_OK);
            if masked != 0 {
                masked
            } else {
                estimator_valid::LSM6DSV_OK
            }
        } else {
            0
        };
        let valid_mask = if imu.available {
            estimator_valid::COM | estimator_valid::DCM | imu_bits
        } else {
            0
        };

        let telemetry = EstimatorTelemetry {
            com_pos_m: com_pos,
            com_vel_m_s: self.com_vel,
            dcm_xy_m: [dcm_x, dcm_z],
            zmp_xy_m: [zmp_x, zmp_z],
            foot_margin_m: signed_margin,
            grf_left_n: 0.0,
            grf_right_n: 0.0,
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

    #[test]
    fn fk_path_zero_pose_keeps_dcm_inside_margin() {
        let mut est = KinematicEstimator::new();
        let angles = [0.0_f32; MAX_JOINTS];
        let vel = [0.0_f32; MAX_JOINTS];
        let (telem, _) = est.step(
            0.005, 21, &angles, &vel, imu_upright(), 0.4, 0b11, 0, false, gains(),
        );
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
        vel[11] = -2.5_f32;
        vel[17] = -2.5_f32;
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
        let (t, _) = est.step(
            0.005, 21, angles, vel, imu_upright(), 0.4, 0b11, tick, false, gains(),
        );
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
        let (after, _) = est.step(
            0.005, 21, &angles, &vel, imu_upright(), 0.4, 0b01, 20, false, gains(),
        );
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
        vel[11] = -0.8_f32;
        vel[17] = -0.8_f32;
        let dcm_for = |h: f32| {
            let mut est = KinematicEstimator::new();
            let mut g = gains();
            g.h_com_m = h;
            let (telem, _) = est.step(
                0.005, 21, &angles, &vel, imu_upright(), 0.4, 0b11, 0, false, g,
            );
            telem.dcm_xy_m
        };
        let low = dcm_for(0.12);
        let high = dcm_for(0.60);
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
        assert_eq!(clamped_hi, dcm_for(0.60));
    }

    #[test]
    fn proxy_fallback_still_works_for_non_21_joint_counts() {
        let mut est = KinematicEstimator::new();
        let angles = [0.0_f32; MAX_JOINTS];
        let vel = [0.0_f32; MAX_JOINTS];
        let mut imu = imu_upright();
        imu.pitch_rad = 0.05;
        let (telem, _) = est.step(
            0.005, 17, &angles, &vel, imu, 0.4, 0b11, 0, false, gains(),
        );
        assert!(telem.valid_mask & estimator_valid::COM != 0);
    }
}
