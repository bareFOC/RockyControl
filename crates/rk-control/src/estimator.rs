//! Pure `#![no_std]` Kinematic CoM, Divergent Component of Motion (DCM / Capture Point),
//! and ZMP Estimator for Rocky Mk1 (`rk-control`).
//!
//! Designed for strict RP2350 Cortex-M33F parity:
//! - Strictly single-precision `f32` via `libm`, zero heap allocations.
//! - Computes 21-DoF lumped-mass Center of Mass (`com_pos_m`), EMA-filtered air-gated
//!   CoM velocity (`com_vel_m_s`, `alpha = 0.30` @ 200 Hz), Capture Point
//!   `xi = c + c_dot / omega` (`omega = sqrt(g / h_com)`), and estimated ZMP.
//! - Encodes the 7 cm printed foot (`foot_half_length_m = 0.035 m`) with ±2.8 cm
//!   (`foot_margin_m = 0.028 m`) usable ankle/CoP authority before kinematic DCM
//!   triggers a recovery step.
//! - Strict DOF partitioning: outputs [`DcmSwingHint`] exclusively for swing-foot
//!   placement modulation — never double-driving the stance hip UVC integrator.
//! - Explicit `valid_mask` (`estimator_valid::*`): v1 sets `COM | DCM | imu_source`,
//!   leaving `GRF` and `MOMENTUM` unset so disabled sensors never read as `0.0 N` free-fall.

use libm::{cosf, sinf, sqrtf};
use rk_proto::{BalanceGains, EstimatorTelemetry, ImuState, MAX_JOINTS, estimator_valid};

const GRAVITY_M_S2: f32 = 9.81_f32;
/// Reference 200 Hz tick duration (5 ms) for EMA velocity filter (`alpha = 0.30`).
const REF_EST_DT_SEC: f32 = 0.005_f32;
const VEL_EMA_ALPHA_200HZ: f32 = 0.30_f32;

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

/// Pure `#![no_std]` kinematic state estimator running at 200 Hz on RP2350 & sim.
#[derive(Clone, Debug)]
pub struct KinematicEstimator {
    initialized: bool,
    prev_com: [f32; 3],
    com_vel: [f32; 3],
    prev_vel: [f32; 3],
    com_acc: [f32; 3],
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
    }

    pub const fn last_telemetry(&self) -> EstimatorTelemetry {
        self.last_telemetry
    }

    /// Executes one deterministic kinematic estimation tick and returns `(telemetry, swing_hint)`.
    #[allow(clippy::too_many_arguments)]
    pub fn step(
        &mut self,
        dt: f32,
        joint_count: usize,
        measured_rad: &[f32; MAX_JOINTS],
        imu: ImuState,
        hip_height_m: f32,
        contact_mask: u8,
        imu_source_mask: u32,
        ukemi_active: bool,
        gains: BalanceGains,
    ) -> (EstimatorTelemetry, DcmSwingHint) {
        let dt = dt.clamp(0.0005_f32, 0.05_f32);

        // 1. Lumped-mass kinematic Center of Mass in stance ground frame:
        //    +X = right, +Y = up, +Z = forward (consistent with Rocky Mk1 URDF & IMU).
        let h_hip = hip_height_m.clamp(0.14_f32, 0.85_f32);
        let torso_lever = h_hip * 0.34_f32;
        let base_com_y = (h_hip * 0.82_f32 + 0.06_f32).max(0.15_f32);

        // Upper-body kinematic shift from shoulder pitch/roll (indices 3,4 left; 6,7 right).
        let (arm_dx, arm_dz) = if joint_count >= 9 {
            let l_sp = measured_rad[3];
            let l_sr = measured_rad[4];
            let r_sp = measured_rad[6];
            let r_sr = measured_rad[7];
            let dx = 0.022_f32 * (sinf(r_sr) - sinf(l_sr));
            let dz = -0.024_f32 * (sinf(l_sp) + sinf(r_sp));
            (dx, dz)
        } else {
            (0.0_f32, 0.0_f32)
        };

        // Torso & pelvis inclination relative to flat support sole.
        let sin_p = sinf(imu.pitch_rad.clamp(-1.45_f32, 1.45_f32));
        let cos_p = cosf(imu.pitch_rad.clamp(-1.45_f32, 1.45_f32));
        let sin_r = sinf(imu.roll_rad.clamp(-1.45_f32, 1.45_f32));
        let cos_r = cosf(imu.roll_rad.clamp(-1.45_f32, 1.45_f32));

        let com_x = (base_com_y * 0.65_f32 + torso_lever) * sin_r + arm_dx;
        let com_z = (base_com_y * 0.65_f32 + torso_lever) * sin_p + arm_dz;
        let com_y = (base_com_y * (cos_p * cos_r).max(0.25_f32)).max(0.12_f32);
        let com_pos = [com_x, com_y, com_z];

        // 2. Air-gated EMA CoM velocity (alpha = 0.30 @ 200 Hz, rate-normalized for dt)
        //    plus gyro angular-rate kinematic projection so Capture Point reacts with zero lag.
        if !self.initialized {
            self.prev_com = com_pos;
            self.com_vel = [0.0; 3];
            self.prev_vel = [0.0; 3];
            self.com_acc = [0.0; 3];
            self.initialized = true;
        } else {
            let alpha = (VEL_EMA_ALPHA_200HZ * (dt / REF_EST_DT_SEC)).clamp(0.05_f32, 0.85_f32);
            if contact_mask == 0 {
                // Air-gated: bleed finite-difference CoM velocity smoothly during fall/lift
                let decay = (1.0_f32 - 8.0_f32 * dt).clamp(0.0_f32, 1.0_f32);
                for axis in 0..3 {
                    self.com_vel[axis] *= decay;
                    self.com_acc[axis] *= decay;
                }
            } else {
                let inv_dt = 1.0_f32 / dt;
                let fd_vx = ((com_pos[0] - self.prev_com[0]) * inv_dt).clamp(-3.0_f32, 3.0_f32);
                let fd_vy = ((com_pos[1] - self.prev_com[1]) * inv_dt).clamp(-3.0_f32, 3.0_f32);
                let fd_vz = ((com_pos[2] - self.prev_com[2]) * inv_dt).clamp(-3.0_f32, 3.0_f32);

                // Fuse kinematic gyro angular-velocity lever term (`r * omega_imu`) for zero-lag DCM
                let lever = base_com_y * 0.65_f32 + torso_lever;
                let gyro_vx = (lever * imu.roll_rate_rad_s * cos_r).clamp(-3.0_f32, 3.0_f32);
                let gyro_vz = (lever * imu.pitch_rate_rad_s * cos_p).clamp(-3.0_f32, 3.0_f32);

                let raw_v = [
                    0.55_f32 * fd_vx + 0.45_f32 * gyro_vx,
                    fd_vy,
                    0.55_f32 * fd_vz + 0.45_f32 * gyro_vz,
                ];
                for axis in 0..3 {
                    self.prev_vel[axis] = self.com_vel[axis];
                    self.com_vel[axis] += alpha * (raw_v[axis] - self.com_vel[axis]);
                    let raw_a = ((self.com_vel[axis] - self.prev_vel[axis]) * inv_dt)
                        .clamp(-12.0_f32, 12.0_f32);
                    self.com_acc[axis] += (alpha * 0.5_f32) * (raw_a - self.com_acc[axis]);
                }
            }
            self.prev_com = com_pos;
        }

        // 3. Linear Inverted Pendulum natural frequency & Divergent Component of Motion (DCM)
        //    omega = sqrt(g / h_com),  xi = c + c_dot / omega
        let h_com = com_pos[1].max(0.14_f32);
        let omega = sqrtf(GRAVITY_M_S2 / h_com);
        let inv_omega = 1.0_f32 / omega.max(0.5_f32);
        let dcm_x = com_pos[0] + self.com_vel[0] * inv_omega;
        let dcm_z = com_pos[2] + self.com_vel[2] * inv_omega;

        // 4. Kinematic ZMP estimate clamped to physical 7 cm printed foot half-length (±3.5 cm)
        let foot_half = gains.foot_half_length_m.clamp(0.015_f32, 0.12_f32);
        let zmp_x = (com_pos[0] - (h_com / GRAVITY_M_S2) * self.com_acc[0])
            .clamp(-foot_half * 2.5_f32, foot_half * 2.5_f32);
        let zmp_z = (com_pos[2] - (h_com / GRAVITY_M_S2) * self.com_acc[2])
            .clamp(-foot_half, foot_half);

        // 5. Support polygon margin against 7 cm foot (±2.8 cm usable sagittal ankle margin)
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
        let valid_mask = estimator_valid::COM | estimator_valid::DCM | imu_bits;

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
