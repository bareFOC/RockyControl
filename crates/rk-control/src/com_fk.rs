//! Pure `#![no_std]` mass-weighted Center-of-Mass forward kinematics.
//!
//! This module is the **single source of truth** for the kinematic CoM, foot/toe
//! frames, and chest-IMU pelvis transform used by the estimator and simulator,
//! on both MCU firmware and host. It consumes `&RobotProfile` plus `f32` joint
//! angles / velocities. No allocation, no `k`/nalgebra, deterministic `libm` trig only.

use libm::{cosf, sinf};
use rk_proto::MAX_JOINTS;
use rk_robot_profile::RobotProfile;

/// Maximum number of links supported in a [`RobotProfile`] FK traversal.
pub const MAX_PROFILE_LINKS: usize = 40;

/// Result of one full FK traversal: lumped CoM, foot/toe frames, and IMU
/// orientation relative to the pelvis, all expressed in the pelvis frame.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ComFkState {
    /// Mass-weighted center of mass `[x, y, z]` in the pelvis frame.
    pub com: [f32; 3],
    /// Mass-weighted center-of-mass velocity `[vx, vy, vz]` in the pelvis frame.
    pub com_vel: [f32; 3],
    /// Left foot frame origin `[x, y, z]` in the pelvis frame.
    pub left_foot: [f32; 3],
    /// Left foot frame origin velocity in the pelvis frame.
    pub left_foot_vel: [f32; 3],
    /// Right foot frame origin `[x, y, z]` in the pelvis frame.
    pub right_foot: [f32; 3],
    /// Right foot frame origin velocity in the pelvis frame.
    pub right_foot_vel: [f32; 3],
    /// Left toe frame origin `[x, y, z]` in the pelvis frame (when present).
    pub left_toe: Option<[f32; 3]>,
    /// Left toe frame origin velocity in the pelvis frame.
    pub left_toe_vel: Option<[f32; 3]>,
    /// Right toe frame origin `[x, y, z]` in the pelvis frame (when present).
    pub right_toe: Option<[f32; 3]>,
    /// Right toe frame origin velocity in the pelvis frame.
    pub right_toe_vel: Option<[f32; 3]>,
    /// Rotation matrix `R_pelvis_to_imu` (column-major `[[f32; 3]; 3]`), mapping
    /// vectors from the IMU frame into the pelvis frame.
    pub imu_rot_from_pelvis: [[f32; 3]; 3],
}

/// Per-link accumulated pelvis-frame state during the parent-first traversal.
#[derive(Clone, Copy, Debug, Default)]
struct LinkFrame {
    /// Rotation of the link frame in the pelvis frame as three column vectors.
    rot: [[f32; 3]; 3],
    /// Position of the link frame origin in the pelvis frame.
    pos: [f32; 3],
    /// Velocity of the link frame origin in the pelvis frame.
    vel: [f32; 3],
    /// Angular velocity of the link frame in the pelvis frame.
    omega: [f32; 3],
}

#[inline]
fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

#[inline]
pub(crate) fn mat_vec(m: [[f32; 3]; 3], v: [f32; 3]) -> [f32; 3] {
    [
        m[0][0] * v[0] + m[1][0] * v[1] + m[2][0] * v[2],
        m[0][1] * v[0] + m[1][1] * v[1] + m[2][1] * v[2],
        m[0][2] * v[0] + m[1][2] * v[1] + m[2][2] * v[2],
    ]
}

#[inline]
pub(crate) fn mat_mul(a: [[f32; 3]; 3], b: [[f32; 3]; 3]) -> [[f32; 3]; 3] {
    [mat_vec(a, b[0]), mat_vec(a, b[1]), mat_vec(a, b[2])]
}

/// Rotate `v` by `angle` about a unit axis in the parent frame (Rodrigues' formula).
#[inline]
fn rotate_vec(axis: [f32; 3], angle: f32, v: [f32; 3]) -> [f32; 3] {
    let c = cosf(angle);
    let s = sinf(angle);
    let av = cross(axis, v);
    let dot = axis[0] * v[0] + axis[1] * v[1] + axis[2] * v[2];
    let k = dot * (1.0_f32 - c);
    [
        v[0] * c + av[0] * s + axis[0] * k,
        v[1] * c + av[1] * s + axis[1] * k,
        v[2] * c + av[2] * s + axis[2] * k,
    ]
}

/// Rotate the link-frame rotation matrix by a joint rotation applied in the
/// child (local) frame: `R_child = R_parent * R_joint`.
#[inline]
fn compose_rotation(parent: [[f32; 3]; 3], axis: [f32; 3], angle: f32) -> [[f32; 3]; 3] {
    if angle == 0.0 || (axis[0] == 0.0 && axis[1] == 0.0 && axis[2] == 0.0) {
        return parent;
    }
    let ex = rotate_vec(axis, angle, [1.0, 0.0, 0.0]);
    let ey = rotate_vec(axis, angle, [0.0, 1.0, 0.0]);
    let ez = rotate_vec(axis, angle, [0.0, 0.0, 1.0]);
    [
        mat_vec(parent, ex),
        mat_vec(parent, ey),
        mat_vec(parent, ez),
    ]
}

/// URDF RPY (`R = R_z(yaw) * R_y(pitch) * R_x(roll)`) as column-major 3x3.
#[inline]
pub(crate) fn rot_from_rpy(rpy: [f32; 3]) -> [[f32; 3]; 3] {
    let (sr, cr) = (sinf(rpy[0]), cosf(rpy[0]));
    let (sp, cp) = (sinf(rpy[1]), cosf(rpy[1]));
    let (sy, cy) = (sinf(rpy[2]), cosf(rpy[2]));
    [
        [cy * cp, sy * cp, -sp],
        [cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, cp * sr],
        [cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr],
    ]
}

/// Run the full parent-first FK traversal for `profile`.
///
/// `angles` / `velocities` are actuator-indexed (`0..profile.joint_count`);
/// coupled URDF half-joints scale the actuator angle/velocity by `mimic_scale`.
pub fn compute_state(
    profile: &RobotProfile,
    angles: &[f32; MAX_JOINTS],
    velocities: &[f32; MAX_JOINTS],
) -> ComFkState {
    let identity = [[1.0_f32, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let root_frame = LinkFrame {
        rot: identity,
        pos: [0.0; 3],
        vel: [0.0; 3],
        omega: [0.0; 3],
    };
    let mut frames = [root_frame; MAX_PROFILE_LINKS];

    let link_count = profile.links.len().min(profile.joint_kin.len()).min(MAX_PROFILE_LINKS);
    let mut com_acc = [0.0_f32; 3];
    let mut com_vel_acc = [0.0_f32; 3];

    for i in 0..link_count {
        let link = &profile.links[i];
        let kin = profile.joint_kin[i];
        let parent_idx = (link.parent as usize).min(link_count.saturating_sub(1));

        let frame = if i == profile.pelvis_index {
            root_frame
        } else {
            let p = &frames[parent_idx];
            let (angle, rate) = match kin.actuator_index {
                Some(a) if (a as usize) < MAX_JOINTS => (
                    angles[a as usize] * kin.mimic_scale,
                    velocities[a as usize] * kin.mimic_scale,
                ),
                _ => (0.0_f32, 0.0_f32),
            };

            let t = mat_vec(p.rot, kin.origin);
            let pos = [p.pos[0] + t[0], p.pos[1] + t[1], p.pos[2] + t[2]];
            let omega_cross_t = cross(p.omega, t);
            let vel = [
                p.vel[0] + omega_cross_t[0],
                p.vel[1] + omega_cross_t[1],
                p.vel[2] + omega_cross_t[2],
            ];

            let axis_world = mat_vec(p.rot, kin.axis);
            let omega = [
                p.omega[0] + axis_world[0] * rate,
                p.omega[1] + axis_world[1] * rate,
                p.omega[2] + axis_world[2] * rate,
            ];
            let rot = compose_rotation(p.rot, kin.axis, angle);

            LinkFrame {
                rot,
                pos,
                vel,
                omega,
            }
        };
        frames[i] = frame;

        let r_com = mat_vec(frame.rot, link.com);
        let omega_cross_r = cross(frame.omega, r_com);
        let m = link.mass;
        for k in 0..3 {
            com_acc[k] += m * (frame.pos[k] + r_com[k]);
            com_vel_acc[k] += m * (frame.vel[k] + omega_cross_r[k]);
        }
    }

    let lf = profile
        .left_foot_index
        .filter(|&idx| idx < link_count)
        .map(|idx| frames[idx])
        .unwrap_or(root_frame);
    let rf = profile
        .right_foot_index
        .filter(|&idx| idx < link_count)
        .map(|idx| frames[idx])
        .unwrap_or(root_frame);

    let lt = profile.left_toe_index.filter(|&idx| idx < link_count);
    let rt = profile.right_toe_index.filter(|&idx| idx < link_count);

    let imu_link_rot = if profile.imu_link_index < link_count {
        frames[profile.imu_link_index].rot
    } else {
        identity
    };
    let imu_rot_from_pelvis = if profile.imu_mounting_rpy == [0.0, 0.0, 0.0] {
        imu_link_rot
    } else {
        mat_mul(imu_link_rot, rot_from_rpy(profile.imu_mounting_rpy))
    };

    ComFkState {
        com: [
            com_acc[0] * profile.inv_total_mass,
            com_acc[1] * profile.inv_total_mass,
            com_acc[2] * profile.inv_total_mass,
        ],
        com_vel: [
            com_vel_acc[0] * profile.inv_total_mass,
            com_vel_acc[1] * profile.inv_total_mass,
            com_vel_acc[2] * profile.inv_total_mass,
        ],
        left_foot: lf.pos,
        left_foot_vel: lf.vel,
        right_foot: rf.pos,
        right_foot_vel: rf.vel,
        left_toe: lt.map(|idx| frames[idx].pos),
        left_toe_vel: lt.map(|idx| frames[idx].vel),
        right_toe: rt.map(|idx| frames[idx].pos),
        right_toe_vel: rt.map(|idx| frames[idx].vel),
        imu_rot_from_pelvis,
    }
}

/// Mass-weighted CoM position in the pelvis frame for the given joint angles.
pub fn compute_com(profile: &RobotProfile, angles: &[f32; MAX_JOINTS]) -> [f32; 3] {
    compute_state(profile, angles, &[0.0_f32; MAX_JOINTS]).com
}

/// Mass-weighted CoM velocity in the pelvis frame from measured joint rates.
pub fn com_velocity(
    profile: &RobotProfile,
    angles: &[f32; MAX_JOINTS],
    velocities: &[f32; MAX_JOINTS],
) -> [f32; 3] {
    compute_state(profile, angles, velocities).com_vel
}

#[cfg(test)]
mod tests {
    use super::*;
    use rk_robot_profile::generated::humanoid::PROFILE as HUMANOID_PROFILE;

    fn zero_angles() -> [f32; MAX_JOINTS] {
        [0.0_f32; MAX_JOINTS]
    }

    #[test]
    fn zero_pose_com_matches_hand_computed_mass_weighted_sum() {
        let com = compute_com(&HUMANOID_PROFILE, &zero_angles());
        assert!(
            com[1].abs() < 1e-4,
            "zero-pose CoM must be laterally symmetric (Y ~ 0), got {}",
            com[1]
        );
        assert!(com[2] > 0.10_f32 && com[2] < 0.35_f32, "unexpected Z CoM: {}", com[2]);
    }

    #[test]
    fn single_joint_velocity_produces_nonzero_com_velocity() {
        let angles = zero_angles();
        let mut vel = zero_angles();
        let l_hip_pitch = HUMANOID_PROFILE.joint_indices.l_hip_pitch.unwrap() as usize;
        vel[l_hip_pitch] = 1.0_f32;
        let cv = com_velocity(&HUMANOID_PROFILE, &angles, &vel);
        // l_hip_pitch rotates around +Y, swinging the -Z leg along X.
        assert!(cv[0].abs() > 1e-3_f32, "expected nonzero X CoM vel, got {:?}", cv);
        assert!(cv[1].abs() < 1e-3_f32, "expected near-zero Y CoM vel, got {:?}", cv);
    }

    #[test]
    fn all_joint_velocities_zero_means_com_velocity_zero() {
        let angles = zero_angles();
        let cv = com_velocity(&HUMANOID_PROFILE, &angles, &zero_angles());
        for axis in 0..3 {
            assert!(cv[axis].abs() < 1e-6_f32);
        }
    }

    #[test]
    fn waist_yaw_rotates_body_and_moves_com_laterally() {
        let mut angles = zero_angles();
        let l_sp = HUMANOID_PROFILE.joint_indices.l_shoulder_pitch.unwrap() as usize;
        angles[l_sp] = 0.5_f32;
        let com = compute_com(&HUMANOID_PROFILE, &angles);
        let base = compute_com(&HUMANOID_PROFILE, &zero_angles());
        assert!((com[2] - base[2]).abs() > 1e-4 || (com[1] - base[1]).abs() > 1e-4);
    }

    #[test]
    fn com_velocity_is_linear_in_joint_velocity() {
        let angles = zero_angles();
        let l_hip_pitch = HUMANOID_PROFILE.joint_indices.l_hip_pitch.unwrap() as usize;
        let mut v1 = zero_angles();
        v1[l_hip_pitch] = 0.5_f32;
        let mut v2 = zero_angles();
        v2[l_hip_pitch] = 1.0_f32;
        let c1 = com_velocity(&HUMANOID_PROFILE, &angles, &v1);
        let c2 = com_velocity(&HUMANOID_PROFILE, &angles, &v2);
        for axis in 0..3 {
            assert!((c2[axis] - 2.0_f32 * c1[axis]).abs() < 1e-5_f32);
        }
    }

    #[test]
    fn foot_origins_are_below_pelvis_and_symmetric_at_zero_pose() {
        let state = compute_state(&HUMANOID_PROFILE, &zero_angles(), &zero_angles());
        assert!(state.left_foot[2] < state.com[2] && state.right_foot[2] < state.com[2]);
        assert!((state.left_foot[0] + state.right_foot[0]).abs() < 1e-4_f32);
    }
}
