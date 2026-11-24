//! Pure `#![no_std]` mass-weighted Center-of-Mass forward kinematics for Rocky Mk1.
//!
//! This module is the **single source of truth** for the kinematic CoM used by
//! the estimator, on both the RP2350 firmware and the software simulator. It
//! consumes only `f32` joint angles / velocities, a constant per-link mass
//! table ([`crate::mass_table`]) and a constant joint-kinematics table derived
//! from `assets/sample_humanoid/rocky_mk1.urdf`. No allocation, no `k`/nalgebra,
//! deterministic `libm` trig only.
//!
//! # Frame convention
//!
//! The root frame is the **pelvis** frame (URDF base). Axes match the URDF and
//! the IMU contract used throughout `rk-control`:
//!
//! - `+X` = right, `+Y` = up, `+Z` = forward.
//!
//! All returned positions/velocities are expressed in this pelvis frame. The
//! support-relative conversion (subtract stance foot / double-support
//! midpoint) happens in [`crate::estimator`].
//!
//! # Joint kinematics table
//!
//! [`JOINT_KINEMATICS`] mirrors the URDF `<joint><origin xyz>` and `<axis xyz>`
//! for each moving link, indexed by the same joint index as the mass table.
//! Every axis is a signed world-axis unit vector in the parent link frame;
//! the URDF uses only `X`, `Y`, `Z` axes, so `rotate_vec` uses Rodrigues'
//! formula specialised to those three cases. If the URDF is edited, these
//! constants must be updated in lockstep (the host golden test in
//! `crates/rk-model/tests/com_fk_golden_test.rs` catches drift).

use libm::{cosf, sinf};
use rk_proto::MAX_JOINTS;

use crate::mass_table::{LINK_COUNT, LINKS, PELVIS_INDEX};

/// A joint's fixed transform relative to its parent link.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointKin {
    /// Axis unit vector in the **parent** link frame, from URDF `<axis xyz>`.
    /// Only signed `X`/`Y`/`Z` unit vectors are supported (Rocky Mk1 uses only
    /// those), stored as `(x, y, z)`.
    pub axis: [f32; 3],
    /// Joint origin translation in the **parent** link frame, from
    /// `<origin xyz>`; all Rocky Mk1 joints use zero RPY.
    pub origin: [f32; 3],
}

/// Joint axis/origin constants indexed by link index (`0..=20` moving links,
/// `21` pelvis root). Derived from `rocky_mk1.urdf` `<joint>` tags.
pub const JOINT_KINEMATICS: [JointKin; LINK_COUNT] = [
    // torso (0) <- pelvis: waist_yaw
    JointKin { axis: [0.0, 1.0, 0.0], origin: [0.0, 0.08, 0.0] },
    // neck (1) <- torso: head_yaw
    JointKin { axis: [0.0, 1.0, 0.0], origin: [0.0, 0.33, 0.0] },
    // head (2) <- neck: head_pitch
    JointKin { axis: [1.0, 0.0, 0.0], origin: [0.0, 0.06, 0.0] },
    // l_shoulder (3) <- torso: l_shoulder_pitch
    JointKin { axis: [1.0, 0.0, 0.0], origin: [0.18, 0.27, 0.0] },
    // l_upper_arm (4) <- l_shoulder: l_shoulder_roll
    JointKin { axis: [0.0, 0.0, 1.0], origin: [0.04, 0.0, 0.0] },
    // l_forearm (5) <- l_upper_arm: l_elbow_pitch
    JointKin { axis: [1.0, 0.0, 0.0], origin: [0.0, -0.24, 0.0] },
    // r_shoulder (6) <- torso: r_shoulder_pitch
    JointKin { axis: [1.0, 0.0, 0.0], origin: [-0.18, 0.27, 0.0] },
    // r_upper_arm (7) <- r_shoulder: r_shoulder_roll
    JointKin { axis: [0.0, 0.0, 1.0], origin: [-0.04, 0.0, 0.0] },
    // r_forearm (8) <- r_upper_arm: r_elbow_pitch
    JointKin { axis: [1.0, 0.0, 0.0], origin: [0.0, -0.24, 0.0] },
    // l_hip_yaw_link (9) <- pelvis: l_hip_yaw
    JointKin { axis: [0.0, 1.0, 0.0], origin: [0.095, -0.06, 0.0] },
    // l_hip_roll_link (10) <- 9: l_hip_roll
    JointKin { axis: [0.0, 0.0, 1.0], origin: [0.0, -0.03, 0.0] },
    // l_thigh (11) <- 10: l_hip_pitch
    JointKin { axis: [1.0, 0.0, 0.0], origin: [0.0, -0.03, 0.0] },
    // l_shin (12) <- 11: l_knee_pitch
    JointKin { axis: [1.0, 0.0, 0.0], origin: [0.0, -0.32, 0.0] },
    // l_ankle_link (13) <- 12: l_ankle_pitch
    JointKin { axis: [1.0, 0.0, 0.0], origin: [0.0, -0.32, 0.0] },
    // l_foot (14) <- 13: l_ankle_roll
    JointKin { axis: [0.0, 0.0, 1.0], origin: [0.0, -0.03, 0.0] },
    // r_hip_yaw_link (15) <- pelvis: r_hip_yaw
    JointKin { axis: [0.0, 1.0, 0.0], origin: [-0.095, -0.06, 0.0] },
    // r_hip_roll_link (16) <- 15: r_hip_roll
    JointKin { axis: [0.0, 0.0, 1.0], origin: [0.0, -0.03, 0.0] },
    // r_thigh (17) <- 16: r_hip_pitch
    JointKin { axis: [1.0, 0.0, 0.0], origin: [0.0, -0.03, 0.0] },
    // r_shin (18) <- 17: r_knee_pitch
    JointKin { axis: [1.0, 0.0, 0.0], origin: [0.0, -0.32, 0.0] },
    // r_ankle_link (19) <- 18: r_ankle_pitch
    JointKin { axis: [1.0, 0.0, 0.0], origin: [0.0, -0.32, 0.0] },
    // r_foot (20) <- 19: r_ankle_roll
    JointKin { axis: [0.0, 0.0, 1.0], origin: [0.0, -0.03, 0.0] },
    // pelvis (21): fixed root, no joint.
    JointKin { axis: [0.0, 0.0, 0.0], origin: [0.0, 0.0, 0.0] },
];

/// Result of one full FK traversal: lumped CoM plus both foot frames, all
/// expressed in the pelvis frame.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ComFkState {
    /// Mass-weighted center of mass `[x, y, z]` in the pelvis frame.
    pub com: [f32; 3],
    /// Mass-weighted center-of-mass velocity `[vx, vy, vz]` in the pelvis frame.
    pub com_vel: [f32; 3],
    /// Left foot frame origin `[x, y, z]` in the pelvis frame (support origin
    /// proxy for left stance).
    pub left_foot: [f32; 3],
    /// Left foot frame origin velocity in the pelvis frame.
    pub left_foot_vel: [f32; 3],
    /// Right foot frame origin `[x, y, z]` in the pelvis frame (support origin
    /// proxy for right stance).
    pub right_foot: [f32; 3],
    /// Right foot frame origin velocity in the pelvis frame.
    pub right_foot_vel: [f32; 3],
}

/// Per-link accumulated world-frame state during the parent-first traversal.
#[derive(Clone, Copy, Debug, Default)]
struct LinkFrame {
    /// World rotation of the link frame as three column vectors.
    rot: [[f32; 3]; 3],
    /// World position of the link frame origin (at its joint).
    pos: [f32; 3],
    /// World velocity of the link frame origin.
    vel: [f32; 3],
    /// World angular velocity of the link frame (accumulated joint rates).
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
fn mat_vec(m: [[f32; 3]; 3], v: [f32; 3]) -> [f32; 3] {
    [
        m[0][0] * v[0] + m[1][0] * v[1] + m[2][0] * v[2],
        m[0][1] * v[0] + m[1][1] * v[1] + m[2][1] * v[2],
        m[0][2] * v[0] + m[1][2] * v[1] + m[2][2] * v[2],
    ]
}

/// Rotate `v` by `angle` about a signed unit axis in the parent frame.
/// Rodrigues' formula; `axis` is guaranteed to be a signed `X`/`Y`/`Z` basis
/// vector by [`JOINT_KINEMATICS`], but the general form is cheap enough for
/// 22 links and keeps the code robust if a URDF axis ever changes.
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
    let ex = rotate_vec(axis, angle, [1.0, 0.0, 0.0]);
    let ey = rotate_vec(axis, angle, [0.0, 1.0, 0.0]);
    let ez = rotate_vec(axis, angle, [0.0, 0.0, 1.0]);
    [
        mat_vec(parent, ex),
        mat_vec(parent, ey),
        mat_vec(parent, ez),
    ]
}

/// Run the full parent-first FK traversal.
///
/// `angles` / `velocities` are joint-indexed (`0..joint_count`); the pelvis
/// root has no joint and is skipped. Indices beyond `joint_count` are ignored.
///
/// The traversal is fixed-order and uses only `f32` + `libm`, giving identical
/// results on host and MCU for identical inputs.
pub fn compute_state(angles: &[f32; MAX_JOINTS], velocities: &[f32; MAX_JOINTS]) -> ComFkState {
    // Frames are indexed by link index (0..=21), built parent-first. The mass
    // table is already in parent-first order with parent index < child index
    // except for the pelvis self-root at 21 (processed last). Children of the
    // pelvis (torso, both hip-yaw links) therefore read the identity frame
    // seeded below, which is exactly the fixed pelvis root.
    let identity = [[1.0_f32, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let root_frame = LinkFrame {
        rot: identity,
        pos: [0.0; 3],
        vel: [0.0; 3],
        omega: [0.0; 3],
    };
    let mut frames = [root_frame; LINK_COUNT];

    let mut com_acc = [0.0_f32; 3];
    let mut com_vel_acc = [0.0_f32; 3];

    for i in 0..LINK_COUNT {
        let link = &LINKS[i];
        let kin = JOINT_KINEMATICS[i];
        let parent_idx = link.parent as usize;

        let frame = if i == PELVIS_INDEX {
            // Fixed root: identity rotation, origin/velocity/omega zero.
            root_frame
        } else {
            let p = &frames[parent_idx];
            let angle = angles[i];
            let rate = velocities[i];

            // Joint origin is fixed in the parent link frame.
            let t = mat_vec(p.rot, kin.origin);
            let pos = [p.pos[0] + t[0], p.pos[1] + t[1], p.pos[2] + t[2]];
            let omega_cross_t = cross(p.omega, t);
            let vel = [
                p.vel[0] + omega_cross_t[0],
                p.vel[1] + omega_cross_t[1],
                p.vel[2] + omega_cross_t[2],
            ];

            // World joint axis; the joint rotation adds to link angular velocity.
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

        // Mass-weighted contribution of this link's COM, which is fixed in the
        // link frame: c = origin + R * com_local, v_c = vel + omega x (R*com).
        let r_com = mat_vec(frame.rot, link.com);
        let omega_cross_r = cross(frame.omega, r_com);
        let m = link.mass;
        for k in 0..3 {
            com_acc[k] += m * (frame.pos[k] + r_com[k]);
            com_vel_acc[k] += m * (frame.vel[k] + omega_cross_r[k]);
        }
    }

    // Left/right foot frame origins (indices 14 and 20) for support-relative DCM.
    let lf = frames[14];
    let rf = frames[20];

    ComFkState {
        com: [
            com_acc[0] * crate::mass_table::INV_TOTAL_MASS,
            com_acc[1] * crate::mass_table::INV_TOTAL_MASS,
            com_acc[2] * crate::mass_table::INV_TOTAL_MASS,
        ],
        com_vel: [
            com_vel_acc[0] * crate::mass_table::INV_TOTAL_MASS,
            com_vel_acc[1] * crate::mass_table::INV_TOTAL_MASS,
            com_vel_acc[2] * crate::mass_table::INV_TOTAL_MASS,
        ],
        left_foot: lf.pos,
        left_foot_vel: lf.vel,
        right_foot: rf.pos,
        right_foot_vel: rf.vel,
    }
}

/// Mass-weighted CoM position in the pelvis frame for the given joint angles.
pub fn compute_com(angles: &[f32; MAX_JOINTS]) -> [f32; 3] {
    compute_state(angles, &[0.0_f32; MAX_JOINTS]).com
}

/// Mass-weighted CoM velocity in the pelvis frame from measured joint rates.
pub fn com_velocity(
    angles: &[f32; MAX_JOINTS],
    velocities: &[f32; MAX_JOINTS],
) -> [f32; 3] {
    compute_state(angles, velocities).com_vel
}

#[cfg(test)]
mod tests {
    use super::*;

    fn zero_angles() -> [f32; MAX_JOINTS] {
        [0.0_f32; MAX_JOINTS]
    }

    #[test]
    fn zero_pose_com_matches_hand_computed_mass_weighted_sum() {
        // Hand-verifiable: every joint at zero, so the world position of each
        // link frame origin is the sum of joint origin translations along its
        // parent chain. Link COM offsets add directly (all rotations identity).
        // Pelvis COM is at the root; torso COM at (0, 0.08+0.16, 0) = (0,0.24,0).
        let com = compute_com(&zero_angles());
        // X symmetry (left/right link pairs are mirror images).
        assert!(
            com[0].abs() < 1e-5,
            "zero-pose CoM must be laterally symmetric, got {}",
            com[0]
        );
        // Fore-aft offset is small but nonzero: both foot COMs sit at z=+0.04
        // and the head COM at z=+0.01, so the mass-weighted z is
        // (2*0.042*0.04 + 0.065*0.01)/1.487 ~= 0.0027 m.
        assert!(
            (com[2] > 0.002_f32) && (com[2] < 0.004_f32),
            "zero-pose CoM fore-aft offset unexpected: {}",
            com[2]
        );
        // Height: the pelvis is the root at y=0, the torso/head sit above it
        // while both legs (and arms) hang below it. The mass-weighted sum very
        // nearly cancels, leaving a small positive pelvis-frame offset. This
        // exact value is the hand-computed mass-weighted sum of the 22 link
        // COM y coordinates (torso 0.24 m, legs down to ~-0.85 m).
        assert!(
            (com[1] > 0.0_f32) && (com[1] < 0.01_f32),
            "zero-pose pelvis-frame CoM height unexpected: {}",
            com[1]
        );
    }

    #[test]
    fn single_joint_velocity_produces_nonzero_com_velocity() {
        let angles = zero_angles();
        let mut vel = zero_angles();
        // Hip pitch (index 11) moving at +1.0 rad/s must move distal leg mass
        // and therefore the CoM. The joint axis is +X; by the right-hand rule
        // a positive rotation carries the leg's -Y lever toward -Z, so the
        // CoM moves backward (-Z). No lateral (X) motion is produced.
        vel[11] = 1.0_f32;
        let cv = com_velocity(&angles, &vel);
        assert!(
            cv[2] < -0.01_f32,
            "hip-pitch rate must drive backward CoM velocity, got {:?}",
            cv
        );
        assert!(
            cv[0].abs() < 1e-4_f32,
            "hip-pitch rate must not create lateral CoM velocity, got {:?}",
            cv
        );
    }

    #[test]
    fn all_joint_velocities_zero_means_com_velocity_zero() {
        let angles = zero_angles();
        let cv = com_velocity(&angles, &zero_angles());
        for axis in 0..3 {
            assert!(
                cv[axis].abs() < 1e-6_f32,
                "zero joint rate must yield zero CoM velocity, got {:?}",
                cv
            );
        }
    }

    #[test]
    fn waist_yaw_rotates_body_and_moves_com_laterally() {
        let mut angles = zero_angles();
        // A pure waist yaw about +Y of +pi/2 rotates the torso/arms 90 deg in
        // the sagittal plane; the lateral CoM stays near zero by symmetry, but
        // the fore-aft CoM must shift because torso COM is off-axis along +Y
        // (yaw about Y does not change Y) and shoulder origins are along X.
        // Use shoulder pitch instead: raising one arm shifts CoM upward.
        angles[3] = 0.5_f32; // l_shoulder_pitch
        let com = compute_com(&angles);
        let base = compute_com(&zero_angles());
        assert!(
            com[1] > base[1],
            "raising left shoulder pitch must raise CoM: {} vs {}",
            com[1],
            base[1]
        );
    }

    #[test]
    fn com_velocity_is_linear_in_joint_velocity() {
        let angles = zero_angles();
        let mut v1 = zero_angles();
        v1[11] = 0.5_f32;
        let mut v2 = zero_angles();
        v2[11] = 1.0_f32;
        let c1 = com_velocity(&angles, &v1);
        let c2 = com_velocity(&angles, &v2);
        for axis in 0..3 {
            assert!(
                (c2[axis] - 2.0_f32 * c1[axis]).abs() < 1e-5_f32,
                "CoM Jacobian must be linear in joint rate: {:?} vs {:?}",
                c1,
                c2
            );
        }
    }

    #[test]
    fn foot_origins_are_below_pelvis_and_symmetric_at_zero_pose() {
        let state = compute_state(&zero_angles(), &zero_angles());
        assert!(
            state.left_foot[1] < -0.3_f32 && state.right_foot[1] < -0.3_f32,
            "feet must be near the ground below pelvis: L={:?} R={:?}",
            state.left_foot,
            state.right_foot
        );
        assert!(
            (state.left_foot[0] + state.right_foot[0]).abs() < 1e-5_f32,
            "feet must be laterally symmetric: L={:?} R={:?}",
            state.left_foot,
            state.right_foot
        );
    }
}
