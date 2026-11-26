//! Golden oracle: generated `rk_robot_profile::generated::*` tables and
//! `rk_control::com_fk` must match the authoritative URDFs under `assets/robots/`.

use std::collections::HashMap;

use rk_control::com_fk::compute_state;
use rk_proto::MAX_JOINTS;
use rk_robot_profile::generated::{humanoid, striker_biped};

const HUMANOID_URDF: &str = include_str!("../../../assets/robots/humanoid.urdf");
const TOL: f32 = 1e-5;

#[test]
fn humanoid_generated_profile_matches_urdf_inertial_and_kinematics() {
    let robot = urdf_rs::read_from_string(HUMANOID_URDF).expect("humanoid.urdf must parse");
    let prof = &humanoid::PROFILE;

    let link_map: HashMap<&str, &urdf_rs::Link> =
        robot.links.iter().map(|l| (l.name.as_str(), l)).collect();
    let joint_by_child: HashMap<&str, &urdf_rs::Joint> = robot
        .joints
        .iter()
        .map(|j| (j.child.link.as_str(), j))
        .collect();

    assert_eq!(prof.links.len(), robot.links.len());
    assert_eq!(prof.joint_kin.len(), robot.links.len());

    let mut total_mass = 0.0_f64;
    for (i, lm) in prof.links.iter().enumerate() {
        let urdf_link = link_map.get(lm.name).expect("link in URDF");
        let m = urdf_link.inertial.mass.value as f32;
        total_mass += urdf_link.inertial.mass.value;
        assert!(
            (lm.mass - m).abs() < TOL,
            "link {} mass mismatch: {} vs {}",
            lm.name,
            lm.mass,
            m
        );
        for k in 0..3 {
            let c = urdf_link.inertial.origin.xyz[k] as f32;
            assert!(
                (lm.com[k] - c).abs() < TOL,
                "link {} com[{k}] mismatch: {} vs {}",
                lm.name,
                lm.com[k],
                c
            );
        }

        if i != prof.pelvis_index {
            let j = joint_by_child.get(lm.name).expect("non-root link has parent joint");
            let kin = prof.joint_kin[i];
            for k in 0..3 {
                assert!((kin.origin[k] - j.origin.xyz[k] as f32).abs() < TOL);
                assert!((kin.axis[k] - j.axis.xyz[k] as f32).abs() < TOL);
            }
        }
    }

    assert!((prof.total_mass_kg as f64 - total_mass).abs() < 1e-5);
    assert!((prof.inv_total_mass as f64 - 1.0 / total_mass).abs() < 1e-5);
    assert_eq!(prof.joint_count, 28);
    assert_eq!(prof.urdf_joint_count, 32);
}

#[test]
fn humanoid_com_fk_coupled_double_joints_preserve_symmetry() {
    let prof = &humanoid::PROFILE;
    let mut angles = [0.0_f32; MAX_JOINTS];
    // Zero pose must be laterally symmetric (Y ~ 0 in REP-103).
    let zero_fk = compute_state(prof, &angles, &[0.0_f32; MAX_JOINTS]);
    assert!(
        zero_fk.com[1].abs() < 1e-4,
        "zero-pose humanoid CoM Y must be symmetric, got {}",
        zero_fk.com[1]
    );
    assert!(zero_fk.left_toe.is_some() && zero_fk.right_toe.is_some());

    // Bend both knees symmetrically by 0.8 rad anatomical bend.
    let lk = prof.joint_indices.l_knee_pitch.unwrap() as usize;
    let rk = prof.joint_indices.r_knee_pitch.unwrap() as usize;
    assert_eq!(prof.motor_to_joint_ratio[lk], 0.5);
    assert_eq!(prof.motor_to_joint_ratio[rk], 0.5);
    angles[lk] = 0.8;
    angles[rk] = 0.8;
    let bent_fk = compute_state(prof, &angles, &[0.0_f32; MAX_JOINTS]);
    assert!(
        bent_fk.left_foot[2] > zero_fk.left_foot[2],
        "bending knee must raise foot Z toward pelvis"
    );
}

#[test]
fn striker_biped_profile_has_8_joints_and_valid_tree() {
    let prof = &striker_biped::PROFILE;
    assert_eq!(prof.joint_count, 8);
    assert_eq!(prof.urdf_joint_count, 8);
    for (i, lm) in prof.links.iter().enumerate() {
        assert!((lm.parent as usize) < prof.links.len());
        if i != prof.pelvis_index {
            assert!((lm.parent as usize) < i, "must be parent-first");
        }
    }
}
