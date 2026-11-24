//! Golden oracle: the checked-in `rk_control::mass_table::LINKS` must match
//! the `<inertial>` blocks of the authoritative URDF `rocky_mk1.urdf`.

use rk_control::mass_table::{LINK_COUNT, LINKS, MOVING_LINK_COUNT, PELVIS_INDEX, TOTAL_MASS_KG};

const TOL: f64 = 1e-6;

fn joint_index_of(name: &str) -> Option<usize> {
    // Must mirror the joint declaration order of rocky_mk1.urdf / lib.rs:36-54.
    const ORDER: &[&str] = &[
        "torso",
        "neck",
        "head",
        "l_shoulder",
        "l_upper_arm",
        "l_forearm",
        "r_shoulder",
        "r_upper_arm",
        "r_forearm",
        "l_hip_yaw_link",
        "l_hip_roll_link",
        "l_thigh",
        "l_shin",
        "l_ankle_link",
        "l_foot",
        "r_hip_yaw_link",
        "r_hip_roll_link",
        "r_thigh",
        "r_shin",
        "r_ankle_link",
        "r_foot",
    ];
    ORDER.iter().position(|&n| n == name)
}

#[test]
fn mass_table_matches_urdf_inertial_tags() {
    let robot = urdf_rs::read_from_string(rk_model::BUNDLED_URDF_XML)
        .expect("bundled rocky_mk1.urdf must parse with urdf-rs");

    assert_eq!(robot.links.len(), LINK_COUNT);
    assert_eq!(robot.joints.len(), MOVING_LINK_COUNT);
    assert_eq!(PELVIS_INDEX, LINK_COUNT - 1);

    let mut total = 0.0_f64;
    for link in &robot.links {
        let inertial = &link.inertial;

        let idx = match joint_index_of(&link.name) {
            Some(i) => i,
            None => {
                assert_eq!(link.name, "pelvis", "unexpected root link {}", link.name);
                PELVIS_INDEX
            }
        };
        let entry = &LINKS[idx];

        assert_eq!(entry.name, link.name, "name mismatch at index {idx}");
        assert_eq!(
            entry.joint_index.map(|v| v as usize),
            joint_index_of(&link.name),
            "joint_index mismatch for {}",
            link.name
        );

        let mass = inertial.mass.value;
        total += mass;
        assert!(
            (entry.mass as f64 - mass).abs() < TOL,
            "mass mismatch for {}: table {} vs URDF {}",
            link.name,
            entry.mass,
            mass
        );

        let xyz = inertial.origin.xyz;
        for k in 0..3 {
            assert!(
                (entry.com[k] as f64 - xyz[k]).abs() < TOL,
                "com[{}] mismatch for {}: table {} vs URDF {}",
                k,
                link.name,
                entry.com[k],
                xyz[k]
            );
        }
    }

    assert!(
        (TOTAL_MASS_KG as f64 - total).abs() < TOL,
        "TOTAL_MASS_KG {} vs URDF sum {}",
        TOTAL_MASS_KG,
        total
    );
    assert!((total - 1.2) > 0.0 && total < 1.8, "total mass out of range: {total}");
    assert!((rk_control::mass_table::INV_TOTAL_MASS as f64 - 1.0 / total).abs() < 1e-6);
}

#[test]
fn mass_table_parent_indices_form_a_tree_rooted_at_pelvis() {
    for (i, entry) in LINKS.iter().enumerate() {
        let parent = entry.parent as usize;
        assert!(parent < LINK_COUNT, "bad parent index at {i}");
        assert_eq!(
            LINKS[parent].name,
            expected_parent_name(entry.name),
            "parent mismatch for {}",
            entry.name
        );
    }
    assert_eq!(LINKS[PELVIS_INDEX].parent as usize, PELVIS_INDEX);
}

fn expected_parent_name(name: &str) -> &'static str {
    match name {
        "torso" | "l_hip_yaw_link" | "r_hip_yaw_link" => "pelvis",
        "l_shoulder" | "r_shoulder" | "neck" => "torso",
        "head" => "neck",
        "l_upper_arm" => "l_shoulder",
        "l_forearm" => "l_upper_arm",
        "r_upper_arm" => "r_shoulder",
        "r_forearm" => "r_upper_arm",
        "l_hip_roll_link" => "l_hip_yaw_link",
        "l_thigh" => "l_hip_roll_link",
        "l_shin" => "l_thigh",
        "l_ankle_link" => "l_shin",
        "l_foot" => "l_ankle_link",
        "r_hip_roll_link" => "r_hip_yaw_link",
        "r_thigh" => "r_hip_roll_link",
        "r_shin" => "r_thigh",
        "r_ankle_link" => "r_shin",
        "r_foot" => "r_ankle_link",
        "pelvis" => "pelvis",
        other => panic!("unexpected link {other}"),
    }
}
