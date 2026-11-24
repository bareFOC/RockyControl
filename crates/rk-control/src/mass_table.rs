//! Per-link mass properties for Rocky Mk1, generated from
//! `assets/sample_humanoid/rocky_mk1.urdf` `<inertial>` tags.
//!
//! The URDF is the single source of truth. Do not hand-edit these constants:
//! `crates/rk-model/tests/mass_table_test.rs` is the golden oracle that
//! parses the URDF and asserts this table matches within f32 tolerance.
//!
//! # Index / parent convention
//!
//! `LINKS[0..=20]` are the 21 moving links of the Rocky Mk1, indexed by the
//! **joint index** used throughout `rk-control` (`lib.rs:36-54`). This is the
//! URDF joint declaration order and the order of the 21-element angle array:
//!
//! | idx | link             | joint            | parent idx |
//! |-----|------------------|------------------|------------|
//! | 0   | torso            | waist_yaw        | 21 (pelvis)|
//! | 1   | neck             | head_yaw         | 0 (torso)  |
//! | 2   | head             | head_pitch       | 1 (neck)   |
//! | 3   | l_shoulder       | l_shoulder_pitch | 0 (torso)  |
//! | 4   | l_upper_arm      | l_shoulder_roll  | 3          |
//! | 5   | l_forearm        | l_elbow_pitch    | 4          |
//! | 6   | r_shoulder       | r_shoulder_pitch | 0 (torso)  |
//! | 7   | r_upper_arm      | r_shoulder_roll  | 6          |
//! | 8   | r_forearm        | r_elbow_pitch    | 7          |
//! | 9   | l_hip_yaw_link   | l_hip_yaw        | 21 (pelvis)|
//! | 10  | l_hip_roll_link  | l_hip_roll       | 9          |
//! | 11  | l_thigh          | l_hip_pitch      | 10         |
//! | 12  | l_shin           | l_knee_pitch     | 11         |
//! | 13  | l_ankle_link     | l_ankle_pitch    | 12         |
//! | 14  | l_foot           | l_ankle_roll     | 13         |
//! | 15  | r_hip_yaw_link   | r_hip_yaw        | 21 (pelvis)|
//! | 16  | r_hip_roll_link  | r_hip_roll       | 15         |
//! | 17  | r_thigh          | r_hip_pitch      | 16         |
//! | 18  | r_shin           | r_knee_pitch     | 17         |
//! | 19  | r_ankle_link     | r_ankle_pitch    | 18         |
//! | 20  | r_foot           | r_ankle_roll     | 19         |
//! | 21  | pelvis           | (fixed root)     | 21 (self)  |
//!
//! `LINKS[21]` is the **pelvis**, the fixed base of FK with no joint. Its
//! `parent` is the self-sentinel `21` (meaning "no parent joint"). Its `com`
//! is expressed in the pelvis frame, which is also the root frame used by
//! the FK traversal.
//!
//! `joint_index` is `Some(i)` for moving links (i.e. `LINKS[i].joint_index ==
//! Some(i)`) and `None` for the pelvis root.

/// Total mass of all 22 links in kilograms (sum of URDF `<mass>` values).
pub const TOTAL_MASS_KG: f32 = 1.487;
/// Reciprocal of [`TOTAL_MASS_KG`], precomputed for the mass-weighted CoM sum.
pub const INV_TOTAL_MASS: f32 = 1.0 / TOTAL_MASS_KG;

/// Number of moving links (equals the 21-DoF angle array length).
pub const MOVING_LINK_COUNT: usize = 21;
/// Index of the pelvis fixed root in [`LINKS`].
pub const PELVIS_INDEX: usize = 21;
/// Total number of links in the table.
pub const LINK_COUNT: usize = 22;

/// Mass properties of one link.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LinkMass {
    /// Human-readable link name (matches the URDF `<link name="...">`).
    pub name: &'static str,
    /// Index of the parent link in [`LINKS`]; `21` (self) for the pelvis root.
    pub parent: u8,
    /// Joint index in the 21-element angle array; `None` for the pelvis root.
    pub joint_index: Option<u8>,
    /// Local COM offset in the link's own frame, from `<inertial><origin xyz>`.
    pub com: [f32; 3],
    /// Link mass in kilograms, from `<inertial><mass value>`.
    pub mass: f32,
}

/// All 22 links in fixed parent-first order (indices 0..=21).
pub const LINKS: [LinkMass; LINK_COUNT] = [
    LinkMass {
        name: "torso",
        parent: 21,
        joint_index: Some(0),
        com: [0.0, 0.16, 0.0],
        mass: 0.480,
    },
    LinkMass {
        name: "neck",
        parent: 0,
        joint_index: Some(1),
        com: [0.0, 0.03, 0.0],
        mass: 0.014,
    },
    LinkMass {
        name: "head",
        parent: 1,
        joint_index: Some(2),
        com: [0.0, 0.08, 0.01],
        mass: 0.065,
    },
    LinkMass {
        name: "l_shoulder",
        parent: 0,
        joint_index: Some(3),
        com: [0.03, 0.0, 0.0],
        mass: 0.028,
    },
    LinkMass {
        name: "l_upper_arm",
        parent: 3,
        joint_index: Some(4),
        com: [0.0, -0.12, 0.0],
        mass: 0.026,
    },
    LinkMass {
        name: "l_forearm",
        parent: 4,
        joint_index: Some(5),
        com: [0.0, -0.13, 0.0],
        mass: 0.018,
    },
    LinkMass {
        name: "r_shoulder",
        parent: 0,
        joint_index: Some(6),
        com: [-0.03, 0.0, 0.0],
        mass: 0.028,
    },
    LinkMass {
        name: "r_upper_arm",
        parent: 6,
        joint_index: Some(7),
        com: [0.0, -0.12, 0.0],
        mass: 0.026,
    },
    LinkMass {
        name: "r_forearm",
        parent: 7,
        joint_index: Some(8),
        com: [0.0, -0.13, 0.0],
        mass: 0.018,
    },
    LinkMass {
        name: "l_hip_yaw_link",
        parent: 21,
        joint_index: Some(9),
        com: [0.0, -0.015, 0.0],
        mass: 0.018,
    },
    LinkMass {
        name: "l_hip_roll_link",
        parent: 9,
        joint_index: Some(10),
        com: [0.0, -0.015, 0.0],
        mass: 0.022,
    },
    LinkMass {
        name: "l_thigh",
        parent: 10,
        joint_index: Some(11),
        com: [0.0, -0.16, 0.0],
        mass: 0.062,
    },
    LinkMass {
        name: "l_shin",
        parent: 11,
        joint_index: Some(12),
        com: [0.0, -0.16, 0.0],
        mass: 0.042,
    },
    LinkMass {
        name: "l_ankle_link",
        parent: 12,
        joint_index: Some(13),
        com: [0.0, -0.015, 0.0],
        mass: 0.016,
    },
    LinkMass {
        name: "l_foot",
        parent: 13,
        joint_index: Some(14),
        com: [0.0, -0.025, 0.04],
        mass: 0.042,
    },
    LinkMass {
        name: "r_hip_yaw_link",
        parent: 21,
        joint_index: Some(15),
        com: [0.0, -0.015, 0.0],
        mass: 0.018,
    },
    LinkMass {
        name: "r_hip_roll_link",
        parent: 15,
        joint_index: Some(16),
        com: [0.0, -0.015, 0.0],
        mass: 0.022,
    },
    LinkMass {
        name: "r_thigh",
        parent: 16,
        joint_index: Some(17),
        com: [0.0, -0.16, 0.0],
        mass: 0.062,
    },
    LinkMass {
        name: "r_shin",
        parent: 17,
        joint_index: Some(18),
        com: [0.0, -0.16, 0.0],
        mass: 0.042,
    },
    LinkMass {
        name: "r_ankle_link",
        parent: 18,
        joint_index: Some(19),
        com: [0.0, -0.015, 0.0],
        mass: 0.016,
    },
    LinkMass {
        name: "r_foot",
        parent: 19,
        joint_index: Some(20),
        com: [0.0, -0.025, 0.04],
        mass: 0.042,
    },
    LinkMass {
        name: "pelvis",
        parent: 21,
        joint_index: None,
        com: [0.0, 0.0, 0.0],
        mass: 0.380,
    },
];
