//! Layer 3 — GUI Domain Engine (`humanoid-model`).
//!
//! Owns:
//! - In-memory `AssetBundle` (`HashMap<String, Vec<u8>>`) with bundled `assets/sample_humanoid/rocky_mk1.urdf`
//! - `urdf-rs` parsing + `k::Chain<f32>` forward kinematics evaluation
//! - Bidirectional mirroring authority state machine (`Teach`, `Drive`, `EStop`) with mandatory **bumpless transfer**
//! - Teach & Action Studio combat move library + C1 cubic Hermite keyframe interpolator
//! - Calibration persistence (`serde_json`) and `.zip` URDF bundle import

use std::collections::HashMap;
use std::io::Read;

use humanoid_control::JointCalibration;
use humanoid_proto::{ConfigCmd, ControlMode, FaultFlags, HostToMcu, MAX_JOINTS, McuToHost};
pub use k::nalgebra;
use k::nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};

pub const BUNDLED_URDF_XML: &str = include_str!("../../../assets/sample_humanoid/rocky_mk1.urdf");

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LimbGroup {
    HeadTorso,
    LeftArm,
    RightArm,
    LeftLeg,
    RightLeg,
}

impl LimbGroup {
    pub fn label(self) -> &'static str {
        match self {
            Self::HeadTorso => "Head & Torso",
            Self::LeftArm => "Left Arm",
            Self::RightArm => "Right Arm",
            Self::LeftLeg => "Left Leg",
            Self::RightLeg => "Right Leg",
        }
    }

    pub const ALL: [Self; 5] = [
        Self::HeadTorso,
        Self::LeftArm,
        Self::RightArm,
        Self::LeftLeg,
        Self::RightLeg,
    ];
}

#[derive(Clone, Debug)]
pub struct JointMeta {
    pub index: usize,
    pub name: String,
    pub label: String,
    pub group: LimbGroup,
    pub child_link: String,
    pub default_min_rad: f32,
    pub default_max_rad: f32,
    pub default_rad: f32,
}

#[derive(Clone, Debug)]
pub enum VisualShape {
    Box([f32; 3]),
    Cylinder { radius: f32, length: f32 },
    Sphere { radius: f32 },
    Mesh { filename: String, scale: [f32; 3] },
}

#[derive(Clone, Debug)]
pub struct LinkVisual {
    pub link_name: String,
    pub joint_index: Option<usize>,
    pub local_offset: Isometry3<f32>,
    pub shape: VisualShape,
    pub rgba: [f32; 4],
}

/// In-memory URDF + mesh asset bundle (zero direct filesystem dependency for WASM compatibility).
#[derive(Clone, Debug, Default)]
pub struct AssetBundle {
    pub robot_name: String,
    pub urdf_xml: String,
    pub files: HashMap<String, Vec<u8>>,
}

impl AssetBundle {
    pub fn bundled_rocky() -> Self {
        Self {
            robot_name: "rocky_mk1".to_string(),
            urdf_xml: BUNDLED_URDF_XML.to_string(),
            files: HashMap::new(),
        }
    }

    /// Loads either a raw `.urdf`/`.xml` file or a `.zip` bundle containing a URDF + STL/OBJ meshes.
    pub fn from_bytes(filename: &str, bytes: &[u8]) -> Result<Self, String> {
        if filename.to_ascii_lowercase().ends_with(".zip") {
            let cursor = std::io::Cursor::new(bytes);
            let mut archive =
                zip::ZipArchive::new(cursor).map_err(|e| format!("Invalid zip archive: {e}"))?;
            let mut files = HashMap::new();
            let mut urdf_xml = None;
            for i in 0..archive.len() {
                let mut entry = archive
                    .by_index(i)
                    .map_err(|e| format!("Zip read error: {e}"))?;
                if entry.is_dir() {
                    continue;
                }
                let name = entry.name().to_string();
                let mut data = Vec::new();
                entry
                    .read_to_end(&mut data)
                    .map_err(|e| format!("Zip entry read error: {e}"))?;
                if name.to_ascii_lowercase().ends_with(".urdf")
                    || name.to_ascii_lowercase().ends_with(".xml")
                {
                    if let Ok(s) = String::from_utf8(data.clone()) {
                        if s.contains("<robot") {
                            urdf_xml = Some(s);
                        }
                    }
                }
                files.insert(name, data);
            }
            let urdf_xml =
                urdf_xml.ok_or_else(|| "Zip bundle does not contain a .urdf file".to_string())?;
            let parsed = urdf_rs::read_from_string(&urdf_xml)
                .map_err(|e| format!("URDF parse failed: {e}"))?;
            Ok(Self {
                robot_name: parsed.name,
                urdf_xml,
                files,
            })
        } else {
            let urdf_xml = String::from_utf8(bytes.to_vec())
                .map_err(|_| "URDF file is not valid UTF-8".to_string())?;
            let parsed = urdf_rs::read_from_string(&urdf_xml)
                .map_err(|e| format!("URDF parse failed: {e}"))?;
            Ok(Self {
                robot_name: parsed.name,
                urdf_xml,
                files: HashMap::new(),
            })
        }
    }
}

/// Forward kinematics evaluator backed by `urdf-rs` + `k::Chain<f32>`.
pub struct KinematicRig {
    pub robot_name: String,
    pub chain: k::Chain<f32>,
    pub joints: Vec<JointMeta>,
    pub visuals: Vec<LinkVisual>,
    /// Root height above floor so Rocky stands cleanly grounded on `y = 0`.
    pub root_y_offset: f32,
}

impl KinematicRig {
    pub fn from_urdf_xml(xml: &str) -> Result<Self, String> {
        let robot =
            urdf_rs::read_from_string(xml).map_err(|e| format!("Failed to parse URDF XML: {e}"))?;

        // Build k::Chain<f32> via explicit k::NodeBuilder tree from urdf_rs::Robot
        let mut child_to_joint: HashMap<String, &urdf_rs::Joint> = HashMap::new();
        let mut parent_to_children: HashMap<String, Vec<&urdf_rs::Joint>> = HashMap::new();
        for j in &robot.joints {
            child_to_joint.insert(j.child.link.clone(), j);
            parent_to_children
                .entry(j.parent.link.clone())
                .or_default()
                .push(j);
        }

        let root_link_name = robot
            .links
            .iter()
            .find(|l| !child_to_joint.contains_key(&l.name))
            .map(|l| l.name.clone())
            .or_else(|| robot.links.first().map(|l| l.name.clone()))
            .ok_or_else(|| "URDF contains no links".to_string())?;

        let root_node = k::Node::new(
            k::NodeBuilder::<f32>::new()
                .name(&root_link_name)
                .joint_type(k::JointType::Fixed)
                .finalize(),
        );

        let mut joint_metas: Vec<JointMeta> = Vec::new();
        let mut link_to_joint_idx: HashMap<String, usize> = HashMap::new();

        fn attach_children(
            parent_link: &str,
            parent_node: &k::Node<f32>,
            parent_to_children: &HashMap<String, Vec<&urdf_rs::Joint>>,
            joint_metas: &mut Vec<JointMeta>,
            link_to_joint_idx: &mut HashMap<String, usize>,
        ) {
            let Some(children) = parent_to_children.get(parent_link) else {
                return;
            };
            for j in children {
                let xyz = j.origin.xyz;
                let rpy = j.origin.rpy;
                let rot = UnitQuaternion::from_euler_angles(
                    rpy[0] as f32,
                    rpy[1] as f32,
                    rpy[2] as f32,
                );
                let trans = Translation3::new(xyz[0] as f32, xyz[1] as f32, xyz[2] as f32);

                let axis_vec =
                    Vector3::new(j.axis.xyz[0] as f32, j.axis.xyz[1] as f32, j.axis.xyz[2] as f32);
                let unit_axis = nalgebra::Unit::try_new(axis_vec, 1e-6)
                    .unwrap_or_else(|| nalgebra::Unit::new_normalize(Vector3::x()));

                let is_movable = matches!(
                    j.joint_type,
                    urdf_rs::JointType::Revolute | urdf_rs::JointType::Continuous
                ) && joint_metas.len() < MAX_JOINTS;

                let mut builder = k::NodeBuilder::<f32>::new()
                    .name(&j.child.link)
                    .translation(trans)
                    .rotation(rot);

                if is_movable {
                    let idx = joint_metas.len();
                    let min_r = (j.limit.lower as f32).clamp(-3.14, 3.14);
                    let max_r = (j.limit.upper as f32).clamp(-3.14, 3.14);
                    let (lo, hi) = if min_r < max_r {
                        (min_r, max_r)
                    } else {
                        (-1.57, 1.57)
                    };
                    builder = builder
                        .joint_type(k::JointType::Rotational { axis: unit_axis })
                        .limits(Some(k::joint::Range::new(lo, hi)));

                    let default_rad = default_stance_angle(&j.name).clamp(lo, hi);
                    joint_metas.push(JointMeta {
                        index: idx,
                        name: j.name.clone(),
                        label: prettify_joint_name(&j.name),
                        group: classify_limb_group(&j.name),
                        child_link: j.child.link.clone(),
                        default_min_rad: lo,
                        default_max_rad: hi,
                        default_rad,
                    });
                    link_to_joint_idx.insert(j.child.link.clone(), idx);
                } else {
                    builder = builder.joint_type(k::JointType::Fixed);
                }

                let child_node = k::Node::new(builder.finalize());
                child_node.set_parent(parent_node);

                attach_children(
                    &j.child.link,
                    &child_node,
                    parent_to_children,
                    joint_metas,
                    link_to_joint_idx,
                );
            }
        }

        attach_children(
            &root_link_name,
            &root_node,
            &parent_to_children,
            &mut joint_metas,
            &mut link_to_joint_idx,
        );

        let chain = k::Chain::from_root(root_node);

        // Extract link visual geometries & RGBA materials
        let mut visuals = Vec::new();
        for link in &robot.links {
            let joint_index = link_to_joint_idx.get(&link.name).copied();
            if link.visual.is_empty() {
                visuals.push(LinkVisual {
                    link_name: link.name.clone(),
                    joint_index,
                    local_offset: Isometry3::identity(),
                    shape: VisualShape::Box([0.08, 0.08, 0.08]),
                    rgba: [0.32, 0.37, 0.44, 1.0],
                });
                continue;
            }
            for vis in &link.visual {
                let xyz = vis.origin.xyz;
                let rpy = vis.origin.rpy;
                let local_offset = Isometry3::from_parts(
                    Translation3::new(xyz[0] as f32, xyz[1] as f32, xyz[2] as f32),
                    UnitQuaternion::from_euler_angles(
                        rpy[0] as f32,
                        rpy[1] as f32,
                        rpy[2] as f32,
                    ),
                );
                let shape = match &vis.geometry {
                    urdf_rs::Geometry::Box { size } => {
                        VisualShape::Box([size[0] as f32, size[1] as f32, size[2] as f32])
                    }
                    urdf_rs::Geometry::Cylinder { radius, length } => VisualShape::Cylinder {
                        radius: *radius as f32,
                        length: *length as f32,
                    },
                    urdf_rs::Geometry::Sphere { radius } => VisualShape::Sphere {
                        radius: *radius as f32,
                    },
                    urdf_rs::Geometry::Mesh { filename, scale } => {
                        let s = scale
                            .map(|v| [v[0] as f32, v[1] as f32, v[2] as f32])
                            .unwrap_or([1.0, 1.0, 1.0]);
                        VisualShape::Mesh {
                            filename: filename.clone(),
                            scale: s,
                        }
                    }
                    _ => VisualShape::Box([0.08, 0.08, 0.08]),
                };

                let rgba = vis
                    .material
                    .as_ref()
                    .and_then(|m| m.color.as_ref())
                    .map(|c| [c.rgba[0] as f32, c.rgba[1] as f32, c.rgba[2] as f32, c.rgba[3] as f32])
                    .unwrap_or([0.32, 0.37, 0.44, 1.0]);

                visuals.push(LinkVisual {
                    link_name: link.name.clone(),
                    joint_index,
                    local_offset,
                    shape,
                    rgba,
                });
            }
        }

        Ok(Self {
            robot_name: robot.name,
            chain,
            joints: joint_metas,
            visuals,
            root_y_offset: 0.78_f32,
        })
    }

    /// Evaluates full 3D world poses for every link in the URDF using `k::Chain<f32>`.
    pub fn compute_link_transforms(&self, angles_rad: &[f32; MAX_JOINTS]) -> HashMap<String, Isometry3<f32>> {
        let dof = self.chain.dof();
        let positions: Vec<f32> = (0..dof)
            .map(|i| angles_rad.get(i).copied().unwrap_or(0.0_f32))
            .collect();
        let _ = self.chain.set_joint_positions_clamped(&positions);
        self.chain.update_transforms();

        let root_shift = Translation3::new(0.0_f32, self.root_y_offset, 0.0_f32);
        let mut map = HashMap::with_capacity(self.visuals.len() + 2);
        for node in self.chain.iter() {
            let maybe_link_name = node.link().as_ref().map(|l| l.name.clone());
            let link_name = maybe_link_name.unwrap_or_else(|| node.joint().name.clone());
            let iso = node.world_transform().unwrap_or_else(Isometry3::identity);
            map.insert(
                link_name,
                Isometry3::from_parts(root_shift * iso.translation, iso.rotation),
            );
        }
        map
    }

    pub fn default_calibration(&self) -> Vec<JointCalibration> {
        self.joints
            .iter()
            .map(|j| JointCalibration {
                zero_offset_rad: 0.0,
                inverted: false,
                min_rad: j.default_min_rad,
                max_rad: j.default_max_rad,
            })
            .collect()
    }

    pub fn default_pose(&self) -> [f32; MAX_JOINTS] {
        let mut pose = [0.0_f32; MAX_JOINTS];
        for j in &self.joints {
            pose[j.index] = j.default_rad;
        }
        pose
    }
}

fn classify_limb_group(name: &str) -> LimbGroup {
    let lower = name.to_ascii_lowercase();
    if lower.starts_with("l_") && (lower.contains("shoulder") || lower.contains("elbow") || lower.contains("arm") || lower.contains("wrist")) {
        LimbGroup::LeftArm
    } else if lower.starts_with("r_") && (lower.contains("shoulder") || lower.contains("elbow") || lower.contains("arm") || lower.contains("wrist")) {
        LimbGroup::RightArm
    } else if lower.starts_with("l_") && (lower.contains("hip") || lower.contains("knee") || lower.contains("ankle") || lower.contains("leg")) {
        LimbGroup::LeftLeg
    } else if lower.starts_with("r_") && (lower.contains("hip") || lower.contains("knee") || lower.contains("ankle") || lower.contains("leg")) {
        LimbGroup::RightLeg
    } else {
        LimbGroup::HeadTorso
    }
}

fn prettify_joint_name(name: &str) -> String {
    name.split('_')
        .map(|part| match part {
            "l" => "L".to_string(),
            "r" => "R".to_string(),
            other => {
                let mut chars = other.chars();
                match chars.next() {
                    Some(c) => format!("{}{}", c.to_ascii_uppercase(), chars.as_str()),
                    None => String::new(),
                }
            }
        })
        .collect::<Vec<_>>()
        .join(" ")
}

fn default_stance_angle(name: &str) -> f32 {
    match name {
        "l_shoulder_pitch" | "r_shoulder_pitch" => -0.22,
        "l_shoulder_roll" => 0.14,
        "r_shoulder_roll" => -0.14,
        "l_elbow_pitch" | "r_elbow_pitch" => -0.52,
        "l_hip_pitch" | "r_hip_pitch" => -0.12,
        "l_hip_roll" => 0.04,
        "r_hip_roll" => -0.04,
        "l_knee_pitch" | "r_knee_pitch" => 0.24,
        "l_ankle_pitch" | "r_ankle_pitch" => -0.12,
        _ => 0.0,
    }
}

/// Teach & Action Studio keyframe.
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct Keyframe {
    pub label: String,
    pub duration_ms: u32,
    pub angles_rad: [f32; MAX_JOINTS],
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct ActionMove {
    pub name: String,
    pub category: String,
    pub keyframes: Vec<Keyframe>,
}

#[derive(Clone, Debug)]
pub struct PlaybackState {
    pub move_idx: usize,
    pub elapsed_ms: f32,
    pub start_pose: [f32; MAX_JOINTS],
    pub progress: f32,
    pub active_keyframe: usize,
}

/// C1 smoothstep cubic Hermite curve in single-precision `f32`.
#[inline]
pub fn cubic_smoothstep(t: f32) -> f32 {
    let c = t.clamp(0.0_f32, 1.0_f32);
    c * c * (3.0_f32 - 2.0_f32 * c)
}

/// Evaluates an `ActionMove` at `elapsed_ms`, scaling tempo conservatively with `speed_limit_pct`.
pub fn sample_action_move(
    action: &ActionMove,
    start_pose: &[f32; MAX_JOINTS],
    elapsed_ms: f32,
    speed_limit_pct: u8,
) -> ([f32; MAX_JOINTS], bool, f32, usize) {
    if action.keyframes.is_empty() {
        return (*start_pose, true, 1.0, 0);
    }

    let rate_scale = (0.25_f32 + (speed_limit_pct as f32 / 100.0_f32) * 0.75_f32).clamp(0.2, 1.0);
    let scaled_elapsed = elapsed_ms * rate_scale;
    let total_ms: f32 = action
        .keyframes
        .iter()
        .map(|k| k.duration_ms.max(60) as f32)
        .sum();

    if scaled_elapsed >= total_ms {
        let last = action.keyframes.last().expect("non-empty keyframes");
        return (last.angles_rad, true, 1.0, action.keyframes.len() - 1);
    }

    let mut accum = 0.0_f32;
    for (idx, kf) in action.keyframes.iter().enumerate() {
        let dur = kf.duration_ms.max(60) as f32;
        if scaled_elapsed <= accum + dur || idx + 1 == action.keyframes.len() {
            let local_t = (scaled_elapsed - accum) / dur;
            let s = cubic_smoothstep(local_t);
            let prev = if idx == 0 {
                start_pose
            } else {
                &action.keyframes[idx - 1].angles_rad
            };
            let mut out = [0.0_f32; MAX_JOINTS];
            for j in 0..MAX_JOINTS {
                out[j] = prev[j] + (kf.angles_rad[j] - prev[j]) * s;
            }
            return (out, false, (scaled_elapsed / total_ms).clamp(0.0, 1.0), idx);
        }
        accum += dur;
    }

    (*start_pose, true, 1.0, 0)
}

pub fn default_combat_library(rig: &KinematicRig) -> Vec<ActionMove> {
    let pose = |overrides: &[(&str, f32)]| -> [f32; MAX_JOINTS] {
        let mut arr = rig.default_pose();
        for (name, val) in overrides {
            if let Some(j) = rig.joints.iter().find(|jm| jm.name == *name) {
                arr[j.index] = val.clamp(j.default_min_rad, j.default_max_rad);
            }
        }
        arr
    };

    let neutral = pose(&[]);
    let guard = pose(&[
        ("waist_yaw", 0.22),
        ("head_yaw", -0.18),
        ("l_shoulder_pitch", -1.10),
        ("l_shoulder_roll", 0.20),
        ("l_elbow_pitch", -1.72),
        ("r_shoulder_pitch", -1.18),
        ("r_shoulder_roll", -0.20),
        ("r_elbow_pitch", -1.85),
        ("l_hip_pitch", -0.26),
        ("l_knee_pitch", 0.50),
        ("l_ankle_pitch", -0.24),
        ("r_hip_pitch", -0.16),
        ("r_knee_pitch", 0.34),
        ("r_ankle_pitch", -0.18),
    ]);

    let left_jab = pose(&[
        ("waist_yaw", -0.35),
        ("head_yaw", 0.28),
        ("l_shoulder_pitch", -1.56),
        ("l_shoulder_roll", 0.05),
        ("l_elbow_pitch", -0.06),
        ("r_shoulder_pitch", -1.20),
        ("r_shoulder_roll", -0.18),
        ("r_elbow_pitch", -1.92),
        ("l_hip_pitch", -0.32),
        ("l_knee_pitch", 0.60),
        ("l_ankle_pitch", -0.28),
    ]);

    let right_cross_windup = pose(&[
        ("waist_yaw", -0.34),
        ("head_yaw", 0.26),
        ("l_shoulder_pitch", -1.12),
        ("l_elbow_pitch", -1.65),
        ("r_shoulder_pitch", -0.80),
        ("r_shoulder_roll", -0.32),
        ("r_elbow_pitch", -1.75),
    ]);

    let right_cross_impact = pose(&[
        ("waist_yaw", 0.56),
        ("head_yaw", -0.44),
        ("l_shoulder_pitch", -1.22),
        ("l_elbow_pitch", -1.90),
        ("r_shoulder_pitch", -1.58),
        ("r_shoulder_roll", -0.04),
        ("r_elbow_pitch", -0.05),
        ("l_hip_pitch", -0.34),
        ("l_knee_pitch", 0.62),
        ("l_ankle_pitch", -0.28),
    ]);

    let kick_chamber = pose(&[
        ("waist_yaw", -0.14),
        ("l_shoulder_pitch", -1.00),
        ("l_shoulder_roll", 0.35),
        ("l_elbow_pitch", -1.55),
        ("r_shoulder_pitch", -1.00),
        ("r_shoulder_roll", -0.35),
        ("r_elbow_pitch", -1.55),
        ("l_hip_roll", 0.14),
        ("r_hip_pitch", -1.35),
        ("r_knee_pitch", 1.85),
        ("r_ankle_pitch", 0.25),
    ]);

    let kick_snap = pose(&[
        ("waist_yaw", 0.18),
        ("l_shoulder_pitch", -0.92),
        ("l_shoulder_roll", 0.48),
        ("l_elbow_pitch", -1.45),
        ("r_shoulder_pitch", -0.86),
        ("r_shoulder_roll", -0.42),
        ("r_elbow_pitch", -1.45),
        ("l_hip_roll", 0.16),
        ("r_hip_pitch", -1.42),
        ("r_knee_pitch", 0.10),
        ("r_ankle_pitch", 0.42),
    ]);

    vec![
        ActionMove {
            name: "Guard".to_string(),
            category: "Stance".to_string(),
            keyframes: vec![Keyframe {
                label: "High Guard".to_string(),
                duration_ms: 600,
                angles_rad: guard,
            }],
        },
        ActionMove {
            name: "Left Jab".to_string(),
            category: "Strike".to_string(),
            keyframes: vec![
                Keyframe {
                    label: "Guard Ready".to_string(),
                    duration_ms: 300,
                    angles_rad: guard,
                },
                Keyframe {
                    label: "Snap Extend".to_string(),
                    duration_ms: 360,
                    angles_rad: left_jab,
                },
                Keyframe {
                    label: "Guard Return".to_string(),
                    duration_ms: 400,
                    angles_rad: guard,
                },
            ],
        },
        ActionMove {
            name: "Right Cross".to_string(),
            category: "Strike".to_string(),
            keyframes: vec![
                Keyframe {
                    label: "Torque Load".to_string(),
                    duration_ms: 320,
                    angles_rad: right_cross_windup,
                },
                Keyframe {
                    label: "Full Impact".to_string(),
                    duration_ms: 420,
                    angles_rad: right_cross_impact,
                },
                Keyframe {
                    label: "Guard Return".to_string(),
                    duration_ms: 460,
                    angles_rad: guard,
                },
            ],
        },
        ActionMove {
            name: "Front Kick".to_string(),
            category: "Kick".to_string(),
            keyframes: vec![
                Keyframe {
                    label: "Chamber Knee".to_string(),
                    duration_ms: 440,
                    angles_rad: kick_chamber,
                },
                Keyframe {
                    label: "Snap Thrust".to_string(),
                    duration_ms: 400,
                    angles_rad: kick_snap,
                },
                Keyframe {
                    label: "Re-Chamber".to_string(),
                    duration_ms: 360,
                    angles_rad: kick_chamber,
                },
                Keyframe {
                    label: "Plant Guard".to_string(),
                    duration_ms: 460,
                    angles_rad: guard,
                },
            ],
        },
        ActionMove {
            name: "Neutral Zero".to_string(),
            category: "Stance".to_string(),
            keyframes: vec![Keyframe {
                label: "Calibration Pose".to_string(),
                duration_ms: 650,
                angles_rad: neutral,
            }],
        },
    ]
}

/// Calibration persistence file format.
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct CalibrationFile {
    pub robot_name: String,
    pub joints: Vec<JointCalibration>,
}

/// Bidirectional mirroring domain state machine.
pub struct StudioModel {
    pub bundle: AssetBundle,
    pub rig: KinematicRig,
    pub requested_mode: ControlMode,
    pub speed_limit_pct: u8,
    pub torque_mask: u32,
    pub target_rad: [f32; MAX_JOINTS],
    pub calibration: Vec<JointCalibration>,
    pub latest_telemetry: McuToHost,
    pub selected_joint: usize,
    pub moves: Vec<ActionMove>,
    pub selected_move: usize,
    pub playback: Option<PlaybackState>,
    pub pending_config: Vec<ConfigCmd>,
    pub tx_seq: u32,
    pub status_message: String,
}

impl StudioModel {
    pub fn new_default() -> Self {
        let bundle = AssetBundle::bundled_rocky();
        let rig = KinematicRig::from_urdf_xml(&bundle.urdf_xml)
            .expect("bundled rocky_mk1.urdf must be valid");
        let calibration = rig.default_calibration();
        let default_pose = rig.default_pose();
        let moves = default_combat_library(&rig);
        let all_mask = if rig.joints.len() >= 32 {
            u32::MAX
        } else {
            (1u32 << rig.joints.len()) - 1
        };

        let mut latest_telemetry = McuToHost::default();
        latest_telemetry.measured_rad = default_pose;
        latest_telemetry.applied_target_rad = default_pose;

        Self {
            bundle,
            rig,
            requested_mode: ControlMode::Teach,
            speed_limit_pct: 25, // Conservative 25% startup default per Section 7.1
            torque_mask: all_mask,
            target_rad: default_pose,
            calibration,
            latest_telemetry,
            selected_joint: 3, // Left shoulder pitch
            moves,
            selected_move: 0,
            playback: None,
            pending_config: Vec::new(),
            tx_seq: 0,
            status_message: "Ready in Teach mode (Torque OFF). Press Space at any time for E-Stop."
                .to_string(),
        }
    }

    pub fn load_bundle(&mut self, bundle: AssetBundle) -> Result<(), String> {
        let rig = KinematicRig::from_urdf_xml(&bundle.urdf_xml)?;
        self.requested_mode = ControlMode::Teach;
        self.playback = None;
        self.calibration = rig.default_calibration();
        self.target_rad = rig.default_pose();
        self.moves = default_combat_library(&rig);
        self.selected_joint = 0;
        self.selected_move = 0;
        self.torque_mask = if rig.joints.len() >= 32 {
            u32::MAX
        } else {
            (1u32 << rig.joints.len()) - 1
        };
        self.status_message = format!(
            "Loaded URDF '{}' ({} DoF) in Teach mode.",
            rig.robot_name,
            rig.joints.len()
        );
        self.bundle = bundle;
        self.rig = rig;
        Ok(())
    }

    pub fn is_estop_active(&self) -> bool {
        self.requested_mode == ControlMode::EStop
            || self.latest_telemetry.active_mode == ControlMode::EStop
            || self
                .latest_telemetry
                .faults
                .intersects(FaultFlags::ESTOP_ACTIVE | FaultFlags::WATCHDOG_TIMEOUT)
    }

    /// Triggers immediate E-Stop (bound to Spacebar and top red button).
    pub fn trigger_estop(&mut self) {
        self.requested_mode = ControlMode::EStop;
        self.playback = None;
        self.status_message = "E-STOP ACTIVE — Torque forced OFF. Click Reset E-Stop to return to Teach.".to_string();
    }

    /// Explicit operator confirmation: clears faults and returns strictly to `Teach` (never directly to `Drive`).
    pub fn reset_estop(&mut self) {
        self.requested_mode = ControlMode::Teach;
        self.playback = None;
        self.target_rad = self.latest_telemetry.measured_rad;
        self.pending_config.push(ConfigCmd::ClearFaults);
        self.status_message = "E-Stop reset -> Teach mode (Torque OFF).".to_string();
    }

    /// Authority mode toggle enforcing **Mandatory Safety Rule 1 (Bumpless Transfer)**.
    pub fn set_authority_mode(&mut self, mode: ControlMode) {
        if self.is_estop_active() && mode != ControlMode::EStop {
            return;
        }
        if mode == ControlMode::Drive && self.requested_mode != ControlMode::Drive {
            // Bumpless transfer: overwrite target_rad = measured_rad BEFORE emitting Drive!
            for i in 0..self.rig.joints.len() {
                self.target_rad[i] = self.latest_telemetry.measured_rad[i];
            }
            self.status_message =
                "Switched Teach -> Drive (Bumpless sync: 0.000 rad delta).".to_string();
        } else if mode == ControlMode::Teach {
            self.playback = None;
            self.status_message = "Switched to Teach mode (Robot -> GUI, Torque OFF).".to_string();
        }
        self.requested_mode = mode;
    }

    /// Enables or disables torque on a single joint while enforcing bumpless sync when enabling.
    pub fn set_joint_torque(&mut self, joint: usize, enabled: bool) {
        if joint >= self.rig.joints.len() {
            return;
        }
        let bit = 1u32 << joint;
        let was_enabled = (self.torque_mask & bit) != 0;
        if enabled {
            if !was_enabled {
                self.target_rad[joint] = self.latest_telemetry.measured_rad[joint];
            }
            self.torque_mask |= bit;
        } else {
            self.torque_mask &= !bit;
        }
    }

    /// Captures live `measured_rad` as a new keyframe on the selected combat move (Teach mode workflow).
    pub fn capture_keyframe(&mut self) {
        if let Some(mv) = self.moves.get_mut(self.selected_move) {
            let n = mv.keyframes.len() + 1;
            mv.keyframes.push(Keyframe {
                label: format!("Captured Pose {n}"),
                duration_ms: 450,
                angles_rad: self.latest_telemetry.measured_rad,
            });
            self.status_message = format!("Captured keyframe #{n} into '{}'.", mv.name);
        }
    }

    /// Starts cubic-interpolated playback of a combat move (Drive mode only).
    pub fn play_move(&mut self, move_idx: usize) {
        if self.requested_mode != ControlMode::Drive || self.is_estop_active() {
            self.status_message =
                "Switch to Drive mode before playing combat actions.".to_string();
            return;
        }
        if move_idx < self.moves.len() {
            self.selected_move = move_idx;
            self.playback = Some(PlaybackState {
                move_idx,
                elapsed_ms: 0.0,
                start_pose: self.latest_telemetry.measured_rad,
                progress: 0.0,
                active_keyframe: 0,
            });
            self.status_message = format!("Playing '{}'.", self.moves[move_idx].name);
        }
    }

    pub fn stop_playback(&mut self) {
        if self.playback.take().is_some() {
            self.target_rad = self.latest_telemetry.measured_rad;
            self.status_message = "Playback stopped.".to_string();
        }
    }

    /// Sets the current raw angle of `joint` as zero offset via `ConfigCmd::SetZeroOffset`.
    pub fn set_current_as_zero(&mut self, joint: usize) {
        if let Some(cal) = self.calibration.get_mut(joint) {
            let measured = self.latest_telemetry.measured_rad[joint];
            let sign = if cal.inverted { -1.0_f32 } else { 1.0_f32 };
            let raw = (sign * measured) + cal.zero_offset_rad;
            cal.zero_offset_rad = raw;
            self.target_rad[joint] = 0.0;
            self.pending_config.push(ConfigCmd::SetZeroOffset {
                joint: joint as u8,
                zero_rad: raw,
            });
            self.status_message = format!(
                "Zeroed joint '{}' (offset {:.2} deg).",
                self.rig.joints[joint].label,
                raw.to_degrees()
            );
        }
    }

    pub fn toggle_invert(&mut self, joint: usize) {
        if let Some(cal) = self.calibration.get_mut(joint) {
            cal.inverted = !cal.inverted;
            let inv = cal.inverted;
            self.pending_config.push(ConfigCmd::SetInvert {
                joint: joint as u8,
                inverted: inv,
            });
        }
    }

    pub fn update_soft_limits(&mut self, joint: usize, min_rad: f32, max_rad: f32) {
        if let Some(cal) = self.calibration.get_mut(joint) {
            let (lo, hi) = if min_rad <= max_rad {
                (min_rad, max_rad)
            } else {
                (max_rad, min_rad)
            };
            cal.min_rad = lo;
            cal.max_rad = hi;
            self.pending_config.push(ConfigCmd::SetSoftLimits {
                joint: joint as u8,
                min_rad: lo,
                max_rad: hi,
            });
        }
    }

    pub fn export_calibration_json(&self) -> Result<String, String> {
        let file = CalibrationFile {
            robot_name: self.rig.robot_name.clone(),
            joints: self.calibration.clone(),
        };
        serde_json::to_string_pretty(&file).map_err(|e| format!("Serialize error: {e}"))
    }

    pub fn import_calibration_json(&mut self, json: &str) -> Result<(), String> {
        let file: CalibrationFile =
            serde_json::from_str(json).map_err(|e| format!("Invalid calibration JSON: {e}"))?;
        for (idx, cal) in file.joints.into_iter().enumerate() {
            if idx < self.calibration.len() {
                self.calibration[idx] = cal;
                self.pending_config.push(ConfigCmd::SetZeroOffset {
                    joint: idx as u8,
                    zero_rad: cal.zero_offset_rad,
                });
                self.pending_config.push(ConfigCmd::SetInvert {
                    joint: idx as u8,
                    inverted: cal.inverted,
                });
                self.pending_config.push(ConfigCmd::SetSoftLimits {
                    joint: idx as u8,
                    min_rad: cal.min_rad,
                    max_rad: cal.max_rad,
                });
            }
        }
        self.status_message = "Calibration imported and queued for MCU.".to_string();
        Ok(())
    }

    /// Advances active keyframe playback and constructs the next `HostToMcu` packet.
    pub fn build_next_host_packet(&mut self, dt_sec: f32) -> HostToMcu {
        if self.requested_mode == ControlMode::Drive && !self.is_estop_active() {
            if let Some(pb) = self.playback.as_mut() {
                pb.elapsed_ms += dt_sec * 1000.0;
                if let Some(mv) = self.moves.get(pb.move_idx) {
                    let (pose, done, progress, active_kf) = sample_action_move(
                        mv,
                        &pb.start_pose,
                        pb.elapsed_ms,
                        self.speed_limit_pct,
                    );
                    self.target_rad = pose;
                    pb.progress = progress;
                    pb.active_keyframe = active_kf;
                    if done {
                        self.playback = None;
                    }
                } else {
                    self.playback = None;
                }
            }
        } else if self.requested_mode == ControlMode::Teach {
            // In Teach mode, keep target_rad mirrored to measured_rad for zero-delta bumpless handoff
            self.target_rad = self.latest_telemetry.measured_rad;
        }

        self.tx_seq = self.tx_seq.wrapping_add(1);
        let config = if !self.pending_config.is_empty() {
            self.pending_config.remove(0)
        } else {
            ConfigCmd::None
        };

        let mode = if self.is_estop_active() && config != ConfigCmd::ClearFaults {
            ControlMode::EStop
        } else {
            self.requested_mode
        };

        let torque_mask = if mode == ControlMode::Drive {
            self.torque_mask
        } else {
            0
        };

        HostToMcu {
            seq: self.tx_seq,
            mode,
            torque_mask,
            speed_limit_pct: self.speed_limit_pct.clamp(1, 100),
            target_rad: self.target_rad,
            config,
        }
    }

    pub fn ingest_telemetry(&mut self, telem: McuToHost) {
        if telem.active_mode == ControlMode::EStop && self.requested_mode == ControlMode::Drive {
            self.requested_mode = ControlMode::EStop;
            self.playback = None;
        }
        self.latest_telemetry = telem;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bundled_urdf_parses_and_evaluates_fk() {
        let rig = KinematicRig::from_urdf_xml(BUNDLED_URDF_XML).expect("parse bundled urdf");
        assert_eq!(rig.joints.len(), 17);
        let transforms = rig.compute_link_transforms(&rig.default_pose());
        assert!(transforms.contains_key("pelvis"));
        assert!(transforms.contains_key("l_forearm"));
        assert!(transforms.contains_key("r_foot"));
    }

    #[test]
    fn bumpless_transfer_has_zero_delta_on_teach_to_drive() {
        let mut model = StudioModel::new_default();
        model.latest_telemetry.measured_rad[3] = -1.23_f32;
        model.target_rad[3] = 0.77_f32; // Stale target

        model.set_authority_mode(ControlMode::Drive);
        assert_eq!(model.requested_mode, ControlMode::Drive);
        assert!((model.target_rad[3] - (-1.23_f32)).abs() < 1e-6);
    }

    #[test]
    fn cubic_interpolator_reaches_exact_endpoints() {
        let rig = KinematicRig::from_urdf_xml(BUNDLED_URDF_XML).unwrap();
        let moves = default_combat_library(&rig);
        let jab = &moves[1];
        let start = rig.default_pose();
        let (p0, done0, _, _) = sample_action_move(jab, &start, 0.0, 100);
        assert!(!done0);
        assert!((p0[3] - start[3]).abs() < 1e-6);

        let (p_end, done_end, prog, _) = sample_action_move(jab, &start, 5000.0, 100);
        assert!(done_end);
        assert!((prog - 1.0).abs() < 1e-6);
        assert!((p_end[3] - jab.keyframes.last().unwrap().angles_rad[3]).abs() < 1e-6);
    }
}
