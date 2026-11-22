//! URDF parsing (`urdf-rs`), forward kinematics (`k::Chain<f32>`), up-axis detection, and AABB grounding.

use std::collections::HashMap;

use humanoid_control::JointCalibration;
use humanoid_proto::MAX_JOINTS;
use k::nalgebra::{self, Isometry3, Point3, Translation3, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};

/// Vertical coordinate convention of a URDF model.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum UpAxis {
    /// Y-up convention (e.g. bundled `rocky_mk1.urdf` / graphics-native exports).
    YUp,
    /// Standard ROS / SolidWorks / Fusion360 / Onshape URDF convention (+Z is up).
    ZUp,
}

impl UpAxis {
    pub const fn label(self) -> &'static str {
        match self {
            Self::YUp => "Y-Up",
            Self::ZUp => "Z-Up (ROS)",
        }
    }

    /// Root rotation aligning the URDF frame into the Y-up 3D viewport frame.
    pub fn root_rotation(self) -> UnitQuaternion<f32> {
        match self {
            Self::YUp => UnitQuaternion::identity(),
            Self::ZUp => UnitQuaternion::from_axis_angle(
                &Vector3::x_axis(),
                -std::f32::consts::FRAC_PI_2,
            ),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointKind {
    Revolute,
    Continuous,
    Prismatic,
}

impl JointKind {
    pub const fn label(self) -> &'static str {
        match self {
            Self::Revolute => "revolute",
            Self::Continuous => "continuous",
            Self::Prismatic => "prismatic",
        }
    }
}

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
    pub kind: JointKind,
    pub group: LimbGroup,
    pub parent_link: String,
    pub child_link: String,
    pub axis: [f32; 3],
    pub origin_xyz: [f32; 3],
    pub default_min_rad: f32,
    pub default_max_rad: f32,
    pub default_rad: f32,
}

#[derive(Clone, Debug)]
pub struct LinkSummary {
    pub name: String,
    pub parent_link: Option<String>,
    pub joint_name: Option<String>,
    pub joint_index: Option<usize>,
    pub visual_count: usize,
    pub mesh_refs: Vec<String>,
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

/// Forward kinematics evaluator backed by `urdf-rs` + `k::Chain<f32>`.
pub struct KinematicRig {
    pub robot_name: String,
    pub chain: k::Chain<f32>,
    pub joints: Vec<JointMeta>,
    pub links: Vec<LinkSummary>,
    pub visuals: Vec<LinkVisual>,
    pub up_axis: UpAxis,
    /// Root height above floor so any uploaded robot stands cleanly grounded on `y = 0`.
    pub root_y_offset: f32,
    pub bounds_min: [f32; 3],
    pub bounds_max: [f32; 3],
}

impl KinematicRig {
    pub fn from_urdf_xml(xml: &str) -> Result<Self, String> {
        Self::from_urdf_xml_with_up(xml, None)
    }

    pub fn from_urdf_xml_with_up(xml: &str, up_override: Option<UpAxis>) -> Result<Self, String> {
        let robot =
            urdf_rs::read_from_string(xml).map_err(|e| format!("Failed to parse URDF XML: {e}"))?;

        // Build top-level material lookup table (`<robot><material name="..."><color .../></material>`)
        let mut material_colors: HashMap<String, [f32; 4]> = HashMap::new();
        for mat in &robot.materials {
            if let Some(c) = &mat.color {
                material_colors.insert(
                    mat.name.clone(),
                    [
                        c.rgba[0] as f32,
                        c.rgba[1] as f32,
                        c.rgba[2] as f32,
                        c.rgba[3] as f32,
                    ],
                );
            }
        }

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

                let axis_raw = [j.axis.xyz[0] as f32, j.axis.xyz[1] as f32, j.axis.xyz[2] as f32];
                let axis_vec = Vector3::new(axis_raw[0], axis_raw[1], axis_raw[2]);
                let unit_axis = nalgebra::Unit::try_new(axis_vec, 1e-6)
                    .unwrap_or_else(|| nalgebra::Unit::new_normalize(Vector3::z()));

                let kind_opt = match j.joint_type {
                    urdf_rs::JointType::Revolute => Some(JointKind::Revolute),
                    urdf_rs::JointType::Continuous => Some(JointKind::Continuous),
                    urdf_rs::JointType::Prismatic => Some(JointKind::Prismatic),
                    _ => None,
                };

                let mut builder = k::NodeBuilder::<f32>::new()
                    .name(&j.child.link)
                    .translation(trans)
                    .rotation(rot);

                if let Some(kind) = kind_opt.filter(|_| joint_metas.len() < MAX_JOINTS) {
                    let idx = joint_metas.len();
                    let (lo, hi) = match kind {
                        JointKind::Continuous => {
                            let min_r = (j.limit.lower as f32).clamp(-3.1416, 3.1416);
                            let max_r = (j.limit.upper as f32).clamp(-3.1416, 3.1416);
                            if min_r < max_r {
                                (min_r, max_r)
                            } else {
                                (-std::f32::consts::PI, std::f32::consts::PI)
                            }
                        }
                        JointKind::Revolute => {
                            let min_r = (j.limit.lower as f32).clamp(-3.1416, 3.1416);
                            let max_r = (j.limit.upper as f32).clamp(-3.1416, 3.1416);
                            if min_r < max_r {
                                (min_r, max_r)
                            } else {
                                (-1.57, 1.57)
                            }
                        }
                        JointKind::Prismatic => {
                            let min_d = (j.limit.lower as f32).clamp(-2.0, 2.0);
                            let max_d = (j.limit.upper as f32).clamp(-2.0, 2.0);
                            if min_d < max_d {
                                (min_d, max_d)
                            } else {
                                (-0.15, 0.15)
                            }
                        }
                    };

                    builder = match kind {
                        JointKind::Prismatic => builder
                            .joint_type(k::JointType::Linear { axis: unit_axis })
                            .limits(Some(k::joint::Range::new(lo, hi))),
                        JointKind::Revolute | JointKind::Continuous => builder
                            .joint_type(k::JointType::Rotational { axis: unit_axis })
                            .limits(Some(k::joint::Range::new(lo, hi))),
                    };

                    let default_rad = if kind == JointKind::Prismatic {
                        0.0_f32.clamp(lo, hi)
                    } else {
                        default_stance_angle(&j.name).clamp(lo, hi)
                    };
                    joint_metas.push(JointMeta {
                        index: idx,
                        name: j.name.clone(),
                        label: prettify_joint_name(&j.name),
                        kind,
                        group: classify_limb_group(&j.name),
                        parent_link: j.parent.link.clone(),
                        child_link: j.child.link.clone(),
                        axis: [unit_axis.x, unit_axis.y, unit_axis.z],
                        origin_xyz: [xyz[0] as f32, xyz[1] as f32, xyz[2] as f32],
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

        // Extract link summaries & visual geometries with full top-level material resolution
        let mut visuals = Vec::new();
        let mut links = Vec::with_capacity(robot.links.len());

        for link in &robot.links {
            let joint_index = link_to_joint_idx.get(&link.name).copied();
            let parent_joint = child_to_joint.get(&link.name).copied();
            let mut mesh_refs = Vec::new();

            if link.visual.is_empty() {
                visuals.push(LinkVisual {
                    link_name: link.name.clone(),
                    joint_index,
                    local_offset: Isometry3::identity(),
                    shape: VisualShape::Box([0.05, 0.05, 0.05]),
                    rgba: [0.32, 0.37, 0.44, 1.0],
                });
            } else {
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
                            mesh_refs.push(filename.clone());
                            VisualShape::Mesh {
                                filename: filename.clone(),
                                scale: s,
                            }
                        }
                        _ => VisualShape::Box([0.06, 0.06, 0.06]),
                    };

                    let rgba = vis
                        .material
                        .as_ref()
                        .and_then(|m| {
                            m.color
                                .as_ref()
                                .map(|c| {
                                    [
                                        c.rgba[0] as f32,
                                        c.rgba[1] as f32,
                                        c.rgba[2] as f32,
                                        c.rgba[3] as f32,
                                    ]
                                })
                                .or_else(|| material_colors.get(&m.name).copied())
                        })
                        .unwrap_or([0.36, 0.42, 0.50, 1.0]);

                    visuals.push(LinkVisual {
                        link_name: link.name.clone(),
                        joint_index,
                        local_offset,
                        shape,
                        rgba,
                    });
                }
            }

            links.push(LinkSummary {
                name: link.name.clone(),
                parent_link: parent_joint.map(|j| j.parent.link.clone()),
                joint_name: parent_joint.map(|j| j.name.clone()),
                joint_index,
                visual_count: link.visual.len(),
                mesh_refs,
            });
        }

        let detected_up = up_override.unwrap_or_else(|| detect_up_axis(&robot, &chain));

        let mut rig = Self {
            robot_name: robot.name,
            chain,
            joints: joint_metas,
            links,
            visuals,
            up_axis: detected_up,
            root_y_offset: 0.0,
            bounds_min: [-0.3, 0.0, -0.3],
            bounds_max: [0.3, 1.2, 0.3],
        };
        rig.recompute_grounding();
        Ok(rig)
    }

    pub fn set_up_axis(&mut self, up_axis: UpAxis) {
        if self.up_axis != up_axis {
            self.up_axis = up_axis;
            self.recompute_grounding();
        }
    }

    /// Computes accurate visual AABB in default pose and grounds the lowest point at `y = 0`.
    pub fn recompute_grounding(&mut self) {
        self.root_y_offset = 0.0;
        let pose = self.default_pose();
        let transforms = self.compute_link_transforms(&pose);

        let mut min_v = Vector3::new(f32::INFINITY, f32::INFINITY, f32::INFINITY);
        let mut max_v = Vector3::new(f32::NEG_INFINITY, f32::NEG_INFINITY, f32::NEG_INFINITY);

        for vis in &self.visuals {
            let Some(iso) = transforms.get(&vis.link_name) else {
                continue;
            };
            let world = iso * vis.local_offset;
            let half = match vis.shape {
                VisualShape::Box(s) => Vector3::new(s[0] * 0.5, s[1] * 0.5, s[2] * 0.5),
                VisualShape::Cylinder { radius, length } => match self.up_axis {
                    UpAxis::YUp => Vector3::new(radius, length * 0.5, radius),
                    UpAxis::ZUp => Vector3::new(radius, radius, length * 0.5),
                },
                VisualShape::Sphere { radius } => Vector3::new(radius, radius, radius),
                VisualShape::Mesh { scale, .. } => {
                    let s = scale[0].abs().max(scale[1].abs()).max(scale[2].abs()).clamp(0.02, 2.0);
                    Vector3::new(0.08 * s, 0.08 * s, 0.08 * s)
                }
            };
            for &sx in &[-1.0_f32, 1.0_f32] {
                for &sy in &[-1.0_f32, 1.0_f32] {
                    for &sz in &[-1.0_f32, 1.0_f32] {
                        let p = world.transform_point(&Point3::new(
                            sx * half.x,
                            sy * half.y,
                            sz * half.z,
                        ));
                        min_v.x = min_v.x.min(p.x);
                        min_v.y = min_v.y.min(p.y);
                        min_v.z = min_v.z.min(p.z);
                        max_v.x = max_v.x.max(p.x);
                        max_v.y = max_v.y.max(p.y);
                        max_v.z = max_v.z.max(p.z);
                    }
                }
            }
        }

        if min_v.y.is_finite() && max_v.y.is_finite() {
            self.root_y_offset = -min_v.y;
            self.bounds_min = [min_v.x, 0.0, min_v.z];
            self.bounds_max = [max_v.x, (max_v.y - min_v.y).max(0.05), max_v.z];
        } else {
            self.root_y_offset = 0.0;
            self.bounds_min = [-0.25, 0.0, -0.25];
            self.bounds_max = [0.25, 1.0, 0.25];
        }
    }

    pub fn bounds_center(&self) -> [f32; 3] {
        [
            0.5 * (self.bounds_min[0] + self.bounds_max[0]),
            0.5 * (self.bounds_min[1] + self.bounds_max[1]),
            0.5 * (self.bounds_min[2] + self.bounds_max[2]),
        ]
    }

    pub fn bounds_radius(&self) -> f32 {
        let dx = self.bounds_max[0] - self.bounds_min[0];
        let dy = self.bounds_max[1] - self.bounds_min[1];
        let dz = self.bounds_max[2] - self.bounds_min[2];
        (0.5 * (dx * dx + dy * dy + dz * dz).sqrt()).clamp(0.15, 10.0)
    }

    /// Evaluates full 3D world poses for every link in the URDF using `k::Chain<f32>`.
    pub fn compute_link_transforms(&self, angles_rad: &[f32; MAX_JOINTS]) -> HashMap<String, Isometry3<f32>> {
        let dof = self.chain.dof();
        let positions: Vec<f32> = (0..dof)
            .map(|i| angles_rad.get(i).copied().unwrap_or(0.0_f32))
            .collect();
        let _ = self.chain.set_joint_positions_clamped(&positions);
        self.chain.update_transforms();

        let root_rot = self.up_axis.root_rotation();
        let root_iso = Isometry3::from_parts(
            Translation3::new(0.0_f32, self.root_y_offset, 0.0_f32),
            root_rot,
        );

        let mut map = HashMap::with_capacity(self.visuals.len() + self.links.len() + 2);
        for node in self.chain.iter() {
            let maybe_link_name = node.link().as_ref().map(|l| l.name.clone());
            let link_name = maybe_link_name.unwrap_or_else(|| node.joint().name.clone());
            let iso = node.world_transform().unwrap_or_else(Isometry3::identity);
            map.insert(link_name, root_iso * iso);
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

/// Automatically detects whether a URDF was authored in standard ROS Z-Up or Y-Up.
fn detect_up_axis(robot: &urdf_rs::Robot, chain: &k::Chain<f32>) -> UpAxis {
    if robot.name == "rocky_mk1" {
        return UpAxis::YUp;
    }

    chain.update_transforms();
    let (mut min_y, mut max_y) = (f32::INFINITY, f32::NEG_INFINITY);
    let (mut min_z, mut max_z) = (f32::INFINITY, f32::NEG_INFINITY);
    for node in chain.iter() {
        let iso = node.world_transform().unwrap_or_else(Isometry3::identity);
        let t = iso.translation.vector;
        min_y = min_y.min(t.y);
        max_y = max_y.max(t.y);
        min_z = min_z.min(t.z);
        max_z = max_z.max(t.z);
    }

    let y_span = if min_y.is_finite() { max_y - min_y } else { 0.0 };
    let z_span = if min_z.is_finite() { max_z - min_z } else { 0.0 };

    let z_axes = robot
        .joints
        .iter()
        .filter(|j| j.axis.xyz[2].abs() > 0.7 && j.name.to_ascii_lowercase().contains("yaw"))
        .count();
    let y_axes = robot
        .joints
        .iter()
        .filter(|j| j.axis.xyz[1].abs() > 0.7 && j.name.to_ascii_lowercase().contains("yaw"))
        .count();

    if z_axes > y_axes {
        UpAxis::ZUp
    } else if y_axes > z_axes && y_span > z_span * 1.15 {
        UpAxis::YUp
    } else if z_span >= y_span * 0.85 {
        UpAxis::ZUp
    } else {
        UpAxis::YUp
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
