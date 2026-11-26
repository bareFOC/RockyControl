use std::collections::HashMap;
use std::fmt::Write as _;
use std::fs;
use std::path::Path;

use craft_xtask::runner;

#[derive(Default)]
struct LegGeomSpec {
    thigh_len: f32,
    shin_len: f32,
    hip_ankle_offset: f32,
    half_hip_spacing: f32,
    nominal_height: f32,
    min_height: f32,
    max_reach: f32,
    max_dx: f32,
    max_dy: f32,
    max_swing_dx: f32,
    max_swing_dy: f32,
    max_foot_lift: f32,
    max_sway: f32,
}

#[derive(Default)]
struct ActuatorSpec {
    urdf_joint: String,
    role: Option<String>,
    coupled_joint: Option<String>,
    motor_to_joint_ratio: f32,
    primary_mimic_scale: f32,
    coupled_mimic_scale: f32,
    bus_id: u8,
    node_addr: u8,
    nm_per_amp: f32,
    max_torque_nm: f32,
    max_velocity_rad_s: f32,
}

impl ActuatorSpec {
    fn new() -> Self {
        Self {
            motor_to_joint_ratio: 1.0,
            primary_mimic_scale: 1.0,
            coupled_mimic_scale: 1.0,
            nm_per_amp: 0.9,
            max_torque_nm: 1.8,
            max_velocity_rad_s: 8.0,
            ..Default::default()
        }
    }
}

#[derive(Default)]
struct RobotToml {
    name: String,
    urdf: String,
    z_up: bool,
    pelvis_link: String,
    left_foot_link: Option<String>,
    right_foot_link: Option<String>,
    left_toe_link: Option<String>,
    right_toe_link: Option<String>,
    imu_link: String,
    imu_mounting_rpy: [f32; 3],
    foot_half_length_m: f32,
    foot_margin_m: f32,
    h_com_m: f32,
    leg_geometry: LegGeomSpec,
    actuators: Vec<ActuatorSpec>,
}

fn unquote(s: &str) -> String {
    let t = s.trim();
    if t.starts_with('"') && t.ends_with('"') && t.len() >= 2 {
        t[1..t.len() - 1].to_string()
    } else {
        t.to_string()
    }
}

fn parse_f32(s: &str) -> Result<f32, runner::Error> {
    s.trim()
        .parse::<f32>()
        .map_err(|e| runner::Error::Config(format!("invalid f32 '{s}': {e}")))
}

fn parse_u8(s: &str) -> Result<u8, runner::Error> {
    let t = s.trim();
    if let Some(hex) = t.strip_prefix("0x").or_else(|| t.strip_prefix("0X")) {
        u8::from_str_radix(hex, 16)
    } else {
        t.parse::<u8>()
    }
    .map_err(|e| runner::Error::Config(format!("invalid u8 '{s}': {e}")))
}

fn parse_f32_3(s: &str) -> Result<[f32; 3], runner::Error> {
    let t = s.trim();
    let inner = t
        .strip_prefix('[')
        .and_then(|x| x.strip_suffix(']'))
        .ok_or_else(|| runner::Error::Config(format!("expected [f32; 3], got '{s}'")))?;
    let parts: Vec<&str> = inner.split(',').collect();
    if parts.len() != 3 {
        return Err(runner::Error::Config(format!("expected 3 floats, got '{s}'")));
    }
    Ok([parse_f32(parts[0])?, parse_f32(parts[1])?, parse_f32(parts[2])?])
}

fn parse_robot_toml(text: &str) -> Result<RobotToml, runner::Error> {
    #[derive(PartialEq)]
    enum Section {
        Root,
        LegGeometry,
        Actuator,
    }
    let mut out = RobotToml::default();
    let mut section = Section::Root;
    for raw_line in text.lines() {
        let line = raw_line.split('#').next().unwrap_or("").trim();
        if line.is_empty() {
            continue;
        }
        if line == "[leg_geometry]" {
            section = Section::LegGeometry;
            continue;
        }
        if line == "[[actuators]]" {
            section = Section::Actuator;
            out.actuators.push(ActuatorSpec::new());
            continue;
        }
        let Some((k, v)) = line.split_once('=') else {
            return Err(runner::Error::Config(format!("invalid TOML line: {line}")));
        };
        let key = k.trim();
        let val = v.trim();
        match section {
            Section::Root => match key {
                "name" => out.name = unquote(val),
                "urdf" => out.urdf = unquote(val),
                "z_up" => out.z_up = val == "true",
                "pelvis_link" => out.pelvis_link = unquote(val),
                "left_foot_link" => out.left_foot_link = Some(unquote(val)),
                "right_foot_link" => out.right_foot_link = Some(unquote(val)),
                "left_toe_link" => out.left_toe_link = Some(unquote(val)),
                "right_toe_link" => out.right_toe_link = Some(unquote(val)),
                "imu_link" => out.imu_link = unquote(val),
                "imu_mounting_rpy" => out.imu_mounting_rpy = parse_f32_3(val)?,
                "foot_half_length_m" => out.foot_half_length_m = parse_f32(val)?,
                "foot_margin_m" => out.foot_margin_m = parse_f32(val)?,
                "h_com_m" => out.h_com_m = parse_f32(val)?,
                _ => return Err(runner::Error::Config(format!("unknown root key: {key}"))),
            },
            Section::LegGeometry => {
                let f = parse_f32(val)?;
                match key {
                    "thigh_len" => out.leg_geometry.thigh_len = f,
                    "shin_len" => out.leg_geometry.shin_len = f,
                    "hip_ankle_offset" => out.leg_geometry.hip_ankle_offset = f,
                    "half_hip_spacing" => out.leg_geometry.half_hip_spacing = f,
                    "nominal_height" => out.leg_geometry.nominal_height = f,
                    "min_height" => out.leg_geometry.min_height = f,
                    "max_reach" => out.leg_geometry.max_reach = f,
                    "max_dx" => out.leg_geometry.max_dx = f,
                    "max_dy" => out.leg_geometry.max_dy = f,
                    "max_swing_dx" => out.leg_geometry.max_swing_dx = f,
                    "max_swing_dy" => out.leg_geometry.max_swing_dy = f,
                    "max_foot_lift" => out.leg_geometry.max_foot_lift = f,
                    "max_sway" => out.leg_geometry.max_sway = f,
                    _ => return Err(runner::Error::Config(format!("unknown leg_geometry key: {key}"))),
                }
            }
            Section::Actuator => {
                let act = out.actuators.last_mut().expect("actuator pushed");
                match key {
                    "urdf_joint" => act.urdf_joint = unquote(val),
                    "role" => act.role = Some(unquote(val)),
                    "coupled_joint" => act.coupled_joint = Some(unquote(val)),
                    "motor_to_joint_ratio" => act.motor_to_joint_ratio = parse_f32(val)?,
                    "primary_mimic_scale" => act.primary_mimic_scale = parse_f32(val)?,
                    "coupled_mimic_scale" => act.coupled_mimic_scale = parse_f32(val)?,
                    "bus_id" => act.bus_id = parse_u8(val)?,
                    "node_addr" => act.node_addr = parse_u8(val)?,
                    "nm_per_amp" => act.nm_per_amp = parse_f32(val)?,
                    "max_torque_nm" => act.max_torque_nm = parse_f32(val)?,
                    "max_velocity_rad_s" => act.max_velocity_rad_s = parse_f32(val)?,
                    _ => return Err(runner::Error::Config(format!("unknown actuator key: {key}"))),
                }
            }
        }
    }
    Ok(out)
}

pub fn run_codegen(root: &Path) -> Result<(), runner::Error> {
    let robots_dir = root.join("assets/robots");
    let out_dir = root.join("crates/rk-robot-profile/src/generated");
    fs::create_dir_all(&out_dir)
        .map_err(|e| runner::Error::Config(format!("create generated dir: {e}")))?;

    let mut toml_files: Vec<_> = fs::read_dir(&robots_dir)
        .map_err(|e| runner::Error::Config(format!("read assets/robots: {e}")))?
        .filter_map(|e| e.ok().map(|x| x.path()))
        .filter(|p| p.extension().and_then(|s| s.to_str()) == Some("toml"))
        .collect();
    toml_files.sort();

    let mut mods = Vec::new();
    for toml_path in toml_files {
        let toml_text = fs::read_to_string(&toml_path)
            .map_err(|e| runner::Error::Config(format!("read {}: {e}", toml_path.display())))?;
        let spec = parse_robot_toml(&toml_text)?;
        let urdf_path = robots_dir.join(&spec.urdf);
        let urdf_text = fs::read_to_string(&urdf_path)
            .map_err(|e| runner::Error::Config(format!("read {}: {e}", urdf_path.display())))?;
        let robot = urdf_rs::read_from_string(&urdf_text)
            .map_err(|e| runner::Error::Config(format!("parse {}: {e}", urdf_path.display())))?;

        let code = emit_profile_module(&spec, &robot)?;
        let mod_name = spec.name.clone();
        fs::write(out_dir.join(format!("{mod_name}.rs")), code)
            .map_err(|e| runner::Error::Config(format!("write {mod_name}.rs: {e}")))?;
        mods.push(mod_name);
    }

    let mut mod_rs = String::from("// @generated by `cargo run -p xtask -- codegen-robots`. Do not edit.\n\n");
    for m in &mods {
        let _ = writeln!(mod_rs, "pub mod {m};");
    }
    fs::write(out_dir.join("mod.rs"), mod_rs)
        .map_err(|e| runner::Error::Config(format!("write mod.rs: {e}")))?;
    println!("codegen-robots: generated {} profiles in {}", mods.len(), out_dir.display());
    Ok(())
}

fn fmt_f32(v: f32) -> String {
    if v == 0.0 {
        "0.0_f32".to_string()
    } else {
        format!("{v:?}_f32")
    }
}

fn emit_profile_module(spec: &RobotToml, robot: &urdf_rs::Robot) -> Result<String, runner::Error> {
    let mut link_map: HashMap<&str, &urdf_rs::Link> = HashMap::new();
    for l in &robot.links {
        link_map.insert(l.name.as_str(), l);
    }
    let mut children_of: HashMap<&str, Vec<&urdf_rs::Joint>> = HashMap::new();
    for j in &robot.joints {
        children_of.entry(j.parent.link.as_str()).or_default().push(j);
    }

    // BFS topological parent-first order starting from pelvis_link.
    let mut ordered_links: Vec<(&str, Option<&urdf_rs::Joint>, usize)> = Vec::new();
    let mut link_to_idx: HashMap<&str, usize> = HashMap::new();
    let mut queue = std::collections::VecDeque::new();
    link_to_idx.insert(spec.pelvis_link.as_str(), 0);
    ordered_links.push((spec.pelvis_link.as_str(), None, 0));
    queue.push_back(spec.pelvis_link.as_str());

    while let Some(parent_name) = queue.pop_front() {
        let parent_idx = link_to_idx[parent_name];
        if let Some(joints) = children_of.get(parent_name) {
            for j in joints {
                let child = j.child.link.as_str();
                if !link_to_idx.contains_key(child) {
                    let idx = ordered_links.len();
                    link_to_idx.insert(child, idx);
                    ordered_links.push((child, Some(j), parent_idx));
                    queue.push_back(child);
                }
            }
        }
    }

    // Map URDF joint name -> (actuator_index, mimic_scale)
    let mut joint_to_act: HashMap<&str, (u8, f32)> = HashMap::new();
    let mut joint_by_name: HashMap<&str, &urdf_rs::Joint> = HashMap::new();
    for j in &robot.joints {
        joint_by_name.insert(j.name.as_str(), j);
    }
    for (i, act) in spec.actuators.iter().enumerate() {
        joint_to_act.insert(act.urdf_joint.as_str(), (i as u8, act.primary_mimic_scale));
        if let Some(cj) = &act.coupled_joint {
            joint_to_act.insert(cj.as_str(), (i as u8, act.coupled_mimic_scale));
        }
    }

    let mut total_mass = 0.0_f64;
    let mut links_code = String::new();
    let mut kin_code = String::new();

    for (link_name, joint_opt, parent_idx) in &ordered_links {
        let link = link_map
            .get(link_name)
            .ok_or_else(|| runner::Error::Config(format!("missing link {link_name}")))?;
        let mass = if link.inertial.mass.value > 0.0 {
            link.inertial.mass.value as f32
        } else {
            0.25_f32
        };
        let com = [
            link.inertial.origin.xyz[0] as f32,
            link.inertial.origin.xyz[1] as f32,
            link.inertial.origin.xyz[2] as f32,
        ];
        total_mass += mass as f64;

        let (act_idx, mimic_scale, axis, origin) = if let Some(j) = joint_opt {
            let (act, scale) = joint_to_act
                .get(j.name.as_str())
                .map(|&(a, s)| (Some(a), s))
                .unwrap_or((None, 1.0_f32));
            (
                act,
                scale,
                [
                    j.axis.xyz[0] as f32,
                    j.axis.xyz[1] as f32,
                    j.axis.xyz[2] as f32,
                ],
                [
                    j.origin.xyz[0] as f32,
                    j.origin.xyz[1] as f32,
                    j.origin.xyz[2] as f32,
                ],
            )
        } else {
            (None, 1.0_f32, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        };

        let act_str = match act_idx {
            Some(a) => format!("Some({a})"),
            None => "None".to_string(),
        };
        let _ = writeln!(
            links_code,
            "    LinkMass {{ name: \"{link_name}\", parent: {parent_idx}, joint_index: {act_str}, com: [{}, {}, {}], mass: {} }},",
            fmt_f32(com[0]), fmt_f32(com[1]), fmt_f32(com[2]), fmt_f32(mass),
        );
        let _ = writeln!(
            kin_code,
            "    JointKin {{ axis: [{}, {}, {}], origin: [{}, {}, {}], actuator_index: {act_str}, mimic_scale: {} }},",
            fmt_f32(axis[0]), fmt_f32(axis[1]), fmt_f32(axis[2]),
            fmt_f32(origin[0]), fmt_f32(origin[1]), fmt_f32(origin[2]),
            fmt_f32(mimic_scale),
        );
    }

    let total_mass_f32 = total_mass as f32;
    let inv_total_mass_f32 = (1.0_f64 / total_mass) as f32;
    let link_count = ordered_links.len();
    let act_count = spec.actuators.len();
    let urdf_revolute_count = robot
        .joints
        .iter()
        .filter(|j| !matches!(j.joint_type, urdf_rs::JointType::Fixed))
        .count();

    let mut roles_code = String::new();
    for (i, act) in spec.actuators.iter().enumerate() {
        if let Some(role) = &act.role {
            let _ = writeln!(roles_code, "    {role}: Some({i}),");
        }
    }

    let mut ratios_code = String::from("const MOTOR_TO_JOINT_RATIO: [f32; MAX_JOINTS] = [\n");
    let mut nm_per_amp_code = String::from("const NM_PER_AMP: [f32; MAX_JOINTS] = [\n");
    let mut max_torque_code = String::from("const MAX_TORQUE_NM: [f32; MAX_JOINTS] = [\n");
    let mut max_vel_code = String::from("const MAX_VELOCITY_RAD_S: [f32; MAX_JOINTS] = [\n");
    let mut endpoints_code = String::from("const SERVO_ENDPOINTS: [ServoEndpoint; MAX_JOINTS] = [\n");
    let mut limits_code = String::from("const DEFAULT_LIMITS: [[f32; 2]; MAX_JOINTS] = [\n");

    for i in 0..36 {
        if let Some(act) = spec.actuators.get(i) {
            let j = joint_by_name
                .get(act.urdf_joint.as_str())
                .ok_or_else(|| runner::Error::Config(format!("missing URDF joint {}", act.urdf_joint)))?;
            let a = (j.limit.lower as f32) / act.primary_mimic_scale;
            let b = (j.limit.upper as f32) / act.primary_mimic_scale;
            let (lo, hi) = if a <= b { (a, b) } else { (b, a) };
            let _ = writeln!(ratios_code, "    {},", fmt_f32(act.motor_to_joint_ratio));
            let _ = writeln!(nm_per_amp_code, "    {},", fmt_f32(act.nm_per_amp));
            let _ = writeln!(max_torque_code, "    {},", fmt_f32(act.max_torque_nm));
            let _ = writeln!(max_vel_code, "    {},", fmt_f32(act.max_velocity_rad_s));
            let _ = writeln!(
                endpoints_code,
                "    ServoEndpoint {{ bus_id: {}, node_addr: 0x{:02X} }},",
                act.bus_id, act.node_addr
            );
            let _ = writeln!(limits_code, "    [{}, {}],", fmt_f32(lo), fmt_f32(hi));
        } else {
            let _ = writeln!(ratios_code, "    1.0_f32,");
            let _ = writeln!(nm_per_amp_code, "    0.9_f32,");
            let _ = writeln!(max_torque_code, "    1.8_f32,");
            let _ = writeln!(max_vel_code, "    8.0_f32,");
            let _ = writeln!(endpoints_code, "    ServoEndpoint {{ bus_id: 0, node_addr: 0x{:02X} }},", 0x20 + i);
            let _ = writeln!(limits_code, "    [-core::f32::consts::PI, core::f32::consts::PI],");
        }
    }
    ratios_code.push_str("];\n");
    nm_per_amp_code.push_str("];\n");
    max_torque_code.push_str("];\n");
    max_vel_code.push_str("];\n");
    endpoints_code.push_str("];\n");
    limits_code.push_str("];\n");

    let lf_idx = spec
        .left_foot_link
        .as_deref()
        .and_then(|n| link_to_idx.get(n).copied())
        .map(|i| format!("Some({i})"))
        .unwrap_or_else(|| "None".to_string());
    let rf_idx = spec
        .right_foot_link
        .as_deref()
        .and_then(|n| link_to_idx.get(n).copied())
        .map(|i| format!("Some({i})"))
        .unwrap_or_else(|| "None".to_string());
    let lt_idx = spec
        .left_toe_link
        .as_deref()
        .and_then(|n| link_to_idx.get(n).copied())
        .map(|i| format!("Some({i})"))
        .unwrap_or_else(|| "None".to_string());
    let rt_idx = spec
        .right_toe_link
        .as_deref()
        .and_then(|n| link_to_idx.get(n).copied())
        .map(|i| format!("Some({i})"))
        .unwrap_or_else(|| "None".to_string());
    let imu_idx = link_to_idx
        .get(spec.imu_link.as_str())
        .copied()
        .ok_or_else(|| runner::Error::Config(format!("missing imu_link {}", spec.imu_link)))?;

    let lg = &spec.leg_geometry;
    let out = format!(
        r#"// @generated by `cargo run -p xtask -- codegen-robots`. Do not edit.

use rk_proto::MAX_JOINTS;

use crate::{{JointIndices, JointKin, LegGeometry, LinkMass, RobotProfile, ServoEndpoint, TorqueConstants}};

pub const TOTAL_MASS_KG: f32 = {total_mass};
pub const INV_TOTAL_MASS: f32 = {inv_total_mass};

pub const LINKS: [LinkMass; {link_count}] = [
{links_code}];

pub const JOINT_KIN: [JointKin; {link_count}] = [
{kin_code}];

pub const INDICES: JointIndices = JointIndices {{
{roles_code}    ..JointIndices::empty()
}};

{ratios_code}
{nm_per_amp_code}
{max_torque_code}
{max_vel_code}
{endpoints_code}
{limits_code}
pub const PROFILE: RobotProfile = RobotProfile {{
    name: "{name}",
    joint_count: {act_count},
    urdf_joint_count: {urdf_revolute_count},
    joint_indices: INDICES,
    links: &LINKS,
    joint_kin: &JOINT_KIN,
    total_mass_kg: TOTAL_MASS_KG,
    inv_total_mass: INV_TOTAL_MASS,
    pelvis_index: 0,
    left_foot_index: {lf_idx},
    right_foot_index: {rf_idx},
    left_toe_index: {lt_idx},
    right_toe_index: {rt_idx},
    imu_link_index: {imu_idx},
    imu_mounting_rpy: [{rpy0}, {rpy1}, {rpy2}],
    z_up: {z_up},
    motor_to_joint_ratio: MOTOR_TO_JOINT_RATIO,
    leg_geometry: LegGeometry {{
        thigh_len: {thigh},
        shin_len: {shin},
        hip_ankle_offset: {hao},
        half_hip_spacing: {hhs},
        nominal_height: {nom_h},
        min_height: {min_h},
        max_reach: {max_r},
        max_dx: {mdx},
        max_dy: {mdy},
        max_swing_dx: {msdx},
        max_swing_dy: {msdy},
        max_foot_lift: {mfl},
        max_sway: {msw},
    }},
    foot_half_length_m: {fhl},
    foot_margin_m: {fm},
    h_com_m: {hcom},
    torque_constants: TorqueConstants {{ nm_per_amp: NM_PER_AMP }},
    max_torque_nm: MAX_TORQUE_NM,
    max_velocity_rad_s: MAX_VELOCITY_RAD_S,
    servo_endpoints: SERVO_ENDPOINTS,
    default_limits: DEFAULT_LIMITS,
}};
"#,
        total_mass = fmt_f32(total_mass_f32),
        inv_total_mass = fmt_f32(inv_total_mass_f32),
        name = spec.name,
        rpy0 = fmt_f32(spec.imu_mounting_rpy[0]),
        rpy1 = fmt_f32(spec.imu_mounting_rpy[1]),
        rpy2 = fmt_f32(spec.imu_mounting_rpy[2]),
        z_up = spec.z_up,
        thigh = fmt_f32(lg.thigh_len),
        shin = fmt_f32(lg.shin_len),
        hao = fmt_f32(lg.hip_ankle_offset),
        hhs = fmt_f32(lg.half_hip_spacing),
        nom_h = fmt_f32(lg.nominal_height),
        min_h = fmt_f32(lg.min_height),
        max_r = fmt_f32(lg.max_reach),
        mdx = fmt_f32(lg.max_dx),
        mdy = fmt_f32(lg.max_dy),
        msdx = fmt_f32(lg.max_swing_dx),
        msdy = fmt_f32(lg.max_swing_dy),
        mfl = fmt_f32(lg.max_foot_lift),
        msw = fmt_f32(lg.max_sway),
        fhl = fmt_f32(spec.foot_half_length_m),
        fm = fmt_f32(spec.foot_margin_m),
        hcom = fmt_f32(spec.h_com_m),
    );
    Ok(out)
}
