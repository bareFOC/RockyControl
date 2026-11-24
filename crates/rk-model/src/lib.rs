//! Layer 3 — GUI Domain Engine (`rk-model`).
//!
//! Organized into focused submodules:
//! - [`bundle`]: In-memory `AssetBundle`, `.zip` mesh resolution, and CRC32-verified MCU flash blobs
//! - [`kinematics`]: `urdf-rs` parsing, `k::Chain<f32>` FK evaluation, ROS Z-Up auto-detection, and grounding
//! - [`studio`]: Bidirectional mirroring authority state machine (`Teach`, `Drive`, `EStop`), keyframes, and playback
//! - [`strike_optimizer`]: Host-only offline impact-aware strike keyframe optimizer (never linked into firmware)
//! - [`samples`]: Built-in test URDF presets

pub mod bundle;
pub mod kinematics;
pub mod samples;
pub mod studio;
pub mod strike_optimizer;

pub use bundle::{AssetBundle, BUNDLED_URDF_XML, BundleDiagnostics, ModelSource};
pub use k::nalgebra;
pub use kinematics::{
    JointKind, JointMeta, KinematicRig, LimbGroup, LinkSummary, LinkVisual, UpAxis, VisualShape,
};
pub use studio::{
    ActionMove, CalibrationFile, Keyframe, PlaybackState, StudioModel, cubic_smoothstep,
    default_combat_library, sample_action_move,
};
pub use strike_optimizer::{
    BindingConstraint, OptimizedStrike, OptimizerError, StrikeExport, StrikeMetrics, StrikeParams,
    StrikeTarget, export_strikes_json, import_strikes_json, optimize_and_export_json,
    optimize_strike, optimize_strikes, optimized_strikes,
};

#[cfg(test)]
mod tests {
    use rk_proto::{ControlMode, ModelFormat, ModelInfo};

    use super::*;

    #[test]
    fn bundled_urdf_parses_and_evaluates_fk() {
        let rig = KinematicRig::from_urdf_xml(BUNDLED_URDF_XML).expect("parse bundled urdf");
        assert_eq!(rig.joints.len(), 21);
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

    #[test]
    fn ros_zup_urdf_auto_detects_up_axis_resolves_materials_and_grounds() {
        let rig = KinematicRig::from_urdf_xml(samples::STRIKER_BIPED_ZUP_URDF)
            .expect("parse ROS Z-up URDF");
        assert_eq!(rig.up_axis, UpAxis::ZUp);
        assert_eq!(rig.joints.len(), 8);
        assert!(rig.bounds_min[1].abs() < 1e-4, "robot must stand on floor y=0");
        assert!(rig.bounds_max[1] > 0.55, "robot height should be positive");
        let visor = rig
            .visuals
            .iter()
            .find(|v| v.link_name == "base_link" && v.rgba[2] > 0.85);
        assert!(visor.is_some(), "top-level material color must resolve");
    }

    #[test]
    fn prismatic_and_continuous_urdf_evaluates_fk() {
        let rig =
            KinematicRig::from_urdf_xml(samples::COMBAT_ARM_6DOF_URDF).expect("parse 6-DoF arm");
        assert_eq!(rig.joints.len(), 6);
        assert_eq!(rig.joints[4].kind, JointKind::Continuous);
        assert_eq!(rig.joints[5].kind, JointKind::Prismatic);

        let mut p0 = rig.default_pose();
        let t0 = rig.compute_link_transforms(&p0);
        p0[5] = 0.08;
        let t1 = rig.compute_link_transforms(&p0);
        let y0 = t0["gripper_jaw"].translation.y;
        let y1 = t1["gripper_jaw"].translation.y;
        assert!((y1 - y0 - 0.08).abs() < 1e-3, "prismatic joint must translate gripper");
    }

    #[test]
    fn zero_dof_static_urdf_loads_cleanly() {
        let xml = r#"<robot name="static_prop">
            <link name="chassis"><visual><geometry><box size="0.3 0.2 0.1"/></geometry></visual></link>
        </robot>"#;
        let mut model = StudioModel::new_default();
        let bundle = AssetBundle::from_bytes("static_prop.urdf", xml.as_bytes()).unwrap();
        model.load_bundle(bundle).expect("load 0-dof urdf");
        assert_eq!(model.rig.joints.len(), 0);
        assert_eq!(model.torque_mask, 0);
    }

    #[test]
    fn mcu_blob_xml_and_deflate_round_trip_with_crc32() {
        let bundle = AssetBundle::bundled_rocky();
        let raw = bundle.urdf_xml.as_bytes();
        let info_raw = ModelInfo::from_static("rocky_mk1", ModelFormat::UrdfXml, raw, 21);
        let parsed_raw = AssetBundle::from_mcu_blob(&info_raw, raw).expect("load raw mcu urdf");
        assert_eq!(parsed_raw.robot_name, "rocky_mk1");

        let deflated = bundle.to_deflated_urdf_bytes().expect("deflate urdf");
        assert!(deflated.len() < raw.len() / 3, "deflate should shrink URDF >3x");
        let info_def =
            ModelInfo::from_static("rocky_mk1", ModelFormat::UrdfDeflate, &deflated, 21);
        let parsed_def =
            AssetBundle::from_mcu_blob(&info_def, &deflated).expect("load deflated mcu urdf");
        assert_eq!(parsed_def.urdf_xml, bundle.urdf_xml);
    }

    #[test]
    fn resolve_mesh_bytes_handles_package_uri_and_basename() {
        let mut bundle = AssetBundle::bundled_rocky();
        bundle
            .files
            .insert("my_robot/meshes/visual/forearm.STL".to_string(), vec![1, 2, 3]);
        assert!(bundle
            .resolve_mesh_bytes("package://my_robot/meshes/visual/forearm.stl")
            .is_some());
        assert!(bundle
            .resolve_mesh_bytes("package://other_pkg/meshes/forearm.stl")
            .is_some());
    }
}
