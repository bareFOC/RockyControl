//! Layer 4 — Embedded 3D Mirror Viewport (`humanoid-viewport`).
//!
//! Renders inside `egui::PaintCallback` sharing `eframe`'s `Arc<glow::Context>` (`glow 0.17`):
//! - Solid PBR meshes tracking live `measured_rad` via `k::Chain<f32>`
//! - Translucent cyan wireframe ghost tracking `target_rad` in `Drive` mode
//! - Orange -> Red heat-tinting when joint `current_ma` > 70% rated current (1400 mA)
//! - Mouse & Android Tablet multi-touch orbit/pan/pinch-zoom + click/tap link picking

mod camera;
mod scene;

use std::cell::RefCell;
use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use glow::HasContext as _;
use humanoid_model::nalgebra::{Isometry3, Point3, Vector3};
use humanoid_model::{AssetBundle, KinematicRig, VisualShape};
use humanoid_proto::MAX_JOINTS;
use three_d::{ScissorBox, Viewport};

pub use camera::{CAMERA_FOV_DEG, CameraPreset, OrbitState};
pub use scene::ViewportFrameData;
use scene::SceneResources;

static NEXT_VIEWPORT_ID: AtomicU64 = AtomicU64::new(1);

thread_local! {
    static VIEWPORT_SCENES: RefCell<HashMap<u64, SceneResources>> = RefCell::new(HashMap::new());
}

/// Public handle held by `humanoid-ui` to render the embedded 3D mirror viewport.
pub struct HumanoidViewport {
    pub orbit: OrbitState,
    pub show_grid: bool,
    pub show_joint_markers: bool,
    pub pick_radius_scale: f32,
    viewport_id: u64,
    rig_generation: u64,
    last_error: Option<String>,
    pick_bounds: Vec<(String, Option<usize>, Isometry3<f32>, f32)>,
}

impl Drop for HumanoidViewport {
    fn drop(&mut self) {
        let id = self.viewport_id;
        VIEWPORT_SCENES.with(|scenes| {
            scenes.borrow_mut().remove(&id);
        });
    }
}

impl Default for HumanoidViewport {
    fn default() -> Self {
        Self::new()
    }
}

impl HumanoidViewport {
    pub fn new() -> Self {
        Self {
            orbit: OrbitState::default(),
            show_grid: true,
            show_joint_markers: true,
            pick_radius_scale: 1.0,
            viewport_id: NEXT_VIEWPORT_ID.fetch_add(1, Ordering::Relaxed),
            rig_generation: 1,
            last_error: None,
            pick_bounds: Vec::new(),
        }
    }

    pub fn notify_rig_changed(&mut self) {
        self.rig_generation = self.rig_generation.wrapping_add(1);
    }

    /// Resets view angle to isometric and frames the active URDF.
    pub fn fit_to_rig(&mut self, rig: &KinematicRig) {
        self.orbit.apply_preset_for_bounds(
            CameraPreset::Iso,
            rig.bounds_center(),
            rig.bounds_radius(),
        );
    }

    pub fn apply_preset_for_rig(&mut self, preset: CameraPreset, rig: &KinematicRig) {
        self.orbit
            .apply_preset_for_bounds(preset, rig.bounds_center(), rig.bounds_radius());
    }

    pub fn last_error(&self) -> Option<&str> {
        self.last_error.as_deref()
    }

    /// Shows the 3D viewport inside the given `egui::Ui` and returns `Some(joint_index)` if the user clicked/tapped a link.
    pub fn show(
        &mut self,
        ui: &mut egui::Ui,
        rig: &KinematicRig,
        bundle: &AssetBundle,
        measured_rad: &[f32; MAX_JOINTS],
        target_rad: &[f32; MAX_JOINTS],
        current_ma: &[u16; MAX_JOINTS],
        selected_joint: usize,
        show_ghost: bool,
        estop_active: bool,
    ) -> Option<usize> {
        let desired_size = ui.available_size();
        let size = egui::vec2(desired_size.x.max(120.0), desired_size.y.max(120.0));
        let (rect, response) = ui.allocate_exact_size(size, egui::Sense::click_and_drag());

        // Single-pointer orbit rotation (mouse drag or touch drag). Zoom/pan are intentionally disabled
        // so the full robot stays centered and fitted to view at all times.
        if response.dragged_by(egui::PointerButton::Primary) {
            let delta = response.drag_delta();
            self.orbit.yaw -= delta.x * 0.009;
            self.orbit.pitch = (self.orbit.pitch + delta.y * 0.008).clamp(-0.65, 1.48);
        }

        // Always fit the robot's bounding sphere cleanly inside the current viewport aspect ratio
        let aspect = (rect.width() / rect.height().max(1.0)).max(0.2);
        self.orbit.fit_to_bounds(
            rig.bounds_center(),
            rig.bounds_radius(),
            aspect,
            CAMERA_FOV_DEG,
        );

        let measured_transforms = rig.compute_link_transforms(measured_rad);
        let target_transforms = if show_ghost {
            rig.compute_link_transforms(target_rad)
        } else {
            HashMap::new()
        };

        // Collect live world-space positions of movable joints for pivot markers
        let mut joint_positions = Vec::with_capacity(rig.joints.len());
        for j in &rig.joints {
            if let Some(iso) = measured_transforms.get(&j.child_link) {
                let t = iso.translation.vector;
                joint_positions.push((j.index, [t.x, t.y, t.z]));
            }
        }

        // Update link pick centers for click/tap-to-select (widened gently on touchscreens)
        let pick_scale = self.pick_radius_scale.clamp(0.8, 2.0);
        self.pick_bounds.clear();
        for vis in &rig.visuals {
            if let Some(iso) = measured_transforms.get(&vis.link_name) {
                let world = iso * vis.local_offset;
                let radius = match vis.shape {
                    VisualShape::Box(s) => (s[0].max(s[1]).max(s[2]) * 0.58).max(0.05),
                    VisualShape::Cylinder { radius, length } => {
                        (radius.max(length * 0.5)).max(0.05)
                    }
                    VisualShape::Sphere { radius } => radius.max(0.05),
                    VisualShape::Mesh { scale, .. } => {
                        (scale[0].abs().max(scale[1].abs()).max(scale[2].abs()) * 0.12)
                            .clamp(0.06, 0.35)
                    }
                } * pick_scale;
                self.pick_bounds
                    .push((vis.link_name.clone(), vis.joint_index, world, radius));
            }
        }

        // Handle click/tap-to-select link picking via ray-sphere intersection
        let mut picked_joint = None;
        if response.clicked() {
            if let Some(pointer_pos) = response.interact_pointer_pos() {
                picked_joint = self.pick_joint_at(pointer_pos, rect);
            }
        }

        let marker_radius = (rig.bounds_radius() * 0.022).clamp(0.010, 0.032);
        let frame_data = ViewportFrameData {
            measured_transforms,
            target_transforms,
            joint_positions,
            show_ghost,
            show_grid: self.show_grid,
            show_joint_markers: self.show_joint_markers,
            selected_joint,
            current_ma: *current_ma,
            estop_active,
            orbit: self.orbit.clone(),
            marker_radius,
        };

        let viewport_id = self.viewport_id;
        let rig_name = rig.robot_name.clone();
        let up_axis = rig.up_axis;
        let urdf_xml = bundle.urdf_xml.clone();
        let bundle_files = bundle.files.clone();
        let bundle_source = bundle.source.clone();
        let generation = self.rig_generation;

        let callback = egui_glow::CallbackFn::new(move |info, painter| {
            let gl = painter.gl().clone();
            VIEWPORT_SCENES.with(|scenes_cell| {
                let mut scenes = scenes_cell.borrow_mut();

                if !scenes.contains_key(&viewport_id) {
                    let bundle_copy = AssetBundle {
                        robot_name: rig_name.clone(),
                        urdf_xml: urdf_xml.clone(),
                        files: bundle_files.clone(),
                        source: bundle_source.clone(),
                    };
                    if let Ok(temp_rig) =
                        KinematicRig::from_urdf_xml_with_up(&bundle_copy.urdf_xml, Some(up_axis))
                    {
                        if let Ok(res) =
                            SceneResources::new(gl.clone(), &temp_rig, &bundle_copy, generation)
                        {
                            scenes.insert(viewport_id, res);
                        }
                    }
                } else if let Some(res) = scenes.get_mut(&viewport_id) {
                    if res.rig_generation != generation {
                        let bundle_copy = AssetBundle {
                            robot_name: rig_name.clone(),
                            urdf_xml: urdf_xml.clone(),
                            files: bundle_files.clone(),
                            source: bundle_source.clone(),
                        };
                        if let Ok(temp_rig) = KinematicRig::from_urdf_xml_with_up(
                            &bundle_copy.urdf_xml,
                            Some(up_axis),
                        ) {
                            res.rebuild_rig(&temp_rig, &bundle_copy, generation);
                        }
                    }
                }

                if let Some(res) = scenes.get_mut(&viewport_id) {
                    let vp = info.viewport_in_pixels();
                    let clip = info.clip_rect_in_pixels();
                    let viewport = Viewport {
                        x: vp.left_px,
                        y: vp.from_bottom_px,
                        width: vp.width_px.max(1) as u32,
                        height: vp.height_px.max(1) as u32,
                    };
                    let x0 = vp.left_px.max(clip.left_px);
                    let y0 = vp.from_bottom_px.max(clip.from_bottom_px);
                    let x1 = (vp.left_px + vp.width_px).min(clip.left_px + clip.width_px);
                    let y1 = (vp.from_bottom_px + vp.height_px)
                        .min(clip.from_bottom_px + clip.height_px);
                    let scissor = ScissorBox {
                        x: x0,
                        y: y0,
                        width: (x1 - x0).max(1) as u32,
                        height: (y1 - y0).max(1) as u32,
                    };
                    let screen_w = info.screen_size_px[0].max(1);
                    let screen_h = info.screen_size_px[1].max(1);

                    res.render(viewport, scissor, screen_w, screen_h, &frame_data);

                    // Cleanly restore GL state before handing control back to egui_glow
                    unsafe {
                        gl.bind_vertex_array(None);
                        gl.disable(glow::DEPTH_TEST);
                        gl.disable(glow::CULL_FACE);
                    }
                }
            });
        });

        ui.painter().add(egui::PaintCallback {
            rect,
            callback: Arc::new(callback),
        });

        picked_joint
    }

    fn pick_joint_at(&self, pos: egui::Pos2, rect: egui::Rect) -> Option<usize> {
        let nx = ((pos.x - rect.left()) / rect.width().max(1.0)) * 2.0 - 1.0;
        let ny = 1.0 - ((pos.y - rect.top()) / rect.height().max(1.0)) * 2.0;
        let aspect = (rect.width() / rect.height().max(1.0)).max(0.1);
        let fov_rad = CAMERA_FOV_DEG.to_radians();
        let tan_half = (fov_rad * 0.5).tan();

        let eye_v = self.orbit.eye_position();
        let tgt_v = self.orbit.target_vec();
        let eye = Point3::new(eye_v.x, eye_v.y, eye_v.z);
        let target = Point3::new(tgt_v.x, tgt_v.y, tgt_v.z);
        let forward = (target - eye).normalize();
        let right = forward.cross(&Vector3::y()).normalize();
        let up = right.cross(&forward).normalize();

        let ray_dir =
            (forward + right * (nx * aspect * tan_half) + up * (ny * tan_half)).normalize();

        let mut best_t = f32::INFINITY;
        let mut best_joint = None;

        for (_, joint_idx, iso, radius) in &self.pick_bounds {
            let Some(j_idx) = joint_idx else {
                continue;
            };
            let center = Point3::new(iso.translation.x, iso.translation.y, iso.translation.z);
            let oc = eye - center;
            let b = oc.dot(&ray_dir);
            let c = oc.dot(&oc) - radius * radius;
            let disc = b * b - c;
            if disc >= 0.0 {
                let t = -b - disc.sqrt();
                if t > 0.05 && t < best_t {
                    best_t = t;
                    best_joint = Some(*j_idx);
                }
            }
        }
        best_joint
    }
}
