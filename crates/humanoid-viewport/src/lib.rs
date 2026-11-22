//! Layer 4 — Embedded 3D Mirror Viewport (`humanoid-viewport`).
//!
//! Renders inside `egui::PaintCallback` sharing `eframe`'s `Arc<glow::Context>` (`glow 0.17`):
//! - Solid PBR meshes tracking live `measured_rad` via `k::Chain<f32>`
//! - Translucent cyan wireframe ghost tracking `target_rad` in `Drive` mode
//! - Orange -> Red heat-tinting when joint `current_ma` > 70% rated current (1400 mA)
//! - Orbit / pan / zoom camera + presets + click-to-select link picking

use std::cell::RefCell;
use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use glow::HasContext as _;
use humanoid_model::nalgebra::{Isometry3, Matrix4, Point3, Vector3};
use humanoid_model::{AssetBundle, KinematicRig, LinkVisual, VisualShape};
use humanoid_proto::{HEAT_TINT_THRESHOLD_MA, MAX_JOINTS, RATED_CURRENT_MA};
use three_d::{
    AmbientLight, Blend, Camera, ClearState, ColorMaterial, Context, CpuMaterial, CpuMesh, Cull,
    DepthTest, DirectionalLight, Gm, Mat4, Mesh, Object, PhysicalMaterial, RenderStates,
    RenderTarget, ScissorBox, Srgba, Vec3, Viewport, WriteMask, degrees, vec3,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CameraPreset {
    Iso,
    Front,
    Side,
    Top,
}

#[derive(Clone, Debug)]
pub struct OrbitState {
    pub yaw: f32,
    pub pitch: f32,
    pub distance: f32,
    pub target: [f32; 3],
}

impl Default for OrbitState {
    fn default() -> Self {
        Self {
            yaw: 0.48,
            pitch: 0.24,
            distance: 2.35,
            target: [0.0, 0.78, 0.0],
        }
    }
}

impl OrbitState {
    pub fn apply_preset(&mut self, preset: CameraPreset) {
        self.target = [0.0, 0.78, 0.0];
        match preset {
            CameraPreset::Iso => {
                self.yaw = 0.48;
                self.pitch = 0.24;
                self.distance = 2.35;
            }
            CameraPreset::Front => {
                self.yaw = 0.0;
                self.pitch = 0.12;
                self.distance = 2.30;
            }
            CameraPreset::Side => {
                self.yaw = std::f32::consts::FRAC_PI_2;
                self.pitch = 0.14;
                self.distance = 2.30;
            }
            CameraPreset::Top => {
                self.yaw = 0.0;
                self.pitch = 1.38;
                self.distance = 2.65;
            }
        }
    }

    pub fn eye_position(&self) -> Vec3 {
        let cos_p = self.pitch.cos();
        let x = self.target[0] + self.distance * cos_p * self.yaw.sin();
        let y = self.target[1] + self.distance * self.pitch.sin();
        let z = self.target[2] + self.distance * cos_p * self.yaw.cos();
        vec3(x, y, z)
    }

    pub fn target_vec(&self) -> Vec3 {
        vec3(self.target[0], self.target[1], self.target[2])
    }
}

/// Snapshot passed each frame into the `PaintCallback` to render the 3D scene.
#[derive(Clone)]
pub struct ViewportFrameData {
    pub measured_transforms: HashMap<String, Isometry3<f32>>,
    pub target_transforms: HashMap<String, Isometry3<f32>>,
    pub show_ghost: bool,
    pub show_grid: bool,
    pub selected_joint: usize,
    pub current_ma: [u16; MAX_JOINTS],
    pub estop_active: bool,
    pub orbit: OrbitState,
}

struct SolidVisualItem {
    link_name: String,
    joint_index: Option<usize>,
    local_offset: Isometry3<f32>,
    base_rgba: [f32; 4],
    mesh: Gm<Mesh, PhysicalMaterial>,
    ghost_mesh: Gm<Mesh, ColorMaterial>,
    ghost_wire: Gm<Mesh, ColorMaterial>,
}

static NEXT_VIEWPORT_ID: AtomicU64 = AtomicU64::new(1);

thread_local! {
    static VIEWPORT_SCENES: RefCell<HashMap<u64, SceneResources>> = RefCell::new(HashMap::new());
}

struct SceneResources {
    td_ctx: Context,
    items: Vec<SolidVisualItem>,
    floor_grid: Vec<Gm<Mesh, ColorMaterial>>,
    floor_plate: Gm<Mesh, PhysicalMaterial>,
    key_light: DirectionalLight,
    fill_light: DirectionalLight,
    rim_light: DirectionalLight,
    ambient_light: AmbientLight,
    rig_generation: u64,
}

impl SceneResources {
    fn new(gl: Arc<glow::Context>, rig: &KinematicRig, bundle: &AssetBundle, generation: u64) -> Result<Self, String> {
        let td_ctx = Context::from_gl_context(gl)
            .map_err(|e| format!("Failed to wrap glow context in three-d: {e:?}"))?;

        let items = build_visual_items(&td_ctx, rig, bundle);
        let floor_grid = build_floor_grid(&td_ctx);

        // Grounding studio platform disc under the humanoid (cylinder axis +X -> rotate +90° Z to +Y)
        let mut disc_cpu = CpuMesh::cylinder(48);
        let disc_mat = Mat4::from_translation(vec3(0.0, -0.014, 0.0))
            * Mat4::from_angle_z(degrees(90.0))
            * Mat4::from_nonuniform_scale(0.012, 0.85, 0.85);
        disc_cpu.transform(disc_mat).ok();
        disc_cpu.compute_normals();
        let floor_plate = Gm::new(
            Mesh::new(&td_ctx, &disc_cpu),
            PhysicalMaterial::new_opaque(
                &td_ctx,
                &CpuMaterial {
                    albedo: Srgba::new(22, 28, 38, 255),
                    roughness: 0.78,
                    metallic: 0.15,
                    ..Default::default()
                },
            ),
        );

        // 3-point warm/cool robotics studio lighting
        let key_light =
            DirectionalLight::new(&td_ctx, 2.3, Srgba::new(255, 245, 232, 255), vec3(-1.4, -2.2, -1.6));
        let fill_light =
            DirectionalLight::new(&td_ctx, 1.0, Srgba::new(175, 210, 255, 255), vec3(1.8, -0.8, 1.0));
        let rim_light =
            DirectionalLight::new(&td_ctx, 1.25, Srgba::new(110, 215, 245, 255), vec3(0.3, -0.7, 2.0));
        let ambient_light = AmbientLight::new(&td_ctx, 0.45, Srgba::new(195, 210, 230, 255));

        Ok(Self {
            td_ctx,
            items,
            floor_grid,
            floor_plate,
            key_light,
            fill_light,
            rim_light,
            ambient_light,
            rig_generation: generation,
        })
    }

    fn rebuild_rig(&mut self, rig: &KinematicRig, bundle: &AssetBundle, generation: u64) {
        self.items = build_visual_items(&self.td_ctx, rig, bundle);
        self.rig_generation = generation;
    }

    fn render(
        &mut self,
        viewport: Viewport,
        scissor: ScissorBox,
        screen_w: u32,
        screen_h: u32,
        frame: &ViewportFrameData,
    ) {
        let mut camera = Camera::new_perspective(
            viewport,
            frame.orbit.eye_position(),
            frame.orbit.target_vec(),
            vec3(0.0, 1.0, 0.0),
            degrees(42.0),
            0.05,
            30.0,
        );
        camera.set_viewport(viewport);

        // Update solid measured transforms + current heat-tinting + selection highlight
        for item in &mut self.items {
            if let Some(iso) = frame.measured_transforms.get(&item.link_name) {
                let world = iso * item.local_offset;
                item.mesh.set_transformation(iso_to_mat4(&world));
            }

            let mut r = item.base_rgba[0];
            let mut g = item.base_rgba[1];
            let mut b = item.base_rgba[2];
            let mut emissive = Srgba::BLACK;

            if let Some(j_idx) = item.joint_index {
                let ma = frame.current_ma[j_idx];
                if ma >= HEAT_TINT_THRESHOLD_MA {
                    let t = ((ma - HEAT_TINT_THRESHOLD_MA) as f32
                        / (RATED_CURRENT_MA - HEAT_TINT_THRESHOLD_MA).max(1) as f32)
                        .clamp(0.0, 1.0);
                    // Orange (245, 158, 11) -> Crimson Red (239, 58, 58)
                    let hr = 0.96;
                    let hg = 0.62 * (1.0 - t) + 0.16 * t;
                    let hb = 0.06 * (1.0 - t) + 0.14 * t;
                    let blend = 0.55 + 0.45 * t;
                    r = r * (1.0 - blend) + hr * blend;
                    g = g * (1.0 - blend) + hg * blend;
                    b = b * (1.0 - blend) + hb * blend;
                    emissive = Srgba::new(
                        (115.0 * (0.4 + 0.6 * t)) as u8,
                        (35.0 * (1.0 - t)) as u8,
                        12,
                        255,
                    );
                } else if j_idx == frame.selected_joint {
                    // Subtle cyan selection highlight
                    r = (r * 0.65 + 0.08 * 0.35).clamp(0.0, 1.0);
                    g = (g * 0.65 + 0.78 * 0.35).clamp(0.0, 1.0);
                    b = (b * 0.65 + 0.94 * 0.35).clamp(0.0, 1.0);
                    emissive = Srgba::new(10, 52, 68, 255);
                }
            }

            item.mesh.material.albedo = Srgba::new(
                (r.clamp(0.0, 1.0) * 255.0) as u8,
                (g.clamp(0.0, 1.0) * 255.0) as u8,
                (b.clamp(0.0, 1.0) * 255.0) as u8,
                255,
            );
            item.mesh.material.emissive = emissive;

            if frame.show_ghost {
                if let Some(iso) = frame.target_transforms.get(&item.link_name) {
                    let world = iso * item.local_offset;
                    let m = iso_to_mat4(&world);
                    item.ghost_mesh.set_transformation(m);
                    item.ghost_wire.set_transformation(m);
                }
            }
        }

        let clear = if frame.estop_active {
            ClearState::color_and_depth(0.095, 0.052, 0.060, 1.0, 1.0)
        } else {
            ClearState::color_and_depth(0.055, 0.070, 0.098, 1.0, 1.0)
        };

        let lights: [&dyn three_d::Light; 4] = [
            &self.key_light,
            &self.fill_light,
            &self.rim_light,
            &self.ambient_light,
        ];

        let target = RenderTarget::screen(&self.td_ctx, screen_w, screen_h);
        target.clear_partially(scissor, clear);

        let mut opaque_objs: Vec<&dyn Object> = Vec::with_capacity(self.items.len() + 2);
        opaque_objs.push(&self.floor_plate);
        if frame.show_grid {
            for line in &self.floor_grid {
                opaque_objs.push(line);
            }
        }
        for item in &self.items {
            opaque_objs.push(&item.mesh);
        }

        target.render_partially(scissor, &camera, opaque_objs, &lights);

        if frame.show_ghost {
            let mut ghost_objs: Vec<&dyn Object> = Vec::with_capacity(self.items.len() * 2);
            for item in &self.items {
                ghost_objs.push(&item.ghost_mesh);
                ghost_objs.push(&item.ghost_wire);
            }
            target.render_partially(scissor, &camera, ghost_objs, &[]);
        }
    }
}

/// Public handle held by `humanoid-ui` to render the embedded 3D mirror viewport.
pub struct HumanoidViewport {
    pub orbit: OrbitState,
    pub show_grid: bool,
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
            viewport_id: NEXT_VIEWPORT_ID.fetch_add(1, Ordering::Relaxed),
            rig_generation: 1,
            last_error: None,
            pick_bounds: Vec::new(),
        }
    }

    pub fn notify_rig_changed(&mut self) {
        self.rig_generation = self.rig_generation.wrapping_add(1);
    }

    pub fn last_error(&self) -> Option<&str> {
        self.last_error.as_deref()
    }

    /// Shows the 3D viewport inside the given `egui::Ui` and returns `Some(joint_index)` if the user clicked a link.
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

        // Handle orbit / pan / zoom pointer interactions
        if response.dragged_by(egui::PointerButton::Primary) {
            let delta = response.drag_delta();
            self.orbit.yaw -= delta.x * 0.009;
            self.orbit.pitch = (self.orbit.pitch + delta.y * 0.008).clamp(-0.65, 1.48);
        }
        if response.dragged_by(egui::PointerButton::Secondary)
            || response.dragged_by(egui::PointerButton::Middle)
        {
            let delta = response.drag_delta();
            let right_x = self.orbit.yaw.cos();
            let right_z = -self.orbit.yaw.sin();
            let pan_scale = self.orbit.distance * 0.0018;
            self.orbit.target[0] -= delta.x * right_x * pan_scale;
            self.orbit.target[2] -= delta.x * right_z * pan_scale;
            self.orbit.target[1] = (self.orbit.target[1] + delta.y * pan_scale).clamp(0.1, 2.0);
        }
        if response.hovered() {
            let scroll = ui.ctx().input(|i| i.smooth_scroll_delta.y);
            if scroll.abs() > 0.01 {
                self.orbit.distance = (self.orbit.distance * (1.0 - scroll * 0.0018)).clamp(0.85, 6.5);
            }
        }

        let measured_transforms = rig.compute_link_transforms(measured_rad);
        let target_transforms = if show_ghost {
            rig.compute_link_transforms(target_rad)
        } else {
            HashMap::new()
        };

        // Update link pick centers for click-to-select
        self.pick_bounds.clear();
        for vis in &rig.visuals {
            if let Some(iso) = measured_transforms.get(&vis.link_name) {
                let world = iso * vis.local_offset;
                let radius = match vis.shape {
                    VisualShape::Box(s) => (s[0].max(s[1]).max(s[2]) * 0.58).max(0.06),
                    VisualShape::Cylinder { radius, length } => (radius.max(length * 0.5)).max(0.06),
                    VisualShape::Sphere { radius } => radius.max(0.06),
                    VisualShape::Mesh { .. } => 0.12,
                };
                self.pick_bounds
                    .push((vis.link_name.clone(), vis.joint_index, world, radius));
            }
        }

        // Handle click-to-select link picking via ray-sphere intersection
        let mut picked_joint = None;
        if response.clicked() {
            if let Some(pointer_pos) = response.interact_pointer_pos() {
                picked_joint = self.pick_joint_at(pointer_pos, rect);
            }
        }

        let frame_data = ViewportFrameData {
            measured_transforms,
            target_transforms,
            show_ghost,
            show_grid: self.show_grid,
            selected_joint,
            current_ma: *current_ma,
            estop_active,
            orbit: self.orbit.clone(),
        };

        let viewport_id = self.viewport_id;
        let rig_name = rig.robot_name.clone();
        let urdf_xml = bundle.urdf_xml.clone();
        let bundle_files = bundle.files.clone();
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
                    };
                    if let Ok(temp_rig) = KinematicRig::from_urdf_xml(&bundle_copy.urdf_xml) {
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
                        };
                        if let Ok(temp_rig) = KinematicRig::from_urdf_xml(&bundle_copy.urdf_xml) {
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
                    let scissor = ScissorBox {
                        x: clip.left_px,
                        y: clip.from_bottom_px,
                        width: clip.width_px.max(1) as u32,
                        height: clip.height_px.max(1) as u32,
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
        let fov_rad = 42.0_f32.to_radians();
        let tan_half = (fov_rad * 0.5).tan();

        let eye_v = self.orbit.eye_position();
        let tgt_v = self.orbit.target_vec();
        let eye = Point3::new(eye_v.x, eye_v.y, eye_v.z);
        let target = Point3::new(tgt_v.x, tgt_v.y, tgt_v.z);
        let forward = (target - eye).normalize();
        let right = forward.cross(&Vector3::y()).normalize();
        let up = right.cross(&forward).normalize();

        let ray_dir = (forward + right * (nx * aspect * tan_half) + up * (ny * tan_half)).normalize();

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

fn build_visual_items(
    ctx: &Context,
    rig: &KinematicRig,
    bundle: &AssetBundle,
) -> Vec<SolidVisualItem> {
    let mut items = Vec::with_capacity(rig.visuals.len());
    for vis in &rig.visuals {
        let (solid_cpu, wire_cpu, _pick_radius) = build_link_meshes(vis, bundle);

        let is_accent = vis.link_name.contains("forearm") || vis.link_name.contains("shoulder");
        let is_metal_joint = vis.link_name.contains("hip") || vis.link_name.contains("neck");
        let solid_mat = PhysicalMaterial::new_opaque(
            ctx,
            &CpuMaterial {
                albedo: Srgba::new(
                    (vis.rgba[0].clamp(0.0, 1.0) * 255.0) as u8,
                    (vis.rgba[1].clamp(0.0, 1.0) * 255.0) as u8,
                    (vis.rgba[2].clamp(0.0, 1.0) * 255.0) as u8,
                    255,
                ),
                roughness: if is_metal_joint {
                    0.30
                } else if is_accent {
                    0.38
                } else {
                    0.52
                },
                metallic: if is_metal_joint { 0.72 } else { 0.35 },
                ..Default::default()
            },
        );

        let mut ghost_mat = ColorMaterial::new_transparent(
            ctx,
            &CpuMaterial {
                albedo: Srgba::new(34, 211, 238, 48),
                ..Default::default()
            },
        );
        ghost_mat.render_states = RenderStates {
            write_mask: WriteMask::COLOR,
            depth_test: DepthTest::LessOrEqual,
            blend: Blend::TRANSPARENCY,
            cull: Cull::None,
        };

        let mut ghost_wire_mat = ColorMaterial::new_transparent(
            ctx,
            &CpuMaterial {
                albedo: Srgba::new(103, 232, 249, 215),
                ..Default::default()
            },
        );
        ghost_wire_mat.render_states = RenderStates {
            write_mask: WriteMask::COLOR,
            depth_test: DepthTest::LessOrEqual,
            blend: Blend::TRANSPARENCY,
            cull: Cull::None,
        };

        items.push(SolidVisualItem {
            link_name: vis.link_name.clone(),
            joint_index: vis.joint_index,
            local_offset: vis.local_offset,
            base_rgba: vis.rgba,
            mesh: Gm::new(Mesh::new(ctx, &solid_cpu), solid_mat),
            ghost_mesh: Gm::new(Mesh::new(ctx, &solid_cpu), ghost_mat),
            ghost_wire: Gm::new(Mesh::new(ctx, &wire_cpu), ghost_wire_mat),
        });
    }
    items
}

fn build_link_meshes(vis: &LinkVisual, bundle: &AssetBundle) -> (CpuMesh, CpuMesh, f32) {
    match &vis.shape {
        VisualShape::Box(size) => {
            let mut solid = CpuMesh::cube();
            let sx = size[0].max(0.01) * 0.5;
            let sy = size[1].max(0.01) * 0.5;
            let sz = size[2].max(0.01) * 0.5;
            let _ = solid.transform(Mat4::from_nonuniform_scale(sx, sy, sz));
            solid.compute_normals();
            let wire = build_box_wireframe_mesh(sx * 1.02, sy * 1.02, sz * 1.02, 0.0035);
            let r = size[0].max(size[1]).max(size[2]) * 0.55;
            (solid, wire, r.max(0.06))
        }
        VisualShape::Cylinder { radius, length } => {
            // three-d CpuMesh::cylinder is along +X from 0..1 with radius 1
            let mut solid = CpuMesh::cylinder(20);
            let rad = radius.max(0.01);
            let len = length.max(0.01);
            let m = Mat4::from_translation(vec3(0.0, -len * 0.5, 0.0))
                * Mat4::from_angle_z(degrees(90.0))
                * Mat4::from_nonuniform_scale(len, rad, rad);
            let _ = solid.transform(m);
            solid.compute_normals();
            let wire = build_box_wireframe_mesh(rad * 1.03, len * 0.52, rad * 1.03, 0.0032);
            (solid, wire, rad.max(len * 0.5).max(0.06))
        }
        VisualShape::Sphere { radius } => {
            let mut solid = CpuMesh::sphere(16);
            let rad = radius.max(0.01);
            let _ = solid.transform(Mat4::from_scale(rad));
            solid.compute_normals();
            let wire = build_box_wireframe_mesh(rad * 1.02, rad * 1.02, rad * 1.02, 0.0032);
            (solid, wire, rad.max(0.06))
        }
        VisualShape::Mesh { filename, scale } => {
            // Attempt loading STL/OBJ from the in-memory AssetBundle via three-d-asset
            let clean = filename.trim_start_matches("package://");
            let found = bundle.files.iter().find(|(k, _)| k.ends_with(clean));
            if let Some((path, bytes)) = found {
                let mut raw = three_d_asset::io::RawAssets::new();
                raw.insert(path.clone(), bytes.clone());
                if let Ok(mut cpu_mesh) = raw.deserialize::<CpuMesh>(path) {
                    let _ = cpu_mesh.transform(Mat4::from_nonuniform_scale(
                        scale[0], scale[1], scale[2],
                    ));
                    cpu_mesh.compute_normals();
                    let wire = build_box_wireframe_mesh(0.06, 0.06, 0.06, 0.003);
                    return (cpu_mesh, wire, 0.12);
                }
            }
            let mut fallback = CpuMesh::cube();
            let _ = fallback.transform(Mat4::from_scale(0.05));
            fallback.compute_normals();
            let wire = build_box_wireframe_mesh(0.052, 0.052, 0.052, 0.003);
            (fallback, wire, 0.08)
        }
    }
}

/// Builds a crisp 12-edge 3D wireframe cage compatible with both Desktop OpenGL and WebGL2.
fn build_box_wireframe_mesh(hx: f32, hy: f32, hz: f32, thickness: f32) -> CpuMesh {
    let corners = [
        vec3(-hx, -hy, -hz),
        vec3(hx, -hy, -hz),
        vec3(hx, hy, -hz),
        vec3(-hx, hy, -hz),
        vec3(-hx, -hy, hz),
        vec3(hx, -hy, hz),
        vec3(hx, hy, hz),
        vec3(-hx, hy, hz),
    ];
    let edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    ];

    let mut positions = Vec::with_capacity(edges.len() * 24);
    let mut indices: Vec<u32> = Vec::with_capacity(edges.len() * 36);
    let base_cube = CpuMesh::cube();
    let cube_pos = base_cube.positions.to_f32();
    let cube_idx = base_cube.indices.to_u32().unwrap_or_default();

    for (a, b) in edges {
        let pa = corners[a];
        let pb = corners[b];
        let mid = (pa + pb) * 0.5;
        let d = pb - pa;
        let sx = if d.x.abs() > 1e-4 { d.x.abs() * 0.5 } else { thickness };
        let sy = if d.y.abs() > 1e-4 { d.y.abs() * 0.5 } else { thickness };
        let sz = if d.z.abs() > 1e-4 { d.z.abs() * 0.5 } else { thickness };

        let base_v = positions.len() as u32;
        for v in &cube_pos {
            positions.push(vec3(mid.x + v.x * sx, mid.y + v.y * sy, mid.z + v.z * sz));
        }
        for idx in &cube_idx {
            indices.push(base_v + *idx);
        }
    }

    CpuMesh {
        positions: three_d::Positions::F32(positions),
        indices: three_d::Indices::U32(indices),
        ..Default::default()
    }
}

fn build_floor_grid(ctx: &Context) -> Vec<Gm<Mesh, ColorMaterial>> {
    let mut lines = Vec::new();
    let half = 1.6_f32;
    let step = 0.2_f32;
    let count = (half / step) as i32;

    let grid_mat = ColorMaterial::new_opaque(
        ctx,
        &CpuMaterial {
            albedo: Srgba::new(42, 54, 72, 255),
            ..Default::default()
        },
    );
    let x_axis_mat = ColorMaterial::new_opaque(
        ctx,
        &CpuMaterial {
            albedo: Srgba::new(220, 72, 72, 255),
            ..Default::default()
        },
    );
    let z_axis_mat = ColorMaterial::new_opaque(
        ctx,
        &CpuMaterial {
            albedo: Srgba::new(56, 189, 248, 255),
            ..Default::default()
        },
    );

    for i in -count..=count {
        let offset = i as f32 * step;
        let is_origin = i == 0;
        let thickness = if is_origin { 0.004 } else { 0.0018 };

        // Line parallel to X
        let mut lx = CpuMesh::cube();
        let _ = lx.transform(
            Mat4::from_translation(vec3(0.0, 0.001, offset))
                * Mat4::from_nonuniform_scale(half, 0.001, thickness),
        );
        lines.push(Gm::new(
            Mesh::new(ctx, &lx),
            if is_origin {
                x_axis_mat.clone()
            } else {
                grid_mat.clone()
            },
        ));

        // Line parallel to Z
        let mut lz = CpuMesh::cube();
        let _ = lz.transform(
            Mat4::from_translation(vec3(offset, 0.001, 0.0))
                * Mat4::from_nonuniform_scale(thickness, 0.001, half),
        );
        lines.push(Gm::new(
            Mesh::new(ctx, &lz),
            if is_origin {
                z_axis_mat.clone()
            } else {
                grid_mat.clone()
            },
        ));
    }
    lines
}

fn iso_to_mat4(iso: &Isometry3<f32>) -> Mat4 {
    let m: Matrix4<f32> = iso.to_homogeneous();
    let s = m.as_slice();
    Mat4::new(
        s[0], s[1], s[2], s[3],
        s[4], s[5], s[6], s[7],
        s[8], s[9], s[10], s[11],
        s[12], s[13], s[14], s[15],
    )
}
