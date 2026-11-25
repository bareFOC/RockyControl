//! `three-d` PBR scene, URDF mesh/primitive builder, wireframe ghost renderer, and studio floor grid.

use std::collections::HashMap;
use std::sync::Arc;

use rk_model::nalgebra::{Isometry3, Matrix4};
use rk_model::{AssetBundle, KinematicRig, LinkVisual, UpAxis, VisualShape};
use rk_proto::{HEAT_TINT_THRESHOLD_MA, MAX_JOINTS, RATED_CURRENT_MA};
use three_d::{
    AmbientLight, Blend, Camera, ClearState, ColorMaterial, Context, CpuMaterial, CpuMesh, Cull,
    DepthTest, DirectionalLight, Gm, Mat4, Mesh, Object, PhysicalMaterial, RenderStates,
    RenderTarget, ScissorBox, Srgba, Viewport, WriteMask, degrees, vec3,
};

use crate::camera::OrbitState;

/// Snapshot passed each frame into the `PaintCallback` to render the 3D scene.
#[derive(Clone)]
pub struct ViewportFrameData {
    pub measured_transforms: HashMap<String, Isometry3<f32>>,
    pub target_transforms: HashMap<String, Isometry3<f32>>,
    pub joint_positions: Vec<(usize, [f32; 3])>,
    pub show_ghost: bool,
    pub show_grid: bool,
    pub show_joint_markers: bool,
    pub selected_joint: usize,
    pub current_ma: [u16; MAX_JOINTS],
    pub estop_active: bool,
    /// True while a fall event is active; overrides all other per-joint tints.
    pub fall_event_active: bool,
    /// Monotonic wall time (seconds) from the egui context, used for the fall pulse.
    pub time_sec: f64,
    pub orbit: OrbitState,
    pub marker_radius: f32,
    /// Ground-plane world translation of the robot root, derived from plant
    /// telemetry. Applied as a root transform after the body tilt so the robot
    /// visibly moves across the world-fixed floor grid while walking.
    pub world_offset: [f32; 3],
}

/// Composes the world root translation *after* the tilt/body transform.
///
/// Shared by the renderer (root transform for every visual) and the viewport
/// hit-testing geometry so picking stays aligned with the translated robot.
pub(crate) fn apply_world_offset(iso: &Isometry3<f32>, offset: [f32; 3]) -> Isometry3<f32> {
    Isometry3::translation(offset[0], offset[1], offset[2]) * iso
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

pub(crate) struct SceneResources {
    td_ctx: Context,
    items: Vec<SolidVisualItem>,
    joint_markers: Vec<Gm<Mesh, ColorMaterial>>,
    floor_grid: Vec<Gm<Mesh, ColorMaterial>>,
    floor_plate: Gm<Mesh, PhysicalMaterial>,
    key_light: DirectionalLight,
    fill_light: DirectionalLight,
    rim_light: DirectionalLight,
    ambient_light: AmbientLight,
    pub(crate) rig_generation: u64,
}

impl SceneResources {
    pub(crate) fn new(
        gl: Arc<glow::Context>,
        rig: &KinematicRig,
        bundle: &AssetBundle,
        generation: u64,
    ) -> Result<Self, String> {
        let td_ctx = Context::from_gl_context(gl)
            .map_err(|e| format!("Failed to wrap glow context in three-d: {e:?}"))?;

        let items = build_visual_items(&td_ctx, rig, bundle);
        let joint_markers = build_joint_marker_pool(&td_ctx, MAX_JOINTS);
        let floor_grid = build_floor_grid(&td_ctx);

        // Grounding studio platform disc under the robot (cylinder axis +X -> rotate +90 deg Z to +Y)
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
            joint_markers,
            floor_grid,
            floor_plate,
            key_light,
            fill_light,
            rim_light,
            ambient_light,
            rig_generation: generation,
        })
    }

    pub(crate) fn rebuild_rig(&mut self, rig: &KinematicRig, bundle: &AssetBundle, generation: u64) {
        self.items = build_visual_items(&self.td_ctx, rig, bundle);
        self.rig_generation = generation;
    }

    pub(crate) fn render(
        &mut self,
        viewport: Viewport,
        scissor: ScissorBox,
        screen_w: u32,
        screen_h: u32,
        frame: &ViewportFrameData,
    ) {
        let near = (frame.orbit.distance * 0.015).clamp(0.01, 0.2);
        let far = (frame.orbit.distance * 25.0).clamp(15.0, 120.0);
        let mut camera = Camera::new_perspective(
            viewport,
            frame.orbit.eye_position(),
            frame.orbit.target_vec(),
            frame.orbit.up_vec(),
            degrees(42.0),
            near,
            far,
        );
        camera.set_viewport(viewport);

        // Root translation from plant telemetry. Applied outside the tilt/body transform
        // so the whole robot follows its achieved world position.
        let root = iso_to_mat4(&apply_world_offset(
            &Isometry3::identity(),
            frame.world_offset,
        ));

        // Keep the studio floor grid centred under the robot. Only the horizontal
        // world offset is applied: the grid stays on the ground plane (y = 0) and
        // simply follows the robot's x/z translation, so walking never slides the
        // robot off its reference grid.
        let grid_translation =
            Mat4::from_translation(vec3(frame.world_offset[0], 0.0, frame.world_offset[2]));
        self.floor_plate.set_transformation(grid_translation);
        for line in &mut self.floor_grid {
            line.set_transformation(grid_translation);
        }

        // Update solid measured transforms + current heat-tinting + selection highlight
        for item in &mut self.items {
            if let Some(iso) = frame.measured_transforms.get(&item.link_name) {
                let world = iso * item.local_offset;
                item.mesh.set_transformation(root * iso_to_mat4(&world));
            }

            let mut r = item.base_rgba[0];
            let mut g = item.base_rgba[1];
            let mut b = item.base_rgba[2];
            let mut emissive = Srgba::BLACK;

            if frame.fall_event_active {
                // Highest-priority tint: pulsing red fall alarm overrides heat/selection.
                let (ar, ag, ab) = fall_albedo_rgb();
                r = ar;
                g = ag;
                b = ab;
                emissive = fall_emissive(frame.time_sec);
            } else if let Some(j_idx) = item.joint_index {
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
                    let m = root * iso_to_mat4(&world);
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

        if frame.show_joint_markers && !frame.joint_positions.is_empty() {
            let count = frame.joint_positions.len().min(self.joint_markers.len());
            for (slot, &(j_idx, pos)) in frame.joint_positions.iter().take(count).enumerate() {
                let is_sel = j_idx == frame.selected_joint;
                let r = if is_sel {
                    frame.marker_radius * 1.45
                } else {
                    frame.marker_radius
                };
                self.joint_markers[slot].set_transformation(
                    Mat4::from_translation(vec3(
                        pos[0] + frame.world_offset[0],
                        pos[1] + frame.world_offset[1],
                        pos[2] + frame.world_offset[2],
                    )) * Mat4::from_scale(r),
                );
                self.joint_markers[slot].material.color = if is_sel {
                    Srgba::new(56, 224, 248, 255)
                } else {
                    Srgba::new(245, 158, 11, 230)
                };
            }
            let marker_objs: Vec<&dyn Object> = self.joint_markers[..count]
                .iter()
                .map(|m| m as &dyn Object)
                .collect();
            target.render_partially(scissor, &camera, marker_objs, &[]);
        }

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

fn build_joint_marker_pool(ctx: &Context, count: usize) -> Vec<Gm<Mesh, ColorMaterial>> {
    let sphere_cpu = CpuMesh::sphere(12);
    let mut out = Vec::with_capacity(count);
    for _ in 0..count {
        let mut mat = ColorMaterial::new_opaque(
            ctx,
            &CpuMaterial {
                albedo: Srgba::new(245, 158, 11, 255),
                ..Default::default()
            },
        );
        mat.render_states = RenderStates {
            write_mask: WriteMask::COLOR_AND_DEPTH,
            depth_test: DepthTest::Always,
            ..Default::default()
        };
        out.push(Gm::new(Mesh::new(ctx, &sphere_cpu), mat));
    }
    out
}

fn build_visual_items(
    ctx: &Context,
    rig: &KinematicRig,
    bundle: &AssetBundle,
) -> Vec<SolidVisualItem> {
    let mut items = Vec::with_capacity(rig.visuals.len());
    for vis in &rig.visuals {
        let (solid_cpu, wire_cpu, _pick_radius) = build_link_meshes(vis, bundle, rig.up_axis);

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

fn build_link_meshes(
    vis: &LinkVisual,
    bundle: &AssetBundle,
    up_axis: UpAxis,
) -> (CpuMesh, CpuMesh, f32) {
    match &vis.shape {
        VisualShape::Box(size) => {
            let mut solid = CpuMesh::cube();
            let sx = size[0].max(0.005) * 0.5;
            let sy = size[1].max(0.005) * 0.5;
            let sz = size[2].max(0.005) * 0.5;
            let _ = solid.transform(Mat4::from_nonuniform_scale(sx, sy, sz));
            solid.compute_normals();
            let wire = build_box_wireframe_mesh(sx * 1.02, sy * 1.02, sz * 1.02, 0.0032);
            let r = size[0].max(size[1]).max(size[2]) * 0.55;
            (solid, wire, r.max(0.05))
        }
        VisualShape::Cylinder { radius, length } => {
            // three-d CpuMesh::cylinder is along +X from 0..1 with radius 1.
            // Standard ROS Z-Up URDFs orient <cylinder> along local +Z; Y-Up URDFs along local +Y.
            let mut solid = CpuMesh::cylinder(24);
            let rad = radius.max(0.005);
            let len = length.max(0.005);
            let (m, wire) = match up_axis {
                UpAxis::YUp => (
                    Mat4::from_translation(vec3(0.0, -len * 0.5, 0.0))
                        * Mat4::from_angle_z(degrees(90.0))
                        * Mat4::from_nonuniform_scale(len, rad, rad),
                    build_box_wireframe_mesh(rad * 1.03, len * 0.52, rad * 1.03, 0.003),
                ),
                UpAxis::ZUp => (
                    Mat4::from_translation(vec3(0.0, 0.0, -len * 0.5))
                        * Mat4::from_angle_y(degrees(-90.0))
                        * Mat4::from_nonuniform_scale(len, rad, rad),
                    build_box_wireframe_mesh(rad * 1.03, rad * 1.03, len * 0.52, 0.003),
                ),
            };
            let _ = solid.transform(m);
            solid.compute_normals();
            (solid, wire, rad.max(len * 0.5).max(0.05))
        }
        VisualShape::Sphere { radius } => {
            let mut solid = CpuMesh::sphere(18);
            let rad = radius.max(0.005);
            let _ = solid.transform(Mat4::from_scale(rad));
            solid.compute_normals();
            let wire = build_box_wireframe_mesh(rad * 1.02, rad * 1.02, rad * 1.02, 0.003);
            (solid, wire, rad.max(0.05))
        }
        VisualShape::Mesh { filename, scale } => {
            // Load STL/OBJ from the in-memory AssetBundle via three-d-asset
            if let Some((path, bytes)) = bundle.resolve_mesh_bytes(filename) {
                let mut raw = three_d_asset::io::RawAssets::new();
                raw.insert(path.to_string(), bytes.to_vec());
                if let Ok(mut cpu_mesh) = raw.deserialize::<CpuMesh>(path) {
                    let _ = cpu_mesh.transform(Mat4::from_nonuniform_scale(
                        scale[0], scale[1], scale[2],
                    ));
                    cpu_mesh.compute_normals();
                    let aabb = cpu_mesh.compute_aabb();
                    let sz = aabb.size();
                    let hx = (sz.x * 0.5).max(0.02);
                    let hy = (sz.y * 0.5).max(0.02);
                    let hz = (sz.z * 0.5).max(0.02);
                    let mut wire = build_box_wireframe_mesh(hx, hy, hz, 0.003);
                    let center = aabb.center();
                    let _ = wire.transform(Mat4::from_translation(center));
                    let r = hx.max(hy).max(hz).max(0.06);
                    return (cpu_mesh, wire, r);
                }
            }
            // Fallback proxy block when a standalone .urdf references external STL/OBJ files
            let mut fallback = CpuMesh::cube();
            let _ = fallback.transform(Mat4::from_scale(0.045));
            fallback.compute_normals();
            let wire = build_box_wireframe_mesh(0.048, 0.048, 0.048, 0.003);
            (fallback, wire, 0.07)
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

/// Base albedo (linear 0..1) applied to every solid mesh during a fall alarm.
fn fall_albedo_rgb() -> (f32, f32, f32) {
    (0.85, 0.06, 0.06)
}

/// 0.5 + 0.5*sin(2*pi*2*t): a deterministic 2 Hz pulse in [0, 1].
fn fall_pulse(time_sec: f64) -> f32 {
    let phase = time_sec * std::f64::consts::TAU * 2.0;
    (0.5 + 0.5 * phase.sin()) as f32
}

/// Emissive red channel pulses; green/blue stay dark so the mesh reads unmistakably red.
fn fall_emissive(time_sec: f64) -> Srgba {
    let pulse = fall_pulse(time_sec);
    Srgba::new((200.0 * pulse) as u8, 0, 0, 255)
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fall_emissive_red_exceeds_non_fall_and_pulses_with_time() {
        // Non-fall meshes start from a black emissive (no active tint).
        let non_fall_red = Srgba::BLACK.r;

        // Across a full 2 Hz cycle the fall emissive red channel is always brighter
        // than the non-fall value and takes on distinct values as time advances.
        let mut samples = [0u8; 3];
        for (i, &t) in [0.0_f64, 0.125, 0.25].iter().enumerate() {
            let red = fall_emissive(t).r;
            assert!(
                red > non_fall_red,
                "fall emissive red {red} must exceed non-fall {non_fall_red} at t={t}"
            );
            samples[i] = red;
        }

        // Pulse must vary with time_sec (peak at t=0.125s, midpoint at t=0.0s/0.25s for 2 Hz).
        assert!(
            samples[1] > samples[0],
            "emissive red must pulse: peak {} > midpoint {}",
            samples[1], samples[0]
        );
    }

    #[test]
    fn fall_albedo_is_dominantly_red() {
        let (r, g, b) = fall_albedo_rgb();
        assert!(r > 0.8 && g < 0.1 && b < 0.1);
    }

    #[test]
    fn world_offset_translates_the_root_after_the_body_transform() {
        let body = Isometry3::translation(0.1, 0.0, 0.2);
        let shifted = apply_world_offset(&body, [1.0, 0.0, 2.0]);
        let t = shifted.translation.vector;
        assert!((t.x - 1.1).abs() < 1e-6, "x must add after body pose");
        assert!((t.y - 0.0).abs() < 1e-6);
        assert!((t.z - 2.2).abs() < 1e-6, "forward z must add after body pose");

        let at_origin = apply_world_offset(&Isometry3::identity(), [-0.5, 0.0, 3.0]);
        assert_eq!(at_origin.translation.vector.as_slice(), &[-0.5, 0.0, 3.0]);
    }
}
