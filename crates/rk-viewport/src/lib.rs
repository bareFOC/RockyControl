//! Layer 4 — Embedded 3D Mirror Viewport (`rk-viewport`).
//!
//! Renders inside `egui::PaintCallback` sharing `eframe`'s `Arc<glow::Context>` (`glow 0.17`):
//! - Solid PBR meshes tracking live `measured_rad` + live IMU `(pitch, roll)` tilt & foot grounding
//! - Translucent cyan wireframe ghost tracking `target_rad` in `Drive` mode
//! - Orange -> Red heat-tinting when joint `current_ma` > 70% rated current (1400 mA)
//! - Always-fit-to-view auto-framing camera + 26-region Fusion 360 / Onshape style ViewCube (camera ONLY)
//! - Direct 3D robot manipulation: drag joints (axis-plane FK + 3D protractor arc) or arms/hands (k Jacobian IK)
//!
//! Per-foot contact/GRF marks use simulator-only WORLD/GRF telemetry channels;
//! hardware v1 reports zero, so feet are marked neutrally rather than falsely loaded.

mod camera;
mod scene;
mod view_cube;

use std::cell::RefCell;
use std::collections::HashMap;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};

use egui::{Color32, CornerRadius, FontId, Pos2, Rect, Stroke, pos2, vec2};
use glow::HasContext as _;
use rk_control::JointCalibration;
use rk_model::nalgebra::{Isometry3, Point3, Vector3};
use rk_model::{AssetBundle, KinematicRig, UpAxis, VisualShape};
use rk_proto::MAX_JOINTS;
use three_d::{ScissorBox, Viewport};

pub use camera::{CAMERA_FOV_DEG, CameraPreset, OrbitState};
pub use scene::ViewportFrameData;
use scene::{SceneResources, apply_world_offset};
use view_cube::{ViewCubeState, interact_view_cube, paint_view_cube};

static NEXT_VIEWPORT_ID: AtomicU64 = AtomicU64::new(1);

thread_local! {
    static VIEWPORT_SCENES: RefCell<HashMap<u64, SceneResources>> = RefCell::new(HashMap::new());
}

/// How direct pointer dragging on the 3D robot manipulates joints and limbs.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum DragMode {
    /// Joint spheres -> single-joint axis FK; forearm/hand/foot -> multi-joint Limb IK.
    #[default]
    Auto,
    /// Always single-joint axis-plane FK rotation.
    FkJoint,
    /// Always multi-joint Limb IK reach on the grabbed link.
    LimbIk,
}

impl DragMode {
    pub const fn label(self) -> &'static str {
        match self {
            Self::Auto => "Auto (FK/IK)",
            Self::FkJoint => "FK Joint",
            Self::LimbIk => "IK Limb",
        }
    }
}

/// Interactive result returned each frame from [`HumanoidViewport::show`].
#[derive(Clone, Debug)]
pub struct ViewportInteraction {
    pub rect: Rect,
    pub picked_joint: Option<usize>,
    /// Single-joint FK drag update: `(joint_idx, new_angle_rad)`.
    pub dragged_joint: Option<(usize, f32)>,
    /// Multi-joint Limb IK drag update: `(link_name, solved_angles_rad)`.
    pub ik_pose: Option<(String, [f32; MAX_JOINTS])>,
    pub is_dragging_robot: bool,
    pub active_drag_label: Option<String>,
}

impl Default for ViewportInteraction {
    fn default() -> Self {
        Self {
            rect: Rect::NOTHING,
            picked_joint: None,
            dragged_joint: None,
            ik_pose: None,
            is_dragging_robot: false,
            active_drag_label: None,
        }
    }
}

/// Per-foot contact/load signal used to color the rendered feet.
///
/// WORLD/GRF telemetry is simulator enrichment: `rk-sim` publishes the GRF
/// validity bit together with nonzero plant forces, while hardware v1 leaves
/// both the mask and the forces zero. A zero `grf_n` therefore means "unknown /
/// unloaded" and the feet render dim rather than claiming a load.
#[derive(Clone, Copy, Debug, Default)]
pub struct FootContactState {
    /// Bit 0 = right foot stance, bit 1 = left foot stance (wire `contact_mask`).
    pub contact_mask: u8,
    pub grf_left_n: f32,
    pub grf_right_n: f32,
}

impl FootContactState {
    /// `true` when the GRF channel is actually published. With no GRF bit there
    /// is no trustworthy load information, so feet stay at their base color.
    fn loaded(self, left: bool) -> bool {
        let bit = if left { 0b10 } else { 0b01 };
        // Require both the contact bit and a positive (finite) GRF so a v1 MCU
        // that leaves GRF at zero never paints a phantom "loaded" foot.
        self.contact_mask & bit != 0 && self.grf(left) > 0.0
    }

    fn grf(self, left: bool) -> f32 {
        if left {
            self.grf_left_n
        } else {
            self.grf_right_n
        }
    }
}

/// Display & interaction options for rendering the 3D viewport.
#[derive(Clone, Copy, Debug)]
pub struct ViewportOptions {
    pub show_ghost: bool,
    pub estop_active: bool,
    pub hud_blocked: bool,
    pub show_view_cube: bool,
    pub show_protractor: bool,
    pub interactive_drag: bool,
    /// When true, orient robot per live real-world IMU pitch/roll; when false, always keep upright (top-up).
    pub imu_available: bool,
    pub imu_pitch_rad: f32,
    pub imu_roll_rad: f32,
    /// True while a fall event is active; drives the pulsing red fall highlight.
    pub fall_event_active: bool,
    /// Ground-plane world translation `[x, y, z]` of the robot root from plant
    /// telemetry. Applied after the body tilt; `y` is normally zero so the
    /// robot's CoM height does not lift it off the floor.
    pub world_offset: [f32; 3],
}

impl ViewportOptions {
    /// Returns the effective `(pitch_rad, roll_rad)` applied to the 3D robot.
    /// Uses real-world IMU orientation when `imu_available` is true, or strictly upright `(0.0, 0.0)` otherwise.
    pub fn effective_imu_tilt(self) -> (f32, f32) {
        if self.imu_available {
            (self.imu_pitch_rad, self.imu_roll_rad)
        } else {
            (0.0, 0.0)
        }
    }

    /// Finite-only world offset: a non-finite telemetry sample must never
    /// teleport the robot or corrupt the camera target.
    pub fn effective_world_offset(self) -> [f32; 3] {
        self.world_offset.map(|v| if v.is_finite() { v } else { 0.0 })
    }
}

impl Default for ViewportOptions {
    fn default() -> Self {
        Self {
            show_ghost: false,
            estop_active: false,
            hud_blocked: false,
            show_view_cube: true,
            show_protractor: true,
            interactive_drag: true,
            imu_available: false,
            imu_pitch_rad: 0.0,
            imu_roll_rad: 0.0,
            fall_event_active: false,
            world_offset: [0.0; 3],
        }
    }
}

#[derive(Clone, Copy, Debug)]
enum PickVolume {
    BoxHalf([f32; 3]),
    Sphere(f32),
}

#[derive(Clone, Debug)]
struct PickBound {
    link_name: String,
    joint_index: Option<usize>,
    world: Isometry3<f32>,
    volume: PickVolume,
}

#[derive(Clone, Debug)]
enum ActiveViewportDrag {
    Orbit {
        button: egui::PointerButton,
    },
    Fk {
        joint_idx: usize,
        pivot: Point3<f32>,
        axis: Vector3<f32>,
        start_dir: Option<Vector3<f32>>,
        start_screen: Pos2,
        tangent_2d: egui::Vec2,
        start_angle: f32,
        current_angle: f32,
    },
    Ik {
        link_name: String,
        joint_idx: usize,
        plane_point: Point3<f32>,
        plane_normal: Vector3<f32>,
    },
}

/// Public handle held by `rk-ui` to render the embedded 3D mirror viewport.
pub struct HumanoidViewport {
    pub orbit: OrbitState,
    pub show_grid: bool,
    pub show_joint_markers: bool,
    pub pick_radius_scale: f32,
    pub touch_mode: bool,
    pub drag_mode: DragMode,
    viewport_id: u64,
    rig_generation: u64,
    last_error: Option<String>,
    pick_bounds: Vec<PickBound>,
    joint_pick_spheres: Vec<(usize, Point3<f32>, f32)>,
    view_cube: ViewCubeState,
    active_drag: Option<ActiveViewportDrag>,
    /// Latest per-foot contact/GRF signal used to mark the rendered feet.
    feet: FootContactState,
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
            pick_radius_scale: 1.25,
            touch_mode: true,
            drag_mode: DragMode::Auto,
            viewport_id: NEXT_VIEWPORT_ID.fetch_add(1, Ordering::Relaxed),
            rig_generation: 1,
            last_error: None,
            pick_bounds: Vec::new(),
            joint_pick_spheres: Vec::new(),
            view_cube: ViewCubeState::default(),
            active_drag: None,
            feet: FootContactState::default(),
        }
    }

    /// Updates the simulator-only per-foot contact/GRF signal used to mark the
    /// feet. Hardware v1 reports a zero state, which leaves feet neutrally marked.
    /// Non-finite force samples are treated as no load.
    pub fn set_foot_contact_state(&mut self, state: FootContactState) {
        self.feet = FootContactState {
            contact_mask: state.contact_mask,
            grf_left_n: if state.grf_left_n.is_finite() {
                state.grf_left_n.max(0.0)
            } else {
                0.0
            },
            grf_right_n: if state.grf_right_n.is_finite() {
                state.grf_right_n.max(0.0)
            } else {
                0.0
            },
        };
    }

    pub fn notify_rig_changed(&mut self) {
        self.rig_generation = self.rig_generation.wrapping_add(1);
        self.active_drag = None;
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

    /// Shows the full-bleed 3D viewport + ViewCube (camera only) + direct 3D joint/arm dragging.
    #[allow(clippy::too_many_arguments)]
    pub fn show(
        &mut self,
        ui: &mut egui::Ui,
        dt_sec: f32,
        rig: &KinematicRig,
        bundle: &AssetBundle,
        measured_rad: &[f32; MAX_JOINTS],
        target_rad: &[f32; MAX_JOINTS],
        calibration: &[JointCalibration],
        current_ma: &[u16; MAX_JOINTS],
        selected_joint: usize,
        opts: ViewportOptions,
    ) -> ViewportInteraction {
        let desired_size = ui.available_size();
        let size = egui::vec2(desired_size.x.max(120.0), desired_size.y.max(120.0));
        let (rect, response) = ui.allocate_exact_size(size, egui::Sense::click_and_drag());

        // Advance any smooth 220 ms ViewCube slerp transition
        self.orbit.advance(dt_sec);

        // 1. Interact with top-right 26-region ViewCube (dedicated camera orientation control)
        let vc_frame = if !opts.show_view_cube {
            None
        } else {
            interact_view_cube(
                ui,
                rect,
                &mut self.orbit,
                &mut self.view_cube,
                self.touch_mode,
                !opts.hud_blocked,
            )
        };
        let cube_consumed = vc_frame.as_ref().is_some_and(|f| f.consumed);

        let world_offset = opts.effective_world_offset();

        // Always fit the robot's bounding sphere cleanly inside the current viewport aspect ratio.
        // The camera follows the plant world position so the walking robot stays framed
        // while the floor grid remains a fixed world-space motion reference.
        let aspect = (rect.width() / rect.height().max(1.0)).max(0.2);
        let rig_center = rig.bounds_center();
        self.orbit.fit_to_bounds(
            [
                rig_center[0] + world_offset[0],
                rig_center[1] + world_offset[1],
                rig_center[2] + world_offset[2],
            ],
            rig.bounds_radius(),
            aspect,
            CAMERA_FOV_DEG,
        );

        let (eff_pitch_rad, eff_roll_rad) = opts.effective_imu_tilt();
        // Heading is not yet threaded into viewport options; pass 0.0 until fall visuals land.
        let eff_yaw_rad = 0.0_f32;
        let measured_transforms =
            rig.compute_link_transforms_with_tilt(measured_rad, eff_pitch_rad, eff_roll_rad, eff_yaw_rad);
        let target_transforms = if opts.show_ghost {
            rig.compute_link_transforms_with_tilt(target_rad, eff_pitch_rad, eff_roll_rad, eff_yaw_rad)
        } else {
            HashMap::new()
        };
        // Hit-testing geometry must follow the same root translation the renderer
        // applies, otherwise dragging a walking robot would grab empty space.
        let world_transforms: HashMap<String, Isometry3<f32>> = measured_transforms
            .iter()
            .map(|(name, iso)| (name.clone(), apply_world_offset(iso, world_offset)))
            .collect();

        let marker_radius = (rig.bounds_radius() * 0.024).clamp(0.012, 0.034);
        let pick_scale = self.pick_radius_scale.clamp(0.9, 2.0);
        let hit_pad = if self.touch_mode { 0.012 } else { 0.008 } * (pick_scale / 1.25);

        // Collect live world-space positions of movable joints for markers & direct grabbing
        let mut joint_positions = Vec::with_capacity(rig.joints.len());
        self.joint_pick_spheres.clear();
        for j in &rig.joints {
            if let Some(iso) = measured_transforms.get(&j.child_link) {
                let t = iso.translation.vector;
                // Marker positions stay in body space; scene.rs adds the root offset.
                joint_positions.push((j.index, [t.x, t.y, t.z]));
                let wt = apply_world_offset(iso, world_offset).translation.vector;
                self.joint_pick_spheres.push((
                    j.index,
                    Point3::new(wt.x, wt.y, wt.z),
                    (marker_radius * 1.9 * pick_scale).max(0.028),
                ));
            }
        }

        // Update exact oriented link pick volumes for direct robot hit-testing vs empty-space orbit
        self.pick_bounds.clear();
        for vis in &rig.visuals {
            if let Some(iso) = world_transforms.get(&vis.link_name) {
                let world = iso * vis.local_offset;
                let volume = match vis.shape {
                    VisualShape::Box(s) => PickVolume::BoxHalf([
                        s[0].max(0.01) * 0.5 + hit_pad,
                        s[1].max(0.01) * 0.5 + hit_pad,
                        s[2].max(0.01) * 0.5 + hit_pad,
                    ]),
                    VisualShape::Cylinder { radius, length } => {
                        let r = radius.max(0.01) + hit_pad;
                        let half_l = length.max(0.01) * 0.5 + hit_pad;
                        match rig.up_axis {
                            UpAxis::YUp => PickVolume::BoxHalf([r, half_l, r]),
                            UpAxis::ZUp => PickVolume::BoxHalf([r, r, half_l]),
                        }
                    }
                    VisualShape::Sphere { radius } => {
                        PickVolume::Sphere(radius.max(0.015) + hit_pad)
                    }
                    VisualShape::Mesh { scale, .. } => PickVolume::Sphere(
                        (scale[0].abs().max(scale[1].abs()).max(scale[2].abs()) * 0.14)
                            .clamp(0.055, 0.25)
                            + hit_pad,
                    ),
                };
                self.pick_bounds.push(PickBound {
                    link_name: vis.link_name.clone(),
                    joint_index: vis.joint_index,
                    world,
                    volume,
                });
            }
        }

        let mut interaction = ViewportInteraction {
            rect,
            ..Default::default()
        };

        // 2. Canvas interaction: dragging a robot part moves the robot (FK/IK);
        //    dragging empty space (or secondary/middle drag) orbits the camera view.
        if opts.hud_blocked || cube_consumed {
            if !ui.ctx().input(|i| i.pointer.any_down()) {
                self.active_drag = None;
            }
        } else {
            // Start drag: check robot part hit first, fall back to empty-space camera orbit
            if response.drag_started_by(egui::PointerButton::Primary) {
                if let Some(ptr) = ui
                    .ctx()
                    .input(|i| i.pointer.press_origin().or_else(|| i.pointer.interact_pos()))
                {
                    if opts.interactive_drag {
                        if let Some((hit_link, hit_joint, hit_pt, from_joint_sphere)) =
                            self.pick_hit_at(ptr, rect)
                        {
                            interaction.picked_joint = Some(hit_joint);
                            let use_ik = match self.drag_mode {
                                DragMode::LimbIk => hit_link.is_some(),
                                DragMode::FkJoint => false,
                                DragMode::Auto => {
                                    !from_joint_sphere
                                        && hit_link
                                            .as_deref()
                                            .is_some_and(|l| rig.is_ik_end_effector(l))
                                }
                            };

                            if use_ik {
                                if let Some(link_name) = hit_link {
                                    let forward = -self.orbit.back_world();
                                    self.active_drag = Some(ActiveViewportDrag::Ik {
                                        link_name,
                                        joint_idx: hit_joint,
                                        plane_point: hit_pt,
                                        plane_normal: forward,
                                    });
                                }
                            } else if let Some((pivot, axis)) =
                                rig.joint_world_pivot_and_axis(hit_joint, &world_transforms)
                            {
                                let (ray_o, ray_d) = self.pointer_ray(ptr, rect);
                                let start_dir = intersect_ray_plane(ray_o, ray_d, pivot, axis)
                                    .and_then(|p| (p - pivot).try_normalize(1e-4));

                                // Compute robust screen-space tangent perpendicular to projected axis for edge-on views
                                let p0 = self.project_world_to_screen(pivot, rect);
                                let p1 = self.project_world_to_screen(pivot + axis * 0.15, rect);
                                let tangent_2d = match (p0, p1) {
                                    (Some(a), Some(b)) => {
                                        let d = b - a;
                                        if d.length_sq() > 1.0 {
                                            egui::vec2(-d.y, d.x).normalized()
                                        } else {
                                            egui::vec2(1.0, 0.0)
                                        }
                                    }
                                    _ => egui::vec2(1.0, 0.0),
                                };

                                let start_angle = target_rad
                                    .get(hit_joint)
                                    .copied()
                                    .unwrap_or(0.0);

                                self.active_drag = Some(ActiveViewportDrag::Fk {
                                    joint_idx: hit_joint,
                                    pivot,
                                    axis,
                                    start_dir,
                                    start_screen: ptr,
                                    tangent_2d,
                                    start_angle,
                                    current_angle: start_angle,
                                });
                            }
                        } else if !self.hits_any_robot_part(ptr, rect) {
                            self.active_drag = Some(ActiveViewportDrag::Orbit {
                                button: egui::PointerButton::Primary,
                            });
                        }
                    } else if !self.hits_any_robot_part(ptr, rect) {
                        self.active_drag = Some(ActiveViewportDrag::Orbit {
                            button: egui::PointerButton::Primary,
                        });
                    }
                }
            } else if response.drag_started_by(egui::PointerButton::Secondary) {
                self.active_drag = Some(ActiveViewportDrag::Orbit {
                    button: egui::PointerButton::Secondary,
                });
            } else if response.drag_started_by(egui::PointerButton::Middle) {
                self.active_drag = Some(ActiveViewportDrag::Orbit {
                    button: egui::PointerButton::Middle,
                });
            }

            // Continue active camera orbit or direct robot joint/limb drag
            match self.active_drag {
                Some(ActiveViewportDrag::Orbit { button }) if response.dragged_by(button) => {
                    let delta = response.drag_delta();
                    if delta.x.abs() > 0.0 {
                        self.orbit.drag_orbit(delta.x, 0.0);
                        ui.ctx().request_repaint();
                    }
                }
                _ if opts.interactive_drag && response.dragged_by(egui::PointerButton::Primary) => {
                    if let Some(ptr) = response.interact_pointer_pos() {
                        let (ray_o, ray_d) = self.pointer_ray(ptr, rect);
                        let mut active_drag = self.active_drag.take();
                        match &mut active_drag {
                            Some(ActiveViewportDrag::Fk {
                                joint_idx,
                                pivot,
                                axis,
                                start_dir,
                                start_screen,
                                tangent_2d,
                                start_angle,
                                current_angle,
                            }) => {
                                let (lo, hi) = calibration
                                    .get(*joint_idx)
                                    .map(|c| (c.min_rad, c.max_rad))
                                    .unwrap_or((-1.57, 1.57));

                                let ray_dot = ray_d.dot(axis).abs();
                                let delta_rad = if ray_dot > 0.22 {
                                    if let (Some(s_dir), Some(hit_pt)) = (
                                        *start_dir,
                                        intersect_ray_plane(ray_o, ray_d, *pivot, *axis),
                                    ) {
                                        if let Some(cur_dir) =
                                            (hit_pt - *pivot).try_normalize(1e-4)
                                        {
                                            let sin_a = s_dir.cross(&cur_dir).dot(axis);
                                            let cos_a = s_dir.dot(&cur_dir);
                                            sin_a.atan2(cos_a)
                                        } else {
                                            0.0
                                        }
                                    } else {
                                        (ptr - *start_screen).dot(*tangent_2d) * 0.012
                                    }
                                } else {
                                    // Smooth edge-on view fallback along screen tangent
                                    (ptr - *start_screen).dot(*tangent_2d) * 0.012
                                };

                                let next_angle = (*start_angle + delta_rad).clamp(lo, hi);
                                *current_angle = next_angle;
                                interaction.picked_joint = Some(*joint_idx);
                                interaction.dragged_joint = Some((*joint_idx, next_angle));
                                interaction.is_dragging_robot = true;
                                if let Some(jm) = rig.joints.get(*joint_idx) {
                                    interaction.active_drag_label = Some(format!(
                                        "FK {} ({:+.1} deg)",
                                        jm.label,
                                        next_angle.to_degrees()
                                    ));
                                }
                            }
                            Some(ActiveViewportDrag::Ik {
                                link_name,
                                joint_idx,
                                plane_point,
                                plane_normal,
                            }) => {
                                if let Some(world_target) =
                                    intersect_ray_plane(ray_o, ray_d, *plane_point, *plane_normal)
                                {
                                    // The IK solver works in un-translated body space.
                                    let body_target = Point3::new(
                                        world_target.x - world_offset[0],
                                        world_target.y - world_offset[1],
                                        world_target.z - world_offset[2],
                                    );
                                    if let Some(solved) = rig.solve_limb_ik_with_yaw(
                                        link_name,
                                        target_rad,
                                        body_target,
                                        eff_pitch_rad,
                                        eff_roll_rad,
                                        eff_yaw_rad,
                                        calibration,
                                    ) {
                                        interaction.picked_joint = Some(*joint_idx);
                                        interaction.ik_pose = Some((link_name.clone(), solved));
                                        interaction.is_dragging_robot = true;
                                        interaction.active_drag_label =
                                            Some(format!("IK Limb Reach: {}", link_name));
                                    }
                                }
                            }
                            Some(ActiveViewportDrag::Orbit { .. }) | None => {}
                        }
                        self.active_drag = active_drag;
                    }
                }
                _ => {}
            }

            if response.drag_stopped() || !ui.ctx().input(|i| i.pointer.any_down()) {
                self.active_drag = None;
            }

            // Simple click/tap selects joint or limb cleanly
            if opts.interactive_drag && response.clicked() && self.active_drag.is_none() {
                if let Some(pointer_pos) = response.interact_pointer_pos() {
                    if let Some((_, j_idx, _, _)) = self.pick_hit_at(pointer_pos, rect) {
                        interaction.picked_joint = Some(j_idx);
                    }
                }
            }
        }

        // Visual cursor feedback when orbiting empty space or hovering/dragging a robot part
        if self.active_drag.is_some() {
            ui.ctx().set_cursor_icon(egui::CursorIcon::Grabbing);
        } else if opts.interactive_drag && !opts.hud_blocked && !cube_consumed {
            if let Some(hpos) = response.hover_pos() {
                if self.pick_hit_at(hpos, rect).is_some() {
                    ui.ctx().set_cursor_icon(egui::CursorIcon::Grab);
                }
            }
        }

        let active_sel = interaction.picked_joint.unwrap_or(selected_joint);
        let frame_data = ViewportFrameData {
            measured_transforms: measured_transforms.clone(),
            target_transforms,
            joint_positions,
            show_ghost: opts.show_ghost,
            show_grid: self.show_grid,
            show_joint_markers: self.show_joint_markers,
            selected_joint: active_sel,
            current_ma: *current_ma,
            estop_active: opts.estop_active,
            fall_event_active: opts.fall_event_active,
            time_sec: ui.ctx().input(|i| i.time),
            orbit: self.orbit.clone(),
            marker_radius,
            world_offset,
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

                    unsafe {
                        gl.bind_vertex_array(None);
                        gl.disable(glow::DEPTH_TEST);
                        gl.disable(glow::CULL_FACE);
                    }
                }
            });
        });

        // 1. Paint WebGL2 3D scene first
        ui.painter().add(egui::PaintCallback {
            rect,
            callback: Arc::new(callback),
        });

        // 2. Paint 3D Protractor Rotation Arc Gizmo over the active/selected joint
        let overlay_painter = ui.painter_at(rect);
        let is_robot_drag = matches!(
            self.active_drag,
            Some(ActiveViewportDrag::Fk { .. } | ActiveViewportDrag::Ik { .. })
        );
        if opts.show_protractor || is_robot_drag {
            self.paint_joint_protractor_gizmo(
                &overlay_painter,
                rect,
                rig,
                &world_transforms,
                measured_rad,
                target_rad,
                calibration,
                active_sel,
            );
        }

        // 3. Paint 26-region ViewCube cleanly ON TOP of WebGL2
        if let Some(frame) = vc_frame {
            paint_view_cube(&overlay_painter, &frame);
        }

        // 4. Mark each foot with its contact/GRF state on top of the 3D scene.
        //    Uses the already-computed per-foot world transforms, so this is just
        //    two projected circles per frame (cheap, no extra GPU passes).
        self.paint_foot_contact_markers(
            &overlay_painter,
            rect,
            rig,
            &world_transforms,
            marker_radius,
        );

        interaction
    }

    /// Draws a colored disc + halo at each foot's projected world position.
    ///
    /// Green intensity tracks GRF magnitude; swinging feet are dimmed. WORLD/GRF
    /// are simulator-only enrichment — hardware v1 leaves them zero, so feet are
    /// marked neutral rather than falsely "unloaded" or "loaded".
    fn paint_foot_contact_markers(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        rig: &KinematicRig,
        world_transforms: &HashMap<String, Isometry3<f32>>,
        marker_radius: f32,
    ) {
        // Approximate full-load GRF for a 1/4-scale Rocky (~2 kg → ~20 N split
        // between two feet). Only used to map Newtons to a visual intensity.
        const GRF_FULL_SCALE_N: f32 = 12.0;

        for vis in &rig.visuals {
            let lower = vis.link_name.to_ascii_lowercase();
            if !lower.contains("_foot") {
                continue;
            }
            let is_left = lower.contains("l_foot") || lower.contains("left");
            let is_right = lower.contains("r_foot") || lower.contains("right");
            if !is_left && !is_right {
                continue;
            }
            let Some(iso) = world_transforms.get(&vis.link_name) else {
                continue;
            };
            let t = (iso * vis.local_offset).translation.vector;
            let Some(pos) = self.project_world_to_screen(
                Point3::new(t.x, t.y, t.z),
                rect,
            ) else {
                continue;
            };

            let grf = if is_left { self.feet.grf_left_n } else { self.feet.grf_right_n };
            let loaded = self.feet.loaded(is_left);
            let radius = (marker_radius * 1.6).clamp(0.018, 0.05);
            // Screen-space radius from world marker size via the camera projection.
            let px = (self.project_world_to_screen(
                Point3::new(t.x + radius, t.y, t.z),
                rect,
            )
            .map(|p| (p.x - pos.x).abs()))
            .unwrap_or(8.0)
            .clamp(5.0, 26.0);

            let (fill, ring) = if loaded {
                let intensity = (grf / GRF_FULL_SCALE_N).clamp(0.25, 1.0);
                (
                    Color32::from_rgba_unmultiplied(
                        (26.0 + 60.0 * (1.0 - intensity)) as u8,
                        (140.0 + 82.0 * intensity) as u8,
                        (60.0 + 40.0 * (1.0 - intensity)) as u8,
                        200,
                    ),
                    Color32::from_rgba_unmultiplied(74, 222, 128, 230),
                )
            } else {
                // Swinging / unmeasured foot: dim slate outline.
                (
                    Color32::from_rgba_unmultiplied(24, 32, 44, 140),
                    Color32::from_rgba_unmultiplied(96, 124, 156, 170),
                )
            };
            painter.circle_filled(pos, px, fill);
            painter.circle_stroke(
                pos,
                px,
                Stroke::new(if loaded { 2.4 } else { 1.2 }, ring),
            );
        }
    }

    fn pointer_ray(&self, pos: Pos2, rect: Rect) -> (Point3<f32>, Vector3<f32>) {
        let nx = ((pos.x - rect.left()) / rect.width().max(1.0)) * 2.0 - 1.0;
        let ny = 1.0 - ((pos.y - rect.top()) / rect.height().max(1.0)) * 2.0;
        let aspect = (rect.width() / rect.height().max(1.0)).max(0.1);
        let fov_rad = CAMERA_FOV_DEG.to_radians();
        let tan_half = (fov_rad * 0.5).tan();

        let eye_v = self.orbit.eye_position();
        let eye = Point3::new(eye_v.x, eye_v.y, eye_v.z);
        let forward = -self.orbit.back_world();
        let right = self.orbit.right_world();
        let up = self.orbit.up_world();

        let ray_dir =
            (forward + right * (nx * aspect * tan_half) + up * (ny * tan_half)).normalize();
        (eye, ray_dir)
    }

    fn project_world_to_screen(&self, pt: Point3<f32>, rect: Rect) -> Option<Pos2> {
        let eye_v = self.orbit.eye_position();
        let eye = Point3::new(eye_v.x, eye_v.y, eye_v.z);
        let rel = pt - eye;
        let forward = -self.orbit.back_world();
        let right = self.orbit.right_world();
        let up = self.orbit.up_world();

        let z = rel.dot(&forward);
        if z <= 0.05 {
            return None;
        }
        let aspect = (rect.width() / rect.height().max(1.0)).max(0.1);
        let tan_half = (CAMERA_FOV_DEG.to_radians() * 0.5).tan();

        let nx = rel.dot(&right) / (z * tan_half * aspect);
        let ny = rel.dot(&up) / (z * tan_half);
        let sx = rect.left() + (nx * 0.5 + 0.5) * rect.width();
        let sy = rect.top() + (0.5 - ny * 0.5) * rect.height();
        Some(pos2(sx, sy))
    }

    /// Picks either a joint pivot sphere (preferred for exact FK) or a movable link visual (for Limb IK / FK).
    /// Returns `(link_name, joint_idx, world_hit_point, hit_joint_sphere)`.
    fn pick_hit_at(&self, pos: Pos2, rect: Rect) -> Option<(Option<String>, usize, Point3<f32>, bool)> {
        let (eye, ray_dir) = self.pointer_ray(pos, rect);

        // 1. Check joint pivot spheres first when visible so grabbing a joint sphere is unambiguous
        if self.show_joint_markers {
            let mut best_t = f32::INFINITY;
            let mut best_hit = None;
            for &(j_idx, center, radius) in &self.joint_pick_spheres {
                if let Some(t) = ray_sphere_hit(eye, ray_dir, center, radius) {
                    if t < best_t {
                        best_t = t;
                        best_hit = Some((None, j_idx, eye + ray_dir * t, true));
                    }
                }
            }
            if best_hit.is_some() {
                return best_hit;
            }
        }

        // 2. Check movable link visual geometries (forearms, hands, torso, legs, feet) via exact OBB/sphere
        let mut best_t = f32::INFINITY;
        let mut best_link = None;
        for bound in &self.pick_bounds {
            let Some(j_idx) = bound.joint_index else {
                continue;
            };
            if let Some(t) = ray_pick_bound_hit(eye, ray_dir, bound) {
                if t < best_t {
                    best_t = t;
                    best_link = Some((Some(bound.link_name.clone()), j_idx, eye + ray_dir * t, false));
                }
            }
        }
        best_link
    }

    /// Returns `true` if the ray hits any robot visual (including static root links like `pelvis`)
    /// or visible joint marker, distinguishing robot clicks from empty-space clicks.
    fn hits_any_robot_part(&self, pos: Pos2, rect: Rect) -> bool {
        let (eye, ray_dir) = self.pointer_ray(pos, rect);
        if self.show_joint_markers {
            for &(_, center, radius) in &self.joint_pick_spheres {
                if ray_sphere_hit(eye, ray_dir, center, radius).is_some() {
                    return true;
                }
            }
        }
        self.pick_bounds
            .iter()
            .any(|bound| ray_pick_bound_hit(eye, ray_dir, bound).is_some())
    }

    #[allow(clippy::too_many_arguments)]
    fn paint_joint_protractor_gizmo(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        rig: &KinematicRig,
        transforms: &HashMap<String, Isometry3<f32>>,
        measured_rad: &[f32; MAX_JOINTS],
        target_rad: &[f32; MAX_JOINTS],
        calibration: &[JointCalibration],
        joint_idx: usize,
    ) {
        let Some(jm) = rig.joints.get(joint_idx) else {
            return;
        };
        let Some((pivot, axis)) = rig.joint_world_pivot_and_axis(joint_idx, transforms) else {
            return;
        };
        let Some(center_2d) = self.project_world_to_screen(pivot, rect) else {
            return;
        };

        let (lo, hi) = calibration
            .get(joint_idx)
            .map(|c| (c.min_rad, c.max_rad))
            .unwrap_or((jm.default_min_rad, jm.default_max_rad));
        let cur_rad = measured_rad.get(joint_idx).copied().unwrap_or(0.0);
        let tgt_rad = target_rad.get(joint_idx).copied().unwrap_or(cur_rad);

        // Build orthonormal basis (u, v) in the joint's rotation plane
        let ref_v = if axis.y.abs() < 0.85 {
            Vector3::y()
        } else {
            Vector3::x()
        };
        let u = axis.cross(&ref_v).normalize();
        let v = axis.cross(&u).normalize();
        let ring_r = (rig.bounds_radius() * 0.14).clamp(0.065, 0.16);

        let sample_arc = |a0: f32, a1: f32, steps: usize| -> Vec<Pos2> {
            let mut pts = Vec::with_capacity(steps + 1);
            for s in 0..=steps {
                let t = (s as f32) / (steps.max(1) as f32);
                let a = a0 + (a1 - a0) * t;
                let world_p = pivot + (u * a.cos() + v * a.sin()) * ring_r;
                if let Some(p2) = self.project_world_to_screen(world_p, rect) {
                    pts.push(p2);
                }
            }
            pts
        };

        // 1. Subtle slate reference ring
        let full_ring = sample_arc(0.0, std::f32::consts::TAU, 36);
        if full_ring.len() >= 2 {
            painter.add(egui::Shape::line(
                full_ring,
                Stroke::new(1.2, Color32::from_rgba_unmultiplied(100, 128, 164, 110)),
            ));
        }

        // 2. Warm amber soft-limit travel arc [lo, hi]
        let limit_arc = sample_arc(lo, hi, 28);
        if limit_arc.len() >= 2 {
            painter.add(egui::Shape::line(
                limit_arc,
                Stroke::new(2.4, Color32::from_rgba_unmultiplied(245, 158, 11, 210)),
            ));
        }

        // 3. Soft-limit stop ticks at lo and hi
        for &lim in &[lo, hi] {
            let p_in = pivot + (u * lim.cos() + v * lim.sin()) * (ring_r * 0.82);
            let p_out = pivot + (u * lim.cos() + v * lim.sin()) * (ring_r * 1.18);
            if let (Some(a), Some(b)) = (
                self.project_world_to_screen(p_in, rect),
                self.project_world_to_screen(p_out, rect),
            ) {
                painter.line_segment([a, b], Stroke::new(2.2, Color32::from_rgb(251, 191, 36)));
            }
        }

        // 4. Active cyan angle arc (0 -> target) and needle
        let active_arc = sample_arc(0.0, tgt_rad, 20);
        if active_arc.len() >= 2 {
            painter.add(egui::Shape::line(
                active_arc,
                Stroke::new(3.0, Color32::from_rgb(56, 224, 248)),
            ));
        }
        let tip_world = pivot + (u * tgt_rad.cos() + v * tgt_rad.sin()) * (ring_r * 1.05);
        if let Some(tip_2d) = self.project_world_to_screen(tip_world, rect) {
            painter.line_segment(
                [center_2d, tip_2d],
                Stroke::new(2.2, Color32::from_rgb(56, 224, 248)),
            );
            painter.circle_filled(tip_2d, 4.5, Color32::from_rgb(56, 224, 248));
        }

        // 5. Crisp floating angle readout pill beside the pivot
        let badge_text = format!("{}  {:+.1} deg", jm.label, cur_rad.to_degrees());
        let font = FontId::monospace(11.5);
        let galley = painter.layout_no_wrap(badge_text, font, Color32::from_rgb(232, 244, 255));
        let pad = vec2(7.0, 4.0);
        let pill_pos = pos2(
            (center_2d.x + 14.0).clamp(rect.left() + 8.0, rect.right() - galley.size().x - 24.0),
            (center_2d.y - 26.0).clamp(rect.top() + 8.0, rect.bottom() - 32.0),
        );
        let pill_rect = Rect::from_min_size(pill_pos, galley.size() + pad * 2.0);
        painter.rect_filled(
            pill_rect,
            CornerRadius::same(5),
            Color32::from_rgba_unmultiplied(12, 18, 28, 228),
        );
        painter.rect_stroke(
            pill_rect,
            CornerRadius::same(5),
            Stroke::new(1.0, Color32::from_rgb(56, 189, 248)),
            egui::StrokeKind::Inside,
        );
        painter.galley(pill_rect.min + pad, galley, Color32::WHITE);
    }
}

#[inline]
fn ray_pick_bound_hit(
    eye: Point3<f32>,
    ray_dir: Vector3<f32>,
    bound: &PickBound,
) -> Option<f32> {
    match bound.volume {
        PickVolume::BoxHalf(half) => ray_obb_hit(eye, ray_dir, &bound.world, half),
        PickVolume::Sphere(radius) => {
            let center = Point3::new(
                bound.world.translation.x,
                bound.world.translation.y,
                bound.world.translation.z,
            );
            ray_sphere_hit(eye, ray_dir, center, radius)
        }
    }
}

#[inline]
fn ray_obb_hit(
    eye: Point3<f32>,
    ray_dir: Vector3<f32>,
    world: &Isometry3<f32>,
    half: [f32; 3],
) -> Option<f32> {
    let inv = world.inverse();
    let o = inv * eye;
    let d = inv.rotation * ray_dir;

    let mut t_min = 0.05_f32;
    let mut t_max = f32::INFINITY;
    let origins = [o.x, o.y, o.z];
    let dirs = [d.x, d.y, d.z];

    for axis in 0..3 {
        let h = half[axis];
        if dirs[axis].abs() < 1e-6 {
            if origins[axis] < -h || origins[axis] > h {
                return None;
            }
        } else {
            let inv_d = 1.0 / dirs[axis];
            let mut t1 = (-h - origins[axis]) * inv_d;
            let mut t2 = (h - origins[axis]) * inv_d;
            if t1 > t2 {
                core::mem::swap(&mut t1, &mut t2);
            }
            t_min = t_min.max(t1);
            t_max = t_max.min(t2);
            if t_min > t_max {
                return None;
            }
        }
    }
    Some(t_min)
}

#[inline]
fn ray_sphere_hit(
    eye: Point3<f32>,
    ray_dir: Vector3<f32>,
    center: Point3<f32>,
    radius: f32,
) -> Option<f32> {
    let oc = eye - center;
    let b = oc.dot(&ray_dir);
    let c = oc.dot(&oc) - radius * radius;
    let disc = b * b - c;
    if disc < 0.0 {
        return None;
    }
    let t = -b - disc.sqrt();
    if t > 0.05 { Some(t) } else { None }
}

#[inline]
fn intersect_ray_plane(
    ray_o: Point3<f32>,
    ray_d: Vector3<f32>,
    plane_pt: Point3<f32>,
    plane_normal: Vector3<f32>,
) -> Option<Point3<f32>> {
    let denom = ray_d.dot(&plane_normal);
    if denom.abs() < 1e-4 {
        return None;
    }
    let t = (plane_pt - ray_o).dot(&plane_normal) / denom;
    if t > 0.05 && t < 50.0 {
        Some(ray_o + ray_d * t)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ray_plane_intersection_recovers_exact_3d_point() {
        let plane_pt = Point3::new(0.2, 0.8, 0.0);
        let plane_n = Vector3::z();
        let eye = Point3::new(0.2, 0.8, 2.0);
        let dir = Vector3::new(0.1, -0.05, -1.0).normalize();
        let hit = intersect_ray_plane(eye, dir, plane_pt, plane_n).expect("hit plane");
        assert!((hit.z - 0.0).abs() < 1e-5);
        assert!((hit.x - 0.4).abs() < 1e-4);
        assert!((hit.y - 0.7).abs() < 1e-4);
    }

    #[test]
    fn world_to_screen_projection_centers_camera_target() {
        let vp = HumanoidViewport::new();
        let rect = Rect::from_min_size(pos2(0.0, 0.0), vec2(1000.0, 800.0));
        let target = Point3::new(vp.orbit.target[0], vp.orbit.target[1], vp.orbit.target[2]);
        let projected = vp.project_world_to_screen(target, rect).expect("project target");
        assert!((projected.x - 500.0).abs() < 1.0);
        assert!((projected.y - 400.0).abs() < 1.0);
    }

    #[test]
    fn foot_contact_requires_both_bit_and_positive_grf() {
        // Left = bit 1, right = bit 0 (matches rk-sim contact_mask convention).
        let s = FootContactState {
            contact_mask: 0b10,
            grf_left_n: 9.0,
            grf_right_n: 0.0,
        };
        assert!(s.loaded(true));
        assert!(!s.loaded(false), "right has no contact bit and no GRF");

        // A stale contact bit with zero GRF (hardware v1 default) is not a load.
        let hw = FootContactState {
            contact_mask: 0b11,
            grf_left_n: 0.0,
            grf_right_n: 0.0,
        };
        assert!(!hw.loaded(true));
        assert!(!hw.loaded(false));
    }

    #[test]
    fn set_foot_contact_state_clamps_non_finite_grf_to_zero() {
        let mut vp = HumanoidViewport::new();
        vp.set_foot_contact_state(FootContactState {
            contact_mask: 0b11,
            grf_left_n: f32::NAN,
            grf_right_n: f32::INFINITY,
        });
        assert_eq!(vp.feet.grf_left_n, 0.0);
        assert_eq!(vp.feet.grf_right_n, 0.0);
        assert!(!vp.feet.loaded(true));
        assert!(!vp.feet.loaded(false));
    }

    #[test]
    fn obb_picking_hits_limb_and_misses_adjacent_empty_space() {
        let mut vp = HumanoidViewport::new();
        vp.orbit.apply_preset(CameraPreset::Front);
        vp.orbit.target = [0.0, 0.5, 0.0];
        vp.orbit.distance = 2.0;

        let rect = Rect::from_min_size(pos2(0.0, 0.0), vec2(1000.0, 800.0));
        vp.pick_bounds.push(PickBound {
            link_name: "l_thigh".into(),
            joint_index: Some(11),
            world: Isometry3::translation(0.0, 0.5, 0.0),
            volume: PickVolume::BoxHalf([0.04, 0.16, 0.04]),
        });

        // Dead center hits the limb
        assert!(vp.hits_any_robot_part(pos2(500.0, 400.0), rect));
        assert_eq!(vp.pick_hit_at(pos2(500.0, 400.0), rect).map(|h| h.1), Some(11));

        // 90 px to the right is empty space outside the thin thigh -> misses robot so drag orbits view
        assert!(!vp.hits_any_robot_part(pos2(590.0, 400.0), rect));
        assert!(vp.pick_hit_at(pos2(590.0, 400.0), rect).is_none());
    }
}
