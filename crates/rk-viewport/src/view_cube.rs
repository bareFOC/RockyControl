//! Fusion 360 / Onshape-style interactive 26-region 3D ViewCube overlay.
//!
//! Renders a watertight chamfered cube (6 cardinal faces, 12 beveled edges, 8 corner chamfers)
//! synchronized with `OrbitState`, plus direct drag-to-orbit on the cube, a Home isometric button,
//! curved 90-degree CCW/CW roll arrows, and an XYZ world-axis triad.

use egui::{Align2, Color32, FontId, Pos2, Rect, Sense, Shape, Stroke, Vec2, pos2, vec2};
use rk_model::nalgebra::Vector3;

use crate::camera::OrbitState;

const BEVEL: f32 = 0.24;
const FACE_HALF: f32 = 1.0 - BEVEL;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RegionKind {
    Face,
    Edge,
    Corner,
}

#[derive(Clone, Debug)]
pub struct ViewCubeRegion {
    pub id: usize,
    pub kind: RegionKind,
    pub label: &'static str,
    pub tooltip: &'static str,
    pub verts: Vec<Vector3<f32>>,
    pub normal: Vector3<f32>,
    pub view_dir: Vector3<f32>,
    pub up_hint: Vector3<f32>,
}

/// Persistent interaction state for the ViewCube overlay.
#[derive(Default)]
pub struct ViewCubeState {
    drag_accum_px: f32,
    dragging_cube: bool,
    pressed_region_id: Option<usize>,
}

/// Builds all 26 regions (6 faces + 12 beveled edges + 8 chamfered corners) of the unit ViewCube.
pub fn build_view_cube_regions() -> Vec<ViewCubeRegion> {
    let mut regions = Vec::with_capacity(26);
    let f = FACE_HALF;

    // 1. Six cardinal faces (ordered CCW when viewed from outside)
    let faces: [(&str, &str, Vector3<f32>, Vector3<f32>, Vector3<f32>, Vector3<f32>); 6] = [
        ("FRONT", "Front (+Z)", Vector3::z(), Vector3::x(), Vector3::y(), Vector3::y()),
        ("BACK", "Back (-Z)", -Vector3::z(), -Vector3::x(), Vector3::y(), Vector3::y()),
        ("RIGHT", "Right (+X)", Vector3::x(), -Vector3::z(), Vector3::y(), Vector3::y()),
        ("LEFT", "Left (-X)", -Vector3::x(), Vector3::z(), Vector3::y(), Vector3::y()),
        ("TOP", "Top (Upright Lock — Reset Home)", Vector3::y(), Vector3::x(), -Vector3::z(), -Vector3::z()),
        ("BOTTOM", "Bottom (-Y)", -Vector3::y(), Vector3::x(), Vector3::z(), Vector3::z()),
    ];

    for (label, tooltip, n, u, v, up_hint) in faces {
        let verts = vec![
            n - u * f - v * f,
            n + u * f - v * f,
            n + u * f + v * f,
            n - u * f + v * f,
        ];
        regions.push(ViewCubeRegion {
            id: regions.len(),
            kind: RegionKind::Face,
            label,
            tooltip,
            verts,
            normal: n,
            view_dir: n,
            up_hint,
        });
    }

    // 2. Twelve beveled edges (pairs of orthogonal cardinal axes)
    let edges: [(&str, Vector3<f32>, Vector3<f32>); 12] = [
        ("Front-Top", Vector3::z(), Vector3::y()),
        ("Front-Bottom", Vector3::z(), -Vector3::y()),
        ("Front-Right", Vector3::z(), Vector3::x()),
        ("Front-Left", Vector3::z(), -Vector3::x()),
        ("Back-Top", -Vector3::z(), Vector3::y()),
        ("Back-Bottom", -Vector3::z(), -Vector3::y()),
        ("Back-Right", -Vector3::z(), Vector3::x()),
        ("Back-Left", -Vector3::z(), -Vector3::x()),
        ("Top-Right", Vector3::y(), Vector3::x()),
        ("Top-Left", Vector3::y(), -Vector3::x()),
        ("Bottom-Right", -Vector3::y(), Vector3::x()),
        ("Bottom-Left", -Vector3::y(), -Vector3::x()),
    ];

    for (tooltip, a, b) in edges {
        let dir = (a + b).normalize();
        let t = a.cross(&b).normalize();
        // Quad joining face A's edge (a + b*f ± t*f) to face B's edge (b + a*f ± t*f)
        let mut verts = vec![
            a + b * f - t * f,
            a + b * f + t * f,
            b + a * f + t * f,
            b + a * f - t * f,
        ];
        ensure_outward_winding(&mut verts, dir);
        regions.push(ViewCubeRegion {
            id: regions.len(),
            kind: RegionKind::Edge,
            label: "",
            tooltip,
            verts,
            normal: dir,
            view_dir: dir,
            up_hint: Vector3::y(),
        });
    }

    // 3. Eight chamfered isometric corners
    let corners: [(&str, f32, f32, f32); 8] = [
        ("Front-Right-Top (Iso)", 1.0, 1.0, 1.0),
        ("Front-Left-Top (Iso)", -1.0, 1.0, 1.0),
        ("Back-Right-Top (Iso)", 1.0, 1.0, -1.0),
        ("Back-Left-Top (Iso)", -1.0, 1.0, -1.0),
        ("Front-Right-Bottom (Iso)", 1.0, -1.0, 1.0),
        ("Front-Left-Bottom (Iso)", -1.0, -1.0, 1.0),
        ("Back-Right-Bottom (Iso)", 1.0, -1.0, -1.0),
        ("Back-Left-Bottom (Iso)", -1.0, -1.0, -1.0),
    ];

    for (tooltip, sx, sy, sz) in corners {
        let dir = Vector3::new(sx, sy, sz).normalize();
        let mut verts = vec![
            Vector3::new(sx, sy * f, sz * f),
            Vector3::new(sx * f, sy, sz * f),
            Vector3::new(sx * f, sy * f, sz),
        ];
        ensure_outward_winding(&mut verts, dir);
        regions.push(ViewCubeRegion {
            id: regions.len(),
            kind: RegionKind::Corner,
            label: "",
            tooltip,
            verts,
            normal: dir,
            view_dir: dir,
            up_hint: Vector3::y(),
        });
    }

    regions
}

fn ensure_outward_winding(verts: &mut [Vector3<f32>], expected_normal: Vector3<f32>) {
    if verts.len() >= 3 {
        let n = (verts[1] - verts[0]).cross(&(verts[2] - verts[0]));
        if n.dot(&expected_normal) < 0.0 {
            verts.reverse();
        }
    }
}

#[derive(Clone)]
struct ProjectedRegionData {
    id: usize,
    kind: RegionKind,
    label: &'static str,
    pts: Vec<Pos2>,
    depth_z: f32,
    normal_cam: Vector3<f32>,
    center_2d: Pos2,
}

/// Prepared ViewCube geometry and hover state from [`interact_view_cube`], ready to be painted
/// on top of the 3D WebGL2 `PaintCallback` via [`paint_view_cube`].
pub struct ViewCubeFrame {
    pub consumed: bool,
    scale: f32,
    box_rect: Rect,
    home_rect: Rect,
    home_hovered: bool,
    widget_hot: bool,
    hovered_id: Option<usize>,
    visible: Vec<ProjectedRegionData>,
    orientation: rk_model::nalgebra::UnitQuaternion<f32>,
}

fn project_regions(
    regions: &[ViewCubeRegion],
    orientation: &rk_model::nalgebra::UnitQuaternion<f32>,
    center: Pos2,
    cube_radius: f32,
) -> Vec<ProjectedRegionData> {
    let inv_rot = orientation.inverse();
    let eye_cam = Vector3::new(0.0, 0.0, 4.2);
    let mut visible = Vec::with_capacity(26);
    for region in regions {
        let n_cam = inv_rot * region.normal;
        let mut pts = Vec::with_capacity(region.verts.len());
        let mut centroid_cam = Vector3::zeros();
        let mut sum_2d = Vec2::ZERO;
        for v in &region.verts {
            let p_cam = inv_rot * v;
            centroid_cam += p_cam;
            let persp = 4.2 / (4.2 - p_cam.z);
            let sx = center.x + p_cam.x * cube_radius * persp;
            let sy = center.y - p_cam.y * cube_radius * persp;
            let p2 = pos2(sx, sy);
            pts.push(p2);
            sum_2d += p2.to_vec2();
        }
        let count = pts.len() as f32;
        centroid_cam /= count;

        // Exact perspective backface culling (eliminates grazing back-face bleed-through)
        let to_eye = eye_cam - centroid_cam;
        if to_eye.dot(&n_cam) <= 0.02 {
            continue;
        }

        visible.push(ProjectedRegionData {
            id: region.id,
            kind: region.kind,
            label: region.label,
            pts,
            depth_z: centroid_cam.z,
            normal_cam: n_cam,
            center_2d: (sum_2d / count).to_pos2(),
        });
    }
    // Sort back-to-front for exact painter's order on a convex solid
    visible.sort_by(|a, b| a.depth_z.total_cmp(&b.depth_z));
    visible
}

/// Processes ViewCube, Home button, and 90-degree CCW/CW roll arrow input in the top-right corner
/// of `viewport_rect` BEFORE main canvas orbit/picking. Call [`paint_view_cube`] AFTER the WebGL2
/// `PaintCallback` so the widget always composites cleanly on top of the 3D robot.
pub fn interact_view_cube(
    ui: &mut egui::Ui,
    viewport_rect: Rect,
    orbit: &mut OrbitState,
    state: &mut ViewCubeState,
    touch_mode: bool,
) -> Option<ViewCubeFrame> {
    let widget_size = if touch_mode { 138.0 } else { 118.0 };
    let margin = if touch_mode { 12.0 } else { 14.0 };
    let box_rect = Rect::from_min_size(
        pos2(
            viewport_rect.right() - widget_size - margin,
            viewport_rect.top() + margin,
        ),
        vec2(widget_size, widget_size + 20.0),
    );
    if viewport_rect.width() < 160.0 || viewport_rect.height() < 150.0 {
        return None;
    }

    let scale = widget_size / 118.0;
    let center = pos2(box_rect.center().x, box_rect.top() + widget_size * 0.50);
    let cube_radius = 30.0 * scale;

    let (pointer_pos, primary_pressed, primary_down, primary_released, pointer_delta) =
        ui.ctx().input(|i| {
            (
                i.pointer.hover_pos().or(i.pointer.latest_pos()),
                i.pointer.primary_pressed(),
                i.pointer.primary_down(),
                i.pointer.primary_released(),
                i.pointer.delta(),
            )
        });
    let widget_hot = pointer_pos.is_some_and(|p| box_rect.expand(8.0).contains(p))
        || state.dragging_cube
        || state.pressed_region_id.is_some()
        || touch_mode;

    let mut consumed = false;

    // 1. Home button (top-left of cube)
    let home_size = if touch_mode { 28.0 } else { 22.0 };
    let home_rect = Rect::from_center_size(
        center + vec2(-42.0, -40.0) * scale,
        vec2(home_size, home_size),
    );
    let home_resp = ui
        .interact(
            home_rect,
            ui.scope_id().with("viewcube_home"),
            Sense::click(),
        )
        .on_hover_text("Home (Upright Isometric)");
    let home_hit = pointer_pos.is_some_and(|p| home_rect.contains(p));
    if home_resp.clicked() || (home_hit && primary_released && !state.dragging_cube) {
        orbit.animate_home();
        consumed = true;
    }
    if home_resp.hovered() || home_resp.is_pointer_button_down_on() || home_hit {
        consumed = true;
    }

    // 2. Project all 26 ViewCube regions using the live gravity-locked camera orientation
    let cube_hit_rect = Rect::from_center_size(center, vec2(84.0, 84.0) * scale);
    let cube_resp = ui.interact(
        cube_hit_rect,
        ui.scope_id().with("viewcube_body"),
        Sense::click_and_drag(),
    );

    let regions = build_view_cube_regions();
    let visible = project_regions(&regions, &orbit.orientation, center, cube_radius);

    // Hit-test pointer front-to-back across the 26 projected polygons
    let hovered_id: Option<usize> = if !state.dragging_cube && !home_hit {
        pointer_pos.and_then(|pos| {
            visible
                .iter()
                .rev()
                .find(|p| point_in_convex_polygon(pos, &p.pts))
                .map(|p| p.id)
        })
    } else {
        None
    };

    // Handle direct cube drag-to-orbit (horizontal yaw only) vs click-to-snap
    let mut orientation_changed = false;
    if (primary_pressed && hovered_id.is_some()) || cube_resp.drag_started() {
        state.drag_accum_px = 0.0;
        state.dragging_cube = false;
        state.pressed_region_id = hovered_id;
        if hovered_id.is_some() {
            consumed = true;
        }
    }

    if state.pressed_region_id.is_some() && primary_down {
        let delta = if cube_resp.dragged() {
            cube_resp.drag_delta()
        } else {
            pointer_delta
        };
        state.drag_accum_px += delta.length();
        if state.drag_accum_px > 4.0 {
            state.dragging_cube = true;
            orbit.drag_orbit(delta.x, 0.0);
            orientation_changed = true;
        }
        consumed = true;
    }

    let released_on_cube = cube_resp.clicked()
        || cube_resp.drag_stopped()
        || (primary_released && (state.pressed_region_id.is_some() || hovered_id.is_some()));

    if released_on_cube && !home_hit {
        if !state.dragging_cube {
            let release_hit = cube_resp
                .interact_pointer_pos()
                .or(pointer_pos)
                .and_then(|pos| {
                    visible
                        .iter()
                        .rev()
                        .find(|p| point_in_convex_polygon(pos, &p.pts))
                        .map(|p| p.id)
                })
                .or(state.pressed_region_id);

            if let Some(hit_id) = release_hit {
                if let Some(reg) = regions.get(hit_id) {
                    orbit.animate_to_dir_up(reg.view_dir, Vector3::y());
                }
            }
        }
        state.dragging_cube = false;
        state.drag_accum_px = 0.0;
        state.pressed_region_id = None;
        consumed = true;
    } else if !primary_down {
        state.dragging_cube = false;
        state.pressed_region_id = None;
    }

    if hovered_id.is_some()
        || state.pressed_region_id.is_some()
        || cube_resp.hovered()
        || cube_resp.is_pointer_button_down_on()
    {
        consumed = true;
    }

    // Show clean hover tooltip for the exact hovered face, edge, or corner
    if let Some(hid) = hovered_id {
        if let Some(reg) = regions.get(hid) {
            cube_resp.on_hover_text(reg.tooltip);
        }
    }

    let final_visible = if orientation_changed {
        project_regions(&regions, &orbit.orientation, center, cube_radius)
    } else {
        visible
    };

    Some(ViewCubeFrame {
        consumed,
        scale,
        box_rect,
        home_rect,
        home_hovered: home_resp.hovered(),
        widget_hot,
        hovered_id,
        visible: final_visible,
        orientation: orbit.orientation,
    })
}

/// Paints the ViewCube, Home icon, and XYZ world-axis triad on top of WebGL2.
pub fn paint_view_cube(painter: &egui::Painter, frame: &ViewCubeFrame) {
    let scale = frame.scale;

    // 1. Home button
    paint_home_button(painter, frame.home_rect, frame.home_hovered, frame.widget_hot);

    // 3. Solid directional-shaded 26-region chamfered ViewCube (sculpted CAD slate look)
    let light_dir = Vector3::new(-0.32, 0.58, 0.75).normalize();
    let base_alpha: u8 = if frame.widget_hot { 255 } else { 246 };

    for proj in &frame.visible {
        let is_hovered = frame.hovered_id == Some(proj.id);
        let ndotl = proj.normal_cam.dot(&light_dir).clamp(0.0, 1.0);

        let fill = if is_hovered {
            match proj.kind {
                RegionKind::Face => Color32::from_rgba_unmultiplied(18, 126, 162, 255),
                RegionKind::Edge => Color32::from_rgba_unmultiplied(14, 152, 194, 255),
                RegionKind::Corner => Color32::from_rgba_unmultiplied(56, 189, 248, 255),
            }
        } else {
            let (base_r, base_g, base_b) = match proj.kind {
                RegionKind::Face => (42.0, 54.0, 74.0),
                RegionKind::Edge => (28.0, 38.0, 54.0),
                RegionKind::Corner => (50.0, 64.0, 86.0),
            };
            let shade = 0.72 + 0.56 * ndotl;
            Color32::from_rgba_unmultiplied(
                (base_r * shade).min(255.0) as u8,
                (base_g * shade).min(255.0) as u8,
                (base_b * shade).min(255.0) as u8,
                base_alpha,
            )
        };

        let stroke = if is_hovered {
            Stroke::new(1.4, Color32::from_rgb(125, 240, 255))
        } else {
            Stroke::new(
                1.0,
                Color32::from_rgba_unmultiplied(
                    104,
                    126,
                    156,
                    if frame.widget_hot { 235 } else { 195 },
                ),
            )
        };

        painter.add(Shape::convex_polygon(proj.pts.clone(), fill, stroke));

        // Render crisp CAD cardinal labels on visible faces
        if proj.kind == RegionKind::Face && proj.normal_cam.z > 0.16 {
            let label_alpha = (((proj.normal_cam.z - 0.16) / 0.45).clamp(0.0, 1.0) * 255.0) as u8;
            let text_col = if is_hovered {
                Color32::WHITE
            } else {
                Color32::from_rgba_unmultiplied(238, 245, 255, label_alpha.max(115))
            };
            let font_size = if proj.label.len() >= 6 {
                8.8 * scale
            } else {
                9.8 * scale
            };
            painter.text(
                proj.center_2d,
                Align2::CENTER_CENTER,
                proj.label,
                FontId::proportional(font_size),
                text_col,
            );
        }
    }

    // 4. Compact XYZ World-Axis Triad anchored at bottom-left of the ViewCube footprint
    let triad_origin = pos2(
        frame.box_rect.left() + 24.0 * scale,
        frame.box_rect.bottom() - 18.0 * scale,
    );
    paint_axis_triad(painter, triad_origin, &frame.orientation, 17.5 * scale);
}

fn paint_home_button(painter: &egui::Painter, rect: Rect, hovered: bool, visible: bool) {
    let alpha = if hovered {
        245
    } else if visible {
        210
    } else {
        130
    };
    let bg = if hovered {
        Color32::from_rgba_unmultiplied(14, 116, 144, alpha)
    } else {
        Color32::from_rgba_unmultiplied(20, 27, 38, alpha)
    };
    let border = if hovered {
        Color32::from_rgb(103, 232, 249)
    } else {
        Color32::from_rgba_unmultiplied(64, 82, 108, alpha)
    };
    painter.rect_filled(rect, 4.0, bg);
    painter.rect_stroke(rect, 4.0, Stroke::new(1.0, border), egui::StrokeKind::Inside);

    // Crisp vector house roof + walls
    let c = rect.center();
    let r = rect.width() * 0.28;
    let col = if hovered {
        Color32::WHITE
    } else {
        Color32::from_rgba_unmultiplied(203, 213, 225, alpha)
    };
    let roof = vec![
        pos2(c.x - r * 1.15, c.y + r * 0.05),
        pos2(c.x, c.y - r * 1.05),
        pos2(c.x + r * 1.15, c.y + r * 0.05),
    ];
    painter.add(Shape::line(roof, Stroke::new(1.5, col)));
    let walls = Rect::from_min_max(
        pos2(c.x - r * 0.72, c.y + r * 0.02),
        pos2(c.x + r * 0.72, c.y + r * 0.95),
    );
    painter.rect_stroke(walls, 1.0, Stroke::new(1.3, col), egui::StrokeKind::Inside);
}

fn paint_axis_triad(
    painter: &egui::Painter,
    origin: Pos2,
    orientation: &rk_model::nalgebra::UnitQuaternion<f32>,
    len: f32,
) {
    let inv_rot = orientation.inverse();
    let mut axes = [
        ("X", Vector3::x(), Color32::from_rgb(239, 68, 68)),
        ("Y", Vector3::y(), Color32::from_rgb(34, 197, 94)),
        ("Z", Vector3::z(), Color32::from_rgb(59, 130, 246)),
    ]
    .map(|(label, vec, color)| {
        let cam = inv_rot * vec;
        (label, cam, color)
    });

    axes.sort_by(|a, b| a.1.z.total_cmp(&b.1.z));
    painter.circle_filled(origin, 2.2, Color32::from_rgb(148, 163, 184));
    for (label, cam, color) in axes {
        let end = origin + vec2(cam.x * len, -cam.y * len);
        painter.line_segment([origin, end], Stroke::new(1.7, color));
        let lbl_pos = origin + vec2(cam.x * (len + 7.5), -cam.y * (len + 7.5));
        painter.text(
            lbl_pos,
            Align2::CENTER_CENTER,
            label,
            FontId::monospace(9.5),
            color,
        );
    }
}

fn point_in_convex_polygon(p: Pos2, poly: &[Pos2]) -> bool {
    if poly.len() < 3 {
        return false;
    }
    let mut sign = 0.0_f32;
    for i in 0..poly.len() {
        let a = poly[i];
        let b = poly[(i + 1) % poly.len()];
        let cross = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
        if cross.abs() < 1e-5 {
            continue;
        }
        if sign == 0.0 {
            sign = cross.signum();
        } else if cross.signum() != sign {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn view_cube_has_exact_26_regions_with_outward_normals() {
        let regions = build_view_cube_regions();
        assert_eq!(regions.len(), 26);
        let faces = regions.iter().filter(|r| r.kind == RegionKind::Face).count();
        let edges = regions.iter().filter(|r| r.kind == RegionKind::Edge).count();
        let corners = regions.iter().filter(|r| r.kind == RegionKind::Corner).count();
        assert_eq!((faces, edges, corners), (6, 12, 8));

        for r in &regions {
            assert!((r.normal.norm() - 1.0).abs() < 1e-5);
            let centroid = r.verts.iter().copied().sum::<Vector3<f32>>() / (r.verts.len() as f32);
            assert!(
                centroid.dot(&r.normal) > 0.5,
                "region {} normal must point outward from cube center",
                r.tooltip
            );
        }
    }

    #[test]
    fn convex_polygon_hit_test_detects_interior_and_exterior() {
        let square = vec![
            pos2(0.0, 0.0),
            pos2(10.0, 0.0),
            pos2(10.0, 10.0),
            pos2(0.0, 10.0),
        ];
        assert!(point_in_convex_polygon(pos2(5.0, 5.0), &square));
        assert!(!point_in_convex_polygon(pos2(12.0, 5.0), &square));
    }
}
