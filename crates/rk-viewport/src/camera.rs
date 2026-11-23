//! Quaternion-based auto-fitted orbit camera with smooth 220 ms transitions and 90-degree roll.
//!
//! Camera distance and target are continuously fitted to the active URDF bounding sphere
//! and viewport aspect ratio so the entire robot always remains centered and framed in view.

use rk_model::nalgebra::{Matrix3, Unit, UnitQuaternion, Vector3};
use three_d::{Vec3, vec3};

pub const CAMERA_FOV_DEG: f32 = 42.0;
const TRANSITION_DURATION_SEC: f32 = 0.22;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CameraPreset {
    Iso,
    Front,
    Back,
    Left,
    Right,
    Top,
    Bottom,
    Side,
}

#[derive(Clone, Debug)]
pub struct CameraTransition {
    pub from: UnitQuaternion<f32>,
    pub to: UnitQuaternion<f32>,
    pub elapsed_sec: f32,
    pub duration_sec: f32,
}

#[derive(Clone, Debug)]
pub struct OrbitState {
    /// Rotation from camera-local basis (`+X` right, `+Y` up, `+Z` toward eye) to world space.
    pub orientation: UnitQuaternion<f32>,
    pub distance: f32,
    pub target: [f32; 3],
    pub transition: Option<CameraTransition>,
}

impl Default for OrbitState {
    fn default() -> Self {
        Self {
            orientation: home_iso_orientation(),
            distance: 2.35,
            target: [0.0, 0.78, 0.0],
            transition: None,
        }
    }
}

/// Default Front-Right-Top isometric workstation orientation.
pub fn home_iso_orientation() -> UnitQuaternion<f32> {
    orientation_from_dir_up(
        Vector3::new(0.56, 0.34, 0.75).normalize(),
        Vector3::y(),
    )
}

/// Constructs a right-handed camera orientation where camera `+Z` points along `eye_dir`
/// (from target toward the camera) and camera `+Y` aligns as closely as possible with `up_hint`.
pub fn orientation_from_dir_up(eye_dir: Vector3<f32>, up_hint: Vector3<f32>) -> UnitQuaternion<f32> {
    let back = Unit::try_new(eye_dir, 1e-6)
        .map(|u| u.into_inner())
        .unwrap_or_else(Vector3::z);

    // Pick a stable fallback up if up_hint is parallel to eye_dir (e.g., Top / Bottom views)
    let up_ref = if back.dot(&up_hint).abs() > 0.995 {
        if back.y > 0.0 {
            -Vector3::z()
        } else {
            Vector3::z()
        }
    } else {
        up_hint
    };

    let right = up_ref
        .cross(&back)
        .try_normalize(1e-6)
        .unwrap_or_else(Vector3::x);
    let up = back.cross(&right).normalize();

    let rot_mat = Matrix3::from_columns(&[right, up, back]);
    UnitQuaternion::from_matrix(&rot_mat)
}

impl OrbitState {
    pub fn apply_preset(&mut self, preset: CameraPreset) {
        let target_q = self.preset_orientation(preset);
        self.orientation = target_q;
        self.transition = None;
    }

    pub fn apply_preset_for_bounds(
        &mut self,
        preset: CameraPreset,
        center: [f32; 3],
        radius: f32,
    ) {
        self.animate_to_orientation(self.preset_orientation(preset));
        self.fit_to_bounds(center, radius, 1.33, CAMERA_FOV_DEG);
    }

    pub fn preset_orientation(&self, preset: CameraPreset) -> UnitQuaternion<f32> {
        match preset {
            CameraPreset::Iso => home_iso_orientation(),
            CameraPreset::Front => orientation_from_dir_up(Vector3::z(), Vector3::y()),
            CameraPreset::Back => orientation_from_dir_up(-Vector3::z(), Vector3::y()),
            CameraPreset::Right | CameraPreset::Side => {
                orientation_from_dir_up(Vector3::x(), Vector3::y())
            }
            CameraPreset::Left => orientation_from_dir_up(-Vector3::x(), Vector3::y()),
            CameraPreset::Top => orientation_from_dir_up(Vector3::y(), -Vector3::z()),
            CameraPreset::Bottom => orientation_from_dir_up(-Vector3::y(), Vector3::z()),
        }
    }

    /// Starts a smooth 220 ms eased quaternion transition to `target_q`.
    pub fn animate_to_orientation(&mut self, target_q: UnitQuaternion<f32>) {
        // Ensure shortest-arc slerp by aligning hemisphere sign
        let aligned = if self.orientation.coords.dot(&target_q.coords) < 0.0 {
            UnitQuaternion::new_unchecked(-target_q.into_inner())
        } else {
            target_q
        };
        self.transition = Some(CameraTransition {
            from: self.orientation,
            to: aligned,
            elapsed_sec: 0.0,
            duration_sec: TRANSITION_DURATION_SEC,
        });
    }

    /// Smoothly rotates camera to look from `eye_dir` toward target using `up_hint`.
    pub fn animate_to_dir_up(&mut self, eye_dir: Vector3<f32>, up_hint: Vector3<f32>) {
        let q = orientation_from_dir_up(eye_dir, up_hint);
        self.animate_to_orientation(q);
    }

    /// Smoothly animates back to the default Front-Right-Top Home isometric view.
    pub fn animate_home(&mut self) {
        self.animate_to_orientation(home_iso_orientation());
    }

    /// Rolls the camera around its current viewing axis by `delta_rad` (e.g., ±90 degrees).
    /// Chains from any in-flight transition destination so rapid clicks add full 90-degree turns.
    pub fn animate_roll(&mut self, delta_rad: f32) {
        let base = self
            .transition
            .as_ref()
            .map(|t| t.to)
            .unwrap_or(self.orientation);
        let roll_q = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), delta_rad);
        self.animate_to_orientation(base * roll_q);
    }

    /// Advances any active smooth ViewCube transition; returns `true` while animating.
    pub fn advance(&mut self, dt_sec: f32) -> bool {
        let Some(mut tr) = self.transition.take() else {
            return false;
        };
        tr.elapsed_sec += dt_sec.max(0.0);
        let t = (tr.elapsed_sec / tr.duration_sec.max(1e-4)).clamp(0.0, 1.0);
        // Smooth cubic ease-out: 1 - (1 - t)^3
        let eased = 1.0 - (1.0 - t).powi(3);
        self.orientation = tr.from.slerp(&tr.to, eased);
        if t < 1.0 {
            self.transition = Some(tr);
            true
        } else {
            self.orientation = tr.to;
            false
        }
    }

    /// Interactive turntable / trackball orbit from a screen pointer drag delta.
    /// Immediately cancels any in-flight transition so user drag always has instant authority.
    pub fn drag_orbit(&mut self, dx: f32, dy: f32) {
        self.transition = None;
        let yaw_delta = -dx * 0.0085;
        let pitch_delta = -dy * 0.0080;

        let cur_up = self.orientation * Vector3::y();
        // Use world Y turntable when upright so the horizon stays crisp; fall back to local up near poles.
        let yaw_axis = if cur_up.y.abs() > 0.2 {
            let sign = if cur_up.y >= 0.0 { 1.0 } else { -1.0 };
            Unit::new_normalize(Vector3::new(0.0, sign, 0.0))
        } else {
            Unit::new_normalize(cur_up)
        };
        let yaw_q = UnitQuaternion::from_axis_angle(&yaw_axis, yaw_delta);
        let pitch_q = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), pitch_delta);

        self.orientation = yaw_q * self.orientation * pitch_q;
        self.orientation.renormalize();
    }

    /// Computes exact perspective distance so a sphere of `radius` at `center` fits inside
    /// both vertical and horizontal frustum planes with balanced margin across desktop and tablet aspect ratios.
    pub fn fit_to_bounds(&mut self, center: [f32; 3], radius: f32, aspect: f32, fov_deg: f32) {
        self.target = center;
        let half_v = (fov_deg.to_radians() * 0.5).clamp(0.1, 1.3);
        let half_h = (half_v.tan() * aspect.max(0.25)).atan();
        let limiting_half = half_v.min(half_h);
        let fit_margin = 1.18_f32;
        self.distance = ((radius.max(0.15) * fit_margin) / limiting_half.sin()).clamp(0.45, 24.0);
    }

    pub fn right_world(&self) -> Vector3<f32> {
        self.orientation * Vector3::x()
    }

    pub fn up_world(&self) -> Vector3<f32> {
        self.orientation * Vector3::y()
    }

    pub fn back_world(&self) -> Vector3<f32> {
        self.orientation * Vector3::z()
    }

    pub fn eye_position(&self) -> Vec3 {
        let back = self.back_world();
        vec3(
            self.target[0] + back.x * self.distance,
            self.target[1] + back.y * self.distance,
            self.target[2] + back.z * self.distance,
        )
    }

    pub fn up_vec(&self) -> Vec3 {
        let up = self.up_world();
        vec3(up.x, up.y, up.z)
    }

    pub fn target_vec(&self) -> Vec3 {
        vec3(self.target[0], self.target[1], self.target[2])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn auto_fit_adapts_distance_for_portrait_and_landscape_aspects() {
        let mut orbit = OrbitState::default();
        orbit.fit_to_bounds([0.0, 0.75, 0.0], 0.8, 1.6, CAMERA_FOV_DEG);
        let landscape_dist = orbit.distance;

        orbit.fit_to_bounds([0.0, 0.75, 0.0], 0.8, 0.65, CAMERA_FOV_DEG);
        let portrait_dist = orbit.distance;

        assert!(
            portrait_dist > landscape_dist,
            "narrow portrait viewport must pull camera back so full robot stays inside horizontal FOV"
        );
        assert!((orbit.target[1] - 0.75).abs() < 1e-6);
    }

    #[test]
    fn roll_rotates_camera_up_by_90_degrees_and_chains_rapid_clicks() {
        let mut orbit = OrbitState::default();
        orbit.apply_preset(CameraPreset::Front);
        let back_before = orbit.back_world();
        let up_before = orbit.up_world();

        // Two rapid 90-degree clicks should chain to 180 degrees total roll
        orbit.animate_roll(std::f32::consts::FRAC_PI_2);
        orbit.animate_roll(std::f32::consts::FRAC_PI_2);
        assert!(orbit.advance(0.11));
        assert!(!orbit.advance(0.20));

        let back_after = orbit.back_world();
        let up_after = orbit.up_world();
        assert!((back_before - back_after).norm() < 1e-4, "roll must preserve viewing axis");
        assert!(
            (up_before + up_after).norm() < 1e-4,
            "two chained 90-deg rolls must invert camera up"
        );
    }
}
