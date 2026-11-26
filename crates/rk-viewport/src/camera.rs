//! Single-axis gravity-locked upright turntable camera with smooth 220 ms horizontal yaw transitions.
//!
//! The camera always keeps world `+Y` (real-world vertical / top) upright with zero roll and a
//! fixed comfortable elevation angle, allowing smooth 360-degree horizontal rotation around the
//! robot without ever tilting or flipping the view upside-down.
//!
//! Camera distance and target are continuously fitted to the active URDF bounding sphere
//! and viewport aspect ratio so the entire robot always remains centered and framed in view.

use std::f32::consts::{FRAC_PI_2, PI, TAU};

use rk_model::nalgebra::{Matrix3, Unit, UnitQuaternion, Vector3};
use three_d::{Vec3, vec3};

pub const CAMERA_FOV_DEG: f32 = 42.0;
const TRANSITION_DURATION_SEC: f32 = 0.22;

/// Fixed upright elevation angle (~19.5 deg above horizon) matching the default Front-Right view.
pub const FIXED_ELEVATION_RAD: f32 = 0.340;
/// Default Front-Right isometric azimuth angle (~36.7 deg).
pub const HOME_YAW_RAD: f32 = 0.641;

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
    pub from_yaw: f32,
    pub to_yaw: f32,
    pub elapsed_sec: f32,
    pub duration_sec: f32,
}

#[derive(Clone, Debug)]
pub struct OrbitState {
    /// Current horizontal turntable azimuth around world `+Y` (radians; 0 = Front `+Z`).
    pub yaw_rad: f32,
    /// Current elevation angle above horizon (radians; defaults to `FIXED_ELEVATION_RAD`).
    pub pitch_rad: f32,
    /// Rotation from camera-local basis (`+X` right, `+Y` up, `+Z` toward eye) to world space.
    /// Always kept strictly synchronized with `turntable_orientation_with_pitch(self.yaw_rad, self.pitch_rad)`.
    pub orientation: UnitQuaternion<f32>,
    pub distance: f32,
    pub target: [f32; 3],
    /// When `true`, per-frame auto-framing (`fit_to_bounds`) is paused so custom agent/user
    /// `target`/`distance` framing persists until `fit_to_rig` or a preset reset is called.
    pub custom_framing: bool,
    pub transition: Option<CameraTransition>,
}

impl Default for OrbitState {
    fn default() -> Self {
        Self {
            yaw_rad: HOME_YAW_RAD,
            pitch_rad: FIXED_ELEVATION_RAD,
            orientation: home_iso_orientation(),
            distance: 2.35,
            target: [0.0, 0.78, 0.0],
            custom_framing: false,
            transition: None,
        }
    }
}

/// Wraps an angle in radians cleanly into `[-PI, PI]`.
fn wrap_angle(rad: f32) -> f32 {
    let mut a = (rad + PI).rem_euclid(TAU) - PI;
    if a <= -PI {
        a += TAU;
    }
    a
}

/// Computes the exact upright gravity-locked turntable camera orientation for azimuth `yaw_rad`
/// and elevation `pitch_rad`. Camera horizon is strictly level (`right.y == 0`), roll is always `0`,
/// and world `+Y` is always up.
pub fn turntable_orientation_with_pitch(yaw_rad: f32, pitch_rad: f32) -> UnitQuaternion<f32> {
    let clamped_pitch = pitch_rad.clamp(-1.45, 1.45);
    let cos_e = clamped_pitch.cos();
    let sin_e = clamped_pitch.sin();
    let eye_dir = Vector3::new(yaw_rad.sin() * cos_e, sin_e, yaw_rad.cos() * cos_e);
    orientation_from_dir_up(eye_dir, Vector3::y())
}

/// Computes the exact upright gravity-locked turntable camera orientation for azimuth `yaw_rad`.
/// Camera horizon is strictly level (`right.y == 0`), roll is always `0`, and world `+Y` is always up.
pub fn turntable_orientation(yaw_rad: f32) -> UnitQuaternion<f32> {
    turntable_orientation_with_pitch(yaw_rad, FIXED_ELEVATION_RAD)
}

/// Default Front-Right upright isometric workstation orientation.
pub fn home_iso_orientation() -> UnitQuaternion<f32> {
    turntable_orientation(HOME_YAW_RAD)
}

/// Constructs a right-handed camera orientation where camera `+Z` points along `eye_dir`
/// (from target toward the camera) and camera `+Y` aligns as closely as possible with `up_hint`.
pub fn orientation_from_dir_up(eye_dir: Vector3<f32>, up_hint: Vector3<f32>) -> UnitQuaternion<f32> {
    let back = Unit::try_new(eye_dir, 1e-6)
        .map(|u| u.into_inner())
        .unwrap_or_else(Vector3::z);

    // Pick a stable fallback up if up_hint is parallel to eye_dir
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

/// Extracts the horizontal turntable yaw angle for a preset (always gravity-locked upright).
pub fn preset_yaw(preset: CameraPreset) -> f32 {
    match preset {
        CameraPreset::Iso | CameraPreset::Top | CameraPreset::Bottom => HOME_YAW_RAD,
        CameraPreset::Front => 0.0,
        CameraPreset::Right | CameraPreset::Side => FRAC_PI_2,
        CameraPreset::Back => PI,
        CameraPreset::Left => -FRAC_PI_2,
    }
}

impl OrbitState {
    pub fn apply_preset(&mut self, preset: CameraPreset) {
        self.yaw_rad = wrap_angle(preset_yaw(preset));
        self.pitch_rad = FIXED_ELEVATION_RAD;
        self.orientation = turntable_orientation_with_pitch(self.yaw_rad, self.pitch_rad);
        self.transition = None;
    }

    pub fn apply_preset_for_bounds(
        &mut self,
        preset: CameraPreset,
        center: [f32; 3],
        radius: f32,
    ) {
        self.custom_framing = false;
        self.pitch_rad = FIXED_ELEVATION_RAD;
        self.animate_to_yaw(preset_yaw(preset));
        self.fit_to_bounds(center, radius, 1.33, CAMERA_FOV_DEG);
    }

    /// Sets custom orbit target, distance, yaw (degrees), and/or pitch (degrees).
    /// Setting `target` or `distance` enables `custom_framing` so per-frame auto-fit
    /// does not overwrite the requested close-up framing.
    pub fn set_custom_orbit(
        &mut self,
        target: Option<[f32; 3]>,
        distance: Option<f32>,
        yaw_deg: Option<f32>,
        pitch_deg: Option<f32>,
    ) {
        self.transition = None;
        if let Some(t) = target {
            if t.iter().all(|v| v.is_finite()) {
                self.target = t;
                self.custom_framing = true;
            }
        }
        if let Some(d) = distance {
            if d.is_finite() {
                self.distance = d.clamp(0.15, 30.0);
                self.custom_framing = true;
            }
        }
        if let Some(yaw) = yaw_deg {
            if yaw.is_finite() {
                self.yaw_rad = wrap_angle(yaw.to_radians());
            }
        }
        if let Some(pitch) = pitch_deg {
            if pitch.is_finite() {
                self.pitch_rad = pitch.to_radians().clamp(-1.45, 1.45);
            }
        }
        self.orientation = turntable_orientation_with_pitch(self.yaw_rad, self.pitch_rad);
    }

    pub fn preset_orientation(&self, preset: CameraPreset) -> UnitQuaternion<f32> {
        turntable_orientation(preset_yaw(preset))
    }

    /// Smoothly rotates camera horizontally around world `+Y` to `target_yaw_rad` along shortest arc.
    pub fn animate_to_yaw(&mut self, target_yaw_rad: f32) {
        self.pitch_rad = FIXED_ELEVATION_RAD;
        let from_yaw = self.yaw_rad;
        let delta = wrap_angle(target_yaw_rad - from_yaw);
        self.transition = Some(CameraTransition {
            from_yaw,
            to_yaw: from_yaw + delta,
            elapsed_sec: 0.0,
            duration_sec: TRANSITION_DURATION_SEC,
        });
    }

    /// Projects any requested orientation onto the gravity-locked horizontal turntable yaw and animates smoothly.
    pub fn animate_to_orientation(&mut self, target_q: UnitQuaternion<f32>) {
        let back = target_q * Vector3::z();
        let horiz = back.x.hypot(back.z);
        let target_yaw = if horiz > 1e-4 {
            back.x.atan2(back.z)
        } else {
            HOME_YAW_RAD
        };
        self.animate_to_yaw(target_yaw);
    }

    /// Smoothly rotates camera horizontally to face `eye_dir` while preserving upright world `+Y`.
    pub fn animate_to_dir_up(&mut self, eye_dir: Vector3<f32>, _up_hint: Vector3<f32>) {
        let horiz = eye_dir.x.hypot(eye_dir.z);
        let target_yaw = if horiz > 1e-4 {
            eye_dir.x.atan2(eye_dir.z)
        } else {
            HOME_YAW_RAD
        };
        self.animate_to_yaw(target_yaw);
    }

    /// Smoothly animates back to the default Front-Right upright isometric view.
    pub fn animate_home(&mut self) {
        self.animate_to_yaw(HOME_YAW_RAD);
    }

    /// Camera roll is disabled in single-axis upright turntable mode so gravity/top always stays level.
    pub fn animate_roll(&mut self, _delta_rad: f32) {
        self.orientation = turntable_orientation_with_pitch(self.yaw_rad, self.pitch_rad);
    }

    /// Advances any active smooth ViewCube horizontal transition; returns `true` while animating.
    pub fn advance(&mut self, dt_sec: f32) -> bool {
        let Some(mut tr) = self.transition.take() else {
            return false;
        };
        tr.elapsed_sec += dt_sec.max(0.0);
        let t = (tr.elapsed_sec / tr.duration_sec.max(1e-4)).clamp(0.0, 1.0);
        // Smooth cubic ease-out: 1 - (1 - t)^3
        let eased = 1.0 - (1.0 - t).powi(3);
        let yaw = tr.from_yaw + (tr.to_yaw - tr.from_yaw) * eased;
        self.yaw_rad = wrap_angle(yaw);
        self.orientation = turntable_orientation_with_pitch(self.yaw_rad, self.pitch_rad);
        if t < 1.0 {
            self.transition = Some(tr);
            true
        } else {
            self.yaw_rad = wrap_angle(tr.to_yaw);
            self.orientation = turntable_orientation_with_pitch(self.yaw_rad, self.pitch_rad);
            false
        }
    }

    /// Single-axis horizontal turntable orbit around world `+Y`.
    /// Vertical drag (`_dy`) is ignored so the robot always stays in its true upright/IMU orientation.
    pub fn drag_orbit(&mut self, dx: f32, _dy: f32) {
        self.transition = None;
        let yaw_delta = -dx * 0.0085;
        self.yaw_rad = wrap_angle(self.yaw_rad + yaw_delta);
        self.orientation = turntable_orientation_with_pitch(self.yaw_rad, self.pitch_rad);
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
    fn turntable_camera_locks_single_axis_upright_rotation() {
        let mut orbit = OrbitState::default();
        orbit.apply_preset(CameraPreset::Front);
        let elev_before = orbit.back_world().y;

        // Large vertical drag must NOT pitch or flip the camera; horizontal drag rotates yaw only
        orbit.drag_orbit(120.0, 600.0);
        assert!(
            (orbit.back_world().y - elev_before).abs() < 1e-5,
            "vertical drag must not alter camera elevation"
        );
        assert!(
            orbit.right_world().y.abs() < 1e-5,
            "camera horizon must remain strictly level (zero roll)"
        );
        assert!(
            orbit.up_world().y > 0.9,
            "world +Y must always remain upright on screen"
        );
        assert!(orbit.yaw_rad.abs() > 0.5, "horizontal drag must rotate yaw around +Y");

        // Roll attempts must be ignored so horizon never tilts
        orbit.animate_roll(FRAC_PI_2);
        assert!(orbit.right_world().y.abs() < 1e-5);

        // 180-degree Front -> Back transition must orbit horizontally, never tipping over top pole
        orbit.apply_preset(CameraPreset::Front);
        orbit.animate_to_yaw(PI);
        assert!(orbit.advance(0.11));
        assert!(
            (orbit.back_world().y - elev_before).abs() < 1e-5,
            "midpoint of 180-deg turn must stay at exact fixed upright elevation"
        );
        assert!(orbit.right_world().y.abs() < 1e-5);
        assert!(!orbit.advance(0.20));
        assert!(orbit.back_world().z < -0.9, "must finish facing Back");
    }

    #[test]
    fn custom_orbit_sets_target_distance_yaw_pitch_and_pauses_auto_fit() {
        let mut orbit = OrbitState::default();
        assert!(!orbit.custom_framing);

        orbit.set_custom_orbit(Some([0.1, 0.35, -0.2]), Some(0.85), Some(90.0), Some(30.0));
        assert!(orbit.custom_framing);
        assert_eq!(orbit.target, [0.1, 0.35, -0.2]);
        assert!((orbit.distance - 0.85).abs() < 1e-6);
        assert!((orbit.yaw_rad - FRAC_PI_2).abs() < 1e-5);
        assert!((orbit.pitch_rad - 30.0_f32.to_radians()).abs() < 1e-5);
        assert!(orbit.right_world().y.abs() < 1e-5);

        orbit.apply_preset_for_bounds(CameraPreset::Iso, [0.0, 0.5, 0.0], 0.8);
        assert!(!orbit.custom_framing);
        assert!((orbit.pitch_rad - FIXED_ELEVATION_RAD).abs() < 1e-6);
    }
}
