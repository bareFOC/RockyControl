//! Auto-fitted orbit camera state and viewing angle presets for `humanoid-viewport`.
//!
//! Camera distance and target are continuously fitted to the active URDF bounding sphere
//! and viewport aspect ratio so the entire robot always remains centered and framed in view.

use three_d::{Vec3, vec3};

pub const CAMERA_FOV_DEG: f32 = 42.0;

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
        match preset {
            CameraPreset::Iso => {
                self.yaw = 0.52;
                self.pitch = 0.25;
            }
            CameraPreset::Front => {
                self.yaw = 0.0;
                self.pitch = 0.12;
            }
            CameraPreset::Side => {
                self.yaw = std::f32::consts::FRAC_PI_2;
                self.pitch = 0.14;
            }
            CameraPreset::Top => {
                self.yaw = 0.0;
                self.pitch = 1.38;
            }
        }
    }

    pub fn apply_preset_for_bounds(
        &mut self,
        preset: CameraPreset,
        center: [f32; 3],
        radius: f32,
    ) {
        self.apply_preset(preset);
        self.fit_to_bounds(center, radius, 1.33, CAMERA_FOV_DEG);
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
}
