//! Portable, deterministic joint-plant dynamics shared by the MCU bench
//! loopback and the software simulator.
//!
//! Everything here is `f32`, fixed-dt and `#![no_std]`. One implementation
//! drives both `BenchLoopbackBus` and `rk_sim::SimulatedServoPlant`, so a
//! controller tuned against the simulator sees the same actuator on hardware.
//!
//! # Dynamics
//!
//! Position mode:
//! `τ = kp·(target − angle) − kd·velocity − friction·velocity + gravity`
//!
//! Velocity mode:
//! `τ = kv·(velocity_cmd − velocity) − friction·velocity`
//!
//! Torque mode: `τ = target` (the target argument is the torque command).
//!
//! In all energized modes `τ` is clamped to `±torque_limit_nm` and angular
//! velocity is integrated with the URDF-derived diagonal inertia from
//! [`rk_control::EFFECTIVE_INERTIA`]. Reported current is derived from the
//! *applied* torque, so saturation is visible to the kernel's current-based
//! impact detector. A non-positive `torque_limit_nm` means torque OFF: the
//! joint holds its angle, reports quiescent current and cools toward ambient.

use rk_control::EFFECTIVE_INERTIA;
use rk_proto::{JointCommandMode, RATED_CURRENT_MA};

/// Electrical conversion used by the plant: 1 N·m per ampere.
///
/// The BareFOC register mapping uses 0.9 N·m/A (1.8 N·m = 2000 mA). The plant
/// deliberately leaves headroom so the 1.8 N·m software ceiling maps to
/// 1800 mA, below the kernel's 1950 mA over-current advisory.
pub const DEFAULT_NM_PER_AMP: f32 = 1.0_f32;

/// Position-loop natural frequency (rad/s) at nominal responsiveness.
pub const DEFAULT_POSITION_BANDWIDTH_RAD_S: f32 = 30.0_f32;
/// Position-loop damping ratio at nominal responsiveness.
pub const DEFAULT_POSITION_DAMPING: f32 = 0.9_f32;
/// Velocity-loop bandwidth (rad/s) at nominal responsiveness.
pub const DEFAULT_VELOCITY_BANDWIDTH_RAD_S: f32 = 30.0_f32;
/// Viscous friction coefficient (N·m·s/rad) per unit lag factor.
pub const FRICTION_PER_LAG_FACTOR: f32 = 0.002_f32;

/// Velocity ceiling for an integrated joint (rad/s).
pub const MAX_VELOCITY_RAD_S: f32 = 14.0_f32;
/// Angle integration guard against a perpetual command (rad).
pub const MAX_ANGLE_RAD: f32 = 1.0e4_f32;
/// Inertia fallback for joints past [`EFFECTIVE_INERTIA`] (kg·m²).
pub const FALLBACK_INERTIA_KG_M2: f32 = 0.01_f32;

/// Quiescent current drawn by an energized joint (mA).
pub const IDLE_CURRENT_MA: u16 = 120;
/// Quiescent current with torque OFF (mA).
pub const TORQUE_OFF_CURRENT_MA: u16 = 45;
/// Current reported by a stalled joint (mA).
pub const STALL_CURRENT_MA: u16 = 2250;
/// Hard ceiling for a stalled-current spike (mA).
pub const STALL_SPIKE_CEILING_MA: u16 = 3400;
/// Ambient temperature the joint cools toward (°C).
pub const AMBIENT_TEMP_C: f32 = 28.0_f32;
/// Lowest temperature the plant may report (°C).
pub const MIN_TEMP_C: f32 = 26.0_f32;
/// Highest temperature the plant may report (°C).
pub const MAX_TEMP_C: f32 = 98.0_f32;

/// Per-joint integration result: `(angle_rad, velocity_rad_s, current_ma, torque_nm)`.
pub type JointSample = (f32, f32, u16, f32);

/// Deterministic fixed-dt actuator model for up to `N` joints.
#[derive(Clone, Debug)]
pub struct JointPlant<const N: usize> {
    joint_count: usize,
    nm_per_amp: f32,
    angle_rad: [f32; N],
    velocity_rad_s: [f32; N],
    current_ma: [u16; N],
    temp_c: [f32; N],
    gravity_nm: [f32; N],
    /// Position stiffness as `bandwidth² · inertia` (s⁻², scaled by inertia).
    position_bandwidth_rad_s: f32,
    position_damping: f32,
    /// Velocity stiffness as `bandwidth · inertia` (s⁻¹, scaled by inertia).
    velocity_bandwidth_rad_s: f32,
    friction_nm_s_per_rad: f32,
}

impl<const N: usize> JointPlant<N> {
    /// Creates a plant with all joints at zero angle/temperature and no load.
    pub const fn new(joint_count: usize, nm_per_amp: f32) -> Self {
        Self {
            joint_count: if joint_count < N { joint_count } else { N },
            nm_per_amp: if nm_per_amp > 0.0_f32 { nm_per_amp } else { DEFAULT_NM_PER_AMP },
            angle_rad: [0.0_f32; N],
            velocity_rad_s: [0.0_f32; N],
            current_ma: [0u16; N],
            temp_c: [AMBIENT_TEMP_C; N],
            gravity_nm: [0.0_f32; N],
            position_bandwidth_rad_s: DEFAULT_POSITION_BANDWIDTH_RAD_S,
            position_damping: DEFAULT_POSITION_DAMPING,
            velocity_bandwidth_rad_s: DEFAULT_VELOCITY_BANDWIDTH_RAD_S,
            friction_nm_s_per_rad: 0.0_f32,
        }
    }

    /// Number of active joints.
    pub const fn joint_count(&self) -> usize {
        self.joint_count
    }

    /// Advances one joint by `dt` seconds and returns its measured state.
    ///
    /// `torque_limit_nm <= 0.0` selects torque OFF (hold + idle + cooling).
    /// In [`JointCommandMode::Torque`] the `target` argument is the torque
    /// command in N·m, clamped to `torque_limit_nm`.
    pub fn step(
        &mut self,
        joint: usize,
        dt_sec: f32,
        mode: JointCommandMode,
        target: f32,
        velocity_cmd: f32,
        torque_limit_nm: f32,
        stalled: bool,
    ) -> JointSample {
        if joint >= self.joint_count || joint >= N {
            return (0.0_f32, 0.0_f32, 0u16, 0.0_f32);
        }
        let dt = dt_sec.clamp(0.001_f32, 0.05_f32);
        let inertia = EFFECTIVE_INERTIA
            .get(joint)
            .copied()
            .unwrap_or(FALLBACK_INERTIA_KG_M2);
        let limit = torque_limit_nm.max(0.0_f32);
        let angle = self.angle_rad[joint];
        let velocity = self.velocity_rad_s[joint];

        let mut torque = if limit <= 0.0_f32 || stalled {
            // Torque OFF (and stall): no motion is produced by the motor.
            0.0_f32
        } else {
            let raw = match mode {
                JointCommandMode::Position => {
                    let kp = self.position_bandwidth_rad_s * self.position_bandwidth_rad_s * inertia;
                    let kd = 2.0_f32 * self.position_damping * self.position_bandwidth_rad_s * inertia;
                    kp * (target - angle)
                        - kd * velocity
                        - self.friction_nm_s_per_rad * velocity
                        + self.gravity_nm[joint]
                }
                JointCommandMode::Velocity => {
                    let kv = self.velocity_bandwidth_rad_s * inertia;
                    kv * (velocity_cmd - velocity) - self.friction_nm_s_per_rad * velocity
                }
                JointCommandMode::Torque => target,
            };
            raw.clamp(-limit, limit)
        };

        let (next_angle, next_velocity) = if !stalled && limit > 0.0_f32 {
            // Semi-implicit Euler: velocity first, then angle.
            let v = (velocity + torque / inertia * dt).clamp(-MAX_VELOCITY_RAD_S, MAX_VELOCITY_RAD_S);
            ((angle + v * dt).clamp(-MAX_ANGLE_RAD, MAX_ANGLE_RAD), v)
        } else {
            torque = 0.0_f32;
            (angle, 0.0_f32)
        };

        let idle = if limit > 0.0_f32 { IDLE_CURRENT_MA } else { TORQUE_OFF_CURRENT_MA };
        let mut ma = idle.saturating_add((torque.abs() / self.nm_per_amp * 1000.0_f32) as u16);
        if ma > RATED_CURRENT_MA {
            ma = RATED_CURRENT_MA;
        }
        if stalled {
            // A locked rotor draws a large current even though the shaft cannot move.
            let spike = STALL_CURRENT_MA as f32
                + (target - angle).abs() * 250.0_f32 * (limit / DEFAULT_NM_PER_AMP).clamp(0.1_f32, 1.0_f32);
            ma = (spike.min(STALL_SPIKE_CEILING_MA as f32)) as u16;
        }

        let amps = ma as f32 * 0.001_f32;
        let heat = amps * amps * 4.0_f32;
        let cool = (self.temp_c[joint] - AMBIENT_TEMP_C) * 0.14_f32;
        self.temp_c[joint] = (self.temp_c[joint] + (heat - cool) * dt).clamp(MIN_TEMP_C, MAX_TEMP_C);

        self.angle_rad[joint] = next_angle;
        self.velocity_rad_s[joint] = next_velocity;
        self.current_ma[joint] = ma;
        (next_angle, next_velocity, ma, torque)
    }

    /// Applies an external kinematic update (teach-mode hand posing, shoves).
    pub fn set_angle(&mut self, joint: usize, angle_rad: f32) {
        if joint < self.joint_count && joint < N && angle_rad.is_finite() {
            self.angle_rad[joint] = angle_rad.clamp(-MAX_ANGLE_RAD, MAX_ANGLE_RAD);
        }
    }

    /// Applies an external velocity update (e.g. a shove impulse).
    pub fn set_velocity(&mut self, joint: usize, velocity_rad_s: f32) {
        if joint < self.joint_count && joint < N && velocity_rad_s.is_finite() {
            self.velocity_rad_s[joint] = velocity_rad_s.clamp(-MAX_VELOCITY_RAD_S, MAX_VELOCITY_RAD_S);
        }
    }

    /// Overrides the reported current (used by the running thrust model).
    pub fn set_current_ma(&mut self, joint: usize, current_ma: u16) {
        if joint < self.joint_count && joint < N {
            self.current_ma[joint] = current_ma;
        }
    }

    /// Supplies the shared gravity torque (N·m) for one joint, computed by
    /// `rk_control::gravity_torque_nm` from the full current pose.
    pub fn set_gravity_torque(&mut self, joint: usize, gravity_nm: f32) {
        if joint < N && gravity_nm.is_finite() {
            self.gravity_nm[joint] = gravity_nm;
        }
    }

    /// Maps an actuator-lag fault factor onto loop bandwidth and friction.
    pub fn set_lag_factor(&mut self, lag_factor: f32) {
        let lag = lag_factor.clamp(0.0_f32, 1.0_f32);
        let scale = (0.75_f32 + 2.0_f32 * lag).clamp(0.4_f32, 1.6_f32);
        self.position_bandwidth_rad_s = DEFAULT_POSITION_BANDWIDTH_RAD_S * scale;
        self.velocity_bandwidth_rad_s = DEFAULT_VELOCITY_BANDWIDTH_RAD_S * scale;
        self.friction_nm_s_per_rad = FRICTION_PER_LAG_FACTOR * lag;
    }

    /// Angle for one joint (rad).
    pub const fn angle(&self, joint: usize) -> f32 {
        self.angle_rad[joint]
    }

    /// Velocity for one joint (rad/s).
    pub const fn velocity(&self, joint: usize) -> f32 {
        self.velocity_rad_s[joint]
    }

    /// Reported current for one joint (mA).
    pub const fn current_ma(&self, joint: usize) -> u16 {
        self.current_ma[joint]
    }

    /// Temperature for one joint (°C).
    pub const fn temperature_c(&self, joint: usize) -> f32 {
        self.temp_c[joint]
    }

    /// All joint angles.
    pub const fn angles(&self) -> &[f32; N] {
        &self.angle_rad
    }

    /// All joint velocities.
    pub const fn velocities(&self) -> &[f32; N] {
        &self.velocity_rad_s
    }

    /// All reported currents.
    pub const fn currents_ma(&self) -> &[u16; N] {
        &self.current_ma
    }

    /// All joint temperatures.
    pub const fn temperatures_c(&self) -> &[f32; N] {
        &self.temp_c
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn step_position(plant: &mut JointPlant<4>, target: f32, limit: f32) -> JointSample {
        plant.step(0, 0.005_f32, JointCommandMode::Position, target, 0.0_f32, limit, false)
    }

    #[test]
    fn torque_saturation_clamps_torque_and_current() {
        let mut plant = JointPlant::<4>::new(4, DEFAULT_NM_PER_AMP);
        let (_, _, ma, torque) = step_position(&mut plant, 10.0_f32, 1.8_f32);
        assert!((torque - 1.8_f32).abs() < 1.0e-6_f32, "torque must saturate, got {torque}");
        let expected = (IDLE_CURRENT_MA as f32 + 1800.0_f32) as u16;
        assert_eq!(ma, expected, "current must track the applied torque");
        assert!(ma <= RATED_CURRENT_MA);
    }

    #[test]
    fn velocity_mode_tracks_commanded_velocity() {
        let mut plant = JointPlant::<4>::new(4, DEFAULT_NM_PER_AMP);
        let mut last = (0.0_f32, 0.0_f32, 0u16, 0.0_f32);
        for _ in 0..80 {
            last = plant.step(
                0,
                0.005_f32,
                JointCommandMode::Velocity,
                0.0_f32,
                3.0_f32,
                1.8_f32,
                false,
            );
        }
        assert!((last.1 - 3.0_f32).abs() < 0.05_f32, "velocity must reach 3 rad/s, got {}", last.1);
        assert!(last.0 > 0.05_f32, "angle must integrate commanded velocity");
    }

    #[test]
    fn torque_mode_applies_clamped_command() {
        let mut plant = JointPlant::<4>::new(4, DEFAULT_NM_PER_AMP);
        let (_, velocity, _, torque) = plant.step(
            0,
            0.005_f32,
            JointCommandMode::Torque,
            5.0_f32,
            0.0_f32,
            1.8_f32,
            false,
        );
        assert!((torque - 1.8_f32).abs() < 1.0e-6_f32);
        assert!(velocity > 0.0_f32);
    }

    #[test]
    fn torque_off_holds_angle_and_cools() {
        let mut plant = JointPlant::<4>::new(4, DEFAULT_NM_PER_AMP);
        let (angled, _, _, _) = step_position(&mut plant, 1.0_f32, 1.8_f32);
        // Warm the joint up with an energized stall spike.
        let warm = plant.step(0, 0.05_f32, JointCommandMode::Position, angled, 0.0_f32, 1.8_f32, true);
        assert_eq!(warm.2, STALL_CURRENT_MA);
        let temp_before = plant.temperature_c(0);
        let (angle, velocity, ma, torque) = step_position(&mut plant, -2.0_f32, 0.0_f32);
        assert_eq!(angle, angled, "torque OFF must hold the angle");
        assert_eq!(velocity, 0.0_f32);
        assert_eq!(ma, TORQUE_OFF_CURRENT_MA);
        assert_eq!(torque, 0.0_f32);
        assert!(plant.temperature_c(0) <= temp_before, "torque OFF must cool toward ambient");
    }

    #[test]
    fn identical_commands_are_bit_identical() {
        let mut a = JointPlant::<4>::new(4, DEFAULT_NM_PER_AMP);
        let mut b = JointPlant::<4>::new(4, DEFAULT_NM_PER_AMP);
        for step in 0..200 {
            let target = (step as f32 * 0.013_f32).sin() * 0.7_f32;
            let sa = step_position(&mut a, target, 1.8_f32);
            let sb = step_position(&mut b, target, 1.8_f32);
            assert_eq!(sa, sb, "step {step} diverged");
        }
    }
}
