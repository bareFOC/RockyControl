//! Hardware-gated combat impact detection for the shared control kernel.
//!
//! Pure `#![no_std]`, strictly single-precision `f32`, deterministic, zero
//! allocation. The module has no `rk-proto` dependency: the integrator owns the
//! mapping from wire/kernel configuration onto [`ImpactConfig`].
//!
//! # Two hardware paths
//!
//! * **Torque-capable (v2/BLDC):** generalized-momentum observer with
//!   `λ̂ = 0` and backward-Euler integration. Reports the L∞ norm of the
//!   residual joint wrench and flags an impact after a short debounce.
//! * **Non-torque (v1/N20, default):** a per-joint motor-current *jump*
//!   heuristic. The jump is converted to an N·m-equivalent pseudo-torque so a
//!   single telemetry channel has consistent units on both hardware paths.
//!
//! The gate is [`ImpactConfig::torque_capable`] and defaults to `false`, so a
//! v1 build keeps its exact current-spike behaviour until firmware explicitly
//! opts in on torque hardware.
//!
//! # Momentum-observer formulation (research §3.2)
//!
//! With `p = M(q) q̇` and `λ̂ = 0` (no contact wrench fed back),
//!
//! ```text
//! β   = Cᵀ q̇ − G + Aᵀ λ̂ + Sᵀ τ_m
//! r_k = ( r_{k−1} + K_O · ( p_k − p_{k−1} − Δt · β_{k−1} ) ) / (1 + K_O · Δt)
//! ```
//!
//! `K_O ≈ 60–100 s⁻¹` gives an observer time constant of 10–17 ms (2–3 ticks
//! at 200 Hz). Backward Euler is unconditionally stable; no `K_O·Δt < 1`
//! restriction is needed. An impact is declared when `‖r‖∞ > r_th`
//! (`r_th ≈ 0.1–0.3 N·m`) for `debounce_ticks` consecutive ticks, followed by
//! a refractory window that prevents chatter.
//!
//! # Model approximations (documented deliberately)
//!
//! A full CRBA/RNEA mass matrix is out of scope for the shared kernel. Instead:
//!
//! 1. `M(q) ≈ diag(I_i)` with **constant** per-joint effective inertias
//!    [`EFFECTIVE_INERTIA`], computed once from the URDF-derived point masses in
//!    [`crate::mass_table`] at the zero pose. The off-diagonal coupling and
//!    each link's rotational inertia tensor are neglected; the momentum
//!    residual therefore sees a conservative, configuration-independent mass
//!    approximation. This is sufficient for the impulsive velocity jumps that
//!    characterize a shove/strike (the observer is a ~10 ms high-pass).
//! 2. `Cᵀ q̇` is neglected: at 200 Hz this robot's joint rates keep the
//!    Coriolis/centrifugal torque well below the 0.1 N·m detection floor.
//! 3. `G` is approximated by the exact gravity torque of the URDF point masses
//!    about each joint axis, propagated through the current joint angles
//!    ([`gravity_torque_nm`]). Neglecting link rotational inertia introduces a
//!    small pose-dependent bias; the observer's fast time constant plus the
//!    debounce keep it below threshold during normal walking.
//! 4. The reported magnitude is an **L∞ joint norm**, not the Euclidean norm.
//!    With 21 joints an L2 norm would scale by √21 and force an artificially
//!    high threshold; L∞ keeps the 0.1–0.3 N·m per-joint threshold meaningful.
//!
//! The residual is telemetry only: it never changes servo commands, so the
//! existing fall/ukemi safety path strictly outranks combat impacts.

use rk_robot_profile::RobotProfile;

use crate::com_fk::MAX_PROFILE_LINKS;
use crate::{cos_f32, sin_f32};

/// Mirrors `rk-proto::MAX_JOINTS` without introducing a wire dependency.
/// The kernel guards the equality with a compile-time assertion.
pub const MAX_IMPACT_JOINTS: usize = 36;
/// Gravitational acceleration used by the gravity-torque approximation (m/s²).
pub const DEFAULT_GRAVITY_M_S2: f32 = 9.81_f32;
/// Observer gain in s⁻¹ (research range 60–100; 80 → 12.5 ms time constant).
pub const DEFAULT_K_O: f32 = 80.0_f32;
/// Residual impact threshold in N·m (research range 0.1–0.3).
pub const DEFAULT_THRESHOLD_NM: f32 = 0.20_f32;
/// Consecutive samples above threshold before an impact is declared.
pub const DEFAULT_DEBOUNCE_TICKS: u8 = 2;
/// Suppression window after a declared impact in seconds (50–100 ms).
pub const DEFAULT_REFRACTORY_S: f32 = 0.08_f32;
/// Per-joint current jump in mA that signals an impact on non-torque hardware.
pub const DEFAULT_CURRENT_SPIKE_MA: f32 = 200.0_f32;
/// mA per N·m used only to express the v1 jump in telemetry's N·m units
/// (2000 mA rated ÷ 1.8 N·m per-joint ceiling ≈ 1111 mA/N·m).
pub const DEFAULT_MA_PER_NM: f32 = 1111.0_f32;

#[inline]
fn finite_or(x: f32, fallback: f32) -> f32 {
    if x.is_finite() { x } else { fallback }
}

// ---------------------------------------------------------------------------
// Constant diagonal mass approximation
// ---------------------------------------------------------------------------

fn zero_pose_origin(profile: &RobotProfile, link: usize) -> [f32; 3] {
    let mut pos = [0.0_f32; 3];
    let mut i = link;
    let mut steps = 0usize;
    while i != profile.pelvis_index && i < profile.links.len() && steps < MAX_PROFILE_LINKS {
        let o = profile.joint_kin[i].origin;
        pos[0] += o[0];
        pos[1] += o[1];
        pos[2] += o[2];
        i = profile.links[i].parent as usize;
        steps += 1;
    }
    pos
}

fn is_descendant_profile(profile: &RobotProfile, link: usize, ancestor: usize) -> bool {
    let mut i = link;
    let mut steps = 0usize;
    while i != profile.pelvis_index && i < profile.links.len() && steps < MAX_PROFILE_LINKS {
        if i == ancestor {
            return true;
        }
        i = profile.links[i].parent as usize;
        steps += 1;
    }
    false
}

/// Diagonal effective inertia about each actuator axis in kg·m² for `profile`.
pub fn effective_inertia_for_profile(profile: &RobotProfile) -> [f32; MAX_IMPACT_JOINTS] {
    let mut out = [1e-4_f32; MAX_IMPACT_JOINTS];
    let link_count = profile.links.len().min(profile.joint_kin.len()).min(MAX_PROFILE_LINKS);
    for j in 0..link_count {
        if j == profile.pelvis_index {
            continue;
        }
        let Some(act_idx) = profile.joint_kin[j].actuator_index else {
            continue;
        };
        let a = act_idx as usize;
        if a >= MAX_IMPACT_JOINTS {
            continue;
        }
        let axis = profile.joint_kin[j].axis;
        let joint_origin = zero_pose_origin(profile, j);
        let mut acc = 0.0_f32;
        for k in 0..link_count {
            if k != profile.pelvis_index && is_descendant_profile(profile, k, j) {
                let origin = zero_pose_origin(profile, k);
                let rx = origin[0] + profile.links[k].com[0] - joint_origin[0];
                let ry = origin[1] + profile.links[k].com[1] - joint_origin[1];
                let rz = origin[2] + profile.links[k].com[2] - joint_origin[2];
                let r2 = rx * rx + ry * ry + rz * rz;
                let along = rx * axis[0] + ry * axis[1] + rz * axis[2];
                acc += profile.links[k].mass * (r2 - along * along);
            }
        }
        out[a] += acc * profile.joint_kin[j].mimic_scale.abs();
    }
    out
}

// ---------------------------------------------------------------------------
// Gravity-torque approximation (point-mass RNEA gravity term)
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Default)]
struct LinkFrame {
    /// World rotation as three column vectors.
    rot: [[f32; 3]; 3],
    /// World position of the link frame origin.
    pos: [f32; 3],
}

#[inline]
fn mat_vec(m: [[f32; 3]; 3], v: [f32; 3]) -> [f32; 3] {
    [
        m[0][0] * v[0] + m[1][0] * v[1] + m[2][0] * v[2],
        m[0][1] * v[0] + m[1][1] * v[1] + m[2][1] * v[2],
        m[0][2] * v[0] + m[1][2] * v[1] + m[2][2] * v[2],
    ]
}

#[inline]
fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

#[inline]
fn rotate_vec(axis: [f32; 3], angle: f32, v: [f32; 3]) -> [f32; 3] {
    let c = cos_f32(angle);
    let s = sin_f32(angle);
    let av = cross(axis, v);
    let dot = axis[0] * v[0] + axis[1] * v[1] + axis[2] * v[2];
    let k = dot * (1.0_f32 - c);
    [
        v[0] * c + av[0] * s + axis[0] * k,
        v[1] * c + av[1] * s + axis[1] * k,
        v[2] * c + av[2] * s + axis[2] * k,
    ]
}

#[inline]
fn compose_rotation(parent: [[f32; 3]; 3], axis: [f32; 3], angle: f32) -> [[f32; 3]; 3] {
    let ex = rotate_vec(axis, angle, [1.0, 0.0, 0.0]);
    let ey = rotate_vec(axis, angle, [0.0, 1.0, 0.0]);
    let ez = rotate_vec(axis, angle, [0.0, 0.0, 1.0]);
    [
        mat_vec(parent, ex),
        mat_vec(parent, ey),
        mat_vec(parent, ez),
    ]
}

/// Gravity torque `τ_grid_i = axis_i · Σ_{k ∈ subtree(i)} (r_{ik} × m_k g)`
/// about each actuator joint, in N·m, at the supplied joint angles.
pub fn gravity_torque_nm(
    profile: &RobotProfile,
    angles: &[f32],
    out: &mut [f32; MAX_IMPACT_JOINTS],
) {
    let identity = [[1.0_f32, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut frames = [LinkFrame {
        rot: identity,
        pos: [0.0; 3],
    }; MAX_PROFILE_LINKS];
    let link_count = profile.links.len().min(profile.joint_kin.len()).min(MAX_PROFILE_LINKS);
    for i in 0..link_count {
        if i == profile.pelvis_index {
            continue;
        }
        let parent = frames[(profile.links[i].parent as usize).min(link_count.saturating_sub(1))];
        let kin = profile.joint_kin[i];
        let t = mat_vec(parent.rot, kin.origin);
        let angle = match kin.actuator_index {
            Some(a) if (a as usize) < angles.len() => angles[a as usize] * kin.mimic_scale,
            _ => 0.0,
        };
        frames[i] = LinkFrame {
            pos: [
                parent.pos[0] + t[0],
                parent.pos[1] + t[1],
                parent.pos[2] + t[2],
            ],
            rot: compose_rotation(parent.rot, kin.axis, angle),
        };
    }

    for slot in out.iter_mut() {
        *slot = 0.0;
    }
    for j in 0..link_count {
        if j == profile.pelvis_index {
            continue;
        }
        let kin = profile.joint_kin[j];
        let Some(act_idx) = kin.actuator_index else {
            continue;
        };
        let a = act_idx as usize;
        if a >= out.len() {
            continue;
        }
        let parent_idx = (profile.links[j].parent as usize).min(link_count.saturating_sub(1));
        let axis = mat_vec(frames[parent_idx].rot, kin.axis);
        let origin = frames[j].pos;
        let mut tau = 0.0_f32;
        for k in 0..link_count {
            if k == profile.pelvis_index || !is_descendant_profile(profile, k, j) {
                continue;
            }
            let com_offset = mat_vec(frames[k].rot, profile.links[k].com);
            let rx = frames[k].pos[0] + com_offset[0] - origin[0];
            let ry = frames[k].pos[1] + com_offset[1] - origin[1];
            let rz = frames[k].pos[2] + com_offset[2] - origin[2];
            let mg = profile.links[k].mass * DEFAULT_GRAVITY_M_S2;
            if profile.z_up {
                // F_g = [0, 0, -m*g] => r x F_g = [-mg*ry, mg*rx, 0]
                tau += axis[0] * (-mg * ry) + axis[1] * (mg * rx);
            } else {
                // F_g = [0, -m*g, 0] => r x F_g = [mg*rz, 0, -mg*rx]
                tau += axis[0] * (mg * rz) - axis[2] * (mg * rx);
            }
        }
        out[a] += tau * kin.mimic_scale;
    }
}

// ---------------------------------------------------------------------------
// Configuration / API
// ---------------------------------------------------------------------------

/// Complete parameter set of the combat impact detector.
///
/// `torque_capable` is `false` by default: v1 firmware keeps the current-spike
/// heuristic until torque hardware is explicitly enabled by the integrator.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImpactConfig {
    /// Selects the momentum observer (`true`) or the v1 current heuristic (`false`).
    pub torque_capable: bool,
    /// Observer gain `K_O` in s⁻¹ (clamped to `[1, 200]`).
    pub k_o: f32,
    /// Residual threshold in N·m (clamped to `[0.01, 5.0]`).
    pub threshold_nm: f32,
    /// Consecutive ticks above threshold before latching (min 1).
    pub debounce_ticks: u8,
    /// Refractory window after a latched impact in seconds (0–1).
    pub refractory_s: f32,
    /// Current-jump threshold in mA for non-torque builds.
    pub current_spike_ma: f32,
    /// mA per N·m used to publish the v1 pseudo-torque.
    pub ma_per_nm: f32,
}

impl Default for ImpactConfig {
    fn default() -> Self {
        Self {
            torque_capable: false,
            k_o: DEFAULT_K_O,
            threshold_nm: DEFAULT_THRESHOLD_NM,
            debounce_ticks: DEFAULT_DEBOUNCE_TICKS,
            refractory_s: DEFAULT_REFRACTORY_S,
            current_spike_ma: DEFAULT_CURRENT_SPIKE_MA,
            ma_per_nm: DEFAULT_MA_PER_NM,
        }
    }
}

impl ImpactConfig {
    /// Sanitized copy: all numeric fields finite and inside their safe ranges.
    pub fn sanitized(&self) -> Self {
        let mut c = *self;
        c.k_o = finite_or(c.k_o, DEFAULT_K_O).clamp(1.0_f32, 200.0_f32);
        c.threshold_nm = finite_or(c.threshold_nm, DEFAULT_THRESHOLD_NM).clamp(0.01_f32, 5.0_f32);
        c.debounce_ticks = c.debounce_ticks.max(1);
        c.refractory_s = finite_or(c.refractory_s, DEFAULT_REFRACTORY_S).clamp(0.0_f32, 1.0_f32);
        c.current_spike_ma =
            finite_or(c.current_spike_ma, DEFAULT_CURRENT_SPIKE_MA).clamp(1.0_f32, 3000.0_f32);
        c.ma_per_nm = finite_or(c.ma_per_nm, DEFAULT_MA_PER_NM).clamp(100.0_f32, 5000.0_f32);
        c
    }
}

/// One detector input tick. All slices may be shorter than
/// [`MAX_IMPACT_JOINTS`]; missing joints are treated as zero.
#[derive(Clone, Copy, Debug)]
pub struct ImpactSample<'a> {
    pub dt_s: f32,
    pub joint_count: usize,
    /// Calibrated joint angles (rad).
    pub angles: &'a [f32],
    /// Calibrated joint rates (rad/s).
    pub velocities: &'a [f32],
    /// Applied joint torques (N·m), already sign-corrected and clamped.
    pub torque_nm: &'a [f32],
    /// Raw motor currents (mA).
    pub current_ma: &'a [u16],
}

/// Detector result for one tick.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ImpactOutput {
    /// True when the debounced impact latched this tick.
    pub impact: bool,
    /// Impact strength in N·m-equivalent (L∞ residual or v1 pseudo-torque).
    pub magnitude_nm: f32,
    /// Instantaneous pre-debounce threshold crossing (useful for logging).
    pub raw_candidate: bool,
}

/// Stateful, allocation-free impact detector.
#[derive(Clone, Copy, Debug)]
pub struct ImpactDetector {
    cfg: ImpactConfig,
    prev_p: [f32; MAX_IMPACT_JOINTS],
    prev_beta: [f32; MAX_IMPACT_JOINTS],
    residual: [f32; MAX_IMPACT_JOINTS],
    /// Slow current baseline (EMA, ≈100 ms at 200 Hz) used by the v1 path so a
    /// raised current level persists long enough for the debounce to confirm.
    current_baseline_ma: [f32; MAX_IMPACT_JOINTS],
    candidate_ticks: u8,
    refractory_left_s: f32,
    initialized: bool,
    last_magnitude_nm: f32,
    latched: bool,
}

impl Default for ImpactDetector {
    fn default() -> Self {
        Self::new(ImpactConfig::default())
    }
}

impl ImpactDetector {
    pub fn new(cfg: ImpactConfig) -> Self {
        Self {
            cfg: cfg.sanitized(),
            prev_p: [0.0; MAX_IMPACT_JOINTS],
            prev_beta: [0.0; MAX_IMPACT_JOINTS],
            residual: [0.0; MAX_IMPACT_JOINTS],
            current_baseline_ma: [0.0; MAX_IMPACT_JOINTS],
            candidate_ticks: 0,
            refractory_left_s: 0.0,
            initialized: false,
            last_magnitude_nm: 0.0,
            latched: false,
        }
    }

    pub const fn config(&self) -> &ImpactConfig {
        &self.cfg
    }

    pub const fn last_magnitude_nm(&self) -> f32 {
        self.last_magnitude_nm
    }

    pub const fn impact_active(&self) -> bool {
        self.latched
    }

    pub const fn residual_nm(&self) -> &[f32; MAX_IMPACT_JOINTS] {
        &self.residual
    }

    /// Replaces the configuration. Any parameter change clears the filter
    /// state so a stale residual/current baseline can never trigger an impact.
    pub fn set_config(&mut self, cfg: ImpactConfig) {
        let next = cfg.sanitized();
        if next != self.cfg {
            self.cfg = next;
            self.reset();
        }
    }

    pub fn set_torque_capable(&mut self, enabled: bool) {
        if enabled != self.cfg.torque_capable {
            self.cfg.torque_capable = enabled;
            self.reset();
        }
    }

    /// Clears momentum/current baselines, debounce and refractory state.
    /// Called on Teach↔Drive transitions and fault clears by the kernel.
    pub fn reset(&mut self) {
        self.prev_p = [0.0; MAX_IMPACT_JOINTS];
        self.prev_beta = [0.0; MAX_IMPACT_JOINTS];
        self.residual = [0.0; MAX_IMPACT_JOINTS];
        self.current_baseline_ma = [0.0; MAX_IMPACT_JOINTS];
        self.candidate_ticks = 0;
        self.refractory_left_s = 0.0;
        self.initialized = false;
        self.last_magnitude_nm = 0.0;
        self.latched = false;
    }

    /// Dispatches to the configured hardware path.
    pub fn step(&mut self, profile: &RobotProfile, sample: &ImpactSample<'_>) -> ImpactOutput {
        if self.cfg.torque_capable {
            self.step_momentum(
                profile,
                sample.dt_s,
                sample.joint_count,
                sample.angles,
                sample.velocities,
                sample.torque_nm,
            )
        } else {
            self.step_current(sample.dt_s, sample.joint_count, sample.current_ma)
        }
    }

    /// Momentum-observer path (torque-capable hardware).
    pub fn step_momentum(
        &mut self,
        profile: &RobotProfile,
        dt_s: f32,
        joint_count: usize,
        angles: &[f32],
        velocities: &[f32],
        torque_nm: &[f32],
    ) -> ImpactOutput {
        let dt = finite_or(dt_s, 0.005_f32).clamp(0.0001_f32, 0.1_f32);
        let n = joint_count
            .min(MAX_IMPACT_JOINTS)
            .min(angles.len())
            .min(velocities.len())
            .min(torque_nm.len());

        let inertia = effective_inertia_for_profile(profile);
        let mut gravity = [0.0_f32; MAX_IMPACT_JOINTS];
        gravity_torque_nm(profile, angles, &mut gravity);

        let k_o = self.cfg.k_o;
        let denom = 1.0_f32 + k_o * dt;
        let mut magnitude = 0.0_f32;
        for i in 0..n {
            let p = inertia[i] * velocities[i];
            // β = Cᵀq̇ − G + Aᵀλ̂ + Sᵀτ_m, with Cᵀq̇ neglected and λ̂ = 0.
            let beta = gravity[i] + torque_nm[i];
            if self.initialized {
                let delta = p - self.prev_p[i] - dt * self.prev_beta[i];
                let r = (self.residual[i] + k_o * delta) / denom;
                self.residual[i] = if r.is_finite() { r } else { 0.0 };
            }
            self.prev_p[i] = p;
            self.prev_beta[i] = beta;
            magnitude = magnitude.max(self.residual[i].abs());
        }
        for slot in self.residual.iter_mut().skip(n) {
            *slot = 0.0;
        }
        // First tick only seeds the baselines; a non-zero initial velocity
        // must never be mistaken for an external impulse.
        if !self.initialized {
            self.initialized = true;
            return ImpactOutput::default();
        }
        self.finish(magnitude, self.cfg.threshold_nm, magnitude, dt)
    }

    /// Current-spike path (v1 hardware, the default).
    ///
    /// Each joint's current is compared to a slow EMA baseline (≈100 ms time
    /// constant, scaled by `dt`) rather than to the previous sample alone. A
    /// genuine impact therefore stays above threshold for several ticks and
    /// can be debounced, while a single stalled-bus sample is rejected.
    pub fn step_current(
        &mut self,
        dt_s: f32,
        joint_count: usize,
        current_ma: &[u16],
    ) -> ImpactOutput {
        let dt = finite_or(dt_s, 0.005_f32).clamp(0.0001_f32, 0.1_f32);
        let n = joint_count.min(MAX_IMPACT_JOINTS).min(current_ma.len());
        let alpha = (dt / 0.1_f32).clamp(0.01_f32, 0.5_f32);
        let mut spike_ma = 0.0_f32;
        for i in 0..n {
            let current = current_ma[i] as f32;
            let baseline = self.current_baseline_ma[i];
            let jump = current - baseline;
            if jump > spike_ma {
                spike_ma = jump;
            }
            self.current_baseline_ma[i] = baseline + alpha * (current - baseline);
        }
        for slot in self.current_baseline_ma.iter_mut().skip(n) {
            *slot = 0.0;
        }
        if !self.initialized {
            for i in 0..n {
                self.current_baseline_ma[i] = current_ma[i] as f32;
            }
            self.initialized = true;
            return ImpactOutput::default();
        }
        let equivalent_nm = spike_ma / self.cfg.ma_per_nm;
        self.finish(spike_ma, self.cfg.current_spike_ma, equivalent_nm, dt)
    }

    /// Shared debounce / refractory logic. `measure` and `threshold` are in
    /// the same (path-specific) units; `reported_nm` is the telemetry value.
    fn finish(&mut self, measure: f32, threshold: f32, reported_nm: f32, dt: f32) -> ImpactOutput {
        let candidate = measure > threshold;
        let magnitude = if reported_nm.is_finite() {
            reported_nm.max(0.0_f32)
        } else {
            0.0_f32
        };
        self.last_magnitude_nm = magnitude;
        self.latched = false;

        if self.refractory_left_s > 0.0_f32 {
            self.refractory_left_s -= dt;
            if self.refractory_left_s <= 0.0_f32 {
                self.refractory_left_s = 0.0;
                self.candidate_ticks = 0;
            }
            return ImpactOutput {
                impact: false,
                magnitude_nm: magnitude,
                raw_candidate: candidate,
            };
        }

        let mut impact = false;
        if candidate {
            self.candidate_ticks = self.candidate_ticks.saturating_add(1);
            if self.candidate_ticks >= self.cfg.debounce_ticks {
                impact = true;
                self.latched = true;
                self.candidate_ticks = 0;
                self.refractory_left_s = self.cfg.refractory_s;
            }
        } else {
            self.candidate_ticks = 0;
        }
        ImpactOutput {
            impact,
            magnitude_nm: magnitude,
            raw_candidate: candidate,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rk_robot_profile::generated::humanoid::PROFILE as HUMANOID_PROFILE;

    fn zeros() -> [f32; MAX_IMPACT_JOINTS] {
        [0.0_f32; MAX_IMPACT_JOINTS]
    }

    #[test]
    fn diagonal_inertia_is_positive_and_symmetric_across_mirrored_limbs() {
        let inertia = effective_inertia_for_profile(&HUMANOID_PROFILE);
        for i in 0..HUMANOID_PROFILE.joint_count {
            assert!(
                inertia[i] > 0.0_f32 && inertia[i].is_finite(),
                "joint {i} inertia must be positive/finite"
            );
        }
        let ji = &HUMANOID_PROFILE.joint_indices;
        for (l, r) in [
            (ji.l_hip_yaw.unwrap() as usize, ji.r_hip_yaw.unwrap() as usize),
            (ji.l_hip_pitch.unwrap() as usize, ji.r_hip_pitch.unwrap() as usize),
            (ji.l_ankle_pitch.unwrap() as usize, ji.r_ankle_pitch.unwrap() as usize),
        ] {
            assert!(
                (inertia[l] - inertia[r]).abs() < 1e-5_f32,
                "mirrored joints {l}/{r} must share inertia: {} vs {}",
                inertia[l],
                inertia[r]
            );
        }
    }

    #[test]
    fn gravity_torque_is_zero_for_vertical_legs_and_restoring_when_leaning() {
        let angles = zeros();
        let mut g = zeros();
        gravity_torque_nm(&HUMANOID_PROFILE, &angles, &mut g);
        let l_hp = HUMANOID_PROFILE.joint_indices.l_hip_pitch.unwrap() as usize;
        let r_hp = HUMANOID_PROFILE.joint_indices.r_hip_pitch.unwrap() as usize;
        assert!(g[l_hp].abs() < 0.05_f32 && g[r_hp].abs() < 0.05_f32);

        let mut leaned = angles;
        leaned[l_hp] = 0.5_f32;
        let mut g_lean = zeros();
        gravity_torque_nm(&HUMANOID_PROFILE, &leaned, &mut g_lean);
        assert!(
            (g_lean[l_hp] - g[l_hp]).abs() > 0.005_f32,
            "leaning hip must change gravity torque: {} vs {}",
            g_lean[l_hp],
            g[l_hp]
        );
    }

    #[test]
    fn gravity_torque_compensation_leaves_residual_quiet() {
        let mut det = ImpactDetector::new(ImpactConfig {
            torque_capable: true,
            ..Default::default()
        });
        let angles = zeros();
        let vel = zeros();
        let mut hold = zeros();
        gravity_torque_nm(&HUMANOID_PROFILE, &angles, &mut hold);
        for i in 0..hold.len() {
            hold[i] = -hold[i];
        }
        for tick in 1..=20 {
            let out = det.step_momentum(
                &HUMANOID_PROFILE,
                0.005_f32,
                HUMANOID_PROFILE.joint_count,
                &angles,
                &vel,
                &hold,
            );
            assert!(
                !out.impact,
                "balanced stance must not flag an impact at tick {tick}"
            );
        }
        assert!(det.last_magnitude_nm() < 0.01_f32);
    }

    #[test]
    fn momentum_observer_detects_injected_unmodeled_torque_when_enabled() {
        let mut det = ImpactDetector::new(ImpactConfig {
            torque_capable: true,
            ..Default::default()
        });
        let angles = zeros();
        let vel = zeros();
        let mut tau = zeros();
        gravity_torque_nm(&HUMANOID_PROFILE, &angles, &mut tau);
        for i in 0..MAX_IMPACT_JOINTS {
            tau[i] = -tau[i];
        }
        let l_hp = HUMANOID_PROFILE.joint_indices.l_hip_pitch.unwrap() as usize;
        tau[l_hp] += 0.6_f32;
        let mut detected = false;
        for _ in 0..10 {
            let out = det.step_momentum(
                &HUMANOID_PROFILE,
                0.005_f32,
                HUMANOID_PROFILE.joint_count,
                &angles,
                &vel,
                &tau,
            );
            if out.impact {
                detected = true;
                assert!(out.magnitude_nm > 0.20_f32);
                break;
            }
        }
        assert!(
            detected,
            "momentum observer must detect injected external torque"
        );
    }

    #[test]
    fn current_jump_detects_impact_and_refractory_suppresses_retrigger() {
        let mut det = ImpactDetector::default();
        assert!(!det.config().torque_capable, "v1 path is the default");
        let quiet = [200_u16; MAX_IMPACT_JOINTS];
        let out0 = det.step_current(0.005_f32, 21, &quiet);
        assert!(!out0.impact, "first tick only seeds baseline");

        let mut spike = quiet;
        spike[11] = 800; // +600 mA in one tick -> 0.54 N·m equivalent
        let out1 = det.step_current(0.005_f32, 21, &spike);
        assert!(out1.raw_candidate);
        assert!(!out1.impact, "one tick must not pass the 2-tick debounce");

        let out2 = det.step_current(0.005_f32, 21, &spike);
        assert!(out2.impact, "sustained jump must latch an impact");
        // Baseline has risen by one EMA step at this point (≈570 mA jump).
        assert!(out2.magnitude_nm > 0.45_f32 && out2.magnitude_nm < 0.60_f32);

        // Still above threshold, but inside the 80 ms refractory window.
        let out3 = det.step_current(0.005_f32, 21, &spike);
        assert!(!out3.impact, "refractory must suppress retrigger");
    }

    #[test]
    fn reset_clears_baselines_without_false_trigger() {
        let mut det = ImpactDetector::default();
        let quiet = [150_u16; MAX_IMPACT_JOINTS];
        det.step_current(0.005_f32, 21, &quiet);
        det.reset();
        let out = det.step_current(0.005_f32, 21, &[1500_u16; MAX_IMPACT_JOINTS]);
        assert!(
            !out.impact,
            "post-reset first tick must reseed, not trigger"
        );
    }
}
