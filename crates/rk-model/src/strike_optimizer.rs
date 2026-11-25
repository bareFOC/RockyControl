//! Offline, impact-aware strike keyframe optimizer (host-only, PC tool).
//!
//! This module is **never** linked into firmware. It turns an authored strike
//! (windup pose + impact pose from the Action Studio library) into a new
//! `ActionMove` whose impact keyframe is chosen to maximise distal velocity and
//! impulse under a simplified momentum/impulse model with a capture-point
//! (ZMP/CoM-velocity-area) balance constraint.
//!
//! # Formulation (Wang/Kheddar impact-aware QP, simplified for offline authoring)
//!
//! For the active joints of a strike chain, evaluate a 3-DOF translational
//! Jacobian `J` at mid-stroke and a diagonal joint-space inertia estimate
//! `M = diag(m_d * ||J_i||^2 + I_i)` (point mass `m_d` at the end effector plus
//! a small per-joint inertia). Project onto the strike direction `n` derived
//! from the authored end-effector displacement:
//!
//! ```text
//! j_i   = n · J_i                       (m/rad)
//! Λ⁻¹   = Σ_i j_i² / M_i                 (1/kg, articulated effective mass)
//! d_i   = j_i / M_i                      (M⁻¹ Jᵀ n, EE speed per unit impulse)
//! ```
//!
//! A scalar impulse parameter `α` (in kg·m/s) produces peak joint velocities
//! `q̇_i = α d_i` and peak distal speed `v_n = α Λ⁻¹` along `n`. The contact
//! impulse for coefficient of restitution `c_r` is `I = (1 + c_r) α`, and the
//! resulting CoM recoil is `Δċ = I / m_total`. The balancing constraint is the
//! zero-step capture region `|ξ| = |c + Δċ/ω| ≤ z` with `ω = √(g/h_com)`:
//!
//! ```text
//! α ≤ α_balance = m_total · ω · z / (1 + c_r)
//! ```
//!
//! Keyframes are played with `cubic_smoothstep`, whose peak derivative is 1.5,
//! so a segment of duration `T` reaching `Δq_i = α d_i T / 1.5` has peak joint
//! velocity `α d_i`. That couples two more bounds:
//!
//! ```text
//! α ≤ α_speed = q̇_max / max_i |d_i|
//! α ≤ α_limits = min_i (room_i / (|d_i| T / 1.5))
//! ```
//!
//! The optimum is the closed form `α* = min(α_balance, α_speed, α_limits)` —
//! no runtime solver, no external QP crate, fully deterministic `f32`.
//!
//! Output is a studio-compatible `ActionMove` plus serde-serializable metrics
//! (`StrikeExport`) so an operator can archive/import optimized strikes.

use rk_proto::MAX_JOINTS;
use serde::{Deserialize, Serialize};

use crate::kinematics::{JointKind, KinematicRig, LimbGroup};
use crate::studio::{ActionMove, Keyframe, default_combat_library};

const GRAVITY_M_S2: f32 = 9.81;
/// Peak derivative of `cubic_smoothstep(t) = 3t² - 2t³`.
const SMOOTHSTEP_PEAK_RATE: f32 = 1.5;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum BindingConstraint {
    /// Recoil would leave the capture point outside the support margin.
    Balance,
    /// A joint would exceed its velocity budget at peak speed.
    JointVelocity,
    /// A joint would hit its (margin-inset) URDF limit.
    JointLimit,
}

impl BindingConstraint {
    pub fn label(self) -> &'static str {
        match self {
            Self::Balance => "capture-point balance",
            Self::JointVelocity => "joint velocity budget",
            Self::JointLimit => "joint limits",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct StrikeParams {
    /// Mass lumped at the striking end effector (forearm/hand or shin/foot).
    pub distal_mass_kg: f32,
    /// Baseline rotational inertia of each active joint (kg·m²).
    pub joint_inertia_kg_m2: f32,
    /// Whole-robot mass used to turn contact impulse into CoM recoil.
    pub total_mass_kg: f32,
    /// CoM height used for `ω = √(g/h)` (match the shared estimator default,
    /// 0.72 m, the FK sole-to-CoM height at nominal stance crouch).
    pub com_height_m: f32,
    /// Half-width of the usable support region (foot margin, metres).
    pub support_margin_m: f32,
    /// Coefficient of restitution of the strike against the target.
    pub restitution: f32,
    /// Peak joint-speed budget (rad/s); the bundled URDF lists 6 rad/s.
    pub max_joint_velocity_rad_s: f32,
    /// Safety inset from each joint's URDF limit (rad).
    pub limit_margin_rad: f32,
}

impl Default for StrikeParams {
    fn default() -> Self {
        Self::rocky_mk1()
    }
}

impl StrikeParams {
    /// Parameters for the 1.5 kg Rocky Mk1 with the bundled estimator geometry.
    pub const fn rocky_mk1() -> Self {
        Self {
            distal_mass_kg: 0.20,
            joint_inertia_kg_m2: 0.002,
            total_mass_kg: 1.5,
            com_height_m: 0.72,
            support_margin_m: 0.028,
            restitution: 0.1,
            max_joint_velocity_rad_s: 6.0,
            limit_margin_rad: 0.03,
        }
    }

    /// Heavier distal mass for a leg kick (shin + foot).
    pub const fn kicking(self) -> Self {
        Self {
            distal_mass_kg: 0.35,
            ..self
        }
    }

    pub fn omega(&self) -> f32 {
        (GRAVITY_M_S2 / self.com_height_m.max(0.05)).sqrt()
    }

    fn validate(&self) -> Result<(), OptimizerError> {
        if !self.distal_mass_kg.is_finite()
            || self.distal_mass_kg <= 0.0
            || !self.total_mass_kg.is_finite()
            || self.total_mass_kg <= 0.0
            || self.com_height_m <= 0.0
            || self.support_margin_m <= 0.0
            || self.restitution < 0.0
            || self.max_joint_velocity_rad_s <= 0.0
            || self.joint_inertia_kg_m2 < 0.0
            || self.limit_margin_rad < 0.0
        {
            return Err(OptimizerError::InvalidParameters);
        }
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct StrikeTarget {
    pub name: String,
    pub category: String,
    /// Link whose distal velocity is maximized, e.g. `r_forearm` or `r_foot`.
    pub end_effector_link: String,
    /// Windup / ready pose (first authored keyframe).
    pub windup_pose: [f32; MAX_JOINTS],
    /// Authored impact pose: defines the strike direction and which joints move.
    pub authored_impact_pose: [f32; MAX_JOINTS],
    /// Pose the optimized move returns to (studio Guard).
    pub guard_pose: [f32; MAX_JOINTS],
    /// Duration of the windup→impact keyframe segment (ms).
    pub impact_duration_ms: u32,
    pub params: StrikeParams,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct StrikeMetrics {
    /// Projected inverse effective mass `Λ⁻¹` along the strike direction (1/kg).
    pub inverse_effective_mass_kg_inv: f32,
    /// Effective mass at the end effector along `n` (kg).
    pub effective_mass_kg: f32,
    /// Peak end-effector speed along `n` just before impact (m/s).
    pub peak_ee_speed_m_s: f32,
    /// Predicted contact impulse magnitude (N·s).
    pub impulse_n_s: f32,
    /// Predicted CoM velocity jump from recoil (m/s).
    pub recoil_com_speed_m_s: f32,
    /// Capture-point excursion `Δċ / ω` (m); must not exceed the support margin.
    pub capture_point_excursion_m: f32,
    /// Fraction of the support margin consumed (`≤ 1` by construction).
    pub capture_utilization: f32,
    /// World-space strike direction used by the optimizer.
    pub strike_direction: [f32; 3],
    /// Number of joints participating in the optimized stroke.
    pub active_joint_count: usize,
    /// Which bound determined the optimal impulse.
    pub binding_constraint: BindingConstraint,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OptimizedStrike {
    pub action: ActionMove,
    pub metrics: StrikeMetrics,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StrikeExport {
    pub format_version: u32,
    pub strikes: Vec<OptimizedStrike>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OptimizerError {
    InvalidParameters,
    MissingEndEffector,
    MissingJoint,
    DegenerateStrikeDirection,
    DegenerateJacobian,
    Infeasible,
    NonFiniteResult,
}

impl std::fmt::Display for OptimizerError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let msg = match self {
            Self::InvalidParameters => "invalid strike optimizer parameters",
            Self::MissingEndEffector => "end-effector link is not in the rig",
            Self::MissingJoint => "active joint is not in the rig",
            Self::DegenerateStrikeDirection => "authored strike has no end-effector displacement",
            Self::DegenerateJacobian => "projected Jacobian has no authority along strike direction",
            Self::Infeasible => "no positive impulse satisfies the constraints",
            Self::NonFiniteResult => "optimizer produced a non-finite value",
        };
        f.write_str(msg)
    }
}

impl std::error::Error for OptimizerError {}

/// Optimizes one authored strike into a studio-playable `ActionMove`.
pub fn optimize_strike(
    rig: &KinematicRig,
    target: &StrikeTarget,
) -> Result<OptimizedStrike, OptimizerError> {
    target.params.validate()?;
    if target.impact_duration_ms == 0 {
        return Err(OptimizerError::InvalidParameters);
    }

    // 1. Strike direction from the authored end-effector displacement.
    let start_tf = rig.compute_link_transforms(&target.windup_pose);
    let impact_tf = rig.compute_link_transforms(&target.authored_impact_pose);
    let p_start = distal_point(rig, &target.end_effector_link, &start_tf)
        .ok_or(OptimizerError::MissingEndEffector)?;
    let p_impact = distal_point(rig, &target.end_effector_link, &impact_tf)
        .ok_or(OptimizerError::MissingEndEffector)?;
    let n = (p_impact - p_start)
        .try_normalize(1e-6)
        .ok_or(OptimizerError::DegenerateStrikeDirection)?;

    // 2. Active joints: authored stroke magnitude above a small deadband, in the
    //    striking limb (plus waist yaw for torso torque). Keeps head joints out.
    let mut active: Vec<usize> = Vec::new();
    let mut stroke_norm_sq = 0.0_f32;
    for (idx, jm) in rig.joints.iter().enumerate() {
        let delta = target.authored_impact_pose[idx] - target.windup_pose[idx];
        if delta.abs() <= 0.05 {
            continue;
        }
        let in_limb = jm.group == LimbGroup::LeftArm
            || jm.group == LimbGroup::RightArm
            || jm.group == LimbGroup::LeftLeg
            || jm.group == LimbGroup::RightLeg;
        if in_limb || jm.name == "waist_yaw" {
            active.push(idx);
            stroke_norm_sq += delta * delta;
        }
    }
    if active.is_empty() || !stroke_norm_sq.is_finite() || stroke_norm_sq <= 1e-8 {
        return Err(OptimizerError::DegenerateJacobian);
    }
    let stroke_norm = stroke_norm_sq.sqrt();

    // 3. Jacobian + inertia at mid-stroke, where distal speed peaks.
    let mut q_eval = target.windup_pose;
    for &idx in &active {
        q_eval[idx] = target.windup_pose[idx]
            + 0.5 * (target.authored_impact_pose[idx] - target.windup_pose[idx]);
    }
    let eval_tf = rig.compute_link_transforms(&q_eval);
    let ee = distal_point(rig, &target.end_effector_link, &eval_tf)
        .ok_or(OptimizerError::MissingEndEffector)?
        .coords;

    let params = &target.params;
    // (joint index, d_i = j_i / M_i); gated-out joints are absent.
    let mut directions: Vec<(usize, f32)> = Vec::with_capacity(active.len());
    let mut inv_eff_mass = 0.0_f32;
    for &idx in &active {
        let jm = rig.joints.get(idx).ok_or(OptimizerError::MissingJoint)?;
        let (pivot, axis) = rig
            .joint_world_pivot_and_axis(idx, &eval_tf)
            .ok_or(OptimizerError::MissingJoint)?;
        let col = match jm.kind {
            JointKind::Prismatic => axis,
            JointKind::Revolute | JointKind::Continuous => axis.cross(&(ee - pivot.coords)),
        };
        let j = col.dot(&n);
        let col_speed = col.norm(); // EE speed per unit joint velocity (m or m/rad)
        let inertia = params.distal_mass_kg * col_speed * col_speed + params.joint_inertia_kg_m2;
        if !inertia.is_finite() || inertia <= 0.0 {
            return Err(OptimizerError::DegenerateJacobian);
        }

        // Gate joints that would move opposite to the authored stroke intent.
        let authored_dir = (target.authored_impact_pose[idx] - target.windup_pose[idx]) / stroke_norm;
        let mut d = j / inertia;
        if d * authored_dir < 0.0 {
            d = 0.0;
        }
        if d != 0.0 {
            inv_eff_mass += j * j / inertia;
            directions.push((idx, d));
        }
    }
    if inv_eff_mass <= 1e-9 || !inv_eff_mass.is_finite() {
        return Err(OptimizerError::DegenerateJacobian);
    }

    // 4. Three closed-form upper bounds on the impulse parameter α.
    let segment_s = (target.impact_duration_ms as f32 / 1000.0).max(0.02);
    let delta_q_per_alpha = segment_s / SMOOTHSTEP_PEAK_RATE;

    let balance_cap = params.total_mass_kg * params.omega() * params.support_margin_m
        / (1.0 + params.restitution);
    let max_d = directions
        .iter()
        .map(|(_, d)| d.abs())
        .fold(0.0_f32, f32::max);
    let velocity_cap = params.max_joint_velocity_rad_s / max_d;

    let mut limit_cap = f32::INFINITY;
    let mut impact_pose = target.windup_pose;
    for &(idx, d) in &directions {
        let jm = &rig.joints[idx];
        let room = if d > 0.0 {
            jm.default_max_rad - target.windup_pose[idx] - params.limit_margin_rad
        } else {
            target.windup_pose[idx] - jm.default_min_rad - params.limit_margin_rad
        };
        if room <= 0.0 {
            return Err(OptimizerError::Infeasible);
        }
        limit_cap = limit_cap.min(room / (d.abs() * delta_q_per_alpha));
    }

    let (alpha, binding) = if balance_cap <= velocity_cap && balance_cap <= limit_cap {
        (balance_cap, BindingConstraint::Balance)
    } else if velocity_cap <= limit_cap {
        (velocity_cap, BindingConstraint::JointVelocity)
    } else {
        (limit_cap, BindingConstraint::JointLimit)
    };
    if !alpha.is_finite() || alpha <= 0.0 {
        return Err(OptimizerError::Infeasible);
    }

    // 5. Impact keyframe = windup + α·d·(T / 1.5), clamped to URDF limits.
    for &(idx, d) in &directions {
        impact_pose[idx] += alpha * d * delta_q_per_alpha;
    }
    for (idx, value) in impact_pose.iter_mut().enumerate() {
        if let Some(jm) = rig.joints.get(idx) {
            *value = value.clamp(jm.default_min_rad, jm.default_max_rad);
        } else {
            *value = rig.default_pose()[idx];
        }
    }

    let impulse = (1.0 + params.restitution) * alpha;
    let recoil = impulse / params.total_mass_kg;
    let capture_excursion = recoil / params.omega();
    let capture_util = (capture_excursion / params.support_margin_m).clamp(0.0, 1.0);
    let peak_ee = alpha * inv_eff_mass;
    if !impact_pose.iter().all(|v| v.is_finite())
        || !peak_ee.is_finite()
        || !impulse.is_finite()
    {
        return Err(OptimizerError::NonFiniteResult);
    }

    let recoil_pose = lerp_pose(&impact_pose, &target.guard_pose, 0.5, rig);
    let ready_ms = 260_u32;
    let windup_ms = 220_u32;
    let recoil_ms = 200_u32;
    let guard_ms = 420_u32;

    let action = ActionMove {
        name: target.name.clone(),
        category: target.category.clone(),
        keyframes: vec![
            Keyframe {
                label: "Guard Ready".to_string(),
                duration_ms: ready_ms,
                angles_rad: limit_pose(target.guard_pose, rig),
            },
            Keyframe {
                label: "Windup".to_string(),
                duration_ms: windup_ms,
                angles_rad: limit_pose(target.windup_pose, rig),
            },
            Keyframe {
                label: "Optimized Impact".to_string(),
                duration_ms: target.impact_duration_ms,
                angles_rad: impact_pose,
            },
            Keyframe {
                label: "Recoil".to_string(),
                duration_ms: recoil_ms,
                angles_rad: recoil_pose,
            },
            Keyframe {
                label: "Guard Return".to_string(),
                duration_ms: guard_ms,
                angles_rad: limit_pose(target.guard_pose, rig),
            },
        ],
    };

    let metrics = StrikeMetrics {
        inverse_effective_mass_kg_inv: inv_eff_mass,
        effective_mass_kg: 1.0 / inv_eff_mass,
        peak_ee_speed_m_s: peak_ee,
        impulse_n_s: impulse,
        recoil_com_speed_m_s: recoil,
        capture_point_excursion_m: capture_excursion,
        capture_utilization: capture_util,
        strike_direction: [n.x, n.y, n.z],
        active_joint_count: directions.len(),
        binding_constraint: binding,
    };

    Ok(OptimizedStrike { action, metrics })
}

/// Optimizes every authored strike found in `base_library` (warm start).
///
/// Pass the output of [`default_combat_library`] as `base_library`; authored
/// keyframe 0 is the windup and keyframe 1 defines the impact direction and
/// segment duration. Strikes whose links/names are absent from the rig are
/// skipped deterministically.
pub fn optimize_strikes(
    rig: &KinematicRig,
    base_library: &[ActionMove],
) -> Result<Vec<OptimizedStrike>, OptimizerError> {
    let guard = base_library
        .iter()
        .find(|m| m.name.eq_ignore_ascii_case("guard"))
        .and_then(|m| m.keyframes.first())
        .map(|k| k.angles_rad)
        .unwrap_or_else(|| rig.default_pose());

    let recipes: [(&str, &str, &str, StrikeParams); 3] = [
        ("left jab", "l_forearm", "Strike", StrikeParams::rocky_mk1()),
        ("right cross", "r_forearm", "Strike", StrikeParams::rocky_mk1()),
        ("front kick", "r_foot", "Kick", StrikeParams::rocky_mk1().kicking()),
    ];

    let mut out = Vec::new();
    for (needle, link, category, params) in recipes {
        let Some(base) = base_library.iter().find(|m| {
            let lower = m.name.to_ascii_lowercase();
            lower.contains(needle)
                && m.keyframes.len() >= 2
                && m.keyframes[0].angles_rad != m.keyframes[1].angles_rad
        }) else {
            continue;
        };
        let target = StrikeTarget {
            name: format!("Optimized {}", base.name),
            category: category.to_string(),
            end_effector_link: link.to_string(),
            windup_pose: base.keyframes[0].angles_rad,
            authored_impact_pose: base.keyframes[1].angles_rad,
            guard_pose: guard,
            impact_duration_ms: base.keyframes[1].duration_ms.max(60),
            params,
        };
        if let Ok(optimized) = optimize_strike(rig, &target) {
            out.push(optimized);
        }
    }
    Ok(out)
}

/// Studio-facing convenience wrapper that returns only `ActionMove`s.
///
/// Wire into the Action Studio with `moves.extend(optimized_strikes(&rig, &moves))`.
pub fn optimized_strikes(rig: &KinematicRig, base_library: &[ActionMove]) -> Vec<ActionMove> {
    optimize_strikes(rig, base_library)
        .unwrap_or_default()
        .into_iter()
        .map(|o| o.action)
        .collect()
}

/// Serializes optimized strikes (actions + physics metadata) for offline archive/import.
pub fn export_strikes_json(strikes: &[OptimizedStrike]) -> Result<String, String> {
    let export = StrikeExport {
        format_version: 1,
        strikes: strikes.to_vec(),
    };
    serde_json::to_string_pretty(&export).map_err(|e| format!("strike export failed: {e}"))
}

/// Inverse of [`export_strikes_json`].
pub fn import_strikes_json(json: &str) -> Result<Vec<OptimizedStrike>, String> {
    let export: StrikeExport =
        serde_json::from_str(json).map_err(|e| format!("strike import failed: {e}"))?;
    Ok(export.strikes)
}

/// One-call host tool: optimize the authored library and return exportable JSON.
pub fn optimize_and_export_json(rig: &KinematicRig) -> Result<String, String> {
    let authored = default_combat_library(rig);
    let strikes = optimize_strikes(rig, &authored).map_err(|e| e.to_string())?;
    export_strikes_json(&strikes)
}

/// Lumped distal mass point of a terminal link.
///
/// A URDF terminal link's frame origin sits at its parent joint pivot (e.g.
/// `l_forearm` is rooted at the elbow), so using the raw origin would give a
/// zero lever arm for that joint. The link's visual offset marks the centre of
/// its distal segment, which is the correct point for the inertia/Jacobian
/// estimate. Falls back to the link origin when the rig has no visual for it.
fn distal_point(
    rig: &KinematicRig,
    link: &str,
    transforms: &std::collections::HashMap<String, k::nalgebra::Isometry3<f32>>,
) -> Option<k::nalgebra::Point3<f32>> {
    let iso = transforms.get(link)?;
    let local = rig
        .visuals
        .iter()
        .find(|v| v.link_name == link)
        .map(|v| v.local_offset.translation.vector)
        .unwrap_or_else(k::nalgebra::Vector3::zeros);
    Some(iso * k::nalgebra::Point3::from(local))
}

fn lerp_pose(a: &[f32; MAX_JOINTS], b: &[f32; MAX_JOINTS], t: f32, rig: &KinematicRig) -> [f32; MAX_JOINTS] {
    let mut out = [0.0_f32; MAX_JOINTS];
    for i in 0..MAX_JOINTS {
        out[i] = if let Some(jm) = rig.joints.get(i) {
            (a[i] + (b[i] - a[i]) * t).clamp(jm.default_min_rad, jm.default_max_rad)
        } else {
            rig.default_pose()[i]
        };
    }
    out
}

fn limit_pose(pose: [f32; MAX_JOINTS], rig: &KinematicRig) -> [f32; MAX_JOINTS] {
    let mut out = pose;
    for (i, value) in out.iter_mut().enumerate() {
        if let Some(jm) = rig.joints.get(i) {
            *value = value.clamp(jm.default_min_rad, jm.default_max_rad);
        } else {
            *value = rig.default_pose()[i];
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::studio::sample_action_move;
    use crate::BUNDLED_URDF_XML;

    fn rig() -> KinematicRig {
        KinematicRig::from_urdf_xml(BUNDLED_URDF_XML).expect("bundled rocky_mk1.urdf must parse")
    }

    fn assert_pose_within_limits(rig: &KinematicRig, pose: &[f32; MAX_JOINTS]) {
        for (idx, &angle) in pose.iter().enumerate() {
            assert!(angle.is_finite(), "non-finite angle at joint {idx}");
            if let Some(jm) = rig.joints.get(idx) {
                assert!(
                    angle >= jm.default_min_rad - 1e-5 && angle <= jm.default_max_rad + 1e-5,
                    "joint {} ({}) = {angle} outside URDF limits [{}, {}]",
                    idx,
                    jm.name,
                    jm.default_min_rad,
                    jm.default_max_rad,
                );
            } else {
                assert!((angle - rig.default_pose()[idx]).abs() < 1e-6);
            }
        }
    }

    #[test]
    fn optimized_strikes_have_limit_compliant_keyframes_and_bounded_recoil() {
        let rig = rig();
        let authored = default_combat_library(&rig);
        let strikes = optimize_strikes(&rig, &authored).expect("optimize authored strikes");
        assert_eq!(strikes.len(), 3, "jab, cross and kick should all optimize");

        for strike in &strikes {
            assert!(strike.action.keyframes.len() >= 4);
            assert_eq!(
                strike.action.keyframes.last().unwrap().angles_rad,
                strike.action.keyframes.first().unwrap().angles_rad,
                "optimized strike must end where it started (Guard)",
            );
            for kf in &strike.action.keyframes {
                assert_pose_within_limits(&rig, &kf.angles_rad);
                assert!(kf.duration_ms >= 60);
            }
            let m = strike.metrics;
            assert!(m.impulse_n_s > 0.0, "{}: impulse must be positive", strike.action.name);
            assert!(m.peak_ee_speed_m_s > 0.0);
            assert!(m.effective_mass_kg > 0.0 && m.effective_mass_kg.is_finite());
            assert!(
                m.capture_utilization <= 1.0 + 1e-4,
                "{}: recoil left the capture region",
                strike.action.name,
            );
            assert!(m.capture_point_excursion_m <= rig.joints.len() as f32 * 0.028 + 1e-4);
            assert!(m.active_joint_count > 0);
            assert!(m.strike_direction.iter().all(|v| v.is_finite()));
        }
    }

    #[test]
    fn optimized_action_is_serde_round_trippable_and_playable() {
        let rig = rig();
        let authored = default_combat_library(&rig);
        let strikes = optimize_strikes(&rig, &authored).unwrap();
        for strike in &strikes {
            let json = serde_json::to_string(&strike.action).expect("serialize ActionMove");
            let restored: ActionMove = serde_json::from_str(&json).expect("deserialize ActionMove");
            assert_eq!(restored.name, strike.action.name);
            assert_eq!(restored.keyframes.len(), strike.action.keyframes.len());
            for (a, b) in strike.action.keyframes.iter().zip(restored.keyframes.iter()) {
                assert_eq!(a.label, b.label);
                assert_eq!(a.duration_ms, b.duration_ms);
                assert_eq!(a.angles_rad, b.angles_rad, "f32 must survive JSON round trip");
            }

            let start = rig.default_pose();
            let (p0, done0, _, _) = sample_action_move(&restored, &start, 0.0, 100);
            assert!(!done0);
            assert_pose_within_limits(&rig, &p0);
            let mid = restored.keyframes[0].duration_ms as f32
                + restored.keyframes[1].duration_ms as f32
                + restored.keyframes[2].duration_ms as f32 * 0.5;
            let (pm, done_m, progress, kf) = sample_action_move(&restored, &start, mid, 100);
            assert!(!done_m);
            assert_eq!(kf, 2, "mid-stroke must sample the optimized impact frame");
            assert_pose_within_limits(&rig, &pm);
            assert!(progress > 0.0 && progress < 1.0);

            let total: u32 = restored.keyframes.iter().map(|k| k.duration_ms).sum();
            let (p_end, done, prog, _) = sample_action_move(&restored, &start, total as f32 + 1.0, 100);
            assert!(done);
            assert!((prog - 1.0).abs() < 1e-6);
            assert_eq!(p_end, restored.keyframes.last().unwrap().angles_rad);
            assert_pose_within_limits(&rig, &p_end);
        }
    }

    #[test]
    fn export_round_trips_actions_and_metrics() {
        let rig = rig();
        let json = optimize_and_export_json(&rig).expect("export optimizer JSON");
        let parsed = import_strikes_json(&json).expect("import optimizer JSON");
        assert_eq!(parsed.len(), 3);
        for s in parsed {
            assert!(s.metrics.binding_constraint == BindingConstraint::Balance
                || s.metrics.binding_constraint == BindingConstraint::JointVelocity
                || s.metrics.binding_constraint == BindingConstraint::JointLimit);
            assert!(!s.action.keyframes.is_empty());
        }
        assert!(json.contains("format_version"));
    }

    #[test]
    fn tighter_balance_margin_reduces_impulse() {
        let rig = rig();
        let authored = default_combat_library(&rig);
        let base = authored
            .iter()
            .find(|m| m.name.to_ascii_lowercase().contains("jab"))
            .unwrap();
        let guard = authored.iter().find(|m| m.name == "Guard").unwrap().keyframes[0].angles_rad;
        let make = |margin: f32| {
            let mut params = StrikeParams::rocky_mk1();
            params.support_margin_m = margin;
            optimize_strike(
                &rig,
                &StrikeTarget {
                    name: "probe".to_string(),
                    category: "Strike".to_string(),
                    end_effector_link: "l_forearm".to_string(),
                    windup_pose: base.keyframes[0].angles_rad,
                    authored_impact_pose: base.keyframes[1].angles_rad,
                    guard_pose: guard,
                    impact_duration_ms: base.keyframes[1].duration_ms,
                    params,
                },
            )
            .unwrap()
            .metrics
        };
        let loose = make(0.028);
        let tight = make(0.005);
        assert!(tight.impulse_n_s < loose.impulse_n_s);
        assert_eq!(tight.binding_constraint, BindingConstraint::Balance);
    }

    #[test]
    fn bad_targets_fail_cleanly() {
        let rig = rig();
        let authored = default_combat_library(&rig);
        let base = &authored[1];
        let guard = authored[0].keyframes[0].angles_rad;
        let mut target = StrikeTarget {
            name: "bad".to_string(),
            category: "Strike".to_string(),
            end_effector_link: "no_such_link".to_string(),
            windup_pose: base.keyframes[0].angles_rad,
            authored_impact_pose: base.keyframes[1].angles_rad,
            guard_pose: guard,
            impact_duration_ms: 360,
            params: StrikeParams::rocky_mk1(),
        };
        assert_eq!(
            optimize_strike(&rig, &target).err(),
            Some(OptimizerError::MissingEndEffector),
        );
        target.end_effector_link = "l_forearm".to_string();
        target.params.distal_mass_kg = 0.0;
        assert_eq!(
            optimize_strike(&rig, &target).err(),
            Some(OptimizerError::InvalidParameters),
        );
    }
}
