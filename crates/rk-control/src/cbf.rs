//! Scalar Control Barrier Function (CBF) projections for running safety.
//!
//! Pure `#![no_std]`, strictly single-precision `f32`, `libm` only, zero
//! allocation. Every filter is an **explicit scalar projection** (no QP): a
//! commanded scalar is clamped onto the intersection of a forward-invariant
//! safe set and (where applicable) the physical support box.
//!
//! Three barriers are provided, all with the same exponential decay rate
//! `κ ≈ 3 s⁻¹` (WBC §5 / P3 design decision 4):
//!
//! 1. **Torso tilt (relative degree 2, ECBF).** `h = θ_max² − θ²`, requiring
//!    `ḣ + κ h ≥ 0` gives `2 θ θ̇ ≤ κ (θ_max² − θ²)`. A torque command is
//!    projected by first bounding the resulting tilt *rate*.
//! 2. **CoM velocity (relative degree 1).** `h = v_max² − ċ²`, with
//!    `c̈ ≈ ω²(c − p)` affine in the ZMP `p`. `−2ċ ω²(c − p) ≥ −κ h` yields a
//!    linear lower/upper bound on `p`.
//! 3. **Capture point (relative degree 1).** `h = ξ_max² − ‖ξ − p‖²` with
//!    `ξ̇ = ω(ξ − p)` yields `‖ξ − p‖² ≤ κ ξ_max² / (2ω + κ)`, i.e. a disk
//!    around `ξ` intersected with the support box.
//!
//! [`shield_torque`] composes all three into the minimal-modification scalar
//! clamp used by the SLIP running overlay in the control kernel.

use libm::sqrtf;

/// Default barrier decay rate in s⁻¹ (P3 design decision 4).
pub const DEFAULT_KAPPA: f32 = 3.0;
/// Torso tilt barrier: `0.5 rad` sits below `ukemi_tilt_rad = 0.56` and
/// `fall_cutoff_rad = 0.65`, so the shield engages before the safety latches.
pub const DEFAULT_TILT_MAX_RAD: f32 = 0.5;
/// Running CoM velocity ceiling in m/s (research §2.4).
pub const DEFAULT_COM_VEL_MAX_M_S: f32 = 0.6;
/// Support-relative capture-point radius in meters (usable foot margin).
pub const DEFAULT_CAPTURE_MAX_M: f32 = 0.028;
/// LIPM natural frequency `sqrt(9.81 / 0.25)` used by the estimator.
pub const DEFAULT_OMEGA: f32 = 6.26;

#[inline]
fn finite_or(x: f32, fallback: f32) -> f32 {
    if x.is_finite() {
        x
    } else {
        fallback
    }
}

#[inline]
fn clamp(x: f32, lo: f32, hi: f32) -> f32 {
    if x < lo {
        lo
    } else if x > hi {
        hi
    } else {
        x
    }
}

/// Tunable parameters of the scalar CBF shield.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CbfConfig {
    /// Exponential CBF decay rate `κ` in s⁻¹.
    pub kappa: f32,
    /// Maximum safe torso tilt magnitude in radians.
    pub tilt_max_rad: f32,
    /// Maximum safe sagittal CoM velocity magnitude in m/s.
    pub com_vel_max_m_s: f32,
    /// Maximum safe support-relative capture-point distance in meters.
    pub capture_max_m: f32,
    /// LIPM natural frequency `ω = sqrt(g / h_com)` in rad/s.
    pub omega: f32,
}

impl Default for CbfConfig {
    fn default() -> Self {
        Self {
            kappa: DEFAULT_KAPPA,
            tilt_max_rad: DEFAULT_TILT_MAX_RAD,
            com_vel_max_m_s: DEFAULT_COM_VEL_MAX_M_S,
            capture_max_m: DEFAULT_CAPTURE_MAX_M,
            omega: DEFAULT_OMEGA,
        }
    }
}

/// Projects a torso tilt *rate* onto `ḣ + κ h ≥ 0` for `h = θ_max² − θ²`.
///
/// Derived bound: `2 θ θ̇ ≤ κ (θ_max² − θ²)`.
/// - `θ > 0` (forward lean) imposes an upper bound on `θ̇`.
/// - `θ < 0` imposes a lower bound on `θ̇`.
/// - Near upright (`|θ| < 1e-4`) the barrier is inactive.
/// - Already outside the set (`h ≤ 0`) drives the rate back toward the interior.
pub fn project_tilt_rate(tilt_rad: f32, tilt_rate_rad_s: f32, cfg: &CbfConfig) -> f32 {
    let theta = finite_or(tilt_rad, 0.0);
    let theta_dot = finite_or(tilt_rate_rad_s, 0.0);
    let theta_max = finite_or(cfg.tilt_max_rad, DEFAULT_TILT_MAX_RAD).max(0.05_f32);
    let kappa = finite_or(cfg.kappa, DEFAULT_KAPPA).max(0.0_f32);
    let h = theta_max * theta_max - theta * theta;

    if h <= 0.0_f32 {
        // Unsafe: command a rate back toward the safe interior.
        return if theta >= 0.0_f32 { -kappa * theta_max } else { kappa * theta_max };
    }
    if theta > 1e-4_f32 {
        let rate_max = kappa * h / (2.0_f32 * theta);
        theta_dot.min(rate_max)
    } else if theta < -1e-4_f32 {
        // theta is negative: dividing by 2*theta flips the inequality direction.
        let rate_min = kappa * h / (2.0_f32 * theta);
        theta_dot.max(rate_min)
    } else {
        theta_dot
    }
}

/// Projects a scalar torque command through the relative-degree-2 tilt ECBF.
///
/// `θ̇ₙₑₓₜ = θ̇ + a·u·dt` with control authority `a > 0` (rad/s² per N·m).
/// The projected torque is the minimal scalar modification that makes the
/// resulting tilt rate satisfy [`project_tilt_rate`].
pub fn project_tilt_torque(
    torque_nm: f32,
    tilt_rad: f32,
    tilt_rate_rad_s: f32,
    cfg: &CbfConfig,
    authority: f32,
    dt_s: f32,
) -> f32 {
    let u = finite_or(torque_nm, 0.0);
    let dt = finite_or(dt_s, 0.0).clamp(0.0_f32, 0.05_f32);
    let a = finite_or(authority, 1.0_f32).abs().max(1e-3_f32);
    if dt <= 0.0_f32 {
        return u;
    }
    let rate_cmd = finite_or(tilt_rate_rad_s, 0.0) + a * u * dt;
    let rate_safe = project_tilt_rate(tilt_rad, rate_cmd, cfg);
    // Undo the assumed dynamics to recover the admissible torque.
    u + (rate_safe - rate_cmd) / (a * dt)
}

/// Projects a commanded support point (ZMP) `p` through the CoM-velocity CBF.
///
/// `h = v_max² − ċ² ≥ 0`, `c̈ ≈ ω²(c − p)` ⇒
/// `−2 ċ ω²(c − p) ≥ −κ h`. For forward velocity this is a lower bound on
/// `p`; for backward velocity an upper bound. The result is intersected with
/// `[support_min, support_max]`, i.e. minimal movement of `p_cmd`.
pub fn project_com_velocity(
    p_cmd: f32,
    com_pos: f32,
    com_vel: f32,
    cfg: &CbfConfig,
    support_min: f32,
    support_max: f32,
) -> f32 {
    let p_cmd = finite_or(p_cmd, 0.0);
    let c = finite_or(com_pos, 0.0);
    let v = finite_or(com_vel, 0.0);
    let v_max = finite_or(cfg.com_vel_max_m_s, DEFAULT_COM_VEL_MAX_M_S).max(0.05_f32);
    let omega = finite_or(cfg.omega, DEFAULT_OMEGA).max(0.1_f32);
    let kappa = finite_or(cfg.kappa, DEFAULT_KAPPA).max(0.0_f32);

    let mut lo = finite_or(support_min, -1.0_f32).min(finite_or(support_max, 1.0_f32));
    let mut hi = finite_or(support_min, -1.0_f32).max(finite_or(support_max, 1.0_f32));
    let h = v_max * v_max - v * v;

    if h > 0.0_f32 && v.abs() > 1e-4_f32 {
        let bound = kappa * h / (2.0_f32 * v.abs() * omega * omega);
        if v > 0.0_f32 {
            lo = lo.max(c - bound);
        } else {
            hi = hi.min(c + bound);
        }
    } else if h <= 0.0_f32 {
        // Already unsafe: collapse to the end of the box opposing the velocity.
        if v > 0.0_f32 {
            lo = hi;
        } else {
            hi = lo;
        }
    }

    if lo > hi {
        let mid = 0.5_f32 * (lo + hi);
        lo = mid;
        hi = mid;
    }
    clamp(p_cmd, lo, hi)
}

/// Projects a commanded support point `p` through the capture-point CBF.
///
/// `h = ξ_max² − ‖ξ − p‖² ≥ 0`, `ξ̇ = ω(ξ − p)` ⇒
/// `‖ξ − p‖² ≤ κ ξ_max² / (2ω + κ)`. The admissible interval is the disk
/// of that radius around `ξ` intersected with `[support_min, support_max]`.
pub fn project_capture_point(
    p_cmd: f32,
    capture_point: f32,
    cfg: &CbfConfig,
    support_min: f32,
    support_max: f32,
) -> f32 {
    let p_cmd = finite_or(p_cmd, 0.0);
    let xi = finite_or(capture_point, 0.0);
    let xi_max = finite_or(cfg.capture_max_m, DEFAULT_CAPTURE_MAX_M).max(0.005_f32);
    let omega = finite_or(cfg.omega, DEFAULT_OMEGA).max(0.1_f32);
    let kappa = finite_or(cfg.kappa, DEFAULT_KAPPA).max(0.0_f32);

    let radius_sq = kappa * xi_max * xi_max / (2.0_f32 * omega + kappa).max(1e-3_f32);
    let radius = sqrtf(radius_sq.max(0.0_f32));
    let mut lo = finite_or(support_min, -1.0_f32).min(finite_or(support_max, 1.0_f32));
    let mut hi = finite_or(support_min, -1.0_f32).max(finite_or(support_max, 1.0_f32));
    lo = lo.max(xi - radius);
    hi = hi.min(xi + radius);
    if lo > hi {
        let mid = 0.5_f32 * (lo + hi);
        lo = mid;
        hi = mid;
    }
    clamp(p_cmd, lo, hi)
}

/// Composed minimal-modification shield applied to a scalar leg torque command.
///
/// Order: tilt ECBF (hard scalar clamp on torque) followed by scalar margin
/// projections for the CoM-velocity and capture-point barriers. Both margins
/// equal 1 at the centre of the safe set and decay to 0 on its boundary, so a
/// command that would leave the safe set is reduced rather than accepted.
pub fn shield_torque(
    torque_nm: f32,
    tilt_rad: f32,
    tilt_rate_rad_s: f32,
    com_vel_m_s: f32,
    capture_err_m: f32,
    cfg: &CbfConfig,
    authority: f32,
    dt_s: f32,
) -> f32 {
    let u = project_tilt_torque(torque_nm, tilt_rad, tilt_rate_rad_s, cfg, authority, dt_s);
    let v_max = finite_or(cfg.com_vel_max_m_s, DEFAULT_COM_VEL_MAX_M_S).max(0.05_f32);
    let xi_max = finite_or(cfg.capture_max_m, DEFAULT_CAPTURE_MAX_M).max(0.005_f32);

    let h_v = v_max * v_max - finite_or(com_vel_m_s, 0.0) * finite_or(com_vel_m_s, 0.0);
    let alpha_v = if h_v <= 0.0_f32 {
        0.0_f32
    } else {
        clamp(h_v / (v_max * v_max), 0.0_f32, 1.0_f32)
    };
    let err = finite_or(capture_err_m, 0.0);
    let h_xi = xi_max * xi_max - err * err;
    let alpha_xi = if h_xi <= 0.0_f32 {
        0.0_f32
    } else {
        clamp(h_xi / (xi_max * xi_max), 0.0_f32, 1.0_f32)
    };
    u * alpha_v * alpha_xi
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_config_uses_kappa_3_and_expected_safe_sets() {
        let cfg = CbfConfig::default();
        assert_eq!(cfg.kappa, 3.0);
        assert_eq!(cfg.tilt_max_rad, 0.5);
        assert_eq!(cfg.com_vel_max_m_s, 0.6);
        assert_eq!(cfg.capture_max_m, 0.028);
    }

    #[test]
    fn tilt_cbf_reduces_forward_rate_near_the_limit() {
        let cfg = CbfConfig::default();
        // At theta = 0.49 rad the safe upper rate is kappa*h/(2*theta)
        // = 3*(0.25-0.2401)/(0.98) = 0.0303 rad/s.
        let safe = project_tilt_rate(0.49, 0.5, &cfg);
        assert!(safe < 0.5, "tilt CBF must clamp an excessive forward rate");
        assert!((safe - 0.03030612).abs() < 1e-4, "rate bound = {safe}");
        // Negative lean clamps from below.
        let safe_neg = project_tilt_rate(-0.49, -0.5, &cfg);
        assert!(safe_neg > -0.5 && safe_neg < 0.0);
        // Upright: barrier inactive.
        assert_eq!(project_tilt_rate(0.0, 0.2, &cfg), 0.2);
    }

    #[test]
    fn tilt_torque_projection_modifies_a_command_that_would_deepen_the_lean() {
        let cfg = CbfConfig::default();
        let dt = 0.005_f32;
        let authority = 4.0_f32;
        // Already at 0.49 rad and still leaning forward: a large positive
        // push-off torque must be reduced (or reversed) by the ECBF.
        let pushed = 1.8_f32;
        let clamped = project_tilt_torque(pushed, 0.49, 0.0, &cfg, authority, dt);
        assert!(clamped < pushed, "ECBF must reduce the push torque: {clamped}");
        assert!(clamped > 0.0, "the admissible torque is reduced, not reversed: {clamped}");
        assert!((clamped - 1.5153044_f32).abs() < 1e-4, "projection = {clamped}");
        // Upright and quiet: command passes through untouched.
        let free = project_tilt_torque(pushed, 0.0, 0.0, &cfg, authority, dt);
        assert_eq!(free, pushed);
    }

    #[test]
    fn com_velocity_cbf_clips_zmp_command_to_the_support_box() {
        let cfg = CbfConfig::default();
        // Forward CoM velocity: barrier forces p >= c - bound.
        let p = project_com_velocity(-0.10, 0.0, 0.5, &cfg, -0.035, 0.035);
        assert!(p > -0.10, "forward CBF must raise the support point: {p}");
        assert!(p >= -0.035);
        // Backward CoM velocity: barrier forces p <= c + bound.
        let p2 = project_com_velocity(0.10, 0.0, -0.5, &cfg, -0.035, 0.035);
        assert!(p2 < 0.10, "backward CBF must lower the support point: {p2}");
        // Quiet: command passes unchanged.
        assert_eq!(project_com_velocity(0.01, 0.0, 0.0, &cfg, -0.035, 0.035), 0.01);
    }

    #[test]
    fn capture_point_cbf_keeps_support_point_inside_the_disk() {
        let cfg = CbfConfig::default();
        // xi = 0.05 m ahead; the CBF disk radius is
        // sqrt(3*0.028^2 / (2*6.26+3)) = 0.01231 m, so p must be within
        // [0.03769, 0.06231]. A command at the origin is projected forward.
        let p = project_capture_point(0.0, 0.05, &cfg, -0.16, 0.16);
        assert!(p > 0.03, "capture CBF must move the support point toward xi: {p}");
        assert!(p <= 0.05 + 0.0125);
        assert!(p >= 0.05 - 0.0125);
        // Command inside the ξ=0 disk (radius 0.01231 m): unchanged.
        assert_eq!(project_capture_point(0.01, 0.0, &cfg, -0.16, 0.16), 0.01);
    }

    #[test]
    fn shield_torque_modifies_tilt_and_capture_violations() {
        let cfg = CbfConfig::default();
        // Safe centre: tilt projection inactive, margins are 1 -> unchanged.
        assert_eq!(
            shield_torque(0.9, 0.0, 0.0, 0.0, 0.0, &cfg, 4.0, 0.005),
            0.9
        );
        // Tilt violation near the 0.5 rad boundary: torque reduced.
        let tilted = shield_torque(0.9, 0.499, 0.0, 0.0, 0.0, &cfg, 4.0, 0.005);
        assert!(tilted < 0.9);
        // Capture point on the boundary: torque zeroed.
        let captured = shield_torque(0.9, 0.0, 0.0, 0.0, 0.028, &cfg, 4.0, 0.005);
        assert_eq!(captured, 0.0);
    }
}
