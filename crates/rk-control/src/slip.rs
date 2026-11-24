//! SLIP (Spring-Loaded Inverted Pendulum) running template for torque-capable hardware.
//!
//! Pure `#![no_std]`, strictly single-precision `f32`, `libm` only, zero
//! allocation. This module has **no `rk-proto` dependency**: the host/wire
//! `RunningConfig` added by the protocol work may map onto [`SlipConfig`], but
//! the controller itself stays hardware-agnostic and simulation-reusable.
//!
//! # Model (P3 research §2, scaled to a 1/4-scale 1.5 kg humanoid)
//!
//! - Vertical thrust spring: `F = k (L₀ − L)`, `k ≈ 670 N/m`, virtual leg
//!   rest length `L₀ ≈ 0.22 m` (derived from leg geometry). Clamped at zero:
//!   the leg can push but never pull.
//! - Target hop `H = 0.02..0.04 m` (default `0.03 m`) ⇒ takeoff velocity
//!   `v_TO = sqrt(2 g H) = 0.63..0.89 m/s`.
//! - Stance is the SLIP half-period `T_s = π / ω_s` with
//!   `ω_s = sqrt(k/m) ≈ 21.1 rad/s` ⇒ `T_s ≈ 149 ms` (> 24 ticks at 200 Hz).
//! - Flight detection: low summed leg current **and** IMU free-fall signature
//!   (`|a_m| < 0.3 g`), with 10–15 ms entry debounce and a faster (< 5 ms)
//!   touchdown re-entry debounce.
//! - Landing impedance: virtual spring `800 → 4000 N/m` ramped over 25–40 ms
//!   with `D ≈ 40–60 N·s/m` damping; joint torque `τ = F · moment_arm`.

use core::f32::consts::PI;

use libm::sqrtf;

/// Default robot mass used by the P3 SLIP study (kg). `mass_table::TOTAL_MASS_KG`
/// is 1.487 kg; 1.5 kg keeps the published gains exact.
pub const DEFAULT_MASS_KG: f32 = 1.5_f32;
/// Thrust spring stiffness in N/m (research §2.1).
pub const DEFAULT_SPRING_K_N_PER_M: f32 = 670.0_f32;
/// Virtual leg rest length in meters for a 0.62 m nominal hip height.
pub const DEFAULT_REST_LENGTH_M: f32 = 0.22_f32;
/// Default target hop height in meters (2–4 cm envelope).
pub const DEFAULT_TARGET_HOP_M: f32 = 0.03_f32;
/// Gravitational acceleration in m/s².
pub const DEFAULT_GRAVITY_M_S2: f32 = 9.81_f32;
/// Flight-entry debounce in seconds (12 ms → ~2–3 ticks at 200 Hz).
pub const DEFAULT_FLIGHT_DEBOUNCE_S: f32 = 0.012_f32;
/// Touchdown re-entry debounce in seconds (4 ms → < 1 tick at 200 Hz).
pub const DEFAULT_TOUCHDOWN_DEBOUNCE_S: f32 = 0.004_f32;
/// Summed leg-current flight threshold in mA (~10% of stance peak).
pub const DEFAULT_FLIGHT_CURRENT_THRESHOLD_MA: f32 = 200.0_f32;
/// Specific-force fraction of gravity below which the IMU reports free fall.
pub const DEFAULT_FREEFALL_FRACTION: f32 = 0.30_f32;
/// Landing stiffness at touchdown in N/m.
pub const DEFAULT_LANDING_K0_N_PER_M: f32 = 800.0_f32;
/// Landing stiffness at the end of the ramp in N/m.
pub const DEFAULT_LANDING_K1_N_PER_M: f32 = 4000.0_f32;
/// Landing stiffness ramp duration in seconds (30 ms, inside 25–40 ms).
pub const DEFAULT_LANDING_RAMP_S: f32 = 0.030_f32;
/// Landing leg damping in N·s/m (ζ ≈ 0.7 for m_eff ≈ 0.75 kg).
pub const DEFAULT_LANDING_DAMPING_N_S_PER_M: f32 = 50.0_f32;
/// Effective moment arm converting vertical leg force to joint torque (m).
pub const DEFAULT_MOMENT_ARM_M: f32 = 0.05_f32;
/// Conservative ceiling on commanded leg force in N.
pub const DEFAULT_MAX_FORCE_N: f32 = 60.0_f32;
/// Default CBF decay rate shared with [`crate::cbf`].
pub const DEFAULT_CBF_KAPPA: f32 = 3.0_f32;

#[inline]
fn finite_or(x: f32, fallback: f32) -> f32 {
    if x.is_finite() { x } else { fallback }
}

/// Complete parameter set of the SLIP running template.
///
/// `enabled` is `false` by default so v1 builds are completely unaffected.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SlipConfig {
    /// Runtime gate. Off by default; the kernel owns the authoritative flag.
    pub enabled: bool,
    /// Robot mass in kg.
    pub mass_kg: f32,
    /// Vertical thrust spring stiffness in N/m.
    pub spring_k_n_per_m: f32,
    /// Virtual leg rest length in meters.
    pub rest_length_m: f32,
    /// Target hop height in meters (clamped to `[0.02, 0.04]`).
    pub target_hop_m: f32,
    /// Gravitational acceleration in m/s².
    pub gravity_m_s2: f32,
    /// Flight-entry debounce in seconds (10–15 ms).
    pub flight_debounce_s: f32,
    /// Touchdown re-entry debounce in seconds (< 5 ms).
    pub touchdown_debounce_s: f32,
    /// Summed leg-current flight threshold in mA.
    pub flight_current_threshold_ma: f32,
    /// IMU specific-force fraction of gravity for free-fall confirmation.
    pub freefall_fraction: f32,
    /// Landing stiffness at touchdown in N/m.
    pub landing_k0_n_per_m: f32,
    /// Landing stiffness at ramp end in N/m.
    pub landing_k1_n_per_m: f32,
    /// Landing stiffness ramp duration in seconds (25–40 ms).
    pub landing_ramp_s: f32,
    /// Landing damping in N·s/m (40–60).
    pub landing_damping_n_s_per_m: f32,
    /// Force-to-joint-torque moment arm in meters.
    pub moment_arm_m: f32,
    /// Maximum commanded leg force in N.
    pub max_force_n: f32,
    /// CBF decay rate `κ` in s⁻¹ forwarded to [`crate::cbf`].
    pub cbf_kappa: f32,
}

impl Default for SlipConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            mass_kg: DEFAULT_MASS_KG,
            spring_k_n_per_m: DEFAULT_SPRING_K_N_PER_M,
            rest_length_m: DEFAULT_REST_LENGTH_M,
            target_hop_m: DEFAULT_TARGET_HOP_M,
            gravity_m_s2: DEFAULT_GRAVITY_M_S2,
            flight_debounce_s: DEFAULT_FLIGHT_DEBOUNCE_S,
            touchdown_debounce_s: DEFAULT_TOUCHDOWN_DEBOUNCE_S,
            flight_current_threshold_ma: DEFAULT_FLIGHT_CURRENT_THRESHOLD_MA,
            freefall_fraction: DEFAULT_FREEFALL_FRACTION,
            landing_k0_n_per_m: DEFAULT_LANDING_K0_N_PER_M,
            landing_k1_n_per_m: DEFAULT_LANDING_K1_N_PER_M,
            landing_ramp_s: DEFAULT_LANDING_RAMP_S,
            landing_damping_n_s_per_m: DEFAULT_LANDING_DAMPING_N_S_PER_M,
            moment_arm_m: DEFAULT_MOMENT_ARM_M,
            max_force_n: DEFAULT_MAX_FORCE_N,
            cbf_kappa: DEFAULT_CBF_KAPPA,
        }
    }
}

impl SlipConfig {
    /// Derives the virtual rest length from a nominal hip-to-ankle height
    /// (`L₀ ≈ 0.355 · h_hip`, giving 0.22 m for Rocky Mk1's 0.62 m geometry).
    pub fn from_nominal_height(hip_height_m: f32) -> Self {
        let mut cfg = Self::default();
        let l0 = 0.355_f32 * finite_or(hip_height_m, 0.62_f32);
        cfg.rest_length_m = if l0 < 0.10_f32 {
            0.10_f32
        } else if l0 > 0.30_f32 {
            0.30_f32
        } else {
            l0
        };
        cfg
    }

    /// Validates/clamps every field so downstream math never sees NaN or a
    /// nonsensical gain. Returns a sanitized copy.
    pub fn sanitized(&self) -> Self {
        let mut c = *self;
        c.mass_kg = finite_or(c.mass_kg, DEFAULT_MASS_KG).clamp(0.5_f32, 5.0_f32);
        c.spring_k_n_per_m = finite_or(c.spring_k_n_per_m, DEFAULT_SPRING_K_N_PER_M).max(1.0_f32);
        c.rest_length_m =
            finite_or(c.rest_length_m, DEFAULT_REST_LENGTH_M).clamp(0.05_f32, 0.5_f32);
        c.target_hop_m = finite_or(c.target_hop_m, DEFAULT_TARGET_HOP_M).clamp(0.02_f32, 0.04_f32);
        c.gravity_m_s2 = finite_or(c.gravity_m_s2, DEFAULT_GRAVITY_M_S2).clamp(1.0_f32, 20.0_f32);
        c.flight_debounce_s =
            finite_or(c.flight_debounce_s, DEFAULT_FLIGHT_DEBOUNCE_S).clamp(0.010_f32, 0.015_f32);
        c.touchdown_debounce_s = finite_or(c.touchdown_debounce_s, DEFAULT_TOUCHDOWN_DEBOUNCE_S)
            .clamp(0.001_f32, 0.005_f32);
        c.flight_current_threshold_ma = finite_or(
            c.flight_current_threshold_ma,
            DEFAULT_FLIGHT_CURRENT_THRESHOLD_MA,
        )
        .max(1.0_f32);
        c.freefall_fraction =
            finite_or(c.freefall_fraction, DEFAULT_FREEFALL_FRACTION).clamp(0.05_f32, 0.8_f32);
        c.landing_k0_n_per_m =
            finite_or(c.landing_k0_n_per_m, DEFAULT_LANDING_K0_N_PER_M).max(1.0_f32);
        c.landing_k1_n_per_m =
            finite_or(c.landing_k1_n_per_m, DEFAULT_LANDING_K1_N_PER_M).max(c.landing_k0_n_per_m);
        c.landing_ramp_s =
            finite_or(c.landing_ramp_s, DEFAULT_LANDING_RAMP_S).clamp(0.025_f32, 0.040_f32);
        c.landing_damping_n_s_per_m = finite_or(
            c.landing_damping_n_s_per_m,
            DEFAULT_LANDING_DAMPING_N_S_PER_M,
        )
        .clamp(40.0_f32, 60.0_f32);
        c.moment_arm_m = finite_or(c.moment_arm_m, DEFAULT_MOMENT_ARM_M).clamp(0.01_f32, 0.2_f32);
        c.max_force_n = finite_or(c.max_force_n, DEFAULT_MAX_FORCE_N).max(1.0_f32);
        c.cbf_kappa = finite_or(c.cbf_kappa, DEFAULT_CBF_KAPPA).max(0.0_f32);
        c
    }
}

/// Flight/stance state of the SLIP template.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum SlipPhase {
    #[default]
    Stance,
    Flight,
}

/// Debounced flight detector driven by motor current and IMU free-fall.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FlightDetector {
    /// Continuous time the flight candidate condition has held.
    air_candidate_s: f32,
    /// Continuous time the grounded condition has held.
    grounded_s: f32,
    in_flight: bool,
}

impl FlightDetector {
    /// Advances the detector one tick.
    ///
    /// `specific_force_m_s2` is `|a_m|` from the IMU; free fall is confirmed
    /// only when the IMU is available. Stance re-entry uses the short
    /// touchdown debounce; flight entry uses the longer 10–15 ms debounce.
    pub fn update(
        &mut self,
        dt_s: f32,
        leg_current_ma: f32,
        specific_force_m_s2: f32,
        imu_available: bool,
        cfg: &SlipConfig,
    ) -> bool {
        let dt = finite_or(dt_s, 0.005_f32).clamp(0.0_f32, 0.05_f32);
        let low_current = finite_or(leg_current_ma, 1e9_f32) < cfg.flight_current_threshold_ma;
        let free_fall = imu_available
            && finite_or(specific_force_m_s2, 1e9_f32) < cfg.freefall_fraction * cfg.gravity_m_s2;
        let candidate = low_current && free_fall;

        // Two independent debounce timers: the air timer measures how long
        // the flight signature has held, the ground timer measures how long
        // contact has returned. Resetting only the opposing timer lets the
        // short touchdown debounce actually elapse.
        if candidate {
            self.air_candidate_s += dt;
            self.grounded_s = 0.0_f32;
        } else {
            self.grounded_s += dt;
            self.air_candidate_s = 0.0_f32;
        }

        if self.in_flight {
            if self.grounded_s >= cfg.touchdown_debounce_s {
                self.in_flight = false;
                self.air_candidate_s = 0.0_f32;
                self.grounded_s = 0.0_f32;
            }
        } else if self.air_candidate_s >= cfg.flight_debounce_s {
            self.in_flight = true;
            self.air_candidate_s = 0.0_f32;
            self.grounded_s = 0.0_f32;
        }
        self.in_flight
    }

    /// Current debounced decision.
    pub const fn in_flight(&self) -> bool {
        self.in_flight
    }

    /// Resets all debounce state.
    pub fn reset(&mut self) {
        self.air_candidate_s = 0.0_f32;
        self.grounded_s = 0.0_f32;
        self.in_flight = false;
    }
}

/// Per-tick sensor inputs for [`SlipController::step`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SlipInputs {
    /// Fixed control tick in seconds.
    pub dt_s: f32,
    /// Measured virtual leg length `L` in meters.
    pub leg_length_m: f32,
    /// Measured leg-length rate `Ḷ` in m/s (negative during compression).
    pub leg_length_rate_m_s: f32,
    /// Summed absolute leg-joint current in mA.
    pub leg_current_ma: f32,
    /// IMU specific-force magnitude `|a_m|` in m/s².
    pub specific_force_m_s2: f32,
    /// Whether the IMU sample is valid.
    pub imu_available: bool,
}

/// Deterministic per-tick output of the SLIP controller.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SlipOutput {
    pub phase: SlipPhase,
    pub in_flight: bool,
    /// Commanded positive vertical leg force in N (zero during flight).
    pub leg_force_n: f32,
    /// Commanded per-leg joint torque in N·m (positive = push-off/extension).
    pub joint_torque_nm: f32,
    /// Active landing stiffness in N/m (telemetry / test visibility).
    pub landing_stiffness_n_per_m: f32,
    /// Takeoff velocity implied by the target hop in m/s.
    pub takeoff_velocity_m_s: f32,
    /// Positive leg compression `L₀ − L` in meters.
    pub compression_m: f32,
}

/// Stateful, deterministic SLIP running controller.
#[derive(Clone, Debug)]
pub struct SlipController {
    /// Sanitized parameters (kept public for read-only inspection).
    pub config: SlipConfig,
    phase: SlipPhase,
    detector: FlightDetector,
    landing_ramp_s: f32,
}

impl SlipController {
    /// Creates a controller, sanitizing the supplied configuration.
    pub fn new(config: SlipConfig) -> Self {
        let config = config.sanitized();
        Self {
            config,
            phase: SlipPhase::Stance,
            detector: FlightDetector::default(),
            landing_ramp_s: 0.0_f32,
        }
    }

    /// Current phase.
    pub const fn phase(&self) -> SlipPhase {
        self.phase
    }

    /// Debounced flight flag.
    pub const fn in_flight(&self) -> bool {
        self.detector.in_flight()
    }

    /// Resets phase, debounce, and landing ramp (Teach/EStop/ClearFaults).
    pub fn reset(&mut self) {
        self.phase = SlipPhase::Stance;
        self.detector.reset();
        self.landing_ramp_s = 0.0_f32;
    }

    /// Takeoff velocity `v_TO = sqrt(2 g H)` for the target hop (m/s).
    pub fn takeoff_velocity_m_s(&self) -> f32 {
        let g = self.config.gravity_m_s2;
        let h = self.config.target_hop_m.clamp(0.02_f32, 0.04_f32);
        sqrtf((2.0_f32 * g * h).max(0.0_f32))
    }

    /// SLIP natural frequency `ω_s = sqrt(k / m)` in rad/s.
    pub fn natural_frequency_rad_s(&self) -> f32 {
        sqrtf((self.config.spring_k_n_per_m / self.config.mass_kg).max(0.0_f32))
    }

    /// Full SLIP stance half-period `T_s = π / ω_s` in seconds (single leg).
    pub fn stance_duration_s(&self) -> f32 {
        PI / self.natural_frequency_rad_s().max(1e-3_f32)
    }

    /// Landing stiffness after `t_s` seconds of a ramp starting at touchdown.
    pub fn landing_stiffness_at(&self, t_s: f32) -> f32 {
        let alpha = (finite_or(t_s, 0.0_f32).max(0.0_f32)
            / self.config.landing_ramp_s.max(1e-3_f32))
        .min(1.0_f32);
        self.config.landing_k0_n_per_m
            + (self.config.landing_k1_n_per_m - self.config.landing_k0_n_per_m) * alpha
    }

    /// Executes one deterministic control tick.
    pub fn step(&mut self, input: SlipInputs) -> SlipOutput {
        let cfg = self.config;
        let dt = input.dt_s.clamp(0.0005_f32, 0.05_f32);
        let v_to = self.takeoff_velocity_m_s();

        let in_flight = self.detector.update(
            dt,
            input.leg_current_ma,
            input.specific_force_m_s2,
            input.imu_available,
            &cfg,
        );
        if in_flight && self.phase != SlipPhase::Flight {
            self.phase = SlipPhase::Flight;
            self.landing_ramp_s = 0.0_f32;
        } else if !in_flight && self.phase == SlipPhase::Flight {
            // Touchdown: start the compliant landing-impedance ramp.
            self.phase = SlipPhase::Stance;
            self.landing_ramp_s = 0.0_f32;
        }

        if in_flight {
            return SlipOutput {
                phase: SlipPhase::Flight,
                in_flight: true,
                leg_force_n: 0.0_f32,
                joint_torque_nm: 0.0_f32,
                landing_stiffness_n_per_m: cfg.landing_k0_n_per_m,
                takeoff_velocity_m_s: v_to,
                compression_m: 0.0_f32,
            };
        }

        self.landing_ramp_s = (self.landing_ramp_s + dt).min(cfg.landing_ramp_s);
        let compression =
            (cfg.rest_length_m - input.leg_length_m).clamp(0.0_f32, cfg.rest_length_m);
        let damping_force = -cfg.landing_damping_n_s_per_m * input.leg_length_rate_m_s;

        // Landing impedance dominates the first `landing_ramp_s` of stance;
        // afterwards the nominal SLIP thrust spring renders push-off. The
        // force is always non-negative (a leg can push but never pull).
        let landing_force =
            self.landing_stiffness_at(self.landing_ramp_s) * compression + damping_force;
        let thrust_force = cfg.spring_k_n_per_m * compression + damping_force;
        let force = if self.landing_ramp_s < cfg.landing_ramp_s {
            landing_force
        } else {
            thrust_force
        }
        .clamp(0.0_f32, cfg.max_force_n);

        SlipOutput {
            phase: SlipPhase::Stance,
            in_flight: false,
            leg_force_n: force,
            joint_torque_nm: force * cfg.moment_arm_m,
            landing_stiffness_n_per_m: self.landing_stiffness_at(self.landing_ramp_s),
            takeoff_velocity_m_s: v_to,
            compression_m: compression,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn config_is_disabled_by_default_and_sanitizes_bad_values() {
        let cfg = SlipConfig::default();
        assert!(!cfg.enabled);
        assert_eq!(cfg.spring_k_n_per_m, 670.0);
        assert_eq!(cfg.rest_length_m, 0.22);
        let bad = SlipConfig {
            mass_kg: f32::NAN,
            target_hop_m: 9.0,
            landing_ramp_s: -1.0,
            ..Default::default()
        };
        let clean = bad.sanitized();
        assert_eq!(clean.mass_kg, DEFAULT_MASS_KG);
        assert_eq!(clean.target_hop_m, 0.04);
        assert_eq!(clean.landing_ramp_s, 0.025);
    }

    #[test]
    fn hop_height_and_takeoff_velocity_match_the_2_to_4_cm_target() {
        let ctrl = SlipController::new(SlipConfig::default());
        let v = ctrl.takeoff_velocity_m_s();
        assert!(v >= 0.63_f32 && v <= 0.89_f32, "v_TO = {v}");
        let h = v * v / (2.0_f32 * DEFAULT_GRAVITY_M_S2);
        assert!((h - 0.03_f32).abs() < 1e-3_f32, "hop height = {h}");
        let stance = ctrl.stance_duration_s();
        assert!(stance >= 0.12_f32 && stance <= 0.15_f32, "T_s = {stance}");
        assert!(
            stance / 0.005_f32 > 24.0_f32,
            "stance must exceed 24 ticks at 200 Hz"
        );
        // Envelope endpoints respect the spec.
        let mut cfg = SlipConfig::default();
        cfg.target_hop_m = 0.02_f32;
        let v_lo = SlipController::new(cfg).takeoff_velocity_m_s();
        cfg.target_hop_m = 0.04_f32;
        let v_hi = SlipController::new(cfg).takeoff_velocity_m_s();
        assert!((v_lo - 0.6265_f32).abs() < 1e-3_f32, "v_lo = {v_lo}");
        assert!((v_hi - 0.8861_f32).abs() < 1e-3_f32, "v_hi = {v_hi}");
    }

    #[test]
    fn flight_detection_debounces_entry_and_exits_fast_on_touchdown() {
        let cfg = SlipConfig::default();
        let mut detector = FlightDetector::default();
        let dt = 0.005_f32; // 200 Hz
        let air = SlipInputs {
            dt_s: dt,
            leg_current_ma: 50.0,
            specific_force_m_s2: 1.0,
            imu_available: true,
            ..Default::default()
        };
        // 10 ms (2 ticks) is below the 12 ms entry debounce.
        assert!(!detector.update(
            dt,
            air.leg_current_ma,
            air.specific_force_m_s2,
            air.imu_available,
            &cfg
        ));
        assert!(!detector.update(
            dt,
            air.leg_current_ma,
            air.specific_force_m_s2,
            air.imu_available,
            &cfg
        ));
        // Third tick => 15 ms >= 12 ms: flight declared.
        assert!(detector.update(
            dt,
            air.leg_current_ma,
            air.specific_force_m_s2,
            air.imu_available,
            &cfg
        ));

        // One grounded tick (5 ms) exceeds the 4 ms touchdown debounce, so
        // stance re-entry is immediate at 200 Hz (< 5 ms requirement).
        let ground = (300.0_f32, 9.81_f32);
        assert!(!detector.update(dt, ground.0, ground.1, true, &cfg));
        assert!(!detector.in_flight());
    }

    #[test]
    fn imu_outage_or_current_load_suppresses_false_flight() {
        let cfg = SlipConfig::default();
        let mut detector = FlightDetector::default();
        let dt = 0.005_f32;
        // Low current but no IMU -> never flight.
        for _ in 0..20 {
            assert!(!detector.update(dt, 0.0, 0.0, false, &cfg));
        }
        // Free-fall IMU but loaded legs -> never flight.
        for _ in 0..20 {
            assert!(!detector.update(dt, 900.0, 0.0, true, &cfg));
        }
    }

    #[test]
    fn landing_stiffness_ramps_800_to_4000_over_thirty_ms() {
        let ctrl = SlipController::new(SlipConfig::default());
        assert_eq!(ctrl.landing_stiffness_at(0.0), 800.0);
        assert_eq!(ctrl.landing_stiffness_at(0.015), 2400.0);
        assert_eq!(ctrl.landing_stiffness_at(0.030), 4000.0);
        assert_eq!(ctrl.landing_stiffness_at(1.0), 4000.0);
    }

    #[test]
    fn stance_thrust_uses_670_spring_and_landing_uses_the_ramping_spring() {
        let mut ctrl = SlipController::new(SlipConfig::default());
        let l = 0.19_f32; // 3 cm compression
        let input = SlipInputs {
            dt_s: 0.005,
            leg_length_m: l,
            leg_length_rate_m_s: 0.0,
            leg_current_ma: 400.0,
            specific_force_m_s2: 9.81,
            imu_available: true,
        };
        let out = ctrl.step(input);
        assert_eq!(out.phase, SlipPhase::Stance);
        // 5 ms into a 30 ms ramp: K = 800 + 3200*(5/30) = 1333.33 N/m.
        assert!((out.landing_stiffness_n_per_m - 1333.3334_f32).abs() < 0.01);
        assert!(
            (out.leg_force_n - 40.0_f32).abs() < 0.01,
            "F = {}",
            out.leg_force_n
        );
        assert!((out.joint_torque_nm - 2.0_f32).abs() < 0.01);

        // After the ramp the nominal thrust spring governs (670 * 0.03 = 20.1 N).
        for _ in 0..10 {
            let _ = ctrl.step(input);
        }
        let settled = ctrl.step(input);
        assert!(
            (settled.leg_force_n - 20.1_f32).abs() < 0.01,
            "F_to = {}",
            settled.leg_force_n
        );
        assert!(settled.joint_torque_nm <= 1.8_f32 * 2.0_f32);
    }

    #[test]
    fn flight_zeroes_force_and_touchdown_restarts_the_ramp() {
        let mut ctrl = SlipController::new(SlipConfig::default());
        let grounded = SlipInputs {
            dt_s: 0.005,
            leg_length_m: 0.19,
            leg_length_rate_m_s: 0.0,
            leg_current_ma: 400.0,
            specific_force_m_s2: 9.81,
            imu_available: true,
        };
        let _ = ctrl.step(grounded);
        let air = SlipInputs {
            leg_current_ma: 10.0,
            specific_force_m_s2: 0.5,
            ..grounded
        };
        let mut air_out = ctrl.step(air);
        for _ in 0..2 {
            air_out = ctrl.step(air);
        }
        assert_eq!(air_out.phase, SlipPhase::Flight);
        assert_eq!(air_out.leg_force_n, 0.0);
        assert_eq!(air_out.joint_torque_nm, 0.0);

        let land_out = ctrl.step(grounded);
        assert_eq!(land_out.phase, SlipPhase::Stance);
        assert_eq!(
            land_out.landing_stiffness_n_per_m,
            800.0 + 3200.0 * 0.005 / 0.030
        );
    }

    #[test]
    fn rest_length_derives_from_leg_geometry() {
        let cfg = SlipConfig::from_nominal_height(0.62_f32);
        assert!((cfg.rest_length_m - 0.2201_f32).abs() < 1e-4_f32);
    }
}
