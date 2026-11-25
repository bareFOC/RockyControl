//! Deterministic tick-log recording and replay verification for the shared control
//! application.
//!
//! Replay re-executes [`ControlTask`] — the exact wrapper firmware uses — so a
//! firmware-only wrapper change cannot silently diverge from a recorded simulation.
//! The tick record is [`ControlTick`], the same value returned by
//! [`ControlTask::step_recorded`], and is serde-serializable for `.copper` logs and
//! host/MCP use.

use rk_control::JointCalibration;
use rk_proto::MAX_JOINTS;
use serde::{Deserialize, Serialize};

use crate::copper::{ControlTask, ControlTick};

/// One deterministic control tick snapshot stored in a replay log.
///
/// This is the shared [`ControlTick`] produced by [`ControlTask::step_recorded`]; the
/// alias keeps the historic replay-side name without introducing a second record type.
pub type CopperListTick = ControlTick;

// Defaults mirrored from `rk-sim::SimFaultInjection`. They live here because
// `rk-fw-core` cannot depend on the simulator; the simulator's conversion helpers
// are the single source of truth for mapping between the two representations.
const DEFAULT_SIM_SEED: u64 = 0x9E37_79B9_7F4A_7C15;
const DEFAULT_SIM_SERVO_LAG_FACTOR: f32 = 0.20;
const DEFAULT_SIM_FOOT_FRICTION_MU: f32 = 0.7;

const fn default_sim_seed() -> u64 {
    DEFAULT_SIM_SEED
}

const fn default_sim_servo_lag_factor() -> f32 {
    DEFAULT_SIM_SERVO_LAG_FACTOR
}

const fn default_sim_foot_friction_mu() -> f32 {
    DEFAULT_SIM_FOOT_FRICTION_MU
}

/// Portable, serde-friendly snapshot of the simulator's fault/noise configuration.
///
/// `rk-fw-core` cannot depend on `rk-sim`, so experiment scripts store the exact
/// `SimFaultInjection` payload in this wire-neutral form. `rk-sim` converts to and
/// from its native fault type with [`crate::replay`]-adjacent helpers defined in that
/// crate; every field defaults so older or hand-written scripts stay parseable.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct SimFaultSnapshot {
    #[serde(default)]
    pub drop_heartbeat: bool,
    #[serde(default)]
    pub stalled_joints: [bool; MAX_JOINTS],
    /// Per-joint servo-bus degradation: that node's register transactions fail
    /// while every other node keeps replying.
    #[serde(default)]
    pub degraded_joints: [bool; MAX_JOINTS],
    #[serde(default)]
    pub voltage_sag: bool,
    #[serde(default)]
    pub servo_bus_error: bool,
    #[serde(default)]
    pub external_hand_wave: bool,
    #[serde(default)]
    pub imu_disconnected: bool,
    #[serde(default = "default_sim_servo_lag_factor")]
    pub servo_lag_factor: f32,
    #[serde(default = "default_sim_seed")]
    pub seed: u64,
    #[serde(default)]
    pub imu_gyro_noise_rad_s: f32,
    #[serde(default)]
    pub imu_accel_noise_m_s2: f32,
    #[serde(default)]
    pub imu_angle_bias_rad: f32,
    #[serde(default)]
    pub imu_gyro_bias_rad_s: f32,
    #[serde(default)]
    pub imu_accel_bias_m_s2: f32,
    #[serde(default)]
    pub imu_latency_ticks: u16,
    #[serde(default)]
    pub current_noise_pct: f32,
    #[serde(default)]
    pub temperature_offset_c: f32,
    #[serde(default)]
    pub encoder_quantization_lsb_rad: f32,
    #[serde(default = "default_sim_foot_friction_mu")]
    pub foot_friction_mu: f32,
    #[serde(default)]
    pub contact_disagreement: bool,
    #[serde(default)]
    pub rx_crc_error: bool,
    #[serde(default)]
    pub rx_version_mismatch: bool,
    #[serde(default)]
    pub rx_truncated_frame: bool,
    #[serde(default)]
    pub rx_fragmented_frame: bool,
}

impl Default for SimFaultSnapshot {
    fn default() -> Self {
        Self {
            drop_heartbeat: false,
            stalled_joints: [false; MAX_JOINTS],
            degraded_joints: [false; MAX_JOINTS],
            voltage_sag: false,
            servo_bus_error: false,
            external_hand_wave: false,
            imu_disconnected: false,
            servo_lag_factor: DEFAULT_SIM_SERVO_LAG_FACTOR,
            seed: DEFAULT_SIM_SEED,
            imu_gyro_noise_rad_s: 0.0,
            imu_accel_noise_m_s2: 0.0,
            imu_angle_bias_rad: 0.0,
            imu_gyro_bias_rad_s: 0.0,
            imu_accel_bias_m_s2: 0.0,
            imu_latency_ticks: 0,
            current_noise_pct: 0.0,
            temperature_offset_c: 0.0,
            encoder_quantization_lsb_rad: 0.0,
            foot_friction_mu: DEFAULT_SIM_FOOT_FRICTION_MU,
            contact_disagreement: false,
            rx_crc_error: false,
            rx_version_mismatch: false,
            rx_truncated_frame: false,
            rx_fragmented_frame: false,
        }
    }
}

/// A non-kernel plant event scheduled for a specific control tick.
///
/// Events are applied *before* the tick at `tick_index`, so a shove at index 40
/// affects the feedback sampled by tick 40. Host commands need no side-channel:
/// they are already stored in each [`ControlTick::rx_cmd`].
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct TimedSimEvent {
    pub tick_index: u64,
    pub kind: SimEventKind,
}

/// The plant mutations that cannot be recovered from a `.copper` tick stream.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub enum SimEventKind {
    /// Replaces the entire simulator fault/noise configuration.
    SetFaults(SimFaultSnapshot),
    /// Angular velocity impulse applied to the simulated torso.
    Shove { pitch_rad_s: f32, roll_rad_s: f32 },
    /// Get-up recovery progress; `None` clears the active recovery sequence.
    Recovery(Option<f32>),
    /// Rebuilds the torso upright with zero angular velocity.
    ResetUpright,
    /// Direct virtual-hand joint angle (used by the action studio).
    VirtualHand { joint: usize, rad: f32 },
}

/// Deterministic `.copper` replay log capturing every wire tick for offline verification.
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct CopperReplayLog {
    pub version: u32,
    pub joint_count: usize,
    pub initial_raw_pose: [f32; MAX_JOINTS],
    pub initial_calibration: Vec<JointCalibration>,
    pub ticks: Vec<CopperListTick>,
    /// Seed of the simulator's deterministic sensor-noise PRNG for this run.
    /// Defaults to zero for legacy v1 logs captured before seeded noise existed.
    #[serde(default)]
    pub seed: u64,
    /// Non-kernel plant events, stamped with the tick they affect. Defaults to
    /// empty for legacy logs; verification never reads this side-channel.
    #[serde(default)]
    pub events: Vec<TimedSimEvent>,
}

/// Result of replaying a [`CopperReplayLog`] through a fresh [`ControlTask`].
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub struct ReplayVerification {
    pub total_ticks: usize,
    pub matched_ticks: usize,
    pub exact_match: bool,
}

/// Replays a recorded `.copper` log through a fresh [`ControlTask`] and asserts
/// bit-identical telemetry and servo commands across every tick.
///
/// Wire-error counts are taken from each recorded tick and fed back through the
/// same explicit parameter the firmware and simulation use, so replay covers the
/// full shared wrapper behavior rather than a simulation-only copy.
pub fn verify_deterministic_replay(log: &CopperReplayLog) -> ReplayVerification {
    let mut replay_task = ControlTask::new(log.joint_count, &log.initial_calibration);
    let mut matched = 0usize;

    for tick in &log.ticks {
        let (out, recorded) = replay_task.step_recorded(
            tick.now_ms,
            tick.dt_sec,
            tick.rx_cmd.as_ref(),
            &tick.rx_state,
            tick.wire_errors,
        );
        if out.telemetry == tick.tx_telem
            && out.servo_cmd == tick.tx_cmd
            && recorded == *tick
        {
            matched += 1;
        }
    }

    ReplayVerification {
        total_ticks: log.ticks.len(),
        matched_ticks: matched,
        exact_match: !log.ticks.is_empty() && matched == log.ticks.len(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::CONTROL_DT_SEC;
    use rk_control::RawServoFeedback;
    use rk_proto::{ControlMode, HostToMcu};

    fn sample_log() -> CopperReplayLog {
        let joint_count = 6;
        let cal = vec![JointCalibration::default(); joint_count];
        let mut task = ControlTask::new(joint_count, &cal);
        let mut ticks = Vec::new();

        for i in 0..4u32 {
            let cmd = HostToMcu {
                seq: i,
                mode: ControlMode::Drive,
                ..Default::default()
            };
            let fb = RawServoFeedback::default();
            let (_, tick) = task.step_recorded(
                i * 10,
                CONTROL_DT_SEC,
                Some(&cmd),
                &fb,
                (i as u16, (i * 2) as u16),
            );
            ticks.push(tick);
        }

        CopperReplayLog {
            version: 1,
            joint_count,
            initial_raw_pose: [0.0; MAX_JOINTS],
            initial_calibration: cal,
            ticks,
            seed: DEFAULT_SIM_SEED,
            events: Vec::new(),
        }
    }

    #[test]
    fn replay_verifies_shared_control_task_exactly() {
        let log = sample_log();
        let check = verify_deterministic_replay(&log);
        assert!(check.exact_match);
        assert_eq!(check.total_ticks, 4);
        assert_eq!(check.matched_ticks, 4);
    }

    #[test]
    fn replay_is_bit_exact_with_nonzero_joint_velocities() {
        // Exercise the P1 mass-weighted FK path: 21 joints with nonzero rates.
        let joint_count = 21usize;
        let cal = vec![JointCalibration::default(); joint_count];
        let mut task = ControlTask::new(joint_count, &cal);
        let mut ticks = Vec::new();
        for i in 0..5u32 {
            let cmd = HostToMcu {
                seq: i,
                mode: ControlMode::Drive,
                ..Default::default()
            };
            let mut fb = RawServoFeedback::default();
            for j in 0..joint_count {
                fb.raw_angle_rad[j] = 0.02 * (j as f32) + 0.01 * (i as f32);
                fb.velocity_rad_s[j] = 0.15 * ((j % 3) as f32) - 0.1;
            }
            let (_, tick) = task.step_recorded(
                i * 10,
                CONTROL_DT_SEC,
                Some(&cmd),
                &fb,
                (i as u16, 0),
            );
            ticks.push(tick);
        }
        let log = CopperReplayLog {
            version: 1,
            joint_count,
            initial_raw_pose: [0.0; MAX_JOINTS],
            initial_calibration: cal,
            ticks,
            seed: 0,
            events: vec![TimedSimEvent {
                tick_index: 2,
                kind: SimEventKind::Shove {
                    pitch_rad_s: 1.25,
                    roll_rad_s: -0.5,
                },
            }],
        };
        // The side-channel is inert during verification: replay covers only the
        // shared ControlTask and must stay bit-exact even with events present.
        let check = verify_deterministic_replay(&log);
        assert!(check.exact_match, "FK-based estimator telemetry must replay bit-exactly");
        assert_eq!(check.matched_ticks, 5);
    }

    #[test]
    fn replay_detects_behavioral_difference_in_recorded_output() {
        let mut log = sample_log();
        // Tamper with a recorded output: replay through the real ControlTask must
        // detect the difference instead of reporting a false exact match.
        log.ticks[2].tx_telem.seq_ack = 0xDEAD;

        let check = verify_deterministic_replay(&log);
        assert!(!check.exact_match);
        assert_eq!(check.total_ticks, 4);
        assert_eq!(check.matched_ticks, 3);
    }

    #[test]
    fn replay_detects_wire_error_input_change() {
        let mut log = sample_log();
        // Wire errors are explicit replay inputs; changing them must break parity
        // because telemetry carries the counts and the tick snapshot records them.
        log.ticks[1].wire_errors = (99, 99);

        let check = verify_deterministic_replay(&log);
        assert!(!check.exact_match);
        assert!(check.matched_ticks < check.total_ticks);
    }

    /// Evidence for the "firmware-only wrapper bug is caught" requirement: a log whose
    /// recorded outputs come from a behaviorally different wrapper is rejected even
    /// though every recorded input is identical to what the real `ControlTask` sees.
    #[test]
    fn firmware_wrapper_behavior_change_is_detected_by_replay() {
        let mut log = sample_log();
        // Simulate a changed wrapper: every tick's telemetry uptime is shifted as if
        // the firmware wrapper's timing computation had been modified.
        for tick in &mut log.ticks {
            tick.tx_telem.uptime_ms += 1;
        }

        let check = verify_deterministic_replay(&log);
        assert!(!check.exact_match);
        assert_eq!(check.matched_ticks, 0);
        assert_eq!(check.total_ticks, log.ticks.len());
    }

    #[test]
    fn replay_types_are_serde_serializable() {
        // Compile-time proof that all replay-facing types implement serde.
        fn assert_serde<T: Serialize + for<'de> Deserialize<'de>>() {}
        assert_serde::<CopperListTick>();
        assert_serde::<CopperReplayLog>();
        assert_serde::<ReplayVerification>();
        assert_serde::<SimFaultSnapshot>();
        assert_serde::<TimedSimEvent>();
        assert_serde::<SimEventKind>();
    }

    #[test]
    fn sim_fault_snapshot_defaults_match_simulator_defaults() {
        let snapshot = SimFaultSnapshot::default();
        assert_eq!(snapshot.seed, DEFAULT_SIM_SEED);
        assert_eq!(snapshot.servo_lag_factor, 0.20);
        assert_eq!(snapshot.foot_friction_mu, 0.7);
        assert!(snapshot.stalled_joints.iter().all(|stalled| !stalled));
        assert!(snapshot.degraded_joints.iter().all(|degraded| !degraded));
    }
}
