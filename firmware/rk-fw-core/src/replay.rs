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

/// Deterministic `.copper` replay log capturing every wire tick for offline verification.
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct CopperReplayLog {
    pub version: u32,
    pub joint_count: usize,
    pub initial_raw_pose: [f32; MAX_JOINTS],
    pub initial_calibration: Vec<JointCalibration>,
    pub ticks: Vec<CopperListTick>,
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
    }
}
