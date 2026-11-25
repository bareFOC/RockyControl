//! Bus-health bookkeeping for the shared multibus servo transport.
//!
//! Ported from `firmware/rk-fw-core/src/i3c_bus.rs` with the bespoke 16B/12B
//! codec (packing, CRC8, unpacking) removed. Timeout isolation semantics are
//! unchanged: a failed joint is counted, becomes degraded after three
//! consecutive misses, and never stalls transactions to other joints.

use rk_control::RawServoFeedback;
use rk_proto::MAX_JOINTS;

use crate::joint_map::{amps_to_milliamps, JointFeedback};

/// Consecutive missed transactions before a joint is flagged degraded.
pub const MAX_CONSECUTIVE_TIMEOUTS: u8 = 3;

/// Timeout/degraded tracker shared by firmware and simulator.
#[derive(Clone, Debug)]
pub struct MultibusBusTracker {
    joint_count: usize,
    consecutive_timeouts: [u8; MAX_JOINTS],
    degraded_mask: u32,
    total_timeout_count: u16,
    tx_seq: u8,
    feedback: RawServoFeedback,
}

impl MultibusBusTracker {
    pub const fn new(joint_count: usize) -> Self {
        Self {
            joint_count,
            consecutive_timeouts: [0u8; MAX_JOINTS],
            degraded_mask: 0,
            total_timeout_count: 0,
            tx_seq: 0,
            feedback: RawServoFeedback {
                raw_angle_rad: [0.0; MAX_JOINTS],
                velocity_rad_s: [0.0; MAX_JOINTS],
                current_ma: [0; MAX_JOINTS],
                temp_c: [28; MAX_JOINTS],
                bus_voltage_mv: 12_200,
                bus_error: false,
                imu: rk_proto::ImuState {
                    available: false,
                    pitch_rad: 0.0,
                    roll_rad: 0.0,
                    yaw_rad: 0.0,
                    pitch_rate_rad_s: 0.0,
                    roll_rate_rad_s: 0.0,
                    yaw_rate_rad_s: 0.0,
                    gravity: [0.0, -9.81, 0.0],
                },
                i3c_timeout_count: 0,
                imu_source_mask: 0,
            },
        }
    }

    pub fn next_seq(&mut self) -> u8 {
        self.tx_seq = self.tx_seq.wrapping_add(1);
        self.tx_seq
    }

    /// Record a successful feedback burst for `joint`, clearing its degraded
    /// state while preserving the cumulative timeout count.
    pub fn record_reply(&mut self, joint: usize, reply: JointFeedback) {
        if !self.is_tracked(joint) {
            return;
        }
        self.consecutive_timeouts[joint] = 0;
        self.degraded_mask &= !(1u32 << joint);
        self.feedback.raw_angle_rad[joint] = reply.position_rad;
        self.feedback.velocity_rad_s[joint] = reply.velocity_rad_s;
        self.feedback.current_ma[joint] = amps_to_milliamps(reply.current_amps);
        self.feedback.temp_c[joint] = celsius_to_u8(reply.temperature_c);
        self.feedback.bus_error = self.degraded_mask != 0;
    }

    /// Record one failed register transaction for `joint` without affecting
    /// other joints or halting the control loop.
    pub fn record_timeout(&mut self, joint: usize) {
        if !self.is_tracked(joint) {
            return;
        }
        self.total_timeout_count = self.total_timeout_count.saturating_add(1);
        self.consecutive_timeouts[joint] = self.consecutive_timeouts[joint].saturating_add(1);
        if self.consecutive_timeouts[joint] >= MAX_CONSECUTIVE_TIMEOUTS {
            self.degraded_mask |= 1u32 << joint;
        }
        self.feedback.i3c_timeout_count = self.total_timeout_count;
        self.feedback.bus_error = self.degraded_mask != 0;
    }

    /// Convenience adapter: record a failed transaction, if any.
    pub fn record_failure<E>(&mut self, joint: usize, result: &Result<(), E>) {
        if result.is_err() {
            self.record_timeout(joint);
        }
    }

    pub const fn degraded_mask(&self) -> u32 {
        self.degraded_mask
    }

    pub const fn total_timeout_count(&self) -> u16 {
        self.total_timeout_count
    }

    pub const fn consecutive_timeouts(&self, joint: usize) -> u8 {
        if joint < MAX_JOINTS {
            self.consecutive_timeouts[joint]
        } else {
            0
        }
    }

    pub fn is_degraded(&self, joint: usize) -> bool {
        joint < MAX_JOINTS && (self.degraded_mask & (1u32 << joint)) != 0
    }

    pub const fn snapshot(&self) -> RawServoFeedback {
        self.feedback
    }

    const fn is_tracked(&self, joint: usize) -> bool {
        joint < self.joint_count && joint < MAX_JOINTS
    }
}

/// Round to the nearest whole degree and clamp into the `u8` feedback unit.
/// (no_std `f32` has no `round` method, so add 0.5 before the cast.)
fn celsius_to_u8(celsius: f32) -> u8 {
    if !celsius.is_finite() {
        return 0;
    }
    ((celsius + 0.5_f32) as i32).clamp(0, u8::MAX as i32) as u8
}

#[cfg(test)]
mod tests {
    use super::*;

    fn reply() -> JointFeedback {
        JointFeedback {
            position_rad: 0.5,
            velocity_rad_s: 0.1,
            current_amps: 0.3,
            temperature_c: 33.0,
        }
    }

    #[test]
    fn faulty_joint_is_isolated_from_healthy_joints() {
        let mut tracker = MultibusBusTracker::new(21);
        tracker.record_reply(0, reply());

        tracker.record_timeout(7);
        tracker.record_timeout(7);
        assert_eq!(tracker.total_timeout_count(), 2);
        assert_eq!(tracker.degraded_mask(), 0);
        assert_eq!(tracker.consecutive_timeouts(7), 2);
        assert!(!tracker.snapshot().bus_error);

        tracker.record_timeout(7);
        assert_eq!(tracker.degraded_mask(), 1u32 << 7);
        assert!(tracker.is_degraded(7));
        assert!(!tracker.is_degraded(0));
        assert!(tracker.snapshot().bus_error);
        assert!((tracker.snapshot().raw_angle_rad[0] - 0.5).abs() < 1e-6);

        tracker.record_reply(7, reply());
        assert_eq!(tracker.degraded_mask(), 0);
        assert!(!tracker.snapshot().bus_error);
        assert_eq!(tracker.total_timeout_count(), 3);
        assert_eq!(tracker.consecutive_timeouts(7), 0);
        assert_eq!(tracker.snapshot().current_ma[7], 300);
    }

    #[test]
    fn untracked_joints_are_ignored() {
        let mut tracker = MultibusBusTracker::new(3);
        tracker.record_timeout(5);
        tracker.record_reply(5, reply());
        assert_eq!(tracker.total_timeout_count(), 0);
        assert_eq!(tracker.snapshot().raw_angle_rad[5], 0.0);
    }

    #[test]
    fn sequence_wraps_and_timeout_count_saturates() {
        let mut tracker = MultibusBusTracker::new(1);
        tracker.tx_seq = u8::MAX;
        assert_eq!(tracker.next_seq(), 0);
        tracker.total_timeout_count = u16::MAX;
        tracker.record_timeout(0);
        assert_eq!(tracker.total_timeout_count(), u16::MAX);
    }
}
