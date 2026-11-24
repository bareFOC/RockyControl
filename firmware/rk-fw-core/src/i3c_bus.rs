//! Pure `#![no_std]` Timeout-Bounded SimpleFOC I3C Servo Bus Wire Framing & Health Tracker.
//!
//! Used by `firmware/rk-rp2350/src/servo_i3c.rs` (PIO I3C master on RP2350) and testable
//! on host without hardware. Isolates per-joint bus timeouts so one noisy or disconnected
//! actuator increments `total_timeout_count` and degrades cleanly without blocking or
//! starving the deterministic 200 Hz `copper-rs` loop.

use rk_control::{RawServoCommand, RawServoFeedback};
use rk_proto::{JointCommandMode, MAX_JOINTS};

/// Fixed 16-byte SimpleFOC I3C command payload per joint.
pub const I3C_CMD_FRAME_BYTES: usize = 16;
/// Fixed 12-byte SimpleFOC I3C feedback payload per joint.
pub const I3C_FB_FRAME_BYTES: usize = 12;
/// Base I3C dynamic address assigned to Joint 0 (`0x20 ..= 0x37` for 24 joints).
pub const I3C_JOINT_BASE_ADDR: u8 = 0x20;
/// Consecutive missed replies before flagging a joint degraded & raising `bus_error`.
pub const MAX_CONSECUTIVE_TIMEOUTS: u8 = 3;

/// Per-joint SimpleFOC I3C command frame (`16 bytes` deterministic wire layout).
///
/// Layout:
/// - `[0]`: `flags` (`bit 0 = torque_on`, `bits 1..=2 = JointCommandMode`)
/// - `[1..5]`: `target_rad` (`f32` LE)
/// - `[5..9]`: `velocity_rad_s` (`f32` LE)
/// - `[9..13]`: `torque_nm` (`f32` LE — saturation ceiling in Pos/Vel modes, torque cmd in Torque mode)
/// - `[13]`: `joint_id` (`u8`)
/// - `[14]`: `seq` (`u8`)
/// - `[15]`: `crc8` (XOR-rotate integrity byte over `[0..15]`)
pub fn pack_i3c_joint_cmd(
    joint_id: u8,
    seq: u8,
    torque_on: bool,
    mode: JointCommandMode,
    target_rad: f32,
    velocity_rad_s: f32,
    torque_nm: f32,
) -> [u8; I3C_CMD_FRAME_BYTES] {
    let mut out = [0u8; I3C_CMD_FRAME_BYTES];
    out[0] = (torque_on as u8) | ((mode.to_wire() & 0x03) << 1);
    out[1..5].copy_from_slice(&target_rad.to_le_bytes());
    out[5..9].copy_from_slice(&velocity_rad_s.to_le_bytes());
    out[9..13].copy_from_slice(&torque_nm.to_le_bytes());
    out[13] = joint_id;
    out[14] = seq;
    out[15] = simple_crc8(&out[..15]);
    out
}

/// Decoded per-joint SimpleFOC I3C feedback payload.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct I3cJointReply {
    pub angle_rad: f32,
    pub velocity_rad_s: f32,
    pub current_ma: u16,
    pub temp_c: u8,
}

/// Packs a SimpleFOC I3C feedback reply (`12 bytes`) — used by unit tests & bench responder.
pub fn pack_i3c_joint_reply(reply: &I3cJointReply) -> [u8; I3C_FB_FRAME_BYTES] {
    let mut out = [0u8; I3C_FB_FRAME_BYTES];
    out[0..4].copy_from_slice(&reply.angle_rad.to_le_bytes());
    out[4..8].copy_from_slice(&reply.velocity_rad_s.to_le_bytes());
    out[8..10].copy_from_slice(&reply.current_ma.to_le_bytes());
    out[10] = reply.temp_c;
    out[11] = simple_crc8(&out[..11]);
    out
}

/// Validates CRC8 and finite floats on a 12-byte SimpleFOC I3C joint reply.
pub fn unpack_i3c_joint_reply(buf: &[u8; I3C_FB_FRAME_BYTES]) -> Option<I3cJointReply> {
    if simple_crc8(&buf[..11]) != buf[11] {
        return None;
    }
    let angle_rad = f32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
    let velocity_rad_s = f32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);
    if !angle_rad.is_finite() || !velocity_rad_s.is_finite() {
        return None;
    }
    let current_ma = u16::from_le_bytes([buf[8], buf[9]]);
    let temp_c = buf[10];
    Some(I3cJointReply {
        angle_rad,
        velocity_rad_s,
        current_ma,
        temp_c,
    })
}

/// Deterministic 8-bit CRC (poly `0x07`) over short I3C servo payloads.
#[inline]
pub fn simple_crc8(bytes: &[u8]) -> u8 {
    let mut crc = 0u8;
    for &b in bytes {
        crc ^= b;
        for _ in 0..8 {
            if (crc & 0x80) != 0 {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

/// Timeout-bounded I3C bus health tracker isolating faulty joints from the 200 Hz control loop.
#[derive(Clone, Debug)]
pub struct I3cBusTracker {
    joint_count: usize,
    consecutive_timeouts: [u8; MAX_JOINTS],
    degraded_mask: u32,
    total_timeout_count: u16,
    tx_seq: u8,
    feedback: RawServoFeedback,
}

impl I3cBusTracker {
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

    /// Builds the 16-byte wire frame for `joint` from `RawServoCommand`.
    pub fn encode_joint_cmd(&self, joint: usize, seq: u8, cmd: &RawServoCommand) -> [u8; I3C_CMD_FRAME_BYTES] {
        let idx = joint.min(MAX_JOINTS - 1);
        let torque_on = ((cmd.torque_mask >> idx) & 1) != 0;
        let mode = JointCommandMode::from_wire(cmd.command_mode[idx]);
        pack_i3c_joint_cmd(
            idx as u8,
            seq,
            torque_on,
            mode,
            cmd.raw_target_rad[idx],
            cmd.raw_velocity_rad_s[idx],
            cmd.torque_nm[idx],
        )
    }

    /// Records a valid reply for `joint`, clearing its consecutive timeout counter.
    pub fn record_reply(&mut self, joint: usize, reply: I3cJointReply) {
        if joint >= self.joint_count.min(MAX_JOINTS) {
            return;
        }
        self.consecutive_timeouts[joint] = 0;
        self.degraded_mask &= !(1u32 << joint);
        self.feedback.raw_angle_rad[joint] = reply.angle_rad;
        self.feedback.velocity_rad_s[joint] = reply.velocity_rad_s;
        self.feedback.current_ma[joint] = reply.current_ma;
        self.feedback.temp_c[joint] = reply.temp_c;
        self.feedback.bus_error = self.degraded_mask != 0;
    }

    /// Records a single I3C transaction timeout / CRC error for `joint` without halting other joints.
    pub fn record_timeout(&mut self, joint: usize) {
        if joint >= self.joint_count.min(MAX_JOINTS) {
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

    pub const fn degraded_mask(&self) -> u32 {
        self.degraded_mask
    }

    pub const fn total_timeout_count(&self) -> u16 {
        self.total_timeout_count
    }

    pub const fn snapshot(&self) -> RawServoFeedback {
        self.feedback
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn i3c_cmd_and_reply_wire_packing_round_trip() {
        let mut cmd = RawServoCommand::default();
        cmd.torque_mask = 1 << 4;
        cmd.command_mode[4] = JointCommandMode::Velocity.to_wire();
        cmd.raw_target_rad[4] = 0.75;
        cmd.raw_velocity_rad_s[4] = 2.5;
        cmd.torque_nm[4] = 0.9;

        let tracker = I3cBusTracker::new(21);
        let frame = tracker.encode_joint_cmd(4, 11, &cmd);
        assert_eq!(frame[0] & 0x01, 1, "torque bit set");
        assert_eq!((frame[0] >> 1) & 0x03, JointCommandMode::Velocity.to_wire());
        assert_eq!(frame[13], 4);
        assert_eq!(frame[14], 11);
        assert_eq!(frame[15], simple_crc8(&frame[..15]));

        let reply = I3cJointReply {
            angle_rad: -0.42,
            velocity_rad_s: 1.35,
            current_ma: 620,
            temp_c: 39,
        };
        let mut wire = pack_i3c_joint_reply(&reply);
        assert_eq!(unpack_i3c_joint_reply(&wire), Some(reply));

        // Corrupt a single bit -> rejected
        wire[2] ^= 0x10;
        assert_eq!(unpack_i3c_joint_reply(&wire), None);
    }

    #[test]
    fn i3c_tracker_isolates_faulty_joint_without_breaking_healthy_joints() {
        let mut tracker = I3cBusTracker::new(21);
        let healthy = I3cJointReply {
            angle_rad: 0.5,
            velocity_rad_s: 0.1,
            current_ma: 300,
            temp_c: 33,
        };
        tracker.record_reply(0, healthy);

        // Two transient timeouts on joint 7 -> counted, not yet degraded
        tracker.record_timeout(7);
        tracker.record_timeout(7);
        assert_eq!(tracker.total_timeout_count(), 2);
        assert_eq!(tracker.degraded_mask(), 0);
        assert!(!tracker.snapshot().bus_error);

        // Third consecutive timeout degrades joint 7 only; joint 0 stays intact
        tracker.record_timeout(7);
        assert_eq!(tracker.degraded_mask(), 1u32 << 7);
        assert!(tracker.snapshot().bus_error);
        assert!((tracker.snapshot().raw_angle_rad[0] - 0.5).abs() < 1e-6);

        // Recovery clears degraded flag while preserving cumulative timeout count
        tracker.record_reply(7, healthy);
        assert_eq!(tracker.degraded_mask(), 0);
        assert!(!tracker.snapshot().bus_error);
        assert_eq!(tracker.total_timeout_count(), 3);
    }
}
