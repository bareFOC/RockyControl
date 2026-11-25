//! Canonical BareFOC control-table register map and protocol constants for the
//! host-side master SDK.
//!
//! Every offset mirrors the target firmware's 512-byte control table defined in
//! `bareFOC/src/control_table.rs`. Register addresses are `u16` because several
//! RAM registers exceed `0xFF`. Values on the wire are **little-endian**; the
//! I2C register pointer itself is **big-endian** (two bytes). Instruction codes
//! mirror `barefoc-comms/barefoc-multibus/src/servo_protocol.rs`.
//!
//! The `extended-telemetry-controls` change expands the table to 512 bytes.
//! All legacy offsets are unchanged; new telemetry occupies the `0x180..0x1FF`
//! tail, including the capability mask at `0x1F8`; `0x1FC..0x1FF` is reserved.

/// Total byte capacity of the standardized Control Table.
///
/// Upstream: `bareFOC/src/control_table.rs` (`CONTROL_TABLE_SIZE`).
pub const CONTROL_TABLE_SIZE: usize = 512;

/// Model number register (`u16`, read-only).
///
/// Upstream: `bareFOC/src/control_table.rs` (`REG_MODEL_NUMBER`, `0x00`).
pub const REG_MODEL_NUMBER: u16 = 0x00;
/// Firmware version register (`u8`, read-only).
///
/// Upstream: `bareFOC/src/control_table.rs` (`REG_FIRMWARE_VERSION`, `0x02`).
pub const REG_FIRMWARE_VERSION: u16 = 0x02;
/// 96-bit silicon UID register (12 bytes, read-only).
///
/// Upstream: `bareFOC/src/control_table.rs` (`REG_HARDWARE_UID`, `0x04`).
pub const REG_HARDWARE_UID: u16 = 0x04;
/// Operating mode register (`u8`).
///
/// Upstream: `bareFOC/src/control_table.rs` (`REG_OPERATING_MODE`, `0xE0`).
pub const REG_OPERATING_MODE: u16 = 0xE0;
/// Multi-motor kinematics topology preset register (`u8`).
///
/// Upstream: `bareFOC/src/control_table.rs` (`REG_TOPOLOGY_PRESET`, `0xE2`).
pub const REG_TOPOLOGY_PRESET: u16 = 0xE2;
/// Motor bridge drive enable register (`u8`; `0` = coast/disarmed).
///
/// Upstream: `bareFOC/src/control_table.rs` (`REG_TORQUE_ENABLE`, `0x120`).
pub const REG_TORQUE_ENABLE: u16 = 0x120;
/// Flash persistence trigger register (`u8`).
///
/// Upstream: `bareFOC/src/control_table.rs` (`REG_SAVE_CONFIG_CMD`, `0x124`).
pub const REG_SAVE_CONFIG_CMD: u16 = 0x124;
/// Goal position register (`f32`, radians).
///
/// Upstream: `bareFOC/src/control_table.rs` (`REG_GOAL_POSITION`, `0x130`).
pub const REG_GOAL_POSITION: u16 = 0x130;
/// Goal velocity register (`f32`, rad/s).
///
/// Upstream: `bareFOC/src/control_table.rs` (`REG_GOAL_VELOCITY`, `0x134`).
pub const REG_GOAL_VELOCITY: u16 = 0x134;
/// Goal current / torque register (`f32`, amperes).
///
/// Upstream: `bareFOC/src/control_table.rs` (`REG_GOAL_CURRENT`, `0x138`).
pub const REG_GOAL_CURRENT: u16 = 0x138;
/// Goal voltage register (`f32`, volts).
///
/// Upstream: `bareFOC/src/control_table.rs` (`REG_GOAL_VOLTAGE`, `0x13C`).
pub const REG_GOAL_VOLTAGE: u16 = 0x13C;
/// Hardware error status register (`u16`, read-only bitfield).
///
/// Upstream: `bareFOC/src/control_table.rs` (`REG_HARDWARE_ERROR_STATUS`, `0x140`).
pub const REG_HARDWARE_ERROR_STATUS: u16 = 0x140;
/// Measured position register (`f32`, radians, read-only).
///
/// Upstream: `bareFOC/src/control_table.rs` (`REG_PRESENT_POSITION`, `0x144`).
pub const REG_PRESENT_POSITION: u16 = 0x144;
/// Measured velocity register (`f32`, rad/s, read-only).
///
/// Upstream: `bareFOC/src/control_table.rs` (`REG_PRESENT_VELOCITY`, `0x148`).
pub const REG_PRESENT_VELOCITY: u16 = 0x148;
/// Measured current register (`f32`, amperes, read-only).
///
/// Upstream: `bareFOC/src/control_table.rs` (`REG_PRESENT_CURRENT`, `0x14C`).
pub const REG_PRESENT_CURRENT: u16 = 0x14C;

// ---------------------------------------------------------------------------
// Extended telemetry & control registers (`extended-telemetry-controls`, D2).
// All offsets are canonical 16-bit addresses; see the authoritative register
// allocation in `openspec/changes/extended-telemetry-controls/design.md`.
// ---------------------------------------------------------------------------

/// Equipped-peripheral capability bitmask (`u32`, read-only).
pub const REG_HARDWARE_CAPABILITIES: u16 = 0x1F8;
/// Measured DC bus current (`f32`, amperes, read-only).
pub const REG_PRESENT_BUS_CURRENT: u16 = 0x180;
/// Measured DC bus power (`f32`, watts, read-only).
pub const REG_PRESENT_BUS_POWER: u16 = 0x184;
/// Integrated energy (`f32`, watt-hours, read-only).
pub const REG_ACCUMULATED_ENERGY: u16 = 0x188;
/// Measured FOC quadrature current Iq (`f32`, amperes, read-only).
pub const REG_FOC_IQ_MEASURED: u16 = 0x18C;
/// Measured FOC direct current Id (`f32`, amperes, read-only).
pub const REG_FOC_ID_MEASURED: u16 = 0x190;
/// SVPWM modulation index (`f32`, read-only).
pub const REG_MODULATION_INDEX: u16 = 0x194;
/// Brake chopper duty cycle (`u8`, read-only).
pub const REG_BRAKE_CHOPPER_DUTY: u16 = 0x198;
/// Estimated shaft torque (`f32`, newton-metres, read-only).
pub const REG_PRESENT_TORQUE: u16 = 0x19C;
/// Estimated shaft acceleration (`f32`, rad/s², read-only).
pub const REG_PRESENT_ACCELERATION: u16 = 0x1A0;
/// Goal-minus-present tracking error (`f32`, radians, read-only).
pub const REG_TRACKING_ERROR: u16 = 0x1A4;
/// Dual-encoder gear deflection (`f32`, radians, read-only).
pub const REG_GEAR_DEFLECTION: u16 = 0x1A8;
/// Inverse-dynamics torque feedforward (`f32`, newton-metres, read-write).
pub const REG_GOAL_TORQUE_FF: u16 = 0x1AC;
/// Virtual joint stiffness (`f32`, N·m/rad, read-write).
pub const REG_IMPEDANCE_KP: u16 = 0x1B0;
/// Virtual joint damping (`f32`, N·m·s/rad, read-write).
pub const REG_IMPEDANCE_KD: u16 = 0x1B4;
/// Magnetic field magnitude (`f32`, mT, read-only).
pub const REG_MAG_FIELD_MAGNITUDE: u16 = 0x1B8;
/// Magnetic sensor status flags (`u8`, read-only).
pub const REG_MAG_STATUS_FLAGS: u16 = 0x1BC;
/// Cumulative sensor CRC error count (`u16`, read-only).
pub const REG_SENSOR_CRC_ERRORS: u16 = 0x1BE;
/// Bus-silence watchdog timeout (`u16`, milliseconds, read-write).
pub const REG_WATCHDOG_TIMEOUT_MS: u16 = 0x1C0;
/// Bus-silence watchdog remaining time (`u16`, milliseconds, read-only).
pub const REG_WATCHDOG_TIME_LEFT_MS: u16 = 0x1C2;
/// Trajectory generator status (`u8`, read-only).
pub const REG_TRAJECTORY_STATUS: u16 = 0x1C4;
/// In-position band indicator (`f32`, read-only).
pub const REG_IN_POSITION_BAND: u16 = 0x1C8;
/// Position deadband half-width (`f32`, radians, read-write).
pub const REG_POSITION_DEADBAND: u16 = 0x1CC;
/// Latest control-loop execution time (`u16`, microseconds, read-only).
pub const REG_LOOP_TIME_US: u16 = 0x1D0;
/// Maximum observed loop jitter (`u16`, microseconds, read-only).
pub const REG_MAX_LOOP_JITTER_US: u16 = 0x1D2;
/// RTOS CPU load (`u8`, percent, read-only).
pub const REG_CPU_LOAD_PCT: u16 = 0x1D4;
/// Extended 32-bit hardware error flags (`u32`, read-only).
pub const REG_HARDWARE_ERROR_STATUS_EXT: u16 = 0x1D8;
/// Lifetime operating hours (`f32`, read-only).
pub const REG_TOTAL_OPERATING_HOURS: u16 = 0x1DC;
/// Lifetime shaft revolutions (`f64`, read-only).
pub const REG_TOTAL_REVOLUTIONS: u16 = 0x1E0;
/// Fault-history FIFO, 16 bytes / four 4-byte records (`0x1E8..0x1F8`).
pub const REG_FAULT_HISTORY: u16 = 0x1E8;
/// Exclusive end of the fault-history FIFO (`0x1F8`).
pub const REG_FAULT_HISTORY_END: u16 = 0x1F8;

/// Current sensing is fitted (capability bit 0).
pub const CAP_CURRENT_SENSE: u32 = 1 << 0;
/// A second encoder is fitted (capability bit 1).
pub const CAP_DUAL_ENCODER: u32 = 1 << 1;
/// At least one temperature channel is fitted (capability bit 2).
pub const CAP_TEMPERATURE: u32 = 1 << 2;
/// A brake chopper is fitted (capability bit 3).
pub const CAP_BRAKE_CHOPPER: u32 = 1 << 3;
/// Magnetic diagnostics are available (capability bit 4).
pub const CAP_MAGNETIC_DIAGNOSTICS: u32 = 1 << 4;
/// Bus power / energy telemetry is available (capability bit 5).
pub const CAP_POWER_TELEMETRY: u32 = 1 << 5;
/// A communication watchdog is implemented (capability bit 6).
pub const CAP_WATCHDOG: u32 = 1 << 6;
/// RTOS execution-time profiling is available (capability bit 7).
pub const CAP_RTOS_PROFILING: u32 = 1 << 7;

/// Extended error flag: motor stall detected (bit 16).
pub const ERR_EXT_STALL_DETECTED: u32 = 1 << 16;
/// Extended error flag: communication watchdog expired (bit 17).
pub const ERR_EXT_BUS_WATCHDOG: u32 = 1 << 17;
/// Extended error flag: position sensor signal lost (bit 18).
pub const ERR_EXT_SENSOR_LOSS: u32 = 1 << 18;
/// Extended error flag: magnetic field fault (bit 19).
pub const ERR_EXT_MAGNET_FAULT: u32 = 1 << 19;
/// Extended error flag: motor phase open-circuit (bit 20).
pub const ERR_EXT_PHASE_OPEN: u32 = 1 << 20;

/// Base of the extended joint-angle target slot array. Slot `i` lives at
/// `REG_JOINT_TARGET_BASE + i*4`; [`crate::FishyServoNode`] uses slots 1..3
/// (`0x170`, `0x174`, `0x178`) because joint 0 uses [`REG_GOAL_POSITION`].
///
/// The region sits after `MultiTurnAngle` (`0x166`) and remains inside the
/// 512-byte table (`0x16C + 4*3 + 4 = 0x17C <= 0x200`).
///
/// Upstream: `bareFOC/src/control_table.rs` (RAM area `0xE0..0x200`).
pub const REG_JOINT_TARGET_BASE: u16 = 0x16C;
/// Number of joint-angle target slots (one per axis of a 4-axis servo).
pub const MAX_JOINTS: u8 = 4;

/// Trigger value written to [`REG_SAVE_CONFIG_CMD`] to commit EEPROM to Flash.
///
/// Upstream: `bareFOC/src/control_table.rs` (`CMD_SAVE_CONFIG`, `0x55`).
pub const CMD_SAVE_CONFIG: u8 = 0x55;

/// Standard servo instruction: ping.
///
/// Upstream: `barefoc-comms/barefoc-multibus/src/servo_protocol.rs` (`INST_PING`).
pub const INST_PING: u8 = 0x01;
/// Standard servo instruction: read registers.
///
/// Upstream: `barefoc-comms/barefoc-multibus/src/servo_protocol.rs` (`INST_READ`).
pub const INST_READ: u8 = 0x02;
/// Standard servo instruction: write registers.
///
/// Upstream: `barefoc-comms/barefoc-multibus/src/servo_protocol.rs` (`INST_WRITE`).
pub const INST_WRITE: u8 = 0x03;
/// Standard servo instruction: queued action.
///
/// Upstream: `barefoc-comms/barefoc-multibus/src/servo_protocol.rs` (`INST_ACTION`).
pub const INST_ACTION: u8 = 0x05;
/// Standard servo instruction: synchronous multi-node write.
///
/// Upstream: `barefoc-comms/barefoc-multibus/src/servo_protocol.rs` (`INST_SYNC_WRITE`).
pub const INST_SYNC_WRITE: u8 = 0x83;
/// Broadcast node identifier used for disarm and sync frames.
///
/// Upstream: `barefoc-comms/barefoc-multibus/src/servo_protocol.rs` (broadcast `0xFE`).
pub const BROADCAST_ID: u8 = 0xFE;

/// Dynamixel v2 frame header (`0xFF 0xFF 0xFD 0x00`).
///
/// Upstream: `barefoc-comms/barefoc-multibus/src/servo_protocol.rs` (`ServoProtocolKind::V2`).
pub const DXL2_HEADER: [u8; 4] = [0xFF, 0xFF, 0xFD, 0x00];

/// Standardized smart servo operating modes.
///
/// Upstream: `bareFOC/src/control_table.rs` (`OperatingMode`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum OperatingMode {
    /// Closed-loop torque / current control.
    Torque = 0,
    /// Closed-loop velocity control.
    Velocity = 1,
    /// Closed-loop single-turn position control [-pi, +pi].
    Position = 2,
    /// Closed-loop continuous multi-turn extended position control.
    MultiTurnPosition = 3,
    /// Closed-loop position control with active current limiting.
    CurrentBasedPosition = 4,
    /// Open-loop direct phase voltage control.
    OpenLoopVoltage = 5,
    /// Open-loop velocity control without sensor feedback.
    OpenLoopVelocity = 6,
    /// Open-loop angle control without sensor feedback.
    OpenLoopAngle = 7,
}

/// Runtime kinematics topology presets.
///
/// Upstream: `bareFOC/src/control_table.rs` (`TopologyPreset`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TopologyPreset {
    /// 4 independent 1-DOF actuators.
    Independent4x1Dof = 0,
    /// 2 independent 2-DOF differential gimbals.
    Differential2x2Dof = 1,
    /// 1x 1-DOF joint + 1x 2-DOF differential gimbal.
    Hybrid1Dof2Dof = 2,
    /// 2x 1-DOF joints + 1x 2-DOF differential gimbal.
    Hybrid2x1Dof2Dof = 3,
    /// Custom kinematics matrix specified via coupling bitmask.
    Custom = 0xFF,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn register_offsets_match_firmware() {
        assert_eq!(REG_MODEL_NUMBER, 0x00);
        assert_eq!(REG_FIRMWARE_VERSION, 0x02);
        assert_eq!(REG_HARDWARE_UID, 0x04);
        assert_eq!(REG_OPERATING_MODE, 0xE0);
        assert_eq!(REG_TOPOLOGY_PRESET, 0xE2);
        assert_eq!(REG_TORQUE_ENABLE, 0x120);
        assert_eq!(REG_SAVE_CONFIG_CMD, 0x124);
        assert_eq!(REG_GOAL_POSITION, 0x130);
        assert_eq!(REG_GOAL_VELOCITY, 0x134);
        assert_eq!(REG_GOAL_CURRENT, 0x138);
        assert_eq!(REG_GOAL_VOLTAGE, 0x13C);
        assert_eq!(REG_HARDWARE_ERROR_STATUS, 0x140);
        assert_eq!(REG_PRESENT_POSITION, 0x144);
        assert_eq!(REG_PRESENT_VELOCITY, 0x148);
        assert_eq!(REG_PRESENT_CURRENT, 0x14C);
        assert_eq!(CONTROL_TABLE_SIZE, 512);
    }

    #[test]
    fn extended_register_offsets_match_firmware() {
        assert_eq!(REG_HARDWARE_CAPABILITIES, 0x1F8);
        assert_eq!(REG_PRESENT_BUS_CURRENT, 0x180);
        assert_eq!(REG_PRESENT_BUS_POWER, 0x184);
        assert_eq!(REG_ACCUMULATED_ENERGY, 0x188);
        assert_eq!(REG_FOC_IQ_MEASURED, 0x18C);
        assert_eq!(REG_FOC_ID_MEASURED, 0x190);
        assert_eq!(REG_MODULATION_INDEX, 0x194);
        assert_eq!(REG_BRAKE_CHOPPER_DUTY, 0x198);
        assert_eq!(REG_PRESENT_TORQUE, 0x19C);
        assert_eq!(REG_PRESENT_ACCELERATION, 0x1A0);
        assert_eq!(REG_TRACKING_ERROR, 0x1A4);
        assert_eq!(REG_GEAR_DEFLECTION, 0x1A8);
        assert_eq!(REG_GOAL_TORQUE_FF, 0x1AC);
        assert_eq!(REG_IMPEDANCE_KP, 0x1B0);
        assert_eq!(REG_IMPEDANCE_KD, 0x1B4);
        assert_eq!(REG_MAG_FIELD_MAGNITUDE, 0x1B8);
        assert_eq!(REG_MAG_STATUS_FLAGS, 0x1BC);
        assert_eq!(REG_SENSOR_CRC_ERRORS, 0x1BE);
        assert_eq!(REG_WATCHDOG_TIMEOUT_MS, 0x1C0);
        assert_eq!(REG_WATCHDOG_TIME_LEFT_MS, 0x1C2);
        assert_eq!(REG_TRAJECTORY_STATUS, 0x1C4);
        assert_eq!(REG_IN_POSITION_BAND, 0x1C8);
        assert_eq!(REG_POSITION_DEADBAND, 0x1CC);
        assert_eq!(REG_LOOP_TIME_US, 0x1D0);
        assert_eq!(REG_MAX_LOOP_JITTER_US, 0x1D2);
        assert_eq!(REG_CPU_LOAD_PCT, 0x1D4);
        assert_eq!(REG_HARDWARE_ERROR_STATUS_EXT, 0x1D8);
        assert_eq!(REG_TOTAL_OPERATING_HOURS, 0x1DC);
        assert_eq!(REG_TOTAL_REVOLUTIONS, 0x1E0);
        assert_eq!(REG_FAULT_HISTORY, 0x1E8);
        assert_eq!(REG_FAULT_HISTORY_END, 0x1F8);
        assert!(REG_FAULT_HISTORY_END as usize <= CONTROL_TABLE_SIZE);

        assert_eq!(ERR_EXT_STALL_DETECTED, 1 << 16);
        assert_eq!(ERR_EXT_BUS_WATCHDOG, 1 << 17);
        assert_eq!(ERR_EXT_SENSOR_LOSS, 1 << 18);
        assert_eq!(ERR_EXT_MAGNET_FAULT, 1 << 19);
        assert_eq!(ERR_EXT_PHASE_OPEN, 1 << 20);
        assert_eq!(CAP_CURRENT_SENSE, 1 << 0);
        assert_eq!(CAP_DUAL_ENCODER, 1 << 1);
    }

    #[test]
    fn enum_discriminants_match_firmware() {
        assert_eq!(OperatingMode::Torque as u8, 0);
        assert_eq!(OperatingMode::Position as u8, 2);
        assert_eq!(OperatingMode::OpenLoopAngle as u8, 7);
        assert_eq!(TopologyPreset::Independent4x1Dof as u8, 0);
        assert_eq!(TopologyPreset::Differential2x2Dof as u8, 1);
        assert_eq!(TopologyPreset::Hybrid1Dof2Dof as u8, 2);
        assert_eq!(TopologyPreset::Hybrid2x1Dof2Dof as u8, 3);
        assert_eq!(TopologyPreset::Custom as u8, 0xFF);
        assert_eq!(CMD_SAVE_CONFIG, 0x55);
        assert_eq!(INST_SYNC_WRITE, 0x83);
        assert_eq!(BROADCAST_ID, 0xFE);
    }
}
