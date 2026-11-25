//! Mapping between Rocky servo commands/feedback and BareFOC control-table
//! registers, with all unit conversions in one place.
//!
//! One BareFOC node serves exactly one Rocky joint: node `0x20 + joint` for
//! joints `0..21`. `REG_JOINT_TARGET_BASE` / `FishyServoNode` axis slots are
//! deliberately **not** used — they overlap the canonical thermal registers at
//! `0x16A`/`0x16E`.
//!
//! Temperature channels are defined locally because `barefoc-master` does not
//! re-export them. They mirror the canonical BareFOC control table:
//! `0x142` is a whole-degree `u16` MOSFET temperature, `0x16A` is an `f32` MCU
//! temperature, and `0x16E` is an `f32` motor-winding temperature. The burst
//! read at [`FEEDBACK_BURST_REG`] covers all four feedback values in one
//! transaction (`0x142..=0x14F`).

use barefoc_master::{
    ClientError, ControlTableClient, OperatingMode, Transport, TransportError,
    REG_GOAL_CURRENT, REG_GOAL_POSITION, REG_GOAL_VELOCITY, REG_OPERATING_MODE,
    REG_PRESENT_CURRENT, REG_PRESENT_POSITION, REG_PRESENT_VELOCITY, REG_TORQUE_ENABLE,
};
use rk_control::{RawServoCommand, RawServoFeedback};
use rk_proto::{JointCommandMode, MAX_JOINTS};

/// First BareFOC node address used by Rocky joints.
pub const JOINT_NODE_BASE: u8 = 0x20;
/// Rocky Mk1 has 21 actuated joints, therefore nodes `0x20..=0x34`.
pub const MAX_SERVO_JOINTS: usize = 21;

/// Legacy whole-degree MOSFET temperature (`u16`, °C). Mirrors BareFOC
/// `REG_PRESENT_TEMPERATURE`; not re-exported by `barefoc-master`.
pub const REG_PRESENT_TEMPERATURE: u16 = 0x142;
/// MCU silicon temperature (`f32`, °C).
pub const REG_PRESENT_TEMP_MCU: u16 = 0x16A;
/// Motor winding temperature (`f32`, °C).
pub const REG_PRESENT_TEMP_MOTOR: u16 = 0x16E;

/// Register at which the contiguous feedback burst starts (`0x142`).
pub const FEEDBACK_BURST_REG: u16 = REG_PRESENT_TEMPERATURE;
/// `u16` temperature + `f32` position + `f32` velocity + `f32` current.
pub const FEEDBACK_BURST_LEN: usize = 14;

// The four feedback registers must stay contiguous for the single burst read.
const _: () = {
    assert!(REG_PRESENT_TEMPERATURE < REG_PRESENT_POSITION);
    assert!(REG_PRESENT_POSITION + 4 == REG_PRESENT_VELOCITY);
    assert!(REG_PRESENT_VELOCITY + 4 == REG_PRESENT_CURRENT);
    assert!(REG_PRESENT_CURRENT + 4 == FEEDBACK_BURST_REG + FEEDBACK_BURST_LEN as u16);
};

/// Default effective torque constant (0.9 N·m per ampere).
///
/// Derived from `rk_control::impact::DEFAULT_MA_PER_NM = 1111 mA/N·m`:
/// `1000 / 1111 ≈ 0.9 N·m/A`. This includes the gearbox and is documented as
/// an integrator-supplied constant — the BareFOC control table has no KV or
/// torque-constant register.
pub const DEFAULT_NM_PER_AMP: f32 = 0.9;

/// Per-joint torque constants used to convert between Rocky's N·m commands
/// and the BareFOC control table's `f32` ampere registers.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TorqueConstants {
    /// Effective N·m per ampere of motor current, indexed by joint.
    pub nm_per_amp: [f32; MAX_JOINTS],
}

impl Default for TorqueConstants {
    fn default() -> Self {
        Self {
            nm_per_amp: [DEFAULT_NM_PER_AMP; MAX_JOINTS],
        }
    }
}

impl TorqueConstants {
    /// Uniform torque constant for every joint.
    pub const fn uniform(nm_per_amp: f32) -> Self {
        Self {
            nm_per_amp: [nm_per_amp; MAX_JOINTS],
        }
    }

    fn constant(&self, joint: usize) -> f32 {
        let k = self.nm_per_amp[joint.min(MAX_JOINTS - 1)];
        if k.is_finite() && k > 0.0 {
            k
        } else {
            0.0
        }
    }

    /// Convert a commanded torque in N·m to BareFOC goal current in amperes.
    pub fn nm_to_amps(&self, joint: usize, torque_nm: f32) -> f32 {
        let k = self.constant(joint);
        if k == 0.0 { 0.0 } else { torque_nm / k }
    }

    /// Convert a BareFOC current reading in amperes to N·m.
    pub fn amps_to_nm(&self, joint: usize, current_amps: f32) -> f32 {
        self.constant(joint) * current_amps
    }

    /// Convert N·m to Rocky's `u16` milliampere feedback unit.
    pub fn nm_to_milliamps(&self, joint: usize, torque_nm: f32) -> u16 {
        amps_to_milliamps(self.nm_to_amps(joint, torque_nm))
    }

    /// Convert Rocky's milliampere feedback unit back to N·m.
    pub fn milliamps_to_nm(&self, joint: usize, current_ma: u16) -> f32 {
        self.amps_to_nm(joint, (current_ma as f32) * 0.001_f32)
    }
}

/// BareFOC node address for a joint, or `None` when the joint has no servo node.
pub const fn joint_node_addr(joint: usize) -> Option<u8> {
    if joint < MAX_SERVO_JOINTS {
        Some(JOINT_NODE_BASE + joint as u8)
    } else {
        None
    }
}

/// Bind a control-table client to the node serving `joint`.
pub fn bind_joint<T>(joint: usize, transport: T) -> Option<ControlTableClient<T>> {
    joint_node_addr(joint).map(|node| ControlTableClient::new(node, transport))
}

/// Rocky command mode → BareFOC operating mode register value.
pub const fn operating_mode_for(mode: JointCommandMode) -> OperatingMode {
    match mode {
        JointCommandMode::Position => OperatingMode::Position,
        JointCommandMode::Velocity => OperatingMode::Velocity,
        JointCommandMode::Torque => OperatingMode::Torque,
    }
}

/// One fully decoded joint goal, ready to be written to the control table.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointCommand {
    pub torque_enable: bool,
    pub mode: JointCommandMode,
    pub operating_mode: OperatingMode,
    pub position_rad: f32,
    pub velocity_rad_s: f32,
    /// Goal current in amperes (BareFOC `REG_GOAL_CURRENT` unit).
    pub current_amps: f32,
}

/// Decode one joint of a `RawServoCommand` into register-ready values.
pub fn decode_joint_command(
    cmd: &RawServoCommand,
    joint: usize,
    constants: &TorqueConstants,
) -> JointCommand {
    let idx = joint.min(MAX_JOINTS - 1);
    let mode = JointCommandMode::from_wire(cmd.command_mode[idx]);
    JointCommand {
        torque_enable: ((cmd.torque_mask >> idx) & 1) != 0,
        mode,
        operating_mode: operating_mode_for(mode),
        position_rad: cmd.raw_target_rad[idx],
        velocity_rad_s: cmd.raw_velocity_rad_s[idx],
        current_amps: constants.nm_to_amps(idx, cmd.torque_nm[idx]),
    }
}

/// Write all goal registers for one joint through a shared BareFOC client.
///
/// The client must be bound to [`joint_node_addr`]`(joint)`. Callers keep
/// ownership of the transport so firmware can reuse a single bus across joints.
pub fn write_joint_command<T: Transport>(
    client: &mut ControlTableClient<T>,
    joint: usize,
    cmd: &RawServoCommand,
    constants: &TorqueConstants,
) -> Result<(), ClientError<T::Error>> {
    let goal = decode_joint_command(cmd, joint, constants);
    client.write_u8(REG_TORQUE_ENABLE, goal.torque_enable as u8)?;
    client.write_u8(REG_OPERATING_MODE, goal.operating_mode as u8)?;
    client.write_f32(REG_GOAL_POSITION, goal.position_rad)?;
    client.write_f32(REG_GOAL_VELOCITY, goal.velocity_rad_s)?;
    client.write_f32(REG_GOAL_CURRENT, goal.current_amps)?;
    Ok(())
}

/// One decoded joint feedback burst.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointFeedback {
    pub position_rad: f32,
    pub velocity_rad_s: f32,
    pub current_amps: f32,
    pub temperature_c: f32,
}

impl JointFeedback {
    /// Decode a 14-byte `0x142..=0x14F` burst (pointer + LE payload).
    pub fn from_burst(buf: &[u8]) -> Option<Self> {
        decode_feedback_burst(buf)
    }

    /// Encode the four registers as one contiguous LE burst for a register
    /// server (used by the simulator plant).
    pub fn to_burst(self) -> [u8; FEEDBACK_BURST_LEN] {
        encode_feedback_burst(&self)
    }
}

/// Convert raw amperes to Rocky's clamped `u16` milliampere unit.
pub fn amps_to_milliamps(current_amps: f32) -> u16 {
    if !current_amps.is_finite() || current_amps <= 0.0 {
        return 0;
    }
    // `as` saturates for out-of-range floats; adding 0.5 rounds to nearest.
    ((current_amps * 1000.0_f32) + 0.5_f32) as u16
}

/// Decode a contiguous feedback burst into typed values, rejecting NaN/Inf.
pub fn decode_feedback_burst(buf: &[u8]) -> Option<JointFeedback> {
    if buf.len() < FEEDBACK_BURST_LEN {
        return None;
    }
    let temperature_c = u16::from_le_bytes([buf[0], buf[1]]) as f32;
    let position_rad = f32::from_le_bytes([buf[2], buf[3], buf[4], buf[5]]);
    let velocity_rad_s = f32::from_le_bytes([buf[6], buf[7], buf[8], buf[9]]);
    let current_amps = f32::from_le_bytes([buf[10], buf[11], buf[12], buf[13]]);
    if !position_rad.is_finite() || !velocity_rad_s.is_finite() || !current_amps.is_finite() {
        return None;
    }
    Some(JointFeedback {
        position_rad,
        velocity_rad_s,
        current_amps,
        temperature_c,
    })
}

/// Encode typed feedback into the contiguous control-table burst.
pub fn encode_feedback_burst(feedback: &JointFeedback) -> [u8; FEEDBACK_BURST_LEN] {
    let mut out = [0u8; FEEDBACK_BURST_LEN];
    let temp = if feedback.temperature_c.is_finite() {
        (feedback.temperature_c + 0.5_f32) as u16
    } else {
        0
    };
    out[0..2].copy_from_slice(&temp.to_le_bytes());
    out[2..6].copy_from_slice(&feedback.position_rad.to_le_bytes());
    out[6..10].copy_from_slice(&feedback.velocity_rad_s.to_le_bytes());
    out[10..14].copy_from_slice(&feedback.current_amps.to_le_bytes());
    out
}

/// Burst-read present position/velocity/current plus temperature for one joint
/// and merge the decoded values into `feedback`.
///
/// The client must be bound to [`joint_node_addr`]`(joint)`.
pub fn read_joint_feedback<T: Transport>(
    client: &mut ControlTableClient<T>,
    joint: usize,
    feedback: &mut RawServoFeedback,
) -> Result<(), ClientError<T::Error>> {
    let mut buf = [0u8; FEEDBACK_BURST_LEN];
    client.read_bytes(FEEDBACK_BURST_REG, FEEDBACK_BURST_LEN, &mut buf)?;
    let sample = decode_feedback_burst(&buf)
        .ok_or(TransportError::Protocol("non-finite servo feedback burst"))?;
    if joint >= MAX_JOINTS {
        return Ok(());
    }
    feedback.raw_angle_rad[joint] = sample.position_rad;
    feedback.velocity_rad_s[joint] = sample.velocity_rad_s;
    feedback.current_ma[joint] = amps_to_milliamps(sample.current_amps);
    feedback.temp_c[joint] = if sample.temperature_c.is_finite() {
        (sample.temperature_c + 0.5_f32) as u8
    } else {
        0
    };
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use barefoc_master::I2cTransport;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTx};

    const NODE: u8 = 0x20;

    #[test]
    fn node_addressing_covers_one_node_per_joint() {
        assert_eq!(joint_node_addr(0), Some(0x20));
        assert_eq!(joint_node_addr(4), Some(0x24));
        assert_eq!(joint_node_addr(20), Some(0x34));
        assert_eq!(joint_node_addr(21), None);
        assert_eq!(joint_node_addr(MAX_JOINTS), None);
    }

    #[test]
    fn mode_mapping_matches_barefoc_operating_modes() {
        assert_eq!(
            operating_mode_for(JointCommandMode::Position),
            OperatingMode::Position
        );
        assert_eq!(
            operating_mode_for(JointCommandMode::Velocity),
            OperatingMode::Velocity
        );
        assert_eq!(
            operating_mode_for(JointCommandMode::Torque),
            OperatingMode::Torque
        );
        let mode = JointCommandMode::from_wire(99);
        assert_eq!(operating_mode_for(mode), OperatingMode::Position);
    }

    #[test]
    fn torque_constants_convert_nm_and_ma() {
        let constants = TorqueConstants::default();
        assert!((constants.nm_to_amps(0, 1.8) - 2.0).abs() < 1e-6);
        assert_eq!(constants.nm_to_milliamps(0, 1.8), 2000);
        assert!((constants.milliamps_to_nm(0, 800) - 0.72).abs() < 1e-6);
        let custom = TorqueConstants::uniform(0.5);
        assert!((custom.nm_to_amps(3, 0.5) - 1.0).abs() < 1e-6);
        assert_eq!(amps_to_milliamps(-1.0), 0);
        assert_eq!(amps_to_milliamps(f32::NAN), 0);
        assert_eq!(amps_to_milliamps(100.0), u16::MAX);
    }

    #[test]
    fn feedback_burst_round_trip() {
        let sample = JointFeedback {
            position_rad: -1.25,
            velocity_rad_s: 2.5,
            current_amps: 0.62,
            temperature_c: 41.0,
        };
        let wire = encode_feedback_burst(&sample);
        assert_eq!(decode_feedback_burst(&wire), Some(sample));
        assert_eq!(decode_feedback_burst(&wire[..13]), None);
        let mut nan = wire;
        nan[2..6].copy_from_slice(&f32::NAN.to_le_bytes());
        assert_eq!(decode_feedback_burst(&nan), None);
    }

    fn register_frame(reg: u16, payload: &[u8]) -> std::vec::Vec<u8> {
        let mut frame = std::vec![(reg >> 8) as u8, reg as u8];
        frame.extend_from_slice(payload);
        frame
    }

    #[test]
    fn write_joint_command_emits_canonical_register_frames() {
        let expectations = [
            I2cTx::write(NODE, register_frame(REG_TORQUE_ENABLE, &[1])),
            I2cTx::write(NODE, register_frame(REG_OPERATING_MODE, &[1])),
            I2cTx::write(NODE, register_frame(REG_GOAL_POSITION, &0.75_f32.to_le_bytes())),
            I2cTx::write(NODE, register_frame(REG_GOAL_VELOCITY, &2.5_f32.to_le_bytes())),
            I2cTx::write(NODE, register_frame(REG_GOAL_CURRENT, &2.0_f32.to_le_bytes())),
        ];
        let mut bus = I2cMock::new(&expectations);
        {
            let mut client =
                ControlTableClient::new(NODE, I2cTransport::<_, 18>::new(&mut bus));
            let mut cmd = RawServoCommand::default();
            cmd.torque_mask = 1 << 4;
            cmd.command_mode[4] = JointCommandMode::Velocity.to_wire();
            cmd.raw_target_rad[4] = 0.75;
            cmd.raw_velocity_rad_s[4] = 2.5;
            cmd.torque_nm[4] = 1.8;
            write_joint_command(&mut client, 4, &cmd, &TorqueConstants::default()).unwrap();
            assert_eq!(client.node(), NODE);
        }
        bus.done();
    }

    #[test]
    fn read_joint_feedback_burst_reads_and_maps_into_raw_feedback() {
        let sample = JointFeedback {
            position_rad: -0.5,
            velocity_rad_s: 1.25,
            current_amps: 0.42,
            temperature_c: 39.0,
        };
        let payload = encode_feedback_burst(&sample);
        let expectations = [
            I2cTx::write(NODE, std::vec![0x01, 0x42]),
            I2cTx::read(NODE, std::vec::Vec::from(payload)),
        ];
        let mut bus = I2cMock::new(&expectations);
        let mut feedback = RawServoFeedback::default();
        {
            let mut client =
                ControlTableClient::new(NODE, I2cTransport::<_, 18>::new(&mut bus));
            read_joint_feedback(&mut client, 0, &mut feedback).unwrap();
        }
        bus.done();
        assert!((feedback.raw_angle_rad[0] + 0.5).abs() < 1e-6);
        assert!((feedback.velocity_rad_s[0] - 1.25).abs() < 1e-6);
        assert_eq!(feedback.current_ma[0], 420);
        assert_eq!(feedback.temp_c[0], 39);
        assert_eq!(feedback.raw_angle_rad[1], 0.0, "other joints untouched");
    }

    #[test]
    fn bind_joint_rejects_unservoed_joints() {
        let transport = ();
        assert!(bind_joint(0, transport).is_some());
        assert!(bind_joint(21, transport).is_none());
    }
}
