//! High-level actuator node proxies.
//!
//! [`FishyServoNode`] targets 4-axis multi-topology DC servo firmware;
//! [`BldcFocNode`] targets single-axis BLDC FOC nodes. Both wrap a
//! [`ControlTableClient`] and therefore inherit its bounds checking and
//! little-endian encoding.
//!
//! Upstream: `bareFOC/src/control_table.rs`, `bareFOC/src/kinematics/assembly.rs`
//! (topology presets), `barefoc-comms/barefoc-multibus/src/servo_protocol.rs`.

use crate::client::{ClientError, ControlTableClient};
use crate::registers::{
    CMD_SAVE_CONFIG, MAX_JOINTS, OperatingMode, REG_GOAL_CURRENT, REG_GOAL_POSITION,
    REG_GOAL_VELOCITY, REG_GOAL_VOLTAGE, REG_HARDWARE_ERROR_STATUS, REG_JOINT_TARGET_BASE,
    REG_OPERATING_MODE, REG_PRESENT_CURRENT, REG_PRESENT_POSITION, REG_PRESENT_VELOCITY,
    REG_SAVE_CONFIG_CMD, REG_TOPOLOGY_PRESET, TopologyPreset,
};
use crate::transport::{Transport, TransportError};

/// Proxy for a 4-axis multi-topology FishyServo node.
///
/// Upstream: `bareFOC/src/control_table.rs`, `bareFOC/src/kinematics/assembly.rs`.
pub struct FishyServoNode<T> {
    client: ControlTableClient<T>,
}

impl<T> FishyServoNode<T> {
    /// Bind a proxy to `node` on `transport`.
    pub const fn new(node: u8, transport: T) -> Self {
        Self {
            client: ControlTableClient::new(node, transport),
        }
    }

    /// Borrow the underlying control-table client.
    pub fn client(&self) -> &ControlTableClient<T> {
        &self.client
    }

    /// Mutably borrow the underlying control-table client.
    pub fn client_mut(&mut self) -> &mut ControlTableClient<T> {
        &mut self.client
    }

    /// Consume the proxy and return its control-table client.
    pub fn into_inner(self) -> ControlTableClient<T> {
        self.client
    }
}

impl<T: Transport> FishyServoNode<T> {
    /// Command a joint angle in radians.
    ///
    /// Joint 0 uses the canonical `REG_GOAL_POSITION` (`0x130`). Joints 1..3 use
    /// the four consecutive 4-byte extended target slots beginning at
    /// `REG_JOINT_TARGET_BASE` (`0x16C`), inside the spare RAM tail of the
    /// 384-byte table. This avoids overwriting the velocity/current/voltage
    /// goal registers at `0x134`/`0x138`/`0x13C`.
    ///
    /// Upstream: `bareFOC/src/control_table.rs` (`REG_GOAL_POSITION`, RAM area).
    pub fn set_joint_target(
        &mut self,
        joint_index: u8,
        target_rad: f32,
    ) -> Result<(), ClientError<T::Error>> {
        if joint_index >= MAX_JOINTS {
            return Err(ClientError::Transport(TransportError::InvalidAddress(
                joint_index as u16,
            )));
        }
        let reg = if joint_index == 0 {
            REG_GOAL_POSITION
        } else {
            REG_JOINT_TARGET_BASE + (joint_index as u16) * 4
        };
        self.client.write_f32(reg, target_rad)
    }

    /// Switch the on-device multi-motor kinematics topology.
    ///
    /// Upstream: `bareFOC/src/control_table.rs` (`REG_TOPOLOGY_PRESET`, `0xE2`).
    pub fn set_topology_mode(
        &mut self,
        preset: TopologyPreset,
    ) -> Result<(), ClientError<T::Error>> {
        self.client.write_u8(REG_TOPOLOGY_PRESET, preset as u8)
    }

    /// Read the measured joint/shaft position in radians.
    ///
    /// Upstream: `bareFOC/src/control_table.rs` (`REG_PRESENT_POSITION`, `0x144`).
    pub fn present_position(&mut self) -> Result<f32, ClientError<T::Error>> {
        self.client.read_f32(REG_PRESENT_POSITION)
    }

    /// Read the measured joint/shaft velocity in rad/s.
    ///
    /// Upstream: `bareFOC/src/control_table.rs` (`REG_PRESENT_VELOCITY`, `0x148`).
    pub fn present_velocity(&mut self) -> Result<f32, ClientError<T::Error>> {
        self.client.read_f32(REG_PRESENT_VELOCITY)
    }

    /// Read the measured motor current in amperes.
    ///
    /// Upstream: `bareFOC/src/control_table.rs` (`REG_PRESENT_CURRENT`, `0x14C`).
    pub fn present_current(&mut self) -> Result<f32, ClientError<T::Error>> {
        self.client.read_f32(REG_PRESENT_CURRENT)
    }

    /// Read the 16-bit hardware fault bitfield.
    ///
    /// Upstream: `bareFOC/src/control_table.rs` (`REG_HARDWARE_ERROR_STATUS`, `0x140`).
    pub fn hardware_error_status(&mut self) -> Result<u16, ClientError<T::Error>> {
        self.client.read_u16(REG_HARDWARE_ERROR_STATUS)
    }

    /// Trigger an on-device Flash commit of the EEPROM configuration area.
    ///
    /// Upstream: `bareFOC/src/control_table.rs` (`REG_SAVE_CONFIG_CMD`, `CMD_SAVE_CONFIG`).
    pub fn save_config(&mut self) -> Result<(), ClientError<T::Error>> {
        self.client.write_u8(REG_SAVE_CONFIG_CMD, CMD_SAVE_CONFIG)
    }
}

/// Proxy for a single-axis BLDC FOC node.
///
/// Upstream: `bareFOC/src/control_table.rs` (`OperatingMode`, goal registers).
pub struct BldcFocNode<T> {
    client: ControlTableClient<T>,
}

impl<T> BldcFocNode<T> {
    /// Bind a proxy to `node` on `transport`.
    pub const fn new(node: u8, transport: T) -> Self {
        Self {
            client: ControlTableClient::new(node, transport),
        }
    }

    /// Borrow the underlying control-table client.
    pub fn client(&self) -> &ControlTableClient<T> {
        &self.client
    }

    /// Mutably borrow the underlying control-table client.
    pub fn client_mut(&mut self) -> &mut ControlTableClient<T> {
        &mut self.client
    }

    /// Consume the proxy and return its control-table client.
    pub fn into_inner(self) -> ControlTableClient<T> {
        self.client
    }
}

impl<T: Transport> BldcFocNode<T> {
    /// Set the operating mode (`REG_OPERATING_MODE`, `0xE0`).
    ///
    /// Upstream: `bareFOC/src/control_table.rs` (`OperatingMode`).
    pub fn set_operating_mode(
        &mut self,
        mode: OperatingMode,
    ) -> Result<(), ClientError<T::Error>> {
        self.client.write_u8(REG_OPERATING_MODE, mode as u8)
    }

    /// Command drive voltage in volts (`REG_GOAL_VOLTAGE`, `0x13C`).
    ///
    /// Upstream: `bareFOC/src/control_table.rs` (`REG_GOAL_VOLTAGE`).
    pub fn set_voltage(&mut self, volts: f32) -> Result<(), ClientError<T::Error>> {
        self.client.write_f32(REG_GOAL_VOLTAGE, volts)
    }

    /// Command shaft velocity in rad/s (`REG_GOAL_VELOCITY`, `0x134`).
    ///
    /// Upstream: `bareFOC/src/control_table.rs` (`REG_GOAL_VELOCITY`).
    pub fn set_velocity(&mut self, rad_per_s: f32) -> Result<(), ClientError<T::Error>> {
        self.client.write_f32(REG_GOAL_VELOCITY, rad_per_s)
    }

    /// Command shaft angle in radians (`REG_GOAL_POSITION`, `0x130`).
    ///
    /// Upstream: `bareFOC/src/control_table.rs` (`REG_GOAL_POSITION`).
    pub fn set_angle(&mut self, angle_rad: f32) -> Result<(), ClientError<T::Error>> {
        self.client.write_f32(REG_GOAL_POSITION, angle_rad)
    }

    /// Alias of [`Self::set_angle`] for position-control callers.
    pub fn set_position(&mut self, position_rad: f32) -> Result<(), ClientError<T::Error>> {
        self.set_angle(position_rad)
    }

    /// Command torque-producing current in amperes (`REG_GOAL_CURRENT`, `0x138`).
    ///
    /// Upstream: `bareFOC/src/control_table.rs` (`REG_GOAL_CURRENT`).
    pub fn set_torque(&mut self, current_a: f32) -> Result<(), ClientError<T::Error>> {
        self.client.write_f32(REG_GOAL_CURRENT, current_a)
    }

    /// Read the measured shaft position in radians.
    ///
    /// Upstream: `bareFOC/src/control_table.rs` (`REG_PRESENT_POSITION`).
    pub fn present_position(&mut self) -> Result<f32, ClientError<T::Error>> {
        self.client.read_f32(REG_PRESENT_POSITION)
    }

    /// Read the measured shaft velocity in rad/s.
    ///
    /// Upstream: `bareFOC/src/control_table.rs` (`REG_PRESENT_VELOCITY`).
    pub fn present_velocity(&mut self) -> Result<f32, ClientError<T::Error>> {
        self.client.read_f32(REG_PRESENT_VELOCITY)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::I2cTransport;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTx};

    const NODE: u8 = 0x40;

    #[test]
    fn fishy_servo_joint_targets_use_documented_slots() {
        let expectations = [
            // Joint 0 -> 0x130, 1.0f32 LE.
            I2cTx::write(NODE, std::vec![0x01, 0x30, 0x00, 0x00, 0x80, 0x3F]),
            // Joint 2 -> 0x16C + 2*4 = 0x174, -2.0f32 LE.
            I2cTx::write(NODE, std::vec![0x01, 0x74, 0x00, 0x00, 0x00, 0xC0]),
        ];
        let mut bus = I2cMock::new(&expectations);
        {
            let mut node = FishyServoNode::new(NODE, I2cTransport::<_, 18>::new(&mut bus));
            node.set_joint_target(0, 1.0).unwrap();
            node.set_joint_target(2, -2.0).unwrap();
            assert_eq!(
                node.set_joint_target(4, 0.0),
                Err(ClientError::Transport(TransportError::InvalidAddress(4)))
            );
        }
        bus.done();
    }

    #[test]
    fn fishy_servo_topology_telemetry_and_save() {
        let expectations = [
            // Topology set to Hybrid2x1Dof2Dof (3).
            I2cTx::write(NODE, std::vec![0x00, 0xE2, 0x03]),
            // present velocity at 0x148 -> 3.5f32 LE.
            I2cTx::write(NODE, std::vec![0x01, 0x48]),
            I2cTx::read(NODE, std::vec![0x00, 0x00, 0x60, 0x40]),
            // save config trigger 0x55 at 0x124.
            I2cTx::write(NODE, std::vec![0x01, 0x24, 0x55]),
        ];
        let mut bus = I2cMock::new(&expectations);
        {
            let mut node = FishyServoNode::new(NODE, I2cTransport::<_, 18>::new(&mut bus));
            node.set_topology_mode(TopologyPreset::Hybrid2x1Dof2Dof)
                .unwrap();
            assert_eq!(node.present_velocity().unwrap(), 3.5);
            node.save_config().unwrap();
        }
        bus.done();
    }

    #[test]
    fn bldc_foc_setters_and_mode() {
        let v = 7.25f32.to_le_bytes();
        let expectations = [
            // OperatingMode::Position (2) at 0xE0.
            I2cTx::write(NODE, std::vec![0x00, 0xE0, 0x02]),
            // Voltage at 0x13C.
            I2cTx::write(NODE, std::vec![0x01, 0x3C, v[0], v[1], v[2], v[3]]),
            // Velocity at 0x134 -> 1.0.
            I2cTx::write(NODE, std::vec![0x01, 0x34, 0x00, 0x00, 0x80, 0x3F]),
            // Angle at 0x130 -> -1.0.
            I2cTx::write(NODE, std::vec![0x01, 0x30, 0x00, 0x00, 0x80, 0xBF]),
            // Torque at 0x138 -> 0.5.
            I2cTx::write(NODE, std::vec![0x01, 0x38, 0x00, 0x00, 0x00, 0x3F]),
        ];
        let mut bus = I2cMock::new(&expectations);
        {
            let mut node = BldcFocNode::new(NODE, I2cTransport::<_, 18>::new(&mut bus));
            node.set_operating_mode(OperatingMode::Position).unwrap();
            node.set_voltage(7.25).unwrap();
            node.set_velocity(1.0).unwrap();
            node.set_angle(-1.0).unwrap();
            node.set_torque(0.5).unwrap();
        }
        bus.done();
    }
}
