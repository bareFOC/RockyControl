//! # `barefoc-master`
//!
//! A `#![no_std]`, zero-heap master-controller SDK for discovering,
//! synchronizing, and commanding multi-axis BareFOC actuator nodes over
//! multidrop buses.
//!
//! The crate mirrors the target firmware's canonical 384-byte control table
//! (see `bareFOC/src/control_table.rs`) and servo instruction set
//! (see `barefoc-comms/barefoc-multibus/src/servo_protocol.rs`). It deliberately
//! depends only on `embedded-hal` 1.0, `embedded-io`, and `heapless` so a host
//! MCU does not need to link motor-control math, ADC, or gate-driver code.
//!
//! All register addresses are `u16` because RAM registers exceed `0xFF`.
//! Multi-byte register **values** are little-endian; the I2C register pointer
//! is a two-byte big-endian address.
//!
//! # Example
//!
//! ```ignore
//! use barefoc_master::{ControlTableClient, FishyServoNode, TopologyPreset, REG_PRESENT_POSITION};
//!
//! let mut node = FishyServoNode::new(0x40, transport);
//! node.set_topology_mode(TopologyPreset::Independent4x1Dof)?;
//! node.set_joint_target(0, 1.5708)?;
//! let angle = node.present_position()?;
//! # Ok::<(), barefoc_master::ClientError<embedded_hal::i2c::ErrorKind>>(())
//! ```

#![no_std]

#[cfg(test)]
extern crate std;

pub mod client;
pub mod passthrough;
pub mod proxies;
pub mod registers;
pub mod scanner;
pub mod sync;
pub mod transport;

pub use client::{BoundsError, ClientError, ControlTableClient};
pub use proxies::{BldcFocNode, FishyServoNode};
pub use registers::{
    OperatingMode, TopologyPreset, BROADCAST_ID, CMD_SAVE_CONFIG,
    CONTROL_TABLE_SIZE, DXL2_HEADER, INST_ACTION, INST_PING, INST_READ, INST_SYNC_WRITE,
    INST_WRITE, MAX_JOINTS, REG_FIRMWARE_VERSION, REG_GOAL_CURRENT, REG_GOAL_POSITION,
    REG_GOAL_VELOCITY, REG_GOAL_VOLTAGE, REG_HARDWARE_ERROR_STATUS, REG_HARDWARE_UID,
    REG_JOINT_TARGET_BASE, REG_MODEL_NUMBER, REG_OPERATING_MODE, REG_PRESENT_CURRENT,
    REG_PRESENT_POSITION, REG_PRESENT_VELOCITY, REG_SAVE_CONFIG_CMD, REG_TOPOLOGY_PRESET,
    REG_TORQUE_ENABLE,
};
pub use scanner::{BusScanner, NodeDescriptor, MAX_NODES};
pub use sync::{SyncGroup, DXL2_CRC_INIT, MAX_PACKET_LEN};
pub use passthrough::{
    bus_kind, crc16_ccitt, decode_frame, encode_frame, response_opcode, BusKind, DeviceInfo,
    Frame, FrameError, PassthroughTransport, ScanMask, StatusCode,
    BUS_BROADCAST_ALL, BUS_CAN_BASE, BUS_CAN_LAST, BUS_CAP_CAN, BUS_CAP_I2C, BUS_CAP_LOCAL,
    BUS_CAP_UART, BUS_I2C_BASE, BUS_I2C_LAST, BUS_LOCAL, BUS_UART_BASE, BUS_UART_LAST,
    DEFAULT_WATCHDOG_MS, DEVICE_INFO_LEN, LOCAL_REG_CAPABILITIES, LOCAL_REG_ERROR_FLAGS,
    LOCAL_REG_FIRMWARE_VERSION, LOCAL_REG_MODE_STATE, LOCAL_REG_TEMPERATURE,
    LOCAL_REG_WATCHDOG_TIMEOUT_MS, MAX_FRAME_LEN, OP_EMERGENCY_STOP, OP_ENTER, OP_EXIT,
    OP_GET_INFO, OP_KEEPALIVE, OP_PING, OP_READ, OP_SCAN_BUS, OP_SYNC_WRITE, OP_WRITE,
    PROTOCOL_VERSION, PT_CRC_LEN, PT_HEADER_LEN, PT_MAX_PAYLOAD, PT_OVERHEAD, PT_SYNC,
    RESP_EMERGENCY_STOP, RESP_ENTER, RESP_EXIT, RESP_GET_INFO, RESP_KEEPALIVE, RESP_PING,
    RESP_READ, RESP_SCAN_BUS, RESP_SYNC_WRITE, RESP_WRITE, RESPONSE_FLAG, SCAN_MASK_LEN,
};
pub use transport::{
    I2cTransport, StreamTransport, Transport, TransportError, MAX_I2C_WRITE_FRAME,
    MAX_STREAM_PAYLOAD,
};
