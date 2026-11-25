//! Atomic multi-node synchronous dispatch and broadcast failsafe frames.
//!
//! Frames use the Dynamixel v2 layout accepted by the target's
//! `StreamingServoParser` (header `FF FF FD 00`, 16-bit little-endian length,
//! 16-bit little-endian register address, 16-bit CRC).
//!
//! Upstream: `barefoc-comms/barefoc-multibus/src/servo_protocol.rs`
//! (`INST_SYNC_WRITE`, `calc_dxl2_crc`, `StreamingServoParser`),
//! `bareFOC/src/control_table.rs` (`REG_TORQUE_ENABLE`).

use crate::client::ClientError;
use crate::registers::{
    BROADCAST_ID, DXL2_HEADER, INST_SYNC_WRITE, INST_WRITE, REG_GOAL_POSITION, REG_TORQUE_ENABLE,
};
use crate::scanner::MAX_NODES;
use crate::transport::{Transport, TransportError};
use heapless::Vec as HVec;

/// Maximum bytes in one compile-time-bounded dispatch packet.
///
/// Header (8) + address/length (4) + CRC (2) + 16 nodes x 5 bytes = 94.
pub const MAX_PACKET_LEN: usize = 128;
/// Initial value of the Dynamixel v2 CRC-16 accumulator.
///
/// Upstream: `barefoc-comms/barefoc-multibus/src/servo_protocol.rs` (`calc_dxl2_crc`).
pub const DXL2_CRC_INIT: u16 = 0;

const SYNC_DATA_LEN: u16 = 4; // one little-endian f32 per node

/// Errors produced while packing a dispatch frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SyncError {
    /// No nodes were supplied.
    Empty,
    /// More than [`MAX_NODES`] targets were supplied.
    TooManyNodes,
    /// The caller's output buffer cannot hold the complete frame.
    BufferTooSmall,
}

impl<E> From<SyncError> for TransportError<E> {
    fn from(err: SyncError) -> Self {
        match err {
            SyncError::Empty => Self::Protocol("sync group is empty"),
            SyncError::TooManyNodes => Self::Protocol("too many nodes in sync group"),
            SyncError::BufferTooSmall => Self::BufferTooSmall,
        }
    }
}

impl core::fmt::Display for SyncError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Empty => f.write_str("sync group is empty"),
            Self::TooManyNodes => f.write_str("too many nodes in sync group"),
            Self::BufferTooSmall => f.write_str("sync frame buffer too small"),
        }
    }
}

/// Dynamixel v2 CRC-16 (polynomial `0x8005`, initial value [`DXL2_CRC_INIT`]).
///
/// Upstream: `barefoc-comms/barefoc-multibus/src/servo_protocol.rs` (`calc_dxl2_crc`).
pub fn calc_dxl2_crc(bytes: &[u8]) -> u16 {
    let mut crc = DXL2_CRC_INIT;
    for &byte in bytes {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if (crc & 0x8000) != 0 {
                crc = (crc << 1) ^ 0x8005;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

/// Pack an `INST_SYNC_WRITE` frame into `out`, returning the used byte count.
///
/// The address header is a little-endian `u16`; each node contributes its id
/// followed by a little-endian `f32` target.
///
/// Upstream: `barefoc-comms/barefoc-multibus/src/servo_protocol.rs` (`INST_SYNC_WRITE`).
pub fn build_sync_write_frame(
    address: u16,
    targets: &[(u8, f32)],
    out: &mut [u8],
) -> Result<usize, SyncError> {
    if targets.is_empty() {
        return Err(SyncError::Empty);
    }
    if targets.len() > MAX_NODES {
        return Err(SyncError::TooManyNodes);
    }
    let needed = 14 + 5 * targets.len();
    if out.len() < needed {
        return Err(SyncError::BufferTooSmall);
    }

    out[..4].copy_from_slice(&DXL2_HEADER);
    out[4] = BROADCAST_ID;
    let packet_len = 7 + 5 * targets.len();
    out[5..7].copy_from_slice(&(packet_len as u16).to_le_bytes());
    out[7] = INST_SYNC_WRITE;
    out[8..10].copy_from_slice(&address.to_le_bytes());
    out[10..12].copy_from_slice(&SYNC_DATA_LEN.to_le_bytes());

    let mut offset = 12;
    for &(node, target) in targets {
        out[offset] = node;
        out[offset + 1..offset + 5].copy_from_slice(&target.to_le_bytes());
        offset += 5;
    }
    let crc = calc_dxl2_crc(&out[..offset]);
    out[offset..offset + 2].copy_from_slice(&crc.to_le_bytes());
    Ok(offset + 2)
}

/// Pack a broadcast `INST_WRITE` frame that disarms every node by writing
/// `0` to `REG_TORQUE_ENABLE` (`0x120`).
///
/// Upstream: `bareFOC/src/control_table.rs` (`REG_TORQUE_ENABLE`),
/// `barefoc-comms/barefoc-multibus/src/servo_protocol.rs` (`INST_WRITE`).
pub fn build_emergency_stop_frame(out: &mut [u8]) -> Result<usize, SyncError> {
    const NEEDED: usize = 13;
    if out.len() < NEEDED {
        return Err(SyncError::BufferTooSmall);
    }
    out[..4].copy_from_slice(&DXL2_HEADER);
    out[4] = BROADCAST_ID;
    out[5..7].copy_from_slice(&6u16.to_le_bytes());
    out[7] = INST_WRITE;
    out[8..10].copy_from_slice(&REG_TORQUE_ENABLE.to_le_bytes());
    out[10] = 0; // torque disable
    let crc = calc_dxl2_crc(&out[..11]);
    out[11..13].copy_from_slice(&crc.to_le_bytes());
    Ok(NEEDED)
}

/// Coordinator that batches multi-node setpoints into a single atomic bus
/// frame and dispatches broadcast failsafe frames.
///
/// Upstream: `barefoc-comms/barefoc-multibus/src/servo_protocol.rs`
/// (`parse_sync_write_setpoints` counterpart on the master side).
pub struct SyncGroup<T> {
    transport: T,
}

impl<T> SyncGroup<T> {
    /// Create a sync group that dispatches through `transport`.
    pub const fn new(transport: T) -> Self {
        Self { transport }
    }

    /// Borrow the transport.
    pub fn transport(&self) -> &T {
        &self.transport
    }

    /// Mutably borrow the transport.
    pub fn transport_mut(&mut self) -> &mut T {
        &mut self.transport
    }

    /// Consume the group and return its transport.
    pub fn into_inner(self) -> T {
        self.transport
    }
}

impl<T: Transport> SyncGroup<T> {
    /// Atomically update `REG_GOAL_POSITION` for every `(node_id, target_rad)`.
    ///
    /// Upstream: `bareFOC/src/control_table.rs` (`REG_GOAL_POSITION`).
    pub fn sync_write_position(
        &mut self,
        targets: &[(u8, f32)],
    ) -> Result<(), ClientError<T::Error>> {
        let mut frame: HVec<u8, MAX_PACKET_LEN> = HVec::new();
        frame
            .resize(MAX_PACKET_LEN, 0)
            .map_err(|_| TransportError::BufferTooSmall)?;
        let len = build_sync_write_frame(REG_GOAL_POSITION, targets, &mut frame)
            .map_err(TransportError::from)?;
        frame.truncate(len);
        self.transport.send_packet(&frame)?;
        Ok(())
    }

    /// Broadcast an emergency disarm: `REG_TORQUE_ENABLE = 0` on all nodes.
    ///
    /// Upstream: `bareFOC/src/control_table.rs` (`REG_TORQUE_ENABLE`).
    pub fn emergency_stop(&mut self) -> Result<(), ClientError<T::Error>> {
        let mut frame: HVec<u8, MAX_PACKET_LEN> = HVec::new();
        frame
            .resize(MAX_PACKET_LEN, 0)
            .map_err(|_| TransportError::BufferTooSmall)?;
        let len =
            build_emergency_stop_frame(&mut frame).map_err(TransportError::from)?;
        frame.truncate(len);
        self.transport.send_packet(&frame)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sync_write_frame_packs_address_and_node_payloads() {
        let targets = [(1u8, 1.5708f32), (2, -3.1415)];
        let mut buf = [0u8; MAX_PACKET_LEN];
        let len = build_sync_write_frame(REG_GOAL_POSITION, &targets, &mut buf).unwrap();
        assert_eq!(len, 24);
        assert_eq!(&buf[..4], &DXL2_HEADER);
        assert_eq!(buf[4], BROADCAST_ID);
        // Length = instruction(1) + addr(2) + data_len(2) + 2*5 + CRC(2) = 17.
        assert_eq!(&buf[5..7], &17u16.to_le_bytes());
        assert_eq!(buf[7], INST_SYNC_WRITE);
        assert_eq!(&buf[8..10], &0x0130u16.to_le_bytes());
        assert_eq!(&buf[10..12], &4u16.to_le_bytes());
        assert_eq!(buf[12], 1);
        assert_eq!(&buf[13..17], &1.5708f32.to_le_bytes());
        assert_eq!(buf[17], 2);
        assert_eq!(&buf[18..22], &(-3.1415f32).to_le_bytes());
        assert_eq!(
            &buf[22..24],
            &calc_dxl2_crc(&buf[..22]).to_le_bytes()
        );
    }

    #[test]
    fn emergency_stop_frame_disables_torque_on_broadcast() {
        let mut buf = [0u8; MAX_PACKET_LEN];
        let len = build_emergency_stop_frame(&mut buf).unwrap();
        assert_eq!(len, 13);
        assert_eq!(&buf[..4], &DXL2_HEADER);
        assert_eq!(buf[4], BROADCAST_ID);
        assert_eq!(&buf[5..7], &6u16.to_le_bytes());
        assert_eq!(buf[7], INST_WRITE);
        assert_eq!(&buf[8..10], &0x0120u16.to_le_bytes());
        assert_eq!(buf[10], 0);
        assert_eq!(&buf[11..13], &calc_dxl2_crc(&buf[..11]).to_le_bytes());
    }

    #[test]
    fn frame_builders_reject_invalid_inputs() {
        let mut buf = [0u8; 8];
        assert_eq!(
            build_sync_write_frame(REG_GOAL_POSITION, &[], &mut buf),
            Err(SyncError::Empty)
        );
        let sixteen = [(1u8, 0.0f32); MAX_NODES + 1];
        assert_eq!(
            build_sync_write_frame(REG_GOAL_POSITION, &sixteen, &mut buf),
            Err(SyncError::TooManyNodes)
        );
        assert_eq!(
            build_sync_write_frame(REG_GOAL_POSITION, &[(1, 0.0)], &mut buf),
            Err(SyncError::BufferTooSmall)
        );
        assert_eq!(
            build_emergency_stop_frame(&mut [0u8; 12]),
            Err(SyncError::BufferTooSmall)
        );
    }

    struct RecordingTransport {
        sent: HVec<u8, MAX_PACKET_LEN>,
    }

    impl Transport for RecordingTransport {
        type Error = ();

        fn write(&mut self, _node: u8, _reg: u16, _bytes: &[u8]) -> Result<(), TransportError<()>> {
            Ok(())
        }

        fn read(&mut self, _node: u8, _reg: u16, _out: &mut [u8]) -> Result<(), TransportError<()>> {
            Ok(())
        }

        fn send_packet(&mut self, packet: &[u8]) -> Result<(), TransportError<()>> {
            self.sent.clear();
            self.sent
                .extend_from_slice(packet)
                .map_err(|_| TransportError::BufferTooSmall)
        }
    }

    #[test]
    fn sync_group_dispatches_exact_frames() {
        let mut group = SyncGroup::new(RecordingTransport {
            sent: HVec::new(),
        });
        group.sync_write_position(&[(3, 0.5)]).unwrap();
        let mut expected = [0u8; MAX_PACKET_LEN];
        let len = build_sync_write_frame(REG_GOAL_POSITION, &[(3, 0.5)], &mut expected).unwrap();
        assert_eq!(&group.transport().sent[..], &expected[..len]);

        group.emergency_stop().unwrap();
        let len = build_emergency_stop_frame(&mut expected).unwrap();
        assert_eq!(&group.transport().sent[..], &expected[..len]);
    }
}
