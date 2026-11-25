//! Bus auto-discovery and hardware fingerprinting.
//!
//! [`BusScanner`] probes candidate 7-bit I2C addresses by reading the model
//! number register. Responding nodes are fingerprinted using firmware version
//! and the 96-bit silicon UID, and returned in a bounded inventory.
//!
//! Upstream: `bareFOC/src/control_table.rs` (`REG_MODEL_NUMBER`,
//! `REG_FIRMWARE_VERSION`, `REG_HARDWARE_UID`),
//! `barefoc-comms/barefoc-multibus/src/client.rs` (register-pointer reads).

use core::ops::RangeInclusive;

use crate::registers::{
    REG_FIRMWARE_VERSION, REG_HARDWARE_UID, REG_MODEL_NUMBER,
};
use crate::transport::{Transport, TransportError};
use heapless::Vec as HVec;

/// Maximum number of nodes retained in a single scan inventory.
pub const MAX_NODES: usize = 16;
/// First candidate 7-bit I2C address.
pub const I2C_SCAN_START: u8 = 0x08;
/// Last candidate 7-bit I2C address.
pub const I2C_SCAN_END: u8 = 0x77;

/// Length of the 96-bit silicon UID in bytes.
const UID_LEN: usize = 12;

/// Fingerprint of a discovered actuator node.
///
/// Upstream: `bareFOC/src/control_table.rs` (identity registers `0x00..0x0F`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NodeDescriptor {
    /// 7-bit bus address the node answered on.
    pub address: u8,
    /// Value of `REG_MODEL_NUMBER` (`0x00`).
    pub model_number: u16,
    /// Value of `REG_FIRMWARE_VERSION` (`0x02`).
    pub firmware_version: u8,
    /// Factory 96-bit silicon UID from `REG_HARDWARE_UID` (`0x04`).
    pub uid: [u8; UID_LEN],
}

/// Discover BareFOC nodes by probing bus addresses and reading identity
/// registers.
///
/// Upstream: `bareFOC/src/control_table.rs`, `barefoc-comms/barefoc-multibus`.
pub struct BusScanner<T> {
    transport: T,
}

impl<T> BusScanner<T> {
    /// Create a scanner using `transport` for all register reads.
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

    /// Consume the scanner and return its transport.
    pub fn into_inner(self) -> T {
        self.transport
    }
}

impl<T: Transport> BusScanner<T> {
    /// Probe every address in `addresses` and return the bounded inventory.
    ///
    /// An address that does not acknowledge the model-number read is skipped.
    /// A model number of `0x0000` or `0xFFFF` is treated as unpopulated (bus
    /// float / pull-up artifact) and also skipped. Fingerprint reads on a
    /// confirmed node are propagated as errors.
    pub fn scan_i2c(
        &mut self,
        addresses: RangeInclusive<u8>,
    ) -> Result<HVec<NodeDescriptor, MAX_NODES>, TransportError<T::Error>> {
        let mut found: HVec<NodeDescriptor, MAX_NODES> = HVec::new();
        for address in addresses {
            if found.len() == MAX_NODES {
                break;
            }
            let mut model_bytes = [0u8; 2];
            if self
                .transport
                .read(address, REG_MODEL_NUMBER, &mut model_bytes)
                .is_err()
            {
                continue;
            }
            let model_number = u16::from_le_bytes(model_bytes);
            if model_number == 0x0000 || model_number == 0xFFFF {
                continue;
            }

            let mut firmware_version = [0u8; 1];
            self.transport
                .read(address, REG_FIRMWARE_VERSION, &mut firmware_version)?;
            let mut uid = [0u8; UID_LEN];
            self.transport.read(address, REG_HARDWARE_UID, &mut uid)?;

            found
                .push(NodeDescriptor {
                    address,
                    model_number,
                    firmware_version: firmware_version[0],
                    uid,
                })
                .map_err(|_| TransportError::BufferTooSmall)?;
        }
        Ok(found)
    }

    /// Scan the standard I2C address range `0x08..=0x77`.
    pub fn scan_all_i2c(
        &mut self,
    ) -> Result<HVec<NodeDescriptor, MAX_NODES>, TransportError<T::Error>> {
        self.scan_i2c(I2C_SCAN_START..=I2C_SCAN_END)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::{I2cTransport, TransportError as TE};
    use embedded_hal::i2c::ErrorKind;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTx};

    #[test]
    fn scan_range_fingerprints_nodes_and_skips_silent_addresses() {
        let uid = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC];
        let expectations = [
            // 0x40: responding node.
            I2cTx::write(0x40, std::vec![0x00, 0x00]),
            I2cTx::read(0x40, std::vec![0x03, 0x05]),
            I2cTx::write(0x40, std::vec![0x00, 0x02]),
            I2cTx::read(0x40, std::vec![0x07]),
            I2cTx::write(0x40, std::vec![0x00, 0x04]),
            I2cTx::read(0x40, uid.to_vec()),
            // 0x41: no acknowledge -> skipped.
            I2cTx::write(0x41, std::vec![0x00, 0x00]).with_error(
                ErrorKind::NoAcknowledge(embedded_hal::i2c::NoAcknowledgeSource::Address),
            ),
            // 0x42: floating bus model 0xFFFF -> skipped.
            I2cTx::write(0x42, std::vec![0x00, 0x00]),
            I2cTx::read(0x42, std::vec![0xFF, 0xFF]),
        ];
        let mut bus = I2cMock::new(&expectations);
        {
            let mut scanner = BusScanner::new(I2cTransport::<_, 18>::new(&mut bus));
            let nodes = scanner.scan_i2c(0x40..=0x42).unwrap();
            assert_eq!(nodes.len(), 1);
            assert_eq!(
                nodes[0],
                NodeDescriptor {
                    address: 0x40,
                    model_number: 0x0503,
                    firmware_version: 7,
                    uid,
                }
            );
        }
        bus.done();
    }

    #[test]
    fn scan_propagates_fingerprint_errors_on_confirmed_node() {
        let expectations = [
            I2cTx::write(0x40, std::vec![0x00, 0x00]),
            I2cTx::read(0x40, std::vec![0x03, 0x05]),
            I2cTx::write(0x40, std::vec![0x00, 0x02]).with_error(ErrorKind::Other),
        ];
        let mut bus = I2cMock::new(&expectations);
        {
            let mut scanner = BusScanner::new(I2cTransport::<_, 18>::new(&mut bus));
            assert_eq!(
                scanner.scan_i2c(0x40..=0x40),
                Err(TE::Bus(ErrorKind::Other))
            );
        }
        bus.done();
    }

    #[test]
    fn standard_i2c_range_constants_and_capacity() {
        assert_eq!(I2C_SCAN_START, 0x08);
        assert_eq!(I2C_SCAN_END, 0x77);
        assert_eq!(MAX_NODES, 16);

        // A transport that acknowledges every address fills the bounded inventory.
        struct AlwaysThere;
        impl Transport for AlwaysThere {
            type Error = ();
            fn write(&mut self, _n: u8, _r: u16, _b: &[u8]) -> Result<(), TE<()>> {
                Ok(())
            }
            fn read(&mut self, _n: u8, r: u16, out: &mut [u8]) -> Result<(), TE<()>> {
                for byte in out.iter_mut() {
                    *byte = match r {
                        REG_MODEL_NUMBER => 0x03,
                        REG_FIRMWARE_VERSION => 0x01,
                        _ => 0xAB,
                    };
                }
                Ok(())
            }
        }
        let mut scanner = BusScanner::new(AlwaysThere);
        let nodes = scanner.scan_all_i2c().unwrap();
        assert_eq!(nodes.len(), MAX_NODES);
        assert_eq!(nodes[0].address, 0x08);
        assert_eq!(nodes[MAX_NODES - 1].address, 0x08 + (MAX_NODES - 1) as u8);
        assert_eq!(nodes[0].model_number, 0x0303);
    }
}
