//! Typed control-table register client.
//!
//! [`ControlTableClient`] is generic over [`Transport`] and exposes typed
//! little-endian accessors for the canonical 512-byte BareFOC control table.
//! Every bulk access is validated against [`CONTROL_TABLE_SIZE`].
//!
//! Upstream: `bareFOC/src/control_table.rs` (`ControlTable::read_u8`,
//! `write_u16`, `read_f32`, ...), `barefoc-comms/barefoc-multibus/src/client.rs`.

use core::fmt;

use crate::registers::CONTROL_TABLE_SIZE;
use crate::transport::{Transport, TransportError};

/// A bulk register access would leave the 512-byte control table.
///
/// Upstream: `bareFOC/src/control_table.rs` (`CONTROL_TABLE_SIZE` bounds guards).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoundsError {
    /// `address + length` exceeds the control table.
    OutOfBounds {
        /// First register address accessed.
        address: u16,
        /// Requested byte count.
        length: usize,
    },
}

impl fmt::Display for BoundsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::OutOfBounds { address, length } => {
                write!(f, "access at 0x{address:04X} for {length} bytes is out of bounds")
            }
        }
    }
}

/// Errors returned by [`ControlTableClient`] operations.
///
/// Upstream: `barefoc-comms/barefoc-multibus/src/client.rs` (`ClientError`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ClientError<E> {
    /// Underlying transport error.
    Transport(TransportError<E>),
    /// Register access outside the 512-byte control table.
    Bounds(BoundsError),
}

impl<E> From<TransportError<E>> for ClientError<E> {
    fn from(err: TransportError<E>) -> Self {
        Self::Transport(err)
    }
}

impl<E> From<BoundsError> for ClientError<E> {
    fn from(err: BoundsError) -> Self {
        Self::Bounds(err)
    }
}

impl<E> fmt::Display for ClientError<E> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Transport(e) => write!(f, "transport error: {e}"),
            Self::Bounds(e) => write!(f, "bounds error: {e}"),
        }
    }
}

/// Strongly typed client for the standardized BareFOC control table.
///
/// All multi-byte values use little-endian encoding, matching the target
/// firmware's `ControlTable::read_u16` / `read_f32` helpers.
///
/// Upstream: `bareFOC/src/control_table.rs` (`ControlTable`).
pub struct ControlTableClient<T> {
    node: u8,
    transport: T,
}

impl<T> ControlTableClient<T> {
    /// Bind a client to a node address on `transport`.
    pub const fn new(node: u8, transport: T) -> Self {
        Self { node, transport }
    }

    /// Node address this client is bound to.
    pub const fn node(&self) -> u8 {
        self.node
    }

    /// Borrow the transport.
    pub fn transport(&self) -> &T {
        &self.transport
    }

    /// Mutably borrow the transport.
    pub fn transport_mut(&mut self) -> &mut T {
        &mut self.transport
    }

    /// Consume the client and return the transport.
    pub fn into_inner(self) -> T {
        self.transport
    }

    /// Validate that `address..address + length` lies inside the control table.
    fn check_bounds(address: u16, length: usize) -> Result<(), BoundsError> {
        match (address as usize).checked_add(length) {
            Some(end) if end <= CONTROL_TABLE_SIZE => Ok(()),
            _ => Err(BoundsError::OutOfBounds {
                address,
                length,
            }),
        }
    }
}

impl<T: Transport> ControlTableClient<T> {
    /// Read a single unsigned byte register.
    pub fn read_u8(&mut self, reg: u16) -> Result<u8, ClientError<T::Error>> {
        let mut buf = [0u8; 1];
        self.read_bytes(reg, 1, &mut buf)?;
        Ok(buf[0])
    }

    /// Write a single unsigned byte register.
    pub fn write_u8(&mut self, reg: u16, value: u8) -> Result<(), ClientError<T::Error>> {
        self.write_bytes(reg, &[value])
    }

    /// Read a little-endian 16-bit register.
    pub fn read_u16(&mut self, reg: u16) -> Result<u16, ClientError<T::Error>> {
        let mut buf = [0u8; 2];
        self.read_bytes(reg, 2, &mut buf)?;
        Ok(u16::from_le_bytes(buf))
    }

    /// Write a little-endian 16-bit register.
    pub fn write_u16(&mut self, reg: u16, value: u16) -> Result<(), ClientError<T::Error>> {
        self.write_bytes(reg, &value.to_le_bytes())
    }

    /// Read a little-endian 32-bit unsigned register (e.g. capability bitmask).
    pub fn read_u32(&mut self, reg: u16) -> Result<u32, ClientError<T::Error>> {
        let mut buf = [0u8; 4];
        self.read_bytes(reg, 4, &mut buf)?;
        Ok(u32::from_le_bytes(buf))
    }

    /// Write a little-endian 32-bit unsigned register.
    pub fn write_u32(&mut self, reg: u16, value: u32) -> Result<(), ClientError<T::Error>> {
        self.write_bytes(reg, &value.to_le_bytes())
    }

    /// Read a little-endian 64-bit float register (odometer channel).
    pub fn read_f64(&mut self, reg: u16) -> Result<f64, ClientError<T::Error>> {
        let mut buf = [0u8; 8];
        self.read_bytes(reg, 8, &mut buf)?;
        Ok(f64::from_le_bytes(buf))
    }

    /// Write a little-endian 64-bit float register.
    pub fn write_f64(&mut self, reg: u16, value: f64) -> Result<(), ClientError<T::Error>> {
        self.write_bytes(reg, &value.to_le_bytes())
    }

    /// Read a little-endian 32-bit float register.
    pub fn read_f32(&mut self, reg: u16) -> Result<f32, ClientError<T::Error>> {
        let mut buf = [0u8; 4];
        self.read_bytes(reg, 4, &mut buf)?;
        Ok(f32::from_le_bytes(buf))
    }

    /// Write a little-endian 32-bit float register.
    pub fn write_f32(&mut self, reg: u16, value: f32) -> Result<(), ClientError<T::Error>> {
        self.write_bytes(reg, &value.to_le_bytes())
    }

    /// Read `length` raw bytes from `reg` into `out`.
    ///
    /// Returns [`BoundsError`] if the access would leave the control table, or
    /// [`TransportError::BufferTooSmall`] if `out` is shorter than `length`.
    pub fn read_bytes(
        &mut self,
        reg: u16,
        length: usize,
        out: &mut [u8],
    ) -> Result<(), ClientError<T::Error>> {
        Self::check_bounds(reg, length)?;
        if out.len() < length {
            return Err(TransportError::BufferTooSmall.into());
        }
        self.transport.read(self.node, reg, &mut out[..length])?;
        Ok(())
    }

    /// Write `bytes` starting at `reg`.
    ///
    /// Returns [`BoundsError`] if the access would leave the control table.
    pub fn write_bytes(&mut self, reg: u16, bytes: &[u8]) -> Result<(), ClientError<T::Error>> {
        Self::check_bounds(reg, bytes.len())?;
        self.transport.write(self.node, reg, bytes)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::registers::{
        REG_FIRMWARE_VERSION, REG_HARDWARE_UID, REG_HARDWARE_CAPABILITIES, REG_MODEL_NUMBER,
        REG_PRESENT_BUS_POWER, REG_PRESENT_POSITION,
    };
    use crate::transport::I2cTransport;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTx};

    const NODE: u8 = 0x40;

    #[test]
    fn typed_reads_decode_le_values() {
        let expectations = [
            I2cTx::write(NODE, std::vec![0x00, 0x00]),
            I2cTx::read(NODE, std::vec![0x03, 0x05]),
            I2cTx::write(NODE, std::vec![0x00, 0x02]),
            I2cTx::read(NODE, std::vec![0x01]),
            I2cTx::write(NODE, std::vec![0x01, 0x44]),
            I2cTx::read(NODE, std::vec![0x00, 0x00, 0x20, 0x40]),
        ];
        let mut bus = I2cMock::new(&expectations);
        {
            let mut client = ControlTableClient::new(NODE, I2cTransport::<_, 18>::new(&mut bus));
            assert_eq!(client.read_u16(REG_MODEL_NUMBER).unwrap(), 0x0503);
            assert_eq!(client.read_u8(REG_FIRMWARE_VERSION).unwrap(), 1);
            assert_eq!(client.read_f32(REG_PRESENT_POSITION).unwrap(), 2.5);
        }
        bus.done();
    }

    #[test]
    fn typed_writes_emit_le_bytes() {
        let expectations = [
            I2cTx::write(NODE, std::vec![0x00, 0x02, 0x07]),
            I2cTx::write(NODE, std::vec![0x01, 0x30, 0x78, 0x56]),
            I2cTx::write(NODE, std::vec![0x01, 0x3C, 0x00, 0x00, 0x80, 0x3F]),
        ];
        let mut bus = I2cMock::new(&expectations);
        {
            let mut client = ControlTableClient::new(NODE, I2cTransport::<_, 18>::new(&mut bus));
            client.write_u8(REG_FIRMWARE_VERSION, 7).unwrap();
            client.write_u16(0x130, 0x5678).unwrap();
            client.write_f32(0x13C, 1.0).unwrap();
        }
        bus.done();
    }

    #[test]
    fn bulk_access_valid_and_out_of_bounds() {
        let expectations = [
            I2cTx::write(NODE, std::vec![0x00, 0x04]),
            I2cTx::read(NODE, std::vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]),
        ];
        let mut bus = I2cMock::new(&expectations);
        {
            let mut client = ControlTableClient::new(NODE, I2cTransport::<_, 18>::new(&mut bus));
            let mut uid = [0u8; 12];
            client.read_bytes(REG_HARDWARE_UID, 12, &mut uid).unwrap();
            assert_eq!(uid, [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]);

            assert_eq!(
                client.write_bytes(0x1FC, &[0u8; 8]),
                Err(ClientError::Bounds(BoundsError::OutOfBounds {
                    address: 0x1FC,
                    length: 8,
                }))
            );
            assert_eq!(
                client.read_bytes(0x200, 1, &mut uid),
                Err(ClientError::Bounds(BoundsError::OutOfBounds {
                    address: 0x200,
                    length: 1,
                }))
            );
            assert_eq!(
                client.write_bytes(0x201, &[]),
                Err(ClientError::Bounds(BoundsError::OutOfBounds {
                    address: 0x201,
                    length: 0,
                }))
            );
        }
        bus.done();
    }

    #[test]
    fn reads_extended_registers_at_final_offsets() {
        // REG_PRESENT_BUS_POWER = 0x184 -> big-endian pointer [0x01, 0x84].
        // REG_HARDWARE_CAPABILITIES = 0x1F8 -> pointer [0x01, 0xF8].
        let expectations = [
            I2cTx::write(NODE, std::vec![0x01, 0x84]),
            I2cTx::read(NODE, std::vec![0x00, 0x00, 0xF6, 0x42]), // 123.0f32
            I2cTx::write(NODE, std::vec![0x01, 0xF8]),
            I2cTx::read(NODE, std::vec![0x42, 0x00, 0x00, 0x00]), // capability bits 1 & 6
        ];
        let mut bus = I2cMock::new(&expectations);
        {
            let mut client = ControlTableClient::new(NODE, I2cTransport::<_, 18>::new(&mut bus));
            assert_eq!(client.read_f32(REG_PRESENT_BUS_POWER).unwrap(), 123.0);
            assert_eq!(
                client.read_u32(REG_HARDWARE_CAPABILITIES).unwrap(),
                (1 << 1) | (1 << 6)
            );
        }
        bus.done();
    }

    #[test]
    fn read_rejects_small_output_buffer() {
        let expectations: [I2cTx; 0] = [];
        let mut bus = I2cMock::new(&expectations);
        {
            let mut client = ControlTableClient::new(NODE, I2cTransport::<_, 18>::new(&mut bus));
            let mut small = [0u8; 2];
            assert_eq!(
                client.read_bytes(REG_HARDWARE_UID, 12, &mut small),
                Err(ClientError::Transport(TransportError::BufferTooSmall))
            );
        }
        bus.done();
    }

    #[test]
    fn client_error_display_is_no_std() {
        let err: ClientError<()> = BoundsError::OutOfBounds {
            address: 0x180,
            length: 1,
        }
        .into();
        assert!(std::format!("{err}").contains("out of bounds"));
    }
}
