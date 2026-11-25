//! `embedded_hal::i2c::I2c` adapter over the RP2350 PIO I3C driver boundary.
//!
//! DAA/CCC and IBI remain in the external PIO driver. This module only
//! translates the two-byte register-pointer / little-endian payload idiom used
//! by [`barefoc_master::I2cTransport`] into PIO transactions.
//!
//! `I2cTransport::read` performs **two separate HAL calls**: a pointer-latch
//! write followed by a burst read. [`PioI3cBus`] therefore remembers the last
//! latched pointer per node and includes it as the TX phase of the following
//! read transaction. Without this state the burst read would sample the wrong
//! register.

use embedded_hal::i2c::{ErrorKind, ErrorType, I2c, NoAcknowledgeSource, Operation};

/// Errors reported by the external PIO I3C transaction boundary.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PioError {
    /// Electrical bus error.
    Bus,
    /// Arbitration lost on the multidrop bus.
    ArbitrationLoss,
    /// The servo node did not acknowledge.
    NoAcknowledge,
    /// The transaction exceeded the driver's deadline.
    Timeout,
    /// Any other driver-specific failure.
    Other,
}

impl From<PioError> for ErrorKind {
    fn from(error: PioError) -> Self {
        match error {
            PioError::Bus => Self::Bus,
            PioError::ArbitrationLoss => Self::ArbitrationLoss,
            PioError::NoAcknowledge => Self::NoAcknowledge(NoAcknowledgeSource::Unknown),
            // `ErrorKind` has no timeout variant in embedded-hal 1.0.
            PioError::Timeout | PioError::Other => Self::Other,
        }
    }
}

/// Minimal boundary implemented by the external PIO I3C driver.
///
/// `tx` is the register pointer (and, on writes, the little-endian payload);
/// `rx` is the burst reply for reads. Exactly one half-duplex transaction is
/// performed per call.
pub trait PioI3c {
    fn transaction(
        &mut self,
        address: u8,
        tx: &[u8],
        rx: &mut [u8],
    ) -> Result<(), PioError>;
}

/// Allow a plain closure to act as the PIO boundary (useful in tests and in
/// firmware wiring code that owns a driver handle).
impl<F> PioI3c for F
where
    F: for<'a, 'b> FnMut(u8, &'a [u8], &'b mut [u8]) -> Result<(), PioError>,
{
    fn transaction(
        &mut self,
        address: u8,
        tx: &[u8],
        rx: &mut [u8],
    ) -> Result<(), PioError> {
        self(address, tx, rx)
    }
}

/// Adapts a [`PioI3c`] driver to `embedded_hal::i2c::I2c`.
#[derive(Clone, Debug)]
pub struct PioI3cBus<P> {
    pio: P,
    latched: Option<(u8, [u8; 2])>,
}

impl<P> PioI3cBus<P> {
    pub const fn new(pio: P) -> Self {
        Self {
            pio,
            latched: None,
        }
    }

    pub fn bus(&self) -> &P {
        &self.pio
    }

    pub fn bus_mut(&mut self) -> &mut P {
        &mut self.pio
    }

    pub fn into_inner(self) -> P {
        self.pio
    }

    /// Currently latched `(node, pointer)` pair, if any.
    pub const fn latched_pointer(&self) -> Option<(u8, [u8; 2])> {
        self.latched
    }
}

impl<P> ErrorType for PioI3cBus<P> {
    type Error = ErrorKind;
}

impl<P: PioI3c> I2c for PioI3cBus<P> {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        for operation in operations {
            match operation {
                Operation::Write(bytes) => self.write(address, bytes)?,
                Operation::Read(buffer) => self.read(address, buffer)?,
            }
        }
        Ok(())
    }

    fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        // Every BareFOC access starts with the two-byte big-endian pointer.
        if bytes.len() < 2 {
            return Err(ErrorKind::Other);
        }
        self.latched = Some((address, [bytes[0], bytes[1]]));
        let mut no_reply = [];
        self.pio
            .transaction(address, bytes, &mut no_reply)
            .map_err(ErrorKind::from)
    }

    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        let pointer = match self.latched {
            Some((node, pointer)) if node == address => pointer,
            // A burst read without a preceding pointer latch cannot address a
            // register; surface it as "other" so the caller can retry.
            _ => return Err(ErrorKind::Other),
        };
        self.pio
            .transaction(address, &pointer, buffer)
            .map_err(ErrorKind::from)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{ControlTableClient, I2cTransport};

    const NODE: u8 = 0x24;
    const REG: u16 = 0x144;

    struct FakePio {
        regs: [u8; 512],
        calls: usize,
        last_address: u8,
        last_tx: std::vec::Vec<u8>,
        fail: Option<PioError>,
    }

    impl Default for FakePio {
        fn default() -> Self {
            Self {
                regs: [0; 512],
                calls: 0,
                last_address: 0,
                last_tx: std::vec::Vec::new(),
                fail: None,
            }
        }
    }

    impl PioI3c for FakePio {
        fn transaction(
            &mut self,
            address: u8,
            tx: &[u8],
            rx: &mut [u8],
        ) -> Result<(), PioError> {
            if let Some(error) = self.fail {
                return Err(error);
            }
            self.calls += 1;
            self.last_address = address;
            self.last_tx.clear();
            self.last_tx.extend_from_slice(tx);
            if tx.len() < 2 {
                return Err(PioError::Timeout);
            }
            let reg = u16::from_be_bytes([tx[0], tx[1]]) as usize;
            if reg >= self.regs.len() {
                return Err(PioError::Other);
            }
            if tx.len() > 2 {
                let end = reg + (tx.len() - 2);
                if end > self.regs.len() {
                    return Err(PioError::Other);
                }
                self.regs[reg..end].copy_from_slice(&tx[2..]);
            } else {
                let end = (reg + rx.len()).min(self.regs.len());
                rx[..end - reg].copy_from_slice(&self.regs[reg..end]);
                if end - reg < rx.len() {
                    return Err(PioError::Other);
                }
            }
            Ok(())
        }
    }

    #[test]
    fn pio_errors_map_to_embedded_hal_error_kinds() {
        assert_eq!(ErrorKind::from(PioError::Bus), ErrorKind::Bus);
        assert_eq!(ErrorKind::from(PioError::ArbitrationLoss), ErrorKind::ArbitrationLoss);
        assert_eq!(
            ErrorKind::from(PioError::NoAcknowledge),
            ErrorKind::NoAcknowledge(NoAcknowledgeSource::Unknown)
        );
        assert_eq!(ErrorKind::from(PioError::Timeout), ErrorKind::Other);
    }

    #[test]
    fn closure_boundary_is_supported() {
        let mut bus = PioI3cBus::new(
            |_address: u8, _tx: &[u8], _rx: &mut [u8]| -> Result<(), PioError> { Ok(()) },
        );
        bus.write(NODE, &[0x01, 0x44, 1, 2, 3, 4]).unwrap();
        assert_eq!(bus.latched_pointer(), Some((NODE, [0x01, 0x44])));
    }

    #[test]
    fn pointer_latch_survives_pointer_write_then_burst_read() {
        let mut bus = PioI3cBus::new(FakePio::default());
        {
            let mut client = ControlTableClient::new(NODE, I2cTransport::<_, 18>::new(&mut bus));
            client.write_f32(REG, 1.25_f32).unwrap();
            assert_eq!(client.read_f32(REG).unwrap(), 1.25_f32);
        }
        assert_eq!(bus.bus().calls, 3, "write + pointer latch + burst");
        assert_eq!(bus.bus().last_address, NODE);
        assert_eq!(bus.bus().last_tx, std::vec![0x01, 0x44]);
        assert_eq!(bus.latched_pointer(), Some((NODE, [0x01, 0x44])));
    }

    #[test]
    fn raw_i2c_transaction_preserves_pointer_across_calls() {
        let mut bus = PioI3cBus::new(FakePio::default());
        I2c::write(&mut bus, NODE, &[0x01, 0x48, 9]).unwrap();
        let mut reply = [0u8; 1];
        I2c::read(&mut bus, NODE, &mut reply).unwrap();
        assert_eq!(reply, [9]);
        assert_eq!(bus.latched_pointer(), Some((NODE, [0x01, 0x48])));

        // A read on a different node must not reuse another node's latch.
        let mut other = [0u8; 1];
        assert_eq!(I2c::read(&mut bus, NODE + 1, &mut other), Err(ErrorKind::Other));
    }

    #[test]
    fn i2c_transaction_supports_pointer_write_then_read_in_one_call() {
        let mut bus = PioI3cBus::new(FakePio::default());
        let mut value = 0x1234_u16.to_le_bytes();
        let mut operations = [
            Operation::Write(&[0x01, 0x4C, 0x34, 0x12][..]),
            Operation::Read(&mut value[..]),
        ];
        bus.transaction(NODE, &mut operations).unwrap();
        assert_eq!(value, 0x1234_u16.to_le_bytes());
    }

    #[test]
    fn pio_failures_surface_as_transport_bus_errors() {
        let mut bus = PioI3cBus::new(FakePio {
            fail: Some(PioError::NoAcknowledge),
            ..FakePio::default()
        });
        let result = {
            let mut client = ControlTableClient::new(NODE, I2cTransport::<_, 18>::new(&mut bus));
            client.read_f32(REG)
        };
        assert_eq!(
            result,
            Err(crate::ClientError::Transport(
                crate::TransportError::Bus(ErrorKind::NoAcknowledge(
                    NoAcknowledgeSource::Unknown
                ))
            ))
        );
    }
}
