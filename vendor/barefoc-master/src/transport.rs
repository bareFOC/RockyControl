//! Byte transports for the host-side BareFOC master SDK.
//!
//! Two generic adapters are provided:
//!
//! - [`I2cTransport`] over [`embedded_hal::i2c::I2c`]. The 16-bit register
//!   pointer is transmitted as **two big-endian bytes** (`[reg_hi, reg_lo]`)
//!   before the payload; payload bytes themselves use the control table's
//!   little-endian encoding. Reads latch the pointer with a write transaction
//!   and then burst the reply bytes back (the same register-pointer idiom as
//!   `barefoc-comms/barefoc-multibus/src/client.rs`).
//! - [`StreamTransport`] over [`embedded_io::Read`] + [`embedded_io::Write`].
//!   It uses a small framed packet:
//!
//!   ```text
//!   [0xA5][node_id][reg_hi][reg_lo][len][payload ...][checksum]
//!     ^ sync                                ^ XOR of all preceding bytes
//!   ```
//!
//!   A read request has `len = 0` and is answered by a response frame carrying
//!   the requested bytes; a write is answered by a zero-length ACK frame.
//!
//! Upstream: `barefoc-comms/barefoc-multibus/src/client.rs` (register-pointer
//! client), `barefoc-comms/barefoc-multibus/src/servo_protocol.rs` (packet
//! framing).

use core::fmt;

use embedded_hal::i2c::I2c;
use embedded_io::{ErrorType as IoErrorType, Read as IoRead, Write as IoWrite};

/// Default I2C staging frame size: a 2-byte big-endian pointer plus a 16-byte
/// payload (enough for a 12-byte silicon UID write).
///
/// Upstream: `bareFOC/src/control_table.rs` (`REG_HARDWARE_UID`, 12 bytes).
pub const MAX_I2C_WRITE_FRAME: usize = 18;
/// Maximum framed payload carried by [`StreamTransport`]. Sized to hold a
/// fully packed `INST_SYNC_WRITE` frame for [`crate::MAX_NODES`] nodes.
pub const MAX_STREAM_PAYLOAD: usize = 128;

const STREAM_SYNC: u8 = 0xA5;
const STREAM_HEADER_LEN: usize = 5; // sync + node + reg_hi + reg_lo + len
const STREAM_OVERHEAD: usize = STREAM_HEADER_LEN + 1; // header + checksum

/// Errors surfaced by a [`Transport`] transaction.
///
/// Upstream: `barefoc-comms/barefoc-multibus/src/client.rs` (`ClientError`),
/// `bareFOC/src/control_table.rs` (bounds validation).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TransportError<E> {
    /// Bus-level error reported by the underlying HAL/IO implementation.
    Bus(E),
    /// The peer did not answer within the transport's timeout window.
    Timeout,
    /// The peer did not acknowledge the transaction.
    NoAcknowledge,
    /// The internal staging frame or supplied output buffer is too small.
    BufferTooSmall,
    /// The node address or register address is not representable on the bus.
    InvalidAddress(u16),
    /// A malformed or unsupported protocol frame was encountered.
    Protocol(&'static str),
}

impl<E> fmt::Display for TransportError<E> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Bus(_) => f.write_str("bus error"),
            Self::Timeout => f.write_str("bus timeout"),
            Self::NoAcknowledge => f.write_str("no acknowledge from node"),
            Self::BufferTooSmall => f.write_str("transport buffer too small"),
            Self::InvalidAddress(addr) => write!(f, "invalid bus address: 0x{addr:04X}"),
            Self::Protocol(msg) => write!(f, "protocol error: {msg}"),
        }
    }
}

/// Combined error type for asymmetric stream transports whose reader and
/// writer report distinct error types.
///
/// Upstream: `barefoc-comms/barefoc-multibus/src/client.rs` (`ClientError::Bus`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StreamError<R, W> {
    /// Error from the receive side.
    Read(R),
    /// Error from the transmit side.
    Write(W),
}

impl<R: fmt::Display, W: fmt::Display> fmt::Display for StreamError<R, W> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Read(e) => write!(f, "stream read error: {e}"),
            Self::Write(e) => write!(f, "stream write error: {e}"),
        }
    }
}

/// Generic register-level byte transport used by [`crate::ControlTableClient`].
///
/// Upstream: `barefoc-comms/barefoc-multibus/src/client.rs` (`RegClientI2c`).
pub trait Transport {
    /// Error type of the underlying bus/IO implementation.
    type Error;

    /// Write `bytes` to `reg` on node `node`.
    fn write(&mut self, node: u8, reg: u16, bytes: &[u8]) -> Result<(), TransportError<Self::Error>>;

    /// Read exactly `out.len()` bytes starting at `reg` on node `node`.
    fn read(&mut self, node: u8, reg: u16, out: &mut [u8]) -> Result<(), TransportError<Self::Error>>;

    /// Send a prebuilt raw servo-protocol packet (e.g. `INST_SYNC_WRITE`).
    ///
    /// Adapters that cannot carry raw frames return
    /// [`TransportError::Protocol`].
    fn send_packet(&mut self, packet: &[u8]) -> Result<(), TransportError<Self::Error>> {
        let _ = packet;
        Err(TransportError::Protocol("raw packets not supported"))
    }
}

impl<T: Transport + ?Sized> Transport for &mut T {
    type Error = T::Error;

    fn write(&mut self, node: u8, reg: u16, bytes: &[u8]) -> Result<(), TransportError<Self::Error>> {
        T::write(&mut **self, node, reg, bytes)
    }

    fn read(&mut self, node: u8, reg: u16, out: &mut [u8]) -> Result<(), TransportError<Self::Error>> {
        T::read(&mut **self, node, reg, out)
    }

    fn send_packet(&mut self, packet: &[u8]) -> Result<(), TransportError<Self::Error>> {
        T::send_packet(&mut **self, packet)
    }
}

/// I2C adapter using a two-byte **big-endian** register pointer.
///
/// Writes stage `[reg_hi, reg_lo, payload...]` in a fixed-size buffer of
/// `MAX_FRAME` bytes. Reads first latch the pointer with a zero-payload write
/// and then burst the little-endian reply into the caller's buffer.
///
/// Upstream: `barefoc-comms/barefoc-multibus/src/client.rs` (`RegClientI2c`).
pub struct I2cTransport<I, const MAX_FRAME: usize = MAX_I2C_WRITE_FRAME> {
    bus: I,
}

impl<I, const MAX_FRAME: usize> I2cTransport<I, MAX_FRAME> {
    /// Wrap an I2C master.
    pub const fn new(bus: I) -> Self {
        Self { bus }
    }

    /// Borrow the underlying bus.
    pub fn bus(&self) -> &I {
        &self.bus
    }

    /// Mutably borrow the underlying bus.
    pub fn bus_mut(&mut self) -> &mut I {
        &mut self.bus
    }

    /// Consume the adapter and return the underlying bus.
    pub fn into_inner(self) -> I {
        self.bus
    }
}

impl<I: I2c, const MAX_FRAME: usize> Transport for I2cTransport<I, MAX_FRAME> {
    type Error = I::Error;

    fn write(&mut self, node: u8, reg: u16, bytes: &[u8]) -> Result<(), TransportError<Self::Error>> {
        if node > 0xFE {
            return Err(TransportError::InvalidAddress(node as u16));
        }
        if 2 + bytes.len() > MAX_FRAME {
            return Err(TransportError::BufferTooSmall);
        }
        let mut frame = [0u8; MAX_FRAME];
        frame[0] = (reg >> 8) as u8;
        frame[1] = reg as u8;
        frame[2..2 + bytes.len()].copy_from_slice(bytes);
        self.bus
            .write(node, &frame[..2 + bytes.len()])
            .map_err(TransportError::Bus)
    }

    fn read(&mut self, node: u8, reg: u16, out: &mut [u8]) -> Result<(), TransportError<Self::Error>> {
        if node > 0xFE {
            return Err(TransportError::InvalidAddress(node as u16));
        }
        let pointer = [(reg >> 8) as u8, reg as u8];
        self.bus.write(node, &pointer).map_err(TransportError::Bus)?;
        self.bus.read(node, out).map_err(TransportError::Bus)
    }

    fn send_packet(&mut self, packet: &[u8]) -> Result<(), TransportError<Self::Error>> {
        self.bus
            .write(crate::BROADCAST_ID, packet)
            .map_err(TransportError::Bus)
    }
}

/// Frame checksum: XOR of every byte preceding the checksum byte.
fn stream_checksum(bytes: &[u8]) -> u8 {
    let mut checksum = 0u8;
    for &byte in bytes {
        checksum ^= byte;
    }
    checksum
}

/// Build a framed stream request/response packet into `out`.
fn build_stream_frame(node: u8, reg: u16, payload: &[u8], out: &mut [u8]) -> usize {
    out[0] = STREAM_SYNC;
    out[1] = node;
    out[2] = (reg >> 8) as u8;
    out[3] = reg as u8;
    out[4] = payload.len() as u8;
    out[STREAM_HEADER_LEN..STREAM_HEADER_LEN + payload.len()].copy_from_slice(payload);
    let end = STREAM_HEADER_LEN + payload.len();
    out[end] = stream_checksum(&out[..end]);
    end + 1
}

/// Serial-stream adapter over `embedded_io` blocking traits.
///
/// Upstream: `barefoc-comms/barefoc-multibus/src/servo_protocol.rs` (framed
/// packet), `barefoc-comms/barefoc-rtt` (`embedded_io::{Read, Write}` streams).
pub struct StreamTransport<R, W> {
    reader: R,
    writer: W,
}

impl<R, W> StreamTransport<R, W> {
    /// Wrap a reader/writer pair.
    pub const fn new(reader: R, writer: W) -> Self {
        Self { reader, writer }
    }

    /// Borrow the reader.
    pub fn reader(&self) -> &R {
        &self.reader
    }

    /// Borrow the writer.
    pub fn writer(&self) -> &W {
        &self.writer
    }

    /// Consume the adapter and return its reader and writer.
    pub fn into_inner(self) -> (R, W) {
        (self.reader, self.writer)
    }
}

impl<R, W> Transport for StreamTransport<R, W>
where
    R: IoRead + IoErrorType,
    W: IoWrite + IoErrorType,
{
    type Error = StreamError<R::Error, W::Error>;

    fn write(&mut self, node: u8, reg: u16, bytes: &[u8]) -> Result<(), TransportError<Self::Error>> {
        if bytes.len() > MAX_STREAM_PAYLOAD {
            return Err(TransportError::BufferTooSmall);
        }
        let mut frame = [0u8; MAX_STREAM_PAYLOAD + STREAM_OVERHEAD];
        let len = build_stream_frame(node, reg, bytes, &mut frame);
        self.writer
            .write_all(&frame[..len])
            .map_err(|e| TransportError::Bus(StreamError::Write(e)))?;

        // Wait for the zero-length ACK frame from the peer.
        let mut ack = [0u8; STREAM_OVERHEAD];
        self.reader
            .read_exact(&mut ack[..STREAM_HEADER_LEN])
            .map_err(map_exact_error)?;
        self.reader
            .read_exact(&mut ack[STREAM_HEADER_LEN..])
            .map_err(map_exact_error)?;
        validate_frame(&ack, node, reg, &[])?;
        Ok(())
    }

    fn read(&mut self, node: u8, reg: u16, out: &mut [u8]) -> Result<(), TransportError<Self::Error>> {
        if out.len() > MAX_STREAM_PAYLOAD {
            return Err(TransportError::BufferTooSmall);
        }
        let mut request = [0u8; STREAM_OVERHEAD];
        let req_len = build_stream_frame(node, reg, &[], &mut request);
        self.writer
            .write_all(&request[..req_len])
            .map_err(|e| TransportError::Bus(StreamError::Write(e)))?;

        let mut header = [0u8; STREAM_HEADER_LEN];
        self.reader
            .read_exact(&mut header)
            .map_err(map_exact_error)?;
        if header[0] != STREAM_SYNC {
            return Err(TransportError::Protocol("invalid stream sync byte"));
        }
        if header[1] != node || ((header[2] as u16) << 8 | header[3] as u16) != reg {
            return Err(TransportError::Protocol("stream response header mismatch"));
        }
        let len = header[4] as usize;
        if len != out.len() || len > MAX_STREAM_PAYLOAD {
            return Err(TransportError::Protocol("stream response length mismatch"));
        }
        self.reader
            .read_exact(&mut out[..len])
            .map_err(map_exact_error)?;
        let mut checksum = [0u8; 1];
        self.reader
            .read_exact(&mut checksum)
            .map_err(map_exact_error)?;
        let mut full = [0u8; MAX_STREAM_PAYLOAD + STREAM_OVERHEAD];
        full[..STREAM_HEADER_LEN].copy_from_slice(&header);
        full[STREAM_HEADER_LEN..STREAM_HEADER_LEN + len].copy_from_slice(&out[..len]);
        if stream_checksum(&full[..STREAM_HEADER_LEN + len]) != checksum[0] {
            return Err(TransportError::Protocol("stream response checksum mismatch"));
        }
        Ok(())
    }

    fn send_packet(&mut self, packet: &[u8]) -> Result<(), TransportError<Self::Error>> {
        self.write(
            crate::BROADCAST_ID,
            crate::REG_TORQUE_ENABLE,
            packet,
        )
    }
}

/// Map `embedded_io::Read::read_exact` failure into the transport error space.
fn map_exact_error<R, W>(
    err: embedded_io::ReadExactError<R>,
) -> TransportError<StreamError<R, W>> {
    match err {
        embedded_io::ReadExactError::Other(e) => TransportError::Bus(StreamError::Read(e)),
        embedded_io::ReadExactError::UnexpectedEof => {
            TransportError::Protocol("unexpected EOF in stream frame")
        }
    }
}

/// Validate an ACK/response frame header and checksum.
fn validate_frame<E>(
    frame: &[u8],
    node: u8,
    reg: u16,
    expected_payload: &[u8],
) -> Result<(), TransportError<E>> {
    if frame.len() < STREAM_OVERHEAD {
        return Err(TransportError::Protocol("short stream frame"));
    }
    if frame[0] != STREAM_SYNC {
        return Err(TransportError::Protocol("invalid stream sync byte"));
    }
    if frame[1] != node || ((frame[2] as u16) << 8 | frame[3] as u16) != reg {
        return Err(TransportError::Protocol("stream response header mismatch"));
    }
    let len = frame[4] as usize;
    if len != expected_payload.len() || frame.len() != STREAM_OVERHEAD + len {
        return Err(TransportError::Protocol("stream response length mismatch"));
    }
    if stream_checksum(&frame[..STREAM_OVERHEAD + len - 1]) != frame[STREAM_OVERHEAD + len - 1] {
        return Err(TransportError::Protocol("stream response checksum mismatch"));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTx};

    const NODE: u8 = 0x40;

    #[test]
    fn i2c_write_uses_big_endian_pointer_and_le_payload() {
        let expectations = [I2cTx::write(
            NODE,
            std::vec![0x01, 0x30, 0x00, 0x00, 0x80, 0x3F],
        )];
        let mut bus = I2cMock::new(&expectations);
        {
            let mut transport = I2cTransport::<_, 18>::new(&mut bus);
            transport
                .write(NODE, 0x130, &1.0f32.to_le_bytes())
                .unwrap();
        }
        bus.done();
    }

    #[test]
    fn i2c_read_latches_pointer_then_bursts_le_bytes() {
        let expectations = [
            I2cTx::write(NODE, std::vec![0x01, 0x44]),
            I2cTx::read(NODE, std::vec![0x00, 0x00, 0x20, 0x40]),
        ];
        let mut bus = I2cMock::new(&expectations);
        {
            let mut transport = I2cTransport::<_, 18>::new(&mut bus);
            let mut out = [0u8; 4];
            transport.read(NODE, 0x144, &mut out).unwrap();
            assert_eq!(f32::from_le_bytes(out), 2.5);
        }
        bus.done();
    }

    #[test]
    fn i2c_write_rejects_oversize_payload_without_bus_traffic() {
        let expectations: [I2cTx; 0] = [];
        let mut bus = I2cMock::new(&expectations);
        {
            let mut transport = I2cTransport::<_, 8>::new(&mut bus);
            assert_eq!(
                transport.write(NODE, 0x130, &[0u8; 8]),
                Err(TransportError::BufferTooSmall)
            );
        }
        bus.done();
    }

    #[test]
    fn i2c_bus_error_is_wrapped() {
        let expectations = [I2cTx::write(NODE, std::vec![0x01, 0x30, 0x05]).with_error(
            embedded_hal::i2c::ErrorKind::NoAcknowledge(
                embedded_hal::i2c::NoAcknowledgeSource::Address,
            ),
        )];
        let mut bus = I2cMock::new(&expectations);
        {
            let mut transport = I2cTransport::<_, 18>::new(&mut bus);
            assert_eq!(
                transport.write(NODE, 0x130, &[5]),
                Err(TransportError::Bus(
                    embedded_hal::i2c::ErrorKind::NoAcknowledge(
                        embedded_hal::i2c::NoAcknowledgeSource::Address
                    )
                ))
            );
        }
        bus.done();
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct TestIoError;

    impl embedded_io::Error for TestIoError {
        fn kind(&self) -> embedded_io::ErrorKind {
            embedded_io::ErrorKind::Other
        }
    }

    struct TestStream {
        tx: [u8; 512],
        tx_len: usize,
        rx: [u8; 512],
        rx_len: usize,
        rx_pos: usize,
    }

    impl TestStream {
        fn new(rx: &[u8]) -> Self {
            let mut s = Self {
                tx: [0; 512],
                tx_len: 0,
                rx: [0; 512],
                rx_len: rx.len(),
                rx_pos: 0,
            };
            s.rx[..rx.len()].copy_from_slice(rx);
            s
        }
    }

    impl IoErrorType for TestStream {
        type Error = TestIoError;
    }

    impl IoWrite for TestStream {
        fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            let n = buf.len().min(self.tx.len() - self.tx_len);
            self.tx[self.tx_len..self.tx_len + n].copy_from_slice(&buf[..n]);
            self.tx_len += n;
            Ok(n)
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    impl IoRead for TestStream {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            let remaining = self.rx_len - self.rx_pos;
            let n = buf.len().min(remaining);
            buf[..n].copy_from_slice(&self.rx[self.rx_pos..self.rx_pos + n]);
            self.rx_pos += n;
            Ok(n)
        }
    }

    fn ack_frame(node: u8, reg: u16) -> [u8; STREAM_OVERHEAD] {
        let mut frame = [0u8; STREAM_OVERHEAD];
        let len = build_stream_frame(node, reg, &[], &mut frame);
        assert_eq!(len, STREAM_OVERHEAD);
        frame
    }

    #[test]
    fn stream_write_sends_framed_packet_and_checks_ack() {
        let ack = ack_frame(NODE, 0x130);
        let expected = {
            let mut f = [0u8; STREAM_OVERHEAD + 4];
            let len = build_stream_frame(NODE, 0x130, &1.5f32.to_le_bytes(), &mut f);
            f[..len].to_vec()
        };
        // Reader carries the ACK; writer records the outgoing request frame.
        let mut transport = StreamTransport::new(TestStream::new(&ack), TestStream::new(&[]));
        transport.write(NODE, 0x130, &1.5f32.to_le_bytes()).unwrap();
        assert_eq!(transport.writer().tx[..transport.writer().tx_len], expected[..]);
    }

    #[test]
    fn stream_read_sends_request_and_decodes_response() {
        let mut response = [0u8; STREAM_OVERHEAD + 4];
        let len = build_stream_frame(NODE, 0x144, &2.5f32.to_le_bytes(), &mut response);
        let mut transport = StreamTransport::new(TestStream::new(&response[..len]), TestStream::new(&[]));
        let mut out = [0u8; 4];
        transport.read(NODE, 0x144, &mut out).unwrap();
        assert_eq!(f32::from_le_bytes(out), 2.5);
        let request = {
            let mut f = [0u8; STREAM_OVERHEAD];
            let n = build_stream_frame(NODE, 0x144, &[], &mut f);
            f[..n].to_vec()
        };
        assert_eq!(transport.writer().tx[..transport.writer().tx_len], request[..]);
    }

    #[test]
    fn stream_rejects_oversize_payload() {
        let mut transport = StreamTransport::new(TestStream::new(&[]), TestStream::new(&[]));
        assert_eq!(
            transport.write(NODE, 0x130, &[0u8; MAX_STREAM_PAYLOAD + 1]),
            Err(TransportError::BufferTooSmall)
        );
    }

    #[test]
    fn stream_rejects_corrupt_response() {
        let mut response = [0u8; STREAM_OVERHEAD + 4];
        let len = build_stream_frame(NODE, 0x144, &2.5f32.to_le_bytes(), &mut response);
        response[len - 1] ^= 0xFF; // corrupt checksum
        let mut transport = StreamTransport::new(TestStream::new(&response[..len]), TestStream::new(&[]));
        let mut out = [0u8; 4];
        assert_eq!(
            transport.read(NODE, 0x144, &mut out),
            Err(TransportError::Protocol("stream response checksum mismatch"))
        );
    }

    #[test]
    fn transport_error_display_is_no_std() {
        let e: TransportError<()> = TransportError::InvalidAddress(0x0180);
        assert_eq!(std::format!("{e}"), "invalid bus address: 0x0180");
        assert_eq!(
            std::format!("{}", TransportError::<()>::Timeout),
            "bus timeout"
        );
    }
}
