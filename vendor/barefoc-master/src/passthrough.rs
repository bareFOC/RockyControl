//! Versioned PC ↔ flight-controller bus-passthrough protocol (host side).
//!
//! This module adds a symmetric framed protocol that tunnels the existing
//! [`Transport`] operations (`write`, `read`, `send_packet`) through a flight
//! controller to downstream I2C / UART / CAN buses, or to the flight
//! controller's own local control table.
//!
//! # Wire format
//!
//! ```text
//! [0xA5][version][opcode][bus][node][hi][lo][len][payload ...][crc16]
//!    0     1        2      3    4    5   6    7      8..8+len   +2
//! ```
//!
//! - `version` is [`PROTOCOL_VERSION`] (`0x01`).
//! - Request opcodes are the `OP_*` constants; a response echoes
//!   [`response_opcode`]`(opcode)` (MSB set).
//! - In a request `[hi, lo]` is the big-endian 16-bit register pointer. In a
//!   response `hi` is the typed [`StatusCode`] and `lo` is a sub-status /
//!   reserved flag byte.
//! - Payload bytes use the control table's little-endian encoding; the maximum
//!   payload is [`MAX_STREAM_PAYLOAD`] (128 bytes), so the largest frame is
//!   [`MAX_FRAME_LEN`] (138 bytes).
//! - `crc16` is CRC-16/CCITT-FALSE (poly `0x1021`, init `0xFFFF`, no
//!   reflection, final XOR `0x0000`) over bytes `0..8+len`, appended
//!   **little-endian**.
//!
//! The device server lives in the separate `barefoc-passthrough` crate
//! (`barefoc-comms/barefoc-passthrough`); both sides share the codec and the
//! constants defined here.
//!
//! Upstream: `barefoc-master/src/transport.rs` (`Transport`, framed stream),
//! `barefoc-master/src/sync.rs` (CRC helpers, broadcast disarm),
//! `barefoc-comms/barefoc-multibus/src/servo_protocol.rs` (register framing).
//! There is no C++ equivalent: this is a BareFOC-native protocol introduced by
//! the `bus-passthrough` OpenSpec change.

use embedded_io::{ErrorType as IoErrorType, Read as IoRead, Write as IoWrite};

use crate::registers::{BROADCAST_ID, REG_TORQUE_ENABLE};
use crate::transport::{Transport, TransportError, MAX_STREAM_PAYLOAD};

/// Synchronization byte of every passthrough frame.
pub const PT_SYNC: u8 = 0xA5;
/// Current passthrough protocol version.
pub const PROTOCOL_VERSION: u8 = 0x01;
/// Number of header bytes before the payload.
pub const PT_HEADER_LEN: usize = 8;
/// Number of trailing CRC bytes.
pub const PT_CRC_LEN: usize = 2;
/// Fixed framing overhead (header + CRC).
pub const PT_OVERHEAD: usize = PT_HEADER_LEN + PT_CRC_LEN;
/// Largest complete passthrough frame (`8 + 128 + 2`).
pub const MAX_FRAME_LEN: usize = PT_HEADER_LEN + MAX_STREAM_PAYLOAD + PT_CRC_LEN;
/// Maximum payload carried by one passthrough frame.
pub const PT_MAX_PAYLOAD: usize = MAX_STREAM_PAYLOAD;
/// Mask applied to a request opcode to obtain its response opcode.
pub const RESPONSE_FLAG: u8 = 0x80;

/// Request opcode: link/status ping (no bus transaction).
pub const OP_PING: u8 = 0x01;
/// Request opcode: enter safety-gated passthrough mode.
pub const OP_ENTER: u8 = 0x02;
/// Request opcode: exit passthrough mode and disarm all buses.
pub const OP_EXIT: u8 = 0x03;
/// Request opcode: capability / version negotiation.
pub const OP_GET_INFO: u8 = 0x04;
/// Request opcode: register read on the selected bus.
pub const OP_READ: u8 = 0x10;
/// Request opcode: register write on the selected bus.
pub const OP_WRITE: u8 = 0x11;
/// Request opcode: atomic raw packet dispatch (`send_packet`).
pub const OP_SYNC_WRITE: u8 = 0x12;
/// Request opcode: single-frame I2C address discovery.
pub const OP_SCAN_BUS: u8 = 0x20;
/// Request opcode: refresh the device watchdog without a bus transaction.
pub const OP_KEEPALIVE: u8 = 0x30;
/// Request opcode: emergency disarm; leaves passthrough mode.
pub const OP_EMERGENCY_STOP: u8 = 0x31;

/// Response opcode: ping.
pub const RESP_PING: u8 = OP_PING | RESPONSE_FLAG;
/// Response opcode: enter.
pub const RESP_ENTER: u8 = OP_ENTER | RESPONSE_FLAG;
/// Response opcode: exit.
pub const RESP_EXIT: u8 = OP_EXIT | RESPONSE_FLAG;
/// Response opcode: capability response.
pub const RESP_GET_INFO: u8 = OP_GET_INFO | RESPONSE_FLAG;
/// Response opcode: register read reply.
pub const RESP_READ: u8 = OP_READ | RESPONSE_FLAG;
/// Response opcode: register write ACK.
pub const RESP_WRITE: u8 = OP_WRITE | RESPONSE_FLAG;
/// Response opcode: raw packet ACK.
pub const RESP_SYNC_WRITE: u8 = OP_SYNC_WRITE | RESPONSE_FLAG;
/// Response opcode: I2C address bitmask.
pub const RESP_SCAN_BUS: u8 = OP_SCAN_BUS | RESPONSE_FLAG;
/// Response opcode: keepalive ACK.
pub const RESP_KEEPALIVE: u8 = OP_KEEPALIVE | RESPONSE_FLAG;
/// Response opcode: emergency-stop ACK.
pub const RESP_EMERGENCY_STOP: u8 = OP_EMERGENCY_STOP | RESPONSE_FLAG;

/// First I2C bus ID (`I2C0`).
pub const BUS_I2C_BASE: u8 = 0x00;
/// Last I2C bus ID (`I2C15`).
pub const BUS_I2C_LAST: u8 = 0x0F;
/// First UART bus ID (`UART0`).
pub const BUS_UART_BASE: u8 = 0x10;
/// Last UART bus ID (`UART15`).
pub const BUS_UART_LAST: u8 = 0x1F;
/// First CAN bus ID (`CAN0`); reserved in protocol v1.
pub const BUS_CAN_BASE: u8 = 0x20;
/// Last CAN bus ID (`CAN15`); reserved in protocol v1.
pub const BUS_CAN_LAST: u8 = 0x2F;
/// Local flight-controller control-table bus ID.
pub const BUS_LOCAL: u8 = 0xF0;
/// Broadcast-to-all-buses ID (used for disarm and mode entry verification).
pub const BUS_BROADCAST_ALL: u8 = 0xFF;

/// Capability bit for I2C buses in [`DeviceInfo::supported_buses`].
pub const BUS_CAP_I2C: u32 = 1 << 0;
/// Capability bit for UART buses in [`DeviceInfo::supported_buses`].
pub const BUS_CAP_UART: u32 = 1 << 1;
/// Capability bit for CAN buses in [`DeviceInfo::supported_buses`].
pub const BUS_CAP_CAN: u32 = 1 << 2;
/// Capability bit for the local flight-controller bus.
pub const BUS_CAP_LOCAL: u32 = 1 << 3;

/// Local FC register: passthrough mode state (`u8`: `0` normal, `1` active).
pub const LOCAL_REG_MODE_STATE: u16 = 0x00;
/// Local FC register: firmware version (`u32`).
pub const LOCAL_REG_FIRMWARE_VERSION: u16 = 0x02;
/// Local FC register: capability bitmask (`u32`).
pub const LOCAL_REG_CAPABILITIES: u16 = 0x08;
/// Local FC register: error flags (`u32`).
pub const LOCAL_REG_ERROR_FLAGS: u16 = 0x0C;
/// Local FC register: thermal telemetry, degrees Celsius (`f32`).
pub const LOCAL_REG_TEMPERATURE: u16 = 0x10;
/// Local FC register: passthrough watchdog timeout mirror, milliseconds (`u16`).
pub const LOCAL_REG_WATCHDOG_TIMEOUT_MS: u16 = 0x14;

/// Number of bytes in an encoded [`DeviceInfo`] payload.
pub const DEVICE_INFO_LEN: usize = 11;
/// Number of bytes in a 128-address I2C scan bitmask.
pub const SCAN_MASK_LEN: usize = 16;
/// Default passthrough watchdog timeout in milliseconds.
pub const DEFAULT_WATCHDOG_MS: u16 = 500;

/// Typed status code carried in `reg_hi` of a response frame.
///
/// Upstream: `barefoc-comms/barefoc-multibus/src/client.rs` (`ClientError`)
/// and `barefoc-master/src/transport.rs` (`TransportError`), reduced to a
/// single-byte wire code.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum StatusCode {
    /// Transaction succeeded.
    Ok = 0x00,
    /// A downstream node (or the FC) reported torque enabled at mode entry.
    NotDisarmed = 0x01,
    /// Command received while passthrough mode is inactive.
    NotInPassthrough = 0x02,
    /// The bus ID is not routed by the device.
    InvalidBus = 0x03,
    /// Downstream bus did not acknowledge.
    BusNak = 0x04,
    /// Downstream bus transaction timed out.
    BusTimeout = 0x05,
    /// Downstream bus is owned by another task.
    BusBusy = 0x06,
    /// Payload length is zero or exceeds the frame MTU.
    InvalidLength = 0x07,
    /// Frame CRC-16 did not match.
    InvalidCrc = 0x08,
    /// Opcode is unknown or not permitted.
    UnsupportedOpcode = 0x09,
    /// Frame version is not [`PROTOCOL_VERSION`].
    UnsupportedVersion = 0x0A,
    /// Device-side execution or safety fault.
    ExecutionFault = 0x0B,
}

impl StatusCode {
    /// Convert a wire byte into a status code; unknown values are reported as
    /// [`StatusCode::ExecutionFault`].
    pub const fn from_byte(byte: u8) -> Self {
        match byte {
            0x00 => Self::Ok,
            0x01 => Self::NotDisarmed,
            0x02 => Self::NotInPassthrough,
            0x03 => Self::InvalidBus,
            0x04 => Self::BusNak,
            0x05 => Self::BusTimeout,
            0x06 => Self::BusBusy,
            0x07 => Self::InvalidLength,
            0x08 => Self::InvalidCrc,
            0x09 => Self::UnsupportedOpcode,
            0x0A => Self::UnsupportedVersion,
            _ => Self::ExecutionFault,
        }
    }

    /// Wire byte of this status code.
    pub const fn as_byte(self) -> u8 {
        self as u8
    }

    /// Human-readable description, used for [`TransportError::Protocol`].
    pub const fn description(self) -> &'static str {
        match self {
            Self::Ok => "passthrough ok",
            Self::NotDisarmed => "passthrough entry refused: not disarmed",
            Self::NotInPassthrough => "passthrough mode is not active",
            Self::InvalidBus => "passthrough bus is not routed",
            Self::BusNak => "passthrough downstream bus no-acknowledge",
            Self::BusTimeout => "passthrough downstream bus timeout",
            Self::BusBusy => "passthrough downstream bus busy",
            Self::InvalidLength => "passthrough invalid payload length",
            Self::InvalidCrc => "passthrough frame CRC mismatch",
            Self::UnsupportedOpcode => "passthrough unsupported opcode",
            Self::UnsupportedVersion => "passthrough unsupported protocol version",
            Self::ExecutionFault => "passthrough execution fault",
        }
    }
}

/// Partitioned bus identifier classes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BusKind {
    /// I2C bus `0x00..=0x0F`.
    I2c,
    /// UART bus `0x10..=0x1F` (framed register protocol only).
    Uart,
    /// CAN bus `0x20..=0x2F` (reserved in v1).
    Can,
    /// Local flight-controller control table (`0xF0`).
    Local,
    /// All buses (`0xFF`).
    BroadcastAll,
    /// Unassigned / reserved bus ID.
    Invalid,
}

/// Classify a raw bus ID.
pub const fn bus_kind(bus: u8) -> BusKind {
    match bus {
        BUS_I2C_BASE..=BUS_I2C_LAST => BusKind::I2c,
        BUS_UART_BASE..=BUS_UART_LAST => BusKind::Uart,
        BUS_CAN_BASE..=BUS_CAN_LAST => BusKind::Can,
        BUS_LOCAL => BusKind::Local,
        BUS_BROADCAST_ALL => BusKind::BroadcastAll,
        _ => BusKind::Invalid,
    }
}

/// Obtain the response opcode for a request opcode (set the MSB).
pub const fn response_opcode(opcode: u8) -> u8 {
    opcode | RESPONSE_FLAG
}

/// Decoded passthrough frame borrowed from the input buffer.
///
/// Both request and response frames share this layout. Use [`Frame::register`]
/// for request frames and [`Frame::status`] / [`Frame::flags`] for responses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame<'a> {
    /// Protocol version byte.
    pub version: u8,
    /// Request or response opcode.
    pub opcode: u8,
    /// Partitioned bus ID.
    pub bus: u8,
    /// Target node address on that bus.
    pub node: u8,
    /// Request: register pointer high byte; response: [`StatusCode`].
    pub hi: u8,
    /// Request: register pointer low byte; response: sub-status flags.
    pub lo: u8,
    /// Frame payload (little-endian register value or opcode-specific data).
    pub payload: &'a [u8],
}

impl<'a> Frame<'a> {
    /// Build a request frame with a 16-bit register pointer.
    pub const fn request(
        opcode: u8,
        bus: u8,
        node: u8,
        reg: u16,
        payload: &'a [u8],
    ) -> Self {
        Self {
            version: PROTOCOL_VERSION,
            opcode,
            bus,
            node,
            hi: (reg >> 8) as u8,
            lo: reg as u8,
            payload,
        }
    }

    /// Build a response frame carrying a typed status code.
    pub const fn response(
        request_opcode: u8,
        bus: u8,
        node: u8,
        status: StatusCode,
        flags: u8,
        payload: &'a [u8],
    ) -> Self {
        Self {
            version: PROTOCOL_VERSION,
            opcode: response_opcode(request_opcode),
            bus,
            node,
            hi: status.as_byte(),
            lo: flags,
            payload,
        }
    }

    /// Big-endian register pointer of a request frame.
    pub const fn register(&self) -> u16 {
        ((self.hi as u16) << 8) | self.lo as u16
    }

    /// Typed status carried in `hi` of a response frame.
    pub const fn status(&self) -> StatusCode {
        StatusCode::from_byte(self.hi)
    }

    /// Sub-status / reserved flags carried in `lo` of a response frame.
    pub const fn flags(&self) -> u8 {
        self.lo
    }
}

/// Errors produced by the frame codec.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameError {
    /// Fewer than [`PT_OVERHEAD`] bytes were supplied.
    ShortFrame,
    /// Sync byte was not [`PT_SYNC`].
    InvalidSync,
    /// Unsupported version byte.
    UnsupportedVersion(u8),
    /// Payload exceeds [`PT_MAX_PAYLOAD`].
    PayloadTooLarge,
    /// Declared length does not match the supplied byte count.
    LengthMismatch,
    /// Opcode-specific payload has an invalid length.
    InvalidLength,
    /// CRC-16/CCITT-FALSE did not match.
    InvalidCrc,
    /// The caller's output buffer is too small.
    BufferTooSmall,
}

impl FrameError {
    /// Map a codec error to the closest wire status code.
    pub const fn status(self) -> StatusCode {
        match self {
            Self::ShortFrame
            | Self::LengthMismatch
            | Self::InvalidLength
            | Self::BufferTooSmall => StatusCode::InvalidLength,
            Self::InvalidSync | Self::InvalidCrc => StatusCode::InvalidCrc,
            Self::UnsupportedVersion(_) => StatusCode::UnsupportedVersion,
            Self::PayloadTooLarge => StatusCode::InvalidLength,
        }
    }
}

impl core::fmt::Display for FrameError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::ShortFrame => f.write_str("short frame"),
            Self::InvalidSync => f.write_str("invalid sync byte"),
            Self::UnsupportedVersion(v) => write!(f, "unsupported version: {v}"),
            Self::PayloadTooLarge => f.write_str("payload exceeds MTU"),
            Self::LengthMismatch => f.write_str("length mismatch"),
            Self::InvalidLength => f.write_str("invalid payload length"),
            Self::InvalidCrc => f.write_str("CRC mismatch"),
            Self::BufferTooSmall => f.write_str("buffer too small"),
        }
    }
}

/// CRC-16/CCITT-FALSE (poly `0x1021`, init `0xFFFF`).
///
/// Upstream: `barefoc-master/src/sync.rs` (`calc_dxl2_crc`) — the same
/// bitwise accumulator with a different polynomial and initial value.
pub fn crc16_ccitt(bytes: &[u8]) -> u16 {
    let mut crc = 0xFFFFu16;
    for &byte in bytes {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if (crc & 0x8000) != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

/// Serialize a frame into `out`, returning the number of bytes written.
pub fn encode_frame(frame: &Frame<'_>, out: &mut [u8]) -> Result<usize, FrameError> {
    let len = frame.payload.len();
    if len > PT_MAX_PAYLOAD {
        return Err(FrameError::PayloadTooLarge);
    }
    let total = PT_OVERHEAD + len;
    if out.len() < total {
        return Err(FrameError::BufferTooSmall);
    }
    out[0] = PT_SYNC;
    out[1] = PROTOCOL_VERSION;
    out[2] = frame.opcode;
    out[3] = frame.bus;
    out[4] = frame.node;
    out[5] = frame.hi;
    out[6] = frame.lo;
    out[7] = len as u8;
    out[PT_HEADER_LEN..PT_HEADER_LEN + len].copy_from_slice(frame.payload);
    let crc = crc16_ccitt(&out[..PT_HEADER_LEN + len]);
    out[PT_HEADER_LEN + len..total].copy_from_slice(&crc.to_le_bytes());
    Ok(total)
}

/// Decode and validate one complete frame from `bytes`.
pub fn decode_frame(bytes: &[u8]) -> Result<Frame<'_>, FrameError> {
    if bytes.len() < PT_OVERHEAD {
        return Err(FrameError::ShortFrame);
    }
    if bytes[0] != PT_SYNC {
        return Err(FrameError::InvalidSync);
    }
    if bytes[1] != PROTOCOL_VERSION {
        return Err(FrameError::UnsupportedVersion(bytes[1]));
    }
    let len = bytes[7] as usize;
    if len > PT_MAX_PAYLOAD {
        return Err(FrameError::PayloadTooLarge);
    }
    let total = PT_OVERHEAD + len;
    if bytes.len() < total {
        return Err(FrameError::ShortFrame);
    }
    if bytes.len() != total {
        return Err(FrameError::LengthMismatch);
    }
    let expected = u16::from_le_bytes([bytes[PT_HEADER_LEN + len], bytes[PT_HEADER_LEN + len + 1]]);
    if crc16_ccitt(&bytes[..PT_HEADER_LEN + len]) != expected {
        return Err(FrameError::InvalidCrc);
    }
    Ok(Frame {
        version: bytes[1],
        opcode: bytes[2],
        bus: bytes[3],
        node: bytes[4],
        hi: bytes[5],
        lo: bytes[6],
        payload: &bytes[PT_HEADER_LEN..PT_HEADER_LEN + len],
    })
}

/// Capability response payload of [`OP_GET_INFO`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeviceInfo {
    /// Protocol version implemented by the device.
    pub protocol_version: u8,
    /// Bitmask of routed buses ([`BUS_CAP_I2C`], [`BUS_CAP_UART`], ...).
    pub supported_buses: u32,
    /// Maximum payload in one frame (normally [`PT_MAX_PAYLOAD`]).
    pub max_payload: u16,
    /// Flight-controller firmware version (application-defined).
    pub firmware_version: u32,
}

impl DeviceInfo {
    /// Encode this structure into `DEVICE_INFO_LEN` bytes.
    pub fn encode(&self, out: &mut [u8]) -> Result<usize, FrameError> {
        if out.len() < DEVICE_INFO_LEN {
            return Err(FrameError::BufferTooSmall);
        }
        out[0] = self.protocol_version;
        out[1..5].copy_from_slice(&self.supported_buses.to_le_bytes());
        out[5..7].copy_from_slice(&self.max_payload.to_le_bytes());
        out[7..11].copy_from_slice(&self.firmware_version.to_le_bytes());
        Ok(DEVICE_INFO_LEN)
    }

    /// Decode a `GET_INFO` response payload.
    pub fn decode(bytes: &[u8]) -> Result<Self, FrameError> {
        if bytes.len() < DEVICE_INFO_LEN {
            return Err(FrameError::ShortFrame);
        }
        let mut word = [0u8; 4];
        word.copy_from_slice(&bytes[1..5]);
        let supported_buses = u32::from_le_bytes(word);
        let max_payload = u16::from_le_bytes([bytes[5], bytes[6]]);
        word.copy_from_slice(&bytes[7..11]);
        Ok(Self {
            protocol_version: bytes[0],
            supported_buses,
            max_payload,
            firmware_version: u32::from_le_bytes(word),
        })
    }
}

/// Bitmask of 128 responding 7-bit I2C addresses (16 bytes).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScanMask {
    /// Raw bitmask; bit `n` represents I2C address `n`.
    pub bytes: [u8; SCAN_MASK_LEN],
}

impl ScanMask {
    /// Empty scan mask.
    pub const fn new() -> Self {
        Self {
            bytes: [0u8; SCAN_MASK_LEN],
        }
    }

    /// Build a mask from a list of 7-bit addresses.
    pub fn from_addresses(addresses: &[u8]) -> Self {
        let mut mask = Self::new();
        for &address in addresses {
            mask.set(address);
        }
        mask
    }

    /// Mark `address` as responding.
    pub fn set(&mut self, address: u8) {
        if address < 128 {
            self.bytes[(address / 8) as usize] |= 1 << (address % 8);
        }
    }

    /// True when `address` responded.
    pub fn contains(&self, address: u8) -> bool {
        address < 128 && (self.bytes[(address / 8) as usize] & (1 << (address % 8))) != 0
    }

    /// Decode a scan payload; returns [`FrameError::InvalidLength`] for a
    /// payload other than [`SCAN_MASK_LEN`] bytes.
    pub fn decode(bytes: &[u8]) -> Result<Self, FrameError> {
        if bytes.len() != SCAN_MASK_LEN {
            return Err(FrameError::InvalidLength);
        }
        let mut mask = Self::new();
        mask.bytes.copy_from_slice(bytes);
        Ok(mask)
    }
}

impl Default for ScanMask {
    fn default() -> Self {
        Self::new()
    }
}

/// Map a device status code into the host [`TransportError`] space.
///
/// `BusNak` becomes `NoAcknowledge`, `BusTimeout` becomes `Timeout`, and all
/// protocol/safety statuses become `Protocol` with a stable static message.
fn map_status<E>(status: StatusCode) -> Result<(), TransportError<E>> {
    match status {
        StatusCode::Ok => Ok(()),
        StatusCode::BusNak => Err(TransportError::NoAcknowledge),
        StatusCode::BusTimeout => Err(TransportError::Timeout),
        other => Err(TransportError::Protocol(other.description())),
    }
}

/// Host-side adapter implementing [`Transport`] over a framed passthrough
/// link to a flight controller.
///
/// Replacing an `I2cTransport` with this type is the only change needed for
/// [`crate::ControlTableClient`], [`crate::BusScanner`] and
/// [`crate::SyncGroup`] to operate over passthrough.
///
/// Upstream: `barefoc-master/src/transport.rs` (`StreamTransport`); protocol
/// introduced by the `bus-passthrough` OpenSpec change.
pub struct PassthroughTransport<IO> {
    io: IO,
    bus: u8,
    entered: bool,
}

impl<IO> PassthroughTransport<IO> {
    /// Wrap a byte stream; frames target I2C bus `0` by default.
    pub const fn new(io: IO) -> Self {
        Self {
            io,
            bus: BUS_I2C_BASE,
            entered: false,
        }
    }

    /// Wrap a byte stream with an explicit default bus ID.
    pub const fn with_bus(io: IO, bus: u8) -> Self {
        Self {
            io,
            bus,
            entered: false,
        }
    }

    /// Borrow the underlying stream.
    pub const fn io(&self) -> &IO {
        &self.io
    }

    /// Mutably borrow the underlying stream.
    pub fn io_mut(&mut self) -> &mut IO {
        &mut self.io
    }

    /// Consume the adapter and return the stream.
    pub fn into_inner(self) -> IO {
        self.io
    }

    /// Default bus ID used by [`Transport::read`] / [`Transport::write`].
    pub const fn bus(&self) -> u8 {
        self.bus
    }

    /// Change the default downstream bus ID.
    pub fn set_bus(&mut self, bus: u8) {
        self.bus = bus;
    }

    /// Whether [`Self::enter_passthrough`] completed successfully on this link.
    pub const fn is_entered(&self) -> bool {
        self.entered
    }
}

impl<IO> PassthroughTransport<IO>
where
    IO: IoRead + IoWrite + IoErrorType,
{
    /// Exchange one request frame for a response frame.
    ///
    /// `response` receives up to its length of response payload bytes. The
    /// returned tuple is the typed status and sub-status flags.
    fn exchange(
        &mut self,
        opcode: u8,
        bus: u8,
        node: u8,
        reg: u16,
        payload: &[u8],
        response: Option<&mut [u8]>,
    ) -> Result<(StatusCode, u8), TransportError<IO::Error>> {
        let mut tx = [0u8; MAX_FRAME_LEN];
        let len = encode_frame(
            &Frame::request(opcode, bus, node, reg, payload),
            &mut tx,
        )
        .map_err(|e| TransportError::Protocol(match e {
            FrameError::PayloadTooLarge => "passthrough payload exceeds MTU",
            FrameError::BufferTooSmall => "passthrough frame buffer too small",
            _ => "passthrough frame encoding error",
        }))?;
        self.io
            .write_all(&tx[..len])
            .map_err(TransportError::Bus)?;

        let mut rx = [0u8; MAX_FRAME_LEN];
        self.read_exact_io(&mut rx[..PT_HEADER_LEN])?;
        let payload_len = rx[7] as usize;
        if payload_len > PT_MAX_PAYLOAD {
            return Err(TransportError::Protocol(
                "passthrough response exceeds MTU",
            ));
        }
        let total = PT_OVERHEAD + payload_len;
        self.read_exact_io(&mut rx[PT_HEADER_LEN..total])?;
        let frame = decode_frame(&rx[..total]).map_err(|e| {
            TransportError::Protocol(match e {
                FrameError::InvalidCrc => "passthrough response CRC mismatch",
                FrameError::UnsupportedVersion(_) => "passthrough response version mismatch",
                _ => "passthrough malformed response",
            })
        })?;
        if frame.opcode != response_opcode(opcode) {
            return Err(TransportError::Protocol(
                "passthrough response opcode mismatch",
            ));
        }
        if let Some(out) = response {
            if out.len() < frame.payload.len() {
                return Err(TransportError::BufferTooSmall);
            }
            out[..frame.payload.len()].copy_from_slice(frame.payload);
        }
        Ok((frame.status(), frame.flags()))
    }

    fn read_exact_io(&mut self, buf: &mut [u8]) -> Result<(), TransportError<IO::Error>> {
        self.io.read_exact(buf).map_err(|e| match e {
            embedded_io::ReadExactError::Other(err) => TransportError::Bus(err),
            embedded_io::ReadExactError::UnexpectedEof => {
                TransportError::Protocol("passthrough link closed while reading frame")
            }
        })
    }

    /// Send `OP_PING` to `node` on the default bus.
    pub fn ping(&mut self, node: u8) -> Result<(), TransportError<IO::Error>> {
        let (status, _) = self.exchange(OP_PING, self.bus, node, 0, &[], None)?;
        map_status(status)?;
        Ok(())
    }

    /// Enter passthrough mode; the device refuses while any node is armed.
    pub fn enter_passthrough(&mut self) -> Result<(), TransportError<IO::Error>> {
        let (status, _) = self.exchange(
            OP_ENTER,
            BUS_BROADCAST_ALL,
            BROADCAST_ID,
            0,
            &[],
            None,
        )?;
        map_status(status)?;
        self.entered = true;
        Ok(())
    }

    /// Exit passthrough mode; the device disarms every downstream bus.
    pub fn exit_passthrough(&mut self) -> Result<(), TransportError<IO::Error>> {
        let result = self.exchange(
            OP_EXIT,
            BUS_BROADCAST_ALL,
            BROADCAST_ID,
            0,
            &[],
            None,
        );
        self.entered = false;
        let (status, _) = result?;
        map_status(status)?;
        Ok(())
    }

    /// Refresh the device watchdog without touching a downstream bus.
    pub fn keepalive(&mut self) -> Result<(), TransportError<IO::Error>> {
        let (status, _) = self.exchange(OP_KEEPALIVE, self.bus, 0, 0, &[], None)?;
        map_status(status)?;
        Ok(())
    }

    /// Emergency stop: leave passthrough and broadcast `REG_TORQUE_ENABLE = 0`.
    pub fn emergency_stop(&mut self) -> Result<(), TransportError<IO::Error>> {
        let result = self.exchange(
            OP_EMERGENCY_STOP,
            BUS_BROADCAST_ALL,
            BROADCAST_ID,
            REG_TORQUE_ENABLE,
            &[],
            None,
        );
        self.entered = false;
        let (status, _) = result?;
        map_status(status)?;
        Ok(())
    }

    /// Query device capabilities and version.
    pub fn get_info(&mut self) -> Result<DeviceInfo, TransportError<IO::Error>> {
        let mut payload = [0u8; DEVICE_INFO_LEN];
        let (status, _) = self.exchange(
            OP_GET_INFO,
            BUS_LOCAL,
            0,
            0,
            &[],
            Some(&mut payload),
        )?;
        map_status(status)?;
        DeviceInfo::decode(&payload)
            .map_err(|_| TransportError::Protocol("passthrough GET_INFO payload length"))
    }

    /// Single-round-trip I2C address discovery on `bus`.
    pub fn scan_bus_fast(&mut self, bus: u8) -> Result<ScanMask, TransportError<IO::Error>> {
        let mut payload = [0u8; SCAN_MASK_LEN];
        let (status, _) = self.exchange(OP_SCAN_BUS, bus, 0, 0, &[], Some(&mut payload))?;
        map_status(status)?;
        ScanMask::decode(&payload)
            .map_err(|_| TransportError::Protocol("passthrough SCAN_BUS payload length"))
    }
}

impl<IO> Transport for PassthroughTransport<IO>
where
    IO: IoRead + IoWrite + IoErrorType,
{
    // A single stream supplies both directions, so one error type covers both.
    type Error = IO::Error;

    fn write(&mut self, node: u8, reg: u16, bytes: &[u8]) -> Result<(), TransportError<Self::Error>> {
        let bus = self.bus;
        let (status, _) = self.exchange(OP_WRITE, bus, node, reg, bytes, None)?;
        map_status(status)?;
        Ok(())
    }

    fn read(&mut self, node: u8, reg: u16, out: &mut [u8]) -> Result<(), TransportError<Self::Error>> {
        if out.len() > PT_MAX_PAYLOAD {
            return Err(TransportError::BufferTooSmall);
        }
        let bus = self.bus;
        let (status, _) = self.exchange(OP_READ, bus, node, reg, &[], Some(out))?;
        map_status(status)?;
        Ok(())
    }

    fn send_packet(&mut self, packet: &[u8]) -> Result<(), TransportError<Self::Error>> {
        if packet.len() > PT_MAX_PAYLOAD {
            return Err(TransportError::BufferTooSmall);
        }
        let bus = self.bus;
        let (status, _) = self.exchange(
            OP_SYNC_WRITE,
            bus,
            BROADCAST_ID,
            REG_TORQUE_ENABLE,
            packet,
            None,
        )?;
        map_status(status)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::client::ControlTableClient;
    use crate::registers::{
        REG_FIRMWARE_VERSION, REG_HARDWARE_UID, REG_MODEL_NUMBER, REG_PRESENT_POSITION,
    };
    use crate::scanner::BusScanner;
    use crate::sync::SyncGroup;

    #[test]
    fn crc16_ccitt_matches_reference_vector() {
        assert_eq!(crc16_ccitt(b"123456789"), 0x29B1);
        assert_eq!(crc16_ccitt(b""), 0xFFFF);
    }

    #[test]
    fn frame_round_trip_and_exact_bytes() {
        let payload = 1.5f32.to_le_bytes();
        let frame = Frame::request(OP_WRITE, BUS_I2C_BASE, 0x40, 0x0130, &payload);
        let mut buf = [0u8; MAX_FRAME_LEN];
        let len = encode_frame(&frame, &mut buf).unwrap();
        assert_eq!(len, PT_OVERHEAD + 4);
        assert_eq!(buf[0], PT_SYNC);
        assert_eq!(buf[1], PROTOCOL_VERSION);
        assert_eq!(buf[2], OP_WRITE);
        assert_eq!(buf[3], BUS_I2C_BASE);
        assert_eq!(buf[4], 0x40);
        assert_eq!(&buf[5..7], &[0x01, 0x30]);
        assert_eq!(buf[7], 4);
        assert_eq!(&buf[8..12], &1.5f32.to_le_bytes());
        assert_eq!(
            &buf[12..14],
            &crc16_ccitt(&buf[..12]).to_le_bytes()
        );
        let decoded = decode_frame(&buf[..len]).unwrap();
        assert_eq!(decoded, frame);
        assert_eq!(decoded.register(), 0x0130);
    }

    #[test]
    fn corrupt_and_malformed_frames_are_rejected() {
        let frame = Frame::request(OP_PING, 0, 0, 0, &[]);
        let mut buf = [0u8; MAX_FRAME_LEN];
        let len = encode_frame(&frame, &mut buf).unwrap();
        buf[len - 1] ^= 0xFF;
        assert_eq!(decode_frame(&buf[..len]), Err(FrameError::InvalidCrc));
        buf[len - 1] ^= 0xFF;
        buf[1] = 0x02;
        assert_eq!(
            decode_frame(&buf[..len]),
            Err(FrameError::UnsupportedVersion(2))
        );
        buf[1] = PROTOCOL_VERSION;
        buf[0] = 0x00;
        assert_eq!(decode_frame(&buf[..len]), Err(FrameError::InvalidSync));
        assert_eq!(decode_frame(&buf[..PT_OVERHEAD - 1]), Err(FrameError::ShortFrame));
    }

    #[test]
    fn oversized_payload_is_rejected() {
        let payload = [0u8; PT_MAX_PAYLOAD + 1];
        let frame = Frame::request(OP_WRITE, 0, 1, 0, &payload);
        assert_eq!(
            encode_frame(&frame, &mut [0u8; MAX_FRAME_LEN + 1]),
            Err(FrameError::PayloadTooLarge)
        );
        let mut buf = [0u8; MAX_FRAME_LEN + 2];
        let full = Frame::request(OP_WRITE, 0, 1, 0, &[0u8; PT_MAX_PAYLOAD]);
        let len = encode_frame(&full, &mut buf).unwrap();
        buf[7] = PT_MAX_PAYLOAD as u8 + 1;
        assert_eq!(
            decode_frame(&buf[..len]),
            Err(FrameError::PayloadTooLarge)
        );
    }

    #[test]
    fn status_and_bus_partitions() {
        assert_eq!(StatusCode::from_byte(0x08), StatusCode::InvalidCrc);
        assert_eq!(StatusCode::from_byte(0xFE), StatusCode::ExecutionFault);
        assert_eq!(StatusCode::BusNak.as_byte(), 0x04);
        assert_eq!(response_opcode(OP_SCAN_BUS), RESP_SCAN_BUS);
        assert_eq!(bus_kind(0x00), BusKind::I2c);
        assert_eq!(bus_kind(0x0F), BusKind::I2c);
        assert_eq!(bus_kind(0x10), BusKind::Uart);
        assert_eq!(bus_kind(0x20), BusKind::Can);
        assert_eq!(bus_kind(BUS_LOCAL), BusKind::Local);
        assert_eq!(bus_kind(BUS_BROADCAST_ALL), BusKind::BroadcastAll);
        assert_eq!(bus_kind(0x7F), BusKind::Invalid);
    }

    #[test]
    fn get_info_serializes_exact_bytes() {
        let info = DeviceInfo {
            protocol_version: 1,
            supported_buses: BUS_CAP_I2C | BUS_CAP_UART | BUS_CAP_LOCAL,
            max_payload: 128,
            firmware_version: 0x0002_0007,
        };
        let mut buf = [0u8; DEVICE_INFO_LEN];
        assert_eq!(info.encode(&mut buf), Ok(DEVICE_INFO_LEN));
        assert_eq!(buf[0], 1);
        assert_eq!(&buf[1..5], &(0x0Bu32).to_le_bytes());
        assert_eq!(&buf[5..7], &128u16.to_le_bytes());
        assert_eq!(&buf[7..11], &0x0002_0007u32.to_le_bytes());
        assert_eq!(DeviceInfo::decode(&buf), Ok(info));
    }

    #[test]
    fn scan_mask_sets_only_listed_addresses() {
        let mask = ScanMask::from_addresses(&[0x40, 0x41, 0x7F]);
        assert!(mask.contains(0x40));
        assert!(mask.contains(0x41));
        assert!(mask.contains(0x7F));
        assert!(!mask.contains(0x42));
        assert_eq!(mask.bytes[8], 0x03);
        assert_eq!(mask.bytes[15], 0x80);
        assert_eq!(ScanMask::decode(&mask.bytes), Ok(mask));
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct TestIoError;

    impl embedded_io::Error for TestIoError {
        fn kind(&self) -> embedded_io::ErrorKind {
            embedded_io::ErrorKind::Other
        }
    }

    /// In-memory stream that parses frames written by the host and queues the
    /// matching response for the next read, emulating a simple device.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum StatusMode {
        Ok,
        BusNak,
        BusTimeout,
    }

    struct LoopbackIo {
        tx: [u8; 4096],
        tx_len: usize,
        parsed: usize,
        rx: [u8; 4096],
        rx_len: usize,
        rx_pos: usize,
        mask: ScanMask,
        status: StatusMode,
    }

    impl LoopbackIo {
        fn new() -> Self {
            Self {
                tx: [0; 4096],
                tx_len: 0,
                parsed: 0,
                rx: [0; 4096],
                rx_len: 0,
                rx_pos: 0,
                mask: ScanMask::from_addresses(&[0x40, 0x41]),
                status: StatusMode::Ok,
            }
        }

        fn respond(&mut self, frame: Frame<'_>) {
            let status = match self.status {
                StatusMode::Ok => StatusCode::Ok,
                StatusMode::BusNak => StatusCode::BusNak,
                StatusMode::BusTimeout => StatusCode::BusTimeout,
            };
            let mut payload = [0u8; PT_MAX_PAYLOAD];
            let payload_len: usize;
            match frame.opcode {
                OP_READ => {
                    payload_len = match frame.register() {
                        REG_MODEL_NUMBER => {
                            let model: u16 = if frame.node == 0x40 || frame.node == 0x41 {
                                0x0503
                            } else {
                                0xFFFF // floating bus: scanner skips it
                            };
                            payload[..2].copy_from_slice(&model.to_le_bytes());
                            2
                        }
                        REG_FIRMWARE_VERSION => {
                            payload[0] = 7;
                            1
                        }
                        REG_HARDWARE_UID => {
                            for (i, byte) in payload[..12].iter_mut().enumerate() {
                                *byte = i as u8;
                            }
                            12
                        }
                        _ => {
                            payload[..4].copy_from_slice(&1.5f32.to_le_bytes());
                            4
                        }
                    };
                }
                OP_SCAN_BUS => {
                    payload[..SCAN_MASK_LEN].copy_from_slice(&self.mask.bytes);
                    payload_len = SCAN_MASK_LEN;
                }
                OP_GET_INFO => {
                    let info = DeviceInfo {
                        protocol_version: PROTOCOL_VERSION,
                        supported_buses: BUS_CAP_I2C | BUS_CAP_UART | BUS_CAP_LOCAL,
                        max_payload: PT_MAX_PAYLOAD as u16,
                        firmware_version: 7,
                    };
                    payload_len = info.encode(&mut payload).unwrap();
                }
                _ => payload_len = 0,
            }
            let response = Frame::response(
                frame.opcode,
                frame.bus,
                frame.node,
                status,
                0,
                &payload[..payload_len],
            );
            let mut bytes = [0u8; MAX_FRAME_LEN];
            let len = encode_frame(&response, &mut bytes).unwrap();
            let end = self.rx_len + len;
            self.rx[self.rx_len..end].copy_from_slice(&bytes[..len]);
            self.rx_len = end;
        }

        fn pump(&mut self) {
            while self.parsed + PT_OVERHEAD <= self.tx_len {
                let len_field = self.tx[self.parsed + 7] as usize;
                let total = PT_OVERHEAD + len_field;
                if self.parsed + total > self.tx_len {
                    break;
                }
                let mut frame_bytes = [0u8; MAX_FRAME_LEN];
                frame_bytes[..total]
                    .copy_from_slice(&self.tx[self.parsed..self.parsed + total]);
                if let Ok(frame) = decode_frame(&frame_bytes[..total]) {
                    self.respond(frame);
                }
                self.parsed += total;
            }
        }
    }

    impl IoErrorType for LoopbackIo {
        type Error = TestIoError;
    }

    impl IoWrite for LoopbackIo {
        fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            let n = buf.len().min(self.tx.len() - self.tx_len);
            self.tx[self.tx_len..self.tx_len + n].copy_from_slice(&buf[..n]);
            self.tx_len += n;
            self.pump();
            Ok(n)
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    impl IoRead for LoopbackIo {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            let remaining = self.rx_len - self.rx_pos;
            let n = buf.len().min(remaining);
            buf[..n].copy_from_slice(&self.rx[self.rx_pos..self.rx_pos + n]);
            self.rx_pos += n;
            Ok(n)
        }
    }

    #[test]
    fn typed_client_and_sync_group_work_over_passthrough() {
        let mut transport = PassthroughTransport::new(LoopbackIo::new());
        transport.enter_passthrough().unwrap();
        assert!(transport.is_entered());

        let mut client = ControlTableClient::new(0x40, &mut transport);
        assert_eq!(client.read_u16(REG_MODEL_NUMBER).unwrap(), 0x0503);
        let value = client.read_f32(REG_PRESENT_POSITION).unwrap();
        assert_eq!(value, 1.5);
        client.write_f32(REG_PRESENT_POSITION, 2.0).unwrap();

        let mut uid = [0u8; 12];
        client.read_bytes(REG_HARDWARE_UID, 12, &mut uid).unwrap();
        assert_eq!(uid[11], 11);

        let mut group = SyncGroup::new(client.into_inner());
        group.sync_write_position(&[(1, 0.5), (2, -0.5)]).unwrap();
        let transport = group.into_inner();
        transport.exit_passthrough().unwrap();
        assert!(!transport.is_entered());
    }

    #[test]
    fn scanner_and_host_helpers_work_over_passthrough() {
        let mut transport = PassthroughTransport::new(LoopbackIo::new());
        transport.enter_passthrough().unwrap();
        let mut scanner = BusScanner::new(&mut transport);
        let nodes = scanner.scan_i2c(0x40..=0x42).unwrap();
        assert_eq!(nodes.len(), 2);
        assert_eq!(nodes[0].address, 0x40);
        assert_eq!(nodes[1].address, 0x41);

        let transport = scanner.into_inner();
        let mask = transport.scan_bus_fast(BUS_I2C_BASE).unwrap();
        assert!(mask.contains(0x40) && mask.contains(0x41));
        let info = transport.get_info().unwrap();
        assert_eq!(info.protocol_version, PROTOCOL_VERSION);
        assert_eq!(info.max_payload, PT_MAX_PAYLOAD as u16);
        transport.ping(0x40).unwrap();
        transport.keepalive().unwrap();
        transport.emergency_stop().unwrap();
    }

    #[test]
    fn status_codes_map_to_transport_errors() {
        let mut transport = PassthroughTransport::new(LoopbackIo::new());
        transport.io_mut().status = StatusMode::BusNak;
        let err = ControlTableClient::new(0x40, &mut transport)
            .read_u8(REG_FIRMWARE_VERSION)
            .unwrap_err();
        assert!(matches!(
            err,
            crate::client::ClientError::Transport(TransportError::NoAcknowledge)
        ));
        transport.io_mut().status = StatusMode::BusTimeout;
        let err = ControlTableClient::new(0x40, &mut transport)
            .read_u8(REG_FIRMWARE_VERSION)
            .unwrap_err();
        assert!(matches!(
            err,
            crate::client::ClientError::Transport(TransportError::Timeout)
        ));
        let err = transport.enter_passthrough().unwrap_err();
        assert!(matches!(err, TransportError::Timeout));
    }
}
