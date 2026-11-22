//! Zero-allocation `#![no_std]` wired host transport framing & Embassy <-> Copper mailbox.
//!
//! Supports both wired bring-up links with identical `postcard` COBS packets:
//! 1. **USB CDC-ACM** (byte-stream framing via [`CobsStreamFramer`])
//! 2. **USB Networking (CDC-NCM + `embassy-net` UDP)** (datagram framing via [`decode_udp_host_frame`])

use core::cell::RefCell;

use critical_section::Mutex;
use humanoid_proto::{
    CodecError, HostToMcu, MAX_COBS_FRAME_BYTES, McuToHost, decode_host_cobs, encode_cobs,
};

/// Zero-allocation streaming COBS frame accumulator for USB CDC-ACM bulk transfers.
///
/// USB Full-Speed bulk endpoints deliver chunks up to 64 bytes (`MAX_PACKET_SIZE = 64`),
/// so a single `HostToMcu` packet (~120 bytes COBS-encoded) spans 2–3 USB reads.
/// Feed every received USB byte slice into [`push_bytes`](Self::push_bytes); whenever a
/// `0x00` COBS delimiter completes a valid packet, the latest decoded [`HostToMcu`] is returned.
pub struct CobsStreamFramer {
    buf: [u8; MAX_COBS_FRAME_BYTES],
    len: usize,
    overflowed: bool,
    pub rx_ok: u32,
    pub rx_err: u32,
}

impl Default for CobsStreamFramer {
    fn default() -> Self {
        Self::new()
    }
}

impl CobsStreamFramer {
    pub const fn new() -> Self {
        Self {
            buf: [0u8; MAX_COBS_FRAME_BYTES],
            len: 0,
            overflowed: false,
            rx_ok: 0,
            rx_err: 0,
        }
    }

    /// Feeds raw bytes from USB CDC-ACM. Calls `on_packet` for every complete, valid `HostToMcu`.
    pub fn push_bytes<F: FnMut(HostToMcu)>(&mut self, chunk: &[u8], mut on_packet: F) {
        for &byte in chunk {
            if byte == 0x00 {
                if self.overflowed {
                    // Discard truncated oversized frame and resynchronize at next byte.
                    self.overflowed = false;
                    self.len = 0;
                    self.rx_err = self.rx_err.saturating_add(1);
                    continue;
                }
                if self.len == 0 {
                    // Ignore empty back-to-back delimiters.
                    continue;
                }
                if self.len < MAX_COBS_FRAME_BYTES {
                    self.buf[self.len] = 0x00;
                    let frame_len = self.len + 1;
                    match decode_host_cobs(&mut self.buf[..frame_len]) {
                        Ok(pkt) => {
                            self.rx_ok = self.rx_ok.saturating_add(1);
                            on_packet(pkt);
                        }
                        Err(_) => {
                            self.rx_err = self.rx_err.saturating_add(1);
                        }
                    }
                } else {
                    self.rx_err = self.rx_err.saturating_add(1);
                }
                self.len = 0;
            } else if !self.overflowed {
                if self.len + 1 < MAX_COBS_FRAME_BYTES {
                    self.buf[self.len] = byte;
                    self.len += 1;
                } else {
                    self.overflowed = true;
                }
            }
        }
    }
}

/// Decodes a single UDP datagram carrying a `postcard` COBS-framed `HostToMcu` packet.
pub fn decode_udp_host_frame(datagram: &[u8]) -> Result<HostToMcu, CodecError> {
    if datagram.is_empty() || datagram.len() > MAX_COBS_FRAME_BYTES {
        return Err(CodecError::DecodeFailed);
    }
    let mut scratch = [0u8; MAX_COBS_FRAME_BYTES];
    scratch[..datagram.len()].copy_from_slice(datagram);
    decode_host_cobs(&mut scratch[..datagram.len()])
}

/// Encodes `McuToHost` telemetry into a COBS-delimited wire frame suitable for USB CDC-ACM or UDP.
pub fn encode_telemetry_frame<'a>(
    telem: &McuToHost,
    out: &'a mut [u8; MAX_COBS_FRAME_BYTES],
) -> Result<&'a [u8], CodecError> {
    encode_cobs(telem, out)
}

/// Lock-free / critical-section mailbox decoupling asynchronous Embassy USB/UDP I/O tasks
/// from the deterministic synchronous `copper-rs` `HostLinkBridge` control tick.
struct MailboxInner {
    pending_cmd: Option<HostToMcu>,
    latest_telem: Option<McuToHost>,
    rx_packets: u32,
    tx_packets: u32,
    rx_errors: u32,
}

impl MailboxInner {
    const fn new() -> Self {
        Self {
            pending_cmd: None,
            latest_telem: None,
            rx_packets: 0,
            tx_packets: 0,
            rx_errors: 0,
        }
    }
}

pub struct HostLinkMailbox {
    inner: Mutex<RefCell<MailboxInner>>,
}

impl Default for HostLinkMailbox {
    fn default() -> Self {
        Self::new()
    }
}

impl HostLinkMailbox {
    pub const fn new() -> Self {
        Self {
            inner: Mutex::new(RefCell::new(MailboxInner::new())),
        }
    }

    /// Called by Embassy USB CDC-ACM / USB-NCM UDP RX task whenever a fresh `HostToMcu` arrives.
    /// Keeps the freshest packet (latest-wins semantics for real-time control).
    pub fn push_rx_cmd(&self, cmd: HostToMcu) {
        critical_section::with(|cs| {
            let mut m = self.inner.borrow_ref_mut(cs);
            m.pending_cmd = Some(cmd);
            m.rx_packets = m.rx_packets.saturating_add(1);
        });
    }

    /// Records a malformed frame error from the transport layer.
    pub fn record_rx_error(&self) {
        critical_section::with(|cs| {
            let mut m = self.inner.borrow_ref_mut(cs);
            m.rx_errors = m.rx_errors.saturating_add(1);
        });
    }

    /// Called by `copper-rs` `HostLinkBridge::receive` once per control tick.
    pub fn take_rx_cmd(&self) -> Option<HostToMcu> {
        critical_section::with(|cs| self.inner.borrow_ref_mut(cs).pending_cmd.take())
    }

    /// Called by `copper-rs` `HostLinkBridge::send` once per control tick.
    pub fn publish_tx_telem(&self, telem: McuToHost) {
        critical_section::with(|cs| {
            let mut m = self.inner.borrow_ref_mut(cs);
            m.latest_telem = Some(telem);
            m.tx_packets = m.tx_packets.saturating_add(1);
        });
    }

    /// Called by Embassy USB CDC-ACM / USB-NCM UDP TX task to drain the freshest telemetry frame.
    pub fn take_tx_telem(&self) -> Option<McuToHost> {
        critical_section::with(|cs| self.inner.borrow_ref_mut(cs).latest_telem.take())
    }

    /// Diagnostic counters `(rx_packets, tx_packets, rx_errors)`.
    pub fn stats(&self) -> (u32, u32, u32) {
        critical_section::with(|cs| {
            let m = self.inner.borrow_ref(cs);
            (m.rx_packets, m.tx_packets, m.rx_errors)
        })
    }
}

/// Global singleton mailbox bridging board Embassy transport tasks to `copper-rs` `HostLinkBridge`.
pub static HOST_LINK_MAILBOX: HostLinkMailbox = HostLinkMailbox::new();

#[cfg(test)]
mod tests {
    use super::*;
    use humanoid_proto::{ConfigCmd, ControlMode};

    #[test]
    fn cobs_stream_framer_reassembles_64_byte_usb_chunks() {
        let mut cmd = HostToMcu::default();
        cmd.seq = 42;
        cmd.mode = ControlMode::Drive;
        cmd.torque_mask = 0x1FFFF;
        cmd.speed_limit_pct = 30;
        cmd.target_rad[3] = -0.75_f32;
        cmd.config = ConfigCmd::SetSoftLimits {
            joint: 3,
            min_rad: -1.5,
            max_rad: 1.5,
        };

        let mut wire = [0u8; MAX_COBS_FRAME_BYTES];
        let encoded = encode_cobs(&cmd, &mut wire).expect("encode host cmd");
        assert!(encoded.len() > 64, "packet should span multiple 64B USB chunks");

        let mut framer = CobsStreamFramer::new();
        let mut received = None;
        for chunk in encoded.chunks(64) {
            framer.push_bytes(chunk, |pkt| {
                received = Some(pkt);
            });
        }

        assert_eq!(received, Some(cmd));
        assert_eq!(framer.rx_ok, 1);
        assert_eq!(framer.rx_err, 0);
    }

    #[test]
    fn cobs_stream_framer_recovers_after_corrupt_bytes() {
        let mut framer = CobsStreamFramer::new();
        let mut count = 0usize;
        // Feed garbage followed by a delimiter
        framer.push_bytes(&[0xFF, 0xAA, 0x11, 0x00], |_| count += 1);
        assert_eq!(count, 0);
        assert_eq!(framer.rx_err, 1);

        // Next valid frame must still decode cleanly
        let mut cmd = HostToMcu::default();
        cmd.seq = 7;
        let mut wire = [0u8; MAX_COBS_FRAME_BYTES];
        let encoded = encode_cobs(&cmd, &mut wire).unwrap();
        let mut got = None;
        framer.push_bytes(encoded, |p| got = Some(p));
        assert_eq!(got, Some(cmd));
        assert_eq!(framer.rx_ok, 1);
    }

    #[test]
    fn udp_datagram_round_trip_succeeds() {
        let mut cmd = HostToMcu::default();
        cmd.seq = 99;
        cmd.mode = ControlMode::Teach;
        let mut wire = [0u8; MAX_COBS_FRAME_BYTES];
        let encoded = encode_cobs(&cmd, &mut wire).unwrap();
        let decoded = decode_udp_host_frame(encoded).expect("udp decode");
        assert_eq!(decoded, cmd);
    }
}
