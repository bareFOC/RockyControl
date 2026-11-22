//! Zero-allocation `#![no_std]` MCU flash-resident URDF / kinematics model store.
//!
//! Embeds the robot's URDF directly into MCU `.rodata` XIP flash (`0` bytes of SRAM overhead),
//! computes its IEEE CRC32 at compile time, and serves fixed-size `postcard` + COBS chunks so
//! any Rocky Studio desktop or browser workstation can connect to the MCU and automatically
//! pull the exact kinematics model without shipping local URDF files.
//!
//! # Flash Budget (RP2350 & ESP32-P4)
//! - `rocky_mk1.urdf` raw UTF-8 XML: ~9.0 KB (~0.22% of 4 MB RP2350 Pico 2 flash).
//! - Optional zlib/deflate URDF: ~1.6 KB (~0.04% of 4 MB flash).
//! - Dedicated flash partition (e.g., last 64 KB erase sector) can later accept runtime URDF
//!   updates without recompiling firmware while keeping the exact same `ModelRequest` wire API.

use crate::config::DEFAULT_JOINT_COUNT;
use humanoid_proto::{
    MAX_MODEL_COBS_FRAME_BYTES, MODEL_CHUNK_BYTES, ModelChunk, ModelFormat, ModelInfo,
    ModelRequest, ModelResponse, decode_model_req_cobs, encode_model_resp_cobs,
};

/// Robot URDF embedded directly in MCU `.rodata` XIP flash (zero SRAM cost).
pub const EMBEDDED_URDF_BYTES: &[u8] =
    include_bytes!("../../../assets/sample_humanoid/rocky_mk1.urdf");

/// Compile-time metadata + CRC32 for the flash-resident URDF blob.
pub const EMBEDDED_MODEL_INFO: ModelInfo = ModelInfo::from_static(
    "rocky_mk1",
    ModelFormat::UrdfXml,
    EMBEDDED_URDF_BYTES,
    DEFAULT_JOINT_COUNT as u8,
);

/// Read-only flash-backed model provider served off the real-time servo control path.
#[derive(Clone, Copy, Debug)]
pub struct FlashModelStore<'a> {
    info: ModelInfo,
    blob: &'a [u8],
}

impl FlashModelStore<'static> {
    /// Returns the default compile-time flash store containing `rocky_mk1.urdf`.
    pub const fn bundled_rocky() -> Self {
        Self {
            info: EMBEDDED_MODEL_INFO,
            blob: EMBEDDED_URDF_BYTES,
        }
    }
}

impl<'a> FlashModelStore<'a> {
    pub const fn new(info: ModelInfo, blob: &'a [u8]) -> Self {
        Self { info, blob }
    }

    pub const fn info(&self) -> ModelInfo {
        self.info
    }

    pub const fn blob(&self) -> &'a [u8] {
        self.blob
    }

    /// Processes a single `ModelRequest` with zero heap allocations.
    pub fn handle_request(&self, req: ModelRequest) -> ModelResponse {
        if self.blob.is_empty() {
            return ModelResponse::NotAvailable;
        }
        match req {
            ModelRequest::GetInfo => ModelResponse::Info(self.info),
            ModelRequest::ReadChunk { offset, max_len } => {
                let start = offset as usize;
                if start >= self.blob.len() {
                    return ModelResponse::Chunk(ModelChunk {
                        offset,
                        total_bytes: self.blob.len() as u32,
                        crc32: self.info.crc32,
                        format: self.info.format,
                        len: 0,
                        data: [0u8; MODEL_CHUNK_BYTES],
                    });
                }
                let req_len = (max_len as usize).clamp(1, MODEL_CHUNK_BYTES);
                let avail = self.blob.len() - start;
                let copy_len = req_len.min(avail);
                let mut data = [0u8; MODEL_CHUNK_BYTES];
                data[..copy_len].copy_from_slice(&self.blob[start..start + copy_len]);
                ModelResponse::Chunk(ModelChunk {
                    offset,
                    total_bytes: self.blob.len() as u32,
                    crc32: self.info.crc32,
                    format: self.info.format,
                    len: copy_len as u16,
                    data,
                })
            }
        }
    }

    /// Decodes a COBS-framed `ModelRequest` datagram and encodes the `ModelResponse` into `out_buf`.
    pub fn handle_cobs_datagram<'b>(
        &self,
        rx_datagram: &[u8],
        out_buf: &'b mut [u8; MAX_MODEL_COBS_FRAME_BYTES],
    ) -> Option<&'b [u8]> {
        if rx_datagram.is_empty() || rx_datagram.len() > MAX_MODEL_COBS_FRAME_BYTES {
            return None;
        }
        let mut scratch = [0u8; MAX_MODEL_COBS_FRAME_BYTES];
        scratch[..rx_datagram.len()].copy_from_slice(rx_datagram);
        let req = decode_model_req_cobs(&mut scratch[..rx_datagram.len()]).ok()?;
        let resp = self.handle_request(req);
        encode_model_resp_cobs(&resp, out_buf).ok()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use humanoid_proto::{crc32, decode_model_resp_cobs, encode_model_req_cobs};

    #[test]
    fn flash_model_store_streams_full_urdf_with_exact_crc32() {
        let store = FlashModelStore::bundled_rocky();
        let mut req_wire = [0u8; MAX_MODEL_COBS_FRAME_BYTES];
        let mut resp_wire = [0u8; MAX_MODEL_COBS_FRAME_BYTES];

        // 1. Query ModelInfo over COBS
        let req_len = encode_model_req_cobs(&ModelRequest::GetInfo, &mut req_wire)
            .unwrap()
            .len();
        let resp_len = store
            .handle_cobs_datagram(&req_wire[..req_len], &mut resp_wire)
            .expect("info response")
            .len();
        let info = match decode_model_resp_cobs(&mut resp_wire[..resp_len]).unwrap() {
            ModelResponse::Info(info) => info,
            other => panic!("expected ModelResponse::Info, got {other:?}"),
        };
        assert_eq!(info.name_str(), "rocky_mk1");
        assert_eq!(info.joint_count, DEFAULT_JOINT_COUNT as u8);
        assert_eq!(info.total_bytes as usize, EMBEDDED_URDF_BYTES.len());

        // 2. Pull all chunks and reassemble
        let mut reassembled = std::vec::Vec::with_capacity(info.total_bytes as usize);
        while (reassembled.len() as u32) < info.total_bytes {
            let req = ModelRequest::ReadChunk {
                offset: reassembled.len() as u32,
                max_len: MODEL_CHUNK_BYTES as u16,
            };
            let rlen = encode_model_req_cobs(&req, &mut req_wire).unwrap().len();
            let olen = store
                .handle_cobs_datagram(&req_wire[..rlen], &mut resp_wire)
                .expect("chunk response")
                .len();
            match decode_model_resp_cobs(&mut resp_wire[..olen]).unwrap() {
                ModelResponse::Chunk(chunk) => {
                    assert_eq!(chunk.offset as usize, reassembled.len());
                    assert!(chunk.len > 0);
                    reassembled.extend_from_slice(chunk.payload());
                }
                other => panic!("expected chunk, got {other:?}"),
            }
        }

        assert_eq!(reassembled.len(), EMBEDDED_URDF_BYTES.len());
        assert_eq!(crc32(&reassembled), info.crc32);
        assert_eq!(reassembled.as_slice(), EMBEDDED_URDF_BYTES);
    }
}
