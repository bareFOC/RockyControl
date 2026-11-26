//! Zero-allocation `#![no_std]` model-store request handler.
//!
//! The MCU does **not** bundle a URDF: model data is owned by the Rocky Studio
//! application (bundled preset, upload, or staged flash blob) and is never
//! parsed by the real-time control path. Consequently the MCU exposes no model
//! pull service (the former UDP `:7778` service is gone) and the app's
//! `mcu_sync_model` path reports "not available" rather than downloading a
//! firmware-embedded model.
//!
//! [`FlashModelStore`] remains a caller-provided, read-only blob server over the
//! shared `ModelRequest`/`ModelResponse` wire API. It is used by the simulator
//! and host tests to validate the exact code path a runtime flash-loaded model
//! would use; it costs no SRAM and links no model bytes unless a caller supplies
//! them.

use rk_proto::{
    MAX_MODEL_COBS_FRAME_BYTES, MODEL_CHUNK_BYTES, ModelChunk, ModelInfo, ModelRequest,
    ModelResponse, decode_model_req_cobs, encode_model_resp_cobs,
};

/// Read-only model provider served off the real-time servo control path.
///
/// The caller owns the blob; no model bytes are compiled into this crate.
#[derive(Clone, Copy, Debug)]
pub struct FlashModelStore<'a> {
    info: ModelInfo,
    blob: &'a [u8],
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
    use rk_proto::{ModelFormat, crc32, decode_model_resp_cobs, encode_model_req_cobs};

    // Caller-owned test blob: no URDF is bundled in the firmware crate.
    const TEST_BLOB: &[u8] = b"<robot name=\"test\"><joint name=\"a\"/></robot>";
    const TEST_INFO: ModelInfo = ModelInfo::from_static("test", ModelFormat::UrdfXml, TEST_BLOB, 1);

    #[test]
    fn flash_model_store_streams_caller_owned_blob_with_exact_crc32() {
        let store = FlashModelStore::new(TEST_INFO, TEST_BLOB);
        let mut req_wire = [0u8; MAX_MODEL_COBS_FRAME_BYTES];
        let mut resp_wire = [0u8; MAX_MODEL_COBS_FRAME_BYTES];

        // 1. Query ModelInfo over COBS.
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
        assert_eq!(info.name_str(), "test");
        assert_eq!(info.joint_count, 1);
        assert_eq!(info.total_bytes as usize, TEST_BLOB.len());

        // 2. Pull all chunks and reassemble.
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

        assert_eq!(reassembled.len(), TEST_BLOB.len());
        assert_eq!(crc32(&reassembled), info.crc32);
        assert_eq!(reassembled.as_slice(), TEST_BLOB);
    }

    #[test]
    fn flash_model_store_without_blob_reports_not_available() {
        let store = FlashModelStore::new(ModelInfo::default(), &[]);
        assert_eq!(store.handle_request(ModelRequest::GetInfo), ModelResponse::NotAvailable);
    }
}
