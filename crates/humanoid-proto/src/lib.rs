#![cfg_attr(not(feature = "std"), no_std)]
//! Layer 0 — Shared Wire Protocol (`humanoid-proto`).
//!
//! Strictly `#![no_std]` with fixed-capacity `f32` arrays and `postcard` COBS framing
//! shared identically across RP2350 MCU, Native Desktop, and Browser WASM.

use bitflags::bitflags;
use serde::{Deserialize, Serialize};

pub const MAX_JOINTS: usize = 20;
pub const WATCHDOG_TIMEOUT_MS: u32 = 100;
pub const MAX_COBS_FRAME_BYTES: usize = 512;
pub const RATED_CURRENT_MA: u16 = 2000;
pub const HEAT_TINT_THRESHOLD_MA: u16 = 1400; // 70% of rated current

#[repr(u8)]
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ControlMode {
    EStop = 0,
    #[default]
    Teach = 1, // Torque OFF: MCU streams measured_rad -> GUI mirrors robot
    Drive = 2, // Torque ON:  GUI streams target_rad -> robot mirrors GUI
}

bitflags! {
    #[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq, Default)]
    #[serde(transparent)]
    pub struct FaultFlags: u16 {
        const WATCHDOG_TIMEOUT = 1 << 0;
        const ESTOP_ACTIVE     = 1 << 1;
        const OVER_CURRENT     = 1 << 2;
        const OVER_TEMP        = 1 << 3;
        const UNDER_VOLTAGE    = 1 << 4;
        const SERVO_BUS_ERROR  = 1 << 5;
        const LIMIT_CLAMPED    = 1 << 6;
    }
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Default)]
pub enum ConfigCmd {
    #[default]
    None,
    SetZeroOffset {
        joint: u8,
        zero_rad: f32,
    },
    SetInvert {
        joint: u8,
        inverted: bool,
    },
    SetSoftLimits {
        joint: u8,
        min_rad: f32,
        max_rad: f32,
    },
    CommitFlash,
    ClearFaults,
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub struct HostToMcu {
    pub seq: u32,
    pub mode: ControlMode,
    pub torque_mask: u32,
    pub speed_limit_pct: u8,
    pub target_rad: [f32; MAX_JOINTS],
    pub config: ConfigCmd,
}

impl Default for HostToMcu {
    fn default() -> Self {
        Self {
            seq: 0,
            mode: ControlMode::Teach,
            torque_mask: 0,
            speed_limit_pct: 25,
            target_rad: [0.0_f32; MAX_JOINTS],
            config: ConfigCmd::None,
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub struct McuToHost {
    pub seq_ack: u32,
    pub uptime_ms: u32,
    pub active_mode: ControlMode,
    pub faults: FaultFlags,
    pub torque_state_mask: u32,
    pub measured_rad: [f32; MAX_JOINTS],
    pub applied_target_rad: [f32; MAX_JOINTS],
    pub current_ma: [u16; MAX_JOINTS],
    pub temp_c: [u8; MAX_JOINTS],
    pub bus_voltage_mv: u16,
}

impl Default for McuToHost {
    fn default() -> Self {
        Self {
            seq_ack: 0,
            uptime_ms: 0,
            active_mode: ControlMode::Teach,
            faults: FaultFlags::empty(),
            torque_state_mask: 0,
            measured_rad: [0.0_f32; MAX_JOINTS],
            applied_target_rad: [0.0_f32; MAX_JOINTS],
            current_ma: [0_u16; MAX_JOINTS],
            temp_c: [28_u8; MAX_JOINTS],
            bus_voltage_mv: 12_400,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CodecError {
    EncodeFailed,
    DecodeFailed,
    NonFiniteFloat,
}

/// Zero-allocation `postcard` + COBS encode into a caller-supplied buffer.
/// Output is terminated by a single `0x00` delimiter byte.
pub fn encode_cobs<'a, T: Serialize>(
    value: &T,
    buf: &'a mut [u8],
) -> Result<&'a [u8], CodecError> {
    postcard::to_slice_cobs(value, buf)
        .map(|slice| &*slice)
        .map_err(|_| CodecError::EncodeFailed)
}

/// Zero-allocation `postcard` + COBS decode from a mutable frame slice (returns parsed value and remainder).
/// Never panics on malformed or corrupted frames.
pub fn decode_cobs<'a, T: Deserialize<'a>>(
    buf: &'a mut [u8],
) -> Result<(T, &'a [u8]), CodecError> {
    postcard::take_from_bytes_cobs(buf)
        .map(|(val, rest)| (val, &*rest))
        .map_err(|_| CodecError::DecodeFailed)
}

/// Validates that a `HostToMcu` payload contains only finite single-precision floats.
pub fn validate_host_packet(pkt: &HostToMcu) -> Result<(), CodecError> {
    if pkt.target_rad.iter().any(|v| !v.is_finite()) {
        return Err(CodecError::NonFiniteFloat);
    }
    match pkt.config {
        ConfigCmd::SetZeroOffset { zero_rad, .. } if !zero_rad.is_finite() => {
            return Err(CodecError::NonFiniteFloat);
        }
        ConfigCmd::SetSoftLimits {
            min_rad, max_rad, ..
        } if !min_rad.is_finite() || !max_rad.is_finite() => {
            return Err(CodecError::NonFiniteFloat);
        }
        _ => {}
    }
    Ok(())
}

/// Decodes a `HostToMcu` frame and rejects NaN/Inf floats.
pub fn decode_host_cobs(buf: &mut [u8]) -> Result<HostToMcu, CodecError> {
    let (pkt, _) = decode_cobs::<HostToMcu>(buf)?;
    validate_host_packet(&pkt)?;
    Ok(pkt)
}

/// Decodes an `McuToHost` frame and rejects NaN/Inf floats.
pub fn decode_mcu_cobs(buf: &mut [u8]) -> Result<McuToHost, CodecError> {
    let (pkt, _) = decode_cobs::<McuToHost>(buf)?;
    if pkt.measured_rad.iter().any(|v| !v.is_finite())
        || pkt.applied_target_rad.iter().any(|v| !v.is_finite())
    {
        return Err(CodecError::NonFiniteFloat);
    }
    Ok(pkt)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn host_to_mcu_cobs_round_trip() {
        let mut pkt = HostToMcu {
            seq: 42,
            mode: ControlMode::Drive,
            torque_mask: 0x0001_FFFF,
            speed_limit_pct: 60,
            target_rad: [0.0; MAX_JOINTS],
            config: ConfigCmd::SetSoftLimits {
                joint: 3,
                min_rad: -2.1,
                max_rad: 1.25,
            },
        };
        pkt.target_rad[0] = 0.25;
        pkt.target_rad[5] = -1.75;

        let mut wire = [0u8; MAX_COBS_FRAME_BYTES];
        let encoded = encode_cobs(&pkt, &mut wire).expect("encode host");
        assert_eq!(*encoded.last().unwrap(), 0x00, "COBS delimiter");
        let len = encoded.len();

        let decoded = decode_host_cobs(&mut wire[..len]).expect("decode host");
        assert_eq!(decoded, pkt);
    }

    #[test]
    fn mcu_to_host_cobs_round_trip() {
        let mut pkt = McuToHost {
            seq_ack: 99,
            uptime_ms: 12_345,
            active_mode: ControlMode::Drive,
            faults: FaultFlags::OVER_CURRENT | FaultFlags::LIMIT_CLAMPED,
            torque_state_mask: 0x0000_00FF,
            bus_voltage_mv: 12_150,
            ..Default::default()
        };
        pkt.measured_rad[3] = -0.88;
        pkt.applied_target_rad[3] = -0.92;
        pkt.current_ma[3] = 2150;
        pkt.temp_c[3] = 51;

        let mut wire = [0u8; MAX_COBS_FRAME_BYTES];
        let encoded = encode_cobs(&pkt, &mut wire).expect("encode mcu");
        let len = encoded.len();

        let decoded = decode_mcu_cobs(&mut wire[..len]).expect("decode mcu");
        assert_eq!(decoded, pkt);
    }

    #[test]
    fn cobs_corrupted_and_truncated_frames_never_panic() {
        let cases: [&[u8]; 5] = [
            &[],
            &[0x00],
            &[0x05, 0xAA, 0xBB, 0x00],
            &[0xFF, 0x01, 0x02, 0x03, 0x00],
            &[0x02, 0x7F, 0x00],
        ];
        for sample in cases {
            let mut buf = [0u8; 32];
            buf[..sample.len()].copy_from_slice(sample);
            assert!(decode_host_cobs(&mut buf[..sample.len()]).is_err());
            assert!(decode_mcu_cobs(&mut buf[..sample.len()]).is_err());
        }
    }
}
