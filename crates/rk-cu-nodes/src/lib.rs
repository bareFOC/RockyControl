#![cfg_attr(not(feature = "std"), no_std)]
//! Layer 2 — Copper-rs Task & Bridge Wrappers (`rk-cu-nodes`).
//!
//! Wraps the `#![no_std]` [`DirectServoKernel`] from `rk-control` as a deterministic
//! `DirectServoTask` plus `HostLinkBridge` / `ServoBusBridge` descriptors matching
//! `copperconfig.ron` for both Stage 1 desktop/WASM simulation and Stage 2 RP2350 firmware.

use rk_control::{
    DirectServoKernel, JointCalibration, KernelStepOutput, RawServoCommand, RawServoFeedback,
};
use rk_proto::{
    CodecError, HostToMcu, MAX_COBS_FRAME_BYTES, McuToHost, decode_host_cobs, encode_cobs,
};
use serde::{Deserialize, Serialize};

/// Embedded `copperconfig.ron` task-graph specification shared across sim and firmware.
pub const COPPER_CONFIG_RON: &str = include_str!("../copperconfig.ron");

/// One deterministic CopperList execution cycle snapshot.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub struct CopperListTick {
    pub tick_index: u64,
    pub now_ms: u32,
    pub dt_sec: f32,
    pub rx_cmd: Option<HostToMcu>,
    pub rx_state: RawServoFeedback,
    pub tx_cmd: RawServoCommand,
    pub tx_telem: McuToHost,
}

/// Copper task wrapping `DirectServoKernel`.
#[derive(Clone, Debug)]
pub struct DirectServoTask {
    kernel: DirectServoKernel,
    tick_index: u64,
}

impl DirectServoTask {
    pub fn new(joint_count: usize, initial_cal: &[JointCalibration]) -> Self {
        Self {
            kernel: DirectServoKernel::new(joint_count, initial_cal),
            tick_index: 0,
        }
    }

    pub fn kernel(&self) -> &DirectServoKernel {
        &self.kernel
    }

    pub fn kernel_mut(&mut self) -> &mut DirectServoKernel {
        &mut self.kernel
    }

    pub fn tick_index(&self) -> u64 {
        self.tick_index
    }

    /// Executes one deterministic Copper DAG step (`rx_cmd` + `rx_state` -> `tx_cmd` + `tx_telem`).
    pub fn process(
        &mut self,
        now_ms: u32,
        dt_sec: f32,
        rx_cmd: Option<&HostToMcu>,
        rx_state: &RawServoFeedback,
    ) -> (KernelStepOutput, CopperListTick) {
        let out = self.kernel.step(now_ms, dt_sec, rx_cmd, rx_state);
        let record = CopperListTick {
            tick_index: self.tick_index,
            now_ms,
            dt_sec,
            rx_cmd: rx_cmd.copied(),
            rx_state: *rx_state,
            tx_cmd: out.servo_cmd,
            tx_telem: out.telemetry,
        };
        self.tick_index = self.tick_index.saturating_add(1);
        (out, record)
    }
}

/// Host link bridge framing adapter (`postcard` COBS over USB CDC-ACM or Stage 1 Sim pipe).
#[derive(Default, Debug)]
pub struct HostLinkBridge {
    pub rx_decode_errors: u64,
    pub rx_crc_errors: u16,
    pub rx_version_mismatches: u16,
    pub tx_frames: u64,
}

impl HostLinkBridge {
    pub fn decode_rx(&mut self, wire_frame: &mut [u8]) -> Option<HostToMcu> {
        match decode_host_cobs(wire_frame) {
            Ok(pkt) => Some(pkt),
            Err(err) => {
                self.rx_decode_errors = self.rx_decode_errors.saturating_add(1);
                match err {
                    CodecError::CrcMismatch => {
                        self.rx_crc_errors = self.rx_crc_errors.saturating_add(1);
                    }
                    CodecError::VersionMismatch => {
                        self.rx_version_mismatches = self.rx_version_mismatches.saturating_add(1);
                    }
                    _ => {}
                }
                None
            }
        }
    }

    pub fn encode_tx<'a>(
        &mut self,
        telem: &McuToHost,
        buf: &'a mut [u8; MAX_COBS_FRAME_BYTES],
    ) -> Result<&'a [u8], CodecError> {
        let encoded = encode_cobs(telem, buf)?;
        self.tx_frames = self.tx_frames.saturating_add(1);
        Ok(encoded)
    }
}

/// Servo bus bridge abstraction (Stage 1: `SimulatedServoPlant`, Stage 2: RP2350 PIO UART / CAN).
pub trait ServoBusDriver {
    fn read_feedback(&mut self, dt_sec: f32) -> RawServoFeedback;
    fn write_command(&mut self, cmd: &RawServoCommand);
}
