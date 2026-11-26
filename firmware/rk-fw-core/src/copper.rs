//! No-alloc [`ControlTask`] helper for the shared host+MCU control application.
//!
//! All scheduling, timing, USB, IMU, and watchdog work is handled by Embassy on the
//! board crates, or by the fixed-dt harness in simulation. This module intentionally
//! contains no copper-rs (`cu29`) glue: [`ControlTask`] is a thin, deterministic,
//! host-callable wrapper that calls [`DirectServoKernel::step`] directly with **all**
//! inputs supplied explicitly by the caller.
//!
//! All safety invariants live inside `rk_control::DirectServoKernel` (`f32` only):
//! - Boots strictly in `Teach` mode (`torque_mask = 0`)
//! - 100 ms host deadman watchdog forces `EStop`
//! - Per-joint soft stops & slew-rate velocity clamping
//! - `ConfigCmd::ClearFaults` resets strictly to `Teach` (never directly to `Drive`)

use core::cell::RefCell;

use critical_section::Mutex;
use rk_control::{
    DirectServoKernel, JointCalibration, KernelStepOutput, RawServoCommand, RawServoFeedback,
};
use rk_proto::{ControlMode, FaultFlags, HostToMcu, McuToHost};
use serde::{Deserialize, Serialize};

use crate::status_led::StatusLedMode;

/// Latest firmware safety snapshot published by [`ControlTask::step`] for the board status LED.
static LATEST_LED_MODE: Mutex<RefCell<StatusLedMode>> =
    Mutex::new(RefCell::new(StatusLedMode::TeachHeartbeat));

/// Returns the live status LED pattern computed from the most recent control tick.
pub fn current_led_mode() -> StatusLedMode {
    critical_section::with(|cs| *LATEST_LED_MODE.borrow_ref(cs))
}

/// Updates the cached status LED mode from the kernel's safety snapshot.
fn update_led_from_state(mode: ControlMode, faults: FaultFlags) {
    let led_mode = StatusLedMode::from_state(mode, faults);
    critical_section::with(|cs| {
        *LATEST_LED_MODE.borrow_ref_mut(cs) = led_mode;
    });
}

// ---------------------------------------------------------------------------
// Control Task: plain `DirectServoKernel` wrapper
// ---------------------------------------------------------------------------

/// One deterministic control tick snapshot for replay/telemetry recording.
///
/// This is a plain value type: the caller owns every tick record explicitly, so
/// [`ControlTask`] keeps no hidden recording state. It captures the exact inputs
/// and outputs of a [`ControlTask::step_recorded`] call.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub struct ControlTick {
    pub tick_index: u64,
    pub now_ms: u32,
    pub dt_sec: f32,
    pub rx_cmd: Option<HostToMcu>,
    pub rx_state: RawServoFeedback,
    pub wire_errors: (u16, u16),
    pub tx_cmd: RawServoCommand,
    pub tx_telem: McuToHost,
}

/// Owns a [`DirectServoKernel`] and drives one control tick per [`Self::step`] call.
///
/// The wrapper performs no I/O itself and reads no global mailbox: the caller supplies
/// `now_ms`, `dt_sec`, the optional host command, the servo feedback snapshot, and the
/// wire error counts. Identical inputs therefore produce identical outputs on the host
/// and on the MCU.
pub struct ControlTask {
    kernel: DirectServoKernel,
    tick_index: u64,
}

impl ControlTask {
    pub fn new(joint_count: usize, initial_cal: &[JointCalibration]) -> Self {
        Self {
            kernel: DirectServoKernel::new(joint_count, initial_cal),
            tick_index: 0,
        }
    }

    pub fn new_with_profile(
        profile: &'static rk_robot_profile::RobotProfile,
        initial_cal: &[JointCalibration],
    ) -> Self {
        Self {
            kernel: DirectServoKernel::new_with_profile(profile, initial_cal),
            tick_index: 0,
        }
    }

    /// Borrows the underlying kernel (e.g. for configuration after construction).
    pub fn kernel(&self) -> &DirectServoKernel {
        &self.kernel
    }

    /// Mutably borrows the underlying kernel (e.g. for configuration after construction).
    pub fn kernel_mut(&mut self) -> &mut DirectServoKernel {
        &mut self.kernel
    }

    /// Number of control ticks executed so far by this instance.
    pub fn tick_index(&self) -> u64 {
        self.tick_index
    }

    /// Runs one deterministic control tick and returns the kernel's raw output.
    ///
    /// `wire_errors` is `(rx_crc_errors, rx_version_mismatches)` from the transport
    /// layer. It is an explicit parameter so the step is pure and replay-safe: firmware
    /// passes the live host-link mailbox counts, simulation passes its bridge counts (or
    /// zero). The status LED mode is updated from the kernel's safety snapshot.
    pub fn step(
        &mut self,
        now_ms: u32,
        dt_sec: f32,
        cmd: Option<&HostToMcu>,
        fb: &RawServoFeedback,
        wire_errors: (u16, u16),
    ) -> KernelStepOutput {
        self.kernel
            .set_wire_error_counts(wire_errors.0, wire_errors.1);

        let step = self.kernel.step(now_ms, dt_sec, cmd, fb);
        update_led_from_state(step.telemetry.active_mode, step.telemetry.faults);
        self.tick_index = self.tick_index.saturating_add(1);
        step
    }

    /// Runs one control tick and additionally returns an explicit [`ControlTick`] record.
    ///
    /// Use this in recording/replay harnesses. The caller owns the returned snapshot;
    /// no hidden global state is involved.
    pub fn step_recorded(
        &mut self,
        now_ms: u32,
        dt_sec: f32,
        cmd: Option<&HostToMcu>,
        fb: &RawServoFeedback,
        wire_errors: (u16, u16),
    ) -> (KernelStepOutput, ControlTick) {
        let tick_index = self.tick_index;
        let step = self.step(now_ms, dt_sec, cmd, fb, wire_errors);
        let record = ControlTick {
            tick_index,
            now_ms,
            dt_sec,
            rx_cmd: cmd.copied(),
            rx_state: *fb,
            wire_errors,
            tx_cmd: step.servo_cmd,
            tx_telem: step.telemetry,
        };
        (step, record)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rk_proto::MAX_JOINTS;

    /// `ControlTask::step()` must boot into `Teach` mode and never panic on default inputs.
    #[test]
    fn control_task_step_defaults_to_teach() {
        let cal = [JointCalibration::default(); MAX_JOINTS];
        let mut task = ControlTask::new(MAX_JOINTS, &cal);
        let cmd = HostToMcu::default();
        let fb = RawServoFeedback::default();

        let out = task.step(
            0,
            crate::config::CONTROL_DT_SEC,
            Some(&cmd),
            &fb,
            (0, 0),
        );
        assert_eq!(out.telemetry.active_mode, ControlMode::Teach);
        assert_eq!(current_led_mode(), StatusLedMode::TeachHeartbeat);
        assert_eq!(task.tick_index(), 1);
    }

    /// Identical explicit inputs (including wire errors) produce identical outputs,
    /// and the wire error counts are reflected in telemetry.
    #[test]
    fn step_is_pure_wrt_explicit_wire_error_inputs() {
        let cal = [JointCalibration::default(); MAX_JOINTS];
        let cmd = HostToMcu::default();
        let fb = RawServoFeedback::default();

        let mut a = ControlTask::new(MAX_JOINTS, &cal);
        let mut b = ControlTask::new(MAX_JOINTS, &cal);
        let out_a = a.step(100, crate::config::CONTROL_DT_SEC, Some(&cmd), &fb, (3, 5));
        let out_b = b.step(100, crate::config::CONTROL_DT_SEC, Some(&cmd), &fb, (3, 5));
        assert_eq!(out_a.telemetry, out_b.telemetry);
        assert_eq!(out_a.servo_cmd, out_b.servo_cmd);
        assert_eq!(out_a.telemetry.rx_crc_errors, 3);
        assert_eq!(out_a.telemetry.rx_version_mismatches, 5);
    }

    /// `step_recorded` returns a full snapshot and advances the tick index.
    #[test]
    fn step_recorded_captures_inputs_and_outputs() {
        let cal = [JointCalibration::default(); MAX_JOINTS];
        let mut task = ControlTask::new(MAX_JOINTS, &cal);
        let cmd = HostToMcu::default();
        let fb = RawServoFeedback::default();

        let (out, tick) = task.step_recorded(
            250,
            crate::config::CONTROL_DT_SEC,
            Some(&cmd),
            &fb,
            (1, 2),
        );
        assert_eq!(tick.tick_index, 0);
        assert_eq!(tick.now_ms, 250);
        assert_eq!(tick.rx_cmd, Some(cmd));
        assert_eq!(tick.rx_state, fb);
        assert_eq!(tick.wire_errors, (1, 2));
        assert_eq!(tick.tx_cmd, out.servo_cmd);
        assert_eq!(tick.tx_telem, out.telemetry);
        assert_eq!(task.tick_index(), 1);
    }
}
