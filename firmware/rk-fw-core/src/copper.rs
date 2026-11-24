//! Plain control-loop message structs and a no-alloc `ControlTask` helper for Rocky MCU firmware.
//!
//! All scheduling, timing, USB, IMU, and watchdog work is handled by Embassy on the board crates.
//! This module intentionally contains no copper-rs (`cu29`) glue: it only defines the message
//! structs exchanged between tasks and [`ControlTask`], a thin helper that calls
//! [`DirectServoKernel::step`] directly and publishes the status LED mode.
//!
//! All safety invariants live inside `rk_control::DirectServoKernel` (`f32` only):
//! - Boots strictly in `Teach` mode (`torque_mask = 0`)
//! - 100 ms host deadman watchdog forces `EStop`
//! - Per-joint soft stops & slew-rate velocity clamping
//! - `ConfigCmd::ClearFaults` resets strictly to `Teach` (never directly to `Drive`)

use core::cell::RefCell;

use critical_section::Mutex;
use rk_control::{DirectServoKernel, JointCalibration, KernelStepOutput, RawServoCommand, RawServoFeedback};
use rk_proto::{
    ConfigCmd, ControlMode, EstimatorTelemetry, FaultFlags, HostToMcu, MAX_JOINTS, McuToHost,
    PROTOCOL_VERSION,
};
use serde::{Deserialize, Serialize};

use crate::status_led::StatusLedMode;
use crate::transport::HOST_LINK_MAILBOX;

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
// Plain Message Types
// ---------------------------------------------------------------------------

/// Decoded host command delivered into the control step.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct HostCmdMsg {
    pub present: bool,
    pub protocol_version: u16,
    pub seq: u32,
    /// `0 = Teach`, `1 = Drive`, `2 = EStop`.
    pub mode: u8,
    pub torque_mask: u32,
    pub speed_limit_pct: u8,
    pub target_rad: [f32; MAX_JOINTS],
    pub command_mode: [u8; MAX_JOINTS],
    pub velocity_rad_s: [f32; MAX_JOINTS],
    pub torque_limit_nm: [f32; MAX_JOINTS],
    // High-level walking & balance command
    pub walk_enabled: bool,
    pub balance_enabled: bool,
    pub walk_vx: f32,
    pub walk_vy: f32,
    pub walk_yaw: f32,
    pub walk_crouch: f32,
    pub walk_step_h: f32,
    pub walk_cadence_hz: f32,
    /// Encoded `ConfigCmd` tag (`0=None`, `1=SetZero`, `2=SetLimits`, `3=SetInvert`, `4=ClearFaults`, `5=CommitFlash`, `6=SetBalanceGains`).
    pub cfg_tag: u8,
    pub cfg_joint: u8,
    pub cfg_f: [f32; 13],
    pub cfg_bool: bool,
}

impl HostCmdMsg {
    pub fn from_host(cmd: &HostToMcu) -> Self {
        let mode = match cmd.mode {
            ControlMode::Teach => 0,
            ControlMode::Drive => 1,
            ControlMode::EStop => 2,
        };
        let mut cfg_f = [0.0_f32; 13];
        let (cfg_tag, cfg_joint, cfg_bool) = match cmd.config {
            ConfigCmd::None => (0, 0, false),
            ConfigCmd::SetZeroOffset { joint, zero_rad } => {
                cfg_f[0] = zero_rad;
                (1, joint, false)
            }
            ConfigCmd::SetSoftLimits {
                joint,
                min_rad,
                max_rad,
            } => {
                cfg_f[0] = min_rad;
                cfg_f[1] = max_rad;
                (2, joint, false)
            }
            ConfigCmd::SetInvert { joint, inverted } => (3, joint, inverted),
            ConfigCmd::ClearFaults => (4, 0, false),
            ConfigCmd::CommitFlash => (5, 0, false),
            ConfigCmd::SetBalanceGains(g) => {
                cfg_f = [
                    g.uvc_gain_pitch,
                    g.uvc_gain_roll,
                    g.tilt_deadband_rad,
                    g.gyro_damp_pitch,
                    g.gyro_damp_roll,
                    g.arm_assist,
                    g.fall_cutoff_rad,
                    g.dcm_step_gain,
                    g.foot_half_length_m,
                    g.foot_margin_m,
                    g.ukemi_tilt_rad,
                    g.ukemi_torque_scale,
                    g.compliance_scale,
                ];
                (6, 0, false)
            }
        };
        Self {
            present: true,
            protocol_version: cmd.protocol_version,
            seq: cmd.seq,
            mode,
            torque_mask: cmd.torque_mask,
            speed_limit_pct: cmd.speed_limit_pct,
            target_rad: cmd.target_rad,
            command_mode: cmd.command_mode,
            velocity_rad_s: cmd.velocity_rad_s,
            torque_limit_nm: cmd.torque_limit_nm,
            walk_enabled: cmd.walk.walk_enabled,
            balance_enabled: cmd.walk.balance_enabled,
            walk_vx: cmd.walk.vx,
            walk_vy: cmd.walk.vy,
            walk_yaw: cmd.walk.yaw_rate,
            walk_crouch: cmd.walk.stance_crouch,
            walk_step_h: cmd.walk.step_height,
            walk_cadence_hz: cmd.walk.cadence_hz,
            cfg_tag,
            cfg_joint,
            cfg_f,
            cfg_bool,
        }
    }

    pub fn to_host(&self) -> Option<HostToMcu> {
        if !self.present {
            return None;
        }
        let mode = match self.mode {
            0 => ControlMode::Teach,
            1 => ControlMode::Drive,
            _ => ControlMode::EStop,
        };
        let config = match self.cfg_tag {
            1 => ConfigCmd::SetZeroOffset {
                joint: self.cfg_joint,
                zero_rad: self.cfg_f[0],
            },
            2 => ConfigCmd::SetSoftLimits {
                joint: self.cfg_joint,
                min_rad: self.cfg_f[0],
                max_rad: self.cfg_f[1],
            },
            3 => ConfigCmd::SetInvert {
                joint: self.cfg_joint,
                inverted: self.cfg_bool,
            },
            4 => ConfigCmd::ClearFaults,
            5 => ConfigCmd::CommitFlash,
            6 => ConfigCmd::SetBalanceGains(rk_proto::BalanceGains {
                uvc_gain_pitch: self.cfg_f[0],
                uvc_gain_roll: self.cfg_f[1],
                tilt_deadband_rad: self.cfg_f[2],
                gyro_damp_pitch: self.cfg_f[3],
                gyro_damp_roll: self.cfg_f[4],
                arm_assist: self.cfg_f[5],
                fall_cutoff_rad: self.cfg_f[6],
                dcm_step_gain: self.cfg_f[7],
                foot_half_length_m: self.cfg_f[8],
                foot_margin_m: self.cfg_f[9],
                ukemi_tilt_rad: self.cfg_f[10],
                ukemi_torque_scale: self.cfg_f[11],
                compliance_scale: self.cfg_f[12],
            }),
            _ => ConfigCmd::None,
        };
        Some(HostToMcu {
            protocol_version: self.protocol_version,
            seq: self.seq,
            mode,
            torque_mask: self.torque_mask,
            speed_limit_pct: self.speed_limit_pct,
            target_rad: self.target_rad,
            command_mode: self.command_mode,
            velocity_rad_s: self.velocity_rad_s,
            torque_limit_nm: self.torque_limit_nm,
            walk: rk_proto::WalkCmd {
                walk_enabled: self.walk_enabled,
                balance_enabled: self.balance_enabled,
                vx: self.walk_vx,
                vy: self.walk_vy,
                yaw_rate: self.walk_yaw,
                stance_crouch: self.walk_crouch,
                step_height: self.walk_step_h,
                cadence_hz: self.walk_cadence_hz,
            },
            config,
        })
    }
}

/// Raw servo bus + IMU sensor feedback delivered into the control step.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ServoFeedbackMsg {
    pub raw_rad: [f32; MAX_JOINTS],
    pub velocity_rad_s: [f32; MAX_JOINTS],
    pub current_ma: [u16; MAX_JOINTS],
    pub temp_c: [u8; MAX_JOINTS],
    pub bus_voltage_mv: u16,
    pub bus_error: bool,
    pub i3c_timeout_count: u16,
    pub imu_source_mask: u32,
    pub imu_available: bool,
    pub imu_pitch_rad: f32,
    pub imu_roll_rad: f32,
    pub imu_yaw_rad: f32,
    pub imu_pitch_rate_rad_s: f32,
    pub imu_roll_rate_rad_s: f32,
    pub imu_yaw_rate_rad_s: f32,
    pub imu_gravity_x: f32,
    pub imu_gravity_y: f32,
    pub imu_gravity_z: f32,
}

impl From<RawServoFeedback> for ServoFeedbackMsg {
    fn from(fb: RawServoFeedback) -> Self {
        Self {
            raw_rad: fb.raw_angle_rad,
            velocity_rad_s: fb.velocity_rad_s,
            current_ma: fb.current_ma,
            temp_c: fb.temp_c,
            bus_voltage_mv: fb.bus_voltage_mv,
            bus_error: fb.bus_error,
            i3c_timeout_count: fb.i3c_timeout_count,
            imu_source_mask: fb.imu_source_mask,
            imu_available: fb.imu.available,
            imu_pitch_rad: fb.imu.pitch_rad,
            imu_roll_rad: fb.imu.roll_rad,
            imu_yaw_rad: fb.imu.yaw_rad,
            imu_pitch_rate_rad_s: fb.imu.pitch_rate_rad_s,
            imu_roll_rate_rad_s: fb.imu.roll_rate_rad_s,
            imu_yaw_rate_rad_s: fb.imu.yaw_rate_rad_s,
            imu_gravity_x: fb.imu.gravity[0],
            imu_gravity_y: fb.imu.gravity[1],
            imu_gravity_z: fb.imu.gravity[2],
        }
    }
}

impl From<&ServoFeedbackMsg> for RawServoFeedback {
    fn from(msg: &ServoFeedbackMsg) -> Self {
        Self {
            raw_angle_rad: msg.raw_rad,
            velocity_rad_s: msg.velocity_rad_s,
            current_ma: msg.current_ma,
            temp_c: msg.temp_c,
            bus_voltage_mv: msg.bus_voltage_mv,
            bus_error: msg.bus_error,
            imu: rk_proto::ImuState {
                available: msg.imu_available,
                pitch_rad: msg.imu_pitch_rad,
                roll_rad: msg.imu_roll_rad,
                yaw_rad: msg.imu_yaw_rad,
                pitch_rate_rad_s: msg.imu_pitch_rate_rad_s,
                roll_rate_rad_s: msg.imu_roll_rate_rad_s,
                yaw_rate_rad_s: msg.imu_yaw_rate_rad_s,
                gravity: [msg.imu_gravity_x, msg.imu_gravity_y, msg.imu_gravity_z],
            },
            i3c_timeout_count: msg.i3c_timeout_count,
            imu_source_mask: msg.imu_source_mask,
        }
    }
}

/// Atomic control-step output, fanned out to the servo bus and host link by the board tasks.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ControlStepMsg {
    // Outbound actuator command
    pub cmd_torque_mask: u32,
    pub cmd_raw_target_rad: [f32; MAX_JOINTS],
    pub cmd_mode: [u8; MAX_JOINTS],
    pub cmd_raw_velocity_rad_s: [f32; MAX_JOINTS],
    pub cmd_torque_nm: [f32; MAX_JOINTS],

    // Outbound host telemetry
    pub telem_seq_ack: u32,
    pub telem_uptime_ms: u32,
    pub telem_active_mode: u8,
    pub telem_faults_bits: u16,
    pub telem_torque_state_mask: u32,
    pub telem_bus_voltage_mv: u16,
    pub telem_measured_rad: [f32; MAX_JOINTS],
    pub telem_applied_target_rad: [f32; MAX_JOINTS],
    pub telem_velocity_rad_s: [f32; MAX_JOINTS],
    pub telem_current_ma: [u16; MAX_JOINTS],
    pub telem_temp_c: [u8; MAX_JOINTS],
    pub telem_imu_available: bool,
    pub telem_imu_pitch_rad: f32,
    pub telem_imu_roll_rad: f32,
    pub telem_imu_yaw_rad: f32,
    pub telem_imu_pitch_rate: f32,
    pub telem_imu_roll_rate: f32,
    pub telem_imu_yaw_rate: f32,
    pub telem_imu_gravity_x: f32,
    pub telem_imu_gravity_y: f32,
    pub telem_imu_gravity_z: f32,
    pub telem_gait_phase_rad: f32,
    pub telem_pitch_corr_rad: f32,
    pub telem_roll_corr_rad: f32,
    pub telem_balance_active: bool,
    pub telem_walk_active: bool,
    pub telem_uvc_phase: u8,
    pub telem_support_foot: u8,
    pub telem_foot_dx_m: f32,
    pub telem_foot_dy_m: f32,
    pub telem_hip_height_m: f32,
    /// `fall_orientation::*` wire code captured with `TILT_FALL_CUTOFF`.
    pub telem_fall_orientation: u8,
    // On-MCU kinematic CoM / DCM / ZMP estimator telemetry
    pub est_com_pos_m: [f32; 3],
    pub est_com_vel_m_s: [f32; 3],
    pub est_dcm_xy_m: [f32; 2],
    pub est_zmp_xy_m: [f32; 2],
    pub est_foot_margin_m: f32,
    pub est_grf_left_n: f32,
    pub est_grf_right_n: f32,
    pub est_contact_mask: u8,
    pub est_ukemi_active: bool,
    pub est_valid_mask: u32,
    // Hardened wire & bus diagnostic counters
    pub telem_rx_crc_errors: u16,
    pub telem_rx_version_mismatches: u16,
    pub telem_i3c_timeout_count: u16,
}

impl ControlStepMsg {
    pub fn to_servo_cmd(&self) -> RawServoCommand {
        RawServoCommand {
            torque_mask: self.cmd_torque_mask,
            raw_target_rad: self.cmd_raw_target_rad,
            command_mode: self.cmd_mode,
            raw_velocity_rad_s: self.cmd_raw_velocity_rad_s,
            torque_nm: self.cmd_torque_nm,
        }
    }

    pub fn to_telemetry(&self) -> McuToHost {
        let active_mode = match self.telem_active_mode {
            0 => ControlMode::Teach,
            1 => ControlMode::Drive,
            _ => ControlMode::EStop,
        };
        McuToHost {
            protocol_version: PROTOCOL_VERSION,
            seq_ack: self.telem_seq_ack,
            uptime_ms: self.telem_uptime_ms,
            active_mode,
            faults: FaultFlags::from_bits_truncate(self.telem_faults_bits),
            torque_state_mask: self.telem_torque_state_mask,
            bus_voltage_mv: self.telem_bus_voltage_mv,
            measured_rad: self.telem_measured_rad,
            applied_target_rad: self.telem_applied_target_rad,
            velocity_rad_s: self.telem_velocity_rad_s,
            current_ma: self.telem_current_ma,
            temp_c: self.telem_temp_c,
            imu: rk_proto::ImuState {
                available: self.telem_imu_available,
                pitch_rad: self.telem_imu_pitch_rad,
                roll_rad: self.telem_imu_roll_rad,
                yaw_rad: self.telem_imu_yaw_rad,
                pitch_rate_rad_s: self.telem_imu_pitch_rate,
                roll_rate_rad_s: self.telem_imu_roll_rate,
                yaw_rate_rad_s: self.telem_imu_yaw_rate,
                gravity: [
                    self.telem_imu_gravity_x,
                    self.telem_imu_gravity_y,
                    self.telem_imu_gravity_z,
                ],
            },
            balance: rk_proto::BalanceTelemetry {
                gait_phase_rad: self.telem_gait_phase_rad,
                pitch_corr_rad: self.telem_pitch_corr_rad,
                roll_corr_rad: self.telem_roll_corr_rad,
                balance_active: self.telem_balance_active,
                walk_active: self.telem_walk_active,
                uvc_phase: self.telem_uvc_phase,
                support_foot: self.telem_support_foot,
                foot_dx_m: self.telem_foot_dx_m,
                foot_dy_m: self.telem_foot_dy_m,
                hip_height_m: self.telem_hip_height_m,
                fall_orientation: self.telem_fall_orientation,
            },
            estimator: EstimatorTelemetry {
                com_pos_m: self.est_com_pos_m,
                com_vel_m_s: self.est_com_vel_m_s,
                dcm_xy_m: self.est_dcm_xy_m,
                zmp_xy_m: self.est_zmp_xy_m,
                foot_margin_m: self.est_foot_margin_m,
                grf_left_n: self.est_grf_left_n,
                grf_right_n: self.est_grf_right_n,
                contact_mask: self.est_contact_mask,
                ukemi_active: self.est_ukemi_active,
                valid_mask: self.est_valid_mask,
            },
            rx_crc_errors: self.telem_rx_crc_errors,
            rx_version_mismatches: self.telem_rx_version_mismatches,
            i3c_timeout_count: self.telem_i3c_timeout_count,
        }
    }
}

// ---------------------------------------------------------------------------
// Control Task: plain `DirectServoKernel` wrapper
// ---------------------------------------------------------------------------

/// Owns a [`DirectServoKernel`] and drives one control tick per [`Self::step`] call.
///
/// This replaces the former copper-rs `DirectServoCuTask`. It performs no I/O itself:
/// the Embassy board tasks supply `now_ms`, `dt_sec`, the optional host command, and the
/// current servo feedback snapshot.
pub struct ControlTask {
    kernel: DirectServoKernel,
}

impl ControlTask {
    pub fn new(joint_count: usize, initial_cal: &[JointCalibration]) -> Self {
        Self {
            kernel: DirectServoKernel::new(joint_count, initial_cal),
        }
    }

    /// Runs one deterministic control tick and returns the kernel's raw output.
    ///
    /// Equivalent to the former `DirectServoCuTask::process()`: wire error counters are
    /// refreshed from the host-link mailbox, the kernel is stepped with the supplied
    /// timing and I/O snapshots, and the status LED mode is updated from telemetry.
    pub fn step(
        &mut self,
        now_ms: u32,
        dt_sec: f32,
        cmd: Option<&HostToMcu>,
        fb: &RawServoFeedback,
    ) -> KernelStepOutput {
        let (crc_errs, ver_errs) = HOST_LINK_MAILBOX.wire_error_counts();
        self.kernel.set_wire_error_counts(crc_errs, ver_errs);

        let step = self.kernel.step(now_ms, dt_sec, cmd, fb);
        update_led_from_state(step.telemetry.active_mode, step.telemetry.faults);
        step
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// RP2350 stack guardrail: verifies the 3 `Copy` message structs stay compact
    /// (< 2 KiB total across all three) after adding P1a velocity, torque, and
    /// estimator fields.
    #[test]
    fn message_sizes_fit_rp2350_stack_budget() {
        let host_sz = core::mem::size_of::<HostCmdMsg>();
        let fb_sz = core::mem::size_of::<ServoFeedbackMsg>();
        let step_sz = core::mem::size_of::<ControlStepMsg>();
        let total = host_sz + fb_sz + step_sz;
        assert!(host_sz <= 512, "HostCmdMsg too large for RP2350 stack: {host_sz} B");
        assert!(fb_sz <= 384, "ServoFeedbackMsg too large for RP2350 stack: {fb_sz} B");
        assert!(step_sz <= 1024, "ControlStepMsg too large for RP2350 stack: {step_sz} B");
        assert!(total <= 1792, "Combined message structs exceed 1792 B budget: {total} B");
    }

    /// `ControlTask::step()` must boot into `Teach` mode and never panic on default inputs.
    #[test]
    fn control_task_step_defaults_to_teach() {
        let cal = [JointCalibration::default(); MAX_JOINTS];
        let mut task = ControlTask::new(MAX_JOINTS, &cal);
        let cmd = HostToMcu::default();
        let fb = RawServoFeedback::default();

        let out = task.step(0, crate::config::CONTROL_DT_SEC, Some(&cmd), &fb);
        assert_eq!(out.telemetry.active_mode, ControlMode::Teach);
        assert_eq!(current_led_mode(), StatusLedMode::TeachHeartbeat);
    }
}
