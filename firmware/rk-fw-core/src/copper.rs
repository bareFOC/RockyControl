//! Real `copper-rs` (`cu29 1.3`) `no_std` tasks, bridges, and messages for Rocky MCU firmware.
//!
//! Graph topology (defined identically in `copperconfig.ron` for RP2350 and ESP32-P4):
//!
//! ```text
//!   host_link/rx_cmd  ──(HostCmdMsg)───────┐
//!                                          ▼
//!                                   [servo_ctrl] ──(ControlStepMsg)──┬──▶ servo_bus/tx_cmd
//!                                          ▲                         └──▶ host_link/tx_telem
//!   servo_bus/rx_state ─(ServoFeedbackMsg)─┘
//! ```
//!
//! All safety invariants live inside `rk_control::DirectServoKernel` (`f32` only):
//! - Boots strictly in `Teach` mode (`torque_mask = 0`)
//! - 100 ms host deadman watchdog forces `EStop`
//! - Per-joint soft stops & slew-rate velocity clamping
//! - `ConfigCmd::ClearFaults` resets strictly to `Teach` (never directly to `Drive`)

use core::cell::RefCell;

use bincode::{Decode, Encode};
use critical_section::Mutex;
use cu29::prelude::*;
use rk_control::{DirectServoKernel, JointCalibration, RawServoCommand, RawServoFeedback};
use rk_proto::{ConfigCmd, ControlMode, FaultFlags, HostToMcu, MAX_JOINTS, McuToHost};
use serde::{Deserialize, Serialize};

use crate::config::{CONTROL_DT_SEC, DEFAULT_JOINT_COUNT};
use crate::servo_bus::SERVO_BUS_SLOT;
use crate::status_led::StatusLedMode;
use crate::transport::HOST_LINK_MAILBOX;

/// Latest firmware safety snapshot published by `DirectServoCuTask` for the board status LED.
static LATEST_LED_MODE: Mutex<RefCell<StatusLedMode>> =
    Mutex::new(RefCell::new(StatusLedMode::TeachHeartbeat));

/// Returns the live status LED pattern computed from the most recent `copper-rs` control tick.
pub fn current_led_mode() -> StatusLedMode {
    critical_section::with(|cs| *LATEST_LED_MODE.borrow_ref(cs))
}

// ---------------------------------------------------------------------------
// Copper-rs Message Types (`Encode + Decode + Reflect + Serialize + Deserialize`)
// ---------------------------------------------------------------------------

/// Decoded host command delivered from `host_link/rx_cmd` into `servo_ctrl`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
pub struct HostCmdMsg {
    pub present: bool,
    pub seq: u32,
    /// `0 = Teach`, `1 = Drive`, `2 = EStop`.
    pub mode: u8,
    pub torque_mask: u32,
    pub speed_limit_pct: u8,
    pub target_rad: [f32; MAX_JOINTS],
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
    pub cfg_f0: f32,
    pub cfg_f1: f32,
    pub cfg_f2: f32,
    pub cfg_f3: f32,
    pub cfg_f4: f32,
    pub cfg_f5: f32,
    pub cfg_f6: f32,
    pub cfg_bool: bool,
}

impl HostCmdMsg {
    pub fn from_host(cmd: &HostToMcu) -> Self {
        let mode = match cmd.mode {
            ControlMode::Teach => 0,
            ControlMode::Drive => 1,
            ControlMode::EStop => 2,
        };
        let (cfg_tag, cfg_joint, cfg_f0, cfg_f1, cfg_f2, cfg_f3, cfg_f4, cfg_f5, cfg_f6, cfg_bool) =
            match cmd.config {
                ConfigCmd::None => (0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false),
                ConfigCmd::SetZeroOffset { joint, zero_rad } => {
                    (1, joint, zero_rad, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false)
                }
                ConfigCmd::SetSoftLimits {
                    joint,
                    min_rad,
                    max_rad,
                } => (2, joint, min_rad, max_rad, 0.0, 0.0, 0.0, 0.0, 0.0, false),
                ConfigCmd::SetInvert { joint, inverted } => {
                    (3, joint, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, inverted)
                }
                ConfigCmd::ClearFaults => (4, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false),
                ConfigCmd::CommitFlash => (5, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false),
                ConfigCmd::SetBalanceGains(g) => (
                    6,
                    0,
                    g.kp_pitch,
                    g.kd_pitch,
                    g.kp_roll,
                    g.kd_roll,
                    g.ankle_ratio,
                    g.arm_assist,
                    g.fall_cutoff_rad,
                    false,
                ),
            };
        Self {
            present: true,
            seq: cmd.seq,
            mode,
            torque_mask: cmd.torque_mask,
            speed_limit_pct: cmd.speed_limit_pct,
            target_rad: cmd.target_rad,
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
            cfg_f0,
            cfg_f1,
            cfg_f2,
            cfg_f3,
            cfg_f4,
            cfg_f5,
            cfg_f6,
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
                zero_rad: self.cfg_f0,
            },
            2 => ConfigCmd::SetSoftLimits {
                joint: self.cfg_joint,
                min_rad: self.cfg_f0,
                max_rad: self.cfg_f1,
            },
            3 => ConfigCmd::SetInvert {
                joint: self.cfg_joint,
                inverted: self.cfg_bool,
            },
            4 => ConfigCmd::ClearFaults,
            5 => ConfigCmd::CommitFlash,
            6 => ConfigCmd::SetBalanceGains(rk_proto::BalanceGains {
                kp_pitch: self.cfg_f0,
                kd_pitch: self.cfg_f1,
                kp_roll: self.cfg_f2,
                kd_roll: self.cfg_f3,
                ankle_ratio: self.cfg_f4,
                arm_assist: self.cfg_f5,
                fall_cutoff_rad: self.cfg_f6,
            }),
            _ => ConfigCmd::None,
        };
        Some(HostToMcu {
            seq: self.seq,
            mode,
            torque_mask: self.torque_mask,
            speed_limit_pct: self.speed_limit_pct,
            target_rad: self.target_rad,
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

/// Raw servo bus + IMU sensor feedback delivered from `servo_bus/rx_state` into `servo_ctrl`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
pub struct ServoFeedbackMsg {
    pub raw_rad: [f32; MAX_JOINTS],
    pub current_ma: [u16; MAX_JOINTS],
    pub temp_c: [u8; MAX_JOINTS],
    pub bus_voltage_mv: u16,
    pub bus_error: bool,
    pub imu_pitch_rad: f32,
    pub imu_roll_rad: f32,
    pub imu_pitch_rate_rad_s: f32,
    pub imu_roll_rate_rad_s: f32,
}

impl From<RawServoFeedback> for ServoFeedbackMsg {
    fn from(fb: RawServoFeedback) -> Self {
        Self {
            raw_rad: fb.raw_angle_rad,
            current_ma: fb.current_ma,
            temp_c: fb.temp_c,
            bus_voltage_mv: fb.bus_voltage_mv,
            bus_error: fb.bus_error,
            imu_pitch_rad: fb.imu.pitch_rad,
            imu_roll_rad: fb.imu.roll_rad,
            imu_pitch_rate_rad_s: fb.imu.pitch_rate_rad_s,
            imu_roll_rate_rad_s: fb.imu.roll_rate_rad_s,
        }
    }
}

impl From<&ServoFeedbackMsg> for RawServoFeedback {
    fn from(msg: &ServoFeedbackMsg) -> Self {
        Self {
            raw_angle_rad: msg.raw_rad,
            current_ma: msg.current_ma,
            temp_c: msg.temp_c,
            bus_voltage_mv: msg.bus_voltage_mv,
            bus_error: msg.bus_error,
            imu: rk_proto::ImuState {
                pitch_rad: msg.imu_pitch_rad,
                roll_rad: msg.imu_roll_rad,
                pitch_rate_rad_s: msg.imu_pitch_rate_rad_s,
                roll_rate_rad_s: msg.imu_roll_rate_rad_s,
            },
        }
    }
}

/// Atomic output emitted by `DirectServoCuTask` and fanned out to both `servo_bus` and `host_link`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize, Encode, Decode, Reflect)]
pub struct ControlStepMsg {
    // Outbound actuator command
    pub cmd_torque_mask: u32,
    pub cmd_raw_target_rad: [f32; MAX_JOINTS],

    // Outbound host telemetry
    pub telem_seq_ack: u32,
    pub telem_uptime_ms: u32,
    pub telem_active_mode: u8,
    pub telem_faults_bits: u16,
    pub telem_torque_state_mask: u32,
    pub telem_bus_voltage_mv: u16,
    pub telem_measured_rad: [f32; MAX_JOINTS],
    pub telem_applied_target_rad: [f32; MAX_JOINTS],
    pub telem_current_ma: [u16; MAX_JOINTS],
    pub telem_temp_c: [u8; MAX_JOINTS],
    pub telem_imu_pitch_rad: f32,
    pub telem_imu_roll_rad: f32,
    pub telem_imu_pitch_rate: f32,
    pub telem_imu_roll_rate: f32,
    pub telem_gait_phase_rad: f32,
    pub telem_pitch_corr_rad: f32,
    pub telem_roll_corr_rad: f32,
    pub telem_balance_active: bool,
    pub telem_walk_active: bool,
}

impl ControlStepMsg {
    pub fn to_servo_cmd(&self) -> RawServoCommand {
        RawServoCommand {
            torque_mask: self.cmd_torque_mask,
            raw_target_rad: self.cmd_raw_target_rad,
        }
    }

    pub fn to_telemetry(&self) -> McuToHost {
        let active_mode = match self.telem_active_mode {
            0 => ControlMode::Teach,
            1 => ControlMode::Drive,
            _ => ControlMode::EStop,
        };
        McuToHost {
            seq_ack: self.telem_seq_ack,
            uptime_ms: self.telem_uptime_ms,
            active_mode,
            faults: FaultFlags::from_bits_truncate(self.telem_faults_bits),
            torque_state_mask: self.telem_torque_state_mask,
            bus_voltage_mv: self.telem_bus_voltage_mv,
            measured_rad: self.telem_measured_rad,
            applied_target_rad: self.telem_applied_target_rad,
            current_ma: self.telem_current_ma,
            temp_c: self.telem_temp_c,
            imu: rk_proto::ImuState {
                pitch_rad: self.telem_imu_pitch_rad,
                roll_rad: self.telem_imu_roll_rad,
                pitch_rate_rad_s: self.telem_imu_pitch_rate,
                roll_rate_rad_s: self.telem_imu_roll_rate,
            },
            balance: rk_proto::BalanceTelemetry {
                gait_phase_rad: self.telem_gait_phase_rad,
                pitch_corr_rad: self.telem_pitch_corr_rad,
                roll_corr_rad: self.telem_roll_corr_rad,
                balance_active: self.telem_balance_active,
                walk_active: self.telem_walk_active,
            },
        }
    }
}

// ---------------------------------------------------------------------------
// Copper-rs Control Task: `DirectServoCuTask`
// ---------------------------------------------------------------------------

/// Deterministic `copper-rs` `CuTask` executing [`DirectServoKernel`].
#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct DirectServoCuTask {
    #[reflect(ignore)]
    kernel: DirectServoKernel,
    last_now_ms: u32,
    tick_count: u64,
}

impl Freezable for DirectServoCuTask {}

impl CuTask for DirectServoCuTask {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, HostCmdMsg, ServoFeedbackMsg);
    type Output<'m> = output_msg!(ControlStepMsg);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        let joint_count = config
            .and_then(|c| c.get::<u32>("joint_count").ok().flatten())
            .map(|n| (n as usize).clamp(1, MAX_JOINTS))
            .unwrap_or(DEFAULT_JOINT_COUNT);
        let default_cal = [JointCalibration::default(); MAX_JOINTS];
        Ok(Self {
            kernel: DirectServoKernel::new(joint_count, &default_cal[..joint_count]),
            last_now_ms: 0,
            tick_count: 0,
        })
    }

    fn process(
        &mut self,
        ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        let now_ns = ctx.now().as_nanos();
        let now_ms = (now_ns / 1_000_000) as u32;
        let dt_sec = if self.tick_count == 0 {
            CONTROL_DT_SEC
        } else {
            let delta_ms = now_ms.wrapping_sub(self.last_now_ms);
            if delta_ms == 0 {
                CONTROL_DT_SEC
            } else {
                (delta_ms as f32 * 0.001_f32).clamp(0.001_f32, 0.05_f32)
            }
        };
        self.last_now_ms = now_ms;
        self.tick_count = self.tick_count.saturating_add(1);

        let rx_cmd = input.0.payload().and_then(HostCmdMsg::to_host);
        let rx_fb: RawServoFeedback = input
            .1
            .payload()
            .map(RawServoFeedback::from)
            .unwrap_or_default();

        let step = self.kernel.step(now_ms, dt_sec, rx_cmd.as_ref(), &rx_fb);

        let led_mode = StatusLedMode::from_state(step.telemetry.active_mode, step.telemetry.faults);
        critical_section::with(|cs| {
            *LATEST_LED_MODE.borrow_ref_mut(cs) = led_mode;
        });

        let active_mode_u8 = match step.telemetry.active_mode {
            ControlMode::Teach => 0,
            ControlMode::Drive => 1,
            ControlMode::EStop => 2,
        };

        output.tov = Tov::Time(ctx.now());
        output.set_payload(ControlStepMsg {
            cmd_torque_mask: step.servo_cmd.torque_mask,
            cmd_raw_target_rad: step.servo_cmd.raw_target_rad,
            telem_seq_ack: step.telemetry.seq_ack,
            telem_uptime_ms: step.telemetry.uptime_ms,
            telem_active_mode: active_mode_u8,
            telem_faults_bits: step.telemetry.faults.bits(),
            telem_torque_state_mask: step.telemetry.torque_state_mask,
            telem_bus_voltage_mv: step.telemetry.bus_voltage_mv,
            telem_measured_rad: step.telemetry.measured_rad,
            telem_applied_target_rad: step.telemetry.applied_target_rad,
            telem_current_ma: step.telemetry.current_ma,
            telem_temp_c: step.telemetry.temp_c,
            telem_imu_pitch_rad: step.telemetry.imu.pitch_rad,
            telem_imu_roll_rad: step.telemetry.imu.roll_rad,
            telem_imu_pitch_rate: step.telemetry.imu.pitch_rate_rad_s,
            telem_imu_roll_rate: step.telemetry.imu.roll_rate_rad_s,
            telem_gait_phase_rad: step.telemetry.balance.gait_phase_rad,
            telem_pitch_corr_rad: step.telemetry.balance.pitch_corr_rad,
            telem_roll_corr_rad: step.telemetry.balance.roll_corr_rad,
            telem_balance_active: step.telemetry.balance.balance_active,
            telem_walk_active: step.telemetry.balance.walk_active,
        });

        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Copper-rs Bridge 1: `HostLinkBridge` (USB CDC-ACM or USB-NCM UDP via mailbox)
// ---------------------------------------------------------------------------

rx_channels! {
    pub struct HostLinkRxChannels : HostLinkRxId {
        rx_cmd => HostCmdMsg = "host_link/rx_cmd",
    }
}

tx_channels! {
    pub struct HostLinkTxChannels : HostLinkTxId {
        tx_telem => ControlStepMsg = "host_link/tx_telem",
    }
}

#[derive(Default, Reflect)]
pub struct HostLinkBridge;

impl Freezable for HostLinkBridge {}

impl CuBridge for HostLinkBridge {
    type Resources<'r> = ();
    type Tx = HostLinkTxChannels;
    type Rx = HostLinkRxChannels;

    fn new(
        _config: Option<&ComponentConfig>,
        _tx: &[BridgeChannelConfig<<Self::Tx as BridgeChannelSet>::Id>],
        _rx: &[BridgeChannelConfig<<Self::Rx as BridgeChannelSet>::Id>],
        _resources: Self::Resources<'_>,
    ) -> CuResult<Self> {
        Ok(Self)
    }

    fn send<'a, Payload>(
        &mut self,
        _ctx: &CuContext,
        channel: &'static BridgeChannel<HostLinkTxId, Payload>,
        msg: &CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
        if matches!(channel.id(), HostLinkTxId::TxTelem) {
            if let Some(step) = msg.payload() {
                if let Some(step_msg) = (step as &dyn core::any::Any).downcast_ref::<ControlStepMsg>() {
                    HOST_LINK_MAILBOX.publish_tx_telem(step_msg.to_telemetry());
                }
            }
        }
        Ok(())
    }

    fn receive<'a, Payload>(
        &mut self,
        ctx: &CuContext,
        channel: &'static BridgeChannel<HostLinkRxId, Payload>,
        msg: &mut CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
        msg.tov = Tov::Time(ctx.now());
        if matches!(channel.id(), HostLinkRxId::RxCmd) {
            let cmd_msg = match HOST_LINK_MAILBOX.take_rx_cmd() {
                Some(pkt) => HostCmdMsg::from_host(&pkt),
                None => HostCmdMsg::default(),
            };
            let mut slot = Some(cmd_msg);
            if let Some(target) = (&mut slot as &mut dyn core::any::Any).downcast_mut::<Option<Payload>>() {
                if let Some(val) = target.take() {
                    msg.set_payload(val);
                }
            }
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Copper-rs Bridge 2: `ServoBusBridge` (Bench loopback OR UART/PIO/TWAI bus)
// ---------------------------------------------------------------------------

rx_channels! {
    pub struct ServoBusRxChannels : ServoBusRxId {
        rx_state => ServoFeedbackMsg = "servo_bus/rx_state",
    }
}

tx_channels! {
    pub struct ServoBusTxChannels : ServoBusTxId {
        tx_cmd => ControlStepMsg = "servo_bus/tx_cmd",
    }
}

#[derive(Default, Reflect)]
pub struct ServoBusBridge;

impl Freezable for ServoBusBridge {}

impl CuBridge for ServoBusBridge {
    type Resources<'r> = ();
    type Tx = ServoBusTxChannels;
    type Rx = ServoBusRxChannels;

    fn new(
        _config: Option<&ComponentConfig>,
        _tx: &[BridgeChannelConfig<<Self::Tx as BridgeChannelSet>::Id>],
        _rx: &[BridgeChannelConfig<<Self::Rx as BridgeChannelSet>::Id>],
        _resources: Self::Resources<'_>,
    ) -> CuResult<Self> {
        Ok(Self)
    }

    fn send<'a, Payload>(
        &mut self,
        _ctx: &CuContext,
        channel: &'static BridgeChannel<ServoBusTxId, Payload>,
        msg: &CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
        if matches!(channel.id(), ServoBusTxId::TxCmd) {
            if let Some(step) = msg.payload() {
                if let Some(step_msg) = (step as &dyn core::any::Any).downcast_ref::<ControlStepMsg>() {
                    SERVO_BUS_SLOT.write_command(&step_msg.to_servo_cmd());
                }
            }
        }
        Ok(())
    }

    fn receive<'a, Payload>(
        &mut self,
        ctx: &CuContext,
        channel: &'static BridgeChannel<ServoBusRxId, Payload>,
        msg: &mut CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
        msg.tov = Tov::Time(ctx.now());
        if matches!(channel.id(), ServoBusRxId::RxState) {
            let fb = SERVO_BUS_SLOT.read_feedback(CONTROL_DT_SEC);
            let fb_msg = ServoFeedbackMsg::from(fb);
            let mut slot = Some(fb_msg);
            if let Some(target) = (&mut slot as &mut dyn core::any::Any).downcast_mut::<Option<Payload>>() {
                if let Some(val) = target.take() {
                    msg.set_payload(val);
                }
            }
        }
        Ok(())
    }
}
