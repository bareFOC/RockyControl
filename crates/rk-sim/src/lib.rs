//! Layer 3 — Software Servo Plant & Deterministic Copper Simulation Runner (`rk-sim`).
//!
//! Runs identically on Desktop and Browser (`wasm32-unknown-unknown`) with zero `std::thread`
//! or `std::time::Instant` assumptions. Records `.copper` deterministic replay logs and
//! verifies exact output parity under replay.

use core::ops::{Deref, DerefMut};
use core::ptr::NonNull;

use embassy_executor::raw;
use embassy_time::{Duration, Instant, Ticker};
use rk_control::{
    JointCalibration, RawServoCommand, RawServoFeedback, com_fk::compute_state,
    uvc::LegGeometry,
};
use rk_fw_core::control_task::{RobotIo, step_control_tick};
use rk_fw_core::copper::{ControlTask, ControlTick};
use rk_harness::DeterministicHarness;
pub use rk_fw_core::replay::{
    CopperReplayLog, ReplayVerification, verify_deterministic_replay,
};
use rk_fw_core::servo_bus::ServoBusPort;
use rk_fw_core::transport::{CobsStreamFramer, HostLinkMailbox, encode_telemetry_frame};
use rk_proto::{
    HostToMcu, JointCommandMode, MAX_COBS_FRAME_BYTES, MAX_JOINTS, MAX_MODEL_COBS_FRAME_BYTES,
    McuToHost, ModelFormat, ModelInfo, ModelRequest, ModelResponse, RunningConfig, WalkCmd,
    decode_mcu_cobs, decode_model_req_cobs, decode_model_resp_cobs, encode_cobs,
    encode_model_req_cobs, encode_model_resp_cobs, estimator_valid,
};
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;

use rk_fw_core::config::CONTROL_PERIOD_MS;

const DEFAULT_MCU_FLASH_URDF: &[u8] =
    include_bytes!("../../../assets/sample_humanoid/rocky_mk1.urdf");

// --- Torque-capable running plant constants ---------------------------------
/// Gravity used by the vertical hop integrator (m/s²).
const RUNNING_G_M_S2: f32 = 9.81;
/// Effective joint-torque -> vertical-thrust lever arm in metres (~0.05 m per research §2.1).
const RUNNING_LEVER_ARM_M: f32 = 0.05;
/// Thrust ceiling: ~4× body weight for the 1.5 kg template (N).
const RUNNING_MAX_THRUST_N: f32 = 60.0;
/// Braking (negative thrust) ceiling, allowing energy absorption during landing (N).
const RUNNING_MIN_THRUST_N: f32 = -120.0;
/// Flight detector debounce in seconds (10–15 ms per research §2.2).
const RUNNING_FLIGHT_DEBOUNCE_S: f32 = 0.012;
/// Nominal torque used to translate commanded N·m into reported motor current.
const RUNNING_NOMINAL_TORQUE_NM: f32 = 1.8;

// --- Ground-plane (world) locomotion plant constants ------------------------
/// Vertical distance pelvis root -> hip-pitch pivot from `JOINT_KINEMATICS`
/// (`l_hip_yaw.origin.y + l_hip_roll.origin.y + l_thigh.origin.y`).
const PELVIS_TO_HIP_PITCH_M: f32 = 0.12;
/// The printed foot sole sits slightly below the ankle-roll link origin.
const SOLE_BELOW_ANKLE_M: f32 = 0.018;
/// Foot clearance below which the sole is considered planted (metres).
const CONTACT_TOLERANCE_M: f32 = 0.006;
/// LIPM capture-velocity tracking gain (1/s) for the world CoM.
const WORLD_VEL_TRACK_GAIN: f32 = 6.0;
/// Capture-point gain used when placing the next stance foot.
const WORLD_LANDING_GAIN: f32 = 0.55;
/// CoM acceleration ceiling (m/s²).
const WORLD_MAX_ACCEL: f32 = 6.0;
/// Metres per unit of `WalkCmd.vx`/`vy` (matches the UVC capture law's `v_des`).
const WALK_SPEED_SCALE: f32 = 0.5;
/// Effectiveness of the contact CoP moment on torso tilt (lumped inertia).
const CONTACT_MOMENT_GAIN: f32 = 0.35;

/// Per-foot ground contact state in the world/ground frame.
///
/// `point_m` is `[x, z]` (lateral-right, forward). The plant pins this point
/// while the foot is in stance; its motion relative to the FK foot origin is
/// reported as `slip_m_s`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FootContact {
    /// True while the sole is inside the ground contact tolerance.
    pub contact: bool,
    /// Planted world point `[x, z]` in metres.
    pub point_m: [f32; 2],
    /// Normal ground reaction force in Newtons (>= 0, bounded).
    pub grf_n: f32,
    /// Penetration depth below the ground plane in metres (>= 0).
    pub penetration_m: f32,
    /// Tangential slip speed of the sole in m/s (0 while perfectly pinned).
    pub slip_m_s: f32,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct SimFaultInjection {
    /// Pauses HostToMcu frame delivery so the 100 ms deadman watchdog trips in firmware.
    pub drop_heartbeat: bool,
    /// Per-joint mechanical stall flags (locks motion and spikes current > 2200 mA).
    pub stalled_joints: [bool; MAX_JOINTS],
    /// Drops battery bus voltage below 10.5 V to exercise UNDER_VOLTAGE.
    pub voltage_sag: bool,
    /// Injects a servo bus communication error flag.
    pub servo_bus_error: bool,
    /// Drives a smooth external sinusoidal hand wave in Teach mode to test Robot -> GUI mirroring.
    pub external_hand_wave: bool,
    /// Disables the simulated IMU so Rocky reports no IMU available (3D viewport locks upright).
    #[serde(default)]
    pub imu_disconnected: bool,
    /// Actuator responsiveness factor (0.04 = slow/visible ghost lag, 0.35 = crisp).
    pub servo_lag_factor: f32,
}

impl Default for SimFaultInjection {
    fn default() -> Self {
        Self {
            drop_heartbeat: false,
            stalled_joints: [false; MAX_JOINTS],
            voltage_sag: false,
            servo_bus_error: false,
            external_hand_wave: false,
            imu_disconnected: false,
            servo_lag_factor: 0.12,
        }
    }
}

#[derive(Clone, Debug)]
pub struct SimulatedServoPlant {
    joint_count: usize,
    raw_angle_rad: [f32; MAX_JOINTS],
    velocity_rad_s: [f32; MAX_JOINTS],
    current_ma: [u16; MAX_JOINTS],
    temp_c_f: [f32; MAX_JOINTS],
    virtual_hand_rad: [f32; MAX_JOINTS],
    wave_phase: f32,
    last_cmd: RawServoCommand,
    faults: SimFaultInjection,
    // Deterministic 2-axis stepping-aware LIPM torso dynamics + virtual IMU state
    torso_pitch_rad: f32,
    torso_roll_rad: f32,
    /// Synthetic heading accumulated from roll motion (deterministic, zero on reset).
    torso_yaw_rad: f32,
    pitch_rate_rad_s: f32,
    roll_rate_rad_s: f32,
    act_pitch_corr: f32,
    act_roll_corr: f32,
    act_foot_dx: f32,
    act_foot_dy: f32,
    last_balance: rk_proto::BalanceTelemetry,
    /// Optional `[0.0, 1.0]` normalized progress of an active get-up recovery sequence.
    /// Models ground push-off contact restoring torso orientation smoothly to upright.
    recovery_progress: Option<f32>,
    recovery_start_pitch: f32,
    recovery_start_roll: f32,
    recovery_start_yaw: f32,
    // --- Torque-capable running plant (v2). Inactive unless running_cfg.enabled ---
    /// Current wire running configuration (also holds physics parameters for the plant).
    running_cfg: RunningConfig,
    /// True once `running_cfg.enabled` is set; switches torque-mode joints to the hop model.
    running_mode: bool,
    /// Vertical CoM height above the support sole in metres.
    running_com_height_m: f32,
    /// Vertical CoM velocity in m/s (positive = rising).
    running_com_vel_z_m_s: f32,
    /// Debounced airborne flag published by the plant-side flight detector.
    running_in_flight: bool,
    /// Consecutive raw-airborne time used for the flight debounce (seconds).
    running_air_time_s: f32,
    /// Previous debounced flight state, for touchdown edge detection.
    running_prev_in_flight: bool,
    /// Landing stiffness ramp progress in `[0.0, 1.0]`.
    running_landing_ramp: f32,
    /// Peak ground-reaction force seen during the most recent landing ramp (N).
    running_peak_landing_force_n: f32,
    /// Highest CoM height reached during the current flight (m).
    running_apex_height_m: f32,
    /// Ground reaction force from the most recent hop step (N).
    running_ground_reaction_n: f32,
    /// Vertical thrust converted from joint torque commands in the most recent step (N).
    running_thrust_n: f32,
    // --- Ground-plane world state (locomotion-physics) ----------------------
    /// Latest streamed walk command (ground velocity target + cadence).
    walk_cmd: WalkCmd,
    /// World-frame CoM position `[x, y, z]` in metres (`x` right, `y` up, `z` forward).
    world_com_pos_m: [f32; 3],
    /// World-frame CoM velocity `[vx, vy, vz]` in m/s.
    world_com_vel_m_s: [f32; 3],
    /// Forward CoM position at construction/reset, for displacement reporting.
    world_start_z_m: f32,
    /// Right (index 0) and left (index 1) sole contact state.
    foot_contacts: [FootContact; 2],
    /// Total normal ground reaction from the most recent world step (N).
    world_grf_n: f32,
    /// Support-foot swaps observed while walking (0 = Right, 1 = Left).
    contact_swaps: u32,
    /// Number of world steps each sole spent in contact (right, left).
    contact_ticks: [u32; 2],
    /// Previous support foot, for landing detection.
    last_support_foot: u8,
    /// CoP offset `[x, z]` relative to CoM from the latest contact step (m).
    /// Drives the torso tilt through ground-reaction moments instead of a scripted sway.
    contact_cop_offset_m: [f32; 2],
}

/// Body-frame gravity consistent with the kinematics tilt rotation.
///
/// `tilt_rot = R_x(pitch) * R_z(-roll)` (pitch about +X, roll about -Z, matching
/// `kinematics.rs`), so the sensor reading is `tilt_rot^-1 * (0, -9.81, 0)`
/// `= R_z(roll) * R_x(-pitch) * (0, -9.81, 0)`. This guarantees the rendered
/// orientation and the published gravity vector agree.
fn gravity_from_tilt(pitch_rad: f32, roll_rad: f32) -> [f32; 3] {
    const G: f32 = 9.81_f32;
    // R_x(-pitch) applied to (0, -G, 0).
    let y1 = -G * (-pitch_rad).cos();
    let z1 = -G * (-pitch_rad).sin();
    // R_z(roll) applied to the result (x starts at zero).
    let sr = roll_rad.sin();
    let cr = roll_rad.cos();
    [-y1 * sr, y1 * cr, z1]
}

/// Shared stance spring used by both the vertical hop integrator and the
/// per-foot walking contact model. Keeps one source of truth for ground
/// compliance: a touchdown/stiffness ramp interpolates between the soft
/// touchdown stiffness and the nominal stance stiffness.
fn stance_spring_grf(
    compression_m: f32,
    compression_rate_m_s: f32,
    cfg: &RunningConfig,
    ramp: f32,
) -> f32 {
    let k = cfg.landing_stiffness_touchdown_n_m
        + (cfg.landing_stiffness_nominal_n_m - cfg.landing_stiffness_touchdown_n_m)
            * ramp.clamp(0.0_f32, 1.0_f32);
    (k * compression_m + cfg.landing_damping_n_s_m * compression_rate_m_s).max(0.0_f32)
}

/// Initial world CoM position from the shared mass-weighted FK and leg geometry.
fn initial_world_com(initial_raw_pose: &[f32; MAX_JOINTS]) -> [f32; 3] {
    let fk = compute_state(initial_raw_pose, &[0.0_f32; MAX_JOINTS]);
    [
        fk.com[0],
        LegGeometry::rocky_mk1().nominal_height + PELVIS_TO_HIP_PITCH_M + fk.com[1],
        fk.com[2],
    ]
}

impl SimulatedServoPlant {
    pub fn new(joint_count: usize, initial_raw_pose: [f32; MAX_JOINTS]) -> Self {
        Self {
            joint_count: joint_count.clamp(1, MAX_JOINTS),
            raw_angle_rad: initial_raw_pose,
            velocity_rad_s: [0.0; MAX_JOINTS],
            current_ma: [0; MAX_JOINTS],
            temp_c_f: [28.0; MAX_JOINTS],
            virtual_hand_rad: initial_raw_pose,
            wave_phase: 0.0,
            last_cmd: RawServoCommand {
                torque_mask: 0,
                raw_target_rad: initial_raw_pose,
                ..Default::default()
            },
            faults: SimFaultInjection::default(),
            torso_pitch_rad: 0.0,
            torso_roll_rad: 0.0,
            torso_yaw_rad: 0.0,
            pitch_rate_rad_s: 0.0,
            roll_rate_rad_s: 0.0,
            act_pitch_corr: 0.0,
            act_roll_corr: 0.0,
            act_foot_dx: 0.0,
            act_foot_dy: 0.0,
            last_balance: rk_proto::BalanceTelemetry::default(),
            recovery_progress: None,
            recovery_start_pitch: 0.0,
            recovery_start_roll: 0.0,
            recovery_start_yaw: 0.0,
            running_cfg: RunningConfig::default(),
            running_mode: false,
            running_com_height_m: RunningConfig::default().leg_rest_length_m,
            running_com_vel_z_m_s: 0.0,
            running_in_flight: false,
            running_air_time_s: 0.0,
            running_prev_in_flight: false,
            running_landing_ramp: 0.0,
            running_peak_landing_force_n: 0.0,
            running_apex_height_m: RunningConfig::default().leg_rest_length_m,
            running_ground_reaction_n: 0.0,
            running_thrust_n: 0.0,
            walk_cmd: WalkCmd::default(),
            world_com_pos_m: initial_world_com(&initial_raw_pose),
            world_com_vel_m_s: [0.0; 3],
            world_start_z_m: initial_world_com(&initial_raw_pose)[2],
            foot_contacts: [FootContact::default(); 2],
            world_grf_n: 0.0,
            contact_swaps: 0,
            contact_ticks: [0; 2],
            last_support_foot: 0,
            contact_cop_offset_m: [0.0; 2],
        }
    }

    pub fn faults(&self) -> &SimFaultInjection {
        &self.faults
    }

    pub fn faults_mut(&mut self) -> &mut SimFaultInjection {
        &mut self.faults
    }

    pub fn set_virtual_hand_angle(&mut self, joint: usize, raw_rad: f32) {
        if joint < self.joint_count && raw_rad.is_finite() {
            self.virtual_hand_rad[joint] = raw_rad;
        }
    }

    pub fn virtual_hand_angles(&self) -> &[f32; MAX_JOINTS] {
        &self.virtual_hand_rad
    }

    pub fn raw_angles(&self) -> &[f32; MAX_JOINTS] {
        &self.raw_angle_rad
    }

    /// Injects a combat shove angular velocity impulse (rad/s) into torso pitch and roll.
    pub fn apply_push_impulse(&mut self, pitch_impulse_rad_s: f32, roll_impulse_rad_s: f32) {
        if pitch_impulse_rad_s.is_finite() {
            self.pitch_rate_rad_s = (self.pitch_rate_rad_s + pitch_impulse_rad_s).clamp(-6.5, 6.5);
        }
        if roll_impulse_rad_s.is_finite() {
            self.roll_rate_rad_s = (self.roll_rate_rad_s + roll_impulse_rad_s).clamp(-6.5, 6.5);
        }
    }

    /// Stands Rocky cleanly upright at 0 deg pitch/roll with zero angular velocity.
    pub fn reset_upright(&mut self) {
        self.torso_pitch_rad = 0.0;
        self.torso_roll_rad = 0.0;
        self.torso_yaw_rad = 0.0;
        self.pitch_rate_rad_s = 0.0;
        self.roll_rate_rad_s = 0.0;
        self.act_pitch_corr = 0.0;
        self.act_roll_corr = 0.0;
        self.act_foot_dx = 0.0;
        self.act_foot_dy = 0.0;
        self.recovery_progress = None;
        self.recovery_start_pitch = 0.0;
        self.recovery_start_roll = 0.0;
        self.recovery_start_yaw = 0.0;
        self.reset_running_dynamics();
        self.world_com_pos_m = initial_world_com(&self.raw_angle_rad);
        self.world_start_z_m = self.world_com_pos_m[2];
        self.world_com_vel_m_s = [0.0; 3];
        self.foot_contacts = [FootContact::default(); 2];
        self.world_grf_n = 0.0;
        self.contact_swaps = 0;
        self.contact_ticks = [0; 2];
        self.last_support_foot = 0;
        self.contact_cop_offset_m = [0.0; 2];
    }

    /// Updates active get-up recovery progress (`Some(0.0..=1.0)`) so simulated ground
    /// contact torques lift the fallen torso smoothly upright in sync with keyframes.
    pub fn set_recovery_progress(&mut self, progress: Option<f32>) {
        match progress {
            Some(p) if p.is_finite() => {
                if self.recovery_progress.is_none() {
                    self.recovery_start_pitch = self.torso_pitch_rad;
                    self.recovery_start_roll = self.torso_roll_rad;
                    self.recovery_start_yaw = self.torso_yaw_rad;
                }
                self.recovery_progress = Some(p.clamp(0.0, 1.0));
            }
            _ => {
                self.recovery_progress = None;
            }
        }
    }

    pub fn set_balance_telemetry(&mut self, bal: rk_proto::BalanceTelemetry) {
        self.last_balance = bal;
    }

    // --- Ground-plane world state API ---------------------------------------

    /// Installs the latest streamed walk command. Called by the shared
    /// [`RobotIo::take_cmd`] path so both the synchronous and Embassy harness
    /// backends drive the same world physics.
    pub fn set_walk_cmd(&mut self, walk: WalkCmd) {
        self.walk_cmd = walk;
    }

    /// World-frame CoM position `[x, y, z]` in metres (`x` right, `z` forward).
    pub fn world_position_m(&self) -> [f32; 3] {
        self.world_com_pos_m
    }

    /// World-frame CoM velocity `[vx, vy, vz]` in m/s.
    pub fn world_velocity_m_s(&self) -> [f32; 3] {
        self.world_com_vel_m_s
    }

    /// Forward ground displacement from the initial pose (metres).
    pub fn forward_displacement_m(&self) -> f32 {
        self.world_com_pos_m[2] - self.world_start_z_m
    }

    /// Right (`false`) and left (`true`) sole contact state.
    pub fn foot_contact(&self, left: bool) -> FootContact {
        self.foot_contacts[if left { 1 } else { 0 }]
    }

    /// Both sole contact states, indexed `[right, left]`.
    pub fn foot_contacts(&self) -> &[FootContact; 2] {
        &self.foot_contacts
    }

    /// Contact bitmask (`bit 0 = Right`, `bit 1 = Left`) matching the wire convention.
    pub fn contact_mask(&self) -> u8 {
        (if self.foot_contacts[0].contact { 1 } else { 0 })
            | (if self.foot_contacts[1].contact { 2 } else { 0 })
    }

    /// Number of support-foot swaps observed while walking.
    pub fn contact_swaps(&self) -> u32 {
        self.contact_swaps
    }

    /// Number of world steps each sole spent in contact (`[right, left]`).
    pub fn contact_ticks(&self) -> [u32; 2] {
        self.contact_ticks
    }

    /// Total normal ground reaction in Newtons from the latest world step.
    pub fn ground_reaction_total_n(&self) -> f32 {
        self.world_grf_n
    }

    // --- Torque-capable running plant API -----------------------------------

    /// Installs the wire running configuration. Invalid (non-finite) gains fall back
    /// to `RunningConfig::default()` with the gate forced OFF, so a corrupt host frame
    /// can never activate the hop plant. Disabling running re-initialises the vertical
    /// state so the default plant remains unchanged on the next tick.
    pub fn set_running_config(&mut self, cfg: RunningConfig) {
        let sanitized = if cfg.all_finite() {
            cfg
        } else {
            RunningConfig {
                enabled: false,
                ..Default::default()
            }
        };
        let gate_changed = sanitized.enabled != self.running_mode;
        self.running_cfg = sanitized;
        self.running_mode = sanitized.enabled;
        if gate_changed {
            self.reset_running_dynamics();
        }
    }

    /// Enables/disables running without replacing the gains.
    pub fn set_running_enabled(&mut self, enabled: bool) {
        let mut cfg = self.running_cfg;
        cfg.enabled = enabled;
        self.set_running_config(cfg);
    }

    pub fn running_enabled(&self) -> bool {
        self.running_mode
    }

    /// Debounced flight state reported by the plant-side detector.
    pub fn in_flight(&self) -> bool {
        self.running_in_flight
    }

    pub fn com_height_m(&self) -> f32 {
        self.running_com_height_m
    }

    pub fn com_velocity_z_m_s(&self) -> f32 {
        self.running_com_vel_z_m_s
    }

    /// Stance spring/ground reaction force from the most recent hop step (N).
    pub fn ground_reaction_n(&self) -> f32 {
        self.running_ground_reaction_n
    }

    /// Vertical thrust produced from commanded torque in the most recent step (N).
    pub fn thrust_n(&self) -> f32 {
        self.running_thrust_n
    }

    /// Peak ground reaction during the most recent landing-ramp window (N).
    pub fn peak_landing_force_n(&self) -> f32 {
        self.running_peak_landing_force_n
    }

    /// Peak landing force expressed as equivalent single-joint torque (N·m).
    pub fn peak_landing_torque_nm(&self) -> f32 {
        self.running_peak_landing_force_n * RUNNING_LEVER_ARM_M
    }

    /// Highest CoM height reached in the current flight (m).
    pub fn apex_height_m(&self) -> f32 {
        self.running_apex_height_m
    }

    /// Apex height above the configured leg rest length, i.e. achieved hop height (m).
    pub fn hop_height_m(&self) -> f32 {
        (self.running_apex_height_m - self.running_cfg.leg_rest_length_m).max(0.0)
    }

    /// Directly seeds the vertical hop state. Intended for deterministic validation
    /// of landing/flight behaviour without having to synthesise a full approach run.
    pub fn set_running_state(&mut self, height_m: f32, velocity_z_m_s: f32, in_flight: bool) {
        if !height_m.is_finite() || !velocity_z_m_s.is_finite() {
            return;
        }
        self.running_com_height_m = height_m.max(0.0);
        self.running_com_vel_z_m_s = velocity_z_m_s.clamp(-5.0, 5.0);
        self.running_in_flight = in_flight;
        self.running_prev_in_flight = in_flight;
        self.running_air_time_s = if in_flight {
            RUNNING_FLIGHT_DEBOUNCE_S
        } else {
            0.0
        };
        self.running_apex_height_m = self.running_com_height_m;
        self.running_landing_ramp = 0.0;
        self.running_peak_landing_force_n = 0.0;
        self.running_ground_reaction_n = 0.0;
        self.running_thrust_n = 0.0;
    }

    /// Returns vertical state to standing at leg rest length with zero velocity.
    fn reset_running_dynamics(&mut self) {
        let l0 = if self.running_cfg.leg_rest_length_m.is_finite()
            && self.running_cfg.leg_rest_length_m > 0.0
        {
            self.running_cfg.leg_rest_length_m
        } else {
            RunningConfig::default().leg_rest_length_m
        };
        self.running_com_height_m = l0;
        self.running_com_vel_z_m_s = 0.0;
        self.running_in_flight = false;
        self.running_prev_in_flight = false;
        self.running_air_time_s = 0.0;
        self.running_landing_ramp = 0.0;
        self.running_peak_landing_force_n = 0.0;
        self.running_apex_height_m = l0;
        self.running_ground_reaction_n = 0.0;
        self.running_thrust_n = 0.0;
    }

    /// Deterministic vertical SLIP hop integrator driven by joint torque commands.
    ///
    /// Joint torque in `Torque` mode is converted to upward thrust through a fixed
    /// lever arm; a stance spring with the configured landing-impedance stiffness
    /// (soft at touchdown, ramped to nominal over `landing_ramp_s`) supplies ground
    /// reaction. Gravity is always applied. No wall-clock, randomness or allocation.
    fn step_running_dynamics(&mut self, dt: f32) {
        let cfg = self.running_cfg;
        let mass = cfg.mass_kg.clamp(0.2, 10.0);
        let l0 = cfg.leg_rest_length_m.clamp(0.05, 1.0);
        let ramp_s = cfg.landing_ramp_s.clamp(0.002, 0.2);

        // Sum signed commanded torque over torque-mode, energised joints -> thrust.
        // Negative thrust is permitted so a controller can absorb energy / brake.
        let mut torque_sum_nm = 0.0_f32;
        for i in 0..self.joint_count {
            if ((self.last_cmd.torque_mask >> i) & 1) == 1
                && JointCommandMode::from_wire(self.last_cmd.command_mode[i])
                    == JointCommandMode::Torque
            {
                torque_sum_nm += self.last_cmd.torque_nm[i];
            }
        }
        let thrust = (torque_sum_nm / RUNNING_LEVER_ARM_M)
            .clamp(RUNNING_MIN_THRUST_N, RUNNING_MAX_THRUST_N);
        self.running_thrust_n = thrust;

        // Stance spring with first-order landing stiffening ramp.
        let mut ground = 0.0_f32;
        if self.running_com_height_m < l0 {
            let compression = l0 - self.running_com_height_m;
            // Compression rate is positive while the leg shortens, negative while extending.
            let compression_rate = -self.running_com_vel_z_m_s;
            self.running_landing_ramp = (self.running_landing_ramp + dt / ramp_s).min(1.0);
            // Same stance spring the walking contacts use.
            ground = stance_spring_grf(
                compression,
                compression_rate,
                &cfg,
                self.running_landing_ramp,
            );
            self.running_peak_landing_force_n = self.running_peak_landing_force_n.max(ground);
        } else {
            self.running_landing_ramp = 0.0;
        }
        self.running_ground_reaction_n = ground;

        // Semi-implicit Euler: deterministic for a fixed dt.
        let accel = (ground + thrust) / mass - RUNNING_G_M_S2;
        self.running_com_vel_z_m_s = (self.running_com_vel_z_m_s + accel * dt).clamp(-5.0, 5.0);
        self.running_com_height_m += self.running_com_vel_z_m_s * dt;
        if self.running_com_height_m <= 0.0 {
            self.running_com_height_m = 0.0;
            if self.running_com_vel_z_m_s < 0.0 {
                self.running_com_vel_z_m_s = 0.0;
            }
        }

        // Flight detector: debounce on entry (>=12ms), immediate on touchdown.
        let raw_airborne = self.running_com_height_m > l0 + 1.0e-4_f32;
        if raw_airborne {
            self.running_air_time_s += dt;
            self.running_apex_height_m = self.running_apex_height_m.max(self.running_com_height_m);
            if self.running_air_time_s >= RUNNING_FLIGHT_DEBOUNCE_S {
                self.running_in_flight = true;
            }
        } else {
            self.running_air_time_s = 0.0;
            self.running_in_flight = false;
        }

        // Touchdown edge: reset the compliance ramp and landing-force peak.
        if self.running_prev_in_flight && !self.running_in_flight {
            self.running_landing_ramp = 0.0;
            self.running_peak_landing_force_n = 0.0;
            self.running_apex_height_m = self.running_com_height_m;
        }
        self.running_prev_in_flight = self.running_in_flight;
    }

    /// Ground-plane LIPM + per-foot contact physics.
    ///
    /// The CoM/foot kinematics come from the shared `com_fk` mass table and
    /// `LegGeometry::rocky_mk1()` (one source of truth with the controller).
    /// Stance soles are pinned to a planted world point; the CoM evolves as an
    /// inverted pendulum about the GRF-weighted CoP with a commanded-velocity
    /// tracking term that represents the leg swing. All math is fixed-dt `f32`.
    fn step_world_dynamics(&mut self, dt: f32, legs_energized: bool) {
        if !legs_energized || self.recovery_progress.is_some() {
            // Torque off / get-up recovery: no locomotion plant. Bleed the
            // previous velocity deterministically and drop all contacts.
            let decay = (1.0_f32 - 8.0_f32 * dt).max(0.0_f32);
            self.world_com_vel_m_s[0] *= decay;
            self.world_com_vel_m_s[2] *= decay;
            self.world_grf_n = 0.0;
            self.foot_contacts = [FootContact::default(); 2];
            self.contact_cop_offset_m = [0.0; 2];
            return;
        }

        let geom = LegGeometry::rocky_mk1();
        let fk = compute_state(&self.raw_angle_rad, &self.velocity_rad_s);
        let hip_h = if self.last_balance.hip_height_m.is_finite()
            && self.last_balance.hip_height_m > 0.2_f32
        {
            self.last_balance
                .hip_height_m
                .clamp(geom.min_height, geom.nominal_height)
        } else {
            geom.nominal_height
        };
        // Pelvis root is `hip_h` above the hip pivot plus the fixed URDF chain
        // offset from `JOINT_KINEMATICS` (pelvis -> hip-pitch pivot).
        let pelvis_y = hip_h + PELVIS_TO_HIP_PITCH_M;
        let pelvis_x = self.world_com_pos_m[0] - fk.com[0];
        let pelvis_z = self.world_com_pos_m[2] - fk.com[2];
        let pelvis_vx = self.world_com_vel_m_s[0] - fk.com_vel[0];
        let pelvis_vz = self.world_com_vel_m_s[2] - fk.com_vel[2];

        let mass = self.running_cfg.mass_kg.clamp(0.2_f32, 10.0_f32);
        let body_weight = mass * RUNNING_G_M_S2;
        let fk_feet = [
            (fk.right_foot, fk.right_foot_vel),
            (fk.left_foot, fk.left_foot_vel),
        ];

        // 1. Sole contact detection from shared FK foot heights.
        let mut contacts = [FootContact::default(); 2];
        for foot in 0..2 {
            let (p, pv) = fk_feet[foot];
            let clearance = pelvis_y + p[1] - SOLE_BELOW_ANKLE_M;
            let penetration = (-clearance).max(0.0_f32);
            let contact = clearance <= CONTACT_TOLERANCE_M;
            let world_point = [pelvis_x + p[0], pelvis_z + p[2]];
            let grf = if contact {
                // Generalised stance spring: same touchdown/nominal compliance
                // as the running hop, driven here by sole penetration.
                stance_spring_grf(penetration, (-pv[1]).max(0.0_f32), &self.running_cfg, 1.0)
                    + 0.5_f32 * body_weight
            } else {
                0.0_f32
            };
            let mut c = FootContact {
                contact,
                point_m: if self.foot_contacts[foot].contact {
                    self.foot_contacts[foot].point_m
                } else {
                    world_point
                },
                grf_n: grf.min(4.0_f32 * body_weight),
                penetration_m: penetration,
                slip_m_s: 0.0_f32,
            };
            if contact {
                let fvx = pelvis_vx + pv[0];
                let fvz = pelvis_vz + pv[2];
                c.slip_m_s = (fvx * fvx + fvz * fvz).sqrt().min(2.0_f32);
            }
            contacts[foot] = c;
        }

        // 2. Landing: the controller's support-foot swap marks the instant the
        //    swing sole becomes the new stance sole. Place it at the capture
        //    point ahead of the CoM, bounded by the shared swing-foot reach.
        let walking = self.walk_cmd.walk_enabled
            && (self.walk_cmd.vx.abs() + self.walk_cmd.vy.abs() + self.walk_cmd.yaw_rate.abs())
                > 0.04_f32;
        let support = (self.last_balance.support_foot & 1) as usize;
        if walking && support != self.last_support_foot as usize {
            self.last_support_foot = support as u8;
            self.contact_swaps = self.contact_swaps.saturating_add(1);
            let h = self.world_com_pos_m[1].clamp(0.25_f32, 1.0_f32);
            let omega = (RUNNING_G_M_S2 / h).sqrt();
            let v_des_x = self.walk_cmd.vy * WALK_SPEED_SCALE;
            let v_des_z = self.walk_cmd.vx * WALK_SPEED_SCALE;
            let foot_dx = self.last_balance.foot_dx_m.clamp(-geom.max_dx, geom.max_dx);
            let land_x = (self.world_com_vel_m_s[0]
                + WORLD_LANDING_GAIN * (v_des_x - self.world_com_vel_m_s[0]))
                / omega;
            let land_z = foot_dx
                + (self.world_com_vel_m_s[2]
                    + WORLD_LANDING_GAIN * (v_des_z - self.world_com_vel_m_s[2]))
                    / omega;
            contacts[support].point_m = [
                self.world_com_pos_m[0] + land_x.clamp(-geom.max_swing_dx, geom.max_swing_dx),
                self.world_com_pos_m[2] + land_z.clamp(-geom.max_swing_dx, geom.max_swing_dx),
            ];
            contacts[support].contact = true;
        }

        // 3. GRF-weighted CoP (the support point of the LIPM). Airborne -> CoM,
        //    so no phantom ground moment is applied.
        let grf_sum = contacts[0].grf_n + contacts[1].grf_n;
        let cop = if grf_sum > 1.0e-3_f32 {
            [
                (contacts[0].point_m[0] * contacts[0].grf_n
                    + contacts[1].point_m[0] * contacts[1].grf_n)
                    / grf_sum,
                (contacts[0].point_m[1] * contacts[0].grf_n
                    + contacts[1].point_m[1] * contacts[1].grf_n)
                    / grf_sum,
            ]
        } else {
            [self.world_com_pos_m[0], self.world_com_pos_m[2]]
        };
        self.contact_cop_offset_m = [
            cop[0] - self.world_com_pos_m[0],
            cop[1] - self.world_com_pos_m[2],
        ];
        self.world_grf_n = grf_sum.min(4.0_f32 * body_weight);

        // 4. Semi-implicit LIPM: gravity topples the CoM about the support CoP;
        //    the tracking term represents the commanded leg swing pushing the
        //    body toward the streamed ground velocity.
        let h = self.world_com_pos_m[1].clamp(0.25_f32, 1.0_f32);
        let omega2 = RUNNING_G_M_S2 / h;
        let v_des_x = self.walk_cmd.vy * WALK_SPEED_SCALE;
        let v_des_z = self.walk_cmd.vx * WALK_SPEED_SCALE;
        let mut ax = omega2 * (self.world_com_pos_m[0] - cop[0])
            + WORLD_VEL_TRACK_GAIN * (v_des_x - self.world_com_vel_m_s[0]);
        let mut az = omega2 * (self.world_com_pos_m[2] - cop[1])
            + WORLD_VEL_TRACK_GAIN * (v_des_z - self.world_com_vel_m_s[2]);
        ax = ax.clamp(-WORLD_MAX_ACCEL, WORLD_MAX_ACCEL);
        az = az.clamp(-WORLD_MAX_ACCEL, WORLD_MAX_ACCEL);
        self.world_com_vel_m_s[0] =
            (self.world_com_vel_m_s[0] + ax * dt).clamp(-1.5_f32, 1.5_f32);
        self.world_com_vel_m_s[2] =
            (self.world_com_vel_m_s[2] + az * dt).clamp(-1.5_f32, 1.5_f32);
        if walking && v_des_z > 0.01_f32 && self.world_com_vel_m_s[2] < 0.0_f32 {
            self.world_com_vel_m_s[2] = 0.0_f32;
        }
        self.world_com_pos_m[0] += self.world_com_vel_m_s[0] * dt;
        self.world_com_pos_m[2] += self.world_com_vel_m_s[2] * dt;
        self.world_com_pos_m[1] = pelvis_y + fk.com[1];

        // 5. Slip is the residual sole velocity after the CoM step (a pinned
        //    foot would see zero). Recomputed from the integrated world state.
        for foot in 0..2 {
            if contacts[foot].contact {
                let (_, pv) = fk_feet[foot];
                let fvx = self.world_com_vel_m_s[0] - fk.com_vel[0] + pv[0];
                let fvz = self.world_com_vel_m_s[2] - fk.com_vel[2] + pv[2];
                contacts[foot].slip_m_s = (fvx * fvx + fvz * fvz).sqrt().min(2.0_f32);
            }
        }
        self.foot_contacts = contacts;
        for foot in 0..2 {
            if contacts[foot].contact {
                self.contact_ticks[foot] = self.contact_ticks[foot].saturating_add(1);
            }
        }
    }

    pub fn step_plant(&mut self, dt_sec: f32) -> RawServoFeedback {
        let dt = dt_sec.clamp(0.001_f32, 0.05_f32);
        self.wave_phase += dt * 2.4_f32;

        let mut total_current_ma: u32 = 180;

        for i in 0..self.joint_count {
            let torque_on = ((self.last_cmd.torque_mask >> i) & 1) == 1;
            let stalled = self.faults.stalled_joints[i];
            let running_torque = self.running_mode
                && torque_on
                && JointCommandMode::from_wire(self.last_cmd.command_mode[i])
                    == JointCommandMode::Torque;

            if running_torque {
                // Torque-capable running: torque_nm is a thrust command, not a
                // position-servo target. The vertical hop integrator consumes it;
                // joint angles are held at their last pose. Current tracks the
                // commanded torque so flight detection sees near-zero leg current.
                let tq = self.last_cmd.torque_nm[i].abs();
                self.velocity_rad_s[i] = 0.0;
                self.current_ma[i] = if stalled {
                    2250
                } else {
                    (tq / RUNNING_NOMINAL_TORQUE_NM * rk_proto::RATED_CURRENT_MA as f32)
                        .clamp(0.0, 3400.0) as u16
                };
            } else if torque_on {
                let target = self.last_cmd.raw_target_rad[i];
                let pos = self.raw_angle_rad[i];
                let err = target - pos;

                if stalled {
                    self.velocity_rad_s[i] = 0.0;
                    let torque_scale = (self.last_cmd.torque_nm[i].abs() / 1.8_f32).clamp(0.10, 1.0);
                    let spike =
                        ((2350.0_f32 + err.abs() * 900.0_f32) * torque_scale).min(3400.0_f32) as u16;
                    self.current_ma[i] = spike;
                } else {
                    // Honor per-joint torque_nm saturation limit (software compliance & ukemi relaxation)
                    let torque_scale = (self.last_cmd.torque_nm[i].abs() / 1.8_f32).clamp(0.08, 1.0);
                    let rate = (2.6_f32 + self.faults.servo_lag_factor * 42.0_f32) * torque_scale;
                    let alpha = 1.0_f32 - (-rate * dt).exp();
                    let next_pos = pos + err * alpha;
                    let vel = (next_pos - pos) / dt;
                    self.raw_angle_rad[i] = next_pos;
                    self.velocity_rad_s[i] = vel;
                    self.virtual_hand_rad[i] = next_pos;

                    let max_ma = (2800.0_f32 * torque_scale).max(260.0_f32);
                    let load = (95.0_f32 + (err.abs() * 1450.0_f32 + vel.abs() * 160.0_f32) * torque_scale)
                        .min(max_ma) as u16;
                    self.current_ma[i] = load;
                }
            } else {
                // Torque OFF (Teach or EStop): follows Virtual-Hand posing or external wave
                let mut hand_goal = self.virtual_hand_rad[i];
                if self.faults.external_hand_wave {
                    match i {
                        0 => hand_goal = (self.wave_phase * 0.6).sin() * 0.35, // waist_yaw
                        1 => hand_goal = (self.wave_phase * 0.8).sin() * 0.42, // head_yaw
                        3 => hand_goal = -1.35 + self.wave_phase.sin() * 0.48, // l_shoulder_pitch
                        5 => hand_goal = -1.25 + (self.wave_phase * 1.3).cos() * 0.55, // l_elbow
                        6 => hand_goal = -0.95 + self.wave_phase.cos() * 0.40, // r_shoulder_pitch
                        8 => hand_goal = -1.15 + (self.wave_phase * 1.1).sin() * 0.45, // r_elbow
                        _ => {}
                    }
                }
                let pos = self.raw_angle_rad[i];
                let alpha = 1.0_f32 - (-14.0_f32 * dt).exp();
                let next_pos = pos + (hand_goal - pos) * alpha;
                self.velocity_rad_s[i] = (next_pos - pos) / dt;
                self.raw_angle_rad[i] = next_pos;
                self.current_ma[i] = if stalled { 2250 } else { 35 };
            }

            total_current_ma = total_current_ma.saturating_add(self.current_ma[i] as u32);

            let ambient = 28.0_f32;
            let amps = (self.current_ma[i] as f32) * 0.001_f32;
            let heat = amps * amps * 4.0_f32;
            let cool = (self.temp_c_f[i] - ambient) * 0.14_f32;
            self.temp_c_f[i] = (self.temp_c_f[i] + (heat - cool) * dt).clamp(ambient, 98.0_f32);
        }

        // Deterministic 2-axis stepping-aware LIPM torso tilt dynamics
        let legs_mask: u32 = if self.joint_count >= 21 {
            0x1FFE00 // joints 9..=20 (21-DoF L/R 6-DoF legs)
        } else {
            0x1FE00 // joints 9..=16 (legacy 17-DoF legs)
        };
        let leg_end = if self.joint_count >= 21 { 21 } else { 17 }.min(self.joint_count);
        let legs_energized = (self.last_cmd.torque_mask & legs_mask) != 0 && self.joint_count >= 17;
        let leg_stalled = (9..leg_end).any(|idx| self.faults.stalled_joints[idx]);

        // Ground-plane world physics runs before the tilt integrator so the
        // contact CoP can drive torso attitude through a physical moment.
        self.step_world_dynamics(dt, legs_energized);

        if legs_energized && self.recovery_progress.is_some() {
            // Ground push-off contact dynamics during active get-up recovery:
            // stays near fallen pose during Fallen Tuck (0..0.12), lifts smoothly through
            // Push Off & Kneel (0.12..0.78), and settles upright in Stand & Guard (0.78..1.0).
            // External shoves still act on pitch/roll rates so a re-fall can abort recovery.
            let p = self.recovery_progress.unwrap_or(0.0);
            let f = ((p - 0.12_f32) / 0.66_f32).clamp(0.0_f32, 1.0_f32);
            let s = f * f * (3.0_f32 - 2.0_f32 * f);
            let goal_pitch = self.recovery_start_pitch * (1.0_f32 - s);
            let goal_roll = self.recovery_start_roll * (1.0_f32 - s);
            let goal_yaw = self.recovery_start_yaw * (1.0_f32 - s);

            let kp = 26.0_f32;
            let kd = 8.5_f32;
            let alpha_pitch = kp * (goal_pitch - self.torso_pitch_rad) - kd * self.pitch_rate_rad_s;
            let alpha_roll = kp * (goal_roll - self.torso_roll_rad) - kd * self.roll_rate_rad_s;

            self.pitch_rate_rad_s = (self.pitch_rate_rad_s + alpha_pitch * dt).clamp(-8.0, 8.0);
            self.roll_rate_rad_s = (self.roll_rate_rad_s + alpha_roll * dt).clamp(-8.0, 8.0);
            self.torso_pitch_rad = (self.torso_pitch_rad + self.pitch_rate_rad_s * dt).clamp(-1.35, 1.35);
            self.torso_roll_rad = (self.torso_roll_rad + self.roll_rate_rad_s * dt).clamp(-1.35, 1.35);
            self.torso_yaw_rad += (goal_yaw - self.torso_yaw_rad) * (dt * 6.0_f32).clamp(0.0, 1.0);
        } else if legs_energized {
            // Servo lag on ankle gyro damping & UVC foot CoP placement
            let act_rate = 7.0_f32 + self.faults.servo_lag_factor * 45.0_f32;
            let act_alpha = 1.0_f32 - (-act_rate * dt).exp();
            let balance_ready = self.last_balance.balance_active && !leg_stalled;
            let target_p = if balance_ready {
                self.last_balance.pitch_corr_rad
            } else {
                0.0_f32
            };
            let target_r = if balance_ready {
                self.last_balance.roll_corr_rad
            } else {
                0.0_f32
            };
            let target_dx = if balance_ready {
                self.last_balance.foot_dx_m
            } else {
                0.0_f32
            };
            let target_dy = if balance_ready {
                self.last_balance.foot_dy_m
            } else {
                0.0_f32
            };
            self.act_pitch_corr += (target_p - self.act_pitch_corr) * act_alpha;
            self.act_roll_corr += (target_r - self.act_roll_corr) * act_alpha;
            self.act_foot_dx += (target_dx - self.act_foot_dx) * act_alpha;
            self.act_foot_dy += (target_dy - self.act_foot_dy) * act_alpha;

            // Gravity destabilizing torque (g/L ~ 9.2 rad/s^2) + finite foot-sole support polygon
            let k_grav = 9.2_f32;
            let passive_foot = |theta: f32| {
                // Inside +-4.5 deg foot polygon: modest restoring torque; outside: tips over unless UVC steps
                let clamped = theta.clamp(-0.08_f32, 0.08_f32);
                clamped * 11.5_f32
            };

            // Contact-driven torso moment: `tau = r x F` about the CoM from
            // the ground-reaction CoP. This replaces the old scripted gait
            // sway — tilt now only changes when the contacts/CoP change.
            // The gain accounts for the lumped (distributed) body inertia.
            let h_com = self.last_balance.hip_height_m.clamp(0.42_f32, 0.72_f32);
            let mass = self.running_cfg.mass_kg.clamp(0.2_f32, 10.0_f32);
            let inertia = mass * h_com * h_com;
            let cop_pitch_moment = CONTACT_MOMENT_GAIN * self.world_grf_n
                * self.contact_cop_offset_m[1]
                / inertia;
            let cop_roll_moment = -CONTACT_MOMENT_GAIN * self.world_grf_n
                * self.contact_cop_offset_m[0]
                / inertia;

            // UVC catch-step shifts Center of Pressure `(act_foot_dx, act_foot_dy)` under leaning torso
            let k_cop_pitch = 26.0_f32;
            let k_cop_roll = 24.0_f32;
            let cop_restore_pitch = -(self.act_foot_dx / h_com) * k_cop_pitch;
            let cop_restore_roll = -(self.act_foot_dy / h_com) * k_cop_roll;

            // Direct ankle gyro rate damping + micro-stance compliance
            let k_ankle_pitch = 22.0_f32;
            let k_ankle_roll = 20.0_f32;
            let damping = 2.4_f32;

            let alpha_pitch = k_grav * self.torso_pitch_rad.sin()
                - passive_foot(self.torso_pitch_rad)
                + cop_restore_pitch
                + k_ankle_pitch * self.act_pitch_corr
                - damping * self.pitch_rate_rad_s
                + cop_pitch_moment;
            let alpha_roll = k_grav * self.torso_roll_rad.sin()
                - passive_foot(self.torso_roll_rad)
                + cop_restore_roll
                + k_ankle_roll * self.act_roll_corr
                - damping * self.roll_rate_rad_s
                + cop_roll_moment;

            self.pitch_rate_rad_s = (self.pitch_rate_rad_s + alpha_pitch * dt).clamp(-8.0, 8.0);
            self.roll_rate_rad_s = (self.roll_rate_rad_s + alpha_roll * dt).clamp(-8.0, 8.0);
            self.torso_pitch_rad = (self.torso_pitch_rad + self.pitch_rate_rad_s * dt).clamp(-1.35, 1.35);
            self.torso_roll_rad = (self.torso_roll_rad + self.roll_rate_rad_s * dt).clamp(-1.35, 1.35);
        } else {
            // Torque OFF: if knocked down (> 0.60 rad) stay resting tilted until Reset; otherwise workbench cradle damps upright
            self.act_pitch_corr = 0.0;
            self.act_roll_corr = 0.0;
            self.act_foot_dx = 0.0;
            self.act_foot_dy = 0.0;
            if self.torso_pitch_rad.abs() > 0.60 || self.torso_roll_rad.abs() > 0.60 {
                self.pitch_rate_rad_s *= (1.0_f32 - 8.0_f32 * dt).max(0.0);
                self.roll_rate_rad_s *= (1.0_f32 - 8.0_f32 * dt).max(0.0);
                self.torso_pitch_rad =
                    (self.torso_pitch_rad + self.pitch_rate_rad_s * dt).clamp(-1.25, 1.25);
                self.torso_roll_rad =
                    (self.torso_roll_rad + self.roll_rate_rad_s * dt).clamp(-1.25, 1.25);
            } else {
                let alpha_p = -14.0_f32 * self.torso_pitch_rad - 6.0_f32 * self.pitch_rate_rad_s;
                let alpha_r = -14.0_f32 * self.torso_roll_rad - 6.0_f32 * self.roll_rate_rad_s;
                self.pitch_rate_rad_s += alpha_p * dt;
                self.roll_rate_rad_s += alpha_r * dt;
                self.torso_pitch_rad += self.pitch_rate_rad_s * dt;
                self.torso_roll_rad += self.roll_rate_rad_s * dt;
            }
        }

        // Deterministic synthetic heading: accumulated sway from roll motion,
        // clamped so a violent fall cannot spin the robot arbitrarily far.
        self.torso_yaw_rad = (self.torso_yaw_rad + self.roll_rate_rad_s * 0.06_f32 * dt)
            .clamp(-0.6_f32, 0.6_f32);

        // Torque-capable running hop dynamics (inactive unless the gate is enabled).
        if self.running_mode {
            self.step_running_dynamics(dt);
        }

        let mut temp_c = [28u8; MAX_JOINTS];
        for i in 0..self.joint_count {
            temp_c[i] = self.temp_c_f[i].round() as u8;
        }

        let nominal_mv: i32 = if self.faults.voltage_sag {
            9_950
        } else {
            12_450
        };
        let sag_mv: i32 = ((total_current_ma as f32 * 0.001_f32) * 85.0_f32) as i32;
        let bus_voltage_mv = (nominal_mv - sag_mv).clamp(8_500, 16_800) as u16;

        RawServoFeedback {
            raw_angle_rad: self.raw_angle_rad,
            velocity_rad_s: self.velocity_rad_s,
            current_ma: self.current_ma,
            temp_c,
            bus_voltage_mv,
            bus_error: self.faults.servo_bus_error,
            imu: if self.faults.imu_disconnected {
                rk_proto::ImuState::default()
            } else {
                rk_proto::ImuState {
                    available: true,
                    pitch_rad: self.torso_pitch_rad,
                    roll_rad: self.torso_roll_rad,
                    yaw_rad: self.torso_yaw_rad,
                    pitch_rate_rad_s: self.pitch_rate_rad_s,
                    roll_rate_rad_s: self.roll_rate_rad_s,
                    yaw_rate_rad_s: self.roll_rate_rad_s * 0.06_f32,
                    gravity: gravity_from_tilt(self.torso_pitch_rad, self.torso_roll_rad),
                }
            },
            i3c_timeout_count: if self.faults.servo_bus_error { 1 } else { 0 },
            imu_source_mask: if self.faults.imu_disconnected {
                0
            } else {
                rk_proto::estimator_valid::LSM6DSV_OK
            },
        }
    }
}

impl ServoBusPort for SimulatedServoPlant {
    fn read_feedback(&mut self, dt_sec: f32) -> RawServoFeedback {
        self.step_plant(dt_sec)
    }

    fn write_command(&mut self, cmd: &RawServoCommand) {
        self.last_cmd = *cmd;
    }
}

/// All state owned by the simulated robot's [`RobotIo`] implementation.
///
/// This is the single source of truth for both the deterministic Embassy harness
/// path and the synchronous compatibility path. The public [`CopperSimRunner`]
/// dereferences to it, so existing field access (`runner.plant`, ...) and methods
/// keep working unchanged.
pub struct SimRobotIo {
    pub task: ControlTask,
    /// Shared streaming COBS framer used for host -> MCU command decoding.
    pub host_framer: CobsStreamFramer,
    /// Local (non-global) shared mailbox instance so replay stays deterministic.
    pub host_mailbox: HostLinkMailbox,
    pub plant: SimulatedServoPlant,
    uptime_ms_f: f64,
    host_decode_errors: u64,
    recording: bool,
    recorded_log: Option<CopperReplayLog>,
    /// `tick_index` of the first tick in the current recording. The persistent
    /// control task keeps its kernel across calls, so recorded indices are
    /// rebased to zero to keep each replay log self-contained.
    record_tick_base: Option<u64>,
    max_recorded_ticks: usize,
    mcu_model_info: ModelInfo,
    mcu_model_blob: Vec<u8>,
    // --- RobotIo backend state ---
    /// Lazily-created Embassy ticker driven by the harness mock clock.
    ticker: Option<Ticker>,
    /// `true` in harness mode: `now_ms()` reads the mock clock instead of uptime.
    use_mock_clock: bool,
    /// `Some(dt)` only on the synchronous compatibility path (variable dt tests).
    dt_override: Option<f32>,
    /// Deterministically queued host commands for the harness path.
    pending_cmds: VecDeque<HostToMcu>,
    /// Latest telemetry decoded on the host side of the simulated wire.
    last_host_rx: Option<McuToHost>,
}

impl SimRobotIo {
    pub fn new(
        joint_count: usize,
        initial_cal: &[JointCalibration],
        initial_raw_pose: [f32; MAX_JOINTS],
    ) -> Self {
        let mcu_model_blob = DEFAULT_MCU_FLASH_URDF.to_vec();
        let mcu_model_info =
            ModelInfo::from_static("rocky_mk1", ModelFormat::UrdfXml, &mcu_model_blob, 21);
        Self {
            task: ControlTask::new(joint_count, initial_cal),
            host_framer: CobsStreamFramer::new(),
            host_mailbox: HostLinkMailbox::new(),
            plant: SimulatedServoPlant::new(joint_count, initial_raw_pose),
            uptime_ms_f: 0.0,
            host_decode_errors: 0,
            recording: false,
            recorded_log: None,
            record_tick_base: None,
            max_recorded_ticks: 2400,
            mcu_model_info,
            mcu_model_blob,
            ticker: None,
            use_mock_clock: false,
            dt_override: None,
            pending_cmds: VecDeque::new(),
            last_host_rx: None,
        }
    }

    /// Replaces the simulated MCU's flash-resident URDF / kinematics blob (for testing custom URDF flashing).
    pub fn stage_mcu_model(
        &mut self,
        name: &str,
        format: ModelFormat,
        blob: Vec<u8>,
        joint_count: u8,
    ) {
        self.mcu_model_info = ModelInfo::from_static(name, format, &blob, joint_count);
        self.mcu_model_blob = blob;
    }

    /// Executes a full COBS-framed `ModelRequest` -> `ModelResponse` round-trip against the
    /// simulated MCU flash store, delegating request handling to the shared `FlashModelStore`.
    pub fn exchange_model_request(&mut self, req: ModelRequest) -> Option<ModelResponse> {
        let mut req_wire = [0u8; MAX_MODEL_COBS_FRAME_BYTES];
        let req_len = encode_model_req_cobs(&req, &mut req_wire).ok()?.len();
        let mcu_req = decode_model_req_cobs(&mut req_wire[..req_len]).ok()?;

        let store = rk_fw_core::model_store::FlashModelStore::new(
            self.mcu_model_info,
            &self.mcu_model_blob,
        );
        let mcu_resp = store.handle_request(mcu_req);

        let mut resp_wire = [0u8; MAX_MODEL_COBS_FRAME_BYTES];
        let resp_len = encode_model_resp_cobs(&mcu_resp, &mut resp_wire).ok()?.len();
        decode_model_resp_cobs(&mut resp_wire[..resp_len]).ok()
    }

    /// Decodes a delimited COBS `HostToMcu` frame via the shared `CobsStreamFramer`,
    /// forwarding typed codec errors into the local shared mailbox.
    fn decode_rx(&mut self, wire_frame: &mut [u8]) -> Option<HostToMcu> {
        let mailbox = &self.host_mailbox;
        let mut decoded = None;
        self.host_framer
            .push_bytes_with_error(wire_frame, |pkt| decoded = Some(pkt), |err| {
                mailbox.record_rx_codec_error(err)
            });
        decoded
    }

    /// Encodes `McuToHost` telemetry using the shared `encode_telemetry_frame` helper.
    fn encode_tx<'a>(
        &mut self,
        telem: &McuToHost,
        buf: &'a mut [u8; MAX_COBS_FRAME_BYTES],
    ) -> Result<&'a [u8], rk_proto::CodecError> {
        encode_telemetry_frame(telem, buf)
    }

    /// `(rx_crc_errors, rx_version_mismatches)` from the shared mailbox for explicit injection
    /// into `ControlTask::step_recorded`.
    pub fn wire_error_counts(&self) -> (u16, u16) {
        self.host_mailbox.wire_error_counts()
    }

    pub fn link_errors(&self) -> u64 {
        u64::from(self.host_framer.rx_err) + self.host_decode_errors
    }

    pub fn is_recording(&self) -> bool {
        self.recording
    }

    pub fn start_recording(&mut self) {
        self.recording = true;
        self.record_tick_base = None;
        self.recorded_log = Some(CopperReplayLog {
            version: 1,
            joint_count: self.task.kernel().joint_count(),
            initial_raw_pose: *self.plant.raw_angles(),
            initial_calibration: self.task.kernel().calibration().to_vec(),
            ticks: Vec::new(),
        });
    }

    pub fn stop_recording(&mut self) -> Option<CopperReplayLog> {
        self.recording = false;
        self.recorded_log.clone()
    }

    pub fn recorded_log(&self) -> Option<&CopperReplayLog> {
        self.recorded_log.as_ref()
    }

    /// Writes the recorded `.copper` log to disk on native Desktop targets.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn save_copper_log_to_path(&self, path: &std::path::Path) -> std::io::Result<()> {
        let Some(log) = &self.recorded_log else {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                "no copper log recorded",
            ));
        };
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        let bytes = serde_json::to_vec_pretty(log)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
        std::fs::write(path, bytes)
    }

    /// Synchronous compatibility path: one variable-dt simulation tick.
    ///
    /// This is the legacy `CopperSimRunner::tick()` entry point used by current
    /// tests and by the UI transport. It does **not** duplicate any kernel logic:
    /// after preparing the simulated wire it calls the same [`step_control_tick`]
    /// body that the deterministic Embassy harness task runs. Only the time source
    /// differs (accumulated `uptime_ms_f` + caller-supplied `dt_sec` here, mock
    /// clock + fixed `CONTROL_DT_SEC` in harness mode).
    pub fn tick(&mut self, dt_sec: f32, host_pkt: &HostToMcu) -> McuToHost {
        let dt = dt_sec.clamp(0.002_f32, 0.05_f32);
        // Forward the streamed running gate/gains to the torque-capable plant.
        self.plant.set_running_config(host_pkt.walk.running);
        self.uptime_ms_f += (dt as f64) * 1000.0;
        self.use_mock_clock = false;
        self.dt_override = Some(dt);

        // Encode HostToMcu over COBS and decode on the MCU bridge (unless heartbeat
        // was dropped). The decoded command is queued in the shared mailbox so
        // `RobotIo::take_cmd()` sees exactly what the hardware path would see.
        if !self.plant.faults().drop_heartbeat {
            let mut wire = [0u8; MAX_COBS_FRAME_BYTES];
            match encode_cobs(host_pkt, &mut wire) {
                Ok(encoded) => {
                    let len = encoded.len();
                    if let Some(decoded) = self.decode_rx(&mut wire[..len]) {
                        self.host_mailbox.push_rx_cmd(decoded);
                    }
                }
                Err(_) => self.host_decode_errors += 1,
            }
        }

        // Take the runner's ControlTask out so the whole IO can be borrowed by the
        // shared step function without aliasing the task borrow.
        let placeholder = ControlTask::new(1, &[JointCalibration::default(); MAX_JOINTS]);
        let mut task = core::mem::replace(&mut self.task, placeholder);
        step_control_tick(self, &mut task);
        self.task = task;

        self.last_host_rx.take().unwrap_or_default()
    }

    /// Injects a combat shove angular velocity impulse into the simulated torso.
    pub fn apply_push_impulse(&mut self, pitch_impulse_rad_s: f32, roll_impulse_rad_s: f32) {
        self.plant
            .apply_push_impulse(pitch_impulse_rad_s, roll_impulse_rad_s);
    }

    /// Resets the simulated torso strictly upright at 0 deg pitch/roll.
    pub fn reset_upright(&mut self) {
        self.plant.reset_upright();
    }

    /// Updates active get-up recovery progress so ground push-off lifts the fallen torso upright.
    pub fn set_recovery_progress(&mut self, progress: Option<f32>) {
        self.plant.set_recovery_progress(progress);
    }

    /// Injects a valid COBS frame with a flipped payload bit to exercise pre-COBS CRC-32 drop counting.
    pub fn inject_corrupted_frame(&mut self) {
        let mut wire = [0u8; MAX_COBS_FRAME_BYTES];
        if let Ok(encoded) = encode_cobs(&HostToMcu::default(), &mut wire) {
            let len = encoded.len();
            if len > 8 {
                let idx = len / 2;
                wire[idx] ^= 0x01;
                if wire[idx] == 0 {
                    wire[idx] = 0x02;
                }
                let _ = self.decode_rx(&mut wire[..len]);
                let (rx_crc_errors, rx_version_mismatches) = self.wire_error_counts();
                self.task
                    .kernel_mut()
                    .set_wire_error_counts(rx_crc_errors, rx_version_mismatches);
                return;
            }
        }
        let mut garbage = [0x05_u8, 0xFF, 0x00, 0x12, 0x00];
        let _ = self.decode_rx(&mut garbage);
    }
}

impl RobotIo for SimRobotIo {
    async fn wait_tick(&mut self) {
        if self.ticker.is_none() {
            // Created on first poll (at mock time zero) so the first harness step
            // fires exactly one control tick.
            self.ticker = Some(Ticker::every(Duration::from_millis(CONTROL_PERIOD_MS)));
        }
        self.ticker.as_mut().expect("ticker").next().await;
    }

    fn now_ms(&self) -> u32 {
        if self.use_mock_clock {
            Instant::now().as_millis() as u32
        } else {
            // Synchronous compatibility path: caller-driven uptime, unchanged
            // from the original `tick()` implementation.
            self.uptime_ms_f.round() as u32
        }
    }

    fn dt_sec(&self) -> f32 {
        self.dt_override.unwrap_or(rk_fw_core::config::CONTROL_DT_SEC)
    }

    fn joint_count(&self) -> usize {
        self.task.kernel().joint_count()
    }

    fn calibration(&self) -> [JointCalibration; MAX_JOINTS] {
        let mut cal = [JointCalibration::default(); MAX_JOINTS];
        let source = self.task.kernel().calibration();
        let n = source.len().min(MAX_JOINTS);
        cal[..n].copy_from_slice(&source[..n]);
        cal
    }

    fn take_cmd(&mut self) -> Option<HostToMcu> {
        let cmd = self
            .pending_cmds
            .pop_front()
            .or_else(|| self.host_mailbox.take_rx_cmd())?;
        // The plant needs the streamed walk target (ground velocity) and the
        // running gate on the same tick the kernel sees them.
        self.plant.set_walk_cmd(cmd.walk);
        self.plant.set_running_config(cmd.walk.running);
        Some(cmd)
    }

    fn wire_error_counts(&self) -> (u16, u16) {
        self.host_mailbox.wire_error_counts()
    }

    fn read_feedback(&mut self, dt_sec: f32) -> RawServoFeedback {
        self.plant.read_feedback(dt_sec)
    }

    fn write_command(&mut self, cmd: &RawServoCommand) {
        self.plant.write_command(cmd);
    }

    fn publish_telemetry(&mut self, telem: McuToHost) {
        // Feed the simulated balance plant exactly as the old runner did, then
        // round-trip telemetry through the shared COBS helpers.
        self.plant.set_balance_telemetry(telem.balance);
        let mut mcu_wire = [0u8; MAX_COBS_FRAME_BYTES];
        let mut decoded = match self.encode_tx(&telem, &mut mcu_wire) {
            Ok(encoded) => {
                let len = encoded.len();
                decode_mcu_cobs(&mut mcu_wire[..len]).unwrap_or_else(|_| {
                    self.host_decode_errors += 1;
                    telem
                })
            }
            Err(_) => {
                self.host_decode_errors += 1;
                telem
            }
        };
        // Publish the plant's world-frame state alongside the MCU's own
        // support-relative estimate. Hardware leaves the WORLD channel clear;
        // the viewport/HUD use these fields to show achieved locomotion.
        decoded.balance.world_pos_m = self.plant.world_position_m();
        decoded.balance.world_vel_m_s = self.plant.world_velocity_m_s();
        decoded.estimator.valid_mask |= estimator_valid::WORLD;
        decoded.estimator.contact_mask = self.plant.contact_mask();
        decoded.estimator.grf_left_n = self.plant.foot_contact(false).grf_n;
        decoded.estimator.grf_right_n = self.plant.foot_contact(true).grf_n;
        decoded.estimator.valid_mask |= estimator_valid::GRF;
        self.last_host_rx = Some(decoded);
    }

    fn record_tick(&mut self, tick: &ControlTick) {
        if self.recording {
            if let Some(log) = self.recorded_log.as_mut() {
                if log.ticks.len() < self.max_recorded_ticks {
                    // Replay verification starts a fresh `ControlTask` at index
                    // zero. Rebase the persistent task's running index so a log
                    // recorded across many fixed-step calls stays replayable.
                    let base = *self.record_tick_base.get_or_insert(tick.tick_index);
                    let mut recorded = *tick;
                    recorded.tick_index = tick.tick_index.saturating_sub(base);
                    log.ticks.push(recorded);
                }
            }
        }
    }
}

/// Future type spawned into the deterministic harness executor. A boxed future
/// is used instead of `#[embassy_executor::task]` because an infinite `#[task]`
/// never releases its static pool slot. The storage is created once per session
/// (see [`CopperSimRunner::new`]) and deliberately never reclaimed: the
/// process-global `MockDriver` keeps a waker for the task's periodic alarm, so
/// freeing either the task or its IO while the alarm is pending would leave a
/// dangling waker in the timer queue.
type HarnessFuture = core::pin::Pin<Box<dyn core::future::Future<Output = ()> + 'static>>;

/// Raw IO handle handed to the long-lived control task.
///
/// The task owns only a pointer, not a `&'static mut SimRobotIo`: the runner
/// needs `&mut` access to the same IO between polls (fault injection, replay
/// recording, command handoff). A raw pointer avoids a long-lived exclusive
/// reference inside the future that would alias the runner's accesses. All
/// dereferences happen strictly inside `poll`, which this executor drives
/// synchronously and never concurrently with the runner.
struct PersistentIoPtr(*mut SimRobotIo);

impl RobotIo for PersistentIoPtr {
    async fn wait_tick(&mut self) {
        // SAFETY: only this session's executor polls the future, and it is never
        // polled while the runner holds a `&mut` to the same IO.
        unsafe { (&mut *self.0).wait_tick().await }
    }

    fn now_ms(&self) -> u32 {
        unsafe { (&*self.0).now_ms() }
    }

    fn dt_sec(&self) -> f32 {
        unsafe { (&*self.0).dt_sec() }
    }

    fn joint_count(&self) -> usize {
        unsafe { (&*self.0).joint_count() }
    }

    fn calibration(&self) -> [JointCalibration; MAX_JOINTS] {
        unsafe { (&*self.0).calibration() }
    }

    fn take_cmd(&mut self) -> Option<HostToMcu> {
        unsafe { (&mut *self.0).take_cmd() }
    }

    fn wire_error_counts(&self) -> (u16, u16) {
        unsafe { (&*self.0).wire_error_counts() }
    }

    fn read_feedback(&mut self, dt_sec: f32) -> RawServoFeedback {
        unsafe { (&mut *self.0).read_feedback(dt_sec) }
    }

    fn write_command(&mut self, cmd: &RawServoCommand) {
        unsafe { (&mut *self.0).write_command(cmd) }
    }

    fn publish_telemetry(&mut self, telem: McuToHost) {
        unsafe { (&mut *self.0).publish_telemetry(telem) }
    }

    fn record_tick(&mut self, tick: &ControlTick) {
        unsafe { (&mut *self.0).record_tick(tick) }
    }

    fn post_tick(&mut self, now_ms: u32) {
        unsafe { (&mut *self.0).post_tick(now_ms) }
    }
}

/// Public simulation runner. One instance owns exactly one deterministic
/// Embassy executor and one spawned shared control task for its whole lifetime.
/// Repeated fixed-step calls reuse them instead of creating a fresh executor
/// against the process-global `embassy_time::MockDriver`.
///
/// All methods and fields (most notably `plant`) are reached by auto-deref to
/// [`SimRobotIo`], preserving the historic API used by `rk-link` and tests.
pub struct CopperSimRunner {
    io: NonNull<SimRobotIo>,
    harness: DeterministicHarness,
    // Kept alive for the session; see `HarnessFuture` docs for why it is never
    // reclaimed in `Drop`.
    _storage: &'static raw::TaskStorage<HarnessFuture>,
}

// SAFETY: Embassy's `raw::Executor` and our raw IO pointer are deliberately
// !Send/!Sync because they must never be *polled* concurrently. This runner
// is owned by exactly one `SimTransport` (itself kept behind the MCP server's
// state mutex), and every interaction — including `harness.step_n` — requires
// `&mut self`, so no two threads can poll the executor or touch the IO at the
// same time. Moving the owner between threads is therefore safe: the executor
// and IO heap allocations stay put and are never reclaimed while an alarm may
// still reference them.
unsafe impl Send for CopperSimRunner {}

impl Deref for CopperSimRunner {
    type Target = SimRobotIo;
    fn deref(&self) -> &Self::Target {
        unsafe { self.io.as_ref() }
    }
}

impl DerefMut for CopperSimRunner {
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { self.io.as_mut() }
    }
}

impl CopperSimRunner {
    pub fn new(
        joint_count: usize,
        initial_cal: &[JointCalibration],
        initial_raw_pose: [f32; MAX_JOINTS],
    ) -> Self {
        // SAFETY: the box is never freed — the long-lived task holds a raw
        // pointer to it, and the global mock timer queue may hold a waker into
        // that task after the last `step_n`.
        let io = Box::into_raw(Box::new(SimRobotIo::new(
            joint_count,
            initial_cal,
            initial_raw_pose,
        )));
        unsafe {
            (*io).use_mock_clock = true;
            (*io).dt_override = None;
        }

        let harness = DeterministicHarness::new();
        let storage: &'static raw::TaskStorage<HarnessFuture> =
            Box::leak(Box::new(raw::TaskStorage::new()));
        let token = storage
            .spawn(move || Box::pin(rk_fw_core::control_task::control_task(PersistentIoPtr(io))))
            .expect("fresh harness task storage must be spawnable");
        harness.spawner().spawn(token);

        Self {
            io: NonNull::new(io).expect("Box::into_raw never returns null"),
            harness,
            _storage: storage,
        }
    }

    /// Reset the global deterministic clock. Only valid before the first tick
    /// has been stepped: once the task has registered its `Ticker`, resetting
    /// the clock strands an alarm in the process-global queue and the control
    /// task will never be woken again. Fixed-step sessions never call this.
    pub fn reset_mock_clock(&self) {
        self.harness.reset_clock();
    }

    /// Legacy test helper: run `steps` fixed ticks against the shared task.
    ///
    /// Resets the clock only if this session has not been started yet, so
    /// absolute `now_ms` assertions remain valid. Never re-spawns the task.
    pub fn run_harness(mut self, steps: usize, host_cmds: &[HostToMcu]) -> (Self, McuToHost) {
        if !self.harness.is_started() {
            self.reset_mock_clock();
        }
        let telem = self.step_harness(steps, host_cmds);
        (self, telem)
    }

    /// Backwards-compatible consuming form that keeps simulated time running
    /// continuously across calls (no clock reset, no re-spawn).
    pub fn run_harness_continuous(mut self, steps: usize, host_cmds: &[HostToMcu]) -> (Self, McuToHost) {
        let telem = self.step_harness(steps, host_cmds);
        (self, telem)
    }

    /// Advance this session's persistent executor by exactly `steps` fixed
    /// 5 ms ticks and return the last telemetry frame.
    ///
    /// Host commands are queued once per tick. The task's `Ticker`, the kernel's
    /// internal state and the simulated clock all persist across calls; no
    /// executor is created and no clock reset is performed here.
    pub fn step_harness(&mut self, steps: usize, host_cmds: &[HostToMcu]) -> McuToHost {
        if steps == 0 {
            return self.io_mut().last_host_rx.take().unwrap_or_default();
        }
        {
            let io = self.io_mut();
            io.pending_cmds.extend(host_cmds.iter().copied());
            io.use_mock_clock = true;
            io.dt_override = None;
            io.last_host_rx = None;
        }
        self.harness.step_n(steps);
        self.io_mut().last_host_rx.take().unwrap_or_default()
    }

    fn io_mut(&mut self) -> &mut SimRobotIo {
        unsafe { self.io.as_mut() }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rk_proto::{ControlMode, FaultFlags, WalkCmd};
    use std::sync::Mutex;

    /// `embassy_time::MockDriver` is process-global. Tests that reset the clock
    /// or step a harness must not overlap with each other.
    static HARNESS_TEST_LOCK: Mutex<()> = Mutex::new(());

    /// The deterministic harness must drive the *shared* async control task, and
    /// replay recording must survive through `RobotIo::record_tick`.
    #[test]
    fn harness_drives_shared_control_task_and_records_replay() {
        let _guard = HARNESS_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let cal = [JointCalibration::default(); MAX_JOINTS];
        let mut sim = CopperSimRunner::new(6, &cal, [0.0; MAX_JOINTS]);
        sim.start_recording();
        let cmds = vec![
            HostToMcu {
                seq: 1,
                mode: ControlMode::Teach,
                ..Default::default()
            };
            4
        ];

        let (mut sim, last) = sim.run_harness(4, &cmds);
        assert_eq!(last.active_mode, ControlMode::Teach);

        let log = sim.stop_recording().expect("harness ticks recorded");
        assert_eq!(log.ticks.len(), 4);
        assert_eq!(log.ticks[0].now_ms, 5);
        assert_eq!(log.ticks[3].now_ms, 20);
        for tick in &log.ticks {
            assert_eq!(tick.dt_sec, rk_fw_core::config::CONTROL_DT_SEC);
        }
        let check = verify_deterministic_replay(&log);
        assert!(check.exact_match);
        assert_eq!(check.matched_ticks, 4);
    }

    #[test]
    fn copper_sim_end_to_end_watchdog_and_deterministic_replay() {
        let cal = [JointCalibration {
            min_rad: -2.0,
            max_rad: 2.0,
            ..Default::default()
        }; 6];
        let mut sim = CopperSimRunner::new(6, &cal, [0.0; MAX_JOINTS]);
        sim.start_recording();

        let mut cmd = HostToMcu {
            seq: 1,
            mode: ControlMode::Drive,
            torque_mask: 0b11_1111,
            speed_limit_pct: 50,
            ..Default::default()
        };
        cmd.target_rad[0] = 0.8;

        let mut last = McuToHost::default();
        for i in 1..=20 {
            cmd.seq = i;
            last = sim.tick(0.01, &cmd);
        }
        assert_eq!(last.active_mode, ControlMode::Drive);
        assert!(last.measured_rad[0] > 0.05);

        // Drop heartbeat for >100 ms -> WATCHDOG_TIMEOUT trips & zeroes torque
        sim.plant.faults_mut().drop_heartbeat = true;
        for _ in 0..15 {
            last = sim.tick(0.01, &cmd);
        }
        assert_eq!(last.active_mode, ControlMode::EStop);
        assert_eq!(last.torque_state_mask, 0);
        assert!(last.faults.contains(FaultFlags::WATCHDOG_TIMEOUT));

        // Deterministic replay matches every single recorded CopperList tick bit-for-bit
        let log = sim.stop_recording().expect("log recorded");
        let check = verify_deterministic_replay(&log);
        assert!(check.exact_match);
        assert_eq!(check.matched_ticks, check.total_ticks);

        // Desktop `.copper` file write verification
        #[cfg(not(target_arch = "wasm32"))]
        {
            let tmp = std::env::temp_dir().join("rocky_studio_test.copper");
            sim.save_copper_log_to_path(&tmp).expect("write .copper");
            assert!(tmp.exists());
            let _ = std::fs::remove_file(&tmp);
        }
    }

    #[test]
    fn mcu_uvc_balance_catch_steps_combat_shove_and_unassisted_falls() {
        let cal = [JointCalibration {
            min_rad: -2.2,
            max_rad: 2.2,
            ..Default::default()
        }; 21];
        let mut sim = CopperSimRunner::new(21, &cal, [0.0; MAX_JOINTS]);

        let mut cmd = HostToMcu {
            seq: 1,
            mode: ControlMode::Drive,
            torque_mask: (1u32 << 21) - 1,
            speed_limit_pct: 100,
            walk: WalkCmd {
                walk_enabled: false,
                balance_enabled: true,
                ..Default::default()
            },
            ..Default::default()
        };

        // Warm up in Drive mode through CrouchInit -> CalibrateImu -> ReadyWatch
        for i in 1..=30 {
            cmd.seq = i;
            let _ = sim.tick(0.01, &cmd);
        }

        // Inject a sharp combat shove forward + sideways
        sim.apply_push_impulse(1.6, -1.1);
        let mut peak_pitch = 0.0_f32;
        let mut saw_catch_step = false;
        let mut last = McuToHost::default();
        for i in 31..=320 {
            cmd.seq = i;
            last = sim.tick(0.01, &cmd);
            peak_pitch = peak_pitch.max(last.imu.pitch_rad.abs());
            if last.balance.uvc_phase == rk_proto::uvc_phase::ACTIVE_CATCH_STEP {
                saw_catch_step = true;
            }
        }
        assert!(peak_pitch > 0.12, "shove must visibly tilt torso");
        assert!(saw_catch_step, "UVC must trigger ActiveCatchStep on shove");
        assert_eq!(last.active_mode, ControlMode::Drive, "must stay standing");
        assert!(
            last.imu.pitch_rad.abs() < 0.06 && last.imu.roll_rad.abs() < 0.06,
            "MCU UVC balance must recover upright (pitch={:.3}, roll={:.3})",
            last.imu.pitch_rad,
            last.imu.roll_rad
        );

        // Now disable balance and inject a shove -> Rocky tips past fall cutoff, tucks, and latches TILT_FALL_CUTOFF
        cmd.walk.balance_enabled = false;
        sim.apply_push_impulse(2.1, 0.0);
        for i in 321..=520 {
            cmd.seq = i;
            last = sim.tick(0.01, &cmd);
        }
        assert_eq!(last.active_mode, ControlMode::EStop);
        assert!(last.faults.contains(FaultFlags::TILT_FALL_CUTOFF));
    }

    #[test]
    fn simulated_gravity_matches_forward_tilt() {
        let mut plant = SimulatedServoPlant::new(6, [0.0; MAX_JOINTS]);

        // Upright: gravity points straight down in body frame, yaw is zero.
        let upright = plant.step_plant(0.01);
        assert!(upright.imu.available);
        assert_eq!(upright.imu.yaw_rad, 0.0);
        let g0 = upright.imu.gravity;
        assert!((g0[0]).abs() < 1e-6 && (g0[2]).abs() < 1e-6);
        assert!((g0[1] + 9.81).abs() < 1e-6);

        // Inject a pure forward pitch impulse and let the plant settle into a lean.
        plant.apply_push_impulse(1.4, 0.0);
        let mut tilted = upright;
        for _ in 0..40 {
            tilted = plant.step_plant(0.01);
        }
        let pitch = tilted.imu.pitch_rad;
        assert!(pitch > 0.1, "expected forward lean, pitch={pitch}");

        // Rotating world-down by the inverse tilt quaternion yields a body-frame
        // gravity whose Z component grows with forward lean (classified as prone).
        let g = tilted.imu.gravity;
        let expected_z = 9.81_f32 * pitch.sin();
        assert!(
            (g[2] - expected_z).abs() < 1e-4,
            "gravity Z must match forward tilt: got {:.4}, expected {:.4}",
            g[2],
            expected_z
        );
        assert!(g[2] > 0.0, "forward lean must produce positive body-Z gravity");
        assert!((g[1] + 9.81_f32 * pitch.cos()).abs() < 1e-4);
        assert!(g[0].abs() < 1e-4, "pure pitch must not introduce lateral gravity");

        // Reset zeroes yaw and restores upright gravity.
        plant.reset_upright();
        let reset = plant.step_plant(0.01);
        assert_eq!(reset.imu.yaw_rad, 0.0);
        assert!((reset.imu.gravity[2]).abs() < 1e-6);
    }

    #[test]
    fn recovery_progress_lifts_fallen_torso_back_to_upright() {
        let cal = [JointCalibration {
            min_rad: -2.6,
            max_rad: 2.6,
            ..Default::default()
        }; 21];
        let mut sim = CopperSimRunner::new(21, &cal, [0.0; MAX_JOINTS]);

        let mut cmd = HostToMcu {
            seq: 1,
            mode: ControlMode::Drive,
            torque_mask: (1u32 << 21) - 1,
            speed_limit_pct: 100,
            walk: WalkCmd {
                walk_enabled: false,
                balance_enabled: true,
                ..Default::default()
            },
            ..Default::default()
        };

        // 1. Warm up standing in Drive mode with default UVC balance enabled.
        for i in 1..=30 {
            cmd.seq = i;
            let _ = sim.tick(0.016, &cmd);
        }

        // 2. Inject a simulated prone fall shove -> must trip TILT_FALL_CUTOFF and settle prone.
        sim.apply_push_impulse(6.0, 0.0);
        let mut last = McuToHost::default();
        for i in 31..=160 {
            cmd.seq = i;
            last = sim.tick(0.016, &cmd);
        }
        assert_eq!(last.active_mode, ControlMode::EStop);
        assert!(last.faults.contains(FaultFlags::TILT_FALL_CUTOFF));
        assert_eq!(
            last.balance.fall_orientation,
            rk_proto::fall_orientation::PRONE
        );
        assert!(
            last.imu.pitch_rad > 0.66 && last.imu.pitch_rad < 1.32,
            "expected fallen prone pitch in (0.66, 1.32), got {}",
            last.imu.pitch_rad
        );

        // 3. Temporarily raise fall_cutoff_rad to 1.35 and disable UVC balance, then ClearFaults.
        let mut rec_gains = rk_proto::BalanceGains::default();
        rec_gains.fall_cutoff_rad = 1.35;
        cmd.mode = ControlMode::Teach;
        cmd.walk.balance_enabled = false;
        cmd.config = rk_proto::ConfigCmd::SetBalanceGains(rec_gains);
        cmd.seq = 161;
        let _ = sim.tick(0.016, &cmd);

        cmd.config = rk_proto::ConfigCmd::ClearFaults;
        cmd.seq = 162;
        last = sim.tick(0.016, &cmd);
        assert_eq!(last.active_mode, ControlMode::Teach);
        assert!(!last.faults.contains(FaultFlags::TILT_FALL_CUTOFF));

        // 4. Arm Drive and sweep recovery_progress 0.0 -> 1.0 -> must NOT re-trip and must lift upright.
        cmd.config = rk_proto::ConfigCmd::None;
        cmd.mode = ControlMode::Drive;
        for step_idx in 0..=160 {
            cmd.seq = 163 + step_idx;
            let progress = (step_idx as f32 / 160.0).min(1.0);
            sim.set_recovery_progress(Some(progress));
            last = sim.tick(0.016, &cmd);
            assert_eq!(
                last.active_mode,
                ControlMode::Drive,
                "recovery must stay in Drive at step {step_idx} (pitch={}, faults={:?})",
                last.imu.pitch_rad,
                last.faults
            );
        }
        sim.set_recovery_progress(None);

        // 5. Restore nominal fall cutoff and return to Teach -> verify upright stance.
        cmd.mode = ControlMode::Teach;
        cmd.config = rk_proto::ConfigCmd::SetBalanceGains(rk_proto::BalanceGains::default());
        cmd.seq = 400;
        last = sim.tick(0.016, &cmd);
        assert!(
            last.imu.pitch_rad.abs() < 0.08 && last.imu.roll_rad.abs() < 0.08,
            "recovery progress must lift torso upright (pitch={}, roll={})",
            last.imu.pitch_rad,
            last.imu.roll_rad
        );
    }

    #[test]
    fn compliance_scale_and_crc_injection_propagate_end_to_end() {
        let cal = [JointCalibration {
            min_rad: -2.2,
            max_rad: 2.2,
            ..Default::default()
        }; 6];
        let mut sim_stiff = CopperSimRunner::new(6, &cal, [0.0; MAX_JOINTS]);
        let mut sim_soft = CopperSimRunner::new(6, &cal, [0.0; MAX_JOINTS]);

        let mut soft_gains = rk_proto::BalanceGains::default();
        soft_gains.compliance_scale = 0.25;
        let init_soft = HostToMcu {
            seq: 1,
            mode: ControlMode::Teach,
            config: rk_proto::ConfigCmd::SetBalanceGains(soft_gains),
            ..Default::default()
        };
        let _ = sim_soft.tick(0.01, &init_soft);

        let mut cmd = HostToMcu {
            seq: 2,
            mode: ControlMode::Drive,
            torque_mask: 0b11_1111,
            speed_limit_pct: 100,
            ..Default::default()
        };
        cmd.target_rad[0] = 1.0;

        let mut stiff_peak_ma = 0u16;
        let mut soft_peak_ma = 0u16;
        for step in 2..=30 {
            cmd.seq = step;
            let t_stiff = sim_stiff.tick(0.01, &cmd);
            let t_soft = sim_soft.tick(0.01, &cmd);
            stiff_peak_ma = stiff_peak_ma.max(t_stiff.current_ma[0]);
            soft_peak_ma = soft_peak_ma.max(t_soft.current_ma[0]);
        }
        assert!(
            soft_peak_ma < stiff_peak_ma,
            "25% compliance scale must reduce peak servo current ({soft_peak_ma} vs {stiff_peak_ma} mA)"
        );

        sim_soft.inject_corrupted_frame();
        cmd.seq = 31;
        let after_corrupt = sim_soft.tick(0.01, &cmd);
        assert!(
            after_corrupt.rx_crc_errors >= 1,
            "injected bit-flip must increment rx_crc_errors in telemetry"
        );
    }

    // --- Torque-capable running plant tests ----------------------------------

    /// Minimal deterministic thrust command: joint 9 (a leg joint) is the sole
    /// torque-mode actuator, so its torque maps directly to vertical thrust.
    fn running_thrust_cmd(thrust_n: f32) -> RawServoCommand {
        let mut cmd = RawServoCommand::default();
        cmd.torque_mask = 1u32 << 9;
        cmd.command_mode[9] = JointCommandMode::Torque.to_wire();
        cmd.torque_nm[9] = (thrust_n * RUNNING_LEVER_ARM_M)
            .clamp(RUNNING_MIN_THRUST_N * RUNNING_LEVER_ARM_M, RUNNING_MAX_THRUST_N * RUNNING_LEVER_ARM_M);
        cmd
    }

    /// Drives one deterministic SLIP hop: seeds a touchdown at `-sqrt(2 g H)` and
    /// cancels the stance spring with signed thrust so the CoM accelerates at
    /// `+PUSH_ACCEL` until the target takeoff velocity is reached, then coasts to
    /// liftoff. This is a validation driver only; the plant stays a pure physics model.
    fn drive_single_hop(cfg: RunningConfig, ticks: usize) -> (SimulatedServoPlant, usize) {
        const PUSH_ACCEL: f32 = 20.0_f32;
        const DT: f32 = 0.001_f32;
        let mut plant = SimulatedServoPlant::new(21, [0.0; MAX_JOINTS]);
        plant.set_running_config(cfg);
        let takeoff_vel = (2.0 * RUNNING_G_M_S2 * cfg.hop_height_m).sqrt();
        plant.set_running_state(cfg.leg_rest_length_m, -takeoff_vel, false);

        let mut flight_steps = 0usize;
        for _ in 0..ticks {
            let h = plant.com_height_m();
            let v = plant.com_velocity_z_m_s();
            let stance = h <= cfg.leg_rest_length_m + 1.0e-4_f32;
            let a_des = if !stance {
                0.0
            } else if v < takeoff_vel {
                PUSH_ACCEL
            } else {
                0.0
            };
            let thrust = if stance {
                (cfg.mass_kg * (a_des + RUNNING_G_M_S2) - plant.ground_reaction_n())
                    .clamp(RUNNING_MIN_THRUST_N, RUNNING_MAX_THRUST_N)
            } else {
                0.0
            };
            plant.write_command(&running_thrust_cmd(thrust));
            let _ = plant.step_plant(DT);
            if plant.in_flight() {
                flight_steps += 1;
            }
        }
        (plant, flight_steps)
    }

    #[test]
    fn running_hop_height_tracks_target() {
        let cfg = RunningConfig {
            enabled: true,
            hop_height_m: 0.03,
            ..Default::default()
        };
        // 170 ms at 1 kHz: covers the 0.12–0.15 s stance and the ballistic apex.
        let (plant, _) = drive_single_hop(cfg, 170);
        let achieved = plant.hop_height_m();
        assert!(
            (achieved - cfg.hop_height_m).abs() < 0.008,
            "hop height {achieved:.4} m must track target {:.3} m within 8 mm",
            cfg.hop_height_m
        );
        assert!(plant.running_enabled());
        assert!(plant.apex_height_m() > cfg.leg_rest_length_m);
    }

    #[test]
    fn running_flight_detector_debounces_and_clears_on_touchdown() {
        let cfg = RunningConfig {
            enabled: true,
            hop_height_m: 0.03,
            ..Default::default()
        };
        let (plant, flight_steps) = drive_single_hop(cfg, 170);
        assert!(flight_steps > 0, "flight detector must assert during a hop");

        // Expected ballistic time is 2*v_to/g ≈ 0.156 s; debounce removes ~12 ms.
        let flight_s = flight_steps as f32 * 0.001_f32;
        assert!(
            (0.06..=0.25).contains(&flight_s),
            "flight duration {flight_s:.3} s must be a plausible ballistic arc"
        );
        assert!(
            flight_s >= RUNNING_FLIGHT_DEBOUNCE_S,
            "flight must persist beyond the 12 ms debounce"
        );
        assert!(plant.in_flight(), "plant must be airborne near the apex");

        // After the apex the plant comes back down: flight must clear on touchdown.
        let mut settled = plant;
        let zero = running_thrust_cmd(0.0);
        for _ in 0..120 {
            settled.write_command(&zero);
            let _ = settled.step_plant(0.005_f32);
        }
        assert!(!settled.in_flight(), "flight must clear at touchdown");
    }

    /// Drops the plant from its apex with zero thrust and records the landing peak.
    fn landing_peak(cfg: RunningConfig) -> (f32, f32, u16) {
        let mut plant = SimulatedServoPlant::new(21, [0.0; MAX_JOINTS]);
        plant.set_running_config(cfg);
        let rest = cfg.leg_rest_length_m;
        plant.set_running_state(rest + cfg.hop_height_m, 0.0, true);
        let zero = running_thrust_cmd(0.0);
        let mut peak_force = 0.0_f32;
        let mut peak_current = 0_u16;
        for _ in 0..400 {
            plant.write_command(&zero);
            let fb = plant.step_plant(0.005_f32);
            peak_force = peak_force.max(plant.peak_landing_force_n());
            for i in 0..plant.joint_count {
                peak_current = peak_current.max(fb.current_ma[i]);
            }
        }
        (peak_force, peak_force * RUNNING_LEVER_ARM_M, peak_current)
    }

    #[test]
    fn compliant_landing_does_not_spike_torque_or_current() {
        // Compliant landing uses the specified 25–40 ms ramp at the upper end (40 ms).
        // At 30 ms the ramp completes right at maximum leg compression (~30 ms for
        // k=4000, m=1.5), so its peak force is indistinguishable from a rigid landing;
        // 40 ms keeps stiffness low through the compression and is the physical
        // compliance mechanism described in research §2.3.
        let soft = RunningConfig {
            enabled: true,
            hop_height_m: 0.03,
            landing_ramp_s: 0.04,
            ..Default::default()
        };
        // Rigid position-hold equivalent: full stiffness from touchdown with no ramp.
        // 8000 N/m represents the effectively rigid impedance the ramp avoids.
        let rigid = RunningConfig {
            landing_stiffness_touchdown_n_m: 8000.0,
            landing_stiffness_nominal_n_m: 8000.0,
            landing_ramp_s: 0.002,
            ..soft
        };

        let (soft_force, soft_torque, soft_ma) = landing_peak(soft);
        let (rigid_force, rigid_torque, _rigid_ma) = landing_peak(rigid);
        assert!(soft_force > 0.0, "landing ramp must register ground reaction");
        assert!(
            soft_force < rigid_force * 0.75,
            "compliant touchdown must cut landing force ({soft_force:.1} vs {rigid_force:.1} N)"
        );
        // v2 BLDC joints tolerate more than the v1 1.8 N·m ceiling; 3.5 N·m is a
        // modest torque-capable ceiling that the compliant landing stays under.
        assert!(
            soft_torque < 3.5,
            "compliant landing torque {soft_torque:.2} N·m must stay under the v2 ceiling"
        );
        assert!(
            soft_torque < rigid_torque,
            "compliance must reduce equivalent joint torque ({soft_torque:.2} vs {rigid_torque:.2} N·m)"
        );
        assert!(
            soft_ma < 2200,
            "passive compliant landing must not spike current ({soft_ma} mA)"
        );
    }

    #[test]
    fn running_gate_defaults_off_and_leaves_default_plant_unchanged() {
        let cfg = RunningConfig::default();
        assert!(!cfg.enabled, "running gate must default OFF (v1 unaffected)");
        let mut plant = SimulatedServoPlant::new(6, [0.0; MAX_JOINTS]);
        plant.set_running_config(cfg);
        assert!(!plant.running_enabled());
        assert!(!plant.in_flight());

        let mut cmd = RawServoCommand::default();
        cmd.torque_mask = 0b11_1111;
        cmd.raw_target_rad[0] = 0.8;
        for _ in 0..20 {
            plant.write_command(&cmd);
            let _ = plant.step_plant(0.01);
        }
        assert!(
            plant.raw_angles()[0] > 0.1,
            "default position-servo plant must still track targets when running is off"
        );
        assert!((plant.com_height_m() - cfg.leg_rest_length_m).abs() < 1e-6);
    }

    /// Group 4: the full shared control path must actually translate the plant
    /// forward when walking is commanded, with alternating contacts and tilt
    /// that stays below the fall cutoff.
    #[test]
    fn end_to_end_walking_translates_with_alternating_contacts() {
        let _guard = HARNESS_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let cal = [JointCalibration {
            min_rad: -2.2,
            max_rad: 2.2,
            ..Default::default()
        }; 21];
        let sim = CopperSimRunner::new(21, &cal, [0.0; MAX_JOINTS]);
        let mut cmd = HostToMcu {
            seq: 1,
            mode: ControlMode::Drive,
            torque_mask: (1u32 << 21) - 1,
            speed_limit_pct: 100,
            walk: WalkCmd {
                walk_enabled: true,
                balance_enabled: true,
                vx: 0.5,
                cadence_hz: 1.55,
                ..Default::default()
            },
            ..Default::default()
        };
        let cmds: Vec<HostToMcu> = (1..=800)
            .map(|i| {
                cmd.seq = i;
                cmd
            })
            .collect();

        // 800 fixed 5 ms ticks = 4 s of deterministic simulated time.
        let (sim, last) = sim.run_harness(800, &cmds);
        assert_eq!(last.active_mode, ControlMode::Drive, "walking must stay in Drive");
        assert!(
            sim.plant.forward_displacement_m() > 0.10_f32,
            "vx>0 must produce real forward displacement, got {:.3} m",
            sim.plant.forward_displacement_m()
        );
        assert!(
            sim.plant.contact_swaps() >= 6,
            "contacts must alternate across the 4 s walk, got {} swaps",
            sim.plant.contact_swaps()
        );
        let ticks = sim.plant.contact_ticks();
        assert!(
            ticks[0] > 0 && ticks[1] > 0,
            "both soles must carry ground reaction during the walk, got {:?}",
            ticks
        );
        let cutoff = rk_proto::BalanceGains::default().fall_cutoff_rad;
        assert!(
            last.imu.pitch_rad.abs() < cutoff && last.imu.roll_rad.abs() < cutoff,
            "tilt must stay below fall cutoff: pitch={:.3} roll={:.3}",
            last.imu.pitch_rad,
            last.imu.roll_rad
        );
        assert!(!last.faults.contains(FaultFlags::TILT_FALL_CUTOFF));

        // World state is published on the wire for the viewport/HUD.
        assert_ne!(last.estimator.valid_mask & estimator_valid::WORLD, 0);
        assert!(
            (last.balance.world_pos_m[2] - sim.plant.world_position_m()[2]).abs() < 1e-6_f32
        );
        assert!(last.balance.world_vel_m_s[2] > 0.0_f32, "achieved speed must be forward");
    }

    /// Many sequential fixed-step calls in one process must reuse a single
    /// executor: no clock resets, no stale timer wakers, no panic. The recorded
    /// `now_ms` must keep increasing and replay must stay bit-exact.
    #[test]
    fn persistent_harness_survives_many_fixed_step_calls() {
        let _guard = HARNESS_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let cal = [JointCalibration::default(); MAX_JOINTS];
        let mut sim = CopperSimRunner::new(6, &cal, [0.0; MAX_JOINTS]);
        sim.start_recording();

        let mut last_ms = None;
        for i in 1..=25u32 {
            let cmd = HostToMcu {
                seq: i,
                mode: ControlMode::Teach,
                ..Default::default()
            };
            let last = sim.step_harness(4, &[cmd]);
            assert_eq!(last.active_mode, ControlMode::Teach);
            let log = sim.recorded_log().expect("recording active");
            let now_ms = log.ticks.last().expect("tick recorded").now_ms;
            if let Some(prev) = last_ms {
                assert!(
                    now_ms > prev,
                    "call {i}: clock must advance monotonically ({now_ms} after {prev})"
                );
            }
            last_ms = Some(now_ms);
        }

        let log = sim.stop_recording().expect("recorded log");
        assert_eq!(log.ticks.len(), 25 * 4);
        let check = verify_deterministic_replay(&log);
        assert!(check.exact_match, "replay must stay exact across calls: {check:?}");
        assert_eq!(check.matched_ticks, check.total_ticks);
    }

    /// A late host command must be consumed by the already-running task on the
    /// next call, proving command handoff does not require a new executor.
    #[test]
    fn persistent_harness_applies_commands_from_later_calls() {
        let _guard = HARNESS_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let cal = [JointCalibration::default(); MAX_JOINTS];
        let mut sim = CopperSimRunner::new(6, &cal, [0.0; MAX_JOINTS]);
        sim.start_recording();

        let teach = HostToMcu {
            seq: 1,
            mode: ControlMode::Teach,
            ..Default::default()
        };
        let first = sim.step_harness(2, &[teach]);
        assert_eq!(first.active_mode, ControlMode::Teach);

        let drive = HostToMcu {
            seq: 2,
            mode: ControlMode::Drive,
            torque_mask: 0b11_1111,
            ..Default::default()
        };
        let second = sim.step_harness(2, &[drive]);
        assert_eq!(second.active_mode, ControlMode::Drive);

        let log = sim.stop_recording().expect("recorded log");
        assert_eq!(log.ticks[0].rx_cmd.expect("cmd").mode, ControlMode::Teach);
        assert_eq!(log.ticks[2].rx_cmd.expect("cmd").mode, ControlMode::Drive);
        assert!(verify_deterministic_replay(&log).exact_match);
    }
}
