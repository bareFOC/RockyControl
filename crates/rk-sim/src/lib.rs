//! Layer 3 — Software Servo Plant & Deterministic Copper Simulation Runner (`rk-sim`).
//!
//! Runs identically on Desktop and Browser (`wasm32-unknown-unknown`) with zero `std::thread`
//! or `std::time::Instant` assumptions. Records `.copper` deterministic replay logs and
//! verifies exact output parity under replay.

use rk_control::{JointCalibration, RawServoCommand, RawServoFeedback};
use rk_cu_nodes::{CopperListTick, DirectServoTask, HostLinkBridge, ServoBusDriver};
use rk_proto::{
    HostToMcu, MAX_COBS_FRAME_BYTES, MAX_JOINTS, MAX_MODEL_COBS_FRAME_BYTES, MODEL_CHUNK_BYTES,
    McuToHost, ModelChunk, ModelFormat, ModelInfo, ModelRequest, ModelResponse, decode_mcu_cobs,
    decode_model_req_cobs, decode_model_resp_cobs, encode_cobs, encode_model_req_cobs,
    encode_model_resp_cobs,
};
use serde::{Deserialize, Serialize};

const DEFAULT_MCU_FLASH_URDF: &[u8] =
    include_bytes!("../../../assets/sample_humanoid/rocky_mk1.urdf");

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

    pub fn step_plant(&mut self, dt_sec: f32) -> RawServoFeedback {
        let dt = dt_sec.clamp(0.001_f32, 0.05_f32);
        self.wave_phase += dt * 2.4_f32;

        let mut total_current_ma: u32 = 180;

        for i in 0..self.joint_count {
            let torque_on = ((self.last_cmd.torque_mask >> i) & 1) == 1;
            let stalled = self.faults.stalled_joints[i];

            if torque_on {
                let target = self.last_cmd.raw_target_rad[i];
                let pos = self.raw_angle_rad[i];
                let err = target - pos;

                if stalled {
                    self.velocity_rad_s[i] = 0.0;
                    let spike = (2350.0_f32 + err.abs() * 900.0_f32).min(3400.0_f32) as u16;
                    self.current_ma[i] = spike;
                } else {
                    let rate = 2.6_f32 + self.faults.servo_lag_factor * 42.0_f32;
                    let alpha = 1.0_f32 - (-rate * dt).exp();
                    let next_pos = pos + err * alpha;
                    let vel = (next_pos - pos) / dt;
                    self.raw_angle_rad[i] = next_pos;
                    self.velocity_rad_s[i] = vel;
                    self.virtual_hand_rad[i] = next_pos;

                    let load = (95.0_f32 + err.abs() * 1450.0_f32 + vel.abs() * 160.0_f32)
                        .min(2800.0_f32) as u16;
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

            let walk_sway_pitch = if self.last_balance.walk_active {
                (self.last_balance.gait_phase_rad * 2.0_f32).cos() * 0.45_f32
            } else {
                0.0_f32
            };
            let walk_sway_roll = if self.last_balance.walk_active {
                self.last_balance.gait_phase_rad.sin() * 0.65_f32
            } else {
                0.0_f32
            };

            // UVC catch-step shifts Center of Pressure `(act_foot_dx, act_foot_dy)` under leaning torso
            let h_com = self.last_balance.hip_height_m.clamp(0.42_f32, 0.72_f32);
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
                + walk_sway_pitch;
            let alpha_roll = k_grav * self.torso_roll_rad.sin()
                - passive_foot(self.torso_roll_rad)
                + cop_restore_roll
                + k_ankle_roll * self.act_roll_corr
                - damping * self.roll_rate_rad_s
                + walk_sway_roll;

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
        }
    }
}

impl ServoBusDriver for SimulatedServoPlant {
    fn read_feedback(&mut self, dt_sec: f32) -> RawServoFeedback {
        self.step_plant(dt_sec)
    }

    fn write_command(&mut self, cmd: &RawServoCommand) {
        self.last_cmd = *cmd;
    }
}

/// Deterministic `.copper` replay log capturing every wire tick for offline verification.
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct CopperReplayLog {
    pub version: u32,
    pub joint_count: usize,
    pub initial_raw_pose: [f32; MAX_JOINTS],
    pub initial_calibration: Vec<JointCalibration>,
    pub ticks: Vec<CopperListTick>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ReplayVerification {
    pub total_ticks: usize,
    pub matched_ticks: usize,
    pub exact_match: bool,
}

/// End-to-end simulation runner wiring COBS framing -> `DirectServoTask` -> `SimulatedServoPlant`
/// plus an onboard simulated MCU flash model store matching `firmware/rk-fw-core::model_store`.
pub struct CopperSimRunner {
    pub task: DirectServoTask,
    pub host_bridge: HostLinkBridge,
    pub plant: SimulatedServoPlant,
    uptime_ms_f: f64,
    host_decode_errors: u64,
    recording: bool,
    recorded_log: Option<CopperReplayLog>,
    max_recorded_ticks: usize,
    mcu_model_info: ModelInfo,
    mcu_model_blob: Vec<u8>,
}

impl CopperSimRunner {
    pub fn new(
        joint_count: usize,
        initial_cal: &[JointCalibration],
        initial_raw_pose: [f32; MAX_JOINTS],
    ) -> Self {
        let mcu_model_blob = DEFAULT_MCU_FLASH_URDF.to_vec();
        let mcu_model_info =
            ModelInfo::from_static("rocky_mk1", ModelFormat::UrdfXml, &mcu_model_blob, 21);
        Self {
            task: DirectServoTask::new(joint_count, initial_cal),
            host_bridge: HostLinkBridge::default(),
            plant: SimulatedServoPlant::new(joint_count, initial_raw_pose),
            uptime_ms_f: 0.0,
            host_decode_errors: 0,
            recording: false,
            recorded_log: None,
            max_recorded_ticks: 2400,
            mcu_model_info,
            mcu_model_blob,
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

    /// Executes a full COBS-framed `ModelRequest` -> `ModelResponse` round-trip against the simulated MCU flash store.
    pub fn exchange_model_request(&mut self, req: ModelRequest) -> Option<ModelResponse> {
        let mut req_wire = [0u8; MAX_MODEL_COBS_FRAME_BYTES];
        let req_len = encode_model_req_cobs(&req, &mut req_wire).ok()?.len();
        let mcu_req = decode_model_req_cobs(&mut req_wire[..req_len]).ok()?;

        let mcu_resp = if self.mcu_model_blob.is_empty() {
            ModelResponse::NotAvailable
        } else {
            match mcu_req {
                ModelRequest::GetInfo => ModelResponse::Info(self.mcu_model_info),
                ModelRequest::ReadChunk { offset, max_len } => {
                    let start = offset as usize;
                    if start >= self.mcu_model_blob.len() {
                        ModelResponse::Chunk(ModelChunk {
                            offset,
                            total_bytes: self.mcu_model_blob.len() as u32,
                            crc32: self.mcu_model_info.crc32,
                            format: self.mcu_model_info.format,
                            len: 0,
                            data: [0u8; MODEL_CHUNK_BYTES],
                        })
                    } else {
                        let req_len = (max_len as usize).clamp(1, MODEL_CHUNK_BYTES);
                        let copy_len = req_len.min(self.mcu_model_blob.len() - start);
                        let mut data = [0u8; MODEL_CHUNK_BYTES];
                        data[..copy_len]
                            .copy_from_slice(&self.mcu_model_blob[start..start + copy_len]);
                        ModelResponse::Chunk(ModelChunk {
                            offset,
                            total_bytes: self.mcu_model_blob.len() as u32,
                            crc32: self.mcu_model_info.crc32,
                            format: self.mcu_model_info.format,
                            len: copy_len as u16,
                            data,
                        })
                    }
                }
            }
        };

        let mut resp_wire = [0u8; MAX_MODEL_COBS_FRAME_BYTES];
        let resp_len = encode_model_resp_cobs(&mcu_resp, &mut resp_wire).ok()?.len();
        decode_model_resp_cobs(&mut resp_wire[..resp_len]).ok()
    }

    pub fn link_errors(&self) -> u64 {
        self.host_bridge.rx_decode_errors + self.host_decode_errors
    }

    pub fn is_recording(&self) -> bool {
        self.recording
    }

    pub fn start_recording(&mut self) {
        self.recording = true;
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

    /// Executes one full Copper DAG cycle including real COBS encode/decode in both directions.
    pub fn tick(&mut self, dt_sec: f32, host_pkt: &HostToMcu) -> McuToHost {
        let dt = dt_sec.clamp(0.002_f32, 0.05_f32);
        self.uptime_ms_f += (dt as f64) * 1000.0;
        let now_ms = self.uptime_ms_f.round() as u32;

        // 1. Read simulated servo feedback
        let feedback = self.plant.read_feedback(dt);

        // 2. Encode HostToMcu over COBS and decode on MCU bridge (unless heartbeat dropped)
        let rx_cmd = if self.plant.faults().drop_heartbeat {
            None
        } else {
            let mut wire = [0u8; MAX_COBS_FRAME_BYTES];
            match encode_cobs(host_pkt, &mut wire) {
                Ok(encoded) => {
                    let len = encoded.len();
                    self.host_bridge.decode_rx(&mut wire[..len])
                }
                Err(_) => {
                    self.host_decode_errors += 1;
                    None
                }
            }
        };

        // 3. Execute real #![no_std] DirectServoTask
        let (step_out, tick_record) = self.task.process(now_ms, dt, rx_cmd.as_ref(), &feedback);

        // 4. Write servo command & MCU balance telemetry to simulated servo/LIPM plant
        self.plant.write_command(&step_out.servo_cmd);
        self.plant.set_balance_telemetry(step_out.telemetry.balance);

        if self.recording {
            if let Some(log) = self.recorded_log.as_mut() {
                if log.ticks.len() < self.max_recorded_ticks {
                    log.ticks.push(tick_record);
                }
            }
        }

        // 5. Encode McuToHost over COBS and decode on host side
        let mut mcu_wire = [0u8; MAX_COBS_FRAME_BYTES];
        match self
            .host_bridge
            .encode_tx(&step_out.telemetry, &mut mcu_wire)
        {
            Ok(encoded) => {
                let len = encoded.len();
                match decode_mcu_cobs(&mut mcu_wire[..len]) {
                    Ok(decoded) => decoded,
                    Err(_) => {
                        self.host_decode_errors += 1;
                        step_out.telemetry
                    }
                }
            }
            Err(_) => {
                self.host_decode_errors += 1;
                step_out.telemetry
            }
        }
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

    /// Injects a malformed COBS byte stream to exercise silent drop + error counter increment.
    pub fn inject_corrupted_frame(&mut self) {
        let mut garbage = [0x05_u8, 0xFF, 0x00, 0x12, 0x00];
        let _ = self.host_bridge.decode_rx(&mut garbage);
    }
}

/// Replays a recorded `.copper` log through a fresh `DirectServoTask` and asserts
/// bit-identical `McuToHost` outputs across every tick.
pub fn verify_deterministic_replay(log: &CopperReplayLog) -> ReplayVerification {
    let mut replay_task = DirectServoTask::new(log.joint_count, &log.initial_calibration);
    let mut matched = 0usize;

    for tick in &log.ticks {
        let (out, _) =
            replay_task.process(tick.now_ms, tick.dt_sec, tick.rx_cmd.as_ref(), &tick.rx_state);
        if out.telemetry == tick.tx_telem && out.servo_cmd == tick.tx_cmd {
            matched += 1;
        }
    }

    ReplayVerification {
        total_ticks: log.ticks.len(),
        matched_ticks: matched,
        exact_match: !log.ticks.is_empty() && matched == log.ticks.len(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rk_proto::{ControlMode, FaultFlags, WalkCmd};

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
        let mut plant = SimulatedServoPlant::new(21, [0.0; MAX_JOINTS]);
        // Knock robot prone with torque off so it rests fallen (> 0.60 rad).
        plant.apply_push_impulse(5.5, 0.0);
        let mut fallen = plant.step_plant(0.016);
        for _ in 0..80 {
            fallen = plant.step_plant(0.016);
        }
        assert!(
            fallen.imu.pitch_rad > 0.70,
            "expected fallen prone pitch, got {}",
            fallen.imu.pitch_rad
        );

        // Energize legs and sweep recovery_progress 0.0 -> 1.0.
        plant.write_command(&RawServoCommand {
            torque_mask: (1u32 << 21) - 1,
            raw_target_rad: [0.0; MAX_JOINTS],
        });
        let mut last = fallen;
        for step_idx in 0..=150 {
            let progress = (step_idx as f32 / 150.0).min(1.0);
            plant.set_recovery_progress(Some(progress));
            last = plant.step_plant(0.016);
        }
        plant.set_recovery_progress(None);
        assert!(
            last.imu.pitch_rad.abs() < 0.08 && last.imu.roll_rad.abs() < 0.08,
            "recovery progress must lift torso upright (pitch={}, roll={})",
            last.imu.pitch_rad,
            last.imu.roll_rad
        );
    }
}
