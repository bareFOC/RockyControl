//! Layer 3 — Software Servo Plant & Deterministic Copper Simulation Runner (`humanoid-sim`).
//!
//! Runs identically on Desktop and Browser (`wasm32-unknown-unknown`) with zero `std::thread`
//! or `std::time::Instant` assumptions. Records `.copper` deterministic replay logs and
//! verifies exact output parity under replay.

use humanoid_control::{JointCalibration, RawServoCommand, RawServoFeedback};
use humanoid_cu_nodes::{CopperListTick, DirectServoTask, HostLinkBridge, ServoBusDriver};
use humanoid_proto::{
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
                    let rate = 1.6_f32 + self.faults.servo_lag_factor * 36.0_f32;
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
/// plus an onboard simulated MCU flash model store matching `firmware/humanoid-fw-core::model_store`.
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
            ModelInfo::from_static("rocky_mk1", ModelFormat::UrdfXml, &mcu_model_blob, 17);
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

        // 4. Write servo command to simulated servo bus
        self.plant.write_command(&step_out.servo_cmd);

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
    use humanoid_proto::{ControlMode, FaultFlags};

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
}
