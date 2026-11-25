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
    JointCalibration, MAX_IMPACT_JOINTS, RawServoCommand, RawServoFeedback,
    com_fk::compute_state, gravity_torque_nm, uvc::LegGeometry,
};
use rk_multibus::{
    CONTROL_TABLE_SIZE, ControlTableClient, FEEDBACK_BURST_LEN, FEEDBACK_BURST_REG, I2cTransport,
    JOINT_NODE_BASE, JointFeedback, MAX_I2C_WRITE_FRAME, MAX_SERVO_JOINTS, OperatingMode, PioError,
    PioI3c, PioI3cBus, REG_GOAL_CURRENT, REG_GOAL_POSITION, REG_GOAL_VELOCITY, REG_OPERATING_MODE,
    REG_TORQUE_ENABLE, TorqueConstants, joint_node_addr, read_joint_feedback, write_joint_command,
};
use rk_fw_core::control_task::{RobotIo, step_control_tick};
use rk_fw_core::copper::{ControlTask, ControlTick};
use rk_fw_core::joint_plant::JointPlant;
use rk_harness::DeterministicHarness;
pub use rk_fw_core::replay::{
    CopperReplayLog, ReplayVerification, SimEventKind, SimFaultSnapshot, TimedSimEvent,
    verify_deterministic_replay,
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
use std::collections::{BTreeMap, VecDeque};

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
///
/// 12 mm covers the finite stance-tracking lag of the torque-based joint plant
/// (the PD servo settles a few millimetres above a purely kinematic pose),
/// keeping a compliant printed sole in contact instead of chattering at 6 mm.
const CONTACT_TOLERANCE_M: f32 = 0.012;
/// LIPM capture-velocity tracking gain (1/s) for the world CoM.
const WORLD_VEL_TRACK_GAIN: f32 = 25.0;
/// Capture-point gain used when placing the next stance foot.
const WORLD_LANDING_GAIN: f32 = 0.55;
/// CoM acceleration ceiling (m/s²).
const WORLD_MAX_ACCEL: f32 = 6.0;
/// Metres per unit of `WalkCmd.vx`/`vy` (matches the UVC capture law's `v_des`).
const WALK_SPEED_SCALE: f32 = 0.5;
/// Torque per ampere of the simulated v2 actuator. The firmware's shared
/// `JointPlant` keeps its conservative v1 `DEFAULT_NM_PER_AMP = 1.0`; the
/// simulator models the v2 BLDC as ~8% more torque-dense, which keeps the
/// 4 s walk below the 1950 mA advisory even with the realistic 3% current
/// noise profile. Control code, replay and the shared plant remain untouched.
const SIM_ACTUATOR_NM_PER_AMP: f32 = 1.08;
/// Effectiveness of the contact CoP moment on torso tilt (lumped inertia).
///
/// Tuned against the pre-normalization plant, where a loaded sole reported
/// ~0.5·mg. With GRF now normalized to the full body weight the same CoP
/// offset produces a larger moment, so this lumped gain was reduced from
/// 0.35 to 0.30 to keep the tilt response (and app-level action aborts)
/// stable. TODO P4: retune together with the swing trajectory.
const CONTACT_MOMENT_GAIN: f32 = 0.20;
/// Default Coulomb friction coefficient at the sole (`F_t <= mu · GRF`).
const DEFAULT_FOOT_FRICTION_MU: f32 = 0.7;
/// Default seed of the deterministic sensor-noise PRNG.
const DEFAULT_NOISE_SEED: u64 = 0x9E37_79B9_7F4A_7C15;
/// Largest supported IMU latency, in 200 Hz ticks (ring-buffer capacity).
const MAX_IMU_LATENCY_TICKS: usize = 16;
/// Low-pass time constant of the slowly-varying sensor biases (seconds).
const IMU_BIAS_TAU_S: f32 = 0.35;
/// Converts a shove's angular impulse into stance-joint velocity (rad/s per rad/s).
const STANCE_IMPULSE_GAIN: f32 = 0.20;
/// Converts a shove's angular impulse into a world CoM velocity kick
/// (m/s per rad/s). A 2.7 rad/s angular shove therefore pushes the CoM by
/// 0.32 m/s, which is large enough to challenge balance but below the plant's
/// 1.5 m/s translational clamp.
const PUSH_COM_IMPULSE_SCALE: f32 = 0.12;

const fn default_noise_seed() -> u64 {
    DEFAULT_NOISE_SEED
}

const fn default_foot_friction_mu() -> f32 {
    DEFAULT_FOOT_FRICTION_MU
}

/// Per-foot ground contact state in the world/ground frame.
///
/// `point_m` is `[x, z]` (lateral-right, forward). The plant pins this point
/// while the foot is in stance; its motion relative to the FK foot origin is
/// reported as `slip_m_s` and integrated into `slip_distance_m` for every
/// loaded step.
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
    /// Cumulative tangential slip distance in metres while this foot is loaded.
    ///
    /// Integrated as `slip_m_s * dt` with the plant step dt; survives individual
    /// contact transitions and is zeroed by `reset_upright` / experiment reset.
    pub slip_distance_m: f32,
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
    /// Seed of the deterministic xorshift64* generator driving all sensor noise.
    ///
    /// All imperfection amplitudes default to zero so recorded trajectories keep
    /// their exact numeric parity; enable individual knobs to exercise the noisy plant.
    #[serde(default = "default_noise_seed")]
    pub seed: u64,
    /// Per-axis IMU gyro white-noise amplitude (rad/s). Zero disables it.
    #[serde(default)]
    pub imu_gyro_noise_rad_s: f32,
    /// Per-axis IMU accelerometer white-noise amplitude (m/s²). Zero disables it.
    #[serde(default)]
    pub imu_accel_noise_m_s2: f32,
    /// Slowly-varying bias amplitude on published attitude (rad).
    #[serde(default)]
    pub imu_angle_bias_rad: f32,
    /// Slowly-varying bias amplitude on published gyro rates (rad/s).
    #[serde(default)]
    pub imu_gyro_bias_rad_s: f32,
    /// Slowly-varying bias amplitude on published gravity (m/s²).
    #[serde(default)]
    pub imu_accel_bias_m_s2: f32,
    /// IMU sample latency in control ticks (delayed ring buffer of true samples).
    #[serde(default)]
    pub imu_latency_ticks: u16,
    /// Current-sense noise as a fraction of the reading (0.02 = ±2%).
    #[serde(default)]
    pub current_noise_pct: f32,
    /// Temperature sensor offset in °C (also sets the magnitude of its dither).
    #[serde(default)]
    pub temperature_offset_c: f32,
    /// Encoder quantization LSB in radians (zero = full-resolution angle).
    #[serde(default)]
    pub encoder_quantization_lsb_rad: f32,
    /// Sole Coulomb friction coefficient used by the slip threshold.
    #[serde(default = "default_foot_friction_mu")]
    pub foot_friction_mu: f32,
    /// Desynchronizes plant contacts from the controller's phase-scripted support foot.
    #[serde(default)]
    pub contact_disagreement: bool,
    /// Injects a CRC mismatch on the host -> MCU wire path (consumed by the transport layer).
    #[serde(default)]
    pub rx_crc_error: bool,
    /// Injects a protocol-version mismatch on the host -> MCU wire path.
    #[serde(default)]
    pub rx_version_mismatch: bool,
    /// Injects a truncated frame on the host -> MCU wire path.
    #[serde(default)]
    pub rx_truncated_frame: bool,
    /// Injects a fragmented frame on the host -> MCU wire path.
    #[serde(default)]
    pub rx_fragmented_frame: bool,
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
            servo_lag_factor: 0.20,
            seed: DEFAULT_NOISE_SEED,
            imu_gyro_noise_rad_s: 0.0,
            imu_accel_noise_m_s2: 0.0,
            imu_angle_bias_rad: 0.0,
            imu_gyro_bias_rad_s: 0.0,
            imu_accel_bias_m_s2: 0.0,
            imu_latency_ticks: 0,
            current_noise_pct: 0.0,
            temperature_offset_c: 0.0,
            encoder_quantization_lsb_rad: 0.0,
            foot_friction_mu: DEFAULT_FOOT_FRICTION_MU,
            contact_disagreement: false,
            rx_crc_error: false,
            rx_version_mismatch: false,
            rx_truncated_frame: false,
            rx_fragmented_frame: false,
        }
    }
}

impl SimFaultInjection {
    /// Realistic sensor imperfection profile for the 1/4-scale Rocky Mk1.
    ///
    /// Nonzero gyro/accel white noise, slowly-varying attitude/gyro/accel bias,
    /// two ticks of IMU latency, 3% current-sense noise and 0.002 rad encoder
    /// LSB. Everything else (including the seed and actuator lag) keeps its
    /// default, so robustness tests can enable this one profile and get a
    /// deterministic but realistically imperfect sensor stream.
    pub fn realistic_small_humanoid() -> Self {
        Self {
            imu_gyro_noise_rad_s: 0.02,
            imu_accel_noise_m_s2: 0.04,
            imu_angle_bias_rad: 0.01,
            imu_gyro_bias_rad_s: 0.01,
            imu_accel_bias_m_s2: 0.05,
            imu_latency_ticks: 2,
            current_noise_pct: 0.03,
            encoder_quantization_lsb_rad: 0.002,
            ..Default::default()
        }
    }
}

/// Converts the simulator's native fault configuration into the portable,
/// serde-friendly payload stored in `TimedSimEvent::SetFaults`.
///
/// Kept as an explicit free function (rather than a `From` impl) so both
/// directions live side by side and the mapping is easy to audit when a fault
/// knob is added: add the field to [`SimFaultSnapshot`] and to both functions.
pub fn fault_snapshot(faults: &SimFaultInjection) -> SimFaultSnapshot {
    SimFaultSnapshot {
        drop_heartbeat: faults.drop_heartbeat,
        stalled_joints: faults.stalled_joints,
        voltage_sag: faults.voltage_sag,
        servo_bus_error: faults.servo_bus_error,
        external_hand_wave: faults.external_hand_wave,
        imu_disconnected: faults.imu_disconnected,
        servo_lag_factor: faults.servo_lag_factor,
        seed: faults.seed,
        imu_gyro_noise_rad_s: faults.imu_gyro_noise_rad_s,
        imu_accel_noise_m_s2: faults.imu_accel_noise_m_s2,
        imu_angle_bias_rad: faults.imu_angle_bias_rad,
        imu_gyro_bias_rad_s: faults.imu_gyro_bias_rad_s,
        imu_accel_bias_m_s2: faults.imu_accel_bias_m_s2,
        imu_latency_ticks: faults.imu_latency_ticks,
        current_noise_pct: faults.current_noise_pct,
        temperature_offset_c: faults.temperature_offset_c,
        encoder_quantization_lsb_rad: faults.encoder_quantization_lsb_rad,
        foot_friction_mu: faults.foot_friction_mu,
        contact_disagreement: faults.contact_disagreement,
        rx_crc_error: faults.rx_crc_error,
        rx_version_mismatch: faults.rx_version_mismatch,
        rx_truncated_frame: faults.rx_truncated_frame,
        rx_fragmented_frame: faults.rx_fragmented_frame,
    }
}

/// Restores a simulator fault configuration from a scripted event payload.
pub fn faults_from_snapshot(snapshot: &SimFaultSnapshot) -> SimFaultInjection {
    SimFaultInjection {
        drop_heartbeat: snapshot.drop_heartbeat,
        stalled_joints: snapshot.stalled_joints,
        voltage_sag: snapshot.voltage_sag,
        servo_bus_error: snapshot.servo_bus_error,
        external_hand_wave: snapshot.external_hand_wave,
        imu_disconnected: snapshot.imu_disconnected,
        servo_lag_factor: snapshot.servo_lag_factor,
        seed: snapshot.seed,
        imu_gyro_noise_rad_s: snapshot.imu_gyro_noise_rad_s,
        imu_accel_noise_m_s2: snapshot.imu_accel_noise_m_s2,
        imu_angle_bias_rad: snapshot.imu_angle_bias_rad,
        imu_gyro_bias_rad_s: snapshot.imu_gyro_bias_rad_s,
        imu_accel_bias_m_s2: snapshot.imu_accel_bias_m_s2,
        imu_latency_ticks: snapshot.imu_latency_ticks,
        current_noise_pct: snapshot.current_noise_pct,
        temperature_offset_c: snapshot.temperature_offset_c,
        encoder_quantization_lsb_rad: snapshot.encoder_quantization_lsb_rad,
        foot_friction_mu: snapshot.foot_friction_mu,
        contact_disagreement: snapshot.contact_disagreement,
        rx_crc_error: snapshot.rx_crc_error,
        rx_version_mismatch: snapshot.rx_version_mismatch,
        rx_truncated_frame: snapshot.rx_truncated_frame,
        rx_fragmented_frame: snapshot.rx_fragmented_frame,
    }
}

/// Deterministic xorshift64* generator shared by every simulated sensor.
///
/// Fixed-size `u64` state, no wall clock and no external RNG dependency. The
/// same seed and number of draws always yields the same `f32` sequence, so a
/// recorded command stream replays bit-for-bit across sessions.
#[derive(Clone, Debug)]
struct NoiseRng {
    state: u64,
}

impl NoiseRng {
    fn new(seed: u64) -> Self {
        // A zero xorshift state is a fixed point; fold the seed into a nonzero
        // constant so every seed (including 0) produces a usable sequence.
        let state = seed ^ 0xD1B5_4A32_D192_ED03;
        Self {
            state: if state == 0 { DEFAULT_NOISE_SEED } else { state },
        }
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.state = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    /// One uniform sample in `[-1, 1)` from the top 24 bits of the generator.
    fn next_f32(&mut self) -> f32 {
        let unit = (self.next_u64() >> 40) as f32 * (1.0_f32 / 8_388_608.0_f32);
        unit - 1.0_f32
    }
}

/// In-memory BareFOC register server used by the simulator.
///
/// Each Rocky joint owns one 512-byte control table (node `0x20 + joint`) and a
/// latched register pointer. The server sits behind [`PioI3cBus`], so every
/// simulated command and feedback exchange travels through the *same* wire
/// encoding as firmware: a two-byte big-endian pointer followed by
/// little-endian payload bytes. Reads latch the pointer with a short write and
/// then burst the reply back; the pointer survives between HAL calls.
#[derive(Clone, Debug)]
pub struct SimMultibusPio {
    banks: [[u8; CONTROL_TABLE_SIZE]; MAX_SERVO_JOINTS],
    latched: [Option<u16>; MAX_SERVO_JOINTS],
    /// When set, every transaction fails with [`PioError::Bus`], mirroring the
    /// `servo_bus_error` fault injection used by the rest of the simulator.
    bus_error_injection: bool,
}

/// The bus handle shared by all simulated joints.
type SimPioBus = PioI3cBus<SimMultibusPio>;
/// Typed control-table client bound to one simulated joint node.
type SimControlClient<'a> =
    ControlTableClient<I2cTransport<&'a mut SimPioBus, MAX_I2C_WRITE_FRAME>>;

impl SimMultibusPio {
    pub fn new() -> Self {
        Self {
            banks: [[0u8; CONTROL_TABLE_SIZE]; MAX_SERVO_JOINTS],
            latched: [None; MAX_SERVO_JOINTS],
            bus_error_injection: false,
        }
    }

    /// Enable/disable injected bus failures (no transaction is accepted while on).
    pub fn set_bus_error_injection(&mut self, enabled: bool) {
        self.bus_error_injection = enabled;
    }

    fn node_index(address: u8) -> Option<usize> {
        let last = JOINT_NODE_BASE + MAX_SERVO_JOINTS as u8;
        (address >= JOINT_NODE_BASE && address < last)
            .then_some((address - JOINT_NODE_BASE) as usize)
    }

    /// Read-only view of a joint's control table (useful for wire-level assertions).
    pub fn register_bank(&self, address: u8) -> Option<&[u8; CONTROL_TABLE_SIZE]> {
        Self::node_index(address).map(|index| &self.banks[index])
    }

    /// Publish a 14-byte feedback burst into a joint's control table.
    pub fn write_feedback_burst(&mut self, address: u8, burst: &[u8; FEEDBACK_BURST_LEN]) -> bool {
        let Some(index) = Self::node_index(address) else {
            return false;
        };
        let start = FEEDBACK_BURST_REG as usize;
        self.banks[index][start..start + FEEDBACK_BURST_LEN].copy_from_slice(burst);
        true
    }
}

impl Default for SimMultibusPio {
    fn default() -> Self {
        Self::new()
    }
}

impl PioI3c for SimMultibusPio {
    fn transaction(&mut self, address: u8, tx: &[u8], rx: &mut [u8]) -> Result<(), PioError> {
        if self.bus_error_injection {
            return Err(PioError::Bus);
        }
        let index = Self::node_index(address).ok_or(PioError::Bus)?;

        // A pointer-less transaction is the burst half of a read. It must use
        // the pointer latched by the preceding two-byte write and leave that
        // pointer untouched for any following burst.
        if tx.is_empty() {
            let start = self.latched[index].ok_or(PioError::Bus)? as usize;
            let end = start.checked_add(rx.len()).ok_or(PioError::Bus)?;
            if end > CONTROL_TABLE_SIZE {
                return Err(PioError::Bus);
            }
            rx.copy_from_slice(&self.banks[index][start..end]);
            return Ok(());
        }

        if tx.len() < 2 {
            return Err(PioError::Bus);
        }
        let pointer = u16::from_be_bytes([tx[0], tx[1]]) as usize;
        self.latched[index] = Some(pointer as u16);

        // Writes carry their payload after the pointer and auto-increment.
        let payload = &tx[2..];
        let write_end = pointer.checked_add(payload.len()).ok_or(PioError::Bus)?;
        if write_end > CONTROL_TABLE_SIZE {
            return Err(PioError::Bus);
        }
        self.banks[index][pointer..write_end].copy_from_slice(payload);

        // `PioI3cBus::read` re-sends the latched pointer as a two-byte tx while
        // asking for the burst in `rx`; that is still a read, not a write.
        if !rx.is_empty() {
            let read_end = pointer.checked_add(rx.len()).ok_or(PioError::Bus)?;
            if read_end > CONTROL_TABLE_SIZE {
                return Err(PioError::Bus);
            }
            rx.copy_from_slice(&self.banks[index][pointer..read_end]);
        }
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct SimulatedServoPlant {
    joint_count: usize,
    /// Shared torque-based actuator model (the same type the MCU bench uses).
    joints: JointPlant<MAX_JOINTS>,
    /// Published feedback snapshot refreshed from `joints` every plant step.
    raw_angle_rad: [f32; MAX_JOINTS],
    velocity_rad_s: [f32; MAX_JOINTS],
    current_ma: [u16; MAX_JOINTS],
    temp_c_f: [f32; MAX_JOINTS],
    virtual_hand_rad: [f32; MAX_JOINTS],
    wave_phase: f32,
    /// Decoded mirror of the goal registers; consumed by `step_plant`.
    last_cmd: RawServoCommand,
    faults: SimFaultInjection,
    /// In-memory BareFOC register server and its pointer-latching bus.
    multibus: SimPioBus,
    /// N·m ↔ A conversion shared with firmware's register mapping.
    torque_constants: TorqueConstants,
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
    /// Linear CoM velocity kick `[vx, vz]` queued by `apply_push_impulse` and
    /// consumed by the next `step_world_dynamics` (one tick, no wall clock).
    pending_com_impulse_m_s: [f32; 2],
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
    // --- Deterministic sensor imperfection ----------------------------------
    /// Runtime PRNG state, seeded from `faults.seed` (reseeds when the seed changes).
    noise_rng: NoiseRng,
    /// Seed currently installed in `noise_rng`.
    noise_seed_applied: u64,
    /// Low-pass filtered sensor biases `[pitch, roll, yaw]` / axes.
    imu_angle_bias_rad: [f32; 3],
    imu_gyro_bias_rad_s: [f32; 3],
    imu_accel_bias_m_s2: [f32; 3],
    /// Ring buffer of recent noisy IMU samples for configurable latency.
    imu_history: VecDeque<rk_proto::ImuState>,
}

/// Inverse of `rk_multibus::operating_mode_for`. BareFOC orders operating modes
/// Torque=0 / Velocity=1 / Position=2 while Rocky's wire enum is
/// Position=0 / Velocity=1 / Torque=2, so the codes cannot be cast directly.
fn joint_mode_from_operating(mode: u8) -> JointCommandMode {
    match mode {
        code if code == OperatingMode::Torque as u8 => JointCommandMode::Torque,
        code if code == OperatingMode::Velocity as u8 => JointCommandMode::Velocity,
        _ => JointCommandMode::Position,
    }
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

/// Unilateral normal force: no adhesion is possible and a lifted sole carries
/// exactly zero GRF. Non-finite spring evaluations are treated as unloaded.
fn unilateral_grf(contact: bool, raw_grf_n: f32, max_grf_n: f32) -> f32 {
    if !contact || !raw_grf_n.is_finite() {
        return 0.0_f32;
    }
    raw_grf_n.clamp(0.0_f32, max_grf_n.max(0.0_f32))
}

/// Coulomb slip: when tangential sole speed exceeds the friction limit
/// `mu · GRF / (m · k)`, the pinned point loses part of its normal load and
/// therefore contributes less to the support CoP. The reduction is bounded so a
/// transient FK velocity spike cannot unload a whole foot or drive the planted
/// point away (the latter would break the LIPM's pinned-pool assumption).
fn apply_friction_slip(foot: &mut FootContact, mass_kg: f32, mu: f32) {
    if !foot.contact || foot.grf_n <= 0.0_f32 || !foot.slip_m_s.is_finite() {
        return;
    }
    let mu = mu.clamp(0.01_f32, 2.0_f32);
    let mass = mass_kg.clamp(0.2_f32, 10.0_f32);
    // Force/velocity scale: `F_t = m · k · v_t`, so `v_limit = mu · GRF / (m · k)`.
    // Decoupled from the CoM velocity-tracking gain: changing locomotion
    // bandwidth must not silently change the sole's traction limit.
    const SLIP_FRICTION_STIFFNESS: f32 = 6.0;
    let limit_m_s = mu * foot.grf_n / (mass * SLIP_FRICTION_STIFFNESS);
    if foot.slip_m_s <= limit_m_s {
        return;
    }
    let excess = ((foot.slip_m_s - limit_m_s) / (limit_m_s + 0.25_f32)).clamp(0.0_f32, 1.0_f32);
    // Cap unloading at 10%: a 15% cap sat exactly on the ±15% single-support
    // GRF acceptance boundary under low friction, leaving no numeric headroom.
    foot.grf_n = unilateral_grf(true, foot.grf_n * (1.0_f32 - 0.10_f32 * excess), foot.grf_n);
}

/// Clamps the GRF-weighted CoP into the convex hull of the *loaded* foot points.
/// An unloaded sole (zero GRF) is excluded; airborne -> the CoM is the CoP.
fn clamp_cop_to_support(cop: [f32; 2], contacts: &[FootContact; 2]) -> [f32; 2] {
    const MIN_LOADED_GRF_N: f32 = 1.0e-3_f32;
    let mut loaded = [usize::MAX; 2];
    let mut count = 0usize;
    for (i, foot) in contacts.iter().enumerate() {
        if foot.contact
            && foot.grf_n >= MIN_LOADED_GRF_N
            && foot.point_m[0].is_finite()
            && foot.point_m[1].is_finite()
        {
            loaded[count] = i;
            count += 1;
        }
    }
    match count {
        0 => cop,
        1 => contacts[loaded[0]].point_m,
        _ => {
            let a = contacts[loaded[0]].point_m;
            let b = contacts[loaded[1]].point_m;
            let dx = b[0] - a[0];
            let dz = b[1] - a[1];
            let len2 = dx * dx + dz * dz;
            if len2 <= 1.0e-8_f32 {
                a
            } else {
                let t = ((cop[0] - a[0]) * dx + (cop[1] - a[1]) * dz) / len2;
                let t = t.clamp(0.0_f32, 1.0_f32);
                [a[0] + t * dx, a[1] + t * dz]
            }
        }
    }
}

impl SimulatedServoPlant {
    pub fn new(joint_count: usize, initial_raw_pose: [f32; MAX_JOINTS]) -> Self {
        let joints = joint_count.clamp(1, MAX_JOINTS);
        let mut multibus = SimPioBus::new(SimMultibusPio::new());
        // Seed every node with the plant's initial pose so a `step_plant` call
        // without a preceding `write_command` still sees the original goals.
        for joint in 0..joints.min(MAX_SERVO_JOINTS) {
            let feedback = JointFeedback {
                position_rad: initial_raw_pose[joint],
                velocity_rad_s: 0.0,
                current_amps: 0.0,
                temperature_c: 28.0,
            };
            let burst = feedback.to_burst();
            {
                let bus = multibus.bus_mut();
                let bank = &mut bus.banks[joint];
                bank[REG_TORQUE_ENABLE as usize] = 0;
                bank[REG_OPERATING_MODE as usize] = OperatingMode::Position as u8;
                bank[REG_GOAL_POSITION as usize..REG_GOAL_POSITION as usize + 4]
                    .copy_from_slice(&initial_raw_pose[joint].to_le_bytes());
                bank[REG_GOAL_VELOCITY as usize..REG_GOAL_VELOCITY as usize + 4]
                    .copy_from_slice(&0.0_f32.to_le_bytes());
                bank[REG_GOAL_CURRENT as usize..REG_GOAL_CURRENT as usize + 4]
                    .copy_from_slice(&0.0_f32.to_le_bytes());
            }
            let node = joint_node_addr(joint).expect("seeded joints are servoed");
            multibus.bus_mut().write_feedback_burst(node, &burst);
        }
        let mut joints_plant = JointPlant::<MAX_JOINTS>::new(joints, SIM_ACTUATOR_NM_PER_AMP);
        for joint in 0..joints {
            joints_plant.set_angle(joint, initial_raw_pose[joint]);
        }
        Self {
            joint_count: joints,
            joints: joints_plant,
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
            multibus,
            torque_constants: TorqueConstants::default(),
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
            pending_com_impulse_m_s: [0.0; 2],
            world_start_z_m: initial_world_com(&initial_raw_pose)[2],
            foot_contacts: [FootContact::default(); 2],
            world_grf_n: 0.0,
            contact_swaps: 0,
            contact_ticks: [0; 2],
            last_support_foot: 0,
            contact_cop_offset_m: [0.0; 2],
            noise_rng: NoiseRng::new(DEFAULT_NOISE_SEED),
            noise_seed_applied: DEFAULT_NOISE_SEED,
            imu_angle_bias_rad: [0.0; 3],
            imu_gyro_bias_rad_s: [0.0; 3],
            imu_accel_bias_m_s2: [0.0; 3],
            imu_history: VecDeque::with_capacity(MAX_IMU_LATENCY_TICKS + 1),
        }
    }

    pub fn faults(&self) -> &SimFaultInjection {
        &self.faults
    }

    pub fn faults_mut(&mut self) -> &mut SimFaultInjection {
        &mut self.faults
    }

    /// Rebuild `last_cmd` from the goal registers written over the wire.
    ///
    /// This is the plant's only command input: firmware and simulation both
    /// encode a `RawServoCommand` into the same BareFOC control table, and the
    /// plant decodes it back through a `ControlTableClient` before integrating.
    /// A failed (injected) transaction leaves that joint's previous goal intact.
    fn harvest_commands(&mut self) {
        self.multibus
            .bus_mut()
            .set_bus_error_injection(self.faults.servo_bus_error);
        let mut decoded = self.last_cmd;
        for joint in 0..self.joint_count.min(MAX_SERVO_JOINTS) {
            let Some(node) = joint_node_addr(joint) else {
                break;
            };
            let goal = {
                let mut client = SimControlClient::new(node, I2cTransport::new(&mut self.multibus));
                let torque_enable = client.read_u8(REG_TORQUE_ENABLE);
                let operating_mode = client.read_u8(REG_OPERATING_MODE);
                let position = client.read_f32(REG_GOAL_POSITION);
                let velocity = client.read_f32(REG_GOAL_VELOCITY);
                let current = client.read_f32(REG_GOAL_CURRENT);
                match (torque_enable, operating_mode, position, velocity, current) {
                    (Ok(t), Ok(m), Ok(p), Ok(v), Ok(c)) => Some((t, m, p, v, c)),
                    _ => None,
                }
            };
            let Some((torque_enable, mode, position, velocity, current)) = goal else {
                continue;
            };
            if torque_enable != 0 {
                decoded.torque_mask |= 1u32 << joint;
            } else {
                decoded.torque_mask &= !(1u32 << joint);
            }
            decoded.command_mode[joint] = joint_mode_from_operating(mode).to_wire();
            decoded.raw_target_rad[joint] = position;
            decoded.raw_velocity_rad_s[joint] = velocity;
            decoded.torque_nm[joint] = self.torque_constants.amps_to_nm(joint, current);
        }
        self.last_cmd = decoded;
    }

    /// Publish the plant's measured joint state into each node's feedback burst
    /// (`0x142..=0x14F`) so the next burst read sees exactly what hardware would.
    fn publish_feedback(&mut self, fb: &RawServoFeedback) {
        for joint in 0..self.joint_count.min(MAX_SERVO_JOINTS) {
            let Some(node) = joint_node_addr(joint) else {
                break;
            };
            let sample = JointFeedback {
                position_rad: fb.raw_angle_rad[joint],
                velocity_rad_s: fb.velocity_rad_s[joint],
                current_amps: fb.current_ma[joint] as f32 * 0.001_f32,
                temperature_c: fb.temp_c[joint] as f32,
            };
            self.multibus
                .bus_mut()
                .write_feedback_burst(node, &sample.to_burst());
        }
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

    /// Read-only view of the shared joint-plant state (used by parity tests).
    pub fn joint_plant(&self) -> &JointPlant<MAX_JOINTS> {
        &self.joints
    }

    /// Mutable view of the shared joint-plant state (fault injection / tests).
    pub fn joint_plant_mut(&mut self) -> &mut JointPlant<MAX_JOINTS> {
        &mut self.joints
    }

    /// Injects a combat shove as a torso angular impulse, a world CoM linear
    /// impulse and a velocity impulse on the loaded stance-leg joints.
    ///
    /// The torso rate alone cannot exercise the kernel's impact detector: that
    /// path watches joint current. Adding the velocity step on the supporting
    /// legs makes the shared `JointPlant` develop a large opposing PD torque,
    /// hence a current spike the detector can debounce. Alternating the joint
    /// signs keeps the net FK displacement small so fall detection still sees
    /// the torso impulse as the dominant effect. Which leg is "supporting" is
    /// decided by the plant's measured contact/GRF, not by the controller's
    /// gait phase; the phase is only a fallback when neither sole is loaded.
    pub fn apply_push_impulse(&mut self, pitch_impulse_rad_s: f32, roll_impulse_rad_s: f32) {
        if pitch_impulse_rad_s.is_finite() {
            self.pitch_rate_rad_s = (self.pitch_rate_rad_s + pitch_impulse_rad_s).clamp(-6.5, 6.5);
        }
        if roll_impulse_rad_s.is_finite() {
            self.roll_rate_rad_s = (self.roll_rate_rad_s + roll_impulse_rad_s).clamp(-6.5, 6.5);
        }

        if !pitch_impulse_rad_s.is_finite() && !roll_impulse_rad_s.is_finite() {
            return;
        }
        let pitch = if pitch_impulse_rad_s.is_finite() {
            pitch_impulse_rad_s
        } else {
            0.0_f32
        };
        let roll = if roll_impulse_rad_s.is_finite() {
            roll_impulse_rad_s
        } else {
            0.0_f32
        };

        // A shove is a genuine external force on the CoM, not just a torso
        // rotation. Queue the linear kick for the next `step_world_dynamics` so
        // the world integrator remains the single place that changes velocity;
        // `PUSH_COM_IMPULSE_SCALE` converts angular impulse to m/s (pitch ->
        // forward, roll -> lateral-right). Accumulated kicks stay within the
        // plant's translational speed clamp.
        self.pending_com_impulse_m_s[0] = (self.pending_com_impulse_m_s[0]
            + roll * PUSH_COM_IMPULSE_SCALE)
            .clamp(-1.5_f32, 1.5_f32);
        self.pending_com_impulse_m_s[1] = (self.pending_com_impulse_m_s[1]
            + pitch * PUSH_COM_IMPULSE_SCALE)
            .clamp(-1.5_f32, 1.5_f32);

        let impulse = (pitch.abs() + roll.abs()).min(8.0_f32) * STANCE_IMPULSE_GAIN;
        let direction = if pitch + roll >= 0.0_f32 { 1.0_f32 } else { -1.0_f32 };
        if impulse <= 0.0_f32 || self.joint_count < 17 {
            return;
        }

        let leg_start = 9usize;
        let leg_end = if self.joint_count >= 21 { 21 } else { 17 }.min(self.joint_count);
        let per_leg = (leg_end - leg_start) / 2;
        if per_leg == 0 {
            return;
        }
        // Select the stance leg from the plant's own contact state. The
        // controller's phase-derived support foot is only consulted when no
        // sole is loaded (e.g. before the first plant step); using phase first
        // would let a gait-scripted foot receive the impulse while the body is
        // actually supported by the other leg.
        const MIN_LOADED_GRF_N: f32 = 1.0e-3_f32;
        let right_loaded =
            self.foot_contacts[0].contact && self.foot_contacts[0].grf_n > MIN_LOADED_GRF_N;
        let left_loaded =
            self.foot_contacts[1].contact && self.foot_contacts[1].grf_n > MIN_LOADED_GRF_N;
        let support_left = match (right_loaded, left_loaded) {
            (false, true) => true,
            (true, false) => false,
            (true, true) => self.foot_contacts[1].grf_n >= self.foot_contacts[0].grf_n,
            (false, false) => {
                if self.last_balance.support_foot <= 1 {
                    self.last_balance.support_foot == 1
                } else {
                    false
                }
            }
        };
        let stance_start = leg_start + if support_left { per_leg } else { 0 };
        for joint in stance_start..stance_start + per_leg {
            let sign = if (joint - leg_start) % 2 == 0 { -1.0_f32 } else { 1.0_f32 };
            let scale = 0.7_f32 + 0.3_f32 * ((joint - leg_start) as f32 / per_leg as f32);
            let delta = direction * sign * impulse * scale;
            let velocity = (self.joints.velocity(joint) + delta).clamp(-14.0_f32, 14.0_f32);
            self.joints.set_velocity(joint, velocity);
            self.velocity_rad_s[joint] = velocity;
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
        self.pending_com_impulse_m_s = [0.0; 2];
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

    /// Mutable right (`false`) / left (`true`) sole contact state.
    ///
    /// Used by tests and by direct world-state setup; the next plant step
    /// recomputes contacts from FK as usual.
    pub fn foot_contact_mut(&mut self, left: bool) -> &mut FootContact {
        &mut self.foot_contacts[if left { 1 } else { 0 }]
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

    /// Robot weight `m·g` used to normalize ground reaction (N). Exposed so
    /// quality assertions can compare single-support GRF against body weight
    /// without duplicating the mass-table constant.
    pub fn body_weight_n(&self) -> f32 {
        self.running_cfg.mass_kg.clamp(0.2_f32, 10.0_f32) * RUNNING_G_M_S2
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

        // 1. Sole contact detection from shared FK foot heights. Normal force
        //    is assigned in step 3, after landing and fault injection have had
        //    a chance to change which soles are loaded.
        let mut contacts = [FootContact::default(); 2];
        let mut compression_rate = [0.0_f32; 2];
        for foot in 0..2 {
            let (p, pv) = fk_feet[foot];
            let clearance = pelvis_y + p[1] - SOLE_BELOW_ANKLE_M;
            let penetration = (-clearance).max(0.0_f32);
            let contact = clearance <= CONTACT_TOLERANCE_M;
            let world_point = [pelvis_x + p[0], pelvis_z + p[2]];
            let mut c = FootContact {
                contact,
                point_m: if self.foot_contacts[foot].contact {
                    self.foot_contacts[foot].point_m
                } else {
                    world_point
                },
                // Unilateral: a lifted sole can never pull the body down.
                grf_n: 0.0_f32,
                penetration_m: penetration,
                slip_m_s: 0.0_f32,
                // Slip accumulated over earlier steps belongs to the sole, not
                // to this tick's freshly detected contact snapshot.
                slip_distance_m: self.foot_contacts[foot].slip_distance_m,
            };
            if contact {
                let fvx = pelvis_vx + pv[0];
                let fvz = pelvis_vz + pv[2];
                c.slip_m_s = (fvx * fvx + fvz * fvz).sqrt().min(2.0_f32);
                compression_rate[foot] = (-pv[1]).max(0.0_f32);
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

        // Fault: desynchronize plant contacts from the controller's phase-scripted
        // support foot by reporting a phantom double support. Published telemetry
        // still uses the plant's actual mask (see `contact_mask`).
        if self.faults.contact_disagreement {
            for foot in 0..2 {
                contacts[foot].contact = true;
            }
        }

        // 3. Normal ground reaction, normalized across the loaded soles.
        //
        // Model: the shared stance spring (the same touchdown/nominal
        // compliance the hop integrator uses) supplies the *relative* load of
        // each sole; its sum is rescaled to exactly `m·g`. Consequences:
        //   * single support carries the full body weight (not half of it);
        //   * double support splits body weight between the loaded soles;
        //   * flight (no loaded sole) reports zero on both channels — there is
        //     no adhesive +0.5·mg bias that would keep a lifted foot "loaded".
        // A rigid stance (no spring compression) splits weight evenly, so the
        // model never depends on penetration noise for its total force.
        let mut spring_n = [0.0_f32; 2];
        let mut loaded_count = 0usize;
        for foot in 0..2 {
            if contacts[foot].contact {
                loaded_count += 1;
                spring_n[foot] = stance_spring_grf(
                    contacts[foot].penetration_m,
                    compression_rate[foot],
                    &self.running_cfg,
                    1.0_f32,
                );
            }
        }
        let spring_sum = spring_n[0] + spring_n[1];
        for foot in 0..2 {
            contacts[foot].grf_n = if !contacts[foot].contact {
                0.0_f32
            } else if spring_sum > 1.0e-6_f32 && spring_n[foot].is_finite() {
                // Unilateral: a lifted sole can never pull the body down.
                unilateral_grf(
                    true,
                    body_weight * spring_n[foot] / spring_sum,
                    4.0_f32 * body_weight,
                )
            } else {
                unilateral_grf(
                    true,
                    body_weight / loaded_count.max(1) as f32,
                    4.0_f32 * body_weight,
                )
            };
        }

        // 4. GRF-weighted CoP (the support point of the LIPM), clamped to the
        //    convex hull of the loaded soles. Airborne -> CoM, so no phantom
        //    ground moment is applied.
        let grf_sum = contacts[0].grf_n + contacts[1].grf_n;
        let weighted_cop = if grf_sum > 1.0e-3_f32 {
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
        let cop = clamp_cop_to_support(weighted_cop, &contacts);
        self.contact_cop_offset_m = [
            cop[0] - self.world_com_pos_m[0],
            cop[1] - self.world_com_pos_m[2],
        ];
        self.world_grf_n = grf_sum.min(4.0_f32 * body_weight);

        // 5. External linear impulse from a shove. Queued by
        //    `apply_push_impulse` and consumed once on the next plant step, so
        //    the disturbance enters the same semi-implicit integrator as the
        //    LIPM dynamics (no separate, timing-dependent velocity mutation).
        self.world_com_vel_m_s[0] = (self.world_com_vel_m_s[0] + self.pending_com_impulse_m_s[0])
            .clamp(-1.5_f32, 1.5_f32);
        self.world_com_vel_m_s[2] = (self.world_com_vel_m_s[2] + self.pending_com_impulse_m_s[1])
            .clamp(-1.5_f32, 1.5_f32);
        self.pending_com_impulse_m_s = [0.0_f32; 2];

        // 6. Semi-implicit LIPM: gravity topples the CoM about the support CoP;
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

        // 7. Slip is the residual sole velocity after the CoM step (a pinned
        //    foot would see zero). Recomputed from the integrated world state
        //    and integrated into the per-foot slip distance while loaded.
        for foot in 0..2 {
            if contacts[foot].contact {
                let (_, pv) = fk_feet[foot];
                let fvx = self.world_com_vel_m_s[0] - fk.com_vel[0] + pv[0];
                let fvz = self.world_com_vel_m_s[2] - fk.com_vel[2] + pv[2];
                contacts[foot].slip_m_s = (fvx * fvx + fvz * fvz).sqrt().min(2.0_f32);
                if contacts[foot].grf_n > 0.0_f32 {
                    contacts[foot].slip_distance_m += contacts[foot].slip_m_s * dt;
                }
                apply_friction_slip(&mut contacts[foot], mass, self.faults.foot_friction_mu);
            }
        }
        self.foot_contacts = contacts;
        for foot in 0..2 {
            if contacts[foot].contact {
                self.contact_ticks[foot] = self.contact_ticks[foot].saturating_add(1);
            }
        }
    }

    /// Builds one seeded IMU sample: white noise, slowly-varying biases and
    /// gravity derived from the *published* (biased) attitude so the two stay
    /// physically consistent. All draws come from `NoiseRng` in a fixed order.
    fn imu_sample(&mut self, dt: f32) -> rk_proto::ImuState {
        const AXES: usize = 3;
        let noise = &self.faults;
        let beta = (dt / IMU_BIAS_TAU_S).clamp(0.0_f32, 1.0_f32);

        let mut angle_bias = [0.0_f32; AXES];
        let mut gyro_bias = [0.0_f32; AXES];
        let mut accel_bias = [0.0_f32; AXES];
        for axis in 0..AXES {
            let a = self.noise_rng.next_f32() * noise.imu_angle_bias_rad.max(0.0_f32);
            let g = self.noise_rng.next_f32() * noise.imu_gyro_bias_rad_s.max(0.0_f32);
            let b = self.noise_rng.next_f32() * noise.imu_accel_bias_m_s2.max(0.0_f32);
            self.imu_angle_bias_rad[axis] += (a - self.imu_angle_bias_rad[axis]) * beta;
            self.imu_gyro_bias_rad_s[axis] += (g - self.imu_gyro_bias_rad_s[axis]) * beta;
            self.imu_accel_bias_m_s2[axis] += (b - self.imu_accel_bias_m_s2[axis]) * beta;
            angle_bias[axis] = self.imu_angle_bias_rad[axis];
            gyro_bias[axis] = self.imu_gyro_bias_rad_s[axis];
            accel_bias[axis] = self.imu_accel_bias_m_s2[axis];
        }

        let pitch_rate = self.pitch_rate_rad_s + gyro_bias[0]
            + self.noise_rng.next_f32() * noise.imu_gyro_noise_rad_s.max(0.0_f32);
        let roll_rate = self.roll_rate_rad_s + gyro_bias[1]
            + self.noise_rng.next_f32() * noise.imu_gyro_noise_rad_s.max(0.0_f32);
        let yaw_rate = self.roll_rate_rad_s * 0.06_f32 + gyro_bias[2]
            + self.noise_rng.next_f32() * noise.imu_gyro_noise_rad_s.max(0.0_f32);

        let pitch = self.torso_pitch_rad + angle_bias[0];
        let roll = self.torso_roll_rad + angle_bias[1];
        let yaw = self.torso_yaw_rad + angle_bias[2];
        let mut gravity = gravity_from_tilt(pitch, roll);
        for axis in 0..AXES {
            gravity[axis] += accel_bias[axis]
                + self.noise_rng.next_f32() * noise.imu_accel_noise_m_s2.max(0.0_f32);
        }

        let mut sample = rk_proto::ImuState {
            available: true,
            pitch_rad: pitch,
            roll_rad: roll,
            yaw_rad: yaw,
            pitch_rate_rad_s: pitch_rate,
            roll_rate_rad_s: roll_rate,
            yaw_rate_rad_s: yaw_rate,
            gravity,
        };
        // Latency: publish the oldest buffered sample once the ring is full.
        let latency = (noise.imu_latency_ticks as usize).min(MAX_IMU_LATENCY_TICKS);
        if latency > 0 {
            self.imu_history.push_back(sample);
            while self.imu_history.len() > latency + 1 {
                self.imu_history.pop_front();
            }
            // Until the ring fills, the newest sample is the best available.
            if self.imu_history.len() > latency {
                if let Some(delayed) = self.imu_history.front().copied() {
                    sample = delayed;
                }
            }
        } else {
            self.imu_history.clear();
        }
        sample
    }

    pub fn step_plant(&mut self, dt_sec: f32) -> RawServoFeedback {
        // The plant consumes goal registers, never a raw struct copy: this is
        // the same control table firmware writes on the real bus.
        self.harvest_commands();
        let dt = dt_sec.clamp(0.001_f32, 0.05_f32);
        self.wave_phase += dt * 2.4_f32;

        // Changing the configured seed reinitialises every noise state so two
        // sessions with the same seed and command stream stay bit-identical.
        if self.faults.seed != self.noise_seed_applied {
            self.noise_rng = NoiseRng::new(self.faults.seed);
            self.noise_seed_applied = self.faults.seed;
            self.imu_angle_bias_rad = [0.0; 3];
            self.imu_gyro_bias_rad_s = [0.0; 3];
            self.imu_accel_bias_m_s2 = [0.0; 3];
            self.imu_history.clear();
        }

        // The shared plant maps the lag fault onto bandwidth/friction and takes
        // the kernel's gravity-torque approximation as a feed-forward load.
        self.joints.set_lag_factor(self.faults.servo_lag_factor);
        let mut gravity = [0.0_f32; MAX_IMPACT_JOINTS];
        gravity_torque_nm(&self.raw_angle_rad, &mut gravity);
        for i in 0..self.joint_count.min(MAX_IMPACT_JOINTS) {
            self.joints.set_gravity_torque(i, gravity[i]);
        }

        let mut total_current_ma: u32 = 180;

        for i in 0..self.joint_count {
            let torque_on = ((self.last_cmd.torque_mask >> i) & 1) == 1;
            let stalled = self.faults.stalled_joints[i];
            let mode = JointCommandMode::from_wire(self.last_cmd.command_mode[i]);
            let running_torque = self.running_mode && torque_on && mode == JointCommandMode::Torque;
            // Torque OFF is encoded as a non-positive limit; the plant then holds
            // the joint, draws quiescent current and cools toward ambient.
            let limit = if torque_on { self.last_cmd.torque_nm[i].max(0.0_f32) } else { 0.0_f32 };

            if running_torque {
                // Torque-capable running: torque_nm is a thrust command, not a
                // position-servo target. The vertical hop integrator consumes it;
                // joint angles are held at their last pose. Current tracks the
                // commanded torque so flight detection sees near-zero leg current.
                self.joints.set_angle(i, self.raw_angle_rad[i]);
                let (_, _, stall_ma, _) = self.joints.step(i, dt, mode, 0.0_f32, 0.0_f32, 0.0_f32, stalled);
                let tq = self.last_cmd.torque_nm[i].abs();
                self.velocity_rad_s[i] = 0.0;
                self.current_ma[i] = if stalled {
                    stall_ma
                } else {
                    (tq / RUNNING_NOMINAL_TORQUE_NM * rk_proto::RATED_CURRENT_MA as f32)
                        .clamp(0.0, 3400.0) as u16
                };
            } else {
                let (angle, velocity, ma, _) = self.joints.step(
                    i,
                    dt,
                    mode,
                    self.last_cmd.raw_target_rad[i],
                    self.last_cmd.raw_velocity_rad_s[i],
                    limit,
                    stalled,
                );
                self.raw_angle_rad[i] = angle;
                self.velocity_rad_s[i] = velocity;
                self.current_ma[i] = ma;

                if torque_on {
                    self.virtual_hand_rad[i] = angle;
                } else {
                    // Torque OFF (Teach or EStop): the plant holds the joint;
                    // virtual-hand posing / external wave is an external kinematic
                    // input layered on top (there is no motor torque here).
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
                    let alpha = 1.0_f32 - (-14.0_f32 * dt).exp();
                    let next_pos = angle + (hand_goal - angle) * alpha;
                    self.velocity_rad_s[i] = (next_pos - angle) / dt;
                    self.raw_angle_rad[i] = next_pos;
                    self.joints.set_angle(i, next_pos);
                    self.joints.set_velocity(i, self.velocity_rad_s[i]);
                }
            }

            total_current_ma = total_current_ma.saturating_add(self.current_ma[i] as u32);
            self.temp_c_f[i] = self.joints.temperature_c(i);
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
            let k_cop_pitch = 8.0_f32;
            let k_cop_roll = 22.0_f32;
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

        // Published sensor values are *copies*: the physics plant keeps the
        // true angle/current/temperature so replay and direct plant parity stay
        // unaffected by noise, quantization and offsets.
        let mut published_angles = self.raw_angle_rad;
        let mut published_current = self.current_ma;
        let mut temp_c = [28u8; MAX_JOINTS];
        for i in 0..self.joint_count {
            temp_c[i] = self.temp_c_f[i].round() as u8;
        }

        let lsb = self.faults.encoder_quantization_lsb_rad;
        if lsb.is_finite() && lsb > 0.0_f32 {
            for angle in published_angles.iter_mut().take(self.joint_count) {
                *angle = (*angle / lsb).round() * lsb;
            }
        }
        let current_noise = self.faults.current_noise_pct.clamp(0.0_f32, 1.0_f32);
        if current_noise > 0.0_f32 {
            for ma in published_current.iter_mut().take(self.joint_count) {
                let noisy = *ma as f32 * (1.0_f32 + self.noise_rng.next_f32() * current_noise);
                *ma = noisy.round().clamp(0.0_f32, 65_535.0_f32) as u16;
            }
        }
        let temp_offset = self.faults.temperature_offset_c;
        if temp_offset != 0.0_f32 && temp_offset.is_finite() {
            for temp in temp_c.iter_mut().take(self.joint_count) {
                let dither = self.noise_rng.next_f32() * 0.1_f32 * temp_offset.abs();
                *temp = ((*temp as f32) + temp_offset + dither)
                    .round()
                    .clamp(0.0_f32, 255.0_f32) as u8;
            }
        }

        let nominal_mv: i32 = if self.faults.voltage_sag {
            9_950
        } else {
            12_450
        };
        let sag_mv: i32 = ((total_current_ma as f32 * 0.001_f32) * 85.0_f32) as i32;
        let bus_voltage_mv = (nominal_mv - sag_mv).clamp(8_500, 16_800) as u16;

        let imu = if self.faults.imu_disconnected {
            rk_proto::ImuState::default()
        } else {
            self.imu_sample(dt)
        };
        RawServoFeedback {
            raw_angle_rad: published_angles,
            velocity_rad_s: self.velocity_rad_s,
            current_ma: published_current,
            temp_c,
            bus_voltage_mv,
            bus_error: self.faults.servo_bus_error,
            imu,
            i3c_timeout_count: if self.faults.servo_bus_error { 1 } else { 0 },
            imu_source_mask: if self.faults.imu_disconnected {
                0
            } else {
                rk_proto::estimator_valid::LSM6DSV_OK
            },
            // The plant is the simulated contact sensor. `foot_contacts` were
            // just recomputed by this world step, so publish them with valid
            // CONTACT_SENSED/GRF bits; the kernel closes the support loop from
            // this snapshot instead of trusting its own gait phase.
            contact_mask: self.contact_mask(),
            grf_n: [self.foot_contacts[0].grf_n, self.foot_contacts[1].grf_n],
            contact_valid_mask: estimator_valid::CONTACT_SENSED | estimator_valid::GRF,
        }
    }
}

impl ServoBusPort for SimulatedServoPlant {
    fn read_feedback(&mut self, dt_sec: f32) -> RawServoFeedback {
        // Integrate first, then publish the measured state into the register
        // bank and read it back over the real register-pointer wire encoding.
        let mut feedback = self.step_plant(dt_sec);
        self.publish_feedback(&feedback);
        self.multibus
            .bus_mut()
            .set_bus_error_injection(self.faults.servo_bus_error);
        for joint in 0..self.joint_count.min(MAX_SERVO_JOINTS) {
            let Some(node) = joint_node_addr(joint) else {
                break;
            };
            let mut client = SimControlClient::new(node, I2cTransport::new(&mut self.multibus));
            // On a bus failure the values computed by `step_plant` remain in
            // place, so IMU/voltage/bus health stay independent of servo I/O.
            let _ = read_joint_feedback(&mut client, joint, &mut feedback);
        }
        feedback
    }

    fn write_command(&mut self, cmd: &RawServoCommand) {
        self.multibus
            .bus_mut()
            .set_bus_error_injection(self.faults.servo_bus_error);
        for joint in 0..self.joint_count.min(MAX_SERVO_JOINTS) {
            let Some(node) = joint_node_addr(joint) else {
                break;
            };
            let mut client = SimControlClient::new(node, I2cTransport::new(&mut self.multibus));
            let _ = write_joint_command(&mut client, joint, cmd, &self.torque_constants);
        }
        // No direct `last_cmd = *cmd`: goals now live in the register bank and
        // are decoded by `harvest_commands` at the start of every plant step.
    }
}

/// Quantitative walk-quality snapshot accumulated over every tick since the
/// last experiment reset (`CopperSimRunner::last_walk_quality`).
///
/// All values come from the same telemetry and plant state the rest of the app
/// consumes; no separate measurement model. Fractions are per sampled tick and
/// `single_support_grf_ratio` is the sample with the largest deviation from 1
/// (loaded-foot GRF / body weight), so both under- and over-support are visible.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct WalkQuality {
    /// Mean achieved forward speed (m/s).
    pub mean_forward_speed_m_s: f32,
    /// Peak absolute torso tilt over the whole run (rad).
    pub peak_tilt_rad: f32,
    /// Peak absolute torso tilt over the last second of samples (rad).
    pub steady_tilt_rad: f32,
    /// Lowest/highest world CoM height (m).
    pub com_height_min_m: f32,
    pub com_height_max_m: f32,
    /// Largest lateral deviation from the initial CoM x position (m).
    pub lateral_drift_m: f32,
    /// Largest slip distance a loaded sole accumulated within one plant step
    /// (`slip_m_s * dt`, m). At 200 Hz this is bounded by 0.01 m.
    pub max_foot_slip_m: f32,
    /// Worst single-support GRF / body-weight ratio (1.0 == full weight).
    pub single_support_grf_ratio: f32,
    /// Ticks with no loaded sole (total GRF == 0).
    pub flight_ticks: u32,
    /// Ticks with two loaded soles.
    pub double_support_ticks: u32,
    /// Peak per-joint current over the run (mA).
    pub max_current_ma: u16,
    /// Fraction of ticks whose capturability margin was negative.
    pub margin_negative_fraction: f32,
    /// Fraction of ticks in a genuine catch step: `ACTIVE_CATCH_STEP` while
    /// either not walking or the capture margin is breached. Steady walking
    /// reuses the same wire phase, so `walk_active` disambiguates it.
    pub catch_step_fraction: f32,
    /// Support-foot swaps observed by the plant.
    pub contact_swap_count: u32,
}

/// Per-tick accumulator behind [`WalkQuality`]. Kept inside `SimRobotIo` so the
/// Embassy harness path and the synchronous compatibility path sample the exact
/// same stream, with no wall clock involved.
#[derive(Debug, Default)]
struct WalkQualityRecorder {
    samples: u32,
    speed_sum_m_s: f32,
    peak_tilt_rad: f32,
    /// `(uptime_ms, tilt)` samples for the trailing one-second window.
    tilt_window: VecDeque<(u32, f32)>,
    /// Seeded from the first sample (not from the default 0) so a genuine
    /// zero-height reading is not masked.
    com_min_m: f32,
    com_max_m: f32,
    start_x_m: Option<f32>,
    lateral_drift_m: f32,
    /// Previous cumulative slip, so the recorder can measure the increment of
    /// one plant step rather than the unbounded cumulative distance.
    previous_slip_m: [f32; 2],
    max_foot_slip_m: f32,
    worst_grf_deviation: f32,
    single_support_grf_ratio: f32,
    flight_ticks: u32,
    double_support_ticks: u32,
    max_current_ma: u16,
    margin_negative_ticks: u32,
    catch_step_ticks: u32,
}

impl WalkQualityRecorder {
    /// Samples one published telemetry frame together with the plant state
    /// that produced it (`publish_telemetry` runs immediately after the plant
    /// step, so both are from the same tick).
    fn observe(&mut self, telem: &McuToHost, plant: &SimulatedServoPlant) {
        const MIN_LOADED_GRF_N: f32 = 1.0e-3_f32;
        self.samples += 1;
        self.speed_sum_m_s += telem.balance.world_vel_m_s[2];

        let tilt = telem.imu.pitch_rad.abs().max(telem.imu.roll_rad.abs());
        self.peak_tilt_rad = self.peak_tilt_rad.max(tilt);
        self.tilt_window.push_back((telem.uptime_ms, tilt));
        let window_start = telem.uptime_ms.saturating_sub(1000);
        while self
            .tilt_window
            .front()
            .map_or(false, |(ms, _)| *ms < window_start)
        {
            self.tilt_window.pop_front();
        }

        let com = telem.balance.world_pos_m;
        if self.samples == 1 {
            self.com_min_m = com[1];
            self.com_max_m = com[1];
        } else {
            self.com_min_m = self.com_min_m.min(com[1]);
            self.com_max_m = self.com_max_m.max(com[1]);
        }
        let start_x = *self.start_x_m.get_or_insert(com[0]);
        self.lateral_drift_m = self.lateral_drift_m.max((com[0] - start_x).abs());

        let slip = [
            plant.foot_contact(false).slip_distance_m,
            plant.foot_contact(true).slip_distance_m,
        ];
        for foot in 0..2 {
            self.max_foot_slip_m = self
                .max_foot_slip_m
                .max((slip[foot] - self.previous_slip_m[foot]).max(0.0_f32));
        }
        self.previous_slip_m = slip;

        let loaded = [
            plant.foot_contact(false).contact && plant.foot_contact(false).grf_n > MIN_LOADED_GRF_N,
            plant.foot_contact(true).contact && plant.foot_contact(true).grf_n > MIN_LOADED_GRF_N,
        ];
        match (loaded[0], loaded[1]) {
            (false, false) => self.flight_ticks += 1,
            (true, true) => self.double_support_ticks += 1,
            (true, false) | (false, true) => {
                let grf = if loaded[0] {
                    plant.foot_contact(false).grf_n
                } else {
                    plant.foot_contact(true).grf_n
                };
                let ratio = grf / plant.body_weight_n().max(1.0e-3_f32);
                let deviation = (ratio - 1.0_f32).abs();
                if deviation > self.worst_grf_deviation {
                    self.worst_grf_deviation = deviation;
                    self.single_support_grf_ratio = ratio;
                }
            }
        }

        for ma in telem.current_ma.iter() {
            self.max_current_ma = self.max_current_ma.max(*ma);
        }
        if telem.estimator.foot_margin_m < 0.0_f32 {
            self.margin_negative_ticks += 1;
        }
        // `uvc_phase` has no distinct WALKING wire code (rk-proto is not owned
        // by this phase), so steady walking reports ACTIVE_CATCH_STEP. Discriminate
        // genuine disturbance catch steps with `walk_active` + the reach margin
        // instead of counting every walking tick as a catch step.
        if telem.balance.uvc_phase == rk_proto::uvc_phase::ACTIVE_CATCH_STEP
            && (!telem.balance.walk_active || telem.estimator.foot_margin_m < 0.0_f32)
        {
            self.catch_step_ticks += 1;
        }
    }

    fn snapshot(&self, plant: &SimulatedServoPlant) -> WalkQuality {
        let samples = self.samples.max(1);
        WalkQuality {
            mean_forward_speed_m_s: self.speed_sum_m_s / samples as f32,
            peak_tilt_rad: self.peak_tilt_rad,
            steady_tilt_rad: self
                .tilt_window
                .iter()
                .fold(0.0_f32, |peak, (_, tilt)| peak.max(*tilt)),
            com_height_min_m: if self.samples == 0 {
                0.0_f32
            } else {
                self.com_min_m
            },
            com_height_max_m: self.com_max_m,
            lateral_drift_m: self.lateral_drift_m,
            max_foot_slip_m: self.max_foot_slip_m,
            single_support_grf_ratio: self.single_support_grf_ratio,
            flight_ticks: self.flight_ticks,
            double_support_ticks: self.double_support_ticks,
            max_current_ma: self.max_current_ma,
            margin_negative_fraction: self.margin_negative_ticks as f32 / samples as f32,
            catch_step_fraction: self.catch_step_ticks as f32 / samples as f32,
            contact_swap_count: plant.contact_swaps(),
        }
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
    /// Logical tick index stamped on the next captured plant event. `run_script`
    /// sets this explicitly before applying each tick's events; outside a script
    /// it advances in `record_tick` so UI-driven events land on the next tick.
    event_tick_index: u64,
    /// Fault snapshot last captured into the event side-channel. Faults are
    /// mutated through `&mut SimFaultInjection` (see `plant.faults_mut()`), so
    /// capture is a deterministic diff at each recorded tick — see `record_tick`.
    last_recorded_faults: Option<SimFaultSnapshot>,
    /// Virtual-hand angles last captured into the event side-channel. rk-link
    /// reaches `plant.set_virtual_hand_angle()` directly, bypassing any runner
    /// wrapper, so this is likewise captured by diffing at each recorded tick.
    last_recorded_virtual_hand: Option<[f32; MAX_JOINTS]>,
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
    /// Deterministic walk-quality samples from every published telemetry frame.
    walk_quality: WalkQualityRecorder,
    /// Status LED state evaluated by `RobotIo::post_tick` from the same firmware
    /// state machine the board uses. Purely sim-visible: never recorded in ticks.
    pub sim_led_on: bool,
    /// Number of `post_tick` invocations (diagnostic, not part of replay).
    pub post_tick_calls: u32,
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
            event_tick_index: 0,
            last_recorded_faults: None,
            last_recorded_virtual_hand: None,
            mcu_model_info,
            mcu_model_blob,
            ticker: None,
            use_mock_clock: false,
            dt_override: None,
            pending_cmds: VecDeque::new(),
            last_host_rx: None,
            walk_quality: WalkQualityRecorder::default(),
            sim_led_on: false,
            post_tick_calls: 0,
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

    /// Snapshot of walk-quality metrics accumulated since the last reset.
    /// Available through `CopperSimRunner` via auto-deref as well.
    pub fn last_walk_quality(&self) -> WalkQuality {
        self.walk_quality.snapshot(&self.plant)
    }

    pub fn is_recording(&self) -> bool {
        self.recording
    }

    pub fn start_recording(&mut self) {
        self.recording = true;
        self.record_tick_base = None;
        // Snapshot the current fault/virtual-hand state so the initial
        // configuration is part of the script and subsequent changes are the
        // only events emitted by the per-tick diffs in `record_tick`.
        let initial_faults = fault_snapshot(self.plant.faults());
        self.last_recorded_faults = Some(initial_faults.clone());
        self.last_recorded_virtual_hand = Some(*self.plant.virtual_hand_angles());
        self.event_tick_index = 0;
        self.recorded_log = Some(CopperReplayLog {
            version: 1,
            joint_count: self.task.kernel().joint_count(),
            initial_raw_pose: *self.plant.raw_angles(),
            initial_calibration: self.task.kernel().calibration().to_vec(),
            ticks: Vec::new(),
            seed: initial_faults.seed,
            events: vec![TimedSimEvent {
                tick_index: 0,
                kind: SimEventKind::SetFaults(initial_faults),
            }],
        });
    }

    pub fn stop_recording(&mut self) -> Option<CopperReplayLog> {
        self.recording = false;
        if let Some(log) = self.recorded_log.as_mut() {
            // The seed may have been replaced by a SetFaults event mid-run.
            log.seed = self.plant.faults().seed;
        }
        self.recorded_log.clone()
    }

    /// Overrides the per-recording tick cap (default 2400). Call before
    /// `start_recording`; `run_script` sets it from `ExperimentScript::max_ticks`.
    pub fn set_max_recorded_ticks(&mut self, max_ticks: usize) {
        self.max_recorded_ticks = max_ticks.max(1);
    }

    /// Sets the logical tick index stamped on plant events applied before the
    /// next `step_harness` call. Used by `run_script` to reproduce exact indices.
    pub fn set_event_tick_index(&mut self, tick_index: u64) {
        self.event_tick_index = tick_index;
    }

    /// Records one non-kernel plant event into the active replay log.
    fn record_event(&mut self, kind: SimEventKind) {
        if self.recording {
            if let Some(log) = self.recorded_log.as_mut() {
                log.events.push(TimedSimEvent {
                    tick_index: self.event_tick_index,
                    kind,
                });
            }
        }
    }

    pub fn recorded_log(&self) -> Option<&CopperReplayLog> {
        self.recorded_log.as_ref()
    }

    /// Writes the recorded `.copper` log to disk on native Desktop targets.
    #[cfg(not(any(target_arch = "wasm32", target_os = "android")))]
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
        self.record_event(SimEventKind::Shove {
            pitch_rad_s: pitch_impulse_rad_s,
            roll_rad_s: roll_impulse_rad_s,
        });
    }

    /// Resets the simulated torso strictly upright at 0 deg pitch/roll.
    pub fn reset_upright(&mut self) {
        self.plant.reset_upright();
        self.record_event(SimEventKind::ResetUpright);
    }

    /// Updates active get-up recovery progress so ground push-off lifts the fallen torso upright.
    pub fn set_recovery_progress(&mut self, progress: Option<f32>) {
        self.plant.set_recovery_progress(progress);
        self.record_event(SimEventKind::Recovery(progress));
    }

    /// Applies one scripted plant event. This is the single replay entry point
    /// used by `run_script`; it shares the exact hooks the UI/MCP paths use so
    /// a captured script re-drives the same plant code on replay.
    pub fn apply_event(&mut self, kind: &SimEventKind) {
        match *kind {
            SimEventKind::SetFaults(ref snapshot) => {
                *self.plant.faults_mut() = faults_from_snapshot(snapshot);
            }
            SimEventKind::Shove {
                pitch_rad_s,
                roll_rad_s,
            } => self.apply_push_impulse(pitch_rad_s, roll_rad_s),
            SimEventKind::Recovery(progress) => self.set_recovery_progress(progress),
            SimEventKind::ResetUpright => self.reset_upright(),
            SimEventKind::VirtualHand { joint, rad } => {
                self.plant.set_virtual_hand_angle(joint, rad);
            }
        }
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
        let mut feedback = self.plant.read_feedback(dt_sec);
        // Close the contact loop with the plant's *real* loaded soles. The plant
        // has just stepped the world, so this is the freshest physically
        // available measurement; one control-tick of sensing latency remains
        // because the kernel applies it on the following step. Hardware leaves
        // the validity mask clear and keeps its phase-derived fallback.
        feedback.contact_mask = self.plant.contact_mask();
        // Feedback GRF is `[right, left]` (plant indices [0, 1]).
        feedback.grf_n = [
            self.plant.foot_contact(false).grf_n,
            self.plant.foot_contact(true).grf_n,
        ];
        feedback.contact_valid_mask =
            estimator_valid::CONTACT_SENSED | estimator_valid::GRF;
        feedback
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
        // Contact/GRF are owned by the kernel and come from the sensed feedback
        // it consumed this tick. No post-hoc plant overwrite: telemetry must
        // show the same support origin the controller actually used.
        self.walk_quality.observe(&decoded, &self.plant);
        self.last_host_rx = Some(decoded);
    }

    fn record_tick(&mut self, tick: &ControlTick) {
        if self.recording {
            // Replay verification starts a fresh `ControlTask` at index zero.
            // Rebase the persistent task's running index so a log recorded
            // across many fixed-step calls stays replayable.
            let base = *self.record_tick_base.get_or_insert(tick.tick_index);
            let logical_index = tick.tick_index.saturating_sub(base);

            // Fault and virtual-hand mutations arrive through `&mut` handles
            // (rk-link's `faults_mut()` / `plant.set_virtual_hand_angle()`), so
            // they cannot call a recorder hook at the mutation site. Diffing at
            // each recorded tick is deterministic and captures exactly the
            // state that influenced this tick's feedback. Explicit hooks below
            // handle events with no persistent snapshot to diff.
            let current_faults = fault_snapshot(self.plant.faults());
            if self.last_recorded_faults.as_ref() != Some(&current_faults) {
                self.last_recorded_faults = Some(current_faults.clone());
                if let Some(log) = self.recorded_log.as_mut() {
                    log.events.push(TimedSimEvent {
                        tick_index: logical_index,
                        kind: SimEventKind::SetFaults(current_faults),
                    });
                }
            }
            let previous_hand = self.last_recorded_virtual_hand;
            let current_hand = *self.plant.virtual_hand_angles();
            if previous_hand != Some(current_hand) {
                for (joint, rad) in current_hand.into_iter().enumerate() {
                    if previous_hand.map_or(true, |prev| prev[joint] != rad) {
                        if let Some(log) = self.recorded_log.as_mut() {
                            log.events.push(TimedSimEvent {
                                tick_index: logical_index,
                                kind: SimEventKind::VirtualHand { joint, rad },
                            });
                        }
                    }
                }
                self.last_recorded_virtual_hand = Some(current_hand);
            }
            self.event_tick_index = logical_index.saturating_add(1);

            if let Some(log) = self.recorded_log.as_mut() {
                if log.ticks.len() < self.max_recorded_ticks {
                    let mut recorded = *tick;
                    recorded.tick_index = logical_index;
                    log.ticks.push(recorded);
                }
            }
        }
    }

    fn post_tick(&mut self, now_ms: u32) {
        // Same side effect as the RP2350 board hook: the LED mode was refreshed
        // from the kernel's safety snapshot earlier in `step_control_tick`.
        // This is intentionally outside `record_tick`, so replay stays bit-exact.
        self.post_tick_calls = self.post_tick_calls.saturating_add(1);
        self.sim_led_on = rk_fw_core::copper::current_led_mode().is_lit(now_ms);
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

/// Control loop driven by the deterministic harness.
///
/// Unlike the generic `rk_fw_core::control_task::control_task`, the kernel lives
/// in `SimRobotIo::task` instead of as a local of this future. That is what lets
/// `reset_experiment` install a fresh `ControlTask` while the future stays parked
/// on the global mock timer queue (dropping/re-spawning it would strand an
/// alarm). Every tick still goes through the shared `step_control_tick`, so the
/// firmware and simulator execute the exact same control body.
async fn persistent_control_task(mut io: PersistentIoPtr) {
    loop {
        io.wait_tick().await;
        // SAFETY: the harness polls this future synchronously and never while
        // the runner holds a `&mut` to the same IO. The kernel is moved out for
        // exactly one tick so `io` and the kernel are disjoint borrows.
        let mut task = {
            let io_ref = unsafe { &mut *io.0 };
            let placeholder = ControlTask::new(1, &[JointCalibration::default(); MAX_JOINTS]);
            core::mem::replace(&mut io_ref.task, placeholder)
        };
        step_control_tick(&mut io, &mut task);
        unsafe {
            (&mut *io.0).task = task;
        }
    }
}

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

/// A deterministic, headless experiment specification.
///
/// `packets[i]` is the exact typed `HostToMcu` the kernel consumes on tick `i`
/// (missing packets leave that tick command-less). `events` are applied before
/// the tick named by `tick_index`, and `seed` seeds the simulator's sensor-noise
/// PRNG. No wall-clock or frame-derived packets are involved: the script is the
/// complete input to [`CopperSimRunner::run_script`].
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[serde(default)]
pub struct ExperimentScript {
    pub seed: u64,
    pub events: Vec<TimedSimEvent>,
    pub packets: Vec<HostToMcu>,
    /// Number of control ticks to drive. This is also the recorder cap for the
    /// resulting log; events at indices >= `max_ticks` are ignored.
    pub max_ticks: usize,
}

/// Scripted mid-walk disturbance scenarios.
///
/// Each scenario starts from [`SimFaultInjection::realistic_small_humanoid`]
/// and overlays one realistic imperfection for a bounded window, so a script
/// exercises both the disturbance and the recovery with the same seed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WalkDisturbance {
    /// Sole Coulomb friction drops to 0.25 (slippery ground).
    LowFriction,
    /// IMU latency and noise are raised well beyond the nominal profile.
    LatencyAndNoise,
    /// One right-leg joint mechanically stalls during stance.
    StanceStall,
    /// The IMU stops publishing samples mid-walk.
    ImuDropout,
}

impl WalkDisturbance {
    /// Fault configuration applied while the scenario is active.
    pub fn faults(self) -> SimFaultInjection {
        let mut faults = SimFaultInjection::realistic_small_humanoid();
        match self {
            Self::LowFriction => faults.foot_friction_mu = 0.25,
            Self::LatencyAndNoise => {
                faults.imu_latency_ticks = 6;
                faults.imu_gyro_noise_rad_s = 0.05;
                faults.imu_accel_noise_m_s2 = 0.08;
                faults.current_noise_pct = 0.06;
            }
            Self::StanceStall => faults.stalled_joints[10] = true,
            Self::ImuDropout => faults.imu_disconnected = true,
        }
        faults
    }

    /// Short stable label used in test diagnostics.
    pub const fn label(self) -> &'static str {
        match self {
            Self::LowFriction => "low_friction",
            Self::LatencyAndNoise => "latency_and_noise",
            Self::StanceStall => "stance_stall",
            Self::ImuDropout => "imu_dropout",
        }
    }
}

/// Deterministic walking command used by the scripted disturbance scenarios.
/// Matches the end-to-end walking test (forward 0.5, 1.55 Hz cadence).
pub fn scripted_walk_packet(seq: u32) -> HostToMcu {
    HostToMcu {
        seq,
        mode: rk_proto::ControlMode::Drive,
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
    }
}

/// Builds a headless experiment that walks with the realistic sensor profile,
/// injects `scenario` at `disturb_tick` for `WINDOW` ticks, then restores the
/// nominal profile so recovery can be observed. Suitable for `run_script`,
/// MCP experiments and direct unit tests.
pub fn walking_disturbance_script(
    scenario: WalkDisturbance,
    total_ticks: usize,
    disturb_tick: usize,
) -> ExperimentScript {
    const DISTURBANCE_WINDOW_TICKS: usize = 100;
    let nominal = fault_snapshot(&SimFaultInjection::realistic_small_humanoid());
    let disturbed = fault_snapshot(&scenario.faults());
    let restore_tick = disturb_tick + DISTURBANCE_WINDOW_TICKS;
    let mut events = vec![TimedSimEvent {
        tick_index: 0,
        kind: SimEventKind::SetFaults(nominal.clone()),
    }];
    if disturb_tick < total_ticks {
        events.push(TimedSimEvent {
            tick_index: disturb_tick as u64,
            kind: SimEventKind::SetFaults(disturbed),
        });
    }
    if restore_tick < total_ticks {
        events.push(TimedSimEvent {
            tick_index: restore_tick as u64,
            kind: SimEventKind::SetFaults(nominal),
        });
    }
    ExperimentScript {
        seed: DEFAULT_NOISE_SEED,
        events,
        packets: (0..total_ticks as u32)
            .map(scripted_walk_packet)
            .collect(),
        max_ticks: total_ticks,
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
    /// Construction inputs retained so `reset_experiment` can rebuild an
    /// identical kernel + plant instead of trying to reset a kernel owned by a
    /// parked (and therefore unreachable) control-task future.
    reset_spec: RunnerResetSpec,
}

/// Immutable construction inputs needed to reproduce a runner exactly.
#[derive(Clone)]
struct RunnerResetSpec {
    joint_count: usize,
    initial_cal: Vec<JointCalibration>,
    initial_raw_pose: [f32; MAX_JOINTS],
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
            .spawn(move || Box::pin(persistent_control_task(PersistentIoPtr(io))))
            .expect("fresh harness task storage must be spawnable");
        harness.spawner().spawn(token);

        Self {
            io: NonNull::new(io).expect("Box::into_raw never returns null"),
            harness,
            _storage: storage,
            reset_spec: RunnerResetSpec {
                joint_count,
                initial_cal: initial_cal.to_vec(),
                initial_raw_pose,
            },
        }
    }

    /// Quantitative walk-quality snapshot accumulated from the telemetry and
    /// plant state of every tick since the last reset.
    pub fn last_walk_quality(&self) -> WalkQuality {
        unsafe { self.io.as_ref() }.last_walk_quality()
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

    /// Decodes a delimited COBS `HostToMcu` frame through the exact same
    /// `CobsStreamFramer` the firmware USB CDC-ACM path uses.
    ///
    /// This is the minimal host-frame decoder exposed for the fixed-step
    /// transport: codec errors are forwarded into this runner's local mailbox,
    /// so `wire_error_counts()` (and therefore the next telemetry snapshot)
    /// observe the very same counters the app reads on hardware.
    pub fn decode_host_frame(&mut self, wire_frame: &mut [u8]) -> Option<HostToMcu> {
        self.decode_rx(wire_frame)
    }

    /// Resets the whole simulation experiment to its construction state:
    /// control kernel, plant, fault injection, wire framer/mailbox and replay
    /// recorder all start fresh, so an identical scripted command stream
    /// produces identical replay bytes.
    ///
    /// The persistent executor and its future are intentionally kept alive:
    /// the Embassy mock clock cannot be reset after the task has registered a
    /// periodic alarm (doing so strands the alarm and never wakes the task
    /// again). The harness loop reads the kernel from `SimRobotIo::task` on
    /// every tick, so replacing that field installs a genuinely fresh kernel —
    /// calibration, soft limits, zero offsets and every internal filter — for
    /// the next tick. `uptime_ms` still advances monotonically (as it does on hardware), so
    /// callers compare replay/tick streams rather than absolute wall-clock
    /// fields. The MCP bridge channel is owned outside this runner and is
    /// intentionally untouched.
    pub fn reset_experiment(&mut self) {
        let spec = self.reset_spec.clone();
        let io = self.io_mut();

        // Rebuild the kernel from the construction inputs so every internal
        // filter/integrator/calibration state starts from the same values.
        io.task = ControlTask::new(spec.joint_count, &spec.initial_cal);

        // Plant: replace wholesale with a fresh instance seeded from the same
        // pose. Fault injection defaults to off, so all four wire knobs reset.
        io.plant = SimulatedServoPlant::new(spec.joint_count, spec.initial_raw_pose);

        // Wire: drop any queued commands, clear the stream framer and reset the
        // local mailbox so no stale packet or error count leaks into the run.
        io.host_framer = CobsStreamFramer::new();
        io.host_mailbox = HostLinkMailbox::new();
        io.pending_cmds.clear();
        io.last_host_rx = None;
        io.walk_quality = WalkQualityRecorder::default();
        io.host_decode_errors = 0;
        io.uptime_ms_f = 0.0;
        io.sim_led_on = false;
        io.post_tick_calls = 0;

        // Recorder: stop and clear any log from the previous experiment.
        io.recording = false;
        io.recorded_log = None;
        io.record_tick_base = None;
        io.max_recorded_ticks = 2400;
        io.event_tick_index = 0;
        io.last_recorded_faults = None;
        io.last_recorded_virtual_hand = None;

        // Keep the task driven by the same mock clock (no reset, no re-spawn).
        io.use_mock_clock = true;
        io.dt_override = None;
        // Keep the ticker: dropping it would re-register the first alarm at a
        // different phase against the still-running mock clock. The periodic
        // Ticker resumes from its existing schedule, so ticks stay aligned.
    }

    /// Runs a scripted experiment from a clean reset and returns a replayable log.
    ///
    /// The experiment is reset first (kernel, plant, faults, wire, host command
    /// sequencing and recorder), then each tick applies its scheduled plant
    /// events before feeding the script's typed packet through `step_harness`.
    /// Events are recorded by the same hooks used by UI/MCP experiments, so the
    /// returned log's `events` side-channel is derived from the applied events
    /// rather than copied from the script. Re-running the same script after
    /// another reset produces bit-identical ticks (modulo the monotonically
    /// advancing `now_ms`, which replay normalizes by feeding it back).
    pub fn run_script(&mut self, script: &ExperimentScript) -> CopperReplayLog {
        self.reset_experiment();
        let max_ticks = script.max_ticks.max(1);
        self.set_max_recorded_ticks(max_ticks);
        self.plant.faults_mut().seed = script.seed;
        self.start_recording();

        let mut events_by_tick: BTreeMap<u64, Vec<&TimedSimEvent>> = BTreeMap::new();
        for event in &script.events {
            if event.tick_index < max_ticks as u64 {
                events_by_tick.entry(event.tick_index).or_default().push(event);
            }
        }

        for tick_index in 0..max_ticks as u64 {
            self.set_event_tick_index(tick_index);
            if let Some(events) = events_by_tick.get(&tick_index) {
                for event in events {
                    self.apply_event(&event.kind);
                }
            }
            match script.packets.get(tick_index as usize) {
                Some(packet) => {
                    let _ = self.step_harness(1, core::slice::from_ref(packet));
                }
                None => {
                    let _ = self.step_harness(1, &[]);
                }
            }
        }

        self.stop_recording()
            .expect("start_recording installed a replay log")
    }

    /// Status LED state from the last control tick's `post_tick` side effect.
    /// Other agents expose this through `RobotTransport` / the control HUD.
    pub fn sim_led_on(&self) -> bool {
        self.sim_led_on
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

    /// A command written as BareFOC register transactions must reach the plant:
    /// the raw struct copy is gone, so only the wire can move the joint.
    #[test]
    fn multibus_wire_command_drives_plant() {
        let mut plant = SimulatedServoPlant::new(6, [0.0; MAX_JOINTS]);
        let mut cmd = RawServoCommand::default();
        cmd.torque_mask = 1;
        cmd.raw_target_rad[0] = 0.8;
        let start = plant.raw_angles()[0];
        for _ in 0..20 {
            plant.write_command(&cmd);
            let _ = plant.step_plant(0.01);
        }
        assert!(
            plant.raw_angles()[0] > start + 0.1,
            "wire-written goal must move the joint: {start} -> {}",
            plant.raw_angles()[0]
        );
        let bank = plant
            .multibus
            .bus()
            .register_bank(0x20)
            .expect("node 0x20 exists");
        assert_eq!(bank[REG_TORQUE_ENABLE as usize], 1);
        assert_eq!(bank[REG_OPERATING_MODE as usize], OperatingMode::Position as u8);
        assert_eq!(
            &bank[REG_GOAL_POSITION as usize..REG_GOAL_POSITION as usize + 4],
            &0.8_f32.to_le_bytes()
        );
    }

    /// Feedback published by the plant into `0x142` must come back through a
    /// burst read and populate the returned `RawServoFeedback` joint fields.
    #[test]
    fn multibus_feedback_burst_round_trips_plant_state() {
        let mut plant = SimulatedServoPlant::new(6, [0.0; MAX_JOINTS]);
        let mut cmd = RawServoCommand::default();
        cmd.torque_mask = 1;
        cmd.raw_target_rad[0] = 0.5;
        for _ in 0..10 {
            plant.write_command(&cmd);
            let _ = plant.step_plant(0.01);
        }
        let feedback = plant.read_feedback(0.01);

        let bank = plant
            .multibus
            .bus()
            .register_bank(0x20)
            .expect("node 0x20 exists");
        let start = FEEDBACK_BURST_REG as usize;
        let published =
            JointFeedback::from_burst(&bank[start..start + FEEDBACK_BURST_LEN]).unwrap();
        assert!(
            (published.position_rad - feedback.raw_angle_rad[0]).abs() < 1e-6,
            "burst position must match the returned feedback"
        );
        assert!(
            (published.velocity_rad_s - feedback.velocity_rad_s[0]).abs() < 1e-6,
            "burst velocity must match the returned feedback"
        );
        assert!(published.current_amps > 0.0);
        assert_eq!(feedback.current_ma[0], (published.current_amps * 1000.0 + 0.5) as u16);
        assert_eq!(feedback.temp_c[0], published.temperature_c as u8);
        assert_ne!(feedback.current_ma[1], feedback.current_ma[0]);
    }

    /// The register server honours the pointer-latch idiom, bounds-checks both
    /// directions and does not clobber the latch on a pointer-less burst read.
    #[test]
    fn multibus_register_server_latch_and_bounds() {
        let mut pio = SimMultibusPio::new();
        pio.transaction(0x20, &[0x01, 0x44, 1, 2, 3, 4], &mut [])
            .unwrap();
        let mut burst = [0u8; 4];
        pio.transaction(0x20, &[], &mut burst).unwrap();
        assert_eq!(burst, [1, 2, 3, 4]);
        pio.transaction(0x20, &[], &mut burst).unwrap();
        assert_eq!(burst, [1, 2, 3, 4], "empty-tx read must keep the latch");

        let mut scratch = [0u8; 4];
        assert_eq!(
            pio.transaction(0x20, &[0x01, 0xFE, 0, 0, 0], &mut []),
            Err(PioError::Bus),
            "write past 0x200 must fail"
        );
        assert_eq!(
            pio.transaction(0x20, &[0x01, 0xFE], &mut scratch),
            Err(PioError::Bus),
            "read past 0x200 must fail"
        );
        assert_eq!(
            pio.transaction(0x40, &[0, 0], &mut []),
            Err(PioError::Bus),
            "unmapped node must fail"
        );

        pio.set_bus_error_injection(true);
        assert_eq!(pio.transaction(0x20, &[0x01, 0x44], &mut []), Err(PioError::Bus));
    }

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
        #[cfg(not(any(target_arch = "wasm32", target_os = "android")))]
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
        // With the shared torque-based plant, compliance only changes measured
        // current when the demanded torque exceeds the soft ceiling. Drive a
        // high-inertia hip joint (I≈0.038 kg·m²) so the step genuinely saturates
        // at 0.45 N·m with 25% compliance and at 1.8 N·m when stiff.
        let cal = [JointCalibration {
            min_rad: -2.2,
            max_rad: 2.2,
            ..Default::default()
        }; 21];
        let mut sim_stiff = CopperSimRunner::new(21, &cal, [0.0; MAX_JOINTS]);
        let mut sim_soft = CopperSimRunner::new(21, &cal, [0.0; MAX_JOINTS]);

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
            torque_mask: (1u32 << 21) - 1,
            speed_limit_pct: 100,
            ..Default::default()
        };
        cmd.target_rad[11] = 1.0;

        let mut stiff_peak_ma = 0u16;
        let mut soft_peak_ma = 0u16;
        for step in 2..=30 {
            cmd.seq = step;
            let t_stiff = sim_stiff.tick(0.01, &cmd);
            let t_soft = sim_soft.tick(0.01, &cmd);
            stiff_peak_ma = stiff_peak_ma.max(t_stiff.current_ma[11]);
            soft_peak_ma = soft_peak_ma.max(t_soft.current_ma[11]);
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
        let mut sim = CopperSimRunner::new(21, &cal, [0.0; MAX_JOINTS]);
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

        // 800 fixed 5 ms ticks = 4 s of deterministic simulated time. Stepping
        // one tick at a time lets the test separate the startup transient
        // (first second) from steady-state contact quality.
        // Gait/contact ramp-up lasts ~1.05 s (speed still accelerating and the
        // swing foot settling); the toe-off/heel-strike dwell leaves only the
        // pre-control plant tick 1 airborne. Steady-state flight is asserted
        // separately below.
        const STARTUP_TICKS: usize = 250;
        let mut last = McuToHost::default();
        let mut steady_flight_ticks = 0_u32;
        for i in 1..=800 {
            cmd.seq = i;
            last = sim.step_harness(1, core::slice::from_ref(&cmd));
            if i > STARTUP_TICKS as u32 && sim.plant.ground_reaction_total_n() <= 0.0_f32 {
                steady_flight_ticks += 1;
            }
        }
        let quality = sim.last_walk_quality();
        eprintln!("walk quality: {quality:?}");
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

        // --- Final P4 walk-quality acceptance --------------------------------
        // A loaded sole must carry the full body weight; frictional unloading
        // may reduce it slightly but never by more than 15%.
        assert!(
            (quality.single_support_grf_ratio - 1.0_f32).abs() < 0.15_f32,
            "single-support GRF ratio {:.3} must be within 15% of body weight",
            quality.single_support_grf_ratio
        );
        assert_eq!(
            steady_flight_ticks, 0,
            "steady walking must not go airborne, got {steady_flight_ticks} ticks"
        );
        // The single flight sample is plant tick 1, before the first control
        // output reaches the joints; the swing-sole dwell removes every other
        // flight tick including all support swaps.
        assert!(
            quality.flight_ticks <= 1,
            "walking must have no flight beyond plant initialization, got {} ticks",
            quality.flight_ticks
        );
        assert!(
            !last.faults.contains(FaultFlags::OVER_CURRENT)
                && quality.max_current_ma < 1950,
            "walking must stay below the 1950 mA advisory, peak {} mA",
            quality.max_current_ma
        );
        assert!(
            quality.max_foot_slip_m < 0.05_f32,
            "per-plant-step slip {:.4} m must stay below 0.05 m",
            quality.max_foot_slip_m
        );

        // `v_des = vx * WALK_SPEED_SCALE` (0.5) is the documented effective
        // command, so vx=0.5 targets 0.25 m/s. Allow ±10%.
        const COMMAND_M_S: f32 = 0.5_f32 * 0.5_f32;
        assert!(
            (quality.mean_forward_speed_m_s - COMMAND_M_S).abs() <= 0.10_f32 * COMMAND_M_S,
            "mean speed {:.3} m/s outside ±10% of {COMMAND_M_S:.3}",
            quality.mean_forward_speed_m_s
        );
        assert!(
            quality.peak_tilt_rad < 0.08_f32 && quality.steady_tilt_rad < 0.03_f32,
            "peak/steady tilt {:.3}/{:.3} rad exceeds 0.08/0.03",
            quality.peak_tilt_rad,
            quality.steady_tilt_rad
        );
        assert!(
            quality.lateral_drift_m < 0.03_f32,
            "lateral drift {:.3} m exceeds 0.03 m",
            quality.lateral_drift_m
        );
        // Default `h_com_m` is 0.72 m; the walk must keep the mass-model CoM
        // within ±0.03 m of it.
        assert!(
            (quality.com_height_min_m - 0.72_f32).abs() <= 0.03_f32
                && (quality.com_height_max_m - 0.72_f32).abs() <= 0.03_f32,
            "CoM height range {:.3}..{:.3} m outside 0.72 ± 0.03 m",
            quality.com_height_min_m,
            quality.com_height_max_m
        );
        assert!(
            quality.margin_negative_fraction < 0.30_f32,
            "capturability margin negative on {:.3} of ticks (>=0.30)",
            quality.margin_negative_fraction
        );
        // uvc.rs has no distinct WALKING wire code, so `catch_step_fraction`
        // counts only non-walking or margin-breached catch-step ticks (see the
        // recorder). Steady walking must not be reported as constant catching.
        assert!(
            quality.catch_step_fraction < 0.5_f32,
            "steady walk misreported as catch steps: {:.3}",
            quality.catch_step_fraction
        );
        assert!(
            quality.catch_step_fraction > 0.0_f32,
            "genuine margin-breach catch steps must still engage"
        );
    }

    /// P3 robustness: the same walk under the realistic small-humanoid sensor
    /// profile must not latch a fall or over-current fault, and its measured
    /// quality must stay in loose-but-meaningful bounds. TODO P4: tighten to
    /// the final acceptance values (±10% speed, tilt < 0.08/0.03 rad).
    #[test]
    fn realistic_sensor_walking_has_no_fault_latch() {
        let _guard = HARNESS_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let cal = [JointCalibration {
            min_rad: -2.2,
            max_rad: 2.2,
            ..Default::default()
        }; 21];
        let mut sim = CopperSimRunner::new(21, &cal, [0.0; MAX_JOINTS]);
        *sim.plant.faults_mut() = SimFaultInjection::realistic_small_humanoid();

        let mut cmd = scripted_walk_packet(1);
        let mut last = McuToHost::default();
        for seq in 1..=800u32 {
            cmd.seq = seq;
            last = sim.step_harness(1, core::slice::from_ref(&cmd));
        }
        let quality = sim.last_walk_quality();
        eprintln!("realistic walk quality: {quality:?}");
        assert_eq!(last.active_mode, ControlMode::Drive);
        assert!(
            !last.faults.contains(FaultFlags::TILT_FALL_CUTOFF),
            "realistic sensors must not latch a fall: {:?}",
            last.faults
        );
        assert!(
            !last.faults.contains(FaultFlags::OVER_CURRENT),
            "realistic sensors must not latch over-current: {:?}",
            last.faults
        );
        assert!(
            sim.plant.forward_displacement_m() > 0.10_f32,
            "realistic walk must still translate forward"
        );
        const COMMAND_M_S: f32 = 0.5_f32 * 0.5_f32;
        assert!(
            (quality.mean_forward_speed_m_s - COMMAND_M_S).abs() <= 0.10_f32 * COMMAND_M_S,
            "realistic mean speed {:.3} m/s outside ±10% of {COMMAND_M_S:.3}",
            quality.mean_forward_speed_m_s
        );
        assert!(
            quality.peak_tilt_rad < 0.08_f32 && quality.steady_tilt_rad < 0.03_f32,
            "realistic tilt {:.3}/{:.3} rad exceeds 0.08/0.03",
            quality.peak_tilt_rad,
            quality.steady_tilt_rad
        );
        assert!(
            quality.lateral_drift_m < 0.03_f32,
            "realistic lateral drift {:.3} m exceeds 0.03 m",
            quality.lateral_drift_m
        );
        assert!(
            (quality.com_height_min_m - 0.72_f32).abs() <= 0.03_f32
                && (quality.com_height_max_m - 0.72_f32).abs() <= 0.03_f32,
            "realistic CoM height {:.3}..{:.3} m outside 0.72 ± 0.03",
            quality.com_height_min_m,
            quality.com_height_max_m
        );
        assert!(
            (quality.single_support_grf_ratio - 1.0_f32).abs() < 0.15_f32,
            "realistic single-support GRF ratio {:.3} outside ±15%",
            quality.single_support_grf_ratio
        );
        assert!(sim.plant.ground_reaction_total_n() > 0.0_f32);
        assert!(
            quality.flight_ticks <= 1,
            "realistic walk has {} flight ticks (only plant tick 1 is allowed)",
            quality.flight_ticks
        );
        assert!(
            quality.max_current_ma < 1950,
            "realistic peak current {} mA exceeds 1950 mA advisory",
            quality.max_current_ma
        );
        assert!(
            quality.margin_negative_fraction < 0.30_f32,
            "realistic capturability margin negative on {:.3} of ticks",
            quality.margin_negative_fraction
        );
    }

    /// P3 robustness: a shove mid-walk with realistic sensors must be recovered
    /// without latching fall/over-current faults; speed/tilt/GRF stay bounded.
    #[test]
    fn realistic_sensor_shove_while_walking_recovers() {
        let _guard = HARNESS_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let cal = [JointCalibration {
            min_rad: -2.2,
            max_rad: 2.2,
            ..Default::default()
        }; 21];
        let mut sim = CopperSimRunner::new(21, &cal, [0.0; MAX_JOINTS]);
        *sim.plant.faults_mut() = SimFaultInjection::realistic_small_humanoid();

        let mut cmd = scripted_walk_packet(1);
        let mut last = McuToHost::default();
        for seq in 1..=800u32 {
            cmd.seq = seq;
            if seq == 300 {
                sim.apply_push_impulse(1.2_f32, -0.4_f32);
            }
            last = sim.step_harness(1, core::slice::from_ref(&cmd));
        }
        let quality = sim.last_walk_quality();
        eprintln!("realistic shoved walk quality: {quality:?}");
        assert_eq!(last.active_mode, ControlMode::Drive);
        assert!(
            !last.faults.contains(FaultFlags::TILT_FALL_CUTOFF)
                && !last.faults.contains(FaultFlags::OVER_CURRENT),
            "realistic shove must not latch faults: {:?}",
            last.faults
        );
        // A mid-walk shove is a disturbance, so peak tilt may exceed the
        // undisturbed 0.08 rad bound, but speed/GRF/current and the recovered
        // trailing-second tilt are still held to tightened P4 limits.
        const COMMAND_M_S: f32 = 0.5_f32 * 0.5_f32;
        assert!(
            (quality.mean_forward_speed_m_s - COMMAND_M_S).abs() <= 0.10_f32 * COMMAND_M_S,
            "shoved mean speed {:.3} m/s outside ±10% of {COMMAND_M_S:.3}",
            quality.mean_forward_speed_m_s
        );
        assert!(
            quality.peak_tilt_rad < 0.35_f32 && quality.steady_tilt_rad < 0.05_f32,
            "shoved tilt {:.3}/{:.3} rad out of range",
            quality.peak_tilt_rad,
            quality.steady_tilt_rad
        );
        assert!(
            quality.lateral_drift_m < 0.08_f32,
            "shove-induced lateral drift {:.3} m out of range",
            quality.lateral_drift_m
        );
        assert!(
            (quality.single_support_grf_ratio - 1.0_f32).abs() < 0.15_f32,
            "shoved single-support GRF ratio {:.3} out of range",
            quality.single_support_grf_ratio
        );
        assert!(
            quality.max_current_ma < 1950,
            "shoved peak current {} mA exceeds advisory",
            quality.max_current_ma
        );
        assert!(
            quality.margin_negative_fraction < 0.30_f32,
            "shoved capturability margin negative on {:.3} of ticks",
            quality.margin_negative_fraction
        );
    }

    /// P3 scripted scenarios: each disturbance runs through `ExperimentScript`
    /// and `run_script`, is recorded, and leaves bounded quality metrics after
    /// the fault window is restored. TODO P4: per-scenario recovery thresholds.
    #[test]
    fn scripted_walk_disturbance_scenarios_are_replayable_and_bounded() {
        let _guard = HARNESS_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let cal = [JointCalibration {
            min_rad: -2.2,
            max_rad: 2.2,
            ..Default::default()
        }; 21];
        let mut sim = CopperSimRunner::new(21, &cal, [0.0; MAX_JOINTS]);
        for scenario in [
            WalkDisturbance::LowFriction,
            WalkDisturbance::LatencyAndNoise,
            WalkDisturbance::StanceStall,
            WalkDisturbance::ImuDropout,
        ] {
            let script = walking_disturbance_script(scenario, 600, 250);
            let log = sim.run_script(&script);
            assert_eq!(log.ticks.len(), 600, "{} tick count", scenario.label());
            assert!(
                log.events.iter().any(|event| {
                    event.tick_index == 250
                        && matches!(event.kind, SimEventKind::SetFaults(ref f) if f == &fault_snapshot(&scenario.faults()))
                }),
                "{} must be scripted at tick 250",
                scenario.label()
            );
            let quality = sim.last_walk_quality();
            let last_faults = log.ticks.last().expect("script ticks").tx_telem.faults;
            eprintln!(
                "{} quality: {quality:?} faults={last_faults:?}",
                scenario.label()
            );
            // Friction and sensor disturbances must recover cleanly under the
            // realistic profile. Stall/dropout are intentionally severe, so
            // their peak tilt/current budgets are wider, but speed, GRF and
            // capturability are still held to the P4 acceptance band.
            if matches!(
                scenario,
                WalkDisturbance::LowFriction | WalkDisturbance::LatencyAndNoise
            ) {
                assert!(
                    !last_faults.contains(FaultFlags::TILT_FALL_CUTOFF)
                        && !last_faults.contains(FaultFlags::OVER_CURRENT),
                    "{} must latch no fall/over-current fault",
                    scenario.label()
                );
            }
            const COMMAND_M_S: f32 = 0.5_f32 * 0.5_f32;
            assert!(
                (quality.mean_forward_speed_m_s - COMMAND_M_S).abs() <= 0.10_f32 * COMMAND_M_S,
                "{} mean speed {:.3} outside ±10% of {COMMAND_M_S:.3}",
                scenario.label(),
                quality.mean_forward_speed_m_s
            );
            assert!(
                quality.peak_tilt_rad < 0.20_f32,
                "{} peak tilt {:.3} out of range",
                scenario.label(),
                quality.peak_tilt_rad
            );
            assert!(
                (quality.single_support_grf_ratio - 1.0_f32).abs() <= 0.15_f32,
                "{} single-support GRF {:.3} outside ±15%",
                scenario.label(),
                quality.single_support_grf_ratio
            );
            assert!(
                quality.margin_negative_fraction < 0.30_f32,
                "{} capturability margin negative on {:.3} of ticks",
                scenario.label(),
                quality.margin_negative_fraction
            );
            assert!(
                quality.mean_forward_speed_m_s.is_finite()
                    && quality.peak_tilt_rad.is_finite()
                    && quality.single_support_grf_ratio.is_finite(),
                "{} must produce finite quality metrics",
                scenario.label()
            );
            assert!(
                verify_deterministic_replay(&log).exact_match,
                "{} replay must stay bit-exact",
                scenario.label()
            );
        }
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

    // --- simulator-fidelity-hardening / Group 1 ------------------------------

    /// The plant stores soles as `[right, left]` but `foot_contact(left)` indexes
    /// 1/0. Telemetry must not invert the two GRF channels or the contact mask.
    #[test]
    fn telemetry_maps_left_and_right_grf_to_correct_channels() {
        let cal = [JointCalibration::default(); MAX_JOINTS];
        let mut io = SimRobotIo::new(21, &cal, [0.0; MAX_JOINTS]);

        // Asymmetric sensed load: only the left sole carries ground reaction.
        let mut fb = RawServoFeedback::default();
        fb.contact_mask = 0b10;
        fb.grf_n = [0.0, 42.0]; // `[right, left]`
        fb.contact_valid_mask = estimator_valid::CONTACT_SENSED | estimator_valid::GRF;
        let mut task = ControlTask::new(21, &cal);
        let out = task.step(0, 0.005, None, &fb, (0, 0));

        // The plant's live contacts deliberately disagree with the kernel.
        // `publish_telemetry` must not overwrite the kernel's support estimate.
        let right = io.plant.foot_contact_mut(false);
        right.contact = true;
        right.grf_n = 99.0;
        let left = io.plant.foot_contact_mut(true);
        left.contact = false;
        left.grf_n = 0.0;

        RobotIo::publish_telemetry(&mut io, out.telemetry);
        let telem = io.last_host_rx.expect("telemetry published");
        assert_eq!(
            telem.estimator.grf_left_n, 42.0,
            "left-loaded feedback must report left GRF, got {}",
            telem.estimator.grf_left_n
        );
        assert_eq!(
            telem.estimator.grf_right_n, 0.0,
            "unloaded right sole must report zero GRF"
        );
        assert_eq!(
            telem.estimator.contact_mask, 0b10,
            "contact mask bit 1 = left only"
        );
        assert_ne!(telem.estimator.valid_mask & estimator_valid::GRF, 0);
    }

    #[test]
    fn read_feedback_publishes_real_plant_contact_after_world_step() {
        let cal = [JointCalibration::default(); 21];
        let mut io = SimRobotIo::new(21, &cal, [0.0; MAX_JOINTS]);
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
        // Warm up the gait until a single sole is actually loaded.
        let mut found = false;
        for seq in 1..=120 {
            cmd.seq = seq;
            let _ = io.tick(0.005, &cmd);
            let contacts = io.plant.foot_contacts();
            let loaded = (contacts[0].contact && contacts[0].grf_n > 1.0e-3_f32)
                ^ (contacts[1].contact && contacts[1].grf_n > 1.0e-3_f32);
            if loaded {
                found = true;
                break;
            }
        }
        assert!(found, "walking plant must eventually load a single sole");
        let fb = RobotIo::read_feedback(&mut io, 0.005);
        assert_eq!(fb.contact_mask, io.plant.contact_mask());
        assert_eq!(fb.grf_n[0], io.plant.foot_contact(false).grf_n);
        assert_eq!(fb.grf_n[1], io.plant.foot_contact(true).grf_n);
        assert_ne!(
            fb.contact_valid_mask & estimator_valid::CONTACT_SENSED,
            0
        );
        assert_ne!(fb.contact_valid_mask & estimator_valid::GRF, 0);
    }

    /// Velocity-mode joints integrate the commanded speed into their angle and
    /// report a lagged velocity; a reversed command reverses both.
    #[test]
    fn velocity_mode_drives_plant_angle_and_measured_velocity() {
        let mut plant = SimulatedServoPlant::new(6, [0.0; MAX_JOINTS]);
        let mut cmd = RawServoCommand::default();
        cmd.torque_mask = 1;
        cmd.command_mode[0] = JointCommandMode::Velocity.to_wire();
        cmd.raw_velocity_rad_s[0] = 3.0;

        let start = plant.raw_angles()[0];
        let mut fb = RawServoFeedback::default();
        for _ in 0..20 {
            plant.write_command(&cmd);
            fb = plant.step_plant(0.01);
        }
        assert!(
            plant.raw_angles()[0] > start + 0.1,
            "velocity mode must move the joint in the commanded direction: {start} -> {}",
            plant.raw_angles()[0]
        );
        assert!(
            fb.velocity_rad_s[0] > 2.5 && fb.velocity_rad_s[0] <= 3.0,
            "measured velocity must approach the 3.0 rad/s command, got {}",
            fb.velocity_rad_s[0]
        );

        let midpoint = plant.raw_angles()[0];
        cmd.raw_velocity_rad_s[0] = -2.0;
        for _ in 0..20 {
            plant.write_command(&cmd);
            fb = plant.step_plant(0.01);
        }
        assert!(
            fb.velocity_rad_s[0] < -1.5,
            "measured velocity must follow the reversed command, got {}",
            fb.velocity_rad_s[0]
        );
        assert!(
            plant.raw_angles()[0] < midpoint,
            "negative velocity must reverse joint motion"
        );
    }

    /// The simulator must not own a private actuator model: stepping the shared
    /// `JointPlant` directly with the same wire-decoded command and dt must be
    /// bit-identical to stepping it through `SimulatedServoPlant::step_plant`.
    #[test]
    fn joint_plant_direct_step_matches_simulator_step() {
        const DT: f32 = 0.005_f32;
        let mut plant = SimulatedServoPlant::new(6, [0.0; MAX_JOINTS]);
        let mut cmd = RawServoCommand::default();
        cmd.torque_mask = 1;
        cmd.raw_target_rad[0] = 0.6_f32;
        cmd.torque_nm[0] = 1.2_f32;
        plant.write_command(&cmd);
        plant.harvest_commands();
        let wire = plant.last_cmd;

        // Clone the simulator's exact joint state and feed it the same inputs
        // `step_plant` will use (lag mapping + shared gravity approximation).
        let mut direct = plant.joint_plant().clone();
        direct.set_lag_factor(plant.faults().servo_lag_factor);
        let mut gravity = [0.0_f32; MAX_IMPACT_JOINTS];
        gravity_torque_nm(plant.raw_angles(), &mut gravity);
        for i in 0..plant.joint_count {
            direct.set_gravity_torque(i, gravity[i]);
        }

        let expected = direct.step(
            0,
            DT,
            JointCommandMode::from_wire(wire.command_mode[0]),
            wire.raw_target_rad[0],
            wire.raw_velocity_rad_s[0],
            wire.torque_nm[0],
            false,
        );
        let feedback = plant.step_plant(DT);
        assert_eq!(feedback.raw_angle_rad[0], expected.0, "shared plant angle diverged");
        assert_eq!(feedback.velocity_rad_s[0], expected.1, "shared plant velocity diverged");
        assert_eq!(feedback.current_ma[0], expected.2, "shared plant current diverged");
    }

    /// Host velocity-mode commands reach the kernel as Velocity and remain
    /// bit-exact under replay; the plant change must not touch recorded ticks.
    #[test]
    fn velocity_mode_commands_replay_bit_exact() {
        let _guard = HARNESS_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let cal = [JointCalibration::default(); MAX_JOINTS];
        let mut sim = CopperSimRunner::new(6, &cal, [0.0; MAX_JOINTS]);
        sim.start_recording();

        let mut cmd = HostToMcu {
            seq: 1,
            mode: ControlMode::Drive,
            torque_mask: 1,
            speed_limit_pct: 100,
            ..Default::default()
        };
        cmd.command_mode[0] = JointCommandMode::Velocity.to_wire();
        cmd.velocity_rad_s[0] = 2.0;
        let cmds: Vec<HostToMcu> = (1..=12u32)
            .map(|i| {
                cmd.seq = i;
                cmd
            })
            .collect();
        let _last = sim.step_harness(12, &cmds);

        let log = sim.stop_recording().expect("velocity ticks recorded");
        assert_eq!(log.ticks.len(), 12);
        assert!(
            log.ticks.iter().all(|t| {
                JointCommandMode::from_wire(t.tx_cmd.command_mode[0])
                    == JointCommandMode::Velocity
            }),
            "every recorded kernel command must be velocity mode"
        );
        let check = verify_deterministic_replay(&log);
        assert!(
            check.exact_match,
            "velocity-mode replay must be bit-exact: {check:?}"
        );
        assert_eq!(check.matched_ticks, check.total_ticks);
    }

    /// `SimRobotIo::post_tick` must fire once per control tick and mirror the
    /// firmware LED state machine without altering recorded kernel ticks.
    #[test]
    fn sim_post_tick_evaluates_led_state_once_per_tick() {
        let _guard = HARNESS_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let cal = [JointCalibration::default(); MAX_JOINTS];
        let mut sim = CopperSimRunner::new(6, &cal, [0.0; MAX_JOINTS]);
        const N: u32 = 7;
        for i in 1..=N {
            let cmd = HostToMcu {
                seq: i,
                mode: ControlMode::Teach,
                ..Default::default()
            };
            let _ = sim.step_harness(1, &[cmd]);
        }
        assert_eq!(
            sim.post_tick_calls, N,
            "post_tick must run exactly once per control tick"
        );
        let now_ms = sim.now_ms();
        let expected = rk_fw_core::copper::current_led_mode().is_lit(now_ms);
        assert_eq!(sim.sim_led_on(), expected);
        assert_eq!(sim.sim_led_on, expected);
    }

    /// Wire fault knobs default to off, survive serde round-trips, and may be
    /// absent from older JSON payloads.
    #[test]
    fn sim_fault_injection_wire_knobs_default_and_serde_round_trip() {
        let defaults = SimFaultInjection::default();
        assert!(!defaults.rx_crc_error);
        assert!(!defaults.rx_version_mismatch);
        assert!(!defaults.rx_truncated_frame);
        assert!(!defaults.rx_fragmented_frame);

        let all = SimFaultInjection {
            rx_crc_error: true,
            rx_version_mismatch: true,
            rx_truncated_frame: true,
            rx_fragmented_frame: true,
            ..Default::default()
        };
        let json = serde_json::to_string(&all).expect("serialize faults");
        let back: SimFaultInjection = serde_json::from_str(&json).expect("deserialize faults");
        assert!(back.rx_crc_error && back.rx_version_mismatch);
        assert!(back.rx_truncated_frame && back.rx_fragmented_frame);

        // Simulate an older payload that omits the new knobs: serde defaults apply.
        let mut value = serde_json::to_value(&all).expect("fault value");
        let obj = value.as_object_mut().expect("fault object");
        obj.remove("rx_crc_error");
        obj.remove("rx_version_mismatch");
        obj.remove("rx_truncated_frame");
        obj.remove("rx_fragmented_frame");
        let legacy: SimFaultInjection =
            serde_json::from_value(value).expect("missing wire knobs use defaults");
        assert!(!legacy.rx_crc_error && !legacy.rx_version_mismatch);
        assert!(!legacy.rx_truncated_frame && !legacy.rx_fragmented_frame);

        let _clone = all.clone();
        let _debug = format!("{all:?}");
    }

    /// The realistic small-humanoid profile must carry exactly the P3 sensor
    /// imperfection values, with everything else left at its default.
    #[test]
    fn realistic_small_humanoid_profile_matches_p3_values() {
        let p = SimFaultInjection::realistic_small_humanoid();
        assert_eq!(p.imu_gyro_noise_rad_s, 0.02_f32);
        assert_eq!(p.imu_accel_noise_m_s2, 0.04_f32);
        assert_eq!(p.imu_angle_bias_rad, 0.01_f32);
        assert_eq!(p.imu_gyro_bias_rad_s, 0.01_f32);
        assert_eq!(p.imu_accel_bias_m_s2, 0.05_f32);
        assert_eq!(p.imu_latency_ticks, 2);
        assert_eq!(p.current_noise_pct, 0.03_f32);
        assert_eq!(p.encoder_quantization_lsb_rad, 0.002_f32);
        let defaults = SimFaultInjection::default();
        assert_eq!(p.seed, defaults.seed);
        assert_eq!(p.servo_lag_factor, defaults.servo_lag_factor);
        assert_eq!(p.foot_friction_mu, defaults.foot_friction_mu);
        assert_eq!(p.stalled_joints, defaults.stalled_joints);
        assert!(!p.imu_disconnected && !p.contact_disagreement);
    }

    /// Reset must restore clock, kernel, plant, faults, framer and recorder so a
    /// second identical scripted run is byte-identical to the first, without
    /// breaking the persistent executor (no stale timer wakers, no panic).
    #[test]
    fn reset_experiment_reproduces_identical_telemetry_and_replay() {
        let _guard = HARNESS_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let cal = [JointCalibration::default(); MAX_JOINTS];
        let mut sim = CopperSimRunner::new(6, &cal, [0.0; MAX_JOINTS]);

        let script: Vec<HostToMcu> = (1..=4u32)
            .map(|seq| HostToMcu {
                seq,
                mode: if seq % 2 == 0 {
                    ControlMode::Drive
                } else {
                    ControlMode::Teach
                },
                torque_mask: if seq % 2 == 0 { 0b11 } else { 0 },
                target_rad: [0.05 * seq as f32; MAX_JOINTS],
                ..Default::default()
            })
            .collect();

        let run = |sim: &mut CopperSimRunner| -> (McuToHost, CopperReplayLog) {
            sim.start_recording();
            let mut last = McuToHost::default();
            for cmd in &script {
                last = sim.step_harness(3, &[*cmd]);
            }
            (last, sim.stop_recording().expect("replay log"))
        };

        let (_first_telem, first_log) = run(&mut sim);
        assert_eq!(first_log.ticks.len(), script.len() * 3);
        assert!(verify_deterministic_replay(&first_log).exact_match);

        sim.reset_experiment();
        let (second_telem, second_log) = run(&mut sim);

        // `now_ms` is monotonic (as on hardware), so compare the replay stream
        // with its own time base rebased to zero, not the absolute clock.
        let base = first_log.ticks[0].now_ms;
        let mut expected = first_log.clone();
        for tick in &mut expected.ticks {
            tick.now_ms -= base;
            tick.tx_telem.uptime_ms -= base;
        }
        let mut actual = second_log.clone();
        let second_base = actual.ticks[0].now_ms;
        for tick in &mut actual.ticks {
            tick.now_ms -= second_base;
            tick.tx_telem.uptime_ms -= second_base;
        }
        assert_eq!(expected.ticks, actual.ticks);
        assert_eq!(second_log.version, first_log.version);
        assert_eq!(second_log.joint_count, first_log.joint_count);
        assert_eq!(second_log.initial_raw_pose, first_log.initial_raw_pose);
        assert_eq!(second_log.initial_calibration, first_log.initial_calibration);
        assert_eq!(
            second_telem.uptime_ms - second_base,
            (script.len() * 3 * 5) as u32 - 5
        );
        assert!(verify_deterministic_replay(&second_log).exact_match);

        // The executor must still be alive and receptive after a reset.
        sim.reset_experiment();
        let (third_telem, third_log) = run(&mut sim);
        assert_eq!(third_telem.active_mode, second_telem.active_mode);
        assert_eq!(third_telem.seq_ack, second_telem.seq_ack);
        assert_eq!(third_log.ticks.len(), second_log.ticks.len());
        let third_base = third_log.ticks[0].now_ms;
        let mut third = third_log.clone();
        for tick in &mut third.ticks {
            tick.now_ms -= third_base;
            tick.tx_telem.uptime_ms -= third_base;
        }
        assert_eq!(actual.ticks, third.ticks);
    }

    /// Builds a small log directly through the shared `ControlTask`, so the
    /// legacy-log JSON test does not depend on the mock-clock harness.
    fn sample_log_with_events() -> CopperReplayLog {
        let joint_count = 6;
        let cal = vec![JointCalibration::default(); joint_count];
        let mut task = ControlTask::new(joint_count, &cal);
        let mut ticks = Vec::new();
        for i in 0..4u32 {
            let cmd = HostToMcu {
                seq: i,
                mode: ControlMode::Drive,
                ..Default::default()
            };
            let (_, tick) = task.step_recorded(
                i * 10,
                rk_fw_core::config::CONTROL_DT_SEC,
                Some(&cmd),
                &RawServoFeedback::default(),
                (i as u16, 0),
            );
            ticks.push(tick);
        }
        CopperReplayLog {
            version: 1,
            joint_count,
            initial_raw_pose: [0.0; MAX_JOINTS],
            initial_calibration: cal,
            ticks,
            seed: 0x1234_5678_9ABC_DEF0,
            events: vec![TimedSimEvent {
                tick_index: 2,
                kind: SimEventKind::Shove {
                    pitch_rad_s: 1.5,
                    roll_rad_s: -0.25,
                },
            }],
        }
    }

    #[test]
    fn v1_log_without_seed_or_events_still_deserializes_and_verifies() {
        // Old `.copper` files were written before the seed/events side-channel
        // existed. Removing the two keys from a current log emulates one exactly.
        let current = sample_log_with_events();
        let mut value = serde_json::to_value(&current).expect("log serializes");
        let object = value.as_object_mut().expect("log is a JSON object");
        assert!(object.remove("seed").is_some());
        assert!(object.remove("events").is_some());

        let legacy: CopperReplayLog = serde_json::from_value(value).expect("v1 log parses");
        assert_eq!(legacy.seed, 0);
        assert!(legacy.events.is_empty());
        assert_eq!(legacy.ticks.len(), current.ticks.len());
        let check = verify_deterministic_replay(&legacy);
        assert!(check.exact_match, "legacy log must verify as before");
        assert_eq!(check.matched_ticks, current.ticks.len());
    }

    #[test]
    fn fault_snapshot_round_trips_every_simulator_knob() {
        let mut faults = SimFaultInjection::default();
        faults.drop_heartbeat = true;
        faults.stalled_joints[7] = true;
        faults.voltage_sag = true;
        faults.servo_bus_error = true;
        faults.external_hand_wave = true;
        faults.imu_disconnected = true;
        faults.servo_lag_factor = 0.31;
        faults.seed = 42;
        faults.imu_gyro_noise_rad_s = 0.01;
        faults.imu_accel_noise_m_s2 = 0.02;
        faults.imu_angle_bias_rad = 0.03;
        faults.imu_gyro_bias_rad_s = 0.04;
        faults.imu_accel_bias_m_s2 = 0.05;
        faults.imu_latency_ticks = 3;
        faults.current_noise_pct = 0.06;
        faults.temperature_offset_c = 1.5;
        faults.encoder_quantization_lsb_rad = 0.001;
        faults.foot_friction_mu = 0.42;
        faults.contact_disagreement = true;
        faults.rx_crc_error = true;
        faults.rx_version_mismatch = true;
        faults.rx_truncated_frame = true;
        faults.rx_fragmented_frame = true;

        let snapshot = fault_snapshot(&faults);
        let restored = faults_from_snapshot(&snapshot);
        assert_eq!(fault_snapshot(&restored), snapshot);
    }

    /// The shove@40 + fault@80 script must replay identically from a clean
    /// reset, with events captured at their exact tick indices.
    #[test]
    fn scripted_experiment_replays_bit_identically_after_reset() {
        let _guard = HARNESS_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let cal = [JointCalibration::default(); MAX_JOINTS];
        let mut sim = CopperSimRunner::new(6, &cal, [0.0; MAX_JOINTS]);

        const SEED: u64 = 0xDEAD_BEEF_0000_0001;
        let mut base_faults = SimFaultInjection::default();
        base_faults.seed = SEED;
        let mut toggled = base_faults.clone();
        toggled.voltage_sag = true;

        let packets: Vec<HostToMcu> = (0..100u32)
            .map(|seq| HostToMcu {
                seq,
                mode: ControlMode::Drive,
                torque_mask: 0b11,
                ..Default::default()
            })
            .collect();
        let script = ExperimentScript {
            seed: SEED,
            events: vec![
                TimedSimEvent {
                    tick_index: 40,
                    kind: SimEventKind::Shove {
                        pitch_rad_s: 1.6,
                        roll_rad_s: -0.7,
                    },
                },
                TimedSimEvent {
                    tick_index: 80,
                    kind: SimEventKind::SetFaults(fault_snapshot(&toggled)),
                },
            ],
            packets,
            max_ticks: 100,
        };

        let first = sim.run_script(&script);
        assert_eq!(first.ticks.len(), 100);
        assert_eq!(first.seed, SEED);
        assert!(first.events.iter().any(|event| {
            event.tick_index == 40
                && matches!(event.kind, SimEventKind::Shove { pitch_rad_s, .. } if pitch_rad_s == 1.6)
        }), "shove@40 must be captured");
        assert!(first.events.iter().any(|event| {
            event.tick_index == 80
                && matches!(event.kind, SimEventKind::SetFaults(ref faults) if faults.voltage_sag)
        }), "fault toggle@80 must be captured");

        let first_check = verify_deterministic_replay(&first);
        assert!(first_check.exact_match);
        assert_eq!(first_check.matched_ticks, 100);

        sim.reset_experiment();
        let second = sim.run_script(&script);
        assert_eq!(second.ticks.len(), first.ticks.len());
        assert_eq!(second.seed, first.seed);
        assert_eq!(second.events, first.events, "event stream must repeat exactly");

        // `now_ms`/`uptime_ms` advance monotonically with the mock clock; strip
        // their origin before comparing the kernel-relevant tick stream.
        let rebase = |log: &CopperReplayLog| -> CopperReplayLog {
            let mut rebased = log.clone();
            let base = log.ticks[0].now_ms;
            for tick in &mut rebased.ticks {
                tick.now_ms -= base;
                tick.tx_telem.uptime_ms -= base;
            }
            rebased
        };
        let first_rebased = rebase(&first);
        let second_rebased = rebase(&second);
        for (index, (a, b)) in first_rebased
            .ticks
            .iter()
            .zip(second_rebased.ticks.iter())
            .enumerate()
        {
            assert_eq!(a, b, "tick {index} diverged between runs");
        }

        let second_check = verify_deterministic_replay(&second);
        assert!(second_check.exact_match);
        assert_eq!(second_check.matched_ticks, first_check.matched_ticks);
    }

    // --- Group 2: sensors, constrained contacts, impact path -----------------

    /// A shove must reach the kernel's impact detector through the shared plant's
    /// measured joint current (not only through the torso IMU). The kernel treats
    /// impact as telemetry: fall/ukemi retains priority, so this test also checks
    /// that an impact alone does not latch a fall or change the control mode.
    #[test]
    fn shove_while_guarding_trips_kernel_impact_detector() {
        let _guard = HARNESS_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
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
        for i in 1..=60 {
            cmd.seq = i;
            let _ = sim.tick(0.01, &cmd);
        }

        sim.apply_push_impulse(1.6, -1.1);
        let mut saw_impact = false;
        let mut peak_magnitude_nm = 0.0_f32;
        let mut last = McuToHost::default();
        for i in 61..=100 {
            cmd.seq = i;
            last = sim.tick(0.01, &cmd);
            if last.balance.impact_detected {
                saw_impact = true;
                peak_magnitude_nm = peak_magnitude_nm.max(last.balance.impact_magnitude_nm);
            }
        }
        assert!(
            saw_impact,
            "shove must produce a joint current spike the kernel flags as impact"
        );
        assert!(
            peak_magnitude_nm > rk_control::impact::DEFAULT_CURRENT_SPIKE_MA
                / rk_control::impact::DEFAULT_MA_PER_NM,
            "impact magnitude must reflect a real current jump, got {peak_magnitude_nm}"
        );
        assert_ne!(
            last.estimator.valid_mask & estimator_valid::IMPACT,
            0,
            "impact telemetry must be marked valid"
        );
        assert_eq!(last.active_mode, ControlMode::Drive, "impact alone must not abort Drive");
        assert!(
            !last.faults.contains(FaultFlags::TILT_FALL_CUTOFF),
            "impact must never latch the fall cutoff (fall handling keeps priority)"
        );
    }

    /// Same seed + command stream must produce bit-identical sensor samples in
    /// two independent sessions, while a different seed gives different samples.
    #[test]
    fn seeded_sensor_imperfection_is_repeatable() {
        let mut config = SimFaultInjection {
            seed: 0x1234_5678_9ABC_DEF0,
            imu_gyro_noise_rad_s: 0.02,
            imu_accel_noise_m_s2: 0.05,
            imu_angle_bias_rad: 0.01,
            imu_gyro_bias_rad_s: 0.02,
            imu_accel_bias_m_s2: 0.03,
            imu_latency_ticks: 3,
            current_noise_pct: 0.04,
            temperature_offset_c: 2.0,
            encoder_quantization_lsb_rad: 0.01,
            ..Default::default()
        };
        let run = |cfg: &SimFaultInjection| -> Vec<RawServoFeedback> {
            let mut plant = SimulatedServoPlant::new(6, [0.0; MAX_JOINTS]);
            *plant.faults_mut() = cfg.clone();
            let mut out = Vec::new();
            for step in 0..40u32 {
                let mut cmd = RawServoCommand::default();
                cmd.torque_mask = 0b11_1111;
                cmd.raw_target_rad[0] = (step as f32 * 0.03_f32).sin() * 0.4_f32;
                plant.write_command(&cmd);
                out.push(plant.step_plant(0.005_f32));
            }
            out
        };

        let first = run(&config);
        let second = run(&config);
        assert_eq!(first.len(), second.len());
        for (tick, (a, b)) in first.iter().zip(second.iter()).enumerate() {
            assert_eq!(
                a.raw_angle_rad, b.raw_angle_rad,
                "tick {tick}: encoder samples diverged"
            );
            assert_eq!(
                a.current_ma, b.current_ma,
                "tick {tick}: current samples diverged"
            );
            assert_eq!(a.temp_c, b.temp_c, "tick {tick}: temperature samples diverged");
            assert_eq!(a.imu, b.imu, "tick {tick}: IMU samples diverged");
        }

        // The enabled knobs really perturb the samples, and a different seed
        // produces a different (but still deterministic) stream.
        config.seed = 0xDEAD_BEEF_0000_0001;
        let other_seed = run(&config);
        assert_ne!(first[39].imu, other_seed[39].imu, "seed must change the IMU stream");
        assert_ne!(
            first[39].current_ma, other_seed[39].current_ma,
            "seed must change the current-sense stream"
        );
    }

    /// Biases persist across ticks; latency publishes a delayed copy of the
    /// true sample and eventually settles at exactly `latency_ticks` of delay.
    #[test]
    fn imu_bias_persists_and_latency_delays_true_samples() {
        let mut plant = SimulatedServoPlant::new(6, [0.0; MAX_JOINTS]);
        {
            let faults = plant.faults_mut();
            faults.seed = 77;
            faults.imu_angle_bias_rad = 0.05;
            faults.imu_gyro_bias_rad_s = 0.01;
        }
        plant.apply_push_impulse(0.8, 0.0);
        for _ in 0..120 {
            let _ = plant.step_plant(0.005_f32);
        }
        let biased = plant.step_plant(0.005_f32);
        assert!(
            (biased.imu.pitch_rad - plant.torso_pitch_rad).abs() > 1.0e-3_f32,
            "configured attitude bias must remain visible after many ticks"
        );
        assert!(
            (biased.imu.pitch_rate_rad_s - plant.pitch_rate_rad_s).abs() > 1.0e-4_f32,
            "configured gyro bias must persist"
        );

        let mut delayed = SimulatedServoPlant::new(6, [0.0; MAX_JOINTS]);
        delayed.faults_mut().imu_latency_ticks = 3;
        delayed.apply_push_impulse(0.8, 0.0);
        let mut truth = Vec::new();
        for _ in 0..30 {
            let _ = delayed.step_plant(0.005_f32);
            truth.push(delayed.torso_pitch_rad);
        }
        let delayed_sample = delayed.step_plant(0.005_f32);
        assert_eq!(
            delayed_sample.imu.pitch_rad, truth[truth.len() - 3],
            "latency must publish the sample from exactly 3 ticks earlier"
        );
    }

    /// Quantization, current noise and temperature offset change only the
    /// published feedback; the physical plant state stays full precision so
    /// replay and direct-JointPlant parity are unaffected.
    #[test]
    fn encoder_current_and_temperature_imperfections_are_publish_only() {
        const LSB: f32 = 0.02_f32;
        let mut plant = SimulatedServoPlant::new(6, [0.0; MAX_JOINTS]);
        {
            let faults = plant.faults_mut();
            faults.seed = 9;
            faults.current_noise_pct = 0.10;
            faults.temperature_offset_c = 3.0;
            faults.encoder_quantization_lsb_rad = LSB;
        }
        let mut cmd = RawServoCommand::default();
        cmd.torque_mask = 1;
        cmd.raw_target_rad[0] = 0.37_f32;
        let mut saw_quantized = false;
        let mut saw_current_noise = false;
        let mut saw_temp_offset = false;
        for _ in 0..40 {
            plant.write_command(&cmd);
            let fb = plant.step_plant(0.005_f32);
            let angle = fb.raw_angle_rad[0];
            if (angle / LSB - (angle / LSB).round()).abs() < 1.0e-6_f32 {
                saw_quantized = true;
            }
            if fb.current_ma[0] != plant.joint_plant().current_ma(0) {
                saw_current_noise = true;
            }
            if fb.temp_c[0] >= plant.joint_plant().temperature_c(0).round() as u8 + 2 {
                saw_temp_offset = true;
            }
            // The physical angle is never quantized.
            assert_ne!(
                plant.raw_angles()[0], angle,
                "published angle must be quantized, plant state must not"
            );
        }
        assert!(saw_quantized, "encoder LSB must be visible on published angles");
        assert!(saw_current_noise, "current noise must be visible on published current");
        assert!(saw_temp_offset, "temperature offset must be visible on published temps");
    }

    /// Ground reaction must be normalized by the number of loaded soles (one
    /// sole carries full body weight, never a half-weight bias), and slip
    /// distance must integrate across steps and clear on reset.
    #[test]
    fn plant_grf_normalizes_to_body_weight_and_accumulates_slip() {
        let mut plant = SimulatedServoPlant::new(21, [0.0; MAX_JOINTS]);
        let mut cmd = RawServoCommand::default();
        cmd.torque_mask = (1u32 << 21) - 1;
        plant.write_command(&cmd);
        for _ in 0..30 {
            let _ = plant.step_plant(0.005_f32);
        }
        let feet = *plant.foot_contacts();
        let total = feet[0].grf_n + feet[1].grf_n;
        assert!(
            (total - plant.body_weight_n()).abs() < 0.01_f32 * plant.body_weight_n(),
            "loaded-foot GRF total {total:.3} N must equal body weight {:.3} N",
            plant.body_weight_n()
        );
        for (foot, contact) in feet.iter().enumerate() {
            if !contact.contact {
                assert_eq!(contact.grf_n, 0.0_f32, "lifted foot {foot} must carry no GRF");
            }
        }
        assert!(
            feet.iter().any(|c| c.slip_distance_m > 0.0_f32),
            "loaded soles must accumulate slip distance across steps, got {feet:?}"
        );

        plant.reset_upright();
        assert!(
            plant
                .foot_contacts()
                .iter()
                .all(|c| c.slip_distance_m == 0.0_f32),
            "reset must clear per-foot slip accumulation"
        );
    }

    /// A shove must move the CoM in the world frame and load the leg that
    /// actually carries ground reaction, not the controller's scripted phase.
    #[test]
    fn push_impulse_kicks_com_and_selects_loaded_stance_leg() {
        let mut plant = SimulatedServoPlant::new(21, [0.0; MAX_JOINTS]);
        let mut cmd = RawServoCommand::default();
        cmd.torque_mask = (1u32 << 21) - 1;
        plant.write_command(&cmd);
        let before = plant.world_velocity_m_s()[2];
        plant.apply_push_impulse(1.0_f32, 0.0_f32);
        let _ = plant.step_plant(0.005_f32);
        let after = plant.world_velocity_m_s()[2];
        assert!(
            (after - before - PUSH_COM_IMPULSE_SCALE).abs() < 0.02_f32,
            "forward CoM kick should be ~{PUSH_COM_IMPULSE_SCALE} m/s, got {:.4}",
            after - before
        );

        // With only the right sole loaded, the right leg joints get the
        // velocity impulse; mirroring the load must mirror the impulse.
        let mut right_loaded = SimulatedServoPlant::new(21, [0.0; MAX_JOINTS]);
        {
            let right = right_loaded.foot_contact_mut(false);
            right.contact = true;
            right.grf_n = 10.0_f32;
        }
        right_loaded.apply_push_impulse(1.0_f32, 0.0_f32);
        let sum_leg = |p: &SimulatedServoPlant, start: usize| -> f32 {
            (start..start + 6)
                .map(|j| p.joint_plant().velocity(j).abs())
                .sum()
        };
        assert!(
            sum_leg(&right_loaded, 9) > sum_leg(&right_loaded, 15),
            "right-loaded shove must kick the right stance leg"
        );

        let mut left_loaded = SimulatedServoPlant::new(21, [0.0; MAX_JOINTS]);
        {
            let left = left_loaded.foot_contact_mut(true);
            left.contact = true;
            left.grf_n = 10.0_f32;
        }
        left_loaded.apply_push_impulse(1.0_f32, 0.0_f32);
        assert!(
            sum_leg(&left_loaded, 15) > sum_leg(&left_loaded, 9),
            "left-loaded shove must kick the left stance leg"
        );
    }

    /// Unilateral GRF and support-polygon CoP: a lifted/unloaded sole carries no
    /// force and contributes no support point, and the CoP stays on the segment.
    #[test]
    fn contact_unilateral_grf_and_cop_support_polygon() {
        assert_eq!(unilateral_grf(false, 100.0_f32, 20.0_f32), 0.0_f32);
        assert_eq!(unilateral_grf(true, -5.0_f32, 20.0_f32), 0.0_f32);
        assert_eq!(unilateral_grf(true, 999.0_f32, 20.0_f32), 20.0_f32);

        let right = FootContact {
            contact: true,
            point_m: [-0.1_f32, 0.0_f32],
            grf_n: 10.0_f32,
            ..Default::default()
        };
        let lifted = FootContact {
            contact: false,
            point_m: [5.0_f32, 5.0_f32],
            grf_n: 0.0_f32,
            ..Default::default()
        };
        assert_eq!(
            clamp_cop_to_support([4.0_f32, 4.0_f32], &[right, lifted]),
            right.point_m,
            "unloaded sole must not pull the CoP toward it"
        );

        let left = FootContact {
            contact: true,
            point_m: [0.1_f32, 0.2_f32],
            grf_n: 10.0_f32,
            ..Default::default()
        };
        let clamped = clamp_cop_to_support([9.0_f32, 9.0_f32], &[right, left]);
        assert!(
            (clamped[0] - left.point_m[0]).abs() < 1.0e-6_f32
                && (clamped[1] - left.point_m[1]).abs() < 1.0e-6_f32,
            "CoP must clamp to the loaded-foot segment endpoint, got {clamped:?}"
        );
    }

    /// Slip: above the friction limit the pinned foot loses GRF (and hence
    /// support influence); below it the contact stays fully loaded.
    #[test]
    fn contact_slip_reduces_loaded_point_when_friction_is_exceeded() {
        let make_foot = |slip_m_s: f32| FootContact {
            contact: true,
            grf_n: 10.0_f32,
            slip_m_s,
            ..Default::default()
        };
        let mut quiet = make_foot(0.1_f32);
        apply_friction_slip(&mut quiet, 1.5_f32, 0.7_f32);
        assert_eq!(quiet.grf_n, 10.0_f32, "sub-threshold sole must stay pinned");

        let mut sliding = make_foot(2.0_f32);
        apply_friction_slip(&mut sliding, 1.5_f32, 0.7_f32);
        assert!(
            sliding.grf_n > 0.0_f32 && sliding.grf_n < 10.0_f32,
            "slip must reduce but never unload the planted point, got {}",
            sliding.grf_n
        );
        assert_eq!(sliding.slip_m_s, 2.0_f32, "reported slip must stay the true speed");
    }

    /// Contact-disagreement fault: plant contacts no longer follow the
    /// controller's phase-scripted support foot, while telemetry reports the
    /// plant's real contacts and the estimator remains stable.
    #[test]
    fn contact_disagreement_desynchronizes_plant_and_controller() {
        let _guard = HARNESS_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let cal = [JointCalibration {
            min_rad: -2.2,
            max_rad: 2.2,
            ..Default::default()
        }; 21];
        let mut sim = CopperSimRunner::new(21, &cal, [0.0; MAX_JOINTS]);
        sim.plant.faults_mut().contact_disagreement = true;
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
        // Warm up until the Drive-mode torque stage energizes the legs; before
        // that the plant intentionally drops all contacts on every tick.
        for i in 1..=60 {
            cmd.seq = i;
            let _ = sim.tick(0.01, &cmd);
        }
        let mut controller_supports = [false; 2];
        let mut last = McuToHost::default();
        for i in 61..=360 {
            cmd.seq = i;
            last = sim.tick(0.01, &cmd);
            assert_eq!(
                last.estimator.contact_mask,
                sim.plant.contact_mask(),
                "telemetry must report the plant's real contact mask"
            );
            assert_eq!(last.estimator.contact_mask, 0b11, "fault must report phantom double support");
            controller_supports[(last.balance.support_foot & 1) as usize] = true;
            assert!(
                !last.faults.contains(FaultFlags::TILT_FALL_CUTOFF),
                "contact disagreement must not destabilize the estimator at tick {i}"
            );
        }
        assert!(
            controller_supports[0] && controller_supports[1],
            "controller must still script alternating support: {:?}",
            controller_supports
        );
        assert_eq!(last.active_mode, ControlMode::Drive);
    }
}
