#![cfg_attr(not(feature = "std"), no_std)]
//! Layer 0 — Shared Wire Protocol (`rk-proto`).
//!
//! Strictly `#![no_std]` with fixed-capacity `f32` arrays and `postcard` COBS framing
//! shared identically across RP2350 MCU, Native Desktop, and Browser WASM.

use bitflags::bitflags;
use serde::{Deserialize, Serialize};

pub const MAX_JOINTS: usize = 24;
pub const WATCHDOG_TIMEOUT_MS: u32 = 100;
pub const MAX_COBS_FRAME_BYTES: usize = 512;
pub const MODEL_CHUNK_BYTES: usize = 256;
pub const MAX_MODEL_NAME_BYTES: usize = 32;
pub const MAX_MODEL_COBS_FRAME_BYTES: usize = 384;
pub const MODEL_SCHEMA_VERSION: u8 = 1;
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

/// Wire-encoded fall orientation identifiers.
///
/// `FallOrientation` is serialized in `BalanceTelemetry.fall_orientation` as the `u8`
/// value defined here (matching the `#[repr(u8)]` discriminant of [`FallOrientation`]).
/// A dedicated `u8` field keeps the telemetry layout allocation-free and `no_std`
/// friendly while allowing the host to reconstruct the enum with
/// [`FallOrientation::from_wire`].
pub mod fall_orientation {
    pub const UNKNOWN: u8 = 0;
    pub const PRONE: u8 = 1;
    pub const SUPINE: u8 = 2;
    pub const SIDE_LEFT: u8 = 3;
    pub const SIDE_RIGHT: u8 = 4;

    pub const fn label(code: u8) -> &'static str {
        match code {
            PRONE => "PRONE",
            SUPINE => "SUPINE",
            SIDE_LEFT => "SIDE_LEFT",
            SIDE_RIGHT => "SIDE_RIGHT",
            _ => "UNKNOWN",
        }
    }
}

/// Coarse fall orientation derived from the torso gravity vector.
///
/// The discriminants match [`fall_orientation`] wire codes. `Unknown` is the
/// default for unavailable or invalid orientation samples.
#[repr(u8)]
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum FallOrientation {
    #[default]
    Unknown = fall_orientation::UNKNOWN,
    Prone = fall_orientation::PRONE,
    Supine = fall_orientation::SUPINE,
    SideLeft = fall_orientation::SIDE_LEFT,
    SideRight = fall_orientation::SIDE_RIGHT,
}

impl FallOrientation {
    pub const fn to_wire(self) -> u8 {
        self as u8
    }

    pub const fn from_wire(code: u8) -> Self {
        match code {
            fall_orientation::PRONE => Self::Prone,
            fall_orientation::SUPINE => Self::Supine,
            fall_orientation::SIDE_LEFT => Self::SideLeft,
            fall_orientation::SIDE_RIGHT => Self::SideRight,
            _ => Self::Unknown,
        }
    }

    pub const fn label(self) -> &'static str {
        fall_orientation::label(self.to_wire())
    }
}

/// Gravity magnitude below which a sample is considered unusable for classification.
const CLASSIFY_MIN_GRAVITY_MAG: f32 = 0.5_f32;
/// Horizontal gravity magnitude below which the torso is effectively upright.
const CLASSIFY_MIN_HORIZONTAL_MAG: f32 = 0.05_f32;

/// Classifies a fall orientation from an IMU sample.
///
/// Pure and `#![no_std]` compatible. The gravity vector is interpreted as the
/// world-frame down vector when it is nearly vertical (which is what the BNO055
/// gravity register and the simulator publish); in that case it is rotated into
/// the body frame using the sample's Euler attitude so that the horizontal
/// components align with the torso axes. If the gravity vector already carries a
/// significant horizontal component it is treated as body-frame directly. This
/// makes the classifier robust to both representations without guessing.
///
/// Body-frame convention (matching `KinematicRig::compute_link_transforms_with_root`):
/// `+X` is the robot's right, `+Z` is forward, `+Y` is up. The dominant horizontal
/// component selects the fall direction:
/// - `+Z` gravity component (`pitch > 0`, falling forward) -> [`FallOrientation::Prone`]
/// - `-Z` gravity component (`pitch < 0`, falling backward) -> [`FallOrientation::Supine`]
/// - `+X` gravity component (falling toward the right) -> [`FallOrientation::SideRight`]
/// - `-X` gravity component (falling toward the left) -> [`FallOrientation::SideLeft`]
///
/// Returns [`FallOrientation::Unknown`] when the IMU is unavailable, gravity is
/// non-finite or near zero, or the torso is effectively upright.
pub fn classify_fall(imu: &ImuState) -> FallOrientation {
    if !imu.available {
        return FallOrientation::Unknown;
    }
    let g = imu.gravity;
    if !g[0].is_finite() || !g[1].is_finite() || !g[2].is_finite() {
        return FallOrientation::Unknown;
    }
    let mag_sq = g[0] * g[0] + g[1] * g[1] + g[2] * g[2];
    if !mag_sq.is_finite() || mag_sq < CLASSIFY_MIN_GRAVITY_MAG * CLASSIFY_MIN_GRAVITY_MAG {
        return FallOrientation::Unknown;
    }

    // Body-frame horizontal components. If the sample already looks like a
    // body-frame gravity vector (non-trivial horizontal component) use it as-is;
    // otherwise rotate the world-frame vector into the body frame.
    let raw_h = g[0] * g[0] + g[2] * g[2];
    let (hx, hz) = if raw_h > CLASSIFY_MIN_HORIZONTAL_MAG * CLASSIFY_MIN_HORIZONTAL_MAG {
        (g[0], g[2])
    } else {
        rotate_world_gravity_to_body(g, imu.pitch_rad, imu.roll_rad, imu.yaw_rad)
    };

    if !hx.is_finite() || !hz.is_finite() {
        return FallOrientation::Unknown;
    }
    if hx.abs() < CLASSIFY_MIN_HORIZONTAL_MAG && hz.abs() < CLASSIFY_MIN_HORIZONTAL_MAG {
        return FallOrientation::Unknown;
    }

    if hz.abs() >= hx.abs() {
        // Forward axis: +Z gravity component means the torso tipped forward.
        if hz >= 0.0 {
            FallOrientation::Prone
        } else {
            FallOrientation::Supine
        }
    } else if hx >= 0.0 {
        FallOrientation::SideRight
    } else {
        FallOrientation::SideLeft
    }
}

/// Rotates a world-frame vector into the torso body frame using the same Euler
/// axis order as `KinematicRig` (`yaw` about world Y, `pitch` about X, `-roll` about Z).
#[inline]
fn rotate_world_gravity_to_body(g: [f32; 3], pitch: f32, roll: f32, yaw: f32) -> (f32, f32) {
    let (sp, cp) = sin_cos_f32(pitch);
    let (sr, cr) = sin_cos_f32(roll);
    let (sy, cy) = sin_cos_f32(yaw);

    // R = R_y(yaw) * R_x(pitch) * R_z(-roll); body = R^T * world.
    // Apply R_z(roll), then R_x(-pitch), then R_y(-yaw).
    let x1 = g[0] * cr - g[1] * sr;
    let y1 = g[0] * sr + g[1] * cr;
    let z1 = g[2];

    let y2 = y1 * cp + z1 * sp;
    let z2 = -y1 * sp + z1 * cp;

    let x3 = x1 * cy - z2 * sy;
    let z3 = x1 * sy + z2 * cy;

    let _ = y2; // vertical component is not used by classification
    (x3, z3)
}

/// Small deterministic sine/cosine pair for the classifier (no external `libm` in proto).
#[inline]
fn sin_cos_f32(x: f32) -> (f32, f32) {
    if !x.is_finite() {
        return (0.0, 1.0);
    }
    let two_pi = core::f32::consts::TAU;
    let mut a = x % two_pi;
    if a > core::f32::consts::PI {
        a -= two_pi;
    } else if a < -core::f32::consts::PI {
        a += two_pi;
    }
    let a2 = a * a;
    let s = a * (1.0
        + a2 * (-0.166_666_67 + a2 * (0.008_333_334 + a2 * (-0.000_198_412_7 + a2 * 0.000_002_755_732))));
    let c = 1.0
        + a2 * (-0.5 + a2 * (0.041_666_664 + a2 * (-0.001_388_888_9 + a2 * 0.000_024_801_588)));
    (s, c)
}

bitflags! {
    #[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq, Default)]
    #[serde(transparent)]
    pub struct FaultFlags: u16 {
        const WATCHDOG_TIMEOUT  = 1 << 0;
        const ESTOP_ACTIVE      = 1 << 1;
        const OVER_CURRENT      = 1 << 2;
        const OVER_TEMP         = 1 << 3;
        const UNDER_VOLTAGE     = 1 << 4;
        const SERVO_BUS_ERROR   = 1 << 5;
        const LIMIT_CLAMPED     = 1 << 6;
        const TILT_FALL_CUTOFF  = 1 << 7;
    }
}

/// High-level locomotion & upper-body vertical balance command streamed to the MCU.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub struct WalkCmd {
    pub walk_enabled: bool,
    pub balance_enabled: bool,
    /// Normalized forward/back velocity command in `[-1.0, 1.0]`.
    pub vx: f32,
    /// Normalized lateral strafe velocity command in `[-1.0, 1.0]`.
    pub vy: f32,
    /// Normalized yaw turn rate command in `[-1.0, 1.0]`.
    pub yaw_rate: f32,
    /// Combat crouch depth in `[0.0, 1.0]`.
    pub stance_crouch: f32,
    /// Swing foot lift scale in `[0.1, 1.0]`.
    pub step_height: f32,
    /// Gait oscillator frequency in Hz (`[0.6, 2.8]`).
    pub cadence_hz: f32,
}

impl Default for WalkCmd {
    fn default() -> Self {
        Self {
            walk_enabled: false,
            balance_enabled: true,
            vx: 0.0,
            vy: 0.0,
            yaw_rate: 0.0,
            stance_crouch: 0.30,
            step_height: 0.55,
            cadence_hz: 1.55,
        }
    }
}

/// MCU-resident UVC (Upper Body Vertical Control) gains, deadband & safety cutoff.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub struct BalanceGains {
    /// UVC sagittal tilt integration gain per 10 ms reference tick (ai2001 default `0.25`).
    pub uvc_gain_pitch: f32,
    /// UVC lateral tilt integration gain per 10 ms reference tick (ai2001 default `0.25`).
    pub uvc_gain_roll: f32,
    /// Radial IMU tilt deadband in radians before triggering catch-step (ai2001 default `0.033` rad ~ 1.9 deg).
    pub tilt_deadband_rad: f32,
    /// Direct ankle-pitch gyro rate damping gain (`rad / (rad/s)`).
    pub gyro_damp_pitch: f32,
    /// Direct ankle-roll gyro rate damping gain (`rad / (rad/s)`).
    pub gyro_damp_roll: f32,
    /// Upper-body arm counter-spread gain proportional to lateral step offset (`[0.0, 1.5]`).
    pub arm_assist: f32,
    /// Protective fall-tuck & torque cutoff threshold in radians (default ~37 deg = `0.65` rad).
    pub fall_cutoff_rad: f32,
}

impl Default for BalanceGains {
    fn default() -> Self {
        Self {
            uvc_gain_pitch: 0.25,
            uvc_gain_roll: 0.25,
            tilt_deadband_rad: 0.033,
            gyro_damp_pitch: 0.08,
            gyro_damp_roll: 0.10,
            arm_assist: 0.65,
            fall_cutoff_rad: 0.65,
        }
    }
}

/// Compact full-orientation torso IMU attitude, gyro rate & gravity snapshot (`f32`, `#![no_std]`).
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Default)]
pub struct ImuState {
    /// True when a hardware or simulated IMU is present and reporting live orientation.
    pub available: bool,
    pub pitch_rad: f32,
    pub roll_rad: f32,
    /// Heading / yaw angle in radians.
    pub yaw_rad: f32,
    pub pitch_rate_rad_s: f32,
    pub roll_rate_rad_s: f32,
    /// Yaw angular rate in radians per second.
    pub yaw_rate_rad_s: f32,
    /// World-frame gravity vector in m/s² (`[x, y, z]`); upright nominal `[0.0, -9.81, 0.0]`.
    pub gravity: [f32; 3],
}

/// Wire-encoded UVC state machine phase identifiers (aligned with ai2001 modes 710..770 + FallTuck).
pub mod uvc_phase {
    pub const IDLE: u8 = 0;
    pub const CROUCH_INIT: u8 = 1;
    pub const CALIBRATE_IMU: u8 = 2;
    pub const READY_WATCH: u8 = 3;
    pub const ACTIVE_CATCH_STEP: u8 = 4;
    pub const POST_STEP_SETTLE: u8 = 5;
    pub const RETURN_TO_NEUTRAL: u8 = 6;
    pub const RECOVERY_SETTLE: u8 = 7;
    pub const FALL_TUCK: u8 = 8;

    pub const fn label(phase: u8) -> &'static str {
        match phase {
            IDLE => "IDLE",
            CROUCH_INIT => "INIT",
            CALIBRATE_IMU => "CAL",
            READY_WATCH => "READY",
            ACTIVE_CATCH_STEP => "STEP",
            POST_STEP_SETTLE => "SETTLE",
            RETURN_TO_NEUTRAL => "RETURN",
            RECOVERY_SETTLE => "RECOVER",
            FALL_TUCK => "TUCK",
            _ => "UNKNOWN",
        }
    }
}

/// Live MCU walking & UVC upper-body vertical balance telemetry returned in `McuToHost`.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Default)]
pub struct BalanceTelemetry {
    pub gait_phase_rad: f32,
    pub pitch_corr_rad: f32,
    pub roll_corr_rad: f32,
    pub balance_active: bool,
    pub walk_active: bool,
    /// Active UVC state machine phase (`uvc_phase::*`).
    pub uvc_phase: u8,
    /// Current support foot (`0 = Right`, `1 = Left`).
    pub support_foot: u8,
    /// Support foot sagittal UVC offset (`dxi`, meters; positive = foot forward under forward lean).
    pub foot_dx_m: f32,
    /// Signed lateral UVC foot offset (meters; positive = stepped right, negative = stepped left).
    pub foot_dy_m: f32,
    /// Active UVC pelvis-to-ankle height (`auto_h`, meters).
    pub hip_height_m: f32,
    /// Fall orientation captured when `TILT_FALL_CUTOFF` latched
    /// (`fall_orientation::*` wire code; `UNKNOWN` when no fall has been classified).
    pub fall_orientation: u8,
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
    SetBalanceGains(BalanceGains),
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
    pub walk: WalkCmd,
    pub config: ConfigCmd,
}

impl Default for HostToMcu {
    fn default() -> Self {
        Self {
            seq: 0,
            mode: ControlMode::Teach,
            torque_mask: 0,
            speed_limit_pct: 55,
            target_rad: [0.0_f32; MAX_JOINTS],
            walk: WalkCmd::default(),
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
    pub imu: ImuState,
    pub balance: BalanceTelemetry,
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
            imu: ImuState::default(),
            balance: BalanceTelemetry::default(),
        }
    }
}

// ---------------------------------------------------------------------------
// MCU Onboard Kinematics / URDF Storage Protocol (no_std, zero allocation)
// ---------------------------------------------------------------------------

/// Standard IEEE 802.3 CRC32 evaluable at compile time (`const fn`) for MCU flash blobs.
pub const fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFF_u32;
    let mut i = 0usize;
    while i < bytes.len() {
        crc ^= bytes[i] as u32;
        let mut bit = 0;
        while bit < 8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320_u32 & mask);
            bit += 1;
        }
        i += 1;
    }
    !crc
}

/// Format of the robot kinematics model stored in MCU flash.
#[repr(u8)]
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ModelFormat {
    /// Raw UTF-8 URDF XML embedded directly in MCU flash (zero MCU CPU overhead).
    #[default]
    UrdfXml = 1,
    /// Zlib/deflate-compressed URDF XML (~5x smaller in MCU flash, decompressed by Rocky Studio).
    UrdfDeflate = 2,
    /// Compact binary kinematics descriptor for ultra-constrained flash targets.
    CompactKinematics = 3,
}

impl ModelFormat {
    pub const fn label(self) -> &'static str {
        match self {
            Self::UrdfXml => "URDF XML (Raw Flash)",
            Self::UrdfDeflate => "URDF XML (Deflate)",
            Self::CompactKinematics => "Compact Binary Kinematics",
        }
    }
}

/// Fixed-size metadata header describing the kinematics model stored in MCU flash.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub struct ModelInfo {
    pub format: ModelFormat,
    pub total_bytes: u32,
    pub crc32: u32,
    pub joint_count: u8,
    pub schema_version: u8,
    pub name_len: u8,
    pub name_utf8: [u8; MAX_MODEL_NAME_BYTES],
}

impl Default for ModelInfo {
    fn default() -> Self {
        Self {
            format: ModelFormat::UrdfXml,
            total_bytes: 0,
            crc32: 0,
            joint_count: 0,
            schema_version: MODEL_SCHEMA_VERSION,
            name_len: 0,
            name_utf8: [0u8; MAX_MODEL_NAME_BYTES],
        }
    }
}

impl ModelInfo {
    /// Builds a compile-time `ModelInfo` descriptor directly from a flash-resident byte slice.
    pub const fn from_static(
        name: &str,
        format: ModelFormat,
        blob: &[u8],
        joint_count: u8,
    ) -> Self {
        let name_bytes = name.as_bytes();
        let copy_len = if name_bytes.len() < MAX_MODEL_NAME_BYTES {
            name_bytes.len()
        } else {
            MAX_MODEL_NAME_BYTES
        };
        let mut buf = [0u8; MAX_MODEL_NAME_BYTES];
        let mut i = 0usize;
        while i < copy_len {
            buf[i] = name_bytes[i];
            i += 1;
        }
        Self {
            format,
            total_bytes: blob.len() as u32,
            crc32: crc32(blob),
            joint_count,
            schema_version: MODEL_SCHEMA_VERSION,
            name_len: copy_len as u8,
            name_utf8: buf,
        }
    }

    pub fn name_str(&self) -> &str {
        let len = (self.name_len as usize).min(MAX_MODEL_NAME_BYTES);
        core::str::from_utf8(&self.name_utf8[..len]).unwrap_or("unknown")
    }
}

/// Host -> MCU model transfer request (served off the real-time servo control path).
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub enum ModelRequest {
    GetInfo,
    ReadChunk { offset: u32, max_len: u16 },
}

/// Zero-allocation fixed-capacity model chunk carrying up to `MODEL_CHUNK_BYTES` bytes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ModelChunk {
    pub offset: u32,
    pub total_bytes: u32,
    pub crc32: u32,
    pub format: ModelFormat,
    pub len: u16,
    pub data: [u8; MODEL_CHUNK_BYTES],
}

impl ModelChunk {
    pub fn payload(&self) -> &[u8] {
        let n = (self.len as usize).min(MODEL_CHUNK_BYTES);
        &self.data[..n]
    }
}

impl Serialize for ModelChunk {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        use serde::ser::SerializeTuple;
        let len = (self.len as usize).min(MODEL_CHUNK_BYTES);
        let mut tup = serializer.serialize_tuple(5)?;
        tup.serialize_element(&self.offset)?;
        tup.serialize_element(&self.total_bytes)?;
        tup.serialize_element(&self.crc32)?;
        tup.serialize_element(&self.format)?;
        tup.serialize_element(&BytesRef(&self.data[..len]))?;
        tup.end()
    }
}

struct BytesRef<'a>(&'a [u8]);

impl Serialize for BytesRef<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        serializer.serialize_bytes(self.0)
    }
}

struct FixedChunkBytes([u8; MODEL_CHUNK_BYTES], u16);

impl<'de> Deserialize<'de> for FixedChunkBytes {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        struct ChunkVisitor;
        impl<'de> serde::de::Visitor<'de> for ChunkVisitor {
            type Value = FixedChunkBytes;

            fn expecting(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                f.write_str("byte slice up to MODEL_CHUNK_BYTES")
            }

            fn visit_bytes<E>(self, v: &[u8]) -> Result<Self::Value, E>
            where
                E: serde::de::Error,
            {
                if v.len() > MODEL_CHUNK_BYTES {
                    return Err(E::custom("chunk exceeds MODEL_CHUNK_BYTES"));
                }
                let mut arr = [0u8; MODEL_CHUNK_BYTES];
                arr[..v.len()].copy_from_slice(v);
                Ok(FixedChunkBytes(arr, v.len() as u16))
            }

            fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
            where
                A: serde::de::SeqAccess<'de>,
            {
                let mut arr = [0u8; MODEL_CHUNK_BYTES];
                let mut idx = 0usize;
                while let Some(b) = seq.next_element::<u8>()? {
                    if idx >= MODEL_CHUNK_BYTES {
                        return Err(serde::de::Error::custom("chunk exceeds MODEL_CHUNK_BYTES"));
                    }
                    arr[idx] = b;
                    idx += 1;
                }
                Ok(FixedChunkBytes(arr, idx as u16))
            }
        }
        deserializer.deserialize_bytes(ChunkVisitor)
    }
}

impl<'de> Deserialize<'de> for ModelChunk {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let (offset, total_bytes, crc32, format, FixedChunkBytes(data, len)) =
            <(u32, u32, u32, ModelFormat, FixedChunkBytes)>::deserialize(deserializer)?;
        Ok(Self {
            offset,
            total_bytes,
            crc32,
            format,
            len,
            data,
        })
    }
}

/// MCU -> Host model transfer response.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub enum ModelResponse {
    Info(ModelInfo),
    Chunk(ModelChunk),
    NotAvailable,
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
    let w = &pkt.walk;
    if !w.vx.is_finite()
        || !w.vy.is_finite()
        || !w.yaw_rate.is_finite()
        || !w.stance_crouch.is_finite()
        || !w.step_height.is_finite()
        || !w.cadence_hz.is_finite()
    {
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
        ConfigCmd::SetBalanceGains(g)
            if !g.uvc_gain_pitch.is_finite()
                || !g.uvc_gain_roll.is_finite()
                || !g.tilt_deadband_rad.is_finite()
                || !g.gyro_damp_pitch.is_finite()
                || !g.gyro_damp_roll.is_finite()
                || !g.arm_assist.is_finite()
                || !g.fall_cutoff_rad.is_finite() =>
        {
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
        || !pkt.imu.pitch_rad.is_finite()
        || !pkt.imu.roll_rad.is_finite()
        || !pkt.imu.yaw_rad.is_finite()
        || !pkt.imu.pitch_rate_rad_s.is_finite()
        || !pkt.imu.roll_rate_rad_s.is_finite()
        || !pkt.imu.yaw_rate_rad_s.is_finite()
        || pkt.imu.gravity.iter().any(|v| !v.is_finite())
        || !pkt.balance.gait_phase_rad.is_finite()
        || !pkt.balance.pitch_corr_rad.is_finite()
        || !pkt.balance.roll_corr_rad.is_finite()
        || !pkt.balance.foot_dx_m.is_finite()
        || !pkt.balance.foot_dy_m.is_finite()
        || !pkt.balance.hip_height_m.is_finite()
    {
        return Err(CodecError::NonFiniteFloat);
    }
    Ok(pkt)
}

/// Encodes a `ModelRequest` into a COBS frame.
pub fn encode_model_req_cobs<'a>(
    req: &ModelRequest,
    buf: &'a mut [u8],
) -> Result<&'a [u8], CodecError> {
    encode_cobs(req, buf)
}

/// Decodes a `ModelRequest` from a COBS frame.
pub fn decode_model_req_cobs(buf: &mut [u8]) -> Result<ModelRequest, CodecError> {
    let (req, _) = decode_cobs::<ModelRequest>(buf)?;
    Ok(req)
}

/// Encodes a `ModelResponse` into a COBS frame.
pub fn encode_model_resp_cobs<'a>(
    resp: &ModelResponse,
    buf: &'a mut [u8],
) -> Result<&'a [u8], CodecError> {
    encode_cobs(resp, buf)
}

/// Decodes a `ModelResponse` from a COBS frame.
pub fn decode_model_resp_cobs(buf: &mut [u8]) -> Result<ModelResponse, CodecError> {
    let (resp, _) = decode_cobs::<ModelResponse>(buf)?;
    Ok(resp)
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
            walk: WalkCmd {
                walk_enabled: true,
                balance_enabled: true,
                vx: 0.75,
                vy: -0.25,
                yaw_rate: 0.4,
                stance_crouch: 0.35,
                step_height: 0.6,
                cadence_hz: 1.7,
            },
            config: ConfigCmd::SetBalanceGains(BalanceGains::default()),
        };
        pkt.target_rad[0] = 0.25;
        pkt.target_rad[5] = -1.75;

        let mut wire = [0u8; MAX_COBS_FRAME_BYTES];
        let encoded = encode_cobs(&pkt, &mut wire).expect("encode host");
        assert_eq!(*encoded.last().unwrap(), 0x00, "COBS delimiter");
        let len = encoded.len();
        assert!(len < MAX_COBS_FRAME_BYTES);

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
            imu: ImuState {
                available: true,
                pitch_rad: 0.08,
                roll_rad: -0.05,
                yaw_rad: 0.31,
                pitch_rate_rad_s: 0.42,
                roll_rate_rad_s: -0.19,
                yaw_rate_rad_s: 0.07,
                gravity: [0.12, -9.78, 0.04],
            },
            balance: BalanceTelemetry {
                gait_phase_rad: 1.25,
                pitch_corr_rad: -0.09,
                roll_corr_rad: 0.06,
                balance_active: true,
                walk_active: true,
                uvc_phase: uvc_phase::ACTIVE_CATCH_STEP,
                support_foot: 1,
                foot_dx_m: 0.045,
                foot_dy_m: -0.032,
                hip_height_m: 0.61,
                fall_orientation: fall_orientation::PRONE,
            },
            ..Default::default()
        };
        pkt.measured_rad[3] = -0.88;
        pkt.applied_target_rad[3] = -0.92;
        pkt.current_ma[3] = 2150;
        pkt.temp_c[3] = 51;

        let mut wire = [0u8; MAX_COBS_FRAME_BYTES];
        let encoded = encode_cobs(&pkt, &mut wire).expect("encode mcu");
        let len = encoded.len();
        assert!(len < MAX_COBS_FRAME_BYTES);

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

    #[test]
    fn classify_fall_world_gravity_four_directions() {
        // World-frame gravity is near-constant; the Euler attitude carries the tilt.
        let sample = |pitch: f32, roll: f32| ImuState {
            available: true,
            pitch_rad: pitch,
            roll_rad: roll,
            gravity: [0.0, -9.81, 0.0],
            ..Default::default()
        };

        // Forward pitch (pitch > 0) -> body +Z gravity -> Prone.
        assert_eq!(classify_fall(&sample(1.2, 0.0)), FallOrientation::Prone);
        // Backward pitch (pitch < 0) -> body -Z gravity -> Supine.
        assert_eq!(classify_fall(&sample(-1.2, 0.0)), FallOrientation::Supine);
        // Right roll (roll > 0) -> body +X gravity -> SideRight.
        assert_eq!(classify_fall(&sample(0.0, 1.2)), FallOrientation::SideRight);
        // Left roll (roll < 0) -> body -X gravity -> SideLeft.
        assert_eq!(classify_fall(&sample(0.0, -1.2)), FallOrientation::SideLeft);
    }

    #[test]
    fn classify_fall_body_frame_gravity_four_directions() {
        // Already-rotated body-frame gravity vectors must classify directly.
        let sample = |x: f32, z: f32| ImuState {
            available: true,
            gravity: [x, -8.0, z],
            ..Default::default()
        };
        assert_eq!(classify_fall(&sample(0.0, 6.0)), FallOrientation::Prone);
        assert_eq!(classify_fall(&sample(0.0, -6.0)), FallOrientation::Supine);
        assert_eq!(classify_fall(&sample(6.0, 0.0)), FallOrientation::SideRight);
        assert_eq!(classify_fall(&sample(-6.0, 0.0)), FallOrientation::SideLeft);
    }

    #[test]
    fn classify_fall_unavailable_and_invalid_are_unknown() {
        let mut s = ImuState {
            pitch_rad: 1.2,
            gravity: [0.0, -9.81, 0.0],
            ..Default::default()
        };
        assert_eq!(classify_fall(&s), FallOrientation::Unknown, "IMU unavailable");

        s.available = true;
        s.gravity = [0.0, 0.0, 0.0];
        assert_eq!(classify_fall(&s), FallOrientation::Unknown, "zero gravity");

        s.gravity = [f32::NAN, -9.81, 0.0];
        assert_eq!(classify_fall(&s), FallOrientation::Unknown, "non-finite gravity");

        s.gravity = [0.0, -9.81, 0.0];
        s.pitch_rad = 0.0;
        s.roll_rad = 0.0;
        assert_eq!(classify_fall(&s), FallOrientation::Unknown, "upright");
    }

    #[test]
    fn fall_orientation_wire_round_trip() {
        for o in [
            FallOrientation::Unknown,
            FallOrientation::Prone,
            FallOrientation::Supine,
            FallOrientation::SideLeft,
            FallOrientation::SideRight,
        ] {
            assert_eq!(FallOrientation::from_wire(o.to_wire()), o);
        }
        assert_eq!(FallOrientation::from_wire(200), FallOrientation::Unknown);
    }

    #[test]
    fn crc32_matches_ieee_standard_vector() {
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn model_protocol_info_and_chunk_round_trip() {
        let sample = b"<robot name=\"mini\"><link name=\"base\"/></robot>";
        let info = ModelInfo::from_static("mini", ModelFormat::UrdfXml, sample, 4);
        assert_eq!(info.name_str(), "mini");
        assert_eq!(info.total_bytes as usize, sample.len());

        let mut wire = [0u8; MAX_MODEL_COBS_FRAME_BYTES];
        let req = ModelRequest::ReadChunk {
            offset: 0,
            max_len: 128,
        };
        let n = encode_model_req_cobs(&req, &mut wire).expect("encode req").len();
        assert_eq!(decode_model_req_cobs(&mut wire[..n]).expect("decode req"), req);

        let resp_info = ModelResponse::Info(info);
        let n = encode_model_resp_cobs(&resp_info, &mut wire)
            .expect("encode info")
            .len();
        assert_eq!(
            decode_model_resp_cobs(&mut wire[..n]).expect("decode info"),
            resp_info
        );

        let mut data = [0u8; MODEL_CHUNK_BYTES];
        data[..sample.len()].copy_from_slice(sample);
        let chunk = ModelChunk {
            offset: 0,
            total_bytes: sample.len() as u32,
            crc32: info.crc32,
            format: ModelFormat::UrdfXml,
            len: sample.len() as u16,
            data,
        };
        let resp_chunk = ModelResponse::Chunk(chunk);
        let n = encode_model_resp_cobs(&resp_chunk, &mut wire)
            .expect("encode chunk")
            .len();
        let decoded = decode_model_resp_cobs(&mut wire[..n]).expect("decode chunk");
        assert_eq!(decoded, resp_chunk);
    }
}
