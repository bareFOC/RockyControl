//! Pure `#![no_std]` ST LSM6DSV16X 6-Axis IMU Register Map, 960 Hz FIFO Burst Parser,
//! 4-State Complementary Attitude Filter (~100 FLOPs), and Zero-Lag Attitude-Derived Gravity.
//!
//! References: ST DS13510 Rev4 (LSM6DSV16X) & AN5763 Rev4.
//!
//! Design invariants:
//! - Dedicated SPI1 bus + INT1 DRDY (never shared with servo I3C bus).
//! - `WHO_AM_I` (`0x0F`) must equal `0x70`; 16X vs 32X distinguished via `CTRL8` bit 2.
//! - Hardware FIFO batches 960 Hz samples (~5 samples per 5 ms 200 Hz control tick) for
//!   hardware anti-aliasing and 5× fewer SPI transactions.
//! - Discards the first `SETTLING_SAMPLES` (6) samples after boot/reset.
//! - Holds last valid gyro rate for up to `GYRO_HOLDOUT_MS` (20 ms) on transient bus read
//!   faults before flagging `ImuState.available = false`.
//! - Gravity vector is computed directly from the complementary-filter attitude
//!   (`R_z(roll) * R_x(-pitch) * (0, -9.81, 0)`) — zero LPF phase lag and guaranteed
//!   compatible with `rk_proto::classify_fall` (`+X` right, `+Y` up, `+Z` forward).

use libm::{atan2f, cosf, sinf, sqrtf};
use rk_proto::ImuState;

// ---------------------------------------------------------------------------
// ST LSM6DSV16X Register Map (DS13510 Rev4)
// ---------------------------------------------------------------------------

pub const REG_FIFO_CTRL1: u8 = 0x07;
pub const REG_FIFO_CTRL2: u8 = 0x08;
pub const REG_FIFO_CTRL3: u8 = 0x09;
pub const REG_FIFO_CTRL4: u8 = 0x0A;
pub const REG_INT1_CTRL: u8 = 0x0D;
pub const REG_WHO_AM_I: u8 = 0x0F;
pub const REG_CTRL1: u8 = 0x10;
pub const REG_CTRL2: u8 = 0x11;
pub const REG_CTRL3: u8 = 0x12;
pub const REG_CTRL6: u8 = 0x15;
pub const REG_CTRL7: u8 = 0x16;
pub const REG_CTRL8: u8 = 0x17;
pub const REG_FIFO_STATUS1: u8 = 0x1B;
pub const REG_FIFO_STATUS2: u8 = 0x1C;
pub const REG_OUTX_L_G: u8 = 0x22;
pub const REG_OUTX_L_A: u8 = 0x28;
pub const REG_FIFO_DATA_OUT_TAG: u8 = 0x78;

/// SPI read bit (MSB = 1 for read, 0 for write).
pub const SPI_READ_BIT: u8 = 0x80;
/// Expected `WHO_AM_I` (`0x0F`) response for LSM6DSV16X / LSM6DSV32X.
pub const WHO_AM_I_EXPECTED: u8 = 0x70;

/// `CTRL3` flags: `BDU` (bit 6), `IF_INC` (bit 2), `SW_RESET` (bit 0).
pub const CTRL3_BDU: u8 = 1 << 6;
pub const CTRL3_IF_INC: u8 = 1 << 2;
pub const CTRL3_SW_RESET: u8 = 1 << 0;

/// High-performance 960 Hz ODR code (`0b1001`) for `CTRL1` (accel) and `CTRL2` (gyro).
pub const ODR_960_HZ: u8 = 0x09;
/// `CTRL6`: `FS_G[3:0] = 0b0011` (±1000 dps = 35 mdps/LSB) + LPF1 BW = `0b010` (`0x20`).
pub const CTRL6_GYRO_1000DPS_LPF1: u8 = 0x23;
/// `CTRL7`: enable gyro digital LPF1 (`LPF1_G_EN`, bit 0).
pub const CTRL7_LPF1_G_EN: u8 = 0x01;
/// `CTRL8`: `FS_XL[1:0] = 0b10` (±8 g = 0.244 mg/LSB on 16X; bit 2 = 0 confirms 16X range).
pub const CTRL8_ACCEL_8G_16X: u8 = 0x02;
/// `FIFO_CTRL3`: batch both gyro (`BDR_GY = 960 Hz = 0x9 << 4`) and accel (`BDR_XL = 960 Hz = 0x9`).
pub const FIFO_CTRL3_BATCH_960HZ: u8 = 0x99;
/// `FIFO_CTRL4`: Continuous FIFO mode (`FIFO_MODE = 0b110`).
pub const FIFO_CTRL4_CONTINUOUS: u8 = 0x06;
/// `INT1_CTRL`: route FIFO threshold / DRDY to INT1 (`INT1_FIFO_TH = bit 3 | INT1_DRDY_G = bit 1`).
pub const INT1_CTRL_FIFO_TH_DRDY: u8 = (1 << 3) | (1 << 1);

/// FIFO tag sensor identifiers (`tag >> 3`).
pub const FIFO_TAG_GYRO_NC: u8 = 0x01;
pub const FIFO_TAG_ACCEL_NC: u8 = 0x02;

/// Gyro sensitivity at ±1000 dps: `35.0 mdps/LSB` -> `rad/s per LSB`.
pub const GYRO_RAD_S_PER_LSB_1000DPS: f32 = 0.035_f32 * (core::f32::consts::PI / 180.0_f32);
/// Accel sensitivity at ±8 g: `0.244 mg/LSB` -> `m/s^2 per LSB`.
pub const ACCEL_M_S2_PER_LSB_8G: f32 = 0.000_244_f32 * 9.806_65_f32;
/// Standard gravity magnitude (`m/s^2`).
pub const GRAVITY_M_S2: f32 = 9.81_f32;
/// Number of initial 960 Hz startup samples discarded while MEMS settles.
pub const SETTLING_SAMPLES: u16 = 6;
/// Hold last valid gyro sample up to 20 ms on transient SPI read error before marking unavailable.
pub const GYRO_HOLDOUT_MS: u32 = 20;
/// Number of 960 Hz paired samples per 5 ms 200 Hz control tick (~4.8 -> 5).
pub const NOMINAL_FIFO_BURST_SAMPLES: usize = 5;

/// Raw signed 16-bit 6-axis sample from LSM6DSV16X (body-aligned axes).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RawLsm6dsvSample {
    pub gyro_raw: [i16; 3],
    pub accel_raw: [i16; 3],
}

/// Validates `WHO_AM_I` (`0x70`) and confirms `CTRL8` bit 2 == 0 for the 16X variant.
#[inline]
pub const fn verify_lsm6dsv16x_id(who_am_i: u8, ctrl8: u8) -> bool {
    who_am_i == WHO_AM_I_EXPECTED && (ctrl8 & 0x04) == 0
}

/// Decodes the 10-bit unread FIFO word count from `FIFO_STATUS1` (`0x1B`) and `FIFO_STATUS2` (`0x1C`).
#[inline]
pub const fn parse_fifo_word_count(status1: u8, status2: u8) -> u16 {
    (status1 as u16) | (((status2 & 0x03) as u16) << 8)
}

/// Parses raw 7-byte LSM6DSV FIFO records (`[tag, xl, xh, yl, yh, zl, zh]`) into averaged
/// gyro + accel LSB vectors over the burst. Returns `(gyro_avg_lsb, accel_avg_lsb, gyro_count, accel_count)`.
pub fn average_fifo_burst(fifo_bytes: &[u8]) -> ([f32; 3], [f32; 3], u16, u16) {
    let mut g_sum = [0.0_f32; 3];
    let mut a_sum = [0.0_f32; 3];
    let mut g_cnt = 0u16;
    let mut a_cnt = 0u16;

    for record in fifo_bytes.chunks_exact(7) {
        let sensor = record[0] >> 3;
        let x = i16::from_le_bytes([record[1], record[2]]) as f32;
        let y = i16::from_le_bytes([record[3], record[4]]) as f32;
        let z = i16::from_le_bytes([record[5], record[6]]) as f32;
        match sensor {
            FIFO_TAG_GYRO_NC => {
                g_sum[0] += x;
                g_sum[1] += y;
                g_sum[2] += z;
                g_cnt = g_cnt.saturating_add(1);
            }
            FIFO_TAG_ACCEL_NC => {
                a_sum[0] += x;
                a_sum[1] += y;
                a_sum[2] += z;
                a_cnt = a_cnt.saturating_add(1);
            }
            _ => {}
        }
    }

    let g_avg = if g_cnt > 0 {
        let inv = 1.0_f32 / (g_cnt as f32);
        [g_sum[0] * inv, g_sum[1] * inv, g_sum[2] * inv]
    } else {
        [0.0; 3]
    };
    let a_avg = if a_cnt > 0 {
        let inv = 1.0_f32 / (a_cnt as f32);
        [a_sum[0] * inv, a_sum[1] * inv, a_sum[2] * inv]
    } else {
        [0.0; 3]
    };

    (g_avg, a_avg, g_cnt, a_cnt)
}

/// Computes zero-lag body-frame gravity directly from estimated pitch & roll:
/// `R_z(roll) * R_x(-pitch) * (0, -9.81, 0)`.
///
/// Exact convention match with `rk-sim` and `rk_proto::classify_fall`:
/// - Upright `(0, 0)` -> `[0, -9.81, 0]`
/// - Prone (`pitch > 0`) -> `g_z > 0`
/// - Supine (`pitch < 0`) -> `g_z < 0`
/// - SideRight (`roll > 0`) -> `g_x > 0`
/// - SideLeft (`roll < 0`) -> `g_x < 0`
#[inline]
pub fn attitude_derived_gravity(pitch_rad: f32, roll_rad: f32) -> [f32; 3] {
    let y1 = -GRAVITY_M_S2 * cosf(-pitch_rad);
    let z1 = -GRAVITY_M_S2 * sinf(-pitch_rad);
    let sr = sinf(roll_rad);
    let cr = cosf(roll_rad);
    [-y1 * sr, y1 * cr, z1]
}

/// 4-state complementary filter `(pitch, roll, bias_pitch, bias_roll)` + gyro-Z yaw integrator.
///
/// Runs in ~100 FLOPs per 200 Hz tick on RP2350 Cortex-M33F (`f32`, zero heap).
#[derive(Clone, Debug)]
pub struct Lsm6dsvFilter {
    pitch_rad: f32,
    roll_rad: f32,
    yaw_rad: f32,
    bias_pitch_rad_s: f32,
    bias_roll_rad_s: f32,
    last_gyro_rad_s: [f32; 3],
    settled_samples: u16,
    last_good_ms: Option<u32>,
    last_state: ImuState,
}

impl Default for Lsm6dsvFilter {
    fn default() -> Self {
        Self::new()
    }
}

impl Lsm6dsvFilter {
    pub const fn new() -> Self {
        Self {
            pitch_rad: 0.0,
            roll_rad: 0.0,
            yaw_rad: 0.0,
            bias_pitch_rad_s: 0.0,
            bias_roll_rad_s: 0.0,
            last_gyro_rad_s: [0.0; 3],
            settled_samples: 0,
            last_good_ms: None,
            last_state: ImuState {
                available: false,
                pitch_rad: 0.0,
                roll_rad: 0.0,
                yaw_rad: 0.0,
                pitch_rate_rad_s: 0.0,
                roll_rate_rad_s: 0.0,
                yaw_rate_rad_s: 0.0,
                gravity: [0.0, -GRAVITY_M_S2, 0.0],
            },
        }
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    pub const fn is_settled(&self) -> bool {
        self.settled_samples >= SETTLING_SAMPLES
    }

    /// Updates the 4-state complementary filter from averaged physical gyro (`rad/s`,
    /// `[pitch_rate, yaw_rate, roll_rate]` in Rocky body frame) and accel (`m/s^2`,
    /// `[ax_right, ay_up, az_forward]`, where upright stationary reads `[0, +9.81, 0]`
    /// specific force).
    pub fn update_physical(
        &mut self,
        gyro_rad_s: [f32; 3],
        accel_m_s2: [f32; 3],
        sample_count: u16,
        dt_sec: f32,
        now_ms: u32,
    ) -> ImuState {
        let dt = dt_sec.clamp(0.0005_f32, 0.05_f32);

        if self.settled_samples < SETTLING_SAMPLES {
            self.settled_samples = self.settled_samples.saturating_add(sample_count.max(1));
            self.last_good_ms = Some(now_ms);
            return self.last_state;
        }

        let gx = gyro_rad_s[0] - self.bias_pitch_rad_s;
        let gy = gyro_rad_s[1];
        let gz = gyro_rad_s[2] - self.bias_roll_rad_s;
        self.last_gyro_rad_s = [gx, gy, gz];

        // Predict attitude from bias-corrected gyro rates
        let pred_pitch = self.pitch_rad + gx * dt;
        let pred_roll = self.roll_rad + gz * dt;
        self.yaw_rad += gy * dt;

        // Specific force magnitude gating: trust accel tilt only near 1g (±0.35 g) so
        // walking impacts don't corrupt attitude.
        let ax = accel_m_s2[0];
        let ay = accel_m_s2[1];
        let az = accel_m_s2[2];
        let a_norm = sqrtf(ax * ax + ay * ay + az * az);
        let near_1g = a_norm > 6.5_f32 && a_norm < 13.5_f32;

        if near_1g {
            // Upright stationary specific force is [0, +9.81, 0] (reaction to gravity [0, -9.81, 0]).
            // Under forward pitch (+pitch), gravity in body is [0, -g cos p, +g sin p],
            // so accelerometer specific force is [0, +g cos p, -g sin p].
            // Under right roll (+roll), gravity in body is [+g cos p sin r, -g cos p cos r, ...],
            // so specific force is [-g cos p sin r, +g cos p cos r, ...].
            let acc_pitch = atan2f(-az, sqrtf(ax * ax + ay * ay).max(1e-4_f32));
            let acc_roll = atan2f(-ax, ay.abs().max(1e-4_f32) * if ay >= 0.0 { 1.0 } else { -1.0 });

            let err_p = acc_pitch - pred_pitch;
            let err_r = acc_roll - pred_roll;

            // Complementary gain (~0.5 s time constant @ 200 Hz -> alpha = 0.02)
            const ALPHA: f32 = 0.02_f32;
            const KI_BIAS: f32 = 0.002_f32;
            self.pitch_rad = (pred_pitch + ALPHA * err_p).clamp(-1.55_f32, 1.55_f32);
            self.roll_rad = (pred_roll + ALPHA * err_r).clamp(-1.55_f32, 1.55_f32);
            self.bias_pitch_rad_s =
                (self.bias_pitch_rad_s - KI_BIAS * err_p * dt).clamp(-0.15_f32, 0.15_f32);
            self.bias_roll_rad_s =
                (self.bias_roll_rad_s - KI_BIAS * err_r * dt).clamp(-0.15_f32, 0.15_f32);
        } else {
            self.pitch_rad = pred_pitch.clamp(-1.55_f32, 1.55_f32);
            self.roll_rad = pred_roll.clamp(-1.55_f32, 1.55_f32);
        }

        let gravity = attitude_derived_gravity(self.pitch_rad, self.roll_rad);
        self.last_good_ms = Some(now_ms);
        self.last_state = ImuState {
            available: true,
            pitch_rad: self.pitch_rad,
            roll_rad: self.roll_rad,
            yaw_rad: self.yaw_rad,
            pitch_rate_rad_s: gx,
            roll_rate_rad_s: gz,
            yaw_rate_rad_s: gy,
            gravity,
        };
        self.last_state
    }

    /// Updates directly from an LSM6DSV16X 960 Hz FIFO burst byte buffer (7 bytes/record).
    pub fn update_from_fifo_burst(
        &mut self,
        fifo_bytes: &[u8],
        dt_sec: f32,
        now_ms: u32,
    ) -> ImuState {
        let (g_lsb, a_lsb, g_cnt, a_cnt) = average_fifo_burst(fifo_bytes);
        if g_cnt == 0 || a_cnt == 0 {
            return self.on_read_fault(now_ms);
        }
        // Remap chip axes -> Rocky body frame (+X right, +Y up, +Z forward):
        // Pitch rate about +X = g_x, Yaw rate about +Y = g_y, Roll rate about -Z = -g_z.
        let gyro_rad_s = [
            g_lsb[0] * GYRO_RAD_S_PER_LSB_1000DPS,
            g_lsb[1] * GYRO_RAD_S_PER_LSB_1000DPS,
            -g_lsb[2] * GYRO_RAD_S_PER_LSB_1000DPS,
        ];
        let accel_m_s2 = [
            a_lsb[0] * ACCEL_M_S2_PER_LSB_8G,
            a_lsb[1] * ACCEL_M_S2_PER_LSB_8G,
            a_lsb[2] * ACCEL_M_S2_PER_LSB_8G,
        ];
        self.update_physical(gyro_rad_s, accel_m_s2, g_cnt.min(a_cnt), dt_sec, now_ms)
    }

    /// Handles a transient SPI read failure: holds the last valid gyro & attitude for up to
    /// `GYRO_HOLDOUT_MS` (20 ms), then marks `available = false`.
    pub fn on_read_fault(&mut self, now_ms: u32) -> ImuState {
        let within_holdout = self
            .last_good_ms
            .map(|t| now_ms.saturating_sub(t) <= GYRO_HOLDOUT_MS)
            .unwrap_or(false);
        if within_holdout && self.last_state.available {
            self.last_state
        } else {
            self.last_state.available = false;
            self.last_state
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rk_proto::{FallOrientation, classify_fall};

    #[test]
    fn who_am_i_and_16x_vs_32x_check() {
        assert!(verify_lsm6dsv16x_id(0x70, CTRL8_ACCEL_8G_16X));
        assert!(!verify_lsm6dsv16x_id(0x6B, CTRL8_ACCEL_8G_16X));
        // Bit 2 set -> 32X range, not 16X
        assert!(!verify_lsm6dsv16x_id(0x70, CTRL8_ACCEL_8G_16X | 0x04));
    }

    #[test]
    fn fifo_burst_averages_five_960hz_samples_and_discards_settling() {
        let mut filter = Lsm6dsvFilter::new();
        // Build 5 paired FIFO records (10 records * 7 bytes = 70 bytes): upright specific force [0, +1g, 0]
        let one_g_lsb = (9.81_f32 / ACCEL_M_S2_PER_LSB_8G) as i16;
        let mut burst = [0u8; 70];
        for i in 0..5 {
            let g_off = (i * 2) * 7;
            burst[g_off] = FIFO_TAG_GYRO_NC << 3;
            let a_off = (i * 2 + 1) * 7;
            burst[a_off] = FIFO_TAG_ACCEL_NC << 3;
            let y_le = one_g_lsb.to_le_bytes();
            burst[a_off + 3] = y_le[0];
            burst[a_off + 4] = y_le[1];
        }

        // First burst has 5 samples (< 6 settling threshold) -> still settling
        let s1 = filter.update_from_fifo_burst(&burst, 0.005, 5);
        assert!(!s1.available, "must discard initial settling samples");

        // Second burst completes settling and publishes upright attitude
        let s2 = filter.update_from_fifo_burst(&burst, 0.005, 10);
        assert!(s2.available);
        assert!(s2.pitch_rad.abs() < 0.01);
        assert!(s2.roll_rad.abs() < 0.01);
        assert!((s2.gravity[1] + 9.81).abs() < 0.05);
    }

    #[test]
    fn zero_lag_attitude_gravity_matches_classify_fall_all_orientations() {
        let cases = [
            (1.15_f32, 0.0_f32, FallOrientation::Prone),
            (-1.15_f32, 0.0_f32, FallOrientation::Supine),
            (0.0_f32, 1.15_f32, FallOrientation::SideRight),
            (0.0_f32, -1.15_f32, FallOrientation::SideLeft),
        ];
        for (pitch, roll, expected) in cases {
            let g = attitude_derived_gravity(pitch, roll);
            let imu = ImuState {
                available: true,
                pitch_rad: pitch,
                roll_rad: roll,
                gravity: g,
                ..Default::default()
            };
            assert_eq!(
                classify_fall(&imu),
                expected,
                "pitch={pitch} roll={roll} gravity={g:?}"
            );
        }
    }

    #[test]
    fn gyro_holdout_preserves_state_up_to_20ms_then_marks_unavailable() {
        let mut filter = Lsm6dsvFilter::new();
        // Settle filter
        let _ = filter.update_physical([0.0; 3], [0.0, 9.81, 0.0], 6, 0.005, 100);
        let good = filter.update_physical([0.2, 0.0, 0.0], [0.0, 9.81, 0.0], 5, 0.005, 105);
        assert!(good.available);

        // At 120 ms (15 ms elapsed <= 20 ms holdout) -> still available
        let held = filter.on_read_fault(120);
        assert!(held.available);
        assert!((held.pitch_rate_rad_s - 0.2).abs() < 1e-4);

        // At 126 ms (21 ms elapsed > 20 ms holdout) -> unavailable
        let expired = filter.on_read_fault(126);
        assert!(!expired.available);
    }
}
