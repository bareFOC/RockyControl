//! Versioned, CRC32-verified `#![no_std]` flash persistence format for Rocky MCU configuration.
//!
//! Stores calibrated `JointCalibration[0..MAX_JOINTS]`, `BalanceGains`, `UvcGaitConfig`,
//! and `ImpactWireConfig` in a fixed 4096-byte sector image with magic + schema version
//! + trailing IEEE CRC-32 (`rk_proto::crc32`).

use rk_control::{DirectServoKernel, JointCalibration};
use rk_proto::{BalanceGains, ImpactWireConfig, MAX_JOINTS, UvcGaitConfig, crc32};

pub const FLASH_CONFIG_MAGIC: [u8; 4] = *b"RKCF";
pub const FLASH_CONFIG_VERSION: u16 = 1;
pub const FLASH_SECTOR_BYTES: usize = 4096;
/// Fixed serialized payload size before the 4-byte trailing CRC-32.
/// 8 (header) + 36 * 21 (calibration) + 56 (BalanceGains) + 40 (UvcGaitConfig) + 26 (ImpactWireConfig) = 886
pub const FLASH_PAYLOAD_BYTES: usize = 8 + MAX_JOINTS * 21 + 14 * 4 + 10 * 4 + 26;
pub const FLASH_RECORD_BYTES: usize = FLASH_PAYLOAD_BYTES + 4;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PersistedConfig {
    pub joint_count: u8,
    pub calibration: [JointCalibration; MAX_JOINTS],
    pub balance_gains: BalanceGains,
    pub uvc_gait: UvcGaitConfig,
    pub impact: ImpactWireConfig,
}

impl PersistedConfig {
    pub fn from_kernel(kernel: &DirectServoKernel) -> Self {
        let mut calibration = [JointCalibration::default(); MAX_JOINTS];
        let src = kernel.calibration();
        let n = src.len().min(MAX_JOINTS);
        calibration[..n].copy_from_slice(&src[..n]);
        Self {
            joint_count: n as u8,
            calibration,
            balance_gains: kernel.balance_gains(),
            uvc_gait: kernel.uvc_gait_config(),
            impact: kernel.impact_wire_config(),
        }
    }

    /// Encodes into a 4096-byte sector buffer (0xFF-padded) with trailing IEEE CRC-32.
    pub fn encode_sector(&self, out: &mut [u8; FLASH_SECTOR_BYTES]) {
        out.fill(0xFF);
        let mut w = Writer::new(&mut out[..FLASH_PAYLOAD_BYTES]);
        w.write_bytes(&FLASH_CONFIG_MAGIC);
        w.write_u16(FLASH_CONFIG_VERSION);
        w.write_u16(self.joint_count as u16);

        for cal in &self.calibration {
            w.write_f32(cal.zero_offset_rad);
            w.write_u8(if cal.inverted { 1 } else { 0 });
            w.write_f32(cal.min_rad);
            w.write_f32(cal.max_rad);
            w.write_f32(cal.max_torque_nm);
            w.write_f32(cal.max_velocity_rad_s);
        }

        let g = &self.balance_gains;
        for f in [
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
            g.h_com_m,
            g.ukemi_tilt_rad,
            g.ukemi_torque_scale,
            g.compliance_scale,
        ] {
            w.write_f32(f);
        }

        let u = &self.uvc_gait;
        for f in [
            u.max_foot_lift,
            u.max_sway,
            u.max_dx,
            u.max_dy,
            u.max_swing_dx,
            u.max_swing_dy,
            u.step_duration_s,
            u.toe_push_off_gain,
            u.waist_balance_gain,
            u.arm_swing_gain,
        ] {
            w.write_f32(f);
        }

        let ic = &self.impact;
        w.write_u8(if ic.torque_capable { 1 } else { 0 });
        w.write_f32(ic.k_o);
        w.write_f32(ic.threshold_nm);
        w.write_u8(ic.debounce_ticks);
        w.write_f32(ic.refractory_s);
        w.write_f32(ic.current_spike_ma);
        w.write_f32(ic.ma_per_nm);

        let sum = crc32(&out[..FLASH_PAYLOAD_BYTES]);
        out[FLASH_PAYLOAD_BYTES..FLASH_RECORD_BYTES].copy_from_slice(&sum.to_le_bytes());
    }

    /// Decodes from a sector buffer, verifying magic, version, CRC-32, and finiteness.
    pub fn decode_sector(buf: &[u8]) -> Option<Self> {
        if buf.len() < FLASH_RECORD_BYTES {
            return None;
        }
        if buf[0..4] != FLASH_CONFIG_MAGIC {
            return None;
        }
        let version = u16::from_le_bytes([buf[4], buf[5]]);
        if version != FLASH_CONFIG_VERSION {
            return None;
        }
        let expected_crc = u32::from_le_bytes([
            buf[FLASH_PAYLOAD_BYTES],
            buf[FLASH_PAYLOAD_BYTES + 1],
            buf[FLASH_PAYLOAD_BYTES + 2],
            buf[FLASH_PAYLOAD_BYTES + 3],
        ]);
        if crc32(&buf[..FLASH_PAYLOAD_BYTES]) != expected_crc {
            return None;
        }

        let mut r = Reader::new(&buf[6..FLASH_PAYLOAD_BYTES]);
        let joint_count = r.read_u16()?;
        if joint_count as usize > MAX_JOINTS {
            return None;
        }

        let mut calibration = [JointCalibration::default(); MAX_JOINTS];
        for cal in &mut calibration {
            cal.zero_offset_rad = r.read_f32()?;
            cal.inverted = r.read_u8()? != 0;
            cal.min_rad = r.read_f32()?;
            cal.max_rad = r.read_f32()?;
            cal.max_torque_nm = r.read_f32()?;
            cal.max_velocity_rad_s = r.read_f32()?;
            if !cal.zero_offset_rad.is_finite()
                || !cal.min_rad.is_finite()
                || !cal.max_rad.is_finite()
                || !cal.max_torque_nm.is_finite()
                || !cal.max_velocity_rad_s.is_finite()
            {
                return None;
            }
        }

        let balance_gains = BalanceGains {
            uvc_gain_pitch: r.read_finite_f32()?,
            uvc_gain_roll: r.read_finite_f32()?,
            tilt_deadband_rad: r.read_finite_f32()?,
            gyro_damp_pitch: r.read_finite_f32()?,
            gyro_damp_roll: r.read_finite_f32()?,
            arm_assist: r.read_finite_f32()?,
            fall_cutoff_rad: r.read_finite_f32()?,
            dcm_step_gain: r.read_finite_f32()?,
            foot_half_length_m: r.read_finite_f32()?,
            foot_margin_m: r.read_finite_f32()?,
            h_com_m: r.read_finite_f32()?,
            ukemi_tilt_rad: r.read_finite_f32()?,
            ukemi_torque_scale: r.read_finite_f32()?,
            compliance_scale: r.read_finite_f32()?,
        };

        let uvc_gait = UvcGaitConfig {
            max_foot_lift: r.read_finite_f32()?,
            max_sway: r.read_finite_f32()?,
            max_dx: r.read_finite_f32()?,
            max_dy: r.read_finite_f32()?,
            max_swing_dx: r.read_finite_f32()?,
            max_swing_dy: r.read_finite_f32()?,
            step_duration_s: r.read_finite_f32()?,
            toe_push_off_gain: r.read_finite_f32()?,
            waist_balance_gain: r.read_finite_f32()?,
            arm_swing_gain: r.read_finite_f32()?,
        };

        let impact = ImpactWireConfig {
            torque_capable: r.read_u8()? != 0,
            k_o: r.read_finite_f32()?,
            threshold_nm: r.read_finite_f32()?,
            debounce_ticks: r.read_u8()?,
            refractory_s: r.read_finite_f32()?,
            current_spike_ma: r.read_finite_f32()?,
            ma_per_nm: r.read_finite_f32()?,
        };

        Some(Self {
            joint_count: joint_count as u8,
            calibration,
            balance_gains,
            uvc_gait: uvc_gait.sanitized(),
            impact,
        })
    }
}

struct Writer<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> Writer<'a> {
    fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }
    fn write_bytes(&mut self, b: &[u8]) {
        self.buf[self.pos..self.pos + b.len()].copy_from_slice(b);
        self.pos += b.len();
    }
    fn write_u8(&mut self, v: u8) {
        self.buf[self.pos] = v;
        self.pos += 1;
    }
    fn write_u16(&mut self, v: u16) {
        self.write_bytes(&v.to_le_bytes());
    }
    fn write_f32(&mut self, v: f32) {
        self.write_bytes(&v.to_le_bytes());
    }
}

struct Reader<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    fn new(buf: &'a [u8]) -> Self {
        Self { buf, pos: 0 }
    }
    fn read_u8(&mut self) -> Option<u8> {
        let b = *self.buf.get(self.pos)?;
        self.pos += 1;
        Some(b)
    }
    fn read_u16(&mut self) -> Option<u16> {
        let s = self.buf.get(self.pos..self.pos + 2)?;
        self.pos += 2;
        Some(u16::from_le_bytes([s[0], s[1]]))
    }
    fn read_f32(&mut self) -> Option<f32> {
        let s = self.buf.get(self.pos..self.pos + 4)?;
        self.pos += 4;
        Some(f32::from_le_bytes([s[0], s[1], s[2], s[3]]))
    }
    fn read_finite_f32(&mut self) -> Option<f32> {
        let v = self.read_f32()?;
        v.is_finite().then_some(v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn persisted_config_round_trip_and_corruption_rejected() {
        let mut kernel = DirectServoKernel::new(21, &[JointCalibration::default(); MAX_JOINTS]);
        let mut gait = UvcGaitConfig::default();
        gait.max_foot_lift = 0.048;
        gait.step_duration_s = 0.22;
        kernel.set_uvc_gait_config(gait);

        let cfg = PersistedConfig::from_kernel(&kernel);
        let mut sector = [0xFFu8; FLASH_SECTOR_BYTES];
        cfg.encode_sector(&mut sector);

        let decoded = PersistedConfig::decode_sector(&sector).expect("valid sector");
        assert_eq!(decoded, cfg);

        // Blank sector -> None
        let blank = [0xFFu8; FLASH_SECTOR_BYTES];
        assert!(PersistedConfig::decode_sector(&blank).is_none());

        // Single bit flip -> CRC mismatch -> None
        sector[42] ^= 0x01;
        assert!(PersistedConfig::decode_sector(&sector).is_none());
    }
}
