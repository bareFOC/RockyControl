//! Deterministic onboard status LED pattern generator reflecting safety mode.
//!
//! - `Teach`: Calm 1 Hz heartbeat (200 ms ON / 800 ms OFF) -> Torque OFF, safe to move by hand
//! - `Drive`: Solid ON -> Active torque tracking host commands
//! - `EStop` / Watchdog / Fault: Urgent 8 Hz double-strobe -> Torque cut, operator reset required

use humanoid_proto::{ControlMode, FaultFlags};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StatusLedMode {
    TeachHeartbeat,
    DriveSolid,
    FaultStrobe,
}

impl StatusLedMode {
    #[inline]
    pub fn from_state(mode: ControlMode, faults: FaultFlags) -> Self {
        if mode == ControlMode::EStop
            || faults.intersects(
                FaultFlags::ESTOP_ACTIVE
                    | FaultFlags::WATCHDOG_TIMEOUT
                    | FaultFlags::UNDER_VOLTAGE
                    | FaultFlags::OVER_CURRENT
                    | FaultFlags::OVER_TEMP
                    | FaultFlags::SERVO_BUS_ERROR,
            )
        {
            Self::FaultStrobe
        } else if mode == ControlMode::Drive {
            Self::DriveSolid
        } else {
            Self::TeachHeartbeat
        }
    }

    /// Evaluates whether the status LED should be illuminated at `now_ms`.
    #[inline]
    pub fn is_lit(self, now_ms: u32) -> bool {
        match self {
            Self::DriveSolid => true,
            Self::TeachHeartbeat => (now_ms % 1000) < 180,
            Self::FaultStrobe => (now_ms % 125) < 62,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn led_pattern_maps_safety_modes_deterministically() {
        assert_eq!(
            StatusLedMode::from_state(ControlMode::Teach, FaultFlags::empty()),
            StatusLedMode::TeachHeartbeat
        );
        assert_eq!(
            StatusLedMode::from_state(ControlMode::Drive, FaultFlags::empty()),
            StatusLedMode::DriveSolid
        );
        assert_eq!(
            StatusLedMode::from_state(ControlMode::EStop, FaultFlags::ESTOP_ACTIVE),
            StatusLedMode::FaultStrobe
        );
        assert_eq!(
            StatusLedMode::from_state(ControlMode::Drive, FaultFlags::WATCHDOG_TIMEOUT),
            StatusLedMode::FaultStrobe
        );
        assert!(StatusLedMode::DriveSolid.is_lit(0));
        assert!(StatusLedMode::TeachHeartbeat.is_lit(50));
        assert!(!StatusLedMode::TeachHeartbeat.is_lit(500));
    }
}
