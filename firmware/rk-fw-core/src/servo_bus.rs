//! Board-agnostic servo bus abstraction (`ServoBusPort`) + Day-1 USB bench loopback plant.
//!
//! - **Milestone 2.1 (Wired USB Bench Bring-Up):** Use [`BenchLoopbackBus`] so you can flash
//!   RP2350 or ESP32-P4, plug in a single USB cable (CDC-ACM or USB Ethernet NCM), and verify
//!   real MCU `copper-rs` 200 Hz mirroring, bumpless transfer, E-Stop, and 100 ms watchdog
//!   with zero servos attached.
//! - **Milestone 2.2 (Hardware Servo Bus):** Bind a board UART/PIO half-duplex or CAN/TWAI
//!   servo driver implementing [`ServoBusPort`] into [`SERVO_BUS_SLOT`].

use core::cell::RefCell;

use critical_section::Mutex;
use rk_control::{RawServoCommand, RawServoFeedback};
use rk_proto::MAX_JOINTS;

use crate::config::DEFAULT_JOINT_COUNT;

/// Hardware-independent servo bus interface called once per `copper-rs` control iteration.
pub trait ServoBusPort: Send {
    fn read_feedback(&mut self, dt_sec: f32) -> RawServoFeedback;
    fn write_command(&mut self, cmd: &RawServoCommand);
}

/// Pure `no_std` single-precision `f32` bench-test servo plant running directly on the MCU.
///
/// Simulates first-order actuator response, nominal 12 V bus voltage, idle/tracking current,
/// and gentle thermal rise when torque is enabled — letting operators validate real MCU USB
/// round-trip latency, E-Stop, and watchdog trips on the bench before powering real servos.
pub struct BenchLoopbackBus {
    joint_count: usize,
    raw_rad: [f32; MAX_JOINTS],
    target_rad: [f32; MAX_JOINTS],
    torque_mask: u32,
    current_ma: [u16; MAX_JOINTS],
    temp_c: [f32; MAX_JOINTS],
    bus_fault: bool,
}

impl Default for BenchLoopbackBus {
    fn default() -> Self {
        Self::new(DEFAULT_JOINT_COUNT)
    }
}

impl BenchLoopbackBus {
    pub const fn new(joint_count: usize) -> Self {
        Self {
            joint_count,
            raw_rad: [0.0_f32; MAX_JOINTS],
            target_rad: [0.0_f32; MAX_JOINTS],
            torque_mask: 0,
            current_ma: [80u16; MAX_JOINTS],
            temp_c: [28.0_f32; MAX_JOINTS],
            bus_fault: false,
        }
    }
}

impl ServoBusPort for BenchLoopbackBus {
    fn read_feedback(&mut self, dt_sec: f32) -> RawServoFeedback {
        let dt = dt_sec.clamp(0.001_f32, 0.05_f32);
        let alpha = (dt * 18.0_f32).clamp(0.0_f32, 1.0_f32);
        let mut total_ma: u32 = 0;

        for i in 0..self.joint_count.min(MAX_JOINTS) {
            let energized = (self.torque_mask & (1u32 << i)) != 0;
            if energized {
                let err = self.target_rad[i] - self.raw_rad[i];
                self.raw_rad[i] += err * alpha;
                let draw = (160.0_f32 + err.abs() * 1150.0_f32).clamp(120.0_f32, 1850.0_f32);
                self.current_ma[i] = draw as u16;
                self.temp_c[i] = (self.temp_c[i] + (draw * 0.00025_f32) * dt).min(68.0_f32);
            } else {
                // Torque OFF: hold current angle, idle quiescent current, cool toward ambient
                self.current_ma[i] = 45;
                self.temp_c[i] = (self.temp_c[i] - 0.15_f32 * dt).max(26.0_f32);
            }
            total_ma = total_ma.saturating_add(self.current_ma[i] as u32);
        }

        let mut temp_u8 = [28u8; MAX_JOINTS];
        for i in 0..MAX_JOINTS {
            temp_u8[i] = self.temp_c[i].clamp(0.0_f32, 120.0_f32) as u8;
        }

        // Mild realistic voltage sag under load from 12.20 V nominal 3S LiPo
        let sag_mv = (total_ma / 18).min(900) as u16;
        let bus_voltage_mv = 12_200u16.saturating_sub(sag_mv);

        RawServoFeedback {
            raw_angle_rad: self.raw_rad,
            current_ma: self.current_ma,
            temp_c: temp_u8,
            bus_voltage_mv,
            bus_error: self.bus_fault,
            imu: rk_proto::ImuState::default(),
        }
    }

    fn write_command(&mut self, cmd: &RawServoCommand) {
        self.torque_mask = cmd.torque_mask;
        self.target_rad = cmd.raw_target_rad;
    }
}

/// Static servo bus registration slot consumed by `copper-rs` `ServoBusBridge`.
///
/// Boards install either [`BenchLoopbackBus`] or a hardware UART/PIO/TWAI driver before
/// starting the `copper-rs` runtime, keeping `ServoBusBridge` 100% portable across MCUs.
pub struct ServoBusSlot {
    /// Active fallback bench bus (used when no custom hardware callback is registered).
    bench: Mutex<RefCell<BenchLoopbackBus>>,
    /// Optional board hardware hooks `(read_fn, write_fn)`.
    hooks: Mutex<RefCell<Option<ServoBusHooks>>>,
}

#[derive(Clone, Copy)]
pub struct ServoBusHooks {
    pub read_feedback: fn(f32) -> RawServoFeedback,
    pub write_command: fn(&RawServoCommand),
}

impl ServoBusSlot {
    pub const fn new() -> Self {
        Self {
            bench: Mutex::new(RefCell::new(BenchLoopbackBus::new(DEFAULT_JOINT_COUNT))),
            hooks: Mutex::new(RefCell::new(None)),
        }
    }

    /// Registers board-specific hardware UART/PIO/TWAI servo bus hooks.
    pub fn register_hooks(&self, hooks: ServoBusHooks) {
        critical_section::with(|cs| {
            *self.hooks.borrow_ref_mut(cs) = Some(hooks);
        });
    }

    /// Clears custom hardware hooks and reverts to the built-in `BenchLoopbackBus`.
    pub fn use_bench_loopback(&self) {
        critical_section::with(|cs| {
            *self.hooks.borrow_ref_mut(cs) = None;
        });
    }

    pub fn read_feedback(&self, dt_sec: f32) -> RawServoFeedback {
        let maybe_fn = critical_section::with(|cs| self.hooks.borrow_ref(cs).map(|h| h.read_feedback));
        if let Some(read_fn) = maybe_fn {
            read_fn(dt_sec)
        } else {
            critical_section::with(|cs| self.bench.borrow_ref_mut(cs).read_feedback(dt_sec))
        }
    }

    pub fn write_command(&self, cmd: &RawServoCommand) {
        let maybe_fn = critical_section::with(|cs| self.hooks.borrow_ref(cs).map(|h| h.write_command));
        if let Some(write_fn) = maybe_fn {
            write_fn(cmd);
        } else {
            critical_section::with(|cs| self.bench.borrow_ref_mut(cs).write_command(cmd));
        }
    }
}

impl Default for ServoBusSlot {
    fn default() -> Self {
        Self::new()
    }
}

/// Global servo bus bridge slot shared between board BSP initialization and `copper-rs`.
pub static SERVO_BUS_SLOT: ServoBusSlot = ServoBusSlot::new();

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bench_loopback_tracks_when_energized_and_holds_when_off() {
        let mut bus = BenchLoopbackBus::new(17);
        let mut cmd = RawServoCommand::default();
        cmd.torque_mask = 1 << 3;
        cmd.raw_target_rad[3] = 1.0_f32;
        bus.write_command(&cmd);

        for _ in 0..40 {
            let _ = bus.read_feedback(0.005);
        }
        let fb_on = bus.read_feedback(0.005);
        assert!(fb_on.raw_angle_rad[3] > 0.85, "energized joint should converge");
        assert!(fb_on.current_ma[3] >= 120);

        // Cut torque -> angle should freeze and current drop to idle
        cmd.torque_mask = 0;
        cmd.raw_target_rad[3] = -1.0_f32;
        bus.write_command(&cmd);
        let fb_off = bus.read_feedback(0.005);
        assert!((fb_off.raw_angle_rad[3] - fb_on.raw_angle_rad[3]).abs() < 1e-6);
        assert_eq!(fb_off.current_ma[3], 45);
    }
}
